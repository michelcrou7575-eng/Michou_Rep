//==============================================================
//
// HotMelt_MLX90640_80032_9_8_1
// Thermal Glue Inspection System
//
// FOUNDATION: MLX90640 + System State + MCP23017 + NS12 Memory Link
//
// Controller     : Waveshare ESP32-S3-Zero-M (ESP32-S3FH4R2)
// Thermal Sensor : MLX90640 32 x 24
// I2C Interface  : Shared I2C (MLX90640 + MCP23017)
// HMI Interface  : Omron NS12-TS00B-V2 Memory Link via HIN232CP RS-232
// IDE            : VS Code + PlatformIO
//
// This PT's firmware uses ESC=0x1B for EVERY Memory Link command,
// including WM (word write) -- it does NOT implement the dual-ESC-byte
// quirk (0x1C for WM/WD) documented in the official Omron NS-Series
// Host Connection Manual (Cat. No. V085-E1-07). That manual detail is
// real and correctly transcribed, but this unit's firmware doesn't
// honor it.
//
// All temporary diagnostic scaffolding (address sweep, deliberate
// garbage-command test, $W500 read-back, $B20 bit-read test) has been
// removed now that root cause is confirmed and fixed. Raw NS12
// TX/RX byte logging is retained but OFF by default -- flip
// NS12_DEBUG_RAW_RX to 1 if the link ever needs re-diagnosing.
//
// Confirmed working configuration:
//   NS12 baud      : 9600  (38400 showed ~15% read timeouts on this
//                            bench setup; 9600 gave 0 timeouts/errors)
//   NS12 format    : 8 data, no parity, 1 stop
//   CX-Designer    : Serial Port A = Memory Link, Response = OFF
//   RS232 levels   : confirmed -9V/+8V swing (HIN232CP charge pump OK)
//
// This foundation program contains:
//   - Proven MLX90640 camera bring-up and diagnostics
//   - Central system-state manager
//   - MCP23017 machine-I/O manager (confirmed working, A0/A1/A2 grounded)
//   - MCP service permitted only in Standby or Tube Gap
//   - Serial test commands for state and output verification
//   - Confirmed-working NS12Manager with non-blocking Memory Link I/O
//
// Encoder, physical tube sensor, and machine RUN/STOP integration
// (fast-stop-on-apply-failure, resume-on-machine-restart) are the next
// integration stages.
//
//==============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X17.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

//==============================================================
// USER-ADJUSTABLE HARDWARE CONFIGURATION
//==============================================================

// MLX90640 I2C wiring.
constexpr uint8_t MLX_SDA_PIN = 8;
constexpr uint8_t MLX_SCL_PIN = 9;

// MLX90640 normal I2C address.
constexpr uint8_t MLX_I2C_ADDRESS = 0x33;

// MCP23017 machine-I/O expander. A0/A1/A2 all grounded = 0x20.
constexpr uint8_t MCP23017_I2C_ADDRESS = 0x20;

// During Standby, slow machine I/O does not need to be scanned faster.
// During production, the same service is allowed only in the tube gap.
constexpr uint32_t MCP_SERVICE_INTERVAL_MS = 20UL;

// CONFIRMED (2026-07-27): 800kHz is the bench-confirmed sweet spot for
// BOTH devices sharing this I2C bus, 1k pull-ups installed.
constexpr uint32_t I2C_CLOCK_HZ = 800000UL;

// Waveshare ESP32-S3-Zero onboard WS2812 RGB LED.
constexpr uint8_t RGB_LED_PIN = 21;
constexpr uint8_t RGB_LED_COUNT = 1;

// Serial reporting interval.
constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 750UL;

// NS12 Memory Link through HIN232CP full-duplex RS-232.
// USB CDC On Boot must remain enabled so Serial diagnostics do not use GPIO43/44.
// HMI confirmed: NS12-TS00B-V2, Memory Link mode.

constexpr uint32_t NS_BAUD = 9600UL;
constexpr uint8_t NS_TX_PIN = 43; // ESP32-S3 TX -> HIN232CP T1IN.
constexpr uint8_t NS_RX_PIN = 44; // ESP32-S3 RX <- HIN232CP R1OUT.
constexpr uint32_t NS_WRITE_INTERVAL_MS = 250UL;
constexpr uint32_t NS_READ_INTERVAL_MS = 500UL;
constexpr uint32_t NS_RESPONSE_TIMEOUT_MS = 250UL;

// Watchdog: recovers from NS12-link hang instead of
// stalling the whole loop indefinitely with no self-recovery.
constexpr uint32_t WATCHDOG_TIMEOUT_S = 3U;

// Set to 1 to re-enable raw NS12 TX/RX byte logging on the USB Serial
// monitor (used to originally diagnose the ESC-byte root cause). Off
// by default in production -- adds console noise and per-byte overhead.
#define NS12_DEBUG_RAW_RX 0

// Number of consecutive frame failures before a recovery attempt.
constexpr uint8_t FRAME_FAILURE_RECOVERY_COUNT = 5;

//==============================================================
// THERMAL CAMERA CONSTANTS
//==============================================================

constexpr uint16_t MLX_COLUMNS = 32;
constexpr uint16_t MLX_ROWS = 24;
constexpr uint16_t MLX_PIXEL_COUNT = MLX_COLUMNS * MLX_ROWS;

uint16_t nextColumnToWrite = 0;
//==============================================================
// NS12 16x8 COLOR MATRIX (Word Lamp array on HMI, $W700 base)
//==============================================================
// CONFIRMED from actual CX-Designer object addresses (per-cell table):
//   - $W700 is DECIMAL 700, same convention as the existing $W0-$W8
//     telemetry block -- NOT hex 0x700. Only the WM/RM wire command
//     itself hex-encodes the numeric address; the $Wn label is decimal.
//   - Layout is COLUMN-MAJOR with a stride of 8 (= MATRIX_ROWS):
//       address(col, row) = 700 + col*8 + row
//     e.g. col0 = 700..707, col1 = 708..715, col2 = 716..723, ...
//     This means colorMatrix[] must be filled in the same column-major
//     order so one linear WM write starting at 700 lands correctly.
//
//   - Matrix is 16 columns x 8 rows = 128 cells, one word per cell.
//   - Word Lamp on this screen has a 10-entry palette (Color1..Color10
//     in CX-Designer, values 0-9), so each cell word is a palette
//     index 0-9.
//   - Downsample is a straight 2x3 block average: 32/16=2 columns,
//     24/8=3 rows per cell, matching MLX_COLUMNS/MLX_ROWS exactly.
//   - Temp->color range CONFIRMED: 20.0-180.0 C.
//==============================================================

// HMI matrix stays 16x8 (confirmed working -- full 32x24 caused the NS12
// to miss RM read requests entirely). Full 768-pixel mlxFrame is still
// read, statted, and max-hold-captured every frame regardless of this --
// only the HMI color-matrix downsample below is reduced to 128 cells.
constexpr uint16_t MATRIX_COLS = 16;
constexpr uint16_t MATRIX_ROWS = 8;
constexpr uint16_t MATRIX_CELL_COUNT = MATRIX_COLS * MATRIX_ROWS; // 128
constexpr uint16_t MATRIX_BLOCK_COLS = MLX_COLUMNS / MATRIX_COLS; // 2
constexpr uint16_t MATRIX_BLOCK_ROWS = MLX_ROWS / MATRIX_ROWS;    // 3
constexpr uint16_t MATRIX_START_WORD = 700U;                      // $W700 (decimal)
constexpr uint16_t MATRIX_COLOR_LEVELS = 10U;                     // Color1..Color10
constexpr float MATRIX_TEMP_MIN_C = 20.0f;
constexpr float MATRIX_TEMP_MAX_C = 40.0f; // TEMPORARY BENCH VALUE, NOT FINAL: real hot glue is expected to run far hotter than ambient tube temperature -- this MUST be re-tuned once the actual thermal signature of glue onset is known, the same way MATRIX_TEMP_MAX_C needs to go back to 180.0 C before production.

constexpr uint16_t MATRIX_COLUMNS_PER_BATCH = 4;
constexpr uint16_t MATRIX_COLUMN_BATCH_INTERVAL_MS = 100;

//==============================================================
// GLOBAL OBJECTS
//==============================================================

Adafruit_MLX90640 mlx;
Adafruit_MCP23X17 mcp;

Adafruit_NeoPixel statusLed(RGB_LED_COUNT, RGB_LED_PIN, NEO_RGB + NEO_KHZ800);

// One complete MLX90640 temperature frame.
// 768 floats x 4 bytes = 3072 bytes.
float mlxFrame[MLX_PIXEL_COUNT];

// Downsampled 16x8 color-index matrix, rebuilt from mlxFrame every time
// a frame is read successfully. Values 0-9, column-major (index = col*8+row).
uint8_t colorMatrix[MATRIX_CELL_COUNT];

// DIAGNOSTIC: when true, updateColorMatrix() is skipped and colorMatrix
// instead holds a fixed, clearly-visible gradient (every non-zero index
// 1-9 used, cycling). Toggle with the 'M' serial command to check
// whether the HMI shows ANYTHING at $W700 independent of real thermal
// data -- isolates "range/index-0-looks-off" from "link/address" issues.
bool matrixTestPatternActive = false;
bool rawFrameDumpRequested = false;

//==============================================================
// TUBE PASS QC COMPOSITE (max-hold capture)
//==============================================================
// Goal: instead of chasing a fast live HMI refresh, watch every camera
// frame in the background (independent of the NS12 push rate). When a
// tube's glue signature is detected, hold a MAX-HOLD composite across
// CAPTURE_SAMPLE_COUNT frames -- not an average -- so that every glue
// trace seen anywhere during that tube's transit through the FOV ends
// up in one image, confirming glue is present everywhere it should be
// and in sufficient quantity. Freeze that composite as the HMI picture
// until manually rearmed, so the operator sees a stable "this tube: OK"
// snapshot of the tube that just passed rather than a live smear.
//
// TUNING VALUE, NOT CONFIRMED: 30.0 C is a placeholder threshold picked
// to be triggerable by a hand during bench testing (hand showed up to
// ~32 C earlier) against the current 40 C test ceiling. Real hot glue
// is expected to run far hotter than ambient tube temperature -- this
// MUST be re-tuned once the actual thermal signature of glue onset is
// known, the same way MATRIX_TEMP_MAX_C needs to go back to 180.0 C
// before production. Trigger logic: frame's own maximumTemperatureC
// crossing this threshold. If that proves unreliable (e.g. triggers on
// ambient drift), the alternative is a frame-to-frame delta spike
// instead of an absolute threshold -- flag it if 30.0 C misfires.

// constexpr float CAPTURE_TRIGGER_TEMP_C = 90.0f;
constexpr float CAPTURE_TRIGGER_TEMP_C = 180.0f;

// TUNING VALUE: how many camera frames the sampling window spans once
// triggered. MAX-HOLD (not averaging -- see below) is used across this
// window, so this is really "how many frames does it take to cover one
// full tube's transit through the FOV." At ~7.8-8.3 FPS that's roughly
// COUNT/8 seconds of coverage. Started at 4 (top of the 2-4 range) for
// margin; if a tube's glue trail is still getting cut off at the edges
// of the composite, raise this further. If two consecutive tubes are
// blurring into one composite, lower it.
constexpr uint8_t CAPTURE_SAMPLE_COUNT = 4U;

float captureAccumulator[MLX_PIXEL_COUNT] = {};
uint8_t captureSamplesCollected = 0U;
bool captureSamplingInProgress = false;
bool captureArmed = true;    // watching for CAPTURE_TRIGGER_TEMP_C
bool captureLatched = false; // true once a capture completes; HMI picture frozen

//==============================================================
// SYSTEM STATE
//==============================================================

enum class SystemState : uint8_t
{
    Startup,
    Standby,
    WaitingForTube,
    InspectingTube,
    TubeGap,
    FaultStop
};

SystemState systemState = SystemState::Startup;

bool mlxDetected = false;
bool mlxInitialized = false;
bool lastFrameValid = false;

uint32_t successfulFrameCount = 0;
uint32_t failedFrameCount = 0;
uint8_t consecutiveFrameFailures = 0;

uint16_t rawFrame0[834];
uint16_t rawFrame1[834];

uint32_t lastSuccessfulFrameMs = 0;
uint32_t lastDiagnosticMs = 0;
uint32_t fpsWindowStartMs = 0;
uint32_t fpsWindowFrameCount = 0;

float measuredFramesPerSecond = 0.0f;
float minimumTemperatureC = NAN;
float maximumTemperatureC = NAN;
float averageTemperatureC = NAN;

float rawBaseline0[834] = {};
float rawBaseline1[834] = {};
bool rawBaselineCaptured = false;
bool rawBaselineCaptureInProgress = false;
uint8_t rawBaselineFramesCollected = 0U;
constexpr uint8_t RAW_BASELINE_FRAME_COUNT = 32U;

//==============================================================
// MCP23017 MACHINE I/O
//==============================================================

// GPA0..GPA5 are machine outputs.
enum McpOutputBit : uint8_t
{
    MCP_OUT_NORMAL_STOP = 0,
    MCP_OUT_FAST_STOP = 1,
    MCP_OUT_HORN = 2,
    MCP_OUT_BEACON = 3,
    MCP_OUT_READY = 4,
    MCP_OUT_WARNING = 5
};

// GPB0..GPB4 are machine inputs, represented by MCP pins 8..12.
// INPUT_PULLUP is used for the present hardware test; active = contact to 0 V.
enum McpInputPin : uint8_t
{
    MCP_IN_ACKNOWLEDGE = 8,
    MCP_IN_RESET = 9,
    MCP_IN_AUTO = 10,
    MCP_IN_MACHINE_STOPPED = 11,
    MCP_IN_GLUE_READY = 12
};

bool mcpDetected = false;
bool mcpInitialized = false;
uint16_t desiredMcpOutputImage = 0;
uint16_t lastWrittenMcpOutputImage = 0xFFFFU;
uint8_t mcpInputImage = 0xFFU;
uint32_t lastMcpServiceMs = 0;
uint32_t mcpServiceCount = 0;

//==============================================================
// NS12 MEMORY LINK MANAGER
//==============================================================
//
// Protocol reference: Omron NS-Series -V1/-V2 Host Connection Manual
// (Cat. No. V085-E1-07), Section 3 "Connection via Memory Link".
// Confirmed applicable to NS12-TS01B-V2.
//
// Command set used here: WM (write PT $W memory), RM (read PT $W memory).
//
// Frame layout (all ASCII):
//   Write : [ESC=0x1B] W M *S *A(4 hex) *L(2 dec) *D(comma hex,zero-suppr) [CR=0x0D]
//   Read  : [ESC=0x1B] R M *S *A(4 hex) *L(2 dec)                          [CR=0x0D]
//   Read response: [ESC=0x1B] R M *A(4 hex) *L(2 dec) *D(comma hex) [CR=0x0D]
//
// *S = '0' selects SUM (checksum) OFF + SET-write / variable length read,
// which is why no checksum is appended here -- omitting it is valid only
// because *S explicitly says SUM is off.
//
// IMPORTANT: the official manual documents ESC=0x1C specifically for WM
// and WD (word writes), with 0x1B used for every other command. That is
// correctly transcribed from the manual -- but THIS UNIT's firmware does
// not honor that distinction: it expects 0x1B uniformly, for writes too.
// Using 0x1C per the manual resulted in every WM write being silently
// ignored (no ack, no error, nothing) despite RM reads working perfectly
// over hundreds of cycles. Switching WM to 0x1B fixed it immediately,
// confirmed via live sensor data and a known test pattern both landing
// correctly on the physical screen. If this firmware is ever ported to
// a different NS-series unit/firmware revision, this is the first thing
// to re-verify if writes silently stop landing again.
//
// The 128-cell color matrix is sent as sixteen 8-word column writes,
// all 16 fired back-to-back in a single burst every
// MATRIX_FULL_REFRESH_INTERVAL_MS --.
//==============================================================

class NS12Manager
{
public:
    enum OutputWord : uint8_t
    {
        OUT_HEARTBEAT = 0,  // NS MW00000
        OUT_CAMERA_FPS_X10, // NS MW00001
        OUT_MIN_TEMP_X10,   // NS MW00002
        OUT_MAX_TEMP_X10,   // NS MW00003
        OUT_AVG_TEMP_X10,   // NS MW00004
        OUT_GOOD_FRAMES,    // NS MW00005
        OUT_FAILED_FRAMES,  // NS MW00006
        OUT_SYSTEM_STATE,   // NS MW00007
        OUT_STATUS_WORD,    // NS MW00008
        OUTPUT_WORD_COUNT
    };

    static constexpr uint16_t OUTPUT_START_WORD = 0U;
    static constexpr uint16_t TEST_INPUT_WORD = 10U;             // NS $W10 (MW00010) -- operator test input
    static constexpr uint16_t MATRIX_COLUMN_WORDS = MATRIX_ROWS; // 8 words/column
    static constexpr uint32_t MATRIX_FULL_REFRESH_INTERVAL_MS = 1000UL;

    NS12Manager() : serialPort(1) {}

    void begin()
    {
        Serial.println();
        Serial.println(F("NS12 MEMORY LINK INITIALIZATION"));
        Serial.println(F("----------------------------------------------------"));
        Serial.print(F("Baud rate          : "));
        Serial.println(NS_BAUD);
        Serial.print(F("ESP TX GPIO        : "));
        Serial.println(NS_TX_PIN);
        Serial.print(F("ESP RX GPIO        : "));
        Serial.println(NS_RX_PIN);
        Serial.println(F("Format             : 8 data, no parity, 1 stop"));
        Serial.println(F("Physical layer     : HIN232CP full-duplex RS-232"));
        Serial.println(F("HMI                : NS12-TS00B-V2, Memory Link mode"));

        serialPort.setTxBufferSize(4096);
        serialPort.begin(NS_BAUD, SERIAL_8N1, NS_RX_PIN, NS_TX_PIN);
        clearReceiveBuffer();

        lastWriteMs = millis();
        lastReadMs = millis();
        lastMatrixColumnMs = millis();
        initialized = true;

        Serial.println(F("NS12 Memory Link manager started."));
        Serial.println(F("ESP outputs        : MW00000...MW00008"));
        Serial.println(F("NS12 test input    : MW00010"));
        Serial.print(F("NS12 color matrix  : $W"));
        Serial.print(MATRIX_START_WORD);
        Serial.print(F(" x "));
        Serial.print(MATRIX_CELL_COUNT);
        Serial.println(F(" words, column-major, stride 8 (16x8)"));
    }

    // Must be called every loop() iteration. Never blocks: the read
    // side is a small state machine that spans multiple service()
    // calls instead of waiting in-place for the NS12's response.
    void service(float fps,
                 float minimumC,
                 float maximumC,
                 float averageC,
                 uint32_t goodFrames,
                 uint32_t badFrames,
                 SystemState currentState,
                 bool cameraDetected,
                 bool cameraInitialized,
                 bool frameValid,
                 bool ioInitialized,
                 bool ioServiceAllowed,
                 const uint8_t *matrixIndices)
    {
        if (!initialized)
        {
            return;
        }

        const uint32_t nowMs = millis();

        if (nowMs - lastWriteMs >= NS_WRITE_INTERVAL_MS)
        {
            lastWriteMs = nowMs;
            buildOutputImage(fps, minimumC, maximumC, averageC,
                             goodFrames, badFrames, currentState,
                             cameraDetected, cameraInitialized, frameValid,
                             ioInitialized, ioServiceAllowed);
            (void)writeWords(OUTPUT_START_WORD, outputWords, OUTPUT_WORD_COUNT);
        }

        // Add near the other matrix state:
        uint16_t nextMatrixColumnToWrite = 0U;
        uint32_t lastMatrixColumnSendMs = 0U;
        constexpr uint32_t MATRIX_COLUMN_SEND_INTERVAL_MS = 25UL; // start here, then sweep

        if (matrixIndices != nullptr &&
            (nowMs - lastMatrixColumnMs >= MATRIX_COLUMN_SEND_INTERVAL_MS))
        {
            lastMatrixColumnMs = nowMs;
            writeMatrixColumn(matrixIndices, nextColumnToWrite);
            nextColumnToWrite = (nextColumnToWrite + 1U) % MATRIX_COLS;
        }

        if (!readPending && (nowMs - lastReadMs >= NS_READ_INTERVAL_MS))
        {
            lastReadMs = nowMs;

            if (sendReadRequest(TEST_INPUT_WORD, 1U))
            {
                readPending = true;
                readSentMs = nowMs;
                readLineUsed = 0;
            }
        }

        if (readPending)
        {
            pollRead(nowMs);
        }
    }

    void printDiagnostics() const
    {
        Serial.print(F("NS12 initialized    : "));
        Serial.println(initialized ? F("YES") : F("NO"));
        Serial.print(F("NS12 writes         : "));
        Serial.println(writeCommandCounter);
        Serial.print(F("NS12 matrix writes  : "));
        Serial.println(matrixWriteCounter);
        Serial.print(F("NS12 read requests  : "));
        Serial.println(readCommandCounter);
        Serial.print(F("NS12 reads OK       : "));
        Serial.println(readOkCounter);
        Serial.print(F("NS12 timeouts       : "));
        Serial.println(timeoutCounter);
        Serial.print(F("NS12 parse errors   : "));
        Serial.println(parseErrorCounter);
        Serial.print(F("NS12 resync discards: "));
        Serial.println(resyncDiscardCounter);
        Serial.print(F("NS12 MW00010        : "));
        Serial.println(testInputValue);
    }

private:
    HardwareSerial serialPort;
    bool initialized = false;
    uint16_t outputWords[OUTPUT_WORD_COUNT] = {};
    uint16_t matrixColumnBuffer[MATRIX_COLUMN_WORDS] = {};
    uint16_t testInputValue = 0U;
    uint32_t heartbeatCounter = 0U;
    uint32_t writeCommandCounter = 0U;
    uint32_t matrixWriteCounter = 0U;
    uint32_t readCommandCounter = 0U;
    uint32_t readOkCounter = 0U;
    uint32_t timeoutCounter = 0U;
    uint32_t parseErrorCounter = 0U;
    uint32_t lastWriteMs = 0U;
    uint32_t lastReadMs = 0U;
    uint32_t lastMatrixColumnMs = 0U;

    // Non-blocking single-word read state.
    bool readPending = false;
    uint32_t readSentMs = 0U;
    char readLineBuffer[32] = {};
    size_t readLineUsed = 0U;
    uint32_t resyncDiscardCounter = 0U;

    static uint16_t toUnsignedX10(float value)
    {
        if (!isfinite(value) || value <= 0.0f)
            return 0U;
        if (value >= 6553.5f)
            return 65535U;
        return static_cast<uint16_t>(lroundf(value * 10.0f));
    }

    void buildOutputImage(float fps,
                          float minimumC,
                          float maximumC,
                          float averageC,
                          uint32_t goodFrames,
                          uint32_t badFrames,
                          SystemState currentState,
                          bool cameraDetected,
                          bool cameraInitialized,
                          bool frameValid,
                          bool ioInitialized,
                          bool ioServiceAllowed)
    {
        outputWords[OUT_HEARTBEAT] = static_cast<uint16_t>(++heartbeatCounter);
        outputWords[OUT_CAMERA_FPS_X10] = toUnsignedX10(fps);
        outputWords[OUT_MIN_TEMP_X10] = toUnsignedX10(minimumC);
        outputWords[OUT_MAX_TEMP_X10] = toUnsignedX10(maximumC);
        outputWords[OUT_AVG_TEMP_X10] = toUnsignedX10(averageC);
        outputWords[OUT_GOOD_FRAMES] = static_cast<uint16_t>(goodFrames);
        outputWords[OUT_FAILED_FRAMES] = static_cast<uint16_t>(badFrames);
        outputWords[OUT_SYSTEM_STATE] = static_cast<uint16_t>(currentState);

        uint16_t status = 0U;
        if (cameraDetected)
            status |= (1U << 0);
        if (cameraInitialized)
            status |= (1U << 1);
        if (frameValid)
            status |= (1U << 2);
        if (ioInitialized)
            status |= (1U << 3);
        if (ioServiceAllowed)
            status |= (1U << 4);
        if (currentState == SystemState::FaultStop)
            status |= (1U << 15);
        outputWords[OUT_STATUS_WORD] = status;
    }

    // Sends ONE column (8 words). address = MATRIX_START_WORD +
    // columnIndex*MATRIX_COLUMN_WORDS, matching the HMI's confirmed
    // column-major layout. Kept small (8 words, same order of magnitude
    // as the already-proven 9-word telemetry write) to avoid any
    // undocumented per-command word-count cap on this PT. Called 16x
    // back-to-back from service() every MATRIX_FULL_REFRESH_INTERVAL_MS.

    void writeMatrixColumn(const uint8_t *matrixIndices, uint16_t columnIndex)
    {
        const uint16_t base = columnIndex * MATRIX_COLUMN_WORDS;
        for (uint16_t i = 0U; i < MATRIX_COLUMN_WORDS; i++)
        {
            matrixColumnBuffer[i] = matrixIndices[base + i];
        }

        const uint16_t address = static_cast<uint16_t>(MATRIX_START_WORD + base);
        if (writeWords(address, matrixColumnBuffer, MATRIX_COLUMN_WORDS))
        {
            matrixWriteCounter++;
        }
    }

    static void printRawByte(uint8_t value)
    {
#if NS12_DEBUG_RAW_RX
        if (value == 0x1B)
        {
            Serial.print(F("[ESC]"));
            return;
        }
        if (value == 0x0D)
        {
            Serial.print(F("[CR]"));
            return;
        }
        if (value >= 0x20 && value < 0x7F)
        {
            Serial.print(static_cast<char>(value));
            return;
        }
        Serial.print('[');
        if (value < 0x10)
            Serial.print('0');
        Serial.print(value, HEX);
        Serial.print(']');
#else
        (void)value;
#endif
    }

    void clearReceiveBuffer()
    {
        while (serialPort.available() > 0)
        {
            (void)serialPort.read();
        }
    }

    // WM: write PT memory ($W). ESC = 0x1B on this unit -- .
    bool writeWords(uint16_t startAddress,
                    const uint16_t *data,
                    uint8_t wordCount)
    {
        if (data == nullptr || wordCount == 0U || wordCount > 64U)
            return false;

        char command[420];
        size_t used = 0U;
        command[used++] = 0x1B;
        command[used++] = 'W';
        command[used++] = 'M';
        command[used++] = '0';

        int written = snprintf(&command[used], sizeof(command) - used,
                               "%04X%02u", startAddress, wordCount);
        if (written < 0 || used + static_cast<size_t>(written) >= sizeof(command))
            return false;
        used += static_cast<size_t>(written);

        for (uint8_t index = 0U; index < wordCount; index++)
        {
            if (index > 0U)
                command[used++] = ',';
            written = snprintf(&command[used], sizeof(command) - used, "%X", data[index]);
            if (written < 0 || used + static_cast<size_t>(written) >= sizeof(command))
                return false;
            used += static_cast<size_t>(written);
        }

        if (used >= sizeof(command) - 1U)
            return false;
        command[used++] = '\r';

#if NS12_DEBUG_RAW_RX
        Serial.print(F("NS12 TX (write): "));
        for (size_t i = 0; i < used; i++)
            printRawByte(static_cast<uint8_t>(command[i]));
        Serial.println();
#endif

        const size_t sent = serialPort.write(
            reinterpret_cast<const uint8_t *>(command), used);

        if (sent != used)
            return false;
        writeCommandCounter++;
        return true;
    }

    // Read PT memory ($W). ESC = 0x1B. Sends the request only --
    // the reply is collected by pollRead() across subsequent service()
    // calls so the caller's loop() is never blocked waiting for it.
    bool sendReadRequest(uint16_t startAddress, uint8_t wordCount)
    {
        if (wordCount == 0U || wordCount > 50U)
            return false;

        char command[16];
        const int length = snprintf(command, sizeof(command),
                                    "%cRM0%04X%02u\r", 0x1B,
                                    startAddress, wordCount);
        if (length <= 0 || length >= static_cast<int>(sizeof(command)))
            return false;

#if NS12_DEBUG_RAW_RX
        Serial.print(F("NS12 TX (read) : "));
        for (int i = 0; i < length; i++)
            printRawByte(static_cast<uint8_t>(command[i]));
        Serial.println();
#endif

        clearReceiveBuffer();
        const size_t sent = serialPort.write(
            reinterpret_cast<const uint8_t *>(command),
            static_cast<size_t>(length));
        serialPort.flush();
        if (sent != static_cast<size_t>(length))
            return false;

        readCommandCounter++;
        return true;
    }

    static bool parseHexWord(const char *textValue, uint16_t &value)
    {
        if (textValue == nullptr || *textValue == '\0')
            return false;
        char *endPointer = nullptr;
        const unsigned long parsed = strtoul(textValue, &endPointer, 16);
        if (endPointer == textValue || *endPointer != '\0' || parsed > 0xFFFFUL)
            return false;
        value = static_cast<uint16_t>(parsed);
        return true;
    }

    // Non-blocking poll: consumes whatever bytes are currently available
    // without waiting. Completes the pending read only once a full line
    // (terminated by CR) has arrived, or aborts it on timeout/overflow.
    // Safe to call every loop() iteration.
    void pollRead(uint32_t nowMs)
    {
        while (serialPort.available() > 0 && readLineUsed < sizeof(readLineBuffer) - 1U)
        {
            const int incoming = serialPort.read();
            if (incoming < 0)
                break;
            const uint8_t byteValue = static_cast<uint8_t>(incoming);
#if NS12_DEBUG_RAW_RX
            if (readLineUsed == 0U)
            {
                Serial.print(F("NS12 RX: "));
            }
            printRawByte(byteValue);
#endif
            const char character = static_cast<char>(byteValue);

            // A genuine RM response always starts with ESC (0x1B).
            // Anything arriving before we've seen that first ESC is stray
            // (e.g. an ack/echo byte from an overlapping WM write) and is
            // discarded rather than corrupting the line.
            if (readLineUsed == 0U && byteValue != 0x1B)
            {
                resyncDiscardCounter++;
                continue;
            }

            if (character == '\r')
            {
#if NS12_DEBUG_RAW_RX
                Serial.println();
#endif
                readLineBuffer[readLineUsed] = '\0';
                if (parseReadResponse(readLineBuffer, readLineUsed, TEST_INPUT_WORD, 1U))
                {
                    readOkCounter++;
                }
                else
                {
                    parseErrorCounter++;
                }
                readPending = false;
                return;
            }

            readLineBuffer[readLineUsed++] = character;
        }

        if (!readPending)
            return; // completed above

        if (readLineUsed >= sizeof(readLineBuffer) - 1U)
        {
            parseErrorCounter++;
            readPending = false;
            return;
        }

        if (nowMs - readSentMs > NS_RESPONSE_TIMEOUT_MS)
        {
            timeoutCounter++;
            readPending = false;
        }
    }

    // Field offset (bytes after ESC 'R' 'M') where the 4-hex-digit address
    // begins in an RM response. Two candidates:
    //   3 -- manual-documented framing: ESC R M AAAA LL data CR (no *S echo)
    //   4 -- this-unit-quirk framing:    ESC R M S AAAA LL data CR (S echoed,
    //        mirroring the same manual-vs-firmware mismatch already found
    //        and fixed on the WM/write side -- see the ESC=0x1B note above)
    // Auto-detected on the first response that validates cleanly against
    // the address/count we actually asked for, then pinned for the rest
    // of the run and reported once over Serial. Delete the losing
    // candidate from candidateFieldOffsets once this has been confirmed
    // stable, to tighten the parser back up.
    int8_t detectedFieldOffset = -1;

    bool parseReadResponse(const char *response, size_t responseLength,
                           uint16_t expectedAddress, uint8_t expectedCount)
    {
        if (responseLength < 9U ||
            static_cast<uint8_t>(response[0]) != 0x1B ||
            response[1] != 'R' || response[2] != 'M')
        {
            return false;
        }

        static const uint8_t candidateFieldOffsets[] = {3U, 4U};

        for (uint8_t offset : candidateFieldOffsets)
        {
            const size_t headerLength = static_cast<size_t>(offset) + 6U; // 4-hex addr + 2-dec count
            if (responseLength < headerLength)
                continue;

            char addressText[5] = {response[offset], response[offset + 1],
                                   response[offset + 2], response[offset + 3], '\0'};
            char countText[3] = {response[offset + 4], response[offset + 5], '\0'};

            const uint16_t returnedAddress = static_cast<uint16_t>(strtoul(addressText, nullptr, 16));
            const uint8_t returnedCount = static_cast<uint8_t>(strtoul(countText, nullptr, 10));

            if (returnedAddress != expectedAddress || returnedCount != expectedCount)
            {
                continue;
            }

            char dataText[16];
            const size_t dataLength = responseLength - headerLength;
            if (dataLength == 0U || dataLength >= sizeof(dataText))
                continue;
            memcpy(dataText, &response[headerLength], dataLength);
            dataText[dataLength] = '\0';

            uint16_t parsedValue = 0U;
            if (!parseHexWord(dataText, parsedValue))
                continue;

            if (detectedFieldOffset != static_cast<int8_t>(offset))
            {
                detectedFieldOffset = static_cast<int8_t>(offset);
                Serial.print(F("NS12: RM response framing locked -- field offset "));
                Serial.print(offset);
                Serial.println((offset == 3U)
                                   ? F(" (manual framing, no *S echoed)")
                                   : F(" (unit echoes *S in response -- manual framing does not apply here)"));
            }

            testInputValue = parsedValue;
            return true;
        }

        return false;
    }
};

NS12Manager ns12;

//==============================================================
// FUNCTION PROTOTYPES
//==============================================================

void printStartupBanner();
void printBoardInformation();
void initializeStatusLed();
void setStatusLed(uint8_t red, uint8_t green, uint8_t blue);
void runI2CScanner();
bool isI2CAddressPresent(uint8_t address);
bool initializeMLX90640();
bool readMLX90640Frame();
void calculateFrameStatistics();
void updateFrameRate();
void updateColorMatrix();
void serviceCaptureStateMachine();
void printDiagnostics();
void printCameraConnectionHelp();
void attemptCameraRecovery();
void printBytesAsKilobytes(const char *label, size_t valueBytes);
void initializeMCP23017();
void serviceMCP23017();
void readMCP23017Inputs();
void writeChangedMCP23017Outputs();
void setMcpOutput(McpOutputBit outputBit, bool state);
bool getMcpInput(McpInputPin inputPin);
bool isMcpServiceAllowed();
void processSerialTestCommands();
void setSystemState(SystemState newState);
const __FlashStringHelper *systemStateName(SystemState state);

//==============================================================
// STARTUP AND BOARD DIAGNOSTICS
//==============================================================

void printStartupBanner()
{
    Serial.println();
    Serial.println(F("===================================================="));
    Serial.println(F(" HotMelt_MLX90640_80032_9_8_0"));
    Serial.println(F(" Thermal Glue Inspection System"));
    Serial.println(F(" MLX90640 + State + MCP23017 + NS12Manager + Color Matrix"));
    Serial.println(F("===================================================="));
}

void printBoardInformation()
{
    Serial.println();
    Serial.println(F("CONTROLLER INFORMATION"));
    Serial.println(F("----------------------------------------------------"));

    Serial.print(F("CPU frequency      : "));
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(F(" MHz"));

    printBytesAsKilobytes("Flash size         : ", ESP.getFlashChipSize());
    printBytesAsKilobytes("Free heap          : ", ESP.getFreeHeap());
    printBytesAsKilobytes("Minimum free heap  : ", ESP.getMinFreeHeap());

    Serial.print(F("PSRAM detected     : "));
    Serial.println(psramFound() ? F("YES") : F("NO"));

    if (psramFound())
    {
        printBytesAsKilobytes("PSRAM size         : ", ESP.getPsramSize());
        printBytesAsKilobytes("Free PSRAM         : ", ESP.getFreePsram());
    }
    else
    {
        Serial.println(F("WARNING: PSRAM is not available."));
        Serial.println(F("The MLX test can still run because its frame"));
        Serial.println(F("buffer is stored in normal internal RAM."));
    }

    Serial.print(F("Internal frame RAM : "));
    Serial.print(sizeof(mlxFrame));
    Serial.println(F(" bytes"));
}

void printBytesAsKilobytes(const char *label, size_t valueBytes)
{
    Serial.print(label);
    Serial.print(valueBytes / 1024.0f, 1);
    Serial.println(F(" kB"));
}

//==============================================================
// ONBOARD RGB STATUS LED
//==============================================================

void initializeStatusLed()
{
    statusLed.begin();
    statusLed.clear();
    statusLed.show();
}

void setStatusLed(uint8_t red, uint8_t green, uint8_t blue)
{
    statusLed.setPixelColor(0, statusLed.Color(red, green, blue));
    statusLed.show();
}

//==============================================================
// I2C DIAGNOSTICS
//==============================================================

void runI2CScanner()
{
    Serial.println();
    Serial.println(F("I2C SCANNER"));
    Serial.println(F("----------------------------------------------------"));

    uint8_t deviceCount = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        const uint8_t result = Wire.endTransmission();

        if (result == 0)
        {
            Serial.print(F("Device found       : 0x"));

            if (address < 0x10)
            {
                Serial.print('0');
            }

            Serial.println(address, HEX);
            deviceCount++;
        }
    }

    if (deviceCount == 0)
    {
        Serial.println(F("No I2C devices found."));
    }
    else
    {
        Serial.print(F("Total devices      : "));
        Serial.println(deviceCount);
    }
}

bool isI2CAddressPresent(uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

//==============================================================
// MLX90640 INITIALIZATION
//==============================================================

bool initializeMLX90640()
{
    Serial.println();
    Serial.println(F("MLX90640 INITIALIZATION"));
    Serial.println(F("----------------------------------------------------"));

    if (!mlx.begin(MLX_I2C_ADDRESS, &Wire))
    {
        return false;
    }

    // Chess mode alternates pixels in a checkerboard pattern and is
    // normally preferred for thermal imaging.
    mlx.setMode(MLX90640_CHESS);

    // 18-bit ADC gives a good balance of resolution and speed.
    mlx.setResolution(MLX90640_ADC_18BIT);

    // 32Hz @ 800kHz I2C
    // is the confirmed ceiling for this hardware (0 failed frames, MCP
    // still enumerating, FPS ~7.8-8.7). Do not push refresh rate further
    // without also raising I2C_CLOCK_HZ again, and re-verify MCP23017
    // survives any such change -- it silently dropped off the bus at
    // 1MHz once already (see I2C_CLOCK_HZ comment above).
    mlx.setRefreshRate(MLX90640_32_HZ);

    Serial.print(F("I2C address        : 0x"));
    Serial.println(MLX_I2C_ADDRESS, HEX);
    Serial.println(F("Readout mode       : CHESS"));
    Serial.println(F("ADC resolution     : 18 bit"));
    Serial.println(F("Requested rate     : 32 Hz "));
    Serial.println(F("Frame dimensions   : 32 x 24"));
    Serial.print(F("Pixel count        : "));
    Serial.println(MLX_PIXEL_COUNT);

    return true;
}

//==============================================================
// MLX90640 FRAME ACQUISITION
//==============================================================

bool readMLX90640RawFrame()
{
    //  const int readResult = mlx.getFrame(mlxFrame);
    const int readResult = mlx.getRawFrame(rawFrame0, rawFrame1);
    if (readResult == 0)
    {
        return true;
    }

    Serial.print(F("WARNING: MLX raw frame read failed. Code: "));
    Serial.println(readResult);
    return false;
}

bool readMLX90640Frame()
{
    // getFrame() returns 0 when a complete frame was read successfully.
    const int readResult = mlx.getFrame(mlxFrame);

    if (readResult == 0)
    {
        return true;
    }

    Serial.print(F("WARNING: MLX frame read failed. Code = "));
    Serial.println(readResult);

    return false;
}

void calculateFrameStatistics()
{
    float sumTemperatureC = 0.0f;
    float frameMinimumC = INFINITY;
    float frameMaximumC = -INFINITY;
    uint16_t validPixelCount = 0;

    for (uint16_t pixelIndex = 0;
         pixelIndex < MLX_PIXEL_COUNT;
         pixelIndex++)
    {
        const float temperatureC = mlxFrame[pixelIndex];

        // Ignore NaN or infinite results so one bad pixel cannot
        // corrupt all frame statistics.
        if (!isfinite(temperatureC))
        {
            continue;
        }

        if (temperatureC < frameMinimumC)
        {
            frameMinimumC = temperatureC;
        }

        if (temperatureC > frameMaximumC)
        {
            frameMaximumC = temperatureC;
        }

        sumTemperatureC += temperatureC;
        validPixelCount++;
    }

    if (validPixelCount > 0)
    {
        minimumTemperatureC = frameMinimumC;
        maximumTemperatureC = frameMaximumC;
        averageTemperatureC =
            sumTemperatureC / static_cast<float>(validPixelCount);
    }
    else
    {
        minimumTemperatureC = NAN;
        maximumTemperatureC = NAN;
        averageTemperatureC = NAN;
    }
}

void updateFrameRate()
{
    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - fpsWindowStartMs;

    if (elapsedMs >= 2000UL)
    {
        measuredFramesPerSecond =
            (static_cast<float>(fpsWindowFrameCount) * 1000.0f) /
            static_cast<float>(elapsedMs);

        fpsWindowStartMs = nowMs;
        fpsWindowFrameCount = 0;
    }
}

// Downsamples ANY 32x24 source frame (live mlxFrame, or an averaged
// capture buffer) into the 16x8 colorMatrix by averaging each 2(col) x
// 3(row) block, then scales into a palette index 0..(LEVELS-1) using
// MATRIX_TEMP_MIN_C/MAX_C. NaN/inf source pixels are skipped; a block
// with zero valid pixels falls back to index 0.
void buildColorMatrixFromFrame(const float *sourceFrame)
{
    for (uint16_t cellRow = 0; cellRow < MATRIX_ROWS; cellRow++)
    {
        for (uint16_t cellCol = 0; cellCol < MATRIX_COLS; cellCol++)
        {
            float blockSum = 0.0f;
            uint16_t blockValidCount = 0;

            const uint16_t sourceRowStart = cellRow * MATRIX_BLOCK_ROWS;
            const uint16_t sourceColStart = cellCol * MATRIX_BLOCK_COLS;

            for (uint16_t sourceRow = sourceRowStart;
                 sourceRow < sourceRowStart + MATRIX_BLOCK_ROWS;
                 sourceRow++)
            {
                for (uint16_t sourceCol = sourceColStart;
                     sourceCol < sourceColStart + MATRIX_BLOCK_COLS;
                     sourceCol++)
                {
                    const uint16_t pixelIndex = sourceRow * MLX_COLUMNS + sourceCol;
                    const float temperatureC = sourceFrame[pixelIndex];

                    if (isfinite(temperatureC))
                    {
                        blockSum += temperatureC;
                        blockValidCount++;
                    }
                }
            }

            uint8_t colorIndex = 0U;

            if (blockValidCount > 0U)
            {
                const float blockAverageC = blockSum / static_cast<float>(blockValidCount);
                float normalized = (blockAverageC - MATRIX_TEMP_MIN_C) /
                                   (MATRIX_TEMP_MAX_C - MATRIX_TEMP_MIN_C);

                if (normalized < 0.0f)
                    normalized = 0.0f;
                if (normalized > 1.0f)
                    normalized = 1.0f;

                colorIndex = static_cast<uint8_t>(
                    normalized * static_cast<float>(MATRIX_COLOR_LEVELS - 1U) + 0.5f);
            }

            // Column-major fill: index = col*MATRIX_ROWS + row, matching the
            // HMI's confirmed address(col,row) = MATRIX_START_WORD + col*8 + row.
            colorMatrix[cellCol * MATRIX_ROWS + cellRow] = colorIndex;
        }
    }
}

// Downsamples the LIVE mlxFrame into colorMatrix -- skipped entirely
// while a capture is latched (frozen picture) so the snapshot doesn't
// get overwritten by live data until explicitly rearmed with 'R'.
void updateColorMatrix()
{
    if (matrixTestPatternActive)
    {
        // Fixed, clearly-visible pattern: index cycles 1-9 (never 0) by
        // column so a working link should show 9 distinct color bands
        // sweeping across the grid regardless of camera data.
        for (uint16_t cellCol = 0; cellCol < MATRIX_COLS; cellCol++)
        {
            const uint8_t bandIndex = static_cast<uint8_t>((cellCol % 9U) + 1U);
            for (uint16_t cellRow = 0; cellRow < MATRIX_ROWS; cellRow++)
            {
                colorMatrix[cellCol * MATRIX_ROWS + cellRow] = bandIndex;
            }
        }
        return;
    }

    if (captureLatched)
    {
        return; // frozen snapshot -- do not overwrite with live data
    }

    buildColorMatrixFromFrame(mlxFrame);
}

// Called once per successful camera frame, AFTER updateColorMatrix().
// Independent state machine: watches for CAPTURE_TRIGGER_TEMP_C (or a
// forced trigger via the 'C' serial command), then averages the next
// CAPTURE_SAMPLE_COUNT frames together and freezes the result as the
// HMI picture via captureLatched. Runs regardless of matrixTestPatternActive
// so the test pattern and a real capture never fight each other --
// capture logic simply does nothing while the test pattern is active.
void serviceCaptureStateMachine()
{
    if (matrixTestPatternActive || captureLatched)
    {
        return;
    }

    if (!captureSamplingInProgress)
    {
        if (!captureArmed)
        {
            return;
        }
        if (maximumTemperatureC < CAPTURE_TRIGGER_TEMP_C)
        {
            return;
        }

        // Trigger: start a new sampling window. Accumulator seeded to
        // -INFINITY per pixel so the first real reading always "wins"
        // the max-hold comparison below.
        captureSamplingInProgress = true;
        captureSamplesCollected = 0U;
        for (uint16_t i = 0U; i < MLX_PIXEL_COUNT; i++)
        {
            captureAccumulator[i] = -INFINITY;
        }
        Serial.print(F("CAPTURE: triggered at "));
        Serial.print(maximumTemperatureC, 2);
        Serial.println(F(" C -- sampling..."));
    }

    // MAX-HOLD: keep the hottest value seen at each cell across the
    // whole window, not an average. The tube moves under a fixed FOV,
    // so different frames in the window see different physical sections
    // of it -- averaging would dilute a glue trace that moved out of
    // frame partway through, exactly the failure mode a QC check needs
    // to avoid. Max-hold instead composites everything that appeared
    // anywhere during the transit into one image.
    for (uint16_t i = 0U; i < MLX_PIXEL_COUNT; i++)
    {
        const float temperatureC = mlxFrame[i];
        if (isfinite(temperatureC) && temperatureC > captureAccumulator[i])
        {
            captureAccumulator[i] = temperatureC;
        }
    }
    captureSamplesCollected++;

    if (captureSamplesCollected < CAPTURE_SAMPLE_COUNT)
    {
        return;
    }

    // Sampling window complete: the accumulator already IS the max-hold
    // composite (no averaging division needed) -- downsample and freeze.
    buildColorMatrixFromFrame(captureAccumulator);

    captureSamplingInProgress = false;
    captureLatched = true;
    captureArmed = false;

    Serial.print(F("CAPTURE: complete, "));
    Serial.print(CAPTURE_SAMPLE_COUNT);
    Serial.println(F(" frames averaged -- HMI picture frozen. Send 'R' to rearm."));
}

//==============================================================
// CAMERA RECOVERY
//==============================================================

void attemptCameraRecovery()
{
    Serial.println();
    Serial.println(F("CAMERA RECOVERY"));
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("Five consecutive frame reads failed."));
    Serial.println(F("Reinitializing I2C and MLX90640..."));

    mlxInitialized = false;
    consecutiveFrameFailures = 0;

    Wire.end();
    delay(50);
    Wire.begin(MLX_SDA_PIN, MLX_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(1000);
    delay(50);

    mlxDetected = isI2CAddressPresent(MLX_I2C_ADDRESS);

    if (mlxDetected)
    {
        mlxInitialized = initializeMLX90640();
    }

    if (mlxInitialized)
    {
        Serial.println(F("Camera recovery successful."));
        setStatusLed(0, 25, 0);
    }
    else
    {
        Serial.println(F("Camera recovery failed."));
        printCameraConnectionHelp();
        setStatusLed(30, 0, 0);
    }
}

//==============================================================
// PERIODIC DIAGNOSTIC REPORT
//==============================================================

void printDiagnostics()
{
    Serial.println();
    Serial.println(F("LIVE CAMERA DIAGNOSTICS"));
    Serial.println(F("----------------------------------------------------"));

    Serial.print(F("System state       : "));
    Serial.println(systemStateName(systemState));

    Serial.print(F("MCP initialized    : "));
    Serial.println(mcpInitialized ? F("YES") : F("NO"));

    Serial.print(F("MCP service allowed: "));
    Serial.println(isMcpServiceAllowed() ? F("YES") : F("NO"));

    Serial.print(F("MCP service count  : "));
    Serial.println(mcpServiceCount);

    Serial.print(F("Camera detected    : "));
    Serial.println(mlxDetected ? F("YES") : F("NO"));

    Serial.print(F("Camera initialized : "));
    Serial.println(mlxInitialized ? F("YES") : F("NO"));

    Serial.print(F("Last frame         : "));
    Serial.println(lastFrameValid ? F("OK") : F("FAILED"));

    Serial.print(F("Capture state      : "));
    if (captureLatched)
    {
        Serial.println(F("LATCHED (frozen picture, send 'R' to rearm)"));
    }
    else if (captureSamplingInProgress)
    {
        Serial.print(F("SAMPLING ("));
        Serial.print(captureSamplesCollected);
        Serial.print(F("/"));
        Serial.print(CAPTURE_SAMPLE_COUNT);
        Serial.println(F(")"));
    }
    else if (captureArmed)
    {
        Serial.println(F("ARMED (watching for trigger)"));
    }
    else
    {
        Serial.println(F("IDLE"));
    }

    Serial.print(F("Measured FPS       : "));
    Serial.println(measuredFramesPerSecond, 2);

    Serial.print(F("Minimum temp       : "));
    if (isfinite(minimumTemperatureC))
    {
        Serial.print(minimumTemperatureC, 2);
        Serial.println(F(" C"));
    }
    else
    {
        Serial.println(F("---"));
    }

    Serial.print(F("Maximum temp       : "));
    if (isfinite(maximumTemperatureC))
    {
        Serial.print(maximumTemperatureC, 2);
        Serial.println(F(" C"));
    }
    else
    {
        Serial.println(F("---"));
    }

    Serial.print(F("Average temp       : "));
    if (isfinite(averageTemperatureC))
    {
        Serial.print(averageTemperatureC, 2);
        Serial.println(F(" C"));
    }
    else
    {
        Serial.println(F("---"));
    }

    Serial.print(F("Good frames        : "));
    Serial.println(successfulFrameCount);

    Serial.print(F("Failed frames      : "));
    Serial.println(failedFrameCount);

    if (lastSuccessfulFrameMs > 0)
    {
        Serial.print(F("Last good frame    : "));
        Serial.print(millis() - lastSuccessfulFrameMs);
        Serial.println(F(" ms ago"));
    }

    ns12.printDiagnostics();

    printBytesAsKilobytes("Free heap          : ", ESP.getFreeHeap());

    if (psramFound())
    {
        printBytesAsKilobytes("Free PSRAM         : ", ESP.getFreePsram());
    }
}

//==============================================================
// CONNECTION HELP
//==============================================================

void printCameraConnectionHelp()
{
    Serial.println();
    Serial.println(F("ERROR: MLX90640 NOT FOUND AT I2C ADDRESS 0x33"));
    Serial.println(F("----------------------------------------------------"));
    Serial.println(F("Check these connections:"));
    Serial.println(F("ESP32 3V3  -> MLX VIN"));
    Serial.println(F("ESP32 GND  -> MLX GND"));
    Serial.println(F("ESP32 IO8  -> MLX SDA"));
    Serial.println(F("ESP32 IO9  -> MLX SCL"));
    Serial.println(F("Check that the module is set/bridged for I2C mode."));
    Serial.println(F("Keep SDA and SCL wiring short for this first test."));
}

//==============================================================
// SYSTEM STATE MANAGER
//==============================================================

void setSystemState(SystemState newState)
{
    if (newState == systemState)
    {
        return;
    }

    Serial.print(F("SYSTEM STATE: "));
    Serial.print(systemStateName(systemState));
    Serial.print(F(" -> "));
    Serial.println(systemStateName(newState));

    systemState = newState;
}

const __FlashStringHelper *systemStateName(SystemState state)
{
    switch (state)
    {
    case SystemState::Startup:
        return F("STARTUP");
    case SystemState::Standby:
        return F("STANDBY");
    case SystemState::WaitingForTube:
        return F("WAITING FOR TUBE");
    case SystemState::InspectingTube:
        return F("INSPECTING TUBE");
    case SystemState::TubeGap:
        return F("TUBE GAP");
    case SystemState::FaultStop:
        return F("FAULT STOP");
    default:
        return F("UNKNOWN");
    }
}

bool isMcpServiceAllowed()
{
    return (systemState == SystemState::Standby) ||
           (systemState == SystemState::TubeGap);
}

//==============================================================
// MCP23017 MACHINE-I/O MANAGER
//==============================================================

void initializeMCP23017()
{
    Serial.println();
    Serial.println(F("MCP23017 INITIALIZATION"));
    Serial.println(F("----------------------------------------------------"));

    mcpDetected = isI2CAddressPresent(MCP23017_I2C_ADDRESS);

    if (!mcpDetected)
    {
        Serial.println(F("MCP23017 not found at I2C address 0x20."));
        Serial.println(F("Camera operation will continue without machine I/O."));
        mcpInitialized = false;
        return;
    }

    if (!mcp.begin_I2C(MCP23017_I2C_ADDRESS, &Wire))
    {
        Serial.println(F("MCP23017 responded but driver initialization failed."));
        mcpInitialized = false;
        return;
    }

    for (uint8_t pin = 0; pin <= 7; pin++)
    {
        mcp.pinMode(pin, OUTPUT);
        mcp.digitalWrite(pin, LOW);
    }

    for (uint8_t pin = 8; pin <= 15; pin++)
    {
        mcp.pinMode(pin, INPUT_PULLUP);
    }

    desiredMcpOutputImage = 0;
    lastWrittenMcpOutputImage = 0xFFFFU;
    mcpInitialized = true;

    // READY is asserted after successful MCP initialization.
    setMcpOutput(MCP_OUT_READY, true);
    writeChangedMCP23017Outputs();

    Serial.println(F("MCP23017 initialization successful."));
    Serial.println(F("Outputs : GPA0..GPA5"));
    Serial.println(F("Inputs  : GPB0..GPB4, active LOW"));
}

void serviceMCP23017()
{
    if (!mcpInitialized || !isMcpServiceAllowed())
    {
        return;
    }

    const uint32_t nowMs = millis();

    if (nowMs - lastMcpServiceMs < MCP_SERVICE_INTERVAL_MS)
    {
        return;
    }

    lastMcpServiceMs = nowMs;

    // One grouped input read and, only if needed, one grouped output write.
    readMCP23017Inputs();
    writeChangedMCP23017Outputs();
    mcpServiceCount++;
}

void readMCP23017Inputs()
{
    // readGPIOAB() reads both ports in one I2C transaction; keep GPB.
    mcpInputImage = static_cast<uint8_t>(mcp.readGPIOAB() >> 8);
}

void writeChangedMCP23017Outputs()
{
    if (desiredMcpOutputImage == lastWrittenMcpOutputImage)
    {
        return;
    }

    // writeGPIOAB() updates both ports together. Only GPA0..GPA5 are used
    // as outputs; GPB remains configured as inputs.
    mcp.writeGPIOAB(desiredMcpOutputImage);
    lastWrittenMcpOutputImage = desiredMcpOutputImage;
}

void setMcpOutput(McpOutputBit outputBit, bool state)
{
    const uint16_t mask = static_cast<uint16_t>(1U << outputBit);

    if (state)
    {
        desiredMcpOutputImage |= mask;
    }
    else
    {
        desiredMcpOutputImage &= static_cast<uint16_t>(~mask);
    }
}

bool getMcpInput(McpInputPin inputPin)
{
    const uint8_t bitNumber = static_cast<uint8_t>(inputPin) - 8U;
    const uint8_t mask = static_cast<uint8_t>(1U << bitNumber);

    // Inputs are active LOW because INPUT_PULLUP is used.
    return (mcpInputImage & mask) == 0U;
}

//==============================================================
// SERIAL FOUNDATION TEST COMMANDS
//==============================================================

void processSerialTestCommands()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());

        switch (command)
        {
        case 'S':
        case 's':
            setSystemState(SystemState::Standby);
            break;
        case 'W':
        case 'w':
            setSystemState(SystemState::WaitingForTube);
            break;
        case 'I':
        case 'i':
            setSystemState(SystemState::InspectingTube);
            break;
        case 'G':
        case 'g':
            setSystemState(SystemState::TubeGap);
            break;
        case 'F':
        case 'f':
            setSystemState(SystemState::FaultStop);
            break;

        case 'M':
        case 'm':
            matrixTestPatternActive = !matrixTestPatternActive;
            Serial.print(F("Matrix test pattern: "));
            Serial.println(matrixTestPatternActive ? F("ON (bands 1-9)") : F("OFF (live thermal)"));
            break;

        case 'X':
        case 'x':
            rawFrameDumpRequested = true;
            Serial.println(F("RAW DUMP: will compare getFrame() vs getRawFrame() on next frame."));
            Serial.print(F(" C"));
            break;

        case 'B':
        case 'b':
            if (!rawBaselineCaptureInProgress)
            {
                rawBaselineCaptureInProgress = true;
                rawBaselineFramesCollected = 0U;
                for (uint16_t i = 0U; i < 834U; i++)
                {
                    rawBaseline0[i] = 0.0f;
                    rawBaseline1[i] = 0.0f;
                }
                Serial.print(F("BASELINE: capturing "));
                Serial.print(RAW_BASELINE_FRAME_COUNT);
                Serial.println(F(" idle raw frames -- point camera at idle scene now."));
            }
            else
            {
                Serial.println(F("BASELINE: capture already in progress."));
            }
            break;

        case 'C':
        case 'c':
            // Force a capture window to start on the NEXT frame,
            // bypassing CAPTURE_TRIGGER_TEMP_C -- useful on the bench
            // when nothing is actually hot enough to trigger it.
            if (!captureLatched && !captureSamplingInProgress)
            {
                captureArmed = true;
                captureSamplingInProgress = true;
                captureSamplesCollected = 0U;
                for (uint16_t i = 0U; i < MLX_PIXEL_COUNT; i++)
                {
                    captureAccumulator[i] = -INFINITY;
                }
                Serial.println(F("CAPTURE: forced -- sampling starts next frame."));
            }
            else
            {
                Serial.println(F("CAPTURE: already latched or sampling -- send 'R' to rearm first."));
            }
            break;

        case 'R':
        case 'r':
            captureLatched = false;
            captureSamplingInProgress = false;
            captureSamplesCollected = 0U;
            captureArmed = true;
            Serial.println(F("CAPTURE: rearmed -- watching for next trigger."));
            break;

        case '1':
            setMcpOutput(MCP_OUT_NORMAL_STOP,
                         (desiredMcpOutputImage & (1U << MCP_OUT_NORMAL_STOP)) == 0U);
            break;
        case '2':
            setMcpOutput(MCP_OUT_FAST_STOP,
                         (desiredMcpOutputImage & (1U << MCP_OUT_FAST_STOP)) == 0U);
            break;
        case '3':
            setMcpOutput(MCP_OUT_HORN,
                         (desiredMcpOutputImage & (1U << MCP_OUT_HORN)) == 0U);
            break;
        case '4':
            setMcpOutput(MCP_OUT_BEACON,
                         (desiredMcpOutputImage & (1U << MCP_OUT_BEACON)) == 0U);
            break;
        case '5':
            setMcpOutput(MCP_OUT_READY,
                         (desiredMcpOutputImage & (1U << MCP_OUT_READY)) == 0U);
            break;
        case '6':
            setMcpOutput(MCP_OUT_WARNING,
                         (desiredMcpOutputImage & (1U << MCP_OUT_WARNING)) == 0U);
            break;

        default:
            break;
        }
    }
}

//==============================================================
// SETUP
//==============================================================

void setup()
{
    // Native USB serial may need a moment after reset.
    Serial.begin(115200);

    const uint32_t serialWaitStartMs = millis();
    while (!Serial && (millis() - serialWaitStartMs < 5000UL))
    {
        delay(10);
    }

    initializeStatusLed();
    setStatusLed(0, 0, 20); // Dim blue during startup.

    printStartupBanner();
    printBoardInformation();

    Serial.println();
    Serial.println(F("I2C INITIALIZATION"));
    Serial.println(F("----------------------------------------------------"));
    Serial.print(F("SDA GPIO           : "));
    Serial.println(MLX_SDA_PIN);
    Serial.print(F("SCL GPIO           : "));
    Serial.println(MLX_SCL_PIN);
    Serial.print(F("I2C clock          : "));
    Serial.print(I2C_CLOCK_HZ);
    Serial.println(F(" Hz"));

    Wire.begin(MLX_SDA_PIN, MLX_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(1000);

    runI2CScanner();
    initializeMCP23017();

    mlxDetected = isI2CAddressPresent(MLX_I2C_ADDRESS);

    if (!mlxDetected)
    {
        setStatusLed(30, 0, 0);
        printCameraConnectionHelp();
    }
    else
    {
        mlxInitialized = initializeMLX90640();

        if (mlxInitialized)
        {
            setStatusLed(0, 25, 0);
            Serial.println();
            Serial.println(F("MLX90640 INITIALIZATION SUCCESSFUL"));
        }
        else
        {
            setStatusLed(30, 10, 0);
            Serial.println();
            Serial.println(F("ERROR: MLX90640 responded at 0x33,"));
            Serial.println(F("but the Adafruit driver could not initialize it."));
        }
    }

    ns12.begin();

    fpsWindowStartMs = millis();
    lastDiagnosticMs = millis();

    setSystemState(SystemState::Standby);

    Serial.println();
    Serial.println(F("Serial test commands: S=Standby, W=Waiting, I=Inspecting,"));
    Serial.println(F("                      G=Tube Gap, F=Fault, 1..6 toggle outputs,"));
    Serial.println(F("                      M=toggle matrix test pattern (bands 1-9),"));
    Serial.println(F("                      C=force capture now, R=rearm capture."));
    Serial.println();
    Serial.println(F("Entering continuous foundation test..."));
    Serial.println(F("===================================================="));

    esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true); // true = panic/reset on timeout
    esp_task_wdt_add(NULL);
}

//==============================================================
// MAIN LOOP
//==============================================================

void loop()
{
    esp_task_wdt_reset();

    processSerialTestCommands();
    serviceMCP23017();

    ns12.service(measuredFramesPerSecond,
                 minimumTemperatureC,
                 maximumTemperatureC,
                 averageTemperatureC,
                 successfulFrameCount,
                 failedFrameCount,
                 systemState,
                 mlxDetected,
                 mlxInitialized,
                 lastFrameValid,
                 mcpInitialized,
                 isMcpServiceAllowed(),
                 lastFrameValid ? colorMatrix : nullptr);

    if (mlxInitialized)
    {
        lastFrameValid = readMLX90640Frame();

        if (lastFrameValid)
        {
            calculateFrameStatistics();

            if (rawFrameDumpRequested)
            {
                rawFrameDumpRequested = false;

                if (readMLX90640RawFrame())
                {
                    Serial.println(F("RAW vs CALIBRATED (subpage0 raw / subpage1 raw / calibrated C):"));
                    static const uint16_t sampleIndices[] = {0, 100, 200, 384, 500, 700};
                    for (uint16_t idx : sampleIndices)
                    {
                        Serial.print(F("  pixel "));
                        Serial.print(idx);
                        Serial.print(F(" : raw0="));
                        Serial.print((int16_t)rawFrame0[idx]);
                        Serial.print(F("  raw1="));
                        Serial.print((int16_t)rawFrame1[idx]);
                        Serial.print(F("  calibrated="));
                        Serial.print(mlxFrame[idx], 2);
                        Serial.print(F(" C"));

                        if (rawBaselineCaptured)
                        {
                            Serial.print(F("  corrected0="));
                            Serial.print((float)(int16_t)rawFrame0[idx] - rawBaseline0[idx], 1);
                            Serial.print(F("  corrected1="));
                            Serial.print((float)(int16_t)rawFrame1[idx] - rawBaseline1[idx], 1);
                        }

                        Serial.println();
                    }
                }
            }

            if (rawBaselineCaptureInProgress)
            {
                if (readMLX90640RawFrame())
                {
                    for (uint16_t i = 0U; i < 834U; i++)
                    {
                        rawBaseline0[i] += (float)(int16_t)rawFrame0[i];
                        rawBaseline1[i] += (float)(int16_t)rawFrame1[i];
                    }
                    rawBaselineFramesCollected++;

                    if (rawBaselineFramesCollected >= RAW_BASELINE_FRAME_COUNT)
                    {
                        for (uint16_t i = 0U; i < 834U; i++)
                        {
                            rawBaseline0[i] /= (float)RAW_BASELINE_FRAME_COUNT;
                            rawBaseline1[i] /= (float)RAW_BASELINE_FRAME_COUNT;
                        }
                        rawBaselineCaptureInProgress = false;
                        rawBaselineCaptured = true;
                        Serial.println(F("BASELINE: capture complete -- 'X' now shows corrected values."));
                    }
                }
            }

            updateFrameRate();
            updateColorMatrix();
            serviceCaptureStateMachine();

            successfulFrameCount++;
            fpsWindowFrameCount++;
            consecutiveFrameFailures = 0;
            lastSuccessfulFrameMs = millis();

            // Brief green heartbeat on every successful frame.
            setStatusLed(0, 18, 0);
        }
        else
        {
            failedFrameCount++;
            consecutiveFrameFailures++;
            setStatusLed(25, 8, 0);

            if (consecutiveFrameFailures >= FRAME_FAILURE_RECOVERY_COUNT)
            {
                attemptCameraRecovery();
            }
        }
    }
    else
    {
        // Retry camera detection every second without locking the CPU.
        static uint32_t lastRetryMs = 0;

        if (millis() - lastRetryMs >= 1000UL)
        {
            lastRetryMs = millis();

            mlxDetected = isI2CAddressPresent(MLX_I2C_ADDRESS);

            if (mlxDetected)
            {
                mlxInitialized = initializeMLX90640();

                if (mlxInitialized)
                {
                    consecutiveFrameFailures = 0;
                    fpsWindowStartMs = millis();
                    fpsWindowFrameCount = 0;
                    setStatusLed(0, 25, 0);
                    Serial.println(F("MLX90640 recovered and initialized."));
                }
            }
        }

        delay(10);
    }

    if (millis() - lastDiagnosticMs >= DIAGNOSTIC_INTERVAL_MS)
    {
        lastDiagnosticMs = millis();
        printDiagnostics();
    }
}
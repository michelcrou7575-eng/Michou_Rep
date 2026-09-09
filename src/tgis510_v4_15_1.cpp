// TGIS-510 -- Thermal Glue Inspection System
// Ref: TGIS-510_cpp_V4_15.1
//
// Home-lab / after-hours project. Separate from the 410 Rotaliner Tubing Seal
// Seam Monitor (factory floor, S7-300/ATmega2560) -- do not conflate.
//
// MERGE NOTE (V4.15.0): the prior V4.14.0 revision of this file was written
// from a project handoff synthesis without access to the actual bench-tested
// firmware, and explicitly flagged its own NS12 wire-protocol implementation
// and 38400-baud claim as unverified ("diff this against whatever is
// actually on disk... before flashing to real hardware"). That protocol
// layer has been replaced here with the one from the actual bench-tested
// HotMelt_MLX90640_80032_9_8_1.cpp snapshot, which documents specific,
// falsifiable bench results (9600 baud measured zero RM timeouts vs ~15% at
// 38400; the ESC=0x1B-for-WM quirk found via live sensor data and a test
// pattern both landing correctly on the physical screen). Where the two
// files disagreed, the version with documented bench evidence won. Board
// diagnostics (I2C scanner, board info), camera-recovery-on-repeated-
// failure, real windowed FPS measurement, an RGB status LED, an ESP task
// watchdog, and the $W0-$W8 telemetry block were all restored from that
// same bench-tested file -- V4.14.0 had dropped them.
//
// Kept from V4.14.0 (real forward progress, not present in the older
// snapshot): encoder position tracking (PCNT), Keyence IV2 trigger/result
// handling, tube presence sensor edge detection, forward-projection tube
// timing, per-strip (strip1/strip2) glue QC evaluation, velocity-adaptive
// HMI display throttling, and the column-paced experimental 32x24 HMI mode
// with an RM-success-rate auto-fallback (whose pacing interval and read
// path are both fixed here -- see the NS12 namespace and NS12Manager).
//
// STILL UNVERIFIED ON REAL HARDWARE: this merge has not itself been bench
// tested. Re-verify the NS12 link (baud, ESC byte, frame framing) and the
// Action-Item placeholder pins below before flashing to the real machine.
//
// FIELD UPDATE (2026-09): a real run of the archived reference file
// (reference/HotMelt_MLX90640_80032_9_8_1.cpp, not this file) surfaced two
// bugs this file inherited by porting that file's logic verbatim:
//   1. RM reads: 0/2968 succeeded on real hardware -- the "confirmed
//      working" claim for non-blocking RM reads was apparently never true
//      for the response-parsing path specifically. parseRmResponse() now
//      dumps the raw rejected bytes on every failure (unconditionally, not
//      gated by a debug flag) so the real response framing can finally be
//      read off directly instead of guessed at a third time.
//   2. Maximum temp reported 782.23C against a 15.71-24.21C frame -- one
//      glitching pixel, unfiltered, became "maximum temp" and would have
//      force-triggered a false QC capture (CAPTURE_TRIGGER_TEMP_C=180C).
//      isPlausibleTemp() (-40C..300C) now gates every pixel used for
//      statistics, the HMI downsample, and the capture trigger/composite/
//      strip evaluation -- see calculateFrameStatistics(), downsampleMaxBlock(),
//      and CaptureController below.
//
// FIELD UPDATE (V4.15.1, this file's own first run): confirmed the
// isPlausibleTemp() fix above holds on real hardware (Implausible pixels: 0,
// sane 20-50C range every report). The RM parse-failure dump then caught
// this file's OWN reads failing too: raw response "ESC WM001F" every time,
// identically -- shaped like our own WM-write signature ('W','M'), not an
// RM reply ('R','M'), so the two-offset guess in parseRmResponse() was
// never the actual bug. NS12Manager::service() now defers the telemetry
// WM write (and serviceDisplayThrottle()/serviceMatrixPacing() defer their
// matrix writes) while an RM read is pending, since RM_READ_TIMEOUT_MS and
// TELEMETRY_WRITE_INTERVAL_MS are close enough (250-300ms) that a write
// could otherwise land mid-read on this shared UART. If the same bogus
// response still appears after this, it points at a genuine hardware
// TX/RX loopback or PT echo rather than a timing overlap -- check wiring.
//
// Also fixed in V4.15.1, unrelated to NS12: a real boot showed
// "Detected size(4096k) smaller than the size in the binary image
// header(8192k)" and a fatal do_core_init assert on every startup --
// platformio.ini's generic devkit board defaulted to 8MB flash against
// this chip's real 4MB (ESP32-S3FH4R2 = Flash 4MB / PSRAM 2MB Quad).
// Fixed in platformio.ini, not in this file.
//
// Industrial QC system detecting hot-melt glue application on tubes moving
// at high speed. Confirms glue presence, temperature, and quantity across
// both glue strips per tube pass, and pushes a stable QC-confirmation image
// to an operator HMI (Omron NS12).
//
// Division of responsibility:
//   - Keyence IV2-G30/G300CA owns hot-melt trace start/end pass/fail. ESP32
//     fires a trigger pulse; Keyence returns a result pulse. It does not
//     detect tube boundaries.
//   - MLX90640 owns strip presence, temperature and quantity (3-4 tube
//     sample window acceptable).
//   - Encoder (single-channel pulse train, no direction) gives real tube
//     position/length, used to project forward and fire triggers at
//     adjustable lead distances ahead of the MLX90640 and Keyence stations.
//   - Tube presence sensor gives the ground-truth leading/trailing edges
//     that projection is anchored to.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>
#include "driver/pcnt.h"

// =====================================================================
// VERSION -- keep filename, header comment and banner in lockstep.
// (Prior audit finding: header said V4.1.0, banner printed V4.16.0.
//  Fixed here by deriving both from one constant.)
// =====================================================================
#ifndef FW_VERSION_STRING
#define FW_VERSION_STRING "V4.15.1"
#endif
#ifndef FW_FILE_STRING
#define FW_FILE_STRING "tgis510_v4_15_1.cpp"
#endif
static const char *FW_VERSION = FW_VERSION_STRING;
static const char *FW_FILE = FW_FILE_STRING;

// Watchdog: recovers from a hung control loop (e.g. a stalled NS12 link)
// instead of leaving fast-stop/interlock outputs stuck indefinitely.
static const uint32_t WATCHDOG_TIMEOUT_S = 3;

// Periodic Serial diagnostic report cadence.
static const uint32_t DIAGNOSTIC_INTERVAL_MS = 1000;

// =====================================================================
// PIN CONFIG
// PLACEHOLDER (Action Item 1): none of ENCODER_PULSE_PIN,
// PRESENCE_SENSOR_PIN, KEYENCE_TRIGGER_PIN, KEYENCE_RESULT_PIN have been
// bench-verified against the physical board silkscreen. Do not solder
// against these numbers without checking first.
// =====================================================================
namespace Pins {
constexpr uint8_t I2C_SDA = 8;
constexpr uint8_t I2C_SCL = 9;
constexpr uint8_t NS12_TX = 43;
constexpr uint8_t NS12_RX = 44;

// Waveshare ESP32-S3-Zero onboard WS2812 RGB LED -- confirmed board
// feature (not a wiring guess like the placeholders below).
constexpr uint8_t RGB_LED = 21;

// PLACEHOLDER -- bench-verify against silkscreen (Action Item 1)
constexpr uint8_t ENCODER_PULSE_PIN = 4;
constexpr uint8_t PRESENCE_SENSOR_PIN = 5;
constexpr uint8_t KEYENCE_TRIGGER_PIN = 6;

// Keyence result is intentionally a *direct* ESP32 GPIO, not an MCP23017
// input. Critical latency finding: MCP23017 is only polled during
// Standby/TubeGap (~20ms cadence); a signal that must be actionable during
// InspectingTube cannot ride on that polling. This pin + a hardware
// interrupt replaces the earlier MCP_IN_KEYENCE_RESULT-on-GPB5 design.
// PLACEHOLDER -- bench-verify against silkscreen, same as above.
constexpr uint8_t KEYENCE_RESULT_PIN = 7;
} // namespace Pins

// =====================================================================
// I2C bus (shared: MLX90640 + MCP23017)
// Confirmed working: 800kHz. 1MHz silently broke MCP23017 enumeration
// (safety-relevant -- MCP owns stop/interlock I/O) with no error other than
// "MCP initialized: NO" in diagnostics. Do NOT return to 1MHz without
// re-verifying MCP23017 survives it.
// =====================================================================
static const uint32_t I2C_CLOCK_HZ = 800000UL;

bool isI2CAddressPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// =====================================================================
// Onboard RGB status LED.
// =====================================================================
Adafruit_NeoPixel statusLed(1, Pins::RGB_LED, NEO_RGB + NEO_KHZ800);

void setStatusLed(uint8_t red, uint8_t green, uint8_t blue) {
  statusLed.setPixelColor(0, statusLed.Color(red, green, blue));
  statusLed.show();
}

// =====================================================================
// Board / I2C bring-up diagnostics -- run once at startup.
// =====================================================================
void printBoardInformation() {
  Serial.println();
  Serial.println(F("CONTROLLER INFORMATION"));
  Serial.println(F("----------------------------------------------------"));
  Serial.printf("CPU frequency      : %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash size         : %.1f kB\n", ESP.getFlashChipSize() / 1024.0f);
  Serial.printf("Free heap          : %.1f kB\n", ESP.getFreeHeap() / 1024.0f);
  Serial.printf("PSRAM detected     : %s\n", psramFound() ? "YES" : "NO");
}

void runI2CScanner() {
  Serial.println();
  Serial.println(F("I2C SCANNER"));
  Serial.println(F("----------------------------------------------------"));
  uint8_t deviceCount = 0;
  for (uint8_t address = 1; address < 127; address++) {
    if (isI2CAddressPresent(address)) {
      Serial.printf("Device found       : 0x%02X\n", address);
      deviceCount++;
    }
  }
  if (deviceCount == 0) {
    Serial.println(F("No I2C devices found."));
  } else {
    Serial.printf("Total devices      : %u\n", deviceCount);
  }
}

// =====================================================================
// MLX90640
// 32Hz is the confirmed-stable *nominal* refresh ceiling at 800kHz, and
// MLX_FRAME_PERIOD_MS below (derived from MLX_MEASURED_FPS) only paces how
// often loop() asks the sensor for a frame -- it is a fixed assumption, not
// a live measurement. The actual windowed measurement lives at runtime in
// `measuredFramesPerSecond` (see updateFrameRate()) and is what diagnostics
// / HMI telemetry report. 64Hz fails with error -8 here -- an I2C
// bandwidth wall (64Hz needs ~196KB/s vs ~100KB/s usable at 800kHz), not a
// timing bug.
// =====================================================================
static const mlx90640_refreshrate_t MLX_REFRESH_RATE_NOMINAL = MLX90640_32_HZ;
static const float MLX_MEASURED_FPS = 8.0f;
static const uint32_t MLX_FRAME_PERIOD_MS = (uint32_t)(1000.0f / MLX_MEASURED_FPS); // 125ms

Adafruit_MLX90640 mlx;
float mlxFrame[32 * 24];

bool mlxDetected = false;
bool mlxInitialized = false;
bool lastFrameValid = false;
uint32_t successfulFrameCount = 0;
uint32_t failedFrameCount = 0;
uint8_t consecutiveFrameFailures = 0;
constexpr uint8_t FRAME_FAILURE_RECOVERY_COUNT = 5;

float minimumTemperatureC = NAN;
float maximumTemperatureC = NAN;
float averageTemperatureC = NAN;

float measuredFramesPerSecond = 0.0f;
uint32_t fpsWindowStartMs = 0;
uint32_t fpsWindowFrameCount = 0;

// Sanity ceiling/floor for a single MLX90640 pixel reading. Real hardware
// (see field report: min 15.71C / avg 24.21C / max 782.23C on one frame)
// shows a single glitching pixel can report values far beyond anything
// physically plausible for this application -- unfiltered, that one pixel
// becomes "maximum temp" and can force a false capture trigger, since
// CAPTURE_TRIGGER_TEMP_C only needs one pixel to cross it. 300C is well
// above the 180C production glue ceiling (margin for a genuinely hot
// reading) and well below observed glitch values; -40C matches the
// sensor's documented operating floor. This does not fix *why* the sensor
// glitches (a MLX90640 bad-pixel table would be the real fix, out of scope
// here) -- it only stops one glitch pixel from corrupting statistics, the
// HMI display, and the QC capture trigger.
constexpr float MIN_PLAUSIBLE_TEMP_C = -40.0f;
constexpr float MAX_PLAUSIBLE_TEMP_C = 300.0f;
uint16_t lastFrameRejectedPixelCount = 0;

bool isPlausibleTemp(float t) {
  return isfinite(t) && t >= MIN_PLAUSIBLE_TEMP_C && t <= MAX_PLAUSIBLE_TEMP_C;
}

void calculateFrameStatistics() {
  float sumC = 0.0f;
  float minC = INFINITY;
  float maxC = -INFINITY;
  uint16_t validCount = 0;
  uint16_t rejectedCount = 0;

  for (size_t i = 0; i < 32 * 24; i++) {
    float t = mlxFrame[i];
    if (isfinite(t) && !isPlausibleTemp(t)) rejectedCount++;
    if (!isPlausibleTemp(t)) continue;
    if (t < minC) minC = t;
    if (t > maxC) maxC = t;
    sumC += t;
    validCount++;
  }

  lastFrameRejectedPixelCount = rejectedCount;

  if (validCount > 0) {
    minimumTemperatureC = minC;
    maximumTemperatureC = maxC;
    averageTemperatureC = sumC / (float)validCount;
  } else {
    minimumTemperatureC = NAN;
    maximumTemperatureC = NAN;
    averageTemperatureC = NAN;
  }
}

void updateFrameRate() {
  uint32_t now = millis();
  uint32_t elapsed = now - fpsWindowStartMs;
  if (elapsed >= 2000UL) {
    measuredFramesPerSecond = (float)fpsWindowFrameCount * 1000.0f / (float)elapsed;
    fpsWindowStartMs = now;
    fpsWindowFrameCount = 0;
  }
}

bool initializeMlx() {
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    return false;
  }
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX_REFRESH_RATE_NOMINAL);
  return true;
}

void attemptCameraRecovery() {
  Serial.println();
  Serial.println(F("CAMERA RECOVERY"));
  Serial.println(F("----------------------------------------------------"));
  Serial.println(F("5 consecutive frame reads failed -- reinitializing I2C + MLX90640."));

  mlxInitialized = false;
  consecutiveFrameFailures = 0;

  Wire.end();
  delay(50);
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(1000);
  delay(50);

  mlxDetected = isI2CAddressPresent(MLX90640_I2CADDR_DEFAULT);
  if (mlxDetected) {
    mlxInitialized = initializeMlx();
  }

  if (mlxInitialized) {
    Serial.println(F("Camera recovery successful."));
    setStatusLed(0, 25, 0);
  } else {
    Serial.println(F("Camera recovery failed."));
    setStatusLed(30, 0, 0);
  }
}

// PLACEHOLDER (Action Item 5): confirmed production range 20.0-180.0C.
static const float MATRIX_TEMP_MIN_C = 20.0f;
static const float MATRIX_TEMP_MAX_C = 180.0f;

// PLACEHOLDER (Action Item 6): guess, needs real glue thermal-signature
// data. Fallback idea if unreliable: frame-to-frame delta spike instead of
// an absolute threshold.
static const float CAPTURE_TRIGGER_TEMP_C = 30.0f;

// PLACEHOLDER (Action Item 7): needs real 200 m/min validation. Tuning knob
// for "does one sampling window match one tube's FOV transit".
static const uint8_t CAPTURE_SAMPLE_COUNT = 4;

// =====================================================================
// Timing constraint (critical, unresolved):
// At ~8 FPS (125ms/frame) and 32mm standoff, a tube's glue trace transits
// the MLX90640 FOV in ~6-27ms depending on lens variant -- 5-20x faster
// than one frame acquisition. Encoder synchronization is therefore
// non-optional. Still unconfirmed: lens variant mounted, tube length along
// travel axis, actual line speed vs the 200 m/min ceiling assumption.
// The capture/composite logic below is deliberately max-hold, not
// averaging, to cope with this: see CaptureController.
// =====================================================================

// =====================================================================
// Glue strip zones within the 32x24 analysis frame.
// PLACEHOLDER: exact column ranges for the two glue strips are not yet
// characterized against a real tube. Defaulting to left/right halves.
// =====================================================================
namespace StripZone {
constexpr uint8_t COLS = 32;
constexpr uint8_t ROWS = 24;
constexpr uint8_t STRIP1_COL_START = 0, STRIP1_COL_END = 15;  // PLACEHOLDER
constexpr uint8_t STRIP2_COL_START = 16, STRIP2_COL_END = 31; // PLACEHOLDER
} // namespace StripZone

// =====================================================================
// MCP23017 -- machine I/O (stop/interlock), shares the I2C bus.
// Address 0x20 (A0/A1/A2 grounded).
// Outputs GPA0-5, Inputs GPB0-4 (active LOW, INPUT_PULLUP).
// MCP is polled only during Standby/TubeGap, ~20ms cadence -- see the
// Keyence-result note above for why that matters.
// =====================================================================
Adafruit_MCP23X17 mcp;
static const uint8_t MCP_I2C_ADDR = 0x20;
static const uint32_t MCP_POLL_INTERVAL_MS = 20;

namespace McpPin {
// Outputs (GPA0-5)
constexpr uint8_t NORMAL_STOP = 0;
constexpr uint8_t FAST_STOP = 1;
constexpr uint8_t HORN = 2;
constexpr uint8_t BEACON = 3;
constexpr uint8_t READY = 4;
constexpr uint8_t WARNING = 5;
// Inputs (GPB0-4), active LOW
constexpr uint8_t ACKNOWLEDGE = 8;
constexpr uint8_t RESET = 9;
constexpr uint8_t AUTO = 10;
constexpr uint8_t MACHINE_STOPPED = 11;
constexpr uint8_t GLUE_READY = 12;
} // namespace McpPin

bool mcpOk = false;

// =====================================================================
// Encoder -- ZATOR LMZ02, single-channel pulse train, no direction.
// Uses the ESP32 PCNT peripheral. 16-bit HW counter is drained into a
// 64-bit running total every service() call, well inside its wrap period
// at any plausible pulse rate for this line.
// =====================================================================
class EncoderTracker {
public:
  void begin(uint8_t pulseGpio) {
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = pulseGpio;
    cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    cfg.channel = PCNT_CHANNEL_0;
    cfg.unit = PCNT_UNIT_0;
    cfg.pos_mode = PCNT_COUNT_INC;
    cfg.neg_mode = PCNT_COUNT_DIS;
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_KEEP;
    cfg.counter_h_lim = 30000;
    cfg.counter_l_lim = 0;
    pcnt_unit_config(&cfg);
    pcnt_set_filter_value(PCNT_UNIT_0, 100); // ns glitch filter
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    lastHwCount = 0;
    totalCounts = 0;
  }

  void service() {
    int16_t hw = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &hw);
    int32_t delta = (int32_t)hw - (int32_t)lastHwCount;
    if (delta < 0) {
      delta += 30000; // wrapped past counter_h_lim
    }
    totalCounts += delta;
    lastHwCount = hw;
  }

  int64_t total() const { return totalCounts; }

private:
  int16_t lastHwCount = 0;
  int64_t totalCounts = 0;
};

EncoderTracker encoder;

// PLACEHOLDER (Action Item 3): blocked on confirming ZATOR LMZ02 encoder
// PPR against real hardware.
static float ENCODER_COUNTS_PER_MM = 1.0f;

// PLACEHOLDER (Action Item 3): distances from the presence sensor to each
// downstream station, used for forward projection.
static float PRESENCE_TO_MLX_DISTANCE_MM = 100.0f;
static float PRESENCE_TO_KEYENCE_DISTANCE_MM = 150.0f;

// =====================================================================
// Tube presence sensor -- ground-truth leading/trailing edge anchor.
// =====================================================================
volatile bool presenceEdgePending = false;
volatile bool presenceState = false;
void IRAM_ATTR presenceIsr() {
  presenceState = digitalRead(Pins::PRESENCE_SENSOR_PIN) == HIGH;
  presenceEdgePending = true;
}

// =====================================================================
// Keyence IV2-G30 trigger.
// KEYENCE_TRIGGER_PIN drives IN1 in external trigger mode. Min ON 100us,
// min OFF 1.2ms. A blocking digitalWrite HIGH->LOW sequence risks a pulse
// too narrow for reliable detection (needs scope verification) -- this is
// therefore a non-blocking pending-low state serviced every loop, not a
// delay()-based pulse.
// =====================================================================
class KeyenceTrigger {
public:
  void begin(uint8_t pin) {
    gpio = pin;
    pinMode(gpio, OUTPUT);
    digitalWrite(gpio, LOW);
  }

  void fire() {
    digitalWrite(gpio, HIGH);
    pulseStartUs = micros();
    pending = true;
  }

  // Comfortably wider than the 100us Keyence minimum and the MCP23017
  // ISR/polling response time. The Keyence-side "Strobe Output One-Shot ON
  // Time" should also be set wider than this on the sensor itself.
  static const uint32_t PULSE_WIDTH_US = 500;

  void service() {
    if (pending && (uint32_t)(micros() - pulseStartUs) >= PULSE_WIDTH_US) {
      digitalWrite(gpio, LOW);
      pending = false;
    }
  }

private:
  uint8_t gpio = 0;
  uint32_t pulseStartUs = 0;
  bool pending = false;
};

KeyenceTrigger keyenceTrigger;

// PLACEHOLDER (Action Item 4): Keyence result pulse polarity not confirmed.
static const int KEYENCE_RESULT_ACTIVE_LEVEL = HIGH;

volatile bool keyenceResultPending = false;
volatile bool keyenceResultPass = false;
void IRAM_ATTR keyenceResultIsr() {
  keyenceResultPass = digitalRead(Pins::KEYENCE_RESULT_PIN) == KEYENCE_RESULT_ACTIVE_LEVEL;
  keyenceResultPending = true;
}

// =====================================================================
// NS12 HMI / Memory Link protocol
// Ref: Omron NS-Series Host Connection Manual (Cat. No. V085-E1-07), S3
// "Connection via Memory Link". Confirmed applicable to this NS12-TS00B-V2
// unit's Memory Link mode. Commands: WM (write $W), RM (read $W).
//
// Frame layout (all ASCII). *S='0' selects SUM (checksum) OFF + SET-write /
// variable-length read -- no checksum byte is appended, which is only
// valid because *S explicitly says SUM is off:
//   Write : ESC 'W' 'M' '0' AAAA(4-hex addr) LL(2-dec count)
//           D,D,...(comma-separated hex, zero-suppressed) CR
//   Read  : ESC 'R' 'M' '0' AAAA(4-hex addr) LL(2-dec count) CR
//   Read response: ESC 'R' 'M' [maybe '0' echoed] AAAA LL D,D,... CR
//
// CONFIRMED ON BENCH -- do not "fix" either of these back without
// re-testing on real hardware:
//   - ESC=0x1B is used for *every* command on this PT unit, including WM.
//     The manual documents 0x1C specifically for WM/WD (word writes); this
//     unit's firmware does not honor that distinction and silently ignores
//     every WM write sent with 0x1C. Switching WM to 0x1B fixed it
//     immediately, confirmed via live sensor data and a known test pattern
//     both landing correctly on the physical screen.
//   - 9600 baud. 38400 measured ~15% RM read timeouts on this exact bench
//     setup; 9600 measured zero. A prior synthesis of this file (V4.14.0)
//     asserted 38400 as "confirmed" without ever having bench access to
//     verify it against the real unit -- that claim is not trusted here.
//
// The exact RM response framing (whether *S is echoed back, shifting the
// address field by one byte) has not been independently pinned down either
// -- parseRmResponse() below tries both candidate offsets and locks onto
// whichever one validates against the address/count actually requested.
//
// FIELD REPORT (2026-09, round 1): on real hardware this offset-guessing
// approach has NEVER actually worked -- 2968/2968 RM read attempts
// returned bytes that parsed at neither offset (0 successes, 0 timeouts,
// all parse errors), while WM writes appear to be going out fine.
//
// FIELD REPORT (round 2, with the raw-dump-on-failure added below): the
// captured raw bytes are consistently, reproducibly `ESC 'W' 'M' '0' '0'
// '1' 'F'` (7 bytes) every single time. This is NOT shaped like an RM
// response at all -- it starts with 'W','M' (our own WM-write command
// signature), not 'R','M', so parseRmResponse() rejects it on its very
// first check, before either candidate offset is even tried. The offset
// guess was never the actual bug. What's arriving on RX while we wait for
// an RM reply looks like a fragment of our own outgoing traffic, most
// plausibly explained by either (a) a hardware TX/RX loopback/echo on the
// NS12 link, or (b) a WM write firing on the shared UART while an RM read
// is still pending and getting vacuumed into the read buffer by
// pollPendingRead(), which doesn't verify a byte's origin, only that it
// starts at an ESC. requestRM() now refuses to fire while readPending is
// already true (unchanged), and sendWM() now defers a telemetry write
// rather than firing while a read is pending, to remove (b) as a variable.
// If the bogus "response" still appears after that, (a) -- a genuine
// wiring/echo issue -- is the remaining explanation and needs a bench
// check (verify NS12_RX is only ever driven by HIN232CP R1OUT, never
// bridged to NS12_TX, and check whether the PT itself echoes received
// characters despite CX-Designer's "Response = OFF" setting).
// =====================================================================

// Set to 1 for a full TX/RX byte trace on the Serial monitor (verbose --
// every WM/RM byte). Temporarily ON by default while the mystery above is
// unresolved -- turn back to 0 once the loopback/interleaving question is
// settled, since it adds real per-byte overhead and console noise. The RM
// parse-failure dump below is unconditional and separate from this, since
// that specific failure needs visibility regardless of this flag.
#define NS12_DEBUG_RAW_RX 1

namespace NS12 {
constexpr uint8_t ESC = 0x1B;
constexpr long BAUD = 9600; // CONFIRMED bench value -- see header note above
constexpr uint32_t RM_READ_TIMEOUT_MS = 250;

// Word Lamp matrix -- default/trusted mode, 16x8 grid at $W700-$W827,
// column-major, stride 8: address(col,row) = 700 + col*8 + row.
constexpr uint16_t MATRIX_BASE_ADDR = 700;
constexpr uint8_t MATRIX_COLS_DEFAULT = 16;
constexpr uint8_t MATRIX_ROWS_DEFAULT = 8;

// Full 32x24 push previously caused 100% NS12 read-request failure
// (0/424 reads OK) -- the PT couldn't service RM reads while absorbing
// that write load. Reverted to 16x8. A later column-paced 32x24 mode is
// kept here as opt-in/experimental, self-monitored: it auto-reverts to
// 16x8 if RM success rate collapses (see checkDisplayAutoFallback()).
constexpr bool ENABLE_EXPERIMENTAL_32x24 = false;
constexpr uint8_t MATRIX_COLS_EXPERIMENTAL = 32;
constexpr uint8_t MATRIX_ROWS_EXPERIMENTAL = 24;

// Word Lamp palette: 10 entries, index 0-9. Index 0 renders as blank/off on
// this PT -- clamp the coldest output to index 1, never 0, so a write
// always shows something.
constexpr uint8_t PALETTE_MIN_INDEX = 1;
constexpr uint8_t PALETTE_MAX_INDEX = 9;

// Display refresh decoupled from live camera streaming, velocity-adaptive
// per-tube throttle (floor only -- see requestDisplayPush()).
constexpr uint32_t TARGET_DISPLAY_REFRESH_MS = 2000;

// Telemetry block $W0-$W8 -- operator/PLC visibility into system health.
// V4.14.0 dropped this entirely; restored from the bench-tested file.
constexpr uint16_t TELEMETRY_BASE_ADDR = 0;
constexpr uint16_t TELEMETRY_WORD_COUNT = 9;
constexpr uint32_t TELEMETRY_WRITE_INTERVAL_MS = 250;

// Low-rate read used only to keep the auto-fallback's RM success-rate
// stats alive (see checkDisplayAutoFallback()) -- without some RM traffic
// those stats never move and the fallback can never trigger. Mirrors the
// bench-tested file's $W10 "operator test input" convention; repoint if
// the CX-Designer project already uses $W10 for something else.
constexpr uint16_t TEST_READ_ADDR = 10;
constexpr uint32_t TEST_READ_INTERVAL_MS = 300;

// Largest single WM burst used anywhere (the 16x8 default matrix: 128
// words). Also sizes the sendWM() stack buffer.
constexpr uint16_t MAX_WM_WORDS = 128;

// Spacing between successive column writes in the experimental 32x24 mode.
//
// Derived, not guessed: one column WM frame is ESC+'W'+'M'+'0' (4) +
// 4-hex address (4) + 2-decimal count (2) + comma-separated hex data
// (rows values, each 1 digit since palette indices are 0-9, plus
// rows-1 commas) + CR (1). At MATRIX_ROWS_EXPERIMENTAL=24 that's 58 bytes;
// at 8N1 (10 bits/byte) and BAUD=9600 that takes ~60.4ms to physically
// drain off the wire. A fixed interval shorter than that would issue a new
// column write before the previous one finished transmitting -- the same
// "PT starved mid-write" failure mode column-pacing exists to avoid, just
// recurring at smaller scale. Computed with a 50% margin so it stays
// correct if BAUD or MATRIX_ROWS_EXPERIMENTAL ever change.
constexpr uint16_t COLUMN_DATA_CHARS =
    (uint16_t)MATRIX_ROWS_EXPERIMENTAL + ((uint16_t)MATRIX_ROWS_EXPERIMENTAL - 1);
constexpr uint16_t COLUMN_FRAME_BYTES =
    4 /*ESC W M '0'*/ + 4 /*hex addr*/ + 2 /*dec count*/ + COLUMN_DATA_CHARS + 1 /*CR*/;
constexpr uint32_t COLUMN_FRAME_TX_TIME_US =
    (uint32_t)COLUMN_FRAME_BYTES * 10UL * 1000000UL / (uint32_t)BAUD;
constexpr uint32_t COLUMN_WRITE_INTERVAL_MS =
    (COLUMN_FRAME_TX_TIME_US * 3UL / 2UL) / 1000UL + 1UL; // +50% margin, ceil to ms
} // namespace NS12

class NS12Manager {
public:
  void begin() {
    Serial2.setTxBufferSize(1024);
    Serial2.begin(NS12::BAUD, SERIAL_8N1, Pins::NS12_RX, Pins::NS12_TX);
    lastTelemetryMs = millis();
    lastTestReadMs = millis();
  }

  // WM: write `count` words starting at `startAddr`. `count` is clamped to
  // NS12::MAX_WM_WORDS -- callers must stay within that (the 16x8 burst,
  // telemetry block, and single-column experimental writes all do).
  void sendWM(uint16_t startAddr, const uint16_t *data, uint16_t count) {
    if (count > NS12::MAX_WM_WORDS) count = NS12::MAX_WM_WORDS;
    char frame[4 + 4 + 2 + NS12::MAX_WM_WORDS * 5 + 1];
    size_t n = 0;
    frame[n++] = (char)NS12::ESC;
    frame[n++] = 'W';
    frame[n++] = 'M';
    frame[n++] = '0';
    n += writeHex4(&frame[n], startAddr);
    n += writeDecimal2(&frame[n], (uint8_t)count);
    for (uint16_t i = 0; i < count; i++) {
      if (i > 0) frame[n++] = ',';
      n += writeHexCompact(&frame[n], data[i]);
    }
    frame[n++] = '\r';

#if NS12_DEBUG_RAW_RX
    Serial.print(F("[NS12] TX WM: "));
    for (size_t i = 0; i < n; i++) printRawByte((uint8_t)frame[i]);
    Serial.println();
#endif

    wmAttempts++;
    size_t sent = Serial2.write(reinterpret_cast<uint8_t *>(frame), n);
    if (sent != n) {
      // Partial write -- the PT never got a complete, valid frame. Left
      // uncounted before, this made a write silently dropped under TX
      // backlog indistinguishable from one that landed cleanly.
      wmFailures++;
    }
    // Blocking flush() intentionally not used for WM -- a stalled TX flush
    // here would block the whole control loop during InspectingTube.
  }

  // RM: request `count` words (max 32) starting at `startAddr`. Sends the
  // request only and returns immediately -- non-blocking, unlike the prior
  // synthesis's spin-wait version, which could stall the whole loop for up
  // to RM_READ_TIMEOUT_MS at a time this file now also drives hard-real-
  // time Keyence pulse timing and encoder tracking. Call service() every
  // loop() iteration to drive the response state machine.
  bool requestRM(uint16_t startAddr, uint8_t count) {
    if (readPending || count == 0 || count > 32) return false;

    char frame[16];
    size_t n = 0;
    frame[n++] = (char)NS12::ESC;
    frame[n++] = 'R';
    frame[n++] = 'M';
    frame[n++] = '0';
    n += writeHex4(&frame[n], startAddr);
    n += writeDecimal2(&frame[n], count);
    frame[n++] = '\r';

#if NS12_DEBUG_RAW_RX
    Serial.print(F("[NS12] TX RM: "));
    for (size_t i = 0; i < n; i++) printRawByte((uint8_t)frame[i]);
    Serial.println();
#endif

    rmAttempts++;
    clearRxBuffer();
    size_t sent = Serial2.write(reinterpret_cast<uint8_t *>(frame), n);
    if (sent != n) {
      // Request itself never fully went out -- don't burn the full
      // RM_READ_TIMEOUT_MS waiting on a reply to a frame the PT never saw.
      rmWriteFailures++;
      return false;
    }
    Serial2.flush(); // request frame is <=11 bytes -- flush cost here is negligible

    readPending = true;
    readSentMs = millis();
    readLineUsed = 0;
    expectedAddr = startAddr;
    expectedCount = count;
    return true;
  }

  // Must be called every loop() iteration. Drives the periodic telemetry
  // push, the periodic low-rate test read, and the non-blocking read
  // state machine. Never blocks.
  void service() {
    uint32_t now = millis();

    // Deliberately gated on !readPending: RM_READ_TIMEOUT_MS and
    // TELEMETRY_WRITE_INTERVAL_MS are both 250-300ms, so a telemetry WM
    // write could otherwise fire in the middle of an in-flight RM read on
    // this shared half-visible UART. pollPendingRead() only checks that a
    // byte stream starts at an ESC, not where it actually came from -- see
    // the field-report comment on the NS12 namespace for why that matters
    // (every captured "RM response" so far has actually had a WM-shaped
    // header). This delays telemetry by at most one RM_READ_TIMEOUT_MS
    // window, not lost -- it fires on the next service() call instead.
    if (!readPending && now - lastTelemetryMs >= NS12::TELEMETRY_WRITE_INTERVAL_MS) {
      lastTelemetryMs = now;
      sendWM(NS12::TELEMETRY_BASE_ADDR, telemetry, NS12::TELEMETRY_WORD_COUNT);
    }

    if (!readPending && (now - lastTestReadMs >= NS12::TEST_READ_INTERVAL_MS)) {
      lastTestReadMs = now;
      requestRM(NS12::TEST_READ_ADDR, 1);
    }

    if (readPending) {
      pollPendingRead(now);
    }
  }

  void setTelemetry(uint16_t heartbeat, uint16_t fpsX10, uint16_t minX10, uint16_t maxX10,
                     uint16_t avgX10, uint16_t goodFrames, uint16_t badFrames,
                     uint16_t stateValue, uint16_t statusWord) {
    telemetry[0] = heartbeat;
    telemetry[1] = fpsX10;
    telemetry[2] = minX10;
    telemetry[3] = maxX10;
    telemetry[4] = avgX10;
    telemetry[5] = goodFrames;
    telemetry[6] = badFrames;
    telemetry[7] = stateValue;
    telemetry[8] = statusWord;
  }

  uint32_t rmAttemptCount() const { return rmAttempts; }
  uint32_t rmSuccessCount() const { return rmSuccesses; }
  uint32_t rmWriteFailureCount() const { return rmWriteFailures; }
  uint32_t rmTimeoutCount() const { return rmTimeouts; }
  uint32_t rmParseErrorCount() const { return rmParseErrors; }
  uint32_t wmAttemptCount() const { return wmAttempts; }
  uint32_t wmFailureCount() const { return wmFailures; }
  void resetRmStats() { rmAttempts = 0; rmSuccesses = 0; }

  // Exposed so the matrix-push free functions (serviceDisplayThrottle(),
  // serviceMatrixPacing()) can defer a WM write the same way service()
  // defers telemetry -- see the field-report comment on the NS12
  // namespace for why a WM write during a pending RM read is suspect.
  bool isReadPending() const { return readPending; }

private:
  uint16_t telemetry[NS12::TELEMETRY_WORD_COUNT] = {};
  uint32_t lastTelemetryMs = 0;
  uint32_t lastTestReadMs = 0;

  bool readPending = false;
  uint32_t readSentMs = 0;
  uint16_t expectedAddr = 0;
  uint8_t expectedCount = 0;
  char readLineBuffer[40] = {};
  size_t readLineUsed = 0;

  uint32_t rmAttempts = 0;
  uint32_t rmSuccesses = 0;
  uint32_t rmWriteFailures = 0;
  uint32_t rmTimeouts = 0;
  uint32_t rmParseErrors = 0;
  uint32_t wmAttempts = 0;
  uint32_t wmFailures = 0;

  void clearRxBuffer() {
    while (Serial2.available() > 0) Serial2.read();
  }

  static void printRawByte(uint8_t value) {
#if NS12_DEBUG_RAW_RX
    if (value == NS12::ESC) { Serial.print(F("[ESC]")); return; }
    if (value == '\r') { Serial.print(F("[CR]")); return; }
    if (value >= 0x20 && value < 0x7F) { Serial.print((char)value); return; }
    Serial.print('[');
    if (value < 0x10) Serial.print('0');
    Serial.print(value, HEX);
    Serial.print(']');
#else
    (void)value;
#endif
  }

  // Unconditional (not gated by NS12_DEBUG_RAW_RX) -- a parse failure is
  // exactly the case that needs visibility by default. Cheap: only fires
  // on the low-rate periodic test read, at most once per TEST_READ_INTERVAL_MS.
  static void dumpRejectedLine(const char *line, size_t len) {
    Serial.print(F("[NS12] RM parse failed, raw response ("));
    Serial.print(len);
    Serial.print(F(" bytes): "));
    for (size_t i = 0; i < len; i++) {
      printRawByteAlways((uint8_t)line[i]);
    }
    Serial.println();
    // Flag explicitly rather than making the reader notice: a genuine RM
    // reply starts 'R','M'. If it starts 'W','M' instead, this isn't a PT
    // response at all -- it's shaped like one of OUR OWN WM writes, most
    // likely a loopback/echo or a write that fired while this read was
    // still pending. See the field-report comment on the NS12 namespace.
    if (len >= 3 && line[1] == 'W' && line[2] == 'M') {
      Serial.println(F("[NS12]   ^ starts 'W','M', not 'R','M' -- looks like "
                        "our own WM traffic, not a genuine RM reply. Check "
                        "for TX/RX loopback or PT echo."));
    }
  }

  static void printRawByteAlways(uint8_t value) {
    if (value == NS12::ESC) { Serial.print(F("[ESC]")); return; }
    if (value >= 0x20 && value < 0x7F) { Serial.print((char)value); return; }
    Serial.print('[');
    if (value < 0x10) Serial.print('0');
    Serial.print(value, HEX);
    Serial.print(']');
  }

  // Non-blocking poll: consumes whatever bytes are currently available
  // without waiting. Completes the pending read only once a full line
  // (terminated by CR) has arrived, or aborts it on timeout/overflow.
  void pollPendingRead(uint32_t now) {
    while (Serial2.available() > 0 && readLineUsed < sizeof(readLineBuffer) - 1) {
      char ch = (char)Serial2.read();

      // A genuine RM response always starts with ESC. Anything arriving
      // before that first ESC is stray (e.g. overlap with a WM write) and
      // is discarded rather than corrupting the line.
      if (readLineUsed == 0 && (uint8_t)ch != NS12::ESC) {
        continue;
      }

      if (ch == '\r') {
        readLineBuffer[readLineUsed] = '\0';
        if (parseRmResponse(readLineBuffer, readLineUsed)) {
          rmSuccesses++;
        } else {
          rmParseErrors++;
          dumpRejectedLine(readLineBuffer, readLineUsed);
        }
        readPending = false;
        return;
      }

      readLineBuffer[readLineUsed++] = ch;
    }

    if (!readPending) return; // completed above

    if (readLineUsed >= sizeof(readLineBuffer) - 1) {
      rmParseErrors++;
      readPending = false;
      return;
    }

    if (now - readSentMs > NS12::RM_READ_TIMEOUT_MS) {
      rmTimeouts++;
      readPending = false;
    }
  }

  // Response framing: ESC 'R' 'M' [maybe '0' echoed] AAAA(4-hex)
  // LL(2-dec) D,D,... CR. The '0' echo has not been independently
  // confirmed on this PT, so both candidate offsets are tried; whichever
  // validates against the address/count actually requested wins.
  bool parseRmResponse(const char *response, size_t len) {
    if (len < 9 || (uint8_t)response[0] != NS12::ESC || response[1] != 'R' || response[2] != 'M') {
      return false;
    }

    static const uint8_t candidateOffsets[] = {3, 4};
    for (uint8_t offset : candidateOffsets) {
      size_t headerLen = (size_t)offset + 6;
      if (len < headerLen) continue;

      char addrText[5] = {response[offset], response[offset + 1], response[offset + 2],
                          response[offset + 3], '\0'};
      char countText[3] = {response[offset + 4], response[offset + 5], '\0'};
      uint16_t addr = (uint16_t)strtoul(addrText, nullptr, 16);
      uint8_t count = (uint8_t)strtoul(countText, nullptr, 10);
      if (addr != expectedAddr || count != expectedCount) continue;

      char dataText[8];
      size_t dataLen = len - headerLen;
      if (dataLen == 0 || dataLen >= sizeof(dataText)) continue;
      memcpy(dataText, &response[headerLen], dataLen);
      dataText[dataLen] = '\0';
      char *comma = strchr(dataText, ',');
      if (comma) *comma = '\0';

      char *endPtr = nullptr;
      strtoul(dataText, &endPtr, 16);
      if (endPtr == dataText) continue;
      return true; // test read only exercises the link right now
    }
    return false;
  }

  static size_t writeHex4(char *dst, uint16_t v) {
    char tmp[5];
    snprintf(tmp, sizeof(tmp), "%04X", v);
    memcpy(dst, tmp, 4);
    return 4;
  }
  static size_t writeDecimal2(char *dst, uint8_t v) {
    char tmp[3];
    snprintf(tmp, sizeof(tmp), "%02u", v);
    memcpy(dst, tmp, 2);
    return 2;
  }
  // Zero-suppressed hex (e.g. 0 -> "0", 10 -> "A") -- matches the comma-
  // separated, variable-width data encoding confirmed on the bench.
  static size_t writeHexCompact(char *dst, uint16_t v) {
    char tmp[5];
    int len = snprintf(tmp, sizeof(tmp), "%X", v);
    memcpy(dst, tmp, (size_t)len);
    return (size_t)len;
  }
};

NS12Manager ns12;

// =====================================================================
// Word Lamp temperature-to-palette mapping and matrix downsample.
// =====================================================================
uint8_t tempToPaletteIndex(float tempC) {
  float t = (tempC - MATRIX_TEMP_MIN_C) / (MATRIX_TEMP_MAX_C - MATRIX_TEMP_MIN_C);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  uint8_t idx = NS12::PALETTE_MIN_INDEX +
                (uint8_t)(t * (NS12::PALETTE_MAX_INDEX - NS12::PALETTE_MIN_INDEX));
  if (idx < NS12::PALETTE_MIN_INDEX) idx = NS12::PALETTE_MIN_INDEX;
  if (idx > NS12::PALETTE_MAX_INDEX) idx = NS12::PALETTE_MAX_INDEX;
  return idx;
}

// Downsamples the 32x24 analysis frame into a displayCols x displayRows
// grid using max-per-block (matches the max-hold philosophy: don't average
// away a hot pixel for HMI visibility).
void downsampleMaxBlock(const float *src, uint8_t displayCols, uint8_t displayRows,
                         float *dst) {
  uint8_t blockW = StripZone::COLS / displayCols;
  uint8_t blockH = StripZone::ROWS / displayRows;
  for (uint8_t dc = 0; dc < displayCols; dc++) {
    for (uint8_t dr = 0; dr < displayRows; dr++) {
      float m = -1000.0f;
      for (uint8_t x = 0; x < blockW; x++) {
        for (uint8_t y = 0; y < blockH; y++) {
          uint8_t sc = dc * blockW + x;
          uint8_t sr = dr * blockH + y;
          float v = src[sr * StripZone::COLS + sc];
          // Skip implausible pixels so one glitching pixel can't paint a
          // false hot spot on the HMI (see isPlausibleTemp()).
          if (isPlausibleTemp(v) && v > m) m = v;
        }
      }
      dst[dc * displayRows + dr] = m;
    }
  }
}

// Display refresh decoupled from live camera streaming, velocity-adaptive
// per-tube throttle. TARGET_DISPLAY_REFRESH_MS (2000ms) is a floor, not a
// fixed cadence: pushes never happen closer together than this (protects
// the PT the same way the column-pacing above does), but when tubes are
// passing slower than that the display still updates once per tube rather
// than sitting idle for the rest of the 2000ms window.
static const uint32_t MIN_DISPLAY_REFRESH_MS = NS12::TARGET_DISPLAY_REFRESH_MS;
uint32_t lastDisplayPushMs = 0;
uint32_t lastTubeLatchMs = 0;
uint32_t currentDisplayRefreshMs = NS12::TARGET_DISPLAY_REFRESH_MS;
bool displayPushQueued = false;
float pendingDisplayFrame[StripZone::COLS * StripZone::ROWS];

void requestDisplayPush(const float *frame) {
  uint32_t now = millis();
  uint32_t interTubeGapMs = now - lastTubeLatchMs;
  lastTubeLatchMs = now;
  currentDisplayRefreshMs = interTubeGapMs > MIN_DISPLAY_REFRESH_MS ? interTubeGapMs
                                                                     : MIN_DISPLAY_REFRESH_MS;
  memcpy(pendingDisplayFrame, frame, sizeof(pendingDisplayFrame));
  displayPushQueued = true;
}

// State for the experimental 32x24 column-paced push: one column (of `rows`
// words) is sent per COLUMN_WRITE_INTERVAL_MS tick from serviceMatrixPacing(),
// instead of one 768-word burst -- the actual mitigation for the PT being
// starved of time to service RM reads by one oversized write.
struct PendingMatrixWrite {
  bool active = false;
  uint8_t cols = 0, rows = 0;
  uint16_t words[NS12::MATRIX_COLS_EXPERIMENTAL * NS12::MATRIX_ROWS_EXPERIMENTAL];
  uint16_t bandAddr = 0;
  uint16_t band01[2] = {0, 0};
  uint8_t nextCol = 0;
  uint32_t lastWriteMs = 0;
};
PendingMatrixWrite pendingMatrix;

// Runtime-effective mode: starts at the compile-time default but can be
// latched false by checkDisplayAutoFallback() below if the experimental
// 32x24 mode is starving RM reads.
bool experimental32x24Effective = NS12::ENABLE_EXPERIMENTAL_32x24;

void pushWordLampMatrix(const float *compositeFrame) {
  uint8_t cols = experimental32x24Effective ? NS12::MATRIX_COLS_EXPERIMENTAL
                                             : NS12::MATRIX_COLS_DEFAULT;
  uint8_t rows = experimental32x24Effective ? NS12::MATRIX_ROWS_EXPERIMENTAL
                                             : NS12::MATRIX_ROWS_DEFAULT;

  static float displayBuf[32 * 24];
  if (cols == StripZone::COLS && rows == StripZone::ROWS) {
    memcpy(displayBuf, compositeFrame, sizeof(float) * cols * rows);
  } else {
    downsampleMaxBlock(compositeFrame, cols, rows, displayBuf);
  }

  uint16_t words[32 * 24];
  // Two halves (rows 0..rows/2-1, rows/2..rows-1), matching the documented
  // $W828 (rows 1-4) / $W829 (rows 5-8) band-maximum layout at 16x8.
  float bandMax[2] = {-1000, -1000};
  for (uint8_t c = 0; c < cols; c++) {
    for (uint8_t r = 0; r < rows; r++) {
      float v = displayBuf[c * rows + r];
      words[c * rows + r] = tempToPaletteIndex(v);
      uint8_t half = (r < rows / 2) ? 0 : 1;
      if (v > bandMax[half]) bandMax[half] = v;
    }
  }
  uint16_t band01[2] = {(uint16_t)(bandMax[0] * 10), (uint16_t)(bandMax[1] * 10)};

  if (cols == NS12::MATRIX_COLS_DEFAULT && rows == NS12::MATRIX_ROWS_DEFAULT) {
    // Trusted 16x8 layout: one 128-word burst, exactly as documented.
    ns12.sendWM(NS12::MATRIX_BASE_ADDR, words, cols * rows);
    ns12.sendWM(828, band01, 2);
  } else {
    // Experimental 32x24: hand off to the column-paced dispatcher. Band-max
    // words go immediately after the matrix block so they never collide
    // with it, unlike the fixed 828/829 addresses the 32x24 block would
    // otherwise overrun.
    memcpy(pendingMatrix.words, words, sizeof(uint16_t) * cols * rows);
    pendingMatrix.cols = cols;
    pendingMatrix.rows = rows;
    pendingMatrix.bandAddr = NS12::MATRIX_BASE_ADDR + cols * rows;
    pendingMatrix.band01[0] = band01[0];
    pendingMatrix.band01[1] = band01[1];
    pendingMatrix.nextCol = 0;
    pendingMatrix.lastWriteMs = 0; // fire the first column on the next service() tick
    pendingMatrix.active = true;
  }
}

// Called every loop() iteration; flushes a queued composite once the
// current velocity-adaptive throttle interval (requestDisplayPush) has
// elapsed.
void serviceDisplayThrottle() {
  if (!displayPushQueued) return;
  if (ns12.isReadPending()) return; // defer -- see field-report comment on NS12 namespace
  uint32_t now = millis();
  if (now - lastDisplayPushMs < currentDisplayRefreshMs) return;
  lastDisplayPushMs = now;
  displayPushQueued = false;
  pushWordLampMatrix(pendingDisplayFrame);
}

// Called every loop() iteration; no-op unless a paced 32x24 push is active.
void serviceMatrixPacing() {
  if (!pendingMatrix.active) return;
  if (ns12.isReadPending()) return; // defer -- see field-report comment on NS12 namespace
  uint32_t now = millis();
  if (now - pendingMatrix.lastWriteMs < NS12::COLUMN_WRITE_INTERVAL_MS) return;
  pendingMatrix.lastWriteMs = now;

  uint8_t c = pendingMatrix.nextCol;
  uint16_t addr = NS12::MATRIX_BASE_ADDR + (uint16_t)c * pendingMatrix.rows;
  ns12.sendWM(addr, &pendingMatrix.words[(size_t)c * pendingMatrix.rows], pendingMatrix.rows);
  pendingMatrix.nextCol++;

  if (pendingMatrix.nextCol >= pendingMatrix.cols) {
    ns12.sendWM(pendingMatrix.bandAddr, pendingMatrix.band01, 2);
    pendingMatrix.active = false;
  }
}

// Self-monitor for the experimental 32x24 column-paced mode: if RM read
// success rate collapses under real traffic (as it did at 0/424 with the
// old full-frame 32x24 push), fall back to the trusted 16x8 mode rather
// than keep pushing into a PT that can't service reads. Fed by the
// periodic low-rate test read in NS12Manager::service() -- previously
// nothing ever called into the RM path, so this safety net could never
// have actually fired.
void checkDisplayAutoFallback() {
  if (!experimental32x24Effective) return;
  if (ns12.rmAttemptCount() >= 50) {
    float successRate = (float)ns12.rmSuccessCount() / (float)ns12.rmAttemptCount();
    if (successRate < 0.5f) {
      experimental32x24Effective = false;
      pendingMatrix.active = false; // abandon any in-flight paced push
      Serial.println(F("[NS12] WARNING: RM success rate collapsed under 32x24 "
                        "traffic, falling back to 16x8 display mode."));
    }
    ns12.resetRmStats();
  }
}

// =====================================================================
// Capture / QC composite -- max-hold per tube pass.
// ARMED watches maximumTemperatureC against CAPTURE_TRIGGER_TEMP_C ->
// SAMPLING accumulates CAPTURE_SAMPLE_COUNT frames -> LATCHED freezes the
// HMI image until rearmed.
//
// Aggregation is deliberately max-hold, not averaging -- the tube moves
// under a fixed FOV, so different frames see different physical sections;
// averaging would dilute/hide a glue trace that exited frame mid-window.
// Max-hold composites the hottest value seen at each cell across the
// whole transit.
// =====================================================================
enum class CaptureState { ARMED, SAMPLING, LATCHED };

struct GlueStripResult {
  bool present = false;
  float maxTempC = 0;
  uint16_t hotPixelCount = 0;
};

class CaptureController {
public:
  void rearm() {
    state = CaptureState::ARMED;
    sampleCount = 0;
  }

  void onNewFrame(const float *frame) {
    switch (state) {
    case CaptureState::ARMED: {
      float maxT = frameMax(frame);
      if (maxT >= CAPTURE_TRIGGER_TEMP_C) {
        // Seed with -INFINITY for implausible pixels rather than copying
        // them verbatim -- otherwise a single glitching pixel elsewhere in
        // the trigger frame (not even the one that crossed the threshold)
        // would ride along into the QC composite and strip evaluation.
        for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
          compositeFrame[i] = isPlausibleTemp(frame[i]) ? frame[i] : -INFINITY;
        }
        sampleCount = 1;
        state = CaptureState::SAMPLING;
      }
      break;
    }
    case CaptureState::SAMPLING: {
      for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
        if (isPlausibleTemp(frame[i]) && frame[i] > compositeFrame[i]) {
          compositeFrame[i] = frame[i];
        }
      }
      sampleCount++;
      if (sampleCount >= CAPTURE_SAMPLE_COUNT) {
        evaluateStrips();
        state = CaptureState::LATCHED;
        requestDisplayPush(compositeFrame);
        Serial.printf("[QC] strip1 present=%d maxT=%.1fC hotPx=%u | "
                      "strip2 present=%d maxT=%.1fC hotPx=%u\n",
                      strip1Result.present, strip1Result.maxTempC, strip1Result.hotPixelCount,
                      strip2Result.present, strip2Result.maxTempC, strip2Result.hotPixelCount);
      }
      break;
    }
    case CaptureState::LATCHED:
      // Frozen until rearm().
      break;
    }
  }

  CaptureState currentState() const { return state; }
  const float *latchedFrame() const { return compositeFrame; }
  const GlueStripResult &strip1() const { return strip1Result; }
  const GlueStripResult &strip2() const { return strip2Result; }

private:
  CaptureState state = CaptureState::ARMED;
  uint8_t sampleCount = 0;
  float compositeFrame[StripZone::COLS * StripZone::ROWS] = {0};
  GlueStripResult strip1Result, strip2Result;

  // -INFINITY if no pixel in the frame is plausible -- always < any real
  // CAPTURE_TRIGGER_TEMP_C, so a fully-glitched frame simply never triggers
  // rather than triggering on frame[0] regardless of its validity (the
  // previous version didn't check frame[0] at all).
  static float frameMax(const float *frame) {
    float m = -INFINITY;
    for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
      if (isPlausibleTemp(frame[i]) && frame[i] > m) m = frame[i];
    }
    return m;
  }

  void evaluateStrips() {
    strip1Result = evalStrip(StripZone::STRIP1_COL_START, StripZone::STRIP1_COL_END);
    strip2Result = evalStrip(StripZone::STRIP2_COL_START, StripZone::STRIP2_COL_END);
  }

  GlueStripResult evalStrip(uint8_t colStart, uint8_t colEnd) {
    GlueStripResult r;
    for (uint8_t c = colStart; c <= colEnd; c++) {
      for (uint8_t row = 0; row < StripZone::ROWS; row++) {
        float v = compositeFrame[row * StripZone::COLS + c];
        if (!isPlausibleTemp(v)) continue; // unfilled cell (-INFINITY seed) or stray glitch
        if (v > r.maxTempC) r.maxTempC = v;
        if (v >= CAPTURE_TRIGGER_TEMP_C) r.hotPixelCount++;
      }
    }
    r.present = r.hotPixelCount > 0;
    return r;
  }
};

CaptureController capture;

// =====================================================================
// System state machine
// Startup -> Standby -> WaitingForTube -> InspectingTube / TubeGap ->
// FaultStop
// =====================================================================
enum class SystemState { Startup, Standby, WaitingForTube, InspectingTube, TubeGap, FaultStop };
SystemState state = SystemState::Startup;
SystemState lastLoggedState = SystemState::Startup;

// Per-tube one-shot trigger flags, reset on each new leading edge.
bool tubeMlxArmed = false;
bool tubeKeyenceFired = false;
int64_t tubeStartEncoderCount = 0;
uint32_t lastMcpPollMs = 0;
uint32_t lastMlxFrameMs = 0;
uint32_t lastDiagnosticMs = 0;
uint32_t heartbeatCounter = 0;

const char *stateName(SystemState s) {
  switch (s) {
  case SystemState::Startup: return "Startup";
  case SystemState::Standby: return "Standby";
  case SystemState::WaitingForTube: return "WaitingForTube";
  case SystemState::InspectingTube: return "InspectingTube";
  case SystemState::TubeGap: return "TubeGap";
  case SystemState::FaultStop: return "FaultStop";
  }
  return "?";
}

void setMcpOutputs(bool normalStop, bool fastStop, bool horn, bool beacon, bool ready,
                    bool warning) {
  if (!mcpOk) return;
  mcp.digitalWrite(McpPin::NORMAL_STOP, normalStop);
  mcp.digitalWrite(McpPin::FAST_STOP, fastStop);
  mcp.digitalWrite(McpPin::HORN, horn);
  mcp.digitalWrite(McpPin::BEACON, beacon);
  mcp.digitalWrite(McpPin::READY, ready);
  mcp.digitalWrite(McpPin::WARNING, warning);
}

void enterFaultStop() {
  state = SystemState::FaultStop;
  setMcpOutputs(true, true, true, true, false, true);
}

// =====================================================================
// Position projection: fires the MLX capture arm and the Keyence trigger
// at the configured lead distances ahead of the tube's leading edge, using
// the encoder as the distance reference and the presence sensor as the
// anchor.
// =====================================================================
void serviceTubePositionTracking() {
  if (state != SystemState::InspectingTube) return;

  float travelledMm = (float)(encoder.total() - tubeStartEncoderCount) / ENCODER_COUNTS_PER_MM;

  if (!tubeMlxArmed && travelledMm >= PRESENCE_TO_MLX_DISTANCE_MM) {
    capture.rearm();
    tubeMlxArmed = true;
  }
  if (!tubeKeyenceFired && travelledMm >= PRESENCE_TO_KEYENCE_DISTANCE_MM) {
    keyenceTrigger.fire();
    tubeKeyenceFired = true;
  }
}

void handlePresenceEdge() {
  bool rising = presenceState;
  presenceEdgePending = false;

  if (rising) {
    // Leading edge -- new tube entering the zone.
    if (state == SystemState::WaitingForTube || state == SystemState::TubeGap) {
      state = SystemState::InspectingTube;
      tubeStartEncoderCount = encoder.total();
      tubeMlxArmed = false;
      tubeKeyenceFired = false;
    }
  } else {
    // Trailing edge -- tube has cleared the zone.
    if (state == SystemState::InspectingTube) {
      state = SystemState::TubeGap;
    }
  }
}

void handleKeyenceResult() {
  bool pass = keyenceResultPass;
  keyenceResultPending = false;
  if (!pass) {
    Serial.println(F("[QC] Keyence result: FAIL"));
  }
}

// =====================================================================
// Telemetry helpers -- feed the NS12 $W0-$W8 block every loop (the manager
// only actually transmits it every NS12::TELEMETRY_WRITE_INTERVAL_MS).
// =====================================================================
uint16_t toUnsignedX10(float value) {
  if (!isfinite(value) || value <= 0.0f) return 0;
  if (value >= 6553.5f) return 65535;
  return (uint16_t)(value * 10.0f + 0.5f);
}

uint16_t buildStatusWord() {
  uint16_t status = 0;
  if (mlxDetected) status |= (1u << 0);
  if (mlxInitialized) status |= (1u << 1);
  if (lastFrameValid) status |= (1u << 2);
  if (mcpOk) status |= (1u << 3);
  if (state == SystemState::FaultStop) status |= (1u << 15);
  return status;
}

// =====================================================================
// Periodic Serial diagnostic report.
// =====================================================================
void printDiagnostics() {
  Serial.println();
  Serial.println(F("---- DIAGNOSTICS ----"));
  Serial.printf("State              : %s\n", stateName(state));
  Serial.printf("MCP initialized    : %s\n", mcpOk ? "YES" : "NO");
  Serial.printf("Camera detected    : %s\n", mlxDetected ? "YES" : "NO");
  Serial.printf("Camera initialized : %s\n", mlxInitialized ? "YES" : "NO");
  Serial.printf("Last frame         : %s\n", lastFrameValid ? "OK" : "FAILED");
  Serial.printf("Measured FPS       : %.2f\n", measuredFramesPerSecond);
  Serial.printf("Min/Max/Avg temp C : %.1f / %.1f / %.1f\n",
                minimumTemperatureC, maximumTemperatureC, averageTemperatureC);
  Serial.printf("Implausible pixels : %u (outside %.0fC..%.0fC, rejected from stats/HMI/capture)\n",
                lastFrameRejectedPixelCount, MIN_PLAUSIBLE_TEMP_C, MAX_PLAUSIBLE_TEMP_C);
  Serial.printf("Good/Failed frames : %lu / %lu\n",
                (unsigned long)successfulFrameCount, (unsigned long)failedFrameCount);

  const char *captureStateStr = "?";
  switch (capture.currentState()) {
  case CaptureState::ARMED: captureStateStr = "ARMED"; break;
  case CaptureState::SAMPLING: captureStateStr = "SAMPLING"; break;
  case CaptureState::LATCHED: captureStateStr = "LATCHED"; break;
  }
  Serial.printf("Capture state      : %s\n", captureStateStr);
  Serial.printf("Strip1 present/maxT/hotPx : %d / %.1f / %u\n",
                capture.strip1().present, capture.strip1().maxTempC,
                capture.strip1().hotPixelCount);
  Serial.printf("Strip2 present/maxT/hotPx : %d / %.1f / %u\n",
                capture.strip2().present, capture.strip2().maxTempC,
                capture.strip2().hotPixelCount);

  Serial.printf("NS12 WM attempts/failures : %lu / %lu\n",
                (unsigned long)ns12.wmAttemptCount(), (unsigned long)ns12.wmFailureCount());
  Serial.printf("NS12 RM attempts/success/writeFail/timeout/parseErr : %lu / %lu / %lu / %lu / %lu\n",
                (unsigned long)ns12.rmAttemptCount(), (unsigned long)ns12.rmSuccessCount(),
                (unsigned long)ns12.rmWriteFailureCount(), (unsigned long)ns12.rmTimeoutCount(),
                (unsigned long)ns12.rmParseErrorCount());
  Serial.printf("NS12 32x24 experimental   : %s\n", experimental32x24Effective ? "ON" : "OFF (16x8)");
  Serial.printf("Free heap          : %.1f kB\n", ESP.getFreeHeap() / 1024.0f);
}

// =====================================================================
// Serial diagnostic commands:
//   S/W/I/G/F  -- force state (bench test)
//   1-6        -- toggle MCP outputs
//   M          -- diagnostic test pattern / one-shot matrix push
//   C          -- force capture rearm
//   B          -- baseline capture (placeholder, not yet characterized)
//   X          -- calibrated frame dump
//   R          -- rearm/clear latch
//   D          -- print diagnostics immediately
// =====================================================================
void handleSerialCommand(char c) {
  switch (c) {
  case 'S': state = SystemState::Standby; break;
  case 'W': state = SystemState::WaitingForTube; break;
  case 'I': state = SystemState::InspectingTube; break;
  case 'G': state = SystemState::TubeGap; break;
  case 'F': enterFaultStop(); break;
  case '1': case '2': case '3': case '4': case '5': case '6': {
    if (mcpOk) {
      uint8_t pin = c - '1';
      mcp.digitalWrite(pin, !mcp.digitalRead(pin));
    }
    break;
  }
  case 'M': {
    float testPattern[StripZone::COLS * StripZone::ROWS];
    for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
      testPattern[i] = MATRIX_TEMP_MIN_C +
                       (MATRIX_TEMP_MAX_C - MATRIX_TEMP_MIN_C) *
                           ((float)i / (StripZone::COLS * StripZone::ROWS));
    }
    pushWordLampMatrix(testPattern);
    Serial.println(F("[DIAG] Test pattern pushed."));
    break;
  }
  case 'C':
    capture.rearm();
    Serial.println(F("[DIAG] Capture forced/rearmed."));
    break;
  case 'B':
    Serial.println(F("[DIAG] Baseline capture (not yet characterized -- placeholder)."));
    break;
  case 'X': {
    Serial.println(F("[DIAG] Frame dump (calibrated, from mlx.getFrame()):"));
    for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
      Serial.print(mlxFrame[i], 1);
      Serial.print(i % StripZone::COLS == StripZone::COLS - 1 ? '\n' : ' ');
    }
    break;
  }
  case 'R':
    capture.rearm();
    Serial.println(F("[DIAG] Rearmed."));
    break;
  case 'D':
    printDiagnostics();
    break;
  default:
    break;
  }
}

// =====================================================================
// setup() / loop()
// =====================================================================
void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 3000UL)) {
    delay(10);
  }

  statusLed.begin();
  statusLed.clear();
  statusLed.show();
  setStatusLed(0, 0, 20); // dim blue during startup

  Serial.printf("TGIS-510 %s (%s) booting...\n", FW_VERSION, FW_FILE);

  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(1000);

  printBoardInformation();
  runI2CScanner();

  mcpOk = mcp.begin_I2C(MCP_I2C_ADDR, &Wire);
  Serial.printf("MCP initialized: %s\n", mcpOk ? "YES" : "NO");
  if (mcpOk) {
    for (uint8_t p : {McpPin::NORMAL_STOP, McpPin::FAST_STOP, McpPin::HORN,
                       McpPin::BEACON, McpPin::READY, McpPin::WARNING}) {
      mcp.pinMode(p, OUTPUT);
    }
    for (uint8_t p : {McpPin::ACKNOWLEDGE, McpPin::RESET, McpPin::AUTO,
                       McpPin::MACHINE_STOPPED, McpPin::GLUE_READY}) {
      mcp.pinMode(p, INPUT_PULLUP);
    }
  }

  mlxDetected = isI2CAddressPresent(MLX90640_I2CADDR_DEFAULT);
  bool mlxOk = mlxDetected && initializeMlx();
  mlxInitialized = mlxOk;
  Serial.printf("MLX90640 initialized: %s\n", mlxOk ? "YES" : "NO");
  setStatusLed(mlxOk ? 0 : 30, mlxOk ? 25 : 0, 0);

  ns12.begin();

  pinMode(Pins::PRESENCE_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(Pins::PRESENCE_SENSOR_PIN), presenceIsr, CHANGE);

  pinMode(Pins::KEYENCE_RESULT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(Pins::KEYENCE_RESULT_PIN), keyenceResultIsr, CHANGE);

  keyenceTrigger.begin(Pins::KEYENCE_TRIGGER_PIN);
  encoder.begin(Pins::ENCODER_PULSE_PIN);

  fpsWindowStartMs = millis();
  lastDiagnosticMs = millis();

  if (mcpOk && mlxOk) {
    state = SystemState::Standby;
    setMcpOutputs(false, false, false, false, true, false);
  } else {
    enterFaultStop();
  }

  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true); // true = panic/reset on timeout
  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset();

  encoder.service();
  keyenceTrigger.service();

  if (presenceEdgePending) {
    handlePresenceEdge();
  }
  if (keyenceResultPending) {
    handleKeyenceResult();
  }

  serviceTubePositionTracking();

  // MCP polled only during Standby/TubeGap, ~20ms cadence -- see the
  // KEYENCE_RESULT_PIN comment for why InspectingTube-critical signals
  // must not depend on this.
  if ((state == SystemState::Standby || state == SystemState::TubeGap) && mcpOk) {
    uint32_t now = millis();
    if (now - lastMcpPollMs >= MCP_POLL_INTERVAL_MS) {
      lastMcpPollMs = now;
      bool machineStopped = !mcp.digitalRead(McpPin::MACHINE_STOPPED);
      bool autoMode = !mcp.digitalRead(McpPin::AUTO);
      if (machineStopped) {
        enterFaultStop();
      } else if (state == SystemState::Standby && autoMode) {
        state = SystemState::WaitingForTube;
      } else if (state == SystemState::TubeGap) {
        state = SystemState::WaitingForTube;
        capture.rearm();
      }
    }
  }

  if (mlxInitialized) {
    uint32_t nowMs = millis();
    if (nowMs - lastMlxFrameMs >= MLX_FRAME_PERIOD_MS) {
      lastMlxFrameMs = nowMs;
      int status = mlx.getFrame(mlxFrame);
      lastFrameValid = (status == 0);

      if (lastFrameValid) {
        calculateFrameStatistics();
        updateFrameRate();
        capture.onNewFrame(mlxFrame);
        successfulFrameCount++;
        fpsWindowFrameCount++;
        consecutiveFrameFailures = 0;
        setStatusLed(0, 18, 0); // brief green heartbeat
      } else {
        failedFrameCount++;
        consecutiveFrameFailures++;
        setStatusLed(25, 8, 0);
        if (consecutiveFrameFailures >= FRAME_FAILURE_RECOVERY_COUNT) {
          attemptCameraRecovery();
        }
      }
    }
  } else {
    // Retry camera detection every second without locking the CPU.
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs >= 1000UL) {
      lastRetryMs = millis();
      mlxDetected = isI2CAddressPresent(MLX90640_I2CADDR_DEFAULT);
      if (mlxDetected) {
        mlxInitialized = initializeMlx();
        if (mlxInitialized) {
          consecutiveFrameFailures = 0;
          fpsWindowStartMs = millis();
          fpsWindowFrameCount = 0;
          setStatusLed(0, 25, 0);
          Serial.println(F("MLX90640 recovered and initialized."));
        }
      }
    }
  }

  ns12.setTelemetry((uint16_t)(++heartbeatCounter),
                     toUnsignedX10(measuredFramesPerSecond),
                     toUnsignedX10(minimumTemperatureC),
                     toUnsignedX10(maximumTemperatureC),
                     toUnsignedX10(averageTemperatureC),
                     (uint16_t)successfulFrameCount,
                     (uint16_t)failedFrameCount,
                     (uint16_t)state,
                     buildStatusWord());
  ns12.service();

  serviceDisplayThrottle();
  serviceMatrixPacing();
  checkDisplayAutoFallback();

  if (Serial.available()) {
    handleSerialCommand((char)Serial.read());
  }

  if (state != lastLoggedState) {
    Serial.printf("[STATE] %s -> %s\n", stateName(lastLoggedState), stateName(state));
    lastLoggedState = state;
  }

  if (millis() - lastDiagnosticMs >= DIAGNOSTIC_INTERVAL_MS) {
    lastDiagnosticMs = millis();
    printDiagnostics();
  }
}

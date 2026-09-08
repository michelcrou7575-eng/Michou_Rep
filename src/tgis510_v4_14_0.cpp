// TGIS-510 -- Thermal Glue Inspection System
// Ref: TGIS-510_cpp_V4_14.0
//
// Home-lab / after-hours project. Separate from the 410 Rotaliner Tubing Seal
// Seam Monitor (factory floor, S7-300/ATmega2560) -- do not conflate.
//
// CLEAN-ROOM NOTE: this file was written directly from a project handoff
// synthesis, not by editing an existing tgis510_v4_14_0.cpp. Neither that
// file nor the earlier HotMelt_MLX90640_80032_9_8_0.cpp snapshot were
// available in the environment this was written in. Diff this against
// whatever is actually on disk at "C:/Users/Admin/Documents/PlatformIO/
// Projects/510 HotMelt Monitor/" before flashing to real hardware.
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
#include "driver/pcnt.h"

// =====================================================================
// VERSION -- keep filename, header comment and banner in lockstep.
// (Prior audit finding: header said V4.1.0, banner printed V4.16.0.
//  Fixed here by deriving both from one constant.)
// =====================================================================
#ifndef FW_VERSION_STRING
#define FW_VERSION_STRING "V4.14.0"
#endif
#ifndef FW_FILE_STRING
#define FW_FILE_STRING "tgis510_v4_14_0.cpp"
#endif
static const char *FW_VERSION = FW_VERSION_STRING;
static const char *FW_FILE = FW_FILE_STRING;

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

// =====================================================================
// MLX90640
// 32Hz is the confirmed-stable *nominal* refresh ceiling at 800kHz, but
// measured actual throughput on this hardware is ~8 FPS. All downstream
// timing math uses the measured rate, not the nominal one. 64Hz fails with
// error -8 here -- an I2C bandwidth wall (64Hz needs ~196KB/s vs ~100KB/s
// usable at 800kHz), not a timing bug.
// =====================================================================
static const mlx90640_refreshrate_t MLX_REFRESH_RATE_NOMINAL = MLX90640_32_HZ;
static const float MLX_MEASURED_FPS = 8.0f;
static const uint32_t MLX_FRAME_PERIOD_MS = (uint32_t)(1000.0f / MLX_MEASURED_FPS); // 125ms

// PLACEHOLDER (Action Item 5): bench-test value was 20-40C (hand
// visibility). Real production range is 20.0-180.0C. Confirm this is set
// before running against real hot melt.
static const float MATRIX_TEMP_MIN_C = 20.0f;
static const float MATRIX_TEMP_MAX_C = 180.0f;

// PLACEHOLDER (Action Item 6): guess, needs real glue thermal-signature
// data. Fallback idea if unreliable: frame-to-frame delta spike instead of
// an absolute threshold.
static const float CAPTURE_TRIGGER_TEMP_C = 30.0f;

// PLACEHOLDER (Action Item 7): needs real 200 m/min validation. Tuning knob
// for "does one sampling window match one tube's FOV transit".
static const uint8_t CAPTURE_SAMPLE_COUNT = 4;

Adafruit_MLX90640 mlx;
float mlxFrame[32 * 24];

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
// Ref: Omron NS-Series Host Connection Manual (Cat. No. V085-E1-07), S3.
// Commands: WM (write $W), RM (read $W). WN/RN don't apply (NT31/NT631
// only).
//
// Firmware quirk on this specific PT unit: ESC=0x1B is used for *every*
// command including WM, not the manual's documented 0x1C for word writes.
// Using 0x1C per spec caused every WM write to be silently ignored --
// already-fixed root cause of an early blank-screen bug.
//
// Baud: 38400 is the confirmed-working, current source of truth. (Stale
// header comments/constants elsewhere may say 9600 or 19200 -- code
// constants win over comments when they conflict, per project convention.)
//
// TX=GPIO43, RX=GPIO44, 8N1, via HIN232CP (confirmed -9V/+8V swing).
//
// The exact FCS checksum byte layout for this PT's Memory Link responses
// has not been independently verified against the manual on real hardware
// (this unit already has >=1 undocumented protocol deviation -- the ESC
// byte). The 8-bit XOR checksum below is a best-effort placeholder;
// bench-verify against real WM/RM traffic before trusting it blindly.
// =====================================================================
namespace NS12 {
constexpr uint8_t ESC = 0x1B;
constexpr long BAUD = 38400; // confirmed source of truth -- do not "fix" back to 9600/19200
constexpr uint32_t RM_READ_TIMEOUT_MS = 250; // widened from 120ms after heavier matrix traffic delayed replies

// Word Lamp matrix -- default/trusted mode, 16x8 grid at $W700-$W827,
// column-major, stride 8: address(col,row) = 700 + col*8 + row.
constexpr uint16_t MATRIX_BASE_ADDR = 700;
constexpr uint8_t MATRIX_COLS_DEFAULT = 16;
constexpr uint8_t MATRIX_ROWS_DEFAULT = 8;

// Full 32x24 push previously caused 100% NS12 read-request failure
// (0/424 reads OK) -- the PT couldn't service RM reads while absorbing
// that write load. Reverted to 16x8. A later column-paced 32x24 mode is
// kept here as opt-in/experimental (ENABLE_EXPERIMENTAL_32x24 below),
// flagged in the latest code audit as needing close monitoring -- not yet
// fully trusted. It self-monitors RM failure rate and auto-reverts to
// 16x8 if that spikes (see NS12Manager::checkAutoFallback()).
constexpr bool ENABLE_EXPERIMENTAL_32x24 = false;
constexpr uint8_t MATRIX_COLS_EXPERIMENTAL = 32;
constexpr uint8_t MATRIX_ROWS_EXPERIMENTAL = 24;

// Word Lamp palette: 10 entries, index 0-9. Index 0 renders as blank/off
// -- clamp the coldest output to index 1, never 0, to avoid the
// "writes succeed, nothing shows" bug this caused previously.
constexpr uint8_t PALETTE_MIN_INDEX = 1;
constexpr uint8_t PALETTE_MAX_INDEX = 9;

// Display refresh decoupled from live camera streaming, velocity-adaptive
// per-tube throttle.
constexpr uint32_t TARGET_DISPLAY_REFRESH_MS = 2000;

// Largest single WM burst used anywhere (the 16x8 default matrix: 128
// words). The experimental 32x24 mode never writes more than one column
// (24 words) per WM command -- see the column-paced mechanism below --
// which is what "column-paced" is meant to buy: no single write big enough
// to starve the PT's ability to service RM reads, unlike the old one-shot
// 768-word push that caused 0/424 RM failures.
constexpr uint16_t MAX_WM_WORDS = 128;

// Spacing between successive column writes in the experimental 32x24 mode.
constexpr uint32_t COLUMN_WRITE_INTERVAL_MS = 15;
} // namespace NS12

class NS12Manager {
public:
  void begin() {
    // setTxBufferSize() must precede begin() on the ESP32 Arduino core.
    Serial2.setTxBufferSize(1024);
    Serial2.begin(NS12::BAUD, SERIAL_8N1, Pins::NS12_RX, Pins::NS12_TX);
  }

  // WM: write `count` words starting at `startAddr`. `count` is clamped to
  // NS12::MAX_WM_WORDS -- callers must stay within that (the 16x8 burst and
  // single-column experimental writes both do).
  void sendWM(uint16_t startAddr, const uint16_t *data, uint16_t count) {
    if (count > NS12::MAX_WM_WORDS) count = NS12::MAX_WM_WORDS;
    uint8_t frame[3 + 4 + 4 * NS12::MAX_WM_WORDS + 2 + 1];
    size_t n = 0;
    frame[n++] = NS12::ESC;
    frame[n++] = 'W';
    frame[n++] = 'M';
    n += writeDecimal4(&frame[n], startAddr);
    for (uint16_t i = 0; i < count; i++) {
      n += writeHex4(&frame[n], data[i]);
    }
    uint8_t fcs = computeFcs(frame, n);
    n += writeHex2(&frame[n], fcs);
    frame[n++] = '\r';

    Serial2.write(frame, n);
    // Blocking flush() intentionally removed from WM writes (kept for RM
    // reads) -- a stalled TX flush here would block the whole control loop
    // during InspectingTube.
  }

  // RM: read `count` words starting at `startAddr`. Returns true on a
  // well-formed response within timeout.
  bool sendRM(uint16_t startAddr, uint8_t count, uint16_t *out) {
    uint8_t frame[16];
    size_t n = 0;
    frame[n++] = NS12::ESC;
    frame[n++] = 'R';
    frame[n++] = 'M';
    n += writeDecimal4(&frame[n], startAddr);
    n += writeDecimal2(&frame[n], count);
    uint8_t fcs = computeFcs(frame, n);
    n += writeHex2(&frame[n], fcs);
    frame[n++] = '\r';

    Serial2.write(frame, n);
    Serial2.flush(); // RM reads keep the blocking flush -- WM writes do not.

    bool ok = pollRead(count, out, NS12::RM_READ_TIMEOUT_MS);
    rmAttempts++;
    if (ok) {
      rmSuccesses++;
    }
    return ok;
  }

  uint32_t rmAttemptCount() const { return rmAttempts; }
  uint32_t rmSuccessCount() const { return rmSuccesses; }
  void resetRmStats() { rmAttempts = 0; rmSuccesses = 0; }

private:
  uint32_t rmAttempts = 0;
  uint32_t rmSuccesses = 0;

  // Resyncs to the next ESC byte rather than trusting byte 0 == frame
  // start, discarding stray bytes instead of corrupting the response.
  bool pollRead(uint8_t wordCount, uint16_t *out, uint32_t timeoutMs) {
    uint32_t start = millis();
    // Discard until ESC.
    while ((millis() - start) < timeoutMs) {
      if (Serial2.available()) {
        if (Serial2.peek() == NS12::ESC) break;
        Serial2.read();
      }
    }
    if (!Serial2.available() || Serial2.peek() != NS12::ESC) return false;
    Serial2.read(); // consume ESC

    // Expect two status/command echo bytes, then wordCount*4 hex digits,
    // then 2 hex FCS digits, then CR.
    uint8_t buf[2 + 4 * 32 + 2 + 1];
    size_t need = 2 + (size_t)wordCount * 4 + 2 + 1;
    size_t got = 0;
    while (got < need && (millis() - start) < timeoutMs) {
      if (Serial2.available()) {
        buf[got++] = Serial2.read();
      }
    }
    if (got < need) return false;

    char hex[5];
    hex[4] = '\0';
    for (uint8_t i = 0; i < wordCount; i++) {
      hex[0] = (char)buf[2 + i * 4];
      hex[1] = (char)buf[3 + i * 4];
      hex[2] = (char)buf[4 + i * 4];
      hex[3] = (char)buf[5 + i * 4];
      out[i] = (uint16_t)strtol(hex, nullptr, 16);
    }
    return true;
  }

  static size_t writeDecimal4(uint8_t *dst, uint16_t v) {
    char tmp[5];
    snprintf(tmp, sizeof(tmp), "%04u", v);
    memcpy(dst, tmp, 4);
    return 4;
  }
  static size_t writeDecimal2(uint8_t *dst, uint8_t v) {
    char tmp[3];
    snprintf(tmp, sizeof(tmp), "%02u", v);
    memcpy(dst, tmp, 2);
    return 2;
  }
  static size_t writeHex4(uint8_t *dst, uint16_t v) {
    char tmp[5];
    snprintf(tmp, sizeof(tmp), "%04X", v);
    memcpy(dst, tmp, 4);
    return 4;
  }
  static size_t writeHex2(uint8_t *dst, uint8_t v) {
    char tmp[3];
    snprintf(tmp, sizeof(tmp), "%02X", v);
    memcpy(dst, tmp, 2);
    return 2;
  }
  // PLACEHOLDER: best-effort 8-bit XOR checksum -- bench-verify against
  // real PT traffic (see namespace-level comment above).
  static uint8_t computeFcs(const uint8_t *buf, size_t n) {
    uint8_t fcs = 0;
    for (size_t i = 0; i < n; i++) fcs ^= buf[i];
    return fcs;
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
          if (v > m) m = v;
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
// instead of one 768-word burst. This is the actual "column-paced"
// mitigation the spec describes -- the earlier draft of this file computed
// the whole matrix but still sent it as a single WM burst, which reproduces
// the exact failure mode (PT starved of time to service RM reads) this mode
// exists to avoid.
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
// 32x24 mode is starving RM reads. Reading this (not the raw constant)
// is what makes the fallback actually take effect.
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
  uint32_t now = millis();
  if (now - lastDisplayPushMs < currentDisplayRefreshMs) return;
  lastDisplayPushMs = now;
  displayPushQueued = false;
  pushWordLampMatrix(pendingDisplayFrame);
}

// Called every loop() iteration; no-op unless a paced 32x24 push is active.
void serviceMatrixPacing() {
  if (!pendingMatrix.active) return;
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
// than keep pushing into a PT that can't service reads. This is a runtime
// safeguard standing in for the "close monitoring" the code audit called
// for -- it has not been exercised against real hardware.
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
        memcpy(compositeFrame, frame, sizeof(compositeFrame));
        sampleCount = 1;
        state = CaptureState::SAMPLING;
      }
      break;
    }
    case CaptureState::SAMPLING: {
      for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
        if (frame[i] > compositeFrame[i]) compositeFrame[i] = frame[i];
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

  static float frameMax(const float *frame) {
    float m = frame[0];
    for (size_t i = 1; i < StripZone::COLS * StripZone::ROWS; i++) {
      if (frame[i] > m) m = frame[i];
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
// Serial diagnostic commands:
//   S/W/I/G/F  -- force state (bench test)
//   1-6        -- toggle MCP outputs
//   M          -- diagnostic test pattern / one-shot matrix push
//   C          -- continuous bench stream / force capture
//   B          -- baseline capture
//   X          -- raw/corrected/calibrated dump
//   R          -- rearm/clear latch
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
  default:
    break;
  }
}

// =====================================================================
// setup() / loop()
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("TGIS-510 %s (%s) booting...\n", FW_VERSION, FW_FILE);

  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

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

  bool mlxOk = mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire);
  Serial.printf("MLX90640 initialized: %s\n", mlxOk ? "YES" : "NO");
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX_REFRESH_RATE_NOMINAL);

  ns12.begin();

  pinMode(Pins::PRESENCE_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(Pins::PRESENCE_SENSOR_PIN), presenceIsr, CHANGE);

  pinMode(Pins::KEYENCE_RESULT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(Pins::KEYENCE_RESULT_PIN), keyenceResultIsr, CHANGE);

  keyenceTrigger.begin(Pins::KEYENCE_TRIGGER_PIN);
  encoder.begin(Pins::ENCODER_PULSE_PIN);

  if (mcpOk && mlxOk) {
    state = SystemState::Standby;
    setMcpOutputs(false, false, false, false, true, false);
  } else {
    enterFaultStop();
  }
}

void loop() {
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

  uint32_t nowMs = millis();
  if (nowMs - lastMlxFrameMs >= MLX_FRAME_PERIOD_MS) {
    lastMlxFrameMs = nowMs;
    int status = mlx.getFrame(mlxFrame);
    if (status == 0) {
      capture.onNewFrame(mlxFrame);
    }
  }

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
}

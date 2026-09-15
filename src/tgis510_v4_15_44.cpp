// TGIS-510 -- Thermal Glue Inspection System
// Ref: TGIS-510_cpp_V4_15.44
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
// FIELD UPDATE (V4.15.2): NS12 write traffic confirmed alive on a scope,
// and separately confirmed the "no Serial output" problem reproduces even
// on the archived reference file with a completely different .cpp, which
// rules out application code as the cause -- it's a build-config issue
// (native USB CDC not enabled), fixed in platformio.ini alongside the
// flash-size fix, not yet verified on hardware.
//
// Also V4.15.2: added (not yet wired into acquisition) placeholder
// constants for a raw-ADC-delta HMI path -- see MATRIX_RAW_DELTA_MIN/MAX
// and CAPTURE_TRIGGER_RAW_DELTA below. Intent: the HMI only shows 10
// colors, so running full per-pixel MLX90640 calibration every frame to
// throw away that precision is wasted CPU time; a raw read + per-pixel
// baseline subtraction + linear map straight to a palette index is
// intended to replace it for a faster response. Blocked on the actual
// getRawFrame() signature from this project's vendored library fork
// (not the standard Adafruit_MLX90640 API) before the real acquisition
// path and the two-subpage-to-32x24 combination can be written --
// guessing that part risks a silently checkerboard-corrupted image
// rather than a compile error, so it isn't attempted here yet.
//
// FIELD UPDATE (V4.15.3): got the vendored library's actual source
// (Adafruit_MLX90640.h/.cpp + utility/MLX90640_API.cpp). getRawFrame()
// itself turned out to just call the private MLX90640_GetFrameData()
// twice -- it does NOT resolve subpage-to-pixel mapping for the caller;
// that lives inside the private MLX90640_CalculateTo(). Pulled the real
// formula from that function rather than guessing:
//   row = pixelNumber/32, col = pixelNumber%32
//   chessPattern = (row%2) ^ (col%2)
// A pixel's data is in whichever of the two raw reads has
// frameData[833] (embedded subpage index) == that pixel's chessPattern
// -- getRawFrame() doesn't guarantee frameData0 is always subpage 0, so
// both reads are checked per pixel, not assumed. Verified against the
// actual driver source, not assumed from general MLX90640 knowledge.
//
// The raw-ADC-delta path is now fully wired into the main acquisition
// loop: readMlxRawCombined() (the verified combine above) + rawBaseline
// (captured via 'B', averaging RAW_BASELINE_FRAME_COUNT idle frames) +
// tempToPaletteIndex() against MATRIX_RAW_DELTA_MIN/MAX. mlx.getFrame()
// is no longer called every frame -- only on demand by 'X', for a one-
// off calibrated-C snapshot printed alongside the raw-delta dump, to
// help correlate real thresholds. MATRIX_TEMP_MIN_C/MAX_C and
// CAPTURE_TRIGGER_TEMP_C are unused by the live path now but kept for
// reference. Not yet bench-verified: the two RAW_DELTA placeholder
// values, and the chess-pattern combine logic above (correct by
// inspection of the real driver source, but not yet cross-checked
// against a real image on hardware).
//
// FIELD UPDATE (V4.15.4): with the previous update's write/read overlap
// fix in place, RM went from 100% parse failures ("WM001F" bleeding in
// from writes) to 117/117 clean timeouts -- zero responses, not
// malformed ones. Conclusion: this PT does not answer RM requests in
// this configuration at all; it was never a framing or timing bug.
// Since RM isn't load-bearing for the live 16x8 path (only the
// currently-off experimental 32x24 auto-fallback needs it), and every
// attempt now guarantees paying its full 250ms timeout (during which
// telemetry/matrix writes are deferred) for zero benefit, the periodic
// RM test read is now disabled by default -- see
// NS12_ENABLE_RM_TEST_READ in the NS12 namespace.
//
// FIELD UPDATE (V4.15.5): first full real-hardware run of the raw-delta
// pipeline end to end -- confirmed working correctly, not just compiling.
// Decoded a live telemetry WM frame by hand and every field matched the
// Serial diagnostics panel exactly (fps, state, status bits). Baseline
// recapture correctly freezes stats rather than showing garbage while in
// progress (calculateFrameStatistics()/capture.onNewFrame() are gated
// behind !rawBaselineCaptureInProgress in loop()). Idle raw delta sits
// within +/-75 of baseline, comfortably under CAPTURE_TRIGGER_RAW_DELTA
// (300) -- no false trigger at rest. NS12 WM attempts now climb at the
// full undeferred 4/sec telemetry rate, confirming the V4.15.4 RM-disable
// actually recovered that throughput. Only change this version: the
// "Raw baseline: CAPTURED -- capturing now..." diagnostics line read as
// contradictory (technically accurate -- the previous baseline stays in
// use until a recapture completes -- but confusing) and is now one
// unambiguous message instead of two concatenated ones.
//
// FIELD UPDATE (V4.15.6): root-caused "'M' shows nothing on the PT" from
// the actual captured wire bytes ("WM002BC121,2,3,...") rather than
// guessing again at capture-state gating. Address 02BC(=700) decoded
// correctly, but the count field read "12" while 128 comma-separated
// values followed -- the NS12 wire protocol's LL field is exactly 2
// decimal digits (*L(2 dec)), so the default 16x8 mode's one-shot
// 128-word burst was never a legal frame in the first place, and
// writeDecimal2()'s 2-byte tmp buffer silently truncated "128" to "12"
// instead of erroring. The PT was told to expect 12 words while 128
// followed and never rendered anything -- explains every prior "screen
// shows nothing" report for the default (non-experimental) matrix mode.
// Fixed at the architecture level: pushWordLampMatrix() now always
// column-paces (rows=8 or 24 words per WM command, both well under the
// 99-word protocol ceiling), for both 16x8 and 32x24, never a single
// cols*rows burst. MAX_WM_WORDS corrected from 128 (wrong -- that was
// the bug, not a real limit) to 99 (the actual protocol ceiling), with a
// defensive clamp + counted wmOversizedCount if anything ever requests
// more, so a regression here is visible in diagnostics instead of
// silently corrupting the frame again.
//
// FIELD UPDATE (V4.15.7): confirmed none of ENCODER_PULSE_PIN,
// PRESENCE_SENSOR_PIN, KEYENCE_TRIGGER_PIN, KEYENCE_RESULT_PIN are
// physically wired yet (Action Item 1 still open). The two CHANGE-
// interrupt inputs (presence sensor, Keyence result) were configured as
// bare INPUT -- floating, since nothing's connected -- which invites
// noise-triggered spurious interrupts driving handlePresenceEdge()/
// handleKeyenceResult() off phantom edges instead of a real sensor,
// corrupting bench testing of every other subsystem in the meantime.
// Changed both (and the encoder pulse pin, same reasoning, lower
// operational impact) to INPUT_PULLDOWN for a defined idle state.
// Assumed polarity, not confirmed -- revisit once real sensor output
// types are known, per Action Item 1.
//
// FIELD UPDATE (V4.15.8): replaced the single fixed
// PRESENCE_TO_KEYENCE_DISTANCE_MM with operator-entered HotMelt Start/End
// Position (hotMeltStartPositionMm/hotMeltEndPositionMm), per spec: Start
// is referenced from the tube's leading edge (known immediately), End
// from the trailing edge (only knowable once a tube's length has been
// measured -- so the first tube after a gap gets no End-position check;
// every tube after that uses the previous tube's measured length). Real
// values are meant to come from the HMI via RM (serviceHmiInputPolling(),
// NS12_ENABLE_RM_POLLING) -- currently OFF by default since RM still gets
// zero response from this PT; both positions sit on PLACEHOLDER fallback
// constants until that's resolved. Top suspect for the RM non-response,
// not yet tried: CX-Designer's Memory Link "Response" setting, documented
// OFF in this project's original setup notes -- if that gates ALL PT-
// initiated replies (not just WM-write acknowledgment), it would exactly
// explain WM succeeding at 100% while RM times out at 100%. Try flipping
// it ON and re-downloading the PT project before assuming RM is
// unfixable. Also unconfirmed: the two HOTMELT_*_POSITION_ADDR $W
// addresses (placeholders, never checked against the real CX-Designer
// project) and the HMI numeric input's count-to-mm scale
// (HMI_POSITION_MM_PER_COUNT, assumed 1:1).
//
// FIELD UPDATE (V4.15.9): real screenshot of CX-Designer's Comm. Setting
// screen (Serial Port A / Memory Link) rules out the Response=OFF theory
// above -- Response is confirmed ON. But the same screenshot surfaces a
// field this project had never looked at: "Start Communication $W:
// 16384". Reasoned hypothesis, NOT verified against the official Host
// Connection Manual (Cat. No. V085-E1-07) -- every candidate manual/
// documentation host was blocked by this sandbox's network egress
// policy, so this could not be confirmed against primary documentation:
// if that field defines an offset between a $Wn object's CX-Designer
// label and its actual Memory Link wire address, it would explain the
// exact WM-succeeds/RM-never-responds asymmetry seen throughout this
// project. Added NS12::WORD_ADDRESS_OFFSET (default 0, i.e. unchanged
// behavior) applied centrally in sendWM()/requestRM()/consumeReadWord()
// so every $Wn address elsewhere in this file stays written as its plain
// label regardless of the offset setting. Set it to 16384 (and
// NS12_ENABLE_RM_POLLING to 1) to actually test this on real hardware --
// that's a faster, more reliable answer than continuing to reason about
// it without access to the manual.
//
// FIELD UPDATE (V4.15.10): actually running the V4.15.9 test -- turned
// NS12_ENABLE_RM_POLLING to 1 and, deliberately, split the offset into an
// RM-only constant (NS12::RM_WORD_ADDRESS_OFFSET, set to 16384) instead of
// applying it to WM as well. WM already gets clean transport-level acks at
// its current (unoffset) addresses; there's no evidence WM needs this
// offset, and testing an unverified read-side theory is no reason to risk
// the one path that already works. Next diagnostics capture with this
// build tells the story: NS12 RM attempts/success no longer 0 confirms the
// offset theory (and raises the question of whether WM needs it too, as a
// separate follow-up); still 100% timeout rules it out and this reverts to
// RM_WORD_ADDRESS_OFFSET = 0 pending a new hypothesis.
//
// FIELD UPDATE (V4.15.10, second fix): field report -- "'M' shows the test
// pattern, but it won't go /reset". Root cause: the live display pipeline
// only ever pushes a fresh composite when a real capture completes
// (CaptureController::onNewFrame() -> requestDisplayPush(), by design, so
// the HMI holds a stable QC-confirmation image instead of flickering live
// video); 'R' only reset CaptureController's internal latch, never touched
// the HMI itself. So any one-shot diagnostic push ('M's test pattern, or in
// principle a real QC image) had no way back to blank except waiting for
// the next real capture. 'R' now also pushes an explicit all-minimum
// (coldest palette bucket) frame, making it an actual visible reset.
//
// FIELD UPDATE (V4.15.11): the V4.15.10 RM offset test came back from real
// hardware: "112 / 108 / 0 / 0 / 4" (attempts/success/writeFail/timeout/
// parseErr) -- 0 timeouts, 96% success, versus 117/117 clean timeouts
// before. RM_WORD_ADDRESS_OFFSET=16384 is CONFIRMED correct for this PT/
// project, not just a hypothesis anymore. Diagnostics also showed
// "HotMelt Start/End position (mm) : 33.0 / 34.0 (from HMI)" -- real
// operator-entered values, confirming HOTMELT_START_POSITION_ADDR=11 and
// HOTMELT_END_POSITION_ADDR=12 are correct too. Cross-checked against the
// real CX-Designer Symbol Table (project 510_HotMel_20260902_1): confirmed
// no other object is bound to $W11/$W12; used words nearby are $W0-$W5,
// $HW0, $W10, $W500-$W505, $W700+ (matrix) -- so $W13-$W499 and
// $W506-$W699 are free for future host reads, address budget permitting
// (still needs the same real-project check before use, same as $W11/$W12
// did). Also confirmed from that same symbol table: $B (bit), $W (word),
// $HB and $HW (retentive versions) are independent address spaces on this
// PT, not different views onto the same memory -- $W0 (AutoGen2) and $HW0
// (AutoGen8) already coexist at "address 0" with no conflict, and likewise
// $B10/$B20 sit alongside $W10/$W500s. Not yet investigated: the 4/112 RM
// parse errors (doesn't block operation -- a failed read is just skipped
// and retried next poll cycle) and the HMI_POSITION_MM_PER_COUNT=1.0f
// scale assumption (addresses are now confirmed; whether raw HMI counts
// map 1:1 to mm has not been checked against a physical measurement).
//
// FIELD UPDATE (V4.15.12): added NS12 bit-level RB (read)/WB (write)
// support, extending the existing single-slot non-blocking read state
// machine (readPending/expectedAddr/etc., previously RM-only) with a
// pendingCmdType tag so it now routes 'M' (word) and 'B' (bit) responses
// to separate counters and separate consumeReadWord()/consumeReadBit()
// pop-once accessors. Used for 5 new HMI push-buttons confirmed from the
// real Symbol Table ($B30-$B34: SETUP/ALARM LOG/TREND FULL/TEST/DIAG),
// polled on a faster NS12::BUTTON_POLL_INTERVAL_MS (200ms) than the
// position words since a human is watching for the lamp to react.
// TEST/DIAG button presses call the same pushTestPattern()/
// printDiagnostics() the 'M'/'D' serial commands do (factored
// pushTestPattern() out for this); SETUP/ALARM LOG/TREND FULL are stub
// handlers (lamp + log only) since no such subsystem exists yet -- needs a
// behavior spec before more goes there. Bit addressing uses a new
// NS12::BIT_ADDRESS_OFFSET (16384, reused from the $W offset's own
// screenshot, which separately lists "$B: 16384") applied to BOTH RB and
// WB, unlike the RM/WM split -- there is no already-working $B write to
// protect here, since WB is brand new. UNTESTED as of this writing: the
// offset value for $B, and the entire RB/WB wire format (reasoned by
// analogy to the confirmed WM/RM framing, ESC='W'/'R' + 'B' + '0' + 4-hex
// addr + 2-dec count + data, but never bench-verified). Also PLACEHOLDER:
// the 5 lamp addresses ($B40-$B44) -- no "Button Lamp" objects exist in
// the real Symbol Table yet, so these were picked as the next free block,
// not confirmed. Expect a follow-up FIELD UPDATE once real hardware
// exercises this, the same way RM_WORD_ADDRESS_OFFSET went from
// hypothesis (V4.15.9/10) to confirmed (V4.15.11).
//
// FIELD UPDATE (V4.15.13): the V4.15.12 RB/WB build ran on real hardware --
// diagnostics came back "HMI buttons ... : 1/1/1/1/1", all 5 permanently
// "pressed" (only one setActiveLamp() call ever fired, per WB attempts
// staying at exactly BUTTON_COUNT=5 across dumps -- i.e. one 0->1 edge per
// button, then stuck, not flickering). Physically implausible -- no
// operator holds 5 buttons down continuously -- so this is a read/parse
// bug, not real presses. RB success rate (358/511, ~70%) is also
// meaningfully worse than RM's (112/123, ~91%) under the same traffic, and
// raw captures already show stray bytes shaped like our own WM traffic
// bleeding into a pending RB's response ("[ESC]WM001F" -- correctly
// rejected by the pendingCmdType check, but proves collisions are
// frequent at this address/traffic mix). Leading suspects, not yet
// distinguished: (1) genuine PT replies whose bit-value field isn't a
// plain "0"/"1" the way parseReadResponse() assumes, so it's finding SOME
// nonzero value every time even at rest; (2) our own outgoing RB request
// echoing back on this link (as WM traffic already does per the pre-
// V4.15.4 field report) with just enough trailing garbage after it to
// pass the dataLen>0 check, producing a spuriously nonzero "value" from
// noise, consistently rather than randomly. Added a raw-bytes dump on
// parseReadResponse() SUCCESS (mirroring dumpRejectedLine(), which only
// covered failures) so the next hardware run shows exactly what's being
// matched -- that answers which theory is right instead of guessing
// further. Do not trust HMI button state or lamp behavior until that
// capture is reviewed; TEST/DIAG button dispatch stays wired (so it's
// visible when it misfires) but should be treated as unverified.
//
// FIELD UPDATE (V4.15.14): the stuck-at-1, never-flickering symptom above
// has a much simpler explanation than a wiring/echo issue -- CX-Designer
// momentary bit switches are commonly "host-clear": the panel SETS the
// bit on press and relies on the connected device to clear it back to 0,
// guaranteeing a command isn't missed even under slow host polling. This
// firmware only ever read the buttons, never wrote them back, so the very
// first tap on each would latch it at 1 forever -- exactly what V4.15.12
// showed. serviceHmiButtonPolling() now clears a button's bit back to 0
// (via sendWB) immediately after dispatching its press. The separate
// finding from V4.15.13 (own-WM-traffic bytes bleeding into pending RB
// reads, ~30% of attempts) is still real and still unexplained -- this
// fix addresses why buttons read "always pressed", not why some fraction
// of RB reads fail outright. The raw-bytes-on-success dump added in
// V4.15.13 stays in place to keep validating that the 70% of RB reads
// reported as successful are genuine PT replies.
//
// FIELD UPDATE (V4.15.15): the V4.15.14 build ran on real hardware, and the
// V4.15.13 raw-success dump paid off immediately -- two genuine, address-
// matched RB replies on real button presses ("ALARM LOG"/"TEST") captured
// as "004B" and "0037" (0x4B, 0x37), not "0000"/"0001". Both happen to be
// odd, both correctly corresponded to a real press, so the intended bit
// lives in the LSB of a word whose other content is unexplained (padding?
// an unrelated upper byte?) -- consumeReadBit() now checks `value & 1`
// instead of `value != 0`, which would have misfired on any nonzero-even
// reply. Separately, RM success crashed to 10/52 (~19%, down from ~91-96%)
// once 200ms button polling started competing for the shared link -- RB
// alone made 216 attempts against RM's 52 in the same window. Buttons are
// occasional/diagnostic; HotMelt Start/End Position feeds real tube-
// tracking logic, so BUTTON_POLL_INTERVAL_MS went 200ms->750ms and
// serviceHmiInputPolling() now runs before serviceHmiButtonPolling() each
// loop() tick (reversed from V4.15.12) so positions get first refusal on
// the shared read slot. Also newly observed, once, not yet understood:
// a raw response "[ESC]ER0517" while an RM request was pending -- neither
// our-own-WM-traffic-shaped (that check requires response[1]=='W') nor a
// normal 'R','M'/'B' reply. Could be a genuine PT error/status reply
// (Memory Link protocols often use "ER" for errors) or could be corrupted
// bytes from another overlapping transmission -- one sample isn't enough
// to tell, and no code change follows from it yet. Watch for it recurring.
//
// FIELD UPDATE (V4.15.16): user confirmed real button presses (ALARM LOG,
// TEST) worked correctly on V4.15.15 -- detected, dispatched, cleared --
// but the lamp never visibly lit on the actual HMI. A CX-Designer
// screenshot of the real TEST button object (PB0088) settled the address
// question for good: Write Address $B33 (matches BUTTON_TEST_ADDR),
// Display Address1 $B43 (matches LAMP_TEST_ADDR exactly), Action Type
// Momentary (confirms the host-clear theory from V4.15.14), button type
// "Select Shape(Type2-1)" -- shape lights purely from Display Address1's
// ON/OFF. So the addressing and architecture were right; the bug was
// applying RB_BIT_ADDRESS_OFFSET to WB as well as RB. This is the exact
// asymmetry already known from $W (RM needs the offset, WM does not) --
// V4.15.12 missed applying that same precedent to $B. WB now writes the
// plain address, matching sendWM. If lamps still don't light after this,
// the next suspect is Display Address1 needing an inverted or different
// value convention than a plain 0/1 bit -- but try this first, since it's
// the same fix that was already proven necessary for $W.
//
// FIELD UPDATE (V4.15.17): V4.15.16 field data still showed a familiar
// failure -- "[ESC]WM001F" bleeding into a pending RM read right after a
// burst of matrix column writes, RM down to 67% and RB to 48% (both worse
// than RM's pre-button ~91-96% baseline). Root cause older than the
// buttons: sendWM()/sendWB() are deliberately non-blocking (no flush(),
// so InspectingTube's Keyence/encoder timing never stalls), so a write's
// bytes can still be physically draining off the wire when a read request
// starts a few loop() iterations later -- and this link has echoed its own
// TX back onto RX since before V4.15.4's original write/read-overlap fix.
// Added NS12Manager::markTxBusy(), called from sendWM()/sendWB(), which
// estimates (from frame byte count and NS12::BAUD) when the just-queued
// send will actually finish draining, accumulating across back-to-back
// sends the way a real UART FIFO would. requestRM()/requestRB() now defer
// (return false, retried on their normal poll schedule) until that
// estimate passes -- entirely non-blocking, so it cannot introduce the
// Keyence 100us-pulse jitter an actual flush() would risk.
//
// FIELD UPDATE (V4.15.18): V4.15.17 made no difference -- field report
// "HMI buttons do not reach ESP", meaning the failure may be upstream of
// this firmware's NS12 layer entirely (CX-Designer project setting,
// wiring, or the PT genuinely not answering RB), not something more
// non-blocking-read tuning can fix. Added a minimal direct-GPIO proof rig
// (Pins::TEST_GPIO_*, GPIO_BUTTON_PROOF_TEST) -- 2 outputs + 4 inputs,
// entirely bypassing NS12 -- to isolate "does the ESP32 see a physical
// button press at all" from "does the NS12 link carry it." Inputs reuse
// GPIO4/5/6/7 (confirmed OK for now since the real encoder/presence/
// Keyence sensors those pins belong to are not physically wired yet);
// GPIO10-13 (no conflict at all) were the first choice but are hard to
// reach on the actual board. GPIO_BUTTON_PROOF_TEST makes this and the
// real sensor pin setup in setup() mutually exclusive, since both claim
// the same 4 pins -- set it back to 0 once this test is done and the real
// sensors get wired. Not a replacement for the 5 real HMI buttons/lamps.
//
// FIELD UPDATE (V4.15.19): "Hardwired Done - Final" -- a complete,
// confirmed pin map replacing every remaining PLACEHOLDER pin in this
// file (both the ESP32-direct Pins namespace and the MCP23017 McpPin
// namespace). Changes: KEYENCE_TRIGGER_PIN moved GPIO6->GPIO1; new
// ESP_OPTO_3 (GPIO2) and ESP_INPUT_3 (GPIO6),
// function TBD, wired for I/O testing only (see serviceIoTest() and the
// '1'-'9'/'A'/'K' serial commands); KEYENCE_RESULT_PIN (GPIO7) removed
// entirely -- the field report marks GPIO7 NC, so the direct-GPIO/
// hardware-interrupt Keyence Result mechanism (keyenceResultIsr()) was
// removed. Per the user's explicit choice, Keyence Result moves to the
// MCP23017 (McpPin::INPUT_1 or INPUT_2) -- exact slot not yet decided,
// and this reopens a previously-documented tradeoff: MCP is only polled
// during Standby/TubeGap, never InspectingTube, which was the original
// reason Keyence Result was kept off MCP. Not yet wired into
// keyenceResultPending/Pass -- see that variable's own comment.
//
// GPIO_BUTTON_PROOF_TEST removed entirely (obsolete -- its pins are now
// all claimed by real, final signals). The old McpPin scheme (NORMAL_STOP/
// FAST_STOP/HORN/BEACON/READY/WARNING/ACKNOWLEDGE/RESET/AUTO/
// MACHINE_STOPPED/GLUE_READY) is gone too -- it was a placeholder from
// before any real MCP23017 wiring existed, and none of those names survive
// in the real map (2 status RGB LEDs, 1 spare output, 2 generic inputs, 2
// generic opto outputs). This quietly removed the automatic Standby->
// WaitingForTube transition and the MCP-driven fault path, since both
// depended on AUTO/MACHINE_STOPPED inputs that no longer exist --
// deliberately NOT replaced with a guess at what the 2 real generic MCP
// inputs should mean; Standby now requires an explicit 'W' command until
// that's decided. The field report's own literal pin-index values (e.g.
// "MCP23017_ILED_R = 1" for "GPB0") don't match Adafruit_MCP23X17's actual
// indexing (GPA0-7=0-7, GPB0-7=8-15, this project's own established
// convention from the very scheme just removed) -- corrected using the
// GPA/GPB port+bit each comment names as the source of truth; worth an
// explicit double-check against the board.
//
// FIELD UPDATE (V4.15.20): key clarification -- "ESP32 is NOT the boss
// here." This board is an "Auto Triggered Sensor" reporting to a Siemens
// S7-315-2 PLC, which actually runs the bottomer. Added PlcComms: PLC_
// STATUS out (STOP/ALARM/WARNING/READY, one of 4 mutually-exclusive
// states, binary-encoded across 3 opto outputs -- McpPin::OPTO_1 (bit 0),
// McpPin::OPTO_2 (bit 1), ESP_OPTO_3 (bit 2/MSB), 3 bits chosen over the
// minimum 2 for headroom beyond today's 4 states) and PLC_CONTROL in
// (ACKNOWLEDGE + MACHINE_RUNNING, 2 independent flags, dedicated bits, no
// encoding needed -- McpPin::INPUT_1/INPUT_2). This reclaims the 2 MCP
// inputs V4.15.19 had tentatively reserved for Keyence Result, so Keyence
// Result moved again -- to ESP_INPUT_3/GPIO6, a direct ESP32 GPIO with a
// real hardware interrupt (keyenceResultIsr(), restored to its original
// pre-V4.15.19 shape). This is a strict improvement over the V4.15.19 MCP
// plan: it eliminates the MCP-polling-latency tradeoff entirely (MCP only
// polls during Standby/TubeGap, never InspectingTube) rather than needing
// a decision on whether that tradeoff was acceptable.
//
// Bit-to-pin assignment, the 4 status codes, and which MCP input is
// ACKNOWLEDGE vs MACHINE_RUNNING are ALL this file's placeholder choices,
// clearly separable in PlcComms::setStatus()/servicePlcControl() -- verify
// against the actual S7-315-2 program before trusting them. Serial
// commands 'P' (cycle PLC_STATUS through all 4 real states) and 'K'
// (manual Keyence trigger pulse) added for bench/PLC-program verification;
// '8'/'9'/'A' still do raw per-bit toggles for electrical checks but are
// transient -- overwritten on the next real state-driven PLC_STATUS
// update. No trigger condition exists yet for ALARM/WARNING specifically
// (this project has no concept of "degraded but not faulted" state) --
// only STOP (FaultStop) and READY (everything else) are automatically
// driven; use 'P' for the other two until a real condition is defined.
//
// FIELD UPDATE (V4.15.21): field request -- a physical, tangible way to
// confirm an HMI button press actually completes end-to-end (touch -> RB
// read -> dispatch), without needing to watch Serial. SETUP/ALARM LOG/
// TREND FULL's stub handlers (log-only until now) each toggle one channel
// of the internal RGB LED instead: SETUP->ILED_R, ALARM LOG->ILED_G,
// TREND FULL->ILED_B. Exactly 3 stub buttons and 3 LED channels, so the
// mapping is a natural 1:1, not yet confirmed as the intended final
// pairing -- easy to change if a different assignment is wanted. Still no
// real SETUP/ALARM LOG/TREND FULL behavior; this is a bench-test aid, not
// a step toward one.
//
// FIELD UPDATE (V4.15.22): field report on V4.15.21 -- "ILED_G only, no
// control over it" plus "terminal too fast to read". Two fixes:
// (1) NS12_DEBUG_RAW_RX turned back OFF -- RM/RB/WM/WB have all sat at
// 100% success across several diagnostics dumps since V4.15.17-20's fix,
// so this verbose per-byte trace was only burying other Serial output
// (like the HMI button/LED messages the user was trying to read) with no
// remaining diagnostic value. (2) The MCP output toggle (both '1'-'9' and
// the SETUP/ALARM LOG/TREND FULL LED handlers) used to compute its next
// state from !mcp.digitalRead(pin) -- reading the MCP23017's GPIO
// register back and inverting it. That register isn't guaranteed to
// track the OLAT/output-latch value actually written; "always lights the
// same channel, never toggles" is exactly what a read-back that returns
// the same value every time would produce. Added a firmware-side
// mcpOutputState[9]/toggleMcpOutput() that tracks each of the 9 real MCP
// outputs' commanded state directly, used by both call sites -- toggling
// is now deterministic regardless of what the MCP or the physical LED
// reports back. If ILED_G (or any channel) still won't respond after
// this, the read-back theory is ruled out and the next suspect is
// physical wiring on that specific channel.
//
// FIELD UPDATE (V4.15.23): the toggle fix above worked (real per-channel
// LED control confirmed -- "[HMI] TEST button pressed." / "[HMI] ALARM
// LOG button pressed." / "[HMI] ^ toggled ILED G -> ON" all fired
// correctly) but surfaced a more serious finding: those presses happened
// with nobody touching the screen. Genuine phantom presses, not visible
// noise -- RB had been sitting at 100% success with zero parse errors
// across hundreds of reads in every recent diagnostics dump, so a wrong
// address/count match (which would show up as a parse error, not a
// clean success) is not the explanation. Root cause still unknown.
// Mitigated, not fixed: serviceHmiButtonPolling() now requires the SAME
// button to read "pressed" on two CONSECUTIVE polls (pendingButtonRead[])
// before accepting it and dispatching handleHmiButtonPress() -- a single
// sample is no longer trusted on its own. Costs up to one extra ~750ms
// poll cycle of latency for a real press. Also corrected the McpPin
// naming: what V4.15.19/22 called SPARE_7 is actually ELED_Y, a 4th
// (yellow) channel on the external status indicator -- both internal and
// external LEDs are single RGB(+Y for external) LEDs, one color channel
// lit at a time by design, not 3-4 simultaneous channels mixing colors.
//
// FIELD UPDATE (V4.15.24): field report on V4.15.23 -- ILED_G still turns
// on/off by itself even with the 2-consecutive-read debounce in place.
// This is an important finding, not just "still broken": a single-sample
// glitch is exactly what that debounce should catch, so a phantom
// surviving two confirming polls 750ms apart means the underlying signal
// reads "pressed" consistently for over a second -- a sustained value,
// not electrical noise. That points away from a wiring/collision problem
// and toward $B31 (ALARM LOG) being driven by something in the PT's own
// logic unrelated to touch -- plausible given its name; worth checking in
// CX-Designer whether $B31 is referenced anywhere besides that button
// object (an alarm-summary condition, a macro, etc.), the same way the
// TEST button's screenshot settled an earlier question. Not yet done --
// needs the user to check.
//
// Separately actioned: "use all Internal and External LEDs". The 3 stub
// buttons (SETUP/ALARM LOG/TREND FULL) now share one round-robin cycle
// across all 7 real LED channels (ILED_R/G/B + ELED_R/G/B/Y) instead of
// each owning one fixed internal channel -- advanceLedCycle(), one
// channel lit at a time. Incidental benefit: any future phantom press
// from any of the 3 buttons now visibly advances the same shared LED
// instead of only ever toggling ILED_G, making a recurrence obvious at a
// glance regardless of which button's bit is misbehaving.
//
// FIELD UPDATE (V4.15.25): user confirmed the ALARM LOG phantom "does not
// come from HMI" -- ruling out a PT-side alarm condition and pointing
// squarely at this firmware. Real CX-Designer Address View screenshot of
// the Hot Melt Monitor screen then showed why: "2(P) $W0(W)" at the top --
// a screen-switch object triggered by writing to $W0. TELEMETRY_BASE_ADDR
// had been $W0 since this project's first version, meaning telemetry has
// been writing arbitrary heartbeat/FPS/status values into a page-
// navigation command word every 250ms the entire time. A strong unifying
// explanation for multiple previously-separate symptoms: phantom HMI
// button presses (this firmware's own PLACEHOLDER pin/address guesses
// were never the issue -- $B33(W)/$B43(R1) in the same screenshot matches
// BUTTON_TEST_ADDR/LAMP_TEST_ADDR exactly) and unexpected matrix content
// appearing on 'R' (field report, same session) -- constant bogus page-
// switch attempts could plausibly corrupt the PT's Memory Link state well
// beyond visible screen flicker. Moved TELEMETRY_BASE_ADDR to $W100,
// inside the confirmed-free $W13-$W499 range from the real Symbol Table
// (V4.15.11). The same screenshot also showed a second screen-switch,
// "0(P) $W830(W)" -- not currently written by this file, but worth
// keeping clear of. Root cause not yet proven (needs the fix confirmed on
// hardware), but this is a much better-evidenced explanation than
// anything examined so far, and worth testing before chasing anything
// else.
//
// FIELD UPDATE (V4.15.26): field question -- "never seen 1 ever, are we
// getting the right address?" on the diagnostics "HMI buttons : 0/0/0/0/0"
// line. Addressing is already confirmed correct (the V4.15.25 screenshot's
// $B33(W)/$B43(R1) matches BUTTON_TEST_ADDR/LAMP_TEST_ADDR exactly) --
// the real explanation is simpler: buttonState[i] is force-cleared back to
// false inside the SAME call that detects a press (so the next press is a
// fresh edge without an extra poll-cycle's wait), so the "confirmed
// pressed" state is only ever true for microseconds. A once-a-second
// diagnostics snapshot landing in that window is practically impossible --
// the line reading all-0 forever was never evidence of a broken read.
// Added buttonPressCount[], a lifetime counter incremented in
// handleHmiButtonPress(), printed alongside the existing instantaneous
// line -- this is the actual way to confirm via the periodic diagnostics
// dump (not just a live Serial line caught at the right instant) that
// presses are registering.
//
// FIELD UPDATE (V4.15.27): field report -- even after the V4.15.25 $W0 fix,
// ALARM LOG and TEST still fire phantom presses (lifetime counters 6 and 5
// with nobody touching the screen) while genuine touches on ANY of the 5
// buttons register zero. That combination retroactively casts doubt on the
// V4.15.15 fix this file has relied on since: `value & 1` (LSB) was derived
// from exactly two captured samples (0x4B, 0x37) that were simply assumed to
// be genuine presses because they were nonzero -- if either was itself a
// phantom/garbage read, the whole LSB theory rests on no confirmed-good
// data at all. Rather than propose a fourth bit-position guess blind,
// NS12Manager::consumeReadBit() now also returns the complete raw word
// (lastReadWordValue, unmasked) via a new rawValueOut parameter, and
// serviceHmiButtonPolling() logs it unconditionally as "[HMI-RAW] <name>
// $Bxx raw=0x.... bit0=.." on every poll of every button. This is
// diagnostic-only -- no behavior change to press detection/dispatch --
// meant to produce one unambiguous, timestamped trace of raw values while a
// button is deliberately held pressed vs. deliberately left untouched, so
// the actual bit (or non-bit -- possibly a value that only settles after
// several polls, or an address that isn't independent at all) can be read
// off real evidence instead of guessed again.
//
// FIELD UPDATE (V4.15.28): the V4.15.27 raw-word capture came back with
// nobody touching the screen: $B34(DIAG)=0x38, $B33(TEST)=0x37,
// $B32(TREND FULL)=0x36, $B30(SETUP)=0x4A. Three of the four move in exact
// lockstep with their own requested address ($B32/$B33/$B34 -> raw =
// address+22, precisely) -- a real boolean bit can never track the address
// it was read from like that. This points at the response FRAME itself,
// not the final integer: either the wrong one of parseReadResponse()'s two
// candidateOffsets (3 or 4) is validating by coincidence, or the PT's RB
// data field is wider than the single "0"/"1" character this code has
// always assumed, and bytes belonging to the address/count fields (or a
// neighboring frame) are bleeding into what gets stored as the button's
// value. dumpRejectedLine() already prints raw bytes on a parse FAILURE,
// but these RB reads are structurally succeeding (addr+count match), so
// that path never fires -- there was no visibility into a "successful"
// frame's actual content. Added an unconditional (not gated by
// NS12_DEBUG_RAW_RX) per-successful-RB-poll dump logging the matched
// offset plus the literal addrText/countText/dataText substrings and raw
// line bytes, so the next capture shows the real wire frame instead of
// only its (apparently untrustworthy) parsed value.
//
// FIELD UPDATE (V4.15.29): the V4.15.28 raw-frame dump confirmed the data
// field is a genuine 4-hex-digit WORD (e.g. "ESC RB401F01004B", dataText=
// "004B") -- structurally identical to an RM response, not the single "0"/
// "1" character this code assumed since V4.15.12. Address math still
// checks out (401F = 31+16384 for ALARM LOG). A controlled hold test on
// SETUP ($B30) then came back ambiguous: dataText stayed "004A" every time
// it was sampled, but at BUTTON_POLL_INTERVAL_MS=750ms round-robin across 5
// buttons, any one button is only actually queried once every ~3.75s --
// a 2-3s hold can easily complete without a single read ever landing
// during the touch. That result cannot distinguish "no bit lives in this
// word" from "we just didn't sample it while pressed". Added an 'H' serial
// command: burst-probe mode polls all 5 buttons back-to-back (bounded only
// by the link's own turnaround, not the 750ms cadence) for
// BURST_PROBE_DURATION_MS, guaranteeing several samples land during even a
// short real hold. Diagnostic-only, self-reverting after the duration --
// no change to normal-mode polling cadence or press dispatch.
//
// FIELD UPDATE (V4.15.30): the V4.15.29 burst probe returned a very clean
// answer -- across dozens of rapid-fire samples spanning real button
// presses, every address held its exact value with zero variation:
// $B30=004A, $B31=004B, $B32=0036, $B33=0037, $B34=0038, every single time.
// That's not noise and it's not a live bit -- it's static content that
// never once responded to a genuine touch. RB_BIT_ADDRESS_OFFSET reuses the
// identical constant (16384/0x4000) already confirmed correct for RM's $W
// (word) reads. If that value is actually this PT's *word*-area code
// rather than an offset specific to bit reads, RB($B30..34) would be
// silently landing on $W30..$W34 instead -- real, valid memory (hence no
// timeouts), inside the "$W13-$W499 confirmed free" range from the real
// Symbol Table (explaining the static, unrelated-looking content), but
// never the true $B bit area at all. Added a 'V' serial command: reads
// $W30..$W34 via the independently-confirmed-correct RM path and prints
// each for direct comparison against RB's values. An exact match proves
// RB has been reading word memory all along, not bit memory -- the real
// bug would then be the offset (or the RB command's address encoding
// entirely), never the `& 1` bit-position extraction this file spent
// V4.15.15-27 tuning on data that was never a real bit to begin with.
//
// FIELD UPDATE (V4.15.31): got the actual manual (Cat. No. V085-E1-07) --
// the V4.15.30 word-vs-bit question turned out moot, because the real bug
// was one level lower, in parseReadResponse() itself, affecting RM and RB
// alike. The manual documents SUM as *always* appended by the PT to every
// response ("Be sure that it is added when PT is transmitting"), fully
// independent of the HOST's own *S=SUM-off setting used in outgoing
// commands -- this code had only ever considered *S from OUR side, and
// treated the PT's trailing 2-hex-digit checksum as if it were part of
// *D. Proof, not another guess: computed SUM (lower byte of the sum of
// every byte from ESC through *D) for all 5 real RB frames captured in
// the field (e.g. "RB401F0100" + SUM "4B" for ALARM LOG) and got an exact
// match against the "4B"/"36"/"4A"/"37"/"38" this code had been reading as
// the button's value, for every single one. The real *D in all 5 was "00"
// (OFF) -- every "phantom press" was this code checking the checksum byte,
// not the button; V4.15.15's `& 1` theory was built on 2 samples that were
// never real button data at all. The manual's own worked example also
// shows the true bit lives at bit 7 (MSB) of *D's first byte for a 1-bit
// read, not bit 0. parseReadResponse() now strips and validates SUM
// (rejecting a frame whose checksum doesn't match -- a real integrity
// check this link never had before), and consumeReadBit() now checks
// 0x80. This is the actual fix, not another diagnostic build -- next real
// hardware run should show clean 0x00/0x80 button reads and genuine
// presses finally registering.
//
// FIELD UPDATE (V4.15.32): the checksum fix held up under exhaustive
// real-hardware retesting -- hundreds of burst-probe RB samples, all
// checksum-valid, all genuinely 0x00 -- but genuine touchscreen presses
// still never flip bit7, even after the CX-Designer switch type was
// changed from Momentary to Alternate/SET and re-downloaded to the panel.
// One field report of "$B30 shows ON on the HMI" turned out to be the
// LAMP indicator ($B40, a separate address this firmware itself writes
// via setActiveLamp()/sendWB() -- see that function), not live proof of a
// touch reaching $B30; it was very likely left lit by an earlier
// pre-checksum-fix phantom detection and never confirms anything about
// the touch itself. Separately, a third-party suggestion proposing an
// unsolicited WM-bitmask notify scheme was checked against the real
// manual and rejected: it misstated the *S SET/OR encoding (manual: 1=SET,
// not OR) and proposed reverting ESC/baud/pins to values already disproven
// on this exact unit (0x1C, 38400, GPIO16/17) -- not used here. The
// manual's real SM/SB/SD/SH change-notice commands do exist, but are
// gated by a PT-side "Notice Start $B" threshold that only affects
// unsolicited notify, not our direct RB reads -- irrelevant to why RB
// reads 0.
//
// To isolate the remaining question -- is $B30 ever genuinely written by
// anything, or is there still a firmware bug in the write path nobody has
// tested -- added a 'J' serial command: writes 1 to $B30 via sendWB()
// OURSELVES (no touchscreen involved at all) and bursts RB reads of it.
// If it reads back bit7=1, sendWB()/requestRB()/parseReadResponse() are
// proven correct end to end and the touch not reaching $B30 is
// conclusively a CX-Designer/PT-side configuration issue. If even our own
// write doesn't read back, that points at a real bug in sendWB()'s *D
// encoding -- the manual describes WB's *D as 4-bits-per-hex-digit,
// MSB-first (same convention RB's fix uncovered for reads), which this
// code's literal '0'/'1'-per-bit encoding (unchanged since V4.15.12) has
// never been checked against.
//
// FIELD UPDATE (V4.15.33): the 'J' test came back conclusive -- writing
// count=1, bits[0]=true to $B30 with the OLD encoding (literal ASCII "1")
// read back as 0x00 via a checksum-validated RB, on our OWN write with no
// touchscreen involved at all. That rules out "CX-Designer config only"
// as the sole explanation: a real bug existed in sendWB() itself, and had
// been there since V4.15.12. Fixed to match the manual's documented WB
// *D encoding: 4 bits packed per hex digit, MSB-first ("fills data in
// descending order starting with first digit") -- the same convention
// that turned out to be the real RB fix, just for the write side.
// bits[0]=true for a 1-bit write now encodes as hex digit "8" (binary
// 1000, the requested bit in the nibble's MSB position), not "1". A
// "false" write encodes as "0" either way, which is exactly why the
// host-clear-to-0 path (every prior press-detection cycle) never showed
// a symptom while every write of an ON bit -- lamps, and this test --
// was silently malformed the whole time. Multi-bit writes (the 5-bit
// lamp array) now pack into ceil(count/4) hex digits per the same rule,
// instead of comma-separated literal characters. Send 'J' again on this
// build: if $B30 finally reads back bit7=1, this was the fix.
//
// FIELD UPDATE (V4.15.34): the V4.15.33 WB encoding fix was real (the
// literal-'1' bug is gone) but 'J' still reads back bit7=0 via the
// OFFSET-based RB address -- yet the user directly watched SETUP turn ON
// on the physical screen the instant 'J' was sent. That combination is
// the real finding here: our own write demonstrably reached the PT's
// live, rendered memory, but the offset address RB has always read from
// (startAddr + RB_BIT_ADDRESS_OFFSET = +16384) apparently is NOT the same
// physical bit sendWB()'s plain address writes to. On reflection,
// RB_BIT_ADDRESS_OFFSET's entire "confirmation" history amounts to: (a)
// producing structurally valid, checksummed responses -- true of ANY
// valid address in range, never proof it's the CORRECT one -- and (b) one
// coincidence (the "33/34mm" position values that V4.15.31 proved were
// actually checksum bytes, not real word data). Neither actually verifies
// +16384 reads the logical object CX-Designer calls "$B30". Added an 'N'
// serial command: writes 1 to $B30 (same as 'J'), then reads it back
// using the exact same PLAIN address sendWB() already writes to --
// bypassing RB_BIT_ADDRESS_OFFSET entirely -- to test directly whether
// reads need the same unoffset address writes do.
//
// FIELD UPDATE (V4.15.35): the 'N' test came back conclusive and clean --
// "[ESC]RB001E01804E": addrText="001E" (30 decimal, the plain address,
// exactly as predicted), dataText="80" (bit7=1, real nonzero data), sum
// valid. The very next line, same instant, same button, OFFSET address:
// dataText="00". Two reads of "the same $B30", one showing the write and
// one not, settles it -- RB_BIT_ADDRESS_OFFSET was never the right
// address for bit reads. requestRB() now uses the plain address
// unconditionally (matching sendWB(), which has been right since
// V4.15.16), and consumeReadBit() no longer subtracts the offset when
// reporting addrOut. This is the real fix for the entire "buttons never
// register" saga -- RM, RB, WM, and WB all agree now: $W needs +16384 for
// reads only; $B needs no offset at all, for reads or writes. Next real
// hardware run should finally show genuine touchscreen presses flipping
// bit7 during serviceHmiButtonPolling()'s normal round-robin, no burst
// mode or manual WB test required.
//
// FIELD UPDATE (V4.15.36): buttons fully confirmed working on real
// hardware with SETUP reverted to Momentary (the switch-type experiment
// is no longer needed -- host-clear already handles Momentary/Alternate/
// SET alike). Next field report: "$B40 doesn't toggle" -- the lamp write
// (setActiveLamp() -> sendWB(kLampAddrs[0], lamps, 5)) packs 5 bits into
// 2 hex digits, a case the V4.15.33 encoding fix was never independently
// verified against (only the single-bit $B30 write was, via 'J'/'N').
// sendWB() is fire-and-forget with no response, so there was no way to
// inspect what it actually put on the wire short of the full
// NS12_DEBUG_RAW_RX firehose. Added an unconditional per-send print
// (mirroring RB's [NS12-FRAME]) instead of gating this evidence behind a
// flag or guessing at the multi-bit packing again -- next lamp write will
// show the real addr/count/raw bytes directly.
//
// FIELD UPDATE (V4.15.37): the V4.15.36 [NS12-WB-TX] print confirmed the
// lamp write encoding is correct (addr=0x28, *D="80" for "SETUP lamp ON,
// others off" -- exactly right per the packing rules), so no fix was
// needed there. Separate field question: "why 6 sec to get a toggle?"
// Two costs were stacking: the 5-button round-robin (750ms each = 3.75s
// worst case to poll any one button) and the V4.15.23 2-consecutive-
// reads debounce on top of it (up to one more full cycle, ~7.5s worst
// case combined). That debounce was a mitigation for the checksum bug
// (V4.15.31) and RB offset bug (V4.15.35), both since fixed and hardware-
// confirmed clean -- checksum validation in parseReadResponse() already
// rejects a genuinely corrupted frame outright, a stronger guarantee than
// "wait for two reads to agree" ever was. Removed pendingButtonRead[] and
// the debounce entirely; a single checksum-valid read is now trusted
// directly, roughly halving worst-case detection latency to the
// round-robin's own ~3.75s. BUTTON_POLL_INTERVAL_MS (750ms, chosen in
// V4.15.15 to stop RM position polling from starving) is untouched here
// -- a further latency win, but a separate, riskier change to verify
// against RM success rate before making.
//
// FIELD UPDATE (V4.15.38): "we'll do toggle in HMI" -- the switches moved
// from Momentary to native panel-side Alternate, since ESP32-side
// detection latency (~3.75s worst case, per V4.15.37) is too slow for a
// status lamp that needs to react the instant a finger lands. Field
// pushback on the follow-up plan ("what good is a status lamp if it can't
// show absolute reality") was correct: this file's own setActiveLamp()
// (one WB-driven lamp for "whoever was pressed most recently") was never
// a real-state indicator, only an approximation, and the host-clear-to-0
// write on $B30-$B34 was actively WRONG for Alternate switches --
// forcing the panel's own toggle bit back off immediately after every
// press it was supposed to own. Both removed. $B30-$B34 are now pure
// read-only reflections of each switch's real, panel-owned state;
// buttonState[] mirrors that live and unconditionally (no more force-
// clearing), so the diagnostics "instantaneous" line is finally
// meaningful too, not just the lifetime counters. handleHmiButtonPress()
// still fires once per rising edge for its one-shot actions (LED cycle/
// test pattern/diagnostics) -- only the writing back stops.
//
// FIELD UPDATE (V4.15.39): "clean the terminal and show which function is
// on" -- the per-poll debug prints ([NS12-FRAME] for every RM/RB read,
// [HMI-RAW] for every button poll, [NS12-WB-TX] for every WB send) were
// all made unconditional at various points during the checksum/offset/
// encoding investigation, specifically so real evidence couldn't be
// missed. That investigation is done and every fix confirmed on hardware,
// so all three are re-gated behind NS12_DEBUG_RAW_RX (default off) --
// they remain available by flipping that one flag if this link ever needs
// re-diagnosing. A redundant, already-gated duplicate of [NS12-FRAME]
// (V4.15.13, never removed when V4.15.28 superseded it) is deleted rather
// than also gated. In their place: a new unconditional "[SWITCH] <name>
// -> ON/OFF" line, printed only when a switch's real state actually
// changes -- the clean, at-a-glance "which function is on" the terminal
// was missing. The existing periodic diagnostics dump's "instantaneous"
// line (now genuinely live, per V4.15.38) still gives the full snapshot
// on demand.
//
// FIELD UPDATE (V4.15.40): "need 5 different led as status" -- each of
// the 5 HMI switches now has its own dedicated physical LED (ILED_R/G/B,
// ELED_R/G for SETUP/ALARM LOG/TREND FULL/TEST/DIAG respectively), lit
// exactly when that switch is really on and off exactly when it's really
// off. Replaces advanceLedCycle(), which only ever showed "whoever was
// pressed most recently" cycling through one shared LED -- never a real
// per-switch indicator, same category of problem as the removed
// ESP32-driven lamp write (V4.15.38). setButtonStatusLed() is called from
// serviceHmiButtonPolling() on every real state change (both edges), not
// from handleHmiButtonPress() (rising-edge-only, one-shot dispatch) --
// a status LED has to track the switch turning off too. TEST and DIAG
// keep their existing one-shot actions (test pattern / diagnostics
// print) in addition to their own dedicated status LED; SETUP/ALARM LOG/
// TREND FULL still have no other defined action, same as before.
//
// FIELD UPDATE (V4.15.41): field report -- even with the dedicated status
// LEDs and the V4.15.37 debounce removal, still "3-5 sec to react", which
// is consistent with (not a new bug beyond) BUTTON_POLL_INTERVAL_MS=750ms
// x 5 buttons = 3.75s worst-case round-robin. Lowered to 250ms as a
// verify-first step -- see BUTTON_POLL_INTERVAL_MS's own comment for why
// not the original 200ms (that specific value has a documented real-
// hardware failure from V4.15.15, predating markTxBusy() and the
// checksum/offset fixes) and why 250ms is now plausibly safe (burst mode
// already polls all 5 buttons far faster than 200ms with zero anomalies
// across hundreds of samples). Needs confirmation: watch NS12 RM/RB
// success rates in the next diagnostics dump for any regression before
// going lower.
//
// FIELD UPDATE (V4.15.42): raised NS12::BAUD 9600 -> 38400 at the operator's
// request, on record that the earlier 9600 choice traces to a documented
// ~15% RM timeout rate at 38400 in the original bench testing (see the
// MERGE NOTE above) -- this reverses that call, not repeats it blind.
// Requires the matching change on the panel side (CX-Designer Communication
// Settings for the port driving Memory Link, redownloaded to the PT) before
// this link will work at all; the two sides do not auto-negotiate. Watch
// the next diagnostics dump's RM/RB success rates for a return of the old
// timeout pattern -- markTxBusy()'s per-frame timing estimate scales with
// NS12::BAUD automatically, so no other constant needs to move with it.
//
// FIELD UPDATE (V4.15.43): "reset the other button on a new push" -- the 5
// SETUP/ALARM LOG/TREND FULL/TEST/DIAG switches are mutually-exclusive
// screen/function selectors, but native panel-side Alternate toggling
// (V4.15.38) only guarantees each switch reflects ITS OWN real state; it
// has no notion of the other 4, so more than one could stay lit at once
// once switches are pressed out of order. serviceHmiButtonPolling() now
// WB-clears every OTHER switch that's currently on at the moment a new one
// is detected going ON, restoring one-active-at-a-time. This does not
// revive the V4.15.38 host-clear-to-0 pattern the operator objected to --
// that used to fight a switch's own bit right after ITS OWN press; this
// only ever writes a DIFFERENT switch's bit, and only the one the panel's
// own screen-selection logic didn't already turn off itself, so each
// switch's own toggle is still the sole authority on its own state.
//
// FIELD UPDATE (V4.15.44): encoder now wired to a real ZATOR LMZ02 --
// printDiagnostics() ('D') gains an "Encoder count" line (raw PCNT total
// and the ENCODER_COUNTS_PER_MM-derived mm figure) so it's actually
// visible over serial; encoder.total() was only ever consumed internally
// (forward-projection math) before this. ENCODER_COUNTS_PER_MM is still
// the placeholder 1.0 from Action Item 3 (real pulses-per-mm unconfirmed
// for this encoder) -- use the raw count to derive it: move the tube a
// known distance and divide counts by mm.
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
#define FW_VERSION_STRING "V4.15.41"
#endif
#ifndef FW_FILE_STRING
#define FW_FILE_STRING "tgis510_v4_15_41.cpp"
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
// CONFIRMED (V4.15.19): "Hardwired Done - Final" per field report --
// superseded the old Action Item 1 PLACEHOLDER pins below. See the
// CONFIRMED comment on the Pins namespace itself for details.
// =====================================================================
// CONFIRMED (V4.15.19) -- "Hardwired Done - Final" per field report. This
// replaces every earlier PLACEHOLDER pin guess in this namespace; real
// electrical specs given alongside each pin (24V field I/O through 220ohm
// current-limit resistors on outputs and 10K/1.5K dividers down to 3.2V on
// inputs) confirm these are opto-isolated real-world connections, not bare
// GPIO. GPIO0/GPIO3/GPIO7 are S3 boot-strapping pins or explicitly marked
// NC by the field report -- never use them.
namespace Pins {
constexpr uint8_t KEYENCE_TRIGGER_PIN = 1;   // 24V/220ohm OUTPUT, Conn 9
// V4.15.20: bit 2 (MSB) of the 3-bit PLC_STATUS word to the S7-315-2 --
// see the PlcComms namespace below. Real 24V/220ohm opto output, same
// electrical class as McpPin::OPTO_1/OPTO_2 (the other 2 bits), just
// routed through the ESP32 directly instead of the MCP23017.
constexpr uint8_t ESP_OPTO_3 = 2; // 24V/220ohm OUTPUT, Conn 8
// GPIO3: S3 boot-strapping pin, avoid.
constexpr uint8_t ENCODER_PULSE_PIN = 4;     // 24V/10K-1.5K divider -> 3.2V INPUT, Conn 7
constexpr uint8_t PRESENCE_SENSOR_PIN = 5;   // 24V/10K-1.5K divider -> 3.2V INPUT, Conn 6
// V4.15.20: reassigned to Keyence Result. The 2 MCP inputs (INPUT_1/
// INPUT_2) are now spoken for by PLC_CONTROL (ACKNOWLEDGE/MACHINE_RUNNING,
// see PlcComms namespace) instead, so Keyence Result needed a new home --
// this direct ESP32 GPIO is a strictly better fit than the MCP ever was:
// restores the original hardware-interrupt mechanism (keyenceResultIsr()),
// eliminating the MCP-polling latency tradeoff entirely (MCP is only
// polled during Standby/TubeGap, never InspectingTube -- exactly why
// Keyence Result was on a direct GPIO in the first place, before V4.15.19
// tentatively moved it to MCP for lack of a better slot at the time).
// This is my call, not yet confirmed with the user -- flag if ESP_INPUT_3
// was meant for something else.
constexpr uint8_t ESP_INPUT_3 = 6;   // 24V/10K-1.5K divider -> 3.2V INPUT, Conn 5
// GPIO7: field report marks this NC -- never use it. Keyence Result's
// original GPIO7 slot moved twice since: briefly to the MCP23017
// (V4.15.19), then to ESP_INPUT_3/GPIO6 (V4.15.20, see that pin's own
// comment) once the MCP inputs were claimed by PLC_CONTROL instead.
constexpr uint8_t I2C_SDA = 8;
constexpr uint8_t I2C_SCL = 9;
// GPIO10, GPIO11: field report marks these NC.

// Waveshare ESP32-S3-Zero onboard WS2812 RGB LED.
constexpr uint8_t RGB_LED = 21;

constexpr uint8_t NS12_TX = 43;
constexpr uint8_t NS12_RX = 44;
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
// Holds raw-ADC-delta values (see below) in the live acquisition path, NOT
// calibrated degrees C -- kept as "mlxFrame" and the same 32x24 float
// layout because CaptureController, downsampleMaxBlock(), and the rest of
// the analysis pipeline were already written generically against "a float
// per pixel compared to MIN/MAX constants" and don't care what physical
// unit that float represents.
float mlxFrame[32 * 24];

bool mlxDetected = false;
bool mlxInitialized = false;
bool lastFrameValid = false;
uint32_t successfulFrameCount = 0;
uint32_t failedFrameCount = 0;
uint8_t consecutiveFrameFailures = 0;
constexpr uint8_t FRAME_FAILURE_RECOVERY_COUNT = 5;

// Despite the "TemperatureC" names (kept to minimize churn against
// telemetry/diagnostics call sites), these hold raw-ADC-delta statistics
// as of V4.15.3, not degrees C -- see the raw-acquisition section below.
float minimumTemperatureC = NAN;
float maximumTemperatureC = NAN;
float averageTemperatureC = NAN;

float measuredFramesPerSecond = 0.0f;
uint32_t fpsWindowStartMs = 0;
uint32_t fpsWindowFrameCount = 0;

// =====================================================================
// Raw-ADC-delta acquisition (V4.15.3) -- replaces per-frame calibrated
// getFrame() in the main loop. The HMI only ever shows 10 colors, so
// running the MLX90640's full floating-point calibration on all 768
// pixels every frame wasted CPU time for that output resolution; this
// reads raw subpage data (getRawFrame(), this project's own addition to
// Adafruit_MLX90640, NOT the standard API) and subtracts a once-captured
// per-pixel idle baseline instead.
//
// Subpage-to-pixel combination verified against the actual driver source
// (utility/MLX90640_API.cpp, MLX90640_CalculateTo()), not guessed:
//   row = pixelNumber / 32, col = pixelNumber % 32
//   chessPattern = (row % 2) ^ (col % 2)
// A pixel's valid data lives in whichever of the two raw reads has
// frameData[833] (the subpage index) equal to that pixel's chessPattern.
// getRawFrame() does not guarantee frameData0 is always subpage 0 -- it
// just returns "whichever subpage was next ready" twice -- so both reads
// are checked per pixel rather than assumed.
// =====================================================================
uint16_t rawPage0[834];
uint16_t rawPage1[834];

// Per-pixel idle baseline (raw ADC counts, signed), captured once via the
// 'B' serial command. Needed because each pixel has its own EEPROM offset/
// gain trim -- real fixed-pattern noise, not sensor noise -- so raw counts
// are only meaningful for threshold detection relative to a pixel's own
// idle value, not compared directly against a single global threshold.
float rawBaseline[32 * 24] = {0};
bool rawBaselineCaptured = false;
bool rawBaselineCaptureInProgress = false;
uint8_t rawBaselineFramesCollected = 0;
float rawBaselineAccumulator[32 * 24] = {0};
constexpr uint8_t RAW_BASELINE_FRAME_COUNT = 32;

// Combines one getRawFrame() result into a signed, per-pixel raw-ADC-count
// array (NOT baseline-subtracted -- see subtractBaseline() below). Returns
// false if getRawFrame() itself failed.
bool readMlxRawCombined(float *outPixels) {
  int status = mlx.getRawFrame(rawPage0, rawPage1);
  if (status != 0) return false;

  uint16_t subpageOf0 = rawPage0[833];
  uint16_t subpageOf1 = rawPage1[833];

  for (int pixelNumber = 0; pixelNumber < 32 * 24; pixelNumber++) {
    int row = pixelNumber / 32;
    int col = pixelNumber % 32;
    int chessPattern = (row % 2) ^ (col % 2);

    uint16_t raw;
    if (chessPattern == subpageOf0) {
      raw = rawPage0[pixelNumber];
    } else if (chessPattern == subpageOf1) {
      raw = rawPage1[pixelNumber];
    } else {
      // Neither read claims this pixel's subpage -- shouldn't happen if
      // frameData0/1 are genuinely the two different subpages, but don't
      // fabricate a value if it does.
      outPixels[pixelNumber] = NAN;
      continue;
    }

    // Raw ADC counts are signed via two's complement, same convention the
    // driver's own MLX90640_CalculateTo() uses on frameData[pixelNumber].
    int32_t signedRaw = (int32_t)raw;
    if (signedRaw > 32767) signedRaw -= 65536;
    outPixels[pixelNumber] = (float)signedRaw;
  }
  return true;
}

void subtractBaseline(const float *rawPixels, float *outDelta) {
  for (int i = 0; i < 32 * 24; i++) {
    outDelta[i] = rawPixels[i] - rawBaseline[i];
  }
}

void startRawBaselineCapture() {
  rawBaselineCaptureInProgress = true;
  rawBaselineFramesCollected = 0;
  for (int i = 0; i < 32 * 24; i++) rawBaselineAccumulator[i] = 0;
  Serial.printf("[MLX] Baseline capture starting -- averaging %u idle frames.\n",
                RAW_BASELINE_FRAME_COUNT);
}

// Call once per successful raw read while a baseline capture is running.
void serviceRawBaselineCapture(const float *rawPixels) {
  if (!rawBaselineCaptureInProgress) return;
  for (int i = 0; i < 32 * 24; i++) rawBaselineAccumulator[i] += rawPixels[i];
  rawBaselineFramesCollected++;
  if (rawBaselineFramesCollected >= RAW_BASELINE_FRAME_COUNT) {
    for (int i = 0; i < 32 * 24; i++) {
      rawBaseline[i] = rawBaselineAccumulator[i] / (float)RAW_BASELINE_FRAME_COUNT;
    }
    rawBaselineCaptureInProgress = false;
    rawBaselineCaptured = true;
    Serial.println(F("[MLX] Baseline capture complete."));
  }
}

// PLACEHOLDER, no physical grounding yet (see MATRIX_RAW_DELTA_MIN/MAX
// below for why): sanity bound wide enough to allow real signal up to
// several times CAPTURE_TRIGGER_RAW_DELTA with margin, while still
// rejecting a wildly out-of-range single-pixel glitch. Needs the same
// bench characterization as MATRIX_RAW_DELTA_MIN/MAX.
constexpr float MIN_PLAUSIBLE_RAW_DELTA = -5000.0f;
constexpr float MAX_PLAUSIBLE_RAW_DELTA = 5000.0f;
uint16_t lastFrameRejectedPixelCount = 0;

bool isPlausibleTemp(float t) {
  return isfinite(t) && t >= MIN_PLAUSIBLE_RAW_DELTA && t <= MAX_PLAUSIBLE_RAW_DELTA;
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

// SUPERSEDED as of V4.15.3 -- the live acquisition/palette/capture-trigger
// path now runs on raw-ADC-delta units (MATRIX_RAW_DELTA_MIN/MAX,
// CAPTURE_TRIGGER_RAW_DELTA below), not degrees C. Kept, unused by the
// live path, only in case degree-accurate calibration is ever needed
// again (e.g. an audit trail, or reverting to mlx.getFrame()).
static const float MATRIX_TEMP_MIN_C = 20.0f;
static const float MATRIX_TEMP_MAX_C = 180.0f;
static const float CAPTURE_TRIGGER_TEMP_C = 30.0f;

// PLACEHOLDER (Action Item 7): needs real 200 m/min validation. Tuning knob
// for "does one sampling window match one tube's FOV transit".
static const uint8_t CAPTURE_SAMPLE_COUNT = 4;

// =====================================================================
// Raw-ADC-delta thresholds (V4.15.3) -- these, not the Celsius constants
// above, are what the live acquisition/palette/capture-trigger path
// actually uses now. See the raw-acquisition section (readMlxRawCombined,
// subtractBaseline, rawBaseline) above for how mlxFrame[] gets populated.
//
// These are placeholders in a more literal sense than MATRIX_TEMP_MIN_C/
// MAX_C ever were: a Celsius guess has real-world grounding (hot melt
// glue is known to run 150-200C); a raw ADC delta has none -- it could
// plausibly be tens or thousands depending on gain and resolution
// settings. These values are arbitrary compile-time stand-ins only. To
// get real numbers: capture a baseline with 'B' at room temp, then use
// 'X' against both an idle scene and a known-hot scene, read the
// reported raw-minus-baseline deltas, and use those.
static const float MATRIX_RAW_DELTA_MIN = 0.0f;      // PLACEHOLDER, no physical grounding
static const float MATRIX_RAW_DELTA_MAX = 1000.0f;   // PLACEHOLDER, no physical grounding
static const float CAPTURE_TRIGGER_RAW_DELTA = 300.0f; // PLACEHOLDER, no physical grounding

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
// MCP is polled only during Standby/TubeGap, ~20ms cadence -- see
// keyenceResultPending's comment for why that matters now that Keyence
// Result is slated to live on this chip.
// =====================================================================
Adafruit_MCP23X17 mcp;
static const uint8_t MCP_I2C_ADDR = 0x20;
static const uint32_t MCP_POLL_INTERVAL_MS = 20;

// CONFIRMED (V4.15.19) -- "Hardwired Done - Final" per field report,
// replacing the old NORMAL_STOP/FAST_STOP/HORN/BEACON/READY/WARNING/
// ACKNOWLEDGE/RESET/AUTO/MACHINE_STOPPED/GLUE_READY scheme entirely (that
// was a placeholder guess from before any real MCP23017 wiring existed;
// none of those names survive in the real hardware). Adafruit_MCP23X17
// pin indices: GPA0-7 = 0-7, GPB0-7 = 8-15 -- the field report's own
// literal constants (1-7, 21-24) don't follow that convention, so these
// are corrected to match the GPA/GPB port+bit the report's comments name,
// which is the actual physical fact; the report's raw integers looked
// like an different, informal numbering (possibly its own connector/
// channel labels) rather than real Adafruit_MCP23X17 pin arguments.
// Double-check this correction against the board before trusting it blind.
namespace McpPin {
// Internal status LED (enclosure-mounted) -- a single RGB LED, one color
// channel lit at a time (not 3 simultaneous channels mixing to a blended
// color).
constexpr uint8_t ILED_R = 8;  // GPB0, OUTPUT
constexpr uint8_t ILED_G = 9;  // GPB1, OUTPUT
constexpr uint8_t ILED_B = 10; // GPB2, OUTPUT
// External status LED (operator-visible) -- single RGB LED (same one-
// channel-at-a-time convention as internal) PLUS a separate discrete
// yellow LED, so 4 selectable colors total, not 3.
constexpr uint8_t ELED_R = 12; // GPB4, OUTPUT
constexpr uint8_t ELED_G = 13; // GPB5, OUTPUT
constexpr uint8_t ELED_B = 14; // GPB6, OUTPUT
constexpr uint8_t ELED_Y = 15; // GPB7, OUTPUT (3.3V/470ohm) -- external yellow LED
// V4.15.20: the ESP32 is NOT the boss here -- it's an "Auto Triggered
// Sensor" reporting to a Siemens S7-315-2 PLC, which actually runs the
// bottomer. These 2 inputs + 2 outputs (plus ESP_OPTO_3, the 3rd status
// bit) are that link -- see the PlcComms namespace below for the encoding.
// INPUT_1/INPUT_2 carry the S7's 2 independent control flags (dedicated
// one bit each, not encoded -- ACKNOWLEDGE and MACHINE_RUNNING can be true
// or false in any combination, unlike PLC_STATUS's 4 mutually-exclusive
// states). Which physical input is which is my placeholder pairing below
// (PlcComms::readPlcControl()) -- confirm against the S7 program.
constexpr uint8_t INPUT_2 = 0; // GPA0, INPUT, Conn 1
constexpr uint8_t OPTO_2 = 1;  // GPA1, OUTPUT (24V/220ohm), Conn 3 -- PLC_STATUS bit 1
constexpr uint8_t INPUT_1 = 2; // GPA2, INPUT, Conn 2
constexpr uint8_t OPTO_1 = 3;  // GPA3, OUTPUT (24V/220ohm), Conn 4 -- PLC_STATUS bit 0
} // namespace McpPin

bool mcpOk = false;

// V4.15.22: the 9 real MCP outputs, addressable by index 0-8 (matches the
// '1'-'9' serial command order and, not by coincidence, ILED_R/G/B's own
// indices 0/1/2, so handleHmiButtonPress()'s SETUP/ALARM LOG/TREND FULL
// can drive the same array). Firmware tracks each output's own commanded
// state here rather than reading it back from the MCP23017 to compute a
// toggle (mcp.digitalRead() on an output pin reads the chip's GPIO
// register, which is not guaranteed to track the OLAT/output-latch value
// that was actually written -- field report "ILED_G only, no control"
// looks exactly like a toggle that reads back the same value every time
// and never advances). This makes toggling deterministic in firmware
// regardless of what the MCP or the physical LED reports back.
constexpr uint8_t kMcpOutputPins[9] = {McpPin::ILED_R, McpPin::ILED_G, McpPin::ILED_B,
                                        McpPin::ELED_R, McpPin::ELED_G, McpPin::ELED_B,
                                        McpPin::ELED_Y, McpPin::OPTO_1, McpPin::OPTO_2};
bool mcpOutputState[9] = {false, false, false, false, false, false, false, false, false};

void toggleMcpOutput(uint8_t idx) {
  if (!mcpOk || idx >= 9) return;
  mcpOutputState[idx] = !mcpOutputState[idx];
  mcp.digitalWrite(kMcpOutputPins[idx], mcpOutputState[idx]);
}

// REMOVED (V4.15.40): advanceLedCycle() used to share one round-robin LED
// across SETUP/ALARM LOG/TREND FULL, lighting whichever channel came next
// on each press -- never a real per-button indicator, just an activity
// animation (useful at the time for spotting V4.15.23's phantom presses,
// since long since fixed). Replaced by setButtonStatusLed(), defined
// further down alongside the other HMI button state (needs
// NS12::BUTTON_COUNT, not yet in scope this early in the file) -- see
// that function's comment.

// =====================================================================
// Encoder -- ZATOR LMZ02, single-channel pulse train, no direction.
// Uses the ESP32 PCNT peripheral. 16-bit HW counter is drained into a
// 64-bit running total every service() call, well inside its wrap period
// at any plausible pulse rate for this line.
// =====================================================================
class EncoderTracker {
public:
  void begin(uint8_t pulseGpio) {
    // PULLDOWN: same reasoning as PRESENCE_SENSOR_PIN below. CONFIRMED
    // (V4.15.19) this pin now carries a real 24V field signal through a
    // 10K/1.5K divider to 3.2V -- that divider network dominates the
    // ESP32's own weak (~45k) internal pull, so this is effectively a
    // no-op now rather than the noise-guard it was for a floating pin;
    // left in place since it's harmless either way. pcnt_unit_config()
    // doesn't set a pull resistor on its own. The 100ns glitch filter
    // below only rejects very short noise, not sustained floating-pin
    // toggling. Low operational impact either way
    // (encoder.total() is re-zeroed via tubeStartEncoderCount at the start
    // of every real tube cycle) but cheap to make consistent.
    pinMode(pulseGpio, INPUT_PULLDOWN);
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

// PLACEHOLDER (Action Item 3): distance from the presence sensor to the
// MLX90640 station, used for forward projection.
static float PRESENCE_TO_MLX_DISTANCE_MM = 100.0f;

// =====================================================================
// HotMelt Start/End Position (V4.15.8) -- replaces the old single fixed
// PRESENCE_TO_KEYENCE_DISTANCE_MM with two operator-entered positions:
//   - Start Position: distance from the tube's LEADING edge to where the
//     glue bead should start. Known immediately at the leading edge (same
//     mechanism the old single distance already used), so this fires from
//     the very first tube.
//   - End Position: distance from the tube's TRAILING edge to where the
//     glue bead should end. The trailing edge isn't knowable until a full
//     tube has passed the presence sensor and its length has been
//     measured -- see tubeLengthLearned/learnedTubeLengthMm below. Per
//     spec: the first tube gets no End-position check; from the second
//     tube on, the previous tube's measured length is used to project
//     where the current tube's trailing edge will be.
//
// Both values are meant to be read from the HMI via RM (operator types
// them into numeric input objects on the touchscreen) -- see
// serviceHmiInputPolling() below. RM currently gets zero response from
// this PT (see the NS12 field-report comment on why), so these start at
// PLACEHOLDER fallback values and stay there, with
// hotMeltPositionsFromHmi staying false, until that's resolved.
static float hotMeltStartPositionMm = 150.0f; // PLACEHOLDER fallback
static float hotMeltEndPositionMm = 10.0f;    // PLACEHOLDER fallback
bool hotMeltPositionsFromHmi = false;

// PLACEHOLDER: HMI numeric-input word scale unconfirmed against the real
// CX-Designer project -- assumed here as plain integer mm (one $W count =
// 1mm). If those input objects actually use a decimal-shifted scale (e.g.
// x10 for 0.1mm resolution, matching the x10 convention already used
// elsewhere in this file's telemetry), change this to match.
constexpr float HMI_POSITION_MM_PER_COUNT = 1.0f;

// Tube length, learned from the previous tube's presence-sensor
// leading/trailing edge encoder distance (see handlePresenceEdge()).
// Needed to project where the CURRENT tube's trailing edge will be, since
// End Position above is referenced from that edge, not a fixed distance
// from the presence sensor.
float learnedTubeLengthMm = 0.0f;
bool tubeLengthLearned = false;
int64_t tubeEndEncoderCount = 0;

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

// RESTORED (V4.15.20): Keyence Result is a direct ESP32 GPIO again
// (ESP_INPUT_3/GPIO6) with a hardware interrupt, same shape as the
// original pre-V4.15.19 design -- see ESP_INPUT_3's own comment for why.
// This resolves the MCP-polling-latency tradeoff V4.15.19 introduced
// entirely, rather than needing a decision on it.
static const int KEYENCE_RESULT_ACTIVE_LEVEL = HIGH; // PLACEHOLDER, not confirmed

volatile bool keyenceResultPending = false;
volatile bool keyenceResultPass = false;
void IRAM_ATTR keyenceResultIsr() {
  keyenceResultPass = digitalRead(Pins::ESP_INPUT_3) == KEYENCE_RESULT_ACTIVE_LEVEL;
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
// every WM/RM/RB/WB byte). Turned back OFF (V4.15.22): RM/RB/WM/WB all
// confirmed 100% success across several diagnostics dumps (V4.15.17-20's
// deferred-read fix worked) -- the collision-hunting this was on for is
// done, and it was field-reported as burying other Serial output (HMI
// button-press/LED-toggle messages) under constant TX-line spam. Flip
// back to 1 if NS12 traffic needs this level of visibility again. The RM
// parse-failure dump below is unconditional and separate from this, since
// that specific failure needs visibility regardless of this flag.
#define NS12_DEBUG_RAW_RX 0

namespace NS12 {
constexpr uint8_t ESC = 0x1B;
constexpr long BAUD = 38400; // CHANGED (V4.15.42) -- see FIELD UPDATE above
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

// CORRECTED (V4.15.25): moved off $W0. A real CX-Designer Address View
// screenshot showed "2(P) $W0(W)" at the top of the Hot Melt Monitor
// screen -- a screen-switch object triggered by writing to $W0. Telemetry
// had been writing arbitrary heartbeat/FPS/status values into that exact
// word every TELEMETRY_WRITE_INTERVAL_MS since this project's first
// version, continuously feeding garbage into what the PT treats as a
// page-navigation command. This is a strong unifying explanation for
// multiple previously-unexplained symptoms (phantom HMI button presses
// that didn't originate from a touch; unexpected matrix content appearing
// on 'R') -- constant bogus page-switch attempts could plausibly corrupt
// the PT's Memory Link state in ways that produce stale or wrong replies
// for extended periods, not just visible screen flicker. Moved to $W100,
// inside the confirmed-free $W13-$W499 range from the real Symbol Table
// (V4.15.11) -- still not bench-verified against every object on this
// specific screen, but clear of every address this project has actually
// seen used ($W0-$W5, $HW0, $W10, $W500-$W505, $W700+ matrix, $W828/829
// band, $W830 the other screen-switch this screenshot showed).
constexpr uint16_t TELEMETRY_BASE_ADDR = 100;
constexpr uint16_t TELEMETRY_WORD_COUNT = 9;
constexpr uint32_t TELEMETRY_WRITE_INTERVAL_MS = 250;

// Low-rate read used only to keep the auto-fallback's RM success-rate
// stats alive (see checkDisplayAutoFallback()) -- without some RM traffic
// those stats never move and the fallback can never trigger. Mirrors the
// bench-tested file's $W10 "operator test input" convention; repoint if
// the CX-Designer project already uses $W10 for something else.
//
// DISABLED BY DEFAULT (field report, V4.15.3 run): once the WM-write-vs-
// pending-read overlap was eliminated (see NS12Manager::service()'s
// !readPending gating), RM went from 100% parse failures to 117/117
// clean timeouts -- zero responses, not malformed ones. This PT does not
// appear to answer RM requests in this configuration at all. RM isn't
// load-bearing for the live 16x8 path (only the currently-OFF
// experimental 32x24 mode's auto-fallback depends on it), so polling for
// a read that always times out was pure cost: every attempt guarantees a
// 250ms wait during which telemetry/matrix WM writes are deferred, for a
// safety net that never actually monitors anything live right now. Set
// to 1 to re-enable -- required again if ENABLE_EXPERIMENTAL_32x24 is
// ever turned on, since its auto-fallback has no data without this.
// Renamed from NS12_ENABLE_RM_TEST_READ (V4.15.8): this poll now has a
// real purpose -- reading the operator-entered HotMelt Start/End Position
// values back from the HMI (see serviceHmiInputPolling() and
// hotMeltStartPositionMm/hotMeltEndPositionMm below) -- not just
// exercising the link for the (currently off) 32x24 auto-fallback.
// V4.15.10 FIELD TEST: turned ON (was 0) to test the RM_WORD_ADDRESS_OFFSET
// hypothesis (see that constant in the NS12 namespace below) -- RM had
// gotten zero response on this PT (117/117 clean timeouts) with the
// "Response=OFF" theory ruled out (screenshot confirmed Response is ON).
// CONFIRMED (V4.15.11): real hardware diagnostics came back
// "112 / 108 / 0 / 0 / 4" (attempts/success/writeFail/timeout/parseErr) --
// 0 timeouts, 96% success. The offset theory was correct. 4/112 parse
// errors remain unexplained -- not investigated yet, doesn't block normal
// operation since consumeReadWord() just skips a failed read and tries
// again next poll cycle.
#define NS12_ENABLE_RM_POLLING 1

// CONFIRMED (V4.15.11) on real hardware: diagnostics showed
// "HotMelt Start/End position (mm) : 33.0 / 34.0 (from HMI)" -- real
// operator-entered values, not the PLACEHOLDER fallback (150.0/10.0) --
// once RM_WORD_ADDRESS_OFFSET actually got RM responding. Cross-checked
// against the real CX-Designer symbol table (Symbol Table screenshot,
// 510_HotMel_20260902_1 project): no other object is bound to $W11 or
// $W12 (used words nearby: $W0-$W5, $HW0, $W10, $W500-$W505, $W700+ for
// the matrix), so no conflict either. No longer a placeholder guess.
constexpr uint16_t HOTMELT_START_POSITION_ADDR = 11;
constexpr uint16_t HOTMELT_END_POSITION_ADDR = 12;
constexpr uint32_t RM_POLL_INTERVAL_MS = 1000; // operator input changes rarely -- no need to poll fast

// Hard protocol ceiling, not a tuning knob: the WM/RM wire format's LL
// field is exactly 2 decimal digits (*L(2 dec) in the protocol reference),
// so no single WM command can legitimately carry more than 99 words --
// sending more doesn't fail loudly, it silently desyncs the frame (see the
// V4.15.6 field-report comment on the NS12 namespace for exactly how that
// broke the default 16x8 matrix push). Also sizes the sendWM() stack
// buffer; the largest real chunk today is 24 words (experimental 32x24
// columns), so 99 leaves comfortable margin without being the bug again.
constexpr uint16_t MAX_WM_WORDS = 99;

// Same LL-field ceiling applied to WB (bit write). Only ever used for 5
// lamp bits at once (BUTTON_COUNT) in this file, so this is a generous
// margin, not a tight fit.
constexpr uint16_t MAX_WB_BITS = 99;

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

// PLACEHOLDER, UNVERIFIED HYPOTHESIS (V4.15.9): CX-Designer's own Comm.
// Setting screen (Serial Port A / Memory Link) shows a real, screenshot-
// confirmed field: "Start Communication $W: 16384". This project had
// never looked at that field before. If it defines an offset between a
// $Wn object's CX-Designer label and its actual Memory Link wire address,
// it would explain the exact asymmetry seen throughout this project's
// history: WM writes "succeed" at the transport level even when aimed at
// the wrong location (nothing surfaces as an error from writing
// somewhere unintended), while RM appears to strictly validate the
// address and simply never respond outside it (0/117 responses, zero
// exceptions, never a malformed reply once the write/read overlap bug
// was fixed). That fits better than the "Response=OFF" theory this
// replaces -- Response is confirmed ON in the same screenshot.
//
// COULD NOT VERIFY against the official Host Connection Manual (Cat. No.
// V085-E1-07) -- WebFetch to every candidate manual/documentation host
// was blocked by this sandbox's network egress policy. This was reasoned
// from the available evidence, not confirmed against primary
// documentation, at the time it was written.
//
// CONFIRMED (V4.15.11) by an actual hardware test, which is a better
// answer than the manual would have been anyway: real diagnostics with
// this offset applied and NS12_ENABLE_RM_POLLING=1 came back
// "112 / 108 / 0 / 0 / 4" (attempts/success/writeFail/timeout/parseErr) --
// 0 timeouts, 96% success, versus 117/117 clean timeouts before. The
// offset theory is correct for this PT/project. Whether WM also needs it
// has not been tested (see the RM-only rationale below) and should stay
// that way unless WM traffic itself shows a problem.
//
// RM-only, deliberately NOT applied to WM (see sendWM): WM writes already
// get transport-level acks with plain (unoffset) addresses -- there is no
// evidence WM needs this offset, and applying an unverified offset to the
// one path that already "works" would risk breaking known-good telemetry/
// matrix traffic just to test a read-side hypothesis. If this offset turns
// out to be real and WM also needs it, that's a separate, deliberate change
// once RM confirms the theory -- not bundled in here.
//
// CONFIRMED (V4.15.11) on real hardware: with this set to 16384 and
// NS12_ENABLE_RM_POLLING=1, RM went from 117/117 clean timeouts to
// 108/112 successful reads (0 timeouts). $W11 (HOTMELT_START_POSITION_ADDR)
// really is at wire address 11+16384=16395, not plain 11. Applied once,
// centrally, here -- every $Wn address elsewhere in this file stays
// written as its plain CX-Designer label; only the wire-encoded value
// changes. Do not revert to 0 -- that was the pre-fix, all-timeouts state.
constexpr uint16_t RM_WORD_ADDRESS_OFFSET = 16384;

// PLACEHOLDER, UNVERIFIED HYPOTHESIS (V4.15.12): CX-Designer's Comm.
// Setting screen (same screenshot RM_WORD_ADDRESS_OFFSET came from) also
// lists "Start Communication $B: 16384" -- $B (bit) memory has its own
// offset field, separate from $W's, that happens to carry the same value
// in this project.
//
// "CONFIRMED for RB" (V4.15.13/15) turned out to be a false confirmation:
// real RB reads with this offset applied got exact address+count matches
// against real hardware, which was read at the time as proof the wire
// address really is offset for reads -- but a structurally valid,
// checksummed response only proves the address is IN RANGE, not that
// it's the CORRECT one. The real test (V4.15.34, 'N' command) came later:
// writing 1 to $B30 via sendWB() (plain address) and reading it back at
// the PLAIN address gave "80" (bit7=1, checksum valid); reading the exact
// same bit at the OFFSET address, at the exact same instant, gave 0x00.
// Bits don't need this offset at all -- only words (RM, via
// RM_WORD_ADDRESS_OFFSET above) do. requestRB()/consumeReadBit() no
// longer apply it (V4.15.35). Left declared, unused, for the historical
// record and because the comments above and below still refer to it by
// name.
//
// RENAMED to RB_BIT_ADDRESS_OFFSET (V4.15.16) and REMOVED from WB: V4.15.12
// applied this to both RB and WB on the reasoning that WB was untested so
// there was nothing already-working to protect (unlike WM, deliberately
// left unoffset for exactly that reason). WB has used the plain address
// (matching WM) ever since, which V4.15.34/35 confirmed was the correct
// call all along -- it just turned out RB needed the same plain address
// too, not the offset.
constexpr uint16_t RB_BIT_ADDRESS_OFFSET = 16384; // no longer applied -- see comment above

// HMI push-button inputs, confirmed from the real CX-Designer Symbol Table
// (project 510_HotMel_20260902_1, I/O Comments "SETUP Button" / "ALARM LOG
// Button" / "TREND FULL Button" / "TEST Button" / "DIAG Button").
//
// CORRECTED (V4.15.14): originally assumed read-only (touch panel owns
// setting/clearing its own bit). Real hardware showed all 5 stuck
// permanently at 1 after a single tap each, never flickering -- exactly
// what a "momentary, host-clear" CX-Designer bit switch does: the panel
// SETS the bit on press and relies on the connected device to clear it
// back to 0, precisely so a command isn't missed under slow host polling.
//
// CHANGED (V4.15.38): the switches are now configured as native panel-
// side Alternate (self-toggling) objects, not Momentary -- see
// serviceHmiButtonPolling()'s comment. ESP32 no longer writes back to
// these addresses at all; the bit is purely read, and its value IS each
// switch's real, live, current state, owned entirely by the panel.
constexpr uint16_t BUTTON_SETUP_ADDR = 30;
constexpr uint16_t BUTTON_ALARM_LOG_ADDR = 31;
constexpr uint16_t BUTTON_TREND_FULL_ADDR = 32;
constexpr uint16_t BUTTON_TEST_ADDR = 33;
constexpr uint16_t BUTTON_DIAG_ADDR = 34;
constexpr uint8_t BUTTON_COUNT = 5;

// PLACEHOLDER (V4.15.12) -- no "Button Lamp" objects exist in the real
// Symbol Table yet, so these addresses are picked, not confirmed: the next
// free $B block after the buttons themselves ($B30-$B34) and well clear of
// the next known-used bit further down the table ($B870). Confirm/correct
// against the real CX-Designer project once lamp objects are added there,
// the same way HOTMELT_START/END_POSITION_ADDR went from placeholder to
// confirmed in V4.15.11. Contiguous and in the same SETUP/ALARM LOG/
// TREND FULL/TEST/DIAG order as the buttons above so sendWB() can push all
// 5 lamp states in one write.
//
// UNUSED (V4.15.38): setActiveLamp() removed -- the panel's own native
// Alternate switch visual is now the sole status indicator, and it needs
// no host write at all. Left declared for reference (still an unconfirmed
// placeholder block, per the note above), not because anything writes to
// them.
constexpr uint16_t LAMP_SETUP_ADDR = 40;
constexpr uint16_t LAMP_ALARM_LOG_ADDR = 41;
constexpr uint16_t LAMP_TREND_FULL_ADDR = 42;
constexpr uint16_t LAMP_TEST_ADDR = 43;
constexpr uint16_t LAMP_DIAG_ADDR = 44;

// CORRECTED (V4.15.15): originally 200ms ("snappier than position
// polling"). Real hardware showed this badly oversubscribes the shared
// link -- RM (position reads) crashed from ~91-96% success to 10/52
// (~19%) once 200ms button polling was added, with RB itself only 58/216
// (~27%). At the time this pointed at genuine wire-level corruption (RB
// failing that badly despite being the dominant request type isn't just
// "starved of turns"), not merely a scheduling issue. Bumped to 750ms.
//
// LOWERED (V4.15.41): with 750ms x 5 buttons = 3.75s worst-case round-
// robin, field report was "still takes 3-5 sec to react" even after
// V4.15.37 removed the debounce that used to double it. The V4.15.15
// failure predates markTxBusy() (V4.15.17, non-blocking TX-drain tracking
// that ended the write/read collisions this link used to suffer) and the
// checksum/offset fixes (V4.15.31/35) -- all of which independently
// confirmed 100% RM/RB success at much higher request rates since (e.g.
// the V4.15.29 burst-probe mode polls all 5 buttons back-to-back, far
// faster than even the original 200ms, with zero anomalies). Deliberately
// NOT returning to exactly 200ms -- that's the specific value with a
// documented failure on this hardware -- 250ms instead, as a middle
// ground to verify against real RM/RB success rates before going lower.
constexpr uint32_t BUTTON_POLL_INTERVAL_MS = 250;
} // namespace NS12

class NS12Manager {
public:
  void begin() {
    Serial2.setTxBufferSize(1024);
    Serial2.begin(NS12::BAUD, SERIAL_8N1, Pins::NS12_RX, Pins::NS12_TX);
    lastTelemetryMs = millis();
  }

  // WM: write `count` words starting at `startAddr`. count > 99 is a
  // protocol violation (the wire LL field is exactly 2 decimal digits),
  // not a soft limit -- clamped defensively and counted as a failure so a
  // future regression is visible in diagnostics instead of silently
  // desyncing the frame the way the pre-V4.15.6 count-field truncation did.
  void sendWM(uint16_t startAddr, const uint16_t *data, uint16_t count) {
    if (count > NS12::MAX_WM_WORDS) {
      count = NS12::MAX_WM_WORDS;
      wmOversizedCount++;
    }
    char frame[4 + 4 + 2 + NS12::MAX_WM_WORDS * 5 + 1];
    size_t n = 0;
    frame[n++] = (char)NS12::ESC;
    frame[n++] = 'W';
    frame[n++] = 'M';
    frame[n++] = '0';
    // WM intentionally uses the plain, unoffset address -- see the
    // RM_WORD_ADDRESS_OFFSET comment in the NS12 namespace for why this
    // stays decoupled from the RM-side offset test.
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
    markTxBusy(n);
  }

  // RM: request `count` words (max 32) starting at `startAddr`. Sends the
  // request only and returns immediately -- non-blocking, unlike the prior
  // synthesis's spin-wait version, which could stall the whole loop for up
  // to RM_READ_TIMEOUT_MS at a time this file now also drives hard-real-
  // time Keyence pulse timing and encoder tracking. Call service() every
  // loop() iteration to drive the response state machine.
  bool requestRM(uint16_t startAddr, uint8_t count) {
    if (readPending || count == 0 || count > 32) return false;
    // See markTxBusy()'s comment: defer starting a read until any recent
    // WM/WB send is estimated to have actually finished draining off the
    // wire, not just been handed to Serial2.write().
    if ((int32_t)(millis() - txBusyUntilMs) < 0) return false;

    // See NS12::RM_WORD_ADDRESS_OFFSET -- currently 16384 under field test.
    // wireAddr (not the caller's plain startAddr) is what's actually sent
    // AND what the response is validated against below, since the PT would
    // echo back whatever address it actually processed.
    uint16_t wireAddr = (uint16_t)(startAddr + NS12::RM_WORD_ADDRESS_OFFSET);

    char frame[16];
    size_t n = 0;
    frame[n++] = (char)NS12::ESC;
    frame[n++] = 'R';
    frame[n++] = 'M';
    frame[n++] = '0';
    n += writeHex4(&frame[n], wireAddr);
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
    pendingCmdType = 'M';
    readSentMs = millis();
    readLineUsed = 0;
    expectedAddr = wireAddr; // matched against the response's own address field, which is the wire address
    expectedCount = count;
    return true;
  }

  // RB: request `count` bits (max 32) starting at `startAddr`. Same non-
  // blocking shape as requestRM() -- shares the single read-pending state
  // machine (only one request, word or bit, can ever be in flight), routed
  // by pendingCmdType so the response validates against 'B' instead of 'M'
  // and completion counts into the separate rb* counters. Introduced
  // V4.15.12 for the HMI push-buttons ($B30-$B34).
  //
  // CORRECTED (V4.15.35): no offset -- uses the plain address, matching
  // sendWB(). RB_BIT_ADDRESS_OFFSET (+16384) was applied here since
  // V4.15.12 by analogy with RM's confirmed $W offset, but was never
  // independently verified for bits; every "confirmation" it had turned
  // out to be either structural (any valid address parses successfully,
  // which proves nothing about whether it's the RIGHT address) or
  // coincidental (V4.15.31's checksum discovery). The V4.15.34 'N' test
  // proved it directly: writing 1 to $B30 (plain address, via sendWB())
  // then reading the PLAIN address back gave "80" (bit7=1, checksum
  // valid) -- while the OFFSET address read 0x00 for the exact same
  // write, in the exact same test. Bits don't need the offset; only words
  // (RM) do.
  bool requestRB(uint16_t startAddr, uint8_t count) {
    if (readPending || count == 0 || count > 32) return false;
    if ((int32_t)(millis() - txBusyUntilMs) < 0) return false; // see markTxBusy()

    uint16_t wireAddr = startAddr;

    char frame[16];
    size_t n = 0;
    frame[n++] = (char)NS12::ESC;
    frame[n++] = 'R';
    frame[n++] = 'B';
    frame[n++] = '0';
    n += writeHex4(&frame[n], wireAddr);
    n += writeDecimal2(&frame[n], count);
    frame[n++] = '\r';

#if NS12_DEBUG_RAW_RX
    Serial.print(F("[NS12] TX RB: "));
    for (size_t i = 0; i < n; i++) printRawByte((uint8_t)frame[i]);
    Serial.println();
#endif

    rbAttempts++;
    clearRxBuffer();
    size_t sent = Serial2.write(reinterpret_cast<uint8_t *>(frame), n);
    if (sent != n) {
      rbWriteFailures++;
      return false;
    }
    Serial2.flush();

    readPending = true;
    pendingCmdType = 'B';
    readSentMs = millis();
    readLineUsed = 0;
    expectedAddr = wireAddr;
    expectedCount = count;
    return true;
  }

  // WB: write `count` bits starting at `startAddr`. Fire-and-forget, same
  // shape as sendWM() -- no response expected.
  //
  // CORRECTED (V4.15.16): originally applied RB_BIT_ADDRESS_OFFSET here
  // too (V4.15.12), reasoning that WB was untested so there was nothing
  // already-working to protect by leaving it unoffset -- unlike sendWM.
  // That missed the precedent sitting right next to it: RM needs the $W
  // offset but WM does not. Real hardware confirmed the same asymmetry for
  // bits -- a confirmed CX-Designer button (Write $B33 / Display Address1
  // $B43, "TEST", Momentary) showed zero visible lamp change while WB was
  // writing to the offset address (0x4028). WB now writes the plain
  // address, matching sendWM.
  //
  // CORRECTED (V4.15.32): *D was being sent as one literal ASCII '0'/'1'
  // character per bit, comma-separated -- mimicking WM's per-value comma
  // format, never checked against the manual. The manual documents WB's
  // *D as PACKED: 4 bits per hex digit, MSB-first ("fills data in
  // descending order starting with first digit"), no commas -- the same
  // MSB-first convention RB's own fix (V4.15.31) proved correct for reads.
  // Proof this mattered: the V4.15.32 'J' self-test (writing count=1,
  // bits[0]=true with the OLD encoding -- literal "1") read back as 0x00
  // via a checksum-validated RB, on OUR OWN write with no touchscreen
  // involved at all -- meaning a "true" bit was never actually reaching
  // the PT's real memory in a form RB could see. A "false" bit happened to
  // encode identically under both schemes ("0" either way), which is
  // exactly why the host-clear-to-0 writes never showed a symptom while
  // every write of an ON bit (lamps, this test) was silently wrong. Now
  // packs bits[] into ceil(count/4) hex digits, bit k of digit d occupying
  // position (3-k) within that nibble (MSB-first, matching the RB
  // convention and the manual's own worked example).
  void sendWB(uint16_t startAddr, const bool *bits, uint8_t count) {
    if (count > NS12::MAX_WB_BITS) {
      count = NS12::MAX_WB_BITS;
      wbOversizedCount++;
    }
    static const char kHexDigits[] = "0123456789ABCDEF";
    uint8_t hexDigitCount = (uint8_t)((count + 3) / 4);
    char frame[4 + 4 + 2 + (NS12::MAX_WB_BITS + 3) / 4 + 1];
    size_t n = 0;
    frame[n++] = (char)NS12::ESC;
    frame[n++] = 'W';
    frame[n++] = 'B';
    frame[n++] = '0';
    n += writeHex4(&frame[n], startAddr);
    n += writeDecimal2(&frame[n], count);
    for (uint8_t d = 0; d < hexDigitCount; d++) {
      uint8_t nibble = 0;
      for (uint8_t k = 0; k < 4; k++) {
        uint8_t bitIndex = (uint8_t)(d * 4 + k);
        if (bitIndex < count && bits[bitIndex]) {
          nibble = (uint8_t)(nibble | (1 << (3 - k)));
        }
      }
      frame[n++] = kHexDigits[nibble];
    }
    frame[n++] = '\r';

    // CHANGED (V4.15.39): re-gated behind NS12_DEBUG_RAW_RX -- was made
    // unconditional in V4.15.36 to check the multi-bit lamp-write
    // encoding, which is now confirmed correct. WB fires on every button
    // press/clear and telemetry write, so this was clutter for normal
    // operation.
#if NS12_DEBUG_RAW_RX
    Serial.print(F("[NS12-WB-TX] addr=0x"));
    Serial.print(startAddr, HEX);
    Serial.print(F(" count="));
    Serial.print(count);
    Serial.print(F(" raw ("));
    Serial.print(n);
    Serial.print(F(" bytes): "));
    for (size_t i = 0; i < n; i++) printRawByteAlways((uint8_t)frame[i]);
    Serial.println();
#endif

    wbAttempts++;
    size_t sent = Serial2.write(reinterpret_cast<uint8_t *>(frame), n);
    if (sent != n) {
      wbFailures++;
    }
    // No blocking flush -- same rationale as sendWM(): never stall the loop.
    markTxBusy(n);
  }

  // Must be called every loop() iteration. Drives the periodic telemetry
  // push and the non-blocking read state machine. Never blocks. RM
  // requests themselves are now driven externally by
  // serviceHmiInputPolling() (see below), not from inside here -- this
  // just needs to keep pumping pollPendingRead() regardless of who called
  // requestRM().
  void service() {
    uint32_t now = millis();

    // Gated on !readPending: RM_READ_TIMEOUT_MS and
    // TELEMETRY_WRITE_INTERVAL_MS are both 250-300ms (and the HMI input
    // poll adds a third, slower request source), so a telemetry WM write
    // could otherwise fire in the middle of an in-flight RM read on this
    // shared half-visible UART. pollPendingRead() only checks that a byte
    // stream starts at an ESC, not where it actually came from -- see the
    // field-report comment on the NS12 namespace for why that matters
    // (every captured "RM response" so far had a WM-shaped header, before
    // this gating existed). This delays telemetry by at most one
    // RM_READ_TIMEOUT_MS window, not lost. Harmless no-op with RM polling
    // disabled (readPending then never becomes true).
    if (!readPending && now - lastTelemetryMs >= NS12::TELEMETRY_WRITE_INTERVAL_MS) {
      lastTelemetryMs = now;
      sendWM(NS12::TELEMETRY_BASE_ADDR, telemetry, NS12::TELEMETRY_WORD_COUNT);
    }

    if (readPending) {
      pollPendingRead(now);
    }
  }

  // Pop semantics: returns true (once) for the most recently completed
  // successful RM read, then clears until the next one lands. Written by
  // parseReadResponse() on success; consumed by serviceHmiInputPolling().
  // Gated on lastReadKind == Word so this never accidentally consumes a
  // completed RB (bit) read meant for consumeReadBit() instead -- both
  // share the same single-slot "last completed read" state since only one
  // request (word or bit) is ever in flight at a time. addrOut is
  // translated back to the caller's plain $Wn label (WIRE address minus
  // NS12::RM_WORD_ADDRESS_OFFSET) -- the offset, if any, stays entirely
  // internal to this class; external code never has to think about it, in
  // either direction.
  bool consumeReadWord(uint16_t &addrOut, uint16_t &valueOut) {
    if (!lastReadValid || lastReadKind != ReadKind::Word) return false;
    addrOut = (uint16_t)(lastReadAddrValue - NS12::RM_WORD_ADDRESS_OFFSET);
    valueOut = lastReadWordValue;
    lastReadValid = false;
    return true;
  }

  // Bit-read counterpart to consumeReadWord() -- see that method's comment
  // for the shared-slot/lastReadKind rationale. Introduced V4.15.12 for the
  // HMI push-buttons.
  //
  // SUPERSEDED (V4.15.15's "`& 1`" LSB theory): that fix was derived from
  // two samples ("004B"/"0037") that were WRONGLY assumed to be the real
  // data -- they were actually [checksum-included data], and the LSB they
  // shared was coincidental to the checksum arithmetic, not a real button
  // bit. See parseReadResponse()'s V4.15.31 header comment for the full
  // checksum-math proof (verified against all 5 real HMI buttons) that the
  // PT always appends a trailing 2-hex-digit SUM this code had been
  // misreading as part of *D. Now that parseReadResponse() strips SUM
  // before storing lastReadWordValue, that value is genuinely just *D's
  // real byte content (0x00-0xFF for a 1-bit request). Per the manual's
  // own worked example ("$B10 11 12 13 14 15 * * -> 10101100 -> AC"), the
  // FIRST (and, for a 1-bit request, only) requested bit lands at bit 7
  // (MSB) of that byte, not bit 0 -- bits 6-0 are documented padding
  // ("fills any of the last 8 bits that does not actually have a valid
  // read-out data with 0").
  //
  // CORRECTED (V4.15.35): no offset in addrOut either, matching
  // requestRB()'s now-plain wireAddr -- see that method's comment for the
  // real-hardware proof (writing 1 to $B30 read back as bit7=1 at the
  // plain address, 0x00 at the offset address, same write, same instant).
  bool consumeReadBit(uint16_t &addrOut, bool &valueOut, uint16_t &rawValueOut) {
    if (!lastReadValid || lastReadKind != ReadKind::Bit) return false;
    addrOut = lastReadAddrValue;
    rawValueOut = lastReadWordValue;
    valueOut = (lastReadWordValue & 0x80) != 0;
    lastReadValid = false;
    return true;
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
  uint32_t wmOversizedCountValue() const { return wmOversizedCount; }
  void resetRmStats() { rmAttempts = 0; rmSuccesses = 0; }

  uint32_t rbAttemptCount() const { return rbAttempts; }
  uint32_t rbSuccessCount() const { return rbSuccesses; }
  uint32_t rbWriteFailureCount() const { return rbWriteFailures; }
  uint32_t rbTimeoutCount() const { return rbTimeouts; }
  uint32_t rbParseErrorCount() const { return rbParseErrors; }
  uint32_t wbAttemptCount() const { return wbAttempts; }
  uint32_t wbFailureCount() const { return wbFailures; }

  // Exposed so the matrix-push free functions (serviceDisplayThrottle(),
  // serviceMatrixPacing()) can defer a WM write the same way service()
  // defers telemetry -- see the field-report comment on the NS12
  // namespace for why a WM write during a pending RM read is suspect.
  bool isReadPending() const { return readPending; }

private:
  uint16_t telemetry[NS12::TELEMETRY_WORD_COUNT] = {};
  uint32_t lastTelemetryMs = 0;

  bool readPending = false;
  char pendingCmdType = 'M'; // 'M' (RM/word) or 'B' (RB/bit) -- which request is in flight
  uint32_t readSentMs = 0;
  uint16_t expectedAddr = 0;
  uint8_t expectedCount = 0;
  char readLineBuffer[40] = {};
  size_t readLineUsed = 0;

  // Set by parseReadResponse() on a successful parse, popped by
  // consumeReadWord() or consumeReadBit() depending on lastReadKind -- see
  // those methods' comments.
  enum class ReadKind : uint8_t { Word, Bit };
  bool lastReadValid = false;
  ReadKind lastReadKind = ReadKind::Word;
  uint16_t lastReadAddrValue = 0;
  uint16_t lastReadWordValue = 0;

  uint32_t rmAttempts = 0;
  uint32_t rmSuccesses = 0;
  uint32_t rmWriteFailures = 0;
  uint32_t rmTimeouts = 0;
  uint32_t rmParseErrors = 0;
  uint32_t wmAttempts = 0;
  uint32_t wmFailures = 0;
  uint32_t wmOversizedCount = 0;

  uint32_t rbAttempts = 0;
  uint32_t rbSuccesses = 0;
  uint32_t rbWriteFailures = 0;
  uint32_t rbTimeouts = 0;
  uint32_t rbParseErrors = 0;
  uint32_t wbAttempts = 0;
  uint32_t wbFailures = 0;
  uint32_t wbOversizedCount = 0;

  // V4.15.17: estimated millis() timestamp when the most recent WM/WB send
  // will have actually finished draining off the wire (not just been
  // handed to Serial2.write(), which returns immediately -- see sendWM()'s
  // "no blocking flush" comment). requestRM()/requestRB() defer starting a
  // new read until this passes.
  uint32_t txBusyUntilMs = 0;

  void clearRxBuffer() {
    while (Serial2.available() > 0) Serial2.read();
  }

  // Called from sendWM()/sendWB() right after Serial2.write(). Root cause
  // this addresses: those writes are deliberately non-blocking (no
  // flush()), so their bytes can still be physically draining off the wire
  // when a read request starts a few loop() iterations later -- and this
  // link has always echoed/leaked its own transmitted bytes back onto RX
  // (established since before V4.15.4's write/read-overlap fix, and still
  // visible in V4.15.16 field data: "[ESC]WM001F" bleeding into a pending
  // RM read right after a burst of matrix column writes). Accumulates
  // rather than overwrites -- if a previous send's estimated drain time
  // hasn't passed yet, a new send's transmission time queues up behind it,
  // approximating how the actual UART TX FIFO drains sends in order.
  // Non-blocking by construction: this only changes when requestRM()/
  // requestRB() are willing to start, never stalls loop() itself, so it
  // cannot introduce the Keyence 100us-pulse jitter a real flush() would.
  void markTxBusy(size_t frameBytes) {
    uint32_t txMs = (uint32_t)((frameBytes * 10UL * 1000UL) / (uint32_t)NS12::BAUD) + 5UL;
    uint32_t now = millis();
    uint32_t baseline = ((int32_t)(txBusyUntilMs - now) > 0) ? txBusyUntilMs : now;
    txBusyUntilMs = baseline + txMs;
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
  // Non-static (V4.15.12) so it can check pendingCmdType to name the right
  // command letter in the diagnostic message below.
  void dumpRejectedLine(const char *line, size_t len) {
    Serial.print(F("[NS12] R"));
    Serial.print(pendingCmdType);
    Serial.print(F(" parse failed, raw response ("));
    Serial.print(len);
    Serial.print(F(" bytes): "));
    for (size_t i = 0; i < len; i++) {
      printRawByteAlways((uint8_t)line[i]);
    }
    Serial.println();
    // Flag explicitly rather than making the reader notice: a genuine reply
    // starts 'R' followed by the command letter we requested ('M' or 'B').
    // If it starts 'W' instead, this isn't a PT response at all -- it's
    // shaped like one of OUR OWN WM/WB writes, most likely a loopback/echo
    // or a write that fired while this read was still pending. See the
    // field-report comment on the NS12 namespace.
    if (len >= 3 && line[1] == 'W' && (line[2] == 'M' || line[2] == 'B')) {
      Serial.print(F("[NS12]   ^ starts 'W',"));
      Serial.print(line[2]);
      Serial.print(F(" -- looks like our own W"));
      Serial.print(line[2]);
      Serial.println(F(" traffic, not a genuine reply. Check for TX/RX "
                        "loopback or PT echo."));
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
        bool isBit = (pendingCmdType == 'B');
        if (parseReadResponse(readLineBuffer, readLineUsed)) {
          if (isBit) rbSuccesses++; else rmSuccesses++;
        } else {
          if (isBit) rbParseErrors++; else rmParseErrors++;
          dumpRejectedLine(readLineBuffer, readLineUsed);
        }
        readPending = false;
        return;
      }

      readLineBuffer[readLineUsed++] = ch;
    }

    if (!readPending) return; // completed above

    if (readLineUsed >= sizeof(readLineBuffer) - 1) {
      if (pendingCmdType == 'B') rbParseErrors++; else rmParseErrors++;
      readPending = false;
      return;
    }

    if (now - readSentMs > NS12::RM_READ_TIMEOUT_MS) {
      if (pendingCmdType == 'B') rbTimeouts++; else rmTimeouts++;
      readPending = false;
    }
  }

  // Response framing: ESC 'R' 'M'/'B' [maybe '0' echoed] AAAA(4-hex)
  // LL(2-dec) D,D,... SUM(2-hex) CR -- shared by both RM (word) and RB (bit)
  // reads, distinguished by pendingCmdType (set in requestRM()/requestRB()).
  // The '0' echo has not been independently confirmed on this PT, so both
  // candidate offsets are tried; whichever validates wins.
  //
  // V4.15.31: SUM added to this comment and this method after getting the
  // actual manual (Cat. No. V085-E1-07) text. It documents SUM as *always*
  // appended by the PT to every response ("Be sure that it is added when PT
  // is transmitting"), independent of whatever *S the HOST used in its own
  // request -- this code's *S='0' only omits OUR outgoing checksum, never
  // the PT's. This method had been treating those trailing 2 SUM chars as
  // part of *D the whole time. Proof, not guesswork: computing SUM (lower
  // byte of the sum of every byte from ESC through *D) for all 5 real RB
  // frames captured in the field --
  //   RB401F01[00]4B (ALARM LOG), RB402001[00]36 (TREND FULL),
  //   RB401E01[00]4A (SETUP), RB402101[00]37 (TEST), RB402201[00]38 (DIAG)
  // -- gives 4B/36/4A/37/38 in every single case, an exact match with what
  // this code had been misreading as "the button's value". The real *D in
  // all 5 was "00" (OFF) -- every "phantom press" (ALARM LOG/TEST reading
  // bit0=1) was this code checking a byte that was actually the checksum,
  // not the button. The manual also documents the true bit-packing (RB:
  // "Read PT memory ($B)"): *D holds ceil(count/8) bytes, and the FIRST
  // requested bit lands at bit 7 (MSB) of the first byte, not bit 0 --
  // worked example: "$B10 11 12 13 14 15 * * -> 1 0 1 0 1 1 0 0 -> AC"
  // reads $B10 into the leftmost (MSB) position. consumeReadBit() below
  // now checks bit 0x80, not bit 0x01.
  bool parseReadResponse(const char *response, size_t len) {
    if (len < 9 || (uint8_t)response[0] != NS12::ESC || response[1] != 'R' ||
        response[2] != pendingCmdType) {
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

      // V4.15.31: the last 2 characters of every response are the PT's own
      // SUM, not part of *D -- see this method's header comment for the
      // checksum proof. Reject (try the other offset, then fail) rather
      // than silently accept if it doesn't validate: this doubles as a
      // frame-integrity check that would have caught prior link-collision
      // garbage (e.g. WM/WB echo) structurally satisfying addr/count by
      // coincidence.
      size_t totalTailLen = len - headerLen;
      if (totalTailLen < 3 || totalTailLen - 2 >= 8) continue; // >=1 data char + 2 sum chars
      size_t dataLen = totalTailLen - 2;
      char dataText[8];
      memcpy(dataText, &response[headerLen], dataLen);
      dataText[dataLen] = '\0';
      char *comma = strchr(dataText, ',');
      if (comma) *comma = '\0';

      char sumText[3] = {response[headerLen + dataLen], response[headerLen + dataLen + 1], '\0'};
      uint8_t receivedSum = (uint8_t)strtoul(sumText, nullptr, 16);
      uint8_t computedSum = 0;
      for (size_t i = 0; i < headerLen + dataLen; i++) computedSum += (uint8_t)response[i];
      if (computedSum != receivedSum) continue;

      char *endPtr = nullptr;
      unsigned long parsedValue = strtoul(dataText, &endPtr, 16);
      if (endPtr == dataText) continue;

      lastReadAddrValue = addr;
      lastReadWordValue = (uint16_t)parsedValue;
      lastReadKind = (pendingCmdType == 'B') ? ReadKind::Bit : ReadKind::Word;
      lastReadValid = true;

      // CHANGED (V4.15.39): re-gated behind NS12_DEBUG_RAW_RX. This was
      // made unconditional in V4.15.28/31 to root-cause the checksum and
      // address-offset bugs on real hardware -- both are now fixed and
      // confirmed, so this firehose (one line per RM/RB read, several
      // times a second) is pure terminal clutter for normal operation.
      // Turn NS12_DEBUG_RAW_RX back on (below) if that evidence is ever
      // needed again. The older near-duplicate debug block this
      // superseded (V4.15.13) has been removed rather than gating both.
#if NS12_DEBUG_RAW_RX
      Serial.print(F("[NS12-FRAME] R"));
      Serial.print(pendingCmdType);
      Serial.print(F(" offset="));
      Serial.print(offset);
      Serial.print(F(" addrText=\""));
      Serial.print(addrText);
      Serial.print(F("\" countText=\""));
      Serial.print(countText);
      Serial.print(F("\" dataText=\""));
      Serial.print(dataText);
      Serial.print(F("\" sum=0x"));
      Serial.print(receivedSum, HEX);
      Serial.print(F(" (ok) raw ("));
      Serial.print(len);
      Serial.print(F(" bytes): "));
      for (size_t i = 0; i < len; i++) printRawByteAlways((uint8_t)response[i]);
      Serial.println();
#endif
      return true;
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
// Word Lamp raw-delta-to-palette mapping and matrix downsample. Despite
// the name (kept to minimize churn at call sites), this maps a raw-ADC-
// delta value (see MATRIX_RAW_DELTA_MIN/MAX above), not degrees C.
// =====================================================================
uint8_t tempToPaletteIndex(float rawDelta) {
  float t = (rawDelta - MATRIX_RAW_DELTA_MIN) / (MATRIX_RAW_DELTA_MAX - MATRIX_RAW_DELTA_MIN);
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

  // Always column-paced, including the default 16x8 mode -- NEVER a single
  // cols*rows-word burst. Root cause (field report): the NS12 wire protocol's
  // LL field is exactly 2 decimal digits (*L(2 dec) in the protocol
  // reference), so no single WM command can legitimately carry more than 99
  // words. The old "one 128-word burst for 16x8" path violated that, and
  // writeDecimal2()'s 2-byte tmp buffer silently truncated the count field
  // from "128" to "12" rather than erroring -- so the PT was being told to
  // expect 12 words while 128 followed, and (per the real captured bytes:
  // "WM002BC121,2,3,...") never rendered anything. Splitting into column
  // writes (rows=8 or 24 words each, both well under 99) fixes this at the
  // architecture level rather than patching the symptom. For 16x8 this
  // lands the band-max words at the same $828/$829 the old direct path
  // used (MATRIX_BASE_ADDR + cols*rows = 700+128 = 828), so no HMI-side
  // address change.
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

// Shared by the 'M' serial command and the HMI TEST button (V4.15.12) --
// factored out so both trigger the exact same diagnostic pattern instead of
// two copies drifting apart.
void pushTestPattern() {
  float testPattern[StripZone::COLS * StripZone::ROWS];
  for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
    testPattern[i] = MATRIX_RAW_DELTA_MIN + (MATRIX_RAW_DELTA_MAX - MATRIX_RAW_DELTA_MIN) *
                                                 ((float)i / (StripZone::COLS * StripZone::ROWS));
  }
  pushWordLampMatrix(testPattern);
  Serial.println(F("[DIAG] Test pattern pushed. Send 'R' to clear it."));
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

// Called every loop() iteration; no-op unless a paced matrix push is
// active. Handles both the default 16x8 mode and experimental 32x24 --
// pendingMatrix.cols/rows are set per-push in pushWordLampMatrix(), never
// hardcoded here.
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
// ARMED watches the frame's raw-delta max against CAPTURE_TRIGGER_RAW_DELTA ->
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
      if (maxT >= CAPTURE_TRIGGER_RAW_DELTA) {
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
  // CAPTURE_TRIGGER_RAW_DELTA, so a fully-glitched frame simply never
  // triggers rather than triggering on frame[0] regardless of its
  // validity (the previous version didn't check frame[0] at all).
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
        if (v >= CAPTURE_TRIGGER_RAW_DELTA) r.hotPixelCount++;
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
bool tubeKeyenceStartFired = false;
bool tubeKeyenceEndFired = false;
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

// REMOVED (V4.15.19): setMcpOutputs(normalStop, fastStop, horn, beacon,
// ready, warning) drove the old NORMAL_STOP/FAST_STOP/HORN/BEACON/READY/
// WARNING placeholder scheme, which no longer exists in the real hardware
// map (McpPin::OPTO_1/OPTO_2 are the only real MCP outputs left besides
// the status LEDs, and their function is not yet decided). enterFaultStop()
// now only transitions state -- TODO once opto output function is decided:
// drive whichever of OPTO_1/OPTO_2 (if any) should activate on a fault.
void enterFaultStop() {
  state = SystemState::FaultStop;
}

// =====================================================================
// Position projection: fires the MLX capture arm and the two Keyence
// checks (Start/End of glue bead) at the configured lead distances ahead
// of the tube's leading edge, using the encoder as the distance reference
// and the presence sensor as the anchor.
// =====================================================================
void serviceTubePositionTracking() {
  if (state != SystemState::InspectingTube) return;

  float travelledMm = (float)(encoder.total() - tubeStartEncoderCount) / ENCODER_COUNTS_PER_MM;

  if (!tubeMlxArmed && travelledMm >= PRESENCE_TO_MLX_DISTANCE_MM) {
    capture.rearm();
    tubeMlxArmed = true;
  }

  // Start-of-glue check: referenced from the LEADING edge, known
  // immediately -- fires from the very first tube.
  if (!tubeKeyenceStartFired && travelledMm >= hotMeltStartPositionMm) {
    keyenceTrigger.fire();
    tubeKeyenceStartFired = true;
  }

  // End-of-glue check: referenced from the TRAILING edge, which isn't
  // knowable until a full tube has passed the presence sensor and its
  // length has been measured (tubeLengthLearned, set in
  // handlePresenceEdge() below). Stays un-fired for the entire first
  // tube; from the second tube on, uses the previous tube's measured
  // length to compute where this tube's trailing edge will be.
  if (tubeLengthLearned && !tubeKeyenceEndFired) {
    float endTriggerMm = learnedTubeLengthMm - hotMeltEndPositionMm;
    if (travelledMm >= endTriggerMm) {
      // Both checks share one KeyenceTrigger with a single pending-pulse
      // slot -- fine as long as Start/End are far enough apart in travel
      // distance to not overlap the 500us pulse, true for any plausible
      // tube length and line speed.
      keyenceTrigger.fire();
      tubeKeyenceEndFired = true;
    }
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
      tubeKeyenceStartFired = false;
      tubeKeyenceEndFired = false;
    }
  } else {
    // Trailing edge -- tube has cleared the zone. Measure this tube's
    // length now (leading-to-trailing encoder distance) for the NEXT
    // tube's End-position projection -- see hotMeltEndPositionMm comment.
    if (state == SystemState::InspectingTube) {
      tubeEndEncoderCount = encoder.total();
      learnedTubeLengthMm =
          (float)(tubeEndEncoderCount - tubeStartEncoderCount) / ENCODER_COUNTS_PER_MM;
      bool hadNoEndCheck = !tubeKeyenceEndFired;
      tubeLengthLearned = true;
      if (hadNoEndCheck) {
        Serial.printf("[QC] Tube cleared without an End-position check -- measured length "
                      "%.1fmm now available for the next tube.\n",
                      learnedTubeLengthMm);
      }
      state = SystemState::TubeGap;
    }
  }
}

// =====================================================================
// HMI input polling -- rotates a low-rate RM read between the two
// operator-entered position words (HOTMELT_START_POSITION_ADDR,
// HOTMELT_END_POSITION_ADDR) and routes any successful result into
// hotMeltStartPositionMm/hotMeltEndPositionMm. Entirely inert whenever
// NS12_ENABLE_RM_POLLING is 0: no request ever goes out, so the fallback
// PLACEHOLDER values above stay in effect and hotMeltPositionsFromHmi
// stays false. V4.15.10: turned ON to field-test RM_WORD_ADDRESS_OFFSET
// (see the NS12 namespace) -- if RM still times out 100% with the offset
// applied, that rules the offset theory out and this should go back to 0.
// Safe to call unconditionally from loop() either way.
// =====================================================================
#if NS12_ENABLE_RM_POLLING
uint32_t lastHmiPollMs = 0;
uint8_t nextHmiPollIndex = 0;

// V4.15.30: word-vs-bit address verification walk ('V' serial command).
// The V4.15.29 burst-probe proved RB($B30..$B34) returns perfectly STATIC
// values (004A/004B/0036/0037/0038, unchanged across dozens of rapid
// samples spanning real button presses) -- not noise, not live state, just
// fixed content unrelated to the physical switches. RB_BIT_ADDRESS_OFFSET
// reuses the exact same 16384 (0x4000) already confirmed correct for RM's
// $W (word) reads -- if that constant is actually this PT's *word* memory
// area code rather than a bit-read-specific offset, then RB($B30..34) is
// silently landing on $W30..$W34 (real, valid, but unrelated memory,
// consistent with it sitting inside the "$W13-$W499 confirmed free" range
// found in the real Symbol Table) instead of true bit memory. This walks
// $W30..$W34 via the independently-confirmed-correct RM path so the two
// can be compared directly: if RM($W30..34) == RB($B30..34) exactly, that
// proves RB is reading word memory, not bit memory, and offset -- not bit
// position -- is the real bug. -1 = idle; 0..4 = which of $W30..$W34 is
// currently being requested.
int8_t wordVerifyIndex = -1;
#endif

// HMI push-button polling state (V4.15.12). Declared here (ahead of
// printDiagnostics(), which reports buttonState[]) even though the
// service function that updates it (serviceHmiButtonPolling(), below
// printDiagnostics() so it can call printDiagnostics()/pushTestPattern())
// comes later -- same split as pendingDisplayFrame/displayPushQueued
// elsewhere in this file.
#if NS12_ENABLE_RM_POLLING
constexpr uint16_t kButtonAddrs[NS12::BUTTON_COUNT] = {
    NS12::BUTTON_SETUP_ADDR, NS12::BUTTON_ALARM_LOG_ADDR, NS12::BUTTON_TREND_FULL_ADDR,
    NS12::BUTTON_TEST_ADDR, NS12::BUTTON_DIAG_ADDR};
const char *const kButtonNames[NS12::BUTTON_COUNT] = {"SETUP", "ALARM LOG", "TREND FULL", "TEST",
                                                       "DIAG"};
// V4.15.38: this is now the real, live, continuously-accurate state of
// each switch -- see the FIELD UPDATE below. No more host-clear forcing
// it back to false, no more ESP32-driven lamp approximation.
bool buttonState[NS12::BUTTON_COUNT] = {};
uint32_t lastButtonPollMs = 0;
uint8_t nextButtonPollIndex = 0;

// V4.15.40: field request -- "5 different LED as status", one per switch,
// each showing that switch's real current on/off state (not the old
// advanceLedCycle() round-robin, which only ever showed "who was pressed
// last"). kMcpOutputPins[0..4] (ILED_R, ILED_G, ILED_B, ELED_R, ELED_G)
// are dedicated 1:1 to SETUP/ALARM LOG/TREND FULL/TEST/DIAG in that
// order; kMcpOutputPins[5..6] (ELED_B, ELED_Y) are unused by this feature
// and stay free for manual bench testing (serial '6'/'7'). Called from
// serviceHmiButtonPolling() on every real state change (both edges -- a
// status LED must go off again when the switch does, unlike
// handleHmiButtonPress()'s one-shot rising-edge actions).
void setButtonStatusLed(uint8_t buttonIndex, bool on) {
  if (!mcpOk || buttonIndex >= NS12::BUTTON_COUNT) return;
  mcpOutputState[buttonIndex] = on;
  mcp.digitalWrite(kMcpOutputPins[buttonIndex], on);
}

// V4.15.29: burst-probe mode ('H' serial command) -- the normal
// BUTTON_POLL_INTERVAL_MS=750ms round-robin across 5 buttons only samples
// any ONE of them every ~3.75s. A held-button test (checking whether the
// underlying raw word ever changes) at that cadence can easily hold for
// 2-3 seconds and still never land a read during the actual touch --
// exactly the ambiguous "held it and 004A showed 3 times" result seen in
// the field, which doesn't distinguish "no bit exists in this word" from
// "we just didn't happen to sample it while pressed". While active, the
// interval below drops to 0 (still bounded by isReadPending()/txBusy, so
// it can't exceed the link's real turnaround), sampling all 5 addresses
// back-to-back for BURST_PROBE_DURATION_MS so a hold of even ~1s is
// guaranteed several reads during the press.
uint32_t burstProbeUntilMs = 0;
constexpr uint32_t BURST_PROBE_DURATION_MS = 8000;

// V4.15.34: 'N' serial command state -- write-then-read-back sanity check
// on $B30. Originally built to test the plain address against the offset
// address; that test CONFIRMED the offset was wrong (plain address read
// back "80"/bit7=1 for a write the offset address still read 0x00 for),
// and requestRB() itself was fixed in V4.15.35 to always use the plain
// address. Kept as a quick manual confirmation that the fix holds.
uint16_t noOffsetTestAddr = 0;
bool noOffsetTestPending = false;
// V4.15.26: added so a press could be confirmed via the periodic
// diagnostics dump without needing to watch Serial at the exact right
// moment -- originally because buttonState[] was force-cleared back to
// false immediately after every detected press (V4.15.14's host-clear),
// so the "instantaneous" line practically never caught a real "1". As of
// V4.15.38 buttonState[] is no longer cleared and genuinely reflects live
// switch state, so the instantaneous line is meaningful too now -- this
// counter stays useful regardless, as the total-presses-since-boot record.
uint32_t buttonPressCount[NS12::BUTTON_COUNT] = {};
#endif

// =====================================================================
// PlcComms (V4.15.20) -- the link to the Siemens S7-315-2 that actually
// runs the bottomer. This ESP32 is an "Auto Triggered Sensor" reporting
// to that PLC, not the boss of the line. Two directions:
//
//   ESP32 -> PLC: PLC_STATUS, one of 4 mutually-exclusive states (STOP/
//   ALARM/WARNING/READY), binary-encoded across 3 opto-isolated 24V
//   outputs (2 bits would exactly fit today's 4 states with zero spare;
//   3 bits leaves headroom -- ESP_OPTO_3 is bit 2/MSB, McpPin::OPTO_1 is
//   bit 0, McpPin::OPTO_2 is bit 1. Bit-to-pin assignment and the STOP=0/
//   ALARM=1/WARNING=2/READY=3 code values are both this file's choice,
//   not yet confirmed against the S7 program -- verify before relying on
//   it, and update setPlcStatus()/PlcStatus together if either needs to
//   change).
//
//   PLC -> ESP32: PLC_CONTROL, 2 independent flags (ACKNOWLEDGE,
//   MACHINE_RUNNING -- unlike PLC_STATUS these can be true/false in any
//   combination, so each gets its own dedicated bit, no encoding) read
//   from McpPin::INPUT_1/INPUT_2. Which physical input is which is also
//   this file's placeholder pairing -- confirm against the S7 program.
//   Read only during Standby/TubeGap, same ~20ms-cadence restriction as
//   the rest of this file's MCP polling (an I2C transaction every loop()
//   iteration would cost InspectingTube latency it can't afford) -- flag
//   to the user if the S7 program needs to see these change faster than
//   that, e.g. mid-tube-pass.
// =====================================================================
namespace PlcComms {
enum class PlcStatus : uint8_t { STOP = 0, ALARM = 1, WARNING = 2, READY = 3 };

void setStatus(PlcStatus s) {
  uint8_t code = static_cast<uint8_t>(s);
  digitalWrite(Pins::ESP_OPTO_3, (code >> 2) & 1);
  if (!mcpOk) return;
  mcp.digitalWrite(McpPin::OPTO_1, (code >> 0) & 1);
  mcp.digitalWrite(McpPin::OPTO_2, (code >> 1) & 1);
}
} // namespace PlcComms

bool plcAcknowledge = false;
bool plcMachineRunning = false;
PlcComms::PlcStatus plcLastCommandedStatus = PlcComms::PlcStatus::STOP;

void serviceHmiInputPolling() {
#if NS12_ENABLE_RM_POLLING
  uint32_t now = millis();
  // V4.15.30: the word-vs-bit verify walk takes priority over the normal
  // position poll while active -- it's a short, deliberate 5-step manual
  // test ('V'), not a background cadence, so it fires as soon as the link
  // is free rather than waiting for RM_POLL_INTERVAL_MS.
  if (wordVerifyIndex >= 0) {
    if (!ns12.isReadPending()) {
      ns12.requestRM((uint16_t)(30 + wordVerifyIndex), 1);
    }
  } else if (!ns12.isReadPending() && now - lastHmiPollMs >= NS12::RM_POLL_INTERVAL_MS) {
    lastHmiPollMs = now;
    uint16_t addr = (nextHmiPollIndex == 0) ? NS12::HOTMELT_START_POSITION_ADDR
                                             : NS12::HOTMELT_END_POSITION_ADDR;
    ns12.requestRM(addr, 1);
    nextHmiPollIndex = (nextHmiPollIndex + 1) % 2;
  }

  uint16_t addr, value;
  if (ns12.consumeReadWord(addr, value)) {
    if (wordVerifyIndex >= 0 && addr == (uint16_t)(30 + wordVerifyIndex)) {
      static const char *const kVerifyNames[5] = {"SETUP", "ALARM LOG", "TREND FULL", "TEST",
                                                    "DIAG"};
      Serial.printf("[WORD-VERIFY] RM $W%u (%s) = 0x%04X\n", (unsigned)(30 + wordVerifyIndex),
                    kVerifyNames[wordVerifyIndex], value);
      wordVerifyIndex++;
      if (wordVerifyIndex >= 5) {
        wordVerifyIndex = -1;
        Serial.println(F("[WORD-VERIFY] Done -- compare these 5 values against the "
                          "[HMI-RAW] lines for the same buttons."));
      }
    } else if (addr == NS12::HOTMELT_START_POSITION_ADDR) {
      hotMeltStartPositionMm = (float)value * HMI_POSITION_MM_PER_COUNT;
      hotMeltPositionsFromHmi = true;
    } else if (addr == NS12::HOTMELT_END_POSITION_ADDR) {
      hotMeltEndPositionMm = (float)value * HMI_POSITION_MM_PER_COUNT;
      hotMeltPositionsFromHmi = true;
    }
  }
#endif
}

void handleKeyenceResult() {
  bool pass = keyenceResultPass;
  keyenceResultPending = false;
  if (!pass) {
    Serial.println(F("[QC] Keyence result: FAIL"));
  }
}

// =====================================================================
// Telemetry helpers -- feed the NS12 $W100-$W108 block every loop (the
// manager only actually transmits it every NS12::TELEMETRY_WRITE_INTERVAL_MS).
// V4.15.25: moved off $W0-$W8, see TELEMETRY_BASE_ADDR's own comment.
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
  if (rawBaselineCaptureInProgress) {
    Serial.printf("Raw baseline       : capturing now... (%u/%u frames, previous baseline %s)\n",
                  rawBaselineFramesCollected, RAW_BASELINE_FRAME_COUNT,
                  rawBaselineCaptured ? "still in use until this completes" : "none yet -- stats frozen");
  } else {
    Serial.printf("Raw baseline       : %s\n",
                  rawBaselineCaptured ? "CAPTURED" : "NOT CAPTURED (send 'B')");
  }
  Serial.printf("Min/Max/Avg raw delta : %.0f / %.0f / %.0f\n",
                minimumTemperatureC, maximumTemperatureC, averageTemperatureC);
  Serial.printf("Implausible pixels : %u (outside %.0f..%.0f raw delta, rejected)\n",
                lastFrameRejectedPixelCount, MIN_PLAUSIBLE_RAW_DELTA, MAX_PLAUSIBLE_RAW_DELTA);
  Serial.printf("Good/Failed frames : %lu / %lu\n",
                (unsigned long)successfulFrameCount, (unsigned long)failedFrameCount);
  // V4.15.44: encoder now physically wired -- total() was previously
  // consumed only internally (forward-projection math), never printed.
  Serial.printf("Encoder count (raw/mm) : %lld / %.1f\n", (long long)encoder.total(),
                (float)encoder.total() / ENCODER_COUNTS_PER_MM);

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

  Serial.printf("NS12 WM attempts/failures/oversized : %lu / %lu / %lu\n",
                (unsigned long)ns12.wmAttemptCount(), (unsigned long)ns12.wmFailureCount(),
                (unsigned long)ns12.wmOversizedCountValue());
#if NS12_ENABLE_RM_POLLING
  Serial.printf("NS12 RM attempts/success/writeFail/timeout/parseErr : %lu / %lu / %lu / %lu / %lu\n",
                (unsigned long)ns12.rmAttemptCount(), (unsigned long)ns12.rmSuccessCount(),
                (unsigned long)ns12.rmWriteFailureCount(), (unsigned long)ns12.rmTimeoutCount(),
                (unsigned long)ns12.rmParseErrorCount());
  Serial.printf("NS12 RB attempts/success/writeFail/timeout/parseErr : %lu / %lu / %lu / %lu / %lu\n",
                (unsigned long)ns12.rbAttemptCount(), (unsigned long)ns12.rbSuccessCount(),
                (unsigned long)ns12.rbWriteFailureCount(), (unsigned long)ns12.rbTimeoutCount(),
                (unsigned long)ns12.rbParseErrorCount());
  Serial.printf("NS12 WB attempts/failures : %lu / %lu\n", (unsigned long)ns12.wbAttemptCount(),
                (unsigned long)ns12.wbFailureCount());
  Serial.print(F("HMI buttons (SETUP/ALARM LOG/TREND FULL/TEST/DIAG), instantaneous : "));
  for (uint8_t i = 0; i < NS12::BUTTON_COUNT; i++) {
    Serial.print(buttonState[i] ? '1' : '0');
    Serial.print(i + 1 < NS12::BUTTON_COUNT ? '/' : '\n');
  }
  // V4.15.26: the line above will practically always read all-0 -- see
  // buttonPressCount[]'s own comment for why that's not evidence of a
  // broken read. This lifetime counter is the real way to confirm presses
  // are registering via this periodic dump.
  Serial.print(F("HMI button presses (lifetime)                                  : "));
  for (uint8_t i = 0; i < NS12::BUTTON_COUNT; i++) {
    Serial.print((unsigned long)buttonPressCount[i]);
    Serial.print(i + 1 < NS12::BUTTON_COUNT ? '/' : '\n');
  }
#else
  Serial.println(F("NS12 RM polling    : disabled (PT doesn't respond -- see NS12 namespace comment)"));
#endif
  // V4.15.20: S7-315-2 PLC comms + Keyence Result (see PlcComms namespace
  // and keyenceResultIsr()). Raw per-bit toggles for OPTO_1/OPTO_2/
  // ESP_OPTO_3 stay available via serial commands '8'/'9'/'A' for
  // electrical verification -- not repeated here every diagnostics tick.
  Serial.printf("PLC_STATUS (commanded) : %u (0=STOP/1=ALARM/2=WARNING/3=READY)\n",
                (unsigned)plcLastCommandedStatus);
  Serial.printf("PLC_CONTROL ACKNOWLEDGE/MACHINE_RUNNING : %d / %d\n", plcAcknowledge,
                plcMachineRunning);
  Serial.printf("Keyence Result pending/pass (ESP_INPUT_3/GPIO%u) : %d / %d\n", Pins::ESP_INPUT_3,
                keyenceResultPending, keyenceResultPass);
  Serial.printf("NS12 32x24 experimental   : %s\n", experimental32x24Effective ? "ON" : "OFF (16x8)");
  Serial.printf("HotMelt Start/End position (mm) : %.1f / %.1f (%s)\n",
                hotMeltStartPositionMm, hotMeltEndPositionMm,
                hotMeltPositionsFromHmi ? "from HMI" : "PLACEHOLDER fallback, not from HMI yet");
  if (tubeLengthLearned) {
    Serial.printf("Tube length          : %.1f mm (from previous tube)\n", learnedTubeLengthMm);
  } else {
    Serial.println(F("Tube length          : not yet learned (no tube has cleared the sensor yet)"));
  }
  Serial.printf("Free heap          : %.1f kB\n", ESP.getFreeHeap() / 1024.0f);
}

// =====================================================================
// HMI push-button polling and lamp feedback (V4.15.12). Rotates a fast RB
// (bit-read) request between the 5 SETUP/ALARM LOG/TREND FULL/TEST/DIAG
// buttons ($B30-$B34, confirmed from the real Symbol Table) and dispatches
// a rising-edge (not-pressed -> pressed) to handleHmiButtonPress(). Mirrors
// serviceHmiInputPolling()'s shape but on its own faster interval
// (NS12::BUTTON_POLL_INTERVAL_MS) since a human is watching for the lamp
// to react. Entirely inert whenever NS12_ENABLE_RM_POLLING is 0, same as
// the position polling above.
// =====================================================================
#if NS12_ENABLE_RM_POLLING
// REMOVED (V4.15.38): setActiveLamp() used to WB-write a single "most
// recently pressed" lamp across $B40-$B44 -- an ESP32-driven approximation
// that stopped being meaningful once the switches moved to native
// panel-side Alternate toggling (see the FIELD UPDATE below): (a) it never
// reflected any button's REAL current on/off state, only "who was pressed
// last", and (b) this file's own host-clear-to-0 write on $B30-$B34 was
// actively fighting the panel's own toggle bit, forcing it back off right
// after every detected press. The panel's native Alternate visual is now
// the sole, correct status indicator -- it IS each switch's real state,
// continuously, with no serial round-trip involved at all.

void handleHmiButtonPress(uint8_t index) {
  buttonPressCount[index]++;
  Serial.printf("[HMI] %s button pressed.\n", kButtonNames[index]);
  switch (index) {
  case 3: // TEST -- same one-shot pattern as the 'M' serial command
    pushTestPattern();
    break;
  case 4: // DIAG -- same immediate report as the 'D' serial command
    printDiagnostics();
    break;
  // SETUP / ALARM LOG / TREND FULL: no subsystem exists yet for these --
  // no setup-parameter screen, no alarm log, no trend recording. Real
  // behavior still needs a spec -- what SETUP should configure, where the
  // alarm log lives, what TREND FULL should show -- before more goes
  // here, same as HotMelt Start/End Position waited on the operator-entry
  // spec before V4.15.7 built the position-tracking logic. V4.15.40: each
  // now has a dedicated status LED (set in serviceHmiButtonPolling(), not
  // here -- see setButtonStatusLed()'s comment) as its physical, no-spec-
  // needed confirmation instead of the old shared round-robin cycle.
  default:
    break;
  }
}
#endif

// CHANGED (V4.15.38): the switches moved from Momentary to native
// panel-side Alternate toggling ("we'll do toggle in HMI" -- ESP32-side
// detection latency, ~3.75s worst case round-robin, was too slow for a
// status lamp that needs to show reality the instant a finger lands).
// The WB host-clear-to-0 this function used to do after every detected
// press is GONE: for a Momentary switch that acknowledge was correct
// (the panel expects it, or the bit latches forever), but for Alternate
// it was actively wrong -- forcing the panel's own toggle bit back off
// immediately after every press, fighting the very state it's supposed
// to own. buttonState[i] is no longer force-cleared either; it's now a
// live, continuously-accurate mirror of each switch's real bit,
// unconditionally read every poll rather than reset by our own writes.
// handleHmiButtonPress() still fires once per rising edge (0->1) for its
// one-shot side effects (LED cycle, test pattern, diagnostics) -- that
// part doesn't change; only the writing back stops.
//
// V4.15.43: "only the writing back stops" above is no longer the whole
// picture -- see the mutual-exclusion write added at the bottom of this
// function's rising-edge branch, and its own FIELD UPDATE comment. It
// writes a DIFFERENT switch's bit than the one that just changed, so it
// does not conflict with this paragraph's still-true claim about a
// switch's own bit.
void serviceHmiButtonPolling() {
#if NS12_ENABLE_RM_POLLING
  uint32_t now = millis();
  // V4.15.34: no-offset RB test ('N' command) takes priority over the
  // normal round-robin -- see that command's comment. Retries every tick
  // (like the txBusy-deferred pattern elsewhere) until the link is free
  // right after the WB write that precedes it.
  if (noOffsetTestPending) {
    if (!ns12.isReadPending() && ns12.requestRB(noOffsetTestAddr, 1)) {
      noOffsetTestPending = false;
    }
  } else {
    uint32_t pollInterval = (now < burstProbeUntilMs) ? 0 : NS12::BUTTON_POLL_INTERVAL_MS;
    if (!ns12.isReadPending() && now - lastButtonPollMs >= pollInterval) {
      lastButtonPollMs = now;
      ns12.requestRB(kButtonAddrs[nextButtonPollIndex], 1);
      nextButtonPollIndex = (nextButtonPollIndex + 1) % NS12::BUTTON_COUNT;
    }
  }

  uint16_t addr;
  bool pressed;
  uint16_t rawValue;
  if (ns12.consumeReadBit(addr, pressed, rawValue)) {
    for (uint8_t i = 0; i < NS12::BUTTON_COUNT; i++) {
      if (kButtonAddrs[i] != addr) continue;

      // CHANGED (V4.15.39): re-gated behind NS12_DEBUG_RAW_RX -- was made
      // unconditional in V4.15.27 to root-cause the checksum/bit-position/
      // address-offset bugs, all fixed and confirmed now. Printing this
      // every ~750ms per button (or continuously in burst mode) was pure
      // clutter for normal operation. See the new "[SWITCH]" print just
      // below for the clean, on-change replacement.
#if NS12_DEBUG_RAW_RX
      Serial.printf("[HMI-RAW] %-11s $B%-3u raw=0x%02X bit7=%u (bit0=%u)\n", kButtonNames[i],
                    kButtonAddrs[i], rawValue, (unsigned)((rawValue & 0x80) != 0),
                    (unsigned)(rawValue & 1));
#endif

      // V4.15.23 field report (buttons firing with nobody touching the
      // screen) led to a 2-consecutive-reads debounce here, costing up to
      // one extra ~750ms poll cycle of detection latency (compounding
      // with the 5-button round-robin's own 3.75s worst case into ~6-7.5s
      // total, per a later field report). REMOVED in V4.15.37: that
      // symptom's real cause turned out to be the checksum bug (V4.15.31)
      // and the RB address offset bug (V4.15.35), both now fixed and
      // hardware-confirmed clean (checksum-validated 0x00/0x80 reads,
      // zero anomalies across hundreds of samples). Checksum validation
      // in parseReadResponse() already rejects any genuinely corrupted
      // frame outright -- a stronger guarantee than "wait for two reads
      // to agree" ever was -- so a single accepted read is now trusted
      // directly.
      // V4.15.38: no more host-clear-to-0 write here -- see this
      // function's header comment. buttonState[i] is simply set to the
      // real bit value read; the panel (Alternate switch) owns it
      // entirely, so there is nothing for the host to acknowledge or
      // clear anymore. handleHmiButtonPress() still fires exactly once
      // per rising edge for its one-shot dispatch (test pattern/
      // diagnostics for TEST/DIAG); the falling edge has no one-shot
      // action defined, but V4.15.40 gives it a real job below: turning
      // the matching status LED back off.
      bool wasPressed = buttonState[i];
      buttonState[i] = pressed;
      // V4.15.39: clean, on-change-only status line -- "which function is
      // on" at a glance, replacing the noisy per-poll [HMI-RAW] firehose
      // (now debug-gated, see above) for normal terminal use.
      // V4.15.40: dedicated status LED, tracking both edges (unlike
      // handleHmiButtonPress()'s one-shot rising-edge actions) -- see
      // setButtonStatusLed()'s comment.
      if (pressed != wasPressed) {
        Serial.printf("[SWITCH] %s -> %s\n", kButtonNames[i], pressed ? "ON" : "OFF");
        setButtonStatusLed(i, pressed);
      }
      if (pressed && !wasPressed) {
        // V4.15.43: mutual exclusion -- clear every OTHER switch still
        // showing on, since only one of the 5 screen/function selectors is
        // ever meant to be active at a time. See this function's header
        // FIELD UPDATE for why this doesn't reintroduce the V4.15.38
        // host-clear-on-self pattern.
        for (uint8_t j = 0; j < NS12::BUTTON_COUNT; j++) {
          if (j == i || !buttonState[j]) continue;
          bool offBit = false;
          ns12.sendWB(kButtonAddrs[j], &offBit, 1);
          buttonState[j] = false;
          setButtonStatusLed(j, false);
          Serial.printf("[SWITCH] %s -> OFF (reset by %s)\n", kButtonNames[j], kButtonNames[i]);
        }
        handleHmiButtonPress(i);
      }
      break;
    }
  }
#endif
}

// =====================================================================
// PlcComms input side -- reads the S7-315-2's 2 control flags
// (ACKNOWLEDGE/MACHINE_RUNNING) off the MCP23017. Folded into the same
// Standby/TubeGap-only, ~20ms-cadence MCP poll window loop() already uses
// (see that block's own comment) rather than a separate timer, since an
// extra independent I2C poll interval would just double the I2C traffic
// for no benefit.
// =====================================================================
void servicePlcControl() {
  if (!mcpOk || !(state == SystemState::Standby || state == SystemState::TubeGap)) return;
  bool ack = (mcp.digitalRead(McpPin::INPUT_1) == LOW); // INPUT_PULLUP: idle HIGH
  bool running = (mcp.digitalRead(McpPin::INPUT_2) == LOW);
  if (ack != plcAcknowledge) {
    plcAcknowledge = ack;
    Serial.printf("[PLC] ACKNOWLEDGE -> %s\n", ack ? "ACTIVE" : "idle");
  }
  if (running != plcMachineRunning) {
    plcMachineRunning = running;
    Serial.printf("[PLC] MACHINE_RUNNING -> %s\n", running ? "ACTIVE" : "idle");
  }
}

// =====================================================================
// Serial diagnostic commands:
//   S/W/I/G/F  -- force state (bench test). NOTE (V4.15.19): Standby no
//                 longer auto-advances to WaitingForTube on its own (the
//                 old MCP AUTO/MACHINE_STOPPED inputs it depended on don't
//                 exist in the real hardware map) -- use 'W' explicitly
//                 until real inputs are assigned to that decision.
//   1-9        -- toggle MCP outputs, in order: ILED_R, ILED_G, ILED_B,
//                 ELED_R, ELED_G, ELED_B, ELED_Y, OPTO_1, OPTO_2
//                 (V4.15.19 real hardware map -- verify each with a
//                 meter/LED on the bench). NOTE: '8'/'9' are also
//                 PLC_STATUS bits 0/1 -- a manual toggle here is transient,
//                 overwritten on the next real state-driven PLC_STATUS
//                 update (see loop()).
//   A          -- toggle ESP_OPTO_3 directly (PLC_STATUS bit 2, same
//                 transient-manual-toggle caveat as '8'/'9' above)
//   P          -- cycle PLC_STATUS through STOP->ALARM->WARNING->READY (all
//                 3 bits together via PlcComms::setStatus()) for bench-
//                 verifying the encoding end-to-end against the S7 program
//   K          -- manually fire the Keyence trigger pulse (GPIO1) for
//                 bench-verifying that wiring independent of tube tracking
//   M          -- diagnostic test pattern / one-shot matrix push (send 'R'
//                 to clear it -- the live pipeline won't overwrite it until
//                 the next real capture completes)
//   C          -- force capture rearm
//   B          -- capture per-pixel raw baseline (needed before raw-delta
//                 acquisition produces meaningful values -- run at idle)
//   X          -- frame dump: raw-delta (live) vs one-off calibrated C
//   R          -- rearm capture latch AND clear the HMI display to blank
//   D          -- print diagnostics immediately
//   H          -- HMI button burst-probe (V4.15.29): polls all 5 buttons
//                 back-to-back for BURST_PROBE_DURATION_MS instead of the
//                 normal 750ms-per-button round-robin, so a held real press
//                 is guaranteed several [NS12-FRAME]/[HMI-RAW] samples
//                 during the touch instead of maybe zero
//   V          -- word-vs-bit verify walk (V4.15.30): reads $W30..$W34 via
//                 RM (the independently-confirmed-correct word path) and
//                 prints each, so they can be compared directly against
//                 RB's [HMI-RAW] $B30..$B34 values -- an exact match proves
//                 RB is silently landing on word memory, not bit memory
//   J          -- WB-then-RB round-trip self-test (V4.15.32): writes 1 to
//                 $B30 ourselves (no touchscreen), then bursts RB reads of
//                 it -- isolates whether the read/write/address/checksum
//                 pipeline is correct end to end, independent of whether
//                 the touchscreen object ever actually writes there
//
// Separately, the HMI's own SETUP/ALARM LOG/TREND FULL/TEST/DIAG push-
// buttons ($B30-$B34) are polled over NS12 RB (see serviceHmiButtonPolling()
// below) and dispatch through handleHmiButtonPress() -- TEST and DIAG there
// call the same pushTestPattern()/printDiagnostics() as 'M' and 'D' here.
// PLC_CONTROL (servicePlcControl(), above) and Keyence Result
// (keyenceResultIsr()) are unrelated to any of this -- the S7-315-2 PLC
// link and the Keyence sensor, not the HMI.
// =====================================================================
void handleSerialCommand(char c) {
  switch (c) {
  case 'S': state = SystemState::Standby; break;
  case 'W': state = SystemState::WaitingForTube; break;
  case 'I': state = SystemState::InspectingTube; break;
  case 'G': state = SystemState::TubeGap; break;
  case 'F': enterFaultStop(); break;
  case '1': case '2': case '3': case '4': case '5':
  case '6': case '7': case '8': case '9': {
    uint8_t idx = c - '1';
    if (mcpOk) {
      toggleMcpOutput(idx);
      Serial.printf("[IO-TEST] MCP output #%c (pin %u) -> %s\n", c, kMcpOutputPins[idx],
                    mcpOutputState[idx] ? "HIGH" : "LOW");
    }
    break;
  }
  case 'A': {
    bool newState = !digitalRead(Pins::ESP_OPTO_3);
    digitalWrite(Pins::ESP_OPTO_3, newState);
    Serial.printf("[IO-TEST] ESP_OPTO_3 (GPIO%u) -> %s\n", Pins::ESP_OPTO_3,
                  newState ? "HIGH" : "LOW");
    break;
  }
  case 'K':
    keyenceTrigger.fire();
    Serial.println(F("[IO-TEST] Keyence trigger pulse fired (GPIO1)."));
    break;
  case 'P': {
    static const char *kStatusNames[4] = {"STOP", "ALARM", "WARNING", "READY"};
    uint8_t nextCode = (static_cast<uint8_t>(plcLastCommandedStatus) + 1) % 4;
    plcLastCommandedStatus = static_cast<PlcComms::PlcStatus>(nextCode);
    PlcComms::setStatus(plcLastCommandedStatus);
    Serial.printf("[PLC] PLC_STATUS -> %u (%s)\n", nextCode, kStatusNames[nextCode]);
    break;
  }
  case 'M':
    pushTestPattern();
    break;
  case 'C':
    capture.rearm();
    Serial.println(F("[DIAG] Capture forced/rearmed."));
    break;
  case 'B':
    if (!rawBaselineCaptureInProgress) {
      startRawBaselineCapture();
    } else {
      Serial.println(F("[DIAG] Baseline capture already in progress."));
    }
    break;
  case 'X': {
    Serial.println(F("[DIAG] Frame dump: raw-delta (live pipeline) vs calibrated C (one-off"));
    Serial.println(F("       mlx.getFrame() snapshot, for correlating real thresholds):"));
    if (!rawBaselineCaptured) {
      Serial.println(F("[DIAG] WARNING: no baseline captured yet ('B') -- raw-delta values"));
      Serial.println(F("       below are meaningless (baseline defaults to 0)."));
    }
    static float calibratedSnapshot[StripZone::COLS * StripZone::ROWS];
    bool calibratedOk = mlxInitialized && (mlx.getFrame(calibratedSnapshot) == 0);
    for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
      Serial.print(mlxFrame[i], 0);
      Serial.print('/');
      if (calibratedOk) {
        Serial.print(calibratedSnapshot[i], 1);
      } else {
        Serial.print(F("?"));
      }
      Serial.print(i % StripZone::COLS == StripZone::COLS - 1 ? '\n' : ' ');
    }
    break;
  }
  case 'R': {
    capture.rearm();
    // V4.15.10 FIELD FIX: rearm alone only reset CaptureController's latch
    // state, never the HMI display itself -- since the live pipeline only
    // pushes a new composite when a real capture completes (by design, so
    // the HMI holds a *stable* QC-confirmation image rather than flickering
    // live video), a one-shot diagnostic push like 'M's test pattern had no
    // path back to a blank screen; 'R' looked like it did nothing. Push an
    // explicit all-minimum (coldest palette bucket) frame here so 'R' is
    // actually a visible reset, not just an internal state change.
    float blankFrame[StripZone::COLS * StripZone::ROWS];
    for (size_t i = 0; i < StripZone::COLS * StripZone::ROWS; i++) {
      blankFrame[i] = MATRIX_RAW_DELTA_MIN;
    }
    pushWordLampMatrix(blankFrame);
    Serial.println(F("[DIAG] Rearmed, display cleared."));
    break;
  }
  case 'D':
    printDiagnostics();
    break;
  case 'H':
    burstProbeUntilMs = millis() + BURST_PROBE_DURATION_MS;
    Serial.printf("[HMI-PROBE] Burst mode ON for %lu ms -- hold any HMI button NOW, watch for "
                  "[NS12-FRAME]/[HMI-RAW] lines on its address.\n",
                  (unsigned long)BURST_PROBE_DURATION_MS);
    break;
#if NS12_ENABLE_RM_POLLING
  case 'V':
    wordVerifyIndex = 0;
    Serial.println(F("[WORD-VERIFY] Reading $W30..$W34 via RM (confirmed-correct word path) -- "
                      "compare against the RB [HMI-RAW] values for $B30..$B34."));
    break;
#endif
  case 'J': {
    // V4.15.32: self-contained WB-then-RB round-trip test on $B30 -- writes
    // a 1 to the button address OURSELVES (no touchscreen involved at all),
    // then bursts RB reads of it so the very next poll shows whether our
    // OWN write round-trips as bit7=1. Isolates the question completely:
    // if this comes back 1, the ESP32<->PT read/write/address/checksum
    // pipeline is fully proven end to end, and the touch never reaching
    // $B30 is conclusively a CX-Designer/PT-side issue, not this firmware.
    // If it comes back 0 even for our own write, that's a real remaining
    // bug in sendWB() itself (worth checking against the manual's WB *D
    // encoding, which packs 4 bits per hex digit MSB-first -- the same
    // convention RB's fix already uncovered for reads).
    bool oneBit = true;
    ns12.sendWB(NS12::BUTTON_SETUP_ADDR, &oneBit, 1);
    burstProbeUntilMs = millis() + 4000;
    Serial.println(F("[WB-RB-TEST] Wrote 1 to $B30 (SETUP) ourselves, no touchscreen involved. "
                      "Watching for bit7=1 on the next [HMI-RAW] SETUP line..."));
    break;
  }
#if NS12_ENABLE_RM_POLLING
  case 'N': {
    // V4.15.34/35: writes 1 to $B30 (same as 'J'), then reads it straight
    // back via RB. This is what originally proved RB_BIT_ADDRESS_OFFSET
    // was wrong (the plain address read "80"/bit7=1 for the exact write
    // the offset address still read 0x00 for) -- now that requestRB()
    // itself uses the plain address unconditionally (V4.15.35), this is a
    // plain write-then-read-back sanity check, kept as a quick manual
    // confirmation that the fix holds.
    bool oneBit = true;
    ns12.sendWB(NS12::BUTTON_SETUP_ADDR, &oneBit, 1);
    noOffsetTestAddr = NS12::BUTTON_SETUP_ADDR;
    noOffsetTestPending = true;
    Serial.println(F("[WB-RB-SANITY] Wrote 1 to $B30, reading it back via RB -- watch the next "
                      "[NS12-FRAME] line for dataText=\"80\"."));
    break;
  }
#endif
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
    for (uint8_t p : {McpPin::ILED_R, McpPin::ILED_G, McpPin::ILED_B, McpPin::ELED_R,
                       McpPin::ELED_G, McpPin::ELED_B, McpPin::ELED_Y, McpPin::OPTO_1,
                       McpPin::OPTO_2}) {
      mcp.pinMode(p, OUTPUT);
      mcp.digitalWrite(p, LOW);
    }
    for (uint8_t p : {McpPin::INPUT_1, McpPin::INPUT_2}) {
      mcp.pinMode(p, INPUT_PULLUP);
    }
  }

  mlxDetected = isI2CAddressPresent(MLX90640_I2CADDR_DEFAULT);
  bool mlxOk = mlxDetected && initializeMlx();
  mlxInitialized = mlxOk;
  Serial.printf("MLX90640 initialized: %s\n", mlxOk ? "YES" : "NO");
  setStatusLed(mlxOk ? 0 : 30, mlxOk ? 25 : 0, 0);

  ns12.begin();

  // PULLDOWN: gives a defined idle LOW consistent with the code's existing
  // assumption (presenceState reads HIGH-active per handlePresenceEdge()'s
  // rising-edge = "tube entering"). CONFIRMED (V4.15.19) this pin now
  // carries a real 24V signal through a 10K/1.5K divider -- see
  // ENCODER_PULSE_PIN's begin() comment for why that makes the internal
  // pull practically a no-op rather than a noise-guard now.
  pinMode(Pins::PRESENCE_SENSOR_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(Pins::PRESENCE_SENSOR_PIN), presenceIsr, CHANGE);

  keyenceTrigger.begin(Pins::KEYENCE_TRIGGER_PIN);
  encoder.begin(Pins::ENCODER_PULSE_PIN);

  // ESP_OPTO_3 (V4.15.20): PLC_STATUS bit 2 -- see PlcComms namespace.
  // Starts LOW (code 0 = STOP), matching plcLastCommandedStatus's default
  // and the MCP outputs' own LOW init above, so there's no boot-time
  // mismatch before the first loop() iteration corrects it to READY/STOP
  // as appropriate.
  pinMode(Pins::ESP_OPTO_3, OUTPUT);
  digitalWrite(Pins::ESP_OPTO_3, LOW);

  // ESP_INPUT_3 (V4.15.20): Keyence Result, restored to a direct-GPIO
  // hardware interrupt -- see keyenceResultIsr()'s comment for why.
  // PULLDOWN: same reasoning as PRESENCE_SENSOR_PIN above -- defined idle
  // state, effectively a no-op given the real 24V/10K-1.5K divider on this
  // pin, kept for consistency.
  pinMode(Pins::ESP_INPUT_3, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(Pins::ESP_INPUT_3), keyenceResultIsr, CHANGE);

  fpsWindowStartMs = millis();
  lastDiagnosticMs = millis();

  if (mcpOk && mlxOk) {
    state = SystemState::Standby;
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
  // CORRECTED (V4.15.15): positions before buttons now, reversed from
  // V4.15.12's original ordering. Both share the single in-flight RM/RB
  // read slot; buttons going first seemed reasonable (a human is watching
  // for the lamp to react) but real hardware showed it starved position
  // reads badly under combined RM+RB+WM+WB traffic (RM success crashed to
  // ~19% -- see NS12::BUTTON_POLL_INTERVAL_MS's comment). HotMelt Start/
  // End Position feeds real tube-tracking logic; a button's lamp reacting
  // a poll cycle later than it otherwise would is a fair trade.
  serviceHmiInputPolling();
  serviceHmiButtonPolling();

  // MCP polled only during Standby/TubeGap, ~20ms cadence -- Keyence
  // Result no longer depends on this (V4.15.20 restored it to a direct-
  // GPIO hardware interrupt, see keyenceResultIsr()), but PLC_CONTROL
  // (servicePlcControl(), below) does -- flag to the S7 side if it needs
  // ACKNOWLEDGE/MACHINE_RUNNING visibility faster than this.
  //
  // REMOVED (V4.15.19): the old MACHINE_STOPPED/AUTO reads that drove
  // Standby->WaitingForTube and the machineStopped->enterFaultStop() fault
  // path -- both pins were part of the placeholder scheme the real
  // hardware map has no equivalent for. Deliberately NOT replaced with a
  // guess at which McpPin::INPUT_1/INPUT_2 means "auto" or "stopped" --
  // that's exactly the kind of safety-relevant assumption this project
  // shouldn't invent unasked. Standby now requires an explicit 'W' serial
  // command to advance to WaitingForTube until real inputs are assigned;
  // there is currently no MCP-driven fault path either. TubeGap's own
  // advance below is unaffected (never depended on these pins).
  if ((state == SystemState::Standby || state == SystemState::TubeGap) && mcpOk) {
    uint32_t now = millis();
    if (now - lastMcpPollMs >= MCP_POLL_INTERVAL_MS) {
      lastMcpPollMs = now;
      servicePlcControl();
      if (state == SystemState::TubeGap) {
        state = SystemState::WaitingForTube;
        capture.rearm();
      }
    }
  }

  // PLC_STATUS output: STOP whenever FaultStop, READY otherwise. ALARM and
  // WARNING have no trigger condition defined yet -- this project has no
  // existing concept of a "warning, but not a fault" state to map onto
  // them; use 'P' to drive them manually for bench/PLC-program testing
  // until a real condition is decided.
  {
    PlcComms::PlcStatus wantStatus =
        (state == SystemState::FaultStop) ? PlcComms::PlcStatus::STOP : PlcComms::PlcStatus::READY;
    if (wantStatus != plcLastCommandedStatus) {
      plcLastCommandedStatus = wantStatus;
      PlcComms::setStatus(wantStatus);
    }
  }

  if (mlxInitialized) {
    uint32_t nowMs = millis();
    if (nowMs - lastMlxFrameMs >= MLX_FRAME_PERIOD_MS) {
      lastMlxFrameMs = nowMs;
      // Raw read + baseline subtract (V4.15.3) replaces per-frame
      // mlx.getFrame() here -- see the raw-acquisition section above.
      static float rawPixelsNow[32 * 24];
      bool rawOk = readMlxRawCombined(rawPixelsNow);
      lastFrameValid = rawOk;

      if (rawOk) {
        successfulFrameCount++;
        fpsWindowFrameCount++;
        consecutiveFrameFailures = 0;
        updateFrameRate();
        setStatusLed(0, 18, 0); // brief green heartbeat

        if (rawBaselineCaptureInProgress) {
          serviceRawBaselineCapture(rawPixelsNow);
        } else if (rawBaselineCaptured) {
          subtractBaseline(rawPixelsNow, mlxFrame);
          calculateFrameStatistics();
          capture.onNewFrame(mlxFrame);
        }
        // else: no baseline yet and none in progress -- raw reads succeed
        // (fps/heartbeat/recovery logic all still work) but nothing feeds
        // the QC/HMI pipeline until 'B' is run once. See the 'B'/'X'
        // serial commands.
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

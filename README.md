# TGIS-510 — Thermal Glue Inspection System

Home-lab / after-hours project. **Separate from the 410 Rotaliner Tubing
Seal Seam Monitor** (factory floor, S7-300/ATmega2560) — not related.

Industrial QC system detecting hot-melt glue application on tubes moving at
high speed. Confirms glue presence, temperature, and quantity across both
glue strips per tube pass, and pushes a stable QC-confirmation image to an
operator HMI (Omron NS12).

## Status: clean-room implementation, not yet bench-verified

This repository started empty. `src/tgis510_v4_14_0.cpp` was written
directly from a project handoff synthesis — it is **not** a copy or edit of
an existing file. Neither the real, currently-flashed `tgis510_v4_14_0.cpp`
nor the earlier `HotMelt_MLX90640_80032_9_8_0.cpp` snapshot referenced in
the handoff were available to produce this. Before flashing:

1. Diff this against whatever is actually on disk at
   `C:\Users\Admin\Documents\PlatformIO\Projects\510 HotMelt Monitor\` on
   the PlatformIO machine, and reconcile any divergence.
2. Work through the open placeholders and action items below — several
   require physical access to the hardware (oscilloscope, silkscreen,
   encoder datasheet) that this environment does not have.

## Hardware

| Component | Role |
|---|---|
| ESP32-S3-Zero-M (ESP32-S3FH4R2) | Main controller |
| MLX90640 32×24 thermal camera | Glue strip presence/temp/quantity (I2C 800kHz) |
| Omron NS12-TS00B-V2 HMI | Operator display, Memory Link protocol, RS-232 via HIN232CP |
| MCP23017 I/O expander | Machine I/O (stop/interlock), shares I2C bus |
| Keyence IV2-G300CA + IV2-G30 | Vision sensor — owns glue trace start/end detection |
| ZATOR LMZ02 encoder | Tube position/length tracking |

Division of responsibility, and why, is documented at the top of
`src/tgis510_v4_14_0.cpp`.

## Build

PlatformIO, Arduino framework — see `platformio.ini`. Board is currently a
generic `esp32-s3-devkitc-1` definition as a placeholder for the actual
ESP32-S3-Zero-M board.

```
pio run
pio run -t upload
pio device monitor
```

## What this implementation does, following the handoff exactly

- I2C at 800kHz (not 1MHz — that silently broke MCP23017 enumeration).
- MLX90640 timing driven off the **measured** 8 FPS, not the nominal 32Hz
  refresh setting.
- `MATRIX_TEMP_MIN_C`/`MAX_C` set to the production range (20–180°C), not
  the 20–40°C bench value.
- NS12 Memory Link: ESC=0x1B for every command (this PT's confirmed
  deviation from the Omron manual's 0x1C), 9600 baud (confirmed against
  the live NS12 Comm. Setting screen; a prior in-code claim that 38400 was
  "confirmed-working" was not hardware-backed and has been corrected),
  non-blocking WM writes, blocking-flush RM reads with a 250ms timeout,
  byte-resyncing `pollRead()`.
- Word Lamp palette clamped to indices 1–9 (index 0 is blank/off).
- 16×8 Word Lamp matrix (`$W700`–`$W827`) as the trusted default; the
  32×24 mode is implemented as a genuinely **column-paced** push (one
  column/WM command, spaced by `COLUMN_WRITE_INTERVAL_MS`) behind
  `NS12::ENABLE_EXPERIMENTAL_32x24` (default off), with a runtime
  auto-fallback to 16×8 if RM read success rate collapses under load.
- Capture/QC state machine is max-hold (ARMED → SAMPLING → LATCHED), not
  averaging, per the FOV-transit reasoning in the handoff.
- Keyence result arrives on a **direct ESP32 GPIO with a hardware
  interrupt**, not through MCP23017 polling (MCP is only polled in
  Standby/TubeGap, ~20ms cadence — too slow for a signal that must be
  actionable during InspectingTube).
- Keyence trigger pulse is a non-blocking, `micros()`-timed pending-low
  state, not a blocking `digitalWrite` sequence.
- Encoder uses the ESP32 PCNT peripheral, drained into a 64-bit running
  total to avoid 16-bit wraparound.
- Serial diagnostic commands: `S/W/I/G/F` (force state), `1`–`6` (toggle
  MCP outputs), `M` (test pattern), `C`/`R` (force capture/rearm), `B`
  (baseline, placeholder), `X` (frame dump).

## Open placeholders (unresolved, from the handoff — need real hardware)

These are marked `PLACEHOLDER` at their definition in
`src/tgis510_v4_14_0.cpp`:

1. **GPIO assignments** — `ENCODER_PULSE_PIN`, `PRESENCE_SENSOR_PIN`,
   `KEYENCE_TRIGGER_PIN`, and the newly-added `KEYENCE_RESULT_PIN` are not
   bench-verified against the physical board silkscreen.
2. `ENCODER_COUNTS_PER_MM` — blocked on confirming the ZATOR LMZ02 encoder
   PPR.
3. `PRESENCE_TO_MLX_DISTANCE_MM`, `PRESENCE_TO_KEYENCE_DISTANCE_MM`.
4. Keyence result pulse polarity (`KEYENCE_RESULT_ACTIVE_LEVEL`).
5. Confirm `MATRIX_TEMP_MAX_C` (180.0°C) is correct before running against
   real hot melt.
6. `CAPTURE_TRIGGER_TEMP_C` (30.0°C) — a guess; consider a frame-to-frame
   delta spike instead of an absolute threshold if unreliable.
7. `CAPTURE_SAMPLE_COUNT` (4 frames) and the glue-strip column ranges in
   `StripZone` — none of this has been validated at real line speed
   (200 m/min assumption) or against a real tube.
8. The NS12 Memory Link FCS checksum (`computeFcs`, 8-bit XOR) is a
   best-effort guess — this PT already has one undocumented protocol
   deviation (the ESC byte), so treat the checksum as unverified until
   confirmed against real WM/RM traffic.

## Immediate next actions (priority order, per the handoff)

Items 1–5 require physical access to the hardware and cannot be done from
here:

1. Bench-verify GPIO 4/5/6/7 against the physical silkscreen before more
   soldering.
2. Scope-verify the Keyence trigger pulse width (the non-blocking
   pending-low fix is implemented; it still needs a scope check).
3. Confirm the ZATOR LMZ02 encoder PPR → populate `ENCODER_COUNTS_PER_MM`.
4. Confirm the mounted lens variant + measure real tube length along the
   travel axis.
5. Confirm actual line speed vs. the 200 m/min assumption.

Items handled in this rewrite:

6. Baud constant (38400), version banner (single `FW_VERSION` source of
   truth), stale trigger-temp comment, and dead local variables are all
   fixed by construction in this clean-room version — verify this actually
   matches what's flashed once diffed against the real file.
7. The reintroduced 32×24 mode is now genuinely column-paced (one column
   per WM command) with a runtime RM-failure auto-fallback to 16×8 — still
   needs monitoring under real traffic load, as flagged in the handoff.

## Known structural limitation

**Single-tube-in-flight only.** The position-projection model breaks if
tube gap < presence-sensor-to-farthest-station distance. Needs production
validation before trusting on the real line.

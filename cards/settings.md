# settings

Where every configurable value lives, how live gains are stored, and the rules for adding or changing one. Values current as of 2026-10-04 (`esp32_cube_enc`, development = main code).

## Three kinds

| Kind | Source | Changed by | Persisted |
|------|--------|------------|-----------|
| Live gains | `GAIN_DEFS` table, `web_interface.cpp` | dashboard Apply (RAM), Save (`gains_save`) | EEPROM at `GAINS_EEPROM_ADDR` 32, magic `GAINS_ID` 0x6D |
| Saved state | `OffsetsObj offsets` (calibration), `trimX/trimY` | calibration commands; auto-trim; `trim_reset` | offsets at EEPROM 0 (`ID == 96`); trim inside the gains record |
| Build settings | `#define`s in `ESP32.h`, a few globals in `esp32_cube_enc.cpp`, `platformio.ini` | edit + rebuild + flash | firmware |

## Live gains (`GAIN_DEFS`, order matters)

| # | Name | Default | Range | Used in |
|---|------|---------|-------|---------|
| 0 | `K1` | 180 | 0–500 | vertex `pwm_X/Y`: angle |
| 1 | `K2` | 30 | 0–200 | vertex: filtered tilt rate |
| 2 | `K3` | 1.6 | 0–50 | vertex: `speed_X/Y` from wheel speeds |
| 3 | `K4` | 0.008 | 0–1 | vertex: `motors_speed_X/Y` (integrated) |
| 4 | `zK2` | 8 | 0–200 | `pwm_Z`: spin-rate error |
| 5 | `zK3` | 0.30 | 0–50 | `pwm_Z`: wheel sum `motors_speed_Z` |
| 6 | `eK1` | 190 | 0–500 | edge `pwm_X` (motor 3): angle |
| 7 | `eK2` | 31 | 0–200 | edge: tilt rate |
| 8 | `eK3` | 2.5 | 0–50 | edge: `motor3_speed` |
| 9 | `eK4` | 0.014 | 0–1 | edge: integrated |
| 10 | `tK` | 0 | −0.5–0.5 | auto-trim rate; negative flips sign |
| 11 | `zK1` | 1.0 | 0–10 | heading hold, °/s per ° error |
| 12 | `vNom` | 0 | 0–13 | battery compensation, 0 = off |
| 13 | `autoArm` | 0 | 0–1 | ≥ 0.5 arms at end of `setup()` |
| 14 | `zK3s` | 1.0 | 0–1 | `zK3` scale while a spin is commanded |

`NUM_GAINS` = 15. The dashboard builds its form from `GET /api/gains`, so a row in the table is all the UI needs.

EEPROM map (128 bytes, NVS-backed, survives flashing): 0–27 `OffsetsObj` (ID 96, vertex then edge accel offsets); 32–47 `GainsObj` header (ID 0x6D, count, trimX, trimY); 48 + 4·i gain `i` (bytes 48–107 for 15 gains). Everything else (armed, spin/turn state, guard, battery state, gyro bias, all `#define`s) is not stored. `docs/settings.md` has the per-item table.

## Rules when touching gains

- **Append only.** `loadGains()` matches saved values to `GAIN_DEFS` by position and reads `min(stored count, NUM_GAINS)`. New rows go at the end with `NUM_GAINS` bumped; no `GAINS_ID` change needed. Reordering or inserting needs a `GAINS_ID` bump (invalidates saved gains).
- Removing the last row is safe: a longer saved record is truncated on load (nudge-to-spin's `nudgeDeg` was removed this way on 2026-10-04).
- `static_assert`s check `NUM_GAINS` against the table and that `GainsObj` fits in `EEPROM_SIZE` (128; with 15 gains the record ends at byte 108).
- `tools/test_estimator.py::test_gain_table_appends_only` pins the first 12 names and checks `NUM_GAINS` against the table.
- Out-of-range or NaN saved values are skipped on load, leaving the default.
- Save is refused while `balancingActive()` (NVS write stalls the loop, encoder ISRs not in IRAM).
- Default in `GAIN_DEFS` (`def`, used by Restore Defaults) must match the global's initial value in `esp32_cube_enc.cpp`.

## Build settings that are measured per cube

Do not "clean up" these to round or upstream values; each was measured on the user's cube.

| Setting | Value | Measured how |
|---------|-------|--------------|
| motor pin groups | 1 = D4 group, 2 = D5, 3 = D15 | motors_test + edge/vertex balance tests |
| `IMU_MOUNT` | 1 (upright holder) | raw chip X read −1 g on the vertex |
| `ACC_OFFSET_X/Y/Z` | −77, −85, 6255 | three face readings, `tools/accel_offsets.py` |
| `BATT_ADC_PER_VOLT` | 225 | 12.0 V pack read raw 2700, `battery_test` |
| `upload_speed` | 115200 | 460800 fails on this board (serial noise) |

## Build settings chosen from traces

| Setting | Value | Evidence |
|---------|-------|----------|
| `YAW_RATE_MAX`, `YAW_TURN_RATE` | 20 °/s | ~19 °/s real held; ~24 °/s fell after a jolt |
| `YAW_WHEEL_LIMIT` / `_RESUME` | 70 / 30 (average signed wheel speed) | replay: stops a 20 °/s spin after ~27 s |
| `YAW_PWM_MAX` | 60 | spin effort used −8…+4 in practice; cap is a safety bound |
| `YAW_ACCEL` | 30 °/s² | ramp seen working in traces |
| `BATT_*` | warn 10.5 / clear 10.7 / cutoff 9.9 ×4 / rearm 10.8 / present 6.0 V | 3S LiPo per-cell limits; USB back-feeds ~4 V |

Other build settings (sensor ranges, `loop_time` 15, `Gyro_amount` 0.996, `alpha` 0.7, `TRIM_MAX` 3, PWM 20 kHz/8 bit, `TRACE_LEN` 336, Wi-Fi name/password, mDNS name) are upstream or development-branch values, untouched by tuning.

## Hard-coded thresholds (not settings)

In `functions.cpp`: vertex latch raw `|AcX|` < 2000 and acc tilt < 0.4°; edge latch raw `|AcX|` 7000–10000 and tilt < 0.3°; fall at 7°; capture windows `|AcX|,|AcY|` < 2000 (vertex) and 7000–10000 / < 2000 (edge). These assume the firmware frame after `ACC_OFFSET_*` and `IMU_MOUNT`.

## Human-facing reference

`docs/settings.md` explains each item for builders; keep the two in step when a gain or define is added, removed or re-tuned.

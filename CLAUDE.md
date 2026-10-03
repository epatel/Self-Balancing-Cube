# Self-Balancing-Cube

ESP32 firmware for a Cubli-style cube that balances on a vertex or an edge using three
reaction wheels (Nidec 24H brushless motors) and an MPU6050 IMU. This is a fork of ReM-RC's
project; the `development` branch adds a Wi-Fi dashboard, safety arming and a PlatformIO
build. Keep diffs small and local, and match the existing heavily-commented style.

## Cards

Load on demand; each card is self-contained.

- [firmware-assessment](cards/firmware-assessment.md) — comparing sketches or branches, choosing which firmware fits a physical build, working out motor order or sensor orientation, or looking for known defects before changing control code
- [development-branch-assessment](cards/development-branch-assessment.md) — deciding which branch or commit to flash, merging or porting from `development`, anything about the Wi-Fi dashboard, arming, PlatformIO build, or why the cube does not hold with default gains

## Layout

| Path | What it is |
|------|------------|
| `esp32_cube_enc/ESP32.h` | Pin map, constants, structs, `extern` globals and all cross-file prototypes |
| `esp32_cube_enc/esp32_cube_enc.cpp` | Storage for the globals, `setup()`, `loop()` (the control loop) |
| `esp32_cube_enc/functions.cpp` | IMU read and angle estimate, motor mixing and drive, encoder ISRs, calibration, trace, USB serial commands |
| `esp32_cube_enc/web_interface.cpp` | Wi-Fi AP, HTTP API, gain table and EEPROM save/load, inlined dashboard page |
| `motors_test/motors_test.ino` | Standalone Arduino sketch for bring-up; duplicates the pin map and motor helpers |
| `battery_test/battery_test.ino` | Standalone sketch: holds motors stopped, prints battery voltage, derives the firmware's VBAT divider from a typed multimeter reading |
| `tools/` | `gzip_dashboard.py` (pre-build step), `test_estimator.py` (math and source checks) |
| `platformio.ini` | Build config; `src_dir` is `esp32_cube_enc` |

These are separate translation units, not concatenated `.ino` files: anything called across
files needs a prototype in `ESP32.h`, and a global must be defined once in
`esp32_cube_enc.cpp` and declared `extern` in the header.

The legacy sketches (`ESP32_cube`, `arduino_cube`) are not on this branch; they are on
`main`.

## Build, flash, test

```
pio run                          # build
pio run -t upload                # flash
pio device monitor               # 115200 baud
python tools/test_estimator.py   # after touching angle_calc() or the heading loop
```

- Needs arduino-esp32 **core 3.x** (`ledcAttach(pin, freq, res)`, `ledcWrite(pin, …)`).
  The platform is pinned to pioarduino `55.03.311` in `platformio.ini`; later releases
  require PlatformIO Core ≥ 6.2.0.
- `esp32_cube_enc/dashboard_gz.h` is generated from the `DASHBOARD_HTML` literal by the
  pre-build script and is git-ignored. Edit the literal, never the header. The Arduino IDE
  cannot build this firmware.
- `motors_test` is built from the Arduino IDE with the esp32 board package 3.x.
- `test_estimator.py` models the arithmetic in Python and greps the source; it does not
  run firmware. Nothing can be verified without hardware — say so rather than claiming a
  change works.

## How the control loop works

Runs every `loop_time` (15 ms), using the measured `dt` clamped to 5–45 ms:

1. `Tuning()` — two-byte commands from USB serial.
2. `angle_calc(dt)` — reads the MPU6050, applies accelerometer offsets, fuses gyro and
   accelerometer (`Gyro_amount` = 0.996), and latches `vertical_vertex` / `vertical_edge`
   near a balancing point. Balance is dropped beyond 7°.
3. Encoder counts since the last tick become `motorN_speed`; `threeWay_to_XY()` projects
   them onto the X/Y balance axes.
4. Any pending web command (`web_cmd_pending`) is applied.
5. If `armed`, calibrated and a pose is latched:
   - Vertex: `pwm = K1·(angle − trim) + K2·gyro + K3·wheel_speed + K4·integrated_speed`
     per axis, plus a yaw rate loop (`zK2`, `zK3`) with optional heading hold (`zK1`),
     mixed to three motors by `XYZ_to_threeWay()`.
   - Edge: same law with `eK1..eK4`, driving motor 3 only.
6. Otherwise motors are zeroed and the brake is applied.
7. `traceRecord()` appends one telemetry sample.

`handleWebInterface()` runs at the end of every `loop()` pass, outside the timed block.

## Rules for the web interface

- HTTP handlers never call motor functions. They validate, store a request
  (`web_cmd_pending`, `yaw_rate_request`, `yaw_turn_request`) and return; the control loop
  acts on it.
- Handlers run from `loop()`, so they cannot interrupt a control iteration, but anything
  slow in a handler stalls balancing. Keep responses small and never block.
- A pending STOP is never overwritten by another command.
- Calibration and EEPROM writes are refused while `balancingActive()`.
- New gains are **appended** to `GAIN_DEFS` and `NUM_GAINS` is bumped; `loadGains()`
  matches saved values by position. Do not reorder without bumping `GAINS_ID`.

## Things that are easy to get wrong

- **The cube boots disarmed.** `armed` is `false` at start; nothing balances until ARM
  from the dashboard or `a+` over serial.
- **PWM is inverted**: motors are driven with `255 - abs(sp)`, so a duty of 255 is stopped.
  A floating PWM pin means full drive, which is why `setup()` attaches PWM before anything
  else. Keep that block first.
- **`BRAKE` is active-low**: `HIGH` releases the brake (running), `LOW` brakes.
- **`MotorN_control(0)` is not zero drive**: it adds the measured wheel speed before
  clamping. Deliberately left as is.
- **Encoder ISRs** modify `enc_countN`; the main loop reads and zeroes them without masking
  interrupts, and the ISRs are not in IRAM, so counts are lost across a flash write.
- **EEPROM layout**: calibration offsets at address 0 (`ID == 96`), gains and learned trim
  at address 32 (`GAINS_ID`). Changing `OffsetsObj` invalidates a calibrated cube.
- **Sensor frame**: `angle_calc()` subtracts `ACC_OFFSET_*` (chip axes) and then rotates
  readings into the 2024-holder frame (`IMU_MOUNT`) before anything else uses them, so
  `AcX..GyZ` are never raw chip axes. This cube: `IMU_MOUNT 1` (board upright), chip Z
  accelerometer offset +6255 (0.38 g). A big accel offset looks like a tilt — don't
  "fix" it with a rotation, which also turns the gyro. Measure with
  `tools/accel_offsets.py` (three face readings).
- **Motor numbering** is set by the control geometry: motor 3 alone balances the edge;
  swapping motors 1 and 2 reverses the vertex Y axis. This cube: 1 = D4, 2 = D5, 3 = D15
  pin groups (upstream 1 = D4, 2 = D15, 3 = D5).
- **Gyro scale**: always convert raw rates with `GYRO_LSB_PER_DPS`; never hard-code 131 or
  65.536 (`test_estimator.py` checks this).
- **Gyro offsets are measured at boot** (about 8 s). The cube must sit still during startup.
- **Battery protection** lives in `battCheck()` / `battIndicate()` with thresholds in
  `ESP32.h` (`BATT_*`): warn below 10.5 V, latched cutoff (disarm) below 9.9 V for 4
  checks, re-arm only above 10.8 V, "no battery" below 6 V because USB back-feeds the rail
  to ~4 V. `BATT_ADC_PER_VOLT` (225) is hand-measured per board — use `battery_test`.
  Upstream had `/ 204` and only a buzzer between 8 and 9.5 V.
- **`INT_LED` (GPIO2)** is the low-battery indicator; this cube has no buzzer.
- **Battery compensation**: gain `vNom` (0 = off) makes `battCheck()` set `batt_comp =
  vNom / batt_voltage` (clamped 0.90–1.25), applied in `MotorN_control()` after the
  speed term. Leave it off when comparing tuning changes, or results get confounded.
- **Default gains** are upstream's and predate the gyro-scale fix. Don't change them as a
  side effect of other work.
- **The Wi-Fi password** is a constant in `web_interface.cpp`. Don't commit a real one.

## Commands

USB serial, two bytes: `a+` / `a-` arm and disarm, `c+` start calibration, `c-` capture the
current pose.

HTTP: `GET /api/state`, `GET /api/trace?since=N`, `GET|POST /api/gains`,
`POST /api/command` with `cmd=` one of `stop`, `disarm`, `arm`, `cal_start`, `cal_capture`,
`cal_save`, `gains_save`, `trim_reset`, `yaw` (`rate=`), `turn` (`deg=`), `yaw_free`.

## Docs

`README.md` is the builder-facing guide. The schematic is `schematic.pdf`, with a PNG copy
in `pictures/`. Keep README and this file in sync when commands, calibration steps or the
build change.

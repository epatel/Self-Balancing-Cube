# Self-Balancing-Cube

Arduino firmware for a Cubli-style cube that balances on a vertex or an edge using three
reaction wheels (Nidec 24H brushless motors) and an MPU6050 IMU. This is a fork of ReM-RC's
project; the code style and structure are upstream's, so keep diffs small and local.

## Cards

Load on demand; each card is self-contained.

- [firmware-assessment](cards/firmware-assessment.md) — comparing sketches or branches, choosing which firmware fits a physical build, working out motor order or sensor orientation, or looking for known defects before changing control code
- [development-branch-assessment](cards/development-branch-assessment.md) — deciding which branch or commit to flash, merging or porting from `development`, anything about the Wi-Fi dashboard, arming, PlatformIO build, or why the cube does not hold with default gains

## Layout

Each directory is an independent Arduino sketch. There is no shared code between them —
pin maps, motor helpers and encoder ISRs are copy-pasted per sketch, so a fix in one usually
has to be repeated by hand in the others.

| Sketch | Board | Status | Notes |
|--------|-------|--------|-------|
| `esp32_cube_enc/` | ESP32 (classic) | **current** | Encoders, WS2812B LEDs (FastLED), different IMU orientation |
| `ESP32_cube/` | ESP32 (classic) | legacy | No encoders, 4 calibrated balancing points |
| `arduino_cube/` | Arduino Nano | legacy | Same algorithm as `ESP32_cube`, tuning over hardware `Serial` |
| `motors_test/` | ESP32 (classic) | diagnostic | Spins each motor both ways, checks encoders, prints to Serial |

Within a sketch: the `.h` holds pin defines **and all global state/gains**, the main `.ino`
holds `setup()`/`loop()`, and `functions.ino` holds everything else (Arduino concatenates
`.ino` files, so there are no prototypes or includes between them).

Default to `esp32_cube_enc/` unless the user names another sketch.

## Build and flash

No build system, tests or CI — sketches are built from the Arduino IDE. Nothing can be
verified without hardware; say so rather than claiming a change works.

- ESP32 sketches use `ledcSetup` / `ledcAttachPin` / `ledcWrite(channel, …)`. Those are
  arduino-esp32 **core 2.x** APIs and were removed in core 3.x (replaced by
  `ledcAttach(pin, freq, res)` and `ledcWrite(pin, …)`). Build with a 2.x core, or port the
  PWM calls.
- `BluetoothSerial` is Bluetooth Classic: original ESP32 only, not S2/S3/C3.
- `esp32_cube_enc` additionally needs the FastLED library.
- Serial monitor is 115200 baud everywhere.

## How the control loop works (`esp32_cube_enc`)

Runs every `loop_time` (15 ms):

1. `Tuning()` — reads two-byte commands from Bluetooth (`ESP32-Cube`).
2. `angle_calc()` — reads MPU6050 over I2C, applies accelerometer offsets, fuses gyro and
   accelerometer with a complementary filter (`Gyro_amount` = 0.996), and sets
   `vertical_vertex` / `vertical_edge` when the cube is held near a balancing point.
3. Encoder counts since the last tick become `motorN_speed`; `threeWay_to_XY()` projects
   them onto the X/Y balance axes.
4. Vertex mode: `pwm = K1·angle + K2·gyro + K3·wheel_speed + K4·integrated_speed` per axis,
   plus a yaw term (`zK2`, `zK3`), mixed back to three motors by `XYZ_to_threeWay()`.
   Edge mode: same law with `eK1..eK4`, driving motor 3 only.
5. Otherwise motors are zeroed and the brake is applied.

Balance is dropped when the angle exceeds 7°. A 2 s side loop checks battery voltage and
nags (BT message + blinking LEDs) while uncalibrated.

## Things that are easy to get wrong

- **PWM is inverted**: motors are driven with `255 - abs(sp)`, so a duty of 255 is stopped.
- **`BRAKE` is active-low**: `HIGH` releases the brake (running), `LOW` brakes.
- **Motor numbering is not pin order**: channels are PWM1→CH1, PWM2→CH0, PWM3→CH2.
- **Encoder ISRs** modify `enc_countN` (`volatile`); the main loop reads and zeroes them
  without disabling interrupts. Upstream behaviour — don't "fix" it unasked.
- **EEPROM layout differs per sketch.** `esp32_cube_enc` stores raw accelerometer offsets
  for vertex + one edge with magic `ID == 96`. The legacy sketches store angle offsets for
  vertex + three edges with `ID1..ID4 == 99`. Changing `OffsetsObj` invalidates a
  calibrated cube; bump the magic if you do.
- **Axes differ between sketches.** The IMU is mounted differently in `esp32_cube_enc`
  (X-angle integrates `GyX`) than in the legacy sketches (X-angle integrates `GyZ`). Don't
  port angle math between them verbatim.
- **Gyro offsets are measured at boot** (3 × 512 samples, about 8 s). The cube must sit
  still during startup.
- **Battery divider is hand-tuned** (`/ 204`, `/ 207`, or `bat_divider`); the buzzer sounds
  between 8 V and 9.5 V. The constant depends on the builder's resistors.
- Gains (`K1..K4`, `eK1..eK4`, `zK2`, `zK3`) are tuned to the physical build. Don't change
  them as a side effect of other work.

## Serial / Bluetooth commands

Two bytes: a parameter letter followed by `+` or `-`.

| Command | `esp32_cube_enc` | `ESP32_cube` | `arduino_cube` |
|---------|------------------|--------------|----------------|
| `c+` / `c-` | start calibration / capture point | same | same |
| `p±` | — | `K1` ± 1 | `pGain` ± 1 |
| `i±` | — | `K2` ± 0.05 | `iGain` ± 0.05 |
| `s±` | — | `K3` ± 0.005 | `sGain` ± 0.005 |
| `b±` | — | — | `bat_divider` ± 1 |

Live gain tuning does not exist in `esp32_cube_enc`; gains there are compile-time only.
Tuned values are never persisted in any sketch — only calibration offsets are.

## Docs

`README.md` is the builder-facing guide (hardware, wiring, calibration, videos). Schematics
are `schematic.pdf` (ESP32) and `arduino_schematic.pdf` (Nano), with PNG copies in
`pictures/`. Keep README and this file in sync when commands or calibration steps change.

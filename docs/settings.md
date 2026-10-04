# Settings reference

Everything about the cube's behaviour that can be changed, where it lives, and what it
does. There are three kinds:

| Kind | Where | How to change | Survives a reboot? |
|------|-------|---------------|--------------------|
| **Live gains** | dashboard → Gains panel | edit, **Apply** (takes effect at once), **Save** (writes to flash) | only after Save |
| **Saved state** | calibration and learned trim | Calibration panel, Reset trim, Save | yes |
| **Build settings** | `esp32_cube_enc/ESP32.h` and other sources | edit, rebuild, flash | yes (part of the firmware) |

Save writes **all** gains plus the current trim in one go, and is refused while the cube is
balancing (the button is greyed out then; disarm or lay the cube down). Values typed into
the form are applied first, so pressing Save alone is enough. The pill at the top of the
Gains panel is reported by the firmware itself: **ALL GAINS SAVED** (green) only when the
running gains are the ones stored in flash, **UNSAVED CHANGES** (amber) otherwise. The
learned trim is not part of that check, since auto-trim changes it continuously.
**Restore Defaults** puts the gains back to the values below without saving.

## All settings at a glance

✅ = stored in EEPROM (flash) and restored at every boot. ❌ = not stored: build settings
come with the firmware, runtime state starts fresh at each boot.

| Setting | Kind | EEPROM | Address (bytes) | Written when |
|---------|------|:------:|-----------------|--------------|
| Calibration ID (`96` = calibrated) | saved state | ✅ | 0–3 | calibration finishes (edge captured), or Save in the Calibration panel |
| Vertex pose `acXv`, `acYv`, `acZv` | saved state | ✅ | 4–15 | same |
| Edge pose `acXe`, `acYe`, `acZe` | saved state | ✅ | 16–27 | same |
| Gains record ID (`0x6D`) and count | – | ✅ | 32–39 | gains **Save** |
| Learned trim X, Y | saved state | ✅ | 40–47 | gains **Save** (whatever the trim is at that moment) |
| `K1` `K2` `K3` `K4` | live gain | ✅ | 48–63 | gains **Save** |
| `zK2` `zK3` | live gain | ✅ | 64–71 | gains **Save** |
| `eK1` `eK2` `eK3` `eK4` | live gain | ✅ | 72–87 | gains **Save** |
| `tK` (auto-trim) | live gain | ✅ | 88–91 | gains **Save** |
| `zK1` (heading hold) | live gain | ✅ | 92–95 | gains **Save** |
| `vNom` (battery compensation) | live gain | ✅ | 96–99 | gains **Save** |
| `autoArm` (demo mode) | live gain | ✅ | 100–103 | gains **Save** |
| `zK3s` (spin lag) | live gain | ✅ | 104–107 | gains **Save** |
| Armed / disarmed | runtime | ❌ | – | boots disarmed, unless `autoArm` = 1 |
| Spin slider rate, turn target, heading | runtime | ❌ | – | reset at boot and whenever the cube falls or is disarmed |
| Wind-up guard, battery state, gyro bias | runtime | ❌ | – | recomputed at boot (gyro bias measured during the LED-on phase) |
| Motor pins, `IMU_MOUNT`, `ACC_OFFSET_*` | build | ❌ | – | change in `ESP32.h`, rebuild, flash |
| `BATT_*` thresholds, `BATT_ADC_PER_VOLT` | build | ❌ | – | same |
| `YAW_*` spin limits, `TRIM_MAX` | build | ❌ | – | same |
| Sensor ranges, `loop_time`, `Gyro_amount`, `alpha`, PWM, `TRACE_LEN` | build | ❌ | – | same |
| Wi-Fi name / password, mDNS name | build | ❌ | – | `web_interface.cpp`, rebuild, flash |

The EEPROM area is 128 bytes (bytes 28–31 and 108–127 unused). Flashing new firmware does
**not** erase it, so calibration and saved gains survive updates. A gain added to the end of
the list by a newer firmware starts at its default until the next Save; one removed from
the end is simply ignored.

## Live gains

Values are those of the firmware as built; ranges are what the dashboard accepts.

### Balancing on the vertex

| Gain | Default | Range | What it does |
|------|---------|-------|--------------|
| `K1` | 180 | 0–500 | Tilt angle → wheel effort. Main "stiffness". Too low: slow wander and falls; too high: fast shaking. |
| `K2` | 30 | 0–200 | Tilt rate → effort. Damping. Raise it if the cube oscillates. |
| `K3` | 1.6 | 0–50 | Wheel speed → effort. Keeps the wheels from running away. |
| `K4` | 0.008 | 0–1 | Accumulated wheel speed → effort. Slowly pulls the wheels back to rest. |

### Spinning around the vertical (vertex only)

| Gain | Default | Range | What it does |
|------|---------|-------|--------------|
| `zK2` | 8 | 0–200 | Spin-rate error → effort on all three wheels. Holds the cube still, or at the commanded spin rate. |
| `zK3` | 0.3 | 0–50 | Sum of wheel speeds → effort. Unwinds the wheels after a spin. During a spin it also makes the cube lag the command by about `zK3 × wheel sum ÷ zK2`. |
| `zK3s` | 1 | 0–1 | Multiplies `zK3` while a spin is commanded. **0.25** cuts the spin lag from ~7 to ~1.5 °/s (tested); the wind-up guard then ends a long spin a little sooner. 1 = off. |
| `zK1` | 1 | 0–10 | Heading hold: °/s of spin commanded per ° of heading error. Used by the turn and Hold heading buttons. 0 turns heading hold off. |

### Balancing on the edge

| Gain | Default | Range | What it does |
|------|---------|-------|--------------|
| `eK1` | 190 | 0–500 | Tilt angle → effort (motor 3 only). |
| `eK2` | 31 | 0–200 | Tilt rate → effort. |
| `eK3` | 2.5 | 0–50 | Motor 3 speed → effort. |
| `eK4` | 0.014 | 0–1 | Accumulated motor 3 speed → effort. |

### Helpers and modes

| Gain | Default | Range | What it does |
|------|---------|-------|--------------|
| `tK` | 0 | −0.5–0.5 | **Auto-trim** rate. Learns the true balance point from steady wheel speed and shifts the setpoint (at most ±3°). 0 = off; the dashboard's Auto-trim button sets 0.005. If the trim runs to ±3° and balancing gets worse, use a negative value. The learned trim is stored by Save. |
| `vNom` | 0 | 0–13 | **Battery compensation.** 0 = off. Otherwise the pack voltage the gains were tuned at (about 11.5 V): motor commands are scaled by `vNom ÷ battery voltage` (limited to ×0.90–×1.25) so the cube behaves the same as the pack drains. |
| `autoArm` | 0 | 0–1 | **Demo mode.** 1 = arm automatically at the end of boot, so the cube balances when stood up without a phone. Takes effect at the next boot after Save. |

## Saved state

| What | Set by | Notes |
|------|--------|-------|
| Calibration (vertex and edge poses) | Calibration panel: Start, Capture vertex, Capture edge | Saved automatically after the edge capture. Redo after changing the sensor mount, `ACC_OFFSET_*` or `IMU_MOUNT`. |
| Learned trim (X, Y) | auto-trim while balancing; **Reset trim** clears it | Stored with the gains when you press Save, and printed at boot ("Trim X … Y …"). |

## Build settings (`esp32_cube_enc/ESP32.h` unless noted)

Change, rebuild (`pio run`) and flash (`pio run -t upload`).

### This cube's hardware (measured or wired, specific to one build)

| Setting | Value | Meaning |
|---------|-------|---------|
| `DIR1/PWM1/ENC1_*` | 4 / 32 / 35, 33 | Motor 1 pin group (wheel C on this cube) |
| `DIR2/PWM2/ENC2_*` | 5 / 18 / 16, 17 | Motor 2 pin group (wheel B) |
| `DIR3/PWM3/ENC3_*` | 15 / 25 / 13, 14 | Motor 3 pin group (wheel A, the one that balances the edge) |
| `BRAKE`, `BUZZER`, `VBAT`, `INT_LED`, `LED_PIN` | 26, 27, 34, 2, 19 | Brake (LOW = braking), buzzer, battery sense, board LED, WS2812 data |
| `IMU_MOUNT` | 1 | 0 = 2024 sensor holder; 1 = original upright holder (axes rotated in firmware) |
| `ACC_OFFSET_X/Y/Z` | −77, −85, 6255 | Accelerometer zero offsets on the chip's axes; measure with `tools/accel_offsets.py` |
| `BATT_ADC_PER_VOLT` | 225 | Battery divider constant; measure with the `battery_test` sketch |

Motor numbering follows the control geometry: motor 3 alone balances the edge, and if the
vertex throws the cube sideways, motors 1 and 2 are swapped.

### Battery protection

| Setting | Value | Meaning |
|---------|-------|---------|
| `BATT_WARN_V` / `BATT_WARN_CLEAR_V` | 10.5 / 10.7 V | Low warning on / off (slow blink) |
| `BATT_CUTOFF_V`, `BATT_CUTOFF_COUNT` | 9.9 V, 4 checks | Disarm after 2 s below this (fast blink) |
| `BATT_REARM_V` | 10.8 V | Arming refused after a cutoff until the pack reads this |
| `BATT_PRESENT_V` | 6.0 V | Below this: no battery (USB only), no warnings |
| `BATT_CHECK_MS` | 500 ms | Check period |
| `BATT_COMP_MIN/MAX` | 0.90 / 1.25 | Limits of the `vNom` compensation factor |

### Spinning

| Setting | Value | Meaning |
|---------|-------|---------|
| `YAW_RATE_MAX` | 20 °/s | Slider and API limit. ~24 °/s fell in testing; don't raise without a trace that holds. |
| `YAW_TURN_RATE` | 20 °/s | Fastest rate the turn / heading loop asks for |
| `YAW_ACCEL` | 30 °/s² | How fast the commanded spin rate may change |
| `YAW_PWM_MAX` | 60 | Most spin effort added to a motor (of 255); balancing always gets priority |
| `YAW_WHEEL_LIMIT` / `YAW_WHEEL_RESUME` | 70 / 30 | Wind-up guard: average wheel speed (counts per tick) that stops a spin / allows the next |
| `YAW_TURN_MAX` | 720° | Largest single turn command |

### Sensor and control loop

| Setting | Where | Value | Meaning |
|---------|-------|-------|---------|
| `accSens`, `gyroSens` | ESP32.h | 0, 0 | ±2 g, ±250 °/s ranges |
| `loop_time` | esp32_cube_enc.cpp | 15 ms | Control period (measured `dt` is clamped to 5–45 ms) |
| `Gyro_amount` | esp32_cube_enc.cpp | 0.996 | Complementary filter: gyro weight (time constant ~3.7 s) |
| `alpha` | esp32_cube_enc.cpp | 0.7 | Smoothing of the tilt-rate signal used by K2 / eK2 |
| `TRIM_MAX` | ESP32.h | 3° | Auto-trim limit |
| `BASE_FREQ`, `TIMER_BIT` | ESP32.h | 20 kHz, 8 bit | Motor PWM |
| `TRACE_LEN` | ESP32.h | 336 samples | Telemetry ring (~5 s) |

### Fixed in code (`functions.cpp`, `angle_calc()` / `calCapture()`)

These are not settings, but are worth knowing when the cube "won't start":

| Rule | Value |
|------|-------|
| Vertex is recognised when | raw `|AcX|` < 2000 and both tilt angles within 0.4° |
| Edge is recognised when | raw `|AcX|` 7000–10000 and tilt within 0.3° |
| Balancing stops when | tilt exceeds 7° |
| Vertex capture accepted when | raw `|AcX|` and `|AcY|` < 2000 |
| Edge capture accepted when | raw `|AcX|` 7000–10000 and `|AcY|` < 2000 (after a vertex) |

### Wi-Fi and build

| Setting | Where | Value |
|---------|-------|-------|
| `WIFI_NAME` / `WIFI_PASSWORD` | web_interface.cpp | `Cube-Control` / change before use (at least 8 characters) |
| `MDNS_NAME` | web_interface.cpp | `cube` → `http://cube.local` |
| `platform` | platformio.ini | pioarduino 55.03.311 (later releases need PlatformIO Core 6.2+) |
| `upload_speed` / `monitor_speed` | platformio.ini | 115200 / 115200 |

## Commands (not settings, for completeness)

Dashboard buttons and `POST /api/command` with `cmd=`: `stop`, `disarm`, `arm`,
`cal_start`, `cal_capture`, `cal_save`, `gains_save`, `trim_reset`, `yaw` (`rate=`),
`turn` (`deg=`), `yaw_free`. Over USB serial at 115200: `a+` / `a-` arm and disarm,
`c+` start calibration, `c-` capture the current pose.

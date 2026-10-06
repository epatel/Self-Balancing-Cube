// Per-cube settings.  One firmware source serves several physical cubes;
// everything here was measured or wired differently on each one.  The cube is
// chosen by the PlatformIO environment, which defines CUBE:
//
//     pio run -e cube1 -t upload      (default when no -e is given)
//     pio run -e cube2 -t upload
//
// The boot log and the Wi-Fi name say which cube a firmware was built for.
//
// What each setting means:
//
// Motor pins - motor numbering follows the control geometry, not the wiring.
//   Motor 3 is the wheel on the firmware's X axis (it balances the edge
//   alone); motors 1 and 2 share the Y axis, and swapping them reverses it.
//
// IMU_MOUNT - how the MPU6050 board is mounted.  The firmware works in the
//   frame of upstream's 2024 sensor holder: chip Z points down through the
//   balancing vertex, chip X runs (seen from above) along motor 3's wheel.
//     0 = 2024 holder, readings used as-is.
//     1 = original holder, board upright: chip X points down and chip Z points
//         horizontally toward motor 3's wheel.  Readings are rotated into the
//         2024 frame as they are read (X = -Z, Y = Y, Z = X), so everything
//         downstream - pose detection, calibration, the dashboard's raw
//         values - sees the 2024 frame.
//
// ACC_OFFSET_* - accelerometer zero offsets in raw counts on the CHIP's own
//   axes, removed before the IMU_MOUNT rotation.  Every MPU6050 has its own.
//   A large offset looks like the sensor being tilted, but correcting it with
//   a rotation also turns the gyro and leaks the cube's spin into the balance
//   axes.  Measure with tools/accel_offsets.py --cube N: lay the cube still
//   on its three wheel faces and give it the dashboard's raw values.
//
// BATT_ADC_PER_VOLT - analogRead(VBAT) counts per battery volt; depends on
//   the cube's resistor divider.  Measure with battery_test/ or from the
//   "Battery ... V" boot line against a multimeter.

#if !defined(CUBE)
#error "No cube selected: build with pio run -e cube1 (or -e cube2)"

#elif CUBE == 1
// --- Cube1: the first build -------------------------------------------
#define CUBE_NAME       "Cube1"
#define CUBE_WIFI_NAME  "Cube1-Control"
#define CUBE_MDNS_NAME  "cube1"            // http://cube1.local

// Motor 1 = D4 pin group, 2 = D5, 3 = D15 (upstream: 1 = D4, 2 = D15,
// 3 = D5).  Verified 2026-10-03: motors_test showed each group's encoder
// belongs to its motor, the edge balanced on motor 3, and with 1 and 2 the
// other way round the vertex threw the cube over sideways.
#define DIR1        4
#define ENC1_1      35
#define ENC1_2      33
#define PWM1        32
#define PWM1_CH     1

#define DIR2        5
#define ENC2_1      16
#define ENC2_2      17
#define PWM2        18
#define PWM2_CH     2

#define DIR3        15
#define ENC3_1      13
#define ENC3_2      14
#define PWM3        25
#define PWM3_CH     0

// On the vertex the raw chip axes read X -16290, Y ~100: upright board.
#define IMU_MOUNT 1

// 2026-10-03: chip Z reads +6255 (0.38 g) too high; with that removed the
// faces come out 90.0° apart and the board's real tilt is 1.6°.
#define ACC_OFFSET_X    -77
#define ACC_OFFSET_Y    -85
#define ACC_OFFSET_Z   6255

// 12.0 V pack read raw 2700 (upstream's 204 read 13.2 V).
#define BATT_ADC_PER_VOLT  225.0f

#elif CUBE == 2
// --- Cube2: second build (2026-10-06) ---------------------------------
#define CUBE_NAME       "Cube2"
#define CUBE_WIFI_NAME  "Cube2-Control"
#define CUBE_MDNS_NAME  "cube2"            // http://cube2.local

// ASSUMED the same wiring as Cube1 until verified with motors_test and an
// edge and vertex balance test (edge must balance on motor 3; if the vertex
// throws the cube sideways, swap the motor 1 and 2 groups).
#define DIR1        4
#define ENC1_1      35
#define ENC1_2      33
#define PWM1        32
#define PWM1_CH     1

#define DIR2        5
#define ENC2_1      16
#define ENC2_2      17
#define PWM2        18
#define PWM2_CH     2

#define DIR3        15
#define ENC3_1      13
#define ENC3_2      14
#define PWM3        25
#define PWM3_CH     0

// On the vertex the raw chip axes read X -15877, Y 115: upright board, the
// same holder as Cube1.
#define IMU_MOUNT 1

// 2026-10-06, three wheel faces: faces 90.5 / 89.7 / 89.9° apart after
// correction, 1 g = 16377 counts, real board tilt 1.6°.  The vertex then
// reads X -494, Y 176, inside the recognition window.
#define ACC_OFFSET_X    575
#define ACC_OFFSET_Y    -61
#define ACC_OFFSET_Z   1261

// PLACEHOLDER (Cube1's value): Cube2 has a different divider.  Measure it
// before relying on the battery warning and cutoff.
#define BATT_ADC_PER_VOLT  225.0f

#else
#error "Unknown CUBE number: add its block to cube_config.h"
#endif

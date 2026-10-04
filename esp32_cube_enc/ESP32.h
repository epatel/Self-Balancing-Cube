// ESP32 pin assignments.  Keeping these in one file makes it easier to
// adapt the firmware if the PCB wiring changes.
//
// This header is shared across separate .cpp translation units (rather than
// being concatenated Arduino-.ino-style into one file), so it only declares
// things: types, macros, and `extern` globals.  The actual storage for the
// globals below is defined once, in esp32_cube_enc.cpp.
#include <Arduino.h>
#include <FastLED.h>

#define BUZZER      27
#define VBAT        34
#define INT_LED     2

#define BRAKE       26       // Motor-driver brake/enable input

// Motor numbering follows the control geometry, not the wiring.  Motor 3 is
// the wheel on the firmware's X axis (the one that balances the edge alone);
// motors 1 and 2 share the Y axis, and swapping them reverses it.
// On this cube motor 1 is the D4 pin group, motor 2 D5 and motor 3 D15;
// upstream's wiring had 1 = D4, 2 = D15, 3 = D5.  Verified 2026-10-03:
// motors_test showed each group's encoder belongs to its motor, the edge
// balanced on motor 3, and with 1 and 2 the other way round the vertex
// threw the cube over sideways.
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

#define TIMER_BIT   8        // 8-bit PWM: values range from 0 to 255
#define BASE_FREQ   20000    // 20 kHz PWM, above the audible motor range

#define MPU6050       0x68   // I2C device address
#define ACCEL_CONFIG  0x1C   // Accelerometer configuration register
#define GYRO_CONFIG   0x1B   // Gyroscope configuration register
#define PWR_MGMT_1    0x6B   // Power-management register
#define PWR_MGMT_2    0x6C

#define accSens 0            // 0 = ±2 g, 1 = ±4 g, 2 = ±8 g, 3 = ±16 g
#define gyroSens 0           // 0 = ±250°/s, 1 = ±500°/s, 2 = ±1000°/s, 3 = ±2000°/s

// How the MPU6050 board is mounted.  The firmware works in the frame of
// upstream's 2024 sensor holder: chip Z points down through the balancing
// vertex, chip X runs (seen from above) along motor 3's wheel.
//   0 = 2024 holder, readings used as-is.
//   1 = original holder, board upright: chip X points down and chip Z points
//       horizontally toward motor 3's wheel.  Readings are rotated into the
//       2024 frame as they are read (X = -Z, Y = Y, Z = X), so everything
//       downstream - pose detection, calibration, the dashboard's raw
//       values - sees the 2024 frame.
// This cube (2026-10-03): on the vertex the raw chip axes read X -16290,
// Y ~100, so it uses mount 1.
#define IMU_MOUNT 1

// Accelerometer zero offsets, in raw counts on the CHIP's own axes, removed
// before the IMU_MOUNT rotation.  Some MPU6050s read far from zero on one
// axis; a large offset looks like the sensor being tilted, but correcting it
// with a rotation also turns the gyro and leaks the cube's spin into the
// balance axes.  Measure with tools/accel_offsets.py: lay the cube still on
// three faces that meet at a corner and give it the dashboard's raw values.
// This cube (2026-10-03): chip Z reads +6255 (0.38 g) too high; with that
// removed the faces come out 90.0° apart and the board's real tilt is 1.6°,
// small enough for the calibration capture to absorb.
#define ACC_OFFSET_X    -77
#define ACC_OFFSET_Y    -85
#define ACC_OFFSET_Z   6255

// Raw gyro counts per degree/second, derived from the range selected above
// (131 at ±250°/s, halving with each step up).  Every place that converts a
// raw GyX/GyY/GyZ reading into °/s MUST use this - the angle integration and
// the controller's rate terms used to carry two different hard-coded
// constants (65.536 and 131.0), so the estimator ran at twice the scale of
// the loop that consumed it.
#define GYRO_LSB_PER_DPS (131.0f / (1 << gyroSens))

// Was 64.  The offsets struct occupies bytes 0-27; the tuning gains are
// stored after it, so the allocation needs to be larger.  The ESP32 EEPROM
// library is NVS-backed and expands in place, so existing saved calibration
// data is preserved when this grows.
#define EEPROM_SIZE   128

// Tuning gains are saved after the offsets struct.
#define GAINS_EEPROM_ADDR 32
#define NUM_GAINS         16   // 10 balancing gains, auto-trim rate, heading hold,
                               // battery-compensation nominal voltage, auto-arm,
                               // zK3 scale while spinning, nudge-to-spin turn
// Marks a valid saved gain set.  Bump it only if the ORDER of GAIN_DEFS
// changes or the fixed fields below move - not merely because a gain was
// added or removed.  The record carries its own count and keeps the
// variable-length array last, so the loader reads whichever is smaller of
// the stored and current gain counts and the fixed fields stay put.
// History: 0x6A original 10 gains, 0x6B added tK, 0x6C added the learned
// trim, 0x6D reordered to the self-describing layout used now.
#define GAINS_ID          0x6D
struct GainsObj {
  int   ID;
  int   count;      // number of gains in v[] when this record was written
  // The learned balance point is saved with the gains so the cube applies it
  // immediately at boot, instead of spending the first minute re-learning it
  // and drifting in the meantime.  It is a physical property of the cube, so
  // it barely changes between sessions.
  float trimX;
  float trimY;
  float v[NUM_GAINS];   // must stay LAST - see the note above
};

#define LED_PIN       19     // Pin that connects to WS2812B
#define NUM_PIXELS    3      // The number of LEDs (pixels) on WS2812B

// Complementary-filter weight.  The gyro responds quickly, while the
// accelerometer slowly corrects gyro drift using the direction of gravity.
extern float Gyro_amount;

// These flags describe the cube's current operating state.
// Balancing is allowed only after calibration and when a valid upright pose
// has been detected.
extern bool vertical_vertex;
extern bool vertical_edge;
extern bool calibrating;
extern bool vertex_calibrated;
extern bool calibrated;
extern bool calibrated_leds;

// PID-like balancing gains for vertex mode:
// K1 = angle, K2 = angular rate, K3 = translational speed, K4 = motor speed.
// The Z axis uses separate gains because it is controlled differently.
extern float K1;
extern float K2;
extern float K3;
extern float K4;
extern float zK2;
extern float zK3;
// Outer heading loop: commanded yaw rate per degree of heading error.
// 0 disables heading hold, leaving the yaw rate command fully manual.
extern float zK1;

// Gains used while balancing on an edge.  Edge mode uses motor 3 directly.
extern float eK1;
extern float eK2;
extern float eK3;
extern float eK4;
// Auto-trim adaptation rate.  0 disables auto-trim (the default), which
// leaves balancing exactly as it was before the feature existed.
extern float tK;

extern int loop_time;        // Main control period in milliseconds

// Accelerometer offsets measured in the two calibration poses.
// The vertex and edge poses have different gravity vectors, so each gets
// its own reference values.
struct OffsetsObj {
  int ID;
  float acXv;
  float acYv;
  float acZv;
  float acXe;
  float acYe;
  float acZe;
};
extern OffsetsObj offsets;

extern float alpha;          // Low-pass filter for gyro rate used by control

// Raw and corrected MPU6050 readings.
extern int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
extern float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
extern float speed_X, speed_Y;

// Gyro bias found during startup while the cube is stationary.
extern int16_t  GyZ_offset;
extern int16_t  GyY_offset;
extern int16_t  GyX_offset;
extern int32_t  GyZ_offset_sum;
extern int32_t  GyY_offset_sum;
extern int32_t  GyX_offset_sum;

extern float robot_angleX, robot_angleY; // Fused orientation estimates
extern float Acc_angleX, Acc_angleY;     // Orientation estimated from gravity only
extern int32_t motors_speed_X;            // Integrated speed feedback in X/Y/Z
extern int32_t motors_speed_Y;
extern int32_t motors_speed_Z;

// Two independent timers are used: one for the fast balancing loop and one
// for slower battery/calibration status messages.
extern long currentT, previousT_1, previousT_2;

// --- Battery monitoring (see battCheck() in functions.cpp) ---------------
// analogRead(VBAT) counts per battery volt.  Board-specific: measure the pack
// and derive it with battery_test/.  225 was measured on this cube (12.0 V
// pack read raw 2700); upstream's 204 read 13.2 V here.
#define BATT_ADC_PER_VOLT  225.0f
// Thresholds for a 3S LiPo, in volts at the pack.
#define BATT_PRESENT_V      6.0f   // below this there is no pack: on USB power
                                   // alone the battery rail still reads ~4 V
#define BATT_WARN_V        10.5f   // 3.5 V/cell: start warning
#define BATT_WARN_CLEAR_V  10.7f   // hysteresis, so the warning cannot flicker
#define BATT_CUTOFF_V       9.9f   // 3.3 V/cell: disarm to protect the pack
#define BATT_CUTOFF_COUNT   4      // consecutive low checks before cutting off,
                                   // so a dip under load does not end a balance
#define BATT_REARM_V       10.8f   // after a cutoff, arming is refused until
                                   // the pack reads this (charged or swapped)
#define BATT_CHECK_MS     500      // battery check period
enum BattState { BATT_NONE, BATT_OK, BATT_LOW, BATT_CUTOFF };
// Smoothed battery voltage and its classification, updated by battCheck().
// Stored here so the web interface can report them through /api/state.
extern float batt_voltage;
extern BattState batt_state;

// Battery-voltage compensation.  PWM duty sets roughly what fraction of the
// pack voltage reaches a motor, so as the pack sags the same command gives
// less torque and the whole controller gets weaker.  With compensation on,
// every motor command is multiplied by vNom / batt_voltage, so the motors see
// the drive they would at vNom and one set of gains holds across the
// discharge.  It cannot add torque the pack does not have: near full output a
// low pack still saturates sooner.
// vNom is a tunable gain: 0 = off (default), otherwise the voltage the gains
// were tuned at (about 11.5 V on this cube).  The factor is clamped and is
// 1 whenever no pack is detected.
#define BATT_COMP_MIN  0.90f
#define BATT_COMP_MAX  1.25f
extern float vNom;
extern float batt_comp;      // factor currently applied, for the dashboard

// Demo mode: 1 = arm at the end of boot, so the cube balances when stood up
// without connecting to the dashboard.  0 = boot disarmed (default).  Stored
// as a gain so it is switched and saved from the dashboard.
extern float autoArm;

// Spin lag fix: zK3 (wheel unwind) is multiplied by zK3s while a spin is
// commanded, which cuts the lag behind the commanded rate.  1 = unchanged
// (default); e.g. 0.25 cuts the lag about fourfold.  The cube then really
// spins faster, so friction winds the wheels up sooner and the wind-up guard
// ends a long spin earlier.
extern float zK3s;

// Nudge to spin (demo).  With nudgeDeg > 0, twisting the balancing cube by
// hand starts a turn of nudgeDeg in that direction (360 = one revolution),
// which then holds the new heading.  A nudge is the cube turning more than
// NUDGE_TURN_DEG within the last NUDGE_WINDOW ticks (a gentle twist: 4.0°;
// ordinary balancing where nudges are allowed: at most 1.4°, all traces to
// 2026-10-04).  Ignored for NUDGE_QUIET_MS after the cube is stood up, during
// a turn or slider spin, while the wind-up guard is active, and while the
// smoothed balance effort is above NUDGE_CALM.  A 360° turn at 20°/s takes
// ~20 s, inside the ~27 s the guard allows.
#define NUDGE_TURN_DEG  2.5f    // degrees
#define NUDGE_WINDOW    27      // control ticks (~0.4 s)
#define NUDGE_CALM      100.0f  // smoothed balance effort, of 255
#define NUDGE_QUIET_MS  2000
extern float nudgeDeg;

// --- Web command interface (see web_interface.cpp) ---------------------
// HTTP handlers never touch the motors.  They only store a request here,
// and the main control loop acts on it at the start of a control cycle.
#define WEB_CMD_NONE    0
#define WEB_CMD_STOP    1    // stop now, engage brake, and disarm
#define WEB_CMD_DISARM  2    // disarm (same effect; separate for clarity)
#define WEB_CMD_ARM     3    // allow balancing again
#define WEB_CMD_CAL_START   4  // begin calibration (same as Bluetooth "c+")
#define WEB_CMD_CAL_CAPTURE 5  // record the current pose (same as "c-")
#define WEB_CMD_CAL_SAVE    6  // write the offsets to EEPROM
#define WEB_CMD_GAINS_SAVE  7  // write the current gains to EEPROM
#define WEB_CMD_TRIM_RESET  8  // clear the learned balance-point trim
// --- Yaw rate command -------------------------------------------------
// Commanded rotation rate about the vertical axis, in degrees/second, used
// only in vertex mode.  Zero means "hold heading", which is the original
// behaviour.  Written by an HTTP handler, so volatile.  The control loop
// ramps yaw_rate_cmd toward it at YAW_ACCEL: a step in the rate command is a
// step in yaw effort on all three wheels at once, which used to take the
// balance's headroom away and knock the cube over.
// Spin speed limit, tested 2026-10-03 with zK3s = 0.25: about 19 °/s real
// spin balanced calmly until the wind-up guard ended it; at about 24 °/s a
// single jolt (likely the corner slipping) could not be recovered and the
// cube fell.  Upstream allowed 90 (zK2 * 90 is three times the motor range).
#define YAW_RATE_MAX  20.0f    // slider/API clamp, °/s
#define YAW_TURN_RATE 20.0f    // fastest rate the heading loop asks for, °/s
#define YAW_ACCEL     30.0f    // max change of yaw_rate_cmd, °/s per second
// Yaw effort added to every motor is capped here, and further limited to the
// headroom the balancing axes leave on the busiest motor (XYZ_to_threeWay),
// so spinning can never take drive away from staying upright.
#define YAW_PWM_MAX   60.0f
// Wheel wind-up guard.  Friction at the contact corner keeps slowing a spin,
// so holding one makes the wheels speed up continuously (30 -> 100 counts per
// tick over 70 s at 20°/s, trace 2026-10-03); near their top speed (~250)
// they leave balancing no authority, and stopping a spin with them wound up
// is what preceded the fall in that trace.  wheel_wind is the smoothed
// average of the three signed wheel speeds (balancing alone kept it under 3
// in that trace).  Above the limit any spin or turn is ramped to a stop and
// new ones are refused until the wheels are back under the resume level.
// Replayed against the trace, 70 would have stopped the 20°/s spin after
// ~27 s with the wheels at 70 instead of 95.
#define YAW_WHEEL_LIMIT    70.0f   // counts per control tick
#define YAW_WHEEL_RESUME   30.0f
extern float wheel_wind;
extern bool yaw_guard;
extern volatile float yaw_rate_request;
extern float yaw_rate_cmd;     // ramped command actually used by the loop

// --- Heading hold and scripted turns ----------------------------------
// robot_yaw integrates gyroZ while balancing on a vertex, giving a heading
// in degrees relative to wherever the cube latched upright (it is zeroed at
// each latch).  An outer proportional loop closes on it: the rate loop above
// already holds whatever rate it is told, so commanding
// zK1 * (yaw_target - robot_yaw) turns "spin at X°/s" into "sit at heading Y"
// and, with a target set some degrees away, into "turn exactly that far".
//
// This is dead reckoning, not a compass: the MPU6050 has no magnetometer, so
// robot_yaw accumulates the residual gyro bias left after the boot
// calibration and will creep over minutes.  Good enough to hold still and to
// land a 90° turn; not an absolute heading reference.
#define YAW_TURN_MAX 720.0f    // largest single commanded turn, degrees
extern float robot_yaw;        // integrated heading, degrees; 0 at latch
extern float yaw_target;       // heading the outer loop is driving toward
// Written by an HTTP handler (which only ever clears yaw_hold and raises
// yaw_turn_new) and by the control loop, hence volatile.  Safe without
// locking for the same reason the rest of this interface is: handlers run
// from handleWebInterface() inside loop(), strictly between control cycles.
extern volatile bool  yaw_hold;      // outer loop active?
extern volatile bool  yaw_turn_new;  // a turn request is waiting
extern volatile float yaw_turn_request;  // degrees, relative to current

// --- Telemetry trace ---------------------------------------------------
// A ring buffer of one sample per control-loop iteration (66.7 Hz), so the
// dashboard can plot the real dynamics instead of a 3.3 Hz alias of them.
// The control loop writes; the /api/trace handler reads.  Single-core-safe
// by construction: both run from loop(), never concurrently.
//
// 336 samples x 32 bytes = 10752 bytes of RAM, a 5 s window - enough to
// cover many missed polls, since the browser accumulates the stream.
#define TRACE_LEN 336
struct TraceSample {
  uint32_t seq;        // monotonically increasing sample number
  uint32_t t_ms;       // millis() at capture
  int16_t  angX10;     // robot_angleX * 100, centidegrees
  int16_t  angY10;     // robot_angleY * 100
  int16_t  gyrX10;     // gyroXfilt * 10
  int16_t  gyrY10;     // gyroYfilt * 10
  int16_t  m1, m2, m3; // wheel speeds, counts/loop
  int16_t  pwmX, pwmY; // last commanded axis efforts (vertex mode)
  int16_t  gyrZ10;     // yaw rate * 10, deg/s
  int16_t  ycmd10;     // ramped yaw rate command * 10
  int16_t  pwmZ;       // yaw effort actually added to each motor
};
extern TraceSample trace_buf[TRACE_LEN];
extern uint32_t trace_seq;           // next sequence number to be written
extern int16_t trace_pwmX, trace_pwmY;  // captured by the balancing branch
extern int16_t trace_pwmZ;           // captured by XYZ_to_threeWay()
void traceRecord();                  // append one sample (functions.cpp)

// --- Balance-point auto-trim ------------------------------------------
// The cube's true balance point is rarely at exactly zero degrees: the
// centre of mass sits a little off the contact point.  Holding a setpoint
// of zero therefore means a permanent small angle error, so the wheels
// accelerate steadily to hold the cube up until they saturate and it falls.
//
// The trim below is a slowly-learned offset applied to the angle setpoint.
// It is driven by persistent wheel speed: sustained speed in one direction
// means the setpoint is wrong, so the trim moves until the wheels settle.
// Adaptation rate is the tunable gain tK; tK = 0 disables it entirely.
#define TRIM_MAX 3.0f          // clamp, degrees - a runaway trim cannot
                               // command more than a small lean
extern float trimX, trimY;
// volatile because it is written by an HTTP handler and read by the loop.
extern volatile uint8_t web_cmd_pending;

// Master enable for balancing.  Defaults to true so the cube behaves exactly
// as before unless the web interface explicitly disarms it.  While false,
// the control loop takes its normal "not balancing" path: no drive, brake
// engaged.  Only an ARM command or a restart clears it.
extern bool armed;

// Encoder counts are modified inside interrupt handlers, so they must be
// volatile.  The main loop periodically copies and resets them.
extern volatile int  enc_count1, enc_count2, enc_count3;
extern int16_t motor1_speed;
extern int16_t motor2_speed;
extern int16_t motor3_speed;

// Objects defined in esp32_cube_enc.cpp, used from functions.cpp.
extern CRGB leds[NUM_PIXELS];

// Result of the most recent calibration capture, shown on the dashboard.
// Bluetooth used to carry this feedback; now it travels in /api/state.
// Always points at a string literal, so it needs no allocation and is
// safe to embed in JSON (no quotes or backslashes in any of the values).
extern const char* cal_result;

// Entry points implemented in web_interface.cpp.  Every .ino file used to be
// concatenated into one translation unit, so Arduino auto-generated forward
// declarations for whatever a later file defined.  Now that each file is
// compiled on its own, that no longer happens - so every function called
// from a *different* .cpp file needs an explicit prototype here.
void startWebInterface();
void handleWebInterface();
void loadGains();
void saveGains();
bool balancingActive();

// Entry points implemented in functions.cpp.
void writeTo(byte device, byte address, byte value);
void beep();
void save();
void angle_setup();
// dt is the MEASURED time since the previous call, in seconds.  It used to
// integrate the constant loop_time instead, so any iteration that ran long
// (typically handleWebInterface() pushing the dashboard) silently corrupted
// the angle by the amount it overran.
void angle_calc(float dt);
void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z);
void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3);
void battCheck();
void battIndicate();
const char* battStateName();
void pwmSet(uint8_t pin, uint32_t value);
void Motor1_control(int sp);
void Motor2_control(int sp);
void Motor3_control(int sp);
void ENC1_READ();
void ENC2_READ();
void ENC3_READ();
void calStart();
void calCapture();
int Tuning();

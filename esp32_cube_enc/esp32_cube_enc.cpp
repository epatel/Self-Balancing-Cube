#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>

// Storage for the globals declared `extern` in ESP32.h.  This is the one
// translation unit that defines them; every other file just links against
// these.  (Split out of ESP32.h itself so including that header from
// multiple .cpp files doesn't define each variable more than once.)
float Gyro_amount = 0.996;

bool vertical_vertex = false;
bool vertical_edge = false;
bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;

float K1 = 180;
float K2 = 30.00;
float K3 = 1.6;
float K4 = 0.008;
float zK2 = 8.00;
float zK3 = 0.30;
float zK1 = 1.00;              // heading hold: °/s commanded per ° of error

float eK1 = 190;
float eK2 = 31.00;
float eK3 = 2.5;
float eK4 = 0.014;
float tK = 0;                  // auto-trim off until the user enables it

int loop_time = 15;

OffsetsObj offsets;

float alpha = 0.7;

int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
float speed_X, speed_Y;

int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;
int32_t  GyX_offset_sum = 0;

float robot_angleX, robot_angleY;
float Acc_angleX, Acc_angleY;
int32_t motors_speed_X;
int32_t motors_speed_Y;
int32_t motors_speed_Z;

long currentT, previousT_1, previousT_2;

float batt_voltage = 0;
BattState batt_state = BATT_NONE;
float vNom = 0;                // battery compensation off until enabled
float autoArm = 0;             // demo mode: arm at boot when 1
float batt_comp = 1.0f;

volatile uint8_t web_cmd_pending = WEB_CMD_NONE;

// Boot DISARMED.  angle_calc() detects an upright pose on its own, inside a
// 0.4 degree window, with no human input - so booting armed means a cube
// left standing on its vertex spins up three wheels a couple of seconds
// after power-on, unattended.  Arm deliberately from the dashboard, or with
// "a+" over USB serial if the access point is unavailable.
bool armed = false;

volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int16_t motor1_speed;
int16_t motor2_speed;
int16_t motor3_speed;

// Yaw-rate command and learned balance-point trim (see ESP32.h).
volatile float yaw_rate_request = 0;
float yaw_rate_cmd = 0;
float trimX = 0, trimY = 0;
float wheel_wind = 0;          // wind-up guard (see ESP32.h)
bool yaw_guard = false;

// Heading hold / scripted turns (see ESP32.h).
float robot_yaw = 0;
float yaw_target = 0;
volatile bool  yaw_hold = false;
volatile bool  yaw_turn_new = false;
volatile float yaw_turn_request = 0;

CRGB leds[NUM_PIXELS];
const char* cal_result = "";   // see ESP32.h

// Telemetry trace ring (see ESP32.h).  ~8 KB of the ~270 KB free heap;
// statically allocated so a failed malloc can't silently disable tracing.
TraceSample trace_buf[TRACE_LEN];
uint32_t trace_seq = 0;
int16_t trace_pwmX = 0, trace_pwmY = 0;
int16_t trace_pwmZ = 0;

void setup() {
  // ---- MOTOR SAFETY: this block must run before anything slow. ----
  // Until LEDC is attached, the PWM pins float, and this driver hardware
  // treats a floating/LOW PWM input as FULL DRIVE (the PWM is inverted:
  // 255 = stopped).  Engage the brake and force all three PWM outputs to
  // the stopped level immediately, so the motors cannot run away during
  // the LED animation and gyro-bias measurement below.
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, LOW);   // brake engaged; the balancing loop releases
                              // it only when an upright pose is active
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  // ESP32 core 3.x API: ledcAttach() replaces ledcSetup()+ledcAttachPin()
  // and manages the channel internally; PWM is now addressed by pin.
  bool pwm_ok = ledcAttach(PWM1, BASE_FREQ, TIMER_BIT);
  pwm_ok = ledcAttach(PWM2, BASE_FREQ, TIMER_BIT) && pwm_ok;
  pwm_ok = ledcAttach(PWM3, BASE_FREQ, TIMER_BIT) && pwm_ok;
  Motor1_control(0);          // duty 255 = stopped (inverted PWM)
  Motor2_control(0);
  Motor3_control(0);
  // ---- end motor safety block ----

  // Start the two serial interfaces: USB serial is useful for diagnostics,
  // while Bluetooth is used to tune gains and run calibration.
  Serial.begin(115200);
  // If any LEDC attach failed, the PWM pins are still floating and the
  // motors are NOT safe: report it loudly and keep the brake engaged.
  if (!pwm_ok)
    Serial.println("ERROR: motor PWM attach failed - outputs unconfigured!");
  EEPROM.begin(EEPROM_SIZE);

  // The three WS2812B LEDs provide visual feedback during startup,
  // calibration, and low-battery warnings.
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS);  // GRB ordering is typical

  pinMode(BUZZER, OUTPUT);
  // The dev board's own LED doubles as the low-battery indicator, so a
  // warning is visible even on a cube without a buzzer or WS2812s.
  pinMode(INT_LED, OUTPUT);
  digitalWrite(INT_LED, LOW);

  // Cycle through red, green, and blue to show that the LEDs are working.
  for (int i=0;i<=255;i+=10) {
    leds[0] = CRGB(i, 0, 0);
    leds[1] = CRGB(i, 0, 0);
    leds[2] = CRGB(i, 0, 0);
    FastLED.show();
    delay(5);
  }
  delay(300);
  for (int i=0;i<=255;i+=10) {
    leds[0] = CRGB(0, i, 0);
    leds[1] = CRGB(0, i, 0);
    leds[2] = CRGB(0, i, 0);
    FastLED.show();
    delay(5);
  }
  delay(300);
  for (int i=0;i<=255;i+=10) {
    leds[0] = CRGB(0, 0, i);
    leds[1] = CRGB(0, 0, i);
    leds[2] = CRGB(0, 0, i);
    FastLED.show();
    delay(5);
  }
  delay(300);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();
  
  // Encoder inputs: both channels of each encoder trigger the same
  // quadrature decoder on every edge.  (Direction/PWM outputs were already
  // configured in the motor-safety block at the top of setup().)
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);

  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);

  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);

  // A valid ID means that accelerometer offsets were previously saved.
  // EEPROM data survives power cycles, so calibration is normally needed only
  // after changing the hardware or clearing the EEPROM.
  EEPROM.get(0, offsets);
  if (offsets.ID == 96)
    calibrated = true;

  // Restore any tuning gains saved from the web interface.  If none were
  // ever saved, the compiled-in defaults stay in force.
  loadGains();

  delay(200);
  // Configure the MPU6050 and measure the gyro's stationary bias.
  angle_setup();

  // Demo mode: arm at boot so no phone is needed (autoArm gain, saved with
  // the gains).  Still safe in the same ways as a manual ARM: nothing spins
  // until the cube is stood within 0.4° of a calibrated pose, a fall or the
  // battery cutoff disarms, and the dashboard can still disarm.  Done before
  // the access point starts, so an AP failure still disarms as it always did.
  if (autoArm >= 0.5f && calibrated) {
    armed = true;
    Serial.println("Auto-arm: armed at boot (set autoArm to 0 to disable).");
  }

  // Start the Wi-Fi access point and web server (see web_interface.cpp).
  // Done last so it cannot disturb the gyro-bias measurement above.
  startWebInterface();
}

void loop() {
  currentT = millis();
  // This is the fast control loop.  It is deliberately time-based rather
  // than delay-based so sensor and motor work can run at a stable period.
  if (currentT - previousT_1 >= loop_time) {
    // Time actually elapsed since the last control pass.  The estimator used
    // to assume exactly loop_time, so an iteration that ran long - almost
    // always handleWebInterface() pushing the dashboard or a trace batch -
    // under-integrated the gyro by however far it overran.  Clamped because
    // an overrun long enough to matter (a stalled client, the first pass
    // after setup) means the rate sample is stale, and integrating it over
    // the full gap would be worse than admitting the gap.
    float dt = (currentT - previousT_1) / 1000.0f;
    dt = constrain(dt, 0.005f, 0.045f);
    Tuning();
    angle_calc(dt);

    // Encoder interrupts accumulate counts between control-loop iterations.
    // Copy each interval's count as a speed estimate, then start a new one.
    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;
    // Convert the three motor speeds into the cube's X/Y motion components.
    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;

    // Wheel wind-up guard (see YAW_WHEEL_LIMIT in ESP32.h).  A spin winds all
    // three wheels up together while balancing moves them in opposite
    // directions, so the average of the signed speeds isolates the wind-up.
    // Smoothed over ~0.3 s so a single fast tick does not trip it.
    wheel_wind += 0.05f * (abs(motors_speed_Z) / 3.0f - wheel_wind);
    if (!yaw_guard && wheel_wind > YAW_WHEEL_LIMIT) {
      yaw_guard = true;
      yaw_rate_request = 0;          // slider spin ramps down to zero
      yaw_turn_new = false;
      if (yaw_hold) yaw_target = robot_yaw;   // a turn stops where it is
      Serial.println("Wheels wound up: spin stopped until they slow down.");
    } else if (yaw_guard && wheel_wind < YAW_WHEEL_RESUME) {
      yaw_guard = false;
    }
    
    // Act on any command left by the web interface.  This runs after
    // angle_calc() (which can set the pose flags) and before the balancing
    // branches below, so a stop cannot be undone within the same iteration.
    // The motors are never touched here: clearing these flags routes the
    // control loop into its existing "not balancing" branch, which stops
    // the drive and engages the brake.
    switch (web_cmd_pending) {
      case WEB_CMD_STOP:
      case WEB_CMD_DISARM:
        armed = false;              // blocks balancing until re-armed
        vertical_vertex = false;    // forget the current upright pose
        vertical_edge = false;
        yaw_rate_request = 0;       // never resume a spin on its own
        yaw_rate_cmd = 0;
        yaw_hold = false;           // nor resume chasing a heading target
        yaw_turn_new = false;
        break;
      case WEB_CMD_ARM:
        // A battery cutoff holds until the pack is charged or swapped (or
        // the cube is restarted); the HTTP handler refuses too, but the
        // serial "a+" path arrives only here.
        if (batt_state == BATT_CUTOFF) {
          Serial.println("Refused to arm: battery cutoff. Charge or replace the pack.");
          break;
        }
        armed = true;
        // Leave the pose flags cleared: angle_calc() re-detects an upright
        // pose only within its tight angle window, so arming can never make
        // the cube jump straight back into balancing from a stale pose.
        vertical_vertex = false;
        vertical_edge = false;
        yaw_rate_request = 0;       // arm to a standstill, not into a spin
        yaw_rate_cmd = 0;
        yaw_hold = false;
        yaw_turn_new = false;
        break;
      case WEB_CMD_TRIM_RESET:
        // Forget the learned balance point and start again from zero.
        trimX = 0;
        trimY = 0;
        break;
      // Calibration commands run the same functions as the Bluetooth "c+"
      // and "c-" commands.  Each is refused while the cube is actively
      // balancing; the HTTP handler checks this too, but it is re-checked
      // here because the cube may have started balancing in between.
      case WEB_CMD_CAL_START:
        if (!balancingActive() && !calibrating) calStart();
        break;
      case WEB_CMD_CAL_CAPTURE:
        // Only meaningful once calibration has been started.
        if (!balancingActive() && calibrating) calCapture();
        break;
      case WEB_CMD_CAL_SAVE:
        // Commit the offsets recorded so far.  A normal two-pose
        // calibration already saves automatically after the edge pose;
        // this is for saving explicitly from the dashboard.
        if (!balancingActive() && calibrating && vertex_calibrated) save();
        break;
      case WEB_CMD_GAINS_SAVE:
        // Persist the gains currently in use.  Editing gains from the
        // dashboard only changes RAM; the EEPROM write happens here, once
        // per explicit Save, so tuning never wears out the flash.
        //
        // Refused while balancing, like the calibration writes: an NVS
        // commit stalls the control loop for tens of milliseconds with the
        // flash cache disabled, and the encoder interrupt handlers are not
        // in IRAM, so wheel counts are lost across the write.  The HTTP
        // handler checks this too; re-checked here because the cube can
        // start balancing between request and action.
        if (!balancingActive()) saveGains();
        break;
    }
    web_cmd_pending = WEB_CMD_NONE; // request consumed

    // Take the yaw-rate request set by the HTTP handler and clamp it.  The
    // handler validates too; this is the authoritative limit, so a bad
    // value can never reach the controller.  It is a target: the vertex
    // branch ramps yaw_rate_cmd toward it.
    float rate_target = constrain(yaw_rate_request, -YAW_RATE_MAX, YAW_RATE_MAX);

    // Vertex mode controls two tilt axes and the common Z rotation axis.
    // "armed" gates both balancing branches; when false the else branch
    // below stops the motors and engages the brake.
    if (armed && vertical_vertex && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / GYRO_LSB_PER_DPS;
      gyroY = GyY / GYRO_LSB_PER_DPS;
      gyroZ = GyZ / GYRO_LSB_PER_DPS;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;

      // --- Heading: dead-reckon it, then optionally close a loop on it ---
      // Integrating here rather than above the branch is deliberate: gyroZ
      // is only meaningful in vertex mode, and a heading accumulated while
      // the cube lay on its side would be nonsense to hold on to.
      robot_yaw += gyroZ * dt;
      if (yaw_turn_new) {
        // Turns are RELATIVE to where the cube is pointing right now, so
        // repeated "+90" presses walk it around a square.
        yaw_target = robot_yaw + constrain(yaw_turn_request,
                                           -YAW_TURN_MAX, YAW_TURN_MAX);
        yaw_hold = true;
        yaw_turn_new = false;
      }
      // Outer proportional loop; feeds the same clamped rate command the
      // slider drives, so the inner rate loop below is untouched.  No
      // angle wrapping on purpose: target and heading share one unbounded
      // frame, which is what makes "turn 720" mean two full spins.
      // Turns are capped at YAW_TURN_RATE, gentler than the slider's limit.
      if (yaw_hold)
        rate_target = constrain(zK1 * (yaw_target - robot_yaw),
                                -YAW_TURN_RATE, YAW_TURN_RATE);
      // Ramp the command toward the target instead of stepping it, so the
      // yaw effort builds gradually on all three wheels.
      float yaw_step = YAW_ACCEL * dt;
      yaw_rate_cmd += constrain(rate_target - yaw_rate_cmd, -yaw_step, yaw_step);

      // Learn the true balance point.  Sustained wheel speed in one
      // direction means the setpoint is on the wrong side of the real
      // balance point, so move the trim AGAINST that speed until the wheels
      // settle.  The rate is tiny (tK is small and this runs every 15 ms) so
      // it tracks only the slow drift, never the fast balancing dynamics.
      // tK = 0 freezes adaptation but a trim already learned (or restored
      // from EEPROM) still applies - that is the "learn once, then hold"
      // workflow.  Reset trim gives back exactly the original controller.
      //
      // Sign: subtracting is correct for this controller's convention, where
      // +K3 * speed_X is the stabilising wheel-unwind term.  If the trim on
      // your hardware runs to the +/-TRIM_MAX clamp and balancing gets worse
      // instead of better, the encoder polarity is inverted - set tK
      // negative rather than editing this line.
      if (tK != 0) {
        trimX = constrain(trimX - tK * speed_X * loop_time / 1000.0,
                          -TRIM_MAX, TRIM_MAX);
        trimY = constrain(trimY - tK * speed_Y * loop_time / 1000.0,
                          -TRIM_MAX, TRIM_MAX);
      }

      // Each term counters a different part of the motion:
      // angle keeps the cube upright, gyro rate damps it, and the speed terms
      // reduce motion that would otherwise build up around the equilibrium.
      // Subtracting the trim shifts the angle setpoint onto the balance point.
      int pwm_X = constrain(K1 * (robot_angleX - trimX) + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * (robot_angleY - trimY) + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
      // Z is a rate loop: driving (gyroZ - yaw_rate_cmd) to zero holds the
      // heading when the command is zero, and spins at the commanded rate
      // otherwise.  Injecting the setpoint here leaves the loop's stability
      // untouched - only its target changes.
      int pwm_Z = constrain(zK2 * (gyroZ - yaw_rate_cmd) + zK3 * motors_speed_Z, -255, 255);

      // A small accumulated speed correction acts like an integral term.
      motors_speed_X += speed_X / 5; 
      motors_speed_Y += speed_Y / 5;
      // Transform desired X/Y/Z forces into the three motor commands.
      XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
      trace_pwmX = pwm_X;          // captured for the telemetry trace
      trace_pwmY = pwm_Y;
    } else if (armed && vertical_edge && calibrated && !calibrating) {
      // In edge mode, only motor 3 is used to correct the detected tilt.
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / GYRO_LSB_PER_DPS;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;

      // Same balance-point learning as vertex mode, but edge mode measures
      // its wheel speed from motor 3 alone (see the eK3 term below).
      if (tK != 0) {
        trimX = constrain(trimX - tK * motor3_speed * loop_time / 1000.0,
                          -TRIM_MAX, TRIM_MAX);
      }

      int pwm_X = constrain(eK1 * (robot_angleX - trimX) + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);

      motors_speed_X += motor3_speed / 5;
      Motor3_control(pwm_X);
      trace_pwmX = pwm_X;          // edge mode drives only the X effort
      trace_pwmY = 0;
      trace_pwmZ = 0;
      yaw_rate_cmd = 0;            // no yaw control on an edge
    } else {
      // If the cube is not in a recognized balancing pose, stop applying
      // drive and engage the brake.  This protects the motors during setup or
      // after the cube has fallen.
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0;
      trace_pwmX = 0;              // no drive commanded while idle
      trace_pwmY = 0;
      yaw_rate_cmd = 0;            // re-entering vertex mode ramps from rest
    }
    // Record this iteration into the telemetry trace, whichever branch ran:
    // the moments around a fall are the ones worth plotting.
    traceRecord();
    previousT_1 = currentT;
  }
  
  // Battery check: warns, and disarms on a sustained low voltage.
  static long previousT_batt = 0;
  if (currentT - previousT_batt >= BATT_CHECK_MS) {
    battCheck();
    previousT_batt = currentT;
  }
  battIndicate();   // blinks the warning; cheap when nothing changes

  // Slow status loop: blink LEDs until calibration has been completed.  A
  // battery warning owns the LEDs while it is active.
  if (currentT - previousT_2 >= 2000) {
    if (!calibrated && !calibrating) {
      Serial.println("Not calibrated yet - use the web dashboard "
                     "(http://192.168.4.1) or send c+ / c- over USB serial.");
    }
    if (!calibrated && !calibrating
        && batt_state != BATT_LOW && batt_state != BATT_CUTOFF) {
      if (!calibrated_leds) {
        leds[0] = CRGB(0, 255, 0);
        leds[1] = CRGB(0, 255, 0);
        leds[2] = CRGB(0, 255, 0);
        FastLED.show();
        calibrated_leds = true; 
      } else {
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        leds[2] = CRGB::Black;
        FastLED.show();
        calibrated_leds = false; 
      }
    }
    previousT_2 = currentT;
  }

  // Service pending HTTP clients (web_interface.cpp).  Non-blocking: it
  // returns immediately when no client is connected, so the timed balancing
  // loop above is unaffected.
  handleWebInterface();
}

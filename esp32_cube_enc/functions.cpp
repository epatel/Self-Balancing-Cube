#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>

void writeTo(byte device, byte address, byte value) {
  // Helper for writing one byte to an MPU6050 register over I2C.
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
    // The active buzzer sounds while its pin is HIGH.
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

void save() {
    // ESP32 EEPROM emulation needs commit() to persist the new calibration.
    EEPROM.put(0, offsets);
    EEPROM.commit();
    EEPROM.get(0, offsets);
    if (offsets.ID == 96) calibrated = true;
    calibrating = false;
    Serial.println("Calibrating off.");
    beep();
}

void angle_setup() {
  // Wake the MPU6050 and select the measurement ranges from ESP32.h.
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);
  
  // Average 512 stationary samples on each gyro axis to measure its bias.
  // The board LED stays on while this runs: keep the cube still.
  digitalWrite(INT_LED, HIGH);
  beep();
  leds[2] = CRGB(0, 0, 200);
  FastLED.show();
  // The dt passed here just matches the delay below.  Bias measurement only
  // sums the raw rates; the integrated angle is discarded when a pose latches.
  for (int i = 0; i < 512; i++) {
    angle_calc(0.005f);
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 9;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  beep();
  leds[2] = CRGB::Black;
  FastLED.show();

  leds[1] = CRGB(0, 0, 200);
  FastLED.show();
  for (int i = 0; i < 512; i++) {
    angle_calc(0.005f);
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 9;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  beep();
  leds[1] = CRGB::Black;
  FastLED.show();

  leds[0] = CRGB(0, 0, 200);
  FastLED.show();
  for (int i = 0; i < 512; i++) {
    angle_calc(0.005f);
    GyX_offset_sum += GyX;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 9;
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
  beep();
  beep();
  leds[0] = CRGB::Black;
  FastLED.show();

  leds[0] = CRGB(255, 0, 0);
  leds[1] = CRGB(255, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  FastLED.show();
  delay(300);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();
  delay(150);
  leds[0] = CRGB(255, 0, 0);
  leds[1] = CRGB(255, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  FastLED.show();
  delay(300);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();
  delay(300);

  // Gyro check done: the board LED goes off and blinks three times quickly,
  // so the cube can be picked up.
  for (int i = 0; i < 3; i++) {
    digitalWrite(INT_LED, LOW);
    delay(120);
    digitalWrite(INT_LED, HIGH);
    delay(120);
  }
  digitalWrite(INT_LED, LOW);
}

void angle_calc(float dt) {
  // Read the three raw gyro registers (0x43 through 0x48).
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // Read the three raw accelerometer registers (0x3B through 0x40).
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true); 
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // Remove the accelerometer's zero offsets (ACC_OFFSET_* in ESP32.h), still
  // in the chip's own axes.
  AcX = constrain((int32_t)AcX - ACC_OFFSET_X, -32768, 32767);
  AcY = constrain((int32_t)AcY - ACC_OFFSET_Y, -32768, 32767);
  AcZ = constrain((int32_t)AcZ - ACC_OFFSET_Z, -32768, 32767);

#if IMU_MOUNT == 1
  // Rotate the upright board's axes into the firmware's frame (see
  // IMU_MOUNT in ESP32.h).  Done before the gyro bias is removed, so the
  // bias measured at boot is in the same frame as the readings it corrects.
  { int16_t x = AcX, z = AcZ; AcX = -z; AcZ = x; }
  { int16_t x = GyX, z = GyZ; GyX = -z; GyZ = x; }
#endif

  // Use the calibration values for the currently detected pose: small |AcX|
  // means vertex mode, while a larger |AcX| means edge mode.
  // The angles are relative to the captured pose; pose_tilt* is that pose's
  // own tilt from gravity in the sensor frame, needed below for the spin
  // correction.  (acZ* was stored as AcZ + 16384.)
  float pose_tiltX = 0, pose_tiltY = 0;
  if (abs(AcX) < 2000) {
    AcXc = AcX - offsets.acXv;
    AcYc = AcY - offsets.acYv;
    AcZc = AcZ - offsets.acZv;
    if (offsets.ID == 96) {
      pose_tiltX = -atan2(offsets.acYv, -(offsets.acZv - 16384)) * 57.2958;
      pose_tiltY = atan2(offsets.acXv, -(offsets.acZv - 16384)) * 57.2958;
    }
  } else {
    AcXc = AcX - offsets.acXe;
    AcYc = AcY - offsets.acYe;
    AcZc = AcZ - offsets.acZe;
    if (offsets.ID == 96) {
      pose_tiltX = -atan2(offsets.acYe, -(offsets.acZe - 16384)) * 57.2958;
      pose_tiltY = atan2(offsets.acXe, -(offsets.acZe - 16384)) * 57.2958;
    }
  }
  // Remove the stationary gyro bias before integrating angular velocity.
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  // Integrate gyro rate into an angle.  All-float on purpose: this was
  //     GyY * loop_time / 1000 / 65.536
  // where GyY and loop_time are both integers, so `GyY * 15 / 1000` was an
  // INTEGER division that truncated before the float divide ever ran.  Any
  // rate under 67 counts (~0.5°/s at ±250°/s) integrated to exactly zero -
  // a dead zone sitting right where a balancing cube lives - and everything
  // above it came out as a coarse staircase.  The divisor was wrong too:
  // 65.536 is the ±500°/s figure, so the estimate ran at twice the scale the
  // controller's own GyX / 131.0 rate terms assumed.  See GYRO_LSB_PER_DPS.
  //
  // Spin correction: when the cube spins about the true vertical, a sensor
  // that is tilted from it (the balance point is never exactly on the sensor
  // axis) sees part of the spin on its X/Y gyros although nothing is tipping.
  // Integrated as tilt, that put the estimate ~2° off while spinning at
  // 12-15°/s (trace, 2026-10-03).  The cross terms are the small-angle
  // kinematics of a gravity vector seen from a rotating body; they cancel
  // that component exactly.  Tilt here is from gravity, so the captured
  // pose's own tilt is added back.
  float spin_dps = GyZ / GYRO_LSB_PER_DPS;
  float tiltX_rad = (robot_angleX + pose_tiltX) / 57.2958f;
  float tiltY_rad = (robot_angleY + pose_tiltY) / 57.2958f;
  robot_angleY += (GyY / GYRO_LSB_PER_DPS - spin_dps * tiltX_rad) * dt;
  Acc_angleY = atan2(AcXc, -AcZc) * 57.2958;
  // Combine fast gyro response with the accelerometer's long-term reference.
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  robot_angleX += (GyX / GYRO_LSB_PER_DPS + spin_dps * tiltY_rad) * dt;
  Acc_angleX = -atan2(AcYc, -AcZc) * 57.2958;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  // Recognize a stable upright vertex or edge.  The tight angle thresholds
  // prevent balancing from starting while the cube is being placed.
  if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_vertex = true;
    // Every balancing session starts facing "zero" with no target, so a
    // turn commanded before the cube fell can never be resumed against
    // whoever just stood it back up.
    robot_yaw = 0;
    yaw_target = 0;
    yaw_hold = false;
  } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_edge = true;
  // Leaving the upright range disables the active balancing mode.
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_vertex) {
    vertical_vertex = false;
    yaw_rate_request = 0;   // see below
    yaw_rate_cmd = 0;
    yaw_hold = false;       // and never chase a heading target after a fall
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_edge) {
    vertical_edge = false;
    // Losing the pose means the cube fell.  Forget any commanded spin: the
    // pose flags re-set on their own once it is upright again, so without
    // this the cube would resume balancing AND spin straight back up to the
    // old yaw rate - with whoever just picked it up still holding it.
    yaw_rate_request = 0;
    yaw_rate_cmd = 0;
  }
}

void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z) {
  // The three motors are arranged 120° apart.  Convert desired X/Y/Z axis
  // commands into individual motor commands using the inverse mix.
  // 0.5 and 0.866 are cos(60°) and sin(60°); the other factors compensate
  // for this hardware's motor geometry and scale.
  float m1 = (0.5 * pwm_X - 0.866 * pwm_Y) / 1.37;
  float m2 = (0.5 * pwm_X + 0.866 * pwm_Y) / 1.37;
  float m3 = -pwm_X / 1.37;

  // Balance has priority over yaw.  pwm_Z is added to every motor, so it is
  // limited to what the busiest motor has left after its balancing command
  // and speed feedback (both as MotorN_control will add them, before battery
  // compensation scales the result), and to YAW_PWM_MAX overall.  Without
  // this a yaw command could use up the range the tilt axes need.
  float lim = 255.0f / batt_comp;
  float a1 = m1 + motor1_speed, a2 = m2 + motor2_speed, a3 = m3 + motor3_speed;
  float room_up   = lim - max(a1, max(a2, a3));   // most positive Z allowed
  float room_down = -lim - min(a1, min(a2, a3));  // most negative Z allowed
  float z = constrain(pwm_Z, -YAW_PWM_MAX, YAW_PWM_MAX);
  z = constrain(z, min(0.0f, room_down), max(0.0f, room_up));
  trace_pwmZ = lroundf(z);

  Motor1_control(lroundf(m1 + z));
  Motor2_control(lroundf(m2 + z));
  Motor3_control(lroundf(m3 + z));
}

void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3) {
  // Forward mix: reconstruct X/Y movement from measured motor speeds.
  speed_X = ((in_speed3 - (in_speed2 + in_speed1) * 0.5) * 0.5) * 1.81;
  speed_Y = -(-0.866 * (in_speed2 - in_speed1)) / 1.1;
}

void battCheck() {
  // Read the pack through the VBAT divider and classify it.  Called every
  // BATT_CHECK_MS from loop(); the thresholds are in ESP32.h.
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(VBAT);
  float v = sum / 16.0f / BATT_ADC_PER_VOLT;

  // Smooth over a couple of seconds, so a current spike while balancing is
  // not mistaken for a flat pack.  Plugging a pack in (or pulling it) jumps
  // straight to the new reading instead of ramping through the thresholds.
  if (v < BATT_PRESENT_V || batt_voltage < BATT_PRESENT_V)
    batt_voltage = v;
  else
    batt_voltage = 0.7f * batt_voltage + 0.3f * v;

  // Compensation factor for the motor commands (see vNom in ESP32.h).
  if (vNom > 0 && batt_voltage >= BATT_PRESENT_V)
    batt_comp = constrain(vNom / batt_voltage, BATT_COMP_MIN, BATT_COMP_MAX);
  else
    batt_comp = 1.0f;

  static int low_count = 0;
  BattState previous = batt_state;
  if (batt_voltage < BATT_PRESENT_V) {
    // No pack: the ESP32 is on USB alone.  Nothing to warn about, and a
    // cutoff is cleared since the pack that caused it has been removed.
    batt_state = BATT_NONE;
    low_count = 0;
  } else if (batt_state == BATT_CUTOFF) {
    // Latched: only a clearly recovered pack releases it, not the rebound
    // of a flat one once the motors stop drawing current.
    if (batt_voltage >= BATT_REARM_V) batt_state = BATT_OK;
  } else if (batt_voltage < BATT_CUTOFF_V) {
    batt_state = BATT_LOW;
    if (++low_count >= BATT_CUTOFF_COUNT) {
      batt_state = BATT_CUTOFF;
      // Same effect as DISARM: the control loop takes its "not balancing"
      // branch on the next pass, which stops the drive and brakes.  The cube
      // drops if it was balancing - that is the point of a cutoff.
      armed = false;
    }
  } else {
    low_count = 0;
    if (batt_voltage < BATT_WARN_V) batt_state = BATT_LOW;
    else if (batt_voltage > BATT_WARN_CLEAR_V || batt_state == BATT_NONE)
      batt_state = BATT_OK;
    // Between WARN and WARN_CLEAR the previous state stands (hysteresis).
  }

  if (batt_state != previous) {
    Serial.print("Battery "); Serial.print(batt_voltage, 2);
    Serial.print(" V: "); Serial.println(battStateName());
    if (batt_state == BATT_CUTOFF)
      Serial.println("Battery cutoff - disarmed. Charge or replace the pack.");
  }
}

void battIndicate() {
  // Low battery: slow blink.  Cutoff: fast blink.  Shown on the dev board's
  // own LED (INT_LED), the buzzer if one is fitted, and the WS2812s unless
  // calibration is using them.  Called every loop() pass; it only touches
  // the outputs when the blink phase changes.
  static bool shown = false;
  bool on = false;
  if (batt_state == BATT_LOW)         on = (millis() / 500) % 2;
  else if (batt_state == BATT_CUTOFF) on = (millis() / 125) % 2;
  if (on == shown) return;
  shown = on;
  digitalWrite(INT_LED, on ? HIGH : LOW);
  digitalWrite(BUZZER, on ? HIGH : LOW);
  if (!calibrating) {
    CRGB c = on ? CRGB(255, 0, 0) : CRGB::Black;   // the firmware's "red"
    leds[0] = c;
    leds[1] = c;
    leds[2] = c;
    FastLED.show();
  }
}

const char* battStateName() {
  // Also the value of "batt_state" in /api/state.
  switch (batt_state) {
    case BATT_OK:     return "ok";
    case BATT_LOW:    return "low";
    case BATT_CUTOFF: return "cutoff";
    default:          return "none";
  }
}

void pwmSet(uint8_t pin, uint32_t value) {
  // Write an 8-bit duty-cycle value to a PWM output.  ESP32 core 3.x
  // addresses LEDC by pin number (the old API used channel numbers).
  ledcWrite(pin, value);
}

void Motor1_control(int sp) {
  // Add the measured speed so the command includes motor-speed feedback.
  sp = sp + motor1_speed;
  // Battery compensation (vNom in ESP32.h): scale the whole drive, speed
  // feedback included, so the motor sees the voltage it would at vNom.
  sp = lroundf(sp * batt_comp);
  // The caller's command is already limited to +/-255, but adding the
  // encoder feedback can push past it.  Without this clamp, 255 - abs(sp)
  // would go negative and wrap to a huge value in pwmSet's uint32_t duty
  // argument, producing an out-of-range duty instead of full braking.
  sp = constrain(sp, -255, 255);
  if (sp < 0)
    digitalWrite(DIR1, LOW);
  else 
    digitalWrite(DIR1, HIGH);
  // The driver uses inverted PWM: 255 is stopped and smaller values drive
  // the motor harder.
  pwmSet(PWM1, 255 - abs(sp));
}

void Motor2_control(int sp) {
  // Motor 2 uses the same direction and inverted-PWM convention as motor 1.
  sp = sp + motor2_speed;
  sp = lroundf(sp * batt_comp);    // see Motor1_control
  sp = constrain(sp, -255, 255);   // see Motor1_control
  if (sp < 0)
    digitalWrite(DIR2, LOW);
  else 
    digitalWrite(DIR2, HIGH);
  pwmSet(PWM2, 255 - abs(sp));
}

void Motor3_control(int sp) {
  // Motor 3 uses the same direction and inverted-PWM convention as motor 1.
  sp = sp + motor3_speed;
  sp = lroundf(sp * batt_comp);    // see Motor1_control
  sp = constrain(sp, -255, 255);   // see Motor1_control
  if (sp < 0)
    digitalWrite(DIR3, LOW);
  else 
    digitalWrite(DIR3, HIGH);
  pwmSet(PWM3, 255 - abs(sp));
}

void ENC1_READ() {
  // Quadrature decoder: remember the previous channel states and count only
  // valid clockwise/counter-clockwise transitions.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}

void ENC2_READ() {
  // Same quadrature decoder for motor 2's encoder.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}

void ENC3_READ() {
  // Same quadrature decoder for motor 3's encoder.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}

void calStart() {
  // Calibration is a two-step process: record a valid vertex first,
  // then record a valid edge and save both offsets to EEPROM.
  // Extracted from Tuning() so the Bluetooth and web interfaces run the
  // exact same calibration code rather than two copies that could drift.
  calibrating = true;
  // Re-record BOTH poses every run.  This flag survives a completed
  // calibration (only a reboot cleared it), so without this reset a second
  // calibration in the same power session skipped straight to the edge step
  // and silently reused the stale vertex offsets.
  vertex_calibrated = false;
  cal_result = "Calibration started.";
  Serial.println("Calibrating on.");
  Serial.println("Set the cube on vertex...");
  leds[0] = CRGB(250, 250, 0);
  leds[1] = CRGB(250, 250, 0);
  leds[2] = CRGB(250, 250, 0);
  FastLED.show();
}

void calCapture() {
  // Record whichever pose the cube is currently in.  Shared by the
  // Bluetooth "c-" command and the web interface's Capture Pose button.
  Serial.print("X: "); Serial.print(AcX); Serial.print(" Y: "); Serial.print(AcY); Serial.print(" Z: "); Serial.println(AcZ + 16384);
  // Vertex pose: gravity is mostly along Z, so X and Y are near zero.
  if (abs(AcX) < 2000 && abs(AcY) < 2000) {
    offsets.ID = 96;
    offsets.acXv = AcX;
    offsets.acYv = AcY;
    offsets.acZv = AcZ + 16384;
    cal_result = "Vertex captured. Now set the cube on an edge.";
    Serial.println("Vertex OK.");
    Serial.println("Set the cube on edge...");
    vertex_calibrated = true;
    leds[0] = CRGB(0, 250, 250);
    leds[1] = CRGB(0, 250, 250);
    leds[2] = CRGB(0, 250, 250);
    FastLED.show();
    beep();
  // Edge pose: X has a characteristic gravity reading and Y remains
  // near zero.  Refuse edge calibration until the vertex was accepted.
  } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000 && vertex_calibrated) {
    Serial.print("X: "); Serial.print(AcX); Serial.print(" Y: "); Serial.print(AcY); Serial.print(" Z: "); Serial.println(AcZ + 16384);
    cal_result = "Edge captured. Calibration saved.";
    Serial.println("Edge OK.");
    offsets.acXe = AcX;
    offsets.acYe = AcY;
    offsets.acZe = AcZ + 16384;
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    leds[2] = CRGB::Black;
    FastLED.show();
    save();
  } else {
    // Neither pose matched.  This is the feedback Bluetooth used to carry;
    // it now reaches the dashboard through cal_result in /api/state.
    cal_result = "Pose not recognised - check the cube is settled and level.";
    Serial.println("The angles are wrong!!!");
    beep();
    beep();
  }
}

// Append one sample to the telemetry trace ring.  Called once at the end of
// every control-loop iteration, balancing or not - a trace that stops when
// the cube falls would hide exactly the moment worth looking at.  Fixed
// point keeps the sample at 32 bytes; the dashboard rescales for display.
void traceRecord() {
  TraceSample& s = trace_buf[trace_seq % TRACE_LEN];
  s.t_ms   = (uint32_t)currentT;
  s.angX10 = (int16_t)constrain(robot_angleX * 100.0f, -32767.0f, 32767.0f);
  s.angY10 = (int16_t)constrain(robot_angleY * 100.0f, -32767.0f, 32767.0f);
  s.gyrX10 = (int16_t)constrain(gyroXfilt * 10.0f, -32767.0f, 32767.0f);
  s.gyrY10 = (int16_t)constrain(gyroYfilt * 10.0f, -32767.0f, 32767.0f);
  s.m1 = motor1_speed;
  s.m2 = motor2_speed;
  s.m3 = motor3_speed;
  s.pwmX = trace_pwmX;
  s.pwmY = trace_pwmY;
  // Yaw: measured rate (computed here, so it is fresh in every mode), the
  // ramped command, and the effort XYZ_to_threeWay() actually applied.
  s.gyrZ10 = (int16_t)constrain(GyZ / GYRO_LSB_PER_DPS * 10.0f, -32767.0f, 32767.0f);
  s.ycmd10 = (int16_t)constrain(yaw_rate_cmd * 10.0f, -32767.0f, 32767.0f);
  s.pwmZ = trace_pwmZ;
  // seq is written LAST: a reader that sees the new seq is guaranteed the
  // rest of the sample is already in place.
  s.seq = trace_seq;
  trace_seq++;
}

// True while the control loop is actively driving the motors to balance.
// Mirrors the balancing branch conditions in loop(); calibration commands
// are refused while this is true.
bool balancingActive() {
  return armed && (vertical_vertex || vertical_edge) && calibrated && !calibrating;
}

int Tuning() {
  // Wired fallback for calibration, over USB serial.  This used to be the
  // Bluetooth channel; it moved to Serial when Bluetooth was removed, so
  // there is still a way in if the Wi-Fi access point ever fails to start.
  // The protocol is unchanged: two characters, a parameter then an action.
  // c+ starts calibration, c- records the current pose.
  if (!Serial.available())  return 0;
  char param = Serial.read();                 // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                   // get command byte
  switch (param) {
    case 'a':
      // Arm/disarm over the wire.  This is the way back in when the Wi-Fi
      // access point fails to start, which is also the case that disarms the
      // cube automatically - without this it would be unusable, not merely
      // unstoppable.  Routed through the same command flag the dashboard
      // uses, so both paths behave identically; Tuning() runs earlier in the
      // same loop iteration that consumes it.
      // A stop already waiting must never be overwritten by an arm - the
      // same rule the HTTP handler enforces.
      if (cmd == '+' && web_cmd_pending != WEB_CMD_STOP) {
        web_cmd_pending = WEB_CMD_ARM;
        Serial.println("Arming.");
      } else if (cmd == '-') {
        web_cmd_pending = WEB_CMD_DISARM;
        Serial.println("Disarming.");
      }
      break;
    case 'c':
      // Refuse to calibrate while the motors are actively balancing - the
      // same rule the web interface enforces.  The old Bluetooth path was
      // missing this check, so a c+ mid-balance dropped the cube.
      if (balancingActive()) {
        Serial.println("Refused: cannot calibrate while balancing. Disarm first.");
        break;
      }
      if (cmd == '+' && !calibrating) {
        calStart();
      }
      if (cmd == '-' && calibrating)  {
        calCapture();
      }
      break;
   }
   return 1;
}

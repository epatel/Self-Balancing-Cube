// Battery check for the ESP32 cube.  Holds all three motors stopped with the
// brake engaged, then prints the battery voltage every second on the serial
// monitor (115200 baud).
//
// The battery is read through the resistor divider on VBAT (GPIO34).  Three
// figures are printed so they can be compared with a multimeter:
//   raw      - analogRead() value, which is what the balancing firmware uses
//   fw       - raw / FW_DIVIDER, the balancing firmware's own battery formula
//   divider  - calibrated pin millivolts scaled by DIVIDER_RATIO
//
// Type the voltage your multimeter shows across the battery (e.g. 11.84) and
// press enter: the sketch prints the FW_DIVIDER value that would make the
// firmware read correctly on this cube.

#define BRAKE       26
#define BUZZER      27
#define VBAT        34

#define DIR1        4
#define PWM1        32
#define DIR2        15
#define PWM2        25
#define DIR3        5
#define PWM3        18

// Battery volts per volt at the VBAT pin.  Measured on this cube: 2.317 V at
// the pin with a 12.0 V pack (multimeter).  The 33 kΩ / 10 kΩ divider read off
// schematic.pdf would give 4.3, which under-reads by about 2 V, so the board
// evidently has different resistors.
#define DIVIDER_RATIO  5.18

// The balancing firmware computes analogRead(VBAT) / 225, warns below
// 10.5 V and disarms below 9.9 V (it smooths and debounces; this sketch
// shows the instantaneous value).  Keep these in step with esp32_cube_enc/ESP32.h.
#define FW_DIVIDER  225.0
#define FW_PRESENT  6.0
#define FW_WARN     10.5
#define FW_CUTOFF   9.9

#define CELLS       3        // 3S LiPo
#define SAMPLES     64       // averaged per reading to smooth ADC noise

float last_raw = 0;
float last_pin_v = 0;
String line;

void motorsSafe() {
  // The motor drivers use inverted PWM: a HIGH (or 255 duty) input is stopped
  // and a floating or LOW input is full drive.  Hold every PWM input HIGH and
  // the brake engaged (LOW) for as long as this sketch runs.
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, LOW);
  const int pwm[] = {PWM1, PWM2, PWM3};
  const int dir[] = {DIR1, DIR2, DIR3};
  for (int i = 0; i < 3; i++) {
    pinMode(pwm[i], OUTPUT);
    digitalWrite(pwm[i], HIGH);
    pinMode(dir[i], OUTPUT);
    digitalWrite(dir[i], HIGH);
  }
}

float cellPercent(float v) {
  // Approximate state of charge for one LiPo cell at rest.  Under load, or
  // just after charging, the voltage reads lower or higher than this assumes.
  const float volts[] = {3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
                         3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20};
  const int n = sizeof(volts) / sizeof(volts[0]);
  if (v <= volts[0]) return 0;
  if (v >= volts[n - 1]) return 100;
  for (int i = 1; i < n; i++) {
    if (v < volts[i]) {
      float f = (v - volts[i - 1]) / (volts[i] - volts[i - 1]);
      return (i - 1 + f) * 100.0 / (n - 1);
    }
  }
  return 100;
}

const char* status(float cell) {
  // With the battery unplugged and the ESP32 on USB, the battery rail is not
  // at 0 V: USB power leaks back into it (about 3.4 V measured), so treat
  // anything far below a usable pack as "no battery".
  if (cell < 2.0)  return "NO BATTERY (or a dead pack) - running from USB";
  if (cell < 3.0)  return "CRITICAL - disconnect and charge now";
  if (cell < 3.5)  return "LOW - charge before use";
  if (cell < 3.7)  return "getting low";
  if (cell > 4.25) return "ABOVE FULL - check the reading or the charger";
  return "OK";
}

void setup() {
  motorsSafe();                 // first, before anything slow
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  Serial.begin(115200);
  // 12-bit readings at the default 11 dB attenuation: full scale is about
  // 3.1 V at the pin, enough for a full 3S pack through the divider.
  analogReadResolution(12);
  delay(500);
  Serial.println();
  Serial.println("Battery check - motors held stopped, brake engaged.");
  Serial.println("Type your multimeter reading (e.g. 11.84) and press enter to calibrate.");
}

void loop() {
  static unsigned long previousT = 0;

  // Collect a typed multimeter reading.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      float measured = line.toFloat();
      line = "";
      if (measured > 1.0 && last_raw > 0) {
        Serial.print(">> measured "); Serial.print(measured, 2);
        Serial.print(" V: firmware divider should be ");
        Serial.print(last_raw / measured, 1);
        Serial.print(" (now "); Serial.print(FW_DIVIDER, 0); Serial.print(")");
        Serial.print(", DIVIDER_RATIO should be ");
        Serial.println(measured / last_pin_v, 2);
      }
    } else {
      line += c;
    }
  }

  if (millis() - previousT < 1000) return;
  previousT = millis();

  uint32_t raw_sum = 0, mv_sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    raw_sum += analogRead(VBAT);
    mv_sum += analogReadMilliVolts(VBAT);
  }
  last_raw = raw_sum / (float)SAMPLES;
  float pin_v = mv_sum / (float)SAMPLES / 1000.0;
  last_pin_v = pin_v;
  float batt_v = pin_v * DIVIDER_RATIO;
  float fw_v = last_raw / FW_DIVIDER;
  float cell = batt_v / CELLS;
  const char* fw_state = fw_v < FW_PRESENT ? "" :
                         (fw_v < FW_CUTOFF ? " [fw CUTOFF]" :
                         (fw_v < FW_WARN ? " [fw LOW]" : ""));

  Serial.print("raw "); Serial.print(last_raw, 0);
  Serial.print("  pin "); Serial.print(pin_v, 3); Serial.print(" V");
  Serial.print("  | divider "); Serial.print(batt_v, 2); Serial.print(" V");
  Serial.print(" ("); Serial.print(cell, 2); Serial.print(" V/cell, ~");
  Serial.print(cellPercent(cell), 0); Serial.print("%)");
  Serial.print("  | fw "); Serial.print(fw_v, 2); Serial.print(" V");
  Serial.print(fw_state);
  Serial.print("  -> "); Serial.println(status(cell));
}

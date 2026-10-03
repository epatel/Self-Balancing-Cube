// Pin map for the diagnostic sketch.  It matches the balancing firmware so
// this sketch can test the same assembled hardware.
#define BRAKE       26
#define BUZZER      27

// Motor numbers follow esp32_cube_enc/ESP32.h (remapped for this cube:
// 1 = D5 group, 2 = D4 group, 3 = D15 group; upstream was 1 = D4, 2 = D15,
// 3 = D5).
#define DIR1        5
#define ENC1_1      16
#define ENC1_2      17
#define PWM1        18
#define PWM1_CH     2

#define DIR2        4
#define ENC2_1      35
#define ENC2_2      33
#define PWM2        32
#define PWM2_CH     1

#define DIR3        15
#define ENC3_1      13
#define ENC3_2      14
#define PWM3        25
#define PWM3_CH     0

#define TIMER_BIT  8
#define BASE_FREQ  20000

// Encoder counts are updated by interrupts and sampled every 100 ms.
volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int16_t motor1_speed;         
int16_t motor2_speed;         
int16_t motor3_speed;    

long currentT, previousT_1, previousT_2;

int f;                     // Current test step, 1 through 18
boolean lock = false;      // Print each step's message only once

void beep() {
    // Active buzzer: HIGH sounds it, LOW silences it.
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

void pwmSet(uint8_t pin, uint32_t value) {
  // Send an 8-bit duty-cycle value to a PWM output.  ESP32 core 3.x
  // addresses LEDC by pin number (the old API used channel numbers).
  ledcWrite(pin, value);
}

void Motor1_control(int sp) {
  // This test sketch uses the motor driver's direction convention directly.
  // Its PWM is inverted: 255 is stopped and lower values apply more drive.
  if (sp > 0) digitalWrite(DIR1, LOW);
    else digitalWrite(DIR1, HIGH);
  pwmSet(PWM1, 255 - abs(sp));
}

void Motor2_control(int sp) {
  // Motor 2 has the same driver interface as motor 1.
  if (sp > 0) digitalWrite(DIR2, LOW);
    else digitalWrite(DIR2, HIGH);
  pwmSet(PWM2, 255 - abs(sp));
}

void Motor3_control(int sp) {
  // Motor 3 has the same driver interface as motor 1.
  if (sp > 0) digitalWrite(DIR3, LOW);
    else digitalWrite(DIR3, HIGH);
  pwmSet(PWM3, 255 - abs(sp));
}

void ENC1_READ() {
  // Decode the two encoder channels as a quadrature signal.  The transition
  // table increments for one direction and decrements for the other.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}

void ENC2_READ() {
  // Quadrature decoder for motor 2's encoder.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}

void ENC3_READ() {
  // Quadrature decoder for motor 3's encoder.
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}

void setup() {

  // Initialize the brake, each motor's direction/PWM pins, and both encoder
  // interrupts.  Motor outputs start at zero before testing begins.
  Serial.begin(115200);

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  
  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
  // ESP32 core 3.x API: ledcAttach() replaces ledcSetup()+ledcAttachPin()
  // and manages the channel internally; PWM is now addressed by pin.
  ledcAttach(PWM1, BASE_FREQ, TIMER_BIT);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
  ledcAttach(PWM2, BASE_FREQ, TIMER_BIT);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  ledcAttach(PWM3, BASE_FREQ, TIMER_BIT);
  Motor3_control(0);

  delay(2000);
}

void loop() {

    currentT = millis();
    
    // Sample encoder counts every 100 ms and execute the selected test step.
    if (currentT - previousT_1 >= 100) {
      motor1_speed = enc_count1;
      enc_count1 = 0;
      motor2_speed = enc_count2;
      enc_count2 = 0;
      motor3_speed = enc_count3;
      enc_count3 = 0;

      // Each motor has six steps: rotate one way, stop, rotate the other way,
      // stop, run while checking the encoder, then report the result.
      switch (f) {
       case 1:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Rotating motor 1.");
        lock = 0;
        Motor1_control(-50);
        break;
      case 2:
        digitalWrite(BRAKE, LOW);
        if (lock) Serial.println("Stop.");
        lock = 0;
        Motor1_control(0);
        break;      
      case 3:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Change direction...");
        lock = 0;
        Motor1_control(50);
        break;
      case 4:
        digitalWrite(BRAKE, LOW);
        if (lock) Serial.println("Stop.");
        lock = 0;
        Motor1_control(0);
        break; 
      case 5:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Checking encoder...");
        lock = 0;
        Motor1_control(-70);
        break; 
      case 6:
        digitalWrite(BRAKE, LOW);
        if (lock && motor1_speed > 300) {
          Serial.println("Encoder OK.");
          Serial.print("Speed: "); Serial.println(motor1_speed);
          Serial.println("Stop.");
        } else if (lock && motor1_speed <= 0) {
          Serial.println("Encoder FAIL.");
          Serial.print("Speed: "); Serial.println(motor1_speed);
          Serial.println("Stop.");
        }
        lock = 0;
        Motor1_control(0);
        break;      

      case 7:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Rotating motor 2.");
        lock = 0;
        Motor2_control(-50);
        break;
      case 8:
        digitalWrite(BRAKE, LOW);
        if (lock) Serial.println("Stop.");
        lock = 0;
        Motor2_control(0);
        break;      
      case 9:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Change direction...");
        lock = 0;
        Motor2_control(50);
        break;
      case 10:
        digitalWrite(BRAKE, LOW);
        if (lock) Serial.println("Stop.");
        lock = 0;
        Motor2_control(0);
        break; 
      case 11:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Checking encoder...");
        lock = 0;
        Motor2_control(-70);
        break; 
      case 12:
        digitalWrite(BRAKE, LOW);
        if (lock && motor2_speed > 300) {
          Serial.println("Encoder OK.");
          Serial.print("Speed: "); Serial.println(motor2_speed);
          Serial.println("Stop.");
        } else if (lock && motor2_speed <= 0) {
          Serial.println("Encoder FAIL.");
          Serial.print("Speed: "); Serial.println(motor2_speed);
          Serial.println("Stop.");
        }
        lock = 0;
        Motor2_control(0);
        break;        

      case 13:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Rotating motor 3.");
        lock = 0;
        Motor3_control(-50);
        break;
      case 14:
        digitalWrite(BRAKE, LOW);
        if (lock) Serial.println("Stop.");
        lock = 0;
        Motor3_control(0);
        break;      
      case 15:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Change direction...");
        lock = 0;
        Motor3_control(50);
        break;
      case 16:
        digitalWrite(BRAKE, LOW);
        if (lock) Serial.println("Stop.");
        lock = 0;
        Motor3_control(0);
        break; 
      case 17:
        digitalWrite(BRAKE, HIGH);
        if (lock) Serial.println("Checking encoder...");
        lock = 0;
        Motor3_control(-70);
        break; 
      case 18:
        digitalWrite(BRAKE, LOW);
        if (lock && motor3_speed > 300) {
          Serial.println("Encoder OK.");
          Serial.print("Speed: "); Serial.println(motor3_speed);
          Serial.println("Stop.");
        } else if (lock && motor3_speed <= 0) {
          Serial.println("Encoder FAIL.");
          Serial.print("Speed: "); Serial.println(motor3_speed);
          Serial.println("Stop.");
        }
        lock = 0;
        Motor3_control(0);
        break;        
      }
      previousT_1 = currentT;
    }
    
    // Move to the next test step every three seconds.  After all 18 steps,
    // start again with motor 1 and beep to mark the new cycle.
    if (currentT - previousT_2 >= 3000) { 
     f++;
     if (f == 19) {
      f = 1;
      beep();
      Serial.println("Beep!");
     }
     lock = 1;
     previousT_2 = currentT;
  }
  
}

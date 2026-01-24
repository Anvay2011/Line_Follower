#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <vector>
#include <Preferences.h>   // NVS storage

std::vector<String> myList;   // path storage
Preferences prefs;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// === Encoder settings & pins ===
#define L_ENC_A 19
#define L_ENC_B 23
#define R_ENC_A 5
#define R_ENC_B 18

volatile long leftCount = 0;
volatile long rightCount = 0;

const int CPR = 1414;                    // counts per revolution (your measured value)
const float wheel_circumference_cm = 13.2; 
// --- Motor pins ---
#define lf 14
#define lb 27
#define rf 13
#define rb 12
String shortestPath = "";  
int pathIndex = 0;
// --- Sensors ---
#define NUM_SENSORS 8
int stateVar = 0;              // variable you want to set
int lastButtonState = HIGH;  
int L4 = 26, L3 = 25, L2 = 33, L1 = 32;
int R1 = 35, R2 = 34, R3 = 15, R4 = 2;
int run = 0;
int end = 0;
int sensorPins[NUM_SENSORS] = {L4, L3, L2, L1, R1, R2, R3, R4};
int sensorValues[NUM_SENSORS];
int l4, l3, l2, l1, r1, r2, r3, r4;
float targetYaw = 0.0;   
// --- PID Params ---
int speed = 255;
int maxSpeed = speed;
int baseSpeed = 255;    // cruising forward speed when straight
int minSpeed = 150;     // minimum motor speed (avoid stalling)
int prevLeftSpeed = 0;
int prevRightSpeed = 0;
const int SLEW_LIMIT = 18;
float Kp = 50000000000.0;
float Kp2 = 110.0;

// Behavior tuning
const int CENTER_DEADZONE = 200; // error units (0..7000) treated as "straight"
const int CENTER_LOW_INDEX = 3;  // index of left-center sensor
const int CENTER_HIGH_INDEX = 4; // index of right-center sensor

// Last known error (used for lost-line recovery)
int lastKnownError = 0;// --- Utils ---
float getYaw() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

float wrapAngle(float angle) {
  if (angle > 180) angle -= 360;
  if (angle < -180) angle += 360;
  return angle;
}



// === Simplification Logic ===
String simplifyPath(String path) {
  bool changed = true;
  while (changed) {
    changed = false;

    if (path.indexOf("LUL") != -1) {
      path.replace("LUL", "S");
      changed = true;
    }
    else if (path.indexOf("RUR") != -1) {
      path.replace("RUR", "S");
      changed = true;
    }
    else if (path.indexOf("LUR") != -1) {
      path.replace("LUR", "U");
      changed = true;
    }
    else if (path.indexOf("RUL") != -1) {
      path.replace("RUL", "U");
      changed = true;
    }
    else if (path.indexOf("LUS") != -1) {
      path.replace("LUS", "R");
      changed = true;
    }
    else if (path.indexOf("SUL") != -1) {
      path.replace("SUL", "R");
      changed = true;
    }
    else if (path.indexOf("RUS") != -1) {
      path.replace("RUS", "L");
      changed = true;
    }
    else if (path.indexOf("SUR") != -1) {
      path.replace("SUR", "L");
      changed = true;
    }
    else if (path.indexOf("SUS") != -1) {
      path.replace("SUS", "U");
      changed = true;
    }
  }
  return path;
}

// === NVS ===
String readPathFromNVS() {
  prefs.begin("storage", true);
  String storedPath = prefs.getString("path", "");
  prefs.end();
  return storedPath;
}

void clearNVS() {
  prefs.begin("storage", false);
  prefs.clear();
  prefs.end();
}
void applyMotors(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  // Slew limiter (smooth transitions)
  int dL = leftPWM - prevLeftSpeed;
  if (dL > SLEW_LIMIT) leftPWM = prevLeftSpeed + SLEW_LIMIT;
  else if (dL < -SLEW_LIMIT) leftPWM = prevLeftSpeed - SLEW_LIMIT;

  int dR = rightPWM - prevRightSpeed;
  if (dR > SLEW_LIMIT) rightPWM = prevRightSpeed + SLEW_LIMIT;
  else if (dR < -SLEW_LIMIT) rightPWM = prevRightSpeed - SLEW_LIMIT;

  // Drive left motor
  if (leftPWM >= 0) {
    analogWrite(lf, leftPWM);
    analogWrite(lb, 0);
  } else {
    analogWrite(lf, 0);
    analogWrite(lb, -leftPWM);
  }

  // Drive right motor
  if (rightPWM >= 0) {
    analogWrite(rf, rightPWM);
    analogWrite(rb, 0);
  } else {
    analogWrite(rf, 0);
    analogWrite(rb, -rightPWM);
  }

  prevLeftSpeed = leftPWM;
  prevRightSpeed = rightPWM;
}
void lineFollowP() {
  for (int i = 0; i < 8; i++) {
    // invert so black line -> 1, white -> 0 (user requested that logic)
    sensorValues[i] = !digitalRead(sensorPins[i]);
  }

  // Weighted position (indices 0..7 => weights 0..7000)
  long numerator = 0;
  int activeCount = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i]) {
      numerator += (long)i * 1000L;
      activeCount++;
    }
  }

  // If no sensors active -> lost line
  if (activeCount == 0) {
    // gentle search toward last known side (use lastKnownError sign)
    int steer = 0;
    if (lastKnownError > 0) steer = 80;       // steer right
    else if (lastKnownError < 0) steer = -80; // steer left
    else steer = 0; // unknown, go moderately forward

    int leftSpeed = baseSpeed - steer;
    int rightSpeed = baseSpeed + steer;

    // keep within min/max
    leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

    applyMotors(leftSpeed, rightSpeed);
    return;
  }

  int position = (int)(numerator / activeCount); // 0..7000
  const int center = 3500;
  int error = position - center; // negative -> line left, positive -> line right

  // Remember last known error for recovery
  lastKnownError = error;

  // If error is small => treat as straight line (no correction)
  if (abs(error) <= CENTER_DEADZONE) {
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;
    applyMotors(leftSpeed, rightSpeed);
    return;
  }

  // Quick central-sensors shortcut: if only the two center sensors detect line, go straight
  bool onlyCenter = (sensorValues[CENTER_LOW_INDEX] && sensorValues[CENTER_HIGH_INDEX]);
  for (int i = 0; i < 8; i++) {
    if (i != CENTER_LOW_INDEX && i != CENTER_HIGH_INDEX && sensorValues[i]) {
      onlyCenter = false;
      break;
    }
  }
  if (onlyCenter) {
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;
    applyMotors(leftSpeed, rightSpeed);
    return;
  }

  // Proportional correction (scale error so it maps well to PWM)
  // position range center..edge ~= 3500 -> use /3500 to normalize
  int rawCorrection = (int)(Kp * (float)error / 3500.0);

  // Constrain correction so P doesn't overshoot strongly on sharp errors
  int maxCorrection = baseSpeed - minSpeed; // prevent stopping one motor completely
  rawCorrection = constrain(rawCorrection, -maxCorrection, maxCorrection);

  // If the outer sensors are active, we can allow slightly larger corrections
  if (sensorValues[0] || sensorValues[7]) {
    // outermost sensor triggered -> increase correction margin slightly (still P-only)
    rawCorrection = constrain(rawCorrection, -maxCorrection, maxCorrection);
  }

  // Compute motor speeds
  int leftSpeed = baseSpeed - rawCorrection;
  int rightSpeed = baseSpeed + rawCorrection;

  // Ensure min and max
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  applyMotors(leftSpeed, rightSpeed);
}



void update() {
  r4 = !digitalRead(R4);
  r3 = !digitalRead(R3);
  r2 = !digitalRead(R2);
  r1 = !digitalRead(R1);
  l4 = !digitalRead(L4);
  l3 = !digitalRead(L3);
  l2 = !digitalRead(L2);
  l1 = !digitalRead(L1);
}
int conv(int spd){
  int k =160*200;
  int delay_time =k/spd;
  return delay_time;

}

// ---------- NEW forward(speed, distance_cm) using driveMotors ----------
void forward(int spd, float distance_cm) {

  // Reset encoder counts
  leftCount = 0;
  rightCount = 0;

  // Convert target distance → encoder counts
  long targetCounts = (long)((distance_cm / wheel_circumference_cm) * CPR);

  // Start motors forward
  driveMotors(spd, spd);

  // Move until BOTH wheels reach the target
  while ((leftCount < targetCounts) && (rightCount < targetCounts)) {
    // Do nothing → just wait until encoders reach distance
    delay(1);  
  }

  // Stop motors
  stop();

}

// ---------- STOP function ----------
void stop() {
  analogWrite(lf, 0); analogWrite(lb, 0);
  analogWrite(rf, 0); analogWrite(rb, 0);
}

// ---------- Encoder ISRs ----------
void IRAM_ATTR isr_left() {
  int A = digitalRead(L_ENC_A);
  int B = digitalRead(L_ENC_B);
  if (A == B) leftCount++;
  else leftCount--;
}

void IRAM_ATTR isr_right() {
  int A = digitalRead(R_ENC_A);
  int B = digitalRead(R_ENC_B);
  if (A == B) rightCount++;
  else rightCount--;
}

void turnToYaw(float targetYaw) {
  unsigned long startTime = millis();

  while (true) {
    float currentYaw = getYaw();
    float error = wrapAngle(targetYaw - currentYaw); // -180 .. +180
    float absError = abs(error);
    update();
    if((l1 == 0 || r1 == 0)&& absError <45){
      break;
    }
    // Stop condition
    if (absError <= 2.0) break;
    if (millis() - startTime > 4000) break;  // safety timeout

    // Proportional speed control
    int turnSpeed = (int)(absError * 3.0);   // KP ≈ 3
    turnSpeed = constrain(turnSpeed, 70, 220);

    // Direction based on sign of error
    if (error > 0) {
      // CW
      driveMotors(turnSpeed, -turnSpeed);
    } else {
      // CCW
      driveMotors(-turnSpeed, turnSpeed);
    }

    delay(10);
  }

  stop();
  delay(50);
}
void left(int speed) {
  float startYaw = getYaw();
  float targetYaw = startYaw - 110.0;
  if (targetYaw < 0) targetYaw += 360.0;

  turnToYaw(targetYaw);
}
void right(int speed) {
  float startYaw = getYaw();
  float targetYaw = startYaw + 110.0;
  if (targetYaw >= 360.0) targetYaw -= 360.0;

  turnToYaw(targetYaw);
}


// ---- U-TURN (180° shortest spin) ----
void u_turn(int speed) {
  float startYaw = getYaw();
  float targetYaw = startYaw - 170.0;
  if (targetYaw < 0) targetYaw += 360.0;

  // Force CCW rotation
  while (true) {
    float currentYaw = getYaw();
    float error = wrapAngle(targetYaw - currentYaw);
      if((l1 == 0 || l2 == 0)&& abs(error) <45){
      break;
    }
    if (abs(error) <= 2.0) break;

    int turnSpeed = constrain(abs(error) * 3.0, 80, 220);

    driveMotors(-turnSpeed, turnSpeed);  // CCW ONLY
    delay(10);
  }

  stop();
}



void driveMotors(int leftPWM, int rightPWM) {
  // Clamp PWM values between -255 and 255
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM > 0) {
    analogWrite(lf, leftPWM);
    analogWrite(lb, 0);
  } else if (leftPWM < 0) {
    analogWrite(lf, 0);
    analogWrite(lb, -leftPWM);
  } else {
    analogWrite(lf, 0);
    analogWrite(lb, 0);
  }

  if (rightPWM > 0) {
    analogWrite(rf, rightPWM);
    analogWrite(rb, 0);
  } else if (rightPWM < 0) {
    analogWrite(rf, 0);
    analogWrite(rb, -rightPWM);
  } else {
    analogWrite(rf, 0);
    analogWrite(rb, 0);
  }
}

void alignToLine(int alignSpeed = 150) {
  update();

  // Already aligned
  if (l1 == 0 || r1 == 0) {
    return;
  }

  // Line detected on RIGHT side → rotate RIGHT
  if (r2 == 0 || r3 == 0 || r4 == 0) {
    while (true) {
      update();
      if (l1 == 0 || r1 == 0) break;

      analogWrite(lf, alignSpeed);
      analogWrite(lb, 0);
      analogWrite(rf, 0);
      analogWrite(rb, alignSpeed);
    }
  }
  // Line detected on LEFT side → rotate LEFT
  else if (l2 == 0 || l3 == 0 || l4 == 0) {
    while (true) {
      update();
      if (l1 == 0 || r1 == 0) break;

      analogWrite(lf, 0);
      analogWrite(lb, alignSpeed);
      analogWrite(rf, alignSpeed);
      analogWrite(rb, 0);
    }
  }
}

#define BUTTON_PIN 4  // your button connected to GPIO14 and GND
#define led 16
void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);   // active-HIGH button
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // --- Motor pins ---
  pinMode(lf, OUTPUT);
  pinMode(lb, OUTPUT);
  pinMode(rf, OUTPUT);
  pinMode(rb, OUTPUT);

  // --- Encoder pins ---
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), isr_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), isr_right, CHANGE);

  // --- Sensor pins ---
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // ===============================
  //  LOAD STORED PATH FROM NVS
  // ===============================
  shortestPath = readPathFromNVS();  // empty string if none
  pathIndex = 0;
  end = 0;
  myList.clear();

  // ===============================
  //  BUTTON PRESS DURATION CHECK
  // ===============================

  // Wait for button press
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(5);
  }

  unsigned long pressStart = millis();

  // Wait while button is held
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(5);
  }

  unsigned long pressDuration = millis() - pressStart;

  // -------------------------------
  //  SELECT RUN MODE
  // -------------------------------
  if (pressDuration > 4000) {
    run = 2;            // > 4s → debug / calibration
  }
  else if (pressDuration >= 2000) {
    clearNVS();
    run = 1;            // 2–4s → shortest path
  }
  else {
        clearNVS();

    run = 0;            // < 2s → DFS explore
  }

  // ===============================
  //  LED FEEDBACK
  // ===============================
  blinkLED(run + 1);    // run=0→1 blink, run=1→2, run=2→3

  // ===============================
  //   IMU INITIALIZATION
  // ===============================
  if (!bno.begin()) {
    while (1); // IMU failure
  }

  bno.setExtCrystalUse(true);
  delay(1000); // allow gyro to stabilize

  stateVar = 0;

  delay(50);
  delay(50);

  Serial.print("Run mode selected: ");
  Serial.println(run);
}



void loop() {
  String storedPath = readPathFromNVS();
  int buttonState = digitalRead(BUTTON_PIN);
  if(run == 0){
update();

  // =========================
  // ===== DFS EXPLORATION ===


    if (end == 1) {
      stop();
      delay(100);
      digitalWrite(16, HIGH);

      if (storedPath == "") {
        String rawPath = "";
        for (auto &s : myList) rawPath += s;
        String shortest = simplifyPath(rawPath);
        prefs.begin("storage", false);
        prefs.putString("path", shortest);
        prefs.end();
      }
      while (1);
    }

    // -------- JUNCTION CANDIDATE DETECTION --------
bool junctionCandidate =
    (r3 == 0 && r4 == 0 && l3 == 0 && l4 == 0 && r2 == 0 && r1 == 0 && l2 == 0 && l1 == 0) ||
    (r2 == 0 && r3 == 0 && r4 == 0 && l3 == 1 && l4 == 1) ||
    (r3 == 1 && r4 == 1 && l3 == 0 && l4 == 0 && l2 == 0)||(r3 == 1 && r4 == 1 && l3 == 1 && l4 == 1 && r2 == 1 && r1 == 1 && l2 == 1 && l1 == 1);

    if (junctionCandidate) {
      // 🔹 STEP 1: move slightly into junction
      stop();
      forward(speed, 1);
      delay(5);
      update();

      // 🔹 STEP 2: NOW evaluate full junction logic

      // ===== DEAD END / END NODE =====
      if (r3 == 0 && r4 == 0 && l3 == 0 && l4 == 0 &&
          r2 == 0 && r1 == 0 && l2 == 0 && l1 == 0) {

        stop();
        forward(speed, 6);
        stop();
        update();

        if (r3 == 0 && r4 == 0 && l3 == 0 && l4 == 0 &&
            r2 == 0 && r1 == 0 && l2 == 0 && l1 == 0) {
          end = 1;
        } else {
          float startYaw = getYaw();
          float target = startYaw - 90;
          if (target < 0) target += 360;
          left(speed);
          if (abs(wrapAngle(getYaw() - target)) < 50) {
            myList.push_back("L");
          }
        }
        return;
      }

      // ===== LEFT JUNCTION =====
      if (r3 == 1 && r4 == 1 && l3 == 0 && l4 == 0 && l2 == 0) {
        stop();
        forward(speed, 6);
        stop();
        update();

        if (r3 == 1 && r4 == 1 && l3 == 0 && l4 == 0 && l2 == 0) {
          end = 1;
        } else {
          float target = getYaw() - 90;
          if (target < 0) target += 360;
          left(speed);
          if (abs(wrapAngle(getYaw() - target)) < 50) {
            myList.push_back("L");
          }
        }
        return;
      }

      // ===== RIGHT / STRAIGHT =====
      if (r3 == 0 && r4 == 0 && l3 == 1 && l4 == 1 && r2 == 0) {
        stop();
        forward(speed, 6);
        stop();
        update();

        if (r3 == 0 && r4 == 0 && l3 == 1 && l4 == 1) {
          end = 1;
        } else {
          if (l1 && l2 && l3 && l4 && r1 && r2 && r3 && r4) {
            float target = getYaw() + 80;
            right(speed);
            if (abs(wrapAngle(getYaw() - target)) < 40) {
              myList.push_back("R");
            }
          } else {
            myList.push_back("S");
          }
        }
        return;
      }

      // ===== U TURN =====
      if (l1 && l2 && l3 && l4 && r1 && r2 && r3 && r4) {
        forward(speed, 2);
        u_turn(speed);
        myList.push_back("U");
        return;
      }
    }

    // -------- NO JUNCTION → LINE FOLLOW --------
    lineFollowP();
  }

  // =========================
  // ===== SHORTEST PATH RUN ==
  // ========================
else if(run ==1){
    // Always update sensor reads first
  update();

  // Define the initial "junction candidate" check you used at top before.
  bool initialJunctionCandidate =
    (r3 == 0 && r4 == 0 && l3 == 0 && l4 == 0 && r2 == 0 && r1 == 0 && l2 == 0 && l1 == 0) ||
    (r2 == 0 && r3 == 0 && r4 == 0 && l3 == 1 && l4 == 1) ||
    (r3 == 1 && r4 == 1 && l3 == 0 && l4 == 0 && l2 == 0)||(r3 == 1 && r4 == 1 && l3 == 1 && l4 == 1 && r2 == 1 && r1 == 1 && l2 == 1 && l1 == 1);

  if (initialJunctionCandidate) {
    // Stop, take a small forward step (1 cm), then re-check all the junction logic.
    stop();
    forward(speed, 1);
    // allow sensors/IMU to stabilise, then refresh sensor state
    delay(8);
    update();

    // === Now re-evaluate the full junction handling / end logic ===
    if (end == 1) {
      stop();
      delay(100);
      digitalWrite(16, HIGH);
      if (storedPath == "") {
        String rawPath = "";
        for (auto &s : myList) rawPath += s;
        delay(10);
        String shortest = simplifyPath(rawPath);
        prefs.begin("storage", false);
        prefs.putString("path", shortest);
        prefs.end();
        delay(10);
      }
      while (1); // halt
    } else {
      update();

      if (r3 == 0 && r4 == 0 && l3 == 0 && l4 == 0 && r2 == 0 && r1 == 0 && l2 == 0 && l1 == 0) {
        stop();
        forward(speed, 6);
        stop();
        update();

        if (r3 == 0 && r4 == 0 && l3 == 0 && l4 == 0 && r2 == 0 && r1 == 0 && l2 == 0 && l1 == 0) {
          end = 1;
          delay(30);
        } else {
          float startYaw = getYaw();
          float target = startYaw + 90;
          if (target < 0) target -= 360;
          right(speed);
          float end_yaw = getYaw();
          float error = wrapAngle(end_yaw - target);
          if (abs(error) <= 50) {
            myList.push_back("R");
            delay(30);
          }
        }
        return;
      } 

      else if (r2 == 0 && r3 == 0 && r4 == 0 && l3 == 1 && l4 == 1) {
        stop();
        forward(speed, 6);
        stop();
        update();

        if (r2 == 0 && r3 == 0 && r4 == 0 && l3 == 1 && l4 == 1) {
          delay(30);
          end = 1;
        } else {
          float startYaw = getYaw();
          float target = startYaw + 90;
          if (target < 0) target -= 360;
          right(speed);
          float end_yaw = getYaw();
          float error = wrapAngle(end_yaw - target);
          if (abs(error) <= 50) {
            delay(30);
            myList.push_back("R");
          }
        }
        return;
      }  

      else if (r3 == 1 && r4 == 1 && l3 == 0 && l4 == 0 && l2 == 0) {
        float startYaw = getYaw();
        delay(30);
        stop();
        forward(speed, 6);
        stop();
        update();

        if (r3 == 1 && r4 == 1 && l3 == 0 && l4 == 0 && l2 == 0) {
          end = 1;
          delay(30);
        } else {
          if (r3 == 1 && r4 == 1 && l3 == 1 && l4 == 1 && r2 == 1 && r1 == 1 && l2 == 1 && l1 == 1) {
            float target = startYaw - 90;
            if (target < 0) target += 360;
            left(speed);
            float end_yaw = getYaw();
            float error = wrapAngle(end_yaw - target);
            if (abs(error) <= 40) {
              delay(30);
              myList.push_back("L");
            } 
          } else {
     
              delay(30);
              myList.push_back("S");
             
            return;
          }
          return;
        }
      }

      else if (r3 == 1 && r4 == 1 && l3 == 1 && l4 == 1 && r2 == 1 && r1 == 1 && l2 == 1 && l1 == 1) {
        forward(speed, 2);
        float startYaw = getYaw();
        float target = startYaw - 180;
        if (target < 0) target += 360;
        u_turn(speed);
        float end_yaw = getYaw();
        float error = wrapAngle(end_yaw - target);
        if (abs(error) <= 25) {
          delay(30);
          myList.push_back("U");
        } 
        return;
      }

      else if (
        (l4 == 0 && l3 == 1 && l2 == 1 && l1 == 1 &&
         r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1) ||
        (l3 == 0 && l4 == 1 && l2 == 1 && l1 == 1 &&
         r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1) ||
        (l2 == 0 && l3 == 0 && l4 == 1 && l1 == 1 &&
         r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1) ||
        (l4 == 0 && l3 == 1 && l2 == 0 && l1 == 1 &&
         r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1) ||
        (l4 == 0 && l3 == 0 && l2 == 1 && l1 == 1 &&
         r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1) ||
        (l4 == 0 && l2 == 0 && l3 == 1 && l1 == 1 &&
         r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1) ||
        (l4 == 0 && r4 == 0 &&
         l3 == 1 && l2 == 1 && l1 == 1 &&
         r1 == 1 && r2 == 1 && r3 == 1) ||
        (l3 == 0 && r3 == 0 &&
         l4 == 1 && l2 == 1 && l1 == 1 &&
         r1 == 1 && r2 == 1 && r4 == 1)
      ) {
        while (true) {
          update();
          if (l1 == 0) break;
          analogWrite(lf, 0);
          analogWrite(lb, 100);
          analogWrite(rf, 100);
          analogWrite(rb, 0);
        }
      }

      else if (
        (r4 == 0 && r3 == 1 && r2 == 1 && r1 == 1 &&
         l1 == 1 && l2 == 1 && l3 == 1 && l4 == 1) ||
        (r3 == 0 && r4 == 1 && r2 == 1 && r1 == 1 &&
         l1 == 1 && l2 == 1 && l3 == 1 && l4 == 1) ||
        (r2 == 0 && r3 == 0 && r4 == 1 && r1 == 1 &&
         l1 == 1 && l2 == 1 && l3 == 1 && l4 == 1) ||
        (r4 == 0 && r3 == 1 && r2 == 0 && r1 == 1 &&
         l1 == 1 && l2 == 1 && l3 == 1 && l4 == 1) ||
        (r4 == 0 && r3 == 0 && r2 == 1 && r1 == 1 &&
         l1 == 1 && l2 == 1 && l3 == 1 && l4 == 1) ||
        (r4 == 0 && r2 == 0 && r3 == 1 && r1 == 1 &&
         l1 == 1 && l2 == 1 && l3 == 1 && l4 == 1)
      ) {
        while (true) {
          update();
          if (r1 == 0) break;
          analogWrite(lf, 100);
          analogWrite(lb, 0);
          analogWrite(rf, 0);
          analogWrite(rb, 100);
        }
      }
    } // end of else (not end==1)

    // After handling the junction-case we return to loop() start (or exit via returns above)
    return;
  } // end of if(initialJunctionCandidate)

  // Not at an initial junction candidate -> normal continuous behavior
  // Always line-follow when not doing a junction step
  lineFollowP();
}
else if(run == 2){
  if (pathIndex >= shortestPath.length()) { 
          clearNVS(); 
run = 0;    stop(); }


  update();

  // 🚨 Check if all sensors detect black → stop immediately
  bool allBlack = (l1 == 1 && l2 == 1 && l3 == 1 && l4 == 1 &&
                   r1 == 1 && r2 == 1 && r3 == 1 && r4 == 1);

  if (allBlack) {
    stop();
    forward(speed,200);
    u_turn(speed);
    Serial.println("⚫ All sensors black — stopping and ending run.");
    return;  // stop everything immediately
  }

  // 🌀 Check if current move is 'U' before junction logic
 

  // ---- Normal junction detection for L / R / S ----
  if ((l2 == 0 && l3 == 0 && l4 == 0) || (r2 == 0 && r3 == 0 && r4 == 0)) {
    if (pathIndex < shortestPath.length()) {
      char move = shortestPath[pathIndex++];

      if (move == 'L') {

        forward(speed, 6);
        left(speed);

  delay(50);
      }
      else if (move == 'R') {

        forward(speed, 6);
        right(speed);

  delay(50);
      }
      else if (move == 'U') {
        forward(speed, 2);
        u_turn(speed);
      }
      else if (move == 'S') {

        forward(speed, 6);
        stop();
      }
    }
  } 
  else {
    lineFollowP();
  }

}
}



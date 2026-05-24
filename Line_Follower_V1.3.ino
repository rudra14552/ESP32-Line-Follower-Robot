/*
  ============================================================
  PID LINE FOLLOWER — ESP32 + DRV8833 + SmartElex 5-IR + BT
  ============================================================
  Device name : Roshogulla
  PIN         : 080326

  SENSOR: SmartElex 5-ch analog IR tracker
          Low raw  = BLACK line
          High raw = WHITE surface
          sensorNorm: 1.0 = on black line, 0.0 = white

  MOTOR DRIVER: DRV8833
          Forward : (PWM, 0)
          Reverse : (0, PWM)
          Brake   : (0, 0)

  PINOUT
  ------
  IR Sensors : IR1-33  IR2-32  IR3-35  IR4-34  IR5-G4
  Motor L    : IN1-G25  IN2-G26
  Motor R    : IN3-G27  IN4-G14
  Button CAL : G18  (press to start calibration)
  Button RUN : G19  (press to start line following)
  LED        : G5   (blinks during calibration, solid when following)

  NEW FEATURES
  ------------
  1. Calibration + PID + Speed saved to NVS — survives power off
  2. G18 button → start calibration (same as 'c' command)
  3. G19 button → start line following (same as 's' command)
  4. LED blinks once per second during calibration
  5. Auto-stop if line is lost for 1 second

  BLUETOOTH COMMANDS (all still work)
  ------------------------------------
  'c'     → Start 10-sec calibration
  's'     → Start line following (after calibration)
  'x'     → Stop
  '?'     → Status
  'Px'    → Set Kp  e.g. P2.5
  'Ix'    → Set Ki  e.g. I0.01
  'Dx'    → Set Kd  e.g. D1.8
  'UPx'   → Set speed to x%  e.g. UP35
  'DOWNx' → Set speed to x%  e.g. DOWN20
  ============================================================
*/

#include "BluetoothSerial.h"
#include <Preferences.h>        // ESP32 NVS (replaces EEPROM, more reliable)

BluetoothSerial SerialBT;
Preferences     prefs;

// ── IR Sensor pins ────────────────────────────────────────────
const int IR_PIN[5] = {33, 32, 35, 34, 4};

// ── DRV8833 Motor pins ────────────────────────────────────────
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

// ── Button & LED pins ─────────────────────────────────────────
#define BTN_CAL  18   // press → start calibration
#define BTN_RUN  19   // press → start line following
#define LED_PIN   5   // blinks during calib, solid while following

// ── PWM config (ESP32 core v3) ────────────────────────────────
#define PWM_FREQ  5000
#define PWM_RES   8

// ── Calibration ───────────────────────────────────────────────
const unsigned long CALIB_DURATION   = 10000;  // 10 sec
const unsigned long LINE_LOST_LIMIT  = 1000;   // 1 sec before auto-stop

int   sensorMin[5], sensorMax[5];
float sensorNorm[5];

bool          calibDone    = false;
bool          calibRunning = false;
unsigned long calibStart   = 0;

// ── PID ───────────────────────────────────────────────────────
float Kp = 0.08;
float Ki = 0.0;
float Kd = 0.6;

int   previousError = 0;
float I_term        = 0;

// ── Speed & state ─────────────────────────────────────────────
int  baseSpeed = 51;
bool following = false;

// ── Line-lost auto-stop timer ─────────────────────────────────
unsigned long lineLostSince = 0;   // millis() when line was first lost
bool          lineLost      = false;

// ── BT command buffer ─────────────────────────────────────────
String btBuffer = "";

// ── Button debounce ───────────────────────────────────────────
bool          lastBtn18       = HIGH;
bool          lastBtn19       = HIGH;
unsigned long lastBtn18Time   = 0;
unsigned long lastBtn19Time   = 0;
const unsigned long DEBOUNCE  = 50;

// ── Forward declarations ──────────────────────────────────────
void stopMotors();
void motorDrive(int left, int right);
void startCalibration();
void startFollowing();
void parseCommand(String cmd);
void handleCalibration();
void handleBluetooth();
void handleButtons();
void handleLED();
void runPID();
void saveCalibToNVS();
void loadCalibFromNVS();
void savePIDToNVS();
void loadPIDFromNVS();

// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  SerialBT.begin("Roshogulla");
  SerialBT.setPin("080326", 6);

  // Buttons with internal pullup (press = LOW)
  pinMode(BTN_CAL, INPUT_PULLUP);
  pinMode(BTN_RUN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // DRV8833 PWM
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcAttach(IN2, PWM_FREQ, PWM_RES);
  ledcAttach(IN3, PWM_FREQ, PWM_RES);
  ledcAttach(IN4, PWM_FREQ, PWM_RES);

  stopMotors();

  // Load saved calibration + PID + speed from NVS
  loadCalibFromNVS();
  loadPIDFromNVS();

  if (calibDone) {
    SerialBT.println("Calibration loaded from memory.");
    SerialBT.println("Press G19 or send 's' to start. Send 'c' or press G18 to recalibrate.");
    Serial.println("Calibration loaded from NVS.");
  } else {
    SerialBT.println("No calibration found. Press G18 or send 'c' to calibrate.");
    Serial.println("No calibration in NVS.");
  }
}

// ════════════════════════════════════════════════════════════
void loop() {
  handleBluetooth();
  handleButtons();
  handleCalibration();
  handleLED();

  if (following) {
    runPID();
  }
}

// ── Save calibration to NVS (ESP32 flash) ────────────────────
void saveCalibToNVS() {
  prefs.begin("calib", false);
  for (int i = 0; i < 5; i++) {
    prefs.putInt(("mn" + String(i)).c_str(), sensorMin[i]);
    prefs.putInt(("mx" + String(i)).c_str(), sensorMax[i]);
  }
  prefs.putBool("done", true);
  prefs.end();
  Serial.println("Calibration saved to NVS.");
}

// ── Load calibration from NVS ─────────────────────────────────
void loadCalibFromNVS() {
  prefs.begin("calib", true);   // read-only
  bool saved = prefs.getBool("done", false);
  if (saved) {
    for (int i = 0; i < 5; i++) {
      sensorMin[i] = prefs.getInt(("mn" + String(i)).c_str(), 0);
      sensorMax[i] = prefs.getInt(("mx" + String(i)).c_str(), 4095);
    }
    calibDone = true;
  } else {
    for (int i = 0; i < 5; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
    calibDone = false;
  }
  prefs.end();
}


// ── Save PID + speed to NVS ───────────────────────────────────
void savePIDToNVS() {
  prefs.begin("pid", false);
  prefs.putFloat("kp",    Kp);
  prefs.putFloat("ki",    Ki);
  prefs.putFloat("kd",    Kd);
  prefs.putInt  ("spd",   baseSpeed);
  prefs.end();
  SerialBT.println("PID+Speed saved to memory.");
  Serial.println("PID+Speed saved to NVS.");
}

// ── Load PID + speed from NVS ─────────────────────────────────
void loadPIDFromNVS() {
  prefs.begin("pid", true);
  Kp        = prefs.getFloat("kp",  0.08);
  Ki        = prefs.getFloat("ki",  0.0);
  Kd        = prefs.getFloat("kd",  0.6);
  baseSpeed = prefs.getInt  ("spd", 51);
  prefs.end();
  Serial.println("PID loaded: Kp=" + String(Kp,4) +
                 " Ki=" + String(Ki,4) +
                 " Kd=" + String(Kd,4) +
                 " Speed=" + String((baseSpeed*100)/255) + "%");
}

// ── Start calibration (shared by button + BT command) ─────────
void startCalibration() {
  following     = false;
  stopMotors();
  calibDone     = false;
  calibRunning  = true;
  calibStart    = millis();
  previousError = 0;
  I_term        = 0;
  for (int i = 0; i < 5; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
  SerialBT.println("=== CALIBRATION STARTED (10 sec) ===");
  SerialBT.println("Sweep sensor over BLACK line AND WHITE surface!");
  Serial.println("Calibration started.");
}

// ── Start following (shared by button + BT command) ───────────
void startFollowing() {
  if (calibRunning) { SerialBT.println("Calibration running, wait..."); return; }
  if (!calibDone)   { SerialBT.println("Not calibrated! Press G18 or send 'c'."); return; }
  previousError = 0;
  I_term        = 0;
  lineLost      = false;
  lineLostSince = 0;
  following     = true;
  digitalWrite(LED_PIN, HIGH);   // solid LED = following
  SerialBT.println("Line following STARTED at " + String((baseSpeed * 100) / 255) + "%");
  Serial.println("Following started.");
}

// ── Physical button handler ───────────────────────────────────
void handleButtons() {
  unsigned long now = millis();

  // G18 → Calibration
  bool btn18 = digitalRead(BTN_CAL);
  if (btn18 != lastBtn18 && (now - lastBtn18Time) > DEBOUNCE) {
    lastBtn18Time = now;
    lastBtn18     = btn18;
    if (btn18 == LOW) {   // pressed
      startCalibration();
    }
  }

  // G19 → Start following
  bool btn19 = digitalRead(BTN_RUN);
  if (btn19 != lastBtn19 && (now - lastBtn19Time) > DEBOUNCE) {
    lastBtn19Time = now;
    lastBtn19     = btn19;
    if (btn19 == LOW) {   // pressed
      if (following) {
        // press again while following → stop
        following = false;
        stopMotors();
        digitalWrite(LED_PIN, LOW);
        SerialBT.println("Stopped by button.");
      } else {
        startFollowing();
      }
    }
  }
}

// ── LED indicator ─────────────────────────────────────────────
void handleLED() {
  if (calibRunning) {
    // Blink once per second during calibration
    unsigned long elapsed = millis() - calibStart;
    // LED HIGH for first 100ms of each second, LOW for rest
    digitalWrite(LED_PIN, (elapsed % 1000) < 100 ? HIGH : LOW);
    return;
  }
  if (following) {
    digitalWrite(LED_PIN, HIGH);   // solid = following
    return;
  }
  digitalWrite(LED_PIN, LOW);      // off = idle
}

// ── Non-blocking calibration tick ─────────────────────────────
void handleCalibration() {
  if (!calibRunning) return;

  unsigned long elapsed = millis() - calibStart;

  for (int i = 0; i < 5; i++) {
    int val = analogRead(IR_PIN[i]);
    if (val < sensorMin[i]) sensorMin[i] = val;
    if (val > sensorMax[i]) sensorMax[i] = val;
  }

  static unsigned long lastTick = 0;
  if (millis() - lastTick >= 1000) {
    lastTick = millis();
    int secsLeft = (int)((CALIB_DURATION - elapsed) / 1000) + 1;
    SerialBT.println("Calibrating... " + String(secsLeft) + "s left");
    Serial.println("Calibrating... " + String(secsLeft) + "s left");
  }

  if (elapsed >= CALIB_DURATION) {
    calibRunning = false;
    calibDone    = true;

    // Save to NVS so it survives power-off
    saveCalibToNVS();

    SerialBT.println("=== CALIBRATION DONE ===");
    for (int i = 0; i < 5; i++) {
      String s = "IR" + String(i+1) + " Min:" + String(sensorMin[i]) + " Max:" + String(sensorMax[i]);
      SerialBT.println(s);
      Serial.println(s);
    }
    SerialBT.println("Saved to memory. Press G19 or send 's' to start.");
    digitalWrite(LED_PIN, LOW);
  }
}

// ── Read raw sensors → normalized (1.0=black, 0.0=white) ─────
void readSensors() {
  for (int i = 0; i < 5; i++) {
    int raw   = analogRead(IR_PIN[i]);
    int range = sensorMax[i] - sensorMin[i];
    if (range == 0) {
      sensorNorm[i] = 0.0;
    } else {
      float n = (float)(raw - sensorMin[i]) / range;
      sensorNorm[i] = constrain(1.0 - n, 0.0, 1.0);
    }
  }
}

// ── Compute line position 0–4000 ──────────────────────────────
int readLinePosition() {
  readSensors();
  float weightedSum = 0, totalWeight = 0;
  for (int i = 0; i < 5; i++) {
    weightedSum += (float)(i * 1000) * sensorNorm[i];
    totalWeight += sensorNorm[i];
  }
  if (totalWeight < 0.1) return -1;
  return (int)(weightedSum / totalWeight);
}

// ── PID + auto-stop if line lost for 1 sec ────────────────────
void runPID() {
  int position = readLinePosition();

  // ── Line lost handling ───────────────────────────────────────
  if (position == -1) {
    if (!lineLost) {
      lineLost      = true;
      lineLostSince = millis();
    }

    // Auto-stop after 1 second of lost line
    if (millis() - lineLostSince >= LINE_LOST_LIMIT) {
      following = false;
      stopMotors();
      digitalWrite(LED_PIN, LOW);
      SerialBT.println("Line lost for 1 sec — AUTO STOPPED.");
      Serial.println("Auto-stopped: line lost.");
      lineLost = false;
      return;
    }

    // While still within recovery window — spin toward last known side
    int spinSpeed = max(baseSpeed / 2, 60);
    if (previousError > 0) {
      motorDrive(-spinSpeed, spinSpeed);
    } else {
      motorDrive(spinSpeed, -spinSpeed);
    }
    return;
  }

  // Line found — reset lost timer
  lineLost = false;

  // ── PID calculation ─────────────────────────────────────────
  int error    = 2000 - position;
  int absError = abs(error);

  int P = error;
  I_term = constrain(I_term + error, -500, 500);
  int D = error - previousError;

  // Non-linear Kp: scales up on sharp turns
  float errorFactor = 1.0 + (2.0 * absError / 2000.0);
  float PIDvalue = (Kp * errorFactor * P) + (Ki * I_term) + (Kd * D);

  previousError = error;

  // Dynamic speed reduction on sharp turns
  int dynamicBase = baseSpeed;
  if (absError > 800) {
    float speedScale = 1.0 - (0.5 * (absError - 800) / 1200.0);
    dynamicBase = (int)(baseSpeed * speedScale);
    dynamicBase = max(dynamicBase, 50);
  }

  int leftSpeed  = constrain((int)(dynamicBase - PIDvalue), -255, 255);
  int rightSpeed = constrain((int)(dynamicBase + PIDvalue), -255, 255);

  if (leftSpeed  > 0  && leftSpeed  < 50)  leftSpeed  = 50;
  if (rightSpeed > 0  && rightSpeed < 50)  rightSpeed = 50;
  if (leftSpeed  < 0  && leftSpeed  > -50) leftSpeed  = -50;
  if (rightSpeed < 0  && rightSpeed > -50) rightSpeed = -50;

  motorDrive(leftSpeed, rightSpeed);
}

// ── DRV8833 motor drive ───────────────────────────────────────
void motorDrive(int left, int right) {
  if (left > 0)       { ledcWrite(IN1, left);  ledcWrite(IN2, 0);     }
  else if (left < 0)  { ledcWrite(IN1, 0);     ledcWrite(IN2, -left); }
  else                { ledcWrite(IN1, 0);     ledcWrite(IN2, 0);     }

  if (right > 0)      { ledcWrite(IN3, right); ledcWrite(IN4, 0);      }
  else if (right < 0) { ledcWrite(IN3, 0);     ledcWrite(IN4, -right); }
  else                { ledcWrite(IN3, 0);     ledcWrite(IN4, 0);      }
}

void stopMotors() {
  ledcWrite(IN1, 0); ledcWrite(IN2, 0);
  ledcWrite(IN3, 0); ledcWrite(IN4, 0);
}

// ── Bluetooth reader ──────────────────────────────────────────
void handleBluetooth() {
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      btBuffer.trim();
      if (btBuffer.length() > 0) parseCommand(btBuffer);
      btBuffer = "";
    } else {
      btBuffer += c;
    }
  }
}

// ── Command parser ────────────────────────────────────────────
void parseCommand(String cmd) {
  Serial.print("CMD: "); Serial.println(cmd);

  if (cmd == "c" || cmd == "C") { startCalibration(); return; }

  if (cmd == "s" || cmd == "S") { startFollowing(); return; }

  if (cmd == "x" || cmd == "X") {
    following = false;
    stopMotors();
    digitalWrite(LED_PIN, LOW);
    SerialBT.println("Stopped.");
    return;
  }

  if (cmd == "?" || cmd == "status") {
    SerialBT.println("──── Status ────");
    SerialBT.println("Kp=" + String(Kp,4) + " Ki=" + String(Ki,4) + " Kd=" + String(Kd,4));
    SerialBT.println("Speed=" + String((baseSpeed*100)/255) + "%");
    SerialBT.println("Calibrated=" + String(calibDone  ? "YES" : "NO"));
    SerialBT.println("Following="  + String(following   ? "YES" : "NO"));
    SerialBT.println("────────────────");
    return;
  }

  if (cmd.startsWith("P") || cmd.startsWith("p")) {
    Kp = cmd.substring(1).toFloat();
    savePIDToNVS();
    SerialBT.println("Kp = " + String(Kp, 4)); return;
  }
  if (cmd.startsWith("I") || cmd.startsWith("i")) {
    Ki = cmd.substring(1).toFloat(); I_term = 0;
    savePIDToNVS();
    SerialBT.println("Ki = " + String(Ki, 4) + " (I reset)"); return;
  }
  if (cmd.startsWith("D") || cmd.startsWith("d")) {
    Kd = cmd.substring(1).toFloat();
    savePIDToNVS();
    SerialBT.println("Kd = " + String(Kd, 4)); return;
  }

  if (cmd.startsWith("UP") || cmd.startsWith("up") || cmd.startsWith("Up")) {
    int pct = constrain(cmd.substring(2).toInt(), 0, 100);
    baseSpeed = (int)((pct / 100.0) * 255);
    savePIDToNVS();
    SerialBT.println("Speed -> " + String(pct) + "% (" + String(baseSpeed) + "/255)"); return;
  }
  if (cmd.startsWith("DOWN") || cmd.startsWith("down") || cmd.startsWith("Down")) {
    int pct = constrain(cmd.substring(4).toInt(), 0, 100);
    baseSpeed = (int)((pct / 100.0) * 255);
    savePIDToNVS();
    SerialBT.println("Speed -> " + String(pct) + "% (" + String(baseSpeed) + "/255)"); return;
  }

  // Reset PID+speed to defaults and clear NVS
  if (cmd == "reset" || cmd == "RESET") {
    Kp = 0.08; Ki = 0.0; Kd = 0.6; baseSpeed = 51;
    savePIDToNVS();
    SerialBT.println("PID+Speed reset to defaults and saved.");
    return;
  }

  SerialBT.println("Unknown: " + cmd);
  SerialBT.println("Cmds: c  s  x  ?  Px  Ix  Dx  UPx  DOWNx  reset");
}

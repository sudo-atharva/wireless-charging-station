/*
  V-Shaped Conveyor Belt - Auto-Tilt Controller
  --------------------------------------------------------------------
  - Drive motor is a stepper driven by an A4988 (STEP/DIR/ENABLE),
    run continuously at a fixed speed via the ACS712 on its supply
    line (drive motor current sensed - ACS712 analog Hall sensor)
  - When current is high (heavy load), both V-side actuator motors
    (via L298N) raise the sides to contain material; lower again once
    current drops back to normal.
  - No position sensor: actuator position is tracked by motor run time
    (open loop). On every boot the sides are in setup mode, 3D-printer
    style: jog LEFT/RIGHT independently from the web page, press
    "Set Start" at the low point, jog up, press "Set End" at the high
    point. The travel time of each side is recorded and angles are
    mapped linearly between the Start/End angles entered in Settings.
  - Modes: AUTO (current limit picks base/raised angle) or MANUAL
    (web slider picks the angle).
  - MPU6050 code is kept below but unused (no tilt feedback for now).
  - ESP32 runs its own WiFi Access Point + web dashboard (current,
    angle, raise/lower state, live current graph, a manual up/down
    slider, and a settings form for default/raised angle + current
    limit - saved to flash via LittleFS so they survive reboot). The
    dashboard page itself is data/index.html, served straight from
    LittleFS - upload it once with "pio run -t uploadfs" (separate
    from the normal firmware upload).
  - 16x2 I2C LCD mirrors the same readings locally.

  Wiring (change pins to match your board):
    A4988 STEP        -> 32
    A4988 DIR         -> 33
    A4988 ENABLE      -> 15   (active LOW - LOW enables driver)
    A4988 VMOT/GND    -> stepper motor supply, VDD/GND -> 3.3V/GND
    ACS712 OUT        -> 34   (ADC1, on stepper's motor supply line)
    ACS712 VCC/GND    -> 5V/GND
    I2C SDA           -> 21   (LCD + MPU6050)
    I2C SCL           -> 22
    MPU6050           -> AD0 to GND -> address 0x68, VCC -> 3.3V, GND -> GND
    L298N (single board) Channel A (LEFT):  ENA -> 25, IN1 -> 26, IN2 -> 27
    L298N (single board) Channel B (RIGHT): ENB -> 14, IN3 -> 12, IN4 -> 13
    L298N 12V -> actuator motor supply, 5V/GND -> ESP32 5V/GND (shared ground)
    L298N ENA/ENB wired to GPIO but held HIGH in firmware (see setup) -
      not switched per direction, so they could also be hardwired to 5V.
    LCD VCC -> 5V, GND -> GND, SDA/SCL as above

  IMPORTANT - tune before running on real hardware:
    - ACS_SENSITIVITY_V_PER_A / ACS_ZERO_OFFSET_V for your ACS712 variant
    - Default/raised angle and current limit can be tuned live from the
      web dashboard (Settings card) - stored in /config.txt on LittleFS.
      The constants below are only the fallback defaults.
    - Set Start/End angles in Settings to the real angles of the two
      calibration points, or the displayed angle is meaningless.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LittleFS.h>
#include <Arduino.h>
#include <math.h>

// ---------------- USER CONFIG ----------------

const char* AP_SSID = "ConveyorBelt";
const char* AP_PASS = "12345678"; // 8+ chars, or "" for open network

// A4988 stepper driver (conveyor drive motor)
const int STEP_PIN = 32;
const int DIR_PIN = 33;
const int ENABLE_PIN = 15; // active LOW
const unsigned long STEP_PULSE_INTERVAL_US = 800; // half-period -> tune for belt speed

// Current sensor
const int CURRENT_SENSOR_PIN = 34;
const float ADC_MAX  = 4095.0;  // ESP32 12-bit ADC
const float ADC_VREF = 3.3;
const float ACS_SENSITIVITY_V_PER_A = 0.100; // ACS712-20A. Change for your module.
const float ACS_ZERO_OFFSET_V = ADC_VREF / 2.0;

// MPU6050 I2C address (AD0 to GND) - unused for now, kept for later
const uint8_t MPU_ADDR = 0x68;

// L298N (single board) - Channel A: LEFT actuator, Channel B: RIGHT actuator
// EN pins held HIGH always (see setup) - only IN1-IN4 switch direction.
const int ENA_PIN = 25;
const int IN1_PIN = 26;
const int IN2_PIN = 27;
const int ENB_PIN = 14;
const int IN3_PIN = 12;
const int IN4_PIN = 13;

// LCD
const uint8_t LCD_I2C_ADDR = 0x27;
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

const float POS_TOLERANCE_MS = 40;         // bang-bang deadband, in ms of actuator travel
const unsigned long JOG_TIMEOUT_MS = 500;  // jog stops if web page stops sending (lost pointerup / WiFi drop)

const unsigned long SAMPLE_INTERVAL_MS = 100;
const unsigned long LCD_UPDATE_MS      = 500;

// ------------------------------------------------

// Tunable via web Settings card, persisted to LittleFS (/config.txt).
// Values below are only the fallback defaults for a fresh flash.
struct Config {
  float startAngle  = 0.0;   // real angle at the "Set Start" point (deg)
  float endAngle    = 45.0;  // real angle at the "Set End" point (deg)
  float baseAngle   = 0.0;   // resting tilt (deg)
  float raisedAngle = 30.0;  // tilt when overloaded (deg)
  float currentLimit = 3.0;  // amps - above this, raise sides
};
Config cfg;
const char* CONFIG_PATH = "/config.txt";

void loadConfig() {
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) return; // keep defaults
  while (f.available()) {
    String line = f.readStringUntil('\n');
    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String key = line.substring(0, eq);
    float val = line.substring(eq + 1).toFloat();
    if (key == "start") cfg.startAngle = val;
    else if (key == "end") cfg.endAngle = val;
    else if (key == "base") cfg.baseAngle = val;
    else if (key == "raised") cfg.raisedAngle = val;
    else if (key == "cur") cfg.currentLimit = val;
  }
  f.close();
}

void saveConfig() {
  File f = LittleFS.open(CONFIG_PATH, "w");
  if (!f) return;
  f.printf("start=%.2f\nend=%.2f\nbase=%.2f\nraised=%.2f\ncur=%.2f\n",
    cfg.startAngle, cfg.endAngle, cfg.baseAngle, cfg.raisedAngle, cfg.currentLimit);
  f.close();
}

WebServer server(80);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

float driveCurrent = 0, angle = 0, targetAngle = 0;
bool raised = false;
bool manualMode = true; // start in manual after setup so nothing moves unexpectedly
float manualAngle = 0; // target angle when manualMode is on, set by web slider

// Actuators: index 0 = LEFT (IN1/IN2), 1 = RIGHT (IN3/IN4)
const int MOTOR_IN[2][2] = {{IN1_PIN, IN2_PIN}, {IN3_PIN, IN4_PIN}};
int stage = 0;               // 0 = need Set Start, 1 = need Set End, 2 = calibrated
float posMs[2] = {0, 0};     // ms of "up" travel from start point
float endMs[2] = {0, 0};     // travel time start->end, recorded at Set End
int motorDir[2] = {0, 0};    // +1 up, -1 down, 0 stop
int jogDir[2] = {0, 0};
unsigned long lastJogMs[2] = {0, 0};
unsigned long lastMoveMs = 0;
unsigned long lastSample = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastStepToggle = 0;
bool stepPinHigh = false;

// ---------- A4988 stepper (drive motor) ----------
// ponytail: fixed-speed toggle loop, swap for AccelStepper if accel/decel needed.
void runStepper() {
  unsigned long nowUs = micros();
  if (nowUs - lastStepToggle >= STEP_PULSE_INTERVAL_US) {
    lastStepToggle = nowUs;
    stepPinHigh = !stepPinHigh;
    digitalWrite(STEP_PIN, stepPinHigh);
  }
}

// ---------- MPU6050 (raw register access, no extra lib) ----------
// Unused right now - position is time-tracked. Kept for future closed loop.

void mpuWake(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // wake up
  Wire.endTransmission();
}

// Tilt angle from accelerometer only (no gyro fusion needed for slow tilt).
// ponytail: accel-only angle, add complementary/DMP filter if vibration noise matters.
float mpuAngle(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((int)addr, 6, true);
  if (Wire.available() < 6) return 0;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  (void)ax;
  return atan2((float)ay, (float)az) * 180.0 / PI;
}

// ---------- Current sensor ----------

float readCurrent(int pin) {
  int raw = analogRead(pin);
  float vAdc = (raw / ADC_MAX) * ADC_VREF;
  return (vAdc - ACS_ZERO_OFFSET_V) / ACS_SENSITIVITY_V_PER_A;
}

// ---------- Actuator control (time-tracked position, bang-bang) ----------

float angleSpan() {
  return cfg.endAngle - cfg.startAngle;
}

float angleToMs(int side, float a) {
  float span = angleSpan();
  float frac = span == 0 ? 0 : constrain((a - cfg.startAngle) / span, 0.0f, 1.0f);
  return frac * endMs[side];
}

float estimatedAngle() {
  float frac = (posMs[0] / endMs[0] + posMs[1] / endMs[1]) / 2;
  return cfg.startAngle + frac * angleSpan();
}

// ponytail: open-loop timing assumes equal up/down speed; drift builds over
// many moves - re-run setup, or add a limit switch / MPU feedback if it matters.
void updateActuators() {
  unsigned long now = millis();
  unsigned long dt = now - lastMoveMs;
  lastMoveMs = now;

  for (int i = 0; i < 2; i++) {
    posMs[i] += motorDir[i] * (float)dt; // account for travel since last call

    int dir;
    if (stage < 2) {
      dir = (now - lastJogMs[i] < JOG_TIMEOUT_MS) ? jogDir[i] : 0;
    } else {
      float t = angleToMs(i, targetAngle);
      dir = posMs[i] < t - POS_TOLERANCE_MS ? 1 : posMs[i] > t + POS_TOLERANCE_MS ? -1 : 0;
    }

    motorDir[i] = dir;
    digitalWrite(MOTOR_IN[i][0], dir > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_IN[i][1], dir < 0 ? HIGH : LOW); // both LOW = brake
  }
}

// ---------- LCD ----------

void updateLcd() {
  char line0[17], line1[17];
  if (stage < 2) {
    snprintf(line0, sizeof(line0), "SETUP: set %s", stage == 0 ? "START" : "END");
    snprintf(line1, sizeof(line1), "%s", WiFi.softAPIP().toString().c_str());
  } else {
    snprintf(line0, sizeof(line0), "I:%5.2fA %s", driveCurrent, manualMode ? "MAN" : "AUTO");
    snprintf(line1, sizeof(line1), "Ang:%5.1f %s", angle, raised ? "RAISED" : "");
  }
  lcd.setCursor(0, 0);
  lcd.print(line0);
  for (int i = strlen(line0); i < LCD_COLS; i++) lcd.print(' ');
  lcd.setCursor(0, 1);
  lcd.print(line1);
  for (int i = strlen(line1); i < LCD_COLS; i++) lcd.print(' ');
}

// ---------- Web dashboard ----------

void handleData() {
  String json = "{";
  json += "\"current\":" + String(driveCurrent, 3) + ",";
  json += "\"angle\":" + String(angle, 2) + ",";
  json += "\"raised\":" + String(raised ? "true" : "false") + ",";
  json += "\"manual\":" + String(manualMode ? "true" : "false") + ",";
  json += "\"manualAngle\":" + String(manualAngle, 2) + ",";
  json += "\"stage\":" + String(stage) + ",";
  json += "\"posL\":" + String(posMs[0] / 1000, 2) + ",";
  json += "\"posR\":" + String(posMs[1] / 1000, 2);
  json += "}";
  server.send(200, "application/json", json);
}

void handleGetConfig() {
  String json = "{";
  json += "\"start\":" + String(cfg.startAngle, 2) + ",";
  json += "\"end\":" + String(cfg.endAngle, 2) + ",";
  json += "\"base\":" + String(cfg.baseAngle, 2) + ",";
  json += "\"raised\":" + String(cfg.raisedAngle, 2) + ",";
  json += "\"cur\":" + String(cfg.currentLimit, 2);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetConfig() {
  if (server.hasArg("start")) cfg.startAngle = server.arg("start").toFloat();
  if (server.hasArg("end")) cfg.endAngle = server.arg("end").toFloat();
  if (server.hasArg("base")) cfg.baseAngle = server.arg("base").toFloat();
  if (server.hasArg("raised")) cfg.raisedAngle = server.arg("raised").toFloat();
  if (server.hasArg("cur")) cfg.currentLimit = server.arg("cur").toFloat();
  saveConfig();
  server.send(200, "text/plain", "ok");
}

void handleManual() {
  if (server.hasArg("enabled")) manualMode = server.arg("enabled").toInt() != 0;
  if (server.hasArg("angle")) manualAngle = server.arg("angle").toFloat();
  server.send(200, "text/plain", "ok");
}

void handleJog() {
  int side = server.arg("side").toInt();
  if (stage >= 2 || side < 0 || side > 1) { server.send(400, "text/plain", "not in setup"); return; }
  jogDir[side] = constrain(server.arg("dir").toInt(), -1, 1);
  lastJogMs[side] = millis();
  server.send(200, "text/plain", "ok");
}

void handleSetStart() {
  if (stage >= 2) { server.send(400, "text/plain", "not in setup"); return; }
  posMs[0] = posMs[1] = 0;
  stage = 1;
  server.send(200, "text/plain", "ok");
}

void handleSetEnd() {
  if (stage != 1 || posMs[0] <= 0 || posMs[1] <= 0) {
    server.send(400, "text/plain", "set start first, then jog both sides UP");
    return;
  }
  endMs[0] = posMs[0];
  endMs[1] = posMs[1];
  manualMode = true;
  manualAngle = cfg.endAngle; // hold where it is
  stage = 2;
  server.send(200, "text/plain", "ok");
}

void handleRecal() {
  stage = 0;
  jogDir[0] = jogDir[1] = 0;
  server.send(200, "text/plain", "ok");
}

// ---------- Setup / loop ----------

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(ENABLE_PIN, LOW); // enable A4988 (active LOW)

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH); // L298N channels always enabled, speed fixed by IN1/IN2 logic only
  digitalWrite(ENB_PIN, HIGH);

  Wire.begin(); // SDA=21, SCL=22 (LCD)

  LittleFS.begin(true); // format on first boot if needed
  loadConfig();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Conveyor Belt");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP started: ");
  Serial.println(AP_SSID);
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());

  server.serveStatic("/", LittleFS, "/index.html");
  server.on("/data", handleData);
  server.on("/config", HTTP_GET, handleGetConfig);
  server.on("/config", HTTP_POST, handleSetConfig);
  server.on("/manual", HTTP_POST, handleManual);
  server.on("/jog", HTTP_POST, handleJog);
  server.on("/setstart", HTTP_POST, handleSetStart);
  server.on("/setend", HTTP_POST, handleSetEnd);
  server.on("/recal", HTTP_POST, handleRecal);
  server.begin();

  delay(1000);
  lcd.clear();
  lastMoveMs = millis();
}

void loop() {
  server.handleClient();
  runStepper();
  updateActuators();

  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    driveCurrent = readCurrent(CURRENT_SENSOR_PIN);

    if (manualMode) {
      targetAngle = manualAngle;
      raised = false;
    } else {
      raised = driveCurrent > cfg.currentLimit;
      targetAngle = raised ? cfg.raisedAngle : cfg.baseAngle;
    }
    angle = stage == 2 ? estimatedAngle() : 0;

    Serial.printf("I:%5.2fA stage:%d L:%.2fs R:%.2fs A:%6.1f target:%6.1f %s\n",
      driveCurrent, stage, posMs[0] / 1000, posMs[1] / 1000, angle, targetAngle,
      manualMode ? "MANUAL" : raised ? "AUTO RAISED" : "AUTO BASE");
  }

  if (now - lastLcdUpdate >= LCD_UPDATE_MS) {
    lastLcdUpdate = now;
    updateLcd();
  }
}

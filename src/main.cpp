/*
  Wireless Charging Station - Battery/Charger Monitor
  --------------------------------------------------------------------
  - Creates its own WiFi Access Point (no router needed)
  - Serves a live web dashboard (data/index.html via LittleFS)
  - Reads CHARGER voltage+current and BATTERY voltage+current
  - Reads pack temperature (DS18B20)
  - Shows the same readings on a 16x2 I2C LCD

  Hardware assumptions (CHANGE THESE TO MATCH YOUR ACTUAL SENSORS):
  - Current sensors: ACS712-type analog Hall sensor (output centered
    at ~Vcc/2, linear mV/A). Swap the math in readCurrent() if you're
    using something else (INA219, shunt+amp module, ACS758, etc.)
  - Voltage sensors: simple resistive divider into an ADC pin
  - Temp sensor: DS18B20 (1-Wire), needs a 4.7k pull-up between DATA and 3V3
  - LCD: generic 16x2 character LCD on a PCF8574 I2C backpack
    (I2C address is usually 0x27 or 0x3F - check yours if blank)

  Wiring (change pins to whatever's free on your board):
    CHARGER_VOLT_PIN -> 34   (ADC1, input-only pin, safe choice on ESP32)
    CHARGER_CURR_PIN -> 35
    BATT_VOLT_PIN    -> 32
    BATT_CURR_PIN    -> 33
    DS18B20_PIN      -> 26   (1-Wire data, with 4.7k pull-up to 3V3)
    LCD SDA          -> 21   (default ESP32 I2C SDA)
    LCD SCL          -> 22   (default ESP32 I2C SCL)
    LCD VCC          -> 5V
    LCD GND          -> GND

  Arduino IDE setup:
    Board: "ESP32 Dev Module" (or whatever your board is)
    Install "ESP32" board package via Boards Manager if not already done
    Install libraries: "LiquidCrystal I2C" by Frank de Brabander,
      "OneWire" by Paul Stoffregen, "DallasTemperature" by Miles Burton
*/

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
// ---------------- USER CONFIG ----------------

// WiFi Access Point credentials
const char* AP_SSID = "WirelessChargingStation";
const char* AP_PASS = "12345678";   // must be 8+ chars, or "" for open network

// Sensor pins
const int CHARGER_VOLT_PIN = 34;
const int CHARGER_CURR_PIN = 35;
const int BATT_VOLT_PIN    = 32;
const int BATT_CURR_PIN    = 33;
const int DS18B20_PIN      = 26;

// LCD config: address, columns, rows
// Common addresses: 0x27 or 0x3F. Run an I2C scanner sketch if unsure.
const uint8_t LCD_I2C_ADDR = 0x27;
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

// ---- Calibration ----
const float ADC_MAX   = 4095.0;   // ESP32 ADC is 12-bit
const float ADC_VREF  = 3.3;      // ESP32 ADC reference voltage

// Voltage divider: Vin = Vadc * (R1+R2)/R2
// Example: R1 = 30k (top, to the high voltage), R2 = 7.5k (bottom, to GND)
// -> ratio = 5.0 means a 15V input reads as 3.0V at the ADC pin
const float CHARGER_VOLT_DIVIDER_RATIO = 5.0;
const float BATT_VOLT_DIVIDER_RATIO    = 5.0;

// ACS712 current sensor
// - ACS712-05B: 185 mV/A,  zero-current output = Vcc/2
// - ACS712-20A: 100 mV/A
// - ACS712-30A: 66  mV/A
const float ACS_SENSITIVITY_V_PER_A = 0.100;  // set to match your module (20A version here)
const float ACS_ZERO_OFFSET_V       = ADC_VREF / 2.0; // volts at 0A, adjust after testing

const unsigned long SAMPLE_INTERVAL_MS = 200;
const unsigned long LCD_UPDATE_MS      = 500;
const unsigned long LCD_PAGE_MS        = 2000;  // how long each LCD page is shown before cycling
const unsigned long TEMP_INTERVAL_MS   = 1000; // DS18B20 conversion is slow, sample it less often

// Battery pack voltage range - 0% / 100% points, tune to your pack chemistry & cell count.
// Example here: 3S lead-acid/Li-ion-ish 12V pack.
const float BATT_MIN_V = 10.5;
const float BATT_MAX_V = 12.6;
const float CHARGE_CURRENT_THRESHOLD_A = 0.05; // above this, count as "charging"

// ------------------------------------------------

WebServer server(80);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

float chargerVoltage = 0, chargerCurrent = 0, battVoltage = 0, battCurrent = 0, battTempC = 0;
unsigned long lastSample = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastLcdPageChange = 0;
uint8_t lcdPage = 0; // 0 = charger/battery, 1 = battery %/temp
unsigned long lastTempRequest = 0;
bool tempConversionPending = false;

bool lcdPresent = false;

// ---------- Sensor reading helpers ----------

float readVoltage(int pin, float dividerRatio) {
  int raw = analogRead(pin);
  float vAdc = (raw / ADC_MAX) * ADC_VREF;
  return vAdc * dividerRatio;
}

float readCurrent(int pin) {
  int raw = analogRead(pin);
  float vAdc = (raw / ADC_MAX) * ADC_VREF;
  float amps = (vAdc - ACS_ZERO_OFFSET_V) / ACS_SENSITIVITY_V_PER_A;
  return amps;
}

float batteryPercent(float voltage) {
  float pct = (voltage - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V) * 100.0;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

// ---------- LCD ----------

void updateLcd() {
  if (!lcdPresent) return;
  char line0[17];
  char line1[17];
  if (lcdPage == 0) {
    snprintf(line0, sizeof(line0), "C %5.2fV %4.2fA", chargerVoltage, chargerCurrent);
    snprintf(line1, sizeof(line1), "B %5.2fV %4.2fA", battVoltage, battCurrent);
  } else {
    snprintf(line0, sizeof(line0), "Batt %5.1f %%", batteryPercent(battVoltage));
    snprintf(line1, sizeof(line1), "Temp %5.1f C", battTempC);
  }

  lcd.setCursor(0, 0);
  lcd.print(line0);
  for (int i = strlen(line0); i < LCD_COLS; i++) lcd.print(' ');
  lcd.setCursor(0, 1);
  lcd.print(line1);
  for (int i = strlen(line1); i < LCD_COLS; i++) lcd.print(' ');
}

// ---------- Web page (served from LittleFS: data/index.html) ----------

void handleData() {
  float pct = batteryPercent(battVoltage);
  bool charging = battCurrent > CHARGE_CURRENT_THRESHOLD_A;

  String json = "{";
  json += "\"chargerV\":" + String(chargerVoltage, 3) + ",";
  json += "\"chargerI\":" + String(chargerCurrent, 3) + ",";
  json += "\"battV\":" + String(battVoltage, 3) + ",";
  json += "\"battI\":" + String(battCurrent, 3) + ",";
  json += "\"battPct\":" + String(pct, 1) + ",";
  json += "\"battTempC\":" + String(battTempC, 1) + ",";
  json += "\"charging\":" + String(charging ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// ---------- Setup / loop ----------

void setup() {
  Serial.begin(115200);

  analogReadResolution(12); // ESP32 default, explicit for clarity

  Wire.begin(); // default SDA=21, SCL=22 on most ESP32 boards
  tempSensor.begin();
  tempSensor.setWaitForConversion(false); // async reads, don't block loop()

  lcdPresent = true;
  if (lcdPresent) {
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Charging Station");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
  } else {
    Serial.println("LCD not found at LCD_I2C_ADDR - continuing without LCD.");
  }

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed.");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP started. Connect to WiFi: ");
  Serial.println(AP_SSID);
  Serial.print("Then open: http://");
  Serial.println(WiFi.softAPIP());

  server.serveStatic("/", LittleFS, "/index.html");
  server.on("/data", handleData);
  server.begin();

  delay(1000);
  if (lcdPresent) lcd.clear();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    chargerVoltage = readVoltage(CHARGER_VOLT_PIN, CHARGER_VOLT_DIVIDER_RATIO);
    chargerCurrent = readCurrent(CHARGER_CURR_PIN);
    battVoltage    = readVoltage(BATT_VOLT_PIN, BATT_VOLT_DIVIDER_RATIO);
    battCurrent    = readCurrent(BATT_CURR_PIN);

    Serial.printf(
      "CHG %5.2fV %5.2fA | BATT %5.2fV %5.2fA | temp %5.1fC | batt %5.1f%% | %s\n",
      chargerVoltage, chargerCurrent, battVoltage, battCurrent, battTempC,
      batteryPercent(battVoltage),
      battCurrent > CHARGE_CURRENT_THRESHOLD_A ? "chg" : "dis"
    );
  }

  // DS18B20 conversion takes ~750ms; kick it off, then read the result on the next round.
  if (now - lastTempRequest >= TEMP_INTERVAL_MS) {
    lastTempRequest = now;
    if (tempConversionPending) {
      float t = tempSensor.getTempCByIndex(0);
      if (t != DEVICE_DISCONNECTED_C) battTempC = t;
    }
    tempSensor.requestTemperatures();
    tempConversionPending = true;
  }

  if (now - lastLcdPageChange >= LCD_PAGE_MS) {
    lastLcdPageChange = now;
    lcdPage = 1 - lcdPage;
  }

  if (now - lastLcdUpdate >= LCD_UPDATE_MS) {
    lastLcdUpdate = now;
    updateLcd();
  }
}

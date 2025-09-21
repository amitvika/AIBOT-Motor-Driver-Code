#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <AS5600.h>                // Rob Tillaart’s AS5600 library

// -------------------- Pin Configuration --------------------
const int POWER_PIN     = 12;      // holds the driver 3.3 V rail high
const int MOTOR_FWD_PIN = 13;      // up
const int MOTOR_BWD_PIN = 14;      // down

// -------------------- Wiring Direction --------------------
bool reverseMotor = true;          // set true if your wiring inverts up/down

// -------------------- PWM Scaling --------------------
const int ACTIVE_PWM = 255;        // 8-bit LEDC

// -------------------- Incoming Commands --------------------
int upCmd   = 0;                   // 0–100%
int downCmd = 0;

// -------------------- AS5600 Encoder --------------------
AS5600 encoder;                    // uses default &Wire :contentReference[oaicite:1]{index=1}

// -------------------- Hold-position P-only in raw counts --------------------
bool     holdActive = false;
uint16_t holdTarget = 0;           // 0–4095 counts
const float Kp_counts = 0.2;       // PWM units per count — tune as needed
const int   DEAD_BAND  = 5;        // counts (~0.4°) dead-zone

// ------------------------------------------------------------------
// Parse “u=…;d=…;” semicolon-delimited lines
// ------------------------------------------------------------------
void parseKeyValueLine(char* line) {
  for (char* token = strtok(line, ";"); token; token = strtok(nullptr, ";")) {
    char* eq = strchr(token, '=');
    if (!eq) continue;
    *eq = '\0';
    int val = atoi(eq + 1);
    if (strcmp(token, "u") == 0)      upCmd   = val;
    else if (strcmp(token, "d") == 0) downCmd = val;
  }
  Serial.printf("Commands ⇒ up=%d  down=%d\n", upCmd, downCmd);
}

void onDataRecv(const uint8_t*, const uint8_t* data, int len) {
  if (len <= 0) return;
  char buf[128];
  int L = min(len, (int)sizeof(buf) - 1);
  memcpy(buf, data, L);
  buf[L] = '\0';
  parseKeyValueLine(buf);
}

// ------------------------------------------------------------------
// Map up/down % → PWM & direction
// ------------------------------------------------------------------
void setMotor(int upPct, int dnPct, bool rev) {
  int u = upPct, d = dnPct;
  if (rev) { int t = u; u = d; d = t; }  // manual swap
  int fwd = (u > 0 && d == 0) ? map(u,0,100,0,ACTIVE_PWM) : 0;
  int bwd = (d > 0 && u == 0) ? map(d,0,100,0,ACTIVE_PWM) : 0;
  ledcWrite(0, fwd);
  ledcWrite(1, bwd);
}

// ------------------------------------------------------------------
// PWM & pin setup
// ------------------------------------------------------------------
void setupMotors() {
  pinMode(POWER_PIN,     OUTPUT);  digitalWrite(POWER_PIN, HIGH);
  pinMode(MOTOR_FWD_PIN, OUTPUT);
  pinMode(MOTOR_BWD_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);  ledcAttachPin(MOTOR_FWD_PIN, 0);
  ledcSetup(1, 5000, 8);  ledcAttachPin(MOTOR_BWD_PIN, 1);
  ledcWrite(0,0);
  ledcWrite(1,0);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();               // I²C before sensor init
  setupMotors();

  // init AS5600
  if (!encoder.begin()) {     // returns false if no device at 0x36 :contentReference[oaicite:2]{index=2}
    Serial.println("ERROR: AS5600 not found!");
    while (1) delay(10);
  }

  // grab initial hold target
  holdTarget = encoder.readAngle();  // 0–4095 counts :contentReference[oaicite:3]{index=3}

  // ESP-NOW receive setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.printf("Ready. MAC=%s\n", WiFi.macAddress().c_str());
}

void loop() {
  uint16_t current = encoder.readAngle();  // 0–4095 counts :contentReference[oaicite:4]{index=4}

  // Open-loop while commanding motion
  if (upCmd > 0 || downCmd > 0) {
    holdActive = false;
    setMotor(upCmd, downCmd, reverseMotor);
    holdTarget = current;  // reset hold target
  }
  else {
    // Closed-loop “hold” in raw counts
    if (!holdActive) {
      holdActive = true;
      holdTarget  = current;
      Serial.printf("Holding at %u counts\n", holdTarget);
    }

    // Compute shortest-path error around the 0–4095 wrap
    int32_t error = int32_t(holdTarget) - int32_t(current);
    if (error > 2048)   error -= 4096;
    if (error < -2048)  error += 4096;

    if (abs(error) > DEAD_BAND) {
      float corr = Kp_counts * error;            // PWM units
      int pct = constrain(int(abs(corr)), 0, ACTIVE_PWM) * 100 / ACTIVE_PWM;
      if (corr > 0)  setMotor(pct, 0, reverseMotor);
      else           setMotor(0, pct, reverseMotor);
    } else {
      setMotor(0, 0, reverseMotor);
    }
  }

  delay(1);
}

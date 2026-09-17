// Stereo vergence scanner — NodeMCU firmware.
// Two VL53L0X ToF sensors on two SG90 servos. Both sensors verge on a target
// point at each bearing; raw readings are streamed over UDP as JSON.
// Geometry and fusion happen on the laptop — firmware is a dumb sensor pipe.

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

// ---------- User config ----------
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
IPAddress   LAPTOP_IP(192, 168, 1, 100);   // laptop running scanner.py
const uint16_t LAPTOP_PORT = 5005;

// Physical rig
const float BASELINE_M       = 0.10f;    // center-to-center between sensors
const float VERGE_RANGE_M    = 1.5f;     // target point distance for vergence
const float THETA_MIN_DEG    = -89.0f;   // sweep start (right-hand rule: 0 = forward, + = right)
const float THETA_MAX_DEG    = 89.0f;
const float THETA_STEP_DEG   = 1.0f;
const uint8_t SAMPLES_PER_PT = 3;        // median-filtered

// Servo zero offsets (fine-tune after calibration)
const int SERVO_L_OFFSET_DEG = 0;
const int SERVO_R_OFFSET_DEG = 0;

// Servo settle time after move (ms). Small moves settle faster than big ones.
const uint16_t SERVO_SETTLE_MS = 60;

// Pins (NodeMCU Dx labels on the right)
const uint8_t PIN_SCL       = 5;    // D1
const uint8_t PIN_SDA       = 4;    // D2
const uint8_t PIN_XSHUT_L   = 13;   // D7
const uint8_t PIN_XSHUT_R   = 16;   // D0 (safe at boot, no interrupts needed)
const uint8_t PIN_SERVO_L   = 14;   // D5
const uint8_t PIN_SERVO_R   = 12;   // D6

// I2C addresses after boot dance
const uint8_t ADDR_LOX_L = 0x30;
const uint8_t ADDR_LOX_R = 0x29;
// ---------------------------------

Adafruit_VL53L0X loxL;
Adafruit_VL53L0X loxR;
Servo servoL;
Servo servoR;
WiFiUDP udp;

uint32_t seq = 0;

static int medianOf3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

// Return measured mm, or -1 if out-of-range / status bad.
static int readOneMm(Adafruit_VL53L0X& lox) {
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  if (m.RangeStatus == 4) return -1;
  return (int)m.RangeMilliMeter;
}

static int medianReadMm(Adafruit_VL53L0X& lox) {
  int r[3];
  for (uint8_t i = 0; i < SAMPLES_PER_PT; i++) r[i] = readOneMm(lox);
  int valid[3]; uint8_t nv = 0;
  for (uint8_t i = 0; i < SAMPLES_PER_PT; i++) if (r[i] >= 0) valid[nv++] = r[i];
  if (nv == 0) return -1;
  if (nv == 1) return valid[0];
  if (nv == 2) return (valid[0] + valid[1]) / 2;
  return medianOf3(valid[0], valid[1], valid[2]);
}

// Compute servo commands (deg, 0..180) to verge both sensors on target at
// bearing theta (deg, 0 = forward, + = right) at distance VERGE_RANGE_M.
// Convention: servo.write(90) aims the sensor straight forward (+Y).
static void vergeAngles(float thetaDeg, int& sLout, int& sRout) {
  float thetaRad = thetaDeg * (float)M_PI / 180.0f;
  float xT = VERGE_RANGE_M * sinf(thetaRad);
  float yT = VERGE_RANGE_M * cosf(thetaRad);
  // L sensor at (-B/2, 0), R sensor at (+B/2, 0)
  float sL = atan2f(xT - (-BASELINE_M / 2.0f), yT) * 180.0f / (float)M_PI;
  float sR = atan2f(xT - ( BASELINE_M / 2.0f), yT) * 180.0f / (float)M_PI;
  int cmdL = (int)roundf(90.0f + sL) + SERVO_L_OFFSET_DEG;
  int cmdR = (int)roundf(90.0f + sR) + SERVO_R_OFFSET_DEG;
  if (cmdL < 0)   cmdL = 0;   if (cmdL > 180) cmdL = 180;
  if (cmdR < 0)   cmdR = 0;   if (cmdR > 180) cmdR = 180;
  sLout = cmdL;
  sRout = cmdR;
}

static void sendPacket(float theta, int sL, int sR, int dL, int dR) {
  char buf[160];
  int n = snprintf(buf, sizeof(buf),
    "{\"seq\":%lu,\"t\":%lu,\"theta\":%.2f,\"sL\":%d,\"sR\":%d,\"dL\":%d,\"dR\":%d}\n",
    (unsigned long)seq++, (unsigned long)millis(), theta, sL, sR, dL, dR);
  udp.beginPacket(LAPTOP_IP, LAPTOP_PORT);
  udp.write((const uint8_t*)buf, n);
  udp.endPacket();
  Serial.print(buf);
}

static bool initSensors() {
  pinMode(PIN_XSHUT_L, OUTPUT);
  pinMode(PIN_XSHUT_R, OUTPUT);
  digitalWrite(PIN_XSHUT_L, LOW);
  digitalWrite(PIN_XSHUT_R, LOW);
  delay(10);

  // Bring up L alone, remap to ADDR_LOX_L.
  digitalWrite(PIN_XSHUT_L, HIGH);
  delay(10);
  if (!loxL.begin(ADDR_LOX_L)) {
    Serial.println("Left VL53L0X init failed");
    return false;
  }

  // Bring up R at default 0x29.
  digitalWrite(PIN_XSHUT_R, HIGH);
  delay(10);
  if (!loxR.begin(ADDR_LOX_R)) {
    Serial.println("Right VL53L0X init failed");
    return false;
  }

  Serial.println("Both VL53L0X ready");
  return true;
}

static void connectWifi() {
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.printf("\nIP: %s  → streaming to %s:%u\n",
                WiFi.localIP().toString().c_str(),
                LAPTOP_IP.toString().c_str(), LAPTOP_PORT);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nStereo vergence scanner booting");

  Wire.begin(PIN_SDA, PIN_SCL);

  if (!initSensors()) { while (true) delay(1000); }

  servoL.attach(PIN_SERVO_L, 500, 2400);
  servoR.attach(PIN_SERVO_R, 500, 2400);
  servoL.write(90);
  servoR.write(90);
  delay(400);

  connectWifi();
  udp.begin(LAPTOP_PORT);
}

void loop() {
  for (float theta = THETA_MIN_DEG; theta <= THETA_MAX_DEG; theta += THETA_STEP_DEG) {
    int sL, sR;
    vergeAngles(theta, sL, sR);
    servoL.write(sL);
    servoR.write(sR);
    delay(SERVO_SETTLE_MS);

    int dL = medianReadMm(loxL);
    int dR = medianReadMm(loxR);
    sendPacket(theta, sL, sR, dL, dR);

    yield();
  }
  // one full sweep done; small pause, then sweep again
  delay(250);
}

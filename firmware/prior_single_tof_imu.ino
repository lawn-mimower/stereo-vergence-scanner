#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>

// Create instances for both sensors
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing sensors...");

  // Initialize I2C only once
  Wire.begin(D2, D1);  // D2 = SDA, D1 = SCL for ESP8266 NodeMCU

  // Initialize VL53L0X sensor
  if (!lox.begin()) {
    Serial.println("Failed to initialize VL53L0X sensor!");
    while (1); // Halt the program if sensor is not detected
  }
  Serial.println("VL53L0X initialized successfully!");

  // Initialize MPU6050 sensor
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Failed to initialize MPU6050 sensor!");
    while (1); // Halt if MPU6050 is not detected
  }
  Serial.println("MPU6050 initialized successfully!");
}

void loop() {
  // VL53L0X distance measurement
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {  // 4 means out of range
    Serial.print("VL53L0X Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("VL53L0X: Out of range");
  }

  // MPU6050 accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("MPU6050 Acceleration (mg): X=");
  Serial.print(ax);
  Serial.print(" Y=");
  Serial.print(ay);
  Serial.print(" Z=");
  Serial.println(az);

  Serial.print("MPU6050 Gyroscope (°/s): X=");
  Serial.print(gx);
  Serial.print(" Y=");
  Serial.print(gy);
  Serial.print(" Z=");
  Serial.println(gz);

  delay(1);  // Delay between readings for readability and yield control
}

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);  // stop if sensor not connected
  }

  Serial.println("MPU6050 connected successfully!");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Send only AX and AY values, formatted for easy reading in Python
  Serial.print("AcX=");
  Serial.print(ax);
  Serial.print("|AcY=");
  Serial.println(ay);

  delay(100);  // adjust for smoother / faster readings
}

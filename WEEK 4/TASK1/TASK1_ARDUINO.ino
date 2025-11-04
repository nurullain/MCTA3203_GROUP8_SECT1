#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
void setup() {
Serial.begin (9600) ;
Wire.begin () ;
mpu.initialize ();
if (!mpu.testConnection ()) {
Serial.println("MPU6050 connection failed!");
while (1);
}
Serial.println ("MPU6050 connected successfully!");
}
void loop() {
int16_t ax, ay, az, gx, gy, gz;
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
// Send only AX and AY values, separated by a comma
Serial.print (ax) ;
Serial.print (",");
Serial.println (ay);
delay (100) ; // Adjust speed (lower = smoother graph, but more data)
}

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo myServo;

const int GREEN_LED = 7;
const int RED_LED = 8;
const int SERVO_PIN = 9;

bool motionDetected = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  myServo.attach(SERVO_PIN);
  myServo.write(0); // initial position

  Serial.println("System Ready - Waiting for command from PC...");
}

void loop() {
  // --- Detect motion ---
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  static int16_t last_ax = ax, last_ay = ay, last_az = az;
  long diff = abs(ax - last_ax) + abs(ay - last_ay) + abs(az - last_az);
  last_ax = ax;
  last_ay = ay;
  last_az = az;

  if (diff > 2000) {  // Adjust threshold for your sensitivity
    motionDetected = true;
    Serial.println("MOTION DETECTED");
  } else {
    motionDetected = false;
  }

  // --- Listen for command from PyCharm ---
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == '1' && motionDetected) {   // Authorized + motion
      Serial.println("✅ Authorized & Motion detected!");
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      myServo.write(180);  // 🔁 Rotate fully to 180 degrees
      delay(5000);         // Hold for 5s
      myServo.write(0);    // Return to 0 degrees
      digitalWrite(GREEN_LED, LOW);
    }
    else if (cmd == '1' && !motionDetected) {
      Serial.println("⛔ No motion detected — Access blocked.");
      digitalWrite(RED_LED, HIGH);
      delay(2000);
      digitalWrite(RED_LED, LOW);
    }
    else if (cmd == '2') {  // Unauthorized card
      Serial.println("❌ Unauthorized card!");
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      myServo.write(0);
      delay(3000);
      digitalWrite(RED_LED, LOW);
    }
  }

  delay(100);
}

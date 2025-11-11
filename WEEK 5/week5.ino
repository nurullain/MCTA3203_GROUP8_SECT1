// === L298N Dual DC Motor Control Experiment (Forward & Reverse, 4s Each) ===
// Motor A -> ENA D3, IN1 D12, IN2 D13
// Motor B -> ENB D5, IN3 D10, IN4 D11

const int ENA = 3;     // PWM pin for Motor A
const int IN1 = 12;
const int IN2 = 13;
const int ENB = 5;     // PWM pin for Motor B
const int IN3 = 10;
const int IN4 = 11;

// PWM levels to test
int pwmValues[] = {0, 64, 128, 255};
int numSpeeds = 4;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
  Serial.println("=== L298N Dual DC Motor Experiment Starting ===");
}

void loop() {
  // === Forward Test ===
  Serial.println("\n=== Forward Direction ===");
  setDirection(true); // Forward

  for (int i = 0; i < numSpeeds; i++) {
    int pwm = pwmValues[i];
    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);

    Serial.print("PWM = ");
    Serial.println(pwm);

    delay(4000); // Hold each speed for 4 seconds
  }

  stopMotors();
  delay(2000); // Small pause before reversing

  // === Reverse Test ===
  Serial.println("\n=== Reverse Direction ===");
  setDirection(false); // Reverse

  for (int i = 0; i < numSpeeds; i++) {
    int pwm = pwmValues[i];
    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);

    Serial.print("PWM = ");
    Serial.println(pwm);

    delay(4000); // Hold each speed for 4 seconds
  }

  stopMotors();
  Serial.println("\n=== Experiment Finished ===");

  while (true); // Stop program after one full cycle
}

// === Helper Functions ===
void setDirection(bool forward) {
  if (forward) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Motors stopped.");
}

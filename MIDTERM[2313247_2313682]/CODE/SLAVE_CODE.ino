/* ===================== LIBRARIES ===================== */
#include <WiFi.h>          // Required for ESP-NOW (uses WiFi hardware)
#include <esp_now.h>       // ESP-NOW communication protocol
#include <ESP32Servo.h>    // Servo control library for ESP32

/* ===================== DEFINES ===================== */
#define SERIAL_PLOTTER_MODE 1   // 1 = Serial Plotter output, 0 = normal text output

#define SERVO_PIN 18            // GPIO pin connected to servo signal
#define LED_PIN 2               // GPIO pin connected to LED

// LED PWM configuration
#define LED_CHANNEL 1           // PWM channel number
#define LED_FREQ 5000           // PWM frequency (Hz)
#define LED_RES 8               // PWM resolution (8-bit → 0–255)

// Servo pulse width limits (microseconds)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

// Operating modes
#define MODE_AUTO   0
#define MODE_MANUAL 1

/* ===================== OBJECTS ===================== */
Servo myservo;   // Servo object

/* ===================== DATA STRUCTURE ===================== */
// Structure to receive data via ESP-NOW
typedef struct {
  int angle;          // Desired servo angle (0–180)
  bool isDark;        // LDR-based darkness flag
  int potValue;       // Potentiometer value (unused here)
  int ldrValue;       // Raw LDR sensor value
  uint8_t mode;       // AUTO or MANUAL mode
  uint8_t manualLed;  // Manual LED ON/OFF command
} DataPacket;

DataPacket data;  // Variable to store received data

/* ===================== SERVO VARIABLES ===================== */
int currentAngle = 0;     // Current servo angle
int targetAngle  = 0;     // Target servo angle received via ESP-NOW

unsigned long lastServoUpdate = 0;  // Timestamp of last servo movement
const int SERVO_INTERVAL = 20;      // Servo update interval (ms)

/* ===================== LED VARIABLES ===================== */
int ledBrightness = 0;               // Current LED brightness
int fadeAmount = 6;                  // Step size for fading
unsigned long lastFadeUpdate = 0;    // Timestamp of last fade update
const int FADE_INTERVAL = 20;        // Fade update interval (ms)

/* ===================== ESP-NOW RECEIVE CALLBACK ===================== */
// Called automatically when ESP-NOW data is received
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copy received bytes into the data structure
  memcpy(&data, incomingData, sizeof(data));

  // Limit servo angle to valid range
  targetAngle = constrain(data.angle, 0, 180);
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);   // Start serial communication

  // Configure PWM for LED control
  ledcSetup(LED_CHANNEL, LED_FREQ, LED_RES);   // Setup PWM channel
  ledcAttachPin(LED_PIN, LED_CHANNEL);         // Attach LED pin to PWM

  // Initialize WiFi in Station mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  esp_now_init();

  // Register receive callback function
  esp_now_register_recv_cb(onReceive);

  // Attach servo with min/max pulse width
  myservo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  // Set initial servo position
  myservo.write(currentAngle);

  Serial.println("SLAVE READY");  // Indicate receiver is ready
}

/* ===================== LOOP ===================== */
void loop() {

  /* -------- SERVO CONTROL -------- */
  // Smoothly move servo toward target angle
  if (millis() - lastServoUpdate >= SERVO_INTERVAL) {
    lastServoUpdate = millis();

    if (currentAngle < targetAngle) currentAngle++;
    else if (currentAngle > targetAngle) currentAngle--;

    myservo.write(currentAngle);  // Update servo position
  }

  /* -------- LED CONTROL -------- */
  if (data.mode == MODE_MANUAL) {
    // MANUAL MODE: LED controlled directly by sender
    ledcWrite(LED_CHANNEL, data.manualLed ? 255 : 0);
  } else {
    // AUTO MODE
    if (data.isDark) {
      // If environment is dark → fade LED
      if (millis() - lastFadeUpdate >= FADE_INTERVAL) {
        lastFadeUpdate = millis();

        ledBrightness += fadeAmount;

        // Reverse fade direction at limits
        if (ledBrightness <= 0 || ledBrightness >= 255)
          fadeAmount = -fadeAmount;

        ledcWrite(LED_CHANNEL, ledBrightness);
      }
    } else {
      // If bright → LED fully ON
      ledcWrite(LED_CHANNEL, 255);
    }
  }

  /* -------- SERIAL OUTPUT -------- */
#if SERIAL_PLOTTER_MODE
  // Output scaled values for Arduino Serial Plotter
  Serial.print(map(currentAngle, 0, 180, 0, 100));
  Serial.print(",");
  Serial.print(map(ledBrightness, 0, 255, 0, 100));
  Serial.print(",");
  Serial.println(map(data.ldrValue, 0, 4095, 0, 100));
#else
  // Human-readable serial output
  Serial.print("MODE=");
  Serial.print(data.mode == MODE_AUTO ? "AUTO" : "MANUAL");
  Serial.print(" | LED=");
  Serial.print(data.mode == MODE_MANUAL ?
               (data.manualLed ? "ON" : "OFF") :
               (data.isDark ? "BLINK" : "ON"));
  Serial.print(" | SERVO=");
  Serial.print(currentAngle);
  Serial.print(" | LDR=");
  Serial.println(data.ldrValue);
#endif

  delay(20);  // Small delay to stabilize loop timing
}

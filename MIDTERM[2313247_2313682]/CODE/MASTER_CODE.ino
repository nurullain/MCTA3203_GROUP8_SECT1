#include <WiFi.h>        // Enables WiFi functionality required for ESP-NOW
#include <esp_now.h>    // Enables ESP-NOW peer-to-peer communication (no router needed)

/* =========================================================
   PIN DEFINITIONS
   Define which ESP32 pins are connected to sensors
   ========================================================= */
#define POT_PIN 32      // Analog pin connected to the potentiometer
#define LDR_PIN 34      // Analog pin connected to the LDR (light sensor)

/* =========================================================
   LIGHT THRESHOLD
   Used to decide whether the environment is DARK or BRIGHT
   ========================================================= */
#define LIGHT_THRESHOLD 2000   // ADC value below this means DARK

/* =========================================================
   MODE DEFINITIONS
   AUTO  : System controlled by sensors
   MANUAL: User controls system via Serial commands
   ========================================================= */
#define MODE_AUTO   0
#define MODE_MANUAL 1

/* =========================================================
   SLAVE MAC ADDRESS
   MAC address of the ESP32 that will receive data (SLAVE)
   ========================================================= */
uint8_t SLAVE_MAC[] = {0x00, 0x70, 0x07, 0x83, 0x77, 0x40};

/* =========================================================
   DATA PACKET STRUCTURE
   This structure bundles all data sent via ESP-NOW
   ========================================================= */
typedef struct {
  int angle;            // Servo angle calculated from potentiometer
  bool isDark;          // Light condition flag (true = DARK)
  int potValue;         // Raw potentiometer ADC value (0–4095)
  int ldrValue;         // Raw LDR ADC value (0–4095)
  uint8_t mode;         // Current operating mode (AUTO / MANUAL)
  uint8_t manualLed;    // LED control in MANUAL mode (1 = ON, 0 = OFF)
} DataPacket;

DataPacket data;        // Instance of the data packet

/* =========================================================
   SETUP FUNCTION
   Runs once when ESP32 starts
   ========================================================= */
void setup() {
  Serial.begin(115200);   // Start Serial Monitor for debugging

  // Configure sensor pins as inputs
  pinMode(POT_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  // Initialize default system state
  data.mode = MODE_AUTO;     // Start in AUTO mode
  data.manualLed = 0;        // LED OFF by default

  // Set ESP32 to Station mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW protocol
  esp_now_init();

  // Register the SLAVE ESP32 as a peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, SLAVE_MAC, 6);
  esp_now_add_peer(&peer);

  // Display system status and available commands
  Serial.println("MASTER READY");
  Serial.println("Commands: 2=MANUAL, 4=AUTO, 1=LED ON, 0=LED OFF");
}

/* =========================================================
   SERIAL COMMAND HANDLER
   Reads and processes user commands from Serial Monitor
   ========================================================= */
void handleSerial() {
  if (!Serial.available()) return;  // Exit if no input

  char cmd = Serial.read();          // Read one character

  // Switch to MANUAL mode
  if (cmd == '2') {
    data.mode = MODE_MANUAL;
    Serial.println("MODE: MANUAL");
  }

  // Switch to AUTO mode
  else if (cmd == '4') {
    data.mode = MODE_AUTO;
    Serial.println("MODE: AUTO");
  }

  // Turn LED ON (only in MANUAL mode)
  else if (cmd == '1' && data.mode == MODE_MANUAL) {
    data.manualLed = 1;
    Serial.println("LED: ON");
  }

  // Turn LED OFF (only in MANUAL mode)
  else if (cmd == '0' && data.mode == MODE_MANUAL) {
    data.manualLed = 0;
    Serial.println("LED: OFF");
  }
}

/* =========================================================
   MAIN LOOP
   Runs continuously after setup()
   ========================================================= */
void loop() {
  handleSerial();   // Check for user input

  // Read analog sensor values
  int potValue = analogRead(POT_PIN);  // Potentiometer value
  int ldrValue = analogRead(LDR_PIN);  // LDR value

  // AUTO MODE logic
  if (data.mode == MODE_AUTO) {

    // Convert potentiometer reading to servo angle (0–180°)
    data.angle = map(potValue, 0, 4095, 0, 180);

    // Determine light condition
    data.isDark = (ldrValue < LIGHT_THRESHOLD);
  }

  // Store raw sensor readings for monitoring
  data.potValue = potValue;
  data.ldrValue = ldrValue;

  // Send the data packet to the SLAVE ESP32 via ESP-NOW
  esp_now_send(SLAVE_MAC, (uint8_t *)&data, sizeof(data));

  /* ---------------------------------------------------------
     SERIAL MONITOR OUTPUT (for debugging and observation)
     --------------------------------------------------------- */
  Serial.print("MODE=");
  Serial.print(data.mode == MODE_AUTO ? "AUTO" : "MANUAL");
  Serial.print(" | POT=");
  Serial.print(potValue);
  Serial.print(" | ANGLE=");
  Serial.print(data.angle);
  Serial.print(" | LDR=");
  Serial.print(ldrValue);
  Serial.print(" | LIGHT=");
  Serial.println(data.isDark ? "BRIGHT" : "DARK");

  delay(100);  // Small delay to stabilize readings and communication
}

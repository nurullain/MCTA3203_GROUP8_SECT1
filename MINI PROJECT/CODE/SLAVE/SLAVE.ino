/*
 * SLAVE ESP - Visually Impaired Assistant
 * Output: 1 Vibration Motor with Different Patterns
 * Communication: ESP-NOW receiver
 * 
 * UPDATED: 
 * - Removed all fall detection vibration
 * - Obstacle detection = continuous vibration only
 */

#include <esp_now.h>
#include <WiFi.h>

// ========== WIFI CONFIGURATION ==========
const char* WIFI_SSID = "t9";         // ⚠️ CHANGE THIS (same as Master)
const char* WIFI_PASSWORD = "10092004"; // ⚠️ CHANGE THIS (same as Master)

// ========== CONFIGURATION ==========
// Single vibration motor pin
const uint8_t VIBRATION_MOTOR_PIN = 12;

// Vibration pattern timings
const unsigned long UNLOCK_DURATION = 3000;          // 3 seconds continuous
const unsigned long DARK_PULSE_DURATION = 200;       // Each pulse 200ms
const unsigned long DARK_PULSE_GAP = 150;            // Gap between pulses

// ========== DATA STRUCTURES ==========
struct SensorData {
  bool unlocked;
  bool dark;
  bool obstacleNear;  // New: obstacle detection
};

struct ResponseData {
  bool userOkay;
};

// ========== VIBRATION STATES ==========
enum VibrationPattern {
  PATTERN_NONE,
  PATTERN_UNLOCK,      // 3 seconds continuous
  PATTERN_DARK,        // 2 quick pulses
  PATTERN_OBSTACLE     // Continuous (highest priority)
};

// ========== GLOBAL STATE ==========
SensorData receivedData = {false, false, false};
ResponseData responseData = {false};

VibrationPattern currentPattern = PATTERN_NONE;
unsigned long patternStartTime = 0;
int pulseCount = 0;
bool motorState = false;

// Previous states to detect transitions
bool wasUnlocked = false;
bool wasDark = false;

// Master MAC address (will be filled from received data)
uint8_t masterMac[6] = {0};
unsigned long lastWiFiCheck = 0;

// ========== VIBRATION CONTROL ==========
void startPattern(VibrationPattern pattern) {
  currentPattern = pattern;
  patternStartTime = millis();
  pulseCount = 0;
  motorState = false;
  
  switch(pattern) {
    case PATTERN_UNLOCK:
      Serial.println("🔓 UNLOCK - 3 second vibration");
      break;
    case PATTERN_DARK:
      Serial.println("🌙 DARK - 2 pulse vibration");
      break;
    case PATTERN_OBSTACLE:
      Serial.println("⚠️ OBSTACLE - Continuous vibration");
      break;
    default:
      break;
  }
}

void processVibrationPattern() {
  unsigned long elapsed = millis() - patternStartTime;
  
  switch(currentPattern) {
    case PATTERN_NONE:
      digitalWrite(VIBRATION_MOTOR_PIN, LOW);
      break;
      
    case PATTERN_UNLOCK:
      // 3 seconds continuous vibration
      if (elapsed < UNLOCK_DURATION) {
        digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
      } else {
        digitalWrite(VIBRATION_MOTOR_PIN, LOW);
        currentPattern = PATTERN_NONE;
        Serial.println("✅ Unlock vibration complete");
      }
      break;
      
    case PATTERN_DARK:
      // 2 quick pulses: ON-OFF-ON-OFF
      if (pulseCount == 0) {
        // First pulse
        if (elapsed < DARK_PULSE_DURATION) {
          digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
        } else if (elapsed < DARK_PULSE_DURATION + DARK_PULSE_GAP) {
          digitalWrite(VIBRATION_MOTOR_PIN, LOW);
        } else {
          pulseCount = 1;
          patternStartTime = millis();
        }
      } else if (pulseCount == 1) {
        // Second pulse
        if (elapsed < DARK_PULSE_DURATION) {
          digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
        } else {
          digitalWrite(VIBRATION_MOTOR_PIN, LOW);
          currentPattern = PATTERN_NONE;
          pulseCount = 0;
          Serial.println("✅ Dark vibration complete");
        }
      }
      break;
      
    case PATTERN_OBSTACLE:
      // Continuous vibration until obstacle is clear
      digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
      break;
  }
}

// ========== STATE MANAGEMENT ==========
void updateStates() {
  // Priority: Obstacle > Dark > Unlock
  
  // Check for OBSTACLE (highest priority - overrides everything)
  if (receivedData.unlocked && receivedData.obstacleNear) {
    if (currentPattern != PATTERN_OBSTACLE) {
      startPattern(PATTERN_OBSTACLE);
    }
    return;  // Skip all other patterns
  }
  
  // If obstacle was active but now clear, stop obstacle vibration
  if (currentPattern == PATTERN_OBSTACLE && !receivedData.obstacleNear) {
    currentPattern = PATTERN_NONE;
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);
    Serial.println("✅ Obstacle cleared - vibration stopped");
  }
  
  // If obstacle pattern is active, don't process other patterns
  if (currentPattern == PATTERN_OBSTACLE) {
    return;
  }
  
  // Check for UNLOCK (only when not unlocked before)
  if (receivedData.unlocked && !wasUnlocked) {
    wasUnlocked = true;
    startPattern(PATTERN_UNLOCK);
    return;
  }
  
  // Check for DARK (only on transition from bright to dark)
  if (receivedData.unlocked && receivedData.dark && !wasDark) {
    wasDark = true;
    // Only start dark pattern if not in another pattern
    if (currentPattern == PATTERN_NONE) {
      startPattern(PATTERN_DARK);
    }
    return;
  }
  
  // Update previous states
  if (!receivedData.unlocked) {
    wasUnlocked = false;
  }
  
  if (!receivedData.dark) {
    wasDark = false;
  }
}

// ========== ESP-NOW CALLBACK ==========
void onDataReceive(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
  if (len != sizeof(SensorData)) {
    Serial.println("⚠️ Invalid data size received");
    return;
  }
  
  // Store Master MAC address (for sending response)
  if (masterMac[0] == 0) {
    memcpy(masterMac, macAddr, 6);
    Serial.print("📍 Master MAC saved: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", masterMac[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
  }
  
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  // Debug output
  Serial.print("📥 U:");
  Serial.print(receivedData.unlocked);
  Serial.print(" D:");
  Serial.print(receivedData.dark);
  Serial.print(" O:");
  Serial.println(receivedData.obstacleNear);
  
  updateStates();
}

// ========== INITIALIZATION ==========
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("📡 WiFi Channel: ");
    Serial.println(WiFi.channel());
  } else {
    Serial.println("\n⚠️ WiFi connection failed");
    Serial.println("⏩ Continuing without WiFi");
  }
}

void initMotor() {
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);
  Serial.println("✅ Vibration motor initialized");
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed");
    while (1) delay(100);
  }
  
  esp_now_register_recv_cb(onDataReceive);
  
  Serial.print("Slave MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  Serial.println("✅ ESP-NOW initialized");
  Serial.println("⚠️ Note: Master MAC will be auto-detected from first message");
}

// ========== MAIN FUNCTIONS ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========== SLAVE ESP STARTING ==========");
  
  initMotor();
  
  // Test motor on startup
  Serial.println("🔦 Testing vibration motor...");
  digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
  delay(500);
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);
  Serial.println("✅ Motor test complete");
  
  // Connect to WiFi first (sets the channel)
  connectWiFi();
  
  // Initialize ESP-NOW (will use WiFi channel)
  initESPNow();
  
  Serial.println("========== SYSTEM READY ==========");
  Serial.println("Waiting for data from Master...\n");
  Serial.println("📋 Vibration Patterns:");
  Serial.println("  1. OBSTACLE: Continuous (highest priority)");
  Serial.println("  2. DARK: 2 quick pulses");
  Serial.println("  3. UNLOCK: 3 second continuous\n");
}

void loop() {
  // Check WiFi connection every 30 seconds
  if (millis() - lastWiFiCheck >= 30000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ WiFi disconnected, reconnecting...");
      connectWiFi();
    }
    lastWiFiCheck = millis();
  }
  
  // Process current vibration pattern
  processVibrationPattern();
  
  // Minimal delay
  delay(5);
}

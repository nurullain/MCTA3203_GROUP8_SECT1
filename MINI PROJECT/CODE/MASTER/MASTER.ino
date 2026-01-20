/*
 * MASTER ESP - Visually Impaired Assistant with Fall Detection
 * Sensors: MPU6050, LDR, HuskyLens, DHT11, Ultrasonic
 * Actuators: Buzzer (warns when dark AND obstacle near)
 * Communication: ESP-NOW to slave + Telegram notifications
 * 
 * FIXED: All MPU6050 issues resolved
 */

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <MPU6050.h>
#include <DHT.h>
#include "HUSKYLENS.h"

// ========== WIFI CONFIGURATION ==========
const char* WIFI_SSID = "t9";
const char* WIFI_PASSWORD = "10092004";

// ========== TELEGRAM CONFIGURATION ==========
const char* BOT_TOKEN = "8568136064:AAF1mkFxOpP3Lwf6fxAee_RjOJHOPbaMFNw";
const char* CHAT_ID = "766241973";

// ========== HARDWARE CONFIGURATION ==========
const uint8_t I2C_SDA = 22;
const uint8_t I2C_SCL = 21;
const uint8_t LDR_PIN = 34;
const uint8_t HUSKY_RX = 16;
const uint8_t HUSKY_TX = 17;
const uint8_t DHT_PIN = 25;
const uint8_t ULTRASONIC_TRIG = 33;
const uint8_t ULTRASONIC_ECHO = 32;
const uint8_t BUZZER_PIN = 26;
#define DHT_TYPE DHT11

// ========== SENSOR THRESHOLDS ==========
const float MOVEMENT_THRESHOLD = 1.7;
const int DARK_THRESHOLD = 1000;
const float TEMP_WARNING_THRESHOLD = 35.0;
const int OBSTACLE_DISTANCE_CM = 50;

// ========== BUZZER SETTINGS ==========
const int BUZZER_FREQUENCY = 2000;
const int BUZZER_CHANNEL = 0;
const int BUZZER_RESOLUTION = 8;

// ========== FALL DETECTION SETTINGS ==========
const unsigned long FALL_COOLDOWN_MS = 10000;
const unsigned long TEMP_WARNING_COOLDOWN_MS = 1000;

// Recognized face IDs
const uint8_t VALID_FACE_IDS[] = {1, 2, 5};
const uint8_t NUM_VALID_FACES = 3;

// Slave device MAC address
const uint8_t SLAVE_MAC[] = {0x00, 0x70, 0x07, 0x83, 0x77, 0x40};

// Update intervals
const unsigned long SENSOR_UPDATE_INTERVAL_MS = 50;
const unsigned long ESPNOW_SEND_INTERVAL_MS = 50;

// ========== DATA STRUCTURES ==========
struct SensorData {
  bool unlocked;
  bool dark;
  bool obstacleNear;
};

struct ResponseData {
  bool userOkay;
};

// ========== GLOBAL OBJECTS ==========
MPU6050 mpu;
HUSKYLENS huskylens;
DHT dht(DHT_PIN, DHT_TYPE);
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

// ========== GLOBAL VARIABLES ==========
SensorData dataToSend = {false, false, false};
bool faceUnlocked = false;
bool huskyLensConnected = false;
unsigned long lastFallAlert = 0;
unsigned long lastTempWarning = 0;
unsigned long lastSensorRead = 0;
unsigned long lastESPNowSend = 0;
unsigned long lastDHTRead = 0;
unsigned long lastWiFiCheck = 0;
float lastMovementMagnitude = 0.0;
float currentTemperature = 0.0;
float currentHumidity = 0.0;
bool wifiConnected = false;
int espnowFailCount = 0;
int espnowSuccessCount = 0;
bool buzzerActive = false;

// ========== ESP-NOW CALLBACKS ==========
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    espnowFailCount++;
    if (espnowFailCount % 10 == 0) {
      Serial.print("⚠️ ESP-NOW failures: ");
      Serial.print(espnowFailCount);
      Serial.print(" | Success: ");
      Serial.println(espnowSuccessCount);
    }
  } else {
    espnowSuccessCount++;
  }
}

void onDataReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == sizeof(ResponseData)) {
    ResponseData response;
    memcpy(&response, incomingData, sizeof(response));
    
    if (response.userOkay) {
      Serial.println("✅ USER IS OKAY - Received from Slave");
      
      if (wifiConnected && WiFi.status() == WL_CONNECTED) {
        time_t now = time(nullptr);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        char timeStr[64];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
        
        String message = "✅ *USER IS OKAY*\n\n";
        message += "⏰ Time: " + String(timeStr) + "\n";
        message += "👤 User confirmed safe status\n";
        message += "🌡️ Temperature: " + String(currentTemperature, 1) + "°C\n";
        message += "💧 Humidity: " + String(currentHumidity, 1) + "%\n\n";
        message += "✅ System operating normally.";
        
        if (bot.sendMessage(CHAT_ID, message, "Markdown")) {
          Serial.println("📤 'User Okay' message sent to Telegram");
        } else {
          Serial.println("❌ Failed to send 'User Okay' to Telegram");
        }
      }
    }
  }
}

// ========== BUZZER FUNCTIONS ==========
void initBuzzer() {
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQUENCY, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0);
  Serial.print("✅ Buzzer initialized on GPIO");
  Serial.println(BUZZER_PIN);
}

void buzzerOn() {
  if (!buzzerActive) {
    ledcWrite(BUZZER_CHANNEL, 128);
    buzzerActive = true;
    Serial.println("🔔 BUZZER ON - Dark & Obstacle Detected!");
  }
}

void buzzerOff() {
  if (buzzerActive) {
    ledcWrite(BUZZER_CHANNEL, 0);
    buzzerActive = false;
    Serial.println("🔕 Buzzer off");
  }
}

// ========== WIFI FUNCTIONS ==========
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
    wifiConnected = true;
  } else {
    Serial.println("\n⚠️ WiFi connection failed");
    Serial.println("⏩ Continuing without Telegram notifications");
    wifiConnected = false;
  }
}

// ========== TELEGRAM FUNCTIONS ==========
void sendTelegramAlert(float magnitude) {
  if (!wifiConnected) {
    Serial.println("⚠️ No WiFi - cannot send Telegram alert");
    return;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi disconnected, reconnecting...");
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("❌ Cannot send alert - no WiFi");
      return;
    }
  }
  
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char timeStr[64];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  
  String message = "🚨 *FALL DETECTED!*\n\n";
  message += "⏰ Time: " + String(timeStr) + "\n";
  message += "📊 Movement level: " + String(magnitude, 2) + " g\n\n";
  message += "⚠️ Please check on the user immediately!";
  
  Serial.println("📤 Sending fall alert to Telegram...");
  if (bot.sendMessage(CHAT_ID, message, "Markdown")) {
    Serial.println("✅ Fall alert sent successfully!");
  } else {
    Serial.println("❌ Failed to send fall alert");
  }
  
  // Reset MPU after alert
  Serial.println("🔄 Resetting MPU6050 after alert...");
  delay(100);
  mpu.setSleepEnabled(false);
  delay(10);
  Serial.println("✅ MPU reset complete");
}

void sendTemperatureWarning() {
  if (!wifiConnected) {
    return;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) {
      return;
    }
  }
  
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char timeStr[64];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  
  String message = "🌡️ *HIGH TEMPERATURE WARNING*\n\n";
  message += "⏰ Time: " + String(timeStr) + "\n";
  message += "🔥 Temperature: " + String(currentTemperature, 1) + "°C\n";
  message += "💧 Humidity: " + String(currentHumidity, 1) + "%\n\n";
  message += "⚠️ Environment temperature is high!\n";
  message += "Please ensure user stays hydrated and cool.";
  
  Serial.println("📤 Sending temperature warning to Telegram...");
  if (bot.sendMessage(CHAT_ID, message, "Markdown")) {
    Serial.println("✅ Temperature warning sent!");
  } else {
    Serial.println("❌ Failed to send temperature warning");
  }
}

// ========== INITIALIZATION FUNCTIONS ==========
void initUltrasonic() {
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  Serial.println("✅ Ultrasonic sensor initialized (TRIG=33, ECHO=32)");
}

void initMPU() {
  Serial.println("Initializing MPU6050...");
  
  // Try both pin configurations
  uint8_t sda_pin = 21;
  uint8_t scl_pin = 22;
  
  Serial.println("\n=== Trying Pin Configuration 1 ===");
  Serial.print("SDA: GPIO");
  Serial.print(sda_pin);
  Serial.print(", SCL: GPIO");
  Serial.println(scl_pin);
  
  Wire.end();
  delay(100);
  
  Wire.begin(sda_pin, scl_pin);
  pinMode(sda_pin, INPUT_PULLUP);
  pinMode(scl_pin, INPUT_PULLUP);
  delay(200);
  
  Wire.setClock(50000);
  delay(100);
  
  // Scan for I2C devices
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  bool foundMPU = false;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  ✅ Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
      
      if (address == 0x68 || address == 0x69) {
        foundMPU = true;
        Serial.println("     ^ This looks like MPU6050!");
      }
    }
  }
  
  if (nDevices == 0) {
    Serial.println("\n❌ NO I2C devices found!");
    Serial.println("\n=== Trying Pin Configuration 2 ===");
    Serial.println("SDA: GPIO22, SCL: GPIO21 (swapped)");
    
    sda_pin = 22;
    scl_pin = 21;
    
    Wire.end();
    delay(100);
    Wire.begin(sda_pin, scl_pin);
    pinMode(sda_pin, INPUT_PULLUP);
    pinMode(scl_pin, INPUT_PULLUP);
    delay(200);
    Wire.setClock(50000);
    delay(100);
    
    Serial.println("Scanning I2C bus again...");
    for(address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      
      if (error == 0) {
        Serial.print("  ✅ Found device at 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
        nDevices++;
        
        if (address == 0x68 || address == 0x69) {
          foundMPU = true;
          Serial.println("     ^ This looks like MPU6050!");
        }
      }
    }
    
    if (nDevices == 0) {
      Serial.println("\n❌ STILL no devices found!");
      Serial.println("\n🔧 HARDWARE TROUBLESHOOTING:");
      Serial.println("  1. Check wiring:");
      Serial.println("     MPU VCC → ESP32 3.3V (NOT 5V!)");
      Serial.println("     MPU GND → ESP32 GND");
      Serial.println("     MPU SDA → ESP32 GPIO21 or GPIO22");
      Serial.println("     MPU SCL → ESP32 GPIO22 or GPIO21");
      Serial.println("  2. Try connecting VCC to 5V if module has regulator");
      Serial.println("  3. Add external 4.7kΩ pull-up resistors");
      Serial.println("  4. Check for loose connections");
      Serial.println("  5. Try a different MPU6050 module");
      while (1) delay(1000);
    }
  }
  
  if (!foundMPU) {
    Serial.println("\n⚠️ Found I2C devices but none at 0x68/0x69");
    Serial.println("MPU6050 should be at address 0x68 or 0x69");
  }
  
  Serial.print("\nUsing pins - SDA: GPIO");
  Serial.print(sda_pin);
  Serial.print(", SCL: GPIO");
  Serial.println(scl_pin);
  
  // Initialize MPU
  mpu.initialize();
  delay(100);
  
  // Test connection with retries
  int attempts = 0;
  bool connected = false;
  while (attempts < 5 && !connected) {
    connected = mpu.testConnection();
    if (!connected) {
      Serial.print("⚠️ MPU6050 attempt ");
      Serial.print(attempts + 1);
      Serial.println(" failed, retrying...");
      
      mpu.reset();
      delay(100);
      mpu.initialize();
      delay(100);
      
      attempts++;
    }
  }
  
  if (!connected) {
    Serial.println("\n❌ MPU6050 connection failed after 5 attempts");
    Serial.println("Device found on I2C bus but not responding correctly");
    Serial.println("\nPossible causes:");
    Serial.println("  - Damaged MPU6050");
    Serial.println("  - Insufficient power supply");
    Serial.println("  - Faulty module");
    while (1) delay(1000);
  }
  
  Serial.println("✅ MPU6050 connection successful!");
  
  // Wake up and configure
  mpu.setSleepEnabled(false);
  delay(10);
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true);
  delay(10);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  delay(10);
  
  Serial.println("✅ MPU6050 configured");
  
  // Test readings
  Serial.println("\nTesting sensor readings...");
  delay(100);
  
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  Serial.print("Raw values - ax: ");
  Serial.print(ax);
  Serial.print(", ay: ");
  Serial.print(ay);
  Serial.print(", az: ");
  Serial.println(az);
  
  if (ax == 0 && ay == 0 && az == 0) {
    Serial.println("⚠️ WARNING: All zeros - sensor not reading!");
  } else if (ax == -1 && ay == -1 && az == -1) {
    Serial.println("⚠️ WARNING: All -1 - communication error!");
  } else {
    float gx = ax / 16384.0;
    float gy = ay / 16384.0;
    float gz = az / 16384.0;
    float mag = sqrt(gx*gx + gy*gy + gz*gz);
    
    Serial.print("G-force magnitude: ");
    Serial.print(mag, 3);
    Serial.println("g");
    
    if (mag < 0.5 || mag > 1.5) {
      Serial.println("⚠️ Unusual magnitude - expected ~1.0g when stationary");
    } else {
      Serial.println("✅ Sensor readings look good!");
    }
  }
  
  Serial.println("\n✅✅✅ MPU6050 FULLY INITIALIZED ✅✅✅\n");
}

// ========== MPU KEEP-ALIVE FUNCTION ==========
void keepMPUAwake() {
  static unsigned long lastWakeUp = 0;
  
  if (millis() - lastWakeUp >= 5000) {
    mpu.setSleepEnabled(false);
    lastWakeUp = millis();
  }
}

void initDHT() {
  dht.begin();
  delay(2000);
  
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  
  if (isnan(temp) || isnan(hum)) {
    Serial.println("⚠️ DHT11 not responding (will retry)");
  } else {
    Serial.print("✅ DHT11 initialized - Temp: ");
    Serial.print(temp);
    Serial.print("°C, Humidity: ");
    Serial.print(hum);
    Serial.println("%");
  }
}

void initHuskyLens() {
  Serial.println("Initializing HuskyLens...");
  Serial.print("Using pins - RX: ");
  Serial.print(HUSKY_RX);
  Serial.print(", TX: ");
  Serial.println(HUSKY_TX);
  
  Serial2.end();
  delay(100);
  
  Serial2.begin(9600, SERIAL_8N1, HUSKY_RX, HUSKY_TX);
  delay(500);
  
  Serial.println("Attempting to connect to HuskyLens...");
  
  int attempts = 0;
  while (!huskylens.begin(Serial2) && attempts < 5) {
    Serial.print("❌ Attempt ");
    Serial.print(attempts + 1);
    Serial.println(" failed. Retrying...");
    delay(1000);
    attempts++;
  }
  
  if (attempts >= 5) {
    Serial.println("\n❌ HuskyLens connection FAILED");
    Serial.println("⏩ System will continue WITHOUT face recognition");
    huskyLensConnected = false;
    faceUnlocked = false;
    return;
  }
  
  huskylens.writeAlgorithm(ALGORITHM_FACE_RECOGNITION);
  huskyLensConnected = true;
  Serial.println("✅ HuskyLens initialized and ready!");
}

void initESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed");
    while (1) delay(100);
  }
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceive);
  
  Serial.print("Master MAC: ");
  Serial.println(WiFi.macAddress());
  
  int32_t channel = WiFi.channel();
  Serial.print("📡 Current WiFi Channel: ");
  Serial.println(channel);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, SLAVE_MAC, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    while (1) delay(100);
  }
  
  Serial.print("✅ ESP-NOW initialized on channel ");
  Serial.println(channel);
}

void initTelegram() {
  if (!wifiConnected) {
    return;
  }
  
  secured_client.setInsecure();
  Serial.println("✅ Telegram bot initialized");
  
  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("✅ Time synchronized");
}

// ========== SENSOR READING FUNCTIONS ==========
bool checkFaceRecognition() {
  if (!huskyLensConnected) {
    return false;
  }
  
  if (!huskylens.request()) {
    return false;
  }
  
  if (!huskylens.available()) {
    return false;
  }
  
  while (huskylens.available()) {
    HUSKYLENSResult result = huskylens.read();
    
    for (uint8_t i = 0; i < NUM_VALID_FACES; i++) {
      if (result.ID == VALID_FACE_IDS[i]) {
        Serial.print("🔓 Face recognized → ID ");
        Serial.println(result.ID);
        return true;
      }
    }
  }
  
  return false;
}

// ========== ENHANCED CHECK MOVEMENT WITH ERROR RECOVERY ==========
bool checkMovement(float &magnitude) {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // CHECK FOR ZERO VALUES (sensor died)
  if (ax == 0 && ay == 0 && az == 0) {
    static unsigned long lastZeroWarning = 0;
    if (millis() - lastZeroWarning >= 5000) {
      Serial.println("⚠️ MPU returning zeros - attempting recovery...");
      
      mpu.setSleepEnabled(false);
      delay(10);
      
      mpu.getAcceleration(&ax, &ay, &az);
      
      if (ax == 0 && ay == 0 && az == 0) {
        Serial.println("❌ MPU recovery failed - still zeros");
        Serial.println("💡 Try restarting ESP32");
      } else {
        Serial.println("✅ MPU recovered!");
      }
      
      lastZeroWarning = millis();
    }
    
    magnitude = 0.0;
    return false;
  }
  
  // CHECK FOR -1 VALUES (I2C error)
  if (ax == -1 && ay == -1 && az == -1) {
    static unsigned long lastErrorWarning = 0;
    if (millis() - lastErrorWarning >= 5000) {
      Serial.println("⚠️ MPU I2C error - attempting recovery...");
      
      mpu.initialize();
      delay(50);
      mpu.setSleepEnabled(false);
      delay(10);
      
      lastErrorWarning = millis();
    }
    
    magnitude = 0.0;
    return false;
  }
  
  // Normal operation - convert to g-force
  float gx = ax / 16384.0;
  float gy = ay / 16384.0;
  float gz = az / 16384.0;
  magnitude = sqrt(gx * gx + gy * gy + gz * gz);
  
  // Debug output every 2 seconds
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 2000) {
    Serial.print("📊 MPU → Raw: [");
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.print(az);
    Serial.print("] | Magnitude: ");
    Serial.print(magnitude, 3);
    Serial.print("g");
    
    if (magnitude > MOVEMENT_THRESHOLD) {
      Serial.print(" ⚠️ ABOVE THRESHOLD!");
    } else {
      Serial.print(" ✅ Normal");
    }
    Serial.println();
    
    lastDebug = millis();
  }
  
  return (magnitude > MOVEMENT_THRESHOLD);
}

bool checkDarkness() {
  int ldrValue = analogRead(LDR_PIN);
  bool isDark = (ldrValue > DARK_THRESHOLD);
  
  static unsigned long lastLDRDebug = 0;
  if (millis() - lastLDRDebug >= 3000) {
    Serial.print("💡 LDR: ");
    Serial.print(ldrValue);
    Serial.print(" | Threshold: ");
    Serial.print(DARK_THRESHOLD);
    Serial.print(" | Dark: ");
    Serial.println(isDark ? "YES" : "NO");
    lastLDRDebug = millis();
  }
  
  return isDark;
}

int getDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  
  int distance = duration * 0.034 / 2;
  
  if (duration == 0) {
    return 999;
  }
  
  return distance;
}

bool checkObstacle() {
  int distance = getDistance();
  bool isNear = (distance < OBSTACLE_DISTANCE_CM && distance > 0);
  
  static unsigned long lastObstacleDebug = 0;
  if (millis() - lastObstacleDebug >= 1000) {
    Serial.print("📏 Distance: ");
    Serial.print(distance);
    Serial.print(" cm | Threshold: ");
    Serial.print(OBSTACLE_DISTANCE_CM);
    Serial.print(" cm | Obstacle: ");
    Serial.println(isNear ? "YES ⚠️" : "NO");
    lastObstacleDebug = millis();
  }
  
  return isNear;
}

void readDHT() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  
  if (!isnan(temp) && !isnan(hum)) {
    currentTemperature = temp;
    currentHumidity = hum;
  }
}

// ========== MAIN FUNCTIONS ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========== MASTER ESP STARTING ==========");
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  initMPU();
  initDHT();
  initHuskyLens();
  initUltrasonic();
  initBuzzer();
  
  connectWiFi();
  initTelegram();
  
  initESPNow();
  
  Serial.println("========== SYSTEM READY ==========\n");
}

void loop() {
  unsigned long currentTime = millis();
  
  // ✅ STEP 3: Keep MPU awake
  keepMPUAwake();
  
  // Check WiFi connection every 30 seconds
  if (currentTime - lastWiFiCheck >= 30000) {
    if (WiFi.status() != WL_CONNECTED && wifiConnected) {
      Serial.println("⚠️ WiFi disconnected, reconnecting...");
      connectWiFi();
    }
    lastWiFiCheck = currentTime;
  }
  
  if (!faceUnlocked) {
    faceUnlocked = checkFaceRecognition();
    if (faceUnlocked) {
      Serial.println("✅✅✅ SYSTEM UNLOCKED! ✅✅✅");
    }
  }
  
  dataToSend.unlocked = faceUnlocked;
  
  if (currentTime - lastSensorRead >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorRead = currentTime;
    
    if (currentTime - lastDHTRead >= 2000) {
      readDHT();
      lastDHTRead = currentTime;
      
      if (faceUnlocked && currentTemperature >= TEMP_WARNING_THRESHOLD) {
        if (currentTime - lastTempWarning >= TEMP_WARNING_COOLDOWN_MS) {
          Serial.println("🌡️ HIGH TEMPERATURE DETECTED!");
          sendTemperatureWarning();
          lastTempWarning = currentTime;
        }
      }
    }
    
    if (faceUnlocked) {
      float magnitude = 0.0;
      bool movementDetected = checkMovement(magnitude);
      lastMovementMagnitude = magnitude;
      
      if (movementDetected) {
        Serial.println("╔═════════════════════════════╗");
        Serial.print  ("║ 🚨 FALL! Movement: ");
        Serial.print(magnitude, 2);
        Serial.println(" g ║");
        Serial.println("╚═════════════════════════════╝");
        
        if (currentTime - lastFallAlert >= FALL_COOLDOWN_MS) {
          sendTelegramAlert(magnitude);
          lastFallAlert = currentTime;
        }
      }
      
      dataToSend.dark = checkDarkness();
      dataToSend.obstacleNear = checkObstacle();
      
      // Control buzzer based on darkness AND obstacle detection
      if (dataToSend.dark && dataToSend.obstacleNear) {
        buzzerOn();
      } else {
        buzzerOff();
      }
    } else {
      dataToSend.dark = false;
      dataToSend.obstacleNear = false;
      buzzerOff();
    }
  }
  
  if (currentTime - lastESPNowSend >= ESPNOW_SEND_INTERVAL_MS) {
    lastESPNowSend = currentTime;
    
    esp_now_send(SLAVE_MAC, (uint8_t*)&dataToSend, sizeof(dataToSend));
    
    static unsigned long lastDebugPrint = 0;
    if (currentTime - lastDebugPrint >= 2000) {
      Serial.print("Status → U:");
      Serial.print(dataToSend.unlocked);
      Serial.print(" D:");
      Serial.print(dataToSend.dark);
      Serial.print(" O:");
      Serial.print(dataToSend.obstacleNear);
      Serial.print(" 🔔:");
      Serial.print(buzzerActive ? "ON" : "OFF");
      Serial.print(" T:");
      Serial.print(currentTemperature, 1);
      Serial.print("°C");
      
      if (currentTemperature >= TEMP_WARNING_THRESHOLD) {
        Serial.print(" ⚠️HOT!");
      }
      
      Serial.print(" H:");
      Serial.print(currentHumidity, 0);
      Serial.print("% | ESP-NOW: ");
      Serial.print(espnowSuccessCount);
      Serial.print("✓ ");
      Serial.print(espnowFailCount);
      Serial.println("✗");
      lastDebugPrint = currentTime;
    }
  }
  
  delay(5);
}

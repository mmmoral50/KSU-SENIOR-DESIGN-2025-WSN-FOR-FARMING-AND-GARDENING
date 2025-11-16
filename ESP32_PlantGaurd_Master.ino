/*
 * Heltec WiFi LoRa 32 V3.2 – MASTER Gateway Node with Power Management
 * ------------------------------------------------------------
 * Role: MASTER (WiFi+MQTT Gateway - receives LoRa from slaves)
 * 
 * FEATURES:
 *  ✓ WiFi Manager for easy network configuration
 *  ✓ MQTT connectivity to PlantGuard dashboard
 *  ✓ LoRa receiver for SLAVE node data
 *  ✓ Automatic packet forwarding to MQTT broker
 *  ✓ Optional local RS485 sensor (can monitor own location too)
 *  ✓ Battery monitoring with voltage and percentage
 *  ✓ OLED auto-sleep after 5 minutes (configurable)
 *  ✓ PRG button wakes OLED and restarts timer
 *  ✓ Auto page rotation every 15 seconds
 *  ✓ Configurable power settings via MQTT
 *  ✓ Light sleep mode for battery optimization
 *  ✓ RSSI/SNR signal quality display
 *  
 * BUTTON CONTROLS (PROGRAM/BOOT button - GPIO 0):
 *  - Quick press (<1s): Wake OLED and scroll to next page
 *  - Hold 3 seconds: RESET WiFi settings and restart
 *  - Hold 10 seconds: SHUT DOWN (deep sleep until button press)
 *  
 * LIBRARIES REQUIRED:
 *  - WiFiManager by tzapu
 *  - PubSubClient
 *  - ArduinoJson
 *  - Heltec ESP32 LoRa v3 by ropg
 */

#include <heltec_unofficial.h>  // Must be first - includes display driver
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/*********************** Configuration ************************/
const char* MQTT_SERVER = "test.mosquitto.org";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASSWORD = "";
const char* NODE_ID = "MASTER_001";  // ← CHANGE FOR EACH MASTER NODE

// LoRa Configuration (must match SLAVE nodes)
// Default to 915 MHz for US (can be configured via dashboard)
#define LORA_FREQUENCY 915.0    // EU: 868.0, US: 915.0, Asia: 433.0
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_TX_POWER 14

// Runtime LoRa configuration (updated via MQTT)
struct LoRaConfig {
  float frequency = LORA_FREQUENCY;
  float bandwidth = LORA_BANDWIDTH;
  int spreadingFactor = LORA_SPREADING_FACTOR;
  int txPower = LORA_TX_POWER;
  int codingRate = 1;  // 4/5
  int preambleLength = 8;
  int syncWord = 0x12;
} loraConfig;

/*********************** Power Management ************************/
#define RESET_BUTTON 0
#define OLED_SLEEP_TIMEOUT 300000    // 5 minutes in milliseconds
#define DEFAULT_UPDATE_INTERVAL 5    // 5 seconds default

// Power settings (will be updated from MQTT config messages)
uint32_t updateInterval = DEFAULT_UPDATE_INTERVAL * 1000;  // Convert to ms
uint32_t oledSleepTimeout = OLED_SLEEP_TIMEOUT;
bool oledAsleep = false;
uint32_t lastOledActivity = 0;

/*********************** WiFi Manager ************************/
WiFiManager wifiManager;

/*********************** RS485 Configuration (Optional) ************************/
#define RS485_TX   35
#define RS485_RX   33
#define RS485_EN   38

HardwareSerial& RS485 = Serial2;

inline void rs485_tx() { digitalWrite(RS485_EN, HIGH); }
inline void rs485_rx() { digitalWrite(RS485_EN, LOW); }

/*********************** Battery Monitor ************************/
float readVBAT() {
  const int VBAT_Read = 1;
  const int ADC_Ctrl  = 37;
  const int resolution = 12;
  const int adcMax = pow(2, resolution) - 1;
  const float adcMaxVoltage = 3.3;
  const int R1 = 390;
  const int R2 = 100;
  const float measuredVoltage = 4.2;
  const float reportedVoltage = 4.095;
  const float factor = (adcMaxVoltage / adcMax) * ((R1 + R2) / (float)R2) * (measuredVoltage / reportedVoltage);

  pinMode(ADC_Ctrl, OUTPUT);
  pinMode(VBAT_Read, INPUT);
  analogReadResolution(resolution);
  analogSetPinAttenuation(VBAT_Read, ADC_11db);

  digitalWrite(ADC_Ctrl, HIGH);
  delay(100);
  int analogValue = analogRead(VBAT_Read);
  digitalWrite(ADC_Ctrl, LOW);

  float floatVoltage = factor * analogValue;
  if (floatVoltage < 0.5) floatVoltage = 0.0;
  return floatVoltage;
}

int batteryPercent(float vbat) {
  if (vbat < 3.3) return 0;
  if (vbat > 4.2) return 100;
  return (int)((vbat - 3.3f) / (4.2f - 3.3f) * 100);
}

/*********************** Global Variables ************************/
uint16_t regs[7];
float H, T, EC, PH, N, P, K;
uint32_t lastPoll = 0;
uint32_t lastReconnect = 0;
int page = 0;
uint32_t resetButtonPressTime = 0;
uint32_t lastPageChange = 0;
bool wifiResetTriggered = false;
bool shutdownTriggered = false;

// Gateway statistics
uint32_t packetsReceived = 0;
uint32_t packetsForwarded = 0;
uint32_t packetsFailed = 0;
String lastSlaveId = "";
int lastRSSI = 0;
float lastSNR = 0;

// Last received SLAVE sensor data (for display)
float slaveH = 0, slaveT = 0, slaveEC = 0, slavePH = 0;
float slaveN = 0, slaveP = 0, slaveK = 0;
int slaveBattery = 0;
bool slaveDataReceived = false;

/*********************** OLED Display (Standard SSD1306Wire from heltec_unofficial.h) ************************/
// Note: heltec_unofficial.h includes the standard SSD1306Wire, not HT_SSD1306Wire
// Constructor: (address, sda, scl, geometry, i2c_bus, frequency)
SSD1306Wire oled(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_128_64, I2C_ONE, 500000);

/*********************** OLED Display Functions ************************/
void wakeOLED() {
  if (oledAsleep) {
    oled.displayOn();
    oledAsleep = false;
    Serial.println("OLED woken up");
  }
  lastOledActivity = millis();
}

void sleepOLED() {
  if (!oledAsleep) {
    oled.clear();
    oled.displayOff();
    oledAsleep = true;
    Serial.println("OLED sleeping to save power");
  }
}

void checkOLEDSleep() {
  // Don't sleep OLED during WiFi configuration
  if (WiFi.status() != WL_CONNECTED) {
    return; // Keep OLED on during WiFi setup
  }
  
  if (!oledAsleep && (millis() - lastOledActivity > oledSleepTimeout)) {
    sleepOLED();
  }
}

void showLogo() {
  wakeOLED();
  oled.clear();
  
  // PlantGuard Logo (text-based - same as SLAVE)
  oled.setFont(ArialMT_Plain_16);
  oled.drawString(10, 8, "PlantGuard");
  
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(5, 28, "IoT Plant Monitor");
  
  // Plant icon using simple graphics
  oled.drawLine(64, 45, 64, 55);  // Stem
  oled.drawCircle(64, 42, 3);      // Leaf
  oled.drawCircle(58, 48, 2);      // Leaf
  oled.drawCircle(70, 48, 2);      // Leaf
  
  oled.display();
}

void showConfigMode() {
  wakeOLED();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "WiFi Config Mode");
  oled.drawString(0, 14, "Connect to:");
  oled.drawString(0, 28, "PlantGuard-" + String(NODE_ID));
  oled.drawString(0, 42, "on your phone");
  oled.drawString(0, 52, "to setup WiFi");
  oled.display();
}

/*********************** MQTT Client ************************/
WiFiClient espClient;
PubSubClient mqtt(espClient);

void applyLoRaConfig() {
  Serial.println("Applying new LoRa configuration...");
  
  // Stop receiving to reconfigure
  radio.standby();
  
  // Apply new settings
  int state = radio.begin(loraConfig.frequency);
  if (state == RADIOLIB_ERR_NONE) {
    radio.setSpreadingFactor(loraConfig.spreadingFactor);
    radio.setBandwidth(loraConfig.bandwidth);
    radio.setOutputPower(loraConfig.txPower);
    radio.setCodingRate(loraConfig.codingRate + 4);  // 4/5 = 1, 4/6 = 2, etc.
    radio.setPreambleLength(loraConfig.preambleLength);
    radio.setSyncWord(loraConfig.syncWord);
    
    Serial.printf("✓ Frequency: %.1f MHz\n", loraConfig.frequency);
    Serial.printf("✓ Bandwidth: %.1f kHz\n", loraConfig.bandwidth);
    Serial.printf("✓ SF: %d\n", loraConfig.spreadingFactor);
    Serial.printf("✓ TX Power: %d dBm\n", loraConfig.txPower);
    
    // Resume receiving
    radio.startReceive();
    Serial.println("LoRa configuration updated successfully!");
  } else {
    Serial.printf("Failed to apply LoRa config: %d\n", state);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT message received on topic: ");
  Serial.println(topic);
  
  // Check if it's a config message
  String topicStr = String(topic);
  if (topicStr.endsWith("/config")) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (!error) {
      // Check if this is a LoRa configuration update
      if (doc.containsKey("configType") && doc["configType"] == "lora") {
        Serial.println("LoRa configuration received from dashboard");
        
        // Update LoRa settings
        if (doc.containsKey("frequency")) {
          loraConfig.frequency = doc["frequency"].as<float>();
        }
        if (doc.containsKey("bandwidth")) {
          loraConfig.bandwidth = doc["bandwidth"].as<float>();
        }
        if (doc.containsKey("spreadingFactor")) {
          loraConfig.spreadingFactor = doc["spreadingFactor"].as<int>();
        }
        if (doc.containsKey("txPower")) {
          loraConfig.txPower = doc["txPower"].as<int>();
        }
        if (doc.containsKey("codingRate")) {
          loraConfig.codingRate = doc["codingRate"].as<int>();
        }
        if (doc.containsKey("preambleLength")) {
          loraConfig.preambleLength = doc["preambleLength"].as<int>();
        }
        if (doc.containsKey("syncWord")) {
          loraConfig.syncWord = doc["syncWord"].as<int>();
        }
        
        // Apply the new configuration
        applyLoRaConfig();
      } else {
        // Update power settings from dashboard
        if (doc.containsKey("updateInterval")) {
          updateInterval = doc["updateInterval"].as<uint32_t>() * 1000; // Convert to ms
          Serial.printf("Update interval set to: %d seconds\n", updateInterval / 1000);
        }
        if (doc.containsKey("oledSleepTimeout")) {
          oledSleepTimeout = doc["oledSleepTimeout"].as<uint32_t>() * 1000; // Convert to ms
          Serial.printf("OLED sleep timeout set to: %d seconds\n", oledSleepTimeout / 1000);
        }
      }
    }
  }
}

void mqttReconnect() {
  if (millis() - lastReconnect < 5000) return;
  lastReconnect = millis();

  Serial.print("MQTT connecting...");
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  
  String clientId = "PlantGuard-" + String(NODE_ID);
  
  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    Serial.println("connected!");
    
    // Subscribe to config topic to receive power management updates
    String configTopic = "esp32/" + String(NODE_ID) + "/config";
    mqtt.subscribe(configTopic.c_str());
    Serial.printf("Subscribed to: %s\n", configTopic.c_str());
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
  }
}

/*********************** LoRa Functions ************************/
void initLoRa() {
  Serial.println("Initializing LoRa...");
  Serial.printf("Default frequency: %.1f MHz (US)\n", loraConfig.frequency);
  
  int state = radio.begin(loraConfig.frequency);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init success!");
    
    radio.setSpreadingFactor(loraConfig.spreadingFactor);
    radio.setBandwidth(loraConfig.bandwidth);
    radio.setOutputPower(loraConfig.txPower);
    radio.setCodingRate(loraConfig.codingRate + 4);  // 4/5 = 1, 4/6 = 2, etc.
    radio.setPreambleLength(loraConfig.preambleLength);
    radio.setSyncWord(loraConfig.syncWord);
    
    Serial.printf("Frequency: %.1f MHz, SF: %d\n", loraConfig.frequency, loraConfig.spreadingFactor);
    Serial.printf("Bandwidth: %.1f kHz, Power: %d dBm\n", loraConfig.bandwidth, loraConfig.txPower);
    
    radio.startReceive();
  } else {
    Serial.printf("LoRa init failed: %d\n", state);
  }
}

void checkLoRaMessages() {
  String message;
  int state = radio.readData(message);
  
  if (state == RADIOLIB_ERR_NONE && message.length() > 0) {
    wakeOLED(); // Wake display when receiving data
    packetsReceived++;
    
    lastRSSI = radio.getRSSI();
    lastSNR = radio.getSNR();
    
    Serial.println("═════════════════════════════");
    Serial.printf("📡 LoRa RX: %s\n", message.c_str());
    Serial.printf("📊 RSSI: %d dBm, SNR: %.2f dB\n", lastRSSI, lastSNR);
    Serial.printf("📦 Total received: %d\n", packetsReceived);
    
    // Parse and forward to MQTT
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (!error) {
      lastSlaveId = doc["nodeId"] | "UNKNOWN";
      
      // Store SLAVE sensor data for display
      // Support both new and legacy field names for backward compatibility
      if (doc.containsKey("soilMoisture") || doc.containsKey("moisture")) {
        // Use new field names if available, fall back to legacy names
        slaveH = doc.containsKey("soilMoisture") ? doc["soilMoisture"].as<float>() : doc["moisture"].as<float>();
        slaveT = doc["temperature"].as<float>();
        slaveEC = doc.containsKey("electricalConductivity") ? doc["electricalConductivity"].as<float>() : doc["ec"].as<float>();
        slavePH = doc.containsKey("pH") ? doc["pH"].as<float>() : doc["ph"].as<float>();
        slaveN = doc["nitrogen"].as<float>();
        slaveP = doc["phosphorus"].as<float>();
        slaveK = doc["potassium"].as<float>();
        slaveBattery = doc.containsKey("batteryLevel") ? doc["batteryLevel"].as<int>() : doc["batteryPercent"].as<int>();
        slaveDataReceived = true;
        
        Serial.printf("📊 SLAVE Data: T=%.1f°C H=%.1f%% pH=%.1f EC=%.0f N=%.0f P=%.0f K=%.0f Batt=%d%%\n",
                      slaveT, slaveH, slavePH, slaveEC, slaveN, slaveP, slaveK, slaveBattery);
      }
      
      if (mqtt.connected()) {
        String topic = "esp32/" + lastSlaveId + "/sensors";
        bool published = mqtt.publish(topic.c_str(), message.c_str());
        
        if (published) {
          packetsForwarded++;
          Serial.printf("✓ Forwarded to MQTT (total: %d)\n", packetsForwarded);
        } else {
          packetsFailed++;
          Serial.printf("✗ MQTT publish failed (total fails: %d)\n", packetsFailed);
        }
      } else {
        packetsFailed++;
        Serial.println("✗ MQTT not connected - packet dropped");
      }
    } else {
      packetsFailed++;
      Serial.println("✗ JSON parse error - invalid packet");
    }
    
    Serial.println("═════════════════════════════");
  }
}

/*********************** Data Publishing (Optional Local Sensor) ************************/
void publishLocalSensorData(bool sensorOk, float vbat) {
  StaticJsonDocument<512> doc;
  doc["nodeId"] = NODE_ID;
  doc["timestamp"] = millis();
  doc["role"] = "MASTER";
  
  if (sensorOk) {
    doc["temperature"] = T;
    doc["soilMoisture"] = H;
    doc["electricalConductivity"] = EC;
    doc["pH"] = PH;
    doc["nitrogen"] = N;
    doc["phosphorus"] = P;
    doc["potassium"] = K;
    doc["npk"] = (N + P + K) / 3.0;
  }
  
  doc["batteryLevel"] = batteryPercent(vbat);
  doc["batteryVoltage"] = vbat;
  doc["packetsForwarded"] = packetsForwarded;
  
  String payload;
  serializeJson(doc, payload);
  
  if (mqtt.connected()) {
    String topic = "esp32/" + String(NODE_ID) + "/sensors";
    mqtt.publish(topic.c_str(), payload.c_str());
    Serial.println("📤 MQTT TX (local): " + payload);
  }
}

/*********************** CRC16-Modbus ************************/
uint16_t mbCRC(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

/*********************** RS485 Sensor Read (Optional) ************************/
bool readSoil(uint16_t* out) {
  uint8_t req[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};

  rs485_tx();
  RS485.write(req, 8);
  RS485.flush();
  rs485_rx();

  uint8_t buf[32];
  size_t idx = 0;
  uint32_t start = millis();

  while (millis() - start < 500) {
    while (RS485.available() && idx < sizeof(buf)) {
      buf[idx++] = RS485.read();
    }
  }

  if (idx < 5 || buf[1] != 0x03) {
    return false;
  }

  for (int i = 0; i < 7; i++)
    out[i] = (buf[3 + 2 * i] << 8) | buf[4 + 2 * i];

  H  = out[0] * 0.1f;
  T  = (int16_t)out[1] * 0.1f;
  EC = out[2];
  PH = out[3] * 0.1f;
  N  = out[4];
  P  = out[5];
  K  = out[6];

  return true;
}

/*********************** OLED Page Functions ************************/
void drawMasterStatus(float vbat) {
  wakeOLED();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  
  oled.drawString(0, 0, "WiFi: " + WiFi.SSID());
  oled.drawString(0, 14, "MQTT: " + String(mqtt.connected() ? "✓" : "✗"));
  oled.drawString(0, 28, "Bat: " + String(batteryPercent(vbat)) + "% (" + String(vbat, 2) + "V)");
  oled.drawString(0, 42, "Fwd: " + String(packetsForwarded) + " pkts");
  oled.drawString(0, 52, "Role: MASTER Gateway");
  
  oled.display();
}

void drawLoRaStats() {
  wakeOLED();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  
  oled.drawString(0, 0, "LoRa Gateway Stats");
  oled.drawString(0, 14, "RX: " + String(packetsReceived) + " pkts");
  oled.drawString(0, 28, "Fwd: " + String(packetsForwarded));
  oled.drawString(0, 42, "Fail: " + String(packetsFailed));
  
  if (lastSlaveId.length() > 0) {
    oled.drawString(0, 52, "Last: " + lastSlaveId);
  }
  
  oled.display();
}

void drawSignalQuality() {
  wakeOLED();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  
  oled.drawString(0, 0, "Last Signal Quality");
  
  if (lastSlaveId.length() > 0) {
    oled.drawString(0, 14, "From: " + lastSlaveId);
    oled.drawString(0, 28, "RSSI: " + String(lastRSSI) + " dBm");
    oled.drawString(0, 42, "SNR: " + String(lastSNR, 1) + " dB");
    
    // Signal quality indicator
    String quality = "";
    if (lastRSSI > -50) quality = "Excellent";
    else if (lastRSSI > -80) quality = "Good";
    else if (lastRSSI > -100) quality = "Fair";
    else quality = "Poor";
    
    oled.drawString(0, 52, "Quality: " + quality);
  } else {
    oled.drawString(0, 28, "No packets received");
  }
  
  oled.display();
}

void drawLocalSensor() {
  wakeOLED();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  
  oled.drawString(0, 0, "Local Sensor Data");
  oled.drawString(0, 14, "Temp: " + String(T, 1) + "°C");
  oled.drawString(0, 28, "Moist: " + String(H, 1) + "%");
  oled.drawString(0, 42, "pH: " + String(PH, 1));
  oled.drawString(0, 52, "EC: " + String(EC, 0) + " µS/cm");
  
  oled.display();
}

void drawSlaveEnv() {
  wakeOLED();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  
  oled.drawString(0, 0, "Soil Data 1");
  oled.drawString(0, 14, "H:" + String(slaveH, 1) + "%  T:" + String(slaveT, 1) + "C");
  oled.drawString(0, 28, "EC:" + String(slaveEC, 0) + " uS/cm");
  oled.drawString(0, 42, "pH:" + String(slavePH, 1));
  if (lastSlaveId.length() > 0) {
    oled.drawString(0, 52, "From: " + lastSlaveId);
  }
  
  oled.display();
}

void drawSlaveNPK() {
  wakeOLED();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  
  oled.drawString(0, 0, "Soil Data 2");
  oled.drawString(0, 14, "N:" + String(slaveN, 0));
  oled.drawString(0, 28, "P:" + String(slaveP, 0));
  oled.drawString(0, 42, "K:" + String(slaveK, 0));
  oled.drawString(0, 52, "Batt: " + String(slaveBattery) + "%");
  
  oled.display();
}

void updateDisplay(float vbat) {
  switch (page) {
    case 0: drawMasterStatus(vbat); break;
    case 1: drawLoRaStats(); break;
    case 2: drawSignalQuality(); break;
    case 3: drawSlaveEnv(); break;       // SLAVE Soil Data 1
    case 4: drawSlaveNPK(); break;       // SLAVE Soil Data 2
  }
}

/*********************** Button Handler ************************/
void handleButton() {
  bool pressed = (digitalRead(RESET_BUTTON) == LOW);
  
  if (pressed && resetButtonPressTime == 0) {
    resetButtonPressTime = millis();
  }
  
  if (!pressed && resetButtonPressTime > 0) {
    uint32_t pressDuration = millis() - resetButtonPressTime;
    
    if (pressDuration < 1000) {
      // Quick press: wake OLED and next page
      page = (page + 1) % 5;  // 5 screens total
      lastPageChange = millis();
      wakeOLED();
      Serial.println("Button: Next page -> " + String(page));
      
      // Immediately update display to show new page
      float vbat = readVBAT();
      updateDisplay(vbat);
    }
    
    resetButtonPressTime = 0;
    wifiResetTriggered = false;
    shutdownTriggered = false;
  }
  
  if (pressed && resetButtonPressTime > 0) {
    uint32_t pressDuration = millis() - resetButtonPressTime;
    
    // WiFi reset at 3 seconds
    if (pressDuration > 3000 && !wifiResetTriggered) {
      wifiResetTriggered = true;
      Serial.println("WiFi RESET triggered!");
      oled.clear();
      oled.setFont(ArialMT_Plain_10);
      oled.drawString(0, 20, "WiFi Reset...");
      oled.display();
      delay(1000);
      wifiManager.resetSettings();
      ESP.restart();
    }
    
    // Shutdown at 10 seconds
    if (pressDuration > 10000 && !shutdownTriggered) {
      shutdownTriggered = true;
      Serial.println("SHUTDOWN triggered!");
      oled.clear();
      oled.setFont(ArialMT_Plain_10);
      oled.drawString(0, 20, "Shutting down...");
      oled.display();
      delay(1000);
      oled.displayOff();
      // Heltec V3.2 library handles power management
      esp_deep_sleep_start();
    }
  }
}

/*********************** Setup ************************/
void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("\n\n");
  Serial.println("==============================");
  Serial.println("   PlantGuard MASTER Node");
  Serial.println("   WiFi/MQTT Gateway");
  Serial.println("==============================");
  Serial.println("Node ID: " + String(NODE_ID));
  Serial.println("==============================\n");
  
  // Power on display
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);
  
  // Initialize OLED
  oled.init();
  oled.setFont(ArialMT_Plain_10);
  oled.clear();
  
  // Initialize Heltec board (LoRa radio)
  heltec_setup();
  
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(RS485_EN, OUTPUT);
  rs485_rx();
  
  // Initialize RS485 (optional for local sensor)
  RS485.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);
  
  // Initialize LoRa
  initLoRa();
  
  // Show logo
  showLogo();
  delay(2000);
  
  // WiFi Manager Setup
  wifiManager.setConfigPortalTimeout(180);  // 3 minutes timeout
  String apName = "PlantGuard-" + String(NODE_ID);
  
  showConfigMode();
  Serial.println("Starting WiFi Manager...");
  
  if (!wifiManager.autoConnect(apName.c_str())) {
    Serial.println("WiFi connection failed - restarting");
    delay(3000);
    ESP.restart();
  }
  
  Serial.println("WiFi connected!");
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  
  // Initial MQTT connection
  mqttReconnect();
  
  wakeOLED();
  lastPoll = millis();
  lastPageChange = millis();
}

/*********************** Main Loop ************************/
void loop() {
  // Handle button presses
  handleButton();
  
  // Check OLED sleep timer
  checkOLEDSleep();
  
  // Maintain WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected - attempting reconnect");
    WiFi.reconnect();
    delay(1000);
    return;
  }
  
  // Maintain MQTT connection
  if (!mqtt.connected()) {
    mqttReconnect();
  } else {
    mqtt.loop();
  }
  
  // Check for incoming LoRa messages (MAIN GATEWAY FUNCTION)
  checkLoRaMessages();
  
  // Auto-rotate pages every 15 seconds
  if (millis() - lastPageChange > 15000) {
    page = (page + 1) % 5;  // 5 screens total
    lastPageChange = millis();
    wakeOLED();
  }
  
  // Optional: Read and publish local sensor data
  if (millis() - lastPoll >= updateInterval) {
    lastPoll = millis();
    
    float vbat = readVBAT();
    bool sensorOk = readSoil(regs);
    
    if (sensorOk) {
      Serial.printf("📊 Local Sensor: T=%.1f°C H=%.1f%% pH=%.1f EC=%.0f N=%.0f P=%.0f K=%.0f\n",
                    T, H, PH, EC, N, P, K);
      publishLocalSensorData(true, vbat);
    } else {
      Serial.println("⚠️  Local sensor read failed (optional)");
      publishLocalSensorData(false, vbat);
    }
    
    updateDisplay(vbat);
  }
  
  // Light sleep between operations to save power
  delay(100);
}

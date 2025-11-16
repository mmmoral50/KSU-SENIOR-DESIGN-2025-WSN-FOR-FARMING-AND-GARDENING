/*
 * Heltec WiFi LoRa 32 V3.2 – SLAVE Sensor Node (LoRa Only)
 * ------------------------------------------------------------
 * Role: SLAVE - Battery-powered sensor node
 * Transmits sensor data via LoRa to MASTER gateway
 * 
 * FEATURES:
 *  ✓ RS485 sensor reading (7-in-1: NPK, pH, EC, Temp, Moisture)
 *  ✓ LoRa transmitter to send data to MASTER gateway
 *  ✓ Battery monitoring with calibrated voltage and percentage
 *  ✓ OLED display with auto-scroll (5s)
 *  
 * LIBRARIES REQUIRED:
 *  - Heltec ESP32 LoRa v3 by ropg
 */

#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <ArduinoJson.h>
#include <RadioLib.h>

/*********************** Pin Definitions ************************/
#define SCL_OLED 18
#define SDA_OLED 17
#define RST_OLED 21
#define Vext 36

/*********************** Configuration ************************/
const char* NODE_ID = "SLAVE_001";  // ← CHANGE FOR EACH SLAVE NODE

// LoRa Configuration (must match MASTER node)
#define LORA_FREQUENCY 915.0        // US: 915.0, EU: 868.0, Asia: 433.0
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODING_RATE 5          // 4/5
#define LORA_TX_POWER 14
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYNC_WORD 0x12

/*********************** Power Management ************************/
#define RESET_BUTTON 0
#define DEFAULT_UPDATE_INTERVAL 5     // 5 seconds default

uint32_t updateInterval = DEFAULT_UPDATE_INTERVAL * 1000;  // Convert to ms

/*********************** RS485 Configuration ************************/
#define RS485_TX   35
#define RS485_RX   33
#define RS485_EN   38

HardwareSerial& RS485 = Serial2;

// RS485 control functions
inline void rs485_tx() { digitalWrite(RS485_EN, HIGH); }
inline void rs485_rx() { digitalWrite(RS485_EN, LOW); }

/*********************** Battery Monitor (Calibrated) ************************/
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

/*********************** OLED Display ************************/
SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED,
                 GEOMETRY_128_64, RST_OLED);

/*********************** LoRa Radio ************************/
SX1262 radio = new Module(8, 14, 12, 13);

/*********************** Sensor Data ************************/
uint16_t regs[7];
float H = 0, T = 0, EC = 0, PH = 0, N = 0, P = 0, K = 0;

/*********************** LoRa Statistics ************************/
int packetsSent = 0;
int packetsFailed = 0;
int lastRSSI = 0;
float lastSNR = 0;
uint32_t lastPoll = 0;

/*********************** Display Management ************************/
int page = 0;

/*********************** OLED Display Functions ************************/
void showLogo() {
  oled.clear();
  
  // PlantGuard Logo (text-based)
  oled.setFont(ArialMT_Plain_16);
  oled.drawString(10, 8, "PlantGuard");
  
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(5, 28, "IoT Plant Monitor");
  
  // Plant icon using simple graphics
  oled.drawLine(64, 45, 64, 55);  // Stem
  oled.drawCircle(64, 42, 3);      // Leaf
  oled.drawCircle(58, 48, 2);      // Leaf
  oled.drawCircle(70, 48, 2);      // Leaf
  
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(25, 52, String(NODE_ID));
  
  oled.display();
}

void drawStatus(float vbat) {
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, String(NODE_ID));
  oled.drawString(0, 14, "Mode: LoRa Only");
  oled.drawString(0, 28, "TX:" + String(packetsSent) + " Fail:" + String(packetsFailed));
  oled.drawString(0, 42, "RSSI:" + String(lastRSSI) + " SNR:" + String(lastSNR, 1));
  oled.drawString(0, 52, "Batt: " + String(vbat, 2) + "V");
  oled.display();
}

void drawEnv() {
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Soil Data 1");
  oled.drawString(0, 14, "H:" + String(H, 1) + "%  T:" + String(T, 1) + "C");
  oled.drawString(0, 28, "EC:" + String(EC, 0) + " uS/cm");
  oled.drawString(0, 42, "pH:" + String(PH, 1));
  oled.display();
}

void drawNPK() {
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Soil Data 2");
  oled.drawString(0, 14, "N:" + String(N, 0));
  oled.drawString(0, 28, "P:" + String(P, 0));
  oled.drawString(0, 42, "K:" + String(K, 0));
  oled.display();
}

/*********************** RS485 Sensor Reading ************************/
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
    Serial.println("No valid Modbus reply");
    return false;
  }

  Serial.print("RS485 RX [");
  Serial.print(idx);
  Serial.print(" bytes]: ");
  for (int i = 0; i < idx; i++) Serial.printf("%02X ", buf[i]);
  Serial.println();

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

/*********************** LoRa Functions ************************/
void initLoRa() {
  Serial.println("==============================");
  Serial.println("Initializing LoRa radio...");
  Serial.println("==============================");
  
  int state = radio.begin(LORA_FREQUENCY);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init success!");
    
    // Set all parameters
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
    radio.setBandwidth(LORA_BANDWIDTH);
    radio.setOutputPower(LORA_TX_POWER);
    radio.setCodingRate(LORA_CODING_RATE);
    radio.setPreambleLength(LORA_PREAMBLE_LENGTH);
    radio.setSyncWord(LORA_SYNC_WORD);
    
    // Display configuration (factory test style)
    Serial.println("LoRa Configuration:");
    Serial.printf("  Frequency      : %.1f MHz\n", LORA_FREQUENCY);
    Serial.printf("  Bandwidth      : %.0f kHz\n", LORA_BANDWIDTH);
    Serial.printf("  Spreading Factor: SF%d\n", LORA_SPREADING_FACTOR);
    Serial.printf("  Coding Rate    : 4/%d\n", LORA_CODING_RATE);
    Serial.printf("  TX Power       : %d dBm\n", LORA_TX_POWER);
    Serial.printf("  Preamble Length: %d\n", LORA_PREAMBLE_LENGTH);
    Serial.printf("  Sync Word      : 0x%02X\n", LORA_SYNC_WORD);
    Serial.println("==============================");
    
    // Start listening for incoming packets
    radio.startReceive();
    Serial.println("LoRa ready - listening for config packets");
  } else {
    Serial.printf("LoRa init FAILED with code: %d\n", state);
  }
}

void checkLoRaConfig() {
  // Only check every 100ms to avoid interfering with display
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 100) return;
  lastCheck = millis();
  
  String message;
  int state = radio.readData(message);
  
  if (state == RADIOLIB_ERR_NONE && message.length() > 0) {
    Serial.println("LoRa RX: " + message);
    
    // Parse JSON config packet
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (!error && doc.containsKey("type")) {
      String type = doc["type"];
      
      // Check if this config is for this node
      if (type == "config" && doc["nodeId"] == NODE_ID) {
        Serial.println("Config update received from MASTER");
        
        // Update interval (in seconds)
        if (doc.containsKey("updateInterval")) {
          updateInterval = doc["updateInterval"].as<uint32_t>() * 1000;
          Serial.printf("Update interval: %d seconds\n", updateInterval / 1000);
        }
        
        Serial.println("Config applied");
      }
    }
  }
}

void sendLoRa(String payload) {
  // Stop receiving to transmit
  radio.standby();
  
  int state = radio.transmit(payload);
  
  if (state == RADIOLIB_ERR_NONE) {
    packetsSent++;
    Serial.println("LoRa TX: " + payload);
  } else {
    packetsFailed++;
    Serial.printf("LoRa TX failed: %d\n", state);
  }
  
  // Resume listening
  radio.startReceive();
}

void publishSensorData(bool sensorOk, float vbat) {
  StaticJsonDocument<512> doc;
  doc["nodeId"] = NODE_ID;
  doc["type"] = "sensor_data";
  doc["rssi"] = lastRSSI;
  doc["snr"] = lastSNR;
  doc["battery"] = vbat;
  doc["batteryPercent"] = batteryPercent(vbat);
  doc["sensorOk"] = sensorOk;
  
  if (sensorOk) {
    doc["moisture"] = H;
    doc["temperature"] = T;
    doc["ec"] = EC;
    doc["ph"] = PH;
    doc["nitrogen"] = N;
    doc["phosphorus"] = P;
    doc["potassium"] = K;
  }
  
  String payload;
  serializeJson(doc, payload);
  
  Serial.println("Sending via LoRa: " + payload);
  sendLoRa(payload);
}


/*********************** Setup ************************/
void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("\n\n");
  Serial.println("==============================");
  Serial.println("   PlantGuard SLAVE Node");
  Serial.println("   LoRa Sensor Transmitter");
  Serial.println("==============================");
  Serial.println("Node ID: " + String(NODE_ID));
  Serial.println("==============================\n");

  // Power on display
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);
  
  // Initialize display
  oled.init();
  
  // Show PlantGuard logo
  Serial.println("Displaying PlantGuard logo...");
  showLogo();
  delay(2000);

  // Initialize RS485
  Serial.println("Initializing RS485 sensor interface...");
  pinMode(RS485_EN, OUTPUT);
  rs485_rx();
  RS485.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.println("RS485 ready (4800 baud, RX=33, TX=35, EN=38)\n");

  // Initialize LoRa radio
  initLoRa();

  Serial.println("\n==============================");
  Serial.println("Setup complete - entering main loop");
  Serial.println("==============================\n");
}

/*********************** Main Loop ************************/
void loop() {
  // Check for LoRa config packets from MASTER
  checkLoRaConfig();
  
  if (millis() - lastPoll > updateInterval) {
    lastPoll = millis();
    
    bool ok = readSoil(regs);
    float vbat = readVBAT();

    switch (page) {
      case 0: drawStatus(vbat); break;
      case 1: drawEnv(); break;
      case 2: drawNPK(); break;
    }
    page = (page + 1) % 3;

    StaticJsonDocument<256> doc;
    doc["ok"] = ok;
    doc["H"] = H;
    doc["T"] = T;
    doc["EC"] = EC;
    doc["pH"] = PH;
    doc["N"] = N;
    doc["P"] = P;
    doc["K"] = K;
    doc["Vbat"] = vbat;

    String payload;
    serializeJson(doc, payload);
    Serial.println("Sensor Data: " + payload);

    publishSensorData(ok, vbat);
  }

  delay(10);
}

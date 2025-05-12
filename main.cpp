#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define LSM6DSO_ADDR 0x6B
#define LED_PIN 2

const char* ssid = "ATTmtSBq2N";
const char* password = "t8rwd4u#byxa";
const char* serverUrl = "http://3.129.89.3:5000/data";  // <-- Replace with your Flask server's IP

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;

int stepCount = 0;
int jumpCount = 0;
float stepThreshold = 1.2;
float jumpThreshold = 2.0;
unsigned long lastStepTime = 0;
unsigned long lastJumpTime = 0;
const unsigned long stepCooldown = 300;
const unsigned long jumpCooldown = 600;

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
}

float readAxis(uint8_t lowReg) {
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(lowReg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSO_ADDR, 2);
  if (Wire.available() == 2) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    int16_t raw = (high << 8) | low;
    return raw * 0.061f / 1000.0f;  // mg to g
  }
  return 0.0f;
}

bool initLSM6DSO() {
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(0x10); // CTRL1_XL: 416Hz, ±2g
  Wire.write(0x60);
  return (Wire.endTransmission() == 0);
}

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    if (value == "ON") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED ON");
    } else if (value == "OFF") {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED OFF");
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  connectWiFi();

  // WHO_AM_I check
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(0x0F);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSO_ADDR, 1);
  if (Wire.available()) {
    uint8_t whoami = Wire.read();
    Serial.printf("WHO_AM_I: 0x%02X\n", whoami);
    if (whoami != 0x6C) {
      Serial.println("Sensor not recognized!");
      while (1);
    }
  }

  if (!initLSM6DSO()) {
    Serial.println("Failed to initialize sensor");
    while (1);
  }

  BLEDevice::init("ESP32_StepJump_Counter");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Steps: 0, Jumps: 0");

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE advertising started!");
}

void loop() {
  float ax = readAxis(0x28);
  float ay = readAxis(0x2A);
  float az = readAxis(0x2C);
  float rms = sqrt(ax * ax + ay * ay + az * az);
  unsigned long now = millis();

  bool eventDetected = false;

  if (rms > jumpThreshold && (now - lastJumpTime) > jumpCooldown) {
    jumpCount++;
    lastJumpTime = now;
    eventDetected = true;
    Serial.printf("[BLE] JUMP #%d at %lu ms\n", jumpCount, now);
  }
  else if (rms > stepThreshold && (now - lastStepTime) > stepCooldown) {
    stepCount++;
    lastStepTime = now;
    eventDetected = true;
    Serial.printf("[BLE] STEP #%d at %lu ms\n", stepCount, now);
  }

  if (eventDetected) {
    // BLE update
    char buffer[50];
    sprintf(buffer, "Steps: %d, Jumps: %d", stepCount, jumpCount);
    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();

    // Send to cloud
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");

      unsigned long sendTime = millis();
      Serial.printf("[ESP32] Sending data at %lu ms, ax=%.2f, ay=%.2f, az=%.2f\n",
                    sendTime, ax, ay, az);

      String json = "{\"ax\":" + String(ax, 3) + ",\"ay\":" + String(ay, 3) + ",\"az\":" + String(az, 3) + "}";
      int httpResponseCode = http.POST(json);

      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("Cloud response: " + response);
      } else {
        Serial.printf("HTTP POST failed: %s\n", http.errorToString(httpResponseCode).c_str());
      }

      http.end();
    }
  }

  delay(20);  // keep short for responsiveness
}

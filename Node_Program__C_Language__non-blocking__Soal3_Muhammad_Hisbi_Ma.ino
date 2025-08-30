
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>

// ---------- CONFIG ----------
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASS";
const char* MQTT_LOCAL_HOST = "192.168.1.10";
const uint16_t MQTT_LOCAL_PORT = 1883;
const char* CLIENT_ID = "node1";
const char* TOPIC_LOCAL = "DATA/LOCAL/SENSOR/PANEL_1";

// RS485 UART pins (ubah sesuai wiring)
#define UART_RX 16
#define UART_TX 17
#define DE_RE   25   // control pin untuk DE/RE pada transceiver

// Fan control
#define PIN_FAN 26

// Modbus slave IDs (sesuaikan)
const uint8_t ID_EM4M = 1;
const uint8_t ID_TX4S = 2;

// Modbus registers placeholders (ISI DARI DATASHEET)
const uint16_t REG_VOLTAGE = 0x0000;
const uint16_t REG_CURRENT = 0x0006;
const uint16_t REG_POWER   = 0x000C;
const uint16_t REG_TEMP    = 0x0100;

// scales (sesuaikan datasheet)
const float SCALE_V = 0.1f;
const float SCALE_I = 0.001f;
const float SCALE_P = 1.0f;
const float SCALE_T = 0.1f;

const float T_NORMAL = 27.0f;    // reference temp
const float T_STEP_PCT = 0.02f;  // 2%

// ---------- GLOBALS ----------
WiFiClient espClient;
PubSubClient mqtt(espClient);

ModbusMaster mbEM4M;
ModbusMaster mbTX4S;

unsigned long tPoll = 0;
unsigned long tMqtt = 0;
unsigned long tNet  = 0;

float gV=0, gI=0, gP=0, gT=0;
bool fanOn = false;

// DE/RE control helpers (called by ModbusMaster before/after transmission)
void preTransmission()  { digitalWrite(DE_RE, HIGH);  delayMicroseconds(40); } // enable TX
void postTransmission() { delayMicroseconds(40); digitalWrite(DE_RE, LOW); }  // enable RX

void wifiEnsure() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void mqttEnsure() {
  if (mqtt.connected()) return;
  mqtt.setServer(MQTT_LOCAL_HOST, MQTT_LOCAL_PORT);
  if (mqtt.connect(CLIENT_ID)) {
    // connected
  }
}

float reg16ToFloat(uint16_t reg, float scale) {
  return (float)reg * scale;
}

bool readEM4M() {
  // contoh: baca voltage (2 reg?) - implement sesuai datasheet
  // disini ambil 1 register contoh saja (sesuaikan)
  if (mbEM4M.readHoldingRegisters(REG_VOLTAGE, 1) == mbEM4M.ku8MBSuccess) {
    gV = reg16ToFloat(mbEM4M.getResponseBuffer(0), SCALE_V);
  } else return false;

  if (mbEM4M.readHoldingRegisters(REG_CURRENT, 1) == mbEM4M.ku8MBSuccess) {
    gI = reg16ToFloat(mbEM4M.getResponseBuffer(0), SCALE_I);
  } else return false;

  if (mbEM4M.readHoldingRegisters(REG_POWER, 1) == mbEM4M.ku8MBSuccess) {
    gP = reg16ToFloat(mbEM4M.getResponseBuffer(0), SCALE_P);
  } else return false;

  return true;
}

bool readTX4S() {
  if (mbTX4S.readHoldingRegisters(REG_TEMP, 1) == mbTX4S.ku8MBSuccess) {
    gT = reg16ToFloat(mbTX4S.getResponseBuffer(0), SCALE_T);
    return true;
  }
  return false;
}

void fanControl() {
  float upThresh = T_NORMAL * (1.0 + T_STEP_PCT);
  if (gT >= upThresh && !fanOn) {
    fanOn = true; digitalWrite(PIN_FAN, HIGH);
  }
  if (gT <= T_NORMAL && fanOn) {
    fanOn = false; digitalWrite(PIN_FAN, LOW);
  }
}

void publishNow() {
  StaticJsonDocument<256> doc;
  doc["ts"] = (uint32_t)(millis()/1000);
  doc["device"] = CLIENT_ID;
  doc["voltage"] = gV;
  doc["current"] = gI;
  doc["power"] = gP;
  doc["temperature"] = gT;
  doc["fan"] = fanOn ? 1 : 0;
  char buf[256]; size_t n = serializeJson(doc, buf);
  mqtt.publish(TOPIC_LOCAL, buf, n);
}

void setup() {
  pinMode(DE_RE, OUTPUT); digitalWrite(DE_RE, LOW);
  pinMode(PIN_FAN, OUTPUT); digitalWrite(PIN_FAN, LOW);

  // Init UART1 (Serial1) with chosen pins
  Serial1.begin(9600, SERIAL_8N1, UART_RX, UART_TX);

  // Modbus init: pass Serial1 (Stream&) to ModbusMaster.begin
  mbEM4M.begin(ID_EM4M, Serial1);
  mbTX4S.begin(ID_TX4S, Serial1);

  mbEM4M.preTransmission(preTransmission);
  mbEM4M.postTransmission(postTransmission);
  mbTX4S.preTransmission(preTransmission);
  mbTX4S.postTransmission(postTransmission);

  // WiFi
  WiFi.mode(WIFI_STA);
  wifiEnsure();

  // MQTT client
  mqtt.setKeepAlive(20);
  // note: don't block here, ensure reconnect in loop()
}

void loop() {
  unsigned long now = millis();

  // Network maintenance (every 2s)
  if (now - tNet >= 2000) {
    tNet = now;
    wifiEnsure();
    mqttEnsure();
  }
  if (mqtt.connected()) mqtt.loop();

  // Polling Modbus every 1s
  if (now - tPoll >= 1000) {
    tPoll = now;
    // read devices (functions return false on fail; handle as needed)
    readEM4M();
    readTX4S();
    fanControl();
  }

  // Publish every 1s if connected
  if ((now - tMqtt >= 1000) && mqtt.connected()) {
    tMqtt = now;
    publishNow();
  }

  // other non-blocking tasks can go here
}

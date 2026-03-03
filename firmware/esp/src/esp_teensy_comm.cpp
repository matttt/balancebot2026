#include <Arduino.h>
#include "esp_teensy_comm.h"

HardwareSerial teensySerial(2);
#define TX_PIN 17
#define RX_PIN 16

struct __attribute__((packed)) ControlPacket {
  uint8_t start1 = 0xAA;
  uint8_t start2 = 0x55;
  float left_torque;
  float right_torque;
  uint8_t mode;
  uint8_t checksum;
};

uint8_t computeChecksum(ControlPacket &pkt) {
  uint8_t sum = 0;

  sum ^= pkt.start1;
  sum ^= pkt.start2;

  uint8_t *lt = (uint8_t*)&pkt.left_torque;
  uint8_t *rt = (uint8_t*)&pkt.right_torque;

  for (int i = 0; i < 4; i++) sum ^= lt[i];
  for (int i = 0; i < 4; i++) sum ^= rt[i];

  sum ^= pkt.mode;

  return sum;
}

void esp_comm_init() {
  Serial.println("ESP comm init");
  teensySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("UART2 started");
}

void esp_comm_update() {

  static uint32_t last_send = 0;

  Serial.println("esp_comm_update running");

  if (millis() - last_send >= 20) {   // 50 Hz high-level loop
    last_send = millis();

    float t = millis() * 0.001f;
    float torque = 1.0f * sinf(2.0f * PI * 0.5f * t); // amplitude is measured in Nm

    ControlPacket pkt;
    pkt.left_torque  = torque;
    pkt.right_torque = torque;
    pkt.mode = 1;

    pkt.checksum = computeChecksum(pkt);

    Serial.print("Sending torque: ");
    Serial.println(torque);

    teensySerial.write((uint8_t*)&pkt, sizeof(pkt));

  }
}
#include <Arduino.h>
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "driver/twai.h"

// ====== PIN CONFIG (change these) ======
static const gpio_num_t TWAI_TX_PIN = static_cast<gpio_num_t>(4);
static const gpio_num_t TWAI_RX_PIN = static_cast<gpio_num_t>(5);

// ====== CAN BITRATE ======
static const twai_timing_config_t TIMING = TWAI_TIMING_CONFIG_500KBITS();

// ====== MODE ======
// - TWAI_MODE_NORMAL: participates on the bus (requires another node to ACK your frames)
// - TWAI_MODE_NO_ACK: good for self-testing / bench setups with no other node to ACK
// - TWAI_MODE_LISTEN_ONLY: sniff only; cannot transmit
static const twai_mode_t MODE = TWAI_MODE_NORMAL;

static bool driver_installed = false;
static uint32_t lastTxMs = 0;
static uint8_t counter = 0;

static void printFrame(const twai_message_t &msg) {
  Serial.print("RX id=0x");
  Serial.print(msg.identifier, HEX);
  Serial.print(msg.extd ? " (ext)" : " (std)");
  Serial.print(" len=");
  Serial.print(msg.data_length_code);
  Serial.print(" data=");

  if (msg.rtr) {
    Serial.print("(RTR)");
  } else {
    for (int i = 0; i < msg.data_length_code; i++) {
      if (msg.data[i] < 16) Serial.print('0');
      Serial.print(msg.data[i], HEX);
      Serial.print(' ');
    }
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("ESP32 TWAI CAN beacon + sniffer starting...");

  // General config (pins + mode)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, MODE);

  // Optional: tweak queue depths (defaults are usually fine)
  g_config.rx_queue_len = 32;
  g_config.tx_queue_len = 8;

  // Accept all frames
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install + start driver
  if (twai_driver_install(&g_config, &TIMING, &f_config) != ESP_OK) {
    Serial.println("ERROR: twai_driver_install failed");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("ERROR: twai_start failed");
    return;
  }

  driver_installed = true;
  Serial.println("TWAI driver installed + started");
  Serial.printf("Mode: %s\n",
                (MODE == TWAI_MODE_NORMAL) ? "NORMAL" :
                (MODE == TWAI_MODE_NO_ACK) ? "NO_ACK" : "LISTEN_ONLY");
}

void loop() {
  if (!driver_installed) {
    delay(1000);
    return;
  }

  // --- Transmit a beacon once per second (like your Teensy sketch) ---
  uint32_t now = millis();
  if (MODE != TWAI_MODE_LISTEN_ONLY && (now - lastTxMs >= 1000)) {
    twai_message_t tx_msg = {};
    tx_msg.identifier = 0x123;
    tx_msg.extd = 0;                 // standard 11-bit ID
    tx_msg.rtr  = 0;                 // data frame
    tx_msg.data_length_code = 8;
    for (int i = 0; i < 8; i++) tx_msg.data[i] = counter + i;
    counter++;

    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
    if (err == ESP_OK) {
      Serial.println("TX: sent 0x123");
    } else {
      Serial.printf("TX failed (err=%d)\n", (int)err);
      // If you see TX failures on a bench with only one node, switch MODE to TWAI_MODE_NO_ACK.
      // No-Ack mode exists specifically for that use case.  [oai_citation:3‡Espressif Systems](https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/twai.html?utm_source=chatgpt.com)
    }

    lastTxMs = now;
  }

  // --- Poll receive ---
  twai_message_t rx_msg;
  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    printFrame(rx_msg);
  }
}
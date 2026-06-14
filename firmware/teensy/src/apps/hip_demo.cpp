#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// ----------------------------
// ODrive Command IDs
// ----------------------------
#define CMD_SET_AXIS_STATE      0x07
#define CMD_SET_CONTROLLER_MODE 0x0B
#define CMD_SET_INPUT_TORQUE    0x0E

#define AXIS_STATE_CLOSED_LOOP  8
#define CONTROL_MODE_TORQUE     1
#define INPUT_MODE_PASSTHROUGH  1

// ----------------------------
// Node IDs
// ----------------------------
#define NODE_LEFT   1
#define NODE_RIGHT  2

// ----------------------------
// Helper: Build ODrive CAN ID
// ----------------------------
uint16_t make_can_id(uint8_t node, uint8_t cmd) {
  return (node << 5) | cmd;
}

// ----------------------------
// Send Axis State Command
// ----------------------------
void set_axis_state(uint8_t node, uint32_t state) {
  CAN_message_t msg;
  msg.id = make_can_id(node, CMD_SET_AXIS_STATE);
  msg.len = 8;

  memset(msg.buf, 0, 8);
  memcpy(msg.buf, &state, 4);

  can1.write(msg);
}

// ----------------------------
// Set Controller Mode
// ----------------------------
void set_controller_mode(uint8_t node) {
  CAN_message_t msg;
  msg.id = make_can_id(node, CMD_SET_CONTROLLER_MODE);
  msg.len = 8;

  uint32_t control_mode = CONTROL_MODE_TORQUE;
  uint32_t input_mode   = INPUT_MODE_PASSTHROUGH;

  memcpy(msg.buf, &control_mode, 4);
  memcpy(msg.buf + 4, &input_mode, 4);

  can1.write(msg);
}

// ----------------------------
// Send Torque Command
// ----------------------------
void send_torque(uint8_t node, float torque) {
  CAN_message_t msg;
  msg.id = make_can_id(node, CMD_SET_INPUT_TORQUE);
  msg.len = 8;

  memset(msg.buf, 0, 8);
  memcpy(msg.buf, &torque, 4);

  can1.write(msg);
}

// ----------------------------
// Setup
// ----------------------------
void setup() {

  can1.begin();
  can1.setBaudRate(500000);

  delay(1000);

  // Put both motors into CLOSED_LOOP_CONTROL
  set_axis_state(NODE_LEFT,  AXIS_STATE_CLOSED_LOOP);
  set_axis_state(NODE_RIGHT, AXIS_STATE_CLOSED_LOOP);

  delay(100);

  // Set both to TORQUE mode
  set_controller_mode(NODE_LEFT);
  set_controller_mode(NODE_RIGHT);

  delay(100);
}

// ----------------------------
// 500 Hz Control Loop
// ----------------------------
void loop() {

  static uint32_t last_tick = 0;
  const uint32_t period_us = 2000; // 500 Hz

  if (micros() - last_tick >= period_us) {

    last_tick += period_us;

    
    // WARNING: LOTS OF TORQUE BEING GENERATED, PLEASE RUN DEMOS WITH CAUTION, HOLD THE MOTOR BEFORE RUNNING
    
    // sine wave
    float t = millis() * 0.001f;
    float torque = 0.05f * sinf(2.0f * PI * 0.5f * t);


    // Send identical torque to both hips
    send_torque(NODE_LEFT,  torque);
    send_torque(NODE_RIGHT, torque);
  }
}
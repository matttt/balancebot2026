// Balance experiment — Teensy 4.1 + BNO055 + ODrive (CAN).
// Reads IMU pitch and commands both hip ODrives in position mode toward
// upright. Streams telemetry for the host dashboard.
//
// WARNING: drives real motors. Hold the robot/motors before running.
//
// Commands:  z = zero orientation   h = help

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "imu.h"
#include "telemetry.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// ----------------------------
// ODrive Command IDs
// ----------------------------
#define CMD_SET_AXIS_STATE      0x07
#define CMD_SET_CONTROLLER_MODE 0x0B
#define CMD_SET_INPUT_TORQUE    0x0E
#define CMD_SET_INPUT_POSITION  0x0C

#define AXIS_STATE_CLOSED_LOOP  8
#define CONTROL_MODE_TORQUE     1
#define CONTROL_MODE_POSITION   3
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

Imu imu;
Telemetry tel;
float pitch_zero = 0, roll_zero = 0, yaw_zero = 0;

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

  uint32_t control_mode = CONTROL_MODE_POSITION;
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
// Send Position Command
// ----------------------------
void send_position_cmd(uint8_t node, float position) {
  CAN_message_t msg;
  msg.id = make_can_id(node, CMD_SET_INPUT_POSITION);
  msg.len = 8;

  memset(msg.buf, 0, 8);
  memcpy(msg.buf, &position, 4);

  can1.write(msg);
}


// ----------------------------
// Setup
// ----------------------------
void setup() {

  can1.begin();
  can1.setBaudRate(500000);

  delay(1000);


  Serial.println("[balance_demo] Initializing BNO055...");

    Wire.begin();
    Wire.setClock(400000);

    // I2C scan
    Serial.print("[balance_demo] I2C scan: ");
    bool found = false;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("0x%02X ", addr);
            found = true;
        }
    }
    if (!found) Serial.print("No devices found");
    Serial.println();

    if (imu.begin()) {
        Serial.println("[balance_demo] Send 'z' to zero, 'h' for help");
    } else {
        Serial.println("[balance_demo] BNO055 FAILED");
        while (true) { digitalToggle(LED_BUILTIN); delay(200); }
    }

    delay(100);


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
    // float torque = 0.05f * sinf(2.0f * PI * 0.5f * t);

    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'z') {
            imu.update();
            pitch_zero = imu.pitch();
            roll_zero  = imu.roll();
            yaw_zero   = imu.yaw();
            Serial.printf("ZERO,%.4f,%.4f,%.4f\n", pitch_zero, roll_zero, yaw_zero);
        } else if (c == 'h') {
            Serial.println("[balance_demo] Send 'z' to zero, 'h' for help");
        }
    }

    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < 20) return;
    last_ms = now;

    imu.update();
    uint8_t cal = imu.calib_status();

    float p = (imu.pitch() - pitch_zero) * 57.2958f;
    float r = (imu.roll()  - roll_zero)  * 57.2958f;
    float y = (imu.yaw()   - yaw_zero)   * 57.2958f;

    float torque = 0.0008 * p;
    /*
    if (p > 0.0) {
      torque = 0.05f;
    } else {
      torque = -0.05f;
    }
    */

    // Send identical torque to both hips
    //send_torque(NODE_LEFT,  torque);
    //send_torque(NODE_RIGHT, torque);

    // float position;
    // if (p > 0.0) {
    //   position = 0.5f;
    // } else {
    //   position = -0.5f;
    // }

    float position = -p / 30.0;

    send_position_cmd(NODE_LEFT,  position);
    send_position_cmd(NODE_RIGHT,  position);

    tel.put("euler.pitch", p);
    tel.put("euler.roll",  r);
    tel.put("euler.yaw",   y);
    tel.put("cmd.position", position);
    tel.put("cal.sys",   (cal >> 6) & 3);
    tel.put("cal.gyro",  (cal >> 4) & 3);
    tel.put("cal.accel", (cal >> 2) & 3);
    tel.put("cal.mag",    cal       & 3);
    tel.send();
  }
}

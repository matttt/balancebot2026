#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "RobStrideMIT.h"

// Teensy 4.1 CAN1 pins are CRX1=22 and CTX1=23. Use a 3.3 V CAN transceiver,
// common ground, and 120 ohm termination at the two physical ends of the bus.

#ifndef ROBSTRIDE_MOTOR_ID
#define ROBSTRIDE_MOTOR_ID 55
#endif

#ifndef ROBSTRIDE_HOST_ID
#define ROBSTRIDE_HOST_ID 0
#endif

static constexpr uint32_t CAN_BAUD = 1000000;
static constexpr uint32_t MIT_PERIOD_US = 5000;  // 200 Hz

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
robstride::RobStrideMIT<decltype(can1)> motor(can1, ROBSTRIDE_MOTOR_ID, ROBSTRIDE_HOST_ID);

static uint8_t motor_id = ROBSTRIDE_MOTOR_ID;
static uint8_t host_id = ROBSTRIDE_HOST_ID;
static bool mit_streaming = false;
static float target_position = 0.0f;
static float target_velocity = 0.0f;
static float target_torque = 0.0f;
static float kp = 30.0f;
static float kd = 0.5f;
static char command_line[96];
static size_t command_len = 0;

static bool send_raw(uint32_t id, const uint8_t* payload = nullptr) {
  CAN_message_t tx = {};
  tx.id = id;
  tx.flags.extended = 1;
  tx.len = 8;
  if (payload) {
    memcpy(tx.buf, payload, 8);
  }
  return can1.write(tx) == 1;
}

static void print_help() {
  Serial.println();
  Serial.println("RobStride Teensy demo commands:");
  Serial.println("  help                 print this menu");
  Serial.println("  scan                 request device IDs 0..127");
  Serial.println("  id <n>               set target motor CAN ID");
  Serial.println("  host <n>             set host CAN ID");
  Serial.println("  en                   enable motor and select MIT mode");
  Serial.println("  hold [rad] [kp] [kd] stream MIT hold command at 200 Hz");
  Serial.println("  jog <rad>            add a small position offset to hold target");
  Serial.println("  speed <rad_s>        switch to speed mode and command velocity");
  Serial.println("  pos <rad> [max_spd]  switch to position mode and command position");
  Serial.println("  limits <Nm> <rad_s>  set torque and speed limits");
  Serial.println("  zero                 set current mechanical position as zero");
  Serial.println("  stop                 stop streaming and reset/disable motor");
  Serial.println("  status               print last decoded feedback");
  Serial.println();
}

static void print_status() {
  const robstride::MotorState& s = motor.state();
  Serial.print("[STATE] id=");
  Serial.print(motor_id);
  Serial.print(" host=");
  Serial.print(host_id);
  Serial.print(" streaming=");
  Serial.print(mit_streaming ? "yes" : "no");
  Serial.print(" pos=");
  Serial.print(s.position, 4);
  Serial.print(" vel=");
  Serial.print(s.velocity, 4);
  Serial.print(" tq=");
  Serial.print(s.torque, 4);
  Serial.print(" temp=");
  Serial.print(s.temperature_c, 1);
  Serial.print(" mode=");
  Serial.print(s.mode_status);
  Serial.print(" faults=0x");
  Serial.print(s.fault_bits, HEX);
  Serial.print(" age_ms=");
  Serial.println(s.last_rx_ms == 0 ? 999999UL : millis() - s.last_rx_ms);
}

static bool next_u8(char*& token, uint8_t& value) {
  token = strtok(nullptr, " ");
  if (!token) {
    return false;
  }
  const int parsed = atoi(token);
  if (parsed < 0 || parsed > 255) {
    return false;
  }
  value = static_cast<uint8_t>(parsed);
  return true;
}

static bool next_float(char*& token, float& value) {
  token = strtok(nullptr, " ");
  if (!token) {
    return false;
  }
  value = atof(token);
  return true;
}

static void scan_for_motors() {
  Serial.println("[SCAN] requesting RobStride device IDs 0..127");

  uint8_t found = 0;
  for (uint8_t id = 0; id < 128; ++id) {
    const uint32_t request_id =
        robstride::make_extended_id(robstride::CommType::GET_DEVICE_ID, host_id, id);
    send_raw(request_id);

    const uint32_t deadline = millis() + 8;
    while ((int32_t)(millis() - deadline) < 0) {
      CAN_message_t rx;
      while (can1.read(rx)) {
        if (!rx.flags.extended) {
          continue;
        }

        const uint8_t type = (rx.id >> 24) & 0x1F;
        const uint16_t data_area = (rx.id >> 8) & 0xFFFF;
        const uint8_t target = rx.id & 0xFF;
        Serial.print("[RX] ext_id=0x");
        Serial.print(rx.id, HEX);
        Serial.print(" type=");
        Serial.print(type);
        Serial.print(" data=0x");
        Serial.print(data_area, HEX);
        Serial.print(" target=0x");
        Serial.print(target, HEX);
        if (type == static_cast<uint8_t>(robstride::CommType::GET_DEVICE_ID)) {
          Serial.print(" candidate_motor_id=");
          Serial.print(data_area & 0xFF);
          ++found;
        }
        Serial.println();
      }
    }
  }

  Serial.print("[SCAN] done, replies=");
  Serial.println(found);
}

static void configure_ids(uint8_t new_motor_id, uint8_t new_host_id) {
  motor_id = new_motor_id;
  host_id = new_host_id;
  motor.set_motor_id(motor_id);
  motor.set_host_id(host_id);
  Serial.print("[CONFIG] motor_id=");
  Serial.print(motor_id);
  Serial.print(" host_id=");
  Serial.println(host_id);
}

static void enable_mit_mode() {
  mit_streaming = false;
  motor.set_mode(robstride::ControlMode::MIT);
  delay(10);
  motor.enable();
  Serial.println("[MOTOR] enabled, MIT mode selected");
}

static void stop_motor() {
  mit_streaming = false;
  motor.write_operation(target_position, 0.0f, 0.0f, 0.0f, 0.0f);
  delay(5);
  motor.reset();
  Serial.println("[MOTOR] streaming stopped, reset/disable sent");
}

static void handle_command(char* line) {
  for (char* p = line; *p; ++p) {
    *p = static_cast<char>(tolower(*p));
  }

  char* token = strtok(line, " ");
  if (!token) {
    return;
  }

  if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0) {
    print_help();
  } else if (strcmp(token, "scan") == 0) {
    scan_for_motors();
  } else if (strcmp(token, "id") == 0) {
    uint8_t new_id = motor_id;
    if (next_u8(token, new_id)) {
      configure_ids(new_id, host_id);
    } else {
      Serial.println("[ERR] usage: id <0..255>");
    }
  } else if (strcmp(token, "host") == 0) {
    uint8_t new_host = host_id;
    if (next_u8(token, new_host)) {
      configure_ids(motor_id, new_host);
    } else {
      Serial.println("[ERR] usage: host <0..255>");
    }
  } else if (strcmp(token, "en") == 0 || strcmp(token, "enable") == 0) {
    enable_mit_mode();
  } else if (strcmp(token, "hold") == 0) {
    float parsed = target_position;
    if (next_float(token, parsed)) {
      target_position = parsed;
    }
    if (next_float(token, parsed)) {
      kp = parsed;
    }
    if (next_float(token, parsed)) {
      kd = parsed;
    }
    motor.set_mode(robstride::ControlMode::MIT);
    delay(5);
    motor.enable();
    mit_streaming = true;
    Serial.print("[MIT] hold pos=");
    Serial.print(target_position, 4);
    Serial.print(" kp=");
    Serial.print(kp, 3);
    Serial.print(" kd=");
    Serial.println(kd, 3);
  } else if (strcmp(token, "jog") == 0) {
    float delta = 0.0f;
    if (next_float(token, delta)) {
      target_position += delta;
      mit_streaming = true;
      Serial.print("[MIT] target pos=");
      Serial.println(target_position, 4);
    } else {
      Serial.println("[ERR] usage: jog <rad>");
    }
  } else if (strcmp(token, "speed") == 0) {
    float speed = 0.0f;
    if (next_float(token, speed)) {
      mit_streaming = false;
      motor.set_mode(robstride::ControlMode::SPEED);
      delay(5);
      motor.enable();
      motor.set_speed_target(speed);
      Serial.print("[SPEED] target=");
      Serial.println(speed, 4);
    } else {
      Serial.println("[ERR] usage: speed <rad_s>");
    }
  } else if (strcmp(token, "pos") == 0) {
    float position = 0.0f;
    float max_speed = 2.0f;
    if (next_float(token, position)) {
      next_float(token, max_speed);
      mit_streaming = false;
      motor.set_mode(robstride::ControlMode::POSITION);
      delay(5);
      motor.enable();
      motor.set_position_target(position, max_speed);
      Serial.print("[POSITION] target=");
      Serial.print(position, 4);
      Serial.print(" max_speed=");
      Serial.println(max_speed, 4);
    } else {
      Serial.println("[ERR] usage: pos <rad> [max_spd]");
    }
  } else if (strcmp(token, "limits") == 0) {
    float torque_limit = 6.0f;
    float speed_limit = 10.0f;
    if (next_float(token, torque_limit) && next_float(token, speed_limit)) {
      motor.write_param_float(robstride::ParamID::LIMIT_TORQUE, torque_limit);
      motor.write_param_float(robstride::ParamID::LIMIT_SPD, speed_limit);
      Serial.print("[LIMITS] torque=");
      Serial.print(torque_limit, 3);
      Serial.print(" speed=");
      Serial.println(speed_limit, 3);
    } else {
      Serial.println("[ERR] usage: limits <Nm> <rad_s>");
    }
  } else if (strcmp(token, "zero") == 0) {
    mit_streaming = false;
    motor.set_zero();
    Serial.println("[MOTOR] set-zero command sent");
  } else if (strcmp(token, "stop") == 0 || strcmp(token, "reset") == 0 || strcmp(token, "dis") == 0) {
    stop_motor();
  } else if (strcmp(token, "status") == 0) {
    print_status();
  } else {
    Serial.println("[ERR] unknown command, type help");
  }
}

static void read_serial_commands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      command_line[command_len] = '\0';
      handle_command(command_line);
      command_len = 0;
    } else if (command_len < sizeof(command_line) - 1) {
      command_line[command_len++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("[BOOT] RobStride Teensy 4.1 demo");

  can1.begin();
  can1.setBaudRate(CAN_BAUD);
  can1.setMaxMB(16);
  can1.enableFIFO();

  configure_ids(motor_id, host_id);
  Serial.println("[CAN] CAN1 @ 1 Mbps");
  print_help();
}

void loop() {
  static uint32_t last_mit_us = 0;
  static uint32_t last_status_ms = 0;

  read_serial_commands();
  motor.poll_rx();

  if (mit_streaming && (uint32_t)(micros() - last_mit_us) >= MIT_PERIOD_US) {
    last_mit_us += MIT_PERIOD_US;
    motor.write_operation(target_position, target_velocity, kp, kd, target_torque);
  }

  if ((uint32_t)(millis() - last_status_ms) >= 500) {
    last_status_ms = millis();
    // print_status();
  }
}

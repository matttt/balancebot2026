#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

namespace robstride {

// RobStride / Xiaomi CyberGear protocol ranges used by the Arduino example in
// tianrking/RobStride_Control. Adjust these if your motor model uses wider
// limits and you have verified them in the vendor manual.
static constexpr float P_MIN = -12.5f;
static constexpr float P_MAX = 12.5f;
static constexpr float V_MIN = -30.0f;
static constexpr float V_MAX = 30.0f;
static constexpr float KP_MIN = 0.0f;
static constexpr float KP_MAX = 500.0f;
static constexpr float KD_MIN = 0.0f;
static constexpr float KD_MAX = 5.0f;
static constexpr float T_MIN = -12.0f;
static constexpr float T_MAX = 12.0f;

enum class CommType : uint8_t {
  GET_DEVICE_ID = 0,
  OPERATION = 1,
  FEEDBACK = 2,
  ENABLE = 3,
  RESET = 4,
  SET_ZERO = 6,
  WRITE_PARAM = 18,
};

enum class ControlMode : uint8_t {
  MIT = 0,
  POSITION = 1,
  SPEED = 2,
  CURRENT = 3,
};

enum class ParamID : uint16_t {
  RUN_MODE = 0x7005,
  IQ_REF = 0x7006,
  SPD_REF = 0x700A,
  LIMIT_TORQUE = 0x700B,
  LOC_REF = 0x7016,
  LIMIT_SPD = 0x7017,
  LIMIT_CUR = 0x7018,
  LOC_KP = 0x701E,
  SPD_KP = 0x701F,
  SPD_KI = 0x7020,
};

struct MotorState {
  uint8_t master_id = 0;
  uint8_t motor_id = 0;
  uint8_t mode_status = 0;
  uint8_t fault_bits = 0;
  bool hall_fault = false;
  bool magnet_fault = false;
  bool temperature_fault = false;
  bool current_fault = false;
  bool voltage_fault = false;
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  float temperature_c = 0.0f;
  uint32_t last_rx_ms = 0;
};

inline uint16_t float_to_uint(float x, float x_min, float x_max) {
  if (x > x_max) {
    x = x_max;
  } else if (x < x_min) {
    x = x_min;
  }

  const float span = x_max - x_min;
  return static_cast<uint16_t>((x - x_min) * 65535.0f / span);
}

inline float uint_to_float(uint16_t x, float x_min, float x_max) {
  return static_cast<float>(x) * (x_max - x_min) / 65535.0f + x_min;
}

inline uint32_t make_extended_id(CommType type, uint16_t data_area, uint8_t target_id) {
  return (static_cast<uint32_t>(type) << 24) |
         (static_cast<uint32_t>(data_area) << 8) |
         target_id;
}

template <typename CanT>
class RobStrideMIT {
 public:
  RobStrideMIT(CanT& can, uint8_t motor_id, uint8_t host_id = 0)
      : can_(can), motor_id_(motor_id), host_id_(host_id) {}

  void set_motor_id(uint8_t motor_id) { motor_id_ = motor_id; }
  void set_host_id(uint8_t host_id) { host_id_ = host_id; }
  uint8_t motor_id() const { return motor_id_; }
  const MotorState& state() const { return state_; }

  bool request_device_id() {
    return send_frame(CommType::GET_DEVICE_ID, host_id_, nullptr);
  }

  bool enable() {
    return send_frame(CommType::ENABLE, host_id_, nullptr);
  }

  bool reset() {
    return send_frame(CommType::RESET, host_id_, nullptr);
  }

  bool set_zero() {
    return send_frame(CommType::SET_ZERO, host_id_, nullptr);
  }

  bool set_mode(ControlMode mode) {
    uint8_t payload[8] = {};
    const uint16_t param = static_cast<uint16_t>(ParamID::RUN_MODE);
    memcpy(&payload[0], &param, sizeof(param));
    payload[4] = static_cast<uint8_t>(mode);
    return send_frame(CommType::WRITE_PARAM, host_id_, payload);
  }

  bool write_param_float(ParamID param, float value) {
    uint8_t payload[8] = {};
    const uint16_t param_id = static_cast<uint16_t>(param);
    memcpy(&payload[0], &param_id, sizeof(param_id));
    memcpy(&payload[4], &value, sizeof(value));
    return send_frame(CommType::WRITE_PARAM, host_id_, payload);
  }

  bool write_operation(float position, float velocity, float kp, float kd, float torque = 0.0f) {
    uint8_t payload[8] = {};
    const uint16_t p = float_to_uint(position, P_MIN, P_MAX);
    const uint16_t v = float_to_uint(velocity, V_MIN, V_MAX);
    const uint16_t k_p = float_to_uint(kp, KP_MIN, KP_MAX);
    const uint16_t k_d = float_to_uint(kd, KD_MIN, KD_MAX);
    payload[0] = static_cast<uint8_t>(p >> 8);
    payload[1] = static_cast<uint8_t>(p);
    payload[2] = static_cast<uint8_t>(v >> 8);
    payload[3] = static_cast<uint8_t>(v);
    payload[4] = static_cast<uint8_t>(k_p >> 8);
    payload[5] = static_cast<uint8_t>(k_p);
    payload[6] = static_cast<uint8_t>(k_d >> 8);
    payload[7] = static_cast<uint8_t>(k_d);

    const uint16_t torque_area = float_to_uint(torque, T_MIN, T_MAX);
    return send_frame(CommType::OPERATION, torque_area, payload);
  }

  bool set_position_target(float position, float max_speed) {
    const bool speed_ok = write_param_float(ParamID::LIMIT_SPD, max_speed);
    const bool pos_ok = write_param_float(ParamID::LOC_REF, position);
    return speed_ok && pos_ok;
  }

  bool set_speed_target(float speed) {
    return write_param_float(ParamID::SPD_REF, speed);
  }

  bool poll_rx() {
    bool decoded = false;
    CAN_message_t rx;
    while (can_.read(rx)) {
      decoded = decode_feedback(rx) || decoded;
    }
    return decoded;
  }

 private:
  bool send_frame(CommType type, uint16_t data_area, const uint8_t* payload) {
    CAN_message_t tx = {};
    tx.id = make_extended_id(type, data_area, motor_id_);
    tx.flags.extended = 1;
    tx.len = 8;
    if (payload) {
      memcpy(tx.buf, payload, 8);
    }
    return can_.write(tx) == 1;
  }

  bool decode_feedback(const CAN_message_t& rx) {
    if (!rx.flags.extended || rx.len < 8) {
      return false;
    }

    const uint8_t host_id = rx.id & 0xFF;
    const uint8_t motor_id = (rx.id >> 8) & 0xFF;
    if (host_id != host_id_ || motor_id != motor_id_) {
      return false;
    }

    state_.master_id = host_id;
    state_.motor_id = motor_id;
    state_.fault_bits = (rx.id >> 16) & 0x3F;
    state_.voltage_fault = ((rx.id >> 16) & 0x01) != 0;
    state_.current_fault = ((rx.id >> 17) & 0x01) != 0;
    state_.temperature_fault = ((rx.id >> 18) & 0x01) != 0;
    state_.magnet_fault = ((rx.id >> 19) & 0x01) != 0;
    state_.hall_fault = ((rx.id >> 20) & 0x01) != 0;
    state_.mode_status = (rx.id >> 22) & 0x03;

    const uint16_t raw_position = (static_cast<uint16_t>(rx.buf[0]) << 8) | rx.buf[1];
    const uint16_t raw_velocity = (static_cast<uint16_t>(rx.buf[2]) << 8) | rx.buf[3];
    const uint16_t raw_torque = (static_cast<uint16_t>(rx.buf[4]) << 8) | rx.buf[5];
    const uint16_t raw_temp = (static_cast<uint16_t>(rx.buf[6]) << 8) | rx.buf[7];

    state_.position = uint_to_float(raw_position, -4.0f * PI, 4.0f * PI);
    state_.velocity = uint_to_float(raw_velocity, V_MIN, V_MAX);
    state_.torque = uint_to_float(raw_torque, T_MIN, T_MAX);
    state_.temperature_c = static_cast<float>(raw_temp) / 10.0f;
    state_.last_rx_ms = millis();
    return true;
  }

  CanT& can_;
  uint8_t motor_id_;
  uint8_t host_id_;
  MotorState state_;
};

}  // namespace robstride

#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>
#include "config.h"

namespace sensors {

// Measurement validation
struct ValidationConfig {
    float min_value       = -100.0f;
    float max_value       =  100.0f;
    float max_rate        =  50.0f;
    float stuck_tolerance =  1e-6f;
    uint16_t stuck_limit  =  50;
};

// Channel validity configs
namespace cfgs {
    // Pitch/roll/yaw: 180 deg, 30 rad/s max rate at 100 Hz
    inline constexpr ValidationConfig angle{
        .min_value = -3.2f, .max_value = 3.2f,
        .max_rate = 0.3f, .stuck_tolerance = 1e-5f,
        .stuck_limit = 100
    };
    // BNO055 gyro: 35 rad/s max, 500 rad/s^2 peak change
    inline constexpr ValidationConfig gyro{
        .min_value = -35.0f, .max_value = 35.0f,
        .max_rate = 5.0f, .stuck_tolerance = 1e-5f,
        .stuck_limit = 200
    };
    // BNO055 linear accel: 160 m/s^2 max range
    inline constexpr ValidationConfig accel{
        .min_value = -160.0f, .max_value = 160.0f,
        .max_rate = 30.0f, .stuck_tolerance = 1e-4f,
        .stuck_limit = 200
    };
    inline constexpr ValidationConfig enc_pos{
        .min_value = -200.0f, .max_value = 200.0f,
        .max_rate = 5.0f, .stuck_tolerance = 1e-6f,
        .stuck_limit = 500
    };
    inline constexpr ValidationConfig enc_vel{
        .min_value = -100.0f, .max_value = 100.0f,
        .max_rate = 20.0f, .stuck_tolerance = 1e-5f,
        .stuck_limit = 500
    };
}

// Sensor channel
class SensorChannel {
public:
    explicit SensorChannel(const ValidationConfig& cfg = {}) : cfg_(cfg) {}

    struct Result {
        float value    = 0.0f;
        bool  valid    = false;
        bool  stuck    = false;
        bool  spike    = false;
        bool  range_ok = true;
    };

    Result update(float raw) {
        Result r;
        r.value = raw;

        // NaN / Inf rejection
        if (!std::isfinite(raw)) {
            r.valid = false;
            nan_count_++;
            return r;
        }

        if (raw < cfg_.min_value || raw > cfg_.max_value) {
            r.valid = false;
            r.range_ok = false;
            range_fault_count_++;
            return r;
        }

        // Rate-of-change check
        if (has_prev_) {
            float rate = std::abs(raw - prev_raw_);
            if (rate > cfg_.max_rate) {
                r.spike = true;
                r.valid = false;
                spike_count_++;
                return r;
            }
        }

        // Stuck sensor detection
        if (has_prev_ && std::abs(raw - prev_raw_) < cfg_.stuck_tolerance) {
            stuck_counter_++;
        } else {
            stuck_counter_ = 0;
        }
        r.stuck = (stuck_counter_ >= cfg_.stuck_limit);
        r.valid = true;
        prev_raw_ = raw;
        has_prev_ = true;
        last_good_ = raw;
        return r;
    }

    float last_good()     const { return last_good_; }
    uint32_t nan_count()  const { return nan_count_; }
    uint32_t spike_count() const { return spike_count_; }
    uint32_t range_faults() const { return range_fault_count_; }
    bool is_stuck()       const { return stuck_counter_ >= cfg_.stuck_limit; }

    void reset() {
        has_prev_ = false;
        stuck_counter_ = 0;
        nan_count_ = 0;
        spike_count_ = 0;
        range_fault_count_ = 0;
    }

private:
    ValidationConfig cfg_;
    float prev_raw_  = 0;
    float last_good_ = 0;
    bool  has_prev_  = false;
    uint16_t stuck_counter_ = 0;
    uint32_t nan_count_     = 0;
    uint32_t spike_count_   = 0;
    uint32_t range_fault_count_ = 0;
};

// Low-pass filter
class LowPass {
public:
    constexpr LowPass(float cutoff_hz, float dt)
        : alpha_(dt / (dt + 1.0f / (2.0f * 3.14159f * cutoff_hz))) {}

    float update(float x) {
        y_ = alpha_ * x + (1.0f - alpha_) * y_;
        return y_;
    }

    float value() const { return y_; }
    void reset(float v = 0) { y_ = v; }

private:
    float alpha_;
    float y_ = 0;
};

// IMU Preprocessor
class ImuPreprocessor {
public:
    struct ProcessedImu {
        float pitch, roll, yaw;
        float roll_rate, pitch_rate, yaw_rate;
        float accel_x, accel_y;
        bool  valid;
        uint8_t fault_mask;
    };

    ProcessedImu process(float pitch, float roll, float yaw,
                         float roll_rate, float pitch_rate, float yaw_rate,
                         float accel_x, float accel_y) {
        ProcessedImu out{};
        uint8_t faults = 0;

        auto handle = [&](SensorChannel& ch, float raw, int bit) -> float {
            auto r = ch.update(raw);
            if (!r.valid) {
                faults |= (1 << bit);
                return ch.last_good();  // hold last good value
            }
            return r.value;
        };

        out.pitch      = handle(ch_pitch_,     pitch,      0);
        out.roll       = handle(ch_roll_,      roll,       1);
        out.yaw        = handle(ch_yaw_,       yaw,        2);
        out.roll_rate  = lp_roll_rate_.update( handle(ch_roll_rate_,  roll_rate,  3));
        out.pitch_rate = lp_pitch_rate_.update(handle(ch_pitch_rate_, pitch_rate, 4));
        out.yaw_rate   = lp_yaw_rate_.update(  handle(ch_yaw_rate_,   yaw_rate,   5));
        out.accel_x    = lp_accel_x_.update(   handle(ch_accel_x_,    accel_x,    6));
        out.accel_y    = lp_accel_y_.update(   handle(ch_accel_y_,    accel_y,    7));

        out.fault_mask = faults;
        // pitch (bit 0) and pitch_rate (bit 4) must be valid
        out.valid = !(faults & 0x01) && !(faults & 0x10);
        return out;
    }

    bool any_stuck() const {
        return ch_pitch_.is_stuck() || ch_roll_.is_stuck() ||
               ch_pitch_rate_.is_stuck() || ch_roll_rate_.is_stuck();
    }

private:
    static constexpr float dt_ = 1.0f / cfg::control_rate_hz;

    SensorChannel ch_pitch_{cfgs::angle}, ch_roll_{cfgs::angle}, ch_yaw_{cfgs::angle};
    SensorChannel ch_roll_rate_{cfgs::gyro}, ch_pitch_rate_{cfgs::gyro}, ch_yaw_rate_{cfgs::gyro};
    SensorChannel ch_accel_x_{cfgs::accel}, ch_accel_y_{cfgs::accel};

    LowPass lp_roll_rate_{30.0f, dt_}, lp_pitch_rate_{30.0f, dt_}, lp_yaw_rate_{30.0f, dt_};
    LowPass lp_accel_x_{10.0f, dt_}, lp_accel_y_{10.0f, dt_};
};

// Motor Encoder Preprocessor
class EncoderPreprocessor {
public:
    struct ProcessedEncoder {
        float position;
        float velocity;
        bool valid;
    };

    ProcessedEncoder process(float pos, float vel) {
        ProcessedEncoder out{};
        auto rp = ch_pos_.update(pos);
        auto rv = ch_vel_.update(vel);
        out.position = rp.valid ? rp.value : ch_pos_.last_good();
        out.velocity = rv.valid ? rv.value : ch_vel_.last_good();
        out.valid = rp.valid && rv.valid;
        return out;
    }

    bool is_stuck() const { return ch_pos_.is_stuck(); }

private:
    SensorChannel ch_pos_{cfgs::enc_pos};
    SensorChannel ch_vel_{cfgs::enc_vel};
};

}

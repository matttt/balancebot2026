#pragma once
#include <Wire.h>
#include "config.h"

class Imu {
public:
    bool begin(TwoWire& wire = Wire) {
        wire_ = &wire;
        wire_->begin();
        wire_->setClock(400'000);

        // Reset        
        if (read8(bno::CHIP_ID_ADDR) != bno::CHIP_ID_VALUE) {
            delay(1000);
            if (read8(bno::CHIP_ID_ADDR) != bno::CHIP_ID_VALUE) {
                return false;
            }
        }

        write8(bno::SYS_TRIGGER_ADDR, 0x20);
        delay(1000);

        while (read8(bno::CHIP_ID_ADDR) != bno::CHIP_ID_VALUE) {
            delay(10);
        }
        delay(50);

        // Power mode
        write8(bno::PWR_MODE_ADDR, 0x00);
        delay(10);

        // Units (SI)
        write8(bno::UNIT_SEL_ADDR, 0x06);
        delay(10);

        // IMU fusion mode
        write8(bno::OPR_MODE_ADDR, bno::MODE_IMU);
        delay(20);

        ok_ = true;
        return true;
    }

    void update() {
        if (!ok_) return;

        int16_t raw[9];
        read_vec3(bno::EULER_H_LSB,        &raw[0]);
        read_vec3(bno::GYRO_X_LSB,         &raw[3]);
        read_vec3(bno::LINEAR_ACCEL_X_LSB, &raw[6]);

        bool changed = false;
        for (int i = 0; i < 9; i++) {
            if (raw[i] != prev_raw_[i]) changed = true;
            prev_raw_[i] = raw[i];
        }

        // BNO055 SI scaling: 1 LSB = 1/900 rad (euler), 1/900 rad/s (gyro), 1/100 m/s^2 (linear accel)
        yaw_   = raw[0] / 900.0f;
        roll_  = raw[1] / 900.0f;
        pitch_ = raw[2] / 900.0f;
        gyro_x_ = raw[3] / 900.0f;
        gyro_y_ = raw[4] / 900.0f;
        gyro_z_ = raw[5] / 900.0f;
        lin_accel_x_ = raw[6] / 100.0f;
        lin_accel_y_ = raw[7] / 100.0f;
        lin_accel_z_ = raw[8] / 100.0f;

        uint32_t now = millis();
        last_update_ms_ = now;
        if (changed || !had_first_) last_change_ms_ = now;
        had_first_ = true;
    }

    float pitch()      const { return pitch_; }
    float roll()       const { return roll_; }
    float yaw()        const { return yaw_; }

    // Depends on IMU mounting in chassis body frame
    float pitch_rate() const { return gyro_x_; }
    float roll_rate()  const { return gyro_y_; }
    float yaw_rate()   const { return gyro_z_; }

    float accel_x()    const { return lin_accel_x_; }
    float accel_y()    const { return lin_accel_y_; }
    float accel_z()    const { return lin_accel_z_; }

    bool ok() const { return ok_; }

    uint32_t ms_since_update() const { return millis() - last_update_ms_; }
    uint32_t ms_since_change() const { return millis() - last_change_ms_; }
    bool     is_stale() const { return ms_since_change() > cfg::imu_stale_ms; }

private:
    TwoWire* wire_ = nullptr;
    bool ok_ = false;

    float pitch_ = 0, roll_ = 0, yaw_ = 0;
    float gyro_x_ = 0, gyro_y_ = 0, gyro_z_ = 0;
    float lin_accel_x_ = 0, lin_accel_y_ = 0, lin_accel_z_ = 0;

    int16_t prev_raw_[9] = {0};
    uint32_t last_update_ms_ = 0;
    uint32_t last_change_ms_ = 0;
    bool     had_first_ = false;

    void write8(uint8_t reg, uint8_t val) {
        wire_->beginTransmission(bno::DEFAULT_ADDR);
        wire_->write(reg);
        wire_->write(val);
        wire_->endTransmission();
    }

    uint8_t read8(uint8_t reg) {
        wire_->beginTransmission(bno::DEFAULT_ADDR);
        wire_->write(reg);
        wire_->endTransmission(false);
        wire_->requestFrom(bno::DEFAULT_ADDR, uint8_t(1));
        return wire_->read();
    }

    void read_vec3(uint8_t reg, int16_t out[3]) {
        wire_->beginTransmission(bno::DEFAULT_ADDR);
        wire_->write(reg);
        wire_->endTransmission(false);
        wire_->requestFrom(bno::DEFAULT_ADDR, uint8_t(6));
        for (int i = 0; i < 3; i++) {
            uint8_t lo = wire_->read();
            uint8_t hi = wire_->read();
            out[i] = static_cast<int16_t>((hi << 8) | lo);
        }
    }
};

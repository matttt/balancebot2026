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

        auto raw_euler = read_vec3(bno::EULER_H_LSB);
        yaw_   = raw_euler[0] / 900.0f;
        roll_  = raw_euler[1] / 900.0f;
        pitch_ = raw_euler[2] / 900.0f;

        auto raw_gyro = read_vec3(bno::GYRO_X_LSB);
        gyro_x_ = raw_gyro[0] / 900.0f;
        gyro_y_ = raw_gyro[1] / 900.0f;
        gyro_z_ = raw_gyro[2] / 900.0f;

        auto raw_accel = read_vec3(bno::LINEAR_ACCEL_X_LSB);
        lin_accel_x_ = raw_accel[0] / 100.0f;
        lin_accel_y_ = raw_accel[1] / 100.0f;
        lin_accel_z_ = raw_accel[2] / 100.0f;
    }

    float pitch()      const { return pitch_; }
    float roll()       const { return roll_; }
    float yaw()        const { return yaw_; }
    float pitch_rate() const { return gyro_y_; }
    float roll_rate()  const { return gyro_x_; }
    float yaw_rate()   const { return gyro_z_; }
    float accel_x()    const { return lin_accel_x_; }
    float accel_y()    const { return lin_accel_y_; }
    float accel_z()    const { return lin_accel_z_; }

    bool ok() const { return ok_; }

private:
    TwoWire* wire_ = nullptr;
    bool ok_ = false;

    float pitch_ = 0, roll_ = 0, yaw_ = 0;
    float gyro_x_ = 0, gyro_y_ = 0, gyro_z_ = 0;
    float lin_accel_x_ = 0, lin_accel_y_ = 0, lin_accel_z_ = 0;

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

    int16_t* read_vec3(uint8_t reg) {
        static int16_t vals[3];
        wire_->beginTransmission(bno::DEFAULT_ADDR);
        wire_->write(reg);
        wire_->endTransmission(false);
        wire_->requestFrom(bno::DEFAULT_ADDR, uint8_t(6));
        for (int i = 0; i < 3; i++) {
            uint8_t lo = wire_->read();
            uint8_t hi = wire_->read();
            vals[i] = static_cast<int16_t>((hi << 8) | lo);
        }
        return vals;
    }
};

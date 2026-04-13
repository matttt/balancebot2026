#pragma once
#include <cstdint>

// BNO055 registers
namespace bno
{   
    inline constexpr uint8_t DEFAULT_ADDR = 0x28;
    inline constexpr uint8_t CHIP_ID_ADDR = 0x00;
    inline constexpr uint8_t CHIP_ID_VALUE = 0xA0;
    inline constexpr uint8_t OPR_MODE_ADDR = 0x3D;
    inline constexpr uint8_t PWR_MODE_ADDR = 0x3E;
    inline constexpr uint8_t SYS_TRIGGER_ADDR = 0x3F;
    inline constexpr uint8_t UNIT_SEL_ADDR = 0x3B;
    inline constexpr uint8_t EULER_H_LSB = 0x1A;
    inline constexpr uint8_t EULER_R_LSB = 0x1C;
    inline constexpr uint8_t EULER_P_LSB = 0x1E;
    inline constexpr uint8_t GYRO_X_LSB = 0x14;
    inline constexpr uint8_t LINEAR_ACCEL_X_LSB = 0x28;
    inline constexpr uint8_t MODE_CONFIG = 0x00;
    inline constexpr uint8_t MODE_NDOF = 0x0C;
    inline constexpr uint8_t MODE_IMU = 0x08;
}
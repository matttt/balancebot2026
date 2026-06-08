#pragma once
#include <cstdint>

// Parameter config
namespace cfg {
    inline constexpr uint8_t bno055_addr      = 0x28;
    inline constexpr float   control_rate_hz  = 100.0f;
    inline constexpr float   wheel_radius_m   = 0.037f;
    inline constexpr float   gravity          = 9.81f;

    // Safety thresholds
    inline constexpr float   fall_pitch_rad   = 0.6f;
    inline constexpr uint16_t fall_latch_ms   = 500;

    // Health watchdogs
    inline constexpr uint16_t imu_stale_ms    = 50;

    // Per-wheel consistency
    inline constexpr float   wheel_divergence_rad     = 0.5f;
    inline constexpr uint16_t wheel_divergence_ms     = 200;

    // IMU/wheel cross-validation
    inline constexpr float   xval_accel_threshold_mps2 = 4.0f;
    inline constexpr uint16_t xval_window_ms           = 500;
}

// BNO055 registers
namespace bno {
    inline constexpr uint8_t DEFAULT_ADDR        = 0x28;
    inline constexpr uint8_t CHIP_ID_ADDR        = 0x00;
    inline constexpr uint8_t CHIP_ID_VALUE       = 0xA0;
    inline constexpr uint8_t OPR_MODE_ADDR       = 0x3D;
    inline constexpr uint8_t PWR_MODE_ADDR       = 0x3E;
    inline constexpr uint8_t SYS_TRIGGER_ADDR    = 0x3F;
    inline constexpr uint8_t UNIT_SEL_ADDR       = 0x3B;
    inline constexpr uint8_t CALIB_STAT_ADDR     = 0x35;
    inline constexpr uint8_t EULER_H_LSB         = 0x1A;
    inline constexpr uint8_t GYRO_X_LSB          = 0x14;
    inline constexpr uint8_t LINEAR_ACCEL_X_LSB  = 0x28;
    inline constexpr uint8_t MODE_CONFIG         = 0x00;
    inline constexpr uint8_t MODE_NDOF           = 0x0C;
    inline constexpr uint8_t MODE_IMU            = 0x08;
}

#pragma once
#include "linalg.h"
#include "config.h"
#include <cmath>

// ── EKF for wheeled bipedal robot  ────
// State vector x (7 elements):
//   [0] pitch          (rad)   — from BNO055
//   [1] pitch_rate     (rad/s) — from BNO055 gyro
//   [2] yaw_rate       (rad/s) — from BNO055 gyro
//   [3] wheel_pos      (rad)   — averaged wheel encoder position
//   [4] wheel_vel      (rad/s) — averaged wheel encoder velocity
//   [5] x_pos          (m)     — forward position
//   [6] x_vel          (m/s)   — forward velocity
//
// Measurements z (5 elements):
//   [0] pitch_imu      — BNO055 fused euler pitch
//   [1] pitch_rate_imu — BNO055 gyro, pitch axis
//   [2] yaw_rate_imu   — BNO055 gyro, yaw axis
//   [3] wheel_pos_enc  — averaged wheel encoder position
//   [4] wheel_vel_enc  — averaged wheel encoder velocity

constexpr int NX = 7;
constexpr int NZ = 5;
using StateVec = linalg::Vec<NX>;
using StateMat = linalg::Mat<NX, NX>;
using MeasVec  = linalg::Vec<NZ>;
using MeasMat  = linalg::Mat<NZ, NZ>;
using HMat     = linalg::Mat<NZ, NX>;

class BipedEKF {
public:
    struct Estimate {
        float pitch;
        float pitch_rate;
        float yaw_rate;
        float wheel_pos;
        float wheel_vel;
        float x_pos;
        float x_vel;
    };

    BipedEKF() { reset(); }

    void reset() {
        x_ = StateVec::zeros();
        P_ = StateMat::identity() * 0.1f;

        Q_ = StateMat::zeros();
        Q_(0,0) = 0.1f;     // pitch: high
        Q_(1,1) = 0.5f;     // pitch_rate: high
        Q_(2,2) = 0.5f;     // yaw_rate: high
        Q_(3,3) = 0.001f;   // wheel_pos: very low
        Q_(4,4) = 0.05f;    // wheel_vel: low
        Q_(5,5) = 0.001f;   // x_pos: very low
        Q_(6,6) = 0.01f;    // x_vel: medium

        R_ = MeasMat::zeros();
        R_(0,0) = 0.001f;   // BNO055 fused pitch: very low
        R_(1,1) = 0.005f;   // BNO055 gyro pitch_rate: low
        R_(2,2) = 0.005f;   // BNO055 gyro yaw_rate: low
        R_(3,3) = 0.0005f;  // wheel encoder position: very low
        R_(4,4) = 0.02f;    // wheel encoder velocity: medium
    }

    void predict(float dt) {
        // Random-walk
        x_(0,0) += x_(1,0) * dt;
        x_(3,0) += x_(4,0) * dt;
        x_(5,0) += x_(6,0) * dt;
        x_(6,0)  = x_(4,0) * cfg::wheel_radius_m;

        StateMat Jac = StateMat::identity();
        Jac(0,1) = dt;
        Jac(3,4) = dt;
        Jac(5,6) = dt;
        Jac(6,4) = cfg::wheel_radius_m;

        P_ = Jac * P_ * Jac.transpose() + Q_;
        symmetrize_P();
    }

    bool update(const MeasVec& z, uint8_t imu_fault_mask, bool wheel_data_valid) {
        // Inflate noise for faulted channels
        MeasMat R_adaptive = R_;
        if (imu_fault_mask & 0x01) R_adaptive(0,0) *= 1e6f;  // pitch bad
        if (imu_fault_mask & 0x10) R_adaptive(1,1) *= 1e6f;  // pitch_rate bad
        if (imu_fault_mask & 0x20) R_adaptive(2,2) *= 1e6f;  // yaw_rate bad
        if (!wheel_data_valid) {
            R_adaptive(3,3) *= 1e6f;
            R_adaptive(4,4) *= 1e6f;
        }

        MeasVec y = z - H_ * x_;
        MeasMat S = H_ * P_ * H_.transpose() + R_adaptive;

        MeasMat S_inv;
        if (!linalg::invert(S, S_inv)) return false;

        linalg::Mat<NX, NZ> K = P_ * H_.transpose() * S_inv;
        x_ = x_ + K * y;

        StateMat IKH = StateMat::identity() - K * H_;
        P_ = IKH * P_ * IKH.transpose() + K * R_adaptive * K.transpose();

        symmetrize_P();
        for (int i = 0; i < NX; i++)
            P_(i,i) = std::clamp(P_(i,i), 1e-6f, 10.0f);

        return true;
    }

    Estimate estimate() const {
        return {
            .pitch      = x_(0,0),
            .pitch_rate = x_(1,0),
            .yaw_rate   = x_(2,0),
            .wheel_pos  = x_(3,0),
            .wheel_vel  = x_(4,0),
            .x_pos      = x_(5,0),
            .x_vel      = x_(6,0),
        };
    }

    float pitch_variance()      const { return P_(0,0); }
    float wheel_vel_variance()  const { return P_(4,4); }
    float x_vel_variance()      const { return P_(6,6); }

    float pitch_kalman_gain() const {
        float S = P_(0,0) + R_(0,0);
        return (S > 1e-12f) ? P_(0,0) / S : 0.0f;
    }

    void set_process_noise(int idx, float val) {
        if (idx >= 0 && idx < NX) Q_(idx, idx) = val;
    }
    void set_measurement_noise(int idx, float val) {
        if (idx >= 0 && idx < NZ) R_(idx, idx) = val;
    }

private:
    void symmetrize_P() {
        for (int r = 0; r < NX; r++)
            for (int c = r + 1; c < NX; c++) {
                float avg = 0.5f * (P_(r,c) + P_(c,r));
                P_(r,c) = avg;
                P_(c,r) = avg;
            }
    }

    StateVec x_;
    StateMat P_;
    StateMat Q_;
    MeasMat  R_;

    // Measurement matrix
    const HMat H_ = []{
        HMat m;
        m(0,0) = 1.0f; m(1,1) = 1.0f; m(2,2) = 1.0f; m(3,3) = 1.0f; m(4,4) = 1.0f;
        return m;
    }();
};

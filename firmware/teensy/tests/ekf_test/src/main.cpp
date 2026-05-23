// EKF test — Teensy 4.1 + BNO055
// Live IMU + simulated L/R wheel encoders through sensors.h preprocessors
// and BipedEKF. Streams CSV at 50 Hz for run_ekf_test.py.
//
// Commands:
//   z = zero pitch
//   1 = inject IMU pitch fault (offset)
//   2 = inject wheel fault (left wheel goes wild)
//   3 = inject wheel divergence (L/R disagree)
//   4 = inject IMU stall (freeze raw samples — simulated)
//   n = clear all faults / unlatch fall

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "ekf.h"
#include "sensors.h"

Imu imu;
BipedEKF ekf;

sensors::ImuPreprocessor imu_pre;
sensors::EncoderPreprocessor enc_L;
sensors::EncoderPreprocessor enc_R;

float pitch_zero = 0;
uint32_t tick = 0;

// Simulated per-wheel state
float sim_wpos_L = 0, sim_wpos_R = 0;
float sim_wvel_L = 0, sim_wvel_R = 0;

// Cross-validation: previous EKF wheel-velocity to compute wheel-derived accel
float prev_wheel_vel = 0;
float wheel_accel_x = 0;

// Latched/sustained faults
uint32_t fall_start_ms      = 0;
bool     fallen             = false;
uint32_t wheel_div_start_ms = 0;
bool     wheel_diverged     = false;
uint32_t xval_bad_start_ms  = 0;
bool     xval_bad           = false;

// Injected faults
bool fault_imu   = false;
bool fault_wheel = false;
bool fault_div   = false;
bool fault_stall = false;

uint8_t read_calib_status() {
    Wire.beginTransmission(cfg::bno055_addr);
    Wire.write(0x35);
    Wire.endTransmission(false);
    Wire.requestFrom(cfg::bno055_addr, uint8_t(1));
    return Wire.read();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("[ekf_test] Initializing BNO055...");
    Wire.begin();
    Wire.setClock(400000);

    Serial.print("[ekf_test] I2C scan: ");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
            Serial.printf("0x%02X ", addr);
    }
    Serial.println();

    if (imu.begin()) {
        Serial.println("[ekf_test] BNO055 OK");
    } else {
        Serial.println("[ekf_test] BNO055 FAIL — halting");
        while (true) { digitalToggle(LED_BUILTIN); delay(200); }
    }

    ekf.reset();
    delay(100);
    Serial.println("[ekf_test] Streaming CSV at 50 Hz");
    Serial.println("[ekf_test] Keys: z=zero 1=imu_fault 2=wheel_fault 3=L/R_diverge 4=imu_stall n=clear");
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if      (c == 'z') { imu.update(); pitch_zero = imu.pitch(); Serial.printf("ZERO,%.4f\n", pitch_zero); }
        else if (c == '1') { fault_imu = true;    Serial.println("FAULT,imu,on"); }
        else if (c == '2') { fault_wheel = true;  Serial.println("FAULT,wheel,on"); }
        else if (c == '3') { fault_div = true;    Serial.println("FAULT,divergence,on"); }
        else if (c == '4') { fault_stall = true;  Serial.println("FAULT,stall,on"); }
        else if (c == 'n') {
            fault_imu = fault_wheel = fault_div = fault_stall = false;
            fallen = wheel_diverged = xval_bad = false;
            fall_start_ms = wheel_div_start_ms = xval_bad_start_ms = 0;
            Serial.println("FAULT,all,off");
        }
    }

    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < 10) return;
    last_ms = now;
    tick++;

    // Simulate IMU stall by skipping the I2C read — Imu.is_stale() will trip
    if (!fault_stall) imu.update();

    const float dt = 0.01f;
    const float t  = tick * dt;

    // Simulated per-wheel motion
    float base_vel = 2.0f * sinf(t * 0.5f);
    sim_wvel_L = base_vel;
    sim_wvel_R = base_vel;
    if (fault_wheel) sim_wvel_L = 999.0f;
    if (fault_div)   sim_wvel_R = base_vel + 1.5f;  // sustained L/R mismatch
    sim_wpos_L += sim_wvel_L * dt;
    sim_wpos_R += sim_wvel_R * dt;

    // IMU preprocessing
    float raw_pitch_in = fault_imu ? (imu.pitch() + 1.5f) : imu.pitch();
    auto pi = imu_pre.process(
        raw_pitch_in,        imu.roll(),         imu.yaw(),
        imu.roll_rate(),     imu.pitch_rate(),   imu.yaw_rate(),
        imu.accel_x(),       imu.accel_y());

    uint8_t imu_fault_mask = pi.fault_mask & 0x31;  // pitch | pitch_rate | yaw_rate
    bool imu_stale = imu.is_stale();
    if (imu_stale) imu_fault_mask |= 0x31;

    // Encoder preprocessing
    auto eL = enc_L.process(sim_wpos_L, sim_wvel_L);
    auto eR = enc_R.process(sim_wpos_R, sim_wvel_R);

    // L/R divergence watchdog (latched after sustained mismatch)
    float div = fabsf(eL.velocity - eR.velocity);
    if (div > cfg::wheel_divergence_rad) {
        if (wheel_div_start_ms == 0) wheel_div_start_ms = now;
        if (now - wheel_div_start_ms > cfg::wheel_divergence_ms) wheel_diverged = true;
    } else {
        wheel_div_start_ms = 0;
    }

    bool wheel_valid = eL.valid && eR.valid && !wheel_diverged;
    float wpos_mean = 0.5f * (eL.position + eR.position);
    float wvel_mean = 0.5f * (eL.velocity + eR.velocity);

    // Run EKF
    ekf.predict(dt);

    MeasVec z;
    z(0,0) = pi.pitch;
    z(1,0) = pi.pitch_rate;
    z(2,0) = pi.yaw_rate;
    z(3,0) = wpos_mean;
    z(4,0) = wvel_mean;

    bool accepted = ekf.update(z, imu_fault_mask, wheel_valid);
    auto est = ekf.estimate();
    float pitch_rel = est.pitch - pitch_zero;

    // wheel-derived accel vs IMU lin-accel
    wheel_accel_x = (est.wheel_vel - prev_wheel_vel) / dt * cfg::wheel_radius_m;
    prev_wheel_vel = est.wheel_vel;
    float xval_err = fabsf(wheel_accel_x - pi.accel_x);
    bool xval_eligible = wheel_valid && !imu_stale && fabsf(pitch_rel) < 0.2f;
    if (xval_eligible && xval_err > cfg::xval_accel_threshold_mps2) {
        if (xval_bad_start_ms == 0) xval_bad_start_ms = now;
        if (now - xval_bad_start_ms > cfg::xval_window_ms) xval_bad = true;
    } else if (xval_err < 0.5f * cfg::xval_accel_threshold_mps2) {
        xval_bad_start_ms = 0;
    }

    // Fall detection
    if (fabsf(pitch_rel) > cfg::fall_pitch_rad) {
        if (fall_start_ms == 0) fall_start_ms = now;
        if (now - fall_start_ms > cfg::fall_latch_ms) fallen = true;
    } else {
        fall_start_ms = 0;
    }

    if (tick % 2 != 0) return;

    uint8_t cal = read_calib_status();
    float raw_pitch = (imu.pitch() - pitch_zero) * 57.2958f;
    float ekf_pitch = (est.pitch  - pitch_zero) * 57.2958f;

    Serial.printf("D,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                  "%.5f,%.4f,%d,%d,%d,%d,%d,%d,%d,"
                  "%d,%d,%d,%d,%u,%.2f,%.2f\n",
                  raw_pitch, ekf_pitch,
                  imu.pitch_rate(), est.pitch_rate,
                  imu.yaw_rate(),   est.yaw_rate,
                  wpos_mean,        est.wheel_pos,
                  wvel_mean,        est.wheel_vel,
                  est.x_pos,        est.x_vel,
                  ekf.pitch_variance(), ekf.pitch_kalman_gain(),
                  accepted ? 1 : 0,
                  fault_imu ? 1 : 0, fault_wheel ? 1 : 0,
                  (cal >> 6) & 3, (cal >> 4) & 3, (cal >> 2) & 3, cal & 3,
                  imu_stale ? 1 : 0, wheel_diverged ? 1 : 0,
                  xval_bad ? 1 : 0, fallen ? 1 : 0,
                  (unsigned)imu_fault_mask, pi.accel_x, wheel_accel_x);
}

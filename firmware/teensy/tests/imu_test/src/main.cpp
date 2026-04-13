#include <Arduino.h>
#include <Wire.h>
#include "imu.h"

Imu imu;
float pitch_zero = 0, roll_zero = 0, yaw_zero = 0;

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

    Serial.println("[imu_test] Initializing BNO055...");

    Wire.begin();
    Wire.setClock(400000);

    // I2C scan
    Serial.print("[imu_test] I2C scan: ");
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
        Serial.println("[imu_test] Send 'z' to zero, 'h' for help");
    } else {
        Serial.println("[imu_test] BNO055 FAILED");
        while (true) { digitalToggle(LED_BUILTIN); delay(200); }
    }

    delay(100);
}

void loop() {
    
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'z') {
            imu.update();
            pitch_zero = imu.pitch();
            roll_zero  = imu.roll();
            yaw_zero   = imu.yaw();
            Serial.printf("ZERO,%.4f,%.4f,%.4f\n", pitch_zero, roll_zero, yaw_zero);
        } else if (c == 'h') {
            Serial.println("[imu_test] Send 'z' to zero, 'h' for help");
        }
    }

    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if (now - last_ms < 20) return;
    last_ms = now;

    imu.update();
    uint8_t cal = read_calib_status();

    float p = (imu.pitch() - pitch_zero) * 57.2958f;
    float r = (imu.roll()  - roll_zero)  * 57.2958f;
    float y = (imu.yaw()   - yaw_zero)   * 57.2958f;

    Serial.printf("D,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
        p, r, y,
        imu.pitch_rate(), imu.roll_rate(), imu.yaw_rate(),
        imu.accel_x(), imu.accel_y(), imu.accel_z(),
        (cal >> 6) & 3, (cal >> 4) & 3, (cal >> 2) & 3, cal & 3);
}

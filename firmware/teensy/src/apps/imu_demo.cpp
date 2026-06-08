#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "telemetry.h"

Imu imu;
Telemetry tel;
float pitch_zero = 0, roll_zero = 0, yaw_zero = 0;

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
    uint8_t cal = imu.calib_status();

    const float RAD2DEG = 57.2958f;

    tel.put("euler.pitch", (imu.pitch() - pitch_zero) * RAD2DEG);
    tel.put("euler.roll",  (imu.roll()  - roll_zero)  * RAD2DEG);
    tel.put("euler.yaw",   (imu.yaw()   - yaw_zero)   * RAD2DEG);
    tel.put("gyro.pitch", imu.pitch_rate() * RAD2DEG);
    tel.put("gyro.roll",  imu.roll_rate()  * RAD2DEG);
    tel.put("gyro.yaw",   imu.yaw_rate()   * RAD2DEG);
    tel.put("accel.x", imu.accel_x());
    tel.put("accel.y", imu.accel_y());
    tel.put("accel.z", imu.accel_z());
    tel.put("cal.sys",   (cal >> 6) & 3);
    tel.put("cal.gyro",  (cal >> 4) & 3);
    tel.put("cal.accel", (cal >> 2) & 3);
    tel.put("cal.mag",    cal       & 3);
    tel.send();
}

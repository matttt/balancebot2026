#pragma once
#include <Arduino.h>

// Self-describing CSV telemetry over Serial.
//
// Build a row by calling put() for each channel, then send(). send() prints a
// header "H,<name>,<name>,..." derived from the row's names, then a data row
// "D,<v>,<v>,...". Because names and values are supplied together, the header
// and data can never drift out of sync.
//
// The header is re-emitted every HEADER_EVERY rows (not just at boot) so a host
// dashboard that connects mid-stream still discovers the channels — opening the
// serial port does not reset the Teensy, so a once-only header would be missed.
//
// Channel names use a "group.series" convention: the host dashboard overlays
// every channel sharing a group on one axis (the part after '.' is the line
// label). A name with no '.' gets its own subplot. Emit values already in
// display units so the dashboard stays unit-agnostic.
class Telemetry {
public:
    void put(const char* name, float value) {
        if (n_ >= MAXF) return;
        names_[n_] = name;
        vals_[n_++] = value;
    }

    void send() {
        if (rows_until_header_ == 0) {
            Serial.print("H");
            for (uint8_t i = 0; i < n_; i++) { Serial.print(','); Serial.print(names_[i]); }
            Serial.println();
            rows_until_header_ = HEADER_EVERY;
        }
        rows_until_header_--;

        Serial.print("D");
        for (uint8_t i = 0; i < n_; i++) { Serial.print(','); Serial.print(vals_[i], 4); }
        Serial.println();
        n_ = 0;
    }

private:
    static constexpr uint8_t MAXF = 40;
    static constexpr uint16_t HEADER_EVERY = 50;  // ~1 s at 50 Hz
    const char* names_[MAXF];
    float vals_[MAXF];
    uint8_t n_ = 0;
    uint16_t rows_until_header_ = 0;  // 0 => emit header on next send()
};

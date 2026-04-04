#include <Arduino.h>
#include "master_presets.h"
#include "master_can_tx.h"

struct MotionPreset {
    char          key;
    const __FlashStringHelper* label;
    uint8_t       dir;
    uint16_t      steps;
    uint16_t      rpm;
    uint8_t       accel;
};

static const MotionPreset PRESETS[] = {
    {'f', F("Fwd 200 @ 60 RPM"), 1, 200, 60, 50},
    {'b', F("Back 200 @ 60 RPM"), 0, 200, 60, 50},
    {'F', F("Fast fwd 1000 @ 120"), 1, 1000, 120, 80},
    {'B', F("Fast back 1000 @ 120"), 0, 1000, 120, 80},
    {'x', F("1 rev CW @ 90"), 1, 1600, 90, 60},
    {'X', F("5 rev CW @ 150"), 1, 8000, 150, 100},
};

bool masterPresetTry(char c) {
    for (unsigned i = 0; i < sizeof(PRESETS) / sizeof(PRESETS[0]); i++) {
        if (PRESETS[i].key == c) {
            Serial.println(PRESETS[i].label);
            masterSendMotorCmd(PRESETS[i].dir, PRESETS[i].steps, PRESETS[i].rpm, PRESETS[i].accel);
            return true;
        }
    }
    return false;
}

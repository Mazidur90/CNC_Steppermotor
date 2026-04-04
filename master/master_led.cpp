#include "master_led.h"
#include "master_globals.h"

void masterUpdateLed() {
    uint32_t now = millis();
    uint8_t ref = (gTargetNode <= 8) ? gTargetNode : 1;
    uint8_t st = nodes[ref].state;

    uint32_t interval;
    if (st == CSP_STATE_FAULT) {
        interval = 80;
    } else if (now - lastTxTime < 400) {
        interval = 100;
    } else if (st == CSP_STATE_RUNNING || st == CSP_STATE_HOMING || st == CSP_STATE_JOGGING) {
        interval = 500;
    } else {
        digitalWrite(STATUS_LED, HIGH);
        return;
    }

    if (now - lastLedToggle >= interval) {
        lastLedToggle = now;
        ledState = !ledState;
        digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
    }
}

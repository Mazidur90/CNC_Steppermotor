#include "slave_peripherals.h"
#include "slave_globals.h"
#include "slave_config.h"

void slavePeripheralsInitPins() {
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
    pinMode(SPINDLE_PIN, OUTPUT);
    pinMode(COOLANT_PIN, OUTPUT);
    slaveApplyDoutPins();
    pinMode(HOME_PIN, INPUT_PULLUP);
    pinMode(CAN_INT_PIN, INPUT);
    pinMode(LIMIT_MIN_PIN, INPUT_PULLUP);
    pinMode(LIMIT_MAX_PIN, INPUT_PULLUP);
}

void slaveApplyDoutPins() {
    digitalWrite(SPINDLE_PIN, (doutState & CSP_DOUT_SPINDLE) ? HIGH : LOW);
    digitalWrite(COOLANT_PIN, (doutState & CSP_DOUT_COOLANT) ? HIGH : LOW);
}

bool slaveLimitMinTripped() {
    return digitalRead(LIMIT_MIN_PIN) == LOW;
}

bool slaveLimitMaxTripped() {
    return digitalRead(LIMIT_MAX_PIN) == LOW;
}

void slavePollMcpErrors() {
    uint32_t now = millis();
    if (now - lastMcpPoll < MCP_POLL_MS) {
        return;
    }
    lastMcpPoll = now;
    uint8_t eflg = mcp2515.getErrorFlags();
    if (eflg & (MCP2515::EFLG_TXBO | MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR | MCP2515::EFLG_TXEP |
                MCP2515::EFLG_RXEP)) {
        static uint32_t lastLog = 0;
        if (now - lastLog > 3000) {
            lastLog = now;
            Serial.print(F("MCP EFLG=0x"));
            Serial.println(eflg, HEX);
        }
        mcp2515.clearRXnOVRFlags();
    }
}

void slaveUpdateStatusLed() {
    uint32_t now = millis();
    uint32_t interval;

    switch (currentState) {
        case STATE_RUNNING:
            interval = 200;
            break;
        case STATE_HOMING:
            interval = 100;
            break;
        case STATE_FAULT:
            interval = 80;
            break;
        case STATE_STOPPED:
            interval = 1000;
            break;
        case STATE_JOGGING:
            interval = 150;
            break;
        default:
            interval = 0;
            break;
    }

    if (interval == 0) {
        digitalWrite(STATUS_LED, LOW);
        return;
    }

    if (now - lastLedToggle >= interval) {
        lastLedToggle = now;
        ledState = !ledState;
        digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
    }
}

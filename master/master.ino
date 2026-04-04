// =============================================================================
// CAN bus stepper master — Arduino Mega 2560 + MCP2515
//
// Sends CANStepperProtocol v1 frames (see libraries/CANStepperProtocol).
// Serial Monitor @ 115200 for the command menu.
//
// Pins: CS 10, INT 2, LED 13 | SPI 50–52 (Mega)
// MCP2515 crystal: set MCP_8MHZ or MCP_16MHZ to match the module.
// =============================================================================

#include <stdlib.h>
#include <SPI.h>
#include <mcp2515.h>
#include "../libraries/CANStepperProtocol/CANStepperProtocol.h"

#ifndef CSP_VERBOSE
#define CSP_VERBOSE 0
#endif

#define CAN_CS_PIN        10
#define CAN_INT_PIN        2
#define STATUS_LED        13

#define CAN_TX_RETRY_COUNT  3
#define CAN_TX_RETRY_DELAY  10

static const char* const STATE_NAMES[] = {
    "IDLE", "RUNNING", "HOMING", "FAULT", "STOPPED"
};

MCP2515 mcp2515(CAN_CS_PIN);

struct can_frame txFrame;
struct can_frame rxFrame;

volatile bool canInterruptFlag = false;

uint8_t  slaveState    = CSP_STATE_IDLE;
int32_t  slavePosition = 0;
uint8_t  slaveError    = CSP_ERR_NONE;
uint16_t slaveSpeed    = 0;
uint32_t lastStatusRx  = 0;
uint32_t statusCrcDrops = 0;

uint32_t lastLedToggle = 0;
bool     ledState      = false;
uint32_t lastTxTime    = 0;

void canISR() {
    canInterruptFlag = true;
}

bool sendCanFrame(struct can_frame* frame);
void sendMotorCmd(uint8_t dir, uint16_t steps, uint16_t rpm, uint8_t accelFactor);
void sendStopCmd(uint8_t stopType);
void sendHomeCmd(uint8_t dir, uint8_t speedPct);
void handleRxFrame();
void handleSerialCmd(char c);
void printStatus();
void printMenu();
void updateLED();

void setup() {
    Serial.begin(115200);
    Serial.println(F("=== CAN STEPPER MASTER (protocol v1, CRC8) ==="));

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
    pinMode(CAN_INT_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);

    SPI.begin();

    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 reset FAILED — check wiring"));
        while (true) {
            digitalWrite(STATUS_LED, HIGH);
            delay(80);
            digitalWrite(STATUS_LED, LOW);
            delay(80);
        }
    }

    if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 setBitrate FAILED — check crystal (8 vs 16 MHz)"));
        while (true) {
            digitalWrite(STATUS_LED, HIGH);
            delay(200);
            digitalWrite(STATUS_LED, LOW);
            delay(200);
        }
    }

    mcp2515.setNormalMode();

    Serial.println(F("CAN 500 kbit/s, normal mode"));
    printMenu();
}

void loop() {
    if (canInterruptFlag) {
        canInterruptFlag = false;

        uint8_t irq = mcp2515.getInterrupts();

        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &rxFrame) == MCP2515::ERROR_OK) {
                handleRxFrame();
            }
        }
        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &rxFrame) == MCP2515::ERROR_OK) {
                handleRxFrame();
            }
        }
    }

    if (Serial.available()) {
        char c = (char)Serial.read();
        if (c != '\n' && c != '\r') {
            handleSerialCmd(c);
        }
    }

    static uint32_t lastStaleWarn = 0;
    if (lastStatusRx != 0 && millis() - lastStatusRx > 1000) {
        if (millis() - lastStaleWarn > 3000) {
            lastStaleWarn = millis();
            Serial.println(F("WARNING: no valid STATUS from slave for >1 s"));
        }
    }

    updateLED();
}

void handleSerialCmd(char c) {
    switch (c) {
        case 'f':
            Serial.println(F("CMD: Forward 200 steps @ 60 RPM"));
            sendMotorCmd(1, 200, 60, 50);
            break;
        case 'b':
            Serial.println(F("CMD: Backward 200 steps @ 60 RPM"));
            sendMotorCmd(0, 200, 60, 50);
            break;
        case 'F':
            Serial.println(F("CMD: Fast forward 1000 steps @ 120 RPM"));
            sendMotorCmd(1, 1000, 120, 80);
            break;
        case 'B':
            Serial.println(F("CMD: Fast backward 1000 steps @ 120 RPM"));
            sendMotorCmd(0, 1000, 120, 80);
            break;
        case 'x':
            Serial.println(F("CMD: Full rotation CW (1600 microsteps) @ 90 RPM"));
            sendMotorCmd(1, 1600, 90, 60);
            break;
        case 'X':
            Serial.println(F("CMD: 5 full CW rotations @ 150 RPM"));
            sendMotorCmd(1, 8000, 150, 100);
            break;
        case 's':
            Serial.println(F("CMD: Decelerate stop"));
            sendStopCmd(CSP_STOP_DECELERATE);
            break;
        case 'S':
            Serial.println(F("CMD: Immediate stop (software)"));
            sendStopCmd(CSP_STOP_IMMEDIATE);
            break;
        case 'h':
            Serial.println(F("CMD: Home CCW @ 20%"));
            sendHomeCmd(0, 20);
            break;
        case 'H':
            Serial.println(F("CMD: Home CW @ 20%"));
            sendHomeCmd(1, 20);
            break;
        case '?':
            printStatus();
            break;
        case 'm':
            printMenu();
            break;
        default:
            Serial.print(F("Unknown: "));
            Serial.println(c);
            Serial.println(F("Type 'm' for menu"));
            break;
    }
}

void sendMotorCmd(uint8_t dir, uint16_t steps, uint16_t rpm, uint8_t accelFactor) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_MOTOR_CMD;
    txFrame.can_dlc = CSP_DLC;
    if (!csp_build_motor_cmd(txFrame.data, dir, steps, rpm, accelFactor)) {
        Serial.println(F("TX MOTOR_CMD rejected (steps/rpm must be non-zero)"));
        return;
    }
    if (sendCanFrame(&txFrame)) {
        lastTxTime = millis();
        Serial.print(F("TX MOTOR_CMD dir="));
        Serial.print(dir);
        Serial.print(F(" steps="));
        Serial.print(steps);
        Serial.print(F(" rpm="));
        Serial.print(rpm);
        Serial.print(F(" accel="));
        Serial.println(accelFactor);
    }
}

void sendStopCmd(uint8_t stopType) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_STOP;
    txFrame.can_dlc = CSP_DLC;
    csp_build_stop(txFrame.data, stopType);
    if (sendCanFrame(&txFrame)) {
        lastTxTime = millis();
        Serial.print(F("TX STOP type=0x"));
        Serial.println(stopType, HEX);
    }
}

void sendHomeCmd(uint8_t dir, uint8_t speedPct) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_HOME;
    txFrame.can_dlc = CSP_DLC;
    csp_build_home(txFrame.data, dir, speedPct);
    if (sendCanFrame(&txFrame)) {
        lastTxTime = millis();
        Serial.print(F("TX HOME dir="));
        Serial.print(dir);
        Serial.print(F(" speed_pct="));
        Serial.println(speedPct);
    }
}

bool sendCanFrame(struct can_frame* frame) {
    for (int attempt = 0; attempt < CAN_TX_RETRY_COUNT; attempt++) {
        MCP2515::ERROR err = mcp2515.sendMessage(frame);
        if (err == MCP2515::ERROR_OK) {
            return true;
        }
        Serial.print(F("TX attempt "));
        Serial.print(attempt + 1);
        Serial.print(F(" failed err="));
        Serial.println((int)err);
        delay(CAN_TX_RETRY_DELAY);
    }
    Serial.println(F("TX FAILED after retries — check bus / termination"));
    return false;
}

void handleRxFrame() {
    if (rxFrame.can_id != CSP_CAN_ID_STATUS) {
#if CSP_VERBOSE
        Serial.print(F("RX unknown ID=0x"));
        Serial.println(rxFrame.can_id, HEX);
#endif
        return;
    }
    if (rxFrame.can_dlc < CSP_DLC) {
        return;
    }

    uint8_t st;
    int32_t pos;
    uint8_t err;
    uint16_t sp;
    if (!csp_parse_status(rxFrame.data, &st, &pos, &err, &sp)) {
        statusCrcDrops++;
        static uint32_t lastBad = 0;
        if (millis() - lastBad > 2000) {
            lastBad = millis();
            Serial.println(F("STATUS dropped: CRC or format error"));
        }
        return;
    }

    slaveState    = st;
    slavePosition = pos;
    slaveError    = err;
    slaveSpeed    = sp;
    lastStatusRx  = millis();

    static uint8_t lastPrintedState = 0xFF;
    static int32_t lastPrintedPos   = 0x7FFFFFFFL;
    if (slaveState != lastPrintedState || labs(slavePosition - lastPrintedPos) > 50) {
        lastPrintedState = slaveState;
        lastPrintedPos   = slavePosition;

        Serial.print(F("SLAVE: state="));
        if (slaveState <= CSP_STATE_STOPPED) {
            Serial.print(STATE_NAMES[slaveState]);
        } else {
            Serial.print(F("UNKNOWN("));
            Serial.print(slaveState);
            Serial.print(')');
        }
        Serial.print(F(" pos="));
        Serial.print(slavePosition);
        Serial.print(F(" speed_x10="));
        Serial.print(slaveSpeed);
        if (slaveError != CSP_ERR_NONE) {
            Serial.print(F(" ERR=0x"));
            Serial.print(slaveError, HEX);
        }
        Serial.println();
    }
}

void printStatus() {
    Serial.println(F("--- MASTER STATUS ---"));
    Serial.print(F("  Slave state : "));
    if (slaveState <= CSP_STATE_STOPPED) {
        Serial.println(STATE_NAMES[slaveState]);
    } else {
        Serial.print(F("UNKNOWN "));
        Serial.println(slaveState);
    }
    Serial.print(F("  Slave pos   : "));
    Serial.println(slavePosition);
    Serial.print(F("  Slave speed : "));
    Serial.print((float)slaveSpeed / 10.0f);
    Serial.println(F(" steps/s"));
    Serial.print(F("  Slave error : 0x"));
    Serial.println(slaveError, HEX);
    Serial.print(F("  STATUS drops: "));
    Serial.println(statusCrcDrops);
    uint32_t age = (lastStatusRx == 0) ? 99999UL : (millis() - lastStatusRx);
    Serial.print(F("  Status age  : "));
    Serial.print(age);
    Serial.println(F(" ms"));
    Serial.println(F("---------------------"));
}

void printMenu() {
    Serial.println();
    Serial.println(F("======= CAN MASTER MENU ======="));
    Serial.println(F("  f / b / F / B / x / X  motion"));
    Serial.println(F("  s = decel stop   S = immediate stop"));
    Serial.println(F("  h / H = home CCW / CW @ 20%"));
    Serial.println(F("  ? = status   m = menu"));
    Serial.println(F("================================"));
}

void updateLED() {
    uint32_t now = millis();
    uint32_t interval;

    if (slaveState == CSP_STATE_FAULT) {
        interval = 80;
    } else if (now - lastTxTime < 500) {
        interval = 100;
    } else if (slaveState == CSP_STATE_RUNNING || slaveState == CSP_STATE_HOMING) {
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

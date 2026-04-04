// =============================================================================
// CAN bus stepper slave — MCP2515 + STEP/DIR driver (e.g. TMC2208)
//
// CANStepperProtocol v1: CRC-8 on bytes 0..6, byte 7 = checksum.
// Optional: #define CSP_VERBOSE 1 before includes for per-frame Serial trace.
// Optional: #define WOKWI_SIMULATION for Wokwi loopback + Serial injection.
//
// AVR: 2 s watchdog reset in loop() (disabled on non-AVR for portability).
// =============================================================================

// #define CSP_VERBOSE 1
// #define WOKWI_SIMULATION

#include <SPI.h>
#include <mcp2515.h>
#include <AccelStepper.h>
#include "../libraries/CANStepperProtocol/CANStepperProtocol.h"

#if defined(__AVR__)
#include <avr/wdt.h>
#endif

#ifndef CSP_VERBOSE
#define CSP_VERBOSE 0
#endif

#define CAN_CS_PIN   10
#define CAN_INT_PIN   2
#define STEP_PIN      5
#define DIR_PIN       6
#define EN_PIN        7
#define HOME_PIN      3
#define STATUS_LED   13

#define STEPS_PER_REV           200
#define MICROSTEPS                8
#define STEPS_PER_REV_MICRO  (STEPS_PER_REV * MICROSTEPS)
#define MAX_SPEED_STEPS      4000.0f
#define STATUS_TX_INTERVAL_MS   100

typedef enum {
    STATE_IDLE    = CSP_STATE_IDLE,
    STATE_RUNNING = CSP_STATE_RUNNING,
    STATE_HOMING  = CSP_STATE_HOMING,
    STATE_FAULT   = CSP_STATE_FAULT,
    STATE_STOPPED = CSP_STATE_STOPPED,
} SlaveState;

MCP2515 mcp2515(CAN_CS_PIN);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

struct can_frame rxFrame;
struct can_frame txFrame;

volatile bool canInterruptFlag = false;
volatile bool homeSwitchTriggered = false;

SlaveState currentState = STATE_IDLE;
uint8_t    errorCode    = CSP_ERR_NONE;
uint32_t   lastStatusTx = 0;
uint32_t   lastLedToggle = 0;
bool       ledState = false;
uint32_t   rxCrcRejects = 0;

void canISR() {
    canInterruptFlag = true;
}

void homeISR() {
    if (currentState == STATE_HOMING) {
        homeSwitchTriggered = true;
    }
}

void dispatchCanFrame();
void handleMotorCmd();
void handleStopCmd();
void handleHomeCmd();
void sendStatusFrame();
void updateLED();

#if defined(__AVR__)
static void cspEnableWatchdog() {
    wdt_disable();
    wdt_enable(WDTO_2S);
}
#endif

#ifdef WOKWI_SIMULATION
void simulateCanFromSerial();
#endif

void setup() {
    Serial.begin(115200);
    Serial.println(F("=== CAN STEPPER SLAVE (protocol v1) ==="));

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    pinMode(HOME_PIN, INPUT_PULLUP);
    pinMode(CAN_INT_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(HOME_PIN), homeISR, FALLING);

    stepper.setMaxSpeed(MAX_SPEED_STEPS);
    stepper.setAcceleration(500.0f);
    stepper.setCurrentPosition(0);

    SPI.begin();
    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 reset FAILED"));
        currentState = STATE_FAULT;
        errorCode    = CSP_ERR_MCP_RESET;
#if defined(__AVR__)
        cspEnableWatchdog();
#endif
        return;
    }
    if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 setBitrate FAILED — check crystal (8 vs 16 MHz)"));
        currentState = STATE_FAULT;
        errorCode    = CSP_ERR_MCP_BITRATE;
#if defined(__AVR__)
        cspEnableWatchdog();
#endif
        return;
    }

#ifdef WOKWI_SIMULATION
    mcp2515.setLoopbackMode();
    Serial.println(F("WOKWI: loopback + Serial commands (f b F B s S h ?)"));
#else
    mcp2515.setNormalMode();
    Serial.println(F("CAN normal mode"));
#endif

    Serial.println(F("Slave ready."));
#if defined(__AVR__)
    cspEnableWatchdog();
#endif
}

void loop() {
#if defined(__AVR__)
    wdt_reset();
#endif

    if (canInterruptFlag) {
        canInterruptFlag = false;

        uint8_t irq = mcp2515.getInterrupts();

        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &rxFrame) == MCP2515::ERROR_OK) {
                dispatchCanFrame();
            }
        }
        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &rxFrame) == MCP2515::ERROR_OK) {
                dispatchCanFrame();
            }
        }
    }

#ifdef WOKWI_SIMULATION
    simulateCanFromSerial();
#endif

    if (homeSwitchTriggered) {
        homeSwitchTriggered = false;
        stepper.stop();
        stepper.setCurrentPosition(0);
        currentState = STATE_IDLE;
        Serial.println(F("HOME SWITCH — position zeroed"));
    }

    if (currentState == STATE_RUNNING || currentState == STATE_HOMING) {
        stepper.run();

        if (stepper.distanceToGo() == 0) {
            if (currentState == STATE_RUNNING) {
                currentState = STATE_IDLE;
                errorCode    = CSP_ERR_NONE;
                Serial.print(F("Move done. Position="));
                Serial.println(stepper.currentPosition());
            } else if (currentState == STATE_HOMING) {
                currentState = STATE_FAULT;
                errorCode    = CSP_ERR_HOME_FAIL;
                Serial.println(F("HOMING FAULT: switch not found"));
            }
        }
    }

    uint32_t now = millis();
    if (now - lastStatusTx >= STATUS_TX_INTERVAL_MS) {
        lastStatusTx = now;
        sendStatusFrame();
    }

    updateLED();
}

void dispatchCanFrame() {
#if CSP_VERBOSE
    Serial.print(F("RX ID=0x"));
    Serial.print(rxFrame.can_id, HEX);
    Serial.print(F(" DLC="));
    Serial.println(rxFrame.can_dlc);
#endif
    if (rxFrame.can_dlc < CSP_DLC) {
        return;
    }

    switch (rxFrame.can_id) {
        case CSP_CAN_ID_MOTOR_CMD:
            handleMotorCmd();
            break;
        case CSP_CAN_ID_STOP:
            handleStopCmd();
            break;
        case CSP_CAN_ID_HOME:
            handleHomeCmd();
            break;
#if CSP_VERBOSE
        default:
            Serial.print(F("Unknown CAN ID 0x"));
            Serial.println(rxFrame.can_id, HEX);
            break;
#else
        default:
            break;
#endif
    }
}

void handleMotorCmd() {
    uint8_t dir;
    uint16_t steps;
    uint16_t rpm;
    uint8_t accelF;
    if (!csp_parse_motor_cmd(rxFrame.data, &dir, &steps, &rpm, &accelF)) {
        rxCrcRejects++;
#if CSP_VERBOSE
        Serial.println(F("MOTOR_CMD rejected (CRC/cmd)"));
#endif
        return;
    }

#if CSP_VERBOSE
    Serial.print(F("MOTOR dir="));
    Serial.print(dir);
    Serial.print(F(" steps="));
    Serial.print(steps);
    Serial.print(F(" rpm="));
    Serial.print(rpm);
    Serial.print(F(" accel="));
    Serial.println(accelF);
#endif

    float stepsPerSec = (float)rpm * (float)STEPS_PER_REV_MICRO / 60.0f;
    stepsPerSec = constrain(stepsPerSec, 10.0f, MAX_SPEED_STEPS);

    float accelVal = (float)accelF * 10.0f;
    accelVal = constrain(accelVal, 50.0f, 10000.0f);

    stepper.setMaxSpeed(stepsPerSec);
    stepper.setAcceleration(accelVal);
    digitalWrite(EN_PIN, LOW);

    int32_t stepsToMove = (dir == 1) ? (int32_t)steps : -(int32_t)steps;
    stepper.move(stepsToMove);

    currentState = STATE_RUNNING;
    errorCode    = CSP_ERR_NONE;
}

void handleStopCmd() {
    uint8_t stopType;
    if (!csp_parse_stop(rxFrame.data, &stopType)) {
        rxCrcRejects++;
#if CSP_VERBOSE
        Serial.println(F("STOP rejected (CRC)"));
#endif
        return;
    }

    if (stopType != CSP_STOP_IMMEDIATE && stopType != CSP_STOP_DECELERATE) {
        return;
    }

    if (stopType == CSP_STOP_IMMEDIATE) {
        stepper.setCurrentPosition(stepper.currentPosition());
        currentState = STATE_STOPPED;
        Serial.println(F("STOP: immediate"));
    } else {
        stepper.stop();
        currentState = STATE_STOPPED;
        Serial.println(F("STOP: decel"));
    }
}

void handleHomeCmd() {
    uint8_t homeDir;
    uint8_t homePct;
    if (!csp_parse_home(rxFrame.data, &homeDir, &homePct)) {
        rxCrcRejects++;
#if CSP_VERBOSE
        Serial.println(F("HOME rejected (CRC)"));
#endif
        return;
    }

    homePct = constrain(homePct, 1, 100);
    float homeSpeed = MAX_SPEED_STEPS * ((float)homePct / 100.0f);
    homeSpeed = constrain(homeSpeed, 50.0f, 1000.0f);

    stepper.setMaxSpeed(homeSpeed);
    stepper.setAcceleration(200.0f);
    digitalWrite(EN_PIN, LOW);

    int32_t bigMove = (homeDir == 0) ? -100000L : 100000L;
    stepper.move(bigMove);

    currentState        = STATE_HOMING;
    homeSwitchTriggered = false;
    errorCode           = CSP_ERR_NONE;

#if CSP_VERBOSE
    Serial.print(F("HOME dir="));
    Serial.print(homeDir);
    Serial.print(F(" speed="));
    Serial.println(homeSpeed);
#endif
}

void sendStatusFrame() {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_STATUS;
    txFrame.can_dlc = CSP_DLC;

    int32_t pos = stepper.currentPosition();
    uint16_t speedNow = (uint16_t)(stepper.speed() * 10.0f);

    csp_build_status(txFrame.data, (uint8_t)currentState, pos, errorCode, speedNow);

    MCP2515::ERROR err = mcp2515.sendMessage(&txFrame);
    if (err != MCP2515::ERROR_OK) {
        static uint32_t lastTxErrPrint = 0;
        if (millis() - lastTxErrPrint > 2000) {
            lastTxErrPrint = millis();
            Serial.print(F("STATUS TX err="));
            Serial.println((int)err);
        }
    }
}

void updateLED() {
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

#ifdef WOKWI_SIMULATION
void simulateCanFromSerial() {
    if (!Serial.available()) {
        return;
    }

    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
        return;
    }

    memset(&rxFrame, 0, sizeof(rxFrame));
    rxFrame.can_dlc = CSP_DLC;

    switch (c) {
        case 'f':
            rxFrame.can_id = CSP_CAN_ID_MOTOR_CMD;
            csp_build_motor_cmd(rxFrame.data, 1, 200, 60, 50);
            dispatchCanFrame();
            break;
        case 'b':
            rxFrame.can_id = CSP_CAN_ID_MOTOR_CMD;
            csp_build_motor_cmd(rxFrame.data, 0, 200, 60, 50);
            dispatchCanFrame();
            break;
        case 'F':
            rxFrame.can_id = CSP_CAN_ID_MOTOR_CMD;
            csp_build_motor_cmd(rxFrame.data, 1, 1000, 120, 80);
            dispatchCanFrame();
            break;
        case 'B':
            rxFrame.can_id = CSP_CAN_ID_MOTOR_CMD;
            csp_build_motor_cmd(rxFrame.data, 0, 1000, 120, 80);
            dispatchCanFrame();
            break;
        case 's':
            rxFrame.can_id = CSP_CAN_ID_STOP;
            csp_build_stop(rxFrame.data, CSP_STOP_DECELERATE);
            dispatchCanFrame();
            break;
        case 'S':
            rxFrame.can_id = CSP_CAN_ID_STOP;
            csp_build_stop(rxFrame.data, CSP_STOP_IMMEDIATE);
            dispatchCanFrame();
            break;
        case 'h':
            rxFrame.can_id = CSP_CAN_ID_HOME;
            csp_build_home(rxFrame.data, 0, 20);
            dispatchCanFrame();
            break;
        case '?':
            Serial.print(F("STATE="));
            Serial.print((int)currentState);
            Serial.print(F(" POS="));
            Serial.print(stepper.currentPosition());
            Serial.print(F(" RX_CRC_FAIL="));
            Serial.print(rxCrcRejects);
            Serial.print(F(" ERR=0x"));
            Serial.println(errorCode, HEX);
            break;
        default:
            Serial.print(F("Unknown: "));
            Serial.println(c);
            break;
    }
}
#endif

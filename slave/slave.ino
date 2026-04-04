// =============================================================================
// CAN stepper slave — entry point (modular .cpp files in this folder)
//
// Optional: #define CSP_VERBOSE 1 in slave_config.h
// Optional: #define WOKWI_SIMULATION before includes (or -D on CLI)
// =============================================================================

// #define WOKWI_SIMULATION

#include <SPI.h>
#include "slave_config.h"
#include "slave_globals.h"
#include "slave_eeprom.h"
#include "slave_peripherals.h"
#include "slave_can_handlers.h"
#include "slave_motion.h"
#include "slave_wokwi.h"

#if defined(__AVR__)
#include <avr/wdt.h>
#endif

void canISR() {
    canInterruptFlag = true;
}

void homeISR() {
    if (currentState == STATE_HOMING) {
        homeSwitchTriggered = true;
    }
}

#if defined(__AVR__)
static void slaveWatchdogEnable() {
    wdt_disable();
    wdt_enable(WDTO_2S);
}
#endif

void setup() {
    Serial.begin(115200);
    Serial.println(F("=== CAN STEPPER SLAVE v2 (modular) ==="));

    slavePeripheralsInitPins();
    slaveEepromLoad();

    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(HOME_PIN), homeISR, FALLING);

    stepper.setMaxSpeed((float)cfgMaxSpeedSteps);
    stepper.setAcceleration(500.0f);
    stepper.setCurrentPosition(0);

    SPI.begin();
    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 reset FAILED"));
        currentState = STATE_FAULT;
        errorCode    = CSP_ERR_MCP_RESET;
#if defined(__AVR__)
        slaveWatchdogEnable();
#endif
        return;
    }
    if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 setBitrate FAILED"));
        currentState = STATE_FAULT;
        errorCode    = CSP_ERR_MCP_BITRATE;
#if defined(__AVR__)
        slaveWatchdogEnable();
#endif
        return;
    }

#ifdef WOKWI_SIMULATION
    mcp2515.setLoopbackMode();
    Serial.println(F("WOKWI loopback"));
#else
    mcp2515.setNormalMode();
#endif

    Serial.print(F("Node "));
    Serial.print(myNodeId);
    Serial.print(F("  maxSpd "));
    Serial.print(cfgMaxSpeedSteps);
    Serial.print(F("  hbMs "));
    Serial.println(cfgHeartbeatMs);

#if defined(__AVR__)
    slaveWatchdogEnable();
#endif
}

void loop() {
#if defined(__AVR__)
    wdt_reset();
#endif

    slaveMotionSupervisionTick();

    if (canInterruptFlag) {
        canInterruptFlag = false;
        uint8_t irq = mcp2515.getInterrupts();
        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &rxFrame) == MCP2515::ERROR_OK) {
                statRxFramesOk++;
                slaveDispatchCanFrame();
            }
        }
        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &rxFrame) == MCP2515::ERROR_OK) {
                statRxFramesOk++;
                slaveDispatchCanFrame();
            }
        }
    }

    slaveWokwiSimulateSerial();
    slavePollMcpErrors();
    slaveMotionProcessHomeSwitch();
    slaveMotionRunStepper();

    uint32_t now = millis();
    if (now - lastStatusTx >= (uint32_t)cfgStatusIntervalMs) {
        lastStatusTx = now;
        slaveSendStatusFrame();
    }

    slaveUpdateStatusLed();
}

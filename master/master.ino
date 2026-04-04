// =============================================================================
// CAN stepper master — entry point (modular .cpp in this folder)
// =============================================================================

#include <string.h>
#include <SPI.h>
#include "master_config.h"
#include "master_globals.h"
#include "master_can_tx.h"
#include "master_can_rx.h"
#include "master_serial_ui.h"
#include "master_led.h"

void canISR() {
    canInterruptFlag = true;
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("=== CAN STEPPER MASTER v2 (modular) ==="));

    memset(nodes, 0, sizeof(nodes));
    for (int i = 1; i <= 8; i++) {
        nodes[i].state = CSP_STATE_IDLE;
    }

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
    pinMode(CAN_INT_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);

    SPI.begin();

    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 reset FAILED"));
        while (true) {
            digitalWrite(STATUS_LED, HIGH);
            delay(80);
            digitalWrite(STATUS_LED, LOW);
            delay(80);
        }
    }

    if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
        Serial.println(F("MCP2515 setBitrate FAILED"));
        while (true) {
            digitalWrite(STATUS_LED, HIGH);
            delay(200);
            digitalWrite(STATUS_LED, LOW);
            delay(200);
        }
    }

    mcp2515.setNormalMode();

    Serial.println(F("CAN 500 kbit/s | target: 0=broadcast 1-8=node"));
    masterPrintMenu();
}

void loop() {
    if (canInterruptFlag) {
        canInterruptFlag = false;

        uint8_t irq = mcp2515.getInterrupts();

        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &rxFrame) == MCP2515::ERROR_OK) {
                masterHandleRxFrame();
            }
        }
        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &rxFrame) == MCP2515::ERROR_OK) {
                masterHandleRxFrame();
            }
        }
    }

    uint32_t now = millis();
    if (now - lastHeartbeatTx >= HEARTBEAT_PERIOD_MS) {
        lastHeartbeatTx = now;
        masterSendHeartbeat();
    }

    if (Serial.available()) {
        char c = (char)Serial.read();
        if (c != '\n' && c != '\r') {
            masterHandleSerialCmd(c);
        }
    }

    static uint32_t lastStaleWarn = 0;
    bool anyFresh = false;
    for (int i = 1; i <= 8; i++) {
        if (nodes[i].lastMs != 0 && (now - nodes[i].lastMs) < 1500) {
            anyFresh = true;
            break;
        }
    }
    if (!anyFresh && now > 2000) {
        if (now - lastStaleWarn > 4000) {
            lastStaleWarn = now;
            Serial.println(F("WARNING: no STATUS from any node (1.5s window)"));
        }
    }

    masterUpdateLed();
}

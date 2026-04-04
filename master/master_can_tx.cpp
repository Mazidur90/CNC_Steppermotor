#include <string.h>
#include "master_can_tx.h"
#include "master_globals.h"
#include "master_util.h"

bool masterSendCanFrame(struct can_frame* frame) {
    for (int attempt = 0; attempt < CAN_TX_RETRY_COUNT; attempt++) {
        MCP2515::ERROR err = mcp2515.sendMessage(frame);
        if (err == MCP2515::ERROR_OK) {
            return true;
        }
        Serial.print(F("TX retry "));
        Serial.println((int)err);
        delay(CAN_TX_RETRY_DELAY);
    }
    Serial.println(F("TX FAILED"));
    return false;
}

void masterSendHeartbeat() {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_HEARTBEAT;
    txFrame.can_dlc = CSP_DLC;
    csp_build_heartbeat(txFrame.data, millis());
    mcp2515.sendMessage(&txFrame);
}

void masterSendMotorCmd(uint8_t dir, uint16_t steps, uint16_t rpm, uint8_t accelFactor) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_MOTOR_CMD;
    txFrame.can_dlc = CSP_DLC;
    uint8_t r8 = masterRpmToU8(rpm);
    if (!csp_build_motor_cmd(txFrame.data, gTargetNode, dir, steps, r8, accelFactor)) {
        Serial.println(F("MOTOR_CMD invalid (steps/rpm)"));
        return;
    }
    if (masterSendCanFrame(&txFrame)) {
        lastTxTime = millis();
    }
}

void masterSendStopCmd(uint8_t stopType) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_STOP;
    txFrame.can_dlc = CSP_DLC;
    csp_build_stop(txFrame.data, gTargetNode, stopType);
    masterSendCanFrame(&txFrame);
    lastTxTime = millis();
}

void masterSendHomeCmd(uint8_t dir, uint8_t speedPct) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_HOME;
    txFrame.can_dlc = CSP_DLC;
    csp_build_home(txFrame.data, gTargetNode, dir, speedPct);
    masterSendCanFrame(&txFrame);
    lastTxTime = millis();
}

void masterSendJog(uint8_t dir, uint8_t speedPct, uint8_t enable) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_JOG;
    txFrame.can_dlc = CSP_DLC;
    csp_build_jog(txFrame.data, gTargetNode, dir, speedPct, enable);
    masterSendCanFrame(&txFrame);
    lastTxTime = millis();
}

void masterSendDigitalOut(uint8_t mask, uint8_t value) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_DIGITAL_OUT;
    txFrame.can_dlc = CSP_DLC;
    csp_build_digital_out(txFrame.data, gTargetNode, mask, value);
    masterSendCanFrame(&txFrame);
    lastTxTime = millis();
}

void masterSendConfigGet(uint8_t key) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_CONFIG_GET;
    txFrame.can_dlc = CSP_DLC;
    csp_build_config_get(txFrame.data, gTargetNode, key);
    masterSendCanFrame(&txFrame);
}

void masterSendConfigSet(uint8_t key, uint32_t value) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_CONFIG_SET;
    txFrame.can_dlc = CSP_DLC;
    csp_build_config_set(txFrame.data, gTargetNode, key, value);
    masterSendCanFrame(&txFrame);
}

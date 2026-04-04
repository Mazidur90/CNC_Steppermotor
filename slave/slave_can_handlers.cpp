#include <math.h>
#include <string.h>
#include "slave_can_handlers.h"
#include "slave_eeprom.h"
#include "slave_globals.h"
#include "slave_peripherals.h"
#include "slave_config.h"

#ifndef CSP_VERBOSE
#define CSP_VERBOSE 0
#endif

static void sendConfigResp(uint8_t key, uint32_t value) {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_CONFIG_RESP;
    txFrame.can_dlc = CSP_DLC;
    csp_build_config_resp(txFrame.data, myNodeId, key, value);
    mcp2515.sendMessage(&txFrame);
}

void slaveDispatchCanFrame() {
#if CSP_VERBOSE
    Serial.print(F("RX 0x"));
    Serial.println(rxFrame.can_id, HEX);
#endif
    if (rxFrame.can_dlc < CSP_DLC) {
        return;
    }

    switch (rxFrame.can_id) {
        case CSP_CAN_ID_MOTOR_CMD: {
            uint8_t tgt, dir;
            uint16_t steps;
            uint8_t rpm8, accelF;
            if (!csp_parse_motor_cmd(rxFrame.data, &tgt, &dir, &steps, &rpm8, &accelF)) {
                rxCrcRejects++;
                return;
            }
            if (!csp_target_ok(tgt, myNodeId)) {
                return;
            }
            if (cfgInvertDir) {
                dir ^= 1u;
            }
            float stepsPerSec = (float)rpm8 * (float)STEPS_PER_REV_MICRO / 60.0f;
            stepsPerSec = constrain(stepsPerSec, 10.0f, (float)cfgMaxSpeedSteps);
            float accelVal = (float)accelF * 10.0f;
            accelVal = constrain(accelVal, 50.0f, 10000.0f);
            stepper.setMaxSpeed(stepsPerSec);
            stepper.setAcceleration(accelVal);
            digitalWrite(EN_PIN, LOW);
            int32_t delta = (dir == 1) ? (int32_t)steps : -(int32_t)steps;
            stepper.move(delta);
            currentState = STATE_RUNNING;
            errorCode    = CSP_ERR_NONE;
            break;
        }
        case CSP_CAN_ID_STOP: {
            uint8_t tgt, stopType;
            if (!csp_parse_stop(rxFrame.data, &tgt, &stopType)) {
                rxCrcRejects++;
                return;
            }
            if (!csp_target_ok(tgt, myNodeId)) {
                return;
            }
            if (stopType != CSP_STOP_IMMEDIATE && stopType != CSP_STOP_DECELERATE) {
                return;
            }
            stepper.setSpeed(0);
            if (stopType == CSP_STOP_IMMEDIATE) {
                stepper.setCurrentPosition(stepper.currentPosition());
                currentState = STATE_STOPPED;
            } else {
                stepper.stop();
                currentState = STATE_STOPPED;
            }
            break;
        }
        case CSP_CAN_ID_HOME: {
            uint8_t tgt, homeDir, homePct;
            if (!csp_parse_home(rxFrame.data, &tgt, &homeDir, &homePct)) {
                rxCrcRejects++;
                return;
            }
            if (!csp_target_ok(tgt, myNodeId)) {
                return;
            }
            homePct = constrain(homePct, 1, 100);
            float homeSpeed = (float)cfgMaxSpeedSteps * ((float)homePct / 100.0f);
            homeSpeed = constrain(homeSpeed, 50.0f, 1000.0f);
            stepper.setMaxSpeed(homeSpeed);
            stepper.setAcceleration(200.0f);
            digitalWrite(EN_PIN, LOW);
            int32_t bigMove = (homeDir == 0) ? -100000L : 100000L;
            if (cfgInvertDir) {
                bigMove = -bigMove;
            }
            homingSeekBigMove = bigMove;
            stepper.move(bigMove);
            currentState        = STATE_HOMING;
            homePhase           = HP_FAST;
            homeSwitchTriggered = false;
            errorCode           = CSP_ERR_NONE;
            break;
        }
        case CSP_CAN_ID_CONFIG_SET: {
            uint8_t tgt, key;
            uint32_t val;
            if (!csp_parse_config_set(rxFrame.data, &tgt, &key, &val)) {
                rxCrcRejects++;
                return;
            }
            if (!csp_target_ok(tgt, myNodeId)) {
                return;
            }
            switch (key) {
                case CSP_CFG_NODE_ID:
                    if (val >= 1 && val <= 32) {
                        myNodeId = (uint8_t)val;
                        slaveEepromSave();
                    }
                    break;
                case CSP_CFG_MAX_SPEED_STEPS:
                    if (val >= 100 && val <= 20000) {
                        cfgMaxSpeedSteps = (uint16_t)val;
                        stepper.setMaxSpeed((float)cfgMaxSpeedSteps);
                        slaveEepromSave();
                    }
                    break;
                case CSP_CFG_INVERT_DIR:
                    cfgInvertDir = val ? 1u : 0u;
                    slaveEepromSave();
                    break;
                case CSP_CFG_HEARTBEAT_MS:
                    cfgHeartbeatMs = (uint16_t)val;
                    slaveEepromSave();
                    break;
                case CSP_CFG_STATUS_MS:
                    if (val >= 20 && val <= 2000) {
                        cfgStatusIntervalMs = (uint16_t)val;
                        slaveEepromSave();
                    }
                    break;
                default:
                    break;
            }
            sendConfigResp(key, val);
            break;
        }
        case CSP_CAN_ID_CONFIG_GET: {
            uint8_t tgt, key;
            if (!csp_parse_config_get(rxFrame.data, &tgt, &key)) {
                rxCrcRejects++;
                return;
            }
            if (!csp_target_ok(tgt, myNodeId)) {
                return;
            }
            uint32_t v = 0;
            switch (key) {
                case CSP_CFG_NODE_ID:
                    v = myNodeId;
                    break;
                case CSP_CFG_MAX_SPEED_STEPS:
                    v = cfgMaxSpeedSteps;
                    break;
                case CSP_CFG_INVERT_DIR:
                    v = cfgInvertDir;
                    break;
                case CSP_CFG_HEARTBEAT_MS:
                    v = cfgHeartbeatMs;
                    break;
                case CSP_CFG_STATUS_MS:
                    v = cfgStatusIntervalMs;
                    break;
                default:
                    break;
            }
            sendConfigResp(key, v);
            break;
        }
        case CSP_CAN_ID_HEARTBEAT: {
            uint32_t ms;
            if (!csp_parse_heartbeat(rxFrame.data, &ms)) {
                return;
            }
            (void)ms;
            lastMasterHbMs = millis();
            break;
        }
        case CSP_CAN_ID_JOG: {
            uint8_t tgt, dir, pct, en;
            if (!csp_parse_jog(rxFrame.data, &tgt, &dir, &pct, &en)) {
                rxCrcRejects++;
                return;
            }
            if (!csp_target_ok(tgt, myNodeId)) {
                return;
            }
            if (cfgInvertDir) {
                dir ^= 1u;
            }
            if (!en) {
                stepper.setSpeed(0);
                stepper.stop();
                currentState = STATE_IDLE;
                break;
            }
            pct = constrain(pct, 1, 100);
            float js = (float)cfgMaxSpeedSteps * ((float)pct / 100.0f);
            js = constrain(js, 20.0f, (float)cfgMaxSpeedSteps);
            float s = (dir == 1) ? js : -js;
            stepper.setMaxSpeed(js);
            stepper.setSpeed(s);
            digitalWrite(EN_PIN, LOW);
            currentState = STATE_JOGGING;
            errorCode    = CSP_ERR_NONE;
            break;
        }
        case CSP_CAN_ID_DIGITAL_OUT: {
            uint8_t tgt, mask, val;
            if (!csp_parse_digital_out(rxFrame.data, &tgt, &mask, &val)) {
                rxCrcRejects++;
                return;
            }
            if (!csp_target_ok(tgt, myNodeId)) {
                return;
            }
            doutState = (uint8_t)((doutState & ~mask) | (val & mask));
            slaveApplyDoutPins();
            break;
        }
        default:
            break;
    }
}

void slaveSendStatusFrame() {
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.can_id  = CSP_CAN_ID_STATUS;
    txFrame.can_dlc = CSP_DLC;

    int32_t pos = stepper.currentPosition();
    uint16_t speed_x10 = (uint16_t)(fabs(stepper.speed()) * 10.0f);
    csp_build_status(txFrame.data, (uint8_t)currentState, myNodeId, pos, errorCode, speed_x10);

    MCP2515::ERROR err = mcp2515.sendMessage(&txFrame);
    if (err == MCP2515::ERROR_OK) {
        statTxStatus++;
    } else {
        static uint32_t lastTxErrPrint = 0;
        if (millis() - lastTxErrPrint > 2000) {
            lastTxErrPrint = millis();
            Serial.print(F("STATUS TX err="));
            Serial.println((int)err);
        }
    }
}

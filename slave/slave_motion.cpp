#include "slave_motion.h"
#include "slave_globals.h"
#include "slave_config.h"
#include "slave_peripherals.h"

void slaveMotionSupervisionTick() {
    if (cfgHeartbeatMs != 0 && currentState != STATE_FAULT && lastMasterHbMs != 0 &&
        (millis() - lastMasterHbMs > (uint32_t)cfgHeartbeatMs)) {
        stepper.stop();
        currentState = STATE_FAULT;
        errorCode    = CSP_ERR_HEARTBEAT_LOST;
        lastMasterHbMs = 0;
        Serial.println(F("FAULT: heartbeat timeout"));
    }
}

void slaveMotionProcessHomeSwitch() {
    if (!homeSwitchTriggered || currentState != STATE_HOMING) {
        return;
    }
    homeSwitchTriggered = false;
    if (homePhase == HP_FAST) {
        stepper.stop();
        int32_t away = (homingSeekBigMove < 0) ? (int32_t)HOMING_BACKOFF_STEPS : -(int32_t)HOMING_BACKOFF_STEPS;
        stepper.move(away);
        homePhase = HP_BACKOFF;
    } else if (homePhase == HP_SLOW) {
        stepper.stop();
        stepper.setCurrentPosition(0);
        currentState = STATE_IDLE;
        errorCode    = CSP_ERR_NONE;
        homePhase    = HP_FAST;
        Serial.println(F("Home complete (creep)"));
    }
}

void slaveMotionCheckLimits() {
    if (currentState != STATE_RUNNING && currentState != STATE_HOMING && currentState != STATE_JOGGING) {
        return;
    }
    if (slaveLimitMinTripped()) {
        stepper.stop();
        stepper.setSpeed(0);
        currentState = STATE_FAULT;
        errorCode    = CSP_ERR_LIMIT_MIN;
        Serial.println(F("LIMIT MIN"));
    } else if (slaveLimitMaxTripped()) {
        stepper.stop();
        stepper.setSpeed(0);
        currentState = STATE_FAULT;
        errorCode    = CSP_ERR_LIMIT_MAX;
        Serial.println(F("LIMIT MAX"));
    }
}

void slaveMotionRunStepper() {
    if (currentState == STATE_RUNNING || currentState == STATE_HOMING) {
        stepper.run();
        slaveMotionCheckLimits();

        if (currentState == STATE_HOMING && homePhase == HP_BACKOFF && stepper.distanceToGo() == 0) {
            float slow = (float)cfgMaxSpeedSteps * 0.08f;
            slow = constrain(slow, 30.0f, 400.0f);
            stepper.setMaxSpeed(slow);
            stepper.setAcceleration(120.0f);
            int32_t creep = (homingSeekBigMove < 0) ? -50000L : 50000L;
            stepper.move(creep);
            homePhase = HP_SLOW;
        }

        if (stepper.distanceToGo() == 0 && currentState == STATE_RUNNING) {
            currentState = STATE_IDLE;
            errorCode    = CSP_ERR_NONE;
            Serial.print(F("Move done pos="));
            Serial.println(stepper.currentPosition());
        } else if (stepper.distanceToGo() == 0 && currentState == STATE_HOMING && homePhase == HP_FAST) {
            currentState = STATE_FAULT;
            errorCode    = CSP_ERR_HOME_FAIL;
            homePhase    = HP_FAST;
            Serial.println(F("HOMING FAULT: switch not found"));
        }
    } else if (currentState == STATE_JOGGING) {
        stepper.runSpeed();
        slaveMotionCheckLimits();
    }
}

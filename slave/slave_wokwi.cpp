#include <string.h>
#include "slave_wokwi.h"
#include "slave_can_handlers.h"
#include "slave_globals.h"

#ifdef WOKWI_SIMULATION

void slaveWokwiSimulateSerial() {
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
            csp_build_motor_cmd(rxFrame.data, CSP_BROADCAST_ID, 1, 200, 60, 50);
            slaveDispatchCanFrame();
            break;
        case 'b':
            rxFrame.can_id = CSP_CAN_ID_MOTOR_CMD;
            csp_build_motor_cmd(rxFrame.data, CSP_BROADCAST_ID, 0, 200, 60, 50);
            slaveDispatchCanFrame();
            break;
        case 'F':
            rxFrame.can_id = CSP_CAN_ID_MOTOR_CMD;
            csp_build_motor_cmd(rxFrame.data, CSP_BROADCAST_ID, 1, 1000, 120, 80);
            slaveDispatchCanFrame();
            break;
        case 'B':
            rxFrame.can_id = CSP_CAN_ID_MOTOR_CMD;
            csp_build_motor_cmd(rxFrame.data, CSP_BROADCAST_ID, 0, 1000, 120, 80);
            slaveDispatchCanFrame();
            break;
        case 's':
            rxFrame.can_id = CSP_CAN_ID_STOP;
            csp_build_stop(rxFrame.data, CSP_BROADCAST_ID, CSP_STOP_DECELERATE);
            slaveDispatchCanFrame();
            break;
        case 'S':
            rxFrame.can_id = CSP_CAN_ID_STOP;
            csp_build_stop(rxFrame.data, CSP_BROADCAST_ID, CSP_STOP_IMMEDIATE);
            slaveDispatchCanFrame();
            break;
        case 'h':
            rxFrame.can_id = CSP_CAN_ID_HOME;
            csp_build_home(rxFrame.data, CSP_BROADCAST_ID, 0, 20);
            slaveDispatchCanFrame();
            break;
        case 'j':
            rxFrame.can_id = CSP_CAN_ID_JOG;
            csp_build_jog(rxFrame.data, CSP_BROADCAST_ID, 1, 30, 1);
            slaveDispatchCanFrame();
            break;
        case 'k':
            rxFrame.can_id = CSP_CAN_ID_JOG;
            csp_build_jog(rxFrame.data, CSP_BROADCAST_ID, 0, 30, 1);
            slaveDispatchCanFrame();
            break;
        case 'q':
            rxFrame.can_id = CSP_CAN_ID_JOG;
            csp_build_jog(rxFrame.data, CSP_BROADCAST_ID, 0, 1, 0);
            slaveDispatchCanFrame();
            break;
        case '?':
            Serial.print(F("N="));
            Serial.print(myNodeId);
            Serial.print(F(" ST="));
            Serial.print((int)currentState);
            Serial.print(F(" POS="));
            Serial.print(stepper.currentPosition());
            Serial.print(F(" RXok="));
            Serial.print(statRxFramesOk);
            Serial.print(F(" TXst="));
            Serial.print(statTxStatus);
            Serial.print(F(" CRCbad="));
            Serial.println(rxCrcRejects);
            break;
        default:
            break;
    }
}

#else

void slaveWokwiSimulateSerial() {}

#endif

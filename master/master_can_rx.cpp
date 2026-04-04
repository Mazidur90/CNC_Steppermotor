#include "master_can_rx.h"
#include "master_globals.h"

#ifndef CSP_VERBOSE
#define CSP_VERBOSE 0
#endif

void masterHandleRxFrame() {
    if (rxFrame.can_dlc < CSP_DLC) {
        return;
    }

    if (rxFrame.can_id == CSP_CAN_ID_STATUS) {
        uint8_t st, nid, err, spc;
        int32_t pos;
        if (!csp_parse_status(rxFrame.data, &st, &nid, &pos, &err, &spc)) {
            statusCrcDrops++;
            static uint32_t lastBad = 0;
            if (millis() - lastBad > 2000) {
                lastBad = millis();
                Serial.println(F("STATUS CRC error"));
            }
            return;
        }
        if (nid >= 1 && nid <= 8) {
            nodes[nid].state        = st;
            nodes[nid].position     = pos;
            nodes[nid].error        = err;
            nodes[nid].speed_coarse = spc;
            nodes[nid].lastMs       = millis();
        }

        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 200) {
            lastPrint = millis();
            Serial.print(F("N"));
            Serial.print(nid);
            Serial.print(F(" "));
            if (st <= CSP_STATE_JOGGING) {
                Serial.print(gMasterStateNames[st]);
            }
            Serial.print(F(" pos="));
            Serial.print(pos);
            if (err) {
                Serial.print(F(" E=0x"));
                Serial.print(err, HEX);
            }
            Serial.println();
        }
        return;
    }

    if (rxFrame.can_id == CSP_CAN_ID_CONFIG_RESP) {
        uint8_t nid, key;
        uint32_t val;
        if (csp_parse_config_resp(rxFrame.data, &nid, &key, &val)) {
            Serial.print(F("CONFIG node "));
            Serial.print(nid);
            Serial.print(F(" key "));
            Serial.print(key);
            Serial.print(F(" = "));
            Serial.println(val);
        }
        return;
    }

#if CSP_VERBOSE
    Serial.print(F("RX 0x"));
    Serial.println(rxFrame.can_id, HEX);
#endif
}

#include "master_serial_ui.h"
#include "master_presets.h"
#include "master_can_tx.h"
#include "master_globals.h"

void masterPrintStatus() {
    Serial.println(F("--- MASTER ---"));
    Serial.print(F("Target: "));
    Serial.println(gTargetNode == CSP_BROADCAST_ID ? (int)CSP_BROADCAST_ID : gTargetNode);
    Serial.print(F("STATUS CRC drops: "));
    Serial.println(statusCrcDrops);
    for (int i = 1; i <= 8; i++) {
        if (nodes[i].lastMs == 0) {
            continue;
        }
        uint32_t age = millis() - nodes[i].lastMs;
        if (age > 5000) {
            continue;
        }
        Serial.print(F("  Node "));
        Serial.print(i);
        Serial.print(F(": "));
        if (nodes[i].state <= CSP_STATE_JOGGING) {
            Serial.print(gMasterStateNames[nodes[i].state]);
        }
        Serial.print(F(" pos="));
        Serial.print(nodes[i].position);
        Serial.print(F(" ~"));
        Serial.print(nodes[i].speed_coarse);
        Serial.print(F(" st/s err=0x"));
        Serial.print(nodes[i].error, HEX);
        Serial.print(F(" age="));
        Serial.print(age);
        Serial.println(F("ms"));
    }
    Serial.println(F("--------------"));
}

void masterPrintMenu() {
    Serial.println();
    Serial.println(F("=== MENU v2 (modular) ==="));
    Serial.println(F("0=broadcast  1-8=target node"));
    Serial.println(F("f b F B x X  motion presets | s S stop | h H home"));
    Serial.println(F("j k jog +/-  q jog stop"));
    Serial.println(F("y spindle  c coolant"));
    Serial.println(F("v CONFIG read  n hb=3000ms  N hb=OFF"));
    Serial.println(F("? status  m menu"));
    Serial.println(F("========================="));
}

void masterHandleSerialCmd(char c) {
    if (c >= '1' && c <= '8') {
        gTargetNode = (uint8_t)(c - '0');
        Serial.print(F("Target node "));
        Serial.println(gTargetNode);
        return;
    }
    if (c == '0') {
        gTargetNode = CSP_BROADCAST_ID;
        Serial.println(F("Target BROADCAST"));
        return;
    }

    if (masterPresetTry(c)) {
        return;
    }

    switch (c) {
        case 's':
            masterSendStopCmd(CSP_STOP_DECELERATE);
            break;
        case 'S':
            masterSendStopCmd(CSP_STOP_IMMEDIATE);
            break;
        case 'h':
            masterSendHomeCmd(0, 20);
            break;
        case 'H':
            masterSendHomeCmd(1, 20);
            break;
        case 'j':
            Serial.println(F("Jog +"));
            masterSendJog(1, 35, 1);
            break;
        case 'k':
            Serial.println(F("Jog -"));
            masterSendJog(0, 35, 1);
            break;
        case 'q':
            masterSendJog(0, 1, 0);
            Serial.println(F("Jog stop"));
            break;
        case 'y':
            doutShadow ^= CSP_DOUT_SPINDLE;
            masterSendDigitalOut(CSP_DOUT_SPINDLE, doutShadow & CSP_DOUT_SPINDLE);
            Serial.print(F("Spindle "));
            Serial.println((doutShadow & CSP_DOUT_SPINDLE) ? F("ON") : F("OFF"));
            break;
        case 'c':
            doutShadow ^= CSP_DOUT_COOLANT;
            masterSendDigitalOut(CSP_DOUT_COOLANT, doutShadow & CSP_DOUT_COOLANT);
            Serial.print(F("Coolant "));
            Serial.println((doutShadow & CSP_DOUT_COOLANT) ? F("ON") : F("OFF"));
            break;
        case 'v':
            masterSendConfigGet(CSP_CFG_NODE_ID);
            masterSendConfigGet(CSP_CFG_MAX_SPEED_STEPS);
            masterSendConfigGet(CSP_CFG_HEARTBEAT_MS);
            Serial.println(F("CONFIG_GET sent (node, maxSpd, hbMs)"));
            break;
        case 'n':
            masterSendConfigSet(CSP_CFG_HEARTBEAT_MS, 3000);
            Serial.println(F("CONFIG: heartbeat 3000 ms"));
            break;
        case 'N':
            masterSendConfigSet(CSP_CFG_HEARTBEAT_MS, 0);
            Serial.println(F("CONFIG: heartbeat OFF"));
            break;
        case '?':
            masterPrintStatus();
            break;
        case 'm':
            masterPrintMenu();
            break;
        default:
            Serial.print(F("Unknown: "));
            Serial.println(c);
            Serial.println(F("'m' menu"));
            break;
    }
}

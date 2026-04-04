#include <EEPROM.h>
#include "slave_eeprom.h"
#include "slave_globals.h"
#include "slave_config.h"

void slaveEepromLoad() {
    uint8_t a = EEPROM.read(EEPROM_BASE);
    uint8_t b = EEPROM.read(EEPROM_BASE + 1);
    if (a != EEPROM_MAGIC_A || b != EEPROM_MAGIC_B) {
        myNodeId = DEFAULT_NODE_ID;
        cfgMaxSpeedSteps = DEFAULT_MAX_SPEED_STEPS;
        cfgInvertDir = 0;
        cfgHeartbeatMs = DEFAULT_HEARTBEAT_MS;
        cfgStatusIntervalMs = DEFAULT_STATUS_MS;
        return;
    }
    myNodeId = EEPROM.read(EEPROM_BASE + 2);
    if (myNodeId == 0 || myNodeId == CSP_BROADCAST_ID) {
        myNodeId = DEFAULT_NODE_ID;
    }
    cfgMaxSpeedSteps = ((uint16_t)EEPROM.read(EEPROM_BASE + 3) << 8) | EEPROM.read(EEPROM_BASE + 4);
    if (cfgMaxSpeedSteps < 100) {
        cfgMaxSpeedSteps = DEFAULT_MAX_SPEED_STEPS;
    }
    cfgInvertDir = EEPROM.read(EEPROM_BASE + 5) ? 1u : 0u;
    cfgHeartbeatMs = ((uint16_t)EEPROM.read(EEPROM_BASE + 6) << 8) | EEPROM.read(EEPROM_BASE + 7);
    cfgStatusIntervalMs = ((uint16_t)EEPROM.read(EEPROM_BASE + 8) << 8) | EEPROM.read(EEPROM_BASE + 9);
    if (cfgStatusIntervalMs < 20) {
        cfgStatusIntervalMs = DEFAULT_STATUS_MS;
    }
}

void slaveEepromSave() {
    EEPROM.update(EEPROM_BASE, EEPROM_MAGIC_A);
    EEPROM.update(EEPROM_BASE + 1, EEPROM_MAGIC_B);
    EEPROM.update(EEPROM_BASE + 2, myNodeId);
    EEPROM.update(EEPROM_BASE + 3, (uint8_t)(cfgMaxSpeedSteps >> 8));
    EEPROM.update(EEPROM_BASE + 4, (uint8_t)(cfgMaxSpeedSteps & 0xFF));
    EEPROM.update(EEPROM_BASE + 5, cfgInvertDir);
    EEPROM.update(EEPROM_BASE + 6, (uint8_t)(cfgHeartbeatMs >> 8));
    EEPROM.update(EEPROM_BASE + 7, (uint8_t)(cfgHeartbeatMs & 0xFF));
    EEPROM.update(EEPROM_BASE + 8, (uint8_t)(cfgStatusIntervalMs >> 8));
    EEPROM.update(EEPROM_BASE + 9, (uint8_t)(cfgStatusIntervalMs & 0xFF));
}

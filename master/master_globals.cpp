#include "master_globals.h"

MCP2515 mcp2515(CAN_CS_PIN);

struct can_frame txFrame;
struct can_frame rxFrame;
volatile bool canInterruptFlag = false;

NodeTel nodes[9];

uint32_t statusCrcDrops = 0;
uint32_t lastLedToggle = 0;
bool     ledState = false;
uint32_t lastTxTime = 0;

uint8_t  gTargetNode = CSP_BROADCAST_ID;
uint8_t  doutShadow = 0;
uint32_t lastHeartbeatTx = 0;

const char* const gMasterStateNames[] = {
    "IDLE", "RUNNING", "HOMING", "FAULT", "STOPPED", "JOGGING",
};

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "../libraries/CANStepperProtocol/CANStepperProtocol.h"
#include "master_config.h"

struct NodeTel {
    uint8_t  state;
    int32_t  position;
    uint8_t  error;
    uint8_t  speed_coarse;
    uint32_t lastMs;
};

extern MCP2515 mcp2515;
extern struct can_frame txFrame;
extern struct can_frame rxFrame;
extern volatile bool canInterruptFlag;

extern NodeTel nodes[9];

extern uint32_t statusCrcDrops;
extern uint32_t lastLedToggle;
extern bool     ledState;
extern uint32_t lastTxTime;

extern uint8_t  gTargetNode;
extern uint8_t  doutShadow;
extern uint32_t lastHeartbeatTx;

extern const char* const gMasterStateNames[];

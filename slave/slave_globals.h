#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <AccelStepper.h>
#include "../libraries/CANStepperProtocol/CANStepperProtocol.h"
#include "slave_config.h"

typedef enum {
    STATE_IDLE    = CSP_STATE_IDLE,
    STATE_RUNNING = CSP_STATE_RUNNING,
    STATE_HOMING  = CSP_STATE_HOMING,
    STATE_FAULT   = CSP_STATE_FAULT,
    STATE_STOPPED = CSP_STATE_STOPPED,
    STATE_JOGGING = CSP_STATE_JOGGING,
} SlaveState;

typedef enum { HP_FAST = 0, HP_BACKOFF, HP_SLOW } HomePhase;

extern MCP2515      mcp2515;
extern AccelStepper stepper;
extern struct can_frame rxFrame;
extern struct can_frame txFrame;

extern volatile bool canInterruptFlag;
extern volatile bool homeSwitchTriggered;

extern SlaveState currentState;
extern uint8_t    errorCode;
extern uint32_t   lastStatusTx;
extern uint32_t   lastLedToggle;
extern bool       ledState;
extern uint32_t   rxCrcRejects;

extern uint8_t  myNodeId;
extern uint16_t cfgMaxSpeedSteps;
extern uint8_t  cfgInvertDir;
extern uint16_t cfgHeartbeatMs;
extern uint16_t cfgStatusIntervalMs;

extern uint32_t lastMasterHbMs;
extern HomePhase homePhase;
extern int32_t   homingSeekBigMove;
extern uint8_t   doutState;

extern uint32_t lastMcpPoll;

extern uint32_t statRxFramesOk;
extern uint32_t statTxStatus;

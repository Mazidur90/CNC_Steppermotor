#include "slave_globals.h"

MCP2515      mcp2515(CAN_CS_PIN);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

struct can_frame rxFrame;
struct can_frame txFrame;

volatile bool canInterruptFlag = false;
volatile bool homeSwitchTriggered = false;

SlaveState currentState = STATE_IDLE;
uint8_t    errorCode    = CSP_ERR_NONE;
uint32_t   lastStatusTx = 0;
uint32_t   lastLedToggle = 0;
bool       ledState = false;
uint32_t   rxCrcRejects = 0;

uint8_t  myNodeId            = DEFAULT_NODE_ID;
uint16_t cfgMaxSpeedSteps    = DEFAULT_MAX_SPEED_STEPS;
uint8_t  cfgInvertDir        = 0;
uint16_t cfgHeartbeatMs      = DEFAULT_HEARTBEAT_MS;
uint16_t cfgStatusIntervalMs = DEFAULT_STATUS_MS;

uint32_t lastMasterHbMs = 0;
HomePhase homePhase = HP_FAST;
int32_t   homingSeekBigMove = 0;
uint8_t   doutState = 0;

uint32_t lastMcpPoll = 0;

uint32_t statRxFramesOk = 0;
uint32_t statTxStatus   = 0;

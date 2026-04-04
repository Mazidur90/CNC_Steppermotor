#pragma once

#ifndef CSP_VERBOSE
#define CSP_VERBOSE 0
#endif

#define CAN_CS_PIN      10
#define CAN_INT_PIN      2
#define STEP_PIN         5
#define DIR_PIN          6
#define EN_PIN           7
#define HOME_PIN         3
#define STATUS_LED      13
#define SPINDLE_PIN      8
#define COOLANT_PIN      9
#define LIMIT_MIN_PIN   A2
#define LIMIT_MAX_PIN   A3

#define STEPS_PER_REV            200
#define MICROSTEPS                 8
#define STEPS_PER_REV_MICRO   (STEPS_PER_REV * MICROSTEPS)

#define EEPROM_MAGIC_A   0xC5
#define EEPROM_MAGIC_B   0x73
#define EEPROM_BASE      0

#define DEFAULT_NODE_ID            1u
#define DEFAULT_MAX_SPEED_STEPS    4000u
#define DEFAULT_HEARTBEAT_MS       0u
#define DEFAULT_STATUS_MS          100u
#define HOMING_BACKOFF_STEPS       400
#define MCP_POLL_MS                400

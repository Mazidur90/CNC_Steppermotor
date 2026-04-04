/**
 * CANStepperProtocol — shared frame layout and integrity (protocol v1)
 *
 * All frames use CAN 2.0A standard ID, DLC = 8.
 * Byte 7 is always CRC-8 over bytes 0..6 (poly 0x07, init 0, MSB-first, no xorout).
 *
 * 0x100 MOTOR_CMD
 *   [0] 0x01 = move (relative microsteps)
 *   [1] direction: 0 = negative, 1 = positive
 *   [2..3] steps (big-endian uint16)
 *   [4..5] RPM (big-endian uint16)
 *   [6] accel factor (slave maps to steps/s²)
 *   [7] crc8(0..6)
 *
 * 0x101 STOP
 *   [0] 0x00 = immediate, 0x01 = decelerate
 *   [1..6] 0x00
 *   [7] crc8(0..6)
 *
 * 0x102 HOME
 *   [0] home direction (0/1)
 *   [1] speed percent 1..100
 *   [2..6] 0x00
 *   [7] crc8(0..6)
 *
 * 0x200 STATUS (slave → master)
 *   [0] SlaveState
 *   [1..3] position signed int24 big-endian (range ±8_388_607 microsteps)
 *   [4] error code
 *   [5..6] speed × 10 (big-endian uint16, 0.1 steps/s)
 *   [7] crc8(0..6)
 */
#ifndef CAN_STEPPER_PROTOCOL_H
#define CAN_STEPPER_PROTOCOL_H

#include <Arduino.h>
#include <string.h>

#define CSP_CAN_ID_MOTOR_CMD 0x100u
#define CSP_CAN_ID_STOP      0x101u
#define CSP_CAN_ID_HOME      0x102u
#define CSP_CAN_ID_STATUS    0x200u

#define CSP_CMD_MOVE 0x01u

#define CSP_STOP_IMMEDIATE    0x00u
#define CSP_STOP_DECELERATE   0x01u

#define CSP_DLC 8u

/** Slave error codes (application layer, byte 4 of STATUS) */
enum CSP_SlaveError : uint8_t {
    CSP_ERR_NONE        = 0x00,
    CSP_ERR_HOME_FAIL   = 0x01,
    CSP_ERR_MCP_RESET   = 0xFF,
    CSP_ERR_MCP_BITRATE = 0xFE,
};

enum CSP_SlaveState : uint8_t {
    CSP_STATE_IDLE    = 0,
    CSP_STATE_RUNNING = 1,
    CSP_STATE_HOMING  = 2,
    CSP_STATE_FAULT   = 3,
    CSP_STATE_STOPPED = 4,
};

static inline uint8_t csp_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static inline void csp_pack_u16(uint8_t *buf, uint8_t off, uint16_t v) {
    buf[off]     = (uint8_t)((v >> 8) & 0xFFu);
    buf[off + 1] = (uint8_t)(v & 0xFFu);
}

static inline uint16_t csp_unpack_u16(const uint8_t *buf, uint8_t off) {
    return (uint16_t)(((uint16_t)buf[off] << 8) | buf[off + 1]);
}

static inline void csp_pack_i24(uint8_t *buf, uint8_t off, int32_t v) {
    if (v > 0x7FFFFFL) v = 0x7FFFFFL;
    if (v < -0x800000L) v = -0x800000L;
    buf[off]     = (uint8_t)((v >> 16) & 0xFFu);
    buf[off + 1] = (uint8_t)((v >> 8) & 0xFFu);
    buf[off + 2] = (uint8_t)(v & 0xFFu);
}

static inline int32_t csp_unpack_i24(const uint8_t *buf, uint8_t off) {
    int32_t v =
        ((int32_t)(int8_t)buf[off] << 16) | ((uint32_t)buf[off + 1] << 8) | (uint32_t)buf[off + 2];
    return v;
}

static inline bool csp_crc_ok(const uint8_t d[CSP_DLC]) {
    return csp_crc8(d, CSP_DLC - 1) == d[CSP_DLC - 1];
}

/** Build MOTOR_CMD in buf; returns false if steps or rpm are zero (invalid move). */
static inline bool csp_build_motor_cmd(uint8_t buf[CSP_DLC], uint8_t dir, uint16_t steps,
                                       uint16_t rpm, uint8_t accel_factor) {
    if (steps == 0 || rpm == 0) return false;
    memset(buf, 0, CSP_DLC);
    buf[0] = CSP_CMD_MOVE;
    buf[1] = dir & 1u;
    csp_pack_u16(buf, 2, steps);
    csp_pack_u16(buf, 4, rpm);
    buf[6] = accel_factor;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
    return true;
}

static inline bool csp_parse_motor_cmd(const uint8_t buf[CSP_DLC], uint8_t *dir, uint16_t *steps,
                                       uint16_t *rpm, uint8_t *accel_factor) {
    if (buf[0] != CSP_CMD_MOVE) return false;
    if (!csp_crc_ok(buf)) return false;
    *dir = buf[1] & 1u;
    *steps = csp_unpack_u16(buf, 2);
    *rpm = csp_unpack_u16(buf, 4);
    *accel_factor = buf[6];
    return (*steps != 0 && *rpm != 0);
}

static inline void csp_build_stop(uint8_t buf[CSP_DLC], uint8_t stop_type) {
    memset(buf, 0, CSP_DLC);
    buf[0] = stop_type;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_stop(const uint8_t buf[CSP_DLC], uint8_t *stop_type) {
    if (!csp_crc_ok(buf)) return false;
    *stop_type = buf[0];
    return true;
}

static inline void csp_build_home(uint8_t buf[CSP_DLC], uint8_t home_dir, uint8_t speed_pct) {
    memset(buf, 0, CSP_DLC);
    buf[0] = home_dir & 1u;
    buf[1] = speed_pct;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_home(const uint8_t buf[CSP_DLC], uint8_t *home_dir,
                                  uint8_t *speed_pct) {
    if (!csp_crc_ok(buf)) return false;
    *home_dir = buf[0] & 1u;
    *speed_pct = buf[1];
    return true;
}

static inline void csp_build_status(uint8_t buf[CSP_DLC], uint8_t state, int32_t position,
                                    uint8_t error_code, uint16_t speed_x10) {
    memset(buf, 0, CSP_DLC);
    buf[0] = state;
    csp_pack_i24(buf, 1, position);
    buf[4] = error_code;
    csp_pack_u16(buf, 5, speed_x10);
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_status(const uint8_t buf[CSP_DLC], uint8_t *state, int32_t *position,
                                    uint8_t *error_code, uint16_t *speed_x10) {
    if (!csp_crc_ok(buf)) return false;
    *state = buf[0];
    *position = csp_unpack_i24(buf, 1);
    *error_code = buf[4];
    *speed_x10 = csp_unpack_u16(buf, 5);
    return true;
}

#endif

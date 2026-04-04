/**
 * CANStepperProtocol v2 — multi-node CAN stepper / I/O
 *
 * DLC = 8 for all frames. Byte 7 = CRC-8 over bytes 0..6
 * (poly 0x07, init 0, MSB-first, no xorout).
 *
 * Addressing: byte [0] = target node ID. 0xFF = broadcast (all nodes act).
 *
 * --- 0x100 MOTOR_CMD (relative microsteps) ---
 *   [0] target_node
 *   [1] 0x01 = move
 *   [2] direction 0/1
 *   [3..4] steps uint16 BE
 *   [5] rpm uint8 (0..255; maps to speed at slave)
 *   [6] accel_factor (slave: ×10 steps/s² typical)
 *   [7] crc8(0..6)
 *
 * --- 0x101 STOP ---
 *   [0] target_node
 *   [1] 0x00 immediate, 0x01 decelerate
 *   [2..6] 0x00
 *   [7] crc8(0..6)
 *
 * --- 0x102 HOME ---
 *   [0] target_node
 *   [1] home_dir 0/1
 *   [2] speed_pct 1..100
 *   [3..6] 0x00
 *   [7] crc8(0..6)
 *
 * --- 0x103 CONFIG_SET ---
 *   [0] target_node
 *   [1] key (see CSP_CFG_*)
 *   [2..5] value uint32 BE
 *   [6] 0x00
 *   [7] crc8(0..6)
 *
 * --- 0x104 CONFIG_GET (request) ---
 *   [0] target_node
 *   [1] key
 *   [2..6] 0x00
 *   [7] crc8(0..6)
 *
 * --- 0x105 HEARTBEAT (master → slaves) ---
 *   [0..3] master_uptime_ms uint32 BE
 *   [4..6] 0x00
 *   [7] crc8(0..6)
 *
 * --- 0x106 JOG ---
 *   [0] target_node
 *   [1] dir 0/1
 *   [2] speed_pct 1..100
 *   [3] 0 = stop jog, 1 = run jog
 *   [4..6] 0x00
 *   [7] crc8(0..6)
 *
 * --- 0x107 DIGITAL_OUT ---
 *   [0] target_node
 *   [1] mask (bit0 spindle, bit1 coolant, …)
 *   [2] value (bits applied where mask=1)
 *   [3..6] 0x00
 *   [7] crc8(0..6)
 *
 * --- 0x200 STATUS (slave → master) ---
 *   [0] state (CSP_SlaveState)
 *   [1] node_id
 *   [2..4] position int24 BE
 *   [5] error
 *   [6] speed_coarse = min(255, speed_x10 / 10)  (0.1 steps/s lost in quantisation)
 *   [7] crc8(0..6)
 *
 * --- 0x201 CONFIG_RESP (slave → master) ---
 *   [0] node_id
 *   [1] key
 *   [2..5] value uint32 BE
 *   [6] 0x00
 *   [7] crc8(0..6)
 */
#ifndef CAN_STEPPER_PROTOCOL_H
#define CAN_STEPPER_PROTOCOL_H

#include <Arduino.h>
#include <string.h>

#define CSP_PROTOCOL_VERSION 2u

#define CSP_BROADCAST_ID 0xFFu

#define CSP_CAN_ID_MOTOR_CMD   0x100u
#define CSP_CAN_ID_STOP        0x101u
#define CSP_CAN_ID_HOME        0x102u
#define CSP_CAN_ID_CONFIG_SET  0x103u
#define CSP_CAN_ID_CONFIG_GET  0x104u
#define CSP_CAN_ID_HEARTBEAT   0x105u
#define CSP_CAN_ID_JOG         0x106u
#define CSP_CAN_ID_DIGITAL_OUT 0x107u
#define CSP_CAN_ID_STATUS      0x200u
#define CSP_CAN_ID_CONFIG_RESP 0x201u

#define CSP_CMD_MOVE 0x01u

#define CSP_STOP_IMMEDIATE   0x00u
#define CSP_STOP_DECELERATE  0x01u

#define CSP_DLC 8u

#define CSP_CFG_NODE_ID        1u
#define CSP_CFG_MAX_SPEED_STEPS 2u
#define CSP_CFG_INVERT_DIR     3u
#define CSP_CFG_HEARTBEAT_MS   4u
#define CSP_CFG_STATUS_MS      5u

#define CSP_DOUT_SPINDLE 0x01u
#define CSP_DOUT_COOLANT 0x02u

enum CSP_SlaveError : uint8_t {
    CSP_ERR_NONE            = 0x00,
    CSP_ERR_HOME_FAIL       = 0x01,
    CSP_ERR_LIMIT_MIN       = 0x02,
    CSP_ERR_LIMIT_MAX       = 0x03,
    CSP_ERR_HEARTBEAT_LOST   = 0x04,
    CSP_ERR_MCP_RESET       = 0xFF,
    CSP_ERR_MCP_BITRATE     = 0xFE,
};

enum CSP_SlaveState : uint8_t {
    CSP_STATE_IDLE    = 0,
    CSP_STATE_RUNNING = 1,
    CSP_STATE_HOMING  = 2,
    CSP_STATE_FAULT   = 3,
    CSP_STATE_STOPPED = 4,
    CSP_STATE_JOGGING = 5,
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

static inline void csp_pack_u32(uint8_t *buf, uint8_t off, uint32_t v) {
    buf[off]     = (uint8_t)((v >> 24) & 0xFFu);
    buf[off + 1] = (uint8_t)((v >> 16) & 0xFFu);
    buf[off + 2] = (uint8_t)((v >> 8) & 0xFFu);
    buf[off + 3] = (uint8_t)(v & 0xFFu);
}

static inline uint32_t csp_unpack_u32(const uint8_t *buf, uint8_t off) {
    return ((uint32_t)buf[off] << 24) | ((uint32_t)buf[off + 1] << 16) |
           ((uint32_t)buf[off + 2] << 8) | (uint32_t)buf[off + 3];
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

static inline bool csp_target_ok(uint8_t target, uint8_t my_node) {
    return (target == CSP_BROADCAST_ID) || (target == my_node);
}

static inline bool csp_build_motor_cmd(uint8_t buf[CSP_DLC], uint8_t target_node, uint8_t dir,
                                       uint16_t steps, uint8_t rpm8, uint8_t accel_factor) {
    if (steps == 0 || rpm8 == 0) return false;
    memset(buf, 0, CSP_DLC);
    buf[0] = target_node;
    buf[1] = CSP_CMD_MOVE;
    buf[2] = dir & 1u;
    csp_pack_u16(buf, 3, steps);
    buf[5] = rpm8;
    buf[6] = accel_factor;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
    return true;
}

static inline bool csp_parse_motor_cmd(const uint8_t buf[CSP_DLC], uint8_t *target_node,
                                       uint8_t *dir, uint16_t *steps, uint8_t *rpm8,
                                       uint8_t *accel_factor) {
    if (buf[1] != CSP_CMD_MOVE) return false;
    if (!csp_crc_ok(buf)) return false;
    *target_node = buf[0];
    *dir = buf[2] & 1u;
    *steps = csp_unpack_u16(buf, 3);
    *rpm8 = buf[5];
    *accel_factor = buf[6];
    return (*steps != 0 && *rpm8 != 0);
}

static inline void csp_build_stop(uint8_t buf[CSP_DLC], uint8_t target_node, uint8_t stop_type) {
    memset(buf, 0, CSP_DLC);
    buf[0] = target_node;
    buf[1] = stop_type;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_stop(const uint8_t buf[CSP_DLC], uint8_t *target_node,
                                  uint8_t *stop_type) {
    if (!csp_crc_ok(buf)) return false;
    *target_node = buf[0];
    *stop_type = buf[1];
    return true;
}

static inline void csp_build_home(uint8_t buf[CSP_DLC], uint8_t target_node, uint8_t home_dir,
                                  uint8_t speed_pct) {
    memset(buf, 0, CSP_DLC);
    buf[0] = target_node;
    buf[1] = home_dir & 1u;
    buf[2] = speed_pct;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_home(const uint8_t buf[CSP_DLC], uint8_t *target_node,
                                  uint8_t *home_dir, uint8_t *speed_pct) {
    if (!csp_crc_ok(buf)) return false;
    *target_node = buf[0];
    *home_dir = buf[1] & 1u;
    *speed_pct = buf[2];
    return true;
}

static inline void csp_build_config_set(uint8_t buf[CSP_DLC], uint8_t target_node, uint8_t key,
                                        uint32_t value) {
    memset(buf, 0, CSP_DLC);
    buf[0] = target_node;
    buf[1] = key;
    csp_pack_u32(buf, 2, value);
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_config_set(const uint8_t buf[CSP_DLC], uint8_t *target_node,
                                        uint8_t *key, uint32_t *value) {
    if (!csp_crc_ok(buf)) return false;
    *target_node = buf[0];
    *key = buf[1];
    *value = csp_unpack_u32(buf, 2);
    return true;
}

static inline void csp_build_config_get(uint8_t buf[CSP_DLC], uint8_t target_node, uint8_t key) {
    memset(buf, 0, CSP_DLC);
    buf[0] = target_node;
    buf[1] = key;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_config_get(const uint8_t buf[CSP_DLC], uint8_t *target_node,
                                        uint8_t *key) {
    if (!csp_crc_ok(buf)) return false;
    *target_node = buf[0];
    *key = buf[1];
    return true;
}

static inline void csp_build_heartbeat(uint8_t buf[CSP_DLC], uint32_t master_ms) {
    memset(buf, 0, CSP_DLC);
    csp_pack_u32(buf, 0, master_ms);
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_heartbeat(const uint8_t buf[CSP_DLC], uint32_t *master_ms) {
    if (!csp_crc_ok(buf)) return false;
    *master_ms = csp_unpack_u32(buf, 0);
    return true;
}

static inline void csp_build_jog(uint8_t buf[CSP_DLC], uint8_t target_node, uint8_t dir,
                                 uint8_t speed_pct, uint8_t enable) {
    memset(buf, 0, CSP_DLC);
    buf[0] = target_node;
    buf[1] = dir & 1u;
    buf[2] = speed_pct;
    buf[3] = enable ? 1u : 0u;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_jog(const uint8_t buf[CSP_DLC], uint8_t *target_node, uint8_t *dir,
                                 uint8_t *speed_pct, uint8_t *enable) {
    if (!csp_crc_ok(buf)) return false;
    *target_node = buf[0];
    *dir = buf[1] & 1u;
    *speed_pct = buf[2];
    *enable = buf[3] ? 1u : 0u;
    return true;
}

static inline void csp_build_digital_out(uint8_t buf[CSP_DLC], uint8_t target_node, uint8_t mask,
                                         uint8_t value) {
    memset(buf, 0, CSP_DLC);
    buf[0] = target_node;
    buf[1] = mask;
    buf[2] = value;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_digital_out(const uint8_t buf[CSP_DLC], uint8_t *target_node,
                                         uint8_t *mask, uint8_t *value) {
    if (!csp_crc_ok(buf)) return false;
    *target_node = buf[0];
    *mask = buf[1];
    *value = buf[2];
    return true;
}

static inline void csp_build_status(uint8_t buf[CSP_DLC], uint8_t state, uint8_t node_id,
                                    int32_t position, uint8_t error_code, uint16_t speed_x10) {
    memset(buf, 0, CSP_DLC);
    buf[0] = state;
    buf[1] = node_id;
    csp_pack_i24(buf, 2, position);
    buf[5] = error_code;
    uint16_t coarse = speed_x10 / 10;
    buf[6] = (coarse > 255u) ? 255u : (uint8_t)coarse;
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_status(const uint8_t buf[CSP_DLC], uint8_t *state, uint8_t *node_id,
                                    int32_t *position, uint8_t *error_code, uint8_t *speed_coarse) {
    if (!csp_crc_ok(buf)) return false;
    *state = buf[0];
    *node_id = buf[1];
    *position = csp_unpack_i24(buf, 2);
    *error_code = buf[5];
    *speed_coarse = buf[6];
    return true;
}

static inline void csp_build_config_resp(uint8_t buf[CSP_DLC], uint8_t node_id, uint8_t key,
                                         uint32_t value) {
    memset(buf, 0, CSP_DLC);
    buf[0] = node_id;
    buf[1] = key;
    csp_pack_u32(buf, 2, value);
    buf[7] = csp_crc8(buf, CSP_DLC - 1);
}

static inline bool csp_parse_config_resp(const uint8_t buf[CSP_DLC], uint8_t *node_id, uint8_t *key,
                                         uint32_t *value) {
    if (!csp_crc_ok(buf)) return false;
    *node_id = buf[0];
    *key = buf[1];
    *value = csp_unpack_u32(buf, 2);
    return true;
}

#endif

# CANStepper protocol (v2)

**Normative source:** `libraries/CANStepperProtocol/CANStepperProtocol.h` — this document is a human-readable mirror. If they disagree, the header wins.

## Transport

| Property | Value |
|----------|--------|
| CAN | 2.0A standard ID |
| DLC | **8** bytes for all application frames |
| Integrity | **CRC-8** over bytes **0–6** (poly `0x07`, init `0`, MSB-first, no XOR out); byte **7** = CRC |
| Addressing | Byte **0** = **target node**; **`0xFF`** = broadcast (all slaves accept) |

## CAN identifiers

| ID (hex) | Direction | Name |
|----------|-----------|------|
| `100` | Master → slave | MOTOR_CMD |
| `101` | Master → slave | STOP |
| `102` | Master → slave | HOME |
| `103` | Master → slave | CONFIG_SET |
| `104` | Master → slave | CONFIG_GET |
| `105` | Master → slave | HEARTBEAT |
| `106` | Master → slave | JOG |
| `107` | Master → slave | DIGITAL_OUT |
| `200` | Slave → master | STATUS |
| `201` | Slave → master | CONFIG_RESP |

## Frame layouts (data[0]…data[7])

### MOTOR_CMD (`0x100`)

| Offset | Field |
|--------|--------|
| 0 | `target_node` |
| 1 | `0x01` (move) |
| 2 | `dir` (0 / 1) |
| 3–4 | `steps` uint16 **BE** |
| 5 | `rpm` **uint8** (0–255) |
| 6 | `accel_factor` (slave scales, typically ×10 steps/s²) |
| 7 | CRC |

Invalid if `steps == 0` or `rpm == 0`.

### STOP (`0x101`)

| 0 | `target_node` |
| 1 | `0x00` immediate, `0x01` decelerate |
| 2–6 | `0x00` |
| 7 | CRC |

### HOME (`0x102`)

| 0 | `target_node` |
| 1 | `home_dir` |
| 2 | `speed_pct` (1–100) |
| 3–6 | `0x00` |
| 7 | CRC |

### CONFIG_SET (`0x103`) / CONFIG_GET (`0x104`)

| 0 | `target_node` |
| 1 | `key` (see below) |
| 2–5 | **SET:** value uint32 **BE** — **GET:** `0` |
| 6 | `0` |
| 7 | CRC |

**Config keys** (`CSP_CFG_*` in header):

| Key | Meaning |
|-----|--------|
| `1` | Node ID (1–32) |
| `2` | Max speed (steps/s, uint32) |
| `3` | Invert direction (0/1) |
| `4` | Heartbeat timeout (ms, 0 = disabled) |
| `5` | Status TX period (ms) |

### HEARTBEAT (`0x105`)

| 0–3 | Master uptime **uint32 BE** (ms) |
| 4–6 | `0` |
| 7 | CRC |

### JOG (`0x106`)

| 0 | `target_node` |
| 1 | `dir` |
| 2 | `speed_pct` |
| 3 | `0` = stop jog, `1` = run |
| 4–6 | `0` |
| 7 | CRC |

### DIGITAL_OUT (`0x107`)

| 0 | `target_node` |
| 1 | `mask` (e.g. bit0 spindle, bit1 coolant) |
| 2 | `value` (bits applied where `mask` is 1) |
| 3–6 | `0` |
| 7 | CRC |

### STATUS (`0x200`)

| 0 | `state` (`CSP_SlaveState`) |
| 1 | `node_id` |
| 2–4 | Position **int24 BE** |
| 5 | `error` (`CSP_SlaveError`) |
| 6 | Speed coarse ≈ min(255, (speed×10)/10) steps/s |
| 7 | CRC |

### CONFIG_RESP (`0x201`)

| 0 | `node_id` |
| 1 | `key` |
| 2–5 | `value` uint32 **BE** |
| 6 | `0` |
| 7 | CRC |

## States and errors

Enumerations are defined in `CANStepperProtocol.h` (`CSP_STATE_*`, `CSP_ERR_*`). Notable errors:

| Code | Meaning |
|------|--------|
| `0x01` | Home sequence failed (switch not found) |
| `0x02` / `0x03` | Limit min / max |
| `0x04` | Heartbeat timeout |
| `0xFE` / `0xFF` | MCP2515 init / bitrate failure |

## Versioning

Protocol version macro: **`CSP_PROTOCOL_VERSION`** (integer). Bump it and this file when the wire format changes.

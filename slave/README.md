# Slave sketch

CAN **slave** — **MCP2515** + **AccelStepper** (STEP/DIR driver).

## Modules

| File | Role |
|------|------|
| `slave.ino` | `setup` / `loop`, ISRs, CAN read → dispatch |
| `slave_globals.*` | Shared state (`mcp2515`, `stepper`, frames, config) |
| `slave_can_handlers.*` | Protocol dispatch, command handlers, STATUS transmit |
| `slave_motion.*` | Heartbeat timeout, homing phases, `run` / `runSpeed`, limits |
| `slave_eeprom.*` | Load/save EEPROM |
| `slave_peripherals.*` | Digital outs, status LED, MCP2515 error poll |
| `slave_wokwi.*` | Serial-injected frames when `WOKWI_SIMULATION` is defined |
| `slave_config.h` | Pin map and constants |

- **Entry point:** `slave.ino`
- **Protocol:** v2 — see [`docs/PROTOCOL.md`](../docs/PROTOCOL.md)
- **FQBN (CI):** `arduino:avr:uno` (also works on many AVR boards after pin review)

## Pins (default)

| Signal | Pin |
|--------|-----|
| CAN CS / INT | 10 / 2 |
| STEP / DIR / EN | 5 / 6 / 7 |
| Home | 3 |
| Limit min / max | A2 / A3 |
| Spindle / coolant | 8 / 9 |
| Status LED | 13 |

## Simulation

Uncomment `#define WOKWI_SIMULATION` at the top of `slave.ino` for Wokwi loopback + Serial test commands.

## EEPROM

First boot uses RAM defaults until magic bytes are written; **CONFIG_SET** from the master persists node ID, speeds, heartbeat, etc.

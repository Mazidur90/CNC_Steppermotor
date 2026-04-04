# Master sketch

CAN **master** for **Arduino Mega 2560** (or compatible) + **MCP2515** module.

## Modules

| File | Role |
|------|------|
| `master.ino` | `setup` / `loop`, CAN ISR, MCP init |
| `master_globals.*` | `mcp2515`, frames, telemetry `nodes[]`, target node |
| `master_can_tx.*` | Heartbeat + all outbound command builders, TX retry |
| `master_can_rx.*` | STATUS + CONFIG_RESP parsing |
| `master_serial_ui.*` | Serial menu and status print |
| `master_led.*` | Activity / fault LED patterns |
| `master_presets.*` | Table-driven motion keys `f`/`b`/`F`/`B`/`x`/`X` |
| `master_util.*` | Small helpers (`rpm` → uint8) |
| `master_config.h` | Pins and timing |

- **Entry point:** `master.ino`
- **Protocol:** v2 — see [`docs/PROTOCOL.md`](../docs/PROTOCOL.md)
- **FQBN (CI):** `arduino:avr:mega:cpu=atmega2560`

## Pins (default)

| Signal | Pin |
|--------|-----|
| MCP2515 CS | 10 |
| MCP2515 INT | 2 |
| Status LED | 13 |
| SPI (Mega) | 50–52 |

Match **MCP2515 crystal** in code: `MCP_8MHZ` vs `MCP_16MHZ`.

## Serial

115200 baud — full menu in firmware (`m` for help).

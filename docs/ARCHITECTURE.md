# Architecture

## System view

```mermaid
flowchart LR
  subgraph Master
    UI[Serial UI]
    HB[Heartbeat 250ms]
    TX[CAN TX]
    RX[CAN RX]
    UI --> TX
    HB --> TX
    RX --> TEL[Telemetry table]
  end
  subgraph Bus[CAN 500 kbit/s]
    CAN((CAN))
  end
  subgraph Slave1[Slave node N]
    RXs[CAN RX]
    MOT[AccelStepper]
    IO[Limits / home / DOUT]
    EE[EEPROM]
    RXs --> MOT
    RXs --> IO
    EE --- Slave1
    MOT --> STAT[STATUS TX]
  end
  TX --> CAN
  CAN --> RXs
  STAT --> CAN
  CAN --> RX
```

## Components

| Part | Role |
|------|------|
| **`master/master.ino`** | Entry: `setup` / `loop`, CAN ISR, MCP2515 init. |
| **`master/master_*.cpp`** | `master_can_tx` (outgoing frames), `master_can_rx` (STATUS / CONFIG_RESP), `master_serial_ui` (menu), `master_led`, `master_presets` (motion key table), `master_globals` / `master_util`. |
| **`slave/slave.ino`** | Entry: `setup` / `loop`, ISRs, CAN RX pump, watchdog. |
| **`slave/slave_*.cpp`** | `slave_can_handlers` (dispatch + command handlers + STATUS TX), `slave_motion` (homing FSM, limits, stepper tick), `slave_eeprom`, `slave_peripherals` (DOUT, LED, MCP poll), `slave_wokwi` (optional sim). |
| **`libraries/CANStepperProtocol`** | Single source of truth for IDs, CRC, pack/parse — keeps master/slave binary-compatible. |

## Data flow

1. **Commands** flow **master → slaves** with an explicit **target** (or broadcast).
2. **STATUS** flows **each slave → bus**; the master does not assign IDs on the wire beyond filtering/logging by `node_id` in the payload.
3. **CONFIG** is **write-through to EEPROM** on the slave for supported keys; the slave echoes **CONFIG_RESP** for observability.

## Design choices

- **CRC on every frame** catches wiring noise and single-bit errors; it is not authentication.
- **RPM as uint8** trades range for a fixed 8-byte DLC with CRC.
- **Position int24** on STATUS bounds payload size; internal step position on the slave remains **int32** with clamping when encoding.

## Extension points

- Add a new **standard ID** and matching `csp_build_*` / `csp_parse_*` in the library, then dispatch in both sketches.
- Replace Serial UI with **USB host**, **ESP32 Web UI**, or **PC Python** while reusing the same CAN IDs and payloads.
- Add **hardware filters** on MCP2515 for high-traffic buses (not configured in the stock sketches).

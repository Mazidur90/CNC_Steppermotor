# CNC_Steppermotor — CAN bus stepper platform

[![Arduino CI](https://github.com/OWNER/REPO/actions/workflows/arduino.yml/badge.svg)](https://github.com/OWNER/REPO/actions/workflows/arduino.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Educational / prototyping stack for **distributed motion** on a **CAN 2.0A** bus: an Arduino **Mega** (or compatible) **master** with a Serial UI, and one or more **slave** nodes that run **AccelStepper** over STEP/DIR drivers (e.g. A4988, TMC2208).

This repository is structured like a small **firmware product**: shared protocol library, CI compile checks, contributor docs, and a versioned **protocol v2**.

### Documentation

| Doc | Contents |
|-----|----------|
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Wire format, IDs, CRC, config keys |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | System diagram and data flow |
| [CHANGELOG.md](CHANGELOG.md) | Release notes |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to change protocol / PR expectations |
| [SECURITY.md](SECURITY.md) | Safety scope and reporting |
| [master/README.md](master/README.md) / [slave/README.md](slave/README.md) | Sketch-specific notes |

---

## What this project does (summary)

| Area | Behavior |
|------|------------|
| **Bus** | MCP2515 @ **500 kbit/s** (8 MHz crystal assumed; change to `MCP_16MHZ` if your module is 16 MHz). |
| **Integrity** | **CRC-8** (poly `0x07`) over bytes 0–6; byte 7 = checksum on every 8-byte frame. |
| **Addressing** | Per-frame **target node** (`0xFF` = **broadcast**). Slaves ignore traffic not aimed at them. |
| **Motion** | Relative moves (RPM + microsteps), **stop** (immediate / decel), **home** with **fast seek → backoff → slow creep**. |
| **Jog** | Continuous velocity via `AccelStepper::runSpeed()`. |
| **I/O** | **Spindle** and **coolant** (or generic) **digital outputs** on the slave via CAN. |
| **Config** | **EEPROM** on each slave: node ID, max speed, invert dir, heartbeat timeout, status period. **CONFIG_GET / SET** over CAN. |
| **Safety / robustness** | **Limit switches** (optional wiring), **heartbeat loss → fault**, **AVR watchdog** (2 s) on the slave, **MCP2515 error-flag** polling. |
| **Master UI** | Serial menu @ **115200**: motion, jog, I/O toggles, target node `0`–`8`, config shortcuts. |
| **Simulation** | **Wokwi** diagram + `libraries.txt`; enable `#define WOKWI_SIMULATION` in `slave.ino` for loopback + Serial injection. |

---

## Repository layout

```
CNC_Steppermotor/
├── .editorconfig
├── .github/workflows/arduino.yml   ← CI: compile master + slave
├── docs/
│   ├── ARCHITECTURE.md
│   └── PROTOCOL.md
├── scripts/
│   ├── check_compile.sh
│   └── check_compile.ps1
├── master/
│   ├── README.md
│   ├── master.ino
│   ├── master_*.cpp / .h   (CAN TX/RX, Serial UI, LED, presets, globals)
│   └── ...
├── slave/
│   ├── README.md
│   ├── slave.ino
│   ├── slave_*.cpp / .h    (CAN handlers, motion, EEPROM, I/O, Wokwi)
│   └── ...
├── libraries/CANStepperProtocol/
│   ├── CANStepperProtocol.h
│   ├── library.properties
│   └── README.md
├── wokwi/
├── LICENSE
├── CHANGELOG.md
├── CONTRIBUTING.md
├── SECURITY.md
└── README.md
```

**Include path:** sketches use `#include "../libraries/CANStepperProtocol/CANStepperProtocol.h"`. Keep this folder layout, **or** copy `libraries/CANStepperProtocol` into your Arduino sketchbook `libraries` folder and use `#include <CANStepperProtocol.h>`.

---

## Hardware (reference wiring)

### Master (Mega + MCP2515)

- CS **10**, INT **2**, SPI **50–52**, LED **13**
- **120 Ω** termination at each bus end (typical CAN practice)

### Slave (MCU + MCP2515 + driver)

| Signal | Default pin |
|--------|-------------|
| CAN CS / INT | 10 / 2 |
| STEP / DIR / EN | 5 / 6 / 7 |
| Home switch | 3 (INPUT_PULLUP, active LOW) |
| Limit min / max | A2 / A3 (optional, INPUT_PULLUP) |
| Spindle / coolant out | 8 / 9 |
| Status LED | 13 |

Adjust pins in `slave.ino` / `master.ino` for your board.

---

## Protocol v2 (quick reference)

Tables and semantics: **[docs/PROTOCOL.md](docs/PROTOCOL.md)**. C API: **`libraries/CANStepperProtocol/CANStepperProtocol.h`**.

| CAN ID | Direction | Purpose |
|--------|-----------|---------|
| `0x100` | M→S | Motor relative move |
| `0x101` | M→S | Stop |
| `0x102` | M→S | Home |
| `0x103` | M→S | CONFIG set (+ EEPROM on slave) |
| `0x104` | M→S | CONFIG get |
| `0x105` | M→S | Heartbeat (master uptime ms) |
| `0x106` | M→S | Jog run/stop |
| `0x107` | M→S | Digital outputs (mask + value) |
| `0x200` | S→M | Status (state, node, pos, error, speed coarse) |
| `0x201` | S→M | CONFIG response |

**Motor note:** RPM is sent as **uint8** (0–255) to fit the 8-byte CRC frame.

**Config keys:** `CSP_CFG_NODE_ID`, `CSP_CFG_MAX_SPEED_STEPS`, `CSP_CFG_INVERT_DIR`, `CSP_CFG_HEARTBEAT_MS`, `CSP_CFG_STATUS_MS` (see header).

---

## Building and flashing

1. Install **Arduino IDE** 2.x (or CLI) and libraries:
   - **MCP2515** (autowp)
   - **AccelStepper**
2. Open `master/master.ino` or `slave/slave.ino`.
3. Select the correct board (Mega for master; your slave MCU).
4. Flash **both** after a protocol change so CRC and layouts stay matched.

**CLI (optional):** from repo root, `./scripts/check_compile.sh` or `scripts/check_compile.ps1` (requires [Arduino CLI](https://arduino.github.io/arduino-cli/)).

**Wokwi:** uncomment `#define WOKWI_SIMULATION` at the top of `slave.ino`, open the `wokwi` project, and use Serial keys documented in that file (`f`, `b`, `j`, `k`, `q`, …).

---

## Serial menu (master)

- **`0`** … **`8`**: target **broadcast** or node **1–8**
- **`f` `b` `F` `B` `x` `X`**: moves
- **`s` `S`**: decel / immediate stop
- **`h` `H`**: home
- **`j` `k` `q`**: jog + / − / stop
- **`y` `c`**: spindle / coolant toggle (digital outs on slave)
- **`v`**: CONFIG read (node, max speed, heartbeat ms)
- **`n` / `N`**: set heartbeat supervision **3000 ms** / **off** (writes slave EEPROM when accepted)
- **`?` `m`**: status / menu

---

## What we are *not* claiming

- **No replacement for a hardware e-stop** or machine safety interlocks.
- **Not a full G-code interpreter**; the master is a **menu + CAN bridge** you can replace with a PC or PLC.
- **Position on CAN** is **int24** on the wire (clamped); internal step counter remains **int32** on the slave.

---

## License

[MIT](LICENSE) — replace the copyright line with your legal name or org if you fork.

---

## After you create the GitHub repo

1. Replace **`OWNER/REPO`** in the CI badge at the top of this README with your GitHub path.
2. Add a **Wokwi** or hardware photo under `docs/` and link it here.
3. Tag **`v2.0.0`** when you want a frozen protocol snapshot (see `CHANGELOG.md`).

# Changelog

All notable changes to this project are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [2.0.0] — 2026-04-04

### Added

- Protocol **v2**: per-frame target node (`0xFF` broadcast), CRC-8 on all 8-byte frames.
- Messages: `CONFIG_SET` / `CONFIG_GET` / `CONFIG_RESP`, `HEARTBEAT`, `JOG`, `DIGITAL_OUT`.
- Slave: EEPROM (node ID, max speed, invert dir, heartbeat timeout, status interval).
- Slave: limit inputs, two-stage homing (backoff + creep), jog mode, spindle/coolant outputs.
- Slave: heartbeat loss fault, AVR watchdog, MCP2515 error polling.
- Master: multi-node telemetry, periodic heartbeat, extended Serial menu.
- Shared library `CANStepperProtocol` with pack/parse helpers.
- Documentation: `README`, `docs/PROTOCOL`, `docs/ARCHITECTURE`, CI workflow.
- **Modular sketches:** `master_*.cpp` / `slave_*.cpp` split; **motion presets** table on master; slave **RX / STATUS TX** counters (`statRxFramesOk`, `statTxStatus`, Wokwi `?`).

### Changed

- Motor command: RPM on the wire is **uint8** (0–255) to fit the CRC frame layout.
- Status payload: includes **node_id**; speed on bus is **coarse** (see protocol doc).
- Slave jog: `setMaxSpeed` now uses the computed jog speed (fixes stray `jogSpeed` symbol).

### Breaking

- **Incompatible** with protocol v1 frames; flash master and all slaves together.

[2.0.0]: (tag `v2.0.0` when you publish the repo)

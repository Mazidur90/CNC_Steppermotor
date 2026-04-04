# CANStepperProtocol

Arduino **header-only** library: **CANStepper protocol v2** — frame IDs, CRC-8, and `csp_build_*` / `csp_parse_*` helpers.

- **Authoritative spec:** [`CANStepperProtocol.h`](./CANStepperProtocol.h) and [`../../docs/PROTOCOL.md`](../../docs/PROTOCOL.md)
- **Version:** see `CSP_PROTOCOL_VERSION` in the header and `library.properties`

## Install

1. Copy this folder to your sketchbook `libraries/CANStepperProtocol`, **or**
2. Keep the monorepo layout and use `#include "../libraries/CANStepperProtocol/CANStepperProtocol.h"` from sketches.

## Dependency

Arduino core (provides `Arduino.h`, `uint8_t`, etc.).

## License

Same as the parent repository (`LICENSE` at repo root).

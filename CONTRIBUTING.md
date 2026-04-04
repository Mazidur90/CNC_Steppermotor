# Contributing

Thanks for improving this project. Small, focused changes are easier to review than large refactors.

## Before you change the protocol

1. Bump **`CSP_PROTOCOL_VERSION`** in `libraries/CANStepperProtocol/CANStepperProtocol.h` if the on-wire layout changes.
2. Update **`docs/PROTOCOL.md`** and **`CHANGELOG.md`** in the same pull request.
3. Ensure **master** and **slave** (and any tools) stay in sync; mismatched CRC or field layouts cause silent drops or faults.

## Code style

- Match the existing sketches: section headers, `F()` macros for static Serial strings on AVR, minimal blocking `delay()` except where already used for retries.
- Prefer **`static`** helpers in `.ino` files over growing monolithic `loop()` bodies without structure.

## Pull requests

1. Describe **what** changed and **why** (hardware tested, Wokwi only, or compile-only).
2. If you cannot test on hardware, say so; CI compiles **master** (Mega) and **slave** (Uno) FQBNs.

## Local compile check

With [Arduino CLI](https://arduino.github.io/arduino-cli/) installed:

```bash
./scripts/check_compile.sh
```

On Windows (PowerShell), run the commands inside `scripts/check_compile.ps1` or use Arduino IDE **Verify**.

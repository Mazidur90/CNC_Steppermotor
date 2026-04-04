# Security

This repository is **firmware for embedded motion** on a **CAN bus**. It is not designed as a hardened security boundary.

## Reporting issues

If you find a **safety-critical** defect (e.g. unexpected motion, failure to stop, or a pattern that could damage hardware), please open a **GitHub issue** with:

- Board(s) and wiring summary  
- Steps to reproduce  
- Whether the bus is isolated or shared with other equipment  

Do **not** use public issues for unrelated security scanner noise (dependency false positives on Arduino cores).

## Threat model (explicit)

- **Anyone with physical CAN access** can craft frames unless you add higher-layer authentication (not implemented here).
- **Software stop** is **not** a substitute for **hardware e-stop** and machine guarding for real CNC equipment.

# Project LEDJacket — Firmware & Gateware (`LEDJacket_FW`)

This repository contains the embedded firmware and FPGA gateware for Project LEDJacket.

---

## Documentation & Specifications

- **[`RS485_PROTOCOL_PLAN.md`](./RS485_PROTOCOL_PLAN.md)**: Complete architectural plan for the 3.0 Mbps RS-485 communication protocol between the Central Controller (`Lattice ECP5` FPGA) and the daisy-chained `TriangleNode` and `IMUNode` boards (`PY32F003F18U6`), covering:
  - **Zero-Overhead "Chain-Train" Video Streaming (`0.14%` framing overhead)** using Two-Stage DMA Offset Slicing and `USART_IDLE` global frame latching.
  - **Zero-Flash Commissioning & Ordering Scheme** for ~1,700 factory-identical nodes (Electronic Slotted-Aloha + Binary-Tree 96-bit `UID` discovery, Electrical `+5V` IR-Drop Gradient pre-sorting via internal `1.20V` `VREFINT`, Interactive Visual Chase / 6-Color Block confirmation, and 2-second Camera Gray-Code mapping).
  - **Drop-In Single-Node Field Replacement ("Auto-Heal")** via pigeonhole UID diffing.
  - **Built-In Node Testing Modes** (`R -> B -> G -> 3s White Fade -> Telemetry Report`, pixel walk, visual slot readout, standalone bench self-test).
  - **Piggybacked Thermal/Voltage Telemetry & 200 Hz Dedicated Sleeve IMU Polling**.

---

## Directory Structure

- [`TriangleNodeFirmware/`](./TriangleNodeFirmware/): PlatformIO project targeting the Puya `PY32F003F18U6` MCU on each 52-LED Triangle Node.

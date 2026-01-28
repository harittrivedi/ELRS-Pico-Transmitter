# ELRS Pico TX - Zephyr RTOS Port

Zephyr RTOS port of ELRS-Pico-Transmitter for RP2350 (Pico 2).

## Features (Target)
- USB CRSF Bridge mode (USB CDC → UART → ELRS TX)
- PIO-based inverted UART for ELRS compatibility
- Failsafe injection on USB timeout

## Build Instructions

```bash
# Ensure Zephyr environment is active
cd elrs-pico-tx-zephyr
west build -b rpi_pico2
west flash
```

## Project Structure
```
├── CMakeLists.txt      # Build configuration
├── prj.conf            # Kconfig options
├── boards/
│   └── rpi_pico2.overlay  # Devicetree overlay
├── src/
│   ├── main.c          # Entry point
│   ├── crsf.c/h        # CRSF protocol
│   └── pio_uart.c/h    # PIO inverted UART
└── docs/
    └── port_plan.md    # Detailed porting plan
```

## Original Project
Based on: https://github.com/.../ELRS-Pico-Transmitter

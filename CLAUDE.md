# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Pico Tyre Temperature Monitor - a high-performance thermal tyre monitoring system for Raspberry Pi Pico using the MLX90640 thermal camera. Written in C for the Pico SDK, achieving 11.5 fps (7.6x faster than the original CircuitPython implementation).

## Build Commands

```bash
# Prerequisites: Set PICO_SDK_PATH environment variable
export PICO_SDK_PATH=~/pico-sdk

# Download MLX90640 library (required first time)
./tools/download_mlx_library.sh

# Build
mkdir build && cd build
cmake ..
make -j4

# Output: build/thermal_tyre_pico.uf2
```

**Clean rebuild** (if CMakeLists.txt changes):
```bash
cd build && rm -rf * && cmake .. && make -j4
```

**Flash to Pico**: Hold BOOTSEL button while plugging in USB, copy `thermal_tyre_pico.uf2` to RPI-RP2 drive.

## Architecture

### Peripheral Bus Design
- **I2C0 (GP0/GP1)**: Master mode - communicates with MLX90640 thermal sensor at address 0x33
- **I2C1 (GP26/GP27)**: Slave mode - exposes thermal and laser data to external controllers at address 0x08
- **UART1 (GP4/GP5)**: Laser ranger (DFRobot SEN0366) at 9600 baud

### Core Modules

| File | Purpose |
|------|---------|
| `main.c` | Main loop: sensor read → temperature calc → algorithm → I2C update → serial output |
| `thermal_algorithm.c/h` | Tyre detection: extracts middle rows, region growing, zone statistics (left/centre/right) |
| `i2c_slave.c/h` | I2C slave implementation with 256-byte register map |
| `communication.c/h` | USB serial output (CSV/JSON formats) |
| `laser_ranger.c/h` | Serial laser distance sensor driver (non-blocking, continuous mode) |
| `mlx90640/` | Melexis driver library (downloaded via script) |

### I2C Slave Register Map

- **0x00-0x0F**: Configuration (address, output mode, emissivity, raw mode)
- **0x10-0x1F**: Status (firmware version, frame counter, FPS, detection state)
- **0x20-0x2D**: Zone temperatures (left/centre/right median/avg as int16 tenths)
- **0x30-0x4F**: Raw 16-channel thermal data (when RAW_MODE=1)
- **0x50-0x51**: Full 768-pixel frame access
- **0x60-0x6F**: Laser ranger data (distance mm/um, status, error code)

### Data Flow

1. `MLX90640_GetFrameData()` reads raw sensor data (~60ms)
2. `MLX90640_CalculateTo()` converts to temperatures (~20ms)
3. `thermal_algorithm_process()` detects tyre and calculates zones (~2ms)
4. `i2c_slave_update()` updates thermal registers atomically (<1ms)
5. `laser_ranger_poll()` parses any available laser frames (non-blocking)
6. `i2c_slave_update_laser()` updates laser registers
7. `send_serial_compact/json()` outputs to USB (~3ms)

### Thread Safety

- I2C slave uses interrupt-driven handler
- Register updates use minimal critical sections (<10µs with interrupts disabled)
- Pre-calculate all values before entering critical section to prevent I2C NACKs

### Watchdog

5-second watchdog timer auto-reboots if main loop hangs (e.g., sensor disconnect). Check `watchdog_caused_reboot()` on startup.

## Configuration

**Output format**: Set `COMPACT_OUTPUT` in `main.c` (1=CSV, 0=JSON)

**Algorithm tuning** in `thermal_algorithm.c`:
- `mad_threshold`: Detection sensitivity (default 3.0)
- `min_tyre_width` / `max_tyre_width`: Valid tyre width in pixels

**Emissivity**: Configurable via I2C register 0x04 (value × 100, default 95 = 0.95)

## Hardware

- **MLX90640** at 0x33 on I2C0 (GP0=SDA, GP1=SCL) - 4.7kΩ pull-ups required
- **I2C slave** at 0x08 on I2C1 (GP26=SDA, GP27=SCL) for external controllers
- **UART1 (GP4/GP5)**: Laser ranger (DFRobot SEN0366) at 9600 baud - auto-detected at boot
- Thermal sensor runs at 16Hz refresh rate, 1MHz I2C speed
- Laser runs in continuous mode (~3-20Hz depending on configuration)

## Tests

Python test scripts in `tests/` directory require a connected Pico and I2C adapter (e.g., MCP2221):
```bash
python tests/test_i2c_slave.py
python tests/test_raw_mode.py
```

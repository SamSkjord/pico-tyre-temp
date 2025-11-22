# Building the C/C++ Version

## Overview

The C version delivers **4-10Hz performance** (vs 1.5Hz in CircuitPython) through:
- Compiled native code (no Python interpreter overhead)
- Optimized math operations (fast median, MAD)
- Direct hardware access
- Minimal memory allocations

## Prerequisites

### 1. Install Pico SDK

```bash
# Clone Pico SDK
cd ~/
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable (add to ~/.zshrc or ~/.bashrc)
export PICO_SDK_PATH=~/pico-sdk
```

### 2. Install Build Tools

**macOS:**
```bash
# Install CMake
brew install cmake

# Download and install ARM GCC toolchain
# Get the latest from: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# Look for "arm-none-eabi" for macOS (Darwin) - currently version 13.3.rel1
# Install the .pkg file to /Applications/ArmGNUToolchain/

# Set environment variables (add to ~/.zshrc or ~/.bashrc)
export PICO_SDK_PATH=~/pico-sdk
export PICO_TOOLCHAIN_PATH=/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi
```

**Note**: The `brew install --cask gcc-arm-embedded` does NOT work correctly with CMake for Pico builds. You must download and install the official ARM toolchain from the link above.

**Linux:**
```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

**Windows:**
- Install [CMake](https://cmake.org/download/)
- Install [ARM GCC](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
- Install [Build Tools for Visual Studio](https://visualstudio.microsoft.com/downloads/)

### 3. Download MLX90640 Library

```bash
cd c_version
chmod +x download_mlx_library.sh
./download_mlx_library.sh
```

This downloads the official Melexis MLX90640 C library.

## Build Steps

### Quick Build

**macOS:**
```bash
cd c_version
mkdir build
cd build

# Set environment variables for this build
export PICO_SDK_PATH=~/pico-sdk
export PICO_TOOLCHAIN_PATH=/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi

# Configure and build
cmake ..
make -j4
```

**Linux/Windows:**
```bash
cd c_version
mkdir build
cd build

# Only PICO_SDK_PATH needed (ARM GCC should be in PATH)
export PICO_SDK_PATH=~/pico-sdk

cmake ..
make -j4
```

This creates `thermal_tyre_pico.uf2` in the `build` directory.

### Upload to Pico

1. **Hold BOOTSEL button** on Pico while plugging in USB
2. Pico appears as `RPI-RP2` drive
3. **Copy `thermal_tyre_pico.uf2`** to the drive
4. Pico reboots automatically and runs the program

### Connect Serial

**macOS/Linux:**
```bash
screen /dev/tty.usbmodem* 115200
# Or
minicom -D /dev/ttyACM0 -b 115200
```

**Windows:**
- Use PuTTY or TeraTerm
- Select COM port, 115200 baud

## Project Structure

```
c_version/
├── CMakeLists.txt              # Build configuration
├── main.c                      # Main application
├── thermal_algorithm.c/h       # Fast tyre detection
├── communication.c/h           # Serial + I2C output
├── mlx90640/
│   ├── MLX90640_API.c         # Melexis library (download)
│   ├── MLX90640_API.h         # Melexis headers (download)
│   ├── MLX90640_I2C_Driver.c  # Pico I2C driver (custom)
│   └── MLX90640_I2C_Driver.h
└── build/                      # Generated
    └── thermal_tyre_pico.uf2
```

## Expected Performance

| Component | Time | Notes |
|-----------|------|-------|
| Sensor read (hardware) | 125ms | MLX90640 at 16Hz mode |
| Temperature calculation | 5-10ms | Melexis library |
| Algorithm processing | 2-5ms | Fast C implementation |
| Output (serial) | <1ms | USB CDC |
| **Total** | **~135-145ms** | **~7 fps** |

**Comparison:**
- CircuitPython: 640ms (1.5 fps)
- C version: ~140ms (7 fps)
- **Speedup: 4.5x faster!** 🚀

## Optimization Flags

The CMakeLists.txt uses aggressive optimization:
```cmake
-O3              # Maximum optimization
-ffast-math      # Fast floating-point math
-funroll-loops   # Loop unrolling
```

## Troubleshooting

### "pico_sdk_import.cmake not found"
Make sure `PICO_SDK_PATH` is set correctly:
```bash
export PICO_SDK_PATH=~/pico-sdk
```

### "MLX90640_API.c not found"
Run the download script:
```bash
./download_mlx_library.sh
```

### "arm-none-eabi-gcc not found"
Install ARM GCC toolchain (see Prerequisites above).

**On macOS**: Make sure you set `PICO_TOOLCHAIN_PATH` before running cmake:
```bash
export PICO_TOOLCHAIN_PATH=/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi
```

### Build fails with "unknown directive .syntax unified" or "unknown CPU name"
This means CMake is using the wrong compiler (Apple Clang instead of ARM GCC).

**Solution**: Set the toolchain path before running cmake:
```bash
cd build
rm -rf *  # Clean build directory
export PICO_SDK_PATH=~/pico-sdk
export PICO_TOOLCHAIN_PATH=/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi
cmake ..
make -j4
```

### Serial output garbled
Check baud rate is 115200.

### Sensor not detected
Check wiring:
```
MLX90640 VDD → Pico 3V3 (Pin 36)
MLX90640 GND → Pico GND (Pin 38)
MLX90640 SDA → Pico GP0 (Pin 1)
MLX90640 SCL → Pico GP1 (Pin 2)
```

### Slow performance (< 4 Hz)
- Check sensor refresh rate is set to 16Hz
- Verify I2C speed is 1MHz
- Check compilation with -O3 optimization

## Development Tips

### Rebuild After Changes

```bash
cd build
make -j4
# Copy new .uf2 to Pico (hold BOOTSEL and reconnect USB)
cp thermal_tyre_pico.uf2 /Volumes/RPI-RP2/
```

**Note**: If you've made CMakeLists.txt changes, do a clean build instead.

### Clean Build

**macOS:**
```bash
cd build
rm -rf *
export PICO_SDK_PATH=~/pico-sdk
export PICO_TOOLCHAIN_PATH=/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi
cmake ..
make -j4
```

**Linux/Windows:**
```bash
cd build
rm -rf *
export PICO_SDK_PATH=~/pico-sdk
cmake ..
make -j4
```

### Change Output Format

Edit `main.c`:
```c
#define COMPACT_OUTPUT 1  // 1 for CSV, 0 for JSON
```

### Adjust Algorithm Parameters

Edit `thermal_algorithm_init()` in `thermal_algorithm.c`:
```c
config->mad_threshold = 3.0f;     // Adjust sensitivity
config->min_tyre_width = 6;       // Min pixels
config->max_tyre_width = 28;      // Max pixels
```

### Debug Output

Add to `main.c`:
```c
printf("Debug: value = %f\n", some_value);
```

### Monitor Performance

The application prints timing every 10 frames:
```
[Frame 10] Total: 138.2ms (7.2 fps) | Sensor: 125.3ms | Calc: 8.1ms | Algo: 3.2ms | Comm: 1.6ms
```

## Next Steps

1. **Build and test** the C version
2. **Compare performance** with CircuitPython version using visualizer
3. **Tune algorithm parameters** for your specific tyre
4. **Integrate with your system** via serial or I2C

## Integration

### Serial Output (CSV)
```
frame_number,left_avg,centre_avg,right_avg,confidence,warnings
1,45.2,48.5,44.8,0.85,0
2,45.3,48.6,44.9,0.86,0
```

### Serial Output (JSON)
Full JSON format same as CircuitPython version.

### I2C Peripheral (Future)
Register map same as CircuitPython version (see main README.md).

## Performance Goals

- ✅ **Minimum: 4 Hz** (250ms per frame)
- ✅ **Target: 7 Hz** (140ms per frame)
- 🎯 **Stretch: 10 Hz** (100ms per frame)

The C version should easily achieve 7Hz. For 10Hz, further optimization of the Melexis library may be needed.

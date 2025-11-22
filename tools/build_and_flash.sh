#!/bin/bash
# Quick build and flash script for thermal_tyre_pico

set -e

echo "=========================================="
echo "Thermal Tyre Pico - Build & Flash"
echo "=========================================="
echo ""

# Check prerequisites
if [ -z "$PICO_SDK_PATH" ]; then
    echo "ERROR: PICO_SDK_PATH not set!"
    echo "Please run: export PICO_SDK_PATH=~/pico-sdk"
    exit 1
fi

if ! command -v cmake &> /dev/null; then
    echo "ERROR: cmake not found!"
    echo "Install with: brew install cmake"
    exit 1
fi

# On macOS, check for and set PICO_TOOLCHAIN_PATH if needed
if [[ "$OSTYPE" == "darwin"* ]]; then
    if [ -z "$PICO_TOOLCHAIN_PATH" ]; then
        # Try to find ARM GCC toolchain
        if [ -d "/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi" ]; then
            export PICO_TOOLCHAIN_PATH="/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi"
            echo "Using ARM GCC toolchain: $PICO_TOOLCHAIN_PATH"
        elif [ -d "/Applications/ArmGNUToolchain/14.3.rel1/arm-none-eabi" ]; then
            export PICO_TOOLCHAIN_PATH="/Applications/ArmGNUToolchain/14.3.rel1/arm-none-eabi"
            echo "Using ARM GCC toolchain: $PICO_TOOLCHAIN_PATH"
        else
            echo "ERROR: ARM GCC toolchain not found!"
            echo "Download from: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads"
            echo "Install the .pkg file, then run this script again"
            exit 1
        fi
    fi
fi

# Check if arm-none-eabi-gcc is available
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "ERROR: arm-none-eabi-gcc not found in PATH!"
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "Download from: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads"
    else
        echo "Install with: sudo apt install gcc-arm-none-eabi"
    fi
    exit 1
fi

# Download MLX library if needed
if [ ! -f "mlx90640/MLX90640_API.c" ]; then
    echo "Downloading MLX90640 library..."
    ./download_mlx_library.sh
    echo ""
fi

# Build
echo "Building..."
mkdir -p build
cd build

cmake .. > /dev/null 2>&1
make -j4

if [ $? -eq 0 ]; then
    echo "✓ Build successful!"
    echo ""
    ls -lh thermal_tyre_pico.uf2
    echo ""
else
    echo "✗ Build failed!"
    exit 1
fi

# Check if Pico is in BOOTSEL mode
PICO_MOUNT="/Volumes/RPI-RP2"
if [ -d "$PICO_MOUNT" ]; then
    echo "Pico detected in BOOTSEL mode!"
    echo "Flashing firmware..."
    cp thermal_tyre_pico.uf2 "$PICO_MOUNT/"
    echo "✓ Flashed successfully!"
    echo ""
    echo "Waiting for Pico to reboot..."
    sleep 3

    # Try to find serial port
    SERIAL_PORT=$(ls /dev/tty.usbmodem* 2>/dev/null | head -1)
    if [ -n "$SERIAL_PORT" ]; then
        echo "✓ Pico serial port: $SERIAL_PORT"
        echo ""
        echo "Connect with: screen $SERIAL_PORT 115200"
    fi
else
    echo "⚠️  Pico not detected in BOOTSEL mode"
    echo ""
    echo "To flash manually:"
    echo "1. Hold BOOTSEL button on Pico"
    echo "2. Plug in USB cable"
    echo "3. Copy build/thermal_tyre_pico.uf2 to RPI-RP2 drive"
    echo ""
    echo "OR run this script again after entering BOOTSEL mode"
fi

echo ""
echo "=========================================="
echo "Done!"
echo "=========================================="

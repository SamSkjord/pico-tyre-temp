#!/usr/bin/env python3
"""
Test Pico I2C slave using MCP2221 USB-I2C bridge
Install: pip install EasyMCP2221
"""

try:
    from EasyMCP2221 import SMBus
except ImportError:
    print("ERROR: EasyMCP2221 not installed")
    print("Install with: pip install EasyMCP2221")
    exit(1)

import time

PICO_I2C_ADDR = 0x08

def read_int16_le(data):
    """Convert little-endian bytes to signed int16"""
    val = data[0] | (data[1] << 8)
    if val > 32767:
        val -= 65536
    return val

def main():
    print("=" * 60)
    print("Pico I2C Slave Test - MCP2221 USB-I2C Bridge")
    print("=" * 60)
    print()

    try:
        # Initialize MCP2221 I2C bus
        bus = SMBus()
        print("MCP2221 initialized\n")

        # Scan for devices
        print("Scanning I2C bus...")
        found = False
        for addr in range(0x08, 0x78):
            try:
                bus.read_byte(addr)
                print(f"  Found device at 0x{addr:02X}")
                if addr == PICO_I2C_ADDR:
                    found = True
            except:
                pass

        if not found:
            print(f"\nERROR: Pico not found at address 0x{PICO_I2C_ADDR:02X}")
            print("\nTroubleshooting:")
            print("1. Check wiring:")
            print("   - Pico GP26 (SDA) → MCP2221 SDA")
            print("   - Pico GP27 (SCL) → MCP2221 SCL")
            print("   - Pico GND → MCP2221 GND")
            print("2. Check pull-up resistors (4.7kΩ on SDA/SCL)")
            print("3. Verify Pico is running")
            return

        print(f"\n✓ Pico found at 0x{PICO_I2C_ADDR:02X}\n")

        # Test 1: Read firmware version
        print("[Test 1] Firmware Version")
        version = bus.read_byte_data(PICO_I2C_ADDR, 0x10)
        print(f"  Version: {version}\n")

        # Test 2: Read FPS and frame number
        print("[Test 2] Performance Metrics")
        data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x11, 3)
        frame_num = data[0] | (data[1] << 8)
        fps = data[2]
        print(f"  Frame: {frame_num}")
        print(f"  FPS: {fps}\n")

        # Test 3: Read detection status
        print("[Test 3] Tyre Detection")
        data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x14, 6)
        detected = data[0]
        confidence = data[1]
        width = data[2]
        span_start = data[3]
        span_end = data[4]
        warnings = data[5]

        print(f"  Detected: {'YES ✓' if detected else 'NO'}")
        print(f"  Confidence: {confidence}%")
        print(f"  Tyre Width: {width} pixels")
        print(f"  Span: {span_start} to {span_end}")
        print(f"  Warnings: 0x{warnings:02X}\n")

        # Test 4: Read median temperatures
        print("[Test 4] Zone Median Temperatures")
        data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x20, 6)

        left_tenths = read_int16_le(data[0:2])
        centre_tenths = read_int16_le(data[2:4])
        right_tenths = read_int16_le(data[4:6])

        print(f"  Left:   {left_tenths / 10.0:.1f}°C")
        print(f"  Centre: {centre_tenths / 10.0:.1f}°C")
        print(f"  Right:  {right_tenths / 10.0:.1f}°C\n")

        # Test 5: Read average temperatures
        print("[Test 5] Zone Average Temperatures")
        data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x26, 6)

        left_avg = read_int16_le(data[0:2]) / 10.0
        centre_avg = read_int16_le(data[2:4]) / 10.0
        right_avg = read_int16_le(data[4:6]) / 10.0

        print(f"  Left:   {left_avg:.1f}°C")
        print(f"  Centre: {centre_avg:.1f}°C")
        print(f"  Right:  {right_avg:.1f}°C\n")

        # Test 6: Read lateral gradient
        print("[Test 6] Lateral Temperature Gradient")
        data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x2C, 2)
        gradient = read_int16_le(data) / 10.0
        print(f"  Gradient: {gradient:.1f}°C\n")

        # Test 7: Continuous monitoring
        print("[Test 7] Live Monitoring (10 samples, 0.5s interval)")
        print("  " + "-" * 72)
        print("  Sample | FPS | Centre Med | Centre Avg | Detected | Confidence")
        print("  " + "-" * 72)

        for i in range(10):
            # Read FPS
            fps = bus.read_byte_data(PICO_I2C_ADDR, 0x13)

            # Read centre temperatures
            temp_data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x22, 8)
            centre_med = read_int16_le(temp_data[0:2]) / 10.0
            centre_avg = read_int16_le(temp_data[6:8]) / 10.0

            # Read detection
            det_data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x14, 2)
            detected = "YES" if det_data[0] else "NO "
            confidence = det_data[1]

            print(f"  {i+1:6d} | {fps:3d} | {centre_med:9.1f}° | {centre_avg:9.1f}° | {detected}      | {confidence:3d}%")

            time.sleep(0.5)

        print("  " + "-" * 72)
        print("\n" + "=" * 60)
        print("✓ All tests completed successfully!")
        print("=" * 60)

    except Exception as e:
        print(f"\nERROR: {e}")
        print("\nMake sure:")
        print("1. MCP2221 is connected to computer")
        print("2. I2C wiring is correct (SDA/SCL/GND)")
        print("3. Pico firmware is running")

if __name__ == "__main__":
    main()

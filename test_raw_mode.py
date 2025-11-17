#!/usr/bin/env python3
"""
Test raw 16-channel mode
"""

from EasyMCP2221 import SMBus
import time

PICO_I2C_ADDR = 0x08

def read_int16_le(data, offset=0):
    """Convert little-endian bytes to signed int16"""
    val = data[offset] | (data[offset + 1] << 8)
    if val > 32767:
        val -= 65536
    return val

def read_16_channels(bus):
    """Read all 16 raw channels"""
    data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x30, 32)
    channels = []
    for i in range(16):
        temp_tenths = read_int16_le(data, i * 2)
        channels.append(temp_tenths / 10.0)
    return channels

def main():
    print("=" * 60)
    print("Raw 16-Channel Mode Test")
    print("=" * 60)
    print()

    bus = SMBus()

    # Test 1: Normal mode
    print("[Test 1] Normal Mode (Algorithm Enabled)")
    raw_mode = bus.read_byte_data(PICO_I2C_ADDR, 0x05)
    print(f"  Raw mode: {raw_mode}")

    detected = bus.read_byte_data(PICO_I2C_ADDR, 0x14)
    print(f"  Tyre detected: {'YES' if detected else 'NO'}")
    print()

    # Test 2: Enable raw mode
    print("[Test 2] Enabling Raw Mode...")
    bus.write_byte_data(PICO_I2C_ADDR, 0x05, 0x01)
    time.sleep(0.2)  # Wait for next frame

    raw_mode = bus.read_byte_data(PICO_I2C_ADDR, 0x05)
    print(f"  Raw mode: {raw_mode}")

    detected = bus.read_byte_data(PICO_I2C_ADDR, 0x14)
    print(f"  Tyre detected: {'YES' if detected else 'NO'} (should be NO in raw mode)")
    print()

    # Test 3: Read 16 channels
    print("[Test 3] Reading 16 Channels")
    channels = read_16_channels(bus)

    print("  Ch#  Temp(°C)")
    print("  " + "-" * 20)
    for i, temp in enumerate(channels):
        print(f"  {i:2d}   {temp:6.1f}")
    print()

    # Calculate statistics
    avg_temp = sum(channels) / len(channels)
    min_temp = min(channels)
    max_temp = max(channels)
    spread = max_temp - min_temp

    print(f"  Average: {avg_temp:.1f}°C")
    print(f"  Min:     {min_temp:.1f}°C (Ch {channels.index(min_temp)})")
    print(f"  Max:     {max_temp:.1f}°C (Ch {channels.index(max_temp)})")
    print(f"  Spread:  {spread:.1f}°C")
    print()

    # Test 4: Live monitoring
    print("[Test 4] Live 16-Channel Monitoring (5 samples)")
    print("  " + "-" * 78)
    header = "  Sample |"
    for i in range(16):
        header += f" {i:4d}"
    print(header)
    print("  " + "-" * 78)

    for sample in range(5):
        channels = read_16_channels(bus)
        line = f"  {sample + 1:6d} |"
        for temp in channels:
            line += f" {temp:4.0f}"
        print(line)
        time.sleep(0.5)

    print("  " + "-" * 78)
    print()

    # Test 5: Return to normal mode
    print("[Test 5] Disabling Raw Mode...")
    bus.write_byte_data(PICO_I2C_ADDR, 0x05, 0x00)
    time.sleep(0.2)

    raw_mode = bus.read_byte_data(PICO_I2C_ADDR, 0x05)
    detected = bus.read_byte_data(PICO_I2C_ADDR, 0x14)

    print(f"  Raw mode: {raw_mode}")
    print(f"  Tyre detected: {'YES' if detected else 'NO'} (algorithm running again)")
    print()

    print("=" * 60)
    print("✓ Raw mode test complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Test fallback mode feature
"""

from EasyMCP2221 import SMBus
import time

PICO_I2C_ADDR = 0x08

def read_int16_le(data):
    """Convert little-endian bytes to signed int16"""
    val = data[0] | (data[1] << 8)
    if val > 32767:
        val -= 65536
    return val

def read_temps(bus):
    """Read all zone median temperatures"""
    data = bus.read_i2c_block_data(PICO_I2C_ADDR, 0x20, 6)
    left = read_int16_le(data[0:2]) / 10.0
    centre = read_int16_le(data[2:4]) / 10.0
    right = read_int16_le(data[4:6]) / 10.0
    return left, centre, right

def main():
    print("=" * 60)
    print("Fallback Mode Test")
    print("=" * 60)
    print()

    bus = SMBus()

    # Check if tyre is detected
    detected = bus.read_byte_data(PICO_I2C_ADDR, 0x14)
    print(f"Tyre detected: {'YES' if detected else 'NO'}")
    print()

    # Test with fallback mode OFF (default)
    print("[Test 1] Fallback mode OFF (default)")
    fallback = bus.read_byte_data(PICO_I2C_ADDR, 0x03)
    print(f"  Fallback mode register: {fallback}")

    left, centre, right = read_temps(bus)
    print(f"  Left:   {left:.1f}°C")
    print(f"  Centre: {centre:.1f}°C")
    print(f"  Right:  {right:.1f}°C")
    print()

    # Enable fallback mode
    print("[Test 2] Enabling fallback mode...")
    bus.write_byte_data(PICO_I2C_ADDR, 0x03, 0x01)
    time.sleep(0.1)

    fallback = bus.read_byte_data(PICO_I2C_ADDR, 0x03)
    print(f"  Fallback mode register: {fallback}")

    left, centre, right = read_temps(bus)
    print(f"  Left:   {left:.1f}°C")
    print(f"  Centre: {centre:.1f}°C")
    print(f"  Right:  {right:.1f}°C")

    if not detected:
        if left == centre and right == centre:
            print("  ✓ SUCCESS: Left and right match centre!")
        else:
            print("  ✗ FAIL: Left and right don't match centre")
    print()

    # Disable fallback mode
    print("[Test 3] Disabling fallback mode...")
    bus.write_byte_data(PICO_I2C_ADDR, 0x03, 0x00)
    time.sleep(0.1)

    fallback = bus.read_byte_data(PICO_I2C_ADDR, 0x03)
    print(f"  Fallback mode register: {fallback}")

    left, centre, right = read_temps(bus)
    print(f"  Left:   {left:.1f}°C")
    print(f"  Centre: {centre:.1f}°C")
    print(f"  Right:  {right:.1f}°C")

    if not detected:
        if left == 0.0 and right == 0.0:
            print("  ✓ SUCCESS: Left and right are zero!")
        else:
            print("  ✗ FAIL: Left and right should be zero")
    print()

    print("=" * 60)
    print("Fallback mode test complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()

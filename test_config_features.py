#!/usr/bin/env python3
"""
Test fallback mode and emissivity adjustment features
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
    print("Configuration Features Test")
    print("=" * 60)
    print()

    bus = SMBus()

    # Check if tyre is detected
    detected = bus.read_byte_data(PICO_I2C_ADDR, 0x14)
    print(f"Tyre detected: {'YES' if detected else 'NO'}")
    print()

    # Test 1: Emissivity
    print("[Test 1] Emissivity Configuration")
    emiss = bus.read_byte_data(PICO_I2C_ADDR, 0x04)
    print(f"  Default emissivity: {emiss / 100.0}")

    left, centre, right = read_temps(bus)
    print(f"  Temps at default: Centre={centre:.1f}°C")

    # Change emissivity
    print(f"  Setting emissivity to 0.90...")
    bus.write_byte_data(PICO_I2C_ADDR, 0x04, 90)
    time.sleep(0.2)  # Wait for next frame

    left, centre, right = read_temps(bus)
    print(f"  Temps at 0.90: Centre={centre:.1f}°C")

    # Restore default
    print(f"  Restoring emissivity to 0.95...")
    bus.write_byte_data(PICO_I2C_ADDR, 0x04, 95)
    time.sleep(0.2)

    left, centre, right = read_temps(bus)
    print(f"  Temps at 0.95: Centre={centre:.1f}°C")
    print()

    # Test 2: Fallback Mode
    print("[Test 2] Fallback Mode")

    if not detected:
        # Test with fallback OFF
        bus.write_byte_data(PICO_I2C_ADDR, 0x03, 0x00)
        time.sleep(0.1)

        left, centre, right = read_temps(bus)
        print(f"  Fallback OFF:")
        print(f"    Left:   {left:.1f}°C")
        print(f"    Centre: {centre:.1f}°C")
        print(f"    Right:  {right:.1f}°C")

        if left == 0.0 and right == 0.0:
            print(f"    ✓ Correct: Left/Right are zero")
        else:
            print(f"    ✗ Error: Left/Right should be zero")

        # Test with fallback ON
        bus.write_byte_data(PICO_I2C_ADDR, 0x03, 0x01)
        time.sleep(0.1)

        left, centre, right = read_temps(bus)
        print(f"  Fallback ON:")
        print(f"    Left:   {left:.1f}°C")
        print(f"    Centre: {centre:.1f}°C")
        print(f"    Right:  {right:.1f}°C")

        if left == centre and right == centre:
            print(f"    ✓ Correct: Left/Right match centre")
        else:
            print(f"    ✗ Error: Left/Right should match centre")

        # Restore default
        bus.write_byte_data(PICO_I2C_ADDR, 0x03, 0x00)
    else:
        print(f"  Skipped: Only works when no tyre detected")

    print()

    # Test 3: Combined Features
    print("[Test 3] Combined Features")
    bus.write_byte_data(PICO_I2C_ADDR, 0x03, 0x01)  # Fallback ON
    bus.write_byte_data(PICO_I2C_ADDR, 0x04, 97)    # Emissivity 0.97
    time.sleep(0.2)

    emiss = bus.read_byte_data(PICO_I2C_ADDR, 0x04)
    fallback = bus.read_byte_data(PICO_I2C_ADDR, 0x03)
    left, centre, right = read_temps(bus)

    print(f"  Emissivity: {emiss / 100.0}")
    print(f"  Fallback: {'ON' if fallback else 'OFF'}")
    print(f"  Temps: L={left:.1f}°C, C={centre:.1f}°C, R={right:.1f}°C")

    # Restore defaults
    bus.write_byte_data(PICO_I2C_ADDR, 0x03, 0x00)
    bus.write_byte_data(PICO_I2C_ADDR, 0x04, 95)
    print(f"  ✓ Defaults restored")
    print()

    # Test 4: Read all config registers
    print("[Test 4] Configuration Register Summary")
    i2c_addr = bus.read_byte_data(PICO_I2C_ADDR, 0x00)
    output_mode = bus.read_byte_data(PICO_I2C_ADDR, 0x01)
    frame_rate = bus.read_byte_data(PICO_I2C_ADDR, 0x02)
    fallback = bus.read_byte_data(PICO_I2C_ADDR, 0x03)
    emiss = bus.read_byte_data(PICO_I2C_ADDR, 0x04)

    print(f"  I2C Address:   0x{i2c_addr:02X}")
    print(f"  Output Mode:   0x{output_mode:02X}")
    print(f"  Frame Rate:    {frame_rate}")
    print(f"  Fallback Mode: {fallback}")
    print(f"  Emissivity:    {emiss / 100.0}")
    print()

    print("=" * 60)
    print("✓ All configuration tests complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()

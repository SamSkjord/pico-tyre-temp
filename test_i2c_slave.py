#!/usr/bin/env python3
"""
Test script for Pico I2C slave mode using USB I2C debugger
Compatible with SB Components USB-UART-I2C-Debugger
"""

import serial
import serial.tools.list_ports
import time
import sys

PICO_I2C_ADDR = 0x08

def find_debugger():
    """Find USB I2C debugger port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Look for CH340/CH341 or similar USB serial
        if "usb" in port.device.lower() or "serial" in port.device.lower():
            print(f"Found potential debugger: {port.device} - {port.description}")
            return port.device
    return None

def i2c_write(ser, addr, reg, data=None):
    """Write to I2C device"""
    # Command format for SB Components debugger:
    # W <addr> <reg> [data...]
    if data is None:
        cmd = f"W {addr:02X} {reg:02X}\r\n"
    else:
        data_str = " ".join([f"{b:02X}" for b in data])
        cmd = f"W {addr:02X} {reg:02X} {data_str}\r\n"

    ser.write(cmd.encode())
    time.sleep(0.01)
    return ser.readline().decode().strip()

def i2c_read(ser, addr, reg, length):
    """Read from I2C device"""
    # Command format: R <addr> <reg> <length>
    cmd = f"R {addr:02X} {reg:02X} {length:02X}\r\n"
    ser.write(cmd.encode())
    time.sleep(0.05)

    response = ser.readline().decode().strip()
    # Parse hex response
    if response:
        try:
            # Response format: "OK: AA BB CC DD"
            if "OK" in response or ":" in response:
                hex_str = response.split(":")[-1].strip()
                bytes_list = [int(x, 16) for x in hex_str.split()]
                return bytes_list
        except:
            pass
    return None

def read_temp_int16(ser, addr, reg):
    """Read int16 temperature in tenths of degrees"""
    data = i2c_read(ser, addr, reg, 2)
    if data and len(data) == 2:
        temp_tenths = data[0] | (data[1] << 8)
        # Handle signed int16
        if temp_tenths > 32767:
            temp_tenths -= 65536
        return temp_tenths / 10.0
    return None

def main():
    print("=" * 60)
    print("Pico I2C Slave Test - USB I2C Debugger")
    print("=" * 60)
    print()

    # Find debugger
    port = find_debugger()
    if not port:
        print("ERROR: USB I2C debugger not found")
        print("\nPlease check:")
        print("1. Debugger is plugged in")
        print("2. Drivers are installed (CH340/CH341)")
        print("3. Run: ls /dev/tty.*")
        sys.exit(1)

    print(f"Using debugger on: {port}\n")

    try:
        # Open serial connection
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(1)

        print("Testing I2C slave at address 0x08...\n")

        # Test 1: Read firmware version
        print("[Test 1] Read Firmware Version")
        data = i2c_read(ser, PICO_I2C_ADDR, 0x10, 1)
        if data:
            print(f"  Firmware Version: {data[0]}")
        else:
            print("  ERROR: No response from slave!")
            print("\nTroubleshooting:")
            print("1. Check wiring: GP26=SDA, GP27=SCL")
            print("2. Check pull-ups (4.7kΩ on SDA/SCL)")
            print("3. Verify Pico is running (check USB serial)")
            sys.exit(1)

        # Test 2: Read FPS
        print("\n[Test 2] Read Current FPS")
        data = i2c_read(ser, PICO_I2C_ADDR, 0x13, 1)
        if data:
            print(f"  FPS: {data[0]}")

        # Test 3: Read detection status
        print("\n[Test 3] Read Detection Status")
        data = i2c_read(ser, PICO_I2C_ADDR, 0x14, 6)
        if data:
            detected = data[0]
            confidence = data[1]
            width = data[2]
            span_start = data[3]
            span_end = data[4]
            warnings = data[5]

            print(f"  Detected: {'YES' if detected else 'NO'}")
            print(f"  Confidence: {confidence}%")
            print(f"  Tyre Width: {width} pixels")
            print(f"  Span: {span_start}-{span_end}")
            print(f"  Warnings: {warnings}")

        # Test 4: Read median temperatures
        print("\n[Test 4] Read Median Temperatures")
        left = read_temp_int16(ser, PICO_I2C_ADDR, 0x20)
        centre = read_temp_int16(ser, PICO_I2C_ADDR, 0x22)
        right = read_temp_int16(ser, PICO_I2C_ADDR, 0x24)

        if left is not None and centre is not None and right is not None:
            print(f"  Left:   {left:.1f}°C")
            print(f"  Centre: {centre:.1f}°C")
            print(f"  Right:  {right:.1f}°C")

        # Test 5: Continuous monitoring
        print("\n[Test 5] Continuous Monitoring (10 readings)")
        print("  Time   | FPS | Centre °C | Detected | Confidence")
        print("  " + "-" * 55)

        for i in range(10):
            fps_data = i2c_read(ser, PICO_I2C_ADDR, 0x13, 1)
            centre_temp = read_temp_int16(ser, PICO_I2C_ADDR, 0x22)
            det_data = i2c_read(ser, PICO_I2C_ADDR, 0x14, 2)

            if fps_data and centre_temp is not None and det_data:
                fps = fps_data[0]
                detected = "YES" if det_data[0] else "NO "
                confidence = det_data[1]

                print(f"  {i+1:2d}     | {fps:3d} | {centre_temp:7.1f}  | {detected}      | {confidence:3d}%")

            time.sleep(0.5)

        print("\n" + "=" * 60)
        print("All tests completed successfully!")
        print("=" * 60)

    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()

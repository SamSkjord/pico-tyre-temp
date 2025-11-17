# I2C Slave Mode Documentation

The Pico acts as an I2C slave/peripheral device, allowing other microcontrollers to read thermal tyre data over I2C.

## Hardware Configuration

**Default I2C Slave Address:** `0x08`

**Pins:**
```
GP26 → I2C1 SDA (Pin 31)
GP27 → I2C1 SCL (Pin 32)
```

**Note:** The MLX90640 sensor uses I2C0 (GP0/GP1), so I2C1 is available for slave mode.

## Output Modes

The Pico supports multiple output modes (configurable via register `0x01`):

| Mode | Value | Description |
|------|-------|-------------|
| USB Serial | `0x00` | USB serial output (default) |
| I2C Slave | `0x01` | I2C slave only |
| CAN Bus | `0x02` | CAN bus (future) |
| All | `0xFF` | All outputs enabled |

## Register Map

### Configuration Registers (0x00-0x0F) - Read/Write

| Address | Name | Type | Description |
|---------|------|------|-------------|
| `0x00` | I2C_ADDRESS | uint8 | I2C slave address (7-bit, requires restart) |
| `0x01` | OUTPUT_MODE | uint8 | Output mode select (see above) |
| `0x02` | FRAME_RATE | uint8 | Target frame rate (reserved for future) |
| `0x03-0x0F` | RESERVED | - | Reserved for future use |

### Status Registers (0x10-0x1F) - Read Only

| Address | Name | Type | Description |
|---------|------|------|-------------|
| `0x10` | FIRMWARE_VERSION | uint8 | Firmware version (currently 0x01) |
| `0x11` | FRAME_NUMBER_L | uint8 | Frame counter low byte |
| `0x12` | FRAME_NUMBER_H | uint8 | Frame counter high byte |
| `0x13` | FPS | uint8 | Current FPS (integer) |
| `0x14` | DETECTED | uint8 | Tyre detected flag (0=no, 1=yes) |
| `0x15` | CONFIDENCE | uint8 | Detection confidence (0-100%) |
| `0x16` | TYRE_WIDTH | uint8 | Tyre width in pixels (0-32) |
| `0x17` | SPAN_START | uint8 | Tyre span start pixel |
| `0x18` | SPAN_END | uint8 | Tyre span end pixel |
| `0x19` | WARNINGS | uint8 | Warning flags |

### Temperature Data Registers (0x20-0x3F) - Read Only

All temperatures stored as **int16 in tenths of degrees Celsius** (little-endian).

| Address | Name | Type | Description |
|---------|------|------|-------------|
| `0x20-0x21` | LEFT_MEDIAN | int16 | Left zone median temp (tenths °C) |
| `0x22-0x23` | CENTRE_MEDIAN | int16 | Centre zone median temp |
| `0x24-0x25` | RIGHT_MEDIAN | int16 | Right zone median temp |
| `0x26-0x27` | LEFT_AVG | int16 | Left zone average temp |
| `0x28-0x29` | CENTRE_AVG | int16 | Centre zone average temp |
| `0x2A-0x2B` | RIGHT_AVG | int16 | Right zone average temp |
| `0x2C-0x2D` | LATERAL_GRADIENT | int16 | Lateral gradient (tenths °C) |

**Temperature Format:** `int16_value / 10.0 = degrees Celsius`

Example: `0x0119` = 281 tenths = 28.1°C

### Full Frame Access (0x40+) - Read Only

| Address | Name | Description |
|---------|------|-------------|
| `0x40` | FRAME_ACCESS | Frame read pointer (reserved) |
| `0x41` | FRAME_DATA_START | Streaming full frame data |

Reading from `0x41` returns full 768-pixel frame as int16 tenths (1536 bytes total).
Auto-increments through frame data with each read.

### Command Register (0xFF) - Write Only

| Value | Command | Description |
|-------|---------|-------------|
| `0x01` | RESET | Software reset (reserved) |
| `0x02` | CLEAR_WARNINGS | Clear warning flags |
| `0x10` | FRAME_REQUEST | Request new frame capture (reserved) |

## Usage Examples

### Example 1: Read Median Temperatures

```python
import smbus

bus = smbus.SMBus(1)  # I2C bus 1
PICO_ADDR = 0x08

# Read centre median temperature
data = bus.read_i2c_block_data(PICO_ADDR, 0x22, 2)
temp_tenths = (data[1] << 8) | data[0]  # Little-endian
temp_celsius = temp_tenths / 10.0
print(f"Centre median: {temp_celsius}°C")
```

### Example 2: Check Tyre Detection

```python
# Read detection status
detected = bus.read_byte_data(PICO_ADDR, 0x14)
confidence = bus.read_byte_data(PICO_ADDR, 0x15)
width = bus.read_byte_data(PICO_ADDR, 0x16)

if detected:
    print(f"Tyre detected: width={width}px, confidence={confidence}%")
else:
    print("No tyre detected")
```

### Example 3: Read All Zone Medians

```python
# Read left, centre, right medians in one transaction
data = bus.read_i2c_block_data(PICO_ADDR, 0x20, 6)

left = ((data[1] << 8) | data[0]) / 10.0
centre = ((data[3] << 8) | data[2]) / 10.0
right = ((data[5] << 8) | data[4]) / 10.0

print(f"Left: {left}°C, Centre: {centre}°C, Right: {right}°C")
```

### Example 4: Read Full Frame (Debug/Alignment)

```python
# Read all 768 pixels (1536 bytes as int16 tenths)
frame = []
for i in range(768):
    data = bus.read_i2c_block_data(PICO_ADDR, 0x41, 2)
    temp_tenths = (data[1] << 8) | data[0]
    temp_celsius = temp_tenths / 10.0
    frame.append(temp_celsius)

# Reshape to 24x32
import numpy as np
thermal_image = np.array(frame).reshape(24, 32)
```

### Example 5: Change Output Mode

```python
# Disable USB serial, enable I2C slave only
bus.write_byte_data(PICO_ADDR, 0x01, 0x01)  # OUTPUT_MODE_I2C_SLAVE

# Enable both USB and I2C
bus.write_byte_data(PICO_ADDR, 0x01, 0xFF)  # OUTPUT_MODE_ALL
```

## Arduino Example

```cpp
#include <Wire.h>

#define PICO_ADDR 0x08

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  // Read centre median temperature
  Wire.beginTransmission(PICO_ADDR);
  Wire.write(0x22);  // CENTRE_MEDIAN register
  Wire.endTransmission();

  Wire.requestFrom(PICO_ADDR, 2);
  int16_t temp_tenths = Wire.read() | (Wire.read() << 8);
  float temp_celsius = temp_tenths / 10.0;

  Serial.print("Centre: ");
  Serial.print(temp_celsius);
  Serial.println(" °C");

  delay(100);
}
```

## Protocol Notes

1. **Auto-increment:** Register pointer auto-increments on sequential reads
2. **Little-endian:** All multi-byte values are little-endian (LSB first)
3. **Temperature format:** All temps as int16 tenths (divide by 10 for °C)
4. **Frame access:** Reading from 0x41 streams full frame with auto-increment
5. **Non-blocking:** I2C slave uses interrupts, doesn't block main loop

## Troubleshooting

**No response from slave:**
- Check wiring (GP26=SDA, GP27=SCL)
- Verify address is 0x08
- Ensure pull-up resistors (4.7kΩ) on SDA/SCL
- Check I2C master speed (100kHz or 400kHz)

**Incorrect temperature values:**
- Remember to convert from tenths (divide by 10)
- Use little-endian byte order
- Check for signed int16 (can be negative)

**Frame data not updating:**
- Check FPS register (0x13) to verify Pico is running
- Verify OUTPUT_MODE includes I2C (register 0x01)
- Check FRAME_NUMBER to see if frames are incrementing

## CAN Bus (Future)

CAN bus support will be added in a future firmware update. Set `OUTPUT_MODE` to `0x02` (reserved for now).

## Performance

I2C slave mode uses interrupts and doesn't affect the main 11.5 fps performance. Typical I2C read latency: <1ms.

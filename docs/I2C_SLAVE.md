# I2C Slave Mode Documentation

The Pico acts as an I2C slave/peripheral device, allowing other microcontrollers to read thermal tyre data over I2C.

## Version History

**Current Version:** Register map v2 (November 2025)
- **Breaking Change:** Frame access registers moved from 0x40/0x41 → 0x50/0x51
- **Reason:** Avoided collision with raw 16-channel data (0x30-0x4F)
- **Migration:** Update all references to `REG_FRAME_DATA_START` from 0x41 to 0x51

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
| `0x03` | FALLBACK_MODE | uint8 | Fallback mode: 0=zero temps when no tyre, 1=copy centre temp |
| `0x04` | EMISSIVITY | uint8 | Emissivity × 100 (e.g., 95 = 0.95), default 95 |
| `0x05` | RAW_MODE | uint8 | Raw mode: 0=tyre algorithm, 1=16-channel raw data only |
| `0x06` | LASER_ENABLE | uint8 | Laser enable: 0=disabled, 1=enabled (auto-set at boot) |
| `0x07-0x0F` | RESERVED | - | Reserved for future use |

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

All temperatures stored as **signed int16 in tenths of degrees Celsius** (little-endian).
**Range:** -3276.8°C to +3276.7°C (values -32768 to +32767)

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

### Raw 16-Channel Data (0x30-0x4F) - Read Only

Available when `RAW_MODE=1`. Each channel averages 2 columns × 4 middle rows (rows 10-13) from the 32×24 sensor.

| Address | Name | Description |
|---------|------|-------------|
| `0x30-0x31` | CHANNEL_0 | Channel 0 (leftmost) int16 tenths °C |
| `0x32-0x33` | CHANNEL_1 | Channel 1 int16 tenths °C |
| ... | ... | Channels 2-14 follow sequentially |
| `0x4E-0x4F` | CHANNEL_15 | Channel 15 (rightmost) int16 tenths °C |

**Access pattern:** `REG_BASE = 0x30 + (channel_num × 2)`

### Full Frame Access (0x50+) - Read Only

| Address | Name | Description |
|---------|------|-------------|
| `0x50` | FRAME_ACCESS | Frame read pointer (reserved, not implemented) |
| `0x51` | FRAME_DATA_START | Streaming full frame data |

**IMPORTANT:** Register addresses changed from 0x40/0x41 to 0x50/0x51 to avoid collision with raw channel data (0x30-0x4F).

Reading from `0x51` returns full 768-pixel frame as **signed int16 tenths** (1536 bytes total: 768 pixels × 2 bytes).
Auto-increments through frame data with each read. Wraps to start after reading all 1536 bytes.

### Laser Ranger Data (0x60-0x6F) - Read Only

| Address | Name | Type | Description |
|---------|------|------|-------------|
| `0x60` | LASER_STATUS | uint8 | 0=no data, 1=valid reading, 2+=error (error code + 2) |
| `0x61-0x64` | LASER_DIST_MM | uint32 | Distance in millimeters (little-endian) |
| `0x65-0x68` | LASER_DIST_UM | uint32 | Distance in micrometers (little-endian, 0.001mm precision) |
| `0x69` | LASER_ERROR | uint8 | Last error code (0=OK, see error codes below) |
| `0x6A-0x6B` | LASER_VALID_CNT | uint16 | Count of valid measurements (little-endian) |
| `0x6C-0x6D` | LASER_ERROR_CNT | uint16 | Count of errors (little-endian) |
| `0x6E` | LASER_ENABLED | uint8 | Laser enabled flag (1=enabled) |
| `0x6F` | RESERVED | - | Reserved |

**Laser Error Codes:**
| Code | Meaning |
|------|---------|
| 0x00 | OK - No error |
| 0x01 | No data yet |
| 0x14 | Calculation error |
| 0x15 | Laser low power |
| 0x18 | Weak signal or measurement too long |
| 0x1E | Low power |
| 0x20 | Strong ambient light |
| 0x74 | Out of range |
| 0xFE | Parse error |
| 0xFF | Timeout (triggers auto-recovery) |

**Auto-Detection & Recovery:**
- Laser is auto-detected at boot (500ms timeout, validates 0x80 address byte)
- If detected, enable register (0x06) is set to 1
- If not detected, enable register is set to 0
- If laser stops responding for 3 seconds, non-blocking recovery is attempted
- Recovery uses state machine (no blocking delays in main loop)
- Recovery also triggers if laser never responds after being enabled

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
import struct

bus = smbus.SMBus(1)  # I2C bus 1
PICO_ADDR = 0x08

# Read centre median temperature
data = bus.read_i2c_block_data(PICO_ADDR, 0x22, 2)

# Convert to signed int16 (little-endian)
temp_tenths = struct.unpack('<h', bytes(data))[0]
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
import struct

# Read left, centre, right medians in one transaction
data = bus.read_i2c_block_data(PICO_ADDR, 0x20, 6)

# Unpack three signed int16 values (little-endian)
left_tenths, centre_tenths, right_tenths = struct.unpack('<hhh', bytes(data))

left = left_tenths / 10.0
centre = centre_tenths / 10.0
right = right_tenths / 10.0

print(f"Left: {left}°C, Centre: {centre}°C, Right: {right}°C")
```

### Example 4: Read Full Frame (Debug/Alignment)

```python
import struct
import numpy as np

# Read all 768 pixels (1536 bytes as signed int16 tenths)
# Use register 0x51 (changed from 0x41 to avoid collision with raw channels)

# Method 1: Read all at once (if supported by your I2C master)
try:
    data = bus.read_i2c_block_data(PICO_ADDR, 0x51, 1536)
    # Unpack 768 signed int16 values (little-endian)
    temps_tenths = struct.unpack('<768h', bytes(data))
    frame = [t / 10.0 for t in temps_tenths]
except:
    # Method 2: Read 2 bytes at a time (slower but more compatible)
    frame = []
    for i in range(768):
        data = bus.read_i2c_block_data(PICO_ADDR, 0x51, 2)
        temp_tenths = struct.unpack('<h', bytes(data))[0]  # Little-endian signed int16
        temp_celsius = temp_tenths / 10.0
        frame.append(temp_celsius)

# Reshape to 24x32 (24 rows × 32 columns)
thermal_image = np.array(frame).reshape(24, 32)
```

### Example 5: Change Output Mode

```python
# Disable USB serial, enable I2C slave only
bus.write_byte_data(PICO_ADDR, 0x01, 0x01)  # OUTPUT_MODE_I2C_SLAVE

# Enable both USB and I2C
bus.write_byte_data(PICO_ADDR, 0x01, 0xFF)  # OUTPUT_MODE_ALL
```

### Example 6: Enable Fallback Mode

```python
# Enable fallback mode - when no tyre detected, copy centre temp to left/right
bus.write_byte_data(PICO_ADDR, 0x03, 0x01)  # FALLBACK_MODE = 1

# Now read temps - left/right will match centre when no tyre detected
data = bus.read_i2c_block_data(PICO_ADDR, 0x20, 6)
left = ((data[1] << 8) | data[0]) / 10.0
centre = ((data[3] << 8) | data[2]) / 10.0
right = ((data[5] << 8) | data[4]) / 10.0

# Disable fallback mode - left/right will be 0.0 when no tyre
bus.write_byte_data(PICO_ADDR, 0x03, 0x00)  # FALLBACK_MODE = 0
```

### Example 7: Adjust Emissivity

```python
# Read current emissivity
emiss = bus.read_byte_data(PICO_ADDR, 0x04)
print(f"Current emissivity: {emiss / 100.0}")  # Default: 0.95

# Adjust for different tyre compound
# Soft compound tyres: 0.93
bus.write_byte_data(PICO_ADDR, 0x04, 93)

# Medium compound tyres: 0.95 (default)
bus.write_byte_data(PICO_ADDR, 0x04, 95)

# Hard compound tyres: 0.97
bus.write_byte_data(PICO_ADDR, 0x04, 97)

# Wet weather tyres: 0.98
bus.write_byte_data(PICO_ADDR, 0x04, 98)
```

### Example 8: Raw 16-Channel Mode

```python
import struct

# Enable raw mode - skips tyre detection algorithm
bus.write_byte_data(PICO_ADDR, 0x05, 0x01)  # RAW_MODE = 1

# Read all 16 channels (32 bytes as signed int16)
data = bus.read_i2c_block_data(PICO_ADDR, 0x30, 32)

# Unpack 16 signed int16 values (little-endian)
temps_tenths = struct.unpack('<16h', bytes(data))
channels = [t / 10.0 for t in temps_tenths]

print("16-Channel Temperature Profile:")
for i, temp in enumerate(channels):
    print(f"  Channel {i:2d}: {temp:6.1f}°C")

# Disable raw mode - return to tyre algorithm
bus.write_byte_data(PICO_ADDR, 0x05, 0x00)  # RAW_MODE = 0
```

### Example 9: Read Laser Distance

```python
import struct

# Read laser status first
status = bus.read_byte_data(PICO_ADDR, 0x60)

if status == 0:
    print("Laser: No data yet")
elif status == 1:
    # Valid reading - read distance in mm (4 bytes at 0x61-0x64)
    data = bus.read_i2c_block_data(PICO_ADDR, 0x61, 4)
    distance_mm = struct.unpack('<I', bytes(data))[0]

    # Or read distance in um for higher precision (4 bytes at 0x65-0x68)
    data = bus.read_i2c_block_data(PICO_ADDR, 0x65, 4)
    distance_um = struct.unpack('<I', bytes(data))[0]

    print(f"Laser distance: {distance_mm} mm ({distance_um/1000:.3f} mm)")
else:
    # Error - status is error_code + 2
    error_code = status - 2
    print(f"Laser error: 0x{error_code:02X}")
```

### Example 10: Monitor Laser Health

```python
import struct

# Read laser counters
data = bus.read_i2c_block_data(PICO_ADDR, 0x6A, 4)
valid_count, error_count = struct.unpack('<HH', bytes(data))

enabled = bus.read_byte_data(PICO_ADDR, 0x6E)
last_error = bus.read_byte_data(PICO_ADDR, 0x69)

print(f"Laser enabled: {enabled}")
print(f"Valid readings: {valid_count}")
print(f"Error count: {error_count}")
print(f"Last error: 0x{last_error:02X}")

if valid_count + error_count > 0:
    success_rate = valid_count / (valid_count + error_count) * 100
    print(f"Success rate: {success_rate:.1f}%")
```

### Example 11: Enable/Disable Laser

```python
# Check if laser was detected at boot
laser_enabled = bus.read_byte_data(PICO_ADDR, 0x06)
print(f"Laser enabled: {laser_enabled}")

# Disable laser (e.g., to save power)
bus.write_byte_data(PICO_ADDR, 0x06, 0x00)

# Re-enable laser
bus.write_byte_data(PICO_ADDR, 0x06, 0x01)
```

**Note:** The laser is auto-detected at boot. If no laser is connected, the enable register (0x06) is set to 0. You can enable it later if a laser is connected, but the Pico won't detect it automatically after boot.

## Arduino Example

```cpp
#include <Wire.h>

#define PICO_ADDR 0x08

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  // Read centre median temperature (register 0x22-0x23)
  Wire.beginTransmission(PICO_ADDR);
  Wire.write(0x22);  // CENTRE_MEDIAN_L register
  Wire.endTransmission();

  // Request 2 bytes (little-endian signed int16)
  Wire.requestFrom(PICO_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t low_byte = Wire.read();
    uint8_t high_byte = Wire.read();
    // Combine bytes (little-endian) and handle sign
    int16_t temp_tenths = (int16_t)((high_byte << 8) | low_byte);
    float temp_celsius = temp_tenths / 10.0;

    Serial.print("Centre: ");
    Serial.print(temp_celsius);
    Serial.println(" °C");
  }

  delay(100);
}
```

## Protocol Notes

1. **Auto-increment:** Register pointer auto-increments on sequential reads
2. **Little-endian:** All multi-byte values are little-endian (LSB first)
   - Read LOW byte first, then HIGH byte
   - Combine as: `value = (high << 8) | low`
   - Python: `struct.unpack('<h', bytes([low, high]))[0]`
3. **Temperature format:** All temps as **signed** int16 tenths (divide by 10 for °C)
   - Range: -3276.8°C to +3276.7°C
   - Negative temperatures are properly supported
4. **Frame access:** Reading from 0x51 streams full frame with auto-increment
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

**Frame counter overflow:**
- I2C registers store 16-bit counter (0-65,535)
- Overflows after ~1.6 hours at 11.5 fps
- Wraps cleanly: 65535 → 0
- Detect wraps by checking for frame number decrease
- Internal counter is 32-bit (10+ years before overflow)

**MLX90640 sensor dropout:**
- Firmware detects and recovers automatically
- Prints error message (if USB serial enabled)
- Retries every 100ms indefinitely
- I2C registers retain last valid data during dropout
- Resumes normal operation when sensor reconnects
- No restart required

## CAN Bus (Future)

CAN bus support will be added in a future firmware update. Set `OUTPUT_MODE` to `0x02` (reserved for now).

## Performance

### Frame Rate Breakdown

Actual measured performance at 11.5 fps (~87ms total per frame):

| Component | Time | Notes |
|-----------|------|-------|
| **Sensor read** | **~60ms** | Measured: includes MLX90640_GetFrameData call |
| **Temperature calc** | **~20ms** | Measured: MLX90640_CalculateTo (floating point math) |
| **Tyre algorithm** | **~2ms** | Detection, zone calculations |
| **USB serial output** | **~3ms** | printf, fflush - *removable overhead* |
| **I2C slave update** | **<1ms** | Interrupt-driven, non-blocking |
| **Total** | **~87ms** | **= 11.5 fps actual measured performance** |

**Note:** MLX90640 sensor has theoretical 16Hz (62.5ms) refresh rate, but actual frame acquisition
varies based on I2C bus speed and sensor internal timing. Total time also includes algorithm overhead.

**Theoretical minimum** with no algorithm or serial output: ~80ms (~12.5 fps)

### Performance Optimization

**To maximize performance, disable USB serial output:**

```python
# Disable USB serial - I2C only mode
bus.write_byte_data(PICO_ADDR, 0x01, 0x01)  # OUTPUT_MODE_I2C_SLAVE
```

**Expected gains:**
- Remove ~3-7ms serial overhead per frame
- Remove debug timing printf (~1-2ms every 10 frames)
- Performance: **11.5 fps → 11.8 fps** (minor but free)

**When to use I2C-only mode:**
- Production deployment with embedded controller
- Maximum performance needed
- No need for USB debugging/visualization
- Running on battery (lower USB power consumption)

**When to use USB serial:**
- Development and debugging
- Visualization with Python visualizer
- Data logging to PC
- Monitoring tyre detection in real-time

**Raw mode performance:**
- Enabling `RAW_MODE=1` skips tyre detection algorithm
- Saves ~5ms per frame (algorithm time)
- Best for custom processing or alignment/debugging
- Performance: **11.5 fps → 11.6 fps**

### I2C Performance

- I2C slave uses interrupts - zero impact on main loop
- Typical I2C read latency: <1ms
- Register updates: <0.1ms per frame
- Supports 100kHz and 400kHz I2C bus speeds

### I2C Reliability (v1.1+)

**Minimal Critical Section:**
- All float operations and channel averaging are pre-calculated before disabling interrupts
- Critical section (interrupts disabled) reduced from ~100µs to <10µs
- This prevents I2C NACKs when the master polls during register updates
- If the master reads during an update, the slave can always respond promptly

**Watchdog Timer:**
- 5-second watchdog timer enabled - auto-reboots if main loop hangs
- Protects against I2C bus lockups caused by sensor read failures
- Logs "WARNING: Rebooted by watchdog" on startup if previous hang detected
- Watchdog is paused during debug sessions (pause_on_debug=1)

**Best Practices for Masters:**
- Use short I2C timeouts (100-500ms) to detect unresponsive slaves
- Implement retry logic with exponential backoff
- If using I2C mux (TCA9548A), select channel before each transaction
- Avoid polling faster than the Pico's frame rate (~10 Hz)

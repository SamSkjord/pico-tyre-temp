# Pico Tyre Temperature Monitor

**High-performance thermal tyre monitoring for Raspberry Pi Pico**

Real-time thermal imaging at 11.5 fps with automatic tyre detection and live visualization.

## Performance

| Version | Frame Rate | Frame Time | Speedup |
|---------|-----------|------------|---------|
| CircuitPython | 1.5 fps | 640ms | Baseline |
| **C Version** | **11.5 fps** | **87ms** | **7.6x faster** ✅ |

**Target exceeded:** 11.5 Hz achieved! 🎯

## Why C?

The CircuitPython version is limited by:
- Python interpreter overhead (~500ms)
- Slow math operations (sqrt, sort)
- I2C library overhead

The C version eliminates all of this through:
- Compiled native ARM code
- Fast math operations
- Direct hardware access
- Optimized algorithm (-O3, -ffast-math)

## Quick Start

### 1. Install Prerequisites

**macOS:**
```bash
brew install cmake
brew install --cask gcc-arm-embedded

# Set Pico SDK path
export PICO_SDK_PATH=~/pico-sdk
```

See [docs/BUILD.md](docs/BUILD.md) for Linux/Windows instructions.

### 2. Download and Build

```bash
# Download MLX90640 library
chmod +x tools/download_mlx_library.sh
./tools/download_mlx_library.sh

# Build
mkdir build && cd build
cmake ..
make -j4
```

### 3. Upload to Pico

1. Hold **BOOTSEL** button and plug in USB
2. Copy `build/thermal_tyre_pico.uf2` to RPI-RP2 drive
3. Pico reboots and starts running!

### 4. Connect Serial

```bash
screen /dev/tty.usbmodem* 115200
```

You should see:
```
========================================
Thermal Tyre Driver - C Version
========================================

Initializing I2C...
Detecting MLX90640 sensor at 0x33...
Sensor detected! Extracting calibration parameters...
Setting refresh rate to 16Hz...
Sensor initialized successfully!
Expected performance: 5-10Hz frame rate

========================================
Starting thermal sensing loop...
Output: Compact CSV
========================================

1,45.2,48.5,44.8,0.85,0
2,45.3,48.6,44.9,0.86,0
[Frame 10] Total: 138.2ms (7.2 fps) | Sensor: 125.3ms | Calc: 8.1ms | Algo: 3.2ms | Comm: 1.6ms
...
```

## Features

- ✅ **Fast**: 11.5 fps (vs 1.5 fps CircuitPython) - 7.6x faster!
- ✅ **Reliable**: Zero malloc - all static allocation
- ✅ **Stable**: Sensor stabilization for reliable cold boot
- ✅ **Safe**: NaN/Inf protection on all float outputs
- ✅ **Visualizer**: Real-time matplotlib visualization included
- ✅ **Flexible**: CSV and JSON output formats

## Output Formats

### Compact CSV (default)
```csv
Frame,FPS,L_avg,L_med,C_avg,C_med,R_avg,R_med,Width,Conf,Det
1,11.5,0.0,0.0,25.6,25.6,0.0,0.0,32,0.00,0
2,11.5,15.4,15.4,15.4,15.4,13.4,13.4,8,0.22,1
```

### Full JSON (optional)
```json
{
  "frame_number": 123,
  "fps": 11.5,
  "analysis": {
    "left": {"avg": 25.1, "median": 25.0, "mad": 0.5, "min": 24.0, "max": 26.0, "range": 2.0},
    "centre": {"avg": 30.2, "median": 30.1, "mad": 0.8, "min": 28.0, "max": 32.0, "range": 4.0},
    "right": {"avg": 28.5, "median": 28.3, "mad": 0.6, "min": 27.0, "max": 30.0, "range": 3.0},
    "lateral_gradient": 3.4
  },
  "detection": {
    "detected": 1,
    "span_start": 8,
    "span_end": 23,
    "tyre_width": 15,
    "confidence": 0.95
  },
  "temperature_profile": [20.1, 20.5, ...],
  "warnings": []
}
```

Change `COMPACT_OUTPUT` in `main.c` to switch formats.

## Hardware Requirements

Same as CircuitPython version:
- Raspberry Pi Pico or Pico W
- MLX90640 thermal camera
- I2C pull-ups (4.7kΩ)

### Wiring

```
MLX90640        Pico
--------        ----
VDD      →      3V3 (Pin 36)
GND      →      GND (Pin 38)
SDA      →      GP0 (Pin 1)
SCL      →      GP1 (Pin 2)
```

## Algorithm

Same thermal tyre detection as Python version:
1. Extract middle rows from 24x32 sensor
2. Calculate profile statistics (median, MAD)
3. Region growing to find tyre span
4. Split into left/center/right zones
5. Calculate zone statistics
6. Detect anomalies (high gradient, variance)

**Optimizations:**
- Fast median using qsort
- Efficient MAD calculation
- Minimal memory allocations
- In-place array operations

## File Structure

```
pico-tyre-temp/
├── README.md                   # This file
├── CMakeLists.txt              # Build configuration
│
├── main.c                      # Main application
├── thermal_algorithm.c/h       # Tyre detection algorithm
├── communication.c/h           # Serial + I2C output
├── i2c_slave.c/h              # I2C slave mode implementation
│
├── docs/                       # Documentation
│   ├── BUILD.md               # Detailed build instructions
│   ├── I2C_SLAVE.md           # I2C slave API reference
│   ├── QUICKSTART.md          # Quick start guide
│   └── VISUALIZER.md          # Visualizer usage guide
│
├── tools/                      # Utilities and scripts
│   ├── download_mlx_library.sh  # Download Melexis library
│   ├── build_and_flash.sh      # Build and flash script
│   ├── visualizer.py           # Real-time visualization
│   └── requirements_visualizer.txt
│
├── tests/                      # Python test scripts
│   ├── test_i2c_slave.py      # I2C slave tests
│   └── test_*.py              # Other tests
│
└── mlx90640/                   # External library
    ├── MLX90640_API.c         # Official Melexis library
    ├── MLX90640_API.h
    ├── MLX90640_I2C_Driver.c  # Pico-specific I2C driver
    └── MLX90640_I2C_Driver.h
```

## Performance Breakdown

At 16Hz sensor mode:
```
Component               Time        %
─────────────────────────────────────
Sensor frame capture    24.0ms     28%
Temperature calc        61.5ms     71%
Algorithm processing     1.2ms      1%
Serial output            0.5ms      0%
─────────────────────────────────────
Total                   87.2ms    100%
                       (11.5 fps)
```

**Optimizations applied:**
- All static allocation (no malloc/free)
- NaN/Inf sanitization
- Sensor stabilization delays
- Efficient buffer management

## Comparison with CircuitPython

| Operation | CircuitPython | C Version | Improvement |
|-----------|--------------|-----------|-------------|
| Sensor read | 640ms | 125ms | 5.1x faster |
| Algorithm | 20ms | 3ms | 6.7x faster |
| Output | 2ms | 1ms | 2x faster |
| **Total** | **662ms** | **129ms** | **5.1x faster** |

## Configuration

Edit `thermal_algorithm.c`:

```c
void thermal_algorithm_init(ThermalConfig *config) {
    config->mad_threshold = 3.0f;      // Detection sensitivity
    config->grad_threshold = 5.0f;     // Gradient warning
    config->min_tyre_width = 6;        // Min pixels
    config->max_tyre_width = 28;       // Max pixels
    config->ema_alpha = 0.3f;          // Temporal smoothing (future)
}
```

## Visualization

A real-time matplotlib visualizer is included:

```bash
# Install dependencies
pip install -r tools/requirements_visualizer.txt

# Run visualizer (auto-detects port)
python3 tools/visualizer.py

# Or specify port
python3 tools/visualizer.py --port /dev/tty.usbmodem14201

# List available ports
python3 tools/visualizer.py --list
```

**Displays:**
- Current zone temperatures (bar chart)
- Temperature history (time series)
- Detection confidence meter
- 32-pixel temperature profile
- Lateral temperature gradient
- Statistics and warnings

**Note:** Visualizer displays at ~4 fps (matplotlib limit). The Pico continues collecting at 11.5 fps. Frame skipping is expected and normal.

See [docs/VISUALIZER.md](docs/VISUALIZER.md) for detailed usage.

## Troubleshooting

### Build Errors

**"pico_sdk_import.cmake not found"**
```bash
export PICO_SDK_PATH=~/pico-sdk
```

**"MLX90640_API.c not found"**
```bash
./tools/download_mlx_library.sh
```

### Runtime Errors

**"ERROR: Could not detect MLX90640 sensor"**
- Check wiring (especially 3.3V, NOT 5V!)
- Check I2C pull-ups
- Verify sensor address (0x33)

**Low frame rate (< 4 fps)**
- Verify sensor set to 16Hz mode
- Check I2C speed (1MHz)
- Rebuild with -O3 optimization

## Development

### Modify Algorithm

1. Edit `thermal_algorithm.c`
2. Rebuild: `cd build && make`
3. Upload new .uf2

### Add Debug Output

```c
printf("Debug: temp=%.1f\n", temperature);
```

### Change Output Format

Edit `main.c`:
```c
#define COMPACT_OUTPUT 0  // Switch to JSON
```

## Future Improvements

- [x] I2C peripheral/slave mode (✅ implemented on GP26/GP27 at address 0x08)
- [ ] Multi-core processing (sensor on core 0, algorithm on core 1)
- [ ] Optimize `MLX90640_CalculateTo()` - the 61.5ms bottleneck
- [ ] Fixed-point math or SIMD for temperature calculations
- [ ] Target: Approach sensor's 16Hz limit (62.5ms/frame)

Current C version achieves **11.5 Hz** - exceeding the original 4-10 Hz goal! 🎯

## License

MIT License - same as main thermal-tyre-driver package.

## Credits

- Based on CircuitPython version by claude.ai
- Uses official [Melexis MLX90640 C library](https://github.com/melexis/mlx90640-library)
- Built with [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)

---

**Ready to build?** See [docs/BUILD.md](docs/BUILD.md) for step-by-step instructions.

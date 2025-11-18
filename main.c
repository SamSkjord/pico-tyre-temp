/**
 * main.c
 * Thermal Tyre Driver - High-performance C version for Raspberry Pi Pico
 *
 * Target: 4-10Hz frame rate (vs 1.5Hz in CircuitPython)
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"

// MLX90640 library (use official Melexis library)
// Note: You'll need to download MLX90640_API.c and MLX90640_API.h from:
// https://github.com/melexis/mlx90640-library
#include "mlx90640/MLX90640_API.h"
#include "mlx90640/MLX90640_I2C_Driver.h"

#include "thermal_algorithm.h"
#include "communication.h"
#include "i2c_slave.h"

#define MLX90640_ADDR 0x33
#define COMPACT_OUTPUT 1  // 1 for CSV, 0 for JSON

// LED pin for status indication
#define LED_PIN PICO_DEFAULT_LED_PIN

// Default emissivity and temperature constants
#define DEFAULT_EMISSIVITY 0.95f
#define REFLECTED_TEMP_AMBIENT 23.15f  // Ambient reflected temperature in Celsius
#define SENSOR_ERROR_RETRY_MS 100
#define SENSOR_INIT_STABILIZE_MS 2000
#define USB_ENUM_WAIT_MS 5000

// Timing measurement
static uint64_t last_frame_time = 0;
static uint32_t total_frames = 0;

// MLX90640 parameters
static paramsMLX90640 mlx_params;
static uint16_t mlx_frame_raw[834];  // Raw frame data from sensor
static float mlx_frame[768];  // Calculated temperatures
static uint16_t eeData[832] = {0};  // EEPROM data - explicitly initialized to zero

void setup_mlx90640(void) {
    printf("\n========================================\n");
    printf("Thermal Tyre Driver - C Version\n");
    printf("========================================\n\n");

    printf("Initializing I2C...\n");
    fflush(stdout);
    MLX90640_I2CInit();
    printf("I2C initialized OK\n");
    fflush(stdout);
    sleep_ms(100);

    printf("Detecting MLX90640 sensor at 0x%02X...\n", MLX90640_ADDR);
    fflush(stdout);

    // Try to read EEPROM to verify sensor is present
    printf("About to call MLX90640_DumpEE...\n");
    fflush(stdout);
    int status = MLX90640_DumpEE(MLX90640_ADDR, eeData);
    printf("MLX90640_DumpEE returned: %d\n", status);
    fflush(stdout);
    if (status != 0) {
        printf("ERROR: Could not detect MLX90640 sensor!\n");
        printf("Check wiring:\n");
        printf("  MLX90640 VDD → Pico 3V3 (Pin 36)\n");
        printf("  MLX90640 GND → Pico GND (Pin 38)\n");
        printf("  MLX90640 SDA → Pico GP0 (Pin 1)\n");
        printf("  MLX90640 SCL → Pico GP1 (Pin 2)\n");
        while (1) {
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
        }
    }

    printf("Sensor detected! Extracting calibration parameters...\n");
    status = MLX90640_ExtractParameters(eeData, &mlx_params);
    if (status != 0) {
        printf("ERROR: Failed to extract parameters (code %d)\n", status);
        while (1) {
            sleep_ms(1000);
        }
    }

    printf("Setting refresh rate to 16Hz...\n");
    MLX90640_SetRefreshRate(MLX90640_ADDR, 0x05);  // 16Hz

    printf("Waiting for sensor to stabilize...\n");
    sleep_ms(SENSOR_INIT_STABILIZE_MS);  // Give sensor time to stabilize after power-on

    printf("Sensor initialized successfully!\n");
    printf("Expected performance: 5-10Hz frame rate\n\n");
}

int main(void) {
    // Initialize GPIO FIRST - for debugging
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Rapid blink to show we're alive
    for (int i = 0; i < 10; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(50);
        gpio_put(LED_PIN, 0);
        sleep_ms(50);
    }

    // LED off before stdio_init
    gpio_put(LED_PIN, 0);
    sleep_ms(500);

    // Initialize communication
    communication_init();

    // Wait for USB serial to enumerate (increased for stability)
    sleep_ms(USB_ENUM_WAIT_MS);

    gpio_put(LED_PIN, 1);
    printf("=== USB Serial initialized! ===\n");
    fflush(stdout);

    // Initialize MLX90640
    setup_mlx90640();

    // Initialize thermal algorithm
    ThermalConfig config;
    thermal_algorithm_init(&config);

    // Initialize I2C slave mode (GP26=SDA, GP27=SCL, address 0x08)
    printf("Initializing I2C slave mode at address 0x08...\n");
    i2c_slave_init(I2C_SLAVE_DEFAULT_ADDR);
    printf("I2C slave mode enabled on GP26/GP27\n");

    FrameData result;
    memset(&result, 0, sizeof(result));

    printf("========================================\n");
    printf("Starting thermal sensing loop...\n");
    printf("Output: %s\n", COMPACT_OUTPUT ? "Compact CSV" : "Full JSON");
    printf("I2C Slave: 0x08 (GP26=SDA, GP27=SCL)\n");
    printf("========================================\n\n");

    gpio_put(LED_PIN, 0);  // LED off - ready

    last_frame_time = time_us_64();

    while (1) {
        // Get frame from sensor
        uint64_t t_start = time_us_64();

        // Blink LED to show we're alive
        if (total_frames % 2 == 0) {
            gpio_put(LED_PIN, 1);
        } else {
            gpio_put(LED_PIN, 0);
        }

        // Read frame (this is the slow part - ~125ms minimum for hardware)
        int status = MLX90640_GetFrameData(MLX90640_ADDR, mlx_frame_raw);

        uint64_t t_sensor = time_us_64();

        if (status < 0) {
            printf("ERROR: Frame read failed (code %d) - sensor may be disconnected\n", status);
            fflush(stdout);

            // Blink LED rapidly to indicate error
            for (int i = 0; i < 5; i++) {
                gpio_put(LED_PIN, 1);
                sleep_ms(50);
                gpio_put(LED_PIN, 0);
                sleep_ms(50);
            }

            // Wait before retry to avoid hammering the I2C bus
            sleep_ms(SENSOR_ERROR_RETRY_MS);
            continue;
        }

        // Calculate temperatures from raw data
        float emissivity = i2c_slave_get_emissivity();

        // Validate emissivity is in reasonable range (already clamped by getter)
        if (emissivity < 0.1f || emissivity > 1.0f) {
            emissivity = DEFAULT_EMISSIVITY;  // Fallback to default
        }

        float tr = REFLECTED_TEMP_AMBIENT;

        // Calculate temperatures - this modifies mlx_frame in place
        // Note: No return value to check, assumes success
        MLX90640_CalculateTo(mlx_frame_raw, &mlx_params, emissivity, tr, mlx_frame);

        uint64_t t_calc = time_us_64();

        // Process with thermal algorithm (skip if raw mode enabled)
        if (!i2c_slave_get_raw_mode()) {
            thermal_algorithm_process(mlx_frame, &result, &config);
        } else {
            // Raw mode - clear result data
            memset(&result, 0, sizeof(result));
            result.frame_number = total_frames;
        }

        uint64_t t_algo = time_us_64();

        // Calculate FPS for output
        uint64_t frame_time_us = t_algo - t_start;
        float fps = (frame_time_us > 0) ? (1000000.0f / frame_time_us) : 0.0f;

        // Create temperature profile by averaging 24 rows into single 32-pixel row
        // This gives us a horizontal temperature profile across the sensor
        static float temp_profile[32];
        for (int col = 0; col < 32; col++) {
            float sum = 0.0f;
            for (int row = 0; row < 24; row++) {
                sum += mlx_frame[row * 32 + col];
            }
            temp_profile[col] = sum / 24.0f;
        }

        // Update I2C slave registers
        i2c_slave_update(&result, fps, mlx_frame);

        // Output results (conditional based on output mode)
        if (i2c_slave_output_enabled(OUTPUT_MODE_USB_SERIAL)) {
            #if COMPACT_OUTPUT
                send_serial_compact(&result, fps);
            #else
                send_serial_json(&result, fps, temp_profile);
            #endif
        }

        uint64_t t_end = time_us_64();

        // Calculate total frame time for statistics
        uint64_t total_frame_time_us = t_end - t_start;
        float frame_time_ms = total_frame_time_us / 1000.0f;
        float actual_fps = (total_frame_time_us > 0) ? (1000000.0f / total_frame_time_us) : 0.0f;

        total_frames++;

        // Print timing every 10 frames
        if (total_frames % 10 == 0) {
            float sensor_ms = (t_sensor - t_start) / 1000.0f;
            float calc_ms = (t_calc - t_sensor) / 1000.0f;
            float algo_ms = (t_algo - t_calc) / 1000.0f;
            float comm_ms = (t_end - t_algo) / 1000.0f;

            printf("[Frame %lu] Total: %.1fms (%.1f fps) | "
                   "Sensor: %.1fms | Calc: %.1fms | Algo: %.1fms | Comm: %.1fms\n",
                   total_frames, frame_time_ms, actual_fps,
                   sensor_ms, calc_ms, algo_ms, comm_ms);
        }

        // Blink LED on every frame
        gpio_put(LED_PIN, total_frames % 2);

        // Small delay for first few frames to let USB stabilize
        if (total_frames <= 3) {
            sleep_ms(50);
        }

        // Small delay if running too fast (shouldn't happen)
        if (frame_time_ms < 100.0f) {
            sleep_ms(1);
        }
    }

    return 0;
}

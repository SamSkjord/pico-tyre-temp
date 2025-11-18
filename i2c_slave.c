/**
 * i2c_slave.c
 * I2C slave/peripheral implementation
 *
 * SAFETY: Uses interrupt handler to service I2C requests.
 * Shared data access between ISR and main thread requires
 * careful synchronization (see i2c_slave_update).
 */

#include "i2c_slave.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <string.h>
#include <math.h>

// Use I2C1 for slave mode (I2C0 is used for MLX90640 sensor)
#define I2C_SLAVE_INST i2c1
#define I2C_SLAVE_SDA_PIN 26  // GP26
#define I2C_SLAVE_SCL_PIN 27  // GP27
#define I2C_SLAVE_SPEED 100000  // 100kHz (slave speed largely irrelevant)

// Raw channel calculation constants
#define CHANNEL_ROW_START 10  // Middle 4 rows start
#define CHANNEL_ROW_END 14    // Middle 4 rows end (exclusive)
#define CHANNEL_ROW_COUNT (CHANNEL_ROW_END - CHANNEL_ROW_START)
#define CHANNEL_COL_COUNT 2   // Each channel covers 2 columns
#define CHANNEL_PIXEL_COUNT (CHANNEL_ROW_COUNT * CHANNEL_COL_COUNT)  // 8 pixels per channel

// Register map size constant
#define I2C_REGISTER_MAP_SIZE 256

// Internal state (accessed from ISR and main thread)
static volatile I2CSlaveState state;
static uint8_t register_map[I2C_REGISTER_MAP_SIZE];  // Full register space
static volatile const float *current_frame = NULL;  // Pointer to current frame data (volatile for ISR safety)

// Helper to convert float temp to int16 tenths with overflow protection
static inline int16_t temp_to_int16_tenths(float temp) {
    if (!isfinite(temp)) return 0;

    float scaled = temp * 10.0f;

    // Clamp to int16 range with safe boundaries (avoid exact boundary UB)
    // Range: -32768 to +32767 (representing -3276.8°C to +3276.7°C)
    if (scaled >= 32767.0f) return 32767;
    if (scaled <= -32768.0f) return -32768;

    return (int16_t)scaled;
}

// I2C slave IRQ handler
static void i2c_slave_handler(void) {
    uint32_t status = I2C_SLAVE_INST->hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        // Master is reading from us
        uint8_t value = 0;

        // Snapshot current register to avoid TOCTOU race
        uint8_t reg = state.current_register;

        if (reg == REG_FRAME_DATA_START) {
            // Snapshot offset to avoid TOCTOU race (offset could change during ISR)
            uint16_t offset = state.frame_read_offset;

            // Streaming full frame data (768 pixels × 2 bytes = 1536 bytes total)
            if (current_frame && offset < 1536) {
                uint16_t pixel_idx = offset / 2;
                if (pixel_idx < 768) {  // Bounds check for safety
                    // CRITICAL FIX: Read pixel temperature ONCE to avoid reading from
                    // two different frames if current_frame pointer updates mid-ISR
                    int16_t temp = temp_to_int16_tenths(current_frame[pixel_idx]);

                    if (offset % 2 == 0) {
                        // Low byte
                        value = temp & 0xFF;
                    } else {
                        // High byte
                        value = (temp >> 8) & 0xFF;
                    }
                }
                state.frame_read_offset++;
            }
        } else {
            // Regular register read
            value = register_map[reg];
            state.current_register++;  // Auto-increment
        }

        I2C_SLAVE_INST->hw->data_cmd = value;
        __dmb();  // Memory barrier - ensure write completes before clearing interrupt
        I2C_SLAVE_INST->hw->clr_rd_req;
    }

    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        // Master is writing to us
        uint8_t value = (uint8_t)I2C_SLAVE_INST->hw->data_cmd;
        __dmb();  // Memory barrier after hardware register read

        if (state.current_register == 0xFF) {
            // First byte is register address (all values 0x00-0xFF are valid)
            state.current_register = value;

            // Reset frame read offset when accessing frame data
            if (value == REG_FRAME_DATA_START) {
                state.frame_read_offset = 0;
            }
        } else {
            // Subsequent bytes are data writes
            uint8_t target_reg = state.current_register;

            // Validate target register is writable BEFORE accepting the write
            if (target_reg >= REG_CONFIG_START && target_reg <= REG_RESERVED_0F) {
                // Configuration registers are writable
                register_map[target_reg] = value;

                // Handle special registers
                if (target_reg == REG_I2C_ADDRESS) {
                    // I2C address change (requires restart)
                    state.slave_address = value & 0x7F;
                } else if (target_reg == REG_OUTPUT_MODE) {
                    // Output mode change - validate enum value before cast
                    if (value == OUTPUT_MODE_USB_SERIAL || value == OUTPUT_MODE_I2C_SLAVE ||
                        value == OUTPUT_MODE_CANBUS || value == OUTPUT_MODE_ALL) {
                        state.output_mode = (OutputMode)value;
                    }
                    // Invalid values silently ignored
                }
            } else if (target_reg == REG_CMD) {
                // Command register
                if (value == CMD_RESET) {
                    // Software reset (would need to implement)
                } else if (value == CMD_CLEAR_WARNINGS) {
                    register_map[REG_WARNINGS] = 0;
                }
            }
            // Note: Writes to read-only registers are silently ignored

            state.current_register++;
        }
    }

    if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        // Stop condition - reset register pointer
        // This provides automatic timeout/reset: if master doesn't complete a
        // transaction, the STOP condition resets our state machine
        state.current_register = 0xFF;
        I2C_SLAVE_INST->hw->clr_stop_det;
        __dmb();  // Memory barrier after clearing interrupt
    }
}

void i2c_slave_init(uint8_t address) {
    // Initialize state
    memset(&state, 0, sizeof(state));
    memset(register_map, 0, sizeof(register_map));

    state.slave_address = address;
    state.output_mode = OUTPUT_MODE_USB_SERIAL;  // Default to USB
    state.current_register = 0xFF;
    state.enabled = true;

    // Set firmware version
    register_map[REG_FIRMWARE_VERSION] = 0x01;  // Version 1

    // Set default config
    register_map[REG_I2C_ADDRESS] = address;
    register_map[REG_OUTPUT_MODE] = OUTPUT_MODE_USB_SERIAL;
    register_map[REG_FALLBACK_MODE] = 0;  // Default: return 0 when no tyre detected
    register_map[REG_EMISSIVITY] = 95;    // Default: 0.95 emissivity
    register_map[REG_RAW_MODE] = 0;       // Default: tyre algorithm enabled

    // Initialize I2C1 pins
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    // Initialize I2C1 in slave mode
    i2c_init(I2C_SLAVE_INST, I2C_SLAVE_SPEED);
    i2c_set_slave_mode(I2C_SLAVE_INST, true, address);

    // Enable I2C interrupts
    I2C_SLAVE_INST->hw->intr_mask =
        I2C_IC_INTR_MASK_M_RD_REQ_BITS |
        I2C_IC_INTR_MASK_M_RX_FULL_BITS |
        I2C_IC_INTR_MASK_M_STOP_DET_BITS;

    // Set up IRQ handler
    irq_set_exclusive_handler(I2C1_IRQ, i2c_slave_handler);
    irq_set_enabled(I2C1_IRQ, true);
}

void i2c_slave_update(const FrameData *data, float fps, const float *frame) {
    if (!state.enabled) return;
    if (!data) return;  // Validate data pointer
    if (!frame) return;  // Validate frame pointer

    // CRITICAL SECTION: Disable interrupts for atomic frame pointer update AND register updates
    // This ensures ISR doesn't read partially updated state or stale frame pointer
    uint32_t irq_state = save_and_disable_interrupts();

    // Store frame pointer (must be done with interrupts disabled for true atomicity)
    current_frame = frame;

    // Update status registers
    register_map[REG_FRAME_NUMBER_L] = data->frame_number & 0xFF;
    register_map[REG_FRAME_NUMBER_H] = (data->frame_number >> 8) & 0xFF;
    register_map[REG_FPS] = (uint8_t)fminf(fps, 255.0f);  // Clamp to uint8 range
    register_map[REG_DETECTED] = data->detection.detected ? 1 : 0;
    // Clamp confidence to 0-100% range before scaling to avoid overflow
    float confidence_clamped = fmaxf(0.0f, fminf(data->detection.confidence, 1.0f));
    register_map[REG_CONFIDENCE] = (uint8_t)(confidence_clamped * 100.0f);
    register_map[REG_TYRE_WIDTH] = data->detection.tyre_width;
    register_map[REG_SPAN_START] = data->detection.span_start;
    register_map[REG_SPAN_END] = data->detection.span_end;
    register_map[REG_WARNINGS] = data->warnings;

    // Update temperature data (as int16 tenths of degrees)
    int16_t left_med = temp_to_int16_tenths(data->left.median);
    int16_t centre_med = temp_to_int16_tenths(data->centre.median);
    int16_t right_med = temp_to_int16_tenths(data->right.median);
    int16_t left_avg = temp_to_int16_tenths(data->left.avg);
    int16_t centre_avg = temp_to_int16_tenths(data->centre.avg);
    int16_t right_avg = temp_to_int16_tenths(data->right.avg);
    int16_t lat_grad = temp_to_int16_tenths(data->lateral_gradient);

    // Check fallback mode - if no tyre detected and fallback enabled, copy centre temps
    if (!data->detection.detected && register_map[REG_FALLBACK_MODE] == 1) {
        left_med = centre_med;
        right_med = centre_med;
        left_avg = centre_avg;
        right_avg = centre_avg;
        lat_grad = 0;  // No gradient when copying centre temp
    }

    // Pack into registers (little-endian)
    register_map[REG_LEFT_MEDIAN_L] = left_med & 0xFF;
    register_map[REG_LEFT_MEDIAN_H] = (left_med >> 8) & 0xFF;
    register_map[REG_CENTRE_MEDIAN_L] = centre_med & 0xFF;
    register_map[REG_CENTRE_MEDIAN_H] = (centre_med >> 8) & 0xFF;
    register_map[REG_RIGHT_MEDIAN_L] = right_med & 0xFF;
    register_map[REG_RIGHT_MEDIAN_H] = (right_med >> 8) & 0xFF;

    register_map[REG_LEFT_AVG_L] = left_avg & 0xFF;
    register_map[REG_LEFT_AVG_H] = (left_avg >> 8) & 0xFF;
    register_map[REG_CENTRE_AVG_L] = centre_avg & 0xFF;
    register_map[REG_CENTRE_AVG_H] = (centre_avg >> 8) & 0xFF;
    register_map[REG_RIGHT_AVG_L] = right_avg & 0xFF;
    register_map[REG_RIGHT_AVG_H] = (right_avg >> 8) & 0xFF;

    register_map[REG_LATERAL_GRADIENT_L] = lat_grad & 0xFF;
    register_map[REG_LATERAL_GRADIENT_H] = (lat_grad >> 8) & 0xFF;

    // Calculate 16 raw channels if frame data is available
    // Each channel averages 2 columns × 4 middle rows = 8 pixels
    if (frame) {
        for (int ch = 0; ch < 16; ch++) {
            float sum = 0.0f;
            int col_start = ch * CHANNEL_COL_COUNT;

            // Average 2 columns × 4 middle rows
            for (int row = CHANNEL_ROW_START; row < CHANNEL_ROW_END; row++) {
                for (int col = col_start; col < col_start + CHANNEL_COL_COUNT; col++) {
                    // Bounds check to prevent buffer overflow
                    int idx = row * 32 + col;
                    if (idx >= 0 && idx < 768) {
                        sum += frame[idx];
                    }
                }
            }

            float avg = sum / (float)CHANNEL_PIXEL_COUNT;
            int16_t temp = temp_to_int16_tenths(avg);

            // Pack into registers at 0x30 + (ch * 2) with bounds validation
            uint8_t reg_base = REG_RAW_CH0_L + (ch * 2);
            if (reg_base + 1 < I2C_REGISTER_MAP_SIZE) {  // Ensure we don't overflow register map
                register_map[reg_base] = temp & 0xFF;
                register_map[reg_base + 1] = (temp >> 8) & 0xFF;
            }
        }
    }

    // END CRITICAL SECTION: Re-enable interrupts
    restore_interrupts(irq_state);
}

OutputMode i2c_slave_get_output_mode(void) {
    return state.output_mode;
}

void i2c_slave_set_output_mode(OutputMode mode) {
    state.output_mode = mode;
    register_map[REG_OUTPUT_MODE] = (uint8_t)mode;
}

bool i2c_slave_output_enabled(OutputMode mode) {
    if (state.output_mode == OUTPUT_MODE_ALL) return true;
    return (state.output_mode == mode);
}

float i2c_slave_get_emissivity(void) {
    // Convert uint8 (0-100) to float (0.0-1.0)
    uint8_t emiss = register_map[REG_EMISSIVITY];

    // Validate and clamp to reasonable range (10-100 = 0.10-1.00)
    if (emiss < 10) emiss = 10;    // Minimum 0.10 (highly reflective)
    if (emiss > 100) emiss = 100;  // Maximum 1.00 (perfect blackbody)

    return emiss / 100.0f;
}

bool i2c_slave_get_raw_mode(void) {
    return (register_map[REG_RAW_MODE] != 0);
}

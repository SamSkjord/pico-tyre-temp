/**
 * laser_ranger.c
 * Serial laser distance measurement module driver
 *
 * Non-blocking implementation for integration with thermal camera main loop.
 * Uses UART1 at 9600 baud with ring buffer for frame accumulation.
 */

#include "laser_ranger.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdlib.h>

// UART configuration
#define LASER_UART uart1
#define LASER_UART_TX_PIN 4   // GP4 (Pico TX -> Laser RX)
#define LASER_UART_RX_PIN 5   // GP5 (Pico RX <- Laser TX)
#define LASER_BAUD 9600

// Recovery timing
#define RECOVERY_DELAY_MS 50

// Ring buffer for incoming data
#define LASER_BUFFER_SIZE 64
static uint8_t rx_buffer[LASER_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

// Laser state
static LaserState state = {0};

// Calculate checksum: sum of all bytes, inverted, plus 1
static uint8_t calculate_checksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return ((~sum) + 1) & 0xFF;
}

// Send command with checksum
static void send_command(const uint8_t *cmd, uint8_t len) {
    uint8_t cs = calculate_checksum(cmd, len);
    uart_write_blocking(LASER_UART, cmd, len);
    uart_write_blocking(LASER_UART, &cs, 1);
}

// Parse ASCII distance from frame data field
// Format: "000.619" -> 619000 um (micrometers)
static bool parse_distance(const uint8_t *data, uint8_t len, uint32_t *distance_um) {
    char buf[16];
    uint8_t buf_len = 0;

    // Extract numeric characters
    for (uint8_t i = 0; i < len && buf_len < sizeof(buf) - 1; i++) {
        char c = (char)data[i];
        if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
            buf[buf_len++] = c;
        }
    }
    buf[buf_len] = '\0';

    if (buf_len == 0) return false;

    // Parse as float, convert to micrometers
    // Distance comes in meters, multiply by 1,000,000 for um
    char *endptr;
    float meters = strtof(buf, &endptr);
    if (endptr == buf) return false;

    // Sanity check: 0 to 100 meters
    if (meters < 0.0f || meters > 100.0f) return false;

    *distance_um = (uint32_t)(meters * 1000000.0f);
    return true;
}

// Check if data contains error response
static LaserError check_for_error(const uint8_t *data, uint8_t len) {
    // Look for "ERR-" pattern in ASCII
    for (uint8_t i = 0; i + 4 <= len; i++) {
        if (data[i] == 'E' && data[i+1] == 'R' && data[i+2] == 'R' && data[i+3] == '-') {
            // Found error marker, try to parse error code
            if (i + 6 <= len) {
                char code[3] = {(char)data[i+4], (char)data[i+5], '\0'};
                uint8_t err_code = (uint8_t)strtol(code, NULL, 16);
                switch (err_code) {
                    case 0x1E: return LASER_ERR_LOW_POWER;
                    case 0x14: return LASER_ERR_CALC;
                    case 0x15: return LASER_ERR_LASER_LOW;
                    case 0x18: return LASER_ERR_WEAK_SIGNAL;
                    case 0x20: return LASER_ERR_AMBIENT;
                    case 0x74: return LASER_ERR_OUT_OF_RANGE;
                    default: return LASER_ERR_PARSE;
                }
            }
            return LASER_ERR_PARSE;
        }
    }
    return LASER_OK;
}

// Process a complete frame
static bool process_frame(const uint8_t *frame) {
    // Verify address byte
    if (frame[0] != LASER_ADDR) {
        return false;
    }

    // Check for error in data field (bytes 3-9)
    LaserError err = check_for_error(&frame[3], 7);
    if (err != LASER_OK) {
        state.last_error = err;
        state.error_count++;
        return false;
    }

    // Parse distance from data field
    uint32_t distance_um;
    if (!parse_distance(&frame[3], 7, &distance_um)) {
        state.last_error = LASER_ERR_PARSE;
        state.error_count++;
        return false;
    }

    // Valid measurement
    state.distance_um = distance_um;
    state.distance_mm = distance_um / 1000;
    state.last_error = LASER_OK;
    state.valid_count++;
    state.has_valid_reading = true;

    return true;
}

// Internal function to initialize UART hardware
static void init_uart_hardware(void) {
    if (state.initialized) return;

    // Initialize UART1
    uart_init(LASER_UART, LASER_BAUD);

    // Set pin functions
    gpio_set_function(LASER_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LASER_UART_RX_PIN, GPIO_FUNC_UART);

    // Configure UART: 8N1
    uart_set_format(LASER_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LASER_UART, true);

    state.initialized = true;
}

// Internal function to clear UART buffers
static void clear_buffers(void) {
    // Clear hardware FIFO
    while (uart_is_readable(LASER_UART)) {
        uart_getc(LASER_UART);
    }
    // Clear software ring buffer
    rx_head = 0;
    rx_tail = 0;
}

void laser_ranger_init(void) {
    // Initialize state
    memset(&state, 0, sizeof(state));
    state.enabled = false;  // Start disabled - must be explicitly enabled
    state.initialized = false;

    // Initialize UART hardware
    init_uart_hardware();

    // Small delay for UART to settle
    sleep_ms(100);
    clear_buffers();
}

bool laser_ranger_detect(void) {
    if (!state.initialized) return false;

    // Clear any pending data
    clear_buffers();

    // Send laser off command (simple command that should get a response)
    uint8_t cmd[] = {LASER_ADDR, 0x06, 0x05, 0x00};
    send_command(cmd, sizeof(cmd));

    // Wait for valid response (up to 500ms)
    // Must see the address byte (0x80) to confirm it's the laser, not noise
    uint32_t start = to_ms_since_boot(get_absolute_time());
    bool seen_addr = false;

    while ((to_ms_since_boot(get_absolute_time()) - start) < 500) {
        while (uart_is_readable(LASER_UART)) {
            uint8_t byte = uart_getc(LASER_UART);
            if (byte == LASER_ADDR) {
                // Valid address byte - laser is present
                seen_addr = true;
            }
        }
        if (seen_addr) {
            clear_buffers();
            return true;
        }
        sleep_ms(10);
    }

    // No valid response - laser not present
    return false;
}

bool laser_ranger_poll(void) {
    if (!state.enabled || !state.initialized) return false;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // Non-blocking recovery state machine
    if (state.recovery_state != RECOVERY_IDLE) {
        if ((now_ms - state.recovery_ms) >= RECOVERY_DELAY_MS) {
            if (state.recovery_state == RECOVERY_LASER_OFF_PENDING) {
                laser_ranger_laser_off();
                state.recovery_state = RECOVERY_START_CONTINUOUS_PENDING;
                state.recovery_ms = now_ms;
            } else if (state.recovery_state == RECOVERY_START_CONTINUOUS_PENDING) {
                laser_ranger_start_continuous();
                state.recovery_state = RECOVERY_IDLE;
                state.last_valid_ms = now_ms;  // Reset timeout
            }
        }
        return false;  // Still recovering, don't process data
    }

    // Check for timeout - if no valid data for LASER_TIMEOUT_MS, try recovery
    // Check both: time since last valid reading, OR time since enabled (if never got data)
    uint32_t time_ref = state.has_valid_reading ? state.last_valid_ms : state.enabled_ms;
    if ((now_ms - time_ref) > LASER_TIMEOUT_MS) {
        // Timeout detected - start non-blocking recovery
        state.last_error = LASER_ERR_TIMEOUT;
        state.error_count++;
        state.recovery_count++;

        // Start recovery state machine
        clear_buffers();
        state.recovery_state = RECOVERY_LASER_OFF_PENDING;
        state.recovery_ms = now_ms;
        return false;
    }

    // Read all available data into ring buffer
    while (uart_is_readable(LASER_UART)) {
        uint8_t byte = uart_getc(LASER_UART);
        uint16_t next_head = (rx_head + 1) % LASER_BUFFER_SIZE;

        // Don't overflow buffer
        if (next_head != rx_tail) {
            rx_buffer[rx_head] = byte;
            rx_head = next_head;
        }
    }

    // Calculate bytes in buffer
    uint16_t available = (rx_head >= rx_tail) ?
        (rx_head - rx_tail) :
        (LASER_BUFFER_SIZE - rx_tail + rx_head);

    // Need at least one full frame
    if (available < LASER_FRAME_SIZE) {
        return false;
    }

    // Look for frame start (address byte)
    while (available >= LASER_FRAME_SIZE) {
        // Check if current position is frame start
        if (rx_buffer[rx_tail] == LASER_ADDR) {
            // Extract frame
            uint8_t frame[LASER_FRAME_SIZE];
            for (uint8_t i = 0; i < LASER_FRAME_SIZE; i++) {
                frame[i] = rx_buffer[(rx_tail + i) % LASER_BUFFER_SIZE];
            }

            // Verify checksum
            uint8_t expected_cs = calculate_checksum(frame, LASER_FRAME_SIZE - 1);
            if (frame[LASER_FRAME_SIZE - 1] == expected_cs) {
                // Valid frame, consume it
                rx_tail = (rx_tail + LASER_FRAME_SIZE) % LASER_BUFFER_SIZE;

                // Process frame and update timestamp
                bool result = process_frame(frame);
                if (result) {
                    state.last_valid_ms = now_ms;
                }
                return result;
            }
        }

        // Not a valid frame start, skip byte
        rx_tail = (rx_tail + 1) % LASER_BUFFER_SIZE;
        available--;
    }

    return false;
}

const LaserState* laser_ranger_get_state(void) {
    return &state;
}

uint32_t laser_ranger_get_distance_mm(void) {
    return state.has_valid_reading ? state.distance_mm : 0;
}

uint32_t laser_ranger_get_distance_um(void) {
    return state.has_valid_reading ? state.distance_um : 0;
}

void laser_ranger_laser_on(void) {
    if (!state.initialized) return;
    uint8_t cmd[] = {LASER_ADDR, 0x06, 0x05, 0x01};
    send_command(cmd, sizeof(cmd));
}

void laser_ranger_laser_off(void) {
    if (!state.initialized) return;
    uint8_t cmd[] = {LASER_ADDR, 0x06, 0x05, 0x00};
    send_command(cmd, sizeof(cmd));
}

void laser_ranger_start_continuous(void) {
    if (!state.initialized) return;
    uint8_t cmd[] = {LASER_ADDR, 0x06, 0x03};
    send_command(cmd, sizeof(cmd));
}

void laser_ranger_stop_continuous(void) {
    // Turning laser off also stops continuous mode
    laser_ranger_laser_off();
}

bool laser_ranger_is_active(void) {
    return state.enabled && state.has_valid_reading;
}

void laser_ranger_set_enabled(bool enable) {
    if (enable == state.enabled) return;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    if (enable) {
        // Enable laser
        state.enabled = true;
        state.enabled_ms = now_ms;
        state.last_valid_ms = now_ms;
        state.recovery_state = RECOVERY_IDLE;
        clear_buffers();
        laser_ranger_start_continuous();
    } else {
        // Disable laser
        laser_ranger_laser_off();
        state.enabled = false;
        state.recovery_state = RECOVERY_IDLE;
    }
}

bool laser_ranger_is_enabled(void) {
    return state.enabled;
}

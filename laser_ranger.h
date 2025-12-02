/**
 * laser_ranger.h
 * Serial laser distance measurement module driver
 *
 * Interfaces with DFRobot SEN0366 / generic infrared laser ranging modules.
 * Uses UART1 on GP4 (TX) / GP5 (RX) at 9600 baud.
 *
 * Protocol: 9600 baud, 8N1, device address 0x80
 * Frame format: ADDR(1) + CMD(1) + STATUS(1) + DATA(7) + CS(1) = 11 bytes
 * Distance returned as ASCII in DATA field (e.g., "000.619" for 0.619m)
 */

#ifndef LASER_RANGER_H
#define LASER_RANGER_H

#include <stdint.h>
#include <stdbool.h>

// Default device address
#define LASER_ADDR 0x80

// Frame size (ADDR + CMD + STATUS + 7 bytes data + CS)
#define LASER_FRAME_SIZE 11

// Error codes (from device)
typedef enum {
    LASER_OK = 0,
    LASER_ERR_NO_DATA = 1,       // No measurement available yet
    LASER_ERR_LOW_POWER = 0x1E,  // ERR-1e: Low power
    LASER_ERR_CALC = 0x14,       // ERR-14: Calculation error
    LASER_ERR_LASER_LOW = 0x15,  // ERR-15: Laser low power
    LASER_ERR_WEAK_SIGNAL = 0x18,// ERR-18: Weak signal or measurement time too long
    LASER_ERR_AMBIENT = 0x20,    // ERR-20: Strong ambient light
    LASER_ERR_OUT_OF_RANGE = 0x74,// ERR-74: Out of range
    LASER_ERR_PARSE = 0xFE,      // Failed to parse response
    LASER_ERR_TIMEOUT = 0xFF     // No response
} LaserError;

// Timeout for recovery (restart continuous mode if no data)
#define LASER_TIMEOUT_MS 3000

// Laser state
typedef struct {
    uint32_t distance_um;    // Last valid distance in micrometers (0.001mm precision)
    uint32_t distance_mm;    // Last valid distance in millimeters
    LaserError last_error;   // Last error code
    uint32_t valid_count;    // Number of valid measurements
    uint32_t error_count;    // Number of errors
    uint32_t last_valid_ms;  // Timestamp of last valid reading (for timeout detection)
    uint32_t recovery_count; // Number of recovery attempts
    bool enabled;            // Laser enabled flag (software enable)
    bool initialized;        // Hardware initialized flag
    bool has_valid_reading;  // At least one valid reading received
} LaserState;

/**
 * Initialize laser ranger on UART1 (GP4=TX, GP5=RX)
 * Does NOT start measurement - call laser_ranger_set_enabled(true) to start.
 */
void laser_ranger_init(void);

/**
 * Detect if laser ranger is present
 * Sends a command and waits for response. Call after init.
 *
 * @return true if laser responded, false if no response (not connected)
 */
bool laser_ranger_detect(void);

/**
 * Poll for new laser measurement (non-blocking)
 * Call this regularly from main loop to process incoming data.
 *
 * @return true if a new valid measurement was received
 */
bool laser_ranger_poll(void);

/**
 * Get current laser state
 *
 * @return Pointer to laser state (read-only)
 */
const LaserState* laser_ranger_get_state(void);

/**
 * Get last valid distance in millimeters
 *
 * @return Distance in mm, or 0 if no valid reading
 */
uint32_t laser_ranger_get_distance_mm(void);

/**
 * Get last valid distance in micrometers (0.001mm precision)
 *
 * @return Distance in um, or 0 if no valid reading
 */
uint32_t laser_ranger_get_distance_um(void);

/**
 * Turn laser on (for aiming)
 */
void laser_ranger_laser_on(void);

/**
 * Turn laser off
 */
void laser_ranger_laser_off(void);

/**
 * Start continuous measurement mode
 */
void laser_ranger_start_continuous(void);

/**
 * Stop continuous measurement mode
 */
void laser_ranger_stop_continuous(void);

/**
 * Check if laser is enabled and responding
 *
 * @return true if enabled and has received at least one valid reading
 */
bool laser_ranger_is_active(void);

/**
 * Enable or disable laser ranger
 * When disabled, the laser is turned off and polling does nothing.
 * When enabled, continuous measurement mode is started.
 *
 * @param enable true to enable, false to disable
 */
void laser_ranger_set_enabled(bool enable);

/**
 * Check if laser is enabled
 *
 * @return true if enabled
 */
bool laser_ranger_is_enabled(void);

#endif // LASER_RANGER_H

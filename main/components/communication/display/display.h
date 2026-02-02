#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Display module initialization
 * 
 * Initializes the OLED display and related resources
 * 
 * @return true if initialization successful, false otherwise
 */
bool display_init(void);

/**
 * @brief Display module deinitialization
 * 
 * Cleans up display resources
 */
void display_deinit(void);

/**
 * @brief Update battery level display
 * 
 * @param level Battery level in percentage (0-100)
 */
void display_update_battery(uint8_t level);

/**
 * @brief Update IP address display
 * 
 * @param ip IP address string (e.g., "192.168.1.100")
 */
void display_update_ip(const char* ip);

/**
 * @brief Update WaveGo motion status
 * 
 * @param status Motion status string
 * @param is_moving true if WaveGo is currently moving
 */
void display_update_motion_status(const char* status, bool is_moving);

/**
 * @brief Refresh the display
 * 
 * Updates all display elements and refreshes the screen
 */
void display_refresh(void);

/**
 * @brief Display task entry point
 * 
 * Main task function for the display module
 * 
 * @param pvParameters Task parameters (unused)
 */
void display_task(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif // DISPLAY_H
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

// display module runs an internal refresh task

#ifdef __cplusplus
}
#endif

#endif // DISPLAY_H
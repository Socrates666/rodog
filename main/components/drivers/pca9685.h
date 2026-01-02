#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register Definitions
#define PCA9685_MODE1          0x00  // Mode register 1
#define PCA9685_MODE2          0x01  // Mode register 2
#define PCA9685_SUBADR1        0x02  // I2C-bus subaddress 1
#define PCA9685_SUBADR2        0x03  // I2C-bus subaddress 2
#define PCA9685_SUBADR3        0x04  // I2C-bus subaddress 3
#define PCA9685_ALLCALLADR     0x05  // LED All Call I2C-bus address
#define PCA9685_LED0_ON_L      0x06  // LED0 output and brightness control byte 0
#define PCA9685_LED0_ON_H      0x07  // LED0 output and brightness control byte 1
#define PCA9685_LED0_OFF_L     0x08  // LED0 output and brightness control byte 2
#define PCA9685_LED0_OFF_H     0x09  // LED0 output and brightness control byte 3
#define PCA9685_LED_MULTIPLIER 4     // For the other 15 channels
#define PCA9685_ALL_LED_ON_L   0xFA  // Load all the LEDn_ON registers, byte 0
#define PCA9685_ALL_LED_ON_H   0xFB  // Load all the LEDn_ON registers, byte 1
#define PCA9685_ALL_LED_OFF_L  0xFC  // Load all the LEDn_OFF registers, byte 0
#define PCA9685_ALL_LED_OFF_H  0xFD  // Load all the LEDn_OFF registers, byte 1
#define PCA9685_PRE_SCALE      0xFE  // Prescaler for output frequency

#define PCA9685_CLOCK_FREQ     25000000.0  // 25MHz default oscillator clock
#define PCA9685_DEFAULT_FREQ   1000        // Default PWM frequency in Hz
#define PCA9685_MAX_CHANNELS   16          // Number of PWM channels
#define PCA9685_MAX_PWM_VALUE  4096        // Maximum PWM value (12-bit)

// PCA9685 configuration structure
typedef struct {
    i2c_master_dev_handle_t dev_handle;
    // I2C communication callbacks - must be provided by the user
    esp_err_t (*transmit_data_pca9685)(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, int xfer_timeout_ms);
    esp_err_t (*receive_data_pca9685)(i2c_master_dev_handle_t i2c_dev, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms);
} pca9685_config_t;



/**
 * @brief Initialize PCA9685 device
 * 
 * @param config Configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_init(const pca9685_config_t *config);

/**
 * @brief Set PWM frequency
 * 
 * @param freq PWM frequency in Hz (24-1526)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_set_frequency(uint16_t freq);

/**
 * @brief Get current PWM frequency
 * 
 * @param freq Pointer to store frequency value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_get_frequency(uint16_t *freq);

/**
 * @brief Set PWM output for a specific channel
 * 
 * @param channel Channel number (0-15)
 * @param on_value ON time value (0-4095)
 * @param off_value OFF time value (0-4095)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_value, uint16_t off_value);

/**
 * @brief Set PWM output for a specific channel (simplified version)
 * 
 * @param channel Channel number (0-15)
 * @param value PWM value (0-4095), where 0 = fully off, 4095 = fully on
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_set_pwm_value(uint8_t channel, uint16_t value);

/**
 * @brief Get PWM output value for a specific channel
 * 
 * @param channel Channel number (0-15)
 * @param value Pointer to store PWM value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_get_pwm_value(uint8_t channel, uint16_t *value);

/**
 * @brief Set all PWM outputs at once
 * 
 * @param on_value ON time value (0-4095)
 * @param off_value OFF time value (0-4095)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_set_all_pwm(uint16_t on_value, uint16_t off_value);

/**
 * @brief Reset PCA9685 device
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_reset(void);

/**
 * @brief Put device to sleep mode
 * 
 * @param sleep True to sleep, false to wake up
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pca9685_sleep(bool sleep);

#ifdef __cplusplus
}
#endif

#include "pca9685.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
const char* tag = "pac9685";
static pca9685_t pca9685_dev;
// Internal helper functions
static esp_err_t pca9685_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return pca9685_dev.config.transmit_data_pca9685(pca9685_dev.config.dev_handle, data, 2, 1000);
}

static esp_err_t pca9685_read_register(uint8_t reg, uint8_t *value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = pca9685_dev.config.transmit_data_pca9685(pca9685_dev.config.dev_handle, &reg, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    return pca9685_dev.config.receive_data_pca9685(pca9685_dev.config.dev_handle, value, 1, 1000);
}

static esp_err_t pca9685_write_registers(uint8_t reg, const uint8_t *data, size_t len) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t *data_with_reg = (uint8_t *)malloc(len + 1);
    if (!data_with_reg) {
        return ESP_ERR_NO_MEM;
    }
    data_with_reg[0] = reg;
    memcpy(data_with_reg + 1, data, len);
    esp_err_t ret = pca9685_dev.config.transmit_data_pca9685(pca9685_dev.config.dev_handle, data_with_reg, len + 1, 1000);
    free(data_with_reg);
    return ret;
}

static esp_err_t pca9685_read_registers(uint8_t reg, uint8_t *data, size_t len) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = pca9685_dev.config.transmit_data_pca9685(pca9685_dev.config.dev_handle, &reg, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    return pca9685_dev.config.receive_data_pca9685(pca9685_dev.config.dev_handle, data, len, 1000);
}

esp_err_t pca9685_init(const pca9685_config_t *config) {
    if (!config || !config->transmit_data_pca9685 || !config->receive_data_pca9685) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy configuration
    memcpy(&pca9685_dev.config, config, sizeof(pca9685_config_t));
    pca9685_dev.frequency = 0;  // Will be set after frequency configuration
    
    // Reset device
    esp_err_t ret = pca9685_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set default frequency
    return pca9685_set_frequency(PCA9685_DEFAULT_FREQ);
}

esp_err_t pca9685_set_frequency(uint16_t freq) {
    // Frequency limits (from PCA9685 datasheet)
    if (freq < 24) freq = 24;
    if (freq > 1526) freq = 1526;
    
    // Calculate prescale value
    // prescale = round(osc_clock / (4096 * freq)) - 1
    uint8_t prescale = (uint8_t)((PCA9685_CLOCK_FREQ / (4096.0 * freq)) - 0.5);
    
    // Save old mode1 register
    uint8_t old_mode1;
    esp_err_t ret = pca9685_read_register(PCA9685_MODE1, &old_mode1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Put device to sleep (set SLEEP bit)
    uint8_t sleep_mode = old_mode1 | 0x10;  // Set SLEEP bit
    ret = pca9685_write_register(PCA9685_MODE1, sleep_mode);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write prescale value
    ret = pca9685_write_register(PCA9685_PRE_SCALE, prescale);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Restore old mode1 (clearing SLEEP bit)
    ret = pca9685_write_register(PCA9685_MODE1, old_mode1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for oscillator to stabilize (500us minimum, wait 5ms to be safe)
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Save frequency
    pca9685_dev.frequency = freq;
    
    return ESP_OK;
}

esp_err_t pca9685_get_frequency(uint16_t *freq) {
    if (!freq) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *freq = pca9685_dev.frequency;
    return ESP_OK;
}

esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on_value, uint16_t off_value) {
    if (channel >= PCA9685_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Limit values to 12-bit
    on_value &= 0x0FFF;
    off_value &= 0x0FFF;
    
    // Calculate register addresses
    uint8_t base_reg = PCA9685_LED0_ON_L + (channel * PCA9685_LED_MULTIPLIER);
    
    // Prepare data (4 bytes: ON_L, ON_H, OFF_L, OFF_H)
    uint8_t data[4] = {
        (uint8_t)(on_value & 0xFF),           // ON_L
        (uint8_t)((on_value >> 8) & 0x0F),    // ON_H (only 4 bits valid)
        (uint8_t)(off_value & 0xFF),          // OFF_L
        (uint8_t)((off_value >> 8) & 0x0F)    // OFF_H (only 4 bits valid)
    };
    
    return pca9685_write_registers(base_reg, data, 4);
}

esp_err_t pca9685_set_pwm_value( uint8_t channel, uint16_t value) {
    // For simple PWM control, set ON time to 0 and OFF time to the value
    return pca9685_set_pwm(channel, 0, value);
}

esp_err_t pca9685_get_pwm_value(uint8_t channel, uint16_t *value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (channel >= PCA9685_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate register address for OFF value
    uint8_t base_reg = PCA9685_LED0_OFF_L + (channel * PCA9685_LED_MULTIPLIER);
    
    // Read OFF_L and OFF_H registers
    uint8_t off_data[2];
    esp_err_t ret = pca9685_read_registers( base_reg, off_data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine OFF_L and OFF_H (only 4 bits valid in OFF_H)
    *value = ((off_data[1] & 0x0F) << 8) | off_data[0];
    
    return ESP_OK;
}

esp_err_t pca9685_set_all_pwm(uint16_t on_value, uint16_t off_value) {
    // Limit values to 12-bit
    on_value &= 0x0FFF;
    off_value &= 0x0FFF;
    
    // Prepare data for ALL_LED registers
    uint8_t data[4] = {
        (uint8_t)(on_value & 0xFF),           // ALL_LED_ON_L
        (uint8_t)((on_value >> 8) & 0x0F),    // ALL_LED_ON_H
        (uint8_t)(off_value & 0xFF),          // ALL_LED_OFF_L
        (uint8_t)((off_value >> 8) & 0x0F)    // ALL_LED_OFF_H
    };
    
    return pca9685_write_registers(PCA9685_ALL_LED_ON_L, data, 4);
}

esp_err_t pca9685_reset(void) {
    // Reset to power-on state
    // MODE1: Normal mode, auto-increment enabled, respond to ALLCALL
    esp_err_t ret = pca9685_write_register(PCA9685_MODE1, 0x21);  // Auto-increment, ALLCALL
    if (ret != ESP_OK) {
        return ret;
    }
    
    // MODE2: Totem pole outputs, change on STOP
    return pca9685_write_register(PCA9685_MODE2, 0x04);
}

esp_err_t pca9685_sleep( bool sleep) {
    uint8_t mode1;
    esp_err_t ret = pca9685_read_register(PCA9685_MODE1, &mode1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (sleep) {
        mode1 |= 0x10;  // Set SLEEP bit
    } else {
        mode1 &= ~0x10; // Clear SLEEP bit
    }
    
    return pca9685_write_register(PCA9685_MODE1, mode1);
}

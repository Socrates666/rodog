#include "ina219.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* tag = "ina219";
static ina219_config_t ina219_dev;
static bool ina219_initialized = false;

// Internal helper functions
static esp_err_t ina219_write_register(uint8_t reg, uint16_t value) {
    uint8_t data[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return ina219_dev.transmit_data_ina219(ina219_dev.dev_handle, data, 3, 1000);
}

static esp_err_t ina219_read_register(uint8_t reg, uint16_t *value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = ina219_dev.transmit_data_ina219(ina219_dev.dev_handle, &reg, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t buffer[2];
    ret = ina219_dev.receive_data_ina219(ina219_dev.dev_handle, buffer, 2, 1000);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *value = (buffer[0] << 8) | buffer[1];
    return ESP_OK;
}

esp_err_t ina219_init(const ina219_config_t *config) {
    if (!config || !config->transmit_data_ina219 || !config->receive_data_ina219) {
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    memcpy(&ina219_dev, config, sizeof(ina219_config_t));
    ina219_initialized = true;

    // Reset device
    esp_err_t ret = ina219_configure(INA219_CONFIG_RESET);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait for reset to complete
    
    // Calibrate with default values (0.1 Ohm shunt, 3.2A max current)
    ret = ina219_calibrate(config->shunt_resistor, INA219_DEFAULT_CURRENT_LIMIT);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(tag, "INA219 initialized successfully");
    return ESP_OK;
}

esp_err_t ina219_configure(uint16_t config_value) {
    return ina219_write_register(INA219_REG_CONFIG, config_value);
}

esp_err_t ina219_calibrate(float shunt_resistor, float max_current) {
    if (!ina219_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Calculate current LSB (typically 3.2A/2^15 = ~98uA per bit)
    ina219_dev.current_lsb = max_current / 32768.0f;
    ina219_dev.power_lsb = ina219_dev.current_lsb * 20.0f; // Power LSB is 20 times current LSB
    
    // Calculate calibration value
    // Calibration = 0.04096 / (current_LSB * R_SHUNT)
    uint16_t calibration_value = (uint16_t)(0.04096f / (ina219_dev.current_lsb * shunt_resistor));
    
    // Write calibration register
    esp_err_t ret = ina219_write_register(INA219_REG_CALIBRATION, calibration_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure for continuous operation
    uint16_t config = INA219_DEFAULT_VOLTAGE_RANGE |
                      INA219_CONFIG_GAIN_1_40MV |
                      INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT |
                      INA219_CONFIG_MODE_SHUNT_BUS_CONT;
    
    ret = ina219_configure(config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(tag, "INA219 calibrated with shunt resistor: %.3f Ohm", shunt_resistor);
    return ESP_OK;
}

esp_err_t ina219_read_shunt_voltage_raw(int16_t *voltage) {
    if (!voltage || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t raw_value;
    esp_err_t ret = ina219_read_register(INA219_REG_SHUNTVOLTAGE, &raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert to signed 16-bit value
    *voltage = (int16_t)raw_value;
    return ESP_OK;
}

esp_err_t ina219_read_bus_voltage_raw(uint16_t *voltage) {
    if (!voltage || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t raw_value;
    esp_err_t ret = ina219_read_register(INA219_REG_BUSVOLTAGE, &raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *voltage = raw_value;
    return ESP_OK;
}

esp_err_t ina219_read_current_raw(int16_t *current) {
    if (!current || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t raw_value;
    esp_err_t ret = ina219_read_register(INA219_REG_CURRENT, &raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *current = (int16_t)raw_value;
    return ESP_OK;
}

esp_err_t ina219_read_power_raw(int16_t *power) {
    if (!power || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t raw_value;
    esp_err_t ret = ina219_read_register(INA219_REG_POWER, &raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *power = (int16_t)raw_value;
    return ESP_OK;
}

esp_err_t ina219_read_bus_voltage(float *voltage) {
    if (!voltage || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t raw_value;
    esp_err_t ret = ina219_read_bus_voltage_raw(&raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw value to voltage (Bus voltage LSB = 4mV)
    *voltage = (float)(raw_value >> 3) * 0.004f;
    return ESP_OK;
}

esp_err_t ina219_read_shunt_voltage(float *voltage) {
    if (!voltage || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int16_t raw_value;
    esp_err_t ret = ina219_read_shunt_voltage_raw(&raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw value to voltage (Shunt voltage LSB = 10uV)
    *voltage = (float)raw_value * 0.00001f;
    return ESP_OK;
}

esp_err_t ina219_read_current(float *current) {
    if (!current || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int16_t raw_value;
    esp_err_t ret = ina219_read_current_raw(&raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw value to current using calibrated LSB
    *current = (float)raw_value * ina219_dev.current_lsb;
    return ESP_OK;
}

esp_err_t ina219_read_power(float *power) {
    if (!power || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int16_t raw_value;
    esp_err_t ret = ina219_read_power_raw(&raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw value to power using calibrated LSB
    *power = (float)raw_value * ina219_dev.power_lsb;
    return ESP_OK;
}

esp_err_t ina219_read_measurements(ina219_measurement_t *measurements) {
    if (!measurements || !ina219_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    ret = ina219_read_bus_voltage(&measurements->bus_voltage);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = ina219_read_shunt_voltage(&measurements->shunt_voltage);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = ina219_read_current(&measurements->current);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = ina219_read_power(&measurements->power);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return ESP_OK;
}
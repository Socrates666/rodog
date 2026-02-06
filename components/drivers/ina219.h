#ifndef INA219_H
#define INA219_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

// INA219 Register Addresses
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// INA219 Configuration Settings
#define INA219_CONFIG_RESET             0x8000
#define INA219_CONFIG_BVOLTAGERANGE_MASK 0x2000
#define INA219_CONFIG_GAIN_MASK         0x1800
#define INA219_CONFIG_BADCRES_MASK      0x0780
#define INA219_CONFIG_SADCRES_MASK      0x0078
#define INA219_CONFIG_MODE_MASK         0x0007

// Bus Voltage Range Options
#define INA219_CONFIG_BVOLTAGERANGE_16V 0x0000
#define INA219_CONFIG_BVOLTAGERANGE_32V 0x2000

// Gain Options
#define INA219_CONFIG_GAIN_1_40MV 0x0000
#define INA219_CONFIG_GAIN_2_80MV 0x0800
#define INA219_CONFIG_GAIN_4_160MV 0x1000
#define INA219_CONFIG_GAIN_8_320MV 0x1800

// ADC Resolution and Averaging Options
#define INA219_CONFIG_BADCRES_9BIT      0x0080
#define INA219_CONFIG_BADCRES_10BIT     0x0100
#define INA219_CONFIG_BADCRES_11BIT     0x0200
#define INA219_CONFIG_BADCRES_12BIT     0x0400
#define INA219_CONFIG_BADCRES_12BIT_2SAMP 0x0480
#define INA219_CONFIG_BADCRES_12BIT_4SAMP 0x0500
#define INA219_CONFIG_BADCRES_12BIT_8SAMP 0x0580
#define INA219_CONFIG_BADCRES_12BIT_16SAMP 0x0600
#define INA219_CONFIG_BADCRES_12BIT_32SAMP 0x0680
#define INA219_CONFIG_BADCRES_12BIT_64SAMP 0x0700
#define INA219_CONFIG_BADCRES_12BIT_128SAMP 0x0780

#define INA219_CONFIG_SADCRES_9BIT      0x0008
#define INA219_CONFIG_SADCRES_10BIT     0x0010
#define INA219_CONFIG_SADCRES_11BIT     0x0020
#define INA219_CONFIG_SADCRES_12BIT     0x0040
#define INA219_CONFIG_SADCRES_12BIT_2SAMP 0x0048
#define INA219_CONFIG_SADCRES_12BIT_4SAMP 0x0050
#define INA219_CONFIG_SADCRES_12BIT_8SAMP 0x0058
#define INA219_CONFIG_SADCRES_12BIT_16SAMP 0x0060
#define INA219_CONFIG_SADCRES_12BIT_32SAMP 0x0068
#define INA219_CONFIG_SADCRES_12BIT_64SAMP 0x0070
#define INA219_CONFIG_SADCRES_12BIT_128SAMP 0x0078

// Operating Modes
#define INA219_CONFIG_MODE_POWERDOWN    0x0000
#define INA219_CONFIG_MODE_SHUNT_TRIG   0x0001
#define INA219_CONFIG_MODE_BUS_TRIG     0x0002
#define INA219_CONFIG_MODE_SHUNT_BUS_TRIG 0x0003
#define INA219_CONFIG_MODE_ADC_OFF      0x0004
#define INA219_CONFIG_MODE_SHUNT_CONT   0x0005
#define INA219_CONFIG_MODE_BUS_CONT     0x0006
#define INA219_CONFIG_MODE_SHUNT_BUS_CONT 0x0007

// Default configuration values
#define INA219_DEFAULT_SHUNT_RESISTOR   0.1f  // Ohms
#define INA219_DEFAULT_CURRENT_LIMIT    3.2f  // Amps
#define INA219_DEFAULT_VOLTAGE_RANGE    INA219_CONFIG_BVOLTAGERANGE_32V

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    esp_err_t (*transmit_data_ina219)(i2c_master_dev_handle_t, const uint8_t*, size_t, int);
    esp_err_t (*receive_data_ina219)(i2c_master_dev_handle_t, uint8_t*, size_t, int);
    float shunt_resistor;     // Shunt resistor value in Ohms
    float current_lsb;        // Current LSB value in Amps
    float power_lsb;          // Power LSB value in Watts
} ina219_config_t;

typedef struct {
    float bus_voltage;        // Bus voltage in Volts
    float shunt_voltage;      // Shunt voltage in Volts
    float current;            // Current in Amps
    float power;              // Power in Watts
} ina219_measurement_t;

// Function prototypes
esp_err_t ina219_init(const ina219_config_t *config);
esp_err_t ina219_configure(uint16_t config_value);
esp_err_t ina219_calibrate(float shunt_resistor, float max_current);
esp_err_t ina219_read_shunt_voltage_raw(int16_t *voltage);
esp_err_t ina219_read_bus_voltage_raw(uint16_t *voltage);
esp_err_t ina219_read_current_raw(int16_t *current);
esp_err_t ina219_read_power_raw(int16_t *power);
esp_err_t ina219_read_bus_voltage(float *voltage);
esp_err_t ina219_read_shunt_voltage(float *voltage);
esp_err_t ina219_read_current(float *current);
esp_err_t ina219_read_power(float *power);
esp_err_t ina219_read_measurements(ina219_measurement_t *measurements);

#ifdef __cplusplus
}
#endif

#endif // INA219_H
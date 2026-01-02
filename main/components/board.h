#pragma once

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "icm20948.h"

#define BSP_I2C_NUM I2C_NUM_0
#define BSP_I2C_SDA 32
#define BSP_I2C_SCL 33



esp_err_t bsp_i2c_init(void);

esp_err_t bsp_i2c_deinit(void);

i2c_master_bus_handle_t bsp_i2c_get_handle(void);

esp_err_t bsp_i2c_scan();

esp_err_t bsp_icm20948_init(void);

esp_err_t bsp_read_gyro(axises* data);

esp_err_t bsp_read_accel(axises* data);

esp_err_t bsp_pca9685_init(void);
esp_err_t bsp_ina219_init(uint16_t i);
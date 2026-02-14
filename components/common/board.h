#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "icm20948.h"
#include "pca9685.h"
#include "ina219.h"
#include "ssd1306.h"
#include "leg.h"
// #include "constant.h


#define BSP_I2C_NUM I2C_NUM_0
#define BSP_I2C_SDA 32
#define BSP_I2C_SCL 33

#define BSP_SSD1306_WIDTH 128
#define BSP_SSD1306_HEIGHT 32

esp_err_t bsp_i2c_init(void);
esp_err_t bsp_i2c_deinit(void);
i2c_master_bus_handle_t bsp_i2c_get_handle(void);
esp_err_t bsp_i2c_scan();
esp_err_t bsp_icm20948_init(void);
esp_err_t bsp_icm20948_set_rate(uint16_t hz);
esp_err_t bsp_read_gyro(axises* data);
esp_err_t bsp_read_accel(axises* data);
esp_err_t bsp_pca9685_init(void);
esp_err_t bsp_ina219_init(void);
esp_err_t bsp_ssd1306_init(void);
esp_err_t bsp_read_battery_percent(uint8_t *percent);
esp_err_t bsp_ssd1306_clear(void);
esp_err_t bsp_ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool color);
esp_err_t bsp_ssd1306_print_string(uint8_t x, uint8_t y, const char* str, bool color);
esp_err_t bsp_ssd1306_update_screen(void);

// Control interfaces
esp_err_t start_wavego_task(void);
esp_err_t stop_wavego_task(void);
esp_err_t send_wavego_command(int wave_height, int wave_speed, ActionState state);
ActionState get_current_action_state(void);

// Calibration interfaces
esp_err_t calibration_init(void);
esp_err_t calibration_load_from_nvs(void);
esp_err_t calibration_reset_defaults(void);
esp_err_t calibration_set_middle_pwm(uint8_t channel, uint16_t pwm);
esp_err_t calibration_get_middle_pwm(uint8_t channel, uint16_t *pwm_out);

// Communication interfaces
void webServerInit(void);
void wifiInit(void);
void getMAC(void);
bool getIP(char *out, size_t out_len);
void getWifiStatus(void);

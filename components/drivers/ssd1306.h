#ifndef SSD1306_H
#define SSD1306_H

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

// SSD1306 Commands
#define SSD1306_SET_CONTRAST                    0x81
#define SSD1306_SET_ENTIRE_ON                   0xA4
#define SSD1306_SET_NORM_INV                    0xA6
#define SSD1306_SET_DISP                        0xAE
#define SSD1306_SET_MEM_ADDR                    0x20
#define SSD1306_SET_COL_ADDR                    0x21
#define SSD1306_SET_PAGE_ADDR                   0x22
#define SSD1306_SET_DISP_START_LINE             0x40
#define SSD1306_SET_SEG_REMAP                   0xA0
#define SSD1306_SET_MUX_RATIO                   0xA8
#define SSD1306_SET_COM_OUT_DIR                 0xC0
#define SSD1306_SET_DISP_OFFSET                 0xD3
#define SSD1306_SET_COM_PIN_CFG                 0xDA
#define SSD1306_SET_DISP_CLK_DIV                0xD5
#define SSD1306_SET_PRECHARGE                   0xD9
#define SSD1306_SET_VCOM_DESEL                  0xDB
#define SSD1306_SET_CHARGE_PUMP                 0x8D

// Display dimensions
#define SSD1306_WIDTH                           128
#define SSD1306_HEIGHT                          64
#define SSD1306_PAGES                           (SSD1306_HEIGHT / 8)

// Font definitions
#define FONT_WIDTH                              6
#define FONT_HEIGHT                             8

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    esp_err_t (*transmit_data_ssd1306)(i2c_master_dev_handle_t, const uint8_t*, size_t, int);
    esp_err_t (*receive_data_ssd1306)(i2c_master_dev_handle_t, uint8_t*, size_t, int);
    uint8_t width;
    uint8_t height;
    uint8_t pages;
    uint8_t buffer[SSD1306_HEIGHT][SSD1306_WIDTH];  // Display buffer
} ssd1306_config_t;

typedef enum {
    SSD1306_ADDR_0X3C = 0x3C,
    SSD1306_ADDR_0X3D = 0x3D
} ssd1306_address_t;

typedef enum {
    HORIZONTAL_MODE,
    VERTICAL_MODE,
    PAGE_MODE
} scroll_direction_t;

// Function prototypes
esp_err_t ssd1306_init(const ssd1306_config_t *config, ssd1306_address_t address);
esp_err_t ssd1306_display_on(void);
esp_err_t ssd1306_display_off(void);
esp_err_t ssd1306_clear_screen(void);
esp_err_t ssd1306_fill_screen(void);
esp_err_t ssd1306_contrast(uint8_t contrast);
esp_err_t ssd1306_invert_display(bool invert);
esp_err_t ssd1306_set_cursor(uint8_t x, uint8_t y);
esp_err_t ssd1306_draw_pixel(uint8_t x, uint8_t y, bool color);
esp_err_t ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool color);
esp_err_t ssd1306_draw_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool color);
esp_err_t ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool color);
esp_err_t ssd1306_draw_circle(uint8_t x_center, uint8_t y_center, uint8_t radius, bool color);
esp_err_t ssd1306_print_char(uint8_t x, uint8_t y, char ch, bool color);
esp_err_t ssd1306_print_string(uint8_t x, uint8_t y, const char* str, bool color);
esp_err_t ssd1306_update_screen(void);
esp_err_t ssd1306_draw_bitmap(uint8_t x, uint8_t y, const uint8_t* bitmap, uint8_t width, uint8_t height, bool color);

#ifdef __cplusplus
}
#endif

#endif // SSD1306_H
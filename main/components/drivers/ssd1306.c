#include "ssd1306.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* tag = "ssd1306";
static ssd1306_config_t ssd1306_dev;
static bool ssd1306_initialized = false;

// ASCII font data (5x8 pixels)
static const uint8_t font_data[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    {0x00, 0x00, 0x6F, 0x00, 0x00}, // '!'
    {0x00, 0x07, 0x00, 0x07, 0x00}, // '"'
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // '#'
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // '$'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // '%'
    {0x36, 0x49, 0x55, 0x22, 0x50}, // '&'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '''
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // '('
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // ')'
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // '*'
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // '+'
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ','
    {0x08, 0x08, 0x08, 0x08, 0x08}, // '-'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
    {0x20, 0x10, 0x08, 0x04, 0x02}, // '/'
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ';'
    {0x08, 0x14, 0x22, 0x41, 0x00}, // '<'
    {0x14, 0x14, 0x14, 0x14, 0x14}, // '='
    {0x00, 0x41, 0x22, 0x14, 0x08}, // '>'
    {0x02, 0x01, 0x51, 0x09, 0x06}, // '?'
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 'Z'
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // '['
    {0x02, 0x04, 0x08, 0x10, 0x20}, // '\'
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ']'
    {0x04, 0x02, 0x01, 0x02, 0x04}, // '^'
    {0x40, 0x40, 0x40, 0x40, 0x40}, // '_'
    {0x00, 0x01, 0x02, 0x04, 0x00}, // '`'
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 'a'
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 'b'
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 'c'
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 'd'
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 'e'
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 'f'
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // 'g'
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 'h'
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 'i'
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 'j'
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 'k'
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 'l'
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 'm'
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 'n'
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 'o'
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 'p'
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 'q'
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 'r'
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 's'
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 't'
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 'u'
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 'v'
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 'w'
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 'x'
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 'y'
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00}, // '{'
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // '|'
    {0x00, 0x41, 0x36, 0x08, 0x00}, // '}'
    {0x02, 0x01, 0x02, 0x04, 0x02}, // '~'
    {0x3E, 0x55, 0x55, 0x41, 0x22}  // DEL (for special symbols)
};

// Internal helper functions
static esp_err_t ssd1306_write_command(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd}; // 0x00 indicates command
    return ssd1306_dev.transmit_data_ssd1306(ssd1306_dev.dev_handle, data, 2, 1000);
}

static esp_err_t ssd1306_write_data(const uint8_t *data, size_t len) {
    uint8_t *buf = malloc(len + 1);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }
    
    buf[0] = 0x40; // 0x40 indicates data
    memcpy(buf + 1, data, len);
    
    esp_err_t ret = ssd1306_dev.transmit_data_ssd1306(ssd1306_dev.dev_handle, buf, len + 1, 1000);
    free(buf);
    return ret;
}

esp_err_t ssd1306_init(const ssd1306_config_t *config, ssd1306_address_t address) {
    if (!config || !config->transmit_data_ssd1306 || !config->receive_data_ssd1306) {
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    memcpy(&ssd1306_dev, config, sizeof(ssd1306_config_t));
    ssd1306_dev.width = SSD1306_WIDTH;
    ssd1306_dev.height = SSD1306_HEIGHT;
    ssd1306_dev.pages = SSD1306_PAGES;
    ssd1306_initialized = true;

    // Initialize display buffer
    memset(ssd1306_dev.buffer, 0, sizeof(ssd1306_dev.buffer));

    // SSD1306 Initialization sequence
    esp_err_t ret;
    
    ret = ssd1306_write_command(SSD1306_SET_DISP | 0x00); // Display off
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(0x00); // Set lower column address
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(0x10); // Set higher column address
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_DISP_START_LINE | 0x00); // Start line
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_CONTRAST);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0xCF); // Contrast value
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_ENTIRE_ON); // Entire display on
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_NORM_INV); // Normal display
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_MUX_RATIO);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x3F); // Multiplex ratio (64MUX)
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_DISP_OFFSET);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x00); // No offset
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_DISP_CLK_DIV);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x80); // Clock divide ratio
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_PRECHARGE);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0xF1); // Precharge period
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_VCOM_DESEL);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x40); // VCOM deselect level
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_CHARGE_PUMP);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x14); // Enable charge pump
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_MEM_ADDR);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x00); // Horizontal addressing mode
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_COM_OUT_DIR | 0x00); // COM pins configuration
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_SEG_REMAP | 0x01); // Remap columns
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_SET_COM_PIN_CFG);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x12); // COM pin configuration
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_display_on();
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_clear_screen();
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(tag, "SSD1306 OLED initialized successfully");
    return ESP_OK;
}

esp_err_t ssd1306_display_on(void) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return ssd1306_write_command(SSD1306_SET_DISP | 0x01); // Display ON
}

esp_err_t ssd1306_display_off(void) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return ssd1306_write_command(SSD1306_SET_DISP | 0x00); // Display OFF
}

esp_err_t ssd1306_clear_screen(void) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear internal buffer
    memset(ssd1306_dev.buffer, 0, sizeof(ssd1306_dev.buffer));
    
    // Update screen
    return ssd1306_update_screen();
}

esp_err_t ssd1306_fill_screen(void) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Fill internal buffer
    memset(ssd1306_dev.buffer, 0xFF, sizeof(ssd1306_dev.buffer));
    
    // Update screen
    return ssd1306_update_screen();
}

esp_err_t ssd1306_contrast(uint8_t contrast) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ssd1306_write_command(SSD1306_SET_CONTRAST);
    if (ret != ESP_OK) return ret;
    return ssd1306_write_command(contrast);
}

esp_err_t ssd1306_invert_display(bool invert) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return ssd1306_write_command(SSD1306_SET_NORM_INV | (invert ? 0x01 : 0x00));
}

esp_err_t ssd1306_set_cursor(uint8_t x, uint8_t y) {
    if (!ssd1306_initialized || x >= ssd1306_dev.width || y >= ssd1306_dev.height) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    // Set column address
    ret = ssd1306_write_command(SSD1306_SET_COL_ADDR);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(x);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(ssd1306_dev.width - 1);
    if (ret != ESP_OK) return ret;
    
    // Set page address
    ret = ssd1306_write_command(SSD1306_SET_PAGE_ADDR);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(y / 8);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(ssd1306_dev.pages - 1);
    if (ret != ESP_OK) return ret;
    
    return ESP_OK;
}

esp_err_t ssd1306_draw_pixel(uint8_t x, uint8_t y, bool color) {
    if (!ssd1306_initialized || x >= ssd1306_dev.width || y >= ssd1306_dev.height) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    
    if (color) {
        ssd1306_dev.buffer[page][x] |= (1 << bit);
    } else {
        ssd1306_dev.buffer[page][x] &= ~(1 << bit);
    }
    
    return ESP_OK;
}

esp_err_t ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool color) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        ssd1306_draw_pixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    return ESP_OK;
}

esp_err_t ssd1306_draw_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool color) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Draw horizontal lines
    ssd1306_draw_line(x, y, x + width - 1, y, color);
    ssd1306_draw_line(x, y + height - 1, x + width - 1, y + height - 1, color);
    
    // Draw vertical lines
    ssd1306_draw_line(x, y, x, y + height - 1, color);
    ssd1306_draw_line(x + width - 1, y, x + width - 1, y + height - 1, color);
    
    return ESP_OK;
}

esp_err_t ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool color) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    for (uint8_t i = 0; i < width; i++) {
        for (uint8_t j = 0; j < height; j++) {
            if (x + i < ssd1306_dev.width && y + j < ssd1306_dev.height) {
                ssd1306_draw_pixel(x + i, y + j, color);
            }
        }
    }
    
    return ESP_OK;
}

esp_err_t ssd1306_draw_circle(uint8_t x_center, uint8_t y_center, uint8_t radius, bool color) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    int x = 0;
    int y = radius;
    int d = 3 - 2 * radius;
    
    while (x <= y) {
        ssd1306_draw_pixel(x_center + x, y_center + y, color);
        ssd1306_draw_pixel(x_center - x, y_center + y, color);
        ssd1306_draw_pixel(x_center + x, y_center - y, color);
        ssd1306_draw_pixel(x_center - x, y_center - y, color);
        ssd1306_draw_pixel(x_center + y, y_center + x, color);
        ssd1306_draw_pixel(x_center - y, y_center + x, color);
        ssd1306_draw_pixel(x_center + y, y_center - x, color);
        ssd1306_draw_pixel(x_center - y, y_center - x, color);
        
        if (d < 0) {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
    
    return ESP_OK;
}

esp_err_t ssd1306_print_char(uint8_t x, uint8_t y, char ch, bool color) {
    if (!ssd1306_initialized || ch < ' ' || ch > '~') {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Adjust character index to handle DEL character
    uint8_t char_idx = (ch == 127) ? 95 : (ch - ' ');
    
    for (uint8_t col = 0; col < 5; col++) {
        uint8_t font_byte = font_data[char_idx][col];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (font_byte & (1 << bit)) {
                ssd1306_draw_pixel(x + col, y + bit, color);
            }
        }
    }
    
    // Add space between characters
    for (uint8_t bit = 0; bit < 8; bit++) {
        ssd1306_draw_pixel(x + 5, y + bit, !color);
    }
    
    return ESP_OK;
}

esp_err_t ssd1306_print_string(uint8_t x, uint8_t y, const char* str, bool color) {
    if (!ssd1306_initialized || !str) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t curr_x = x;
    uint8_t str_len = strlen(str);
    
    for (uint8_t i = 0; i < str_len; i++) {
        if (curr_x + FONT_WIDTH > ssd1306_dev.width) {
            // Move to next line if string exceeds width
            curr_x = 0;
            y += FONT_HEIGHT;
            if (y + FONT_HEIGHT > ssd1306_dev.height) {
                break; // Prevent drawing outside screen
            }
        }
        
        if (str[i] == '\n') {
            curr_x = 0;
            y += FONT_HEIGHT;
            if (y + FONT_HEIGHT > ssd1306_dev.height) {
                break; // Prevent drawing outside screen
            }
        } else {
            ssd1306_print_char(curr_x, y, str[i], color);
            curr_x += FONT_WIDTH;
        }
    }
    
    return ESP_OK;
}

esp_err_t ssd1306_update_screen(void) {
    if (!ssd1306_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    // Set column address
    ret = ssd1306_write_command(SSD1306_SET_COL_ADDR);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(SSD1306_WIDTH - 1);
    if (ret != ESP_OK) return ret;
    
    // Set page address
    ret = ssd1306_write_command(SSD1306_SET_PAGE_ADDR);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(SSD1306_PAGES - 1);
    if (ret != ESP_OK) return ret;
    
    // Send display buffer to OLED
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        ret = ssd1306_write_data(ssd1306_dev.buffer[page], SSD1306_WIDTH);
        if (ret != ESP_OK) return ret;
    }
    
    return ESP_OK;
}

esp_err_t ssd1306_draw_bitmap(uint8_t x, uint8_t y, const uint8_t* bitmap, uint8_t width, uint8_t height, bool color) {
    if (!ssd1306_initialized || !bitmap) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (uint8_t i = 0; i < width; i++) {
        for (uint8_t j = 0; j < height; j++) {
            if (x + i < ssd1306_dev.width && y + j < ssd1306_dev.height) {
                uint8_t byte_index = j / 8 + (i * ((height + 7) / 8));
                uint8_t bit_index = j % 8;
                
                bool pixel_color = (bitmap[byte_index] & (1 << bit_index)) ? color : !color;
                ssd1306_draw_pixel(x + i, y + j, pixel_color);
            }
        }
    }
    
    return ESP_OK;
}
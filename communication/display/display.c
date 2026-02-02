#include "display.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "ssd1306.h"

// Tag for logging
static const char* TAG = "display";

// Display dimensions
#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 64

// I2C configuration
#define I2C_MASTER_SCL_IO          22    // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO          21    // GPIO number for I2C master data
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

// OLED display address
#define OLED_ADDR   0x3C

// Display data structure
typedef struct {
    uint8_t battery_level;
    char ip_address[16];
    char motion_status[32];
    bool is_moving;
    bool needs_refresh;
} display_data_t;

// Global display context
static display_data_t g_display_data = {
    .battery_level = 0,
    .ip_address = {0},
    .motion_status = {0},
    .is_moving = false,
    .needs_refresh = true
};

// SSD1306 handle
static SSD1306_t g_dev;

// Mutex for display access
static SemaphoreHandle_t g_display_mutex = NULL;

/**
 * @brief Initialize I2C for OLED display
 */
static bool i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

/**
 * @brief Initialize OLED display
 */
static bool oled_init(void) {
    // Initialize SSD1306
    ssd1306_init(&g_dev, DISPLAY_WIDTH, DISPLAY_HEIGHT, false, true);
    
    // Clear display
    ssd1306_clear_screen(&g_dev, false);
    
    // Display welcome message
    ssd1306_display_text(&g_dev, 0, "WaveGo Display", 14, false);
    ssd1306_display_text(&g_dev, 1, "Initializing...", 15, false);
    ssd1306_display_text(&g_dev, 3, "Battery: --%", 12, false);
    ssd1306_display_text(&g_dev, 4, "IP: --.--.--.--", 15, false);
    ssd1306_display_text(&g_dev, 5, "Status: Idle", 12, false);
    
    return true;
}

/**
 * @brief Update display with current data
 */
static void update_display(void) {
    if (xSemaphoreTake(g_display_mutex, portMAX_DELAY) == pdTRUE) {
        char buffer[32];
        
        // Clear screen
        ssd1306_clear_screen(&g_dev, false);
        
        // Display title
        ssd1306_display_text(&g_dev, 0, "=== WaveGo ===", 14, false);
        
        // Display battery level
        snprintf(buffer, sizeof(buffer), "Battery: %3d%%", g_display_data.battery_level);
        ssd1306_display_text(&g_dev, 2, buffer, strlen(buffer), false);
        
        // Display IP address
        snprintf(buffer, sizeof(buffer), "IP: %s", g_display_data.ip_address);
        ssd1306_display_text(&g_dev, 3, buffer, strlen(buffer), false);
        
        // Display motion status with indicator
        const char* moving_indicator = g_display_data.is_moving ? "[MOVING]" : "[IDLE]";
        snprintf(buffer, sizeof(buffer), "%s %s", moving_indicator, g_display_data.motion_status);
        ssd1306_display_text(&g_dev, 5, buffer, strlen(buffer), false);
        
        // Display separator line
        ssd1306_display_text(&g_dev, 7, "----------------", 16, false);
        
        xSemaphoreGive(g_display_mutex);
    }
}

bool display_init(void) {
    ESP_LOGI(TAG, "Initializing display module");
    
    // Create mutex
    g_display_mutex = xSemaphoreCreateMutex();
    if (g_display_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create display mutex");
        return false;
    }
    
    // Initialize I2C
    if (!i2c_master_init()) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        vSemaphoreDelete(g_display_mutex);
        return false;
    }
    
    // Initialize OLED
    if (!oled_init()) {
        ESP_LOGE(TAG, "Failed to initialize OLED");
        i2c_driver_delete(I2C_MASTER_NUM);
        vSemaphoreDelete(g_display_mutex);
        return false;
    }
    
    // Set default values
    strcpy(g_display_data.ip_address, "0.0.0.0");
    strcpy(g_display_data.motion_status, "Initializing");
    
    ESP_LOGI(TAG, "Display module initialized successfully");
    return true;
}

void display_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing display module");
    
    if (g_display_mutex != NULL) {
        vSemaphoreDelete(g_display_mutex);
        g_display_mutex = NULL;
    }
    
    // Clear display
    ssd1306_clear_screen(&g_dev, false);
    
    // Delete I2C driver
    i2c_driver_delete(I2C_MASTER_NUM);
    
    ESP_LOGI(TAG, "Display module deinitialized");
}

void display_update_battery(uint8_t level) {
    if (xSemaphoreTake(g_display_mutex, portMAX_DELAY) == pdTRUE) {
        g_display_data.battery_level = level > 100 ? 100 : level;
        g_display_data.needs_refresh = true;
        xSemaphoreGive(g_display_mutex);
        
        ESP_LOGI(TAG, "Battery level updated: %d%%", level);
    }
}

void display_update_ip(const char* ip) {
    if (ip == NULL) {
        ESP_LOGW(TAG, "Invalid IP address provided");
        return;
    }
    
    if (xSemaphoreTake(g_display_mutex, portMAX_DELAY) == pdTRUE) {
        strncpy(g_display_data.ip_address, ip, sizeof(g_display_data.ip_address) - 1);
        g_display_data.ip_address[sizeof(g_display_data.ip_address) - 1] = '\0';
        g_display_data.needs_refresh = true;
        xSemaphoreGive(g_display_mutex);
        
        ESP_LOGI(TAG, "IP address updated: %s", ip);
    }
}

void display_update_motion_status(const char* status, bool is_moving) {
    if (status == NULL) {
        ESP_LOGW(TAG, "Invalid motion status provided");
        return;
    }
    
    if (xSemaphoreTake(g_display_mutex, portMAX_DELAY) == pdTRUE) {
        strncpy(g_display_data.motion_status, status, sizeof(g_display_data.motion_status) - 1);
        g_display_data.motion_status[sizeof(g_display_data.motion_status) - 1] = '\0';
        g_display_data.is_moving = is_moving;
        g_display_data.needs_refresh = true;
        xSemaphoreGive(g_display_mutex);
        
        ESP_LOGI(TAG, "Motion status updated: %s (moving: %s)", 
                status, is_moving ? "yes" : "no");
    }
}

void display_refresh(void) {
    if (xSemaphoreTake(g_display_mutex, portMAX_DELAY) == pdTRUE) {
        if (g_display_data.needs_refresh) {
            update_display();
            g_display_data.needs_refresh = false;
        }
        xSemaphoreGive(g_display_mutex);
    }
}

void display_task(void* pvParameters) {
    ESP_LOGI(TAG, "Display task started");
    
    // Initialize display
    if (!display_init()) {
        ESP_LOGE(TAG, "Failed to initialize display, task exiting");
        vTaskDelete(NULL);
        return;
    }
    
    // Main display task loop
    while (1) {
        // Refresh display if needed
        display_refresh();
        
        // Update display periodically (even if no changes)
        static uint32_t last_periodic_update = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (now - last_periodic_update > 1000) { // Update every second
            if (xSemaphoreTake(g_display_mutex, portMAX_DELAY) == pdTRUE) {
                g_display_data.needs_refresh = true;
                xSemaphoreGive(g_display_mutex);
            }
            last_periodic_update = now;
        }
        
        // Task delay
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Cleanup (should never reach here)
    display_deinit();
    vTaskDelete(NULL);
}
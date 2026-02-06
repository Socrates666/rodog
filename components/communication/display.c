#include "display.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "board.h"

// Tag for logging
static const char* TAG = "display";

#define DISPLAY_REFRESH_PERIOD_MS 1000

static TaskHandle_t g_display_task_handle = NULL;

// Forward declarations
static void display_task(void* pvParameters);
static void update_display(void);


static bool read_battery_percent(uint8_t *percent) {
    return bsp_read_battery_percent(percent) == ESP_OK;
}

static void read_ip_address(char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }

    if (!getIP(out, out_len)) {
        snprintf(out, out_len, "0.0.0.0");
    }
}

static void read_motion_status(char *out, size_t out_len, bool *is_moving) {
    if (!out || out_len == 0 || !is_moving) {
        return;
    }

    ActionState state = get_current_action_state();
    switch (state) {
        case WALKING_FORWARD:
            snprintf(out, out_len, "FWD");
            *is_moving = true;
            break;
        case WALKING_BACKWARD:
            snprintf(out, out_len, "BWD");
            *is_moving = true;
            break;
        case TURNING_LEFT:
            snprintf(out, out_len, "TURNL");
            *is_moving = true;
            break;
        case TURNING_RIGHT:
            snprintf(out, out_len, "TURNR");
            *is_moving = true;
            break;
        case WAVING:
            snprintf(out, out_len, "WAVE");
            *is_moving = true;
            break;
        case STANDING:
        default:
            snprintf(out, out_len, "IDLE");
            *is_moving = false;
            break;
    }
}

/**
 * @brief Update display with current data
 */
static void update_display(void) {
    char buffer[64];
    char ip_address[16] = {0};
    char motion_status[16] = {0};
    bool is_moving = false;
    uint8_t battery_level = 0;
    bool has_battery = read_battery_percent(&battery_level);

    read_ip_address(ip_address, sizeof(ip_address));
    read_motion_status(motion_status, sizeof(motion_status), &is_moving);

    bsp_ssd1306_fill_rect(0, 0, BSP_SSD1306_WIDTH, BSP_SSD1306_HEIGHT, false);

    if (has_battery) {
        snprintf(buffer, sizeof(buffer), "BAT: %3d%%", battery_level);
    } else {
        snprintf(buffer, sizeof(buffer), "BAT --%%");
    }
    bsp_ssd1306_print_string(0, 0, buffer, true);

    snprintf(buffer, sizeof(buffer), "IP: %.16s", ip_address);
    bsp_ssd1306_print_string(0, 12, buffer, true);


    snprintf(buffer, sizeof(buffer), "STAT: %.16s", motion_status);
    bsp_ssd1306_print_string(0, 24, buffer, true);

    bsp_ssd1306_update_screen();
}

bool display_init(void) {
    ESP_LOGI(TAG, "Initializing display module");

    if (bsp_ssd1306_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize OLED");
        return false;
    }

    if (xTaskCreate(display_task, "display_task", 4 * 1024, NULL, 6, &g_display_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        return false;
    }

    ESP_LOGI(TAG, "Display module initialized successfully");
    return true;
}

void display_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing display module");

    if (g_display_task_handle != NULL) {
        vTaskDelete(g_display_task_handle);
        g_display_task_handle = NULL;
    }

    bsp_ssd1306_clear();
    ESP_LOGI(TAG, "Display module deinitialized");
}

static void display_task(void* pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Display task started");
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(DISPLAY_REFRESH_PERIOD_MS);

    while (1) {
        update_display();
        vTaskDelayUntil(&last_wake, period);
    }
}
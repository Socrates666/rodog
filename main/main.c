#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "protocol_examples_common.h"

#include "display.h"

// Tag for logging
static const char* TAG = "main";

// Display task handle
static TaskHandle_t g_display_task_handle = NULL;

// Simulated battery level (for testing)
static uint8_t g_simulated_battery = 100;

// Simulated motion states
static const char* g_motion_states[] = {
    "Idle",
    "Walking",
    "Running",
    "Turning",
    "Stopped"
};
static int g_current_motion_state = 0;

/**
 * @brief Simulate battery drain
 */
static void simulate_battery_drain(void) {
    if (g_simulated_battery > 0) {
        g_simulated_battery--;
        display_update_battery(g_simulated_battery);
        ESP_LOGI(TAG, "Battery: %d%%", g_simulated_battery);
    }
}

/**
 * @brief Simulate motion state changes
 */
static void simulate_motion_changes(void) {
    g_current_motion_state = (g_current_motion_state + 1) % 5;
    bool is_moving = (g_current_motion_state != 0 && g_current_motion_state != 4);
    display_update_motion_status(g_motion_states[g_current_motion_state], is_moving);
    ESP_LOGI(TAG, "Motion: %s (moving: %s)", 
            g_motion_states[g_current_motion_state], 
            is_moving ? "yes" : "no");
}

/**
 * @brief Get simulated IP address
 */
static void get_simulated_ip(char* ip_buffer, size_t buffer_size) {
    // Generate a simulated IP based on time
    uint32_t time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint8_t ip_part = (time_ms / 1000) % 256;
    
    snprintf(ip_buffer, buffer_size, "192.168.1.%d", 100 + (ip_part % 156));
}

/**
 * @brief Simulation task
 */
static void simulation_task(void* pvParameters) {
    ESP_LOGI(TAG, "Simulation task started");
    
    uint32_t last_battery_update = 0;
    uint32_t last_motion_update = 0;
    uint32_t last_ip_update = 0;
    
    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Update battery every 30 seconds
        if (now - last_battery_update > 30000) {
            simulate_battery_drain();
            last_battery_update = now;
        }
        
        // Update motion every 10 seconds
        if (now - last_motion_update > 10000) {
            simulate_motion_changes();
            last_motion_update = now;
        }
        
        // Update IP every 60 seconds
        if (now - last_ip_update > 60000) {
            char ip[16];
            get_simulated_ip(ip, sizeof(ip));
            display_update_ip(ip);
            last_ip_update = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "WaveGo Display Application Starting");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize networking
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize example networking (optional, for real IP)
    example_connect();
    
    // Create display task
    BaseType_t xReturn = xTaskCreate(
        display_task,          /* Task function */
        "display_task",        /* Task name */
        4096,                  /* Stack size */
        NULL,                  /* Task parameters */
        tskIDLE_PRIORITY + 2,  /* Priority */
        &g_display_task_handle /* Task handle */
    );
    
    if (xReturn != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        return;
    }
    
    // Wait for display to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Set initial display values
    display_update_battery(g_simulated_battery);
    display_update_motion_status("Initialized", false);
    
    // Get and display initial IP
    char initial_ip[16] = "192.168.1.100";
    display_update_ip(initial_ip);
    
    // Create simulation task
    xReturn = xTaskCreate(
        simulation_task,       /* Task function */
        "simulation_task",     /* Task name */
        2048,                  /* Stack size */
        NULL,                  /* Task parameters */
        tskIDLE_PRIORITY + 1,  /* Priority */
        NULL                   /* Task handle */
    );
    
    if (xReturn != pdPASS) {
        ESP_LOGE(TAG, "Failed to create simulation task");
    }
    
    ESP_LOGI(TAG, "Application started successfully");
    
    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Main task alive");
    }
}
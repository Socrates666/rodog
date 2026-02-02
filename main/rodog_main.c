/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "nvs_flash.h"
#include "board.h"
#include "leg.h"
#include "now_controler.h"
#include "esp_log.h"
#include "web_server.h"
#include "display.h"

static const char* TAG = "MAIN";

void atask(void* vParamter){
    axises data;
    while(1){
        bsp_read_gyro(&data);
        ESP_LOGI("gyro", "x:%.2f  y:%.2f  z:%.2f", data.x, data.y, data.z);
        bsp_read_accel(&data);
        ESP_LOGI("accel", "x:%.2f  y:%.2f  z:%.2f", data.x, data.y, data.z);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }

}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // example_wifi_init();
    // ret = example_espnow_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "ESPNOW初始化失败");
    }
    //  wifiInit();


    // webServerInit();
    ret = bsp_i2c_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "I2C初始化失败");
    }
    ret = bsp_icm20948_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "ICM20948初始化失败");
    }
    ret = bsp_pca9685_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "PCA9685初始化失败");
    }
    
    // Initialize and start display module
    ESP_LOGI(TAG, "Initializing display module");
    if (!display_init()) {
        ESP_LOGE(TAG, "Display module initialization failed");
    } else {
        // Create display task
        xTaskCreate(display_task, "display_task", 4096, NULL, 2, NULL);
        ESP_LOGI(TAG, "Display task created");
        
        // Set initial display values
        display_update_battery(100);  // Start with 100% battery
        display_update_motion_status("Initialized", false);
        display_update_ip("0.0.0.0");  // Will be updated when WiFi connects
    }
    
    start_wavego();
    webServerInit();
    
    // xTaskCreate(atask, "abc", 1024*16, NULL, 3, NULL);
    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

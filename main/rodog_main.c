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
#include "esp_log.h"
#include "web_server.h"
#include "micro_ros_uart.h"
#include "calibration.h"
#include "pca9685.h"
#include "display.h"

static const char* TAG = "MAIN";


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
    ret = bsp_ina219_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "INA219初始化失败");
    }
    ret = bsp_ssd1306_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "SSD1306初始化失败");
    }

    display_init();
    calibration_init();
    webServerInit();
    start_wavego_task();
    


    vTaskDelete(NULL);

}

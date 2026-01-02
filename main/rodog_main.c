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
// #include "components/communication/now_controler.h"
// #include "components/communication/web_server.h"
#include "nvs_flash.h"
#include "board.h"

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
    // start_wavego();
    // webServerInit();
    ESP_ERROR_CHECK(bsp_i2c_init());
    ESP_ERROR_CHECK(bsp_icm20948_init());
    ESP_ERROR_CHECK(bsp_pca9685_init());

    xTaskCreate(atask, "abc", 1024*16, NULL, 3, NULL);
    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

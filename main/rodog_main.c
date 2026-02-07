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

static float pwm_to_angle(uint16_t pwm){
    return (float)(pwm*1000000/4096/50-500)/2000*180;
}

static uint16_t angle_to_pwm(float angle){
    return (500+(angle/180)*2000)*50*4096/1000000;
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
    ret = bsp_ina219_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "INA219初始化失败");
    }
    display_init();
    calibration_init();
    // Apply middle positions from calibration to hardwar
    // for(int i=0;i<180;i++){
    //     pwm = angle_to_pwm(angle+k_servo_direction_defaults[id]*i);
    //     pca9685_set_pwm_value(id, pwm);
    //     ESP_LOGI(TAG, "angle:%f  pwm:%u", angle+k_servo_direction_defaults[id]*i, pwm);
    //     vTaskDelay(10/portTICK_PERIOD_MS);
    // }
    // for(int i=180;i>0;i--){
    //     pwm = angle_to_pwm(angle+k_servo_direction_defaults[id]*i);
    //     pca9685_set_pwm_value(id, pwm);
    //     ESP_LOGI(TAG, "angle:%f  pwm:%u", angle+k_servo_direction_defaults[id]*i, pwm);
    //     vTaskDelay(10/portTICK_PERIOD_MS);
    // }

    start_wavego_task();
    // start_wavego_task();
    // send_wavego_command(WALK_HEIGHT_ANGLE, 50, STANDING);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // send_wavego_command(WALK_HEIGHT_ANGLE, 50, WALKING_FORWARD);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // send_wavego_command(WALK_HEIGHT_ANGLE, 50, WALKING_BACKWARD);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    send_wavego_command(0, 50, INITIALIZING);
    webServerInit();


    vTaskDelete(NULL);

}

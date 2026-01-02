#include "board.h"

//==================================================================================
// i2c
//==================================================================================

i2c_master_bus_handle_t i2c_handle = NULL;
bool i2c_initialized = false;
esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .sda_io_num                   = BSP_I2C_SDA,
        .scl_io_num                   = BSP_I2C_SCL,
        .i2c_port                     = BSP_I2C_NUM,
        .flags.enable_internal_pullup = true,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,

    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_conf, &i2c_handle));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    esp_err_t err = i2c_del_master_bus(i2c_handle);
    i2c_initialized = false;
    return err;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    return i2c_handle;
}
//==================================================================================
// icm20948
//==================================================================================

#define ICM20948_ADDR 0x68

esp_err_t bsp_icm20948_init(void){
    //1.设置低速模式用于复位
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_ADDR,
        .scl_speed_hz = 10000,
        // .scl_wait_us = 2000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_cfg, &dev_handle));
    //2.开始复位
    uint8_t reset_cmd[] = {0x06, 0x80}; 
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, reset_cmd, 2, 10));
        vTaskDelay(pdMS_TO_TICKS(100));
    // === 步骤4：重新选择Bank 0（复位后可能改变）===
    // i2c_master_transmit(dev_handle, bank_sel, 2, 1000);
        vTaskDelay(pdMS_TO_TICKS(1));
    ESP_LOGI("ICM", "复位完成");
    //3.取消低速模式，切换到正常模式
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    dev_cfg.scl_speed_hz = 400000;
    dev_handle=NULL;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_cfg, &dev_handle));
    //4.配置icm20948驱动
    icm20948_cfg_t icm_cfg={
        .dev_handle=dev_handle,
        .transmit_data_icm20948=i2c_master_transmit,
        .receive_data_icm20948=i2c_master_receive,
    };
    icm20948_init(&icm_cfg);
    return ESP_OK;
}

esp_err_t bsp_read_gyro(axises* data){
    icm20948_gyro_read_dps((axises*)data);
    if(data == NULL) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t bsp_read_accel(axises* data){
    icm20948_accel_read_g((axises*)data);
    if(data == NULL) return ESP_FAIL;
    return ESP_OK;
}



//==================================================================================
// pca9685
//==================================================================================
// PCA9685配置
#define PCA9685_ADDR 0x40
#define PCA9685_FREQ 50
esp_err_t bsp_pca9685_init(void){

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9685_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_cfg, &dev_handle));
    esp_err_t res = i2c_master_probe(i2c_handle, PCA9685_ADDR, 1000);
    if(res == ESP_OK){
        ESP_LOGI("PROBE", "Device found at 0x%02X", PCA9685_ADDR);
    }else{
        ESP_LOGE("PROBE", "Device found at 0x%02X failed", PCA9685_ADDR);
    }
    
    return ESP_OK;
}

enum t_i2caddr{
    I2C_ADDR_40 = 0x40, ///< address 0x40 no jumpers required.
    I2C_ADDR_41 = 0x41, ///< address 0x41 bridge A0.
    I2C_ADDR_44 = 0x44, ///< address 0x44 bridge A1.
    I2C_ADDR_45 = 0x45 ///< address 0x45 bridge A0 & A1.

};

esp_err_t bsp_ina219_init(uint16_t i){

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i,
        .scl_speed_hz = 10000,
        .scl_wait_us = 1000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_cfg, &dev_handle));
    esp_err_t res = i2c_master_probe(i2c_handle, i, 1000);
    if(res == ESP_OK){
        ESP_LOGI("PROBE", "Device found at 0x%02X", i);
    }else{
        // ESP_LOGE("PROBE", "Device found at 0x%02X failed", addr[i]);
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    }
     
    // icm20948_cfg_t icm_cfg;
    // icm_cfg.dev_handle=dev_handle;
    // icm_cfg.transmit_data_icm20948=i2c_master_transmit;
    // icm_cfg.receive_data_icm20948=i2c_master_receive;
    // icm20948_init(&icm_cfg);
    return ESP_OK;
}
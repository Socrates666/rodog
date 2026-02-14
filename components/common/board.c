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
    esp_err_t err = i2c_master_transmit(dev_handle, reset_cmd, 2, 10);
    ESP_LOGI("ICM", "复位失败，错误码：%d, 重试，，，", err);
    for(int i=0;i<5;i++){
        err = i2c_master_transmit(dev_handle, reset_cmd, 2, 10);
        if(err == ESP_OK) break;
    }
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

// Set ICM20948 output data rate (Hz)
esp_err_t bsp_icm20948_set_rate(uint16_t hz){
    if (hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Output Data Rate = 1.125kHz / (1 + divider)
    float divider_f = (1125.0f / (float)hz) - 1.0f;
    if (divider_f < 0.0f) divider_f = 0.0f;
    if (divider_f > 255.0f) divider_f = 255.0f;

    uint16_t divider = (uint16_t)(divider_f + 0.5f);
    icm20948_gyro_sample_rate_divider((uint8_t)divider);
    icm20948_accel_sample_rate_divider(divider);
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
        .scl_speed_hz = 1000000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_cfg, &dev_handle));
     esp_err_t res = 0;//i2c_master_probe(i2c_handle, PCA9685_ADDR, 1000);
    if(1){
        ESP_LOGI("PROBE", "Device found at 0x%02X", PCA9685_ADDR);
        pca9685_config_t pca_cfg={
            .dev_handle=dev_handle,
            .transmit_data_pca9685=i2c_master_transmit,
            .receive_data_pca9685=i2c_master_receive,
        };
        res = pca9685_init(&pca_cfg);
        if(res != ESP_OK){
            ESP_LOGE("PCA9685", "Initialization failed");
            return res;
        }
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

#define INA219_ADDR 0x42

// Battery voltage limits for percentage calculation
#define BSP_BATTERY_VOLT_MIN 6.0f
#define BSP_BATTERY_VOLT_MAX 8.4f
esp_err_t bsp_ina219_init(void){

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_ADDR,
        .scl_speed_hz = 10000,
        .scl_wait_us = 1000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_cfg, &dev_handle));
    esp_err_t res = i2c_master_probe(i2c_handle, INA219_ADDR, 1000);
    if(res == ESP_OK){
        ESP_LOGI("PROBE", "Device found at 0x%02X", INA219_ADDR);
        ina219_config_t ina_cfg = {
            .dev_handle = dev_handle,
            .transmit_data_ina219 = i2c_master_transmit,
            .receive_data_ina219 = i2c_master_receive,
            .shunt_resistor = 0.01f,
        };
        res = ina219_init(&ina_cfg);
        if (res != ESP_OK) {
            ESP_LOGE("INA219", "Initialization failed: %d", res);
            ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
            return res;
        }
        return ESP_OK;
    }else{
        ESP_LOGE("PROBE", "Device found at 0x%02X failed", INA219_ADDR);
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
        return res;
    }
}

//==================================================================================
// ina219
//==================================================================================

// Read battery percentage based on INA219 bus voltage
esp_err_t bsp_read_battery_percent(uint8_t *percent){
    if (!percent) {
        return ESP_ERR_INVALID_ARG;
    }

    float voltage = 0.0f;
    esp_err_t res = ina219_read_bus_voltage(&voltage);
    if (res != ESP_OK) {
        return res;
    }

    float pct = (voltage - BSP_BATTERY_VOLT_MIN) / (BSP_BATTERY_VOLT_MAX - BSP_BATTERY_VOLT_MIN) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    *percent = (uint8_t)(pct + 0.5f);
    return ESP_OK;
}

//==================================================================================
// ssd1306
//==================================================================================

#define SSD1306_ADDR 0x3C

static i2c_master_dev_handle_t ssd1306_handle = NULL;
static ssd1306_config_t ssd1306_cfg = {0};

esp_err_t bsp_ssd1306_init(void){
    if (ssd1306_handle != NULL) {
        return ESP_OK;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_ADDR,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &dev_cfg, &ssd1306_handle));
    ssd1306_cfg.dev_handle = ssd1306_handle;
    ssd1306_cfg.transmit_data_ssd1306 = i2c_master_transmit;
    ssd1306_cfg.receive_data_ssd1306 = i2c_master_receive;

    esp_err_t res = ssd1306_init(&ssd1306_cfg, SSD1306_ADDR_0X3C);
    if (res != ESP_OK) {
        ESP_LOGE("SSD1306", "Initialization failed: %d", res);
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(ssd1306_handle));
        ssd1306_handle = NULL;
    }
    return res;
}

// Clear OLED display
esp_err_t bsp_ssd1306_clear(void){
    return ssd1306_clear_screen();
}

// Fill a rectangle on OLED
esp_err_t bsp_ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool color){
    return ssd1306_fill_rect(x, y, width, height, color);
}

// Print string on OLED
esp_err_t bsp_ssd1306_print_string(uint8_t x, uint8_t y, const char* str, bool color){
    return ssd1306_print_string(x, y, str, color);
}

// Update OLED screen
esp_err_t bsp_ssd1306_update_screen(void){
    return ssd1306_update_screen();
}

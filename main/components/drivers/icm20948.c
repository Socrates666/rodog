/*
* icm20948.c
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi
*/


#include "icm20948.h"

const char* tag = "icm20948";

static float gyro_scale_factor;
static float accel_scale_factor;

static icm20948_cfg_t icm;

/* Static Functions */
static void cs_high();
static void cs_low();
static void select_user_bank(userbank ub);
static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg);
static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val);
static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);
static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len);

/* Main Functions */
void icm20948_init(icm20948_cfg_t* icm_cfg)
{	
	memcpy(&icm, icm_cfg, sizeof(icm20948_cfg_t));
	if(icm_cfg->dev_handle == NULL){
		ESP_LOGE(tag, "i2c没有初始化");
		return;
	}
	icm.dev_handle = icm_cfg->dev_handle;
    icm.transmit_data_icm20948 = icm_cfg->transmit_data_icm20948;
    icm.receive_data_icm20948 = icm_cfg->receive_data_icm20948;
	// icm20948_device_reset();
	// icm20948_wakeup();
	// select_user_bank(0);
	// uint8_t reset_cmd[] = {0x06, 0x80}; // PWR_MGMT_1 + 复位位
    // ESP_ERROR_CHECK(i2c_master_transmit(icm.dev_handle, reset_cmd, 2, 1000));
	// vTaskDelay(pdMS_TO_TICKS(100));  // 必须等待100ms

	while(!icm20948_who_am_i());
	// icm20948_device_reset();
	icm20948_wakeup();

	icm20948_clock_source(1);
	icm20948_odr_align_enable();
	
	// icm20948_spi_slave_enable();
	
	icm20948_gyro_low_pass_filter(0);
	icm20948_accel_low_pass_filter(0);

	icm20948_gyro_sample_rate_divider(0);
	icm20948_accel_sample_rate_divider(0);

	icm20948_gyro_calibration();
	icm20948_accel_calibration();

	icm20948_gyro_full_scale_select(_2000dps);
	icm20948_accel_full_scale_select(_16g);
	ESP_LOGI(tag, "icm20948初始化成功");
}

void icm20948_gyro_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_accel_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor; 
	// Add scale factor because calibraiton function offset gravity acceleration.
}



void icm20948_gyro_read_dps(axises* data)
{
	icm20948_gyro_read(data);

	data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(axises* data)
{
	icm20948_accel_read(data);

	data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;
}




/* Sub Functions */
bool icm20948_who_am_i()
{
	uint8_t icm20948_id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);

	if(icm20948_id == ICM20948_ID)
		return true;
	else{

		return false;
	}
}



void icm20948_device_reset()
{
	uint8_t write_reg[2];
	write_reg[0] = B0_PWR_MGMT_1;
	write_reg[1] = 0x80;
	ESP_ERROR_CHECK(icm.transmit_data_icm20948((icm.dev_handle), write_reg, 1, 1000));
	ESP_ERROR_CHECK(icm.receive_data_icm20948((icm.dev_handle), write_reg+1, 1, 1000));

	write_reg[1] |= 0x80;
	ESP_ERROR_CHECK(icm.transmit_data_icm20948((icm.dev_handle), write_reg, 2, 1000));
	vTaskDelay(100);
	
}


void icm20948_wakeup()
{
	uint8_t write_reg[2];
	write_reg[0] = B0_PWR_MGMT_1;
	write_reg[1] = 0;
	ESP_ERROR_CHECK(icm.transmit_data_icm20948((icm.dev_handle), write_reg, 1, 1000));
	ESP_ERROR_CHECK(icm.receive_data_icm20948((icm.dev_handle), write_reg+1, 1, 1000));
	write_reg[1] &= (~BIT6);
	ESP_ERROR_CHECK(icm.transmit_data_icm20948((icm.dev_handle), write_reg, 2, 1000));
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

void icm20948_sleep()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

void icm20948_spi_slave_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x10;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_reset()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x20;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
	new_val |= config;

	write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, new_val);	
}

void icm20948_clock_source(uint8_t source)
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_odr_align_enable()
{
	write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}


void icm20948_gyro_calibration()
{
	axises temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_gyro_read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; 
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF; 
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;
	
	write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

void icm20948_accel_calibration()
{
	axises temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;
	
	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_accel_read(&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;
	}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];
	
	write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	
	switch(full_scale)
	{
		case _250dps :
			new_val |= 0x00;
			gyro_scale_factor = 131.0;
			break;
		case _500dps :
			new_val |= 0x02;
			gyro_scale_factor = 65.5;
			break;
		case _1000dps :
			new_val |= 0x04;
			gyro_scale_factor = 32.8;
			break;
		case _2000dps :
			new_val |= 0x06;
			gyro_scale_factor = 16.4;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	
	switch(full_scale)
	{
		case _2g :
			new_val |= 0x00;
			accel_scale_factor = 16384;
			break;
		case _4g :
			new_val |= 0x02;
			accel_scale_factor = 8192;
			break;
		case _8g :
			new_val |= 0x04;
			accel_scale_factor = 4096;
			break;
		case _16g :
			new_val |= 0x06;
			accel_scale_factor = 2048;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}

// esp_err_t icm20948_wake_up(icm20948_handle_t sensor)
// {
// 	esp_err_t ret;
// 	uint8_t tmp;
// 	ret = icm20948_read(sensor, 0x06, &tmp, 1);
// 	if (ESP_OK != ret) {
// 		return ret;
// 	}
// 	tmp &= (~BIT6);
// 	ret = icm20948_write(sensor, 0x06, &tmp, 1);
// 	return ret;
// }

// esp_err_t icm20948_reset(icm20948_handle_t sensor)
// {
// 	esp_err_t ret;
// 	uint8_t tmp;

// 	ret = icm20948_read(sensor, 0x06, &tmp, 1);
// 	if (ret != ESP_OK)
// 		return ret;
// 	tmp |= 0x80;
// 	ret = icm20948_write(sensor, 0x06, &tmp, 1);
// 	if (ret != ESP_OK)
// 		return ret;

// 	return ret;
// }
/* Static Functions */
static void cs_high()
{
	return;
}

static void cs_low()
{
	return;
}

static void select_user_bank(userbank ub)
{
	uint8_t write_reg[2];
	write_reg[0] = REG_BANK_SEL;
	write_reg[1] = ub;

	if(icm.dev_handle==NULL) printf("handle is null\n");
	ESP_ERROR_CHECK(icm.transmit_data_icm20948((icm.dev_handle), write_reg, 2, 1000));
	vTaskDelay(pdMS_TO_TICKS(1));  // Bank切换延迟
}

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
	uint8_t read_reg = reg;
	uint8_t reg_val=0;
	select_user_bank(ub);
	// if(icm.dev_handle != NULL) printf("handle is wrong\n");
	ESP_ERROR_CHECK(icm.transmit_data_icm20948((icm.dev_handle), &read_reg, 1, 1000));
	ESP_ERROR_CHECK(icm.receive_data_icm20948((icm.dev_handle), &reg_val, 1, 1000));

	return reg_val;
}

static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
	uint8_t write_reg[2];
	write_reg[0] = reg;
	write_reg[1] = val;

	select_user_bank(ub);

	icm.transmit_data_icm20948((icm.dev_handle), write_reg, 2, 1000);

}

static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
	uint8_t read_reg = reg;
	static uint8_t reg_val[6];
	select_user_bank(ub);

	cs_low();
	icm.transmit_data_icm20948(icm.dev_handle, &read_reg, 1, 1000);
	icm.receive_data_icm20948(icm.dev_handle, reg_val, len, 1000);
	cs_high();

	return reg_val;
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = reg;
	select_user_bank(ub);

	cs_low();
	icm.transmit_data_icm20948(icm.dev_handle, &write_reg, 1, 1000);
	icm.transmit_data_icm20948(icm.dev_handle, val, len, 1000);
	cs_high();
}


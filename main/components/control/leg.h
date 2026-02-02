#ifndef LEG_H
#define LEG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pca9685.h"
#include "board.h"
#define LEG_USE_NVS_CALIB 0
// 预编译选项：是否从 NVS 读取校准值（0 则只用工厂默认值）


// PCA9685 配置
#define PCA9685_ADDR 0x40
#define PCA9685_FREQ 50

// 伺服电机配置
#define SERVO_RANGE 90
#define SERVO_FREQ 50
#define SERVOMIN 0
#define SERVOMAX 500
#define MiddlePosition 307

// 腿部定义
#define LEG_A_FORE 8
#define LEG_A_BACK 9
#define LEG_A_WAVE 10

#define LEG_B_WAVE 13
#define LEG_B_FORE 14
#define LEG_B_BACK 15

#define LEG_C_FORE 7
#define LEG_C_BACK 6
#define LEG_C_WAVE 5

#define LEG_D_WAVE 2
#define LEG_D_FORE 1
#define LEG_D_BACK 0

// 机械参数
#define LINKAGE_W 19.15
#define WIGGLE_ERROR 0
#define LINKAGE_S 12.2
#define LINKAGE_A 40.0
#define LINKAGE_B 40.0
#define LINKAGE_C 39.8153
#define LINKAGE_D 31.7750
#define LINKAGE_E 30.8076

// 步行参数
#define WALK_HEIGHT_MAX  110
#define WALK_HEIGHT_MIN  75
#define WALK_HEIGHT      95
#define WALK_LIFT        9
#define WALK_RANGE       40
#define WALK_ACC         5
#define WALK_EXTENDED_X  16
#define WALK_EXTENDED_Z  25
#define WALK_SIDE_MAX    30
#define WALK_MASS_ADJUST 21
#define STAND_HEIGHT     95

// 控制参数
#define SERVO_MOVE_EVERY 4
#define MAX_TEST 125

// 对外接口
void servo_setup(void);
void init_pos_all(void);
void middle_pos_all(void);
void goal_pos_all(void);
void goal_pwm_set(uint8_t servo_num, double angle_input);
void servo_set_default_pwm(uint8_t servo_id, int pwm_value);
esp_err_t servo_config_init(void);
esp_err_t servo_config_load_from_nvs(void);
esp_err_t servo_config_save_to_nvs(void);
esp_err_t servo_config_reset_defaults(void);
void start_wavego(void);

// 新的访问接口，替代直接暴露的全局变量
int leg_get_servo_middle_pwm(uint8_t servo_id);
int leg_get_servo_direction(uint8_t servo_id);
int leg_get_current_pwm(uint8_t servo_id);
void leg_get_servo_snapshot(int middle_out[16], int direction_out[16]);
int leg_clamp_servo_pwm(uint8_t servo_id, int pwm_value);
esp_err_t leg_set_servo_pwm(uint8_t servo_id, int pwm_value);
void set_servo(uint8_t servo_id, int offset);

#ifdef __cplusplus
}
#endif

#endif // LEG_H
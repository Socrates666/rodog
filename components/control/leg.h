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
#include "board.h"

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
#define WALK_HEIGHT_MAX_ANGLE  55
#define WALK_HEIGHT_MIN_ANGLE  5
#define WALK_HEIGHT_ANGLE      30
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

typedef struct {
    double x;
    double y;
    double z;
} LegPosition;

enum {
    WALKING,
    STANDING,
    WAVING,
    TURNING_LEFT,
    TURNING_RIGHT,
} typedef ActionState;

esp_err_t start_wavego_task(void);
esp_err_t stop_wavego_task(void);
esp_err_t send_wavego_command(int wave_height, int wave_speed, ActionState state);

#ifdef __cplusplus
}
#endif

#endif // LEG_H
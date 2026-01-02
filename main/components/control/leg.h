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
// #include <i2cdev.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pca9685.h"
// #include "board.h"


/*
14-
15+

*/


// PCA9685配置
#define PCA9685_ADDR 0x40
#define PCA9685_FREQ 50

// 伺服电机配置
#define SERVOMIN  263 // 最小脉冲长度
#define SERVOMAX  463 // 最大脉冲长度
#define SERVO_RANGE 90
#define SERVO_FREQ 50
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

// 全局变量声明
extern int ServoMiddlePWM[16];
extern int ServoDirection[16];

/// 机械参数 - 改为宏定义
#define LINKAGE_W 19.15
#define WIGGLE_ERROR 0
#define LINKAGE_S 12.2
#define LINKAGE_A 40.0
#define LINKAGE_B 40.0
#define LINKAGE_C 39.8153
#define LINKAGE_D 31.7750
#define LINKAGE_E 30.8076

// 步行参数 - 改为宏定义
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

// 控制参数 - 改为宏定义
#define SERVO_MOVE_EVERY 4
#define MAX_TEST 125
extern float GLOBAL_STEP;
extern int   STEP_DELAY;   
extern float STEP_ITERATE;
extern uint8_t GAIT_TYPE;
extern uint8_t debugMode;
extern uint8_t funcMode;
extern uint8_t STAND_STILL;
extern int moveFB;
extern int moveLR;
extern int gestureUD;
extern int gestureLR;

// 传感器数据
extern float ACC_X;
extern float ACC_Y;
extern float ACC_Z;

// 缓冲区和状态
extern int legPosBuffer[12];
extern int GoalPWM[16];
extern int CurrentPWM[16];
extern int LastPWM[16];
extern float WALK_LIFT_PROP;
extern double linkageBuffer[32];

// 预计算值
extern float LAxLA;
extern float LBxLB;
extern float LWxLW;
extern float LExLE;
extern float LAxLA_LBxLB;
extern float LBxLB_LAxLA;
extern float L_CD;
extern float LAx2;
extern float LBx2;
extern float E_PI;
extern float LSs2;
extern float aLCDE;
extern float sLEDC;
extern float O_WLP;
extern float WALK_ACCx2;
extern float WALK_H_L;

// PCA9685设备句柄
extern i2c_dev_t pca9685_dev;

// 函数声明
void servoDebug(uint8_t servoID, int offset);
void ServoSetup(void);
void setPWM(uint8_t num, uint16_t value);
void initPosAll(void);
void middlePosAll(void);
void GoalPosAll(void);
void goalPWMSet(uint8_t servoNum, double angleInput);
void simpleLinkageIK(double LA, double LB, double aIn, double bIn, 
                     uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta);
void wigglePlaneIK(double LA, double aIn, double bIn, 
                   uint8_t outputAlpha, uint8_t outputLen);
void singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, 
                      double xIn, double yIn, uint8_t outputBeta, 
                      uint8_t outputX, uint8_t outputY);
void singleLegCtrl(uint8_t LegNum, double xPos, double yPos, double zPos);
void standUp(double cmdInput);
void singleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, 
                    float directionInput, double extendedX, double extendedZ);
void simpleGait(float GlobalInput, float directionAngle, int turnCmd);
void triangularGait(float GlobalInput, float directionAngle, int turnCmd);
void gaitTypeCtrl(float GlobalStepInput, float directionCmd, int turnCmd);
void standMassCenter(float aInput, float bInput);
void pitchYawRoll(float pitchInput, float yawInput, float rollInput);
float besselCtrl(float numStart, float numEnd, float rateInput);
void functionStayLow(void);
void functionJump(void);
void start_wavego(void);
#ifdef __cplusplus
}
#endif

#endif // LEG_H
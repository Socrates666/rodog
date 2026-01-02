#include <stdio.h>
#include "leg.h"
#define TAG "WAVEGO"
static SemaphoreHandle_t i2c_mutex;
// 全局变量
int ServoMiddlePWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                          MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                          MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                          MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int ServoDirection[16] = {-1,  1,  1,  1,
                          1, -1, -1,  1,
                          -1,  1,  1,  1,
                          1, -1, -1,  1};

static uint8_t servo_list[12]={0,1,2,5,6,7,8,9,10,13,14,15};

float GLOBAL_STEP  = 0;           // 改为变量
int   STEP_DELAY   = 4;           // 可改为宏，但如果需要运行时修改则保留为变量
float STEP_ITERATE = 0.04;        // 可改为宏，但如果需要运行时修改则保留为变量
float WALK_CYCLE_GLOBAL = 0;    // 0-1.
float WALK_LIFT_PROP    = 0.25; // WALK_LIFT_TIME<1.
uint8_t GAIT_TYPE = 0;
uint8_t debugMode = 0;
uint8_t funcMode = 0;
uint8_t STAND_STILL = 0;
int moveFB = 0;
int moveLR = 0;
int gestureUD = 0;
int gestureLR = 0;

// 传感器数据（需要根据实际传感器添加）
float ACC_X = 0;
float ACC_Y = 0;
float ACC_Z = 0;

// 缓冲区和状态
int legPosBuffer[12] = { WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                        -WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                         WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                        -WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z};

int GoalPWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};
int CurrentPWM[16] = {SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN,
                   SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN,
                   SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN,
                   SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN};
int LastPWM[16] = {SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN,
                   SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN,
                   SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN,
                   SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN};

double linkageBuffer[32] = {0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0};


// 预计算值
float LAxLA;
float LBxLB;
float LWxLW;
float LExLE;
float LAxLA_LBxLB;
float LBxLB_LAxLA;
float L_CD;
float LAx2;
float LBx2;
float E_PI;
float LSs2;
float aLCDE;
float sLEDC;
float O_WLP;
float WALK_ACCx2;
float WALK_H_L;

// PCA9685设备句柄
i2c_dev_t pca9685_dev;
void servoDebug(uint8_t servoID, int offset){
  CurrentPWM[servoID] += offset;
  setPWM(servoID, CurrentPWM[servoID]);  
}
    
// 伺服初始化
void ServoSetup(void) {
    esp_err_t err=1;
    
    // 初始化I2C
    ESP_ERROR_CHECK(i2cdev_init());
    
    // 初始化PCA9685
    memset(&pca9685_dev, 0, sizeof(i2c_dev_t));
    
    err = pca9685_init_desc(&pca9685_dev, PCA9685_ADDR, I2C_MASTER_NUM, I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init desc failed");
        return;
    }
    
    err = pca9685_init(&pca9685_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed");
        return;
    }
    
    err = pca9685_restart(&pca9685_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 restart failed");
        return;
    }
    err = pca9685_sleep(&pca9685_dev, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 wake failed");
        return;
    }
    
    err = pca9685_set_pwm_frequency(&pca9685_dev, PCA9685_FREQ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 set frequency failed");
        return;
    }
    pca9685_set_output_open_drain(&pca9685_dev, false);
    err = pca9685_set_output_inverted(&pca9685_dev, false);
    // 初始化预计算值
    LAxLA = LINKAGE_A * LINKAGE_A;
    LBxLB = LINKAGE_B * LINKAGE_B;
    LWxLW = LINKAGE_W * LINKAGE_W;
    LExLE = LINKAGE_E * LINKAGE_E;
    LAxLA_LBxLB = LAxLA - LBxLB;
    LBxLB_LAxLA = LBxLB - LAxLA;
    L_CD = (LINKAGE_C + LINKAGE_D) * (LINKAGE_C + LINKAGE_D);
    LAx2  = 2 * LINKAGE_A;
    LBx2  = 2 * LINKAGE_B;
    E_PI  = 180 / M_PI;
    LSs2  = LINKAGE_S / 2;
    aLCDE = atan((LINKAGE_C + LINKAGE_D) / LINKAGE_E);
    sLEDC = sqrt(LINKAGE_E * LINKAGE_E + (LINKAGE_D + LINKAGE_C) * (LINKAGE_D + LINKAGE_C));
    O_WLP = 1 - WALK_LIFT_PROP;
    WALK_ACCx2 = WALK_ACC * 2;
    WALK_H_L = WALK_HEIGHT - WALK_LIFT;
    
    ESP_LOGI(TAG, "Servo setup complete");
}

// 设置单个PWM值
void setPWM(uint8_t num, uint16_t value) {
    esp_err_t err = pca9685_set_pwm_value(&pca9685_dev, num, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set PWM failed for servo %d", num);
    }
}

// 设置所有位置到中间值
void initPosAll(void) {
    for (int i = 0; i < 16; i++) {
        setPWM(i, MiddlePosition);
        CurrentPWM[i] = MiddlePosition;
        vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_EVERY));
    }
    // ESP_LOGI(TAG, "All servos initialized to middle position");
}

void middlePosAll(void) {
    for (int i = 0; i < 16; i++) {
        setPWM(i, ServoMiddlePWM[i]);
        CurrentPWM[i] = ServoMiddlePWM[i];
        ESP_LOGD(TAG, "Servo %d: %d", i, CurrentPWM[i]);
        vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_EVERY));
    }
    ESP_LOGI(TAG, "All servos set to middle position");
}

void GoalPosAll(void) {
    for (int i = 0; i < 16; i++) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            setPWM(i, GoalPWM[i]);
            xSemaphoreGive(i2c_mutex);
        }
    
        vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_EVERY));
    }
}

void goalPWMSet(uint8_t servoNum, double angleInput) {
    int pwmGet;
    if (angleInput == 0) {
        pwmGet = SERVOMIN;
    } else {
        pwmGet = round((SERVOMAX - SERVOMIN) * angleInput / SERVO_RANGE);
    }
    pwmGet = pwmGet * ServoDirection[servoNum] + ServoMiddlePWM[servoNum];
    GoalPWM[servoNum] = pwmGet;
    // setPWM(servoNum, pwmGet);
}

// 简单连杆逆运动学
void simpleLinkageIK(double LA, double LB, double aIn, double bIn, 
                     uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta) {
    double psi;
    double alpha;
    double omega;
    double beta;
    double L2C;
    double LC;
    double lambda;
    double delta;
    
    if (bIn == 0) {
        psi = acos((LAxLA_LBxLB + aIn * aIn) / (LAx2 * aIn)) * E_PI;
        alpha = 90 - psi;
        omega = acos((aIn * aIn + LBxLB_LAxLA) / (LBx2 * aIn)) * E_PI;
        beta = psi + omega;
    } else {
        L2C = aIn * aIn + bIn * bIn;
        LC = sqrt(L2C);
        lambda = atan(bIn / aIn) * E_PI;
        psi = acos((LAxLA_LBxLB + L2C) / (2 * LA * LC)) * E_PI;
        alpha = 90 - lambda - psi;
        omega = acos((LBxLB_LAxLA + L2C) / (2 * LC * LB)) * E_PI;
        beta = psi + omega;
    }
    delta = 90 - alpha - beta;
    
    linkageBuffer[outputAlpha] = alpha;
    linkageBuffer[outputBeta] = beta;
    linkageBuffer[outputDelta] = delta;
}

// 摆动平面逆运动学
void wigglePlaneIK(double LA, double aIn, double bIn, 
                   uint8_t outputAlpha, uint8_t outputLen) {
    double LB=0;
    double L2C=0;
    double LC=0;
    double alpha=0;
    double lambda=0;
    double psi=0;
    
    if (bIn > 0) {
        L2C = aIn * aIn + bIn * bIn;
        LC = sqrt(L2C);
        lambda = atan(aIn / bIn) * E_PI;
        psi = acos(LA / LC) * E_PI;
        LB = sqrt(L2C - LWxLW);
        alpha = psi + lambda - 90;
    } else if (bIn == 0) {
        alpha = asin(LA / aIn) * E_PI;
        L2C = aIn * aIn + bIn * bIn;
        LB = sqrt(L2C);
    } else if (bIn < 0) {
        bIn = -bIn;
        L2C = aIn * aIn + bIn * bIn;
        LC = sqrt(L2C);
        lambda = atan(aIn / bIn) * E_PI;
        psi = acos(LA / LC) * E_PI;
        LB = sqrt(L2C - LWxLW);
        alpha = 90 - lambda + psi;
    }
    
    linkageBuffer[outputAlpha] = alpha;
    linkageBuffer[outputLen] = LB - WIGGLE_ERROR;
}

// 单腿平面逆运动学
void singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, 
                      double xIn, double yIn, uint8_t outputBeta, 
                      uint8_t outputX, uint8_t outputY) {
    double bufferS = sqrt((xIn + LSs2) * (xIn + LSs2) + yIn * yIn);
    double lambda = acos(((xIn + LSs2) * (xIn + LSs2) + yIn * yIn + LAxLA - L_CD - LExLE) / (2 * bufferS * LA));
    double delta = atan((xIn + LSs2) / yIn);
    double beta = lambda - delta;
    double betaAngle = beta * E_PI;

    double theta = aLCDE;
    double omega = asin((yIn - cos(beta) * LA) / sLEDC);
    double nu = M_PI - theta - omega;
    double dFX = cos(nu) * LE;
    double dFY = sin(nu) * LE;

    double mu = M_PI / 2 - nu;
    double dEX = cos(mu) * LD;
    double dEY = sin(mu) * LD;

    double positionX = xIn + dFX - dEX;
    double positionY = yIn - dFY - dEY;
    
    linkageBuffer[outputBeta] = betaAngle;
    linkageBuffer[outputX] = positionX;
    linkageBuffer[outputY] = positionY;
}

// 控制单腿
void singleLegCtrl(uint8_t LegNum, double xPos, double yPos, double zPos) {
    uint8_t alphaOut, xPosBuffer, yPosBuffer, betaOut, betaB, betaC;
    uint8_t NumF, NumB, wiggleAlpha, wiggleLen, NumW;
    
    // 根据腿号设置参数
    switch (LegNum) {
        case 1:
            NumF = LEG_A_FORE;
            NumB = LEG_A_BACK;
            NumW = LEG_A_WAVE;
            alphaOut = 0; xPosBuffer = 1; yPosBuffer = 2;
            betaOut = 3; betaB = 4; betaC = 5;
            wiggleAlpha = 6; wiggleLen = 7;
            break;
        case 2:
            NumF = LEG_B_FORE;
            NumB = LEG_B_BACK;
            NumW = LEG_B_WAVE;
            alphaOut = 8; xPosBuffer = 9; yPosBuffer = 10;
            betaOut = 11; betaB = 12; betaC = 13;
            wiggleAlpha = 14; wiggleLen = 15;
            break;
        case 3:
            NumF = LEG_C_FORE;
            NumB = LEG_C_BACK;
            NumW = LEG_C_WAVE;
            alphaOut = 16; xPosBuffer = 17; yPosBuffer = 18;
            betaOut = 19; betaB = 20; betaC = 21;
            wiggleAlpha = 22; wiggleLen = 23;
            break;
        case 4:
            NumF = LEG_D_FORE;
            NumB = LEG_D_BACK;
            NumW = LEG_D_WAVE;
            alphaOut = 24; xPosBuffer = 25; yPosBuffer = 26;
            betaOut = 27; betaB = 28; betaC = 29;
            wiggleAlpha = 30; wiggleLen = 31;
            break;
        default:
            return;
    }

    // 执行逆运动学计算
    wigglePlaneIK(LINKAGE_W, zPos, yPos, wiggleAlpha, wiggleLen);
    singleLegPlaneIK(LINKAGE_S, LINKAGE_A, LINKAGE_C, LINKAGE_D, LINKAGE_E, 
                     xPos, linkageBuffer[wiggleLen], alphaOut, xPosBuffer, yPosBuffer);
    simpleLinkageIK(LINKAGE_B, LINKAGE_B, linkageBuffer[yPosBuffer], 
                   (linkageBuffer[xPosBuffer] - LSs2), betaOut, betaB, betaC);

    // 设置PWM目标值
    goalPWMSet(NumW, linkageBuffer[wiggleAlpha]);
    goalPWMSet(NumF, (90 - linkageBuffer[betaOut]));
    goalPWMSet(NumB, linkageBuffer[alphaOut]);
}

// 站立
void standUp(double cmdInput) {
    singleLegCtrl(1, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
    singleLegCtrl(2, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
    singleLegCtrl(4, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
}

// 控制单腿步态
void singleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, 
                    float directionInput, double extendedX, double extendedZ) {
    double rDist=0;
    double xGait=0;
    double yGait=0;
    double zGait=0;
    double rDirection = directionInput * M_PI / 180;

    if (cycleInput < (1 - WALK_LIFT_PROP)) {
        if (cycleInput <= (WALK_ACC / (WALK_ACCx2 + WALK_RANGE * statusInput)) * (1 - WALK_LIFT_PROP)) {
            yGait = WALK_H_L + cycleInput / (1 - WALK_LIFT_PROP - 
                    ((WALK_ACC + WALK_RANGE * statusInput) / (WALK_ACCx2 + WALK_RANGE * statusInput)) * 
                    (1 - WALK_LIFT_PROP)) * WALK_LIFT;
        } else if (cycleInput > (WALK_ACC / (WALK_ACCx2 + WALK_RANGE * statusInput)) * (1 - WALK_LIFT_PROP) && 
                   cycleInput <= ((WALK_ACC + WALK_RANGE * statusInput) / (WALK_ACCx2 + WALK_RANGE * statusInput)) * (1 - WALK_LIFT_PROP)) {
            yGait = WALK_HEIGHT;
        } else if (cycleInput > ((WALK_ACC + WALK_RANGE * statusInput) / (WALK_ACCx2 + WALK_RANGE * statusInput)) * (1 - WALK_LIFT_PROP) && 
                   cycleInput < ((WALK_ACCx2 + WALK_RANGE * statusInput) / (WALK_ACCx2 + WALK_RANGE * statusInput)) * (1 - WALK_LIFT_PROP)) {
            yGait = WALK_HEIGHT - ((cycleInput - ((WALK_ACC + WALK_RANGE * statusInput) / (WALK_ACCx2 + WALK_RANGE * statusInput)) * 
                    (1 - WALK_LIFT_PROP)) / ((WALK_ACC / (WALK_ACCx2 + WALK_RANGE * statusInput)) * (1 - WALK_LIFT_PROP))) * WALK_LIFT;
        }
        rDist = (WALK_RANGE * statusInput / 2 + WALK_ACC) - 
                (cycleInput / (1 - WALK_LIFT_PROP)) * (WALK_RANGE * statusInput + WALK_ACCx2);
    } else if (cycleInput >= (1 - WALK_LIFT_PROP)) {
        yGait = WALK_H_L;
        rDist = -(WALK_RANGE * statusInput / 2 + WALK_ACC) + 
                ((cycleInput - (1 - WALK_LIFT_PROP)) / WALK_LIFT_PROP) * 
                (WALK_RANGE * statusInput + WALK_ACCx2);
    }

    xGait = cos(rDirection) * rDist;
    zGait = sin(rDirection) * rDist;
    
    singleLegCtrl(LegNum, (xGait + extendedX), yGait, (zGait + extendedZ));
}

// 简单步态
void simpleGait(float GlobalInput, float directionAngle, int turnCmd) {
    float Group_A = GlobalInput;
    float Group_B = GlobalInput + 0.5;
    if (Group_B > 1) Group_B--;

    if (!turnCmd) {
        singleGaitCtrl(1, 1, Group_A, directionAngle, WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(4, 1, Group_A, -directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(2, 1, Group_B, directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(3, 1, Group_B, -directionAngle, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    } else if (turnCmd == -1) {
        singleGaitCtrl(1, 1.5, Group_A, 90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(4, 1.5, Group_A, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(2, 1.5, Group_B, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(3, 1.5, Group_B, -90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    } else if (turnCmd == 1) {
        singleGaitCtrl(1, 1.5, Group_A, -90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(4, 1.5, Group_A, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(2, 1.5, Group_B, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        singleGaitCtrl(3, 1.5, Group_B, 90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    }
}

// 三角形步态
void triangularGait(float GlobalInput, float directionAngle, int turnCmd) {
    float StepA, StepB, StepC, StepD;
    float aInput = 0, bInput = 0;

    StepB = GlobalInput;
    StepC = GlobalInput + 0.25;
    StepD = GlobalInput + 0.5;
    StepA = GlobalInput + 0.75;

    if (StepA > 1) StepA--;
    if (StepB > 1) StepB--;
    if (StepC > 1) StepC--;
    if (StepD > 1) StepD--;

    // 计算质量调整
    if (GlobalInput <= 0.25) {
        aInput = WALK_MASS_ADJUST - (GlobalInput / 0.125) * WALK_MASS_ADJUST;
        bInput = -WALK_MASS_ADJUST;
    } else if (GlobalInput > 0.25 && GlobalInput <= 0.5) {
        float adProp = GlobalInput - 0.25;
        aInput = -WALK_MASS_ADJUST + (adProp / 0.125) * WALK_MASS_ADJUST;
        bInput = -WALK_MASS_ADJUST + (adProp / 0.125) * WALK_MASS_ADJUST;
    } else if (GlobalInput > 0.5 && GlobalInput <= 0.75) {
        float adProp = GlobalInput - 0.5;
        aInput = WALK_MASS_ADJUST - (adProp / 0.125) * WALK_MASS_ADJUST;
        bInput = WALK_MASS_ADJUST;
    } else if (GlobalInput > 0.75 && GlobalInput <= 1) {
        float adProp = GlobalInput - 0.75;
        aInput = -WALK_MASS_ADJUST + (adProp / 0.125) * WALK_MASS_ADJUST;
        bInput = WALK_MASS_ADJUST - (adProp / 0.125) * WALK_MASS_ADJUST;
    }

    if (!turnCmd) {
        singleGaitCtrl(1, 1, StepA, directionAngle, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        singleGaitCtrl(4, 1, StepD, -directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
        singleGaitCtrl(2, 1, StepB, directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        singleGaitCtrl(3, 1, StepC, -directionAngle, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    } else if (turnCmd == -1) {
        singleGaitCtrl(1, 1.5, StepA, 90, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        singleGaitCtrl(4, 1.5, StepD, 90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
        singleGaitCtrl(2, 1.5, StepB, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        singleGaitCtrl(3, 1.5, StepC, -90, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    } else if (turnCmd == 1) {
        singleGaitCtrl(1, 1.5, StepA, -90, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        singleGaitCtrl(4, 1.5, StepD, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
        singleGaitCtrl(2, 1.5, StepB, 90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        singleGaitCtrl(3, 1.5, StepC, 90, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    }
}

// 步态选择
void gaitTypeCtrl(float GlobalStepInput, float directionCmd, int turnCmd) {
    if (GAIT_TYPE == 0) {
        simpleGait(GlobalStepInput, directionCmd, turnCmd);
    } else if (GAIT_TYPE == 1) {
        triangularGait(GlobalStepInput, directionCmd, turnCmd);
    }
}

// 站立并调整质心
void standMassCenter(float aInput, float bInput) {
    singleLegCtrl(1, (WALK_EXTENDED_X - aInput), STAND_HEIGHT, (WALK_EXTENDED_Z - bInput));
    singleLegCtrl(2, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, (WALK_EXTENDED_Z - bInput));
    singleLegCtrl(3, (WALK_EXTENDED_X - aInput), STAND_HEIGHT, (WALK_EXTENDED_Z + bInput));
    singleLegCtrl(4, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, (WALK_EXTENDED_Z + bInput));
}

// 控制俯仰、偏航和横滚
void pitchYawRoll(float pitchInput, float yawInput, float rollInput) {
    legPosBuffer[1] = STAND_HEIGHT + pitchInput + rollInput;
    legPosBuffer[4] = STAND_HEIGHT - pitchInput + rollInput;
    legPosBuffer[7] = STAND_HEIGHT + pitchInput - rollInput;
    legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput;

    // 限制高度
    for (int i = 1; i <= 10; i += 3) {
        if (legPosBuffer[i] > WALK_HEIGHT_MAX) legPosBuffer[i] = WALK_HEIGHT_MAX;
        if (legPosBuffer[i] < WALK_HEIGHT_MIN) legPosBuffer[i] = WALK_HEIGHT_MIN;
    }

    legPosBuffer[2] = WALK_EXTENDED_Z + yawInput - rollInput;
    legPosBuffer[5] = WALK_EXTENDED_Z - yawInput - rollInput;
    legPosBuffer[8] = WALK_EXTENDED_Z - yawInput + rollInput;
    legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;

    // 限制侧向移动
    for (int i = 2; i <= 11; i += 3) {
        if (legPosBuffer[i] > WALK_EXTENDED_Z + WALK_SIDE_MAX) legPosBuffer[i] = WALK_EXTENDED_Z + WALK_SIDE_MAX;
        if (legPosBuffer[i] < WALK_EXTENDED_Z - WALK_SIDE_MAX) legPosBuffer[i] = WALK_EXTENDED_Z - WALK_SIDE_MAX;
    }

    singleLegCtrl(1, WALK_EXTENDED_X, legPosBuffer[1], legPosBuffer[2]);
    singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4], legPosBuffer[5]);
    singleLegCtrl(3, WALK_EXTENDED_X, legPosBuffer[7], legPosBuffer[8]);
    singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
}

// 贝塞尔曲线控制
float besselCtrl(float numStart, float numEnd, float rateInput) {
    return (numEnd - numStart) * ((cos(rateInput * M_PI - M_PI) + 1) / 2) + numStart;
}

// 低姿态功能
void functionStayLow(void) {
    for (float i = 0; i <= 1; i += 0.02) {
        standUp(besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i));
        GoalPosAll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (float i = 0; i <= 1; i += 0.02) {
        standUp(besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MAX, i));
        GoalPosAll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (float i = 0; i <= 1; i += 0.02) {
        standUp(besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i));
        GoalPosAll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// 跳跃功能
void functionJump(void) {
    for (float i = 0; i <= 1; i += 0.02) {
        singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
        singleLegCtrl(2, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
        singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
        singleLegCtrl(4, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
        GoalPosAll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    singleLegCtrl(1, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
    singleLegCtrl(2, -WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
    singleLegCtrl(4, -WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
    GoalPosAll();
    vTaskDelay(pdMS_TO_TICKS(70));

    for (float i = 0; i <= 1; i += 0.02) {
        singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
        singleLegCtrl(2, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
        singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
        singleLegCtrl(4, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
        GoalPosAll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
// 测试所有舵机
void test_all_servos(void) {
    ESP_LOGI(TAG, "=== Testing all servos ===");
    
    // 测试标准角度对应的PWM值
    uint16_t test_pwm_values[] = {
        205+10,
        307, 
        410-10,  
        307
    };
    
    for (int servo = 0; servo < 12; servo++) {
        ESP_LOGI(TAG, "Testing servo %d...", servo_list[servo]);
        
        for (int i = 0; i < sizeof(test_pwm_values)/sizeof(test_pwm_values[0]); i++) {
            ESP_LOGI(TAG, "  Setting PWM = %d", test_pwm_values[i]);
            esp_err_t err = pca9685_set_pwm_value(&pca9685_dev, servo_list[servo], test_pwm_values[i]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "  Failed: %s", esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    
    ESP_LOGI(TAG, "=== Servo test complete ===");
}

// 测试单个舵机的完整范围
static void test_servo_range(uint8_t servo_id) {
    ESP_LOGI(TAG, "Testing full range for servo %d", servo_id);
    
    // 从最小到最大
    for (uint16_t pwm = 263; pwm <= 463; pwm += 5) {
        ESP_LOGI(TAG, "PWM = %d", pwm);
        pca9685_set_pwm_value(&pca9685_dev, servo_id, pwm);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 从最大到最小
    for (uint16_t pwm = 463; pwm >= 263; pwm -= 5) {
        ESP_LOGI(TAG, "PWM = %d", pwm);
        pca9685_set_pwm_value(&pca9685_dev, servo_id, pwm);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 回到中间
    pca9685_set_pwm_value(&pca9685_dev, servo_id, 307);
}

static void robotWork(void *pvParameters) {
    // 机器人工作逻辑
    while (1)
    {
        middlePosAll();
        // test_servo_range(0); 
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    
    
}

// 机器人控制主函数
static void robotCtrl(void) {
    if (!debugMode && !funcMode) {
        if (moveFB == 0 && moveLR == 0 && STAND_STILL == 0) {
            standMassCenter(0, 0);
            GoalPosAll();
            STAND_STILL = 1;
            GLOBAL_STEP = 0;
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
            ESP_LOGI(TAG, "Robot standing still");
        } else if (moveFB == 0 && moveLR == 0 && STAND_STILL == 1) {
            GoalPosAll();
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
        } else {
            STAND_STILL = 0;
            gestureUD = 0;
            gestureLR = 0;
            
            if (GLOBAL_STEP > 1) GLOBAL_STEP = 0;
            
            if (moveFB == 1 && moveLR == 0) {
                gaitTypeCtrl(GLOBAL_STEP, 0, 0);
            } else if (moveFB == -1 && moveLR == 0) {
                gaitTypeCtrl(GLOBAL_STEP, 180, 0);
            } else if (moveFB == 1 && moveLR == -1) {
                gaitTypeCtrl(GLOBAL_STEP, 30, 0);
            } else if (moveFB == 1 && moveLR == 1) {
                gaitTypeCtrl(GLOBAL_STEP, -30, 0);
            } else if (moveFB == -1 && moveLR == 1) {
                gaitTypeCtrl(GLOBAL_STEP, -120, 0);
            } else if (moveFB == -1 && moveLR == -1) {
                gaitTypeCtrl(GLOBAL_STEP, 120, 0);
            } else if (moveFB == 0 && moveLR == -1) {
                gaitTypeCtrl(GLOBAL_STEP, 0, -1);
            } else if (moveFB == 0 && moveLR == 1) {
                gaitTypeCtrl(GLOBAL_STEP, 0, 1);
            }
            
            GoalPosAll();
            GLOBAL_STEP += STEP_ITERATE;
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
        }
    } else if (!debugMode && funcMode) {
        // 功能模式处理
        switch (funcMode) {
            case 1:
                // 平衡模式（需要加速度计数据）
                // accXYZUpdate(); // 需要实现这个函数
                // balancing();
                GoalPosAll();
                break;
            case 2:
                ESP_LOGI(TAG, "stayLow");
                functionStayLow();
                funcMode = 0;
                break;
            case 4:
                ESP_LOGI(TAG, "Jump");
                functionJump();
                funcMode = 0;
                break;
            case 8:
                ESP_LOGI(TAG, "InitPos");
                initPosAll();
                break;
            case 9:
                ESP_LOGI(TAG, "MiddlePos");
                middlePosAll();
                break;
            default:
                funcMode = 0;
                break;
        }
    } else if (debugMode) {
        // 调试模式
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 主任务
static void wavego_task(void *pvParameters) {
    ESP_LOGI(TAG, "WAVEGO task started");
    
    // 初始化伺服系统
    // ServoSetup();
    
    // 初始位置
    // initPosAll();
    // test_servo_range(2); // 测试第一个舵机的完整范围
    test_all_servos();
    // 主循环
    while (1) {
       //robotCtrl();
        middlePosAll();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// 应用入口
void start_wavego(void) {
    ESP_LOGI(TAG, "WAVEGO Starting...");
    i2c_mutex = xSemaphoreCreateMutex();
    ServoSetup();
    initPosAll();
    // 创建WAVEGO任务
    xTaskCreate(wavego_task, "robotWork", 1024*2, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "WAVEGO Started");
}
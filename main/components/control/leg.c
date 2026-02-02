#include <math.h>
#include <stdio.h>
#include <string.h>

#include "leg.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/semphr.h"

#define TAG "WAVEGO"

typedef struct {
    float la_x_la;
    float lb_x_lb;
    float lw_x_lw;
    float le_x_le;
    float la_x_la_minus_lb_x_lb;
    float lb_x_lb_minus_la_x_la;
    float l_cd;
    float la_x2;
    float lb_x2;
    float e_pi;
    float ls_s2;
    float a_lcde;
    float s_ledc;
} kinematics_precalc_t;

typedef struct {
    int init_defaults[16];
    int middle_pwm[16];
    int direction[16];
    int current_pwm[16];
    float walk_lift_prop;
    float walk_acc_x2;
    float walk_h_l;
    kinematics_precalc_t kin;
    SemaphoreHandle_t i2c_mutex;
} leg_state_t;

static leg_state_t s_leg = {
    .direction = {
        -1,  1,  1,  1,
         1, -1, -1,  1,
        -1,  1,  1,  1,
         1, -1, -1,  1},
    .walk_lift_prop = 0.25f,
};

static const uint8_t k_servo_id_list[12] = {0, 1, 2, 5, 6, 7, 8, 9, 10, 13, 14, 15};
static const int k_servo_factory_defaults[16] = {
    397, 195, 145, 307,
    307, 425, 380, 200,
    390, 205, 168, 307,
    307, 408, 370, 217};

static const int k_servo_direction_defaults[16] = {
    -1,  1,  1,  1,
     1, -1, -1,  1,
    -1,  1,  1,  1,
     1, -1, -1,  1};

static const char *SERVO_CONFIG_NAMESPACE = "servo";
static const char *SERVO_CONFIG_KEY = "calib";
static nvs_handle_t servoHandle;

// PWM/angle conversion helpers (50 Hz)
static const float k_servo_period_us = 1000000.0f / SERVO_FREQ;
static const float k_servo_us_per_90deg = 500.0f; // 1.0–2.0 ms pulse span ≈ 180° → 0.5 ms per 90°
static float k_servo_ticks_per_degree = 0; // runtime init in servo_setup
static int k_servo_max_range_ticks = 0;

static inline uint16_t clamp_pwm(int value) {
    if (value < SERVOMIN) {
        return SERVOMIN;
    }
    if (value > SERVOMAX) {
        return SERVOMAX;
    }
    return (uint16_t)value;
}

static inline int pwm_ticks_from_angle(double angle_deg) {
    if (angle_deg > SERVO_RANGE) {
        angle_deg = SERVO_RANGE;
    }
    if (angle_deg < -SERVO_RANGE) {
        angle_deg = -SERVO_RANGE;
    }
    return (int)lround(angle_deg * k_servo_ticks_per_degree);
}

static inline void servo_apply_direction_defaults(void) {
    memcpy(s_leg.direction, k_servo_direction_defaults, sizeof(s_leg.direction));
}

static void servo_compute_neutral_pwm(void) {
    for (int i = 0; i < 16; i++) {
        s_leg.middle_pwm[i] = clamp_pwm(s_leg.init_defaults[i]);
    }
}

static void servo_load_factory_defaults(void) {
    memcpy(s_leg.init_defaults, k_servo_factory_defaults, sizeof(s_leg.init_defaults));
    servo_apply_direction_defaults();
    servo_compute_neutral_pwm();
}

static void servo_apply_loaded_defaults(const int *loaded, size_t count) {
    size_t copy = count < 16 ? count : 16;
    for (size_t i = 0; i < copy; i++) {
        s_leg.init_defaults[i] = clamp_pwm(loaded[i]);
    }
    for (size_t i = copy; i < 16; i++) {
        s_leg.init_defaults[i] = k_servo_factory_defaults[i];
    }
    servo_apply_direction_defaults();
    servo_compute_neutral_pwm();
}

static int clamp_servo_range_internal(uint8_t servo_id, int pwm_value) {
    if (servo_id >= 16) {
        return 0;
    }
    int min_pwm = s_leg.middle_pwm[servo_id] - k_servo_max_range_ticks;
    int max_pwm = s_leg.middle_pwm[servo_id] + k_servo_max_range_ticks;
    if (pwm_value < min_pwm) {
        pwm_value = min_pwm;
    }
    if (pwm_value > max_pwm) {
        pwm_value = max_pwm;
    }
    return clamp_pwm(pwm_value);
}

static void write_servo_pwm(uint8_t servo_id, int pwm_value) {
    if (servo_id >= 16) {
        return;
    }
    s_leg.current_pwm[servo_id] = pwm_value;

    if (s_leg.i2c_mutex && xSemaphoreTake(s_leg.i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        pca9685_set_pwm_value(servo_id, (uint16_t)pwm_value);
        xSemaphoreGive(s_leg.i2c_mutex);
    } else {
        pca9685_set_pwm_value(servo_id, (uint16_t)pwm_value);
    }
}

void set_servo(uint8_t servo_id, int offset) {
    if (servo_id >= 16) {
        return;
    }
    int target_pwm = s_leg.middle_pwm[servo_id] + offset;
    target_pwm = clamp_servo_range_internal(servo_id, target_pwm);
    write_servo_pwm(servo_id, target_pwm);
}

void servo_set_default_pwm(uint8_t servo_id, int pwm_value) {
    if (servo_id >= 16) {
        return;
    }
    s_leg.init_defaults[servo_id] = clamp_pwm(pwm_value);
    servo_compute_neutral_pwm();
}

int leg_get_servo_middle_pwm(uint8_t servo_id) {
    return servo_id < 16 ? s_leg.middle_pwm[servo_id] : 0;
}

int leg_get_servo_direction(uint8_t servo_id) {
    return servo_id < 16 ? s_leg.direction[servo_id] : 0;
}

int leg_get_current_pwm(uint8_t servo_id) {
    return servo_id < 16 ? s_leg.current_pwm[servo_id] : 0;
}

void leg_get_servo_snapshot(int middle_out[16], int direction_out[16]) {
    if (middle_out) {
        memcpy(middle_out, s_leg.middle_pwm, sizeof(s_leg.middle_pwm));
    }
    if (direction_out) {
        memcpy(direction_out, s_leg.direction, sizeof(s_leg.direction));
    }
}

int leg_clamp_servo_pwm(uint8_t servo_id, int pwm_value) {
    return clamp_servo_range_internal(servo_id, pwm_value);
}

esp_err_t leg_set_servo_pwm(uint8_t servo_id, int pwm_value) {
    if (servo_id >= 16) {
        return ESP_ERR_INVALID_ARG;
    }
    int clamped = clamp_servo_range_internal(servo_id, pwm_value);
    write_servo_pwm(servo_id, clamped);
    return ESP_OK;
}

void servo_setup(void) {
    k_servo_ticks_per_degree = (PCA9685_MAX_PWM_VALUE * (k_servo_us_per_90deg / 90.0f)) / k_servo_period_us;
    k_servo_max_range_ticks = (int)lroundf(k_servo_ticks_per_degree * SERVO_RANGE);

    kinematics_precalc_t *k = &s_leg.kin;
    k->la_x_la = LINKAGE_A * LINKAGE_A;
    k->lb_x_lb = LINKAGE_B * LINKAGE_B;
    k->lw_x_lw = LINKAGE_W * LINKAGE_W;
    k->le_x_le = LINKAGE_E * LINKAGE_E;
    k->la_x_la_minus_lb_x_lb = k->la_x_la - k->lb_x_lb;
    k->lb_x_lb_minus_la_x_la = k->lb_x_lb - k->la_x_la;
    k->l_cd = (LINKAGE_C + LINKAGE_D) * (LINKAGE_C + LINKAGE_D);
    k->la_x2 = 2 * LINKAGE_A;
    k->lb_x2 = 2 * LINKAGE_B;
    k->e_pi = 180 / M_PI;
    k->ls_s2 = LINKAGE_S / 2;
    k->a_lcde = atan((LINKAGE_C + LINKAGE_D) / LINKAGE_E);
    k->s_ledc = sqrt(LINKAGE_E * LINKAGE_E + (LINKAGE_D + LINKAGE_C) * (LINKAGE_D + LINKAGE_C));

    s_leg.walk_acc_x2 = WALK_ACC * 2;
    s_leg.walk_h_l = WALK_HEIGHT - WALK_LIFT;

    ESP_LOGI(TAG, "Servo setup complete");
}

void middle_pos_all(void) {
    for (int i = 0; i < 16; i++) {
        double offset_deg = (SERVO_RANGE / 2.0) * s_leg.direction[i];
        int offset_ticks = pwm_ticks_from_angle(offset_deg);
        s_leg.current_pwm[i] = s_leg.middle_pwm[i] + offset_ticks;
        int target_pwm = clamp_servo_range_internal(i, s_leg.current_pwm[i] + offset_ticks);
        write_servo_pwm(i, target_pwm);
        vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_EVERY));
    }
}

void init_pos_all(void) {
    for (int i = 0; i < 16; i++) {
        s_leg.current_pwm[i] = s_leg.init_defaults[i];
        write_servo_pwm(i, s_leg.current_pwm[i]);
        // ESP_LOGD(TAG, "Servo %d: %d", i, s_leg.current_pwm[i]);
        vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_EVERY));
    }
}

void goal_pos_all(void) {
    for (int i = 0; i < 16; i++) {
        write_servo_pwm(i, s_leg.current_pwm[i]);
        vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_EVERY));
    }
}

void goal_pwm_set(uint8_t servo_num, double angle_input) {
    if (servo_num >= 16) {
        return;
    }
    int pwm_delta = pwm_ticks_from_angle(angle_input * s_leg.direction[servo_num]);
    int target_pwm = s_leg.middle_pwm[servo_num] + pwm_delta;
    s_leg.current_pwm[servo_num] = clamp_servo_range_internal(servo_num, target_pwm);
}

// 逆运动学与步态相关函数保持为内部实现
static void simple_linkage_ik(double la, double lb, double a_in, double b_in,
                              double *alpha, double *beta) {
    double psi;
    double omega;
    double L2C;
    double LC;
    double lambda;

    if (b_in == 0) {
        psi = acos((s_leg.kin.la_x_la_minus_lb_x_lb + a_in * a_in) / (s_leg.kin.la_x2 * a_in)) * s_leg.kin.e_pi;
        *alpha = 90 - psi;
        omega = acos((a_in * a_in + s_leg.kin.lb_x_lb_minus_la_x_la) / (s_leg.kin.lb_x2 * a_in)) * s_leg.kin.e_pi;
        *beta = psi + omega;
    } else {
        L2C = a_in * a_in + b_in * b_in;
        LC = sqrt(L2C);
        lambda = atan(b_in / a_in) * s_leg.kin.e_pi;
        psi = acos((s_leg.kin.la_x_la_minus_lb_x_lb + L2C) / (2 * la * LC)) * s_leg.kin.e_pi;
        *alpha = 90 - lambda - psi;
        omega = acos((s_leg.kin.lb_x_lb_minus_la_x_la + L2C) / (2 * LC * lb)) * s_leg.kin.e_pi;
        *beta = psi + omega;
    }
}

static void wiggle_plane_ik(double la, double a_in, double b_in, double *alpha, double *len_out) {
    double LB = 0;
    double L2C = 0;
    double LC = 0;
    double lambda = 0;
    double psi = 0;

    if (b_in > 0) {
        L2C = a_in * a_in + b_in * b_in;
        LC = sqrt(L2C);
        lambda = atan(a_in / b_in) * s_leg.kin.e_pi;
        psi = acos(la / LC) * s_leg.kin.e_pi;
        LB = sqrt(L2C - s_leg.kin.lw_x_lw);
        *alpha = psi + lambda - 90;
    } else if (b_in == 0) {
        *alpha = asin(la / a_in) * s_leg.kin.e_pi;
        L2C = a_in * a_in + b_in * b_in;
        LB = sqrt(L2C);
    } else {
        b_in = -b_in;
        L2C = a_in * a_in + b_in * b_in;
        LC = sqrt(L2C);
        lambda = atan(a_in / b_in) * s_leg.kin.e_pi;
        psi = acos(la / LC) * s_leg.kin.e_pi;
        LB = sqrt(L2C - s_leg.kin.lw_x_lw);
        *alpha = 90 - lambda + psi;
    }

    *len_out = LB - WIGGLE_ERROR;
}

static void single_leg_plane_ik(double ls, double la, double lc, double ld, double le,
                                double x_in, double y_in, double *beta_out, double *x_out, double *y_out) {
    double bufferS = sqrt((x_in + s_leg.kin.ls_s2) * (x_in + s_leg.kin.ls_s2) + y_in * y_in);
    double lambda = acos(((x_in + s_leg.kin.ls_s2) * (x_in + s_leg.kin.ls_s2) + y_in * y_in + s_leg.kin.la_x_la - s_leg.kin.l_cd - s_leg.kin.le_x_le) / (2 * bufferS * la));
    double delta = atan((x_in + s_leg.kin.ls_s2) / y_in);
    double beta = lambda - delta;
    double betaAngle = beta * s_leg.kin.e_pi;

    double theta = s_leg.kin.a_lcde;
    double omega = asin((y_in - cos(beta) * la) / s_leg.kin.s_ledc);
    double nu = M_PI - theta - omega;
    double dFX = cos(nu) * le;
    double dFY = sin(nu) * le;

    double mu = M_PI / 2 - nu;
    double dEX = cos(mu) * ld;
    double dEY = sin(mu) * ld;

    *x_out = x_in + dFX - dEX;
    *y_out = y_in - dFY - dEY;
    *beta_out = betaAngle;
}

static void single_leg_ctrl(uint8_t leg_num, double x_pos, double y_pos, double z_pos) {
    uint8_t NumF, NumB, NumW;

    switch (leg_num) {
        case 1:
            NumF = LEG_A_FORE; NumB = LEG_A_BACK; NumW = LEG_A_WAVE; break;
        case 2:
            NumF = LEG_B_FORE; NumB = LEG_B_BACK; NumW = LEG_B_WAVE; break;
        case 3:
            NumF = LEG_C_FORE; NumB = LEG_C_BACK; NumW = LEG_C_WAVE; break;
        case 4:
            NumF = LEG_D_FORE; NumB = LEG_D_BACK; NumW = LEG_D_WAVE; break;
        default:
            return;
    }

    double wiggle_alpha = 0;
    double wiggle_len = 0;
    double hip_beta = 0;
    double foot_x = 0;
    double foot_y = 0;
    double link_alpha = 0;
    double link_beta = 0;

    wiggle_plane_ik(LINKAGE_W, z_pos, y_pos, &wiggle_alpha, &wiggle_len);
    single_leg_plane_ik(LINKAGE_S, LINKAGE_A, LINKAGE_C, LINKAGE_D, LINKAGE_E,
                        x_pos, wiggle_len, &hip_beta, &foot_x, &foot_y);
    (void)hip_beta;
    simple_linkage_ik(LINKAGE_B, LINKAGE_B, foot_y, (foot_x - s_leg.kin.ls_s2), &link_alpha, &link_beta);

    goal_pwm_set(NumW, wiggle_alpha);
    goal_pwm_set(NumF, (90 - link_beta));
    goal_pwm_set(NumB, link_alpha);
}

static void stand_up(double cmd_input) {
    single_leg_ctrl(1, WALK_EXTENDED_X, cmd_input, WALK_EXTENDED_Z);
    single_leg_ctrl(2, -WALK_EXTENDED_X, cmd_input, WALK_EXTENDED_Z);
    single_leg_ctrl(3, WALK_EXTENDED_X, cmd_input, WALK_EXTENDED_Z);
    single_leg_ctrl(4, -WALK_EXTENDED_X, cmd_input, WALK_EXTENDED_Z);
}

static void single_gait_ctrl(uint8_t leg_num, uint8_t status_input, float cycle_input,
                             float direction_input, double extended_x, double extended_z) {
    double rDist = 0;
    double xGait = 0;
    double yGait = 0;
    double zGait = 0;
    double rDirection = direction_input * M_PI / 180;

    if (cycle_input < (1 - s_leg.walk_lift_prop)) {
        if (cycle_input <= (WALK_ACC / (s_leg.walk_acc_x2 + WALK_RANGE * status_input)) * (1 - s_leg.walk_lift_prop)) {
            yGait = s_leg.walk_h_l + cycle_input / (1 - s_leg.walk_lift_prop -
                    ((WALK_ACC + WALK_RANGE * status_input) / (s_leg.walk_acc_x2 + WALK_RANGE * status_input)) *
                    (1 - s_leg.walk_lift_prop)) * WALK_LIFT;
        } else if (cycle_input > (WALK_ACC / (s_leg.walk_acc_x2 + WALK_RANGE * status_input)) * (1 - s_leg.walk_lift_prop) &&
                   cycle_input <= ((WALK_ACC + WALK_RANGE * status_input) / (s_leg.walk_acc_x2 + WALK_RANGE * status_input)) * (1 - s_leg.walk_lift_prop)) {
            yGait = WALK_HEIGHT;
        } else if (cycle_input > ((WALK_ACC + WALK_RANGE * status_input) / (s_leg.walk_acc_x2 + WALK_RANGE * status_input)) * (1 - s_leg.walk_lift_prop) &&
                   cycle_input < ((s_leg.walk_acc_x2 + WALK_RANGE * status_input) / (s_leg.walk_acc_x2 + WALK_RANGE * status_input)) * (1 - s_leg.walk_lift_prop)) {
            yGait = WALK_HEIGHT - ((cycle_input - ((WALK_ACC + WALK_RANGE * status_input) / (s_leg.walk_acc_x2 + WALK_RANGE * status_input)) *
                    (1 - s_leg.walk_lift_prop)) / ((WALK_ACC / (s_leg.walk_acc_x2 + WALK_RANGE * status_input)) * (1 - s_leg.walk_lift_prop))) * WALK_LIFT;
        }
        rDist = (WALK_RANGE * status_input / 2 + WALK_ACC) -
                (cycle_input / (1 - s_leg.walk_lift_prop)) * (WALK_RANGE * status_input + s_leg.walk_acc_x2);
    } else {
        yGait = s_leg.walk_h_l;
        rDist = -(WALK_RANGE * status_input / 2 + WALK_ACC) +
                ((cycle_input - (1 - s_leg.walk_lift_prop)) / s_leg.walk_lift_prop) *
                (WALK_RANGE * status_input + s_leg.walk_acc_x2);
    }

    xGait = cos(rDirection) * rDist;
    zGait = sin(rDirection) * rDist;

    single_leg_ctrl(leg_num, (xGait + extended_x), yGait, (zGait + extended_z));
}

static void simple_gait(float global_input, float direction_angle, int turn_cmd) {
    float Group_A = global_input;
    float Group_B = global_input + 0.5f;
    if (Group_B > 1) Group_B--;

    if (!turn_cmd) {
        single_gait_ctrl(1, 1, Group_A, direction_angle, WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(4, 1, Group_A, -direction_angle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(2, 1, Group_B, direction_angle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(3, 1, Group_B, -direction_angle, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    } else if (turn_cmd == -1) {
        single_gait_ctrl(1, 1.5f, Group_A, 90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(4, 1.5f, Group_A, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(2, 1.5f, Group_B, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(3, 1.5f, Group_B, -90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    } else if (turn_cmd == 1) {
        single_gait_ctrl(1, 1.5f, Group_A, -90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(4, 1.5f, Group_A, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(2, 1.5f, Group_B, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
        single_gait_ctrl(3, 1.5f, Group_B, 90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
    }
}

static void triangular_gait(float global_input, float direction_angle, int turn_cmd) {
    float StepA, StepB, StepC, StepD;
    float aInput = 0, bInput = 0;

    StepB = global_input;
    StepC = global_input + 0.25f;
    StepD = global_input + 0.5f;
    StepA = global_input + 0.75f;

    if (StepA > 1) StepA--;
    if (StepB > 1) StepB--;
    if (StepC > 1) StepC--;
    if (StepD > 1) StepD--;

    if (global_input <= 0.25f) {
        aInput = WALK_MASS_ADJUST - (global_input / 0.125f) * WALK_MASS_ADJUST;
        bInput = -WALK_MASS_ADJUST;
    } else if (global_input > 0.25f && global_input <= 0.5f) {
        float adProp = global_input - 0.25f;
        aInput = -WALK_MASS_ADJUST + (adProp / 0.125f) * WALK_MASS_ADJUST;
        bInput = -WALK_MASS_ADJUST + (adProp / 0.125f) * WALK_MASS_ADJUST;
    } else if (global_input > 0.5f && global_input <= 0.75f) {
        float adProp = global_input - 0.5f;
        aInput = WALK_MASS_ADJUST - (adProp / 0.125f) * WALK_MASS_ADJUST;
        bInput = WALK_MASS_ADJUST;
    } else {
        float adProp = global_input - 0.75f;
        aInput = -WALK_MASS_ADJUST + (adProp / 0.125f) * WALK_MASS_ADJUST;
        bInput = WALK_MASS_ADJUST - (adProp / 0.125f) * WALK_MASS_ADJUST;
    }

    if (!turn_cmd) {
        single_gait_ctrl(1, 1, StepA, direction_angle, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        single_gait_ctrl(4, 1, StepD, -direction_angle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
        single_gait_ctrl(2, 1, StepB, direction_angle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        single_gait_ctrl(3, 1, StepC, -direction_angle, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    } else if (turn_cmd == -1) {
        single_gait_ctrl(1, 1.5f, StepA, 90, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        single_gait_ctrl(4, 1.5f, StepD, 90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
        single_gait_ctrl(2, 1.5f, StepB, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        single_gait_ctrl(3, 1.5f, StepC, -90, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    } else {
        single_gait_ctrl(1, 1.5f, StepA, -90, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        single_gait_ctrl(4, 1.5f, StepD, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
        single_gait_ctrl(2, 1.5f, StepB, 90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
        single_gait_ctrl(3, 1.5f, StepC, 90, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
    }
}

static void pitch_yaw_roll(float pitch_input, float yaw_input, float roll_input) {
    int leg_pos_buffer[12] = {
        WALK_EXTENDED_X, STAND_HEIGHT, WALK_EXTENDED_Z,
        -WALK_EXTENDED_X, STAND_HEIGHT, WALK_EXTENDED_Z,
         WALK_EXTENDED_X, STAND_HEIGHT, WALK_EXTENDED_Z,
        -WALK_EXTENDED_X, STAND_HEIGHT, WALK_EXTENDED_Z};

    leg_pos_buffer[1] += pitch_input + roll_input;
    leg_pos_buffer[4] += -pitch_input + roll_input;
    leg_pos_buffer[7] += pitch_input - roll_input;
    leg_pos_buffer[10] += -pitch_input - roll_input;

    for (int i = 1; i <= 10; i += 3) {
        if (leg_pos_buffer[i] > WALK_HEIGHT_MAX) leg_pos_buffer[i] = WALK_HEIGHT_MAX;
        if (leg_pos_buffer[i] < WALK_HEIGHT_MIN) leg_pos_buffer[i] = WALK_HEIGHT_MIN;
    }

    leg_pos_buffer[2] += yaw_input - roll_input;
    leg_pos_buffer[5] += -yaw_input - roll_input;
    leg_pos_buffer[8] += -yaw_input + roll_input;
    leg_pos_buffer[11]+= yaw_input + roll_input;

    for (int i = 2; i <= 11; i += 3) {
        if (leg_pos_buffer[i] > WALK_EXTENDED_Z + WALK_SIDE_MAX) leg_pos_buffer[i] = WALK_EXTENDED_Z + WALK_SIDE_MAX;
        if (leg_pos_buffer[i] < WALK_EXTENDED_Z - WALK_SIDE_MAX) leg_pos_buffer[i] = WALK_EXTENDED_Z - WALK_SIDE_MAX;
    }

    single_leg_ctrl(1, WALK_EXTENDED_X, leg_pos_buffer[1], leg_pos_buffer[2]);
    single_leg_ctrl(2, -WALK_EXTENDED_X, leg_pos_buffer[4], leg_pos_buffer[5]);
    single_leg_ctrl(3, WALK_EXTENDED_X, leg_pos_buffer[7], leg_pos_buffer[8]);
    single_leg_ctrl(4, -WALK_EXTENDED_X, leg_pos_buffer[10], leg_pos_buffer[11]);
}

static float bessel_ctrl(float num_start, float num_end, float rate_input) {
    return (num_end - num_start) * ((cos(rate_input * M_PI - M_PI) + 1) / 2) + num_start;
}

static void function_stay_low(void) {
    for (float i = 0; i <= 1; i += 0.02f) {
        stand_up(bessel_ctrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i));
        goal_pos_all();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    for (float i = 0; i <= 1; i += 0.02f) {
        stand_up(bessel_ctrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MAX, i));
        goal_pos_all();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (float i = 0; i <= 1; i += 0.02f) {
        stand_up(bessel_ctrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i));
        goal_pos_all();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void function_jump(void) {
    for (float i = 0; i <= 1; i += 0.02f) {
        single_leg_ctrl(1, WALK_EXTENDED_X, bessel_ctrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
        single_leg_ctrl(2, -WALK_EXTENDED_X, bessel_ctrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
        single_leg_ctrl(3, WALK_EXTENDED_X, bessel_ctrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
        single_leg_ctrl(4, -WALK_EXTENDED_X, bessel_ctrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
        goal_pos_all();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    single_leg_ctrl(1, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
    single_leg_ctrl(2, -WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
    single_leg_ctrl(3, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
    single_leg_ctrl(4, -WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
    goal_pos_all();
    vTaskDelay(pdMS_TO_TICKS(70));

    for (float i = 0; i <= 1; i += 0.02f) {
        single_leg_ctrl(1, WALK_EXTENDED_X, bessel_ctrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
        single_leg_ctrl(2, -WALK_EXTENDED_X, bessel_ctrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
        single_leg_ctrl(3, WALK_EXTENDED_X, bessel_ctrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
        single_leg_ctrl(4, -WALK_EXTENDED_X, bessel_ctrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
        goal_pos_all();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void test_all_servos(void) {
    ESP_LOGI(TAG, "=== Testing all servos ===");

    uint16_t test_pwm_values[] = {205 + 10, 307, 410 - 10, 307};

    for (int servo = 0; servo < 12; servo++) {
        ESP_LOGI(TAG, "Testing servo %d...", k_servo_id_list[servo]);

        for (size_t i = 0; i < sizeof(test_pwm_values) / sizeof(test_pwm_values[0]); i++) {
            ESP_LOGI(TAG, "  Setting PWM = %d", test_pwm_values[i]);
            esp_err_t err = pca9685_set_pwm_value(k_servo_id_list[servo], test_pwm_values[i]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "  Failed: %s", esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }

    ESP_LOGI(TAG, "=== Servo test complete ===");
}

static void wavego_task(void *pvParameters) {
    ESP_LOGI(TAG, "WAVEGO task started");
    // test_all_servos(); // 可按需启用

    while (1) {
        init_pos_all();
        vTaskDelay(pdMS_TO_TICKS(3000));
        middle_pos_all();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

esp_err_t servo_config_init(void) {
    if (servoHandle) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS partition for servo config");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_open(SERVO_CONFIG_NAMESPACE, NVS_READWRITE, &servoHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", SERVO_CONFIG_NAMESPACE, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t servo_config_reset_defaults(void) {
    servo_load_factory_defaults();

    esp_err_t err = servo_config_init();
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(servoHandle, SERVO_CONFIG_KEY, s_leg.init_defaults, sizeof(s_leg.init_defaults));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save factory defaults: %s", esp_err_to_name(err));
        return err;
    }
    return nvs_commit(servoHandle);
}

esp_err_t servo_config_save_to_nvs(void) {
    esp_err_t err = servo_config_init();
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(servoHandle, SERVO_CONFIG_KEY, s_leg.init_defaults, sizeof(s_leg.init_defaults));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write servo calibration: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_commit(servoHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit servo calibration: %s", esp_err_to_name(err));
        return err;
    }
    servo_compute_neutral_pwm();
    return ESP_OK;
}

esp_err_t servo_config_load_from_nvs(void) {
#if !LEG_USE_NVS_CALIB
    // 编译期开关：直接使用工厂默认，不读取 NVS
    return ESP_OK;
#else
    servo_load_factory_defaults();

    esp_err_t err = servo_config_init();
    if (err != ESP_OK) {
        return err;
    }

    int loaded[16] = {0};
    size_t required_size = sizeof(loaded);
    err = nvs_get_blob(servoHandle, SERVO_CONFIG_KEY, loaded, &required_size);

    if (err == ESP_ERR_NVS_NOT_FOUND || required_size != sizeof(loaded)) {
        ESP_LOGW(TAG, "Servo calibration not found, using factory defaults");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read servo calibration: %s", esp_err_to_name(err));
        return err;
    }

    servo_apply_loaded_defaults(loaded, required_size / sizeof(int));
    return ESP_OK;
#endif
}

void start_wavego(void) {
    ESP_LOGI(TAG, "WAVEGO Starting...");
    // Always seed with factory defaults, optionally overridden by NVS
    servo_load_factory_defaults();
    if (servo_config_load_from_nvs() != ESP_OK) {
        ESP_LOGW(TAG, "Using default servo calibration");
    }
    memcpy(s_leg.current_pwm, s_leg.middle_pwm, sizeof(s_leg.current_pwm));

    s_leg.i2c_mutex = xSemaphoreCreateMutex();
    servo_setup();
    init_pos_all();

    xTaskCreate(wavego_task, "robotWork", 1024 * 2, NULL, 3, NULL);

    ESP_LOGI(TAG, "WAVEGO Started");
}
#include <math.h>
#include <stdbool.h>

#include "leg.h"
#include "board.h"
#include "calibration.h"
#include "pca9685.h"

#ifndef CONFIG_CONTROL_SERVO_FREQ_HZ
#define CONFIG_CONTROL_SERVO_FREQ_HZ 50
#endif

#define TAG "WAVEGO"

#ifndef WAVEGO_PWM_DEBUG
#define WAVEGO_PWM_DEBUG 0
#endif

typedef struct {
    uint8_t fore;
    uint8_t back;
    uint8_t wave;
} leg_servo_map_t;

typedef struct {
    int wave_height;
    int wave_speed;
    ActionState state;
} wavego_command_t;

static const int k_servo_direction_defaults[16] = {
    -1,  1,  1,  1,
     1, -1, -1,  1,
    -1,  1,  1,  1,
     1, -1, -1,  1};

static const leg_servo_map_t s_leg_map[4] = {
    {LEG_A_FORE, LEG_A_BACK, LEG_A_WAVE},
    {LEG_B_FORE, LEG_B_BACK, LEG_B_WAVE},
    {LEG_C_FORE, LEG_C_BACK, LEG_C_WAVE},
    {LEG_D_FORE, LEG_D_BACK, LEG_D_WAVE},
};

static uint16_t s_servo_base_pwm[16] = {0};
static bool s_servo_base_ready = false;
static float s_servo_stand_angle[16] = {0};
static float s_servo_current_angle[16] = {0};

static TaskHandle_t s_wavego_task_handle = NULL;
static QueueHandle_t s_wavego_cmd_queue = NULL;
static wavego_command_t s_active_cmd = {
    .wave_height = INIT_HEIGHT_ANGLE,
    .wave_speed = -1,
    .state = INITIALIZING,
};

// 前置加载舵机初始pwm至s_servo_base_pwm
static void load_servo_bases(void) {
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t mid = 0;
        calibration_get_middle_pwm(i, &mid);
        s_servo_base_pwm[i] = mid;
        s_servo_current_angle[i] = 0.0f;
#if WAVEGO_PWM_DEBUG
        ESP_LOGI(TAG, "Base servo %u mid %u angle %.1f", (unsigned)i, (unsigned)mid, pwm_to_angle(mid));
#endif
    }
    s_servo_base_ready = true;
}
// 辅助函数：检查高度角范围是否合理
static float clamp_height_angle(float height_angle) {
    // Special case: allow 0 as "no stand-height offset" for DEBUG/manual modes.
    if (height_angle <= 0.0f) {
        return 0.0f;
    }
    if(height_angle < WALK_HEIGHT_MIN_ANGLE) {
        height_angle = WALK_HEIGHT_MIN_ANGLE;
    }else if(height_angle > WALK_HEIGHT_MAX_ANGLE) {
        height_angle = WALK_HEIGHT_MAX_ANGLE;
    }
    return height_angle;
}
// 以维护全局变量s_servo_stand_angle的方式控制腿部高度
static void control_leg_height(float height_angle) {
    height_angle = clamp_height_angle(height_angle);
    for(int leg_idx = 0; leg_idx < 4; leg_idx++) {
        leg_servo_map_t leg = s_leg_map[leg_idx];
        s_servo_stand_angle[leg.fore] = height_angle;
        s_servo_stand_angle[leg.back] = height_angle;
    }
}
// 辅助函数：pwm与角度转换
static float pwm_to_angle(uint16_t pwm){
    return (float)(pwm*1000000/4096/50-500)/2000*180;
}
// 辅助函数：角度与pwm转换
static uint16_t angle_to_pwm(float angle){
    return (500+(angle/180)*2000)*50*4096/1000000;
}
// 驱动单个舵机到指定相对站立角度
void drive_servo_to_angle(uint8_t id, float angle){
    uint16_t pwm = s_servo_base_pwm[id];
    float target_angle = pwm_to_angle(pwm);
    if(id != LEG_A_WAVE && id != LEG_D_WAVE && id != LEG_B_WAVE && id != LEG_C_WAVE){
        target_angle += k_servo_direction_defaults[id]*(angle+s_servo_stand_angle[id]);
    }else{
        target_angle += k_servo_direction_defaults[id]*(angle);
    }
    pwm = angle_to_pwm(target_angle);
    pca9685_set_pwm_value(id, pwm);
    s_servo_current_angle[id] = angle;
}
// 应用站立姿态
static void apply_stand_pose(const wavego_command_t *cmd) {
    float height_angle = (float)(cmd->wave_height);
    control_leg_height(height_angle);
    for (int servo_idx = 0; servo_idx < 16; servo_idx++) {
        drive_servo_to_angle(servo_idx, 0.0f);
    }
}
// 应用行走姿态
static void apply_walk_pose(const wavego_command_t *cmd, float phase, int direction) {
    control_leg_height((float)(cmd->wave_height));
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        leg_servo_map_t leg = s_leg_map[leg_idx];
        float leg_phase = phase + (float)(M_PI_2 * leg_idx * direction);
        float lift = sinf(leg_phase);
        float fore_back_offset = cosf(leg_phase);

        // Leg lift
        // float lift_angle = 0.0f;
        // if (lift > 0.0f) {
        //     lift_angle = lift * 30.0f; // Max lift angle
        // }
        drive_servo_to_angle(leg.wave, DEBUG_HEIGHT_ANGLE);

        // Fore/back swing
        float swing_angle = fore_back_offset * 20.0f; // Max swing angle
        drive_servo_to_angle(leg.fore, swing_angle);
        drive_servo_to_angle(leg.back, -swing_angle);
    }
    ESP_LOGI(TAG, "Applied walk pose with height %d, phase %.2f", cmd->wave_height, phase);
}
static void apply_turn_pose(const wavego_command_t *cmd, float phase, int direction) {
    control_leg_height((float)(cmd->wave_height));
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        leg_servo_map_t leg = s_leg_map[leg_idx];
        float leg_phase = phase + (float)(M_PI_2 * leg_idx * direction);
        float lift = sinf(leg_phase);
        float side_offset = cosf(leg_phase);

        // Leg lift
        float lift_angle = 0.0f;
        if (lift > 0.0f) {
            lift_angle = lift * 30.0f; // Max lift angle
        }
        drive_servo_to_angle(leg.wave, lift_angle);

        // Side swing
        float swing_angle = side_offset * 15.0f; // Max side swing angle
        drive_servo_to_angle(leg.fore, swing_angle);
        drive_servo_to_angle(leg.back, swing_angle);
    }
    ESP_LOGI(TAG, "Applied turn pose with height %d, phase %.2f", cmd->wave_height, phase);
}

static void apply_debug_pose(const wavego_command_t *cmd) {
    control_leg_height((float)(DEBUG_HEIGHT_ANGLE));
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        drive_servo_to_angle(s_leg_map[leg_idx].wave, 0.0f);
        drive_servo_to_angle(s_leg_map[leg_idx].fore, 0.0f);
        drive_servo_to_angle(s_leg_map[leg_idx].back, 0.0f);
    }
}
static void apply_init_pose(const wavego_command_t *cmd) {
    control_leg_height((float)(INIT_HEIGHT_ANGLE));
    for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
        drive_servo_to_angle(s_leg_map[leg_idx].wave, 90.0f);
        drive_servo_to_angle(s_leg_map[leg_idx].fore, 0);
        drive_servo_to_angle(s_leg_map[leg_idx].back, 0);
    }
}

static void build_target_angles(ActionState state, const wavego_command_t *cmd, float target[16]) {
    for (int i = 0; i < 16; i++) {
        target[i] = 0.0f;
    }

    if (state == STANDING) {
        control_leg_height((float)(cmd->wave_height));
        return;
    }

    if (state == DEBUG) {
        control_leg_height((float)(DEBUG_HEIGHT_ANGLE));
        return;
    }

    if (state == INITIALIZING) {
        control_leg_height((float)(INIT_HEIGHT_ANGLE));
        for (int leg_idx = 0; leg_idx < 4; leg_idx++) {
            target[s_leg_map[leg_idx].wave] = 90.0f;
        }
    }
}

static void smooth_move_to_pose(ActionState state, const wavego_command_t *cmd, TickType_t total_period) {
    float target[16];
    build_target_angles(state, cmd, target);

    const int steps = 10;
    TickType_t step_delay = pdMS_TO_TICKS(1);
    if (total_period > 0) {
        step_delay = total_period / steps;
        if (step_delay == 0) {
            step_delay = pdMS_TO_TICKS(1);
        }
    }

    for (int step = 1; step <= steps; step++) {
        float t = (float)step / (float)steps;
        for (int i = 0; i < 16; i++) {
            float angle = s_servo_current_angle[i] + (target[i] - s_servo_current_angle[i]) * t;
            drive_servo_to_angle(i, angle);
        }
        vTaskDelay(step_delay);
    }
}

static void wavego_task(void *pvParameters) {
    ESP_LOGI(TAG, "wavego task started (100 Hz)");
    TickType_t last_wake = xTaskGetTickCount();
    const static TickType_t period = pdMS_TO_TICKS(10);
    float phase = 0.0f;
    ActionState last_state = s_active_cmd.state;

    while (1) {
        static wavego_command_t incoming;
        static TickType_t changeable_period = pdMS_TO_TICKS(500);
        while(xQueueReceive(s_wavego_cmd_queue, &incoming, 0) == pdPASS) {
            s_active_cmd = incoming;
            if(s_active_cmd.state > STANDING){
                changeable_period = period;   
            }else{
                changeable_period = pdMS_TO_TICKS(500);
            }
        }

        float freq_hz = ((float)s_active_cmd.wave_speed / 50.0f);
        phase += (float)(2.0f * M_PI * freq_hz * 0.01f);
        if (phase > (float)(2.0f * M_PI)) {
            phase -= (float)(2.0f * M_PI);
        }
        switch (s_active_cmd.state) {
            case DEBUG:
                control_leg_height(0.0f);
                break;
            case INITIALIZING:
                if (last_state != s_active_cmd.state) {
                    smooth_move_to_pose(s_active_cmd.state, &s_active_cmd, changeable_period);
                } else {
                    apply_init_pose(&s_active_cmd);
                }
                break;
            case STANDING:
                if (last_state != s_active_cmd.state) {
                    smooth_move_to_pose(s_active_cmd.state, &s_active_cmd, changeable_period);
                } else {
                    apply_stand_pose(&s_active_cmd);
                }
                break;
            case WAVING:
                // apply_wave_pose(&s_active_cmd, phase);
                break;
            case WALKING_FORWARD:
                apply_walk_pose(&s_active_cmd, phase, 1);
                break;
            case WALKING_BACKWARD:
                apply_walk_pose(&s_active_cmd, phase, -1);
                break;
            case TURNING_LEFT:
                apply_turn_pose(&s_active_cmd, phase, -1);
                break;
            case TURNING_RIGHT:
                apply_turn_pose(&s_active_cmd, phase, 1);
                break;
            default:
                apply_stand_pose(&s_active_cmd);
                break;
        }

        last_state = s_active_cmd.state;
        ESP_LOGI(TAG, "State: %d, Height: %d, Speed: %d", s_active_cmd.state, s_active_cmd.wave_height, s_active_cmd.wave_speed);
        vTaskDelayUntil(&last_wake, changeable_period);
    }
}

esp_err_t start_wavego_task(void) {
    if (s_wavego_task_handle) {
        return ESP_OK;
    }

    if (!s_wavego_cmd_queue) {
        s_wavego_cmd_queue = xQueueCreate(1, sizeof(wavego_command_t));
        if (!s_wavego_cmd_queue) {
            return ESP_ERR_NO_MEM;
        }
    }

    load_servo_bases();
    bsp_icm20948_set_rate(LEG_ICM_READ_HZ);


    BaseType_t created = xTaskCreate(wavego_task, "wavego", 4096, NULL, 11, &s_wavego_task_handle);
    if (created != pdPASS) {
        s_wavego_task_handle = NULL;
        vQueueDelete(s_wavego_cmd_queue);
        s_wavego_cmd_queue = NULL;
        return ESP_FAIL;
    }
    // control_leg_height(60);
    // for(int i = 0; i< 16;i++){
    //     drive_servo_to_angle(i, 0);
    // }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // control_leg_height(0);
    // for(int i = 0; i< 16;i++){
    //     drive_servo_to_angle(i, 30);
    // }
    send_wavego_command(s_active_cmd.wave_height, s_active_cmd.wave_speed, INITIALIZING);
    return ESP_OK;
}

esp_err_t stop_wavego_task(void) {
    if (s_wavego_task_handle) {
        vTaskDelete(s_wavego_task_handle);
        s_wavego_task_handle = NULL;
    }

    if (s_wavego_cmd_queue) {
        vQueueDelete(s_wavego_cmd_queue);
        s_wavego_cmd_queue = NULL;
    }

    return ESP_OK;
}

esp_err_t send_wavego_command(int wave_height, int wave_speed, ActionState state) {
    if (!s_wavego_cmd_queue) {
        return ESP_ERR_INVALID_STATE;
    }

    wavego_command_t cmd = {
        .wave_height = (int)((float)wave_height),
        .wave_speed = (int)((float)wave_speed),
        .state = state,
    };

    if (xQueueOverwrite(s_wavego_cmd_queue, &cmd) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

ActionState get_current_action_state(void){
    return s_active_cmd.state;
}

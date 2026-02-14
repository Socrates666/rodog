/**
 * @file bare_metal_example.c
 * @brief 裸机场景示例。
 */

#include "ahrs.h"

/* 以下函数由具体平台实现 */
extern int board_read_imu(float* ax, float* ay, float* az,
                          float* gx, float* gy, float* gz,
                          float* mx, float* my, float* mz,
                          int* mag_valid);
extern uint32_t board_time_us(void);
extern void board_delay_ms(uint32_t ms);

static ahrs_status_t app_read_sensor(ahrs_sensor_sample_t* sample, void* user_ctx)
{
    (void)user_ctx;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    int mag_valid = 0;

    if (board_read_imu(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &mag_valid) != 0) {
        return AHRS_ERR_INTERNAL;
    }

    sample->accel_mps2.x = ax;
    sample->accel_mps2.y = ay;
    sample->accel_mps2.z = az;

    sample->gyro_rps.x = gx;
    sample->gyro_rps.y = gy;
    sample->gyro_rps.z = gz;

    sample->mag_ut.x = mx;
    sample->mag_ut.y = my;
    sample->mag_ut.z = mz;
    sample->mag_valid = (mag_valid != 0);

    return AHRS_OK;
}

static uint32_t app_time_us(void* user_ctx)
{
    (void)user_ctx;
    return board_time_us();
}

static void app_delay_ms(uint32_t delay_ms, void* user_ctx)
{
    (void)user_ctx;
    board_delay_ms(delay_ms);
}

void ahrs_bare_metal_demo(void)
{
    ahrs_core_t core;
    ahrs_adapter_t adapter;
    ahrs_output_t out;

    ahrs_runtime_config_t cfg = ahrs_core_default_config();
    cfg.sample_rate_hz = 200.0f;

    ahrs_calibration_t cal;
    (void)ahrs_calibration_init(&cal);

    (void)ahrs_core_init(&core, &cfg, &cal);

    ahrs_hw_if_t hw = {
        .read_sensor = app_read_sensor,
        .get_time_us = app_time_us,
        .delay_ms = app_delay_ms,
        .user_ctx = NULL,
    };

    (void)ahrs_adapter_init(&adapter, &core, &hw);

    while (1) {
        if (ahrs_adapter_poll_once(&adapter, &out) == AHRS_OK) {
            /* 使用 out.quaternion / out.euler_rad */
        }
        app_delay_ms(5U, NULL);
    }
}

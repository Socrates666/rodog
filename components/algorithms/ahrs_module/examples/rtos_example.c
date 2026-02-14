/**
 * @file rtos_example.c
 * @brief RTOS 场景示例（FreeRTOS）。
 */

#include "ahrs.h"

#if AHRS_CFG_ENABLE_PIPELINE && AHRS_CFG_PIPELINE_USE_FREERTOS

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern ahrs_status_t app_read_sensor(ahrs_sensor_sample_t* sample, void* user_ctx);
extern uint32_t app_time_us(void* user_ctx);

void ahrs_rtos_demo_task(void* arg)
{
    (void)arg;

    ahrs_core_t core;
    ahrs_adapter_t adapter;
    ahrs_pipeline_t pipeline;

    ahrs_runtime_config_t cfg = ahrs_core_default_config();
    ahrs_calibration_t cal;

    (void)ahrs_calibration_init(&cal);
    (void)ahrs_core_init(&core, &cfg, &cal);

    ahrs_hw_if_t hw = {
        .read_sensor = app_read_sensor,
        .get_time_us = app_time_us,
        .delay_ms = NULL,
        .user_ctx = NULL,
    };

    (void)ahrs_adapter_init(&adapter, &core, &hw);

    ahrs_pipeline_config_t pipe_cfg = ahrs_pipeline_default_config();
    pipe_cfg.period_ms = 5U;

    (void)ahrs_pipeline_init(&pipeline, &adapter, &pipe_cfg);
    (void)ahrs_pipeline_start(&pipeline);

    for (;;) {
        ahrs_output_t out;
        if (ahrs_pipeline_get_latest(&pipeline, &out) == AHRS_OK) {
            /* 使用 out.euler_rad */
        }
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

#endif

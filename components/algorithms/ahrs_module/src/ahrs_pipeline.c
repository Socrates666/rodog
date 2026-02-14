/**
 * @file ahrs_pipeline.c
 * @brief 可选流水线层实现。
 */

#include "ahrs_pipeline.h"

#include <string.h>

#if AHRS_CFG_ENABLE_PIPELINE && AHRS_CFG_PIPELINE_USE_FREERTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

ahrs_pipeline_config_t ahrs_pipeline_default_config(void)
{
    ahrs_pipeline_config_t cfg;
    cfg.period_ms = 5U;
    cfg.task_stack_words = 3072U;
    cfg.task_priority = 5U;
    return cfg;
}

ahrs_status_t ahrs_pipeline_init(ahrs_pipeline_t* pipeline,
                                 ahrs_adapter_t* adapter,
                                 const ahrs_pipeline_config_t* config)
{
    if ((pipeline == NULL) || (adapter == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    memset(pipeline, 0, sizeof(*pipeline));
    pipeline->adapter = adapter;
    pipeline->config = (config != NULL) ? *config : ahrs_pipeline_default_config();
    pipeline->initialized = true;
    return AHRS_OK;
}

#if AHRS_CFG_ENABLE_PIPELINE && AHRS_CFG_PIPELINE_USE_FREERTOS
static void ahrs_pipeline_task(void* arg)
{
    ahrs_pipeline_t* pipeline = (ahrs_pipeline_t*)arg;

    while (pipeline->running) {
        (void)ahrs_adapter_poll_once(pipeline->adapter, &pipeline->latest_output);
        vTaskDelay(pdMS_TO_TICKS(pipeline->config.period_ms));
    }

    vTaskDelete(NULL);
}
#endif

ahrs_status_t ahrs_pipeline_start(ahrs_pipeline_t* pipeline)
{
    if (pipeline == NULL) {
        return AHRS_ERR_NULL_PTR;
    }
    if (!pipeline->initialized) {
        return AHRS_ERR_NOT_READY;
    }

#if AHRS_CFG_ENABLE_PIPELINE && AHRS_CFG_PIPELINE_USE_FREERTOS
    if (pipeline->running) {
        return AHRS_OK;
    }

    pipeline->running = true;
    BaseType_t ret = xTaskCreate(ahrs_pipeline_task,
                                 "ahrs_pipe",
                                 pipeline->config.task_stack_words,
                                 pipeline,
                                 pipeline->config.task_priority,
                                 (TaskHandle_t*)&pipeline->task_handle);
    if (ret != pdPASS) {
        pipeline->running = false;
        return AHRS_ERR_INTERNAL;
    }
    return AHRS_OK;
#else
    (void)pipeline;
    return AHRS_ERR_UNSUPPORTED;
#endif
}

ahrs_status_t ahrs_pipeline_stop(ahrs_pipeline_t* pipeline)
{
    if (pipeline == NULL) {
        return AHRS_ERR_NULL_PTR;
    }
    if (!pipeline->initialized) {
        return AHRS_ERR_NOT_READY;
    }

#if AHRS_CFG_ENABLE_PIPELINE && AHRS_CFG_PIPELINE_USE_FREERTOS
    pipeline->running = false;
    return AHRS_OK;
#else
    return AHRS_ERR_UNSUPPORTED;
#endif
}

ahrs_status_t ahrs_pipeline_get_latest(const ahrs_pipeline_t* pipeline,
                                       ahrs_output_t* out)
{
    if ((pipeline == NULL) || (out == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }
    if (!pipeline->initialized) {
        return AHRS_ERR_NOT_READY;
    }

    *out = pipeline->latest_output;
    return AHRS_OK;
}

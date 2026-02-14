/**
 * @file ahrs_pipeline.h
 * @brief 可选流水线层（内部线程处理）。
 * @author <AUTHOR_PLACEHOLDER>
 * @version 1.0.0
 */

#ifndef AHRS_PIPELINE_H
#define AHRS_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

#include "ahrs_adapter.h"
#include "ahrs_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 流水线配置。 */
typedef struct {
    uint32_t period_ms;
    uint16_t task_stack_words;
    uint8_t task_priority;
} ahrs_pipeline_config_t;

/** @brief 流水线对象。 */
typedef struct {
    bool initialized;
    bool running;
    ahrs_adapter_t* adapter;
    ahrs_pipeline_config_t config;
    ahrs_output_t latest_output;
#if AHRS_CFG_ENABLE_PIPELINE && AHRS_CFG_PIPELINE_USE_FREERTOS
    void* task_handle;
#endif
} ahrs_pipeline_t;

/**
 * @brief 获取流水线默认配置。
 */
ahrs_pipeline_config_t ahrs_pipeline_default_config(void);

/**
 * @brief 初始化流水线对象。
 */
ahrs_status_t ahrs_pipeline_init(ahrs_pipeline_t* pipeline,
                                 ahrs_adapter_t* adapter,
                                 const ahrs_pipeline_config_t* config);

/**
 * @brief 启动内部线程。
 */
ahrs_status_t ahrs_pipeline_start(ahrs_pipeline_t* pipeline);

/**
 * @brief 停止内部线程。
 */
ahrs_status_t ahrs_pipeline_stop(ahrs_pipeline_t* pipeline);

/**
 * @brief 获取最近一次融合输出。
 */
ahrs_status_t ahrs_pipeline_get_latest(const ahrs_pipeline_t* pipeline,
                                       ahrs_output_t* out);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_PIPELINE_H */

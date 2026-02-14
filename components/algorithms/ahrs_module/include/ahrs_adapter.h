/**
 * @file ahrs_adapter.h
 * @brief 硬件适配层（函数指针注册）。
 * @author <AUTHOR_PLACEHOLDER>
 * @version 1.0.0
 */

#ifndef AHRS_ADAPTER_H
#define AHRS_ADAPTER_H

#include <stdint.h>

#include "ahrs_core.h"
#include "ahrs_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 读传感器函数签名。 */
typedef ahrs_status_t (*ahrs_adapter_read_fn)(ahrs_sensor_sample_t* sample,
                                              void* user_ctx);

/** @brief 获取时间戳函数签名（微秒）。 */
typedef uint32_t (*ahrs_adapter_time_fn)(void* user_ctx);

/** @brief 延时函数签名（毫秒）。 */
typedef void (*ahrs_adapter_delay_fn)(uint32_t delay_ms,
                                      void* user_ctx);

/** @brief 硬件接口注册对象。 */
typedef struct {
    ahrs_adapter_read_fn read_sensor;
    ahrs_adapter_time_fn get_time_us;
    ahrs_adapter_delay_fn delay_ms;
    void* user_ctx;
} ahrs_hw_if_t;

/** @brief 适配层上下文。 */
typedef struct {
    bool initialized;
    ahrs_hw_if_t hw_if;
    ahrs_core_t* core;
} ahrs_adapter_t;

/**
 * @brief 初始化适配层。
 */
ahrs_status_t ahrs_adapter_init(ahrs_adapter_t* adapter,
                                ahrs_core_t* core,
                                const ahrs_hw_if_t* hw_if);

/**
 * @brief 轮询一次传感器并更新核心算法。
 */
ahrs_status_t ahrs_adapter_poll_once(ahrs_adapter_t* adapter,
                                     ahrs_output_t* out);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_ADAPTER_H */

/**
 * @file ahrs_adapter.c
 * @brief 适配层实现。
 */

#include <stddef.h>

#include "ahrs_adapter.h"

ahrs_status_t ahrs_adapter_init(ahrs_adapter_t* adapter,
                                ahrs_core_t* core,
                                const ahrs_hw_if_t* hw_if)
{
    if ((adapter == NULL) || (core == NULL) || (hw_if == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }
    if ((hw_if->read_sensor == NULL) || (hw_if->get_time_us == NULL)) {
        return AHRS_ERR_INVALID_ARG;
    }

    adapter->core = core;
    adapter->hw_if = *hw_if;
    adapter->initialized = true;
    return AHRS_OK;
}

ahrs_status_t ahrs_adapter_poll_once(ahrs_adapter_t* adapter,
                                     ahrs_output_t* out)
{
    if (adapter == NULL) {
        return AHRS_ERR_NULL_PTR;
    }
    if (!adapter->initialized) {
        return AHRS_ERR_NOT_READY;
    }

    ahrs_sensor_sample_t sample;
    const ahrs_status_t read_ret = adapter->hw_if.read_sensor(&sample, adapter->hw_if.user_ctx);
    if (read_ret != AHRS_OK) {
        return read_ret;
    }

    sample.timestamp_us = adapter->hw_if.get_time_us(adapter->hw_if.user_ctx);
    return ahrs_core_update(adapter->core, &sample, out);
}

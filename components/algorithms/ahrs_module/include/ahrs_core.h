/**
 * @file ahrs_core.h
 * @brief 核心姿态融合算法层（纯 C99，无平台依赖）。
 * @author <AUTHOR_PLACEHOLDER>
 * @version 1.0.0
 */

#ifndef AHRS_CORE_H
#define AHRS_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "ahrs_calibration.h"
#include "ahrs_config.h"
#include "ahrs_error.h"
#include "ahrs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 核心运行时参数。 */
typedef struct {
    float sample_rate_hz;
    float madgwick_beta;

    float accel_lpf_alpha;
    float gyro_lpf_alpha;
    float mag_lpf_alpha;

    float gyro_bias_alpha;
    float accel_rest_threshold_mps2;
    float gyro_rest_threshold_rps;

    float yaw_declination_rad;
} ahrs_runtime_config_t;

/** @brief 核心状态。 */
typedef struct {
    bool initialized;
    uint32_t last_timestamp_us;

    ahrs_runtime_config_t config;
    ahrs_calibration_t calibration;

    ahrs_quat_t q;

    ahrs_vec3f_t accel_lpf;
    ahrs_vec3f_t gyro_lpf;
    ahrs_vec3f_t mag_lpf;

    ahrs_vec3f_t gyro_bias;

    ahrs_output_t output;
} ahrs_core_t;

/**
 * @brief 获取默认运行时配置。
 */
ahrs_runtime_config_t ahrs_core_default_config(void);

/**
 * @brief 初始化核心算法对象。
 */
ahrs_status_t ahrs_core_init(ahrs_core_t* core,
                             const ahrs_runtime_config_t* config,
                             const ahrs_calibration_t* calibration);

/**
 * @brief 重置核心状态（保留配置和校准应由调用者重新传入）。
 */
ahrs_status_t ahrs_core_reset(ahrs_core_t* core);

/**
 * @brief 设置运行时配置。
 */
ahrs_status_t ahrs_core_set_config(ahrs_core_t* core,
                                   const ahrs_runtime_config_t* config);

/**
 * @brief 获取运行时配置。
 */
ahrs_status_t ahrs_core_get_config(const ahrs_core_t* core,
                                   ahrs_runtime_config_t* config_out);

/**
 * @brief 加载校准参数（拷贝）。
 */
ahrs_status_t ahrs_core_load_calibration(ahrs_core_t* core,
                                         const ahrs_calibration_t* calibration);

/**
 * @brief 导出当前校准参数（拷贝）。
 */
ahrs_status_t ahrs_core_get_calibration(const ahrs_core_t* core,
                                        ahrs_calibration_t* calibration_out);

/**
 * @brief 输入一帧原始传感器数据并更新姿态结果。
 */
ahrs_status_t ahrs_core_update(ahrs_core_t* core,
                               const ahrs_sensor_sample_t* sample,
                               ahrs_output_t* out);

/**
 * @brief 获取当前姿态输出快照。
 */
ahrs_status_t ahrs_core_get_output(const ahrs_core_t* core,
                                   ahrs_output_t* out);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_CORE_H */

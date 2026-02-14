/**
 * @file ahrs_calibration.h
 * @brief 加速度计/磁力计校准接口。
 * @author <AUTHOR_PLACEHOLDER>
 * @version 1.0.0
 */

#ifndef AHRS_CALIBRATION_H
#define AHRS_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#include "ahrs_error.h"
#include "ahrs_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 3x3 矩阵。 */
typedef struct {
    float m[3][3];
} ahrs_mat3f_t;

/** @brief 在线六面加速度计校准状态。 */
typedef struct {
    bool initialized;
    uint32_t sample_count;
    ahrs_vec3f_t min_val;
    ahrs_vec3f_t max_val;
} ahrs_accel_online_cal_t;

/** @brief 在线磁力计校准状态。 */
typedef struct {
    bool initialized;
    uint32_t sample_count;
    ahrs_vec3f_t min_val;
    ahrs_vec3f_t max_val;
} ahrs_mag_online_cal_t;

/** @brief 校准参数。 */
typedef struct {
    bool accel_valid;
    bool mag_valid;

    ahrs_vec3f_t accel_bias;
    ahrs_vec3f_t accel_scale;

    ahrs_vec3f_t mag_bias;
    ahrs_mat3f_t mag_soft_iron;

    ahrs_accel_online_cal_t accel_online;
    ahrs_mag_online_cal_t mag_online;
} ahrs_calibration_t;

/**
 * @brief 初始化校准参数为默认值。
 * @param[out] cal 校准参数对象。
 * @return AHRS_OK 或错误码。
 */
ahrs_status_t ahrs_calibration_init(ahrs_calibration_t* cal);

/**
 * @brief 载入加速度计校准参数。
 */
ahrs_status_t ahrs_calibration_set_accel(ahrs_calibration_t* cal,
                                         const ahrs_vec3f_t* bias,
                                         const ahrs_vec3f_t* scale);

/**
 * @brief 载入磁力计校准参数（硬铁 + 软铁矩阵）。
 */
ahrs_status_t ahrs_calibration_set_mag(ahrs_calibration_t* cal,
                                       const ahrs_vec3f_t* bias,
                                       const ahrs_mat3f_t* soft_iron);

/**
 * @brief 在线更新六面加速度计校准采样。
 */
ahrs_status_t ahrs_calibration_accel_online_update(ahrs_calibration_t* cal,
                                                    const ahrs_vec3f_t* accel_raw);

/**
 * @brief 在线更新磁力计校准采样。
 */
ahrs_status_t ahrs_calibration_mag_online_update(ahrs_calibration_t* cal,
                                                  const ahrs_vec3f_t* mag_raw);

/**
 * @brief 依据在线采样结果完成加速度计标定。
 */
ahrs_status_t ahrs_calibration_accel_online_finalize(ahrs_calibration_t* cal,
                                                      float target_g_mps2);

/**
 * @brief 依据在线采样结果完成磁力计硬铁/软铁近似标定。
 */
ahrs_status_t ahrs_calibration_mag_online_finalize(ahrs_calibration_t* cal);

/**
 * @brief 应用加速度计校准。
 */
ahrs_vec3f_t ahrs_calibration_apply_accel(const ahrs_calibration_t* cal,
                                          const ahrs_vec3f_t* raw);

/**
 * @brief 应用磁力计校准。
 */
ahrs_vec3f_t ahrs_calibration_apply_mag(const ahrs_calibration_t* cal,
                                        const ahrs_vec3f_t* raw);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_CALIBRATION_H */

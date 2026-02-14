/**
 * @file ahrs_types.h
 * @brief AHRS 基础数据结构定义。
 * @author <AUTHOR_PLACEHOLDER>
 * @version 1.0.0
 */

#ifndef AHRS_TYPES_H
#define AHRS_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 三维向量。 */
typedef struct {
    float x;
    float y;
    float z;
} ahrs_vec3f_t;

/** @brief 四元数 (w, x, y, z)。 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} ahrs_quat_t;

/** @brief 欧拉角（弧度）。 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} ahrs_euler_t;

/** @brief 传感器输入样本。 */
typedef struct {
    ahrs_vec3f_t accel_mps2;
    ahrs_vec3f_t gyro_rps;
    ahrs_vec3f_t mag_ut;
    uint32_t timestamp_us;
    bool mag_valid;
} ahrs_sensor_sample_t;

/** @brief 姿态融合输出。 */
typedef struct {
    ahrs_quat_t quaternion;
    ahrs_euler_t euler_rad;
    ahrs_vec3f_t gravity_mps2;
    ahrs_vec3f_t linear_accel_mps2;
} ahrs_output_t;

#ifdef __cplusplus
}
#endif

#endif /* AHRS_TYPES_H */

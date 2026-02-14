/**
 * @file ahrs_calibration.c
 * @brief 校准实现。
 */

#include "ahrs_calibration.h"

#include <math.h>
#include <string.h>

#include "ahrs_config.h"

static float clamp_non_zero(float v)
{
    const float eps = 1e-6f;
    if (fabsf(v) < eps) {
        return (v >= 0.0f) ? eps : -eps;
    }
    return v;
}

static ahrs_mat3f_t identity_mat3(void)
{
    ahrs_mat3f_t m = {0};
    m.m[0][0] = 1.0f;
    m.m[1][1] = 1.0f;
    m.m[2][2] = 1.0f;
    return m;
}

ahrs_status_t ahrs_calibration_init(ahrs_calibration_t* cal)
{
    if (cal == NULL) {
        return AHRS_ERR_NULL_PTR;
    }

    memset(cal, 0, sizeof(*cal));
    cal->accel_scale.x = 1.0f;
    cal->accel_scale.y = 1.0f;
    cal->accel_scale.z = 1.0f;
    cal->mag_soft_iron = identity_mat3();
    return AHRS_OK;
}

ahrs_status_t ahrs_calibration_set_accel(ahrs_calibration_t* cal,
                                         const ahrs_vec3f_t* bias,
                                         const ahrs_vec3f_t* scale)
{
    if ((cal == NULL) || (bias == NULL) || (scale == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    cal->accel_bias = *bias;
    cal->accel_scale = *scale;
    cal->accel_valid = true;
    return AHRS_OK;
}

ahrs_status_t ahrs_calibration_set_mag(ahrs_calibration_t* cal,
                                       const ahrs_vec3f_t* bias,
                                       const ahrs_mat3f_t* soft_iron)
{
    if ((cal == NULL) || (bias == NULL) || (soft_iron == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    cal->mag_bias = *bias;
    cal->mag_soft_iron = *soft_iron;
    cal->mag_valid = true;
    return AHRS_OK;
}

ahrs_status_t ahrs_calibration_accel_online_update(ahrs_calibration_t* cal,
                                                    const ahrs_vec3f_t* accel_raw)
{
    if ((cal == NULL) || (accel_raw == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    if (!cal->accel_online.initialized) {
        cal->accel_online.initialized = true;
        cal->accel_online.min_val = *accel_raw;
        cal->accel_online.max_val = *accel_raw;
        cal->accel_online.sample_count = 1U;
        return AHRS_OK;
    }

    if (accel_raw->x < cal->accel_online.min_val.x) {
        cal->accel_online.min_val.x = accel_raw->x;
    }
    if (accel_raw->y < cal->accel_online.min_val.y) {
        cal->accel_online.min_val.y = accel_raw->y;
    }
    if (accel_raw->z < cal->accel_online.min_val.z) {
        cal->accel_online.min_val.z = accel_raw->z;
    }

    if (accel_raw->x > cal->accel_online.max_val.x) {
        cal->accel_online.max_val.x = accel_raw->x;
    }
    if (accel_raw->y > cal->accel_online.max_val.y) {
        cal->accel_online.max_val.y = accel_raw->y;
    }
    if (accel_raw->z > cal->accel_online.max_val.z) {
        cal->accel_online.max_val.z = accel_raw->z;
    }

    cal->accel_online.sample_count++;
    return AHRS_OK;
}

ahrs_status_t ahrs_calibration_mag_online_update(ahrs_calibration_t* cal,
                                                  const ahrs_vec3f_t* mag_raw)
{
    if ((cal == NULL) || (mag_raw == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    if (!cal->mag_online.initialized) {
        cal->mag_online.initialized = true;
        cal->mag_online.min_val = *mag_raw;
        cal->mag_online.max_val = *mag_raw;
        cal->mag_online.sample_count = 1U;
        return AHRS_OK;
    }

    if (mag_raw->x < cal->mag_online.min_val.x) {
        cal->mag_online.min_val.x = mag_raw->x;
    }
    if (mag_raw->y < cal->mag_online.min_val.y) {
        cal->mag_online.min_val.y = mag_raw->y;
    }
    if (mag_raw->z < cal->mag_online.min_val.z) {
        cal->mag_online.min_val.z = mag_raw->z;
    }

    if (mag_raw->x > cal->mag_online.max_val.x) {
        cal->mag_online.max_val.x = mag_raw->x;
    }
    if (mag_raw->y > cal->mag_online.max_val.y) {
        cal->mag_online.max_val.y = mag_raw->y;
    }
    if (mag_raw->z > cal->mag_online.max_val.z) {
        cal->mag_online.max_val.z = mag_raw->z;
    }

    cal->mag_online.sample_count++;
    return AHRS_OK;
}

ahrs_status_t ahrs_calibration_accel_online_finalize(ahrs_calibration_t* cal,
                                                      float target_g_mps2)
{
    if (cal == NULL) {
        return AHRS_ERR_NULL_PTR;
    }
    if ((!cal->accel_online.initialized) || (cal->accel_online.sample_count < 20U) ||
        (target_g_mps2 <= 0.0f)) {
        return AHRS_ERR_NOT_READY;
    }

    const float range_x = clamp_non_zero(cal->accel_online.max_val.x - cal->accel_online.min_val.x);
    const float range_y = clamp_non_zero(cal->accel_online.max_val.y - cal->accel_online.min_val.y);
    const float range_z = clamp_non_zero(cal->accel_online.max_val.z - cal->accel_online.min_val.z);

    cal->accel_bias.x = (cal->accel_online.max_val.x + cal->accel_online.min_val.x) * 0.5f;
    cal->accel_bias.y = (cal->accel_online.max_val.y + cal->accel_online.min_val.y) * 0.5f;
    cal->accel_bias.z = (cal->accel_online.max_val.z + cal->accel_online.min_val.z) * 0.5f;

    cal->accel_scale.x = (2.0f * target_g_mps2) / range_x;
    cal->accel_scale.y = (2.0f * target_g_mps2) / range_y;
    cal->accel_scale.z = (2.0f * target_g_mps2) / range_z;

    cal->accel_valid = true;
    return AHRS_OK;
}

ahrs_status_t ahrs_calibration_mag_online_finalize(ahrs_calibration_t* cal)
{
    if (cal == NULL) {
        return AHRS_ERR_NULL_PTR;
    }
    if ((!cal->mag_online.initialized) || (cal->mag_online.sample_count < 40U)) {
        return AHRS_ERR_NOT_READY;
    }

    const float range_x = clamp_non_zero(cal->mag_online.max_val.x - cal->mag_online.min_val.x);
    const float range_y = clamp_non_zero(cal->mag_online.max_val.y - cal->mag_online.min_val.y);
    const float range_z = clamp_non_zero(cal->mag_online.max_val.z - cal->mag_online.min_val.z);

    const float avg_radius = (range_x + range_y + range_z) / 3.0f;

    cal->mag_bias.x = (cal->mag_online.max_val.x + cal->mag_online.min_val.x) * 0.5f;
    cal->mag_bias.y = (cal->mag_online.max_val.y + cal->mag_online.min_val.y) * 0.5f;
    cal->mag_bias.z = (cal->mag_online.max_val.z + cal->mag_online.min_val.z) * 0.5f;

    cal->mag_soft_iron = identity_mat3();
    cal->mag_soft_iron.m[0][0] = avg_radius / range_x;
    cal->mag_soft_iron.m[1][1] = avg_radius / range_y;
    cal->mag_soft_iron.m[2][2] = avg_radius / range_z;

    cal->mag_valid = true;
    return AHRS_OK;
}

ahrs_vec3f_t ahrs_calibration_apply_accel(const ahrs_calibration_t* cal,
                                          const ahrs_vec3f_t* raw)
{
    ahrs_vec3f_t out = *raw;
    if ((cal == NULL) || !cal->accel_valid) {
        return out;
    }

    out.x = (raw->x - cal->accel_bias.x) * cal->accel_scale.x;
    out.y = (raw->y - cal->accel_bias.y) * cal->accel_scale.y;
    out.z = (raw->z - cal->accel_bias.z) * cal->accel_scale.z;
    return out;
}

ahrs_vec3f_t ahrs_calibration_apply_mag(const ahrs_calibration_t* cal,
                                        const ahrs_vec3f_t* raw)
{
    ahrs_vec3f_t out = *raw;
    if ((cal == NULL) || !cal->mag_valid) {
        return out;
    }

    const float x = raw->x - cal->mag_bias.x;
    const float y = raw->y - cal->mag_bias.y;
    const float z = raw->z - cal->mag_bias.z;

    out.x = cal->mag_soft_iron.m[0][0] * x + cal->mag_soft_iron.m[0][1] * y + cal->mag_soft_iron.m[0][2] * z;
    out.y = cal->mag_soft_iron.m[1][0] * x + cal->mag_soft_iron.m[1][1] * y + cal->mag_soft_iron.m[1][2] * z;
    out.z = cal->mag_soft_iron.m[2][0] * x + cal->mag_soft_iron.m[2][1] * y + cal->mag_soft_iron.m[2][2] * z;
    return out;
}

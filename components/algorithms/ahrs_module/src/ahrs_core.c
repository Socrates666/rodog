/**
 * @file ahrs_core.c
 * @brief AHRS 核心算法实现（Madgwick + 预处理）。
 */

#include "ahrs_core.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float safe_inv_sqrt(float x)
{
    if (x <= 1e-12f) {
        return 0.0f;
    }
    return 1.0f / sqrtf(x);
}

static float vec_norm(const ahrs_vec3f_t* v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

static ahrs_vec3f_t vec_sub(const ahrs_vec3f_t* a, const ahrs_vec3f_t* b)
{
    ahrs_vec3f_t out;
    out.x = a->x - b->x;
    out.y = a->y - b->y;
    out.z = a->z - b->z;
    return out;
}

static ahrs_vec3f_t lpf_update(const ahrs_vec3f_t* prev, const ahrs_vec3f_t* in, float alpha)
{
    ahrs_vec3f_t out;
    const float a = (alpha < 0.0f) ? 0.0f : ((alpha > 1.0f) ? 1.0f : alpha);
    const float ia = 1.0f - a;
    out.x = a * in->x + ia * prev->x;
    out.y = a * in->y + ia * prev->y;
    out.z = a * in->z + ia * prev->z;
    return out;
}

static void quat_normalize(ahrs_quat_t* q)
{
    const float inv = safe_inv_sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (inv <= 0.0f) {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }
    q->w *= inv;
    q->x *= inv;
    q->y *= inv;
    q->z *= inv;
}

static void quat_to_euler(const ahrs_quat_t* q, ahrs_euler_t* e)
{
    const float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    const float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    e->roll = atan2f(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f) {
        e->pitch = copysignf((float)M_PI_2, sinp);
    } else {
        e->pitch = asinf(sinp);
    }

    const float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    const float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    e->yaw = atan2f(siny_cosp, cosy_cosp);
}

static ahrs_vec3f_t gravity_from_quat(const ahrs_quat_t* q)
{
    ahrs_vec3f_t g;
    g.x = 2.0f * (q->x * q->z - q->w * q->y) * AHRS_GRAVITY_MPS2;
    g.y = 2.0f * (q->w * q->x + q->y * q->z) * AHRS_GRAVITY_MPS2;
    g.z = (q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z) * AHRS_GRAVITY_MPS2;
    return g;
}

static void madgwick_update_imu(ahrs_core_t* core,
                                const ahrs_vec3f_t* gyro_rps,
                                const ahrs_vec3f_t* accel_mps2,
                                float dt)
{
    ahrs_quat_t* q = &core->q;

    const float gx = gyro_rps->x;
    const float gy = gyro_rps->y;
    const float gz = gyro_rps->z;

    float ax = accel_mps2->x;
    float ay = accel_mps2->y;
    float az = accel_mps2->z;

    float q1 = q->w;
    float q2 = q->x;
    float q3 = q->y;
    float q4 = q->z;

    const float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_norm > 1e-6f) {
        const float inv_norm = 1.0f / accel_norm;
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;

        const float _2q1 = 2.0f * q1;
        const float _2q2 = 2.0f * q2;
        const float _2q3 = 2.0f * q3;
        const float _2q4 = 2.0f * q4;
        const float _4q1 = 4.0f * q1;
        const float _4q2 = 4.0f * q2;
        const float _4q3 = 4.0f * q3;
        const float _8q2 = 8.0f * q2;
        const float _8q3 = 8.0f * q3;
        const float q1q1 = q1 * q1;
        const float q2q2 = q2 * q2;
        const float q3q3 = q3 * q3;
        const float q4q4 = q4 * q4;

        float s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
        float s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 +
                   _8q2 * q3q3 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 +
                   _8q3 * q3q3 + _4q3 * az;
        float s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;

        const float recip_norm = safe_inv_sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;
        s4 *= recip_norm;

        const float q_dot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - core->config.madgwick_beta * s1;
        const float q_dot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - core->config.madgwick_beta * s2;
        const float q_dot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - core->config.madgwick_beta * s3;
        const float q_dot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - core->config.madgwick_beta * s4;

        q1 += q_dot1 * dt;
        q2 += q_dot2 * dt;
        q3 += q_dot3 * dt;
        q4 += q_dot4 * dt;
    } else {
        const float q_dot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
        const float q_dot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
        const float q_dot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
        const float q_dot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

        q1 += q_dot1 * dt;
        q2 += q_dot2 * dt;
        q3 += q_dot3 * dt;
        q4 += q_dot4 * dt;
    }

    q->w = q1;
    q->x = q2;
    q->y = q3;
    q->z = q4;
    quat_normalize(q);
}

static void update_output(ahrs_core_t* core,
                          const ahrs_vec3f_t* accel_corrected)
{
    core->output.quaternion = core->q;
    quat_to_euler(&core->q, &core->output.euler_rad);
    core->output.euler_rad.yaw += core->config.yaw_declination_rad;

    core->output.gravity_mps2 = gravity_from_quat(&core->q);
    core->output.linear_accel_mps2 = vec_sub(accel_corrected, &core->output.gravity_mps2);
}

ahrs_runtime_config_t ahrs_core_default_config(void)
{
    ahrs_runtime_config_t cfg;
    cfg.sample_rate_hz = AHRS_CFG_DEFAULT_SAMPLE_RATE_HZ;
    cfg.madgwick_beta = AHRS_CFG_DEFAULT_MADGWICK_BETA;
    cfg.accel_lpf_alpha = 0.40f;
    cfg.gyro_lpf_alpha = 0.45f;
    cfg.mag_lpf_alpha = 0.30f;
    cfg.gyro_bias_alpha = 0.001f;
    cfg.accel_rest_threshold_mps2 = 0.15f;
    cfg.gyro_rest_threshold_rps = 0.08f;
    cfg.yaw_declination_rad = 0.0f;
    return cfg;
}

ahrs_status_t ahrs_core_init(ahrs_core_t* core,
                             const ahrs_runtime_config_t* config,
                             const ahrs_calibration_t* calibration)
{
    if (core == NULL) {
        return AHRS_ERR_NULL_PTR;
    }

    memset(core, 0, sizeof(*core));
    core->q.w = 1.0f;

    core->config = (config != NULL) ? *config : ahrs_core_default_config();

    if (calibration != NULL) {
        core->calibration = *calibration;
    } else {
        (void)ahrs_calibration_init(&core->calibration);
    }

    core->initialized = true;
    return AHRS_OK;
}

ahrs_status_t ahrs_core_reset(ahrs_core_t* core)
{
    if (core == NULL) {
        return AHRS_ERR_NULL_PTR;
    }

    const ahrs_runtime_config_t cfg = core->config;
    const ahrs_calibration_t cal = core->calibration;
    return ahrs_core_init(core, &cfg, &cal);
}

ahrs_status_t ahrs_core_set_config(ahrs_core_t* core,
                                   const ahrs_runtime_config_t* config)
{
    if ((core == NULL) || (config == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    if (config->sample_rate_hz <= 0.0f) {
        return AHRS_ERR_INVALID_ARG;
    }

    core->config = *config;
    return AHRS_OK;
}

ahrs_status_t ahrs_core_get_config(const ahrs_core_t* core,
                                   ahrs_runtime_config_t* config_out)
{
    if ((core == NULL) || (config_out == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    *config_out = core->config;
    return AHRS_OK;
}

ahrs_status_t ahrs_core_load_calibration(ahrs_core_t* core,
                                         const ahrs_calibration_t* calibration)
{
    if ((core == NULL) || (calibration == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    core->calibration = *calibration;
    return AHRS_OK;
}

ahrs_status_t ahrs_core_get_calibration(const ahrs_core_t* core,
                                        ahrs_calibration_t* calibration_out)
{
    if ((core == NULL) || (calibration_out == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }

    *calibration_out = core->calibration;
    return AHRS_OK;
}

ahrs_status_t ahrs_core_update(ahrs_core_t* core,
                               const ahrs_sensor_sample_t* sample,
                               ahrs_output_t* out)
{
    if ((core == NULL) || (sample == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }
    if (!core->initialized) {
        return AHRS_ERR_NOT_READY;
    }

    float dt = 1.0f / core->config.sample_rate_hz;
    if ((core->last_timestamp_us != 0U) && (sample->timestamp_us > core->last_timestamp_us)) {
        dt = (float)(sample->timestamp_us - core->last_timestamp_us) * 1e-6f;
    }
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 1.0f / core->config.sample_rate_hz;
    }
    core->last_timestamp_us = sample->timestamp_us;

    const ahrs_vec3f_t accel_cal = ahrs_calibration_apply_accel(&core->calibration, &sample->accel_mps2);
    const ahrs_vec3f_t mag_cal = ahrs_calibration_apply_mag(&core->calibration, &sample->mag_ut);

    core->accel_lpf = lpf_update(&core->accel_lpf, &accel_cal, core->config.accel_lpf_alpha);
    core->gyro_lpf = lpf_update(&core->gyro_lpf, &sample->gyro_rps, core->config.gyro_lpf_alpha);
    core->mag_lpf = lpf_update(&core->mag_lpf, &mag_cal, core->config.mag_lpf_alpha);

    const float accel_norm = vec_norm(&core->accel_lpf);
    const float gyro_norm = vec_norm(&core->gyro_lpf);

    if ((fabsf(accel_norm - AHRS_GRAVITY_MPS2) <= core->config.accel_rest_threshold_mps2) &&
        (gyro_norm <= core->config.gyro_rest_threshold_rps)) {
        core->gyro_bias = lpf_update(&core->gyro_bias, &core->gyro_lpf, core->config.gyro_bias_alpha);
    }

    const ahrs_vec3f_t gyro_debiased = vec_sub(&core->gyro_lpf, &core->gyro_bias);

#if AHRS_CFG_FILTER_MADGWICK
    (void)mag_cal;
#if AHRS_CFG_USE_MAG
    /* 当前使用 IMU 模式更新，磁力计可用于后续扩展全 MARG Madgwick。 */
    (void)sample->mag_valid;
#endif
    madgwick_update_imu(core, &gyro_debiased, &core->accel_lpf, dt);
#else
#error "No valid AHRS filter selected."
#endif

    update_output(core, &core->accel_lpf);

    if (out != NULL) {
        *out = core->output;
    }

    return AHRS_OK;
}

ahrs_status_t ahrs_core_get_output(const ahrs_core_t* core,
                                   ahrs_output_t* out)
{
    if ((core == NULL) || (out == NULL)) {
        return AHRS_ERR_NULL_PTR;
    }
    if (!core->initialized) {
        return AHRS_ERR_NOT_READY;
    }

    *out = core->output;
    return AHRS_OK;
}

/**
 * @file test_main.c
 * @brief 单元测试入口（Unity）。
 */

#include "ahrs.h"

#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

static void test_ahrs_init_and_update_should_keep_quaternion_normalized(void)
{
    ahrs_core_t core;
    ahrs_runtime_config_t cfg = ahrs_core_default_config();
    ahrs_calibration_t cal;

    TEST_ASSERT_EQUAL(AHRS_OK, ahrs_calibration_init(&cal));
    TEST_ASSERT_EQUAL(AHRS_OK, ahrs_core_init(&core, &cfg, &cal));

    ahrs_sensor_sample_t sample = {
        .accel_mps2 = {0.0f, 0.0f, AHRS_GRAVITY_MPS2},
        .gyro_rps = {0.0f, 0.0f, 0.0f},
        .mag_ut = {30.0f, 0.0f, 40.0f},
        .timestamp_us = 10000U,
        .mag_valid = true,
    };

    ahrs_output_t out;
    TEST_ASSERT_EQUAL(AHRS_OK, ahrs_core_update(&core, &sample, &out));

    const float q_norm = out.quaternion.w * out.quaternion.w +
                         out.quaternion.x * out.quaternion.x +
                         out.quaternion.y * out.quaternion.y +
                         out.quaternion.z * out.quaternion.z;

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, q_norm);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ahrs_init_and_update_should_keep_quaternion_normalized);
    return UNITY_END();
}

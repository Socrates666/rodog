/**
 * @file ahrs_config.h
 * @brief AHRS 编译期与运行时配置定义。
 * @author <AUTHOR_PLACEHOLDER>
 * @version 1.0.0
 */

#ifndef AHRS_CONFIG_H
#define AHRS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 采样重力常量。 */
#define AHRS_GRAVITY_MPS2 (9.80665f)

/** @brief 启用磁力计融合（1:启用, 0:禁用）。 */
#ifndef AHRS_CFG_USE_MAG
#define AHRS_CFG_USE_MAG 1
#endif

/** @brief 选择滤波器（当前仅 Madgwick）。 */
#ifndef AHRS_CFG_FILTER_MADGWICK
#define AHRS_CFG_FILTER_MADGWICK 1
#endif

/** @brief 启用流水线线程层（1:启用, 0:禁用）。 */
#ifndef AHRS_CFG_ENABLE_PIPELINE
#define AHRS_CFG_ENABLE_PIPELINE 0
#endif

/** @brief 流水线层启用 FreeRTOS 后端（1:启用, 0:禁用）。 */
#ifndef AHRS_CFG_PIPELINE_USE_FREERTOS
#define AHRS_CFG_PIPELINE_USE_FREERTOS 0
#endif

/** @brief 默认采样率（Hz）。 */
#ifndef AHRS_CFG_DEFAULT_SAMPLE_RATE_HZ
#define AHRS_CFG_DEFAULT_SAMPLE_RATE_HZ (200.0f)
#endif

/** @brief 默认 Madgwick beta。 */
#ifndef AHRS_CFG_DEFAULT_MADGWICK_BETA_MILLI
#define AHRS_CFG_DEFAULT_MADGWICK_BETA_MILLI (80)
#endif

/** @brief 默认 Madgwick beta（由 x1000 整数配置转换）。 */
#ifndef AHRS_CFG_DEFAULT_MADGWICK_BETA
#define AHRS_CFG_DEFAULT_MADGWICK_BETA ((AHRS_CFG_DEFAULT_MADGWICK_BETA_MILLI) / 1000.0f)
#endif

#ifdef __cplusplus
}
#endif

#endif /* AHRS_CONFIG_H */

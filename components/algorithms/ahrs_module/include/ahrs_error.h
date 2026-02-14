/**
 * @file ahrs_error.h
 * @brief 统一错误码定义。
 * @author <AUTHOR_PLACEHOLDER>
 * @version 1.0.0
 */

#ifndef AHRS_ERROR_H
#define AHRS_ERROR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief AHRS 错误码。 */
typedef enum {
    AHRS_OK = 0,
    AHRS_ERR_NULL_PTR = -1,
    AHRS_ERR_INVALID_ARG = -2,
    AHRS_ERR_NOT_READY = -3,
    AHRS_ERR_UNSUPPORTED = -4,
    AHRS_ERR_TIMEOUT = -5,
    AHRS_ERR_INTERNAL = -6
} ahrs_status_t;

#ifdef __cplusplus
}
#endif

#endif /* AHRS_ERROR_H */

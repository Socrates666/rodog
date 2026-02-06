#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize calibration data (loads defaults and optional NVS overrides).
esp_err_t calibration_init(void);

// Load NVS-stored calibration (if enabled).
esp_err_t calibration_load_from_nvs(void);

// Reset calibration to built-in defaults and clear NVS when enabled.
esp_err_t calibration_reset_defaults(void);

// Set/Get middle PWM value for a channel. Setter persists to NVS if enabled.
esp_err_t calibration_set_middle_pwm(uint8_t channel, uint16_t pwm);
esp_err_t calibration_get_middle_pwm(uint8_t channel, uint16_t *pwm_out);

#ifdef __cplusplus
}
#endif

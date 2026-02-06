#include "calibration.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#define TAG "CALIB"

// Defaults derived from servo_calibration_log.txt (index 0..15)
static const uint16_t s_default_middle[16] = {
    397, 195, 145, 307, 307, 425, 380, 200,
    390, 205, 168, 307, 307, 408, 370, 217,
};

#define CAL_NAMESPACE "servo_cal"
#define SERVO_COUNT 16

#ifndef CONFIG_CONTROL_CALIBRATION_ENABLE
#define CONFIG_CONTROL_CALIBRATION_ENABLE 1
#endif

static uint16_t s_middle_pwm[SERVO_COUNT];
static bool s_calibration_enabled = CONFIG_CONTROL_CALIBRATION_ENABLE;

static void load_defaults_into_ram(void) {
    memcpy(s_middle_pwm, s_default_middle, sizeof(s_middle_pwm));
}

static esp_err_t nvs_set_middle(uint8_t channel, uint16_t value) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CAL_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    char key[8];
    snprintf(key, sizeof(key), "mid%u", channel);
    err = nvs_set_u16(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t nvs_get_middle(uint8_t channel, uint16_t *value) {
    if (!value) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CAL_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    char key[8];
    snprintf(key, sizeof(key), "mid%u", channel);
    err = nvs_get_u16(handle, key, value);
    nvs_close(handle);
    return err;
}

esp_err_t calibration_load_from_nvs(void) {
    if (!s_calibration_enabled) {
        return ESP_OK;
    }

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        uint16_t val = 0;
        esp_err_t err = nvs_get_middle(i, &val);
        if (err == ESP_OK) {
            s_middle_pwm[i] = val;
        }
    }
    return ESP_OK;
}

esp_err_t calibration_reset_defaults(void) {
    load_defaults_into_ram();

    if (!s_calibration_enabled) {
        return ESP_OK;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(CAL_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t calibration_init(void) {
    load_defaults_into_ram();

    if (!s_calibration_enabled) {
        ESP_LOGI(TAG, "Calibration: NVS disabled, using defaults");
        return ESP_OK;
    }

    esp_err_t err = calibration_load_from_nvs();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Calibration: load from NVS failed (%s), using defaults", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Calibration initialized (NVS %s)", s_calibration_enabled ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t calibration_set_middle_pwm(uint8_t channel, uint16_t pwm) {
    if (channel >= SERVO_COUNT) return ESP_ERR_INVALID_ARG;
    s_middle_pwm[channel] = pwm;

    if (!s_calibration_enabled) {
        return ESP_OK;
    }

    esp_err_t err = nvs_set_middle(channel, pwm);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save middle for ch%u: %s", channel, esp_err_to_name(err));
    }
    return err;
}

esp_err_t calibration_get_middle_pwm(uint8_t channel, uint16_t *pwm_out) {
    if (channel >= SERVO_COUNT || !pwm_out) return ESP_ERR_INVALID_ARG;
    *pwm_out = s_middle_pwm[channel];
    return ESP_OK;
}

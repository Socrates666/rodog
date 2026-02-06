#ifndef MICRO_ROS_UART_H
#define MICRO_ROS_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    MICRO_ROS_SERVO_MODE_PWM = 0,
    MICRO_ROS_SERVO_MODE_ANGLE = 1,
} micro_ros_servo_mode_t;

typedef struct {
    int uart_port;
    int baud_rate;
    int rx_buffer_size;
    int tx_buffer_size;
} micro_ros_uart_config_t;

typedef esp_err_t (*micro_ros_servo_handler_t)(uint8_t servo_id, int value, micro_ros_servo_mode_t mode);

esp_err_t micro_ros_uart_start(const micro_ros_uart_config_t *config, micro_ros_servo_handler_t handler);
void micro_ros_uart_stop(void);
bool micro_ros_uart_is_running(void);

#ifdef __cplusplus
}
#endif

#endif

#include "micro_ros_uart.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "calibration.h"
#include "pca9685.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <uxr/client/transport.h>

#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 8192
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

#ifndef CONFIG_COMM_MICRO_ROS_UART_PORT
#define CONFIG_COMM_MICRO_ROS_UART_PORT 0
#endif

#ifndef CONFIG_COMM_MICRO_ROS_UART_BAUD
#define CONFIG_COMM_MICRO_ROS_UART_BAUD 115200
#endif

#ifndef CONFIG_COMM_MICRO_ROS_UART_RX_BUFFER
#define CONFIG_COMM_MICRO_ROS_UART_RX_BUFFER 1024
#endif

#ifndef CONFIG_COMM_MICRO_ROS_UART_TX_BUFFER
#define CONFIG_COMM_MICRO_ROS_UART_TX_BUFFER 0
#endif

#ifndef CONFIG_COMM_MICRO_ROS_NODE_NAME
#define CONFIG_COMM_MICRO_ROS_NODE_NAME "rodog_uart_node"
#endif

#ifndef CONFIG_COMM_MICRO_ROS_SERVO_TOPIC
#define CONFIG_COMM_MICRO_ROS_SERVO_TOPIC "/rodog/servo_cmd"
#endif

#ifndef CONFIG_MICROROS_UART_TXD
#define CONFIG_MICROROS_UART_TXD UART_PIN_NO_CHANGE
#endif

#ifndef CONFIG_MICROROS_UART_RXD
#define CONFIG_MICROROS_UART_RXD UART_PIN_NO_CHANGE
#endif

#ifndef CONFIG_MICROROS_UART_RTS
#define CONFIG_MICROROS_UART_RTS UART_PIN_NO_CHANGE
#endif

#ifndef CONFIG_MICROROS_UART_CTS
#define CONFIG_MICROROS_UART_CTS UART_PIN_NO_CHANGE
#endif

#define MICRO_ROS_UART_SPIN_PERIOD_MS 10
#define MICRO_ROS_UART_EXECUTOR_TIMEOUT_MS 50

#define MICRO_ROS_UART_DEFAULT_PORT CONFIG_COMM_MICRO_ROS_UART_PORT
#define MICRO_ROS_UART_DEFAULT_BAUD CONFIG_COMM_MICRO_ROS_UART_BAUD
#define MICRO_ROS_UART_DEFAULT_RX_BUFFER CONFIG_COMM_MICRO_ROS_UART_RX_BUFFER
#define MICRO_ROS_UART_DEFAULT_TX_BUFFER CONFIG_COMM_MICRO_ROS_UART_TX_BUFFER

static const char *TAG = "MICRO_ROS_UART";

typedef struct {
    uart_port_t uart_port;
    int baud_rate;
    int rx_buffer_size;
    int tx_buffer_size;
} micro_ros_uart_transport_t;

typedef struct {
    micro_ros_uart_config_t config;
    micro_ros_uart_transport_t transport;
    micro_ros_servo_handler_t handler;
    TaskHandle_t task_handle;
    bool running;
    bool stop_requested;
} micro_ros_uart_state_t;

typedef struct {
    uint8_t servo_id;
    int value;
    micro_ros_servo_mode_t mode;
} micro_ros_servo_command_t;

static micro_ros_uart_state_t s_state = {0};

static micro_ros_uart_config_t micro_ros_uart_default_config(void) {
    micro_ros_uart_config_t config = {
        .uart_port = MICRO_ROS_UART_DEFAULT_PORT,
        .baud_rate = MICRO_ROS_UART_DEFAULT_BAUD,
        .rx_buffer_size = MICRO_ROS_UART_DEFAULT_RX_BUFFER,
        .tx_buffer_size = MICRO_ROS_UART_DEFAULT_TX_BUFFER,
    };
    return config;
}

static esp_err_t micro_ros_uart_default_servo_handler(uint8_t servo_id, int value, micro_ros_servo_mode_t mode) {
    const float min_us = 500.0f;
    const float max_us = 2500.0f;
    float counts = 0.0f;
    if (mode == MICRO_ROS_SERVO_MODE_ANGLE) {
        double angle = value;
        if (angle < 0.0) angle = 0.0;
        if (angle > 180.0) angle = 180.0;
        float pulse_us = min_us + (float)(angle / 180.0) * (max_us - min_us);
        counts = pulse_us * (float)50 * 4096.0f / 1000000.0f;
    } else {
        counts = (float)value;
    }
    if (counts < 0) counts = 0;
    if (counts > 4095) counts = 4095;
    return pca9685_set_pwm_value(servo_id, (uint16_t)counts);
}

static bool micro_ros_uart_open(struct uxrCustomTransport *transport) {
    micro_ros_uart_transport_t *uart = (micro_ros_uart_transport_t *)transport->args;
    uart_config_t uart_config = {
        .baud_rate = uart->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(uart->uart_port, &uart_config) != ESP_OK) {
        return false;
    }
    if (uart_set_pin(uart->uart_port, CONFIG_MICROROS_UART_TXD, CONFIG_MICROROS_UART_RXD,
                     CONFIG_MICROROS_UART_RTS, CONFIG_MICROROS_UART_CTS) != ESP_OK) {
        return false;
    }
    if (uart_driver_install(uart->uart_port, uart->rx_buffer_size, uart->tx_buffer_size, 0, NULL, 0) != ESP_OK) {
        return false;
    }

    uart_flush_input(uart->uart_port);
    return true;
}

static bool micro_ros_uart_close(struct uxrCustomTransport *transport) {
    micro_ros_uart_transport_t *uart = (micro_ros_uart_transport_t *)transport->args;
    return uart_driver_delete(uart->uart_port) == ESP_OK;
}

static size_t micro_ros_uart_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err) {
    micro_ros_uart_transport_t *uart = (micro_ros_uart_transport_t *)transport->args;
    int written = uart_write_bytes(uart->uart_port, (const char *)buf, len);
    if (written < 0) {
        if (err) {
            *err = 1;
        }
        return 0;
    }
    if (err) {
        *err = 0;
    }
    return (size_t)written;
}

static size_t micro_ros_uart_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
    micro_ros_uart_transport_t *uart = (micro_ros_uart_transport_t *)transport->args;
    int read_len = uart_read_bytes(uart->uart_port, buf, len, timeout / portTICK_PERIOD_MS);
    if (read_len < 0) {
        if (err) {
            *err = 1;
        }
        return 0;
    }
    if (err) {
        *err = 0;
    }
    return (size_t)read_len;
}

static esp_err_t micro_ros_uart_register_transport(void) {
#if !defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    return ESP_ERR_NOT_SUPPORTED;
#else
    s_state.transport.uart_port = (uart_port_t)s_state.config.uart_port;
    s_state.transport.baud_rate = s_state.config.baud_rate;
    s_state.transport.rx_buffer_size = s_state.config.rx_buffer_size;
    s_state.transport.tx_buffer_size = s_state.config.tx_buffer_size;

    rmw_ret_t ret = rmw_uros_set_custom_transport(
        true,
        (void *)&s_state.transport,
        micro_ros_uart_open,
        micro_ros_uart_close,
        micro_ros_uart_write,
        micro_ros_uart_read);

    return (ret == RMW_RET_OK) ? ESP_OK : ESP_FAIL;
#endif
}

static bool micro_ros_parse_servo_command(const std_msgs__msg__Int32MultiArray *msg, micro_ros_servo_command_t *command) {
    if (!msg || !command || msg->data.size < 2 || !msg->data.data) {
        return false;
    }

    int32_t raw_id = msg->data.data[0];
    int32_t raw_value = msg->data.data[1];
    int32_t raw_mode = (msg->data.size >= 3) ? msg->data.data[2] : 0;

    if (raw_id < 0 || raw_id > 255) {
        return false;
    }

    command->servo_id = (uint8_t)raw_id;
    command->value = (int)raw_value;
    command->mode = (raw_mode == MICRO_ROS_SERVO_MODE_ANGLE) ? MICRO_ROS_SERVO_MODE_ANGLE : MICRO_ROS_SERVO_MODE_PWM;
    return true;
}

static void micro_ros_servo_cmd_callback(const void *msgin) {
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    micro_ros_servo_command_t command = {0};

    if (!micro_ros_parse_servo_command(msg, &command)) {
        ESP_LOGW(TAG, "Invalid servo command payload");
        return;
    }

    if (!s_state.handler) {
        ESP_LOGW(TAG, "Servo handler not set");
        return;
    }

    ESP_LOGI(TAG, "Servo cmd id=%u value=%d mode=%s", command.servo_id, command.value,
             (command.mode == MICRO_ROS_SERVO_MODE_ANGLE) ? "angle" : "pwm");
    s_state.handler(command.servo_id, command.value, command.mode);
}

#define RCCHECK(fn) do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
            ESP_LOGE(TAG, "RCL error %d in %s", rc, #fn); \
            goto cleanup; \
        } } while (0)
#define RCSOFTCHECK(fn) do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
            ESP_LOGW(TAG, "RCL soft error %d in %s", rc, #fn); \
        } } while (0)

static void micro_ros_uart_task(void *arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support = {0};
    rcl_node_t node = {0};
    rcl_subscription_t servo_sub = {0};
    rclc_executor_t executor = {0};

    std_msgs__msg__Int32MultiArray servo_cmd_msg;
    int32_t servo_cmd_buffer[3] = {0};
    memset(&servo_cmd_msg, 0, sizeof(servo_cmd_msg));
    servo_cmd_msg.data.data = servo_cmd_buffer;
    servo_cmd_msg.data.size = 0;
    servo_cmd_msg.data.capacity = 3;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, CONFIG_COMM_MICRO_ROS_NODE_NAME, "", &support));
    RCCHECK(rclc_subscription_init_default(
        &servo_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        CONFIG_COMM_MICRO_ROS_SERVO_TOPIC));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &servo_sub, &servo_cmd_msg, &micro_ros_servo_cmd_callback, ON_NEW_DATA));

    ESP_LOGI(TAG, "micro-ROS UART task started (port=%d, baud=%d)",
             s_state.transport.uart_port, s_state.transport.baud_rate);
    s_state.running = true;
    while (!s_state.stop_requested) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(MICRO_ROS_UART_EXECUTOR_TIMEOUT_MS)));
        vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_UART_SPIN_PERIOD_MS));
    }

cleanup:
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_subscription_fini(&servo_sub, &node));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    s_state.running = false;
    s_state.task_handle = NULL;
    s_state.stop_requested = false;
    vTaskDelete(NULL);
}

esp_err_t micro_ros_uart_start(const micro_ros_uart_config_t *config, micro_ros_servo_handler_t handler) {
#ifndef CONFIG_COMM_MICRO_ROS_UART_ENABLE
    ESP_LOGW(TAG, "micro-ROS UART disabled by config");
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (s_state.running) {
        ESP_LOGW(TAG, "micro-ROS UART already running");
        return ESP_ERR_INVALID_STATE;
    }

    s_state.config = config ? *config : micro_ros_uart_default_config();
    s_state.handler = handler ? handler : micro_ros_uart_default_servo_handler;
    s_state.stop_requested = false;

    if (micro_ros_uart_register_transport() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register micro-ROS UART transport");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting micro-ROS UART (port=%d, baud=%d, rx_buf=%d, tx_buf=%d)",
             s_state.config.uart_port, s_state.config.baud_rate,
             s_state.config.rx_buffer_size, s_state.config.tx_buffer_size);

    BaseType_t task_ok = xTaskCreate(
        micro_ros_uart_task,
        "micro_ros_uart",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        &s_state.task_handle);

    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create micro-ROS UART task");
        s_state.running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "micro-ROS UART initialization started; waiting for task ready");

    return ESP_OK;
}

void micro_ros_uart_stop(void) {
    if (!s_state.running) {
        return;
    }

    ESP_LOGI(TAG, "Stopping micro-ROS UART task");
    s_state.stop_requested = true;
}

bool micro_ros_uart_is_running(void) {
    return s_state.running;
}

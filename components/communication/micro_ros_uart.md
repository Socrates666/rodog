# micro-ROS UART Servo Module

该模块通过 UART 自定义传输将设备注册为 micro-ROS 节点，并提供舵机控制接口。

## 功能概览

- 节点名称: `rodog_uart_node`
- 订阅话题: `/rodog/servo_cmd`
- 消息类型: `std_msgs/msg/Int32MultiArray`
- 数据格式:
  - `data[0]`: `servo_id` (0-255)
  - `data[1]`: `value` (PWM 或角度)
  - `data[2]`: `mode` (0 = PWM, 1 = ANGLE，可省略)

## 使用方式

1. 在应用中调用启动接口：

```c
#include "micro_ros_uart.h"

static esp_err_t servo_command_handler(uint8_t servo_id, int value, micro_ros_servo_mode_t mode) {
    if (mode == MICRO_ROS_SERVO_MODE_ANGLE) {
        return servo_set_angle(servo_id, (double)value);
    }
    return servo_set_pwm(servo_id, value);
}

void app_main(void) {
    micro_ros_uart_config_t cfg = {
        .uart_port = UART_NUM_0,
        .baud_rate = 115200,
        .rx_buffer_size = 1024,
        .tx_buffer_size = 0,
    };

    micro_ros_uart_start(&cfg, servo_command_handler);
}
```

2. 确保 `menuconfig` 中启用 micro-ROS UART 传输：

- `micro-ROS Settings` → `Micro XRCE-DDS over UART`
- `micro-ROS Settings` → `Enable micro-ROS UART module`
- `micro-ROS Settings` → UART 端口与波特率等参数（可选）
- 配置 UART 引脚 `MICROROS_UART_TXD/RXD/RTS/CTS`
- 使能自定义传输 `-DRMW_UXRCE_TRANSPORT=custom`

3. 启动 micro-ROS Agent：

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

## 备注

- 若未设置 `servo` 回调，模块将使用默认舵机处理逻辑。
- 可通过 `micro_ros_uart_stop()` 停止节点任务。

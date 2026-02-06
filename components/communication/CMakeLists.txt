if(CONFIG_COMM_MICRO_ROS_UART_ENABLE)
  set(srcs "micro_ros_uart.c")
endif()

idf_component_register(
    SRCS "web_server.c" ${srcs}
    INCLUDE_DIRS "."
    REQUIRES esp_http_server esp_wifi esp_event esp_netif esp_timer lwip esp_driver_gpio esp_driver_uart nvs_flash control micro_ros_espidf_component
)

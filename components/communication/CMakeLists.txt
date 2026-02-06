idf_component_register(
  SRCS "web_server.c" "display.c"
  INCLUDE_DIRS "."
  REQUIRES esp_http_server esp_wifi esp_event esp_netif esp_timer lwip esp_driver_gpio esp_driver_uart nvs_flash control common driver
)

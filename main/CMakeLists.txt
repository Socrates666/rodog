idf_component_register(SRCS "rodog_main.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_http_server esp_timer esp_netif 
                                 esp_driver_i2c lwip esp_driver_gpio
                                 common communication control drivers algorithms
                    INCLUDE_DIRS "."
                                )

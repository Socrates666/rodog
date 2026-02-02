file(GLOB SOURCE_FILES
    "*.c"
    "components/*.c"
    "components/*.cpp"
    "components/communication/*.c"
    "components/control/*.c"
    "components/drivers/*.c"

)



idf_component_register(SRCS ${SOURCE_FILES}
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_http_server esp_timer esp_netif 
                                 esp_driver_i2c lwip esp_driver_gpio
                    INCLUDE_DIRS "."
                                 "components"
                                 "components/communication"
                                 "components/control"
                                 "components/drivers"
                                )

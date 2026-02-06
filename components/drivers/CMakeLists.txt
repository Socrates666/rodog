idf_component_register(SRCS "icm20948.c" "ina219.c" "pca9685.c" "ssd1306.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_i2c)
# Display component CMakeLists.txt

idf_component_register(
    SRCS "display.c"
    INCLUDE_DIRS "."
    REQUIRES freertos driver
    PRIV_REQUIRES ssd1306 board
)
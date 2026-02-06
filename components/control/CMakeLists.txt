idf_component_register(
    SRCS "leg.c" "calibration.c"
    INCLUDE_DIRS "."
    REQUIRES drivers nvs_flash common
)

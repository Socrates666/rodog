# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wavego_display)

# Add components directory
set(EXTRA_COMPONENT_DIRS communication/display)

# Main executable
idf_component_register(SRCS "main/main.c"
                    INCLUDE_DIRS "main"
                    REQUIRES freertos nvs_flash esp_netif esp_event 
                    protocol_examples_common)
# StampFly HAL Project
# ESP-IDF v5.4.1 Compatible
# Target: ESP32-S3

cmake_minimum_required(VERSION 3.16)

# ESP32-S3 target support
set(SUPPORTED_TARGETS esp32s3)

# Project name
set(PROJECT_NAME stampfly_hal)

# Extra component directories
set(EXTRA_COMPONENT_DIRS
    components/stampfly_hal
    components/stampfly_sensors  
    components/stampfly_utils
)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project definition
project(${PROJECT_NAME})
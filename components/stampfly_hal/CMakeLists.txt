# StampFly HAL Component
# Core Hardware Abstraction Layer

idf_component_register(
    SRCS 
        "src/stampfly_hal_base.cpp"
        "src/uart_hal.cpp"
        "src/spi_hal.cpp"
        "src/i2c_hal.cpp"
        # "src/gpio_hal.cpp"
        # "src/timer_hal.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_driver_gpio
        esp_driver_uart
        esp_driver_i2c
        esp_driver_spi
        esp_timer
        freertos
        log
        nvs_flash
        vfs
)
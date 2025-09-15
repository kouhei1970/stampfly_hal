# Main application CMakeLists.txt
# StampFly HAL Project

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_timer
        esp_driver_gpio
        esp_driver_uart
        esp_driver_i2c
        esp_driver_spi
        freertos
        log
        nvs_flash
        spi_flash
        stampfly_hal
        stampfly_rgbled
    PRIV_REQUIRES
        esp_driver_usb_serial_jtag
        vfs
)
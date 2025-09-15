# StampFly HAL Component
# Unified Hardware Abstraction Layer

idf_component_register(
    SRCS
        # 基底クラス
        "src/hal_base.cpp"

        # 通信HAL
        "src/uart.cpp"
        "src/spi.cpp"
        "src/i2c.cpp"
        "src/gpio.cpp"

        # 周辺機器HAL
        "src/rgbled.cpp"
        "src/stamps3_led.cpp"

        # センサーHAL群（今後追加）
        "src/bmi270.cpp"
        "src/bmm150.cpp"
        "src/bmp280.cpp"
        "src/vl53l3cx.cpp"
        "src/pmw3901.cpp"
        "src/ina3221.cpp"

        # ユーティリティ
        "src/stampfly_memory.cpp"

    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_gpio
        esp_driver_uart
        esp_driver_i2c
        esp_driver_spi
        esp_driver_rmt
        esp_timer
        freertos
        log
        nvs_flash
        vfs
        led_strip
)
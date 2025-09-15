idf_component_register(
    SRCS "stampfly_rgbled.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_rmt led_strip
)
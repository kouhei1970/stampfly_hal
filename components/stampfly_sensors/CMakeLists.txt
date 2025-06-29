# StampFly Sensors Component
# Sensor drivers and management

idf_component_register(
    SRCS 
        # "src/sensor_base.cpp"
        # "src/bmi270_sensor.cpp"
        # "src/pmw3901_sensor.cpp"
        # "src/bmm150_sensor.cpp"
        # "src/bmp280_sensor.cpp"
        # "src/vl53lx_sensor.cpp"
        # "src/ina3221_sensor.cpp"
        # "src/sensor_manager.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        stampfly_hal
        log
)
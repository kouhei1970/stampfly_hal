# StampFly Utilities Component
# Math, filters, calibration utilities

idf_component_register(
    SRCS 
        # "src/math_utils.cpp"
        # "src/filter_utils.cpp"
        # "src/calibration.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        log
        freertos
)
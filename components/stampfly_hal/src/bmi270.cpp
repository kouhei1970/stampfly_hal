#include "bmi270.h"

namespace stampfly_hal {

BMI270::BMI270() : HALBase("BMI270") {
}

esp_err_t BMI270::init() {
    log(ESP_LOG_INFO, "BMI270 6-axis IMU init - placeholder implementation");
    set_initialized(true);
    return ESP_OK;
}

esp_err_t BMI270::configure() {
    return ESP_OK;
}

esp_err_t BMI270::enable() {
    set_enabled(true);
    return ESP_OK;
}

esp_err_t BMI270::disable() {
    set_enabled(false);
    return ESP_OK;
}

esp_err_t BMI270::reset() {
    set_initialized(false);
    set_enabled(false);
    return init();
}

esp_err_t BMI270::read_accel(float* x, float* y, float* z) {
    log(ESP_LOG_INFO, "read_accel - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t BMI270::read_gyro(float* x, float* y, float* z) {
    log(ESP_LOG_INFO, "read_gyro - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t BMI270::read_temperature(float* temp) {
    log(ESP_LOG_INFO, "read_temperature - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

} // namespace stampfly_hal
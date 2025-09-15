#include "bmp280.h"

namespace stampfly_hal {

BMP280::BMP280() : HALBase("BMP280") {
}

esp_err_t BMP280::init() {
    log(ESP_LOG_INFO, "BMP280 barometric pressure sensor init - placeholder implementation");
    set_initialized(true);
    return ESP_OK;
}

esp_err_t BMP280::configure() {
    return ESP_OK;
}

esp_err_t BMP280::enable() {
    set_enabled(true);
    return ESP_OK;
}

esp_err_t BMP280::disable() {
    set_enabled(false);
    return ESP_OK;
}

esp_err_t BMP280::reset() {
    set_initialized(false);
    set_enabled(false);
    return init();
}

esp_err_t BMP280::read_pressure(float* pressure) {
    log(ESP_LOG_INFO, "read_pressure - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t BMP280::read_temperature(float* temperature) {
    log(ESP_LOG_INFO, "read_temperature - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t BMP280::read_altitude(float* altitude) {
    log(ESP_LOG_INFO, "read_altitude - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

} // namespace stampfly_hal
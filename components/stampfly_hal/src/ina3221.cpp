#include "ina3221.h"

namespace stampfly_hal {

INA3221::INA3221() : HALBase("INA3221") {
}

esp_err_t INA3221::init() {
    log(ESP_LOG_INFO, "INA3221 3-channel power monitor init - placeholder implementation");
    set_initialized(true);
    return ESP_OK;
}

esp_err_t INA3221::configure() {
    return ESP_OK;
}

esp_err_t INA3221::enable() {
    set_enabled(true);
    return ESP_OK;
}

esp_err_t INA3221::disable() {
    set_enabled(false);
    return ESP_OK;
}

esp_err_t INA3221::reset() {
    set_initialized(false);
    set_enabled(false);
    return init();
}

esp_err_t INA3221::read_bus_voltage(uint8_t channel, float* voltage) {
    log(ESP_LOG_INFO, "read_bus_voltage channel %d - not implemented", channel);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t INA3221::read_shunt_voltage(uint8_t channel, float* voltage) {
    log(ESP_LOG_INFO, "read_shunt_voltage channel %d - not implemented", channel);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t INA3221::read_current(uint8_t channel, float* current) {
    log(ESP_LOG_INFO, "read_current channel %d - not implemented", channel);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t INA3221::read_power(uint8_t channel, float* power) {
    log(ESP_LOG_INFO, "read_power channel %d - not implemented", channel);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t INA3221::set_shunt_resistance(uint8_t channel, float resistance_ohm) {
    log(ESP_LOG_INFO, "set_shunt_resistance channel %d resistance %.3f ohm - not implemented", channel, resistance_ohm);
    return ESP_ERR_NOT_SUPPORTED;
}

} // namespace stampfly_hal
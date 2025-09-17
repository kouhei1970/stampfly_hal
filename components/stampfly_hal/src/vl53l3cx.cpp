#include "vl53l3cx.h"

namespace stampfly_hal {

static constexpr const char* TAG = "VL53L3CX";

VL53L3CX::VL53L3CX() : HALBase("VL53L3CX") {
}

esp_err_t VL53L3CX::init() {
    ESP_LOGI(TAG, "VL53L3CX ToF distance sensor init - placeholder implementation");
    set_initialized(true);
    return ESP_OK;
}

esp_err_t VL53L3CX::configure() {
    return ESP_OK;
}

esp_err_t VL53L3CX::enable() {
    set_enabled(true);
    return ESP_OK;
}

esp_err_t VL53L3CX::disable() {
    set_enabled(false);
    return ESP_OK;
}

esp_err_t VL53L3CX::reset() {
    set_initialized(false);
    set_enabled(false);
    return init();
}

esp_err_t VL53L3CX::read_distance(uint16_t* distance_mm) {
    ESP_LOGI(TAG, "read_distance - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t VL53L3CX::set_timing_budget(uint32_t budget_us) {
    ESP_LOGI(TAG, "set_timing_budget %lu us - not implemented", budget_us);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t VL53L3CX::set_distance_mode(uint8_t mode) {
    ESP_LOGI(TAG, "set_distance_mode %d - not implemented", mode);
    return ESP_ERR_NOT_SUPPORTED;
}

} // namespace stampfly_hal
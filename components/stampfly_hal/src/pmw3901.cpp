#include "pmw3901.h"

namespace stampfly_hal {

static constexpr const char* TAG = "PMW3901";

PMW3901::PMW3901() : HALBase("PMW3901") {
}

esp_err_t PMW3901::init() {
    ESP_LOGI(TAG, "PMW3901 optical flow sensor init - placeholder implementation");
    set_initialized(true);
    return ESP_OK;
}

esp_err_t PMW3901::configure() {
    return ESP_OK;
}

esp_err_t PMW3901::enable() {
    set_enabled(true);
    return ESP_OK;
}

esp_err_t PMW3901::disable() {
    set_enabled(false);
    return ESP_OK;
}

esp_err_t PMW3901::reset() {
    set_initialized(false);
    set_enabled(false);
    return init();
}

esp_err_t PMW3901::read_motion(int16_t* delta_x, int16_t* delta_y) {
    ESP_LOGI(TAG, "read_motion - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t PMW3901::read_surface_quality(uint8_t* quality) {
    ESP_LOGI(TAG, "read_surface_quality - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t PMW3901::set_resolution(uint16_t cpi) {
    ESP_LOGI(TAG, "set_resolution %d CPI - not implemented", cpi);
    return ESP_ERR_NOT_SUPPORTED;
}

} // namespace stampfly_hal
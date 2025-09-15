#include "pmw3901.h"

namespace stampfly_hal {

PMW3901::PMW3901() : HALBase("PMW3901") {
}

esp_err_t PMW3901::init() {
    log(ESP_LOG_INFO, "PMW3901 optical flow sensor init - placeholder implementation");
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
    log(ESP_LOG_INFO, "read_motion - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t PMW3901::read_surface_quality(uint8_t* quality) {
    log(ESP_LOG_INFO, "read_surface_quality - not implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t PMW3901::set_resolution(uint16_t cpi) {
    log(ESP_LOG_INFO, "set_resolution %d CPI - not implemented", cpi);
    return ESP_ERR_NOT_SUPPORTED;
}

} // namespace stampfly_hal
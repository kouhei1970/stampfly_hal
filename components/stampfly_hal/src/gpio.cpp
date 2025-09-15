#include "gpio.h"

namespace stampfly_hal {

Gpio::Gpio() : HALBase("GPIO") {
}

esp_err_t Gpio::init() {
    // プレースホルダー実装
    log(ESP_LOG_INFO, "GPIO HAL init - placeholder implementation");
    set_initialized(true);
    return ESP_OK;
}

esp_err_t Gpio::configure() {
    // プレースホルダー実装
    return ESP_OK;
}

esp_err_t Gpio::enable() {
    // プレースホルダー実装
    set_enabled(true);
    return ESP_OK;
}

esp_err_t Gpio::disable() {
    // プレースホルダー実装
    set_enabled(false);
    return ESP_OK;
}

esp_err_t Gpio::reset() {
    // プレースホルダー実装
    set_initialized(false);
    set_enabled(false);
    return init();
}

esp_err_t Gpio::set_pin_mode(gpio_num_t pin, gpio_mode_t mode) {
    // プレースホルダー実装
    log(ESP_LOG_INFO, "set_pin_mode GPIO%d mode=%d - not implemented", pin, mode);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t Gpio::digital_write(gpio_num_t pin, uint32_t level) {
    // プレースホルダー実装
    log(ESP_LOG_INFO, "digital_write GPIO%d level=%lu - not implemented", pin, level);
    return ESP_ERR_NOT_SUPPORTED;
}

int Gpio::digital_read(gpio_num_t pin) {
    // プレースホルダー実装
    log(ESP_LOG_INFO, "digital_read GPIO%d - not implemented", pin);
    return -1;
}

} // namespace stampfly_hal
#include "hal_base.h"
#include <stdio.h>

namespace stampfly_hal {

HALBase::HALBase(const char* name) : name_(name) {
}

void HALBase::print_status() const {
    ESP_LOGI(name_, "Status: initialized=%d, enabled=%d, last_error=0x%x",
             initialized_, enabled_, last_error_);
}

void HALBase::log(esp_log_level_t level, const char* format, ...) const {
    va_list args;
    va_start(args, format);
    esp_log_writev(level, name_, format, args);
    va_end(args);
}

esp_err_t HALBase::set_error(esp_err_t error) {
    last_error_ = error;
    if (error != ESP_OK) {
        ESP_LOGE(name_, "Error occurred: 0x%x (%s)", error, esp_err_to_name(error));
    }
    return error;
}

} // namespace stampfly_hal
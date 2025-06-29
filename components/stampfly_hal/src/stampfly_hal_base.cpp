/*
 * StampFly HAL Base Class Implementation
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#include "stampfly_hal_base.h"
#include <cstdarg>

namespace stampfly_hal {

HALBase::HALBase(const char* tag) 
    : tag_(tag), initialized_(false), enabled_(false), last_error_(ESP_OK) 
{
}

bool HALBase::is_initialized() const 
{
    return initialized_;
}

bool HALBase::is_enabled() const 
{
    return enabled_;
}

esp_err_t HALBase::enable() 
{
    if (!initialized_) {
        return set_error(ESP_ERR_INVALID_STATE);
    }
    
    enabled_ = true;
    log(ESP_LOG_INFO, "HAL enabled");
    return ESP_OK;
}

esp_err_t HALBase::disable() 
{
    enabled_ = false;
    log(ESP_LOG_INFO, "HAL disabled");
    return ESP_OK;
}

esp_err_t HALBase::reset() 
{
    log(ESP_LOG_INFO, "HAL reset requested");
    
    // 無効化
    esp_err_t ret = disable();
    if (ret != ESP_OK) {
        return set_error(ret);
    }
    
    // 初期化状態をリセット
    initialized_ = false;
    last_error_ = ESP_OK;
    
    log(ESP_LOG_INFO, "HAL reset completed");
    return ESP_OK;
}

void HALBase::log(esp_log_level_t level, const char* format, ...) const 
{
    va_list args;
    va_start(args, format);
    esp_log_writev(level, tag_, format, args);
    va_end(args);
}

esp_err_t HALBase::set_error(esp_err_t error) 
{
    last_error_ = error;
    if (error != ESP_OK) {
        log(ESP_LOG_ERROR, "Error occurred: %s", esp_err_to_name(error));
    }
    return error;
}

void HALBase::set_initialized(bool initialized) 
{
    initialized_ = initialized;
    log(ESP_LOG_INFO, "Initialization state: %s", initialized ? "true" : "false");
}

void HALBase::set_enabled(bool enabled) 
{
    enabled_ = enabled;
    log(ESP_LOG_INFO, "Enabled state: %s", enabled ? "true" : "false");
}

} // namespace stampfly_hal
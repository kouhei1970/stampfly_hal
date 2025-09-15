#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <stdarg.h>

namespace stampfly_hal {

/**
 * HAL基底クラス
 * 全てのHALコンポーネントが継承する統一インターフェース
 */
class HALBase {
public:
    explicit HALBase(const char* name);
    virtual ~HALBase() = default;

    // 必須ライフサイクル
    virtual esp_err_t init() = 0;
    virtual esp_err_t configure() = 0;
    virtual esp_err_t enable() = 0;
    virtual esp_err_t disable() = 0;
    virtual esp_err_t reset() = 0;

    // 状態管理
    bool is_initialized() const { return initialized_; }
    bool is_enabled() const { return enabled_; }
    esp_err_t get_last_error() const { return last_error_; }
    const char* get_name() const { return name_; }

    // デバッグ支援
    virtual void print_status() const;
    void log(esp_log_level_t level, const char* format, ...) const;

protected:
    esp_err_t set_error(esp_err_t error);
    void set_initialized(bool state) { initialized_ = state; }
    void set_enabled(bool state) { enabled_ = state; }

private:
    const char* name_;
    bool initialized_ = false;
    bool enabled_ = false;
    esp_err_t last_error_ = ESP_OK;
};

} // namespace stampfly_hal
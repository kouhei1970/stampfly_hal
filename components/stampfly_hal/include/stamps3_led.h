#pragma once

#include "hal_base.h"
#include "led_strip.h"
#include <stdint.h>

namespace stampfly_hal {

/**
 * @brief StampS3オンボードLED用HALクラス
 * GPIO21のWS2812B 1個専用制御クラス
 */
class StampS3Led : public HALBase {
public:
    StampS3Led();
    ~StampS3Led();

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override { return ESP_OK; }
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // LED制御機能
    esp_err_t set_color(uint32_t color);
    esp_err_t set_rgb(uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t set_brightness(uint8_t brightness);
    esp_err_t clear();
    esp_err_t refresh();

    // 定義済み色設定
    esp_err_t set_red() { return set_color(0xFF0000); }
    esp_err_t set_green() { return set_color(0x00FF00); }
    esp_err_t set_blue() { return set_color(0x0000FF); }
    esp_err_t set_white() { return set_color(0xFFFFFF); }
    esp_err_t set_yellow() { return set_color(0xFFFF00); }
    esp_err_t set_purple() { return set_color(0xFF00FF); }
    esp_err_t set_cyan() { return set_color(0x00FFFF); }
    esp_err_t set_orange() { return set_color(0xFF9933); }
    esp_err_t set_pink() { return set_color(0xDC669B); }
    esp_err_t off() { return clear(); }

    // 状態取得
    uint8_t get_brightness() const { return brightness_; }
    uint32_t get_current_color() const { return current_color_; }

    // デバッグ
    void print_status() const override;

private:
    led_strip_handle_t led_strip_;
    uint8_t brightness_;
    uint32_t current_color_;

    // StampS3固定仕様
    static constexpr int STAMPS3_LED_GPIO = 21;
    static constexpr int LED_COUNT = 1;
    static constexpr int DEFAULT_BRIGHTNESS = 32;
    static constexpr int RMT_RESOLUTION_HZ = 10 * 1000 * 1000;

    esp_err_t apply_brightness(uint32_t color, uint8_t& r, uint8_t& g, uint8_t& b);
};

} // namespace stampfly_hal
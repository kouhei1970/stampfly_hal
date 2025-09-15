#pragma once

#include "hal_base.h"
#include "led_strip.h"
#include <stdint.h>

namespace stampfly_hal {

/**
 * @brief StampFly定義済み色
 */
enum class Color : uint32_t {
    OFF     = 0x000000,  // 消灯
    WHITE   = 0xFFFFFF,  // 白
    RED     = 0xFF0000,  // 赤
    GREEN   = 0x00FF00,  // 緑
    BLUE    = 0x0000FF,  // 青
    YELLOW  = 0xFFFF00,  // 黄
    PURPLE  = 0xFF00FF,  // 紫
    CYAN    = 0x00FFFF,  // シアン
    ORANGE  = 0xFF9933,  // オレンジ（フリップモード用）
    PINK    = 0xDC669B,  // ピンク（アクロモード用）
    LOWBATT = 0x18EBF9,  // 水色（バッテリー低下用）
};

/**
 * @brief RGBLED HALクラス
 * WS2812B制御をオブジェクト指向で実装
 */
class RgbLed : public HALBase {
public:
    RgbLed();
    ~RgbLed();

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override { return ESP_OK; }
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // RGBLED固有機能
    esp_err_t set_color(uint32_t color);
    esp_err_t set_color(Color color);
    esp_err_t set_rgb(uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t set_brightness(uint8_t brightness);
    esp_err_t clear();
    esp_err_t refresh();

    // 状態取得
    uint8_t get_brightness() const { return brightness_; }
    uint32_t get_current_color() const { return current_color_; }

    // デバッグ
    void print_status() const override;

private:
    led_strip_handle_t led_strip_;
    uint8_t brightness_;
    uint32_t current_color_;
    int led_gpio_;

    static constexpr int LED_COUNT = 2;
    static constexpr int DEFAULT_BRIGHTNESS = 15;
    static constexpr int RMT_RESOLUTION_HZ = 10 * 1000 * 1000;

    // 試行GPIO候補
    static constexpr int CANDIDATE_GPIOS[] = {39, 48, 21, 38, 47};
    static constexpr int NUM_CANDIDATES = sizeof(CANDIDATE_GPIOS) / sizeof(CANDIDATE_GPIOS[0]);

    esp_err_t apply_brightness(uint32_t color, uint8_t& r, uint8_t& g, uint8_t& b);
};

} // namespace stampfly_hal
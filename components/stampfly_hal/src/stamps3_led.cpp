#include "stamps3_led.h"

namespace stampfly_hal {

StampS3Led::StampS3Led()
    : HALBase("StampS3LED"),
      led_strip_(nullptr),
      brightness_(DEFAULT_BRIGHTNESS),
      current_color_(0) {
}

StampS3Led::~StampS3Led() {
    if (is_initialized()) {
        disable();
    }
}

esp_err_t StampS3Led::init() {
    if (is_initialized()) {
        log(ESP_LOG_WARN, "Already initialized");
        return ESP_OK;
    }

    log(ESP_LOG_INFO, "Initializing StampS3 onboard LED (GPIO%d)", STAMPS3_LED_GPIO);

    // LED Strip設定（StampS3固定仕様）
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = STAMPS3_LED_GPIO;
    strip_config.max_leds = LED_COUNT;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.flags.invert_out = false;

    // RMT設定
    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = RMT_RESOLUTION_HZ;
    rmt_config.flags.with_dma = false;

    // LED Strip初期化
    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to initialize StampS3 LED: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    set_initialized(true);
    log(ESP_LOG_INFO, "StampS3 LED initialized successfully");

    // 初期設定として青色点灯（StampS3識別用）
    ret = set_blue();
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to set initial color: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}

esp_err_t StampS3Led::enable() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }
    set_enabled(true);
    return refresh();
}

esp_err_t StampS3Led::disable() {
    if (!is_initialized()) {
        return ESP_OK;
    }

    // 消灯してから終了
    clear();

    if (led_strip_) {
        esp_err_t ret = led_strip_del(led_strip_);
        if (ret != ESP_OK) {
            log(ESP_LOG_ERROR, "Failed to delete LED strip: %s", esp_err_to_name(ret));
            return set_error(ret);
        }
        led_strip_ = nullptr;
    }

    set_initialized(false);
    set_enabled(false);
    log(ESP_LOG_INFO, "StampS3 LED disabled");
    return ESP_OK;
}

esp_err_t StampS3Led::reset() {
    disable();
    vTaskDelay(pdMS_TO_TICKS(10));
    return init();
}

esp_err_t StampS3Led::set_color(uint32_t color) {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    return set_rgb(r, g, b);
}

esp_err_t StampS3Led::set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    // 輝度適用
    uint8_t r, g, b;
    uint32_t color = (red << 16) | (green << 8) | blue;
    apply_brightness(color, r, g, b);

    // StampS3の単一LEDに色を設定
    esp_err_t ret = led_strip_set_pixel(led_strip_, 0, r, g, b);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to set LED color: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    current_color_ = color;
    return refresh();
}

esp_err_t StampS3Led::set_brightness(uint8_t brightness) {
    brightness_ = brightness;
    log(ESP_LOG_INFO, "StampS3 LED brightness set to %d/255", brightness);

    // 現在の色を再適用して輝度を反映
    if (is_initialized() && current_color_ != 0) {
        return set_color(current_color_);
    }
    return ESP_OK;
}

esp_err_t StampS3Led::clear() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    esp_err_t ret = led_strip_clear(led_strip_);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to clear StampS3 LED: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    current_color_ = 0;
    return refresh();
}

esp_err_t StampS3Led::refresh() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    esp_err_t ret = led_strip_refresh(led_strip_);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to refresh StampS3 LED: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    return ESP_OK;
}

esp_err_t StampS3Led::apply_brightness(uint32_t color, uint8_t& r, uint8_t& g, uint8_t& b) {
    uint8_t red = (color >> 16) & 0xFF;
    uint8_t green = (color >> 8) & 0xFF;
    uint8_t blue = color & 0xFF;

    // 輝度適用（0-255スケール）
    r = (red * brightness_) / 255;
    g = (green * brightness_) / 255;
    b = (blue * brightness_) / 255;

    return ESP_OK;
}

void StampS3Led::print_status() const {
    HALBase::print_status();
    log(ESP_LOG_INFO, "GPIO: %d, Brightness: %d/255, Color: 0x%06lX",
        STAMPS3_LED_GPIO, brightness_, current_color_);
}

} // namespace stampfly_hal
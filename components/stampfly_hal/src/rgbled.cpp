#include "rgbled.h"
#include <cstring>

namespace stampfly_hal {

static constexpr const char* TAG = "RgbLed";

constexpr int RgbLed::CANDIDATE_GPIOS[];

RgbLed::RgbLed()
    : HALBase("RGBLED"),
      led_strip_(nullptr),
      brightness_(DEFAULT_BRIGHTNESS),
      current_color_(0),
      led_colors_{0, 0},
      led_gpio_(-1) {
}

RgbLed::~RgbLed() {
    if (is_initialized()) {
        disable();
    }
}

esp_err_t RgbLed::init() {
    if (is_initialized()) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing StampFly RGBLED (%d LEDs)", LED_COUNT);

    // 複数のGPIOを試行
    for (int i = 0; i < NUM_CANDIDATES; i++) {
        int gpio = CANDIDATE_GPIOS[i];
        ESP_LOGI(TAG, "Trying GPIO%d for RGBLED...", gpio);

        // LED Strip設定
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = gpio;
        strip_config.max_leds = LED_COUNT;
        strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
        strip_config.led_model = LED_MODEL_WS2812;
        strip_config.flags.invert_out = false;

        // RMT設定
        led_strip_rmt_config_t rmt_config = {};
        rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
        rmt_config.resolution_hz = RMT_RESOLUTION_HZ;
        rmt_config.flags.with_dma = false;

        // LED Strip初期化試行
        esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_);
        if (ret == ESP_OK) {
            led_gpio_ = gpio;
            set_initialized(true);
            ESP_LOGI(TAG, "RGBLED initialized successfully on GPIO%d", gpio);

            // 初期設定として白色点灯
            ret = set_color(Color::WHITE);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set initial color: %s", esp_err_to_name(ret));
            }
            return ESP_OK;
        } else {
            ESP_LOGD(TAG, "GPIO%d failed: %s (0x%x)", gpio, esp_err_to_name(ret), ret);
        }
    }

    ESP_LOGE(TAG, "Failed to initialize RGBLED on any candidate GPIO");
    return set_error(ESP_FAIL);
}

esp_err_t RgbLed::enable() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }
    set_enabled(true);
    return refresh();
}

esp_err_t RgbLed::disable() {
    if (!is_initialized()) {
        return ESP_OK;
    }

    // 消灯してから終了
    clear();

    if (led_strip_) {
        esp_err_t ret = led_strip_del(led_strip_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete LED strip: %s", esp_err_to_name(ret));
            return set_error(ret);
        }
        led_strip_ = nullptr;
    }

    set_initialized(false);
    set_enabled(false);
    ESP_LOGI(TAG, "RGBLED disabled");
    return ESP_OK;
}

esp_err_t RgbLed::reset() {
    disable();
    vTaskDelay(pdMS_TO_TICKS(10));
    return init();
}

esp_err_t RgbLed::set_color(uint32_t color) {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    return set_rgb(r, g, b);
}

esp_err_t RgbLed::set_color(Color color) {
    return set_color(static_cast<uint32_t>(color));
}

esp_err_t RgbLed::set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    // 輝度適用
    uint8_t r, g, b;
    uint32_t color = (red << 16) | (green << 8) | blue;
    apply_brightness(color, r, g, b);

    // 全LEDに同じ色を設定
    for (int i = 0; i < LED_COUNT; i++) {
        esp_err_t ret = led_strip_set_pixel(led_strip_, i, r, g, b);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set pixel %d: %s", i, esp_err_to_name(ret));
            return set_error(ret);
        }
        led_colors_[i] = color;  // 各LEDの色を記録
    }

    current_color_ = color;
    return refresh();
}

esp_err_t RgbLed::set_brightness(uint8_t brightness) {
    brightness_ = brightness;
    ESP_LOGI(TAG, "Brightness set to %d/255", brightness);

    // 現在の色を再適用して輝度を反映
    if (is_initialized() && current_color_ != 0) {
        return set_color(current_color_);
    }
    return ESP_OK;
}

esp_err_t RgbLed::clear() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    esp_err_t ret = led_strip_clear(led_strip_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear LEDs: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    current_color_ = 0;
    for (int i = 0; i < LED_COUNT; i++) {
        led_colors_[i] = 0;
    }
    return refresh();
}

esp_err_t RgbLed::refresh() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    esp_err_t ret = led_strip_refresh(led_strip_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh LEDs: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    return ESP_OK;
}

esp_err_t RgbLed::apply_brightness(uint32_t color, uint8_t& r, uint8_t& g, uint8_t& b) {
    uint8_t red = (color >> 16) & 0xFF;
    uint8_t green = (color >> 8) & 0xFF;
    uint8_t blue = color & 0xFF;

    // 輝度適用（0-255スケール）
    r = (red * brightness_) / 255;
    g = (green * brightness_) / 255;
    b = (blue * brightness_) / 255;

    return ESP_OK;
}

esp_err_t RgbLed::set_led_color(int led_index, uint32_t color) {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    if (led_index < 0 || led_index >= LED_COUNT) {
        ESP_LOGE(TAG, "Invalid LED index: %d (must be 0-%d)", led_index, LED_COUNT-1);
        return set_error(ESP_ERR_INVALID_ARG);
    }

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    return set_led_rgb(led_index, r, g, b);
}

esp_err_t RgbLed::set_led_color(int led_index, Color color) {
    return set_led_color(led_index, static_cast<uint32_t>(color));
}

esp_err_t RgbLed::set_led_rgb(int led_index, uint8_t red, uint8_t green, uint8_t blue) {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    if (led_index < 0 || led_index >= LED_COUNT) {
        ESP_LOGE(TAG, "Invalid LED index: %d (must be 0-%d)", led_index, LED_COUNT-1);
        return set_error(ESP_ERR_INVALID_ARG);
    }

    // 輝度適用
    uint8_t r, g, b;
    uint32_t color = (red << 16) | (green << 8) | blue;
    apply_brightness(color, r, g, b);

    // 指定されたLEDのみ設定
    esp_err_t ret = led_strip_set_pixel(led_strip_, led_index, r, g, b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pixel %d: %s", led_index, esp_err_to_name(ret));
        return set_error(ret);
    }

    led_colors_[led_index] = color;
    return refresh();
}

esp_err_t RgbLed::clear_led(int led_index) {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    if (led_index < 0 || led_index >= LED_COUNT) {
        ESP_LOGE(TAG, "Invalid LED index: %d (must be 0-%d)", led_index, LED_COUNT-1);
        return set_error(ESP_ERR_INVALID_ARG);
    }

    esp_err_t ret = led_strip_set_pixel(led_strip_, led_index, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear pixel %d: %s", led_index, esp_err_to_name(ret));
        return set_error(ret);
    }

    led_colors_[led_index] = 0;
    return refresh();
}

uint32_t RgbLed::get_led_color(int led_index) const {
    if (led_index < 0 || led_index >= LED_COUNT) {
        return 0;
    }
    return led_colors_[led_index];
}

void RgbLed::print_status() const {
    HALBase::print_status();
    ESP_LOGI(TAG, "GPIO: %d, Brightness: %d/255, Color: 0x%06lX",
        led_gpio_, brightness_, current_color_);
    ESP_LOGI(TAG, "LED0: 0x%06lX, LED1: 0x%06lX", led_colors_[0], led_colors_[1]);
}

} // namespace stampfly_hal
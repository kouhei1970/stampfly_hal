/**
 * @file stampfly_rgbled.cpp
 * @brief StampFly RGBLED HAL Implementation
 */

#include "stampfly_rgbled.h"
#include "led_strip.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "STAMPFLY_RGBLED";

// StampFly RGBLED設定（仕様要確認、複数GPIOを試行可能）
#define STAMPFLY_LED_COUNT          2     // LED数（2個）
#define STAMPFLY_LED_DEFAULT_BRIGHTNESS 15    // デフォルト輝度（15/255 ≈ 6%）

// 候補GPIO（StampFly仕様不明のため複数試行）
static const int candidate_gpios[] = {39, 48, 21, 38, 47};
static const int num_candidates = sizeof(candidate_gpios) / sizeof(candidate_gpios[0]);
static int s_led_gpio = -1;

// RMT設定
#define STAMPFLY_RMT_RESOLUTION_HZ  (10 * 1000 * 1000) // 10MHz

static led_strip_handle_t s_led_strip = nullptr;
static uint8_t s_brightness = STAMPFLY_LED_DEFAULT_BRIGHTNESS;
static bool s_initialized = false;

esp_err_t stampfly_rgbled_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "RGBLED already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing StampFly RGBLED (%d LEDs)", STAMPFLY_LED_COUNT);

    // 複数のGPIOを試行
    for (int i = 0; i < num_candidates; i++) {
        int gpio = candidate_gpios[i];
        ESP_LOGI(TAG, "Trying GPIO%d for RGBLED...", gpio);

        // LED Strip設定
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = gpio;
        strip_config.max_leds = STAMPFLY_LED_COUNT;
        strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;  // WS2812BはGRB順序
        strip_config.led_model = LED_MODEL_WS2812;
        strip_config.flags.invert_out = false;

        // RMT設定
        led_strip_rmt_config_t rmt_config = {};
        rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
        rmt_config.resolution_hz = STAMPFLY_RMT_RESOLUTION_HZ;
        rmt_config.flags.with_dma = false;  // 小規模なLED数のためDMA無効

        // LED Strip初期化試行
        esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
        if (ret == ESP_OK) {
            s_led_gpio = gpio;
            s_initialized = true;
            ESP_LOGI(TAG, "RGBLED initialized successfully on GPIO%d", gpio);

            // 初期設定として全て消灯（初期化成功後に実行）
            ret = stampfly_rgbled_clear();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to clear LEDs during init: %s", esp_err_to_name(ret));
            }
            return ESP_OK;
        } else {
            ESP_LOGD(TAG, "GPIO%d failed: %s (0x%x)", gpio, esp_err_to_name(ret), ret);
        }
    }

    ESP_LOGE(TAG, "Failed to initialize RGBLED on any candidate GPIO");
    return ESP_FAIL;
}

esp_err_t stampfly_rgbled_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    // 消灯してから終了
    stampfly_rgbled_clear();

    esp_err_t ret = led_strip_del(s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete LED strip: %s", esp_err_to_name(ret));
        return ret;
    }

    s_led_strip = nullptr;
    s_initialized = false;
    ESP_LOGI(TAG, "RGBLED deinitialized");
    return ESP_OK;
}

esp_err_t stampfly_rgbled_set_color(uint32_t color)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "RGBLED not initialized");
        return ESP_FAIL;
    }

    // 24bit RGB値をR,G,Bに分離
    uint8_t red = (color >> 16) & 0xFF;
    uint8_t green = (color >> 8) & 0xFF;
    uint8_t blue = color & 0xFF;

    return stampfly_rgbled_set_rgb(red, green, blue);
}

esp_err_t stampfly_rgbled_set_preset_color(stampfly_color_t color)
{
    return stampfly_rgbled_set_color((uint32_t)color);
}

esp_err_t stampfly_rgbled_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "RGBLED not initialized");
        return ESP_FAIL;
    }

    // 輝度調整（Arduino版の実装に合わせて）
    red = (red * s_brightness) / 255;
    green = (green * s_brightness) / 255;
    blue = (blue * s_brightness) / 255;

    esp_err_t ret = ESP_OK;

    // 全てのLEDに同じ色を設定（Arduino版の動作に合わせて）
    for (int i = 0; i < STAMPFLY_LED_COUNT; i++) {
        ret = led_strip_set_pixel(s_led_strip, i, red, green, blue);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LED %d color: %s", i, esp_err_to_name(ret));
            return ret;
        }
    }

    // 設定を反映
    ret = stampfly_rgbled_refresh();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh LEDs: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Set RGB color: R=%d, G=%d, B=%d", red, green, blue);
    return ESP_OK;
}

esp_err_t stampfly_rgbled_set_brightness(uint8_t brightness)
{
    s_brightness = brightness;
    ESP_LOGD(TAG, "Set brightness: %d/255", brightness);
    return ESP_OK;
}

esp_err_t stampfly_rgbled_clear(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "RGBLED not initialized");
        return ESP_FAIL;
    }

    esp_err_t ret = led_strip_clear(s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear LEDs: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "LEDs cleared");
    return ESP_OK;
}

esp_err_t stampfly_rgbled_refresh(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "RGBLED not initialized");
        return ESP_FAIL;
    }

    esp_err_t ret = led_strip_refresh(s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh LEDs: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}
/**
 * @file stampfly_rgbled.h
 * @brief StampFly RGBLED HAL - WS2812B制御ライブラリ
 *
 * StampFlyのRGBLED（WS2812B）を制御するためのハードウェア抽象化層
 * ESP-IDF RMTペリフェラルを使用してLED制御を行う
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief StampFly RGBLED定義済み色
 */
typedef enum {
    STAMPFLY_COLOR_OFF     = 0x000000,  // 消灯
    STAMPFLY_COLOR_WHITE   = 0xFFFFFF,  // 白
    STAMPFLY_COLOR_RED     = 0xFF0000,  // 赤
    STAMPFLY_COLOR_GREEN   = 0x00FF00,  // 緑
    STAMPFLY_COLOR_BLUE    = 0x0000FF,  // 青
    STAMPFLY_COLOR_YELLOW  = 0xFFFF00,  // 黄
    STAMPFLY_COLOR_PURPLE  = 0xFF00FF,  // 紫
    STAMPFLY_COLOR_CYAN    = 0x00FFFF,  // シアン
    STAMPFLY_COLOR_ORANGE  = 0xFF9933,  // オレンジ（フリップモード用）
    STAMPFLY_COLOR_PINK    = 0xDC669B,  // ピンク（アクロモード用）
    STAMPFLY_COLOR_LOWBATT = 0x18EBF9,  // 水色（バッテリー低下用）
} stampfly_color_t;

/**
 * @brief RGBLED初期化
 *
 * StampFlyのオンボードRGBLED（GPIO39, 2個）を初期化する
 *
 * @return ESP_OK 成功時
 *         ESP_FAIL 初期化失敗時
 */
esp_err_t stampfly_rgbled_init(void);

/**
 * @brief RGBLED終了処理
 *
 * @return ESP_OK 成功時
 */
esp_err_t stampfly_rgbled_deinit(void);

/**
 * @brief 単色でRGBLEDを設定
 *
 * @param color 設定する色（24bit RGB値）
 * @return ESP_OK 成功時
 *         ESP_FAIL 設定失敗時
 */
esp_err_t stampfly_rgbled_set_color(uint32_t color);

/**
 * @brief 定義済み色でRGBLEDを設定
 *
 * @param color 設定する色（stampfly_color_t）
 * @return ESP_OK 成功時
 *         ESP_FAIL 設定失敗時
 */
esp_err_t stampfly_rgbled_set_preset_color(stampfly_color_t color);

/**
 * @brief RGB値を個別指定してRGBLEDを設定
 *
 * @param red 赤成分（0-255）
 * @param green 緑成分（0-255）
 * @param blue 青成分（0-255）
 * @return ESP_OK 成功時
 *         ESP_FAIL 設定失敗時
 */
esp_err_t stampfly_rgbled_set_rgb(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief RGBLED輝度設定
 *
 * @param brightness 輝度（0-255）
 * @return ESP_OK 成功時
 *         ESP_FAIL 設定失敗時
 */
esp_err_t stampfly_rgbled_set_brightness(uint8_t brightness);

/**
 * @brief RGBLED消灯
 *
 * @return ESP_OK 成功時
 *         ESP_FAIL 設定失敗時
 */
esp_err_t stampfly_rgbled_clear(void);

/**
 * @brief LED更新（設定を実際のLEDに反映）
 *
 * @return ESP_OK 成功時
 *         ESP_FAIL 更新失敗時
 */
esp_err_t stampfly_rgbled_refresh(void);

#ifdef __cplusplus
}
#endif
/*
 * StampFly UART HAL
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#pragma once

#include "hal_base.h"
#include "driver/uart.h"
#include "driver/gpio.h"

namespace stampfly_hal {

/**
 * @brief UART設定構造体
 */
struct UARTConfig {
    uart_port_t port;           ///< UARTポート番号
    uint32_t baud_rate;         ///< ボーレート
    uart_word_length_t data_bits; ///< データビット数
    uart_parity_t parity;       ///< パリティ
    uart_stop_bits_t stop_bits; ///< ストップビット数
    uart_hw_flowcontrol_t flow_ctrl; ///< フロー制御
    gpio_num_t tx_pin;          ///< TX ピン
    gpio_num_t rx_pin;          ///< RX ピン
    gpio_num_t rts_pin;         ///< RTS ピン
    gpio_num_t cts_pin;         ///< CTS ピン
    size_t rx_buffer_size;      ///< 受信バッファサイズ
    size_t tx_buffer_size;      ///< 送信バッファサイズ
    int intr_alloc_flags;       ///< 割り込み配置フラグ
};

/**
 * @brief UART HALクラス
 */
class UartHal : public HALBase {
public:
    /**
     * @brief コンストラクタ
     * @param config UART設定
     */
    explicit UartHal(const UARTConfig& config);

    /**
     * @brief デストラクタ
     */
    ~UartHal() override;

    /**
     * @brief 初期化
     * @return esp_err_t 初期化結果
     */
    esp_err_t init() override;

    /**
     * @brief 設定
     * @return esp_err_t 設定結果
     */
    esp_err_t configure() override;

    /**
     * @brief 有効化
     * @return esp_err_t 有効化結果
     */
    esp_err_t enable() override { set_enabled(true); return ESP_OK; }

    /**
     * @brief 無効化
     * @return esp_err_t 無効化結果
     */
    esp_err_t disable() override { set_enabled(false); return ESP_OK; }

    /**
     * @brief リセット
     * @return esp_err_t リセット結果
     */
    esp_err_t reset() override;

    /**
     * @brief printf出力を本UARTにリダイレクト
     * @return esp_err_t リダイレクト結果
     */
    esp_err_t redirect_printf();

    /**
     * @brief データ送信
     * @param data 送信データ
     * @param length データ長
     * @param timeout_ms タイムアウト（ミリ秒）
     * @return int 送信したバイト数、-1でエラー
     */
    int write(const char* data, size_t length, uint32_t timeout_ms = 1000);

    /**
     * @brief データ受信
     * @param buffer 受信バッファ
     * @param length バッファサイズ
     * @param timeout_ms タイムアウト（ミリ秒）
     * @return int 受信したバイト数、-1でエラー
     */
    int read(char* buffer, size_t length, uint32_t timeout_ms = 1000);

    /**
     * @brief 文字列送信
     * @param str 送信文字列
     * @param timeout_ms タイムアウト（ミリ秒）
     * @return int 送信したバイト数、-1でエラー
     */
    int write_string(const char* str, uint32_t timeout_ms = 1000);

    /**
     * @brief フォーマット文字列送信
     * @param format フォーマット文字列
     * @param ... 可変引数
     * @return int 送信したバイト数、-1でエラー
     */
    int printf(const char* format, ...);

    /**
     * @brief 受信データ存在確認
     * @return size_t 受信可能バイト数
     */
    size_t available();

    /**
     * @brief バッファクリア
     * @return esp_err_t クリア結果
     */
    esp_err_t flush();

    /**
     * @brief StampFly標準UART設定取得
     * @return UARTConfig StampFly用のデフォルト設定
     */
    static UARTConfig get_stampfly_default_config();

private:
    UARTConfig config_;         ///< UART設定
    bool printf_redirected_;    ///< printf リダイレクト状態

    /**
     * @brief ピン設定
     * @return esp_err_t ピン設定結果
     */
    esp_err_t configure_pins();

    /**
     * @brief UARTドライバインストール
     * @return esp_err_t インストール結果
     */
    esp_err_t install_driver();
};

} // namespace stampfly_hal
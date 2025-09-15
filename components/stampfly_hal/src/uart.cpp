/*
 * StampFly UART HAL Implementation
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#include "uart_hal.h"
#include "esp_vfs_dev.h"
#include "driver/uart_vfs.h"
#include <cstdarg>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace stampfly_hal {

UartHal::UartHal(const UARTConfig& config) 
    : HALBase("UART_HAL"), config_(config), printf_redirected_(false) 
{
}

UartHal::~UartHal() 
{
    if (is_initialized()) {
        uart_driver_delete(config_.port);
    }
}

esp_err_t UartHal::init() 
{
    if (is_initialized()) {
        log(ESP_LOG_WARN, "Already initialized");
        return ESP_OK;
    }

    log(ESP_LOG_INFO, "Initializing UART%d", config_.port);

    // ピン設定
    esp_err_t ret = configure_pins();
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    // UARTドライバインストール
    ret = install_driver();
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    set_initialized(true);
    log(ESP_LOG_INFO, "UART%d initialized successfully", config_.port);
    
    return ESP_OK;
}

esp_err_t UartHal::configure() 
{
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    // UART設定の適用
    esp_err_t ret = uart_set_baudrate(config_.port, config_.baud_rate);
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    ret = uart_set_word_length(config_.port, config_.data_bits);
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    ret = uart_set_parity(config_.port, config_.parity);
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    ret = uart_set_stop_bits(config_.port, config_.stop_bits);
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    ret = uart_set_hw_flow_ctrl(config_.port, config_.flow_ctrl, 0);
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    log(ESP_LOG_INFO, "UART%d configured: %ld baud, %d data bits", 
        config_.port, config_.baud_rate, config_.data_bits + 5);
    
    return ESP_OK;
}

esp_err_t UartHal::redirect_printf() 
{
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    if (printf_redirected_) {
        log(ESP_LOG_WARN, "printf already redirected");
        return ESP_OK;
    }

    // stdio を UART にリダイレクト
    uart_vfs_dev_use_driver(config_.port);
    printf_redirected_ = true;
    
    log(ESP_LOG_INFO, "printf redirected to UART%d", config_.port);
    return ESP_OK;
}

int UartHal::write(const char* data, size_t length, uint32_t timeout_ms) 
{
    if (!is_initialized() || !is_enabled()) {
        return -1;
    }

    int bytes_written = uart_write_bytes(config_.port, data, length);
    if (bytes_written < 0) {
        set_error(ESP_FAIL);
        return -1;
    }

    // 送信完了待ち
    if (uart_wait_tx_done(config_.port, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) {
        set_error(ESP_ERR_TIMEOUT);
        return -1;
    }

    return bytes_written;
}

int UartHal::read(char* buffer, size_t length, uint32_t timeout_ms) 
{
    if (!is_initialized() || !is_enabled()) {
        return -1;
    }

    int bytes_read = uart_read_bytes(config_.port, buffer, length, pdMS_TO_TICKS(timeout_ms));
    if (bytes_read < 0) {
        set_error(ESP_FAIL);
        return -1;
    }

    return bytes_read;
}

int UartHal::write_string(const char* str, uint32_t timeout_ms) 
{
    if (!str) {
        return -1;
    }
    
    return write(str, strlen(str), timeout_ms);
}

int UartHal::printf(const char* format, ...) 
{
    if (!is_initialized() || !is_enabled()) {
        return -1;
    }

    char buffer[256];  // 固定バッファサイズ
    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length < 0) {
        return -1;
    }

    // バッファサイズを超えた場合は切り詰め
    if (length >= sizeof(buffer)) {
        length = sizeof(buffer) - 1;
    }

    return write(buffer, length, 1000);
}

size_t UartHal::available() 
{
    if (!is_initialized()) {
        return 0;
    }

    size_t bytes_available = 0;
    esp_err_t ret = uart_get_buffered_data_len(config_.port, &bytes_available);
    if (ret != ESP_OK) {
        set_error(ret);
        return 0;
    }

    return bytes_available;
}

esp_err_t UartHal::reset()
{
    if (is_initialized()) {
        uart_driver_delete(config_.port);
        set_initialized(false);
        set_enabled(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        return init();
    }
    return ESP_OK;
}

esp_err_t UartHal::flush() 
{
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    esp_err_t ret = uart_flush(config_.port);
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    return ESP_OK;
}

UARTConfig UartHal::get_stampfly_default_config() 
{
    UARTConfig config = {
        .port = UART_NUM_0,                     // StampFly標準UART
        .baud_rate = 115200,                    // 標準ボーレート
        .data_bits = UART_DATA_8_BITS,          // 8ビットデータ
        .parity = UART_PARITY_DISABLE,          // パリティなし
        .stop_bits = UART_STOP_BITS_1,          // ストップビット1
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // フロー制御なし
        .tx_pin = static_cast<gpio_num_t>(UART_PIN_NO_CHANGE),    // デフォルトピン使用
        .rx_pin = static_cast<gpio_num_t>(UART_PIN_NO_CHANGE),    // デフォルトピン使用
        .rts_pin = static_cast<gpio_num_t>(UART_PIN_NO_CHANGE),   // RTSピン使用しない
        .cts_pin = static_cast<gpio_num_t>(UART_PIN_NO_CHANGE),   // CTSピン使用しない
        .rx_buffer_size = 1024,                 // 受信バッファ1KB
        .tx_buffer_size = 1024,                 // 送信バッファ1KB
        .intr_alloc_flags = 0                   // 割り込みフラグデフォルト
    };
    return config;
}

esp_err_t UartHal::configure_pins() 
{
    // ピン設定が必要な場合のみ設定
    if (config_.tx_pin != UART_PIN_NO_CHANGE) {
        esp_err_t ret = uart_set_pin(config_.port, config_.tx_pin, config_.rx_pin, 
                                     config_.rts_pin, config_.cts_pin);
        if (ret != ESP_OK) {
            log(ESP_LOG_ERROR, "Failed to set UART pins: %s", esp_err_to_name(ret));
            return ret;
        }
        log(ESP_LOG_INFO, "UART%d pins: TX=%d, RX=%d", config_.port, config_.tx_pin, config_.rx_pin);
    }
    
    return ESP_OK;
}

esp_err_t UartHal::install_driver() 
{
    // UART設定構造体の初期化
    uart_config_t uart_config = {
        .baud_rate = static_cast<int>(config_.baud_rate),
        .data_bits = config_.data_bits,
        .parity = config_.parity,
        .stop_bits = config_.stop_bits,
        .flow_ctrl = config_.flow_ctrl,
        .rx_flow_ctrl_thresh = 122,  // デフォルト値
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = false,
            .backup_before_sleep = false
        }
    };

    // UARTドライバのインストール
    esp_err_t ret = uart_driver_install(config_.port, config_.rx_buffer_size, 
                                        config_.tx_buffer_size, 0, NULL, 
                                        config_.intr_alloc_flags);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // UART設定の適用
    ret = uart_param_config(config_.port, &uart_config);
    if (ret != ESP_OK) {
        uart_driver_delete(config_.port);
        log(ESP_LOG_ERROR, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

} // namespace stampfly_hal
/*
 * UART HAL Debug Test
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#include "uart_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "UART_DEBUG";

/**
 * @brief UART HAL基本テスト
 */
void test_uart_hal_basic() 
{
    ESP_LOGI(TAG, "Starting UART HAL basic test");
    
    // StampFly標準設定でUART HAL初期化
    auto config = stampfly_hal::UartHal::get_stampfly_default_config();
    stampfly_hal::UartHal uart_hal(config);
    
    // 初期化
    esp_err_t ret = uart_hal.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART HAL init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 設定
    ret = uart_hal.configure();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART HAL configure failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 有効化
    ret = uart_hal.enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART HAL enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "UART HAL basic test: PASSED");
}

/**
 * @brief UART HAL文字列送信テスト
 */
void test_uart_hal_write() 
{
    ESP_LOGI(TAG, "Starting UART HAL write test");
    
    auto config = stampfly_hal::UartHal::get_stampfly_default_config();
    stampfly_hal::UartHal uart_hal(config);
    
    uart_hal.init();
    uart_hal.configure();
    uart_hal.enable();
    
    // 文字列送信テスト
    const char* test_msg = "Hello from StampFly UART HAL!\n";
    int bytes_written = uart_hal.write_string(test_msg);
    
    if (bytes_written > 0) {
        ESP_LOGI(TAG, "Successfully wrote %d bytes", bytes_written);
    } else {
        ESP_LOGE(TAG, "Failed to write data");
        return;
    }
    
    // printf形式送信テスト
    int result = uart_hal.printf("Test number: %d, float: %.2f\n", 123, 45.67);
    if (result > 0) {
        ESP_LOGI(TAG, "Printf test successful: %d bytes", result);
    } else {
        ESP_LOGE(TAG, "Printf test failed");
        return;
    }
    
    ESP_LOGI(TAG, "UART HAL write test: PASSED");
}

/**
 * @brief UART HAL printf リダイレクトテスト
 */
void test_uart_hal_printf_redirect() 
{
    ESP_LOGI(TAG, "Starting UART HAL printf redirect test");
    
    auto config = stampfly_hal::UartHal::get_stampfly_default_config();
    stampfly_hal::UartHal uart_hal(config);
    
    uart_hal.init();
    uart_hal.configure();
    uart_hal.enable();
    
    // printf リダイレクト
    esp_err_t ret = uart_hal.redirect_printf();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Printf redirect failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 標準printfを使用してテスト（リダイレクト後）
    printf("This message should appear via UART HAL redirect!\n");
    printf("Test values: int=%d, string=%s, hex=0x%x\n", 42, "StampFly", 0xABCD);
    
    ESP_LOGI(TAG, "UART HAL printf redirect test: PASSED");
}

/**
 * @brief 全てのUART HALテストを実行
 */
void run_uart_hal_debug_tests() 
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  UART HAL Debug Tests");
    ESP_LOGI(TAG, "========================================");
    
    test_uart_hal_basic();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_uart_hal_write();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_uart_hal_printf_redirect();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  All UART HAL tests completed");
    ESP_LOGI(TAG, "========================================");
}
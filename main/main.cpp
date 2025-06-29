/*
 * StampFly HAL Main Application
 * ESP-IDF v5.4.1 Compatible
 * Target: ESP32-S3 (StampFly Hardware)
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "esp_timer.h"

// StampFly HAL includes
#include "uart_hal.h"
#include "spi_hal.h"
#include "i2c_hal.h"

static const char* TAG = "STAMPFLY_HAL";

/**
 * @brief StampFly HAL初期化関数
 * @return esp_err_t 初期化結果
 */
esp_err_t stampfly_hal_init(void)
{
    ESP_LOGI(TAG, "StampFly HAL initialization started");
    
    // NVS初期化（設定保存用）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // ESP Timer初期化
    ESP_ERROR_CHECK(esp_timer_init());
    
    ESP_LOGI(TAG, "StampFly HAL initialization completed");
    return ESP_OK;
}

/**
 * @brief メインタスク
 * @param pvParameters パラメータ（未使用）
 */
void stampfly_main_task(void* pvParameters)
{
    ESP_LOGI(TAG, "StampFly main task started");
    
    // StampFly情報表示
    printf("\n");
    printf("========================================\n");
    printf("  StampFly HAL System\n");
    printf("  ESP-IDF Version: %s\n", esp_get_idf_version());
    printf("  Target: ESP32-S3\n");
    printf("  HAL Version: 1.0.0-dev\n");
    printf("========================================\n");
    printf("\n");
    
    // システム情報表示（printf動作確認）
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    printf("Hardware Information:\n");
    printf("- Chip: %s\n", 
           chip_info.model == CHIP_ESP32S3 ? "ESP32-S3" : "Unknown");
    printf("- Cores: %d\n", chip_info.cores);
    printf("- Revision: %d\n", chip_info.revision);
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    printf("- Flash: %luMB %s\n", 
           (unsigned long)(flash_size / (1024 * 1024)),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    
    if (chip_info.features & CHIP_FEATURE_WIFI_BGN) {
        printf("- WiFi: Yes\n");
    }
    if (chip_info.features & CHIP_FEATURE_BT) {
        printf("- Bluetooth: Yes\n");
    }
    
    printf("\nMemory Information:\n");
    printf("- Free heap: %lu bytes\n", (unsigned long)esp_get_free_heap_size());
    printf("- Minimum free heap: %lu bytes\n", (unsigned long)esp_get_minimum_free_heap_size());
    
    printf("\nStampFly HAL Components Status:\n");
    
    // UART HAL テスト
    auto uart_config = stampfly_hal::UartHal::get_stampfly_default_config();
    stampfly_hal::UartHal uart_hal(uart_config);
    
    esp_err_t uart_ret = uart_hal.init();
    if (uart_ret == ESP_OK) {
        uart_ret = uart_hal.configure();
        if (uart_ret == ESP_OK) {
            uart_ret = uart_hal.enable();
        }
    }
    
    printf("- UART HAL: %s\n", (uart_ret == ESP_OK) ? "OK" : "Failed");
    
    // SPI HAL テスト
    auto spi_config = stampfly_hal::SpiHal::get_stampfly_default_config();
    stampfly_hal::SpiHal spi_hal(spi_config);
    
    esp_err_t spi_ret = spi_hal.init();
    if (spi_ret == ESP_OK) {
        spi_ret = spi_hal.configure();
        if (spi_ret == ESP_OK) {
            spi_ret = spi_hal.enable();
        }
    }
    
    printf("- SPI HAL: %s\n", (spi_ret == ESP_OK) ? "OK" : "Failed");
    
    // I2C HAL テスト
    auto i2c_config = stampfly_hal::I2cHal::get_stampfly_default_config();
    stampfly_hal::I2cHal i2c_hal(i2c_config);
    
    esp_err_t i2c_ret = i2c_hal.init();
    if (i2c_ret == ESP_OK) {
        i2c_ret = i2c_hal.configure();
        if (i2c_ret == ESP_OK) {
            i2c_ret = i2c_hal.enable();
        }
    }
    
    printf("- I2C HAL: %s\n", (i2c_ret == ESP_OK) ? "OK" : "Failed");
    printf("- GPIO HAL: Not implemented\n");
    printf("- Sensors: Not implemented\n");
    
    printf("\nStarting basic operation test loop...\n");
    
    uint32_t loop_count = 0;
    
    while (1) {
        // 基本動作ループ（1秒間隔）
        ESP_LOGI(TAG, "Loop %lu - System running normally", (unsigned long)loop_count);
        
        // printf動作確認
        printf("StampFly HAL: Loop %lu, Free heap: %lu bytes\n", 
               (unsigned long)loop_count, (unsigned long)esp_get_free_heap_size());
        
        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief アプリケーションメイン関数
 */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting StampFly HAL application");
    
    // HAL初期化
    ESP_ERROR_CHECK(stampfly_hal_init());
    
    // メインタスク作成
    xTaskCreate(
        stampfly_main_task,     // タスク関数
        "stampfly_main",        // タスク名
        8192,                   // スタックサイズ
        NULL,                   // パラメータ
        5,                      // 優先度
        NULL                    // ハンドル
    );
    
    ESP_LOGI(TAG, "StampFly HAL application started successfully");
}
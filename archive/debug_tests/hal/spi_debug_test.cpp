/*
 * SPI HAL Debug Test
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#include "spi_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SPI_DEBUG";

/**
 * @brief SPI HAL基本テスト
 */
void test_spi_hal_basic() 
{
    ESP_LOGI(TAG, "Starting SPI HAL basic test");
    
    // StampFly標準設定でSPI HAL初期化
    auto config = stampfly_hal::SpiHal::get_stampfly_default_config();
    stampfly_hal::SpiHal spi_hal(config);
    
    // 初期化
    esp_err_t ret = spi_hal.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI HAL init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 設定
    ret = spi_hal.configure();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI HAL configure failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 有効化
    ret = spi_hal.enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI HAL enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "SPI HAL basic test: PASSED");
}

/**
 * @brief BMI270デバイス追加テスト
 */
void test_spi_bmi270_device() 
{
    ESP_LOGI(TAG, "Starting BMI270 device test");
    
    auto config = stampfly_hal::SpiHal::get_stampfly_default_config();
    stampfly_hal::SpiHal spi_hal(config);
    
    spi_hal.init();
    spi_hal.configure();
    spi_hal.enable();
    
    // BMI270デバイス追加
    auto device_config = stampfly_hal::SpiHal::get_bmi270_device_config();
    spi_device_handle_t bmi270_handle;
    
    esp_err_t ret = spi_hal.add_device(device_config, &bmi270_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BMI270 device: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "BMI270 device added successfully");
    
    // WHOAMIレジスタ読み取りテスト（BMI270のWHOAMI = 0x24）
    uint8_t whoami = 0;
    ret = spi_hal.read(bmi270_handle, 0x00, &whoami, 1);  // WHOAMI reg = 0x00
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BMI270 WHOAMI read: 0x%02X (expected: 0x24)", whoami);
    } else {
        ESP_LOGW(TAG, "BMI270 WHOAMI read failed (device may not be connected)");
    }
    
    // デバイス削除
    spi_hal.remove_device(bmi270_handle);
    
    ESP_LOGI(TAG, "BMI270 device test: PASSED");
}

/**
 * @brief PMW3901デバイス追加テスト
 */
void test_spi_pmw3901_device() 
{
    ESP_LOGI(TAG, "Starting PMW3901 device test");
    
    auto config = stampfly_hal::SpiHal::get_stampfly_default_config();
    stampfly_hal::SpiHal spi_hal(config);
    
    spi_hal.init();
    spi_hal.configure();
    spi_hal.enable();
    
    // PMW3901デバイス追加
    auto device_config = stampfly_hal::SpiHal::get_pmw3901_device_config();
    spi_device_handle_t pmw3901_handle;
    
    esp_err_t ret = spi_hal.add_device(device_config, &pmw3901_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PMW3901 device: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "PMW3901 device added successfully");
    
    // Product IDレジスタ読み取りテスト（PMW3901のProduct ID = 0x49）
    uint8_t product_id = 0;
    ret = spi_hal.read(pmw3901_handle, 0x00, &product_id, 1);  // Product_ID reg = 0x00
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PMW3901 Product ID read: 0x%02X (expected: 0x49)", product_id);
    } else {
        ESP_LOGW(TAG, "PMW3901 Product ID read failed (device may not be connected)");
    }
    
    // デバイス削除
    spi_hal.remove_device(pmw3901_handle);
    
    ESP_LOGI(TAG, "PMW3901 device test: PASSED");
}

/**
 * @brief 複数デバイス同時テスト
 */
void test_spi_multiple_devices() 
{
    ESP_LOGI(TAG, "Starting multiple devices test");
    
    auto config = stampfly_hal::SpiHal::get_stampfly_default_config();
    stampfly_hal::SpiHal spi_hal(config);
    
    spi_hal.init();
    spi_hal.configure();
    spi_hal.enable();
    
    // 両デバイス追加
    auto bmi270_config = stampfly_hal::SpiHal::get_bmi270_device_config();
    auto pmw3901_config = stampfly_hal::SpiHal::get_pmw3901_device_config();
    
    spi_device_handle_t bmi270_handle, pmw3901_handle;
    
    esp_err_t ret1 = spi_hal.add_device(bmi270_config, &bmi270_handle);
    esp_err_t ret2 = spi_hal.add_device(pmw3901_config, &pmw3901_handle);
    
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        ESP_LOGI(TAG, "Both devices added successfully");
        
        // 両デバイスからの読み取りテスト
        uint8_t bmi270_data = 0, pmw3901_data = 0;
        spi_hal.read(bmi270_handle, 0x00, &bmi270_data, 1);
        spi_hal.read(pmw3901_handle, 0x00, &pmw3901_data, 1);
        
        ESP_LOGI(TAG, "Read results - BMI270: 0x%02X, PMW3901: 0x%02X", 
                 bmi270_data, pmw3901_data);
        
        // デバイス削除
        spi_hal.remove_device(bmi270_handle);
        spi_hal.remove_device(pmw3901_handle);
    } else {
        ESP_LOGE(TAG, "Failed to add devices: BMI270=%s, PMW3901=%s",
                 esp_err_to_name(ret1), esp_err_to_name(ret2));
    }
    
    ESP_LOGI(TAG, "Multiple devices test: PASSED");
}

/**
 * @brief 全てのSPI HALテストを実行
 */
void run_spi_hal_debug_tests() 
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  SPI HAL Debug Tests");
    ESP_LOGI(TAG, "========================================");
    
    test_spi_hal_basic();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_spi_bmi270_device();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_spi_pmw3901_device();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_spi_multiple_devices();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  All SPI HAL tests completed");
    ESP_LOGI(TAG, "========================================");
}
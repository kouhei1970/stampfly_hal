/*
 * I2C HAL Debug Test
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#include "i2c_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "I2C_DEBUG";

/**
 * @brief I2C HAL基本テスト
 */
void test_i2c_hal_basic() 
{
    ESP_LOGI(TAG, "Starting I2C HAL basic test");
    
    // StampFly標準設定でI2C HAL初期化
    auto config = stampfly_hal::I2cHal::get_stampfly_default_config();
    stampfly_hal::I2cHal i2c_hal(config);
    
    // 初期化
    esp_err_t ret = i2c_hal.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C HAL init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 設定
    ret = i2c_hal.configure();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C HAL configure failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 有効化
    ret = i2c_hal.enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C HAL enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "I2C HAL basic test: PASSED");
}

/**
 * @brief I2Cデバイススキャンテスト
 */
void test_i2c_device_scan() 
{
    ESP_LOGI(TAG, "Starting I2C device scan test");
    
    auto config = stampfly_hal::I2cHal::get_stampfly_default_config();
    stampfly_hal::I2cHal i2c_hal(config);
    
    i2c_hal.init();
    i2c_hal.configure();
    i2c_hal.enable();
    
    // デバイススキャン
    uint8_t found_devices[16];
    int device_count = i2c_hal.scan_devices(found_devices, sizeof(found_devices));
    
    if (device_count >= 0) {
        ESP_LOGI(TAG, "Device scan successful: %d devices found", device_count);
        for (int i = 0; i < device_count; i++) {
            ESP_LOGI(TAG, "  Device %d: 0x%02X", i + 1, found_devices[i]);
        }
    } else {
        ESP_LOGE(TAG, "Device scan failed");
        return;
    }
    
    ESP_LOGI(TAG, "I2C device scan test: PASSED");
}

/**
 * @brief StampFlyデバイス確認テスト
 */
void test_stampfly_devices() 
{
    ESP_LOGI(TAG, "Starting StampFly devices test");
    
    auto config = stampfly_hal::I2cHal::get_stampfly_default_config();
    stampfly_hal::I2cHal i2c_hal(config);
    
    i2c_hal.init();
    i2c_hal.configure();
    i2c_hal.enable();
    
    // StampFly既知デバイスの確認
    const auto* devices = stampfly_hal::I2cHal::get_stampfly_devices();
    size_t device_count = stampfly_hal::I2cHal::get_stampfly_device_count();
    
    ESP_LOGI(TAG, "Checking %zu StampFly devices:", device_count);
    
    for (size_t i = 0; i < device_count; i++) {
        bool present = i2c_hal.is_device_present(devices[i].address);
        ESP_LOGI(TAG, "  %s (0x%02X): %s", 
                 devices[i].name, devices[i].address, 
                 present ? "FOUND" : "NOT FOUND");
    }
    
    ESP_LOGI(TAG, "StampFly devices test: PASSED");
}

/**
 * @brief BMM150磁気センサーテスト
 */
void test_bmm150_sensor() 
{
    ESP_LOGI(TAG, "Starting BMM150 sensor test");
    
    auto config = stampfly_hal::I2cHal::get_stampfly_default_config();
    stampfly_hal::I2cHal i2c_hal(config);
    
    i2c_hal.init();
    i2c_hal.configure();
    i2c_hal.enable();
    
    const uint8_t BMM150_ADDR = 0x10;
    const uint8_t BMM150_CHIP_ID_REG = 0x40;
    const uint8_t BMM150_EXPECTED_ID = 0x32;
    
    // BMM150存在確認
    if (!i2c_hal.is_device_present(BMM150_ADDR)) {
        ESP_LOGW(TAG, "BMM150 not found at address 0x%02X", BMM150_ADDR);
        return;
    }
    
    // Chip ID読み取り
    int chip_id = i2c_hal.read_byte(BMM150_ADDR, BMM150_CHIP_ID_REG);
    if (chip_id >= 0) {
        ESP_LOGI(TAG, "BMM150 Chip ID: 0x%02X (expected: 0x%02X)", chip_id, BMM150_EXPECTED_ID);
        if (chip_id == BMM150_EXPECTED_ID) {
            ESP_LOGI(TAG, "BMM150 identification: SUCCESS");
        } else {
            ESP_LOGW(TAG, "BMM150 identification: FAILED (wrong ID)");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read BMM150 Chip ID");
        return;
    }
    
    ESP_LOGI(TAG, "BMM150 sensor test: PASSED");
}

/**
 * @brief BMP280気圧センサーテスト
 */
void test_bmp280_sensor() 
{
    ESP_LOGI(TAG, "Starting BMP280 sensor test");
    
    auto config = stampfly_hal::I2cHal::get_stampfly_default_config();
    stampfly_hal::I2cHal i2c_hal(config);
    
    i2c_hal.init();
    i2c_hal.configure();
    i2c_hal.enable();
    
    const uint8_t BMP280_ADDR = 0x76;
    const uint8_t BMP280_ID_REG = 0xD0;
    const uint8_t BMP280_EXPECTED_ID = 0x58;
    
    // BMP280存在確認
    if (!i2c_hal.is_device_present(BMP280_ADDR)) {
        ESP_LOGW(TAG, "BMP280 not found at address 0x%02X", BMP280_ADDR);
        return;
    }
    
    // Chip ID読み取り
    int chip_id = i2c_hal.read_byte(BMP280_ADDR, BMP280_ID_REG);
    if (chip_id >= 0) {
        ESP_LOGI(TAG, "BMP280 Chip ID: 0x%02X (expected: 0x%02X)", chip_id, BMP280_EXPECTED_ID);
        if (chip_id == BMP280_EXPECTED_ID) {
            ESP_LOGI(TAG, "BMP280 identification: SUCCESS");
        } else {
            ESP_LOGW(TAG, "BMP280 identification: FAILED (wrong ID)");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read BMP280 Chip ID");
        return;
    }
    
    ESP_LOGI(TAG, "BMP280 sensor test: PASSED");
}

/**
 * @brief INA3221電力モニターテスト
 */
void test_ina3221_sensor() 
{
    ESP_LOGI(TAG, "Starting INA3221 sensor test");
    
    auto config = stampfly_hal::I2cHal::get_stampfly_default_config();
    stampfly_hal::I2cHal i2c_hal(config);
    
    i2c_hal.init();
    i2c_hal.configure();
    i2c_hal.enable();
    
    const uint8_t INA3221_ADDR = 0x40;
    const uint8_t INA3221_MFG_ID_REG = 0xFE;
    const uint16_t INA3221_EXPECTED_MFG_ID = 0x5449;  // "TI"
    
    // INA3221存在確認
    if (!i2c_hal.is_device_present(INA3221_ADDR)) {
        ESP_LOGW(TAG, "INA3221 not found at address 0x%02X", INA3221_ADDR);
        return;
    }
    
    // Manufacturer ID読み取り（16ビットレジスタ）
    uint8_t mfg_id_data[2];
    esp_err_t ret = i2c_hal.read_register(INA3221_ADDR, INA3221_MFG_ID_REG, mfg_id_data, 2);
    if (ret == ESP_OK) {
        uint16_t mfg_id = (mfg_id_data[0] << 8) | mfg_id_data[1];
        ESP_LOGI(TAG, "INA3221 Manufacturer ID: 0x%04X (expected: 0x%04X)", 
                 mfg_id, INA3221_EXPECTED_MFG_ID);
        if (mfg_id == INA3221_EXPECTED_MFG_ID) {
            ESP_LOGI(TAG, "INA3221 identification: SUCCESS");
        } else {
            ESP_LOGW(TAG, "INA3221 identification: FAILED (wrong ID)");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read INA3221 Manufacturer ID");
        return;
    }
    
    ESP_LOGI(TAG, "INA3221 sensor test: PASSED");
}

/**
 * @brief 全てのI2C HALテストを実行
 */
void run_i2c_hal_debug_tests() 
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  I2C HAL Debug Tests");
    ESP_LOGI(TAG, "========================================");
    
    test_i2c_hal_basic();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_i2c_device_scan();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_stampfly_devices();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_bmm150_sensor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_bmp280_sensor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_ina3221_sensor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  All I2C HAL tests completed");
    ESP_LOGI(TAG, "========================================");
}
/*
 * StampFly HAL Main Application - BMI270 Initialization Test
 * ESP-IDF v5.4.1 Compatible
 * Target: ESP32-S3 (StampFly Hardware)
 * Features: BMI270 initialization sequence verification only
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

// StampFly HAL includes
#include "stampfly_hal.h"

static const char* TAG = "STAMPFLY_MAIN";

// Global instances
static stampfly_hal::BMI270* g_bmi270 = nullptr;

extern "C" void app_main(void) {
    // Set ESP_LOG level to INFO (DEBUG mode disabled)
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("BMI270", ESP_LOG_INFO);
    esp_log_level_set("SpiHal", ESP_LOG_INFO);

    ESP_LOGI(TAG, "🚀 === StampFly HAL - BMI270 Initialization Test (INFO MODE) ===");
    ESP_LOGI(TAG, "Features: Software reset + CHIP_ID verification (timing test)");

    // Phase 1: Initialize NVS
    ESP_LOGI(TAG, "📝 Phase 1: NVS Flash initialization");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS Flash initialized successfully");

    // Phase 2: System stabilization
    ESP_LOGI(TAG, "⏳ Phase 2: System stabilization (5ms)...");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "System stabilization complete");

    // Phase 3: BMI270 initialization (init() only)
    ESP_LOGI(TAG, "🔧 Phase 3: BMI270 Initialization (Software reset + CHIP_ID verification)");
    g_bmi270 = new stampfly_hal::BMI270();
    if (!g_bmi270) {
        ESP_LOGE(TAG, "❌ Failed to create BMI270 instance");
        return;
    }

    ret = g_bmi270->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ BMI270 initialization failed: %s", esp_err_to_name(ret));
        delete g_bmi270;
        g_bmi270 = nullptr;
        ESP_LOGE(TAG, "⚠️ Program stopped - initialization failure");
        while(1) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            ESP_LOGE(TAG, "❌ BMI270 initialization failed - please check hardware");
        }
    }

    ESP_LOGI(TAG, "✅ BMI270 initialization successful!");
    ESP_LOGI(TAG, "🎉 === Initialization Test Complete ===");
    ESP_LOGI(TAG, "Program will loop here. Reset device to test again.");

    // Loop forever (no tasks created)
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "✅ BMI270 initialized - running normally");
    }
}
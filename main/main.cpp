/*
 * StampFly HAL Main Application - BMI270 1600Hz IMU Streaming
 * ESP-IDF v5.4.1 Compatible
 * Target: ESP32-S3 (StampFly Hardware)
 * Features: 1600Hz ODR + 500Hz Teleplot Streaming
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
static stampfly_hal::RgbLed* g_rgbled = nullptr;

// Task handles
static TaskHandle_t task_imu_streaming_handle = nullptr;
static TaskHandle_t task_led_status_handle = nullptr;

// Debug mode control
static bool debug_mode = false;  // Set to true for detailed logging

// === IMU Data Streaming Task (500Hz) ===
void task_imu_streaming(void* pvParameters) {
    stampfly_hal::BMI270::AccelData accel;
    stampfly_hal::BMI270::GyroData gyro;
    float temperature;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(2);  // 2ms = 500Hz

    uint32_t loop_count = 0;
    uint32_t error_count = 0;

    if (debug_mode) {
        ESP_LOGI(TAG, "IMU streaming task started (500Hz)");
    }

    while (1) {
        esp_err_t ret_accel = g_bmi270->read_accel(accel);
        esp_err_t ret_gyro = g_bmi270->read_gyro(gyro);
        esp_err_t ret_temp = g_bmi270->read_temperature(temperature);

        if (ret_accel == ESP_OK && ret_gyro == ESP_OK && ret_temp == ESP_OK) {
            // Teleplot format output
            printf(">accel_x:%.3f\n", accel.x);
            printf(">accel_y:%.3f\n", accel.y);
            printf(">accel_z:%.3f\n", accel.z);
            printf(">gyro_x:%.3f\n", gyro.x);
            printf(">gyro_y:%.3f\n", gyro.y);
            printf(">gyro_z:%.3f\n", gyro.z);
            printf(">temperature:%.2f\n", temperature);

            loop_count++;

            // Print statistics every 500 samples (1 second)
            if (debug_mode && loop_count % 500 == 0) {
                ESP_LOGI(TAG, "📊 Streamed %lu samples, errors: %lu", loop_count, error_count);
            }
        } else {
            error_count++;
            if (debug_mode) {
                ESP_LOGE(TAG, "Failed to read IMU data (errors: %lu)", error_count);
            }
        }

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// === LED Status Task (1Hz) ===
void task_led_status(void* pvParameters) {
    const stampfly_hal::Color colors[] = {
        stampfly_hal::Color::GREEN,
        stampfly_hal::Color::BLUE,
        stampfly_hal::Color::CYAN,
        stampfly_hal::Color::YELLOW,
        stampfly_hal::Color::PURPLE,
        stampfly_hal::Color::WHITE
    };
    const int num_colors = sizeof(colors) / sizeof(colors[0]);
    int color_index = 0;

    if (debug_mode) {
        ESP_LOGI(TAG, "LED status task started");
    }

    while (1) {
        g_rgbled->set_color(colors[color_index]);
        color_index = (color_index + 1) % num_colors;

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second
    }
}

extern "C" void app_main(void) {
    // Set ESP_LOG level based on debug mode
    if (debug_mode) {
        esp_log_level_set("*", ESP_LOG_DEBUG);
        esp_log_level_set(TAG, ESP_LOG_DEBUG);
        esp_log_level_set("BMI270", ESP_LOG_DEBUG);
        esp_log_level_set("SpiHal", ESP_LOG_DEBUG);
        esp_log_level_set("RgbLed", ESP_LOG_DEBUG);
    } else {
        esp_log_level_set("*", ESP_LOG_INFO);
        esp_log_level_set(TAG, ESP_LOG_INFO);
        esp_log_level_set("BMI270", ESP_LOG_INFO);
        esp_log_level_set("SpiHal", ESP_LOG_INFO);
        esp_log_level_set("RgbLed", ESP_LOG_WARN);  // LED logs only on errors
    }

    ESP_LOGI(TAG, "🚀 === StampFly HAL - BMI270 1600Hz IMU Streaming ===");
    ESP_LOGI(TAG, "Features: 1600Hz ODR + 500Hz Teleplot Streaming");

    // === Phase 1: NVS Flash ===
    ESP_LOGI(TAG, "📝 Phase 1: NVS Flash initialization");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS Flash initialized");

    // === Phase 2: System stabilization ===
    ESP_LOGI(TAG, "⏳ Phase 2: System stabilization");
    for (int i = 3; i > 0; i--) {
        ESP_LOGI(TAG, "Starting in %d seconds...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // === Phase 3: BMI270 Complete Initialization ===
    ESP_LOGI(TAG, "🔧 Phase 3: BMI270 Complete Initialization");
    g_bmi270 = new stampfly_hal::BMI270();
    if (!g_bmi270) {
        ESP_LOGE(TAG, "❌ Failed to create BMI270 instance");
        return;
    }

    ret = g_bmi270->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ BMI270 initialization failed: %s", esp_err_to_name(ret));
        delete g_bmi270;
        return;
    }
    ESP_LOGI(TAG, "✅ BMI270 hardware initialized");

    // Configure sensors for 1600Hz ODR
    ret = g_bmi270->configure_sensors();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Sensor configuration failed: %s", esp_err_to_name(ret));
        delete g_bmi270;
        return;
    }
    ESP_LOGI(TAG, "✅ Sensors configured: 1600Hz ODR");

    // === Phase 4: Sensor Data Verification ===
    ESP_LOGI(TAG, "🔍 Phase 4: Sensor data verification");
    stampfly_hal::BMI270::AccelData test_accel;
    stampfly_hal::BMI270::GyroData test_gyro;
    float test_temp;

    ret = g_bmi270->read_accel(test_accel);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Accel test: X=%.2f, Y=%.2f, Z=%.2f m/s²",
                 test_accel.x, test_accel.y, test_accel.z);
    }

    ret = g_bmi270->read_gyro(test_gyro);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Gyro test: X=%.2f, Y=%.2f, Z=%.2f rad/s",
                 test_gyro.x, test_gyro.y, test_gyro.z);
    }

    ret = g_bmi270->read_temperature(test_temp);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Temperature test: %.1f °C", test_temp);
    }

    // === Phase 5: RGB LED Initialization ===
    ESP_LOGI(TAG, "💡 Phase 5: RGB LED initialization");
    g_rgbled = new stampfly_hal::RgbLed();
    if (!g_rgbled) {
        ESP_LOGE(TAG, "❌ Failed to create RgbLed instance");
    } else {
        ret = g_rgbled->init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ RgbLed initialization failed");
        } else {
            ret = g_rgbled->enable();
            if (ret == ESP_OK) {
                g_rgbled->set_color(stampfly_hal::Color::GREEN);  // Green = initialization success
                ESP_LOGI(TAG, "✅ RGB LED initialized (green)");
            }
        }
    }

    // === Phase 6: Task Creation ===
    ESP_LOGI(TAG, "🚀 Phase 6: Creating high-performance tasks");

    // Create IMU streaming task (priority 5, CPU1)
    xTaskCreatePinnedToCore(
        task_imu_streaming,
        "IMU_Stream",
        4096,
        NULL,
        5,
        &task_imu_streaming_handle,
        1  // CPU1
    );
    ESP_LOGI(TAG, "✅ IMU streaming task created (500Hz, CPU1)");

    // Create LED status task (priority 3, CPU0)
    if (g_rgbled) {
        xTaskCreatePinnedToCore(
            task_led_status,
            "LED_Status",
            2048,
            NULL,
            3,
            &task_led_status_handle,
            0  // CPU0
        );
        if (debug_mode) {
            ESP_LOGI(TAG, "✅ LED status task created (1Hz, CPU0)");
        }
    }

    ESP_LOGI(TAG, "🎉 === All Systems Operational ===");
    ESP_LOGI(TAG, "📡 Streaming IMU data at 500Hz (Teleplot format)");
    if (!debug_mode) {
        ESP_LOGI(TAG, "ℹ️  Debug mode OFF - only streaming data shown");
        ESP_LOGI(TAG, "ℹ️  Set debug_mode=true for detailed logging");
    }
}
/*
 * StampFly HAL Main Application - BMI270 IMU Data Streaming
 * ESP-IDF v5.4.1 Compatible
 * Target: ESP32-S3 (StampFly Hardware)
 * Features: Complete BMI270 initialization + Teleplot IMU data streaming at 500Hz
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

// Control flags
static bool initialization_complete = false;
static bool debug_mode = true;  // Set to true for debug messages - ENABLED for debugging

// High-frequency IMU data streaming task (500Hz) - Teleplot format
void task_imu_streaming(void* pvParameters) {
    if (debug_mode) {
        ESP_LOGI(TAG, "IMU streaming task started at 500Hz");
    }

    // Wait for initialization to complete
    while (!initialization_complete) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);  // 500Hz = 2ms period

    uint32_t sample_count = 0;
    uint32_t error_count = 0;

    while (1) {
        if (g_bmi270) {
            stampfly_hal::BMI270::AccelData accel;
            stampfly_hal::BMI270::GyroData gyro;
            float temperature;

            // Read sensor data
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

                sample_count++;

                // Status update every 1000 samples (2 seconds at 500Hz)
                if (debug_mode && (sample_count % 1000 == 0)) {
                    ESP_LOGI(TAG, "IMU samples: %lu, errors: %lu", sample_count, error_count);
                }
            } else {
                error_count++;
                if (debug_mode) {
                    ESP_LOGE(TAG, "IMU read error - Accel: %s, Gyro: %s, Temp: %s",
                             esp_err_to_name(ret_accel),
                             esp_err_to_name(ret_gyro),
                             esp_err_to_name(ret_temp));
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// LED status task (debug mode only)
void task_led_status(void* pvParameters) {
    if (debug_mode) {
        ESP_LOGI(TAG, "LED status task started");
    }

    // Wait for initialization to complete
    while (!initialization_complete) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // Every 1 second

    uint8_t color_index = 0;
    const uint32_t colors[] = {
        0xFF0000,  // Red
        0x00FF00,  // Green
        0x0000FF,  // Blue
        0xFFFF00,  // Yellow
        0xFF00FF,  // Magenta
        0x00FFFF   // Cyan
    };
    const size_t num_colors = sizeof(colors) / sizeof(colors[0]);

    while (1) {
        if (g_rgbled) {
            g_rgbled->set_color(colors[color_index]);
            if (debug_mode) {
                ESP_LOGI(TAG, "LED color changed to: 0x%06lX", colors[color_index]);
            }
            color_index = (color_index + 1) % num_colors;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

extern "C" void app_main(void) {
    // Set ESP_LOG level to DEBUG for all components
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("BMI270", ESP_LOG_DEBUG);
    esp_log_level_set("SpiHal", ESP_LOG_DEBUG);
    esp_log_level_set("RgbLed", ESP_LOG_DEBUG);
    esp_log_level_set("StampS3Led", ESP_LOG_DEBUG);
    esp_log_level_set("HALBase", ESP_LOG_DEBUG);
    esp_log_level_set("UartHal", ESP_LOG_DEBUG);
    esp_log_level_set("I2cHal", ESP_LOG_DEBUG);
    esp_log_level_set("GPIO", ESP_LOG_DEBUG);
    esp_log_level_set("BMM150", ESP_LOG_DEBUG);
    esp_log_level_set("BMP280", ESP_LOG_DEBUG);
    esp_log_level_set("VL53L3CX", ESP_LOG_DEBUG);
    esp_log_level_set("PMW3901", ESP_LOG_DEBUG);
    esp_log_level_set("INA3221", ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "🚀 === StampFly HAL - BMI270 IMU Data Streaming Application ===");
    ESP_LOGI(TAG, "Features: Complete BMI270 initialization + Teleplot streaming at 500Hz");
    ESP_LOGI(TAG, "📝 ESP_LOG Debug Mode: ENABLED for all components");

    // Initialize NVS
    ESP_LOGI(TAG, "📝 Phase 1: NVS Flash initialization");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS Flash initialized successfully");

    // System stabilization
    ESP_LOGI(TAG, "⏳ Phase 2: System stabilization (3 seconds)...");
    for (int i = 3; i > 0; i--) {
        ESP_LOGI(TAG, "Starting BMI270 initialization in %d seconds...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Initialize BMI270 with complete sequence
    ESP_LOGI(TAG, "🔧 Phase 3: BMI270 Complete Initialization Sequence");
    g_bmi270 = new stampfly_hal::BMI270();
    if (!g_bmi270) {
        ESP_LOGE(TAG, "❌ Failed to create BMI270 instance");
        return;
    }

    ESP_LOGI(TAG, "Initializing BMI270 with complete Bosch specification sequence...");
    ret = g_bmi270->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ BMI270 initialization failed: %s", esp_err_to_name(ret));
        delete g_bmi270;
        g_bmi270 = nullptr;
        return;
    }

    ret = g_bmi270->configure();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ BMI270 configuration failed: %s", esp_err_to_name(ret));
        delete g_bmi270;
        g_bmi270 = nullptr;
        return;
    }

    ret = g_bmi270->enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ BMI270 enable failed: %s", esp_err_to_name(ret));
        delete g_bmi270;
        g_bmi270 = nullptr;
        return;
    }

    // Verify sensor data availability
    ESP_LOGI(TAG, "🔍 Phase 4: Sensor data verification");
    stampfly_hal::BMI270::AccelData test_accel;
    stampfly_hal::BMI270::GyroData test_gyro;
    float test_temp;

    ret = g_bmi270->read_accel(test_accel);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Accelerometer test: X=%.3f, Y=%.3f, Z=%.3f m/s²",
                 test_accel.x, test_accel.y, test_accel.z);
    } else {
        ESP_LOGE(TAG, "❌ Accelerometer read failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = g_bmi270->read_gyro(test_gyro);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Gyroscope test: X=%.3f, Y=%.3f, Z=%.3f rad/s",
                 test_gyro.x, test_gyro.y, test_gyro.z);
    } else {
        ESP_LOGE(TAG, "❌ Gyroscope read failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = g_bmi270->read_temperature(test_temp);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Temperature test: %.2f°C", test_temp);
    } else {
        ESP_LOGE(TAG, "❌ Temperature read failed: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize RGB LED
    ESP_LOGI(TAG, "💡 Phase 5: RGB LED initialization");
    g_rgbled = new stampfly_hal::RgbLed();
    if (g_rgbled) {
        ret = g_rgbled->init();
        if (ret == ESP_OK) {
            g_rgbled->enable();
            g_rgbled->set_color(0x00FF00);  // Green for success
            ESP_LOGI(TAG, "✅ RGB LED initialized successfully (Green)");
        } else {
            ESP_LOGW(TAG, "⚠️ RGB LED initialization failed: %s", esp_err_to_name(ret));
            delete g_rgbled;
            g_rgbled = nullptr;
        }
    }

    // Create high-performance tasks
    ESP_LOGI(TAG, "🔧 Phase 6: Creating high-performance tasks");

    // High-priority IMU streaming task (500Hz)
    xTaskCreatePinnedToCore(
        task_imu_streaming,
        "imu_stream",
        4096,
        nullptr,
        5,  // High priority for real-time streaming
        nullptr,
        1   // CPU1 for better performance
    );
    ESP_LOGI(TAG, "✅ IMU streaming task created (500Hz, CPU1, Priority 5)");

    // LED status task
    if (g_rgbled) {
        xTaskCreatePinnedToCore(
            task_led_status,
            "led_status",
            2048,
            nullptr,
            3,  // Lower priority
            nullptr,
            0   // CPU0
        );
        ESP_LOGI(TAG, "✅ LED status task created (1Hz, CPU0, Priority 3)");
    }

    // Set initialization complete flag
    initialization_complete = true;

    ESP_LOGI(TAG, "🎉 === Initialization Complete - Starting IMU Data Streaming ===");
    ESP_LOGI(TAG, "📊 Teleplot format: >accel_x/y/z, >gyro_x/y/z, >temperature");
    ESP_LOGI(TAG, "📡 Streaming frequency: 500Hz (2ms period)");
    ESP_LOGI(TAG, "🔧 Debug mode: %s (change debug_mode variable to enable/disable)",
             debug_mode ? "ENABLED" : "DISABLED");

    // Wait a moment then start streaming
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "🚀 Starting IMU data streaming...");

    // Main task can now be deleted
    vTaskDelete(nullptr);
}
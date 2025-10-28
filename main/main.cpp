/*
 * StampFly HAL Main Application - BMI270 1600Hz IMU Streaming
 * ESP-IDF v5.4.1 Compatible
 * Target: ESP32-S3 (StampFly Hardware)
 * Features: 1600Hz ODR + 500Hz Teleplot Streaming
 * Modes: Polling / Interrupt-driven (switchable)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

// StampFly HAL includes
#include "stampfly_hal.h"

static const char* TAG = "STAMPFLY_MAIN";

// === Configuration Flags ===
// Set to true for interrupt-driven mode, false for polling mode
static bool use_interrupt_mode = false;  // SWITCH HERE: false=polling, true=interrupt

// Debug mode control
static bool debug_mode = false;  // Set to true for detailed logging

// Interrupt mode decimation (output rate reduction)
// Higher value = lower output rate, less serial communication load
// Examples: 1=1600Hz, 2=800Hz, 3=533Hz, 8=200Hz, 16=100Hz
static const uint32_t interrupt_decimation = 16;  // Default: ~100Hz output

// Global instances
static stampfly_hal::BMI270* g_bmi270 = nullptr;
static stampfly_hal::RgbLed* g_rgbled = nullptr;

// Task handles
static TaskHandle_t task_imu_streaming_handle = nullptr;
static TaskHandle_t task_led_status_handle = nullptr;

// Semaphore for interrupt-driven mode
static SemaphoreHandle_t data_ready_semaphore = nullptr;

// === GPIO Interrupt Handler (IRAM) ===
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Release semaphore to wake up IMU task
    xSemaphoreGiveFromISR(data_ready_semaphore, &xHigherPriorityTaskWoken);

    // Yield if a higher priority task was woken
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// === IMU Data Streaming Task (Polling Mode) ===
void task_imu_streaming_polling(void* pvParameters) {
    stampfly_hal::BMI270::AccelData accel;
    stampfly_hal::BMI270::GyroData gyro;
    float temperature;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10);  // 10ms = 100Hz

    uint32_t loop_count = 0;
    uint32_t error_count = 0;

    ESP_LOGI(TAG, "📊 IMU streaming task started (POLLING mode, 100Hz)");

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
            fflush(stdout);  // Force immediate transmission

            loop_count++;

            // Print statistics every 100 samples (1 second at 100Hz)
            if (debug_mode && loop_count % 100 == 0) {
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

// === IMU Data Streaming Task (Interrupt Mode) ===
void task_imu_streaming_interrupt(void* pvParameters) {
    stampfly_hal::BMI270::AccelData accel;
    stampfly_hal::BMI270::GyroData gyro;
    float temperature;

    uint32_t loop_count = 0;
    uint32_t error_count = 0;
    uint32_t timeout_count = 0;
    uint32_t output_count = 0;

    ESP_LOGI(TAG, "📊 IMU streaming task started (INTERRUPT mode)");
    ESP_LOGI(TAG, "📊 Sensor sampling: ~1600Hz, Output rate: ~%luHz (1/%lu decimation)",
             1600 / interrupt_decimation, interrupt_decimation);

    while (1) {
        // Wait for data ready semaphore (released by ISR)
        // Timeout after 100ms to detect potential issues
        if (xSemaphoreTake(data_ready_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_err_t ret_accel = g_bmi270->read_accel(accel);
            esp_err_t ret_gyro = g_bmi270->read_gyro(gyro);
            esp_err_t ret_temp = g_bmi270->read_temperature(temperature);

            if (ret_accel == ESP_OK && ret_gyro == ESP_OK && ret_temp == ESP_OK) {
                loop_count++;

                // Output every Nth sample to reduce serial communication load
                if (loop_count % interrupt_decimation == 0) {
                    // Teleplot format output
                    printf(">accel_x:%.3f\n", accel.x);
                    printf(">accel_y:%.3f\n", accel.y);
                    printf(">accel_z:%.3f\n", accel.z);
                    printf(">gyro_x:%.3f\n", gyro.x);
                    printf(">gyro_y:%.3f\n", gyro.y);
                    printf(">gyro_z:%.3f\n", gyro.z);
                    printf(">temperature:%.2f\n", temperature);
                    fflush(stdout);  // Force immediate transmission

                    output_count++;
                }

                // Print statistics every 1600 samples (~1 second)
                if (debug_mode && loop_count % 1600 == 0) {
                    ESP_LOGI(TAG, "📊 Sampled: %lu, Output: %lu, errors: %lu, timeouts: %lu",
                             loop_count, output_count, error_count, timeout_count);
                }
            } else {
                error_count++;
                if (debug_mode) {
                    ESP_LOGE(TAG, "Failed to read IMU data (errors: %lu)", error_count);
                }
            }
        } else {
            // Semaphore timeout
            timeout_count++;
            if (debug_mode && timeout_count % 10 == 0) {
                ESP_LOGW(TAG, "Data ready timeout (count: %lu)", timeout_count);
            }
        }
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
    ESP_LOGI(TAG, "Features: 1600Hz ODR + Teleplot Streaming");
    ESP_LOGI(TAG, "Mode: %s", use_interrupt_mode ? "INTERRUPT-DRIVEN" : "POLLING");

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

    // === Phase 4.5: Interrupt Configuration (if enabled) ===
    if (use_interrupt_mode) {
        ESP_LOGI(TAG, "⚡ Phase 4.5: Interrupt mode configuration");

        // Create binary semaphore for data ready signal
        data_ready_semaphore = xSemaphoreCreateBinary();
        if (!data_ready_semaphore) {
            ESP_LOGE(TAG, "❌ Failed to create data ready semaphore");
            return;
        }
        ESP_LOGI(TAG, "✅ Data ready semaphore created");

        // Configure GPIO interrupt
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_POSEDGE;  // Rising edge (active high)
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << stampfly_hal::BMI270::GPIO_INT1_PIN);
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Failed to configure GPIO%d for interrupt",
                     stampfly_hal::BMI270::GPIO_INT1_PIN);
            return;
        }
        ESP_LOGI(TAG, "✅ GPIO%d configured for interrupt (rising edge)",
                 stampfly_hal::BMI270::GPIO_INT1_PIN);

        // Install GPIO ISR service
        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            // ESP_ERR_INVALID_STATE means already installed, which is OK
            ESP_LOGE(TAG, "❌ Failed to install GPIO ISR service");
            return;
        }

        // Add ISR handler for INT1 pin
        ret = gpio_isr_handler_add((gpio_num_t)stampfly_hal::BMI270::GPIO_INT1_PIN,
                                    gpio_isr_handler, nullptr);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Failed to add GPIO ISR handler");
            return;
        }
        ESP_LOGI(TAG, "✅ GPIO ISR handler registered");

        // Configure BMI270 data ready interrupt
        ret = g_bmi270->configure_data_ready_interrupt();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Failed to configure BMI270 data ready interrupt");
            return;
        }
        ESP_LOGI(TAG, "✅ BMI270 data ready interrupt configured");
    } else {
        ESP_LOGI(TAG, "ℹ️  Polling mode - skipping interrupt configuration");
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

    // Create IMU streaming task (mode-dependent, priority 5, CPU1)
    if (use_interrupt_mode) {
        xTaskCreatePinnedToCore(
            task_imu_streaming_interrupt,
            "IMU_Int",
            4096,
            NULL,
            5,
            &task_imu_streaming_handle,
            1  // CPU1
        );
        ESP_LOGI(TAG, "✅ IMU streaming task created (INTERRUPT mode, CPU1)");
        ESP_LOGI(TAG, "   Sampling: ~1600Hz, Output: ~%luHz (decimation 1/%lu)",
                 1600 / interrupt_decimation, interrupt_decimation);
    } else {
        xTaskCreatePinnedToCore(
            task_imu_streaming_polling,
            "IMU_Poll",
            4096,
            NULL,
            5,
            &task_imu_streaming_handle,
            1  // CPU1
        );
        ESP_LOGI(TAG, "✅ IMU streaming task created (POLLING mode, 100Hz, CPU1)");
    }

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
    ESP_LOGI(TAG, "📡 Streaming IMU data (Teleplot format)");
    if (use_interrupt_mode) {
        ESP_LOGI(TAG, "📊 Mode: INTERRUPT (Sampling: 1600Hz, Output: ~%luHz)",
                 1600 / interrupt_decimation);
    } else {
        ESP_LOGI(TAG, "📊 Mode: POLLING (100Hz)");
    }
    if (!debug_mode) {
        ESP_LOGI(TAG, "ℹ️  Debug mode OFF - only streaming data shown");
        ESP_LOGI(TAG, "ℹ️  Set debug_mode=true for detailed logging");
    }
    ESP_LOGI(TAG, "ℹ️  Toggle mode: Set use_interrupt_mode = %s in main.cpp",
             use_interrupt_mode ? "false" : "true");
}
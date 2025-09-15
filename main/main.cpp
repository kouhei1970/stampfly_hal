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
#include "driver/usb_serial_jtag.h"

// StampFly HAL includes
#include "uart_hal.h"
#include "spi_hal.h"
#include "i2c_hal.h"
#include "stampfly_rgbled.h"

static const char* TAG = "STAMPFLY_HAL";

/**
 * @brief USB-CDCコンソール初期化
 * @return esp_err_t 初期化結果
 */
esp_err_t init_usb_cdc_console(void)
{
    // USB Serial JTAG ドライバー設定
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();

    // ドライバーインストール
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));

    // 少し待機（USB接続安定化）
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

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

    // ESP Timerは自動初期化済み（app_main前に初期化される）

    // USB-CDCコンソール初期化
    ESP_ERROR_CHECK(init_usb_cdc_console());

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
    
    // USB-CDC接続待機（最大3秒）
    printf("\n");
    printf("Waiting for USB-CDC connection...\n");
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (usb_serial_jtag_is_connected()) {
            printf("USB-CDC connected!\n");
            break;
        }
    }

    // StampFly情報表示
    printf("\n");
    printf("========================================\n");
    printf("  StampFly HAL System\n");
    printf("  ESP-IDF Version: %s\n", esp_get_idf_version());
    printf("  Target: ESP32-S3\n");
    printf("  HAL Version: 1.0.0-dev\n");
    printf("  Console: USB-CDC Serial\n");
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
    printf("USB-CDC Status: %s\n",
           usb_serial_jtag_is_connected() ? "Connected" : "Disconnected");

    uint32_t loop_count = 0;
    bool last_usb_state = usb_serial_jtag_is_connected();

    while (1) {
        // USB接続状態確認
        bool current_usb_state = usb_serial_jtag_is_connected();
        if (current_usb_state != last_usb_state) {
            printf("\n[USB-CDC] Connection status changed: %s\n",
                   current_usb_state ? "Connected" : "Disconnected");
            last_usb_state = current_usb_state;
        }

        // 基本動作ループ（1秒間隔）
        if (current_usb_state) {
            ESP_LOGI(TAG, "Loop %lu - System running normally", (unsigned long)loop_count);

            // printf動作確認
            printf("StampFly HAL: Loop %lu, Free heap: %lu bytes, USB: OK\n",
                   (unsigned long)loop_count, (unsigned long)esp_get_free_heap_size());
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 定期タスクのデモ
static const char *TAGP = "PERIODIC_TASKS";


// タスクの実行回数をカウント
static uint32_t task_500hz_count = 0;
static uint32_t task_30hz_count = 0;

// 500Hz周期タスク (2ms周期)
void task_500hz(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(2);  // 2ms = 500Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // タスクの処理
        task_500hz_count++;
        
        // 10秒ごとにログ出力（頻繁なログ出力を避けるため）
        if (task_500hz_count % 5000 == 0) {
            ESP_LOGI(TAGP, "500Hz Task: count = %lu", task_500hz_count);
        }
        
        // ここに500Hzで実行したい処理を記述
        // 例: センサーデータの読み取り、制御演算など
        
        // 次の周期まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 30Hz周期タスク (約33.33ms周期)
void task_30hz(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(33);  // 33ms ≈ 30Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // タスクの処理
        task_30hz_count++;
        
        // 1秒ごとにログ出力
        if (task_30hz_count % 30 == 0) {
            ESP_LOGI(TAGP, "30Hz Task: count = %lu", task_30hz_count);
        }
        
        // ここに30Hzで実行したい処理を記述
        // 例: ディスプレイ更新、通信処理など
        
        // 次の周期まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 統計情報を表示するタスク（オプション）
void task_monitor(void *pvParameters)
{
    static uint32_t counter = 0;
    // RGBLED色のデモ配列（5秒ごとに切り替え）
    static const stampfly_color_t demo_colors[] = {
        STAMPFLY_COLOR_RED,     // 赤
        STAMPFLY_COLOR_GREEN,   // 緑
        STAMPFLY_COLOR_BLUE,    // 青
        STAMPFLY_COLOR_YELLOW,  // 黄
        STAMPFLY_COLOR_PURPLE,  // 紫
        STAMPFLY_COLOR_CYAN,    // シアン
        STAMPFLY_COLOR_ORANGE,  // オレンジ
        STAMPFLY_COLOR_PINK,    // ピンク
        STAMPFLY_COLOR_WHITE,   // 白
        STAMPFLY_COLOR_OFF      // 消灯
    };
    static const size_t num_colors = sizeof(demo_colors) / sizeof(demo_colors[0]);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒ごとに表示
        counter++;
        ESP_LOGI(TAGP, "=== Task Statistics ===");
        ESP_LOGI(TAGP, "Monitor time: %lu", counter*5);
        ESP_LOGI(TAGP, "500Hz Task: %lu sec",
                 (task_500hz_count/2500)*5);
        ESP_LOGI(TAGP, "30Hz Task: %lu sec",
                 (task_30hz_count/150)*5);
        ESP_LOGI(TAGP, "Free Heap: %lu bytes", esp_get_free_heap_size());

        // RGBLED色変更デモ
        stampfly_color_t current_color = demo_colors[(counter - 1) % num_colors];
        esp_err_t led_ret = stampfly_rgbled_set_preset_color(current_color);
        if (led_ret == ESP_OK) {
            ESP_LOGI(TAGP, "RGBLED Demo: Color changed to 0x%06lX", (unsigned long)current_color);
        } else {
            ESP_LOGW(TAGP, "RGBLED Demo: Failed to change color (not initialized?)");
        }

        ESP_LOGI(TAGP, "======================");
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

    // RGBLED HAL初期化
    esp_err_t led_ret = stampfly_rgbled_init();
    if (led_ret == ESP_OK) {
        ESP_LOGI(TAG, "RGBLED HAL initialized successfully");
        // 初期化成功を示すために白で点灯
        stampfly_rgbled_set_preset_color(STAMPFLY_COLOR_WHITE);
    } else {
        ESP_LOGW(TAG, "RGBLED HAL initialization failed: %s", esp_err_to_name(led_ret));
    }

    #if 1
    //RTOS タスクの作成例
    ESP_LOGI(TAG, "Starting periodic tasks demo");
        
    // 500Hzタスクを作成（高優先度）
    xTaskCreatePinnedToCore(
        task_500hz,           // タスク関数
        "task_500hz",         // タスク名
        4096,                 // スタックサイズ
        NULL,                 // パラメータ
        5,                    // 優先度（高い）
        NULL,                 // タスクハンドル
        1                     // CPU1で実行
    );
    
    // 30Hzタスクを作成（中優先度）
    xTaskCreatePinnedToCore(
        task_30hz,            // タスク関数
        "task_30hz",          // タスク名
        4096,                 // スタックサイズ
        NULL,                 // パラメータ
        3,                    // 優先度（中程度）
        NULL,                 // タスクハンドル
        1                     // CPU1で実行
    );
    
    // モニタータスクを作成（低優先度）
    xTaskCreate(
        task_monitor,         // タスク関数
        "task_monitor",       // タスク名
        2048,                 // スタックサイズ
        NULL,                 // パラメータ
        1,                    // 優先度（低い）
        NULL                  // タスクハンドル
    );
    
    ESP_LOGI(TAG, "All tasks created successfully");

    #else
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
    #endif
}
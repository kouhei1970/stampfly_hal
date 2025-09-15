/*
 * StampFly I2C HAL Implementation
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#include "i2c_hal.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace stampfly_hal {

// StampFly搭載I2Cデバイス情報
static const I2CDeviceInfo stampfly_devices[] = {
    {0x10, 1000, "BMM150"},     // BMM150 3軸磁気センサー
    {0x76, 1000, "BMP280"},     // BMP280 気圧センサー
    {0x29, 1000, "VL53LX_1"},   // VL53LX ToF距離センサー 1
    {0x29, 1000, "VL53LX_2"},   // VL53LX ToF距離センサー 2（アドレス変更要）
    {0x40, 1000, "INA3221"}     // INA3221 3ch電力モニター
};

static const size_t stampfly_device_count = sizeof(stampfly_devices) / sizeof(stampfly_devices[0]);

I2cHal::I2cHal(const I2CConfig& config) 
    : HALBase("I2C_HAL"), config_(config), bus_handle_(nullptr), bus_initialized_(false) 
{
}

I2cHal::~I2cHal() 
{
    if (bus_initialized_ && bus_handle_) {
        i2c_del_master_bus(bus_handle_);
    }
}

esp_err_t I2cHal::init() 
{
    if (is_initialized()) {
        log(ESP_LOG_WARN, "Already initialized");
        return ESP_OK;
    }

    log(ESP_LOG_INFO, "Initializing I2C port %d", config_.port);

    // ピン設定確認
    esp_err_t ret = validate_pins();
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    // バス初期化
    ret = init_bus();
    if (ret != ESP_OK) {
        return set_error(ret);
    }

    set_initialized(true);
    log(ESP_LOG_INFO, "I2C port %d initialized successfully", config_.port);
    
    return ESP_OK;
}

esp_err_t I2cHal::reset()
{
    if (bus_initialized_ && bus_handle_) {
        i2c_del_master_bus(bus_handle_);
        bus_handle_ = nullptr;
        bus_initialized_ = false;
        set_initialized(false);
        set_enabled(false);
        vTaskDelay(pdMS_TO_TICKS(10));
        return init();
    }
    return ESP_OK;
}

esp_err_t I2cHal::configure() 
{
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    log(ESP_LOG_INFO, "I2C port %d configured: %lu Hz", config_.port, config_.clk_speed_hz);
    return ESP_OK;
}

int I2cHal::scan_devices(uint8_t* found_devices, size_t max_devices) 
{
    if (!is_initialized() || !is_enabled()) {
        return -1;
    }

    if (!found_devices || max_devices == 0) {
        return -1;
    }

    log(ESP_LOG_INFO, "Scanning I2C devices...");
    
    int found_count = 0;
    
    // 0x08-0x77の範囲でスキャン（一般的なI2Cアドレス範囲）
    for (uint8_t addr = 0x08; addr <= 0x77 && found_count < max_devices; addr++) {
        if (is_device_present(addr)) {
            found_devices[found_count] = addr;
            found_count++;
            log(ESP_LOG_INFO, "Found device at address 0x%02X", addr);
        }
    }

    log(ESP_LOG_INFO, "I2C scan completed: %d devices found", found_count);
    return found_count;
}

bool I2cHal::is_device_present(uint8_t device_addr) 
{
    if (!is_initialized() || !is_enabled()) {
        return false;
    }

    // 簡単なpingテスト（0バイト書き込み）
    esp_err_t ret = device_communicate(device_addr, nullptr, 0, nullptr, 0, 100);
    return (ret == ESP_OK);
}

esp_err_t I2cHal::read_register(uint8_t device_addr, uint8_t reg_addr, 
                               uint8_t* data, size_t length, uint32_t timeout_ms) 
{
    if (!data || length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    return device_communicate(device_addr, &reg_addr, 1, data, length, timeout_ms);
}

esp_err_t I2cHal::write_register(uint8_t device_addr, uint8_t reg_addr, 
                                const uint8_t* data, size_t length, uint32_t timeout_ms) 
{
    if (!data || length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    // レジスタアドレス + データを結合したバッファを作成
    uint8_t* write_buffer = new uint8_t[length + 1];
    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data, length);

    esp_err_t ret = device_communicate(device_addr, write_buffer, length + 1, nullptr, 0, timeout_ms);
    
    delete[] write_buffer;
    return ret;
}

esp_err_t I2cHal::read_register_16(uint8_t device_addr, uint16_t reg_addr, 
                                  uint8_t* data, size_t length, uint32_t timeout_ms) 
{
    if (!data || length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    // 16ビットレジスタアドレス（ビッグエンディアン）
    uint8_t reg_buf[2] = {
        static_cast<uint8_t>((reg_addr >> 8) & 0xFF),  // 上位バイト
        static_cast<uint8_t>(reg_addr & 0xFF)           // 下位バイト
    };

    return device_communicate(device_addr, reg_buf, 2, data, length, timeout_ms);
}

esp_err_t I2cHal::write_register_16(uint8_t device_addr, uint16_t reg_addr, 
                                   const uint8_t* data, size_t length, uint32_t timeout_ms) 
{
    if (!data || length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    // 16ビットレジスタアドレス + データを結合
    uint8_t* write_buffer = new uint8_t[length + 2];
    write_buffer[0] = static_cast<uint8_t>((reg_addr >> 8) & 0xFF);  // 上位バイト
    write_buffer[1] = static_cast<uint8_t>(reg_addr & 0xFF);         // 下位バイト
    memcpy(&write_buffer[2], data, length);

    esp_err_t ret = device_communicate(device_addr, write_buffer, length + 2, nullptr, 0, timeout_ms);
    
    delete[] write_buffer;
    return ret;
}

int I2cHal::read_byte(uint8_t device_addr, uint8_t reg_addr, uint32_t timeout_ms) 
{
    uint8_t data;
    esp_err_t ret = read_register(device_addr, reg_addr, &data, 1, timeout_ms);
    
    if (ret != ESP_OK) {
        return -1;
    }
    
    return static_cast<int>(data);
}

esp_err_t I2cHal::write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t value, uint32_t timeout_ms) 
{
    return write_register(device_addr, reg_addr, &value, 1, timeout_ms);
}

esp_err_t I2cHal::read_raw(uint8_t device_addr, uint8_t* data, size_t length, uint32_t timeout_ms) 
{
    if (!data || length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    return device_communicate(device_addr, nullptr, 0, data, length, timeout_ms);
}

esp_err_t I2cHal::write_raw(uint8_t device_addr, const uint8_t* data, size_t length, uint32_t timeout_ms) 
{
    if (!data || length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    return device_communicate(device_addr, data, length, nullptr, 0, timeout_ms);
}

I2CConfig I2cHal::get_stampfly_default_config() 
{
    // StampFly I2C仕様（Arduino版から）:
    // SDA: GPIO3, SCL: GPIO4, 400kHz
    I2CConfig config = {
        .port = I2C_NUM_0,              // I2C0ポート使用
        .sda_pin = GPIO_NUM_3,          // SDA ピン
        .scl_pin = GPIO_NUM_4,          // SCL ピン
        .clk_speed_hz = 400000,         // 400kHz（標準速度）
        .sda_pullup_en = true,          // SDAプルアップ有効
        .scl_pullup_en = true,          // SCLプルアップ有効
        .master_timeout_ms = 1000       // マスタータイムアウト1秒
    };
    return config;
}

const I2CDeviceInfo* I2cHal::get_stampfly_devices() 
{
    return stampfly_devices;
}

size_t I2cHal::get_stampfly_device_count() 
{
    return stampfly_device_count;
}

esp_err_t I2cHal::init_bus() 
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = config_.port;
    bus_config.sda_io_num = config_.sda_pin;
    bus_config.scl_io_num = config_.scl_pin;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.intr_priority = 0;
    bus_config.trans_queue_depth = 10;
    bus_config.flags.enable_internal_pullup = config_.sda_pullup_en && config_.scl_pullup_en;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle_);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    bus_initialized_ = true;
    log(ESP_LOG_INFO, "I2C bus initialized: SDA=%d, SCL=%d, %lu Hz", 
        config_.sda_pin, config_.scl_pin, config_.clk_speed_hz);
    
    return ESP_OK;
}

esp_err_t I2cHal::validate_pins() 
{
    // ピン番号の有効性チェック
    if (config_.sda_pin < 0 || config_.scl_pin < 0) {
        log(ESP_LOG_ERROR, "Invalid pin configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // ピンの重複チェック
    if (config_.sda_pin == config_.scl_pin) {
        log(ESP_LOG_ERROR, "SDA and SCL pins cannot be the same");
        return ESP_ERR_INVALID_ARG;
    }

    log(ESP_LOG_DEBUG, "Pin configuration validated");
    return ESP_OK;
}

esp_err_t I2cHal::device_communicate(uint8_t device_addr, 
                                    const uint8_t* write_data, size_t write_len,
                                    uint8_t* read_data, size_t read_len, 
                                    uint32_t timeout_ms) 
{
    if (!is_initialized() || !is_enabled() || !bus_handle_) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    // デバイス設定
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = device_addr;
    dev_config.scl_speed_hz = config_.clk_speed_hz;

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to add I2C device 0x%02X: %s", device_addr, esp_err_to_name(ret));
        return set_error(ret);
    }

    // 通信実行
    if (write_data && write_len > 0 && read_data && read_len > 0) {
        // 書き込み後読み取り
        ret = i2c_master_transmit_receive(dev_handle, write_data, write_len, 
                                         read_data, read_len, timeout_ms);
    } else if (write_data && write_len > 0) {
        // 書き込みのみ
        ret = i2c_master_transmit(dev_handle, write_data, write_len, timeout_ms);
    } else if (read_data && read_len > 0) {
        // 読み取りのみ
        ret = i2c_master_receive(dev_handle, read_data, read_len, timeout_ms);
    } else {
        // ping（デバイス存在確認）
        ret = i2c_master_probe(bus_handle_, device_addr, timeout_ms);
    }

    // デバイス削除
    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        log(ESP_LOG_DEBUG, "I2C communication failed with device 0x%02X: %s", 
            device_addr, esp_err_to_name(ret));
        return set_error(ret);
    }

    return ESP_OK;
}

} // namespace stampfly_hal
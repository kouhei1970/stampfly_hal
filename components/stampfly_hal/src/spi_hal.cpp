/*
 * StampFly SPI HAL Implementation
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#include "spi_hal.h"
#include <cstring>

namespace stampfly_hal {

SpiHal::SpiHal(const SPIConfig& config) 
    : HALBase("SPI_HAL"), config_(config), bus_initialized_(false) 
{
}

SpiHal::~SpiHal() 
{
    if (bus_initialized_) {
        spi_bus_free(config_.host);
    }
}

esp_err_t SpiHal::init() 
{
    if (initialized_) {
        log(ESP_LOG_WARN, "Already initialized");
        return ESP_OK;
    }

    log(ESP_LOG_INFO, "Initializing SPI host %d", config_.host);

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
    log(ESP_LOG_INFO, "SPI host %d initialized successfully", config_.host);
    
    return ESP_OK;
}

esp_err_t SpiHal::configure() 
{
    if (!initialized_) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    log(ESP_LOG_INFO, "SPI host %d configured", config_.host);
    return ESP_OK;
}

esp_err_t SpiHal::add_device(const SPIDeviceConfig& device_config, spi_device_handle_t* device_handle) 
{
    if (!initialized_ || !enabled_) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    if (!device_handle) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    // デバイス設定構造体の作成
    spi_device_interface_config_t dev_config = {};
    dev_config.command_bits = 8;                          
    dev_config.address_bits = 0;                          
    dev_config.dummy_bits = 0;                            
    dev_config.mode = device_config.mode;                 
    dev_config.clock_source = SPI_CLK_SRC_DEFAULT;        
    dev_config.duty_cycle_pos = 0;                        
    dev_config.cs_ena_pretrans = device_config.cs_ena_pretrans;
    dev_config.cs_ena_posttrans = device_config.cs_ena_posttrans;
    dev_config.clock_speed_hz = static_cast<int>(device_config.clock_speed_hz);
    dev_config.input_delay_ns = 0;                        
    dev_config.spics_io_num = device_config.cs_pin;       
    dev_config.flags = device_config.flags;               
    dev_config.queue_size = device_config.queue_size;     
    dev_config.pre_cb = nullptr;                          
    dev_config.post_cb = nullptr;

    esp_err_t ret = spi_bus_add_device(config_.host, &dev_config, device_handle);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    log(ESP_LOG_INFO, "SPI device added successfully (CS pin: %d, clock: %lu Hz)", 
        device_config.cs_pin, device_config.clock_speed_hz);
    
    return ESP_OK;
}

esp_err_t SpiHal::remove_device(spi_device_handle_t device_handle) 
{
    if (!device_handle) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    esp_err_t ret = spi_bus_remove_device(device_handle);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    log(ESP_LOG_INFO, "SPI device removed successfully");
    return ESP_OK;
}

esp_err_t SpiHal::transmit(spi_device_handle_t device_handle, spi_transaction_t* transaction) 
{
    if (!initialized_ || !enabled_) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    if (!device_handle || !transaction) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    esp_err_t ret = spi_device_transmit(device_handle, transaction);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "SPI transmission failed: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    return ESP_OK;
}

esp_err_t SpiHal::read(spi_device_handle_t device_handle, uint8_t cmd, uint8_t* rx_buffer, size_t rx_length) 
{
    if (!rx_buffer || rx_length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = cmd,
        .addr = 0,
        .length = rx_length * 8,  // ビット単位
        .rxlength = rx_length * 8,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = rx_buffer
    };

    return transmit(device_handle, &transaction);
}

esp_err_t SpiHal::write(spi_device_handle_t device_handle, uint8_t cmd, const uint8_t* tx_buffer, size_t tx_length) 
{
    if (!tx_buffer || tx_length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = cmd,
        .addr = 0,
        .length = tx_length * 8,  // ビット単位
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = tx_buffer,
        .rx_buffer = nullptr
    };

    return transmit(device_handle, &transaction);
}

esp_err_t SpiHal::write_read(spi_device_handle_t device_handle, uint8_t cmd, 
                             const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t length) 
{
    if (!tx_buffer || !rx_buffer || length == 0) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = cmd,
        .addr = 0,
        .length = length * 8,     // ビット単位
        .rxlength = length * 8,   // ビット単位
        .user = nullptr,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };

    return transmit(device_handle, &transaction);
}

SPIConfig SpiHal::get_stampfly_default_config() 
{
    // StampFly SPI仕様（Arduino版から）:
    // MISO: GPIO43, MOSI: GPIO14, SCLK: GPIO44
    SPIConfig config = {
        .host = SPI2_HOST,                  // SPI2を使用
        .miso_pin = GPIO_NUM_43,            // MISO ピン
        .mosi_pin = GPIO_NUM_14,            // MOSI ピン  
        .sclk_pin = GPIO_NUM_44,            // SCLK ピン
        .max_transfer_sz = 4096,            // 最大転送サイズ4KB
        .dma_chan = SPI_DMA_CH_AUTO,        // DMA自動選択
        .flags = SPICOMMON_BUSFLAG_MASTER   // マスターモード
    };
    return config;
}

SPIDeviceConfig SpiHal::get_bmi270_device_config() 
{
    // BMI270 IMU設定（StampFly仕様）:
    // CS: GPIO46, 10MHz, SPI Mode 0
    SPIDeviceConfig config = {
        .cs_pin = GPIO_NUM_46,              // CS ピン
        .clock_speed_hz = 10000000,         // 10MHz
        .mode = 0,                          // SPI Mode 0
        .cs_ena_pretrans = 1,               // CS有効プリトランス
        .cs_ena_posttrans = 1,              // CS有効ポストトランス
        .queue_size = 3,                    // キューサイズ
        .flags = 0                          // フラグなし
    };
    return config;
}

SPIDeviceConfig SpiHal::get_pmw3901_device_config() 
{
    // PMW3901オプティカルフロー設定（StampFly仕様）:
    // CS: GPIO47, 2MHz, SPI Mode 3
    SPIDeviceConfig config = {
        .cs_pin = GPIO_NUM_47,              // CS ピン
        .clock_speed_hz = 2000000,          // 2MHz
        .mode = 3,                          // SPI Mode 3
        .cs_ena_pretrans = 1,               // CS有効プリトランス
        .cs_ena_posttrans = 1,              // CS有効ポストトランス
        .queue_size = 3,                    // キューサイズ
        .flags = 0                          // フラグなし
    };
    return config;
}

esp_err_t SpiHal::init_bus() 
{
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = config_.mosi_pin;
    bus_config.miso_io_num = config_.miso_pin;
    bus_config.sclk_io_num = config_.sclk_pin;
    bus_config.quadwp_io_num = -1;                
    bus_config.quadhd_io_num = -1;                
    bus_config.data4_io_num = -1;                 
    bus_config.data5_io_num = -1;                 
    bus_config.data6_io_num = -1;                 
    bus_config.data7_io_num = -1;                 
    bus_config.max_transfer_sz = config_.max_transfer_sz;
    bus_config.flags = config_.flags;
    bus_config.intr_flags = 0;

    esp_err_t ret = spi_bus_initialize(config_.host, &bus_config, config_.dma_chan);
    if (ret != ESP_OK) {
        log(ESP_LOG_ERROR, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    bus_initialized_ = true;
    log(ESP_LOG_INFO, "SPI bus initialized: MISO=%d, MOSI=%d, SCLK=%d", 
        config_.miso_pin, config_.mosi_pin, config_.sclk_pin);
    
    return ESP_OK;
}

esp_err_t SpiHal::validate_pins() 
{
    // ピン番号の有効性チェック
    if (config_.miso_pin < 0 || config_.mosi_pin < 0 || config_.sclk_pin < 0) {
        log(ESP_LOG_ERROR, "Invalid pin configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // ピンの重複チェック
    if (config_.miso_pin == config_.mosi_pin || 
        config_.miso_pin == config_.sclk_pin || 
        config_.mosi_pin == config_.sclk_pin) {
        log(ESP_LOG_ERROR, "Pin conflict detected");
        return ESP_ERR_INVALID_ARG;
    }

    log(ESP_LOG_DEBUG, "Pin configuration validated");
    return ESP_OK;
}

} // namespace stampfly_hal
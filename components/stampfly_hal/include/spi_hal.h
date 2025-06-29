/*
 * StampFly SPI HAL
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#pragma once

#include "stampfly_hal_base.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

namespace stampfly_hal {

/**
 * @brief SPI設定構造体
 */
struct SPIConfig {
    spi_host_device_t host;         ///< SPIホスト
    gpio_num_t miso_pin;            ///< MISO ピン
    gpio_num_t mosi_pin;            ///< MOSI ピン
    gpio_num_t sclk_pin;            ///< SCLK ピン
    int max_transfer_sz;            ///< 最大転送サイズ
    int dma_chan;                   ///< DMAチャンネル
    uint32_t flags;                 ///< フラグ
};

/**
 * @brief SPIデバイス設定構造体
 */
struct SPIDeviceConfig {
    gpio_num_t cs_pin;              ///< CS ピン
    uint32_t clock_speed_hz;        ///< クロック速度
    uint8_t mode;                   ///< SPIモード (0-3)
    uint8_t cs_ena_pretrans;        ///< CS有効プリトランス
    uint8_t cs_ena_posttrans;       ///< CS有効ポストトランス
    int queue_size;                 ///< キューサイズ
    uint32_t flags;                 ///< デバイスフラグ
};

/**
 * @brief SPI HALクラス
 */
class SpiHal : public HALBase {
public:
    /**
     * @brief コンストラクタ
     * @param config SPI設定
     */
    explicit SpiHal(const SPIConfig& config);

    /**
     * @brief デストラクタ
     */
    ~SpiHal() override;

    /**
     * @brief 初期化
     * @return esp_err_t 初期化結果
     */
    esp_err_t init() override;

    /**
     * @brief 設定
     * @return esp_err_t 設定結果
     */
    esp_err_t configure() override;

    /**
     * @brief デバイス追加
     * @param device_config デバイス設定
     * @param device_handle デバイスハンドル（出力）
     * @return esp_err_t デバイス追加結果
     */
    esp_err_t add_device(const SPIDeviceConfig& device_config, spi_device_handle_t* device_handle);

    /**
     * @brief デバイス削除
     * @param device_handle デバイスハンドル
     * @return esp_err_t デバイス削除結果
     */
    esp_err_t remove_device(spi_device_handle_t device_handle);

    /**
     * @brief 同期転送
     * @param device_handle デバイスハンドル
     * @param transaction トランザクション
     * @return esp_err_t 転送結果
     */
    esp_err_t transmit(spi_device_handle_t device_handle, spi_transaction_t* transaction);

    /**
     * @brief 読み取り専用転送
     * @param device_handle デバイスハンドル
     * @param cmd コマンド
     * @param rx_buffer 受信バッファ
     * @param rx_length 受信長
     * @return esp_err_t 転送結果
     */
    esp_err_t read(spi_device_handle_t device_handle, uint8_t cmd, uint8_t* rx_buffer, size_t rx_length);

    /**
     * @brief 書き込み専用転送
     * @param device_handle デバイスハンドル
     * @param cmd コマンド
     * @param tx_buffer 送信バッファ
     * @param tx_length 送信長
     * @return esp_err_t 転送結果
     */
    esp_err_t write(spi_device_handle_t device_handle, uint8_t cmd, const uint8_t* tx_buffer, size_t tx_length);

    /**
     * @brief 読み書き転送
     * @param device_handle デバイスハンドル
     * @param cmd コマンド
     * @param tx_buffer 送信バッファ
     * @param rx_buffer 受信バッファ
     * @param length データ長
     * @return esp_err_t 転送結果
     */
    esp_err_t write_read(spi_device_handle_t device_handle, uint8_t cmd, 
                         const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t length);

    /**
     * @brief StampFly標準SPI設定取得
     * @return SPIConfig StampFly用のデフォルト設定
     */
    static SPIConfig get_stampfly_default_config();

    /**
     * @brief BMI270用デバイス設定取得
     * @return SPIDeviceConfig BMI270用の設定
     */
    static SPIDeviceConfig get_bmi270_device_config();

    /**
     * @brief PMW3901用デバイス設定取得
     * @return SPIDeviceConfig PMW3901用の設定
     */
    static SPIDeviceConfig get_pmw3901_device_config();

private:
    SPIConfig config_;              ///< SPI設定
    bool bus_initialized_;          ///< バス初期化状態

    /**
     * @brief バス初期化
     * @return esp_err_t バス初期化結果
     */
    esp_err_t init_bus();

    /**
     * @brief ピン設定確認
     * @return esp_err_t ピン設定結果
     */
    esp_err_t validate_pins();
};

} // namespace stampfly_hal
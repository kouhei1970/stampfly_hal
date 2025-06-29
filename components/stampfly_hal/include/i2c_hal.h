/*
 * StampFly I2C HAL
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#pragma once

#include "stampfly_hal_base.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

namespace stampfly_hal {

/**
 * @brief I2C設定構造体
 */
struct I2CConfig {
    i2c_port_t port;                ///< I2Cポート番号
    gpio_num_t sda_pin;             ///< SDA ピン
    gpio_num_t scl_pin;             ///< SCL ピン
    uint32_t clk_speed_hz;          ///< クロック速度（Hz）
    bool sda_pullup_en;             ///< SDAプルアップ有効
    bool scl_pullup_en;             ///< SCLプルアップ有効
    uint32_t master_timeout_ms;     ///< マスタータイムアウト（ms）
};

/**
 * @brief I2Cデバイス情報構造体
 */
struct I2CDeviceInfo {
    uint8_t address;                ///< デバイスアドレス（7ビット）
    uint32_t timeout_ms;            ///< 通信タイムアウト（ms）
    const char* name;               ///< デバイス名（デバッグ用）
};

/**
 * @brief I2C HALクラス
 */
class I2cHal : public HALBase {
public:
    /**
     * @brief コンストラクタ
     * @param config I2C設定
     */
    explicit I2cHal(const I2CConfig& config);

    /**
     * @brief デストラクタ
     */
    ~I2cHal() override;

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
     * @brief デバイススキャン
     * @param found_devices 発見されたデバイスアドレス配列
     * @param max_devices 最大デバイス数
     * @return int 発見されたデバイス数、-1でエラー
     */
    int scan_devices(uint8_t* found_devices, size_t max_devices);

    /**
     * @brief デバイス存在確認
     * @param device_addr デバイスアドレス
     * @return bool true: 存在, false: 存在しない
     */
    bool is_device_present(uint8_t device_addr);

    /**
     * @brief レジスタ読み取り（8ビットレジスタアドレス）
     * @param device_addr デバイスアドレス
     * @param reg_addr レジスタアドレス
     * @param data 読み取りデータバッファ
     * @param length データ長
     * @param timeout_ms タイムアウト（ms）
     * @return esp_err_t 読み取り結果
     */
    esp_err_t read_register(uint8_t device_addr, uint8_t reg_addr, 
                           uint8_t* data, size_t length, uint32_t timeout_ms = 1000);

    /**
     * @brief レジスタ書き込み（8ビットレジスタアドレス）
     * @param device_addr デバイスアドレス
     * @param reg_addr レジスタアドレス
     * @param data 書き込みデータ
     * @param length データ長
     * @param timeout_ms タイムアウト（ms）
     * @return esp_err_t 書き込み結果
     */
    esp_err_t write_register(uint8_t device_addr, uint8_t reg_addr, 
                            const uint8_t* data, size_t length, uint32_t timeout_ms = 1000);

    /**
     * @brief レジスタ読み取り（16ビットレジスタアドレス）
     * @param device_addr デバイスアドレス
     * @param reg_addr レジスタアドレス（16ビット）
     * @param data 読み取りデータバッファ
     * @param length データ長
     * @param timeout_ms タイムアウト（ms）
     * @return esp_err_t 読み取り結果
     */
    esp_err_t read_register_16(uint8_t device_addr, uint16_t reg_addr, 
                              uint8_t* data, size_t length, uint32_t timeout_ms = 1000);

    /**
     * @brief レジスタ書き込み（16ビットレジスタアドレス）
     * @param device_addr デバイスアドレス
     * @param reg_addr レジスタアドレス（16ビット）
     * @param data 書き込みデータ
     * @param length データ長
     * @param timeout_ms タイムアウト（ms）
     * @return esp_err_t 書き込み結果
     */
    esp_err_t write_register_16(uint8_t device_addr, uint16_t reg_addr, 
                               const uint8_t* data, size_t length, uint32_t timeout_ms = 1000);

    /**
     * @brief 1バイトレジスタ読み取り
     * @param device_addr デバイスアドレス
     * @param reg_addr レジスタアドレス
     * @param timeout_ms タイムアウト（ms）
     * @return int 読み取り値（0-255）、-1でエラー
     */
    int read_byte(uint8_t device_addr, uint8_t reg_addr, uint32_t timeout_ms = 1000);

    /**
     * @brief 1バイトレジスタ書き込み
     * @param device_addr デバイスアドレス
     * @param reg_addr レジスタアドレス
     * @param value 書き込み値
     * @param timeout_ms タイムアウト（ms）
     * @return esp_err_t 書き込み結果
     */
    esp_err_t write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t value, uint32_t timeout_ms = 1000);

    /**
     * @brief 生データ読み取り（レジスタアドレスなし）
     * @param device_addr デバイスアドレス
     * @param data 読み取りデータバッファ
     * @param length データ長
     * @param timeout_ms タイムアウト（ms）
     * @return esp_err_t 読み取り結果
     */
    esp_err_t read_raw(uint8_t device_addr, uint8_t* data, size_t length, uint32_t timeout_ms = 1000);

    /**
     * @brief 生データ書き込み（レジスタアドレスなし）
     * @param device_addr デバイスアドレス
     * @param data 書き込みデータ
     * @param length データ長
     * @param timeout_ms タイムアウト（ms）
     * @return esp_err_t 書き込み結果
     */
    esp_err_t write_raw(uint8_t device_addr, const uint8_t* data, size_t length, uint32_t timeout_ms = 1000);

    /**
     * @brief StampFly標準I2C設定取得
     * @return I2CConfig StampFly用のデフォルト設定
     */
    static I2CConfig get_stampfly_default_config();

    /**
     * @brief StampFlyデバイス情報取得
     * @return 配列 StampFly搭載I2Cデバイス情報
     */
    static const I2CDeviceInfo* get_stampfly_devices();

    /**
     * @brief StampFlyデバイス数取得
     * @return size_t デバイス数
     */
    static size_t get_stampfly_device_count();

private:
    I2CConfig config_;              ///< I2C設定
    i2c_master_bus_handle_t bus_handle_;  ///< バスハンドル
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

    /**
     * @brief デバイス通信
     * @param device_addr デバイスアドレス
     * @param write_data 書き込みデータ
     * @param write_len 書き込み長
     * @param read_data 読み取りデータ
     * @param read_len 読み取り長
     * @param timeout_ms タイムアウト
     * @return esp_err_t 通信結果
     */
    esp_err_t device_communicate(uint8_t device_addr, 
                                const uint8_t* write_data, size_t write_len,
                                uint8_t* read_data, size_t read_len, 
                                uint32_t timeout_ms);
};

} // namespace stampfly_hal
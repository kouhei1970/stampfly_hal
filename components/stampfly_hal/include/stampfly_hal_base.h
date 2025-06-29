/*
 * StampFly HAL Base Class
 * ESP-IDF v5.4.1 Compatible
 * 
 * MIT License
 * Copyright (c) 2025 Kouhei Ito
 */

#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include <stdint.h>
#include <stdbool.h>

namespace stampfly_hal {

/**
 * @brief HAL基底クラス
 * 全てのHAL実装クラスが継承する基底クラス
 */
class HALBase {
public:
    /**
     * @brief コンストラクタ
     * @param tag ログ用タグ
     */
    explicit HALBase(const char* tag);

    /**
     * @brief デストラクタ
     */
    virtual ~HALBase() = default;

    /**
     * @brief 初期化
     * @return esp_err_t 初期化結果
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief 設定
     * @return esp_err_t 設定結果
     */
    virtual esp_err_t configure() = 0;

    /**
     * @brief 初期化状態取得
     * @return bool true: 初期化済み, false: 未初期化
     */
    bool is_initialized() const;

    /**
     * @brief 有効状態取得
     * @return bool true: 有効, false: 無効
     */
    bool is_enabled() const;

    /**
     * @brief 有効化
     * @return esp_err_t 有効化結果
     */
    virtual esp_err_t enable();

    /**
     * @brief 無効化
     * @return esp_err_t 無効化結果
     */
    virtual esp_err_t disable();

    /**
     * @brief リセット
     * @return esp_err_t リセット結果
     */
    virtual esp_err_t reset();

    /**
     * @brief ログ出力
     * @param level ログレベル
     * @param format フォーマット文字列
     * @param ... 可変引数
     */
    void log(esp_log_level_t level, const char* format, ...) const;

protected:
    const char* tag_;           ///< ログ用タグ
    bool initialized_;          ///< 初期化状態
    bool enabled_;              ///< 有効状態
    esp_err_t last_error_;      ///< 最後のエラー

    /**
     * @brief エラー設定
     * @param error エラーコード
     * @return esp_err_t 設定したエラーコード
     */
    esp_err_t set_error(esp_err_t error);

    /**
     * @brief 初期化状態設定
     * @param initialized 初期化状態
     */
    void set_initialized(bool initialized);

    /**
     * @brief 有効状態設定
     * @param enabled 有効状態
     */
    void set_enabled(bool enabled);
};

} // namespace stampfly_hal
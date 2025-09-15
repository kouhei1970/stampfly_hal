#pragma once

#include "hal_base.h"
#include <esp_err.h>

namespace stampfly_hal {

/**
 * @brief BMM150 3軸磁気センサーHALクラス（プレースホルダー）
 * Bosch Sensortec BMM150 - 測定範囲: ±1300µT（X,Y軸）、±2047µT（Z軸）
 */
class BMM150 : public HALBase {
public:
    BMM150();
    ~BMM150() = default;

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // BMM150固有機能（プレースホルダー）
    esp_err_t read_mag(float* x, float* y, float* z);
    esp_err_t calibrate();
};

} // namespace stampfly_hal
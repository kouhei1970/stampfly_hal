#pragma once

#include "hal_base.h"
#include <esp_err.h>

namespace stampfly_hal {

/**
 * @brief BMP280 気圧センサーHALクラス（プレースホルダー）
 * Bosch Sensortec BMP280 - 測定範囲: 300-1250hPa、精度±30Pa
 */
class BMP280 : public HALBase {
public:
    BMP280();
    ~BMP280() = default;

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // BMP280固有機能（プレースホルダー）
    esp_err_t read_pressure(float* pressure);
    esp_err_t read_temperature(float* temperature);
    esp_err_t read_altitude(float* altitude);
};

} // namespace stampfly_hal
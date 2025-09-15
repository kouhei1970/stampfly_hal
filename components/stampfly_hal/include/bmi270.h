#pragma once

#include "hal_base.h"
#include <esp_err.h>

namespace stampfly_hal {

/**
 * @brief BMI270 6軸IMUセンサーHALクラス（プレースホルダー）
 * Bosch Sensortec BMI270 - 16ビット3軸ジャイロ＋3軸アクセロメーター
 */
class BMI270 : public HALBase {
public:
    BMI270();
    ~BMI270() = default;

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // BMI270固有機能（プレースホルダー）
    esp_err_t read_accel(float* x, float* y, float* z);
    esp_err_t read_gyro(float* x, float* y, float* z);
    esp_err_t read_temperature(float* temp);
};

} // namespace stampfly_hal
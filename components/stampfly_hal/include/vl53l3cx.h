#pragma once

#include "hal_base.h"
#include <esp_err.h>

namespace stampfly_hal {

/**
 * @brief VL53L3CX ToF距離センサーHALクラス（プレースホルダー）
 * STMicroelectronics VL53L3CX - 測定範囲: 25-3000mm、940nm Class1レーザー
 */
class VL53L3CX : public HALBase {
public:
    VL53L3CX();
    ~VL53L3CX() = default;

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // VL53L3CX固有機能（プレースホルダー）
    esp_err_t read_distance(uint16_t* distance_mm);
    esp_err_t set_timing_budget(uint32_t budget_us);
    esp_err_t set_distance_mode(uint8_t mode);
};

} // namespace stampfly_hal
#pragma once

#include "hal_base.h"
#include <esp_err.h>

namespace stampfly_hal {

/**
 * @brief INA3221 3チャンネル電力モニターHALクラス（プレースホルダー）
 * Texas Instruments INA3221 - 3チャンネル電流・電圧監視、バス電圧0-26V
 */
class INA3221 : public HALBase {
public:
    INA3221();
    ~INA3221() = default;

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // INA3221固有機能（プレースホルダー）
    esp_err_t read_bus_voltage(uint8_t channel, float* voltage);
    esp_err_t read_shunt_voltage(uint8_t channel, float* voltage);
    esp_err_t read_current(uint8_t channel, float* current);
    esp_err_t read_power(uint8_t channel, float* power);
    esp_err_t set_shunt_resistance(uint8_t channel, float resistance_ohm);
};

} // namespace stampfly_hal
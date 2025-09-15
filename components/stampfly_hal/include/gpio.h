#pragma once

#include "hal_base.h"
#include <esp_err.h>
#include <driver/gpio.h>

namespace stampfly_hal {

/**
 * @brief GPIO HALクラス（プレースホルダー）
 * 将来的なGPIO制御機能の実装予定
 */
class Gpio : public HALBase {
public:
    Gpio();
    ~Gpio() = default;

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // GPIO固有機能（プレースホルダー）
    esp_err_t set_pin_mode(gpio_num_t pin, gpio_mode_t mode);
    esp_err_t digital_write(gpio_num_t pin, uint32_t level);
    int digital_read(gpio_num_t pin);
};

} // namespace stampfly_hal
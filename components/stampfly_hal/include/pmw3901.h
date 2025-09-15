#pragma once

#include "hal_base.h"
#include <esp_err.h>

namespace stampfly_hal {

/**
 * @brief PMW3901 オプティカルフローセンサーHALクラス（プレースホルダー）
 * PixArt PMW3901MB-TXQT - 作動範囲: 80mm-無限遠、16ビットモーションデータ
 */
class PMW3901 : public HALBase {
public:
    PMW3901();
    ~PMW3901() = default;

    // HALBase純粋仮想関数の実装
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // PMW3901固有機能（プレースホルダー）
    esp_err_t read_motion(int16_t* delta_x, int16_t* delta_y);
    esp_err_t read_surface_quality(uint8_t* quality);
    esp_err_t set_resolution(uint16_t cpi);
};

} // namespace stampfly_hal
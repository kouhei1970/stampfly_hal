#pragma once

/**
 * @file stampfly_hal.h
 * @brief StampFly HAL統一ヘッダー
 *
 * StampFlyドローンの全HALコンポーネントへの統一アクセスポイント
 */

// 基底クラス
#include "hal_base.h"

// 通信HAL
#include "uart_hal.h"
#include "spi_hal.h"
#include "i2c_hal.h"
#include "gpio.h"

// 周辺機器HAL
#include "rgbled.h"
#include "stamps3_led.h"

// センサーHAL群（今後追加）
#include "bmi270.h"
#include "bmm150.h"
#include "bmp280.h"
#include "vl53l3cx.h"
#include "pmw3901.h"
#include "ina3221.h"

// ユーティリティ
#include "stampfly_memory.h"

// 名前空間エイリアス
namespace stampfly = stampfly_hal;
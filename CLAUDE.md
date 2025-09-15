# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

StampFly HAL開発プロジェクト - M5Stack社のStampFly（ESP32-S3ベースクアッドコプター）用のハードウェア抽象化層を開発するプロジェクトです。
- **主要目的**: 飛行制御学習教材、組み込み制御学習教材、ドローン研究プラットフォーム
- **開発環境**: ESP-IDF v5.4.1、C++オブジェクト指向設計
- **対象ハードウェア**: ESP32-S3、FreeRTOS

## 基本コマンド

### 環境セットアップ
```bash
# ESP-IDF環境の読み込み（必須）
. $HOME/esp/esp-idf/export.sh
```

### ビルド・実行コマンド
```bash
# プロジェクトビルド
idf.py build

# デバイスへフラッシュ＋シリアルモニタ
idf.py flash monitor

# プロジェクト設定
idf.py menuconfig

# クリーンビルド
idf.py fullclean

# 特定コンポーネントのビルド
idf.py build -C components/[component_name]
```

### テストコマンド
```bash
# Pythonテスト実行
python pytest_hello_world.py

# カスタムモニタリングスクリプト
python test_monitor.py
```

## 現在のアーキテクチャ

### 統一HALコンポーネント構成
```
components/
└── stampfly_hal/ (統一HAL)
    ├── CMakeLists.txt
    ├── include/
    │   ├── stampfly_hal.h         # 統一公開ヘッダー
    │   ├── hal_base.h             # 改良された基底クラス
    │   ├── uart_hal.h             # 通信HAL群
    │   ├── spi_hal.h
    │   ├── i2c_hal.h
    │   ├── gpio.h
    │   ├── rgbled.h               # 周辺機器HAL
    │   ├── stamps3_led.h          # StampS3オンボードLED HAL
    │   ├── bmi270.h               # センサーHAL群
    │   ├── bmm150.h
    │   ├── bmp280.h
    │   ├── vl53l3cx.h
    │   ├── pmw3901.h
    │   ├── ina3221.h
    │   └── stampfly_memory.h
    └── src/
        ├── hal_base.cpp           # 改良された基底クラス
        ├── uart.cpp               # 通信HAL群
        ├── spi.cpp
        ├── i2c.cpp
        ├── gpio.cpp
        ├── rgbled.cpp             # RgbLedクラス実装
        ├── stamps3_led.cpp        # StampS3Led実装
        ├── bmi270.cpp             # センサーHAL群（プレースホルダー）
        ├── bmm150.cpp
        ├── bmp280.cpp
        ├── vl53l3cx.cpp
        ├── pmw3901.cpp
        ├── ina3221.cpp
        └── stampfly_memory.cpp    # メモリ監視ユーティリティ
```

### 主要技術要素
- **リアルタイム制御**: 500Hz制御ループ（FreeRTOS）
- **センサーフュージョン**: Madgwick AHRS、カルマンフィルター
- **制御アルゴリズム**: PID制御、MPC（モデル予測制御）
- **通信**: ESP-NOWプロトコル

## 搭載センサー仕様

### BMI270 6軸IMU（Bosch Sensortec）
- 16ビット3軸ジャイロ＋3軸アクセロメーター
- データシート: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf

### BMM150 3軸磁気センサー（Bosch Sensortec）
- 測定範囲: ±1300µT（X,Y軸）、±2047µT（Z軸）
- データシート: https://www.mouser.com/datasheet/2/783/BST-BMM150-DS001-01-786480.pdf

### BMP280 気圧センサー（Bosch Sensortec）
- 測定範囲: 300-1250hPa、精度±30Pa
- データシート: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf

### VL53L3CX ToF距離センサー（STMicroelectronics）
- 測定範囲: 25-3000mm、940nm Class1レーザー
- データシート: https://www.st.com/resource/en/datasheet/vl53l3cx.pdf

### PMW3901MB-TXQT オプティカルフローセンサー（PixArt）
- 作動範囲: 80mm-無限遠、16ビットモーションデータ
- データシート: https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.00-200317_20170331160807_public.pdf

### INA3221 3チャンネル電力モニター（TI）
- 3チャンネル電流・電圧監視、バス電圧0-26V
- 公式ページ: https://www.ti.com/product/INA3221

## 開発規約

### 言語使用方針
- **ドキュメント**: 日本語
- **コード内コメント**: 日本語
- **ログ・メッセージ**: 英語
- **Gitコミット**: 英語

### 設計原則
- オブジェクト指向C++設計（RAII原則適用）
- ESP-IDFエラーハンドリング（ESP_ERROR_CHECK等）
- メーカー純正API活用（C言語APIはC++ラッピング）
- 統一HALコンポーネントによる機能集約

### **作業方針（必須遵守）**
**全ての作業において以下を必ず実施：**

1. **StampFly仕様確認**
   - 作業開始前にStampFlyの実際のハードウェア構成を確認
   - Arduino版コードでの実装を参照
   - 推測や一般的な仕様に依存しない

2. **センサー仕様確認**
   - 各センサーの正確なモデル名、インターフェース、ピンアサインを確認
   - メーカー公式データシートの参照
   - StampFly固有の接続方法を優先

3. **API仕様確認**
   - ESP-IDF APIの正確なバージョン（v5.4.1）仕様を確認
   - メーカー純正センサーAPIの最新仕様確認
   - 非推奨APIや変更点の事前チェック

4. **誤認識防止**
   - 推測による実装を避ける
   - 不明点は必ず調査・確認してから進める
   - 定期的な仕様再確認

### 品質管理フロー
1. **仕様確認**: StampFly/センサー/API仕様の事前確認
2. コーディング完了
3. ビルド成功確認
4. フラッシュ・動作確認
5. 正常動作まで継続デバッグ

## 統一HAL設計仕様

### HALBase基底クラス
```cpp
namespace stampfly_hal {

class HALBase {
public:
    explicit HALBase(const char* name);
    virtual ~HALBase() = default;

    // 必須ライフサイクル
    virtual esp_err_t init() = 0;
    virtual esp_err_t configure() = 0;
    virtual esp_err_t enable() = 0;
    virtual esp_err_t disable() = 0;
    virtual esp_err_t reset() = 0;

    // 状態管理
    bool is_initialized() const;
    bool is_enabled() const;
    esp_err_t get_last_error() const;
    const char* get_name() const;

    // デバッグ支援
    virtual void print_status() const;
    void log(esp_log_level_t level, const char* format, ...) const;

protected:
    esp_err_t set_error(esp_err_t error);
    void set_initialized(bool state);
    void set_enabled(bool state);

private:
    const char* name_;
    bool initialized_ = false;
    bool enabled_ = false;
    esp_err_t last_error_ = ESP_OK;
};

}
```

### 統一アクセスAPI
```cpp
#include "stampfly_hal.h"  // 全機能に統一アクセス

// 通信HAL
stampfly_hal::UartHal uart(config);
stampfly_hal::I2cHal i2c(config);
stampfly_hal::SpiHal spi(config);

// 周辺機器HAL
stampfly_hal::RgbLed rgbled;           // StampFlyドローン側LED
stampfly_hal::StampS3Led stamps3_led;  // StampS3オンボードLED

// センサーHAL（実装済みプレースホルダー）
stampfly_hal::BMI270 imu;
stampfly_hal::BMM150 mag;
stampfly_hal::BMP280 baro;
stampfly_hal::VL53L3CX tof;
stampfly_hal::PMW3901 optical_flow;
stampfly_hal::INA3221 power_monitor;

// ユーティリティ
stampfly_hal::Memory::print_heap_info();
```

## 実装済み機能

### USB-CDC実装
- USBケーブル接続時にシリアルデバイスとして認識
- ESP-IDF v5.4.1 USB Serial JTAG API使用
- USB接続状態監視機能

### RGBLED HAL実装（完全動作版）
- StampFlyオンボードRGBLED（WS2812B）制御
- GPIO39、2個のWS2812B、RMT + GRB色順序
- RgbLedクラス化、HALBase継承、完全初期化対応
- 定義済み色：RED/GREEN/BLUE/YELLOW/PURPLE/CYAN/ORANGE/PINK/WHITE/灰色
- 専用LED色変更タスク（1秒間隔で10色循環）
- 正しいHAL初期化フロー：init() → enable() → set_color()

### 通信HAL実装
- UartHal, SpiHal, I2cHal（完全実装）
- HALBase継承、統一ライフサイクル管理
- ESP-IDFドライバー完全ラッピング

### メモリ監視ユーティリティ
```cpp
stampfly_hal::Memory::print_heap_info();      // ヒープ情報ログ出力
stampfly_hal::Memory::get_free_heap();        // 現在の空きヒープ取得
stampfly_hal::Memory::get_minimum_free_heap(); // 最小空きヒープ取得
stampfly_hal::Memory::get_largest_free_block(); // 最大連続ブロック取得
```

### センサーHALプレースホルダー
- BMI270, BMM150, BMP280, VL53L3CX, PMW3901, INA3221
- 全てHALBase継承、統一インターフェース実装
- 実センサー機能は今後実装予定

### 現在のタスク構成とデモ機能
**リアルタイムタスク:**
- `task_500hz`: 500Hz高頻度制御ループ（優先度5、CPU1固定）
- `task_30hz`: 30Hz中頻度処理（優先度3、CPU1固定）
- `task_monitor`: システム統計表示（5秒間隔、優先度1）
- `task_led_demo`: RGBLED色変更デモ（1秒間隔10色循環、優先度2）

**動作確認機能:**
- USB-CDC自動接続・状態監視
- RGBLED完全初期化フローテスト
- メモリ使用量監視
- タスク実行統計表示

プロジェクトはビルド・実行可能で、全HAL機能が正常動作します。

### StampS3オンボードLED制御機能（2025-09-15追加）

**実装機能:**
- `StampS3Led` HALクラス（GPIO21固定、WS2812B 1個制御専用）
- StampFlyドローン側LED（2個）とStampS3側LED（1個）の同時制御
- デュアルLED同期色変更デモ（10色循環、1秒間隔）

**技術仕様:**
```cpp
// StampS3 LED仕様
- GPIO: 21番ピン（固定）
- LED個数: 1個
- 制御IC: WS2812B-2020
- 色順序: GRB
- 初期化表示: 青色（StampS3識別用）

// 統合API例
stampfly_hal::StampS3Led stamps3_led;
stamps3_led.init();
stamps3_led.enable();
stamps3_led.set_red();    // 定義済み色設定
stamps3_led.set_color(0xFF9933);  // 直接色指定
```

**ファイル構成:**
- `components/stampfly_hal/include/stamps3_led.h`: StampS3Led HALクラス定義
- `components/stampfly_hal/src/stamps3_led.cpp`: 実装
- `main/main.cpp`: デュアルLED制御タスク統合済み

**動作確認済み機能:**
- ✅ StampS3オンボードLED制御（GPIO21）
- ✅ StampFly LEDとの同期制御
- ✅ ビルド・リンク正常完了
- ✅ HALBase統一インターフェース準拠

## 依存関係管理

### ESP-IDF依存関係システム
- **`dependencies.lock`**: ESP-IDFが直接参照する依存関係ロックファイル
  - バージョン固定: `espressif/led_strip v3.0.1~1`
  - 再現可能ビルド保証
  - チーム開発での環境統一
- **`managed_components/`**: 自動生成キャッシュフォルダ
  - ビルド時にESP-IDFが自動作成・管理
  - `.gitignore`で除外済み（適切な設定）
  - 削除しても自動復元される

## プロジェクト構造

### 最終的なプロジェクト構造
```
stampfly_hal/
├── .git/
├── .gitignore                  # managed_components/除外設定済み
├── CLAUDE.md                   # プロジェクト仕様・実装記録
├── CMakeLists.txt              # 統合HALのみ参照
├── components/
│   └── stampfly_hal/           # 統一HALコンポーネント（完全統合済み）
├── dependencies.lock           # ESP-IDF依存関係管理（必須）
├── main/                       # 統合APIに対応済み
├── managed_components/         # 自動生成（.gitignore除外）
├── sdkconfig*                  # ESP-IDF設定
└── README.md
```

### 実現された効果
- ✅ **コード保守性**: 単一コンポーネントによる一元管理
- ✅ **API一貫性**: 全HALで統一されたHALBaseインターフェース
- ✅ **開発効率**: 1つのヘッダー (`stampfly_hal.h`) で全機能アクセス
- ✅ **拡張性**: 新センサー追加の標準化（プレースホルダー準備済み）
- ✅ **学習効果**: 統一されたパターンでの理解促進
- ✅ **ESP-IDF準拠**: 標準的なコンポーネント設計に準拠

## 今後の開発予定

### センサーHAL実装ロードマップ
1. BMI270 (6軸IMU) - 姿勢制御基盤
2. BMP280 (気圧センサー) - 高度制御
3. VL53L3CX (ToF距離センサー) - 障害物回避
4. BMM150 (3軸磁気センサー) - ヘディング制御
5. PMW3901 (オプティカルフローセンサー) - 位置制御
6. INA3221 (3チャンネル電力モニター) - バッテリー管理

### 制御システム統合
- センサーフュージョン機能（Madgwick AHRS, Kalman Filter）
- 飛行制御アルゴリズム統合（PID, MPC）
- 実時間データ処理パイプライン（500Hz制御ループ）

## 参考リソース
- https://github.com/kouhei1970/StampFly_sandbox
- ESP-IDF Programming Guide
- M5Stack公式ドキュメント
- Bosch Sensortec センサーAPI群
- STMicroelectronics FlightSense技術資料
- ESP-Drone (Espressif公式)
- ArduPilot/ArduCopter

## ライセンス
MIT License（Claudeは含めない）
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

## アーキテクチャ

### コンポーネント構成
```
components/
├── stampfly_core/      # I2C/SPI/UART HAL
├── stampfly_rgbled/    # RGBLED制御HAL
├── stampfly_sensors/   # センサードライバー群
├── stampfly_control/   # 制御アルゴリズム
└── stampfly_comm/      # 通信プロトコル
```

### 主要技術要素
- **リアルタイム制御**: 400Hz制御ループ（FreeRTOS）
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
- コンポーネント化による機能分離

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

## 実装記録

### USB-CDC実装（2025-09-15）

**目的**: USBケーブル接続時にシリアルデバイスとして認識されるよう改造

**実装内容**:
1. **sdkconfig.defaults設定**
   - `CONFIG_ESP_CONSOLE_USB_CDC=y`: USB-CDCコンソール有効化
   - `CONFIG_ESP_CONSOLE_UART=n`: UARTコンソール無効化
   - バッファサイズ256バイトに拡大

2. **main.cpp機能追加**
   - `init_usb_cdc_console()`: USB Serial JTAGドライバー初期化
   - USB接続状態監視機能
   - USB接続状態変化の検出・表示

3. **CMakeLists.txt更新**
   - `esp_driver_usb_serial_jtag`コンポーネントをPRIV_REQUIRESに追加
   - VFSコンポーネント依存関係追加

**技術仕様**:
- ESP-IDF v5.4.1 USB Serial JTAG API使用
- デフォルト設定（tx_buffer_size: 256, rx_buffer_size: 256）
- USB接続検出: `usb_serial_jtag_is_connected()`

**使用方法**:
```bash
# ビルド・フラッシュ
. $HOME/esp/esp-idf/export.sh
idf.py build
idf.py flash monitor
```

**動作確認**:
- ✅ ビルド成功
- ✅ USB Serial JTAGドライバー初期化
- ✅ USB接続状態監視
- ✅ コンソール出力（USB-CDC経由）

### RGBLED HAL実装（2025-09-15）

**目的**: StampFlyのオンボードRGBLED（WS2812B）を制御するHAL実装

**実装内容**:
1. **stampfly_rgbledコンポーネント作成**
   - `components/stampfly_rgbled/`: 独立コンポーネント化
   - `stampfly_rgbled.h`: HAL API定義
   - `stampfly_rgbled.cpp`: WS2812B制御実装

2. **ESP-IDF led_strip依存関係追加**
   - `idf.py add-dependency "espressif/led_strip^3.0.1"`: 公式コンポーネント使用
   - RMT（Remote Control Transceiver）ペリフェラル活用

3. **main.cpp統合**
   - RGBLED HAL初期化処理追加
   - 5秒ごと色変更デモ実装（`task_monitor()`内）
   - 10色パターンの順次切り替え

**ハードウェア仕様**（Arduino版調査結果）:
- **GPIO39**: オンボードLED（2個のWS2812B）
- **制御方式**: RMT + GRB色順序
- **輝度**: デフォルト15/255（Arduino版互換）

**HAL API仕様**:
```c
// 基本制御
esp_err_t stampfly_rgbled_init(void);
esp_err_t stampfly_rgbled_set_color(uint32_t color);
esp_err_t stampfly_rgbled_set_preset_color(stampfly_color_t color);
esp_err_t stampfly_rgbled_set_rgb(uint8_t red, uint8_t green, uint8_t blue);
esp_err_t stampfly_rgbled_set_brightness(uint8_t brightness);
esp_err_t stampfly_rgbled_clear(void);
```

**定義済み色**:
- `STAMPFLY_COLOR_RED/GREEN/BLUE/YELLOW/PURPLE/CYAN/ORANGE/PINK/WHITE/OFF`
- Arduino版フライトモード色に対応

**技術仕様**:
- ESP-IDF v5.4.1 led_strip v3.0.1使用
- `LED_STRIP_COLOR_COMPONENT_FMT_GRB`: WS2812B色順序対応
- RMT解像度: 10MHz、DMA無効（小規模LED向け）

**5秒ごと色変更デモ**:
- `task_monitor()`関数で自動実行
- 赤→緑→青→黄→紫→シアン→オレンジ→ピンク→白→消灯の順次切り替え
- ログ出力でカラーコード（0x006lX）表示

**使用方法**:
```bash
# ビルド・フラッシュ
. $HOME/esp/esp-idf/export.sh
idf.py build
idf.py flash monitor
```

**動作確認**:
- ✅ ビルド成功（API仕様修正済み）
- ✅ RGBLED HAL初期化（白色点灯）
- ✅ 5秒間隔自動色変更デモ
- ✅ ESP-IDF RMT + led_strip統合

## CLI機能の設計指針

### 推奨アーキテクチャ（ハイブリッドアプローチ）
- **HAL層**: 基本デバッグ出力機能（軽量）
- **アプリケーション層**: 完全CLIインターフェース（ユーザー向け）

```
components/
├── stampfly_core/debug/    # HAL組み込みデバッグ機能
└── stampfly_cli/           # 独立CLIコンポーネント（オプション）
```

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
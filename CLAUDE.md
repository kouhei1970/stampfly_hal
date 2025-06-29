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
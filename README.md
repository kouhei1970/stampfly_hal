# StampFly HAL (Hardware Abstraction Layer)

🚁 **ESP-IDF v5.4.1ベースのM5Stack StampFly用ハードウェア抽象化層**

## プロジェクト概要

StampFly HALは、M5Stack社のStampFly（ESP32-S3ベースクアッドコプター）用のハードウェア抽象化層です。Arduino環境からESP-IDFへの移行を目的とし、飛行制御学習教材・組み込み制御学習教材・ドローン研究プラットフォームとして開発されています。

## 🎯 主な特徴

- **ESP-IDF v5.4.1完全対応**: 最新のESP-IDFフレームワーク使用
- **オブジェクト指向C++設計**: RAII原則に基づく堅牢な設計
- **完全なセンサー対応**: StampFly搭載の全7センサーをサポート
- **Arduino互換**: 既存のArduino CLIコマンドとの互換性
- **printf対応**: デバッグに便利なUART出力リダイレクト機能

## 🔧 対応ハードウェア

### StampFly搭載センサー

**SPI接続 (MISO:43, MOSI:14, SCLK:44)**
- **BMI270** 6軸IMU (CS:46, 10MHz, Mode 0)
- **PMW3901** オプティカルフローセンサー (CS:47, 2MHz, Mode 3)

**I2C接続 (SDA:3, SCL:4, 400kHz)**
- **BMM150** 3軸磁気センサー (0x10)
- **BMP280** 気圧センサー (0x76)
- **VL53LX** ToF距離センサー x2 (0x29)
- **INA3221** 3ch電力モニター (0x40)

## 📁 プロジェクト構造

```
stampfly_hal/
├── components/stampfly_hal/     # 核となるHAL実装
│   ├── include/                 # ヘッダファイル
│   │   ├── stampfly_hal_base.h  # HAL基底クラス
│   │   ├── uart_hal.h           # UART HAL
│   │   ├── spi_hal.h            # SPI HAL
│   │   └── i2c_hal.h            # I2C HAL
│   └── src/                     # 実装ファイル
├── main/                        # メインアプリケーション
├── debug_tests/                 # 開発者用テストコード
├── components/                  # 追加コンポーネント用
└── docs/                        # ドキュメント
```

## 🚀 クイックスタート

### 必要環境

- **ESP-IDF v5.4.1**
- **Python 3.8+**
- **CMake 3.16+**
- **Git**

### セットアップ

```bash
# 1. リポジトリをクローン
git clone https://github.com/kouhei1970/stampfly_hal.git
cd stampfly_hal

# 2. ESP-IDF環境を読み込み
. $HOME/esp/esp-idf/export.sh

# 3. プロジェクトをビルド
idf.py build

# 4. StampFlyにフラッシュ
idf.py -p /dev/tty.usbmodem1101 flash monitor
```
注：'tty.usbmodem1101'は自分の環境に合わせて変える必要があります。
'ls /dev'で調べられます。

## 💻 使用方法

### 基本的なHAL使用例

```cpp
#include "uart_hal.h"
#include "spi_hal.h"
#include "i2c_hal.h"

// UART HAL（printf対応）
auto uart_config = stampfly_hal::UartHal::get_stampfly_default_config();
stampfly_hal::UartHal uart_hal(uart_config);
uart_hal.init();
uart_hal.configure();
uart_hal.enable();
uart_hal.redirect_printf();  // printf出力をUARTへリダイレクト

// SPI HAL（BMI270 IMU用）
auto spi_config = stampfly_hal::SpiHal::get_stampfly_default_config();
stampfly_hal::SpiHal spi_hal(spi_config);
spi_hal.init();
spi_hal.configure();
spi_hal.enable();

auto bmi270_config = stampfly_hal::SpiHal::get_bmi270_device_config();
spi_device_handle_t bmi270_handle;
spi_hal.add_device(bmi270_config, &bmi270_handle);

// I2C HAL（センサースキャン）
auto i2c_config = stampfly_hal::I2cHal::get_stampfly_default_config();
stampfly_hal::I2cHal i2c_hal(i2c_config);
i2c_hal.init();
i2c_hal.configure();
i2c_hal.enable();

uint8_t found_devices[16];
int device_count = i2c_hal.scan_devices(found_devices, 16);
printf("Found %d I2C devices\n", device_count);
```

## 🛠️ 開発状況

### ✅ 実装完了
- [x] HAL基底クラス
- [x] UART HAL（printf対応）
- [x] SPI HAL（BMI270/PMW3901対応）
- [x] I2C HAL（5デバイス対応）
- [x] デバッグテスト体制

### 🔄 実装予定
- [ ] GPIO HAL
- [ ] センサードライバー個別実装
- [ ] Arduino CLI互換機能
- [ ] 制御アルゴリズム（PID/MPC）
- [ ] センサーフュージョン

## 📖 ドキュメント

- [開発進捗記録](DEVELOPMENT_PROGRESS.md) - 詳細な実装状況
- [作業計画書](STAMPFLY_HAL_WORK_PLAN.md) - 8週間実装計画
- [Claude Code用ガイド](CLAUDE.md) - AI開発支援用設定

## 🤝 コントリビューション

1. このリポジトリをフォーク
2. フィーチャーブランチを作成 (`git checkout -b feature/amazing-feature`)
3. 変更をコミット (`git commit -m 'Add amazing feature'`)
4. ブランチにプッシュ (`git push origin feature/amazing-feature`)
5. プルリクエストを作成

## 📄 ライセンス

このプロジェクトは [MIT License](LICENSE) の下で公開されています。

## 🙏 謝辞

- [M5Stack](https://m5stack.com/) - StampFlyハードウェア
- [Espressif](https://www.espressif.com/) - ESP-IDFフレームワーク
- [Claude Code](https://claude.ai/code) - AI開発支援

## 📞 連絡先

**Kouhei Ito**
- GitHub: [@kouhei1970](https://github.com/kouhei1970)

---

⭐ このプロジェクトが役に立ったら、ぜひスターをお願いします！
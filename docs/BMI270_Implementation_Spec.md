# BMI270実装仕様書

## 概要

このドキュメントは、M5Stack StampFlyのBMI270 6軸IMUセンサーのHAL実装に必要な技術仕様をまとめたものです。SPI接続による初期化からチップID取得までのシーケンスを詳細に記述しています。

## BMI270基本仕様

### センサー仕様
- **メーカー**: Bosch Sensortec
- **型番**: BMI270
- **機能**: 16ビット3軸ジャイロスコープ + 3軸アクセロメーター
- **通信インターフェース**: SPI（最大10MHz）/ I2C（最大1MHz）
- **データシート**: [BST-BMI270-DS000](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)

### StampFlyでの接続仕様

#### SPI接続ピンアサイン
```
SPIバス共通ピン:
- MISO: GPIO43
- MOSI: GPIO14
- SCLK: GPIO44

BMI270専用ピン:
- CS (Chip Select): GPIO46
```

#### SPI通信仕様
```
- クロック周波数: 10MHz
- SPI Mode: 0 (CPOL=0, CPHA=0)
- CS有効プリトランス: 1クロック
- CS有効ポストトランス: 1クロック
- データビット順: MSBファースト
```

## BMI270レジスタマップ

### 基本レジスタ
| レジスタ名 | アドレス | アクセス | 説明 |
|-----------|---------|---------|------|
| CHIP_ID | 0x00 | R | チップID (0x24) |
| ERR_REG | 0x02 | R | エラーレジスタ |
| STATUS | 0x03 | R | ステータスレジスタ |
| ACC_X_LSB | 0x0C | R | アクセロメーターX軸下位バイト |
| ACC_X_MSB | 0x0D | R | アクセロメーターX軸上位バイト |
| GYRO_X_LSB | 0x12 | R | ジャイロスコープX軸下位バイト |
| GYRO_X_MSB | 0x13 | R | ジャイロスコープX軸上位バイト |
| TEMPERATURE_LSB | 0x22 | R | 温度センサー下位バイト |
| TEMPERATURE_MSB | 0x23 | R | 温度センサー上位バイト |

### 制御レジスタ
| レジスタ名 | アドレス | アクセス | 説明 |
|-----------|---------|---------|------|
| PWR_CONF | 0x7C | R/W | 電源設定 |
| PWR_CTRL | 0x7D | R/W | 電源制御 |
| ACC_CONF | 0x40 | R/W | アクセロメーター設定 |
| ACC_RANGE | 0x41 | R/W | アクセロメーター範囲 |
| GYRO_CONF | 0x42 | R/W | ジャイロスコープ設定 |
| GYRO_RANGE | 0x43 | R/W | ジャイロスコープ範囲 |
| INIT_CTRL | 0x59 | W | 初期化制御 |
| INIT_ADDR_0 | 0x5B | R/W | 初期化アドレス下位 |
| INIT_ADDR_1 | 0x5C | R/W | 初期化アドレス上位 |
| INIT_DATA | 0x5E | W | 初期化データ |
| INTERNAL_STATUS | 0x21 | R | 内部ステータス |
| CMD | 0x7E | W | コマンドレジスタ |

## SPI通信プロトコル

### SPI読み取りコマンド
```
読み取り手順:
1. CSをLOWにする
2. レジスタアドレス (MSB=1) を送信
3. ダミーバイト (0x00) を送信
4. データバイトを受信
5. CSをHIGHにする

例: チップID読み取り
TX: [0x80] [0x00]    // 0x80 = 0x00 | 0x80 (読み取りフラグ)
RX: [0xFF] [0x24]    // 0x24 = BMI270チップID
```

### SPI書き込みコマンド
```
書き込み手順:
1. CSをLOWにする
2. レジスタアドレス (MSB=0) を送信
3. データバイトを送信
4. CSをHIGHにする

例: 電源制御レジスタ書き込み
TX: [0x7D] [0x04]    // PWR_CTRL = 0x04 (アクセロメーター有効)
```

## 初期化シーケンス（Bosch公式仕様準拠）

### Phase 1: SPI通信確立
```cpp
1. SPIバス初期化
   - HSPI_HOST使用
   - 1MHz (初期化時), Mode 0, MSB First
   - MISO:43, MOSI:14, SCLK:44

2. BMI270デバイス追加
   - CS: GPIO46
   - pre_cb: NULL, post_cb: NULL

3. 電源オンリセット（POR）
   - 電源供給後450μs待機
   - IMUはサスペンドモードで開始

4. SPIモード切り替え
   - I2CからSPIモードに変更するため
   - ダミー読み取り実行 (0x80, 0x00)
```

### Phase 2: チップID確認と初期設定
```cpp
1. チップID読み取り
   - レジスタ0x00から読み取り
   - 期待値: 0x24
   - リトライ機構 (最大3回)

2. 通信状態確認
   - 0x00または0xFFの場合は通信エラー
   - 0x24正常値の確認

3. Advanced Power Save無効化
   - PWR_CONF (0x7C) = 0x00
   - adv_power_saveビットを0に設定
```

### Phase 3: コンフィギュレーションファイルアップロード
```cpp
1. 初期化準備
   - INIT_CTRL (0x59) = 0x00
   - INIT_ADDR_0 (0x5B) = 0x00
   - INIT_ADDR_1 (0x5C) = 0x00

2. コンフィギュレーションファイル書き込み
   - 8192バイトの設定ファイルをバースト書き込み
   - 32バイトずつのチャンクで書き込み可能
   - INIT_DATA (0x5E) レジスタへ書き込み
   - チャンク間でINIT_ADDR_0/1をインクリメント（bytes/2）

3. 初期化開始
   - INIT_CTRL (0x59) = 0x01
   - 一度のみ実行（POR/ソフトリセット後）

4. 初期化完了確認
   - INTERNAL_STATUS (0x21) のmessageフィールドが0x01になるまで待機
   - 最大150ms待機
```

### Phase 4: センサー設定
```cpp
1. 電源モード設定
   - 自動的にConfiguration modeに遷移済み

2. アクセロメーター設定 (ACC_CONF = 0x40)
   - bits[3:0] ODR: 0x08 (100Hz)
   - bits[6:4] BWP: 0x02 (Normal mode)
   - bits[7] acc_perf_mode: 1 (Performance mode)

3. アクセロメーター範囲設定 (ACC_RANGE = 0x41)
   - bits[1:0]: 0x01 (±4g)

4. ジャイロスコープ設定 (GYRO_CONF = 0x42)
   - bits[3:0] ODR: 0x09 (200Hz)
   - bits[5:4] BWP: 0x01 (Normal mode)
   - bit[6] noise_perf_mode: 0 (Power optimized)
   - bit[7] filter_perf_mode: 1 (Performance mode)

5. ジャイロスコープ範囲設定 (GYRO_RANGE = 0x43)
   - bits[2:0]: 0x02 (±1000dps)

6. センサー有効化 (PWR_CTRL = 0x7D)
   - bit[0] aux_en: 0 (補助センサー無効)
   - bit[1] gyr_en: 1 (ジャイロ有効)
   - bit[2] acc_en: 1 (アクセロ有効)
   - bit[3] temp_en: 1 (温度センサー有効)
   - 値: 0x0E
```

### Phase 5: 動作確認
```cpp
1. ステータス確認
   - STATUS (0x03) レジスタ読み取り
   - drdy_acc, drdy_gyr ビット確認

2. エラー確認
   - ERR_REG (0x02) レジスタ読み取り
   - fatal_err, err_code フィールド確認

3. データ読み取りテスト
   - アクセロメーターデータ (0x0C-0x11)
   - ジャイロスコープデータ (0x12-0x17)
   - 温度データ (0x22-0x23)
```

### 初期化タイミング要件
```
- POR後: 450μs待機
- ソフトリセット後: 1ms待機
- Config file upload後: 150ms待機
- センサー有効化後: 30ms待機（データ安定化）
```

## エラーハンドリング

### 一般的な問題と対処法

#### チップID読み取り失敗
```
症状: 0x00, 0xFF, または予期しない値
原因:
- SPI配線問題
- 電源供給問題
- クロック速度問題
- SPIモード設定問題

対処法:
1. クロック速度を1.5MHzに下げて再試行
2. SPI Mode 3 (CPOL=1, CPHA=1) で再試行
3. ハードウェア配線確認
4. 電源電圧確認 (3.3V)
```

#### 初期化失敗
```
症状: CONFIG_LOAD エラー
原因:
- 設定ファイル読み込み失敗
- 内部状態エラー

対処法:
1. INTERNAL_STATUS レジスタ (0x21) 確認
2. 完全パワーサイクル実行
3. より長い待機時間設定
```

## 実装上の注意点

### SPI通信のベストプラクティス
1. **ダミーバイト必須**: SPI読み取り時は必ずダミーバイトを送信
2. **CS制御**: トランザクション前後でCS制御を確実に行う
3. **クロック速度**: 初期化時は低速 (1.5MHz) で開始
4. **エラーリトライ**: 通信エラー時は複数回リトライ

### ESP-IDF固有の考慮事項
```cpp
// SPI設定例
spi_device_interface_config_t dev_config = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,                          // SPI Mode 0
    .duty_cycle_pos = 0,
    .cs_ena_pretrans = 1,               // CS プリトランス
    .cs_ena_posttrans = 1,              // CS ポストトランス
    .clock_speed_hz = 10 * 1000 * 1000, // 10MHz
    .spics_io_num = GPIO_NUM_46,        // CS ピン
    .flags = 0,
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL
};
```

## デバッグ支援

### ログ出力レベル
- **ESP_LOG_DEBUG**: SPI低レベル通信ログ
- **ESP_LOG_INFO**: 初期化進捗とエラー情報
- **ESP_LOG_WARN**: 再試行とフォールバック処理
- **ESP_LOG_ERROR**: 致命的エラーのみ

### 診断コマンド
```cpp
// チップID確認
uint8_t chip_id = read_register(0x00);
ESP_LOGI("BMI270", "Chip ID: 0x%02X", chip_id);

// ステータス確認
uint8_t status = read_register(0x03);
ESP_LOGI("BMI270", "Status: 0x%02X", status);

// エラーレジスタ確認
uint8_t error = read_register(0x02);
ESP_LOGI("BMI270", "Error: 0x%02X", error);
```

## 参考資料

1. [Bosch BMI270 データシート](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
2. [BMI270 Sensor API](https://github.com/boschsensortec/BMI270_SensorAPI)
3. [ESP-IDF SPI Master Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/api-reference/peripherals/spi_master.html)
4. [StampFly Arduino Examples](https://github.com/m5stack/M5StampFly)

---
*このドキュメントはStampFly HAL開発プロジェクトの一部です*
# BMI270 ODR（Output Data Rate）設定仕様

**作成日**: 2025年10月26日
**目的**: 1000Hz以上のサンプリングレート設定とオーバーサンプリング戦略

---

## 📊 BMI270 ODR仕様（Bosch公式）

### 加速度計（Accelerometer）ODR

**利用可能なODR設定値:**
- 12.5 Hz
- 25 Hz
- 50 Hz
- 100 Hz
- 200 Hz
- 400 Hz
- 800 Hz
- **1600 Hz（最大）**

**レジスタ設定値（REG_ACC_CONF: 0x40）:**

| ODR | レジスタ値 | 説明 |
|-----|-----------|------|
| 12.5 Hz | 0x05 | 最低速 |
| 25 Hz | 0x06 | - |
| 50 Hz | 0x07 | - |
| 100 Hz | 0x08 | 現在の設定 |
| 200 Hz | 0x09 | - |
| 400 Hz | 0x0A | - |
| 800 Hz | 0x0B | - |
| **1600 Hz** | **0x0C** | **最高速（推奨）** |

**Performance Mode設定:**
- ビット7（0x80）をセット: Performance Mode（低ノイズ、高精度）
- 完全な設定値 = ODR値 | 0x80

**例:**
```cpp
// 1600Hz, Performance Mode
uint8_t acc_conf = 0x8C;  // 0x0C | 0x80
```

---

### ジャイロスコープ（Gyroscope）ODR

**利用可能なODR設定値:**
- 25 Hz
- 50 Hz
- 100 Hz
- 200 Hz
- 400 Hz
- 800 Hz
- 1600 Hz
- 3200 Hz
- **6400 Hz（最大）**

**レジスタ設定値（REG_GYR_CONF: 0x42）:**

| ODR | レジスタ値 | 説明 |
|-----|-----------|------|
| 25 Hz | 0x06 | 最低速 |
| 50 Hz | 0x07 | - |
| 100 Hz | 0x08 | - |
| 200 Hz | 0x09 | 現在の設定 |
| 400 Hz | 0x0A | - |
| 800 Hz | 0x0B | - |
| **1600 Hz** | **0x0C** | **推奨** |
| 3200 Hz | 0x0D | 超高速 |
| 6400 Hz | 0x0E | 最高速 |

**Filter Performance Mode設定:**
- ビット7（0x80）をセット: Performance Mode
- 完全な設定値 = ODR値 | 0x80

**例:**
```cpp
// 1600Hz, Performance Mode
uint8_t gyr_conf = 0x8C;  // 0x0C | 0x80
```

---

## 🎯 1000Hzサンプリング要件への対応

### 目標
- **制御周期**: 500Hz（2ms）
- **サンプリング周期**: 1000Hz以上（1ms以下）
- **オーバーサンプリング比**: 2倍以上

### 推奨設定

#### オプション1: 1600Hz（推奨）

**設定理由:**
- ✅ 1000Hz要件を満たす（1.6倍）
- ✅ 500Hz制御周期の3.2倍（十分なマージン）
- ✅ 両センサーで同じODRに統一可能
- ✅ 実装が簡単

**設定値:**
```cpp
// 加速度計: 1600Hz, ±4g, Performance Mode
uint8_t acc_conf = 0x8C;   // ODR=1600Hz(0x0C) + Performance(0x80)
uint8_t acc_range = 0x01;  // ±4g

// ジャイロスコープ: 1600Hz, ±1000dps, Performance Mode
uint8_t gyr_conf = 0x8C;   // ODR=1600Hz(0x0C) + Performance(0x80)
uint8_t gyr_range = 0x01;  // ±1000dps
```

**オーバーサンプリング効果:**
- 500Hz制御ループあたり **3.2サンプル** 取得
- デジタルフィルタ適用の余裕あり
- ノイズ低減効果あり

#### オプション2: 3200Hz（ジャイロのみ高速化）

**設定理由:**
- ✅ ジャイロは高速応答が重要
- ✅ より細かいノイズフィルタリング可能
- ⚠️ 消費電力増加
- ⚠️ 加速度計とODR不一致

**設定値:**
```cpp
// 加速度計: 1600Hz, ±4g, Performance Mode
uint8_t acc_conf = 0x8C;   // ODR=1600Hz(0x0C) + Performance(0x80)

// ジャイロスコープ: 3200Hz, ±1000dps, Performance Mode
uint8_t gyr_conf = 0x8D;   // ODR=3200Hz(0x0D) + Performance(0x80)
```

**オーバーサンプリング効果:**
- 500Hz制御ループあたり **6.4サンプル** 取得（ジャイロ）
- より高精度なフィルタリング可能
- CPU負荷とのトレードオフ

---

## 🔧 実装方針

### 推奨アプローチ: **オプション1（1600Hz統一設定）**

**理由:**
1. **シンプル**: 両センサー同じODRで管理が容易
2. **十分な性能**: 3.2倍オーバーサンプリングで十分
3. **省電力**: 3200Hzより消費電力低い
4. **データ同期**: 加速度計・ジャイロが同期

### データフロー設計

```
BMI270 (1600Hz)
    ↓ 0.625ms周期でデータ生成
Data Ready割り込み
    ↓
FIFOバッファ（オプション）
    ↓
データ取得タスク（1600Hz）
    ↓
デジタルローパスフィルタ
    ↓
センサーフュージョン（500Hz）
    ↓
姿勢推定
    ↓
PID制御（500Hz）
```

### フィルタリング戦略

**3段階フィルタリング:**

1. **BMI270内蔵フィルタ**
   - Performance Mode有効
   - アンチエイリアシングフィルタ

2. **デジタルローパスフィルタ（1600Hz→500Hz）**
   - 単純移動平均（3サンプル平均）
   - または、バターワースフィルタ（2次）
   - カットオフ周波数: 200Hz程度

3. **センサーフュージョンフィルタ**
   - 相補フィルタ（高周波ジャイロ＋低周波加速度計）
   - または、カルマンフィルタ

---

## 💻 コード実装例

### configure_sensors() 関数の更新

```cpp
esp_err_t BMI270::configure_sensors() {
    ESP_LOGI(TAG, "Configuring sensors for 1600Hz ODR with oversampling");

    // === 加速度計設定 ===
    // 1600Hz, ±4g, Performance Mode
    uint8_t acc_conf = 0x8C;  // ODR=1600Hz(0x0C) + Performance(0x80)
    esp_err_t ret = write_register(REG_ACC_CONF, acc_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer ODR");
        return ret;
    }

    // Set accelerometer range: ±4g
    accel_range_ = 1;  // 0=±2g, 1=±4g, 2=±8g, 3=±16g
    ret = write_register(REG_ACC_RANGE, accel_range_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer range");
        return ret;
    }

    ESP_LOGI(TAG, "✅ Accelerometer: 1600Hz, ±4g, Performance Mode");

    // === ジャイロスコープ設定 ===
    // 1600Hz, ±1000dps, Performance Mode
    uint8_t gyr_conf = 0x8C;  // ODR=1600Hz(0x0C) + Performance(0x80)
    ret = write_register(REG_GYR_CONF, gyr_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope ODR");
        return ret;
    }

    // Set gyroscope range: ±1000dps
    gyro_range_ = 1;  // 0=±2000dps, 1=±1000dps, 2=±500dps, 3=±250dps, 4=±125dps
    ret = write_register(REG_GYR_RANGE, gyro_range_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope range");
        return ret;
    }

    ESP_LOGI(TAG, "✅ Gyroscope: 1600Hz, ±1000dps, Performance Mode");

    // === センサー有効化 ===
    // Enable: Accelerometer + Gyroscope + Temperature
    uint8_t pwr_ctrl = PWR_CTRL_ACC_EN | PWR_CTRL_GYR_EN | PWR_CTRL_TEMP_EN;
    ret = write_register(REG_PWR_CTRL, pwr_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors");
        return ret;
    }

    // センサー安定化待機
    vTaskDelay(pdMS_TO_TICKS(50));

    // === スケールファクター設定 ===
    const float accel_scales[] = {2.0f, 4.0f, 8.0f, 16.0f};
    const float gyro_scales[] = {2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f};

    accel_scale_ = (accel_scales[accel_range_] / 32768.0f) * 9.80665f;  // m/s^2
    gyro_scale_ = (gyro_scales[gyro_range_] / 32768.0f) * M_PI / 180.0f;  // rad/s

    ESP_LOGI(TAG, "📊 Scale factors - Accel: %.6f m/s²/LSB, Gyro: %.6f rad/s/LSB",
             accel_scale_, gyro_scale_);

    ESP_LOGI(TAG, "🎯 Oversampling ratio: 3.2x (1600Hz / 500Hz control loop)");

    return ESP_OK;
}
```

---

## 📊 性能予測

### タイミング仕様

| 項目 | 値 | 説明 |
|------|-----|------|
| BMI270 ODR | 1600 Hz | 0.625ms周期 |
| データ取得周期 | 0.625 ms | 割り込み駆動 |
| 制御ループ周期 | 2.0 ms | 500Hz |
| オーバーサンプリング比 | 3.2倍 | 500Hz制御時に3.2サンプル |
| フィルタ遅延 | <1ms | 3サンプル移動平均 |
| 総レイテンシ | <3ms | データ取得→制御出力 |

### CPU負荷予測

**1600Hzデータ取得:**
- SPI読み取り: 約100μs/サンプル
- データ処理: 約50μs/サンプル
- 合計: 約150μs × 1600 = 240ms/s = **24% CPU負荷**

**500Hz制御ループ:**
- センサーフュージョン: 約100μs
- PID計算: 約50μs
- 合計: 約150μs × 500 = 75ms/s = **7.5% CPU負荷**

**総CPU負荷: 約32%**（ESP32-S3 240MHzで十分余裕）

---

## ⚠️ 注意事項

### SPI通信速度の確認

**現在の設定: 8MHz**

1600Hzで動作させるには、SPI通信速度が十分か確認が必要:

**計算:**
- 1サンプル = 12バイト（加速度6バイト + ジャイロ6バイト）
- 1バイト = 8ビット + オーバーヘッド（≈10クロック）
- 12バイト × 10クロック/バイト = 120クロック/サンプル
- 8MHz / 120 = 約66kサンプル/秒 = 66kHz

**結論: 8MHzで十分（66kHz >> 1.6kHz）**

可能であれば10MHzに上げることで、さらに余裕を確保できます。

### 割り込み実装の重要性

1600Hzでポーリングは現実的ではありません：
- **必須**: Data Ready割り込みの実装
- **推奨**: FIFOバッファの活用（データロスト防止）

---

## 🎯 次のステップ

1. **コード更新**: `configure_sensors()` を1600Hz設定に変更
2. **割り込み実装**: Data Ready割り込みハンドラー追加
3. **フィルタ実装**: 3サンプル移動平均フィルタ
4. **実機検証**: 1600Hzでのデータ取得確認
5. **性能評価**: CPU負荷・レイテンシ測定

---

## 📚 参考文献

- BMI270 Data Sheet (Bosch Sensortec)
- ESP32-S3 Technical Reference Manual
- 「ディジタル信号処理の基礎」（オーバーサンプリング理論）
- 「組み込みシステムにおけるセンサーフュージョン」

---

**まとめ:**
- **推奨設定: 1600Hz（両センサー統一）**
- **オーバーサンプリング比: 3.2倍**
- **期待効果: 高精度・低ノイズな姿勢推定**

# StampFly HAL開発作業計画

## 作業方針遵守確認

### 1. StampFly仕様確認 ✅
- **実際のハードウェア構成確認済み**
  - SPIセンサー: BMI270 IMU（CS: GPIO46）, PMW3901 オプティカルフロー（CS: GPIO12）
  - I2Cセンサー: BMM150磁気, BMP280気圧, VL53LX ToF x2基, INA3221電力監視
  - SPI設定: MISO=43, MOSI=14, SCLK=44
  - I2C設定: SDA=3, SCL=4, 400kHz
  - ToF制御: XSHUT（GPIO7, GPIO9）, INT（GPIO6, GPIO8）

### 2. センサー仕様確認 ✅
- **Arduino版コード解析による正確なピンアサイン確認済み**
  - sensor.hpp: 基本ピン定義
  - tof.hpp: ToF制御ピン（INT_BOTTOM=6, XSHUT_BOTTOM=7, INT_FRONT=8, XSHUT_FRONT=9）
  - spi_s3.hpp: SPI設定（BMI_CS=46, PMW_CS=12）
  - INA3221アドレス: 0x40（GND接続）

### 3. API仕様確認 ✅
- **ESP-IDF v5.4.1対応確認済み**
  - SPI Master API: 8MHz以上での GPIO matrix遅延考慮
  - I2C Master API: thread-safe, mutex保護
  - GPIO API: esp_driver_gpio コンポーネント依存
  - タイミング最適化とエラーハンドリング改良

## StampFly HAL開発作業計画

### 開発期間: 8週間
### 開発環境: ESP-IDF v5.4.1, ESP32-S3
### 対象: M5Stack StampFly クアッドコプター

## フェーズ1: プロジェクト基盤構築（1週間）

### 1.1 プロジェクト構造作成（1日）
```
stampfly_hal/
├── CMakeLists.txt                 # ESP-IDF v5.4.1 プロジェクト設定
├── sdkconfig.defaults             # ESP32-S3 最適化設定
├── partitions.csv                 # フラッシュパーティション
├── README.md                      # プロジェクト概要
├── LICENSE                        # MIT License
├── .gitignore                     # Git除外設定
│
├── main/                          # メインアプリケーション
│   ├── CMakeLists.txt
│   ├── main.cpp                   # HAL実証アプリケーション
│   ├── stampfly_config.h          # StampFly固有設定
│   └── examples/                  # 使用例
│       ├── sensor_test.cpp        # センサーテスト例
│       └── basic_flight.cpp       # 基本飛行例
│
└── components/                    # HALコンポーネント群
    ├── stampfly_hal/              # コアHAL
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   ├── hal_base.hpp       # HAL基底クラス
    │   │   ├── gpio_hal.hpp       # GPIO制御
    │   │   ├── spi_hal.hpp        # SPI通信
    │   │   ├── i2c_hal.hpp        # I2C通信
    │   │   ├── timer_hal.hpp      # タイマー制御
    │   │   └── interrupt_hal.hpp  # 割り込み管理
    │   └── src/
    │       ├── hal_base.cpp
    │       ├── gpio_hal.cpp
    │       ├── spi_hal.cpp
    │       ├── i2c_hal.cpp
    │       ├── timer_hal.cpp
    │       └── interrupt_hal.cpp
    │
    ├── stampfly_sensors/          # センサーHAL
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   ├── sensor_base.hpp    # センサー基底クラス
    │   │   ├── bmi270_sensor.hpp  # BMI270 IMU
    │   │   ├── pmw3901_sensor.hpp # PMW3901 オプティカルフロー
    │   │   ├── bmm150_sensor.hpp  # BMM150 磁気センサー
    │   │   ├── bmp280_sensor.hpp  # BMP280 気圧センサー
    │   │   ├── vl53lx_sensor.hpp  # VL53LX ToFセンサー
    │   │   ├── ina3221_sensor.hpp # INA3221 電力監視
    │   │   └── sensor_manager.hpp # センサー統合管理
    │   └── src/
    │       └── [対応するcppファイル群]
    │
    └── stampfly_utils/            # ユーティリティ
        ├── CMakeLists.txt
        ├── include/
        │   ├── math_utils.hpp     # 数学演算
        │   ├── filter_utils.hpp   # フィルター処理
        │   └── calibration.hpp    # キャリブレーション
        └── src/
            └── [対応するcppファイル群]
```

### 1.2 基盤HAL実装（3日）

#### HAL基底クラス（hal_base.hpp/cpp）（1日）
```cpp
class HalBase {
protected:
    esp_err_t last_error_;
    const char* component_name_;
    
public:
    virtual esp_err_t initialize() = 0;
    virtual esp_err_t deinitialize() = 0;
    esp_err_t get_last_error() const;
    void log_error(const char* message);
};
```

#### UART HAL（uart_hal.hpp/cpp）（1日）**【重要度向上】**
```cpp
class UartHal : public HalBase {
public:
    enum class OutputFormat { STANDARD, CSV, TSV, TELEPLOT, JSON };
    enum class LogLevel { ERROR, WARN, INFO, DEBUG, VERBOSE };
    
    // printf機能（ユーザー要求）
    esp_err_t printf(const char* format, ...);
    esp_err_t printf_redirect_enable();
    
    // ESP_LOG統合
    esp_err_t esp_log_redirect_enable();
    esp_err_t set_log_level(LogLevel level);
    
    // CLI機能（Arduino版互換）
    esp_err_t cli_process_command(const char* command);
    esp_err_t streaming_start(uint32_t interval_ms, OutputFormat format);
    
    // デバッグ支援
    esp_err_t print_system_info();
    esp_err_t dump_sensor_registers(const char* sensor_name);
};
```

#### GPIO HAL（gpio_hal.hpp/cpp）（0.5日）
- ToF XSHUT/INT制御専用実装
- ESP-IDF v5.4.1 esp_driver_gpio 使用
- 割り込み処理対応

#### Timer HAL（timer_hal.hpp/cpp）（0.5日）
- 400Hz制御ループ基盤
- FreeRTOS高分解能タイマー使用

### 1.3 通信HAL実装（3日）

#### SPI HAL（spi_hal.hpp/cpp）
```cpp
class SpiHal : public HalBase {
private:
    spi_host_device_t host_;
    spi_device_handle_t bmi270_handle_;
    spi_device_handle_t pmw3901_handle_;
    
public:
    esp_err_t initialize() override;
    esp_err_t read_bmi270(uint8_t reg, uint8_t* data, size_t len);
    esp_err_t write_bmi270(uint8_t reg, const uint8_t* data, size_t len);
    esp_err_t read_pmw3901(uint8_t reg, uint8_t* data, size_t len);
    esp_err_t write_pmw3901(uint8_t reg, const uint8_t* data, size_t len);
};
```

**StampFly固有設定**:
- BMI270: CS=GPIO46, 8MHz, Mode 0
- PMW3901: CS=GPIO12, 2MHz, Mode 3
- GPIO matrix遅延考慮

#### I2C HAL（i2c_hal.hpp/cpp）
```cpp
class I2cHal : public HalBase {
private:
    i2c_port_t port_;
    std::map<uint8_t, std::string> device_map_;
    
public:
    esp_err_t initialize() override;
    esp_err_t scan_devices();
    esp_err_t read_device(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);
    esp_err_t write_device(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len);
};
```

**StampFly固有設定**:
- SDA=GPIO3, SCL=GPIO4, 400kHz
- 5デバイス対応（BMM150, BMP280, VL53LX x2, INA3221）

### 1.4 基本テスト（1日）
- **UART機能テスト**: printf動作、ESP_LOG出力、CLI基本動作確認
- 通信確認テスト（各センサーID読み取り）
- エラーハンドリング確認
- ビルド・フラッシュ確認

## フェーズ2: センサー統合実装（3週間）

### 2.1 SPI センサー統合（1週間）

#### BMI270 IMU実装（3日）
```cpp
class Bmi270Sensor : public SensorBase {
private:
    SpiHal* spi_hal_;
    struct bmi2_dev bmi2_dev_;
    
public:
    esp_err_t initialize() override;
    esp_err_t read_accel_data(float* x, float* y, float* z);
    esp_err_t read_gyro_data(float* x, float* y, float* z);
    esp_err_t configure_fifo();
};
```

**実装内容**:
- Bosch BMI270 純正API統合
- Arduino版設定移植（±8G, ±2000DPS）
- FIFOバッファ対応
- 8MHz SPI最適化

#### PMW3901 オプティカルフロー実装（2日）
```cpp
class Pmw3901Sensor : public SensorBase {
private:
    SpiHal* spi_hal_;
    
public:
    esp_err_t initialize() override;
    esp_err_t read_motion_data(int16_t* delta_x, int16_t* delta_y);
    esp_err_t configure_registers();
};
```

**実装内容**:
- PixArt PMW3901 API統合
- Arduino版レジスタ設定移植
- モーション検出最適化

#### SPI統合テスト（2日）
- デュアルデバイス同時動作確認
- データ取得レート確認
- エラー処理確認

### 2.2 I2C基本センサー統合（1週間）

#### BMM150 磁気センサー実装（2日）
```cpp
class Bmm150Sensor : public SensorBase {
private:
    I2cHal* i2c_hal_;
    struct bmm150_dev bmm150_dev_;
    
public:
    esp_err_t initialize() override;
    esp_err_t read_mag_data(float* x, float* y, float* z);
    esp_err_t calibrate();
};
```

#### BMP280 気圧センサー実装（2日）
```cpp
class Bmp280Sensor : public SensorBase {
private:
    I2cHal* i2c_hal_;
    
public:
    esp_err_t initialize() override;
    esp_err_t read_pressure_data(float* pressure, float* temperature);
    esp_err_t calculate_altitude(float* altitude);
};
```

#### INA3221 電力監視実装（2日）
```cpp
class Ina3221Sensor : public SensorBase {
private:
    I2cHal* i2c_hal_;
    
public:
    esp_err_t initialize() override;
    esp_err_t read_voltage(uint8_t channel, float* voltage);
    esp_err_t read_current(uint8_t channel, float* current);
    esp_err_t check_power_limits();
};
```

#### I2C統合テスト（1日）
- 多デバイス同時通信確認
- アドレス競合確認

### 2.3 ToFセンサー特殊制御（1週間）

#### VL53LX ToFセンサー実装（4日）
```cpp
class Vl53lxSensor : public SensorBase {
public:
    enum Position { BOTTOM, FRONT };
    
private:
    I2cHal* i2c_hal_;
    GpioHal* gpio_hal_;
    Position position_;
    uint8_t i2c_address_;
    gpio_num_t xshut_pin_;
    gpio_num_t int_pin_;
    
public:
    esp_err_t initialize(Position pos) override;
    esp_err_t set_i2c_address(uint8_t new_addr);
    esp_err_t read_range_data(uint16_t* range_mm);
    esp_err_t configure_interrupts();
};
```

**実装内容**:
- STMicroelectronics VL53LX API統合
- XSHUT制御による個別管理
- I2Cアドレス動的変更（0x29→0x2A）
- 割り込み処理実装

#### ToFマネージャー実装（2日）
```cpp
class ToFManager {
private:
    Vl53lxSensor bottom_sensor_;
    Vl53lxSensor front_sensor_;
    
public:
    esp_err_t initialize_both_sensors();
    esp_err_t read_both_ranges(uint16_t* bottom, uint16_t* front);
};
```

#### ToF統合テスト（1日）
- 2基同時動作確認
- 割り込み処理確認

## フェーズ3: 制御システム実装（2週間）

### 3.1 センサーフュージョン（1週間）

#### センサーマネージャー（3日）
```cpp
class SensorManager {
private:
    Bmi270Sensor* imu_;
    Bmm150Sensor* mag_;
    Bmp280Sensor* baro_;
    ToFManager* tof_manager_;
    Pmw3901Sensor* optical_flow_;
    Ina3221Sensor* power_monitor_;
    
public:
    esp_err_t initialize_all_sensors();
    esp_err_t read_all_sensor_data();
    SensorData get_fused_data();
};
```

#### AHRS実装（Arduino版移植）（2日）
- Madgwick AHRS移植
- クォータニオン→オイラー角変換

#### Kalmanフィルター実装（2日）
- 高度推定フィルター移植
- 速度推定統合

### 3.2 制御ループ実装（1週間）

#### 400Hz制御ループ（3日）
```cpp
class ControlLoop {
private:
    SensorManager* sensor_manager_;
    TimerHal* timer_hal_;
    
public:
    esp_err_t start_400hz_loop();
    void control_task();
    esp_err_t stop_loop();
};
```

#### PID制御器移植（2日）
- Arduino版PID制御移植
- 姿勢・レート制御統合

#### 安全機能実装（2日）
- 電力監視による安全停止
- センサー異常検出
- フェイルセーフ機能

## フェーズ4: 通信・高度なCLI実装（1週間）

### 4.1 ESP-NOW通信（3日）
- ATOMJoyとの通信実装
- ペアリング機能
- リモートコントロール

### 4.2 高度なCLI機能実装（3日）**【Arduino版完全互換】**
- **Arduino版CLI完全移植**:
  - `imu`, `tof`, `voltage`, `mag`, `attitude`, `all`, `status`コマンド
  - `stream start/stop`, `offset start`, `mag_cal start`コマンド
  - `pid get/set`, `save`, `load`, `reset`コマンド
- **出力フォーマット対応**: CSV, TSV, Teleplot, JSON
- **ストリーミング機能**: リアルタイムデータ配信
- **デバッグ機能**: レジスタダンプ, システム情報表示

### 4.3 テレメトリ・ログ強化（1日）
- **printf統合ログシステム**
- フライトデータログ
- エラーログ自動保存

## フェーズ5: 最終統合・最適化（1週間）

### 5.1 全機能統合テスト（3日）
- 全センサー同時動作確認
- 制御ループ性能確認
- Arduino版との比較検証

### 5.2 パフォーマンス最適化（2日）
- メモリ使用量最適化
- CPU負荷軽減
- 応答性改善

### 5.3 ドキュメント完成（2日）
- APIドキュメント
- 使用例コード
- トラブルシューティング

## 品質保証方針

### 段階的検証
1. **各HAL実装後**: 単体テスト実行
2. **各センサー統合後**: Arduino版との動作比較
3. **各フェーズ完了後**: 統合テスト実行
4. **最終段階**: 実飛行テスト

### エラーハンドリング
- 全HAL関数でesp_err_t戻り値
- センサー異常の自動検出
- 通信エラーの自動復旧

### パフォーマンス要件
- 400Hz制御ループ安定動作
- センサーデータ遅延 <2.5ms
- メモリ使用量 <512KB

## 成功基準

### 機能要件
- ✅ **UART HAL printf機能完全動作**（ユーザー要求）
- ✅ **Arduino版CLI完全互換**（14コマンド対応）
- ✅ 全7センサーからデータ正常取得
- ✅ 400Hz制御ループ安定動作
- ✅ ESP-NOW通信確立
- ✅ Arduino版と同等の飛行性能

### 品質要件
- ✅ 単体テストカバレッジ >90%
- ✅ 統合テスト全項目PASS
- ✅ 3時間連続動作確認
- ✅ 実飛行テスト成功

この作業計画により、StampFlyの実際のハードウェア仕様に完全に対応したHALシステムを構築できます。
# ESP-IDF v5.4 SPI Master API実装ガイド for StampFly

## 目次
1. [概要](#概要)
2. [StampFly SPIピン配置](#stampfly-spiピン配置)
3. [ESP-IDF SPI Masterアーキテクチャ](#esp-idf-spi-masterアーキテクチャ)
4. [SPI初期化フロー](#spi初期化フロー)
5. [トランザクション種別と使い分け](#トランザクション種別と使い分け)
6. [タイミング制御](#タイミング制御)
7. [DMA設定とパフォーマンス最適化](#dma設定とパフォーマンス最適化)
8. [センサー別実装パターン](#センサー別実装パターン)
9. [トラブルシューティング](#トラブルシューティング)
10. [ベストプラクティス](#ベストプラクティス)

## 概要

ESP-IDF v5.4のSPI Master APIは、ESP32-S3の4つのSPIペリフェラル（SPI0/1は内部フラッシュ用、SPI2/3がユーザー用）を制御します。StampFlyでは主にSPI2（HSPI）を使用してセンサー群と通信します。

### 主要特徴
- **最大クロック**: 80MHz（GPIO経由）、40MHz（IOマトリクス経由）
- **DMAサポート**: 最大64KBの連続転送
- **マルチデバイス**: 1バスに最大6デバイス接続可能
- **柔軟なGPIOマッピング**: IOマトリクス経由で任意のピン使用可能

## StampFly SPIピン配置

```cpp
// StampFly標準SPIピン定義（ESP32-S3）
#define PIN_NUM_MISO  43  // Master In Slave Out
#define PIN_NUM_MOSI  14  // Master Out Slave In
#define PIN_NUM_CLK   44  // Serial Clock
#define PIN_NUM_CS    46  // Chip Select (BMI270用)

// その他のセンサーCSピン（要確認）
#define CS_BMM150     xx  // 磁気センサー
#define CS_BMP280     xx  // 気圧センサー
#define CS_PMW3901    12  // オプティカルフロー
```

## ESP-IDF SPI Masterアーキテクチャ

### 1. 階層構造

```
アプリケーション層
    ↓
SPI Master Driver (spi_master.h)
    ↓
SPI HAL (Hardware Abstraction Layer)
    ↓
SPI LL (Low Level Driver)
    ↓
物理SPIペリフェラル (SPI2/SPI3)
```

### 2. 主要構造体

#### spi_bus_config_t - バス設定
```cpp
typedef struct {
    int mosi_io_num;      // MOSIピン番号（-1で未使用）
    int miso_io_num;      // MISOピン番号（-1で未使用）
    int sclk_io_num;      // SCLKピン番号
    int quadwp_io_num;    // WPピン（Quad SPIモード、通常-1）
    int quadhd_io_num;    // HDピン（Quad SPIモード、通常-1）
    int data4_io_num;     // Octal用（ESP32-S3、通常-1）
    int data5_io_num;     // Octal用（ESP32-S3、通常-1）
    int data6_io_num;     // Octal用（ESP32-S3、通常-1）
    int data7_io_num;     // Octal用（ESP32-S3、通常-1）
    int max_transfer_sz;  // 最大転送サイズ（デフォルト4092）
    uint32_t flags;       // SPICOMMON_BUSFLAG_* フラグ
    int intr_flags;       // 割り込みフラグ（0で自動割り当て）
} spi_bus_config_t;
```

#### spi_device_interface_config_t - デバイス設定
```cpp
typedef struct {
    uint8_t command_bits;      // コマンドフェーズビット数（0-16）
    uint8_t address_bits;      // アドレスフェーズビット数（0-64）
    uint8_t dummy_bits;        // ダミービット数
    uint8_t mode;              // SPIモード（0-3）
    uint16_t duty_cycle_pos;   // クロックデューティ比（128 = 50%）
    uint16_t cs_ena_pretrans;  // CS有効前の待機SPI clkサイクル数
    uint8_t cs_ena_posttrans;  // CS有効後の待機SPI clkサイクル数
    int clock_speed_hz;        // クロック周波数（Hz）
    int input_delay_ns;        // MISO信号遅延補償（ns）
    int spics_io_num;          // CSピン番号
    uint32_t flags;            // SPI_DEVICE_* フラグ
    int queue_size;            // トランザクションキューサイズ
    transaction_cb_t pre_cb;   // トランザクション前コールバック
    transaction_cb_t post_cb;  // トランザクション後コールバック
} spi_device_interface_config_t;
```

#### spi_transaction_t - トランザクション構造体
```cpp
typedef struct {
    uint32_t flags;           // SPI_TRANS_* フラグ
    uint16_t cmd;             // コマンドデータ（command_bits分）
    uint64_t addr;            // アドレスデータ（address_bits分）
    size_t length;            // 全データ長（ビット単位）
    size_t rxlength;          // 受信データ長（0=lengthと同じ）
    void *user;               // ユーザーデータポインタ
    union {
        const void *tx_buffer;// 送信バッファポインタ
        uint8_t tx_data[4];   // 4バイト以下の送信データ
    };
    union {
        void *rx_buffer;      // 受信バッファポインタ
        uint8_t rx_data[4];   // 4バイト以下の受信データ
    };
} spi_transaction_t;
```

## SPI初期化フロー

### StampFly標準初期化シーケンス

```cpp
// 1. バス初期化
esp_err_t init_stampfly_spi_bus() {
    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,  // 未使用
        .quadhd_io_num = -1,  // 未使用
        .data4_io_num = -1,   // ESP32-S3 Octal未使用
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 4096,  // 最大転送サイズ
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS,
        .intr_flags = 0       // 自動割り当て
    };

    // DMAチャンネル自動選択でバス初期化
    return spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
}

// 2. デバイス追加（例：BMI270）
esp_err_t add_bmi270_device(spi_device_handle_t* handle) {
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,         // BMI270はコマンドフェーズ不使用
        .address_bits = 0,         // アドレスフェーズ不使用
        .dummy_bits = 0,           // ダミービット不要
        .mode = 0,                 // SPIモード0（CPOL=0, CPHA=0）
        .duty_cycle_pos = 128,     // 50%デューティ
        .cs_ena_pretrans = 1,      // CS有効前1クロック待機
        .cs_ena_posttrans = 1,     // CS有効後1クロック待機
        .clock_speed_hz = 1000000, // 初期化時は1MHz（安定重視）
        .input_delay_ns = 0,       // 入力遅延なし
        .spics_io_num = PIN_NUM_CS,
        .flags = 0,                // 標準設定
        .queue_size = 7,           // キューサイズ
        .pre_cb = NULL,
        .post_cb = NULL
    };

    return spi_bus_add_device(SPI2_HOST, &dev_config, handle);
}
```

## トランザクション種別と使い分け

### 1. ポーリングトランザクション（推奨：センサー読み取り）

```cpp
// 同期的、シンプル、小規模データ向け
esp_err_t spi_polling_read(spi_device_handle_t handle,
                           uint8_t reg_addr,
                           uint8_t* data,
                           size_t len) {
    // 読み取りビット設定（BMI270: MSB=1で読み取り）
    uint8_t tx_data[1] = {reg_addr | 0x80};

    spi_transaction_t trans = {
        .flags = 0,
        .length = 8,              // 送信ビット数
        .rxlength = len * 8,      // 受信ビット数
        .tx_buffer = tx_data,
        .rx_buffer = data,
        .user = NULL
    };

    return spi_device_polling_transmit(handle, &trans);
}
```

### 2. 割り込みトランザクション（大量データ転送）

```cpp
// 非同期、DMA活用、大規模データ向け
esp_err_t spi_interrupt_transfer(spi_device_handle_t handle,
                                 const uint8_t* tx_data,
                                 uint8_t* rx_data,
                                 size_t len) {
    spi_transaction_t trans = {
        .flags = 0,
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    // キューに追加（非ブロッキング）
    esp_err_t ret = spi_device_queue_trans(handle, &trans, portMAX_DELAY);
    if (ret != ESP_OK) return ret;

    // 完了待機
    spi_transaction_t* rtrans;
    ret = spi_device_get_trans_result(handle, &rtrans, portMAX_DELAY);

    return ret;
}
```

### 3. 拡張トランザクション（コマンド/アドレス付き）

```cpp
// コマンドとアドレスフェーズを使用
typedef struct {
    struct spi_transaction_t base;  // 基本トランザクション
    uint8_t command_bits;           // 実際のコマンドビット数
    uint8_t address_bits;           // 実際のアドレスビット数
    uint8_t dummy_bits;             // 実際のダミービット数
} spi_transaction_ext_t;

esp_err_t spi_extended_transaction(spi_device_handle_t handle,
                                   uint8_t cmd,
                                   uint32_t addr,
                                   uint8_t* data,
                                   size_t len) {
    spi_transaction_ext_t trans = {};
    trans.base.flags = SPI_TRANS_VARIABLE_CMD |
                       SPI_TRANS_VARIABLE_ADDR;
    trans.base.cmd = cmd;
    trans.base.addr = addr;
    trans.base.length = len * 8;
    trans.base.rx_buffer = data;
    trans.command_bits = 8;
    trans.address_bits = 24;

    return spi_device_polling_transmit(handle, (spi_transaction_t*)&trans);
}
```

## タイミング制御

### 1. クロック設定と実効速度

```cpp
// 実効クロック計算式
// fclk = fapb / (pre_divider * post_divider)
// ここで、fapb = 80MHz（APBクロック）

// 例：10MHz設定時の実際の周波数
int actual_freq = spi_device_get_actual_freq(10000000, SPI2_HOST);
ESP_LOGI(TAG, "Requested: 10MHz, Actual: %dHz", actual_freq);
```

### 2. CS（チップセレクト）タイミング

```cpp
// CSタイミングパラメータの影響
typedef struct {
    uint16_t cs_ena_pretrans;   // CS↓からSCLK開始までの遅延
    uint8_t cs_ena_posttrans;   // 最終SCLKからCS↑までの遅延
} cs_timing_t;

// BMI270向け推奨設定
cs_timing_t bmi270_timing = {
    .cs_ena_pretrans = 2,   // 2クロックサイクル待機
    .cs_ena_posttrans = 2   // 2クロックサイクル待機
};

// センサー別タイミング要件
// BMI270: 最小CS Low期間 = 40ns @ 10MHz
// BMP280: 最小CS Setup = 10ns
// PMW3901: 最小CS Hold = 50ns
```

### 3. MISO入力遅延補償

```cpp
// 高速通信時のMISO信号遅延補償
// 配線長による遅延計算: delay_ns = length_cm * 0.05

int calculate_input_delay(int wire_length_cm, int pcb_trace_delay_ns) {
    int wire_delay = wire_length_cm * 0.05;  // 概算値
    return wire_delay + pcb_trace_delay_ns;
}

// 10MHz以上で推奨
dev_config.input_delay_ns = calculate_input_delay(5, 10);  // 5cm配線 + PCB遅延
```

## DMA設定とパフォーマンス最適化

### 1. DMA使用条件

```cpp
// DMAが有効になる条件
bool is_dma_enabled(size_t transfer_size) {
    // ESP32-S3: 32バイト以上でDMA自動使用
    return transfer_size >= 32;
}

// DMA最適化バッファアライメント
#define DMA_ALIGN __attribute__((aligned(4)))
uint8_t DMA_ALIGN tx_buffer[1024];
uint8_t DMA_ALIGN rx_buffer[1024];
```

### 2. パフォーマンス最適化テクニック

```cpp
// バーストリード実装（BMI270アクセロメータ6バイト読み取り）
esp_err_t bmi270_burst_read_accel(spi_device_handle_t handle,
                                  int16_t* accel_xyz) {
    uint8_t tx_buf[7] = {0x0C | 0x80};  // ACC_X_LSBレジスタ + 読み取りビット
    uint8_t rx_buf[7] = {0};

    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .length = 7 * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };

    esp_err_t ret = spi_device_polling_transmit(handle, &trans);
    if (ret == ESP_OK) {
        // 最初のバイトはダミー、2-7バイトがデータ
        accel_xyz[0] = (rx_buf[2] << 8) | rx_buf[1];  // X軸
        accel_xyz[1] = (rx_buf[4] << 8) | rx_buf[3];  // Y軸
        accel_xyz[2] = (rx_buf[6] << 8) | rx_buf[5];  // Z軸
    }
    return ret;
}
```

### 3. マルチデバイス最適化

```cpp
// 複数センサー同時アクセス管理
typedef struct {
    spi_device_handle_t bmi270;
    spi_device_handle_t bmp280;
    spi_device_handle_t bmm150;
    SemaphoreHandle_t bus_mutex;  // バス排他制御
} sensor_spi_manager_t;

// トランザクション優先度管理
esp_err_t prioritized_sensor_read(sensor_spi_manager_t* mgr,
                                  sensor_type_t sensor,
                                  uint8_t* data,
                                  size_t len) {
    // 高優先度センサー（IMU）を先に処理
    if (xSemaphoreTake(mgr->bus_mutex, portMAX_DELAY) == pdTRUE) {
        esp_err_t ret;

        switch(sensor) {
            case SENSOR_IMU:
                ret = spi_device_polling_transmit(mgr->bmi270, ...);
                break;
            case SENSOR_BARO:
                ret = spi_device_polling_transmit(mgr->bmp280, ...);
                break;
            // ...
        }

        xSemaphoreGive(mgr->bus_mutex);
        return ret;
    }
    return ESP_ERR_TIMEOUT;
}
```

## センサー別実装パターン

### BMI270（6軸IMU）特有の実装

```cpp
// BMI270 SPI通信特性
// - レジスタアドレスMSB = R/W判定（1:読み取り、0:書き込み）
// - 最初の読み取りバイトはダミー
// - 最大10MHz（初期化時は1MHz推奨）

// 初期化シーケンス
esp_err_t bmi270_init_sequence(spi_device_handle_t handle) {
    esp_err_t ret;

    // 1. ダミーリード（I2C→SPIモード切り替え）
    uint8_t dummy;
    for(int i = 0; i < 10; i++) {
        ret = bmi270_read_register(handle, 0x00, &dummy);
        vTaskDelay(1);
    }

    // 2. チップID確認
    uint8_t chip_id;
    ret = bmi270_read_register(handle, 0x00, &chip_id);
    if(chip_id != 0x24) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X", chip_id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 3. 設定ファイル書き込み（8192バイト）
    ret = bmi270_write_config_file(handle);

    // 4. センサー有効化
    ret = bmi270_enable_sensors(handle);

    return ret;
}
```

### BMP280（気圧センサー）特有の実装

```cpp
// BMP280 SPI通信特性
// - レジスタアドレスMSB = R/W判定（1:読み取り、0:書き込み）
// - 最大10MHz
// - 補償値読み取り必須

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    // ... 他の補償値
} bmp280_calib_data_t;

esp_err_t bmp280_read_calib_data(spi_device_handle_t handle,
                                 bmp280_calib_data_t* calib) {
    uint8_t calib_regs[26];

    // 0x88-0xA1の補償レジスタを一括読み取り
    esp_err_t ret = spi_burst_read(handle, 0x88, calib_regs, 26);

    if(ret == ESP_OK) {
        calib->dig_T1 = (calib_regs[1] << 8) | calib_regs[0];
        calib->dig_T2 = (calib_regs[3] << 8) | calib_regs[2];
        // ... パース処理
    }

    return ret;
}
```

### PMW3901（オプティカルフロー）特有の実装

```cpp
// PMW3901 SPI通信特性
// - 特殊な読み書きプロトコル
// - 最大2MHz
// - モーションバースト読み取りモード

// PMW3901専用書き込み（特殊タイミング）
esp_err_t pmw3901_write_register(spi_device_handle_t handle,
                                 uint8_t reg,
                                 uint8_t value) {
    uint8_t tx_data[2] = {reg | 0x80, value};  // MSB=1で書き込み

    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = NULL
    };

    esp_err_t ret = spi_device_polling_transmit(handle, &trans);
    vTaskDelay(pdMS_TO_TICKS(1));  // 必須待機

    return ret;
}

// モーションバーストリード（12バイト）
typedef struct {
    uint8_t motion;
    uint8_t observation;
    int16_t delta_x;
    int16_t delta_y;
    uint8_t squal;
    uint8_t raw_data_sum;
    uint8_t max_raw_data;
    uint8_t min_raw_data;
    uint16_t shutter;
} pmw3901_motion_data_t;
```

## トラブルシューティング

### 1. チップID読み取り失敗（0x00/0xFF応答）

```cpp
// 診断手順
esp_err_t diagnose_spi_connection(spi_device_handle_t handle) {
    ESP_LOGI(TAG, "=== SPI診断開始 ===");

    // 1. 配線チェック（ループバックテスト）
    if(ENABLE_LOOPBACK_TEST) {
        // MOSI-MISOを短絡してテスト
        uint8_t tx = 0xA5;
        uint8_t rx = 0;
        spi_loopback_test(handle, &tx, &rx, 1);
        if(tx != rx) {
            ESP_LOGE(TAG, "配線エラー: TX=0x%02X, RX=0x%02X", tx, rx);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    // 2. クロック確認（オシロスコープ推奨）
    ESP_LOGI(TAG, "SCLKピン(%d)の信号を確認してください", PIN_NUM_CLK);

    // 3. CS制御確認
    gpio_set_level(PIN_NUM_CS, 0);
    vTaskDelay(1);
    gpio_set_level(PIN_NUM_CS, 1);
    ESP_LOGI(TAG, "CSピン(%d)トグル完了", PIN_NUM_CS);

    // 4. 電源電圧確認
    // 外部計測必要: VDD=1.8V±5%, VDDIO=1.8V/3.3V

    return ESP_OK;
}
```

### 2. データ化け・通信不安定

```cpp
// 通信品質改善策
typedef struct {
    bool use_slower_clock;      // クロック速度低減
    bool increase_cs_timing;    // CSタイミング余裕増加
    bool add_input_delay;       // 入力遅延補償
    bool use_pullups;          // プルアップ有効化
} spi_stabilization_config_t;

esp_err_t stabilize_spi_communication(spi_device_handle_t* handle,
                                      spi_stabilization_config_t* config) {
    // 既存デバイス削除
    spi_bus_remove_device(*handle);

    // 安定化設定で再作成
    spi_device_interface_config_t dev_config = {
        .mode = 0,
        .clock_speed_hz = config->use_slower_clock ? 400000 : 1000000,
        .cs_ena_pretrans = config->increase_cs_timing ? 10 : 2,
        .cs_ena_posttrans = config->increase_cs_timing ? 10 : 2,
        .input_delay_ns = config->add_input_delay ? 50 : 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,  // キューサイズ最小化
        .flags = config->use_pullups ? SPI_DEVICE_PULLUP : 0
    };

    return spi_bus_add_device(SPI2_HOST, &dev_config, handle);
}
```

### 3. DMA転送エラー

```cpp
// DMA問題診断
void diagnose_dma_issues() {
    size_t dma_cap = 0;
    spi_bus_get_max_transaction_len(SPI2_HOST, &dma_cap);
    ESP_LOGI(TAG, "最大DMA転送サイズ: %d bytes", dma_cap);

    // メモリアライメント確認
    uint8_t* unaligned = (uint8_t*)malloc(100);
    if((uint32_t)unaligned % 4 != 0) {
        ESP_LOGW(TAG, "非アライメントバッファ検出: %p", unaligned);
    }
    free(unaligned);

    // DMA安全バッファ確保
    uint8_t* dma_buffer = heap_caps_malloc(256, MALLOC_CAP_DMA);
    if(!dma_buffer) {
        ESP_LOGE(TAG, "DMAバッファ確保失敗");
    } else {
        ESP_LOGI(TAG, "DMAバッファ確保成功: %p", dma_buffer);
        heap_caps_free(dma_buffer);
    }
}
```

## ベストプラクティス

### 1. エラーハンドリング標準パターン

```cpp
// StampFly推奨エラーハンドリング
#define SPI_CHECK(x) do {                                           \
    esp_err_t ret = (x);                                           \
    if (ret != ESP_OK) {                                          \
        ESP_LOGE(TAG, "SPI error %s at %s:%d",                    \
                esp_err_to_name(ret), __FILE__, __LINE__);        \
        return ret;                                               \
    }                                                              \
} while(0)

// 使用例
esp_err_t sensor_init() {
    SPI_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
    SPI_CHECK(spi_bus_add_device(SPI2_HOST, &dev_config, &handle));
    SPI_CHECK(verify_sensor_id(handle));
    return ESP_OK;
}
```

### 2. リトライメカニズム

```cpp
// 通信リトライ実装
esp_err_t spi_read_with_retry(spi_device_handle_t handle,
                              uint8_t reg,
                              uint8_t* data,
                              int max_retries) {
    esp_err_t ret;
    int retry = 0;

    while(retry < max_retries) {
        ret = spi_read_register(handle, reg, data);

        if(ret == ESP_OK) {
            if(retry > 0) {
                ESP_LOGW(TAG, "成功 (リトライ %d回)", retry);
            }
            return ESP_OK;
        }

        ESP_LOGW(TAG, "読み取り失敗 (試行 %d/%d)", retry+1, max_retries);

        if(retry == max_retries/2) {
            // 中間地点でリセット試行
            ESP_LOGW(TAG, "バスリセット実行");
            spi_bus_reset(handle);
        }

        vTaskDelay(pdMS_TO_TICKS(10 * (retry + 1)));  // 指数バックオフ
        retry++;
    }

    return ret;
}
```

### 3. デバッグユーティリティ

```cpp
// SPI通信ログ機能
void log_spi_transaction(const char* operation,
                         uint8_t reg,
                         const uint8_t* data,
                         size_t len) {
    #ifdef DEBUG_SPI
    printf("[SPI] %s reg=0x%02X: ", operation, reg);
    for(size_t i = 0; i < len; i++) {
        printf("0x%02X ", data[i]);
    }
    printf("\n");
    #endif
}

// タイミング測定
int64_t measure_transaction_time(spi_device_handle_t handle,
                                 spi_transaction_t* trans) {
    int64_t start = esp_timer_get_time();
    esp_err_t ret = spi_device_polling_transmit(handle, trans);
    int64_t duration = esp_timer_get_time() - start;

    ESP_LOGI(TAG, "Transaction time: %lld us, status: %s",
            duration, esp_err_to_name(ret));

    return duration;
}
```

### 4. 省電力対策

```cpp
// アイドル時のSPI省電力管理
typedef struct {
    spi_device_handle_t handle;
    bool is_active;
    int64_t last_access_time;
} power_managed_spi_t;

esp_err_t pm_spi_access(power_managed_spi_t* pm_spi,
                        spi_transaction_t* trans) {
    // アクティブ化
    if(!pm_spi->is_active) {
        // クロック有効化、電源投入など
        ESP_LOGI(TAG, "SPIデバイス起動");
        pm_spi->is_active = true;
    }

    // トランザクション実行
    esp_err_t ret = spi_device_polling_transmit(pm_spi->handle, trans);

    // 最終アクセス時刻更新
    pm_spi->last_access_time = esp_timer_get_time();

    return ret;
}

// 定期的な省電力チェック
void pm_spi_idle_check(power_managed_spi_t* pm_spi, int idle_threshold_ms) {
    if(pm_spi->is_active) {
        int64_t idle_time = (esp_timer_get_time() - pm_spi->last_access_time) / 1000;

        if(idle_time > idle_threshold_ms) {
            ESP_LOGI(TAG, "SPIデバイス省電力モード移行");
            // クロック停止、必要に応じて電源カットなど
            pm_spi->is_active = false;
        }
    }
}
```

## まとめ

ESP-IDF v5.4のSPI Master APIは強力で柔軟性が高いが、StampFlyのセンサー群を正しく制御するには各センサーの特性を理解した実装が必要。特に以下の点が重要：

1. **初期化順序の厳守**: バス初期化→デバイス追加→センサー固有初期化
2. **タイミング制御**: 各センサーのデータシート要件に準拠
3. **エラーハンドリング**: 適切なリトライとリカバリメカニズム
4. **パフォーマンス最適化**: DMA活用とバッファアライメント
5. **デバッグ機能**: 問題診断用のユーティリティ実装

本ガイドのコード例はStampFlyの実装で直接使用可能な形で提供しており、センサーHAL開発の基盤として活用できる。
#pragma once

#include "hal_base.h"
#include "spi_hal.h"
#include <esp_err.h>

namespace stampfly_hal {

// ========== FIFO関連のデータ構造定義 ==========

// FIFOモード
enum class FifoMode {
    FIFO,    // FIFOモード（フル時停止）
    STREAM   // Streamモード（フル時上書き）
};

// ダウンサンプリング設定（Phase 4で使用、定義は今回）
enum class FifoDownsample : uint8_t {
    NO_DOWNSAMPLE = 0,  // 1600Hz
    DOWNSAMPLE_2X = 1,  // 800Hz
    DOWNSAMPLE_4X = 2,  // 400Hz
    DOWNSAMPLE_8X = 3,  // 200Hz
    DOWNSAMPLE_16X = 4, // 100Hz
    DOWNSAMPLE_32X = 5, // 50Hz
    DOWNSAMPLE_64X = 6, // 25Hz
    DOWNSAMPLE_128X = 7 // 12.5Hz
};

// FIFOサンプル（解析後データ）
struct FifoSample {
    // 生データ（16ビット整数、2の補数表現）
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    // 物理量変換後（SI単位系）
    float accel_x_mps2;  // m/s²
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_rps;    // rad/s
    float gyro_y_rps;
    float gyro_z_rps;

    // タイムスタンプ（Phase 3で使用、今回は0初期化）
    uint32_t sensor_time_ticks;  // BMI270内部時刻（24ビット）
    uint64_t timestamp_us;        // マイクロ秒変換値

    // コンストラクタ（ゼロ初期化）
    FifoSample() :
        accel_x_raw(0), accel_y_raw(0), accel_z_raw(0),
        gyro_x_raw(0), gyro_y_raw(0), gyro_z_raw(0),
        accel_x_mps2(0.0f), accel_y_mps2(0.0f), accel_z_mps2(0.0f),
        gyro_x_rps(0.0f), gyro_y_rps(0.0f), gyro_z_rps(0.0f),
        sensor_time_ticks(0), timestamp_us(0) {}
};

// FIFOステータス情報
struct FifoStatus {
    bool enabled;                // FIFO有効化状態
    bool overflow;               // オーバーフロー発生フラグ
    uint16_t current_length;     // 現在のFIFOデータ長（バイト）
    uint16_t watermark;          // 設定済みウォーターマーク（バイト）
    uint32_t read_count;         // 累積読み取り回数
    uint32_t overflow_count;     // オーバーフロー回数
    uint16_t max_samples;        // 最大サンプル数記録
    float average_samples;       // 平均サンプル数/読み取り

    // コンストラクタ（ゼロ初期化）
    FifoStatus() :
        enabled(false), overflow(false), current_length(0),
        watermark(0), read_count(0), overflow_count(0),
        max_samples(0), average_samples(0.0f) {}
};

class BMI270 : public HALBase {
public:
    // BMI270 register addresses
    static constexpr uint8_t REG_CHIP_ID = 0x00;
    static constexpr uint8_t REG_ERR_REG = 0x02;
    static constexpr uint8_t REG_STATUS = 0x03;
    static constexpr uint8_t REG_ACC_X_LSB = 0x0C;
    static constexpr uint8_t REG_ACC_X_MSB = 0x0D;
    static constexpr uint8_t REG_ACC_Y_LSB = 0x0E;
    static constexpr uint8_t REG_ACC_Y_MSB = 0x0F;
    static constexpr uint8_t REG_ACC_Z_LSB = 0x10;
    static constexpr uint8_t REG_ACC_Z_MSB = 0x11;
    static constexpr uint8_t REG_GYR_X_LSB = 0x12;
    static constexpr uint8_t REG_GYR_X_MSB = 0x13;
    static constexpr uint8_t REG_GYR_Y_LSB = 0x14;
    static constexpr uint8_t REG_GYR_Y_MSB = 0x15;
    static constexpr uint8_t REG_GYR_Z_LSB = 0x16;
    static constexpr uint8_t REG_GYR_Z_MSB = 0x17;
    static constexpr uint8_t REG_SENSORTIME_0 = 0x18;
    static constexpr uint8_t REG_SENSORTIME_1 = 0x19;
    static constexpr uint8_t REG_SENSORTIME_2 = 0x1A;
    static constexpr uint8_t REG_INT_STATUS_1 = 0x1D;
    static constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
    static constexpr uint8_t REG_TEMPERATURE_LSB = 0x22;
    static constexpr uint8_t REG_TEMPERATURE_MSB = 0x23;
    static constexpr uint8_t REG_FIFO_LENGTH_0 = 0x24;
    static constexpr uint8_t REG_FIFO_LENGTH_1 = 0x25;
    static constexpr uint8_t REG_FIFO_DATA = 0x26;
    static constexpr uint8_t REG_ACC_CONF = 0x40;
    static constexpr uint8_t REG_ACC_RANGE = 0x41;
    static constexpr uint8_t REG_GYR_CONF = 0x42;
    static constexpr uint8_t REG_GYR_RANGE = 0x43;
    static constexpr uint8_t REG_FIFO_DOWNS = 0x45;
    static constexpr uint8_t REG_FIFO_WTM_0 = 0x46;
    static constexpr uint8_t REG_FIFO_WTM_1 = 0x47;
    static constexpr uint8_t REG_FIFO_CONFIG_0 = 0x48;
    static constexpr uint8_t REG_FIFO_CONFIG_1 = 0x49;
    static constexpr uint8_t REG_INT1_IO_CTRL = 0x53;
    static constexpr uint8_t REG_INT_MAP_DATA = 0x58;
    static constexpr uint8_t REG_INIT_CTRL = 0x59;
    static constexpr uint8_t REG_INIT_ADDR_0 = 0x5B;
    static constexpr uint8_t REG_INIT_ADDR_1 = 0x5C;
    static constexpr uint8_t REG_INIT_DATA = 0x5E;
    static constexpr uint8_t REG_PWR_CONF = 0x7C;
    static constexpr uint8_t REG_PWR_CTRL = 0x7D;
    static constexpr uint8_t REG_CMD = 0x7E;

    // BMI270 constants
    static constexpr uint8_t CHIP_ID_VALUE = 0x24;
    static constexpr uint8_t SPI_READ_FLAG = 0x80;
    static constexpr uint8_t CMD_SOFT_RESET = 0xB6;

    // Power control bits
    static constexpr uint8_t PWR_CTRL_AUX_EN = 0x01;
    static constexpr uint8_t PWR_CTRL_GYR_EN = 0x02;
    static constexpr uint8_t PWR_CTRL_ACC_EN = 0x04;
    static constexpr uint8_t PWR_CTRL_TEMP_EN = 0x08;

    // Status bits
    static constexpr uint8_t STATUS_DRDY_ACC = 0x80;
    static constexpr uint8_t STATUS_DRDY_GYR = 0x40;

    // Interrupt configuration bits
    static constexpr uint8_t INT1_OUTPUT_EN = 0x08;    // Enable INT1 output
    static constexpr uint8_t INT1_ACTIVE_HIGH = 0x02;  // INT1 active high (vs active low)
    static constexpr uint8_t INT1_PUSH_PULL = 0x00;    // INT1 push-pull (vs open-drain)
    static constexpr uint8_t INT_DRDY_EN = 0x04;       // Data ready interrupt enable

    // FIFO configuration bits
    // FIFO_CONFIG_0 (0x48) bits
    static constexpr uint8_t FIFO_MODE_FIFO = 0x01;     // Bit 0: FIFOモード有効
    static constexpr uint8_t FIFO_MODE_STREAM = 0x00;   // Bit 0: Streamモード
    static constexpr uint8_t FIFO_STOP_ON_FULL = 0x02;  // Bit 1: フル時停止

    // FIFO_CONFIG_1 (0x49) bits
    static constexpr uint8_t FIFO_ACC_EN = 0x40;        // Bit 6: 加速度有効化
    static constexpr uint8_t FIFO_GYR_EN = 0x80;        // Bit 7: ジャイロ有効化
    static constexpr uint8_t FIFO_TEMP_EN = 0x08;       // Bit 3: 温度有効化
    static constexpr uint8_t FIFO_TIME_EN = 0x02;       // Bit 1: センサータイム有効化
    static constexpr uint8_t FIFO_HEADER_EN = 0x10;     // Bit 4: ヘッダーモード有効化

    // INT_STATUS_1 (0x1D) bits
    static constexpr uint8_t FIFO_FULL_INT_BIT = 0x40;  // Bit 6: FIFOフル割り込み

    // FIFO constants
    static constexpr uint16_t FIFO_CAPACITY_BYTES = 6144;  // 6KB
    static constexpr uint8_t FIFO_FRAME_SIZE_HEADERLESS = 12;  // 加速度(6) + ジャイロ(6)
    static constexpr uint16_t FIFO_MAX_SAMPLES = FIFO_CAPACITY_BYTES / FIFO_FRAME_SIZE_HEADERLESS;  // 512サンプル

    // Hardware pin definitions (StampFly specific)
    static constexpr int GPIO_INT1_PIN = 11;  // INT1 connected to GPIO11

    BMI270();
    ~BMI270() = default;

    // HALBase interface
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // Sensor data structures
    struct AccelData {
        float x;  // in m/s^2
        float y;
        float z;
    };

    struct GyroData {
        float x;  // in rad/s
        float y;
        float z;
    };

    // BMI270 specific functions
    esp_err_t read_chip_id(uint8_t* chip_id);
    esp_err_t test_spi_communication();

    // Complete initialization functions
    esp_err_t upload_config_file();
    esp_err_t configure_sensors();
    esp_err_t soft_reset();

    // Data acquisition
    esp_err_t read_accel(AccelData& data);
    esp_err_t read_gyro(GyroData& data);
    esp_err_t read_temperature(float& temp_c);
    esp_err_t get_status(uint8_t& status);
    esp_err_t get_error_status(uint8_t& error);

    // Interrupt configuration
    esp_err_t configure_data_ready_interrupt();
    esp_err_t enable_data_ready_interrupt();
    esp_err_t disable_data_ready_interrupt();

    // Get INT1 pin for external GPIO interrupt setup
    static constexpr int get_int1_pin() { return GPIO_INT1_PIN; }

    // ========== FIFO機能（Phase 1） ==========

    // FIFO基本操作
    esp_err_t enable_fifo(bool accel, bool gyro);
    esp_err_t disable_fifo();
    esp_err_t flush_fifo();

    // FIFO設定
    esp_err_t set_fifo_watermark(uint16_t bytes);
    esp_err_t set_fifo_mode(FifoMode mode);

    // FIFOデータ読み取り
    esp_err_t read_fifo_length(uint16_t* length);
    esp_err_t read_fifo_data(uint8_t* buffer, uint16_t length);

    // FIFOデータ解析
    esp_err_t parse_fifo_data(
        const uint8_t* buffer,
        uint16_t buffer_length,
        FifoSample* samples,
        uint16_t max_samples,
        uint16_t* parsed_count
    );

    // FIFO診断
    esp_err_t get_fifo_status(FifoStatus* status);
    void print_fifo_diagnostics() const;
    void print_sensor_config_diagnostics();  // センサー設定診断（ODR, FIFO等）

private:
    SpiHal* spi_hal_;
    spi_device_handle_t device_handle_;

    // Configuration settings
    uint8_t accel_range_;  // 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
    uint8_t gyro_range_;   // 0: ±2000dps, 1: ±1000dps, 2: ±500dps, 3: ±250dps, 4: ±125dps
    float accel_scale_;    // Scale factor for accelerometer
    float gyro_scale_;     // Scale factor for gyroscope

    // FIFO state management
    bool fifo_enabled_;
    bool fifo_accel_enabled_;
    bool fifo_gyro_enabled_;
    FifoMode fifo_mode_;
    uint16_t fifo_watermark_;

    // FIFO statistics
    uint32_t fifo_read_count_;
    uint32_t fifo_overflow_count_;
    uint32_t fifo_total_samples_;
    uint16_t fifo_max_samples_;

    // CS control options for burst write
    enum class CsControl {
        NORMAL,      // Normal operation: CS auto-controlled (LOW during transaction, HIGH after)
        KEEP_ACTIVE  // Keep CS active (LOW) after transaction for continuous write
    };

    // Low-level SPI functions
    esp_err_t read_register(uint8_t reg_addr, uint8_t* data);
    esp_err_t burst_read(uint8_t reg_addr, uint8_t* data, size_t len);
    esp_err_t write_register(uint8_t reg_addr, uint8_t data);
    esp_err_t burst_write(uint8_t reg_addr, const uint8_t* data, size_t len);
    esp_err_t burst_write_cs_control(uint8_t reg_addr, const uint8_t* data, size_t len, CsControl cs_ctrl);

    // Initialization helpers
    esp_err_t setup_spi();
    esp_err_t switch_to_spi_mode();
    esp_err_t wait_for_initialization();
    esp_err_t disable_advanced_power_save();

    // FIFO internal helpers
    esp_err_t configure_fifo_registers();
    esp_err_t check_fifo_overflow(bool* overflow);
    uint16_t calculate_sample_count(uint16_t fifo_bytes) const;
    void convert_raw_to_physical(FifoSample& sample);
};

} // namespace stampfly_hal
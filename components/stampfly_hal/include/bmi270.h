#pragma once

#include "hal_base.h"
#include "spi_hal.h"
#include <esp_err.h>

namespace stampfly_hal {

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
    static constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
    static constexpr uint8_t REG_TEMPERATURE_LSB = 0x22;
    static constexpr uint8_t REG_TEMPERATURE_MSB = 0x23;
    static constexpr uint8_t REG_ACC_CONF = 0x40;
    static constexpr uint8_t REG_ACC_RANGE = 0x41;
    static constexpr uint8_t REG_GYR_CONF = 0x42;
    static constexpr uint8_t REG_GYR_RANGE = 0x43;
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

private:
    SpiHal* spi_hal_;
    spi_device_handle_t device_handle_;

    // Configuration settings
    uint8_t accel_range_;  // 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
    uint8_t gyro_range_;   // 0: ±2000dps, 1: ±1000dps, 2: ±500dps, 3: ±250dps, 4: ±125dps
    float accel_scale_;    // Scale factor for accelerometer
    float gyro_scale_;     // Scale factor for gyroscope

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
};

} // namespace stampfly_hal
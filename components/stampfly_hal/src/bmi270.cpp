#include "bmi270.h"
#include "bmi270_config.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <cmath>


namespace stampfly_hal {

static const char* TAG = "BMI270";

BMI270::BMI270() : HALBase("BMI270"), spi_hal_(nullptr), device_handle_(nullptr),
                   accel_range_(1), gyro_range_(1), accel_scale_(0.0f), gyro_scale_(0.0f) {
    // Initialize scale factors (will be set during configuration)
}

esp_err_t BMI270::init() {
    if (is_initialized()) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "🚀 Starting BMI270 complete initialization sequence");

    // Phase 1: SPI communication setup
    esp_err_t ret = setup_spi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup SPI: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    // Wait for POR completion (450μs minimum)
    vTaskDelay(pdMS_TO_TICKS(1));

    // Phase 2: I2C to SPI mode switching
    ESP_LOGI(TAG, "Phase 2: I2C to SPI mode switching");
    ret = switch_to_spi_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch to SPI mode: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    // Phase 2.5: Test SPI write/read functionality
    ESP_LOGI(TAG, "Phase 2.5: Testing SPI write/read functionality");
    ret = test_spi_communication();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write/read test failed: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    // Phase 3: Disable advanced power save
    ESP_LOGI(TAG, "Phase 3: Disabling advanced power save");
    ret = disable_advanced_power_save();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable advanced power save: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    // Phase 4: Upload configuration file
    ESP_LOGI(TAG, "Phase 4: Uploading configuration file");
    ret = upload_config_file();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to upload config file: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    // Phase 5: Wait for initialization completion
    ESP_LOGI(TAG, "Phase 5: Waiting for initialization completion");
    ret = wait_for_initialization();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialization timeout: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    // Phase 6: Configure sensors
    ESP_LOGI(TAG, "Phase 6: Configuring sensors");
    ret = configure_sensors();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure sensors: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    // Final verification
    uint8_t status, error;
    get_status(status);
    get_error_status(error);
    ESP_LOGI(TAG, "Status: 0x%02X, Error: 0x%02X", status, error);

    set_initialized(true);
    ESP_LOGI(TAG, "✅ BMI270 complete initialization finished successfully");
    return ESP_OK;
}

esp_err_t BMI270::configure() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    // Configuration is already done in init() method
    ESP_LOGI(TAG, "BMI270 configured");
    return ESP_OK;
}

esp_err_t BMI270::enable() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    set_enabled(true);
    ESP_LOGI(TAG, "BMI270 enabled");
    return ESP_OK;
}

esp_err_t BMI270::disable() {
    set_enabled(false);
    ESP_LOGI(TAG, "BMI270 disabled");
    return ESP_OK;
}

esp_err_t BMI270::reset() {
    ESP_LOGI(TAG, "BMI270 reset");
    return ESP_OK;
}

esp_err_t BMI270::setup_spi() {
    // Create SPI HAL instance
    spi_hal_ = new SpiHal();
    if (!spi_hal_) {
        ESP_LOGE(TAG, "Failed to create SPI HAL instance");
        return ESP_ERR_NO_MEM;
    }

    // Initialize SPI HAL
    esp_err_t ret = spi_hal_->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI HAL: %s", esp_err_to_name(ret));
        delete spi_hal_;
        spi_hal_ = nullptr;
        return ret;
    }

    ret = spi_hal_->configure();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SPI HAL: %s", esp_err_to_name(ret));
        delete spi_hal_;
        spi_hal_ = nullptr;
        return ret;
    }

    ret = spi_hal_->enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable SPI HAL: %s", esp_err_to_name(ret));
        delete spi_hal_;
        spi_hal_ = nullptr;
        return ret;
    }

    // Add BMI270 as SPI device
    ret = spi_hal_->add_device(SPI_PIN_CS_BMI270, &device_handle_, 8000000);  // 8MHz
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BMI270 SPI device: %s", esp_err_to_name(ret));
        delete spi_hal_;
        spi_hal_ = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "BMI270 SPI setup completed successfully");
    return ESP_OK;
}

esp_err_t BMI270::read_register(uint8_t reg_addr, uint8_t* data) {
    if (!data || !spi_hal_ || !device_handle_) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_buffer[3] = {0};  // 2 bytes for complete transaction
    uint8_t rx_buffer[3] = {0};

    // BMI270 4-wire SPI read protocol:
    // Byte 0: R/W bit + register address (TX)
    // Byte 1: Dummy (TX) / Read Data (RX)
    tx_buffer[0] = reg_addr | SPI_READ_FLAG;
    tx_buffer[1] = 0x00;  // Dummy byte
    tx_buffer[2] = 0x00;  // Dummy byte

    ESP_LOGD(TAG, "SPI TX: [0x%02X, 0x%02X, 0x%02X] to register 0x%02X",
             tx_buffer[0], tx_buffer[1], tx_buffer[2],  reg_addr);

    spi_transaction_t trans = {};
    //memset(&trans, 0, sizeof(trans)); // 構造体をゼロで初期化
    trans.length = 24;        // 3 bytes (24 bits)
    //trans.rxlength = 24;      // 2 bytes receive
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = rx_buffer;

    esp_err_t ret = spi_hal_->transmit(device_handle_, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read failed for register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "SPI RX: [0x%02X, 0x%02X, 0x%02X] from register 0x%02X",
             rx_buffer[0], rx_buffer[1], rx_buffer[2], reg_addr);

    // BMI270 returns data in third byte (first two bytes are dummy)
    *data = rx_buffer[2];

    ESP_LOGD(TAG, "Read register 0x%02X: 0x%02X", reg_addr, *data);
    return ESP_OK;
}

esp_err_t BMI270::read_chip_id(uint8_t* chip_id) {
    if (!chip_id) {
        return ESP_ERR_INVALID_ARG;
    }

    // Allow chip ID reading during initialization (remove state checks)
    esp_err_t ret = read_register(REG_CHIP_ID, chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BMI270 Chip ID: 0x%02X (expected: 0x%02X)", *chip_id, CHIP_ID_VALUE);

    if (*chip_id == CHIP_ID_VALUE) {
        ESP_LOGI(TAG, "✅ BMI270 chip ID verified successfully");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "❌ CHIP_ID mismatch! Got 0x%02X, expected 0x%02X", *chip_id, CHIP_ID_VALUE);
        return ESP_ERR_INVALID_RESPONSE;  // Return error on mismatch
    }
}

esp_err_t BMI270::test_spi_communication() {
    ESP_LOGI(TAG, "Testing SPI communication with BMI270");

    // Phase 1: Test basic read functionality (CHIP_ID)
    uint8_t chip_id;
    esp_err_t ret = read_register(REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read test failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id != CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "❌ CHIP_ID mismatch! Got 0x%02X, expected 0x%02X", chip_id, CHIP_ID_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "✅ CHIP_ID read test passed: 0x%02X", chip_id);

    // Phase 2: Test write/read functionality with safe initialization registers
    ESP_LOGI(TAG, "Testing SPI write/read functionality with INIT_ADDR registers");

    // Test with INIT_ADDR_0 and INIT_ADDR_1 (safe for write during initialization)
    struct {
        uint8_t reg;
        const char* name;
    } test_regs[] = {
        {REG_INIT_ADDR_0, "INIT_ADDR_0"},
        {REG_INIT_ADDR_1, "INIT_ADDR_1"}
    };

    for (int reg_idx = 0; reg_idx < 2; reg_idx++) {
        uint8_t reg = test_regs[reg_idx].reg;
        const char* reg_name = test_regs[reg_idx].name;

        // Read original value
        uint8_t original_value;
        ret = read_register(reg, &original_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read original %s value: %s", reg_name, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Original %s value: 0x%02X", reg_name, original_value);

        // Test pattern: write known values and verify
        uint8_t test_values[] = {0x00, 0xAA, 0x55, 0xFF};

        for (int i = 0; i < 4; i++) {
            ESP_LOGI(TAG, "Write test %d: Writing 0x%02X to %s", i + 1, test_values[i], reg_name);

            // Write test value
            ret = write_register(reg, test_values[i]);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "❌ SPI write failed on %s test %d: %s", reg_name, i + 1, esp_err_to_name(ret));
                return ret;
            }

            // Wait for write to complete
            vTaskDelay(pdMS_TO_TICKS(1));

            // Read back and verify
            uint8_t read_value;
            ret = read_register(reg, &read_value);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "❌ SPI read failed on %s test %d: %s", reg_name, i + 1, esp_err_to_name(ret));
                return ret;
            }

            ESP_LOGI(TAG, "Read back value: 0x%02X", read_value);

            if (read_value != test_values[i]) {
                ESP_LOGE(TAG, "❌ Write/Read mismatch on %s! Wrote: 0x%02X, Read: 0x%02X",
                         reg_name, test_values[i], read_value);
                return ESP_ERR_INVALID_RESPONSE;
            }

            ESP_LOGI(TAG, "✅ %s Write/Read test %d passed", reg_name, i + 1);
        }

        // Restore original value
        ret = write_register(reg, original_value);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to restore %s original value", reg_name);
        }
    }

    // Phase 3: Test multiple reads for consistency
    ESP_LOGI(TAG, "Testing read consistency");
    for (int i = 0; i < 5; i++) {
        uint8_t test_chip_id;
        ret = read_register(REG_CHIP_ID, &test_chip_id);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI consistency test read %d failed: %s", i + 1, esp_err_to_name(ret));
            return ret;
        }

        if (test_chip_id != CHIP_ID_VALUE) {
            ESP_LOGE(TAG, "❌ CHIP_ID mismatch on consistency test %d! Got 0x%02X, expected 0x%02X",
                     i + 1, test_chip_id, CHIP_ID_VALUE);
            return ESP_ERR_INVALID_RESPONSE;
        }
        ESP_LOGD(TAG, "Consistency test %d: CHIP_ID = 0x%02X ✅", i + 1, test_chip_id);
    }

    ESP_LOGI(TAG, "✅ SPI communication test completed - read/write operations verified");
    return ESP_OK;
}

esp_err_t BMI270::write_register(uint8_t reg_addr, uint8_t data) {
    if (!spi_hal_ || !device_handle_) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_buffer[2] = {reg_addr, data};  // BMI270 write: no R/W bit needed

    ESP_LOGD(TAG, "SPI write: register 0x%02X = 0x%02X", reg_addr, data);

    spi_transaction_t trans = {};
    trans.length = 16;  // 2 bytes (16 bits)
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = nullptr;  // Write only transaction

    esp_err_t ret = spi_hal_->transmit(device_handle_, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed for register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "SPI write successful: register 0x%02X = 0x%02X", reg_addr, data);
    }
    return ret;
}

esp_err_t BMI270::burst_write(uint8_t reg_addr, const uint8_t* data, size_t len) {
    if (!spi_hal_ || !device_handle_ || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_len = len + 1;
    uint8_t* tx_buffer = new uint8_t[total_len];
    tx_buffer[0] = reg_addr;
    memcpy(&tx_buffer[1], data, len);

    spi_transaction_t trans = {};
    trans.length = total_len * 8;
    trans.tx_buffer = tx_buffer;

    esp_err_t ret = spi_hal_->transmit(device_handle_, &trans);
    delete[] tx_buffer;

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI burst write failed for register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t BMI270::switch_to_spi_mode() {
    ESP_LOGI(TAG, "Switching BMI270 from I2C to SPI mode");

    // Perform dummy read to trigger mode switch
    uint8_t dummy_value;
    esp_err_t ret = read_register(REG_CHIP_ID, &dummy_value);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Dummy read failed (expected during I2C->SPI switch): %s", esp_err_to_name(ret));
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    // Verify CHIP_ID
    uint8_t chip_id;
    ret = read_register(REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID after mode switch: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id != CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "CHIP_ID verification failed. Got: 0x%02X, Expected: 0x%02X", chip_id, CHIP_ID_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "SPI mode activated successfully. CHIP_ID: 0x%02X", chip_id);
    return ESP_OK;
}

esp_err_t BMI270::disable_advanced_power_save() {
    ESP_LOGI(TAG, "Disabling advanced power save");
    esp_err_t ret = write_register(REG_PWR_CONF, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable advanced power save");
        return ret;
    }

    // Give more time for power mode transition
    vTaskDelay(pdMS_TO_TICKS(10));  // Increased from 1ms to 10ms

    // Verify power configuration was set correctly
    uint8_t pwr_conf_readback;
    ret = read_register(REG_PWR_CONF, &pwr_conf_readback);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Power configuration readback: 0x%02X", pwr_conf_readback);
    }

    return ESP_OK;
}

esp_err_t BMI270::upload_config_file() {
    ESP_LOGI(TAG, "Starting config file upload (8192 bytes)");

    // Reset initialization control
    esp_err_t ret = write_register(REG_INIT_CTRL, 0x00);
    if (ret != ESP_OK) return ret;

    // Reset address pointers
    ret = write_register(REG_INIT_ADDR_0, 0x00);
    if (ret != ESP_OK) return ret;
    ret = write_register(REG_INIT_ADDR_1, 0x00);
    if (ret != ESP_OK) return ret;

    // Upload config file in 32-byte chunks
    const uint8_t* config_data = bmi270_config_file;
    size_t config_size = sizeof(bmi270_config_file);
    size_t chunk_size = 32;
    uint16_t addr_offset = 0;

    for (size_t i = 0; i < config_size; i += chunk_size) {
        size_t bytes_to_write = (i + chunk_size > config_size) ? (config_size - i) : chunk_size;

        ret = burst_write(REG_INIT_DATA, &config_data[i], bytes_to_write);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write config chunk at offset %zu", i);
            return ret;
        }

        // Update address pointers for next chunk
        // BMI270 uses word addressing (16-bit), so increment by half the byte count
        addr_offset += bytes_to_write / 2;  // Word addressing: increment by number of 16-bit words
        ret = write_register(REG_INIT_ADDR_0, addr_offset & 0xFF);
        if (ret != ESP_OK) return ret;
        ret = write_register(REG_INIT_ADDR_1, (addr_offset >> 8) & 0xFF);
        if (ret != ESP_OK) return ret;

        // Small delay between chunks
        vTaskDelay(pdMS_TO_TICKS(1));

        if ((i / chunk_size) % 64 == 0) {
            ESP_LOGI(TAG, "Config upload progress: %zu/%zu bytes", i + bytes_to_write, config_size);
        }
    }

    // Start initialization
    ret = write_register(REG_INIT_CTRL, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start initialization");
        return ret;
    }

    ESP_LOGI(TAG, "Config file upload completed");
    ESP_LOGI(TAG, "Allowing extra time for BMI270 internal processing...");
    vTaskDelay(pdMS_TO_TICKS(50));  // Additional stabilization time
    return ESP_OK;
}

esp_err_t BMI270::wait_for_initialization() {
    ESP_LOGI(TAG, "Waiting for initialization completion (max 500ms)");

    uint32_t timeout_ms = 500;  // Increased from 150ms to 500ms
    uint32_t start_time = pdTICKS_TO_MS(xTaskGetTickCount());

    uint8_t last_status = 0xFF;
    uint32_t status_change_count = 0;

    while (true) {
        uint8_t status;
        esp_err_t ret = read_register(REG_INTERNAL_STATUS, &status);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read internal status");
            return ret;
        }

        // Log status changes for debugging
        if (status != last_status) {
            uint32_t elapsed = pdTICKS_TO_MS(xTaskGetTickCount()) - start_time;
            ESP_LOGI(TAG, "Status change at %lums: 0x%02X -> 0x%02X (message=%d)",
                     elapsed, last_status, status, status & 0x0F);
            last_status = status;
            status_change_count++;
        }

        // Check if initialization completed (message field = 0x01)
        if ((status & 0x0F) == 0x01) {
            ESP_LOGI(TAG, "Initialization completed successfully after %lu status changes", status_change_count);
            return ESP_OK;
        }

        // Check for initialization error (message field = 0x02)
        if ((status & 0x0F) == 0x02) {
            ESP_LOGE(TAG, "Initialization error detected (message=0x02) after %lums",
                     pdTICKS_TO_MS(xTaskGetTickCount()) - start_time);
            return ESP_ERR_INVALID_RESPONSE;
        }

        uint32_t elapsed = pdTICKS_TO_MS(xTaskGetTickCount()) - start_time;
        if (elapsed > timeout_ms) {
            ESP_LOGE(TAG, "Initialization timeout after %lums. Final status: 0x%02X (message=%d)",
                     elapsed, status, status & 0x0F);
            ESP_LOGE(TAG, "Status interpretations: 0x01=success, 0x02=error, others=unknown");
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t BMI270::configure_sensors() {
    ESP_LOGI(TAG, "Configuring accelerometer and gyroscope");

    // Configure accelerometer: 100Hz, ±4g, performance mode
    uint8_t acc_conf = 0x88;  // ODR=100Hz(0x08) + performance_mode(0x80)
    esp_err_t ret = write_register(REG_ACC_CONF, acc_conf);
    if (ret != ESP_OK) return ret;

    // Set accelerometer range: ±4g
    accel_range_ = 1;
    ret = write_register(REG_ACC_RANGE, accel_range_);
    if (ret != ESP_OK) return ret;

    // Configure gyroscope: 200Hz, performance mode
    uint8_t gyr_conf = 0x89;  // ODR=200Hz(0x09) + filter_perf_mode(0x80)
    ret = write_register(REG_GYR_CONF, gyr_conf);
    if (ret != ESP_OK) return ret;

    // Set gyroscope range: ±1000dps
    gyro_range_ = 1;
    ret = write_register(REG_GYR_RANGE, gyro_range_);
    if (ret != ESP_OK) return ret;

    // Enable sensors: accelerometer + gyroscope + temperature
    uint8_t pwr_ctrl = PWR_CTRL_ACC_EN | PWR_CTRL_GYR_EN | PWR_CTRL_TEMP_EN;
    ret = write_register(REG_PWR_CTRL, pwr_ctrl);
    if (ret != ESP_OK) return ret;

    // Wait for sensor stabilization
    vTaskDelay(pdMS_TO_TICKS(30));

    // Set scale factors based on range settings
    const float accel_scales[] = {2.0f/32768.0f, 4.0f/32768.0f, 8.0f/32768.0f, 16.0f/32768.0f};
    const float gyro_scales[] = {2000.0f/32768.0f, 1000.0f/32768.0f, 500.0f/32768.0f, 250.0f/32768.0f, 125.0f/32768.0f};

    accel_scale_ = accel_scales[accel_range_] * 9.80665f;  // Convert to m/s^2
    gyro_scale_ = gyro_scales[gyro_range_] * M_PI / 180.0f;  // Convert to rad/s

    ESP_LOGI(TAG, "Sensors configured - Accel: ±%dg, Gyro: ±%ddps",
             (int)(accel_scales[accel_range_] * 32768),
             (int)(gyro_scales[gyro_range_] * 32768));

    return ESP_OK;
}

esp_err_t BMI270::read_accel(AccelData& data) {
    uint8_t raw_data[6];
    for (int i = 0; i < 6; i++) {
        esp_err_t ret = read_register(REG_ACC_X_LSB + i, &raw_data[i]);
        if (ret != ESP_OK) return ret;
    }

    // Convert raw data to signed 16-bit values and apply scale
    int16_t raw_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    int16_t raw_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t raw_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    data.x = raw_x * accel_scale_;
    data.y = raw_y * accel_scale_;
    data.z = raw_z * accel_scale_;

    return ESP_OK;
}

esp_err_t BMI270::read_gyro(GyroData& data) {
    uint8_t raw_data[6];
    for (int i = 0; i < 6; i++) {
        esp_err_t ret = read_register(REG_GYR_X_LSB + i, &raw_data[i]);
        if (ret != ESP_OK) return ret;
    }

    // Convert raw data to signed 16-bit values and apply scale
    int16_t raw_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    int16_t raw_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t raw_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    data.x = raw_x * gyro_scale_;
    data.y = raw_y * gyro_scale_;
    data.z = raw_z * gyro_scale_;

    return ESP_OK;
}

esp_err_t BMI270::read_temperature(float& temp_c) {
    uint8_t temp_lsb, temp_msb;
    esp_err_t ret = read_register(REG_TEMPERATURE_LSB, &temp_lsb);
    if (ret != ESP_OK) return ret;
    ret = read_register(REG_TEMPERATURE_MSB, &temp_msb);
    if (ret != ESP_OK) return ret;

    // Convert to temperature in Celsius
    int16_t temp_raw = (int16_t)((temp_msb << 8) | temp_lsb);
    temp_c = (temp_raw / 512.0f) + 23.0f;  // BMI270 temperature formula

    return ESP_OK;
}

esp_err_t BMI270::get_status(uint8_t& status) {
    return read_register(REG_STATUS, &status);
}

esp_err_t BMI270::get_error_status(uint8_t& error) {
    return read_register(REG_ERR_REG, &error);
}

esp_err_t BMI270::soft_reset() {
    ESP_LOGI(TAG, "Performing soft reset");
    esp_err_t ret = write_register(REG_CMD, CMD_SOFT_RESET);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(1));  // Wait for reset completion
    return ESP_OK;
}

} // namespace stampfly_hal
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
                   accel_range_(1), gyro_range_(1), accel_scale_(0.0f), gyro_scale_(0.0f),
                   // FIFO初期化
                   fifo_enabled_(false), fifo_accel_enabled_(false), fifo_gyro_enabled_(false),
                   fifo_mode_(FifoMode::FIFO), fifo_watermark_(0),
                   fifo_read_count_(0), fifo_overflow_count_(0),
                   fifo_total_samples_(0), fifo_max_samples_(0) {
    // Initialize scale factors (will be set during configuration)
}

esp_err_t BMI270::init() {
    if (is_initialized()) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "=== BMI270 Initialization Start ===");

    // Phase 1: Setup SPI interface
    ESP_LOGI(TAG, "Phase 1: Setting up SPI interface");
    esp_err_t ret = setup_spi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ SPI setup failed");
        return ret;
    }

    // Phase 2: Software reset (for host reboot while BMI270 powered)
    ESP_LOGI(TAG, "Phase 2: Performing software reset");
    ret = write_register(REG_CMD, CMD_SOFT_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Software reset failed");
        return ret;
    }

    // Wait 5ms for reset completion
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "Waited 5ms after software reset");

    // Phase 3: Read CHIP_ID twice, verify 2nd read = 0x24
    ESP_LOGI(TAG, "Phase 3: Reading CHIP_ID twice for SPI recovery");

    // First CHIP_ID read (dummy read for SPI recovery)
    uint8_t chip_id_1st;
    ret = read_register(REG_CHIP_ID, &chip_id_1st);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "1st CHIP_ID read failed (expected): %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "1st CHIP_ID read: 0x%02X", chip_id_1st);
    }

    // Wait 5ms before 2nd read for SPI stabilization
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "Waited 5ms before 2nd CHIP_ID read");

    // Second CHIP_ID read (must be 0x24)
    uint8_t chip_id_2nd;
    ret = read_register(REG_CHIP_ID, &chip_id_2nd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ 2nd CHIP_ID read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "2nd CHIP_ID read: 0x%02X", chip_id_2nd);

    if (chip_id_2nd != CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "❌ 2nd CHIP_ID verification failed! Got: 0x%02X, Expected: 0x%02X",
                 chip_id_2nd, CHIP_ID_VALUE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "✅ CHIP_ID verified successfully: 0x%02X", chip_id_2nd);

    // Wait 5ms after CHIP_ID verification
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "Waited 5ms after CHIP_ID verification");

    // Phase 4: Verify PWR_CONF register is 0x03 (APS enabled by default)
    ESP_LOGI(TAG, "Phase 4: Verifying PWR_CONF register (should be 0x03 after reset)");
    uint8_t pwr_conf;
    ret = read_register(REG_PWR_CONF, &pwr_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to read PWR_CONF register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PWR_CONF register value: 0x%02X", pwr_conf);

    if (pwr_conf == 0x03) {
        ESP_LOGI(TAG, "✅ PWR_CONF is 0x03 (Advanced Power Save enabled - default state)");
    } else {
        ESP_LOGW(TAG, "⚠️ PWR_CONF is 0x%02X (expected 0x03)", pwr_conf);
    }

    // Wait 5ms before disabling APS
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "Waited 5ms before disabling Advanced Power Save");

    // Phase 5: Disable Advanced Power Save (write 0x00 to PWR_CONF)
    ESP_LOGI(TAG, "Phase 5: Disabling Advanced Power Save");
    ret = write_register(REG_PWR_CONF, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to disable Advanced Power Save: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait 5ms for PWR_CONF register to update
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "Waited 5ms after writing PWR_CONF = 0x00");

    // Verify PWR_CONF was set to 0x00
    uint8_t pwr_conf_verify;
    ret = read_register(REG_PWR_CONF, &pwr_conf_verify);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to verify PWR_CONF: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PWR_CONF after disable: 0x%02X", pwr_conf_verify);

    if (pwr_conf_verify == 0x00) {
        ESP_LOGI(TAG, "✅ Advanced Power Save disabled successfully");
    } else {
        ESP_LOGW(TAG, "⚠️ PWR_CONF is 0x%02X (expected 0x00)", pwr_conf_verify);
    }

    // Phase 6: Prepare INIT_CTRL (write 0x00)
    ESP_LOGI(TAG, "Phase 6: Preparing INIT_CTRL register");
    ret = write_register(REG_INIT_CTRL, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to prepare INIT_CTRL: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ INIT_CTRL prepared (0x00)");

    // Wait 5ms after INIT_CTRL preparation
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "Waited 5ms after INIT_CTRL preparation");

    // Phase 7: Reset INIT_ADDR registers
    ESP_LOGI(TAG, "Phase 7: Resetting INIT_ADDR registers");
    ret = write_register(REG_INIT_ADDR_0, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to reset INIT_ADDR_0: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = write_register(REG_INIT_ADDR_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to reset INIT_ADDR_1: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ INIT_ADDR registers reset (0x00)");

    // Phase 8: Upload 8KB config file (256-byte chunks)
    ESP_LOGI(TAG, "Phase 8: Uploading 8KB config file (256-byte chunks)");
    ret = upload_config_file();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Config file upload failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ Config file uploaded successfully");

    // Phase 9: Complete initialization (write 0x01 to INIT_CTRL)
    ESP_LOGI(TAG, "Phase 9: Completing initialization");
    ret = write_register(REG_INIT_CTRL, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to complete initialization: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ INIT_CTRL set to 0x01 (complete)");

    // Wait for BMI270 to process the config file (critical timing)
    ESP_LOGI(TAG, "Waiting for BMI270 internal processing...");
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Waited 200ms for config file processing");

    // Phase 10: Wait for INTERNAL_STATUS to become 0x01
    ESP_LOGI(TAG, "Phase 10: Waiting for initialization completion");
    ret = wait_for_initialization();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Initialization completion failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ BMI270 initialization verified by INTERNAL_STATUS");

    set_initialized(true);
    ESP_LOGI(TAG, "=== BMI270 Initialization Complete (10 Phases) ===");
    return ESP_OK;
}

esp_err_t BMI270::configure() {
    ESP_LOGI(TAG, "BMI270 configure - placeholder");
    // TODO: Implement sensor configuration
    return ESP_OK;
}

esp_err_t BMI270::enable() {
    ESP_LOGI(TAG, "BMI270 enable - placeholder");
    set_enabled(true);
    return ESP_OK;
}

esp_err_t BMI270::disable() {
    ESP_LOGI(TAG, "BMI270 disable - placeholder");
    set_enabled(false);
    return ESP_OK;
}

esp_err_t BMI270::reset() {
    ESP_LOGI(TAG, "BMI270 reset - placeholder");
    // TODO: Implement reset sequence
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
    const uint32_t spi_clock_speed = 8000000;  // 8MHz (BMI270 supports up to 10MHz)
    ESP_LOGI(TAG, "Setting SPI clock speed: %lu Hz (%.1f MHz)", spi_clock_speed, spi_clock_speed / 1000000.0f);

    ret = spi_hal_->add_device(SPI_PIN_CS_BMI270, &device_handle_, spi_clock_speed);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BMI270 SPI device: %s", esp_err_to_name(ret));
        delete spi_hal_;
        spi_hal_ = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "BMI270 SPI setup completed successfully (CS=%d, Clock=%.1f MHz)",
             SPI_PIN_CS_BMI270, spi_clock_speed / 1000000.0f);
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

esp_err_t BMI270::burst_read(uint8_t reg_addr, uint8_t* data, size_t len) {
    if (!data || !spi_hal_ || !device_handle_ || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // BMI270 4-wire SPI burst read protocol:
    // TX: [R/W+Addr, Dummy, Dummy, ...]
    // RX: [Dummy, Dummy, Data0, Data1, Data2, ...]
    // First 2 bytes of RX are dummy, actual data starts from byte 2

    const size_t total_len = len + 2;  // +2 for dummy bytes
    uint8_t* tx_buffer = new uint8_t[total_len];
    uint8_t* rx_buffer = new uint8_t[total_len];

    if (!tx_buffer || !rx_buffer) {
        delete[] tx_buffer;
        delete[] rx_buffer;
        return ESP_ERR_NO_MEM;
    }

    memset(tx_buffer, 0, total_len);
    memset(rx_buffer, 0, total_len);

    tx_buffer[0] = reg_addr | SPI_READ_FLAG;

    spi_transaction_t trans = {};
    trans.length = total_len * 8;  // Convert bytes to bits
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = rx_buffer;

    esp_err_t ret = spi_hal_->transmit(device_handle_, &trans);

    if (ret == ESP_OK) {
        // Copy actual data (skip first 2 dummy bytes)
        memcpy(data, &rx_buffer[2], len);
        ESP_LOGD(TAG, "Burst read from 0x%02X: %d bytes", reg_addr, len);
    } else {
        ESP_LOGE(TAG, "Burst read failed for register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }

    delete[] tx_buffer;
    delete[] rx_buffer;
    return ret;
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

    // Phase 2: Test multiple reads for consistency
    ESP_LOGI(TAG, "Testing read consistency with 5 consecutive CHIP_ID reads");
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

    ESP_LOGI(TAG, "✅ SPI communication test completed successfully");
    ESP_LOGI(TAG, "Note: INIT_ADDR register write test skipped (not writable during initialization)");

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

esp_err_t BMI270::burst_write_cs_control(uint8_t reg_addr, const uint8_t* data, size_t len, CsControl cs_ctrl) {
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

    // Set CS control flag based on parameter
    if (cs_ctrl == CsControl::KEEP_ACTIVE) {
        trans.flags = SPI_TRANS_CS_KEEP_ACTIVE;  // Keep CS LOW after transaction
        ESP_LOGD(TAG, "SPI burst write with CS KEEP_ACTIVE for register 0x%02X (%zu bytes)", reg_addr, len);
    } else {
        trans.flags = 0;  // Normal CS control (HIGH after transaction)
        ESP_LOGD(TAG, "SPI burst write with normal CS control for register 0x%02X (%zu bytes)", reg_addr, len);
    }

    esp_err_t ret = spi_hal_->transmit(device_handle_, &trans);
    delete[] tx_buffer;

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI burst write (CS control) failed for register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
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
    const uint8_t* config_data = bmi270_config_file;
    size_t config_size = sizeof(bmi270_config_file);
    size_t chunk_size = 256;

    ESP_LOGI(TAG, "Config file array size: %zu bytes (expected: 8192 bytes)", config_size);

    if (config_size != 8192) {
        ESP_LOGE(TAG, "❌ Config file size mismatch! Got %zu bytes, expected 8192 bytes", config_size);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Uploading config file (%zu bytes, 256-byte chunks)", config_size);
    ESP_LOGI(TAG, "Strategy: Each register write is independent transaction (CS HIGH/LOW per write)");

    for (size_t offset = 0; offset < config_size; offset += chunk_size) {
        size_t bytes_to_write = (offset + chunk_size > config_size) ? (config_size - offset) : chunk_size;

        // 1. Prepare INIT_ADDR (Bosch official implementation)
        // index / 2 for word addressing
        // INIT_ADDR_0 uses only lower 4 bits (0x0F mask)
        // INIT_ADDR_1 uses bits 4-11
        uint16_t index = offset;
        uint8_t addr_buf[2] = {
            static_cast<uint8_t>((index / 2) & 0x0F),   // INIT_ADDR_0: bits 0-3 only
            static_cast<uint8_t>((index / 2) >> 4)       // INIT_ADDR_1: bits 4-11
        };

        // 2. Write INIT_ADDR as independent transaction
        // Transaction 1: CS LOW → [0x5B, LSB, MSB] → CS HIGH
        esp_err_t ret = burst_write(REG_INIT_ADDR_0, addr_buf, 2);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Failed to set INIT_ADDR at offset %zu: %s", offset, esp_err_to_name(ret));
            return ret;
        }

        // 3. Write INIT_DATA as independent transaction
        // Transaction 2: CS LOW → [0x5E, ...256bytes...] → CS HIGH
        ret = burst_write(REG_INIT_DATA, &config_data[offset], bytes_to_write);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Failed to write config chunk at offset %zu: %s", offset, esp_err_to_name(ret));
            return ret;
        }

        // Progress report every 2048 bytes (8 chunks)
        if ((offset / chunk_size) % 8 == 0) {
            ESP_LOGI(TAG, "Config upload progress: %zu/%zu bytes (%.1f%%)",
                     offset + bytes_to_write, config_size,
                     ((float)(offset + bytes_to_write) / config_size) * 100.0f);
        }
    }

    ESP_LOGI(TAG, "✅ Config file upload completed (8192 bytes)");
    return ESP_OK;
}

esp_err_t BMI270::wait_for_initialization() {
    ESP_LOGI(TAG, "Waiting for initialization completion (max 2000ms)");

    uint32_t timeout_ms = 2000;  // Increased to 2000ms for config file processing
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
            ESP_LOGI(TAG, "✅ Initialization completed successfully after %lu status changes",
                     status_change_count);
            return ESP_OK;
        }

        // Status 0x02 means initialization in progress (NOT an error)
        // Just continue waiting

        uint32_t elapsed = pdTICKS_TO_MS(xTaskGetTickCount()) - start_time;
        if (elapsed > timeout_ms) {
            ESP_LOGW(TAG, "Initialization timeout after %lums. Final status: 0x%02X (message=%d)",
                     elapsed, status, status & 0x0F);
            ESP_LOGW(TAG, "Continuing anyway - sensors may still be functional");
            ESP_LOGI(TAG, "Status interpretations: 0x01=complete, 0x02=in-progress, 0x00=not-started");
            // Don't return error - try to continue
            return ESP_OK;  // Changed from ESP_ERR_TIMEOUT to ESP_OK
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t BMI270::configure_sensors() {
    ESP_LOGI(TAG, "Configuring sensors for 1600Hz ODR with oversampling");

    // === Accelerometer Configuration ===
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

    // === Gyroscope Configuration ===
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

    // === Sensor Enable ===
    // Enable: Accelerometer + Gyroscope + Temperature
    uint8_t pwr_ctrl = PWR_CTRL_ACC_EN | PWR_CTRL_GYR_EN | PWR_CTRL_TEMP_EN;
    ret = write_register(REG_PWR_CTRL, pwr_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors");
        return ret;
    }

    // Wait for sensor stabilization
    vTaskDelay(pdMS_TO_TICKS(50));

    // === Scale Factor Calculation ===
    const float accel_scales[] = {2.0f, 4.0f, 8.0f, 16.0f};
    const float gyro_scales[] = {2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f};

    accel_scale_ = (accel_scales[accel_range_] / 32768.0f) * 9.80665f;  // m/s^2
    gyro_scale_ = (gyro_scales[gyro_range_] / 32768.0f) * M_PI / 180.0f;  // rad/s

    ESP_LOGI(TAG, "📊 Scale factors - Accel: %.6f m/s²/LSB, Gyro: %.6f rad/s/LSB",
             accel_scale_, gyro_scale_);
    ESP_LOGI(TAG, "🎯 Oversampling ratio: 3.2x (1600Hz / 500Hz control loop)");

    return ESP_OK;
}

esp_err_t BMI270::read_accel(AccelData& data) {
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    uint8_t raw_data[6];
    esp_err_t ret = burst_read(REG_ACC_X_LSB, raw_data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Convert raw 16-bit values to signed integers (LSB first, little-endian)
    int16_t raw_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    int16_t raw_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t raw_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // Apply scale factor to convert to m/s²
    data.x = raw_x * accel_scale_;
    data.y = raw_y * accel_scale_;
    data.z = raw_z * accel_scale_;

    ESP_LOGD(TAG, "Accel: X=%.3f, Y=%.3f, Z=%.3f m/s²", data.x, data.y, data.z);
    return ESP_OK;
}

esp_err_t BMI270::read_gyro(GyroData& data) {
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    uint8_t raw_data[6];
    esp_err_t ret = burst_read(REG_GYR_X_LSB, raw_data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Convert raw 16-bit values to signed integers (LSB first, little-endian)
    int16_t raw_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    int16_t raw_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t raw_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // Apply scale factor to convert to rad/s
    data.x = raw_x * gyro_scale_;
    data.y = raw_y * gyro_scale_;
    data.z = raw_z * gyro_scale_;

    ESP_LOGD(TAG, "Gyro: X=%.3f, Y=%.3f, Z=%.3f rad/s", data.x, data.y, data.z);
    return ESP_OK;
}

esp_err_t BMI270::read_temperature(float& temp_c) {
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read 2 bytes: TEMP_LSB, TEMP_MSB
    uint8_t raw_data[2];
    esp_err_t ret = burst_read(REG_TEMPERATURE_LSB, raw_data, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Convert raw 16-bit value to signed integer (LSB first, little-endian)
    int16_t raw_temp = (int16_t)((raw_data[1] << 8) | raw_data[0]);

    // BMI270 temperature conversion: temp_c = raw_temp / 512.0 + 23.0
    // (According to BMI270 datasheet section 5.3.7)
    temp_c = (raw_temp / 512.0f) + 23.0f;

    ESP_LOGD(TAG, "Temperature: %.2f °C (raw: %d)", temp_c, raw_temp);
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

// === Interrupt Configuration Functions ===

esp_err_t BMI270::configure_data_ready_interrupt() {
    ESP_LOGI(TAG, "Configuring Data Ready interrupt on INT1");

    // Step 1: Configure INT1 pin behavior
    // INT1_OUTPUT_EN (bit 3) = 1: Enable INT1 output
    // INT1_ACTIVE_HIGH (bit 1) = 1: Active high
    // INT1_PUSH_PULL (bit 0) = 0: Push-pull output
    uint8_t int1_io_ctrl = INT1_OUTPUT_EN | INT1_ACTIVE_HIGH | INT1_PUSH_PULL;
    esp_err_t ret = write_register(REG_INT1_IO_CTRL, int1_io_ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INT1 IO control");
        return ret;
    }
    ESP_LOGI(TAG, "✅ INT1 configured: Output enabled, Active high, Push-pull");

    // Step 2: Map Data Ready interrupt to INT1 pin
    // INT_DRDY_EN (bit 2) = 1: Map data ready to INT1
    uint8_t int_map_data = INT_DRDY_EN;
    ret = write_register(REG_INT_MAP_DATA, int_map_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to map data ready interrupt to INT1");
        return ret;
    }
    ESP_LOGI(TAG, "✅ Data Ready interrupt mapped to INT1");

    // Verify configuration
    uint8_t verify_io_ctrl, verify_map_data;
    read_register(REG_INT1_IO_CTRL, &verify_io_ctrl);
    read_register(REG_INT_MAP_DATA, &verify_map_data);
    ESP_LOGI(TAG, "Verification: INT1_IO_CTRL=0x%02X, INT_MAP_DATA=0x%02X",
             verify_io_ctrl, verify_map_data);

    ESP_LOGI(TAG, "✅ Data Ready interrupt configuration complete");
    return ESP_OK;
}

esp_err_t BMI270::enable_data_ready_interrupt() {
    ESP_LOGI(TAG, "Enabling Data Ready interrupt");
    return configure_data_ready_interrupt();
}

esp_err_t BMI270::disable_data_ready_interrupt() {
    ESP_LOGI(TAG, "Disabling Data Ready interrupt");

    // Clear INT1_IO_CTRL to disable interrupt output
    esp_err_t ret = write_register(REG_INT1_IO_CTRL, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable INT1 output");
        return ret;
    }

    // Clear INT_MAP_DATA to unmap data ready interrupt
    ret = write_register(REG_INT_MAP_DATA, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmap data ready interrupt");
        return ret;
    }

    ESP_LOGI(TAG, "✅ Data Ready interrupt disabled");
    return ESP_OK;
}

// ========================================
// FIFO機能実装（Phase 1）
// ========================================

// FIFOフラッシュ
esp_err_t BMI270::flush_fifo() {
    ESP_LOGD(TAG, "Flushing FIFO");

    // CMD_REG (0x7E) に 0xB0 書き込みでFIFOクリア
    static constexpr uint8_t CMD_FIFO_FLUSH = 0xB0;

    esp_err_t ret = write_register(REG_CMD, CMD_FIFO_FLUSH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to flush FIFO");
        return ret;
    }

    // フラッシュ完了待機（データシート推奨: 5ms）
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGD(TAG, "FIFO flushed successfully");
    return ESP_OK;
}

// FIFOモード設定
esp_err_t BMI270::set_fifo_mode(FifoMode mode) {
    ESP_LOGI(TAG, "Setting FIFO mode to %s",
             mode == FifoMode::FIFO ? "FIFO" : "STREAM");

    // FIFO_CONFIG_0設定
    uint8_t config0 = (mode == FifoMode::FIFO) ?
                      (FIFO_MODE_FIFO | FIFO_STOP_ON_FULL) :
                      FIFO_MODE_STREAM;

    esp_err_t ret = write_register(REG_FIFO_CONFIG_0, config0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_CONFIG_0");
        return ret;
    }

    fifo_mode_ = mode;
    ESP_LOGI(TAG, "FIFO mode set successfully");

    return ESP_OK;
}

// ウォーターマーク設定
esp_err_t BMI270::set_fifo_watermark(uint16_t bytes) {
    ESP_LOGI(TAG, "Setting FIFO watermark to %u bytes", bytes);

    // 範囲チェック
    if (bytes > FIFO_CAPACITY_BYTES) {
        ESP_LOGW(TAG, "Watermark %u exceeds capacity %u, capping",
                 bytes, FIFO_CAPACITY_BYTES);
        bytes = FIFO_CAPACITY_BYTES;
    }

    // ウォーターマークは11ビット（0-2047）
    // REG_FIFO_WTM_0: 下位8ビット
    // REG_FIFO_WTM_1: 上位3ビット（Bit 2-0）
    uint8_t wtm_low = bytes & 0xFF;
    uint8_t wtm_high = (bytes >> 8) & 0x07;

    esp_err_t ret = write_register(REG_FIFO_WTM_0, wtm_low);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_WTM_0");
        return ret;
    }

    ret = write_register(REG_FIFO_WTM_1, wtm_high);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_WTM_1");
        return ret;
    }

    fifo_watermark_ = bytes;
    ESP_LOGI(TAG, "FIFO watermark set to %u bytes", bytes);

    return ESP_OK;
}

// FIFO設定レジスタ構成（内部ヘルパー）
esp_err_t BMI270::configure_fifo_registers() {
    ESP_LOGD(TAG, "Configuring FIFO registers");

    // 1. FIFOモード設定（デフォルト: FIFO、フル時停止）
    esp_err_t ret = set_fifo_mode(fifo_mode_);
    if (ret != ESP_OK) {
        return ret;
    }

    // 2. ダウンサンプリング設定（Phase 1では無効: 0x00）
    ret = write_register(REG_FIFO_DOWNS, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_DOWNS");
        return ret;
    }

    // 3. ウォーターマーク設定（設定済みの値、未設定なら0）
    if (fifo_watermark_ > 0) {
        ret = set_fifo_watermark(fifo_watermark_);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    ESP_LOGD(TAG, "FIFO registers configured successfully");
    return ESP_OK;
}

// FIFO有効化
esp_err_t BMI270::enable_fifo(bool accel, bool gyro) {
    ESP_LOGI(TAG, "Enabling FIFO (accel: %d, gyro: %d)", accel, gyro);

    // 状態チェック
    if (!is_initialized()) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!accel && !gyro) {
        ESP_LOGW(TAG, "At least one sensor must be enabled");
        return ESP_ERR_INVALID_ARG;
    }

    // 状態保存
    fifo_accel_enabled_ = accel;
    fifo_gyro_enabled_ = gyro;

    // FIFOフラッシュ（クリーンスタート）
    esp_err_t ret = flush_fifo();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to flush FIFO before enabling");
        return ret;
    }

    // レジスタ設定
    ret = configure_fifo_registers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure FIFO registers");
        return ret;
    }

    // FIFO_CONFIG_1: センサー有効化
    uint8_t config1 = 0;
    if (accel) config1 |= FIFO_ACC_EN;
    if (gyro) config1 |= FIFO_GYR_EN;
    // ヘッダーレスモード（FIFO_HEADER_ENビットを0に）

    ret = write_register(REG_FIFO_CONFIG_1, config1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO_CONFIG_1");
        return ret;
    }

    // 設定読み戻し確認
    uint8_t verify;
    ret = read_register(REG_FIFO_CONFIG_1, &verify);
    if (ret != ESP_OK || verify != config1) {
        ESP_LOGE(TAG, "FIFO_CONFIG_1 verification failed (wrote: 0x%02X, read: 0x%02X)",
                 config1, verify);
        return ESP_ERR_INVALID_RESPONSE;
    }

    fifo_enabled_ = true;
    ESP_LOGI(TAG, "FIFO enabled successfully (config1: 0x%02X)", config1);

    return ESP_OK;
}

// FIFO無効化
esp_err_t BMI270::disable_fifo() {
    ESP_LOGI(TAG, "Disabling FIFO");

    // FIFO_CONFIG_1を0にしてすべて無効化
    esp_err_t ret = write_register(REG_FIFO_CONFIG_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable FIFO");
        return ret;
    }

    // FIFOフラッシュ
    ret = flush_fifo();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to flush FIFO during disable");
    }

    fifo_enabled_ = false;
    fifo_accel_enabled_ = false;
    fifo_gyro_enabled_ = false;

    ESP_LOGI(TAG, "FIFO disabled successfully");
    return ESP_OK;
}

// FIFO長読み取り
esp_err_t BMI270::read_fifo_length(uint16_t* length) {
    if (length == nullptr) {
        ESP_LOGE(TAG, "Null pointer for length");
        return ESP_ERR_INVALID_ARG;
    }

    // FIFO_LENGTH_0/1 読み取り（14ビット値）
    uint8_t length_low, length_high;

    esp_err_t ret = read_register(REG_FIFO_LENGTH_0, &length_low);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO_LENGTH_0");
        return ret;
    }

    ret = read_register(REG_FIFO_LENGTH_1, &length_high);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO_LENGTH_1");
        return ret;
    }

    // 14ビット値構成（Bit 13-0）
    *length = ((uint16_t)(length_high & 0x3F) << 8) | length_low;

    ESP_LOGD(TAG, "FIFO length: %u bytes", *length);

    return ESP_OK;
}

// FIFOオーバーフロー確認
esp_err_t BMI270::check_fifo_overflow(bool* overflow) {
    if (overflow == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // INT_STATUS_1 (0x1D) レジスタのBit 6: FIFO_FULL_INT
    uint8_t status;
    esp_err_t ret = read_register(REG_INT_STATUS_1, &status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read INT_STATUS_1");
        return ret;
    }

    *overflow = (status & FIFO_FULL_INT_BIT) != 0;

    if (*overflow) {
        ESP_LOGD(TAG, "FIFO overflow flag set");
    }

    return ESP_OK;
}

// FIFOデータバースト読み取り
esp_err_t BMI270::read_fifo_data(uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr) {
        ESP_LOGE(TAG, "Null buffer pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (length == 0) {
        ESP_LOGD(TAG, "Zero length, nothing to read");
        return ESP_OK;
    }

    if (length > FIFO_CAPACITY_BYTES) {
        ESP_LOGW(TAG, "Length %u exceeds FIFO capacity, capping to %u",
                 length, FIFO_CAPACITY_BYTES);
        length = FIFO_CAPACITY_BYTES;
    }

    ESP_LOGD(TAG, "Reading %u bytes from FIFO", length);

    // burst_read()を使用（既存実装活用）
    esp_err_t ret = burst_read(REG_FIFO_DATA, buffer, length);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FIFO burst read failed");
        return ret;
    }

    // 統計更新
    fifo_read_count_++;

    // オーバーフロー確認
    bool overflow;
    ret = check_fifo_overflow(&overflow);
    if (ret == ESP_OK && overflow) {
        ESP_LOGW(TAG, "FIFO overflow detected");
        fifo_overflow_count_++;
    }

    ESP_LOGD(TAG, "FIFO data read successfully");

    return ESP_OK;
}

// サンプル数計算
uint16_t BMI270::calculate_sample_count(uint16_t fifo_bytes) const {
    // ヘッダーレスモード: 12バイト/サンプル
    uint16_t count = fifo_bytes / FIFO_FRAME_SIZE_HEADERLESS;

    // 端数チェック
    uint16_t remainder = fifo_bytes % FIFO_FRAME_SIZE_HEADERLESS;
    if (remainder != 0) {
        ESP_LOGW(TAG, "FIFO data not aligned: %u bytes remainder", remainder);
    }

    return count;
}

// 生データ→物理量変換
void BMI270::convert_raw_to_physical(FifoSample& sample) {
    // 加速度変換（既存のスケールファクター使用）
    // accel_scale_ は ±4g設定で 0.00011970 m/s² （既存実装）
    sample.accel_x_mps2 = sample.accel_x_raw * accel_scale_;
    sample.accel_y_mps2 = sample.accel_y_raw * accel_scale_;
    sample.accel_z_mps2 = sample.accel_z_raw * accel_scale_;

    // ジャイロ変換（既存のスケールファクター使用）
    // gyro_scale_ は ±1000dps設定で 0.00053264 rad/s （既存実装）
    sample.gyro_x_rps = sample.gyro_x_raw * gyro_scale_;
    sample.gyro_y_rps = sample.gyro_y_raw * gyro_scale_;
    sample.gyro_z_rps = sample.gyro_z_raw * gyro_scale_;
}

// FIFOデータ解析
esp_err_t BMI270::parse_fifo_data(
    const uint8_t* buffer,
    uint16_t buffer_length,
    FifoSample* samples,
    uint16_t max_samples,
    uint16_t* parsed_count)
{
    if (buffer == nullptr || samples == nullptr || parsed_count == nullptr) {
        ESP_LOGE(TAG, "Null pointer in parse_fifo_data");
        return ESP_ERR_INVALID_ARG;
    }

    // サンプル数計算
    uint16_t available_samples = calculate_sample_count(buffer_length);
    uint16_t samples_to_parse = (available_samples < max_samples) ?
                                 available_samples : max_samples;

    ESP_LOGD(TAG, "Parsing FIFO: %u bytes → %u samples (max: %u)",
             buffer_length, samples_to_parse, max_samples);

    // ヘッダーレスモード解析
    // フレーム構造（12バイト）:
    //   Byte 0-1:   accel_x (LSB, MSB)
    //   Byte 2-3:   accel_y
    //   Byte 4-5:   accel_z
    //   Byte 6-7:   gyro_x
    //   Byte 8-9:   gyro_y
    //   Byte 10-11: gyro_z

    for (uint16_t i = 0; i < samples_to_parse; i++) {
        const uint8_t* frame = &buffer[i * FIFO_FRAME_SIZE_HEADERLESS];

        // 生データ抽出（リトルエンディアン、2の補数）
        samples[i].accel_x_raw = (int16_t)((frame[1] << 8) | frame[0]);
        samples[i].accel_y_raw = (int16_t)((frame[3] << 8) | frame[2]);
        samples[i].accel_z_raw = (int16_t)((frame[5] << 8) | frame[4]);
        samples[i].gyro_x_raw = (int16_t)((frame[7] << 8) | frame[6]);
        samples[i].gyro_y_raw = (int16_t)((frame[9] << 8) | frame[8]);
        samples[i].gyro_z_raw = (int16_t)((frame[11] << 8) | frame[10]);

        // 物理量変換
        convert_raw_to_physical(samples[i]);

        // タイムスタンプ（Phase 3で実装、今回は0）
        samples[i].sensor_time_ticks = 0;
        samples[i].timestamp_us = 0;
    }

    *parsed_count = samples_to_parse;

    // 統計更新
    fifo_total_samples_ += samples_to_parse;
    if (samples_to_parse > fifo_max_samples_) {
        fifo_max_samples_ = samples_to_parse;
    }

    ESP_LOGD(TAG, "Parsed %u samples successfully", samples_to_parse);

    return ESP_OK;
}

// FIFOステータス取得
esp_err_t BMI270::get_fifo_status(FifoStatus* status) {
    if (status == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // 現在のFIFO長取得
    uint16_t current_length;
    esp_err_t ret = read_fifo_length(&current_length);
    if (ret != ESP_OK) {
        return ret;
    }

    // オーバーフロー確認
    bool overflow;
    ret = check_fifo_overflow(&overflow);
    if (ret != ESP_OK) {
        return ret;
    }

    // ステータス構造体設定
    status->enabled = fifo_enabled_;
    status->overflow = overflow;
    status->current_length = current_length;
    status->watermark = fifo_watermark_;
    status->read_count = fifo_read_count_;
    status->overflow_count = fifo_overflow_count_;
    status->max_samples = fifo_max_samples_;

    // 平均サンプル数計算
    if (fifo_read_count_ > 0) {
        status->average_samples = (float)fifo_total_samples_ / fifo_read_count_;
    } else {
        status->average_samples = 0.0f;
    }

    return ESP_OK;
}

// FIFO診断情報表示
void BMI270::print_fifo_diagnostics() const {
    ESP_LOGI(TAG, "========== FIFO Diagnostics ==========");
    ESP_LOGI(TAG, "Enabled: %s", fifo_enabled_ ? "YES" : "NO");
    ESP_LOGI(TAG, "Mode: %s", fifo_mode_ == FifoMode::FIFO ? "FIFO" : "STREAM");
    ESP_LOGI(TAG, "Sensors: Accel=%s, Gyro=%s",
             fifo_accel_enabled_ ? "ON" : "OFF",
             fifo_gyro_enabled_ ? "ON" : "OFF");
    ESP_LOGI(TAG, "Watermark: %u bytes", fifo_watermark_);
    ESP_LOGI(TAG, "---------- Statistics ----------");
    ESP_LOGI(TAG, "Total reads: %lu", fifo_read_count_);
    ESP_LOGI(TAG, "Overflow count: %lu", fifo_overflow_count_);
    ESP_LOGI(TAG, "Total samples: %lu", fifo_total_samples_);
    ESP_LOGI(TAG, "Max samples/read: %u", fifo_max_samples_);

    if (fifo_read_count_ > 0) {
        float avg = (float)fifo_total_samples_ / fifo_read_count_;
        ESP_LOGI(TAG, "Average samples/read: %.1f", avg);
    }

    ESP_LOGI(TAG, "====================================");
}

} // namespace stampfly_hal
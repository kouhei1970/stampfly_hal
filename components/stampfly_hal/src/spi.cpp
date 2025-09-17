#include "spi_hal.h"
#include <esp_log.h>
#include <driver/gpio.h>

namespace stampfly_hal {

static const char* TAG = "SpiHal";

SpiHal::SpiHal() : HALBase("SpiHal") {
}

SpiHal::~SpiHal() {
    if (bus_initialized_) {
        spi_bus_free(config_.host);
    }
}

esp_err_t SpiHal::init() {
    if (is_initialized()) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SPI HAL");

    esp_err_t ret = init_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    set_initialized(true);
    ESP_LOGI(TAG, "SPI HAL initialized successfully");
    return ESP_OK;
}

esp_err_t SpiHal::configure() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    // Initialize PMW3901 CS pin to HIGH to prevent interference
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << SPI_PIN_CS_PMW3901);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level(SPI_PIN_CS_PMW3901, 1);  // Keep PMW3901 deselected

    ESP_LOGI(TAG, "SPI HAL configured - PMW3901 CS initialized to HIGH");
    return ESP_OK;
}

esp_err_t SpiHal::enable() {
    if (!is_initialized()) {
        return set_error(ESP_ERR_INVALID_STATE);
    }

    set_enabled(true);
    ESP_LOGI(TAG, "SPI HAL enabled");
    return ESP_OK;
}

esp_err_t SpiHal::disable() {
    set_enabled(false);
    ESP_LOGI(TAG, "SPI HAL disabled");
    return ESP_OK;
}

esp_err_t SpiHal::reset() {
    ESP_LOGI(TAG, "SPI HAL reset");
    return ESP_OK;
}

esp_err_t SpiHal::init_bus() {
    ESP_LOGI(TAG, "Initializing SPI bus with pins: MISO=%d, MOSI=%d, SCLK=%d",
             SPI_PIN_MISO, SPI_PIN_MOSI, SPI_PIN_SCLK);

    spi_bus_config_t bus_config = {};
    bus_config.miso_io_num = SPI_PIN_MISO;
    bus_config.mosi_io_num = SPI_PIN_MOSI;
    bus_config.sclk_io_num = SPI_PIN_SCLK;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 4096;

    ESP_LOGI(TAG, "SPI bus config: host=%d, max_transfer_sz=%d", config_.host, bus_config.max_transfer_sz);

    esp_err_t ret = spi_bus_initialize(config_.host, &bus_config, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized");
        bus_initialized_ = true;
        return ESP_OK;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    bus_initialized_ = true;
    ESP_LOGI(TAG, "SPI bus initialized successfully: MISO=%d, MOSI=%d, SCLK=%d",
             SPI_PIN_MISO, SPI_PIN_MOSI, SPI_PIN_SCLK);
    return ESP_OK;
}

esp_err_t SpiHal::add_device(gpio_num_t cs_pin, spi_device_handle_t* device_handle, int clock_speed_hz) {
    if (!device_handle || !is_initialized()) {
        ESP_LOGE(TAG, "Invalid arguments or SPI not initialized");
        return set_error(ESP_ERR_INVALID_ARG);
    }

    ESP_LOGI(TAG, "Adding SPI device: CS=%d, clock=%d Hz, mode=%d", cs_pin, clock_speed_hz, config_.mode);

    spi_device_interface_config_t dev_config = {};
    dev_config.command_bits = 0;
    dev_config.address_bits = 0;
    dev_config.dummy_bits = 0;
    dev_config.mode = config_.mode;
    dev_config.duty_cycle_pos = 128;  // 50% duty cycle
    dev_config.cs_ena_pretrans = 1;
    dev_config.cs_ena_posttrans = 1;
    dev_config.clock_speed_hz = clock_speed_hz;
    dev_config.spics_io_num = cs_pin;
    dev_config.queue_size = config_.queue_size;

    ESP_LOGI(TAG, "Device config: pretrans=%d, posttrans=%d, queue_size=%d",
             dev_config.cs_ena_pretrans, dev_config.cs_ena_posttrans, dev_config.queue_size);

    esp_err_t ret = spi_bus_add_device(config_.host, &dev_config, device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    ESP_LOGI(TAG, "SPI device added successfully (CS=%d, clock=%d Hz)", cs_pin, clock_speed_hz);
    return ESP_OK;
}

esp_err_t SpiHal::remove_device(spi_device_handle_t device_handle) {
    if (!device_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = spi_bus_remove_device(device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI device removed successfully");
    return ESP_OK;
}

esp_err_t SpiHal::transmit(spi_device_handle_t device_handle, spi_transaction_t* transaction) {
    if (!device_handle || !transaction || !is_enabled()) {
        return set_error(ESP_ERR_INVALID_ARG);
    }

    esp_err_t ret = spi_device_polling_transmit(device_handle, transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmission failed: %s", esp_err_to_name(ret));
        return set_error(ret);
    }

    return ESP_OK;
}

} // namespace stampfly_hal
#pragma once

#include "hal_base.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>

namespace stampfly_hal {

// StampFly SPI pin definitions
constexpr gpio_num_t SPI_PIN_MISO = GPIO_NUM_43;
constexpr gpio_num_t SPI_PIN_MOSI = GPIO_NUM_14;
constexpr gpio_num_t SPI_PIN_SCLK = GPIO_NUM_44;
constexpr gpio_num_t SPI_PIN_CS_BMI270 = GPIO_NUM_46;
constexpr gpio_num_t SPI_PIN_CS_PMW3901 = GPIO_NUM_12;

struct SPIConfig {
    spi_host_device_t host = SPI2_HOST;
    int clock_speed_hz = 1000000;  // 1MHz for safe initialization
    int mode = 0;  // SPI mode 0
    int queue_size = 3;
};

class SpiHal : public HALBase {
public:
    SpiHal();
    ~SpiHal();

    // HALBase interface
    esp_err_t init() override;
    esp_err_t configure() override;
    esp_err_t enable() override;
    esp_err_t disable() override;
    esp_err_t reset() override;

    // SPI device management
    esp_err_t add_device(gpio_num_t cs_pin, spi_device_handle_t* device_handle, int clock_speed_hz = 1000000);
    esp_err_t remove_device(spi_device_handle_t device_handle);

    // SPI transaction
    esp_err_t transmit(spi_device_handle_t device_handle, spi_transaction_t* transaction);

private:
    SPIConfig config_;
    bool bus_initialized_ = false;

    esp_err_t init_bus();
};

} // namespace stampfly_hal
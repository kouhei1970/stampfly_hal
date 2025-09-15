#include "stampfly_memory.h"
#include <esp_log.h>
#include <esp_heap_caps.h>

namespace stampfly_hal {

static const char* TAG = "MEMORY";

void Memory::print_heap_info() {
    ESP_LOGI(TAG, "Free heap: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    ESP_LOGI(TAG, "Minimum free heap: %zu bytes", heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
    ESP_LOGI(TAG, "Largest free block: %zu bytes", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
}

size_t Memory::get_free_heap() {
    return heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
}

size_t Memory::get_minimum_free_heap() {
    return heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
}

size_t Memory::get_largest_free_block() {
    return heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
}

} // namespace stampfly_hal
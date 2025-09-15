#pragma once

#include <esp_err.h>
#include <cstddef>

namespace stampfly_hal {

/**
 * @brief メモリユーティリティクラス
 * ヒープメモリ管理とデバッグ機能を提供
 */
class Memory {
public:
    static void print_heap_info();
    static size_t get_free_heap();
    static size_t get_minimum_free_heap();
    static size_t get_largest_free_block();
};

} // namespace stampfly_hal
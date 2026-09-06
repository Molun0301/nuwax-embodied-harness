#pragma once

#include <charconv>
#include <cmath>
#include <stdexcept>
#include <string>
#include <system_error>

namespace nuwax_lio {

inline void validate_initial_map_frames(int value) {
    if (value < 1) {
        throw std::invalid_argument("initial_map_frames 必须为 1..INT_MAX 的正整数");
    }
}

inline int parse_initial_map_frames(const std::string& text) {
    int value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value, 10);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value < 1) {
        throw std::invalid_argument("initial_map_frames 必须为 1..INT_MAX 的完整十进制正整数");
    }
    return value;
}

inline void validate_native_voxel_size(double value, const char* name) {
    const float native_value = static_cast<float>(value);
    if (!std::isfinite(value) || value <= 0.0
        || !std::isnormal(native_value) || native_value <= 0.0f) {
        throw std::invalid_argument(std::string(name)
            + " 必须在转换为 float32 后仍为正的有限正规数");
    }
}

inline double parse_native_voxel_size(const std::string& text, const char* name) {
    float value = 0.0f;
    std::size_t consumed = 0;
    try {
        value = std::stof(text, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string(name) + " 不是有效的 float32 体素边长");
    }
    if (consumed != text.size()) {
        throw std::invalid_argument(std::string(name) + " 必须是完整数值，不允许尾随字符");
    }
    validate_native_voxel_size(value, name);
    // 保留旧 arg_float 的 float32 舍入，再赋给内核的 double 字段。
    return static_cast<double>(value);
}

}  // namespace nuwax_lio

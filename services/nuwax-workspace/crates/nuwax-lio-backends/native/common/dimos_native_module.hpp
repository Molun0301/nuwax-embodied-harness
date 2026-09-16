// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Lightweight header-only helper for dimos NativeModule C++ binaries.
// Parses --<port_name> <topic> CLI args passed by the Python NativeModule wrapper.

#pragma once

#include <atomic>
#include <map>
#include <set>
#include <cmath>
#include <stdexcept>
#include <string>

#include "std_msgs/Header.hpp"
#include "std_msgs/Time.hpp"

namespace dimos {

class NativeModule {
public:
    NativeModule(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-' && i + 1 < argc) {
                args_[arg.substr(2)] = argv[++i];
            } else {
                invalid_argument_ = arg;
            }
        }
    }

    /// Get the full LCM channel string for a declared port.
    /// Format is "<topic>#<msg_type>", e.g. "/pointcloud#sensor_msgs.PointCloud2".
    /// This is the exact channel name used by Python LCMTransport subscribers.
    const std::string& topic(const std::string& port) const {
        consumed_.insert(port);
        auto it = args_.find(port);
        if (it == args_.end()) {
            throw std::runtime_error("NativeModule: no topic for port '" + port + "'");
        }
        return it->second;
    }

    /// Get a string arg value, or a default if not present.
    std::string arg(const std::string& key, const std::string& default_val = "") const {
        consumed_.insert(key);
        auto it = args_.find(key);
        return it != args_.end() ? it->second : default_val;
    }

    /// Get a required string arg value; throws if not present.
    /// Use this for values that must come from the Python config rather than
    /// silently falling back to a C++ default that can drift out of sync.
    std::string arg_required(const std::string& key) const {
        consumed_.insert(key);
        auto it = args_.find(key);
        if (it == args_.end()) {
            throw std::runtime_error("NativeModule: missing required arg '--" + key + "'");
        }
        return it->second;
    }

    /// Get a float arg value, or a default if not present.
    float arg_float(const std::string& key, float default_val = 0.0f) const {
        consumed_.insert(key);
        auto it = args_.find(key);
        if (it == args_.end()) return default_val;
        std::size_t end = 0;
        const float value = std::stof(it->second, &end);
        if (strict_ && (end != it->second.size() || !std::isfinite(value)))
            throw std::invalid_argument("数值参数无效：" + key);
        return value;
    }

    /// Get an int arg value, or a default if not present.
    int arg_int(const std::string& key, int default_val = 0) const {
        consumed_.insert(key);
        auto it = args_.find(key);
        if (it == args_.end()) return default_val;
        std::size_t end = 0;
        const int value = std::stoi(it->second, &end);
        if (strict_ && end != it->second.size())
            throw std::invalid_argument("整数参数无效：" + key);
        return value;
    }

    /// Get a bool arg value, or a default if not present.
    /// Present-but-unparseable values throw, matching arg_int/arg_float's
    /// std::stoi/std::stof behaviour — a typo'd value or empty string is a
    /// misconfiguration we want to surface immediately, not silently coerce
    /// to false.
    bool arg_bool(const std::string& key, bool default_val = false) const {
        consumed_.insert(key);
        auto it = args_.find(key);
        if (it == args_.end()) return default_val;
        if (it->second == "true" || it->second == "1") return true;
        if (it->second == "false" || it->second == "0") return false;
        throw std::runtime_error(
            "NativeModule: arg '--" + key + "' has unparseable bool value '"
            + it->second + "' (expected true/false or 1/0)");
    }

    /// Check if a port/arg was provided.
    bool has(const std::string& key) const {
        return args_.count(key) > 0;
    }

// 后端显式启用，其他原生模块保留现有契约。
    void enable_strict_arguments() {
        strict_ = true;
        if (!invalid_argument_.empty())
            throw std::invalid_argument("缺少值或格式错误的参数：" + invalid_argument_);
    }
    void require_all_arguments_consumed() const {
        for (const auto& entry : args_)
            if (!consumed_.count(entry.first))
                throw std::invalid_argument("未知或未消费的参数：" + entry.first);
    }

private:
    mutable std::set<std::string> consumed_;
    std::string invalid_argument_;
    bool strict_ = false;
    std::map<std::string, std::string> args_;
};

/// Convert seconds (double) to a ROS-style Time message.
inline std_msgs::Time time_from_seconds(double t) {
    std_msgs::Time ts;
    ts.sec = static_cast<int32_t>(t);
    ts.nsec = static_cast<int32_t>((t - ts.sec) * 1e9);
    return ts;
}

/// Build a stamped Header with auto-incrementing sequence number.
inline std_msgs::Header make_header(const std::string& frame_id, double ts) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header h;
    h.seq = seq.fetch_add(1, std::memory_order_relaxed);
    h.stamp = time_from_seconds(ts);
    h.frame_id = frame_id;
    return h;
}

}  // namespace dimos

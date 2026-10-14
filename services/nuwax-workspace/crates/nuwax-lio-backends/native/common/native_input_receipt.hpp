#pragma once

#include <cstdint>
#include <cstdio>
#include <vector>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include "nuwax_msgs/LioInputAck.hpp"

namespace nuwax_lio {

// 回执属于当前私有输入通道；只有处理迭代实际返回后才确认该扫描。
class NativeInputReceipt {
public:
    NativeInputReceipt(lcm::LCM& transport, std::string topic)
        : transport_(transport), topic_(std::move(topic)) {}

    bool enabled() const { return !topic_.empty(); }
    bool failed() const { return failed_; }

    // 重试必须逐字节等于上次处理输入；只重发回执，不再次喂入惯性或点云。
    bool stage_packet(int32_t sequence, uint64_t begin, std::vector<uint8_t> bytes) {
        if (!enabled()) return true;
        if (failed_) return false;
        if (last_ack_ && sequence == last_sequence_) {
            if (bytes != last_packet_ || begin != static_cast<uint64_t>(last_ack_->scan_begin_timestamp_ns))
                return reject(sequence, begin, begin, "同一序列的重试载荷发生变化");
            if (transport_.publish(topic_, &*last_ack_) != 0) failed_ = true;
            return false;
        }
        if (failed_ || pending_ || last_sequence_ == std::numeric_limits<int32_t>::max()
            || sequence != last_sequence_ + 1)
            return reject(sequence, begin, begin, "整组输入序列缺失或重复");
        staged_sequence_ = sequence;
        staged_packet_ = std::move(bytes);
        return true;
    }

    bool accept(int32_t sequence, uint64_t begin, uint64_t end) {
        if (!enabled()) return true;
        if (failed_) return false;
        if (pending_ || last_sequence_ == std::numeric_limits<int32_t>::max()
            || sequence != last_sequence_ + 1 || staged_sequence_ != sequence || end < begin
            || end > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
            return reject(sequence, begin, end, "扫描序列缺失、重复或上次处理尚未完成");
        }
        nuwax_msgs::LioInputAck ack;
        ack.schema_version = 1;
        ack.sequence = sequence;
        ack.scan_begin_timestamp_ns = static_cast<int64_t>(begin);
        ack.scan_end_timestamp_ns = static_cast<int64_t>(end);
        ack.success = 1;
        ack.message = "processed";
        pending_ = std::move(ack);
        return true;
    }

    void complete() {
        if (!pending_) return;
        if (failed_) throw std::runtime_error("失败输入不能确认处理完成");
        if (transport_.publish(topic_, &*pending_) != 0)
            throw std::runtime_error("原生扫描处理回执发布失败");
        last_sequence_ = pending_->sequence;
        last_ack_ = *pending_;
        last_packet_ = std::move(staged_packet_);
        pending_.reset();
    }

    bool reject(int32_t sequence, uint64_t begin, uint64_t end,
                             const std::string& reason) {
        failed_ = true;
        if (enabled()) {
            nuwax_msgs::LioInputAck ack;
            ack.schema_version = 1;
            ack.sequence = sequence;
            ack.scan_begin_timestamp_ns = static_cast<int64_t>(begin);
            ack.scan_end_timestamp_ns = static_cast<int64_t>(end);
            ack.success = 0;
            ack.message = reason;
            transport_.publish(topic_, &ack);
        }
        return false;
    }

private:
    lcm::LCM& transport_;
    std::string topic_;
    int32_t last_sequence_ = 0;
    std::optional<nuwax_msgs::LioInputAck> pending_;
    std::optional<nuwax_msgs::LioInputAck> last_ack_;
    std::vector<uint8_t> staged_packet_;
    std::vector<uint8_t> last_packet_;
    int32_t staged_sequence_ = 0;
    bool failed_ = false;
};

}  // 命名空间 nuwax_lio

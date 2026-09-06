#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

namespace nuwax_lio {

// 扫描需要左右两侧的真实惯性样本。保留左界供下一扫描验证，右界喂给内核完成配组；
// 接收去重和内核喂入去重分别记时，避免保留的边界样本被再次送入估计器。
template <typename Sample, typename Feed>
void feed_scan_imu_and_retain_boundary(
    std::vector<Sample>& samples,
    uint64_t scan_end_ns,
    std::optional<uint64_t>& last_fed_timestamp_ns,
    Feed&& feed
) {
    const auto first_after_end = std::upper_bound(
        samples.begin(), samples.end(), scan_end_ns,
        [](uint64_t timestamp_ns, const Sample& sample) {
            return timestamp_ns < sample.timestamp_ns;
        });
    auto feed_until = first_after_end;
    if (feed_until != samples.end()) { ++feed_until; }
    for (auto sample = samples.begin(); sample != feed_until; ++sample) {
        if (last_fed_timestamp_ns && sample->timestamp_ns <= *last_fed_timestamp_ns) {
            continue;
        }
        feed(sample->message);
        last_fed_timestamp_ns = sample->timestamp_ns;
    }
    auto retain_from = first_after_end;
    if (retain_from != samples.begin()) { --retain_from; }
    samples.erase(samples.begin(), retain_from);
}

}  // 命名空间 nuwax_lio

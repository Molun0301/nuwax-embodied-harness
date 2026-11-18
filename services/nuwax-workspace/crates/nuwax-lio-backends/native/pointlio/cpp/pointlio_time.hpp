#pragma once

#include <cmath>
#include <cstdint>

#include "msgs.h"

namespace nuwax_lio {

using SourceTimeNs = std::int64_t;
inline constexpr SourceTimeNs kNanosecondsPerSecond = 1000000000;

inline SourceTimeNs source_time_ns(const custom_messages::Time& stamp) {
    return static_cast<SourceTimeNs>(stamp.sec) * kNanosecondsPerSecond
        + static_cast<SourceTimeNs>(stamp.nsecs);
}

inline custom_messages::Time source_stamp(SourceTimeNs timestamp_ns) {
    custom_messages::Time stamp;
    stamp.sec = timestamp_ns / kNanosecondsPerSecond;
    stamp.nsecs = timestamp_ns % kNanosecondsPerSecond;
    return stamp;
}

inline SourceTimeNs seconds_to_ns(double seconds) {
    return std::llround(seconds * kNanosecondsPerSecond);
}

// 先在同一源时钟上求整数差，再转换积分和插值所需的秒数。
inline double elapsed_seconds(SourceTimeNs end_ns, SourceTimeNs begin_ns) {
    return static_cast<double>(end_ns - begin_ns) / kNanosecondsPerSecond;
}

}

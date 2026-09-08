#pragma once

#include <cstdio>
#include <string>

namespace nuwax_lio {

// 保留既有五组历史的接收突发预算，每组最多携带 4096 条惯性。
// 网关现为一组待发加一组在途；底层预算不等于网关当前队列长度，也不保证无限积压无损。
inline constexpr int kNativeInputBurstGroups = 5;
inline constexpr int kNativeInputMaxImuSamplesPerGroup = 4096;
inline constexpr int kNativeInputImuQueueCapacity =
    kNativeInputBurstGroups * kNativeInputMaxImuSamplesPerGroup;
// 点云保留通信库原有的 30 条容量，不因惯性扩容降低旧点云接收容忍度。
inline constexpr int kNativeInputCloudQueueCapacity = 30;
static_assert(kNativeInputCloudQueueCapacity >= kNativeInputBurstGroups);

template <typename Subscription>
bool configure_native_input_subscription(
    Subscription* subscription,
    int capacity,
    const char* backend,
    const std::string& topic
) {
    if (subscription == nullptr) {
        std::fprintf(stderr, "[%s] create input subscription failed: %s\n", backend, topic.c_str());
        return false;
    }
    if (subscription->setQueueCapacity(capacity) != 0) {
        std::fprintf(stderr, "[%s] set input subscription queue capacity failed: %s (%d)\n",
            backend, topic.c_str(), capacity);
        return false;
    }
    return true;
}

}  // 命名空间 nuwax_lio

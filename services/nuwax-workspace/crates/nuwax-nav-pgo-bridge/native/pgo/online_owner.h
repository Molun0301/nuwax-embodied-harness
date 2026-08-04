#pragma once

#include "simple_pgo.h"

#include <atomic>
#include <cstdint>
#include <string>

struct OnlinePgoOptions {
    std::string lcm_url;
    int64_t runtime_generation{0};
    int64_t backend_generation{0};
    std::string request_channel;
    std::string result_channel;
    std::string status_channel;
    std::string cancel_channel;
    Config graph;
    double loop_search_radius_m{1.0};
    double loop_time_threshold_s{60.0};
    double loop_score_threshold{0.15};
    int loop_submap_half_range{5};
    double submap_resolution_m{0.1};
    double min_loop_detect_duration_s{10.0};
};

bool parseOnlinePgoOptions(int argc, char** argv, OnlinePgoOptions& options,
                           std::string& error);
int runOnlinePgoOwner(const OnlinePgoOptions& options, const std::atomic<bool>& running);

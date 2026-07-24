#include "simple_pgo.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <unistd.h>

namespace {
std::atomic<bool> running{true};
void signalHandler(int) { running.store(false); }

bool hasOnlyParsedFields(std::istringstream& fields) {
    fields >> std::ws;
    return fields.eof();
}

bool isFinite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool isLifecycleOption(const std::string& option) {
    static const std::unordered_set<std::string> options = {
        "--registered_scan",        "--odometry",          "--corrected_odometry",
        "--global_map",             "--pgo_tf",            "--world_frame",
        "--local_frame",            "--key_pose_delta_deg", "--key_pose_delta_trans",
        "--loop_search_radius",     "--loop_time_thresh",  "--loop_score_thresh",
        "--loop_submap_half_range", "--submap_resolution", "--min_loop_detect_duration",
        "--unregister_input",       "--global_map_voxel_size",
        "--global_map_publish_rate", "--debug",
    };
    return options.find(option) != options.end();
}

bool publishOutput(const std::filesystem::path& output_path, const SimplePGO& pgo,
                   const OptimizationSummary& summary) {
    std::ostringstream serialized;
    serialized << std::setprecision(12);
    serialized << "SUMMARY " << pgo.keyPoses().size() << ' ' << summary.cost_before << ' '
               << summary.cost_after << '\n';
    for (std::size_t i = 0; i < pgo.keyPoses().size(); ++i) {
        const auto& pose = pgo.keyPoses()[i].global;
        serialized << "POSE " << i << ' ' << pose.t.x << ' ' << pose.t.y << ' ' << pose.t.z
                   << ' ' << pose.yaw << '\n';
    }
    const auto offset = pgo.offsetT();
    serialized << "OFFSET " << offset.x << ' ' << offset.y << ' ' << offset.z << ' '
               << pgo.offsetYaw() << '\n';
    if (!serialized) return false;

    const std::string output_path_text = output_path.string();
    auto template_path = output_path_text + ".tmp.XXXXXX";
    std::vector<char> template_buffer(template_path.begin(), template_path.end());
    template_buffer.push_back('\0');
    const int file_descriptor = ::mkstemp(template_buffer.data());
    if (file_descriptor < 0) return false;
    std::FILE* output = ::fdopen(file_descriptor, "w");
    if (output == nullptr) {
        ::close(file_descriptor);
        ::unlink(template_buffer.data());
        return false;
    }
    const std::string contents = serialized.str();
    const bool written = std::fwrite(contents.data(), 1, contents.size(), output) == contents.size();
    const bool flushed = written && std::fflush(output) == 0;
    const bool closed = std::fclose(output) == 0;
    if (!flushed || !closed) {
        ::unlink(template_buffer.data());
        return false;
    }
    if (::rename(template_buffer.data(), output_path_text.c_str()) != 0) {
        ::unlink(template_buffer.data());
        return false;
    }
    return true;
}

int runBatch(const std::string& input_path, const std::string& output_path) {
    std::ifstream input(input_path);
    if (!input) {
        std::cerr << "PGO: cannot open batch input\n";
        return 2;
    }
    SimplePGO pgo(Config{});
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#') continue;
        std::istringstream fields(line);
        std::string kind;
        fields >> kind;
        if (kind == "POSE") {
            PoseWithTime pose;
            fields >> pose.second >> pose.t.x >> pose.t.y >> pose.t.z >> pose.yaw;
            const bool valid_seconds =
                std::isfinite(pose.second) && pose.second >= std::numeric_limits<int32_t>::min() &&
                pose.second <= std::numeric_limits<int32_t>::max();
            if (!fields || !hasOnlyParsedFields(fields) || !valid_seconds || !isFinite(pose.t) ||
                !std::isfinite(pose.yaw)) {
                std::cerr << "PGO: invalid or non-key POSE: " << line << '\n';
                return 3;
            }
            pose.setTime(static_cast<int32_t>(pose.second), 0);
            if (!pgo.addKeyPose(pose)) {
                std::cerr << "PGO: invalid or non-key POSE: " << line << '\n';
                return 3;
            }
        } else if (kind == "LOOP") {
            std::size_t source = 0;
            std::size_t target = 0;
            Vec3 delta;
            double delta_yaw = 0.0;
            double weight = 0.0;
            fields >> source >> target >> delta.x >> delta.y >> delta.z >> delta_yaw >> weight;
            if (!fields || !hasOnlyParsedFields(fields) || !isFinite(delta) ||
                !std::isfinite(delta_yaw) || !std::isfinite(weight) ||
                !pgo.addLoopPair(source, target, delta, delta_yaw, weight)) {
                std::cerr << "PGO: invalid LOOP: " << line << '\n';
                return 4;
            }
        } else {
            std::cerr << "PGO: unknown batch record: " << kind << '\n';
            return 5;
        }
    }
    if (!input.eof()) {
        std::cerr << "PGO: failed while reading batch input\n";
        return 2;
    }
    const auto summary = pgo.smoothAndUpdate();
    if (!summary.success) {
        std::cerr << "PGO: optimization failed\n";
        return 6;
    }
    if (!publishOutput(output_path, pgo, summary)) {
        std::cerr << "PGO: failed to publish batch output\n";
        return 7;
    }
    return 0;
}
}

int main(int argc, char** argv) {
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGINT, signalHandler);
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "pgo [LIFECYCLE_OPTION VALUE ...] | "
                     "--batch-input FILE --batch-output FILE\n";
        return 0;
    }

    std::string input_path;
    std::string output_path;
    std::unordered_set<std::string> seen_options;
    bool has_batch_option = false;
    bool has_lifecycle_option = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            std::cerr << "PGO: --help must be used alone\n";
            return 2;
        }
        const bool is_batch_option = arg == "--batch-input" || arg == "--batch-output";
        const bool is_lifecycle_option = isLifecycleOption(arg);
        if (!is_batch_option && !is_lifecycle_option) {
            std::cerr << "PGO: unknown option: " << arg << '\n';
            return 2;
        }
        if (!seen_options.insert(arg).second) {
            std::cerr << "PGO: duplicate option: " << arg << '\n';
            return 2;
        }
        if (i + 1 >= argc) {
            std::cerr << "PGO: option requires a value: " << arg << '\n';
            return 2;
        }
        const std::string value = argv[i + 1];
        if (value.empty() || value.rfind("--", 0) == 0) {
            std::cerr << "PGO: option requires a non-option value: " << arg << '\n';
            return 2;
        }
        ++i;
        if (arg == "--batch-input") {
            input_path = value;
            has_batch_option = true;
        } else if (arg == "--batch-output") {
            output_path = value;
            has_batch_option = true;
        } else {
            has_lifecycle_option = true;
        }
    }
    if (has_batch_option && has_lifecycle_option) {
        std::cerr << "PGO: batch and lifecycle options cannot be mixed\n";
        return 2;
    }
    if (has_batch_option) {
        if (input_path.empty() || output_path.empty()) {
            std::cerr << "PGO: batch mode requires both input and output paths\n";
            return 2;
        }
        return runBatch(input_path, output_path);
    }
    std::cout << "pgo-native-ready mode=lifecycle-only canonical_tf_owner=external\n";
    std::cout.flush();
    while (running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return 0;
}

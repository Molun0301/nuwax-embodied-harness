#include "online_owner.h"

#include <nuwax_msgs/NativePgoRequest.hpp>
#include <nuwax_msgs/NativePgoResult.hpp>
#include <nuwax_msgs/NativePgoStatus.hpp>

#include <lcm/lcm-cpp.hpp>
#include <openssl/evp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace {

using Digest = std::array<uint8_t, 32>;
using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using Request = nuwax_msgs::NativePgoRequest;
using Result = nuwax_msgs::NativePgoResult;
using Status = nuwax_msgs::NativePgoStatus;

constexpr std::size_t kMaxIdentityBytes = 255;
constexpr std::size_t kMaxIcpCloudPoints =
    static_cast<std::size_t>(Result::MAX_LOOP_INLIERS);
constexpr std::size_t kMaxSubmapPoints = 100'000;
constexpr std::size_t kMaxGraphNodes =
    static_cast<std::size_t>(Result::MAX_GRAPH_NODES);
constexpr int kIcpIterationLimit = 50;
constexpr double kIcpMaxCorrespondenceDistanceM = 10.0;
constexpr std::size_t kMinIcpInliers = 20;
constexpr std::size_t kStatusFixedWireBytes = 477;
constexpr std::size_t kStatusStringCount = 6;
constexpr std::size_t kMaxStatusWireBytes =
    kStatusFixedWireBytes +
    kStatusStringCount * (sizeof(int32_t) + kMaxIdentityBytes + 1);
constexpr double kQuaternionNormTolerance = 1.0e-6;
constexpr double kMaxTranslationAbsM = 1'000'000.0;
constexpr auto kHealthPeriod = std::chrono::seconds(1);
constexpr auto kWorkerShutdownBound = std::chrono::milliseconds(300);

static_assert(Result::MAX_GRAPH_NODES == 2'000);
static_assert(Result::MAX_GRAPH_LOOP_EDGES == Result::MAX_GRAPH_NODES - 2);
static_assert(Result::MAX_GRAPH_EDGES ==
              (Result::MAX_GRAPH_NODES - 1) + Result::MAX_GRAPH_LOOP_EDGES);
static_assert(Result::MAX_LOOP_INLIERS == 20'000);
static_assert(Result::MAX_OPTIMIZATION_ITERATIONS == 300);
static_assert(Request::MAX_REGISTERED_BODY_POINT_COUNT >= Result::MAX_LOOP_INLIERS);
static_assert(kMaxStatusWireBytes == 2'037);

constexpr char kRequestHashDomain[] = "nuwax.native-pgo.request.v2\0";
constexpr char kResultHashDomain[] = "nuwax.native-pgo.result.v2\0";
constexpr char kStatusHashDomain[] = "nuwax.native-pgo.status.v2\0";
constexpr char kBodyPointsHashDomain[] = "nuwax.correction.body-points.v1\0";
constexpr char kOdomPoseHashDomain[] = "nuwax.correction.odom-from-body.v1\0";
constexpr char kMapBoundaryHashDomain[] =
    "nuwax.correction.committed-map-boundary.v1\0";
constexpr char kLoopProofHashDomain[] = "nuwax.native-pgo.loop-proof.v1\0";
constexpr char kGraphHashDomain[] = "nuwax.native-pgo.optimized-graph.v2\0";

class DigestBuilder {
public:
    DigestBuilder() : context_(EVP_MD_CTX_new()) {
        good_ = context_ != nullptr && EVP_DigestInit_ex(context_, EVP_sha256(), nullptr) == 1;
    }

    DigestBuilder(const DigestBuilder&) = delete;
    DigestBuilder& operator=(const DigestBuilder&) = delete;

    ~DigestBuilder() {
        if (context_ != nullptr) EVP_MD_CTX_free(context_);
    }

    bool update(const void* data, std::size_t size) {
        if (!good_ || (data == nullptr && size != 0)) return false;
        if (size == 0) return true;
        good_ = EVP_DigestUpdate(context_, data, size) == 1;
        return good_;
    }

    template <std::size_t N>
    bool updateDomain(const char (&domain)[N]) {
        static_assert(N >= 2);
        return update(domain, N - 1);
    }

    std::optional<Digest> finish() {
        if (!good_) return std::nullopt;
        Digest digest{};
        unsigned int size = 0;
        good_ = EVP_DigestFinal_ex(context_, digest.data(), &size) == 1 &&
                size == digest.size();
        if (!good_) return std::nullopt;
        return digest;
    }

private:
    EVP_MD_CTX* context_{nullptr};
    bool good_{false};
};

template <typename Integer>
bool appendBigEndian(DigestBuilder& builder, Integer value) {
    static_assert(std::is_integral_v<Integer>);
    using Unsigned = std::make_unsigned_t<Integer>;
    Unsigned encoded = static_cast<Unsigned>(value);
    std::array<uint8_t, sizeof(Integer)> bytes{};
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        bytes[bytes.size() - index - 1] = static_cast<uint8_t>(encoded & 0xffU);
        encoded >>= 8U;
    }
    return builder.update(bytes.data(), bytes.size());
}

bool appendBigEndianDouble(DigestBuilder& builder, double value) {
    static_assert(sizeof(double) == sizeof(uint64_t));
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return appendBigEndian(builder, bits);
}

bool appendLcmString(DigestBuilder& builder, const std::string& value) {
    if (value.size() >= static_cast<std::size_t>(std::numeric_limits<int32_t>::max())) {
        return false;
    }
    const int32_t encoded_size = static_cast<int32_t>(value.size() + 1);
    const uint8_t terminator = 0;
    return appendBigEndian(builder, encoded_size) && builder.update(value.data(), value.size()) &&
           builder.update(&terminator, 1);
}

template <typename Message, std::size_t DomainSize>
std::optional<Digest> hashGeneratedBody(const Message& message,
                                        const char (&domain)[DomainSize]) {
    const int encoded_size = message.getEncodedSize();
    constexpr int fingerprint_size = 8;
    constexpr int self_hash_size = 32;
    if (encoded_size < fingerprint_size + self_hash_size) return std::nullopt;
    std::vector<uint8_t> wire(static_cast<std::size_t>(encoded_size));
    if (message.encode(wire.data(), 0, encoded_size) != encoded_size) return std::nullopt;
    DigestBuilder builder;
    if (!builder.updateDomain(domain) ||
        !builder.update(wire.data() + fingerprint_size,
                        wire.size() - fingerprint_size - self_hash_size)) {
        return std::nullopt;
    }
    return builder.finish();
}

template <std::size_t N>
bool allZero(const uint8_t (&value)[N]) {
    return std::all_of(std::begin(value), std::end(value),
                       [](uint8_t byte) { return byte == 0; });
}

template <std::size_t N>
bool equalBytes(const uint8_t (&left)[N], const uint8_t (&right)[N]) {
    return std::equal(std::begin(left), std::end(left), std::begin(right));
}

template <std::size_t N>
void copyBytes(const uint8_t (&source)[N], uint8_t (&target)[N]) {
    std::copy(std::begin(source), std::end(source), std::begin(target));
}

template <std::size_t N>
void copyDigest(const Digest& source, uint8_t (&target)[N]) {
    static_assert(N == 32);
    std::copy(source.begin(), source.end(), std::begin(target));
}

template <std::size_t N>
bool digestEquals(const Digest& expected, const uint8_t (&actual)[N]) {
    static_assert(N == 32);
    return std::equal(expected.begin(), expected.end(), std::begin(actual));
}

bool validIdentityString(const std::string& value) {
    if (value.empty() || value.size() > kMaxIdentityBytes ||
        value.find('\0') != std::string::npos) {
        return false;
    }
    const auto whitespace = [](unsigned char byte) { return std::isspace(byte) != 0; };
    return !whitespace(static_cast<unsigned char>(value.front())) &&
           !whitespace(static_cast<unsigned char>(value.back()));
}

bool parsePositiveInt64(const std::string& text, int64_t& value) {
    if (text.empty()) return false;
    const char* begin = text.data();
    const char* end = text.data() + text.size();
    const auto parsed = std::from_chars(begin, end, value);
    return parsed.ec == std::errc{} && parsed.ptr == end && value > 0;
}

bool parseNonnegativeDouble(const std::string& text, double& value) {
    if (text.empty()) return false;
    errno = 0;
    char* end = nullptr;
    value = std::strtod(text.c_str(), &end);
    return errno == 0 && end == text.c_str() + text.size() && std::isfinite(value) &&
           value >= 0.0;
}

bool parsePositiveDouble(const std::string& text, double& value) {
    return parseNonnegativeDouble(text, value) && value > 0.0;
}

bool parseNonnegativeInt(const std::string& text, int& value) {
    if (text.empty()) return false;
    int64_t parsed = 0;
    const char* begin = text.data();
    const char* end = text.data() + text.size();
    const auto converted = std::from_chars(begin, end, parsed);
    if (converted.ec != std::errc{} || converted.ptr != end || parsed < 0 ||
        parsed > std::numeric_limits<int>::max()) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

std::string expectedTypedChannel(int64_t runtime_generation, int64_t backend_generation,
                                 const std::string& leaf, const std::string& type) {
    return "/c/n/" + std::to_string(runtime_generation) + "/b/" +
           std::to_string(backend_generation) + "/" + leaf + "#" + type;
}

int64_t timestampNowNs() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    return std::max<int64_t>(0, nanoseconds);
}

bool finiteQuaternion(const double (&rotation)[4]) {
    double norm_squared = 0.0;
    for (double value : rotation) {
        if (!std::isfinite(value)) return false;
        norm_squared += value * value;
    }
    return std::isfinite(norm_squared) &&
           std::abs(norm_squared - 1.0) <= kQuaternionNormTolerance;
}

bool finiteTranslation(const double (&translation)[3]) {
    for (double value : translation) {
        if (!std::isfinite(value) || std::abs(value) > kMaxTranslationAbsM) return false;
    }
    return true;
}

std::optional<Digest> bodyPointsHash(const Request& request) {
    DigestBuilder builder;
    if (!builder.updateDomain(kBodyPointsHashDomain) ||
        !appendBigEndian(builder, request.registered_body_point_count)) {
        return std::nullopt;
    }
    for (double coordinate : request.registered_body_points_xyz) {
        if (!appendBigEndianDouble(builder, coordinate)) return std::nullopt;
    }
    return builder.finish();
}

std::optional<Digest> odomPoseHash(const Request& request) {
    DigestBuilder builder;
    if (!builder.updateDomain(kOdomPoseHashDomain)) return std::nullopt;
    for (double value : request.frozen_odom_translation_m) {
        if (!appendBigEndianDouble(builder, value)) return std::nullopt;
    }
    for (double value : request.frozen_odom_rotation_xyzw) {
        if (!appendBigEndianDouble(builder, value)) return std::nullopt;
    }
    return builder.finish();
}

std::optional<Digest> mapBoundaryHash(const Request& request) {
    DigestBuilder builder;
    if (!builder.updateDomain(kMapBoundaryHashDomain) ||
        !appendLcmString(builder, request.map_frame_id) ||
        !appendBigEndian(builder, request.committed_map_width) ||
        !appendBigEndian(builder, request.committed_map_height) ||
        !appendBigEndianDouble(builder, request.committed_map_resolution_m) ||
        !appendBigEndianDouble(builder, request.committed_map_origin_x_m) ||
        !appendBigEndianDouble(builder, request.committed_map_origin_y_m)) {
        return std::nullopt;
    }
    return builder.finish();
}

std::optional<Digest> mapPayloadHash(const Request& request) {
    DigestBuilder builder;
    if (!request.committed_map_occupancy.empty() &&
        !builder.update(request.committed_map_occupancy.data(),
                        request.committed_map_occupancy.size())) {
        return std::nullopt;
    }
    return builder.finish();
}

std::optional<Digest> requestPayloadHash(const Request& request) {
    Request canonical = request;
    std::fill(std::begin(canonical.request_payload_sha256),
              std::end(canonical.request_payload_sha256), 0);
    return hashGeneratedBody(canonical, kRequestHashDomain);
}

class RequestWireCursor {
public:
    RequestWireCursor(const void* data, std::size_t size)
        : data_(static_cast<const uint8_t*>(data)), size_(size) {}

    std::optional<int32_t> readI32() {
        const auto value = readUnsigned<4>();
        if (!value) return std::nullopt;
        return static_cast<int32_t>(static_cast<uint32_t>(*value));
    }

    std::optional<int8_t> readI8() {
        const auto value = readUnsigned<1>();
        if (!value) return std::nullopt;
        return static_cast<int8_t>(static_cast<uint8_t>(*value));
    }

    std::optional<int64_t> readI64() {
        const auto value = readUnsigned<8>();
        if (!value) return std::nullopt;
        return static_cast<int64_t>(*value);
    }

    bool skip(std::size_t count) {
        if (count > size_ - position_) return false;
        position_ += count;
        return true;
    }

    bool skipBoundedString() {
        const auto encoded_length = readI32();
        if (!encoded_length || *encoded_length <= 0 ||
            *encoded_length > static_cast<int32_t>(kMaxIdentityBytes + 1) ||
            static_cast<std::size_t>(*encoded_length) > size_ - position_ ||
            data_[position_ + static_cast<std::size_t>(*encoded_length) - 1] != 0 ||
            std::find(data_ + position_,
                      data_ + position_ + static_cast<std::size_t>(*encoded_length) - 1,
                      uint8_t{0}) !=
                data_ + position_ + static_cast<std::size_t>(*encoded_length) - 1) {
            return false;
        }
        return skip(static_cast<std::size_t>(*encoded_length));
    }

    bool finished() const { return position_ == size_; }

private:
    template <std::size_t Width>
    std::optional<uint64_t> readUnsigned() {
        static_assert(Width <= sizeof(uint64_t));
        if (Width > size_ - position_) return std::nullopt;
        uint64_t value = 0;
        for (std::size_t index = 0; index < Width; ++index) {
            value = (value << 8U) | data_[position_++];
        }
        return value;
    }

    const uint8_t* data_;
    std::size_t size_;
    std::size_t position_{0};
};

bool preflightRequestWire(const void* data, std::size_t size) {
    RequestWireCursor cursor(data, size);
    const auto fingerprint = cursor.readI64();
    const auto schema_version = cursor.readI32();
    const auto backend = cursor.readI32();
    if (!fingerprint || *fingerprint != Request::getHash() || !schema_version || !backend ||
        !cursor.skip(3 * 8) || !cursor.skipBoundedString() || !cursor.skip(2 * 8 + 16) ||
        !cursor.skip(6 * 8)) {
        return false;
    }
    const auto point_count = cursor.readI64();
    const auto coordinate_count = cursor.readI32();
    if (!point_count || !coordinate_count || *point_count <= 0 ||
        *point_count > Request::MAX_REGISTERED_BODY_POINT_COUNT || *coordinate_count <= 0 ||
        *coordinate_count > Request::MAX_REGISTERED_BODY_COORDINATE_COUNT ||
        *coordinate_count != *point_count * 3 ||
        !cursor.skip(static_cast<std::size_t>(*coordinate_count) * sizeof(double)) ||
        !cursor.skip(2 * 32 + 2 * 8 + 3 * 8 + 4 * 8 + 32)) {
        return false;
    }
    const auto map_width = cursor.readI64();
    const auto map_height = cursor.readI64();
    if (!map_width || !map_height || *map_width <= 0 || *map_height <= 0 ||
        !cursor.skip(3 * 8)) {
        return false;
    }
    const auto cell_count = cursor.readI32();
    if (!cell_count || *cell_count <= 0 ||
        *cell_count > Request::MAX_COMMITTED_MAP_CELL_COUNT ||
        *map_width > std::numeric_limits<int64_t>::max() / *map_height ||
        *map_width * *map_height != *cell_count ||
        !cursor.skip(static_cast<std::size_t>(*cell_count)) || !cursor.skip(2 * 32 + 8) ||
        !cursor.skipBoundedString() || !cursor.skipBoundedString() ||
        !cursor.skipBoundedString() || !cursor.skipBoundedString() ||
        !cursor.skip(16 + 8 + 32)) {
        return false;
    }
    return cursor.finished();
}

bool preflightStatusWire(const void* data, std::size_t size) {
    RequestWireCursor cursor(data, size);
    const auto fingerprint = cursor.readI64();
    if (!fingerprint || *fingerprint != Status::getHash() ||
        !cursor.skip(2 * sizeof(int32_t) + 4 * sizeof(int64_t))) {
        return false;
    }
    const auto identity_present = cursor.readI8();
    if (!identity_present || *identity_present != 1 ||
        !cursor.skip(sizeof(int64_t)) || !cursor.skipBoundedString() ||
        !cursor.skip(2 * sizeof(int64_t) + 16 + 7 * sizeof(int64_t)) ||
        !cursor.skip(2 * 32) || !cursor.skip(2 * sizeof(int64_t) + 32) ||
        !cursor.skip(2 * sizeof(int64_t) + 3 * sizeof(double)) ||
        !cursor.skip(2 * 32 + sizeof(int64_t)) ||
        !cursor.skipBoundedString() || !cursor.skipBoundedString() ||
        !cursor.skipBoundedString() || !cursor.skipBoundedString() ||
        !cursor.skip(16 + sizeof(int64_t) + 32) ||
        !cursor.skip(5 * sizeof(int32_t)) || !cursor.skipBoundedString() ||
        !cursor.skip(32)) {
        return false;
    }
    return cursor.finished();
}

struct RequestValidationError {
    int32_t reason_code{Status::REASON_INVALID_REQUEST};
    std::string reason;
};

std::optional<RequestValidationError> validateRequest(const Request& request,
                                                       const OnlinePgoOptions& options) {
    const auto invalid = [](std::string reason) {
        return std::optional<RequestValidationError>(
            RequestValidationError{Status::REASON_INVALID_REQUEST, std::move(reason)});
    };
    if (request.schema_version != Request::SCHEMA_VERSION_V2 ||
        request.backend != Request::BACKEND_NATIVE_PGO) {
        return invalid("unsupported request schema or backend");
    }
    if (request.runtime_generation != options.runtime_generation ||
        request.backend_generation != options.backend_generation) {
        return RequestValidationError{Status::REASON_PROTOCOL_IDENTITY_MISMATCH,
                                      "owner generation mismatch"};
    }
    if (request.request_sequence <= 0 || !validIdentityString(request.mapping_session_id) ||
        request.mapping_session_generation <= 0 || request.map_frame_generation <= 0 ||
        allZero(request.mapping_commit_boot_nonce) || request.mapping_commit_sequence <= 0 ||
        request.occupancy_revision <= 0 || request.cloud_revision <= 0) {
        return invalid("incomplete request identity");
    }
    if (request.committed_map_source_timestamp_ns < 0 ||
        request.registered_cloud_source_sequence <= 0 ||
        request.registered_cloud_source_timestamp_ns < 0 ||
        request.correlated_odom_source_sequence <= 0 ||
        request.correlated_odom_source_timestamp_ns < 0 || request.source_timestamp_ns < 0) {
        return invalid("invalid source sequence or timestamp");
    }
    if (request.committed_map_source_timestamp_ns != request.source_timestamp_ns ||
        request.registered_cloud_source_timestamp_ns != request.source_timestamp_ns ||
        request.correlated_odom_source_timestamp_ns > request.source_timestamp_ns) {
        return invalid("uncorrelated source timestamps");
    }
    if (request.registered_body_point_count <= 0 ||
        request.registered_body_point_count > Request::MAX_REGISTERED_BODY_POINT_COUNT ||
        request.registered_body_coordinate_count <= 0 ||
        request.registered_body_coordinate_count > Request::MAX_REGISTERED_BODY_COORDINATE_COUNT ||
        request.registered_body_point_count > std::numeric_limits<int32_t>::max() / 3 ||
        request.registered_body_coordinate_count != request.registered_body_point_count * 3 ||
        request.registered_body_points_xyz.size() !=
            static_cast<std::size_t>(request.registered_body_coordinate_count)) {
        return invalid("registered body cloud dimensions are inconsistent");
    }
    for (double coordinate : request.registered_body_points_xyz) {
        if (!std::isfinite(coordinate) || std::abs(coordinate) > kMaxTranslationAbsM) {
            return invalid("registered body cloud contains invalid coordinate");
        }
    }
    if (allZero(request.registered_cloud_envelope_sha256) ||
        !finiteTranslation(request.frozen_odom_translation_m) ||
        !finiteQuaternion(request.frozen_odom_rotation_xyzw) ||
        allZero(request.correction_lease_boot_nonce) ||
        request.correction_lease_generation <= 0) {
        return invalid("registered cloud, odometry, or lease proof is invalid");
    }
    if (request.committed_map_width <= 0 || request.committed_map_height <= 0 ||
        request.committed_map_width > std::numeric_limits<int32_t>::max() ||
        request.committed_map_height > std::numeric_limits<int32_t>::max() ||
        request.committed_map_width >
            std::numeric_limits<int64_t>::max() / request.committed_map_height ||
        request.committed_map_width * request.committed_map_height !=
            request.committed_map_cell_count ||
        request.committed_map_cell_count <= 0 ||
        request.committed_map_cell_count > Request::MAX_COMMITTED_MAP_CELL_COUNT ||
        request.committed_map_occupancy.size() !=
            static_cast<std::size_t>(request.committed_map_cell_count) ||
        !std::isfinite(request.committed_map_resolution_m) ||
        request.committed_map_resolution_m <= 0.0 ||
        !std::isfinite(request.committed_map_origin_x_m) ||
        !std::isfinite(request.committed_map_origin_y_m)) {
        return invalid("committed map geometry is invalid");
    }
    if (std::any_of(request.committed_map_occupancy.begin(),
                    request.committed_map_occupancy.end(),
                    [](int8_t value) { return value < -1 || value > 100; })) {
        return invalid("committed map occupancy is outside [-1,100]");
    }
    if (!validIdentityString(request.map_frame_id) ||
        !validIdentityString(request.odom_frame_id) ||
        !validIdentityString(request.body_frame_id) ||
        !validIdentityString(request.points_frame_id) ||
        request.map_frame_id == request.odom_frame_id ||
        request.map_frame_id == request.body_frame_id ||
        request.odom_frame_id == request.body_frame_id ||
        request.points_frame_id != request.body_frame_id) {
        return invalid("request frame contract is invalid");
    }
    const auto points_hash = bodyPointsHash(request);
    const auto odom_hash = odomPoseHash(request);
    const auto map_payload_hash = mapPayloadHash(request);
    const auto map_boundary_hash = mapBoundaryHash(request);
    if (!points_hash || !digestEquals(*points_hash, request.registered_body_points_sha256) ||
        !odom_hash || !digestEquals(*odom_hash, request.frozen_odom_pose_sha256) ||
        !map_payload_hash ||
        !digestEquals(*map_payload_hash, request.committed_map_payload_sha256) ||
        !map_boundary_hash ||
        !digestEquals(*map_boundary_hash, request.committed_map_boundary_sha256)) {
        return RequestValidationError{Status::REASON_PROTOCOL_HASH_MISMATCH,
                                      "request proof hash mismatch"};
    }
    const auto request_hash = requestPayloadHash(request);
    if (!request_hash || !digestEquals(*request_hash, request.request_payload_sha256)) {
        return RequestValidationError{Status::REASON_PROTOCOL_HASH_MISMATCH,
                                      "request payload hash mismatch"};
    }
    return std::nullopt;
}

}  // namespace

bool parseOnlinePgoOptions(int argc, char** argv, OnlinePgoOptions& options,
                           std::string& error) {
    std::unordered_map<std::string, std::string> values;
    const std::unordered_set<std::string> allowed = {
        "--mode",          "--lcm-url",          "--runtime-generation",
        "--backend-generation", "--request-channel",  "--result-channel",
        "--status-channel", "--cancel-channel",    "--key_pose_delta_deg",
        "--key_pose_delta_trans", "--loop_search_radius", "--loop_time_thresh",
        "--loop_score_thresh", "--loop_submap_half_range", "--submap_resolution",
        "--min_loop_detect_duration", "--debug",
    };
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        if (allowed.find(option) == allowed.end()) {
            error = "unknown online option: " + option;
            return false;
        }
        if (!values.emplace(option, std::string{}).second) {
            error = "duplicate online option: " + option;
            return false;
        }
        if (index + 1 >= argc) {
            error = "online option requires a value: " + option;
            return false;
        }
        const std::string value = argv[++index];
        if (value.empty() || value.rfind("--", 0) == 0) {
            error = "online option requires a non-option value: " + option;
            return false;
        }
        values[option] = value;
    }
    const auto required = [&](const std::string& option, std::string& target) {
        const auto found = values.find(option);
        if (found == values.end()) {
            error = "missing required online option: " + option;
            return false;
        }
        target = found->second;
        return true;
    };
    std::string mode;
    std::string runtime_generation;
    std::string backend_generation;
    if (!required("--mode", mode) || mode != "online" ||
        !required("--lcm-url", options.lcm_url) ||
        !required("--runtime-generation", runtime_generation) ||
        !required("--backend-generation", backend_generation) ||
        !required("--request-channel", options.request_channel) ||
        !required("--result-channel", options.result_channel) ||
        !required("--status-channel", options.status_channel) ||
        !required("--cancel-channel", options.cancel_channel)) {
        if (error.empty()) error = "--mode must be online";
        return false;
    }
    if (!validIdentityString(options.lcm_url) ||
        !parsePositiveInt64(runtime_generation, options.runtime_generation) ||
        !parsePositiveInt64(backend_generation, options.backend_generation)) {
        error = "invalid online LCM URL or generation";
        return false;
    }
    const auto expect_channel = [&](const std::string& actual, const std::string& leaf,
                                    const std::string& type) {
        return actual == expectedTypedChannel(options.runtime_generation,
                                              options.backend_generation, leaf, type);
    };
    if (!expect_channel(options.request_channel, "request", "nuwax_msgs.NativePgoRequest") ||
        !expect_channel(options.result_channel, "result", "nuwax_msgs.NativePgoResult") ||
        !expect_channel(options.status_channel, "status", "nuwax_msgs.NativePgoStatus") ||
        !expect_channel(options.cancel_channel, "cancel", "nuwax_msgs.NativePgoStatus")) {
        error = "online channels do not match the runtime/backend generation-private contract";
        return false;
    }
    const auto optional_double = [&](const std::string& option, double& target, bool positive) {
        const auto found = values.find(option);
        if (found == values.end()) return true;
        return positive ? parsePositiveDouble(found->second, target)
                        : parseNonnegativeDouble(found->second, target);
    };
    if (!optional_double("--key_pose_delta_deg", options.graph.key_pose_delta_deg, false) ||
        !optional_double("--key_pose_delta_trans", options.graph.key_pose_delta_trans, false) ||
        !optional_double("--loop_search_radius", options.loop_search_radius_m, true) ||
        !optional_double("--loop_time_thresh", options.loop_time_threshold_s, false) ||
        !optional_double("--loop_score_thresh", options.loop_score_threshold, true) ||
        !optional_double("--submap_resolution", options.submap_resolution_m, true) ||
        !optional_double("--min_loop_detect_duration", options.min_loop_detect_duration_s,
                         false)) {
        error = "invalid online floating-point algorithm option";
        return false;
    }
    const auto half_range = values.find("--loop_submap_half_range");
    if (half_range != values.end() &&
        !parseNonnegativeInt(half_range->second, options.loop_submap_half_range)) {
        error = "invalid --loop_submap_half_range";
        return false;
    }
    const auto debug = values.find("--debug");
    if (debug != values.end() && debug->second != "0" && debug->second != "1" &&
        debug->second != "false" && debug->second != "true") {
        error = "invalid --debug value";
        return false;
    }
    if (options.loop_submap_half_range > 100 || options.loop_search_radius_m > 10'000.0 ||
        options.loop_time_threshold_s > 1.0e9 || options.loop_score_threshold > 1.0e6 ||
        options.submap_resolution_m > 1'000.0 ||
        options.min_loop_detect_duration_s > 1.0e9 ||
        options.graph.optimizer_iterations != Result::MAX_OPTIMIZATION_ITERATIONS) {
        error = "online algorithm option exceeds resource bounds";
        return false;
    }
    return true;
}

namespace {

template <typename Target>
void copyRequestIdentity(const Request& request, Target& target) {
    target.runtime_generation = request.runtime_generation;
    target.backend_generation = request.backend_generation;
    target.request_sequence = request.request_sequence;
    target.mapping_session_id = request.mapping_session_id;
    target.mapping_session_generation = request.mapping_session_generation;
    target.map_frame_generation = request.map_frame_generation;
    copyBytes(request.mapping_commit_boot_nonce, target.mapping_commit_boot_nonce);
    target.mapping_commit_sequence = request.mapping_commit_sequence;
    target.occupancy_revision = request.occupancy_revision;
    target.cloud_revision = request.cloud_revision;
    target.committed_map_source_timestamp_ns = request.committed_map_source_timestamp_ns;
    target.registered_cloud_source_sequence = request.registered_cloud_source_sequence;
    target.registered_cloud_source_timestamp_ns = request.registered_cloud_source_timestamp_ns;
    target.registered_body_point_count = request.registered_body_point_count;
    copyBytes(request.registered_cloud_envelope_sha256,
              target.registered_cloud_envelope_sha256);
    copyBytes(request.registered_body_points_sha256, target.registered_body_points_sha256);
    target.correlated_odom_source_sequence = request.correlated_odom_source_sequence;
    target.correlated_odom_source_timestamp_ns = request.correlated_odom_source_timestamp_ns;
    copyBytes(request.frozen_odom_pose_sha256, target.frozen_odom_pose_sha256);
    target.committed_map_width = request.committed_map_width;
    target.committed_map_height = request.committed_map_height;
    target.committed_map_resolution_m = request.committed_map_resolution_m;
    target.committed_map_origin_x_m = request.committed_map_origin_x_m;
    target.committed_map_origin_y_m = request.committed_map_origin_y_m;
    copyBytes(request.committed_map_payload_sha256, target.committed_map_payload_sha256);
    copyBytes(request.committed_map_boundary_sha256, target.committed_map_boundary_sha256);
    target.source_timestamp_ns = request.source_timestamp_ns;
    target.map_frame_id = request.map_frame_id;
    target.odom_frame_id = request.odom_frame_id;
    target.body_frame_id = request.body_frame_id;
    target.points_frame_id = request.points_frame_id;
    copyBytes(request.correction_lease_boot_nonce, target.correction_lease_boot_nonce);
    target.correction_lease_generation = request.correction_lease_generation;
    copyBytes(request.request_payload_sha256, target.request_payload_sha256);
}

bool sealResult(Result& result) {
    std::fill(std::begin(result.result_payload_sha256),
              std::end(result.result_payload_sha256), 0);
    const auto digest = hashGeneratedBody(result, kResultHashDomain);
    if (!digest) return false;
    copyDigest(*digest, result.result_payload_sha256);
    return true;
}

bool sealStatus(Status& status) {
    std::fill(std::begin(status.status_payload_sha256),
              std::end(status.status_payload_sha256), 0);
    const auto digest = hashGeneratedBody(status, kStatusHashDomain);
    if (!digest) return false;
    copyDigest(*digest, status.status_payload_sha256);
    return true;
}

Result resultBase(const Request& request) {
    Result result{};
    result.schema_version = Result::SCHEMA_VERSION_V2;
    result.backend = Result::BACKEND_NATIVE_PGO;
    copyRequestIdentity(request, result);
    result.result_timestamp_ns = timestampNowNs();
    return result;
}

Result noCandidateResult(const Request& request) {
    Result result = resultBase(request);
    result.result_disposition = Result::DISPOSITION_NO_CANDIDATE;
    result.rejection_reason_code = Result::REJECTION_NONE;
    sealResult(result);
    return result;
}

Result rejectedResult(const Request& request, int32_t rejection_code,
                      const std::string& reason) {
    Result result = resultBase(request);
    result.result_disposition = Result::DISPOSITION_REJECTED;
    result.rejection_reason_code = rejection_code;
    result.rejection_reason = reason.substr(0, kMaxIdentityBytes);
    sealResult(result);
    return result;
}

std::optional<Digest> loopProofHash(const Request& request, int64_t source_node,
                                    int64_t target_node, int64_t inlier_count,
                                    double fitness_score, double inlier_rmse_m) {
    DigestBuilder builder;
    if (!builder.updateDomain(kLoopProofHashDomain) ||
        !builder.update(request.request_payload_sha256,
                        sizeof(request.request_payload_sha256)) ||
        !appendBigEndian(builder, source_node) || !appendBigEndian(builder, target_node) ||
        !appendBigEndian(builder, inlier_count) ||
        !appendBigEndianDouble(builder, fitness_score) ||
        !appendBigEndianDouble(builder, inlier_rmse_m)) {
        return std::nullopt;
    }
    return builder.finish();
}

std::optional<Digest> optimizedGraphHash(const Request& request, const Result& result) {
    DigestBuilder builder;
    if (!builder.updateDomain(kGraphHashDomain) ||
        !builder.update(request.request_payload_sha256,
                        sizeof(request.request_payload_sha256)) ||
        !appendBigEndian(builder, result.optimized_graph_node_count) ||
        !appendBigEndian(builder, result.optimized_graph_edge_count) ||
        !appendBigEndian(builder, result.optimized_graph_loop_edge_count) ||
        !appendBigEndian(builder, result.optimization_iterations) ||
        !appendBigEndian(builder, result.optimization_converged) ||
        !appendBigEndianDouble(builder, result.optimization_initial_cost) ||
        !appendBigEndianDouble(builder, result.optimization_final_cost)) {
        return std::nullopt;
    }
    for (double value : result.translation_m) {
        if (!appendBigEndianDouble(builder, value)) return std::nullopt;
    }
    for (double value : result.rotation_xyzw) {
        if (!appendBigEndianDouble(builder, value)) return std::nullopt;
    }
    if (!appendBigEndian(builder,result.trajectory_count)) return std::nullopt;
    for (auto stamp:result.trajectory_timestamp_ns) if (!appendBigEndian(builder,stamp)) return std::nullopt;
    for (const auto& pose:result.trajectory_translation_m) for (double value:pose)
        if (!appendBigEndianDouble(builder,value)) return std::nullopt;
    for (const auto& pose:result.trajectory_rotation_xyzw) for (double value:pose)
        if (!appendBigEndianDouble(builder,value)) return std::nullopt;
    return builder.finish();
}

bool graphShapeWithinLimits(const SimplePGO& graph, bool correction_required) {
    const std::size_t nodes = graph.keyPosesSe3().size();
    const std::size_t edges = graph.constraintCountSe3();
    const std::size_t loop_edges = graph.loopConstraintCountSe3();
    if (nodes > static_cast<std::size_t>(Result::MAX_GRAPH_NODES) ||
        edges > static_cast<std::size_t>(Result::MAX_GRAPH_EDGES) ||
        loop_edges > static_cast<std::size_t>(Result::MAX_GRAPH_LOOP_EDGES)) {
        return false;
    }
    if (nodes == 0) return !correction_required && edges == 0 && loop_edges == 0;
    const std::size_t maximum_loops_for_nodes = nodes >= 2 ? nodes - 2 : 0;
    if (loop_edges > maximum_loops_for_nodes || edges != (nodes - 1) + loop_edges) {
        return false;
    }
    return !correction_required || (nodes >= 3 && loop_edges >= 1 && edges >= nodes);
}

bool correctionProofWithinLimits(const Result& result) {
    if (result.trajectory_count!=result.optimized_graph_node_count || result.trajectory_count<3
        || result.trajectory_timestamp_ns.size()!=static_cast<std::size_t>(result.trajectory_count)
        || result.trajectory_translation_m.size()!=static_cast<std::size_t>(result.trajectory_count)
        || result.trajectory_rotation_xyzw.size()!=static_cast<std::size_t>(result.trajectory_count)) return false;
    int64_t previous=-1;
    for (int32_t i=0;i<result.trajectory_count;++i) {
        const auto stamp=result.trajectory_timestamp_ns[i];
        if (stamp<=previous || stamp>result.source_timestamp_ns
            || result.trajectory_translation_m[i].size()!=3
            || result.trajectory_rotation_xyzw[i].size()!=4) return false;
        previous=stamp;
        double norm=0;
        for (double value:result.trajectory_translation_m[i]) if (!std::isfinite(value)) return false;
        for (double value:result.trajectory_rotation_xyzw[i]) { if (!std::isfinite(value)) return false; norm+=value*value; }
        if (std::abs(norm-1.0)>1e-6) return false;
    }
    if (previous!=result.source_timestamp_ns) return false;
    if (result.result_disposition != Result::DISPOSITION_CORRECTION ||
        result.transform_semantics != Result::TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3 ||
        result.optimized_graph_node_count < 3 ||
        result.optimized_graph_node_count > Result::MAX_GRAPH_NODES ||
        result.optimized_graph_edge_count < result.optimized_graph_node_count ||
        result.optimized_graph_edge_count > Result::MAX_GRAPH_EDGES ||
        result.optimized_graph_loop_edge_count < 1 ||
        result.optimized_graph_loop_edge_count > Result::MAX_GRAPH_LOOP_EDGES ||
        result.optimized_graph_loop_edge_count > result.optimized_graph_node_count - 2 ||
        result.optimized_graph_edge_count !=
            (result.optimized_graph_node_count - 1) +
                result.optimized_graph_loop_edge_count ||
        result.loop_source_node_id < 0 || result.loop_target_node_id < 0 ||
        result.loop_source_node_id == result.loop_target_node_id ||
        result.loop_source_node_id >= result.optimized_graph_node_count ||
        result.loop_target_node_id >= result.optimized_graph_node_count ||
        result.loop_inlier_count < 1 ||
        result.loop_inlier_count > Result::MAX_LOOP_INLIERS ||
        result.optimization_iterations < 1 ||
        result.optimization_iterations > Result::MAX_OPTIMIZATION_ITERATIONS ||
        result.optimization_converged != 1 || !std::isfinite(result.fitness_score) ||
        result.fitness_score < 0.0 || !std::isfinite(result.inlier_rmse_m) ||
        result.inlier_rmse_m < 0.0 || !std::isfinite(result.optimization_initial_cost) ||
        result.optimization_initial_cost < 0.0 ||
        !std::isfinite(result.optimization_final_cost) ||
        result.optimization_final_cost < 0.0 ||
        result.optimization_final_cost > result.optimization_initial_cost ||
        !finiteTranslation(result.translation_m) ||
        !finiteQuaternion(result.rotation_xyzw) ||
        result.rejection_reason_code != Result::REJECTION_NONE ||
        !result.rejection_reason.empty()) {
        return false;
    }
    return true;
}

bool sealedCorrectionResultIsValid(const Request& request, const Result& result) {
    if (!correctionProofWithinLimits(result)) return false;
    const auto expected_loop = loopProofHash(
        request, result.loop_source_node_id, result.loop_target_node_id,
        result.loop_inlier_count, result.fitness_score, result.inlier_rmse_m);
    const auto expected_graph = optimizedGraphHash(request, result);
    Result canonical = result;
    std::fill(std::begin(canonical.result_payload_sha256),
              std::end(canonical.result_payload_sha256), 0);
    const auto expected_result = hashGeneratedBody(canonical, kResultHashDomain);
    return expected_loop && digestEquals(*expected_loop, result.loop_proof_sha256) &&
           expected_graph && digestEquals(*expected_graph, result.optimized_graph_sha256) &&
           expected_result && digestEquals(*expected_result, result.result_payload_sha256);
}

std::optional<Result> correctionResult(const Request& request, const RigidPose& correction,
                                       double fitness_score, double inlier_rmse_m,
                                       int64_t source_node, int64_t target_node,
                                       int64_t inlier_count, const SimplePGO& graph,
                                       const Se3OptimizationSummary& optimization) {
    Result result = resultBase(request);
    result.result_disposition = Result::DISPOSITION_CORRECTION;
    result.transform_semantics = Result::TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3;
    result.translation_m[0] = correction.t.x;
    result.translation_m[1] = correction.t.y;
    result.translation_m[2] = correction.t.z;
    result.rotation_xyzw[0] = correction.q.x;
    result.rotation_xyzw[1] = correction.q.y;
    result.rotation_xyzw[2] = correction.q.z;
    result.rotation_xyzw[3] = correction.q.w;
    result.fitness_score = fitness_score;
    result.inlier_rmse_m = inlier_rmse_m;
    result.loop_source_node_id = source_node;
    result.loop_target_node_id = target_node;
    result.loop_inlier_count = inlier_count;
    result.optimized_graph_node_count = static_cast<int64_t>(graph.keyPosesSe3().size());
    result.trajectory_count=static_cast<int32_t>(graph.keyPosesSe3().size());
    for (const auto& node:graph.keyPosesSe3()) {
        result.trajectory_timestamp_ns.push_back(static_cast<int64_t>(node.local.sec)*1'000'000'000+node.local.nsec);
        result.trajectory_translation_m.push_back({node.global.t.x,node.global.t.y,node.global.t.z});
        result.trajectory_rotation_xyzw.push_back({node.global.q.x,node.global.q.y,node.global.q.z,node.global.q.w});
    }
    result.optimized_graph_edge_count = static_cast<int64_t>(graph.constraintCountSe3());
    result.optimized_graph_loop_edge_count =
        static_cast<int64_t>(graph.loopConstraintCountSe3());
    result.optimization_iterations = optimization.iterations;
    result.optimization_converged = optimization.converged ? 1 : 0;
    result.optimization_initial_cost = optimization.cost_before;
    result.optimization_final_cost = optimization.cost_after;
    result.rejection_reason_code = Result::REJECTION_NONE;
    if (!correctionProofWithinLimits(result)) return std::nullopt;
    const auto loop_hash = loopProofHash(request, source_node, target_node, inlier_count,
                                         fitness_score, inlier_rmse_m);
    const auto graph_hash = optimizedGraphHash(request, result);
    if (!loop_hash || !graph_hash) return std::nullopt;
    copyDigest(*loop_hash, result.loop_proof_sha256);
    copyDigest(*graph_hash, result.optimized_graph_sha256);
    if (!sealResult(result)) return std::nullopt;
    if (!sealedCorrectionResultIsValid(request, result)) return std::nullopt;
    return result;
}

Status ownerStatus(const OnlinePgoOptions& options, int64_t status_sequence,
                   int32_t event_kind, int32_t health_state, int32_t terminal_state,
                   int32_t reason_code, const std::string& reason) {
    Status status{};
    status.schema_version = Status::SCHEMA_VERSION_V2;
    status.backend = Status::BACKEND_NATIVE_PGO;
    status.runtime_generation = options.runtime_generation;
    status.backend_generation = options.backend_generation;
    status.status_sequence = status_sequence;
    status.status_timestamp_ns = timestampNowNs();
    status.event_kind = event_kind;
    status.health_state = health_state;
    status.terminal_state = terminal_state;
    status.reason_code = reason_code;
    status.reason = reason.substr(0, kMaxIdentityBytes);
    sealStatus(status);
    return status;
}

Status requestStatus(const Request& request, int64_t status_sequence, int32_t event_kind,
                     int32_t health_state, int32_t terminal_state, int32_t cancel_state,
                     int32_t reason_code, const std::string& reason) {
    Status status{};
    status.schema_version = Status::SCHEMA_VERSION_V2;
    status.backend = Status::BACKEND_NATIVE_PGO;
    status.runtime_generation = request.runtime_generation;
    status.backend_generation = request.backend_generation;
    status.status_sequence = status_sequence;
    status.status_timestamp_ns = timestampNowNs();
    status.identity_present = 1;
    copyRequestIdentity(request, status);
    status.event_kind = event_kind;
    status.health_state = health_state;
    status.terminal_state = terminal_state;
    status.cancel_state = cancel_state;
    status.reason_code = reason_code;
    status.reason = reason.substr(0, kMaxIdentityBytes);
    sealStatus(status);
    return status;
}

bool sameRequestIdentity(const Status& status, const Request& request) {
    return status.identity_present == 1 &&
           status.runtime_generation == request.runtime_generation &&
           status.backend_generation == request.backend_generation &&
           status.request_sequence == request.request_sequence &&
           status.mapping_session_id == request.mapping_session_id &&
           status.mapping_session_generation == request.mapping_session_generation &&
           status.map_frame_generation == request.map_frame_generation &&
           equalBytes(status.mapping_commit_boot_nonce, request.mapping_commit_boot_nonce) &&
           status.mapping_commit_sequence == request.mapping_commit_sequence &&
           status.occupancy_revision == request.occupancy_revision &&
           status.cloud_revision == request.cloud_revision &&
           status.committed_map_source_timestamp_ns ==
               request.committed_map_source_timestamp_ns &&
           status.registered_cloud_source_sequence ==
               request.registered_cloud_source_sequence &&
           status.registered_cloud_source_timestamp_ns ==
               request.registered_cloud_source_timestamp_ns &&
           status.registered_body_point_count == request.registered_body_point_count &&
           equalBytes(status.registered_cloud_envelope_sha256,
                      request.registered_cloud_envelope_sha256) &&
           equalBytes(status.registered_body_points_sha256,
                      request.registered_body_points_sha256) &&
           status.correlated_odom_source_sequence == request.correlated_odom_source_sequence &&
           status.correlated_odom_source_timestamp_ns ==
               request.correlated_odom_source_timestamp_ns &&
           equalBytes(status.frozen_odom_pose_sha256, request.frozen_odom_pose_sha256) &&
           status.committed_map_width == request.committed_map_width &&
           status.committed_map_height == request.committed_map_height &&
           status.committed_map_resolution_m == request.committed_map_resolution_m &&
           status.committed_map_origin_x_m == request.committed_map_origin_x_m &&
           status.committed_map_origin_y_m == request.committed_map_origin_y_m &&
           equalBytes(status.committed_map_payload_sha256,
                      request.committed_map_payload_sha256) &&
           equalBytes(status.committed_map_boundary_sha256,
                      request.committed_map_boundary_sha256) &&
           status.source_timestamp_ns == request.source_timestamp_ns &&
           status.map_frame_id == request.map_frame_id &&
           status.odom_frame_id == request.odom_frame_id &&
           status.body_frame_id == request.body_frame_id &&
           status.points_frame_id == request.points_frame_id &&
           equalBytes(status.correction_lease_boot_nonce,
                      request.correction_lease_boot_nonce) &&
           status.correction_lease_generation == request.correction_lease_generation &&
           equalBytes(status.request_payload_sha256, request.request_payload_sha256);
}

bool sameRequestIdentity(const Request& left, const Request& right) {
    return left.schema_version == right.schema_version && left.backend == right.backend &&
           left.runtime_generation == right.runtime_generation &&
           left.backend_generation == right.backend_generation &&
           left.request_sequence == right.request_sequence &&
           left.mapping_session_id == right.mapping_session_id &&
           left.mapping_session_generation == right.mapping_session_generation &&
           left.map_frame_generation == right.map_frame_generation &&
           equalBytes(left.mapping_commit_boot_nonce, right.mapping_commit_boot_nonce) &&
           left.mapping_commit_sequence == right.mapping_commit_sequence &&
           left.occupancy_revision == right.occupancy_revision &&
           left.cloud_revision == right.cloud_revision &&
           left.committed_map_source_timestamp_ns ==
               right.committed_map_source_timestamp_ns &&
           left.registered_cloud_source_sequence ==
               right.registered_cloud_source_sequence &&
           left.registered_cloud_source_timestamp_ns ==
               right.registered_cloud_source_timestamp_ns &&
           left.registered_body_point_count == right.registered_body_point_count &&
           equalBytes(left.registered_cloud_envelope_sha256,
                      right.registered_cloud_envelope_sha256) &&
           equalBytes(left.registered_body_points_sha256,
                      right.registered_body_points_sha256) &&
           left.correlated_odom_source_sequence ==
               right.correlated_odom_source_sequence &&
           left.correlated_odom_source_timestamp_ns ==
               right.correlated_odom_source_timestamp_ns &&
           equalBytes(left.frozen_odom_pose_sha256, right.frozen_odom_pose_sha256) &&
           left.committed_map_width == right.committed_map_width &&
           left.committed_map_height == right.committed_map_height &&
           left.committed_map_resolution_m == right.committed_map_resolution_m &&
           left.committed_map_origin_x_m == right.committed_map_origin_x_m &&
           left.committed_map_origin_y_m == right.committed_map_origin_y_m &&
           equalBytes(left.committed_map_payload_sha256,
                      right.committed_map_payload_sha256) &&
           equalBytes(left.committed_map_boundary_sha256,
                      right.committed_map_boundary_sha256) &&
           left.source_timestamp_ns == right.source_timestamp_ns &&
           left.map_frame_id == right.map_frame_id &&
           left.odom_frame_id == right.odom_frame_id &&
           left.body_frame_id == right.body_frame_id &&
           left.points_frame_id == right.points_frame_id &&
           equalBytes(left.correction_lease_boot_nonce,
                      right.correction_lease_boot_nonce) &&
           left.correction_lease_generation == right.correction_lease_generation &&
           equalBytes(left.request_payload_sha256, right.request_payload_sha256);
}

bool validateCancelMessage(const Status& status, const OnlinePgoOptions& options) {
    if (status.schema_version != Status::SCHEMA_VERSION_V2 ||
        status.backend != Status::BACKEND_NATIVE_PGO ||
        status.runtime_generation != options.runtime_generation ||
        status.backend_generation != options.backend_generation || status.status_sequence <= 0 ||
        status.status_timestamp_ns < 0 || status.identity_present != 1 ||
        status.event_kind != Status::EVENT_CANCEL_REQUESTED ||
        status.health_state != Status::HEALTH_UNKNOWN ||
        status.terminal_state != Status::TERMINAL_NONE ||
        status.cancel_state != Status::CANCEL_REQUESTED ||
        status.reason_code != Status::REASON_CANCEL_REQUESTED ||
        !validIdentityString(status.mapping_session_id) ||
        !validIdentityString(status.map_frame_id) ||
        !validIdentityString(status.odom_frame_id) ||
        !validIdentityString(status.body_frame_id) ||
        !validIdentityString(status.points_frame_id) ||
        status.reason != "cancel_requested") {
        return false;
    }
    Status canonical = status;
    std::fill(std::begin(canonical.status_payload_sha256),
              std::end(canonical.status_payload_sha256), 0);
    const auto expected = hashGeneratedBody(canonical, kStatusHashDomain);
    return expected && digestEquals(*expected, status.status_payload_sha256);
}

}  // namespace

namespace {

Eigen::Isometry3d requestOdomPose(const Request& request) {
    Eigen::Quaterniond rotation(request.frozen_odom_rotation_xyzw[3],
                                request.frozen_odom_rotation_xyzw[0],
                                request.frozen_odom_rotation_xyzw[1],
                                request.frozen_odom_rotation_xyzw[2]);
    rotation.normalize();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = rotation.toRotationMatrix();
    pose.translation() = Eigen::Vector3d(request.frozen_odom_translation_m[0],
                                         request.frozen_odom_translation_m[1],
                                         request.frozen_odom_translation_m[2]);
    return pose;
}

RigidPose rigidPoseFromIsometry(const Eigen::Isometry3d& pose) {
    Eigen::Quaterniond rotation(pose.linear());
    rotation.normalize();
    return {{pose.translation().x(), pose.translation().y(), pose.translation().z()},
            {rotation.x(), rotation.y(), rotation.z(), rotation.w()}};
}

Eigen::Isometry3d isometryFromRigidPose(const RigidPose& pose) {
    Eigen::Quaterniond rotation(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
    rotation.normalize();
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = rotation.toRotationMatrix();
    result.translation() = Eigen::Vector3d(pose.t.x, pose.t.y, pose.t.z);
    return result;
}

PointCloud::Ptr deterministicPointCap(const PointCloud::ConstPtr& input, std::size_t limit) {
    PointCloud::Ptr output(new PointCloud);
    if (input->size() <= limit) {
        *output = *input;
        return output;
    }
    output->reserve(limit);
    for (std::size_t index = 0; index < limit; ++index) {
        const std::size_t source = index * input->size() / limit;
        output->push_back((*input)[source]);
    }
    output->width = static_cast<uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

PointCloud::Ptr voxelizedCloud(const PointCloud::ConstPtr& input, double resolution_m,
                               std::size_t limit) {
    PointCloud::Ptr filtered(new PointCloud);
    pcl::VoxelGrid<Point> voxel;
    const float leaf = static_cast<float>(resolution_m);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(input);
    voxel.filter(*filtered);
    return deterministicPointCap(filtered, limit);
}

PointCloud::Ptr requestBodyCloud(const Request& request, double resolution_m) {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->reserve(static_cast<std::size_t>(request.registered_body_point_count));
    for (std::size_t point_index = 0;
         point_index < static_cast<std::size_t>(request.registered_body_point_count);
         ++point_index) {
        const std::size_t coordinate = point_index * 3;
        cloud->emplace_back(static_cast<float>(request.registered_body_points_xyz[coordinate]),
                            static_cast<float>(request.registered_body_points_xyz[coordinate + 1]),
                            static_cast<float>(request.registered_body_points_xyz[coordinate + 2]));
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return voxelizedCloud(cloud, resolution_m, kMaxIcpCloudPoints);
}

struct StoredKeyframe {
    PointCloud::Ptr body_cloud;
    int64_t source_timestamp_ns{0};
};

struct GraphState {
    explicit GraphState(const Config& config) : graph(config) {}

    SimplePGO graph;
    std::vector<StoredKeyframe> keyframes;
    std::vector<std::pair<std::size_t, std::size_t>> loop_pairs;
    std::optional<int64_t> last_loop_timestamp_ns;
    bool initialized{false};
    std::string mapping_session_id;
    int64_t mapping_session_generation{0};
    int64_t map_frame_generation{0};
    std::array<uint8_t, 16> mapping_commit_boot_nonce{};
    int64_t last_mapping_commit_sequence{0};
    int64_t last_occupancy_revision{0};
    int64_t last_cloud_revision{0};
    int64_t last_registered_cloud_sequence{0};
    int64_t last_source_timestamp_ns{-1};
    std::string map_frame_id;
    std::string odom_frame_id;
    std::string body_frame_id;
    std::array<uint8_t, 16> correction_lease_boot_nonce{};
    int64_t correction_lease_generation{0};
};

struct RetainedOdomFence {
    bool initialized{false};
    std::string mapping_session_id;
    int64_t mapping_session_generation{0};
    int64_t map_frame_generation{0};
    std::array<uint8_t, 16> mapping_commit_boot_nonce{};
    std::string map_frame_id;
    std::string odom_frame_id;
    std::string body_frame_id;
    std::array<uint8_t, 16> correction_lease_boot_nonce{};
    int64_t correction_lease_generation{0};
    int64_t source_sequence{0};
    int64_t source_timestamp_ns{0};
    std::array<uint8_t, 32> pose_sha256{};
};

template <std::size_t N>
bool arrayEquals(const std::array<uint8_t, N>& left, const uint8_t (&right)[N]) {
    return std::equal(left.begin(), left.end(), std::begin(right));
}

template <std::size_t N>
void copyArray(const uint8_t (&source)[N], std::array<uint8_t, N>& target) {
    std::copy(std::begin(source), std::end(source), target.begin());
}

std::optional<std::string> validateAndAdvanceRetainedOdomFence(
    RetainedOdomFence& fence, const Request& request) {
    const bool first_session = !fence.initialized;
    const bool newer_session = fence.initialized &&
                               request.mapping_session_generation >
                                   fence.mapping_session_generation;
    if (fence.initialized &&
        request.mapping_session_generation < fence.mapping_session_generation) {
        return "mapping session generation regressed";
    }
    if (!first_session && !newer_session &&
        (request.mapping_session_id != fence.mapping_session_id ||
         request.mapping_session_generation != fence.mapping_session_generation ||
         request.map_frame_generation != fence.map_frame_generation ||
         !arrayEquals(fence.mapping_commit_boot_nonce,
                      request.mapping_commit_boot_nonce) ||
         request.map_frame_id != fence.map_frame_id ||
         request.odom_frame_id != fence.odom_frame_id ||
         request.body_frame_id != fence.body_frame_id ||
         !arrayEquals(fence.correction_lease_boot_nonce,
                      request.correction_lease_boot_nonce) ||
         request.correction_lease_generation != fence.correction_lease_generation)) {
        return "same-generation session, frame, commit, or lease identity changed";
    }
    if (!first_session && !newer_session) {
        if (request.correlated_odom_source_sequence < fence.source_sequence ||
            request.correlated_odom_source_timestamp_ns < fence.source_timestamp_ns) {
            return "frozen odom sequence or timestamp regressed";
        }
        if (request.correlated_odom_source_sequence == fence.source_sequence &&
            (request.correlated_odom_source_timestamp_ns != fence.source_timestamp_ns ||
             !arrayEquals(fence.pose_sha256, request.frozen_odom_pose_sha256))) {
            return "same frozen odom sequence changed timestamp or pose hash";
        }
    }
    if (first_session || newer_session) {
        fence.initialized = true;
        fence.mapping_session_id = request.mapping_session_id;
        fence.mapping_session_generation = request.mapping_session_generation;
        fence.map_frame_generation = request.map_frame_generation;
        copyArray(request.mapping_commit_boot_nonce, fence.mapping_commit_boot_nonce);
        fence.map_frame_id = request.map_frame_id;
        fence.odom_frame_id = request.odom_frame_id;
        fence.body_frame_id = request.body_frame_id;
        copyArray(request.correction_lease_boot_nonce,
                  fence.correction_lease_boot_nonce);
        fence.correction_lease_generation = request.correction_lease_generation;
    }
    fence.source_sequence = request.correlated_odom_source_sequence;
    fence.source_timestamp_ns = request.correlated_odom_source_timestamp_ns;
    copyArray(request.frozen_odom_pose_sha256, fence.pose_sha256);
    return std::nullopt;
}

std::optional<std::string> validateSessionBoundary(const GraphState& state,
                                                   const Request& request) {
    const bool first_session = !state.initialized;
    const bool newer_session = state.initialized &&
                               request.mapping_session_generation >
                                   state.mapping_session_generation;
    if (state.initialized &&
        request.mapping_session_generation < state.mapping_session_generation) {
        return "mapping session generation regressed";
    }
    if (!first_session && !newer_session &&
        (request.mapping_session_id != state.mapping_session_id ||
               request.mapping_session_generation != state.mapping_session_generation ||
               request.map_frame_generation != state.map_frame_generation ||
               !arrayEquals(state.mapping_commit_boot_nonce,
                            request.mapping_commit_boot_nonce) ||
               request.map_frame_id != state.map_frame_id ||
               request.odom_frame_id != state.odom_frame_id ||
               request.body_frame_id != state.body_frame_id ||
               !arrayEquals(state.correction_lease_boot_nonce,
                            request.correction_lease_boot_nonce) ||
               request.correction_lease_generation != state.correction_lease_generation)) {
        return "same-generation session, frame, commit, or lease identity changed";
    }

    const int64_t last_mapping_commit_sequence =
        first_session || newer_session ? 0 : state.last_mapping_commit_sequence;
    const int64_t last_occupancy_revision =
        first_session || newer_session ? 0 : state.last_occupancy_revision;
    const int64_t last_cloud_revision =
        first_session || newer_session ? 0 : state.last_cloud_revision;
    const int64_t last_registered_cloud_sequence =
        first_session || newer_session ? 0 : state.last_registered_cloud_sequence;
    const int64_t last_source_timestamp_ns =
        first_session || newer_session ? -1 : state.last_source_timestamp_ns;
    if (request.mapping_commit_sequence <= last_mapping_commit_sequence ||
        request.occupancy_revision <= last_occupancy_revision ||
        request.cloud_revision <= last_cloud_revision ||
        request.registered_cloud_source_sequence <= last_registered_cloud_sequence ||
        request.source_timestamp_ns <= last_source_timestamp_ns) {
        return "request revision, source, or mapping commit did not advance";
    }

    return std::nullopt;
}

std::optional<std::string> advanceSessionBoundary(GraphState& state, const Request& request,
                                                  const Config& graph_config) {
    if (const auto validation = validateSessionBoundary(state, request)) {
        return validation;
    }
    const bool first_session = !state.initialized;
    const bool newer_session = state.initialized &&
                               request.mapping_session_generation >
                                   state.mapping_session_generation;
    if (first_session || newer_session) {
        state = GraphState(graph_config);
        state.initialized = true;
        state.mapping_session_id = request.mapping_session_id;
        state.mapping_session_generation = request.mapping_session_generation;
        state.map_frame_generation = request.map_frame_generation;
        copyArray(request.mapping_commit_boot_nonce, state.mapping_commit_boot_nonce);
        state.map_frame_id = request.map_frame_id;
        state.odom_frame_id = request.odom_frame_id;
        state.body_frame_id = request.body_frame_id;
        copyArray(request.correction_lease_boot_nonce, state.correction_lease_boot_nonce);
        state.correction_lease_generation = request.correction_lease_generation;
    }
    state.last_mapping_commit_sequence = request.mapping_commit_sequence;
    state.last_occupancy_revision = request.occupancy_revision;
    state.last_cloud_revision = request.cloud_revision;
    state.last_registered_cloud_sequence = request.registered_cloud_source_sequence;
    state.last_source_timestamp_ns = request.source_timestamp_ns;
    return std::nullopt;
}

PointCloud::Ptr globalCloudForNode(const GraphState& state, std::size_t index) {
    PointCloud::Ptr transformed(new PointCloud);
    const Eigen::Matrix4f transform =
        isometryFromRigidPose(state.graph.keyPosesSe3()[index].global).matrix().cast<float>();
    pcl::transformPointCloud(*state.keyframes[index].body_cloud, *transformed, transform);
    return transformed;
}

PointCloud::Ptr loopTargetSubmap(const GraphState& state, std::size_t target_index,
                                 std::size_t current_index, int half_range,
                                 double resolution_m) {
    PointCloud::Ptr accumulated(new PointCloud);
    const std::size_t begin = target_index > static_cast<std::size_t>(half_range)
                                  ? target_index - static_cast<std::size_t>(half_range)
                                  : 0;
    const std::size_t end = std::min(current_index - 1,
                                     target_index + static_cast<std::size_t>(half_range));
    for (std::size_t index = begin; index <= end; ++index) {
        const PointCloud::Ptr transformed = globalCloudForNode(state, index);
        *accumulated += *transformed;
        if (accumulated->size() > kMaxSubmapPoints * 2) {
            accumulated = deterministicPointCap(accumulated, kMaxSubmapPoints);
        }
    }
    return voxelizedCloud(accumulated, resolution_m, kMaxSubmapPoints);
}

std::optional<std::size_t> findLoopTarget(const GraphState& state,
                                          const OnlinePgoOptions& options) {
    const std::size_t node_count = state.graph.keyPosesSe3().size();
    if (node_count < 3) return std::nullopt;
    const std::size_t current_index = node_count - 1;
    const auto& current = state.graph.keyPosesSe3()[current_index];
    if (state.last_loop_timestamp_ns &&
        static_cast<double>(state.keyframes.back().source_timestamp_ns -
                            *state.last_loop_timestamp_ns) <
            options.min_loop_detect_duration_s * 1.0e9) {
        return std::nullopt;
    }
    std::optional<std::size_t> selected;
    double selected_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index + 1 < current_index; ++index) {
        const auto& candidate = state.graph.keyPosesSe3()[index];
        if (std::abs(current.local.second - candidate.local.second) <
            options.loop_time_threshold_s) {
            continue;
        }
        if (std::find(state.loop_pairs.begin(), state.loop_pairs.end(),
                      std::pair<std::size_t, std::size_t>{current_index, index}) !=
            state.loop_pairs.end()) {
            continue;
        }
        const double dx = current.global.t.x - candidate.global.t.x;
        const double dy = current.global.t.y - candidate.global.t.y;
        const double dz = current.global.t.z - candidate.global.t.z;
        const double distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (std::isfinite(distance_m) && distance_m <= options.loop_search_radius_m &&
            distance_m < selected_distance) {
            selected = index;
            selected_distance = distance_m;
        }
    }
    return selected;
}

enum class IcpDisposition { NoCandidate, Correction, Cancelled, Failure };

struct IcpResult {
    IcpDisposition disposition{IcpDisposition::NoCandidate};
    Eigen::Matrix4f source_correction{Eigen::Matrix4f::Identity()};
    double fitness_score{0.0};
    double inlier_rmse_m{0.0};
    int64_t inlier_count{0};
    std::string error;
};

IcpResult registerLoop(const PointCloud::ConstPtr& source, const PointCloud::ConstPtr& target,
                       double score_threshold,
                       const std::shared_ptr<std::atomic<bool>>& cancelled) {
    if (source->size() > static_cast<std::size_t>(Result::MAX_LOOP_INLIERS) ||
        target->size() > kMaxSubmapPoints) {
        IcpResult result;
        result.disposition = IcpDisposition::Failure;
        result.error = "ICP cloud exceeds the frozen V2 proof limits";
        return result;
    }
    if (source->size() < kMinIcpInliers || target->size() < kMinIcpInliers) {
        return {};
    }
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(1);
    icp.setMaxCorrespondenceDistance(kIcpMaxCorrespondenceDistanceM);
    icp.setTransformationEpsilon(1.0e-7);
    icp.setEuclideanFitnessEpsilon(1.0e-7);
    icp.setRANSACIterations(0);
    Eigen::Matrix4f estimate = Eigen::Matrix4f::Identity();
    bool converged = false;
    for (int iteration = 0; iteration < kIcpIterationLimit; ++iteration) {
        if (cancelled->load(std::memory_order_acquire)) {
            IcpResult result;
            result.disposition = IcpDisposition::Cancelled;
            return result;
        }
        PointCloud aligned;
        icp.align(aligned, estimate);
        const Eigen::Matrix4f next = icp.getFinalTransformation();
        if (!next.allFinite()) {
            IcpResult result;
            result.disposition = IcpDisposition::Failure;
            result.error = "ICP produced a non-finite transform";
            return result;
        }
        const Eigen::Matrix4f delta = next * estimate.inverse();
        const double translation_delta = delta.block<3, 1>(0, 3).cast<double>().norm();
        const Eigen::Matrix3d delta_rotation = delta.block<3, 3>(0, 0).cast<double>();
        const Eigen::AngleAxisd angle_axis(delta_rotation);
        estimate = next;
        if (translation_delta <= 1.0e-5 && std::abs(angle_axis.angle()) <= 1.0e-5) {
            converged = true;
            break;
        }
    }
    if (cancelled->load(std::memory_order_acquire)) {
        IcpResult result;
        result.disposition = IcpDisposition::Cancelled;
        return result;
    }
    const double fitness = icp.getFitnessScore(kIcpMaxCorrespondenceDistanceM);
    if ((!converged && !icp.hasConverged()) || !std::isfinite(fitness) ||
        fitness > score_threshold) {
        return {};
    }
    PointCloud::Ptr transformed(new PointCloud);
    pcl::transformPointCloud(*source, *transformed, estimate);
    pcl::KdTreeFLANN<Point> nearest;
    nearest.setInputCloud(target);
    long double squared_error = 0.0;
    int64_t inlier_count = 0;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    const double max_squared =
        kIcpMaxCorrespondenceDistanceM * kIcpMaxCorrespondenceDistanceM;
    for (const Point& point : *transformed) {
        if (cancelled->load(std::memory_order_acquire)) {
            IcpResult result;
            result.disposition = IcpDisposition::Cancelled;
            return result;
        }
        if (nearest.nearestKSearch(point, 1, indices, distances) == 1 &&
            distances[0] <= max_squared) {
            squared_error += distances[0];
            ++inlier_count;
            if (inlier_count > Result::MAX_LOOP_INLIERS) {
                IcpResult result;
                result.disposition = IcpDisposition::Failure;
                result.error = "ICP inlier proof exceeds the frozen V2 limit";
                return result;
            }
        }
    }
    if (inlier_count < static_cast<int64_t>(kMinIcpInliers)) return {};
    const double rmse = std::sqrt(static_cast<double>(squared_error / inlier_count));
    if (!std::isfinite(rmse)) {
        IcpResult result;
        result.disposition = IcpDisposition::Failure;
        result.error = "ICP inlier proof is non-finite";
        return result;
    }
    IcpResult result;
    result.disposition = IcpDisposition::Correction;
    result.source_correction = estimate;
    result.fitness_score = fitness;
    result.inlier_rmse_m = rmse;
    result.inlier_count = inlier_count;
    return result;
}

enum class TerminalKind { Succeeded, Rejected, Cancelled };

struct ProcessedRequest {
    ProcessedRequest(Result result_value, TerminalKind terminal_kind_value,
                     int32_t reason_code_value, std::string reason_value)
        : result(std::move(result_value)),
          terminal_kind(terminal_kind_value),
          reason_code(reason_code_value),
          reason(std::move(reason_value)) {}

    Result result;
    TerminalKind terminal_kind{TerminalKind::Succeeded};
    int32_t reason_code{Status::REASON_NONE};
    std::string reason;
    std::optional<GraphState> proposed_state;
    bool consumes_request_sequence{true};
};

ProcessedRequest cancelledRequest(const Request& request) {
    return {rejectedResult(request, Result::REJECTION_CANCELLED, "cancelled"),
            TerminalKind::Cancelled, Status::REASON_CANCELLED, "cancelled"};
}

ProcessedRequest shutdownInterruptedRequest(const Request& request) {
    return {rejectedResult(request, Result::REJECTION_RESOURCE_LIMIT,
                           "shutdown_requested"),
            TerminalKind::Rejected, Status::REASON_INVALID_REQUEST,
            "shutdown_requested"};
}

ProcessedRequest processRequest(const GraphState& state, int64_t last_request_sequence,
                                const Request& request, const OnlinePgoOptions& options,
                                const std::shared_ptr<std::atomic<bool>>& cancelled) {
    if (request.request_sequence <= last_request_sequence) {
        ProcessedRequest rejected{
            rejectedResult(request, Result::REJECTION_INVALID_GRAPH,
                           "request sequence did not advance"),
            TerminalKind::Rejected, Status::REASON_INVALID_REQUEST,
            "request sequence did not advance"};
        rejected.consumes_request_sequence = false;
        return rejected;
    }
    if (cancelled->load()) return cancelledRequest(request);

    if (const auto boundary_error = validateSessionBoundary(state, request)) {
        return {rejectedResult(request, Result::REJECTION_INVALID_GRAPH, *boundary_error),
                TerminalKind::Rejected, Status::REASON_INVALID_REQUEST, *boundary_error};
    }
    if (cancelled->load()) return cancelledRequest(request);
    GraphState trial = state;
    if (const auto boundary_error = advanceSessionBoundary(trial, request, options.graph)) {
        return {rejectedResult(request, Result::REJECTION_INVALID_GRAPH, *boundary_error),
                TerminalKind::Rejected, Status::REASON_INVALID_REQUEST, *boundary_error};
    }
    if (!graphShapeWithinLimits(trial.graph, false)) {
        return {rejectedResult(request, Result::REJECTION_RESOURCE_LIMIT,
                               "pose graph violates the frozen V2 limits"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "pose graph violates the frozen V2 limits"};
    }
    if (trial.graph.keyPosesSe3().size() >= kMaxGraphNodes) {
        return {rejectedResult(request, Result::REJECTION_RESOURCE_LIMIT,
                               "pose graph node limit reached"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "pose graph node limit reached"};
    }
    const int64_t seconds = request.source_timestamp_ns / 1'000'000'000LL;
    const int64_t nanoseconds = request.source_timestamp_ns % 1'000'000'000LL;
    if (seconds > std::numeric_limits<int32_t>::max()) {
        return {rejectedResult(request, Result::REJECTION_INVALID_GRAPH,
                               "source timestamp exceeds pose graph time range"),
                TerminalKind::Rejected, Status::REASON_INVALID_REQUEST,
                "source timestamp exceeds pose graph time range"};
    }
    PoseWithTimeSe3 pose;
    pose.pose = rigidPoseFromIsometry(requestOdomPose(request));
    pose.setTime(static_cast<int32_t>(seconds), static_cast<uint32_t>(nanoseconds));
    if (!trial.graph.isKeyPoseSe3(pose)) {
        ProcessedRequest completed{noCandidateResult(request), TerminalKind::Succeeded,
                                   Status::REASON_NONE, "not a keyframe"};
        completed.proposed_state = std::move(trial);
        return completed;
    }
    if (!trial.graph.addKeyPoseSe3(pose)) {
        return {rejectedResult(request, Result::REJECTION_INVALID_GRAPH,
                               "pose graph rejected keyframe"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "pose graph rejected keyframe"};
    }
    if (!graphShapeWithinLimits(trial.graph, false)) {
        return {rejectedResult(request, Result::REJECTION_RESOURCE_LIMIT,
                               "keyframe exceeds the frozen V2 graph limits"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "keyframe exceeds the frozen V2 graph limits"};
    }
    PointCloud::Ptr body_cloud = requestBodyCloud(request, options.submap_resolution_m);
    if (body_cloud->size() < kMinIcpInliers ||
        body_cloud->size() > static_cast<std::size_t>(Result::MAX_LOOP_INLIERS)) {
        return {rejectedResult(request, Result::REJECTION_RESOURCE_LIMIT,
                               "registered cloud has too few filtered points"),
                TerminalKind::Rejected, Status::REASON_INVALID_REQUEST,
                "registered cloud has too few filtered points"};
    }
    trial.keyframes.push_back({std::move(body_cloud), request.source_timestamp_ns});
    if (cancelled->load()) return cancelledRequest(request);

    const auto loop_target = findLoopTarget(trial, options);
    if (!loop_target) {
        ProcessedRequest completed{noCandidateResult(request), TerminalKind::Succeeded,
                                   Status::REASON_NONE, "no loop candidate"};
        completed.proposed_state = std::move(trial);
        return completed;
    }
    const std::size_t current_index = trial.graph.keyPosesSe3().size() - 1;
    PointCloud::Ptr source = globalCloudForNode(trial, current_index);
    source = voxelizedCloud(source, options.submap_resolution_m, kMaxIcpCloudPoints);
    PointCloud::Ptr target = loopTargetSubmap(
        trial, *loop_target, current_index, options.loop_submap_half_range,
        options.submap_resolution_m);
    const IcpResult registration =
        registerLoop(source, target, options.loop_score_threshold, cancelled);
    if (registration.disposition == IcpDisposition::Cancelled) {
        return cancelledRequest(request);
    }
    if (registration.disposition == IcpDisposition::Failure) {
        return {rejectedResult(request, Result::REJECTION_NUMERIC_FAILURE,
                               registration.error),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                registration.error};
    }
    if (registration.disposition == IcpDisposition::NoCandidate) {
        ProcessedRequest completed{noCandidateResult(request), TerminalKind::Succeeded,
                                   Status::REASON_NONE, "ICP did not prove a loop"};
        completed.proposed_state = std::move(trial);
        return completed;
    }
    if (cancelled->load()) return cancelledRequest(request);

    const Eigen::Isometry3d current_global =
        isometryFromRigidPose(trial.graph.keyPosesSe3()[current_index].global);
    const Eigen::Isometry3d target_global =
        isometryFromRigidPose(trial.graph.keyPosesSe3()[*loop_target].global);
    Eigen::Isometry3d icp_correction = Eigen::Isometry3d::Identity();
    icp_correction.matrix() = registration.source_correction.cast<double>();
    const Eigen::Isometry3d refined_current = icp_correction * current_global;
    const RigidPose current_from_target =
        rigidPoseFromIsometry(target_global.inverse() * refined_current);
    const double loop_weight =
        std::clamp(1.0 / std::max(registration.fitness_score, 1.0e-6), 1.0, 100.0);
    if (!trial.graph.addLoopPairSe3(*loop_target, current_index, current_from_target,
                                    loop_weight)) {
        return {rejectedResult(request, Result::REJECTION_INVALID_GRAPH,
                               "pose graph rejected ICP loop edge"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "pose graph rejected ICP loop edge"};
    }
    if (!graphShapeWithinLimits(trial.graph, true)) {
        return {rejectedResult(request, Result::REJECTION_RESOURCE_LIMIT,
                               "loop edge exceeds the frozen V2 graph limits"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "loop edge exceeds the frozen V2 graph limits"};
    }
    const Se3OptimizationSummary optimization =
        trial.graph.smoothAndUpdateSe3(*cancelled);
    if (cancelled->load()) return cancelledRequest(request);
    constexpr double zero_residual_tolerance = 1.0e-18;
    if (optimization.success && optimization.converged &&
        std::isfinite(optimization.cost_before) &&
        std::isfinite(optimization.cost_after) &&
        optimization.cost_before <= zero_residual_tolerance &&
        optimization.cost_after <= zero_residual_tolerance) {
        trial.loop_pairs.emplace_back(current_index, *loop_target);
        trial.last_loop_timestamp_ns = request.source_timestamp_ns;
        ProcessedRequest completed{noCandidateResult(request), TerminalKind::Succeeded,
                                   Status::REASON_NONE, "zero residual loop"};
        completed.proposed_state = std::move(trial);
        return completed;
    }
    if (!optimization.success || !optimization.converged ||
        !std::isfinite(optimization.cost_before) ||
        !std::isfinite(optimization.cost_after) ||
        !(optimization.cost_after < optimization.cost_before)) {
        return {rejectedResult(request, Result::REJECTION_NUMERIC_FAILURE,
                               "full-SE3 pose graph optimization failed"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "full-SE3 pose graph optimization failed"};
    }
    if (!graphShapeWithinLimits(trial.graph, true) || optimization.iterations < 1 ||
        optimization.iterations > Result::MAX_OPTIMIZATION_ITERATIONS) {
        return {rejectedResult(request, Result::REJECTION_RESOURCE_LIMIT,
                               "optimized proof exceeds the frozen V2 limits"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "optimized proof exceeds the frozen V2 limits"};
    }
    if (cancelled->load()) return cancelledRequest(request);
    const RigidPose correction = trial.graph.offsetSe3();
    const auto result = correctionResult(
        request, correction, registration.fitness_score, registration.inlier_rmse_m,
        static_cast<int64_t>(current_index), static_cast<int64_t>(*loop_target),
        registration.inlier_count, trial.graph, optimization);
    if (!result) {
        return {rejectedResult(request, Result::REJECTION_NUMERIC_FAILURE,
                               "failed to seal correction proof"),
                TerminalKind::Rejected, Status::REASON_OPTIMIZATION_FAILED,
                "failed to seal correction proof"};
    }
    trial.loop_pairs.emplace_back(current_index, *loop_target);
    trial.last_loop_timestamp_ns = request.source_timestamp_ns;
    ProcessedRequest completed{*result, TerminalKind::Succeeded, Status::REASON_NONE,
                               "correction"};
    completed.proposed_state = std::move(trial);
    return completed;
}

}  // namespace

namespace {

struct WorkItem {
    std::shared_ptr<Request> request;
    std::shared_ptr<std::atomic<bool>> cancelled;
};

enum class RequestPhase { Accepted, Running, CancelAcknowledged, Terminal };

struct RequestLedgerEntry {
    explicit RequestLedgerEntry(std::shared_ptr<Request> request_value)
        : request(std::move(request_value)) {}

    std::shared_ptr<Request> request;
    RequestPhase phase{RequestPhase::Accepted};
    bool cancel_ack_emitted{false};
    bool protocol_cancelled{false};
    bool terminal_committed{false};
};

void releaseRequestPayload(Request& request) {
    request.registered_body_points_xyz.clear();
    request.registered_body_points_xyz.shrink_to_fit();
    request.committed_map_occupancy.clear();
    request.committed_map_occupancy.shrink_to_fit();
}

using OutboundMessage = std::variant<Result, Status>;

class OnlinePgoOwnerImpl {
public:
    explicit OnlinePgoOwnerImpl(OnlinePgoOptions options)
        : options_(std::move(options)),
          lcm_(options_.lcm_url),
          request_lcm_channel_(options_.request_channel),
          result_lcm_channel_(options_.result_channel),
          status_lcm_channel_(options_.status_channel),
          cancel_lcm_channel_(options_.cancel_channel),
          graph_state_(options_.graph) {}

    int run(const std::atomic<bool>& running) {
        if (!lcm_.good()) {
            std::cerr << "PGO: online LCM initialization failed\n";
            return 20;
        }
        request_subscription_ = lcm_.subscribe(
            request_lcm_channel_, &OnlinePgoOwnerImpl::handleRequestMessage, this);
        cancel_subscription_ = lcm_.subscribe(
            cancel_lcm_channel_, &OnlinePgoOwnerImpl::handleCancelMessage, this);
        if (request_subscription_ == nullptr || cancel_subscription_ == nullptr) {
            std::cerr << "PGO: online subscription setup failed\n";
            unsubscribe();
            return 21;
        }
        request_subscription_->setQueueCapacity(64);
        cancel_subscription_->setQueueCapacity(64);
        worker_ = std::thread(&OnlinePgoOwnerImpl::workerLoop, this);

        enqueueOwnerStatus(Status::EVENT_STARTING, Status::HEALTH_UNKNOWN,
                           Status::TERMINAL_NONE, Status::REASON_NONE, "");
        enqueueOwnerStatus(Status::EVENT_READY, Status::HEALTH_HEALTHY,
                           Status::TERMINAL_NONE, Status::REASON_NONE, "");
        drainOutbound();
        auto next_health = std::chrono::steady_clock::now() + kHealthPeriod;
        while (running.load() && !fatal_.load()) {
            const int handled = lcm_.handleTimeout(20);
            if (handled < 0) {
                if (!running.load()) break;
                fatal_.store(true);
                break;
            }
            drainOutbound();
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_health) {
                enqueueOwnerStatus(Status::EVENT_HEALTH, Status::HEALTH_HEALTHY,
                                   Status::TERMINAL_NONE, Status::REASON_NONE, "");
                drainOutbound();
                next_health = now + kHealthPeriod;
            }
        }

        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            stop_requested_ = true;
            if (active_request_ && active_cancel_) {
                if (!current_request_ ||
                    current_request_->request->request_sequence !=
                        active_request_->request_sequence) {
                    protocolError(Status::REASON_INVALID_REQUEST,
                                  "active request is absent from the ledger at shutdown");
                    fatal_.store(true);
                } else {
                    auto& entry = *current_request_;
                    if (entry.phase != RequestPhase::Terminal) {
                        if (entry.phase == RequestPhase::Accepted) {
                            enqueueRequestStatus(
                                *entry.request, Status::EVENT_RUNNING,
                                Status::HEALTH_HEALTHY, Status::TERMINAL_NONE,
                                Status::CANCEL_NONE, Status::REASON_NONE, "");
                            entry.phase = RequestPhase::Running;
                        }
                        active_cancel_->store(true, std::memory_order_release);
                        if (entry.protocol_cancelled) {
                            enqueueResult(cancelledRequest(*entry.request).result);
                        } else {
                            auto interrupted = shutdownInterruptedRequest(*entry.request);
                            enqueueResultAndTerminal(
                                std::move(interrupted.result), *entry.request,
                                Status::TERMINAL_REJECTED, Status::CANCEL_NONE,
                                Status::REASON_INVALID_REQUEST,
                                "shutdown_requested");
                        }
                        entry.phase = RequestPhase::Terminal;
                        entry.terminal_committed = true;
                    }
                }
                pending_work_.reset();
                busy_ = false;
            }
        }
        work_ready_.notify_all();
        drainOutbound();
        {
            std::unique_lock<std::mutex> lock(work_mutex_);
            if (!worker_stopped_.wait_for(
                    lock, kWorkerShutdownBound, [&] { return worker_done_; })) {
                std::cerr << "PGO: worker exceeded the shutdown bound; child exits 24\n"
                          << std::flush;
                std::_Exit(24);
            }
        }
        if (worker_.joinable()) worker_.join();
        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            if (active_request_) releaseRequestPayload(*active_request_);
            active_request_.reset();
            active_cancel_.reset();
        }
        drainOutbound();
        enqueueOwnerStatus(Status::EVENT_STOPPING, Status::HEALTH_UNKNOWN,
                           Status::TERMINAL_NONE, Status::REASON_SHUTDOWN_REQUESTED,
                           "shutdown_requested");
        drainOutbound();
        enqueueOwnerStatus(Status::EVENT_STOPPED, Status::HEALTH_UNKNOWN,
                           Status::TERMINAL_NONE, Status::REASON_SHUTDOWN_REQUESTED,
                           "stopped");
        drainOutbound();
        unsubscribe();
        return fatal_.load() ? 22 : 0;
    }

private:
    int64_t nextStatusSequenceLocked() {
        if (status_sequence_ == std::numeric_limits<int64_t>::max()) {
            fatal_.store(true);
            return status_sequence_;
        }
        return ++status_sequence_;
    }

    void enqueueOwnerStatus(int32_t event_kind, int32_t health_state,
                            int32_t terminal_state, int32_t reason_code,
                            const std::string& reason) {
        std::lock_guard<std::mutex> lock(outbound_mutex_);
        outbound_.emplace_back(ownerStatus(options_, nextStatusSequenceLocked(), event_kind,
                                           health_state, terminal_state, reason_code, reason));
    }

    void enqueueRequestStatus(const Request& request, int32_t event_kind,
                              int32_t health_state, int32_t terminal_state,
                              int32_t cancel_state, int32_t reason_code,
                              const std::string& reason) {
        std::lock_guard<std::mutex> lock(outbound_mutex_);
        outbound_.emplace_back(requestStatus(
            request, nextStatusSequenceLocked(), event_kind, health_state, terminal_state,
            cancel_state, reason_code, reason));
    }

    void enqueueResultAndTerminal(Result result, const Request& request,
                                  int32_t terminal_state, int32_t cancel_state,
                                  int32_t reason_code, const std::string& reason) {
        std::lock_guard<std::mutex> lock(outbound_mutex_);
        outbound_.emplace_back(std::move(result));
        outbound_.emplace_back(requestStatus(
            request, nextStatusSequenceLocked(), Status::EVENT_TERMINAL,
            Status::HEALTH_HEALTHY, terminal_state, cancel_state, reason_code, reason));
    }

    void enqueueResult(Result result) {
        std::lock_guard<std::mutex> lock(outbound_mutex_);
        outbound_.emplace_back(std::move(result));
    }

    void drainOutbound() {
        std::deque<OutboundMessage> pending;
        {
            std::lock_guard<std::mutex> lock(outbound_mutex_);
            pending.swap(outbound_);
        }
        for (auto& message : pending) {
            int published = -1;
            if (auto* result = std::get_if<Result>(&message)) {
                published = lcm_.publish(result_lcm_channel_, result);
            } else {
                published = lcm_.publish(status_lcm_channel_, &std::get<Status>(message));
            }
            if (published != 0) {
                std::cerr << "PGO: online typed publish failed\n";
                fatal_.store(true);
                break;
            }
        }
    }

    void protocolError(int32_t reason_code, const std::string& reason) {
        enqueueOwnerStatus(Status::EVENT_PROTOCOL_ERROR, Status::HEALTH_DEGRADED,
                           Status::TERMINAL_NONE, reason_code, reason);
    }

    void handleRequestMessage(const lcm::ReceiveBuffer* buffer, const std::string&) {
        constexpr std::size_t max_request_wire_bytes =
            8 + 4 * 16 + 8 * 32 + Request::MAX_REGISTERED_BODY_COORDINATE_COUNT * 8ULL +
            Request::MAX_COMMITTED_MAP_CELL_COUNT + 4'096;
        if (buffer == nullptr || buffer->data == nullptr || buffer->data_size <= 0 ||
            static_cast<std::size_t>(buffer->data_size) > max_request_wire_bytes) {
            protocolError(Status::REASON_PROTOCOL_DECODE_FAILED,
                          "request wire size is invalid");
            return;
        }
        if (!preflightRequestWire(buffer->data, buffer->data_size)) {
            protocolError(Status::REASON_PROTOCOL_DECODE_FAILED,
                          "request wire preflight failed");
            return;
        }
        Request decoded{};
        int consumed = -1;
        try {
            consumed = decoded.decode(buffer->data, 0, buffer->data_size);
        } catch (const std::exception&) {
            protocolError(Status::REASON_PROTOCOL_DECODE_FAILED,
                          "request decode raised an exception");
            return;
        }
        if (consumed < 0 || static_cast<uint32_t>(consumed) != buffer->data_size) {
            protocolError(Status::REASON_PROTOCOL_DECODE_FAILED,
                          "request decode failed or left trailing bytes");
            return;
        }
        if (const auto validation = validateRequest(decoded, options_)) {
            protocolError(validation->reason_code, validation->reason);
            return;
        }

        auto request = std::make_shared<Request>(std::move(decoded));
        std::lock_guard<std::mutex> lock(work_mutex_);
        const int64_t request_sequence = request->request_sequence;
        if (request_sequence <= retired_request_sequence_through_) return;

        if (current_request_) {
            const int64_t current_sequence =
                current_request_->request->request_sequence;
            if (request_sequence == current_sequence) {
                if (sameRequestIdentity(*request, *current_request_->request)) return;
                protocolError(
                    Status::REASON_PROTOCOL_IDENTITY_MISMATCH,
                    "request sequence was replayed with a conflicting identity or hash");
                fatal_.store(true);
                return;
            }
            if (current_sequence == std::numeric_limits<int64_t>::max() ||
                request_sequence != current_sequence + 1) {
                protocolError(Status::REASON_INVALID_REQUEST,
                              "request sequence skipped the unacknowledged terminal");
                return;
            }
            if (busy_ || active_request_ || pending_work_ ||
                current_request_->phase != RequestPhase::Terminal ||
                !current_request_->terminal_committed) {
                protocolError(Status::REASON_INVALID_REQUEST,
                              "next request arrived before the prior terminal was acknowledged");
                return;
            }
            if (last_request_sequence_ < current_sequence) {
                protocolError(Status::REASON_INVALID_REQUEST,
                              "terminal request was not committed to the sequence fence");
                fatal_.store(true);
                return;
            }
            retired_request_sequence_through_ = current_sequence;
            current_request_.reset();
        } else {
            if (busy_ || active_request_ || pending_work_) {
                protocolError(Status::REASON_INVALID_REQUEST,
                              "request ledger is empty while work remains active");
                fatal_.store(true);
                return;
            }
            if (retired_request_sequence_through_ ==
                    std::numeric_limits<int64_t>::max() ||
                request_sequence != retired_request_sequence_through_ + 1) {
                protocolError(Status::REASON_INVALID_REQUEST,
                              "request sequence did not continue the retired watermark");
                return;
            }
        }
        auto& ledger = current_request_.emplace(request);
        if (stop_requested_) {
            enqueueResultAndTerminal(
                rejectedResult(*request, Result::REJECTION_RESOURCE_LIMIT,
                               "owner is stopping"),
                *request, Status::TERMINAL_REJECTED, Status::CANCEL_NONE,
                Status::REASON_INVALID_REQUEST, "owner is stopping");
            ledger.phase = RequestPhase::Terminal;
            ledger.terminal_committed = true;
            releaseRequestPayload(*request);
            return;
        }
        busy_ = true;
        active_request_ = request;
        active_cancel_ = std::make_shared<std::atomic<bool>>(false);
        pending_work_ = WorkItem{request, active_cancel_};
        enqueueRequestStatus(*request, Status::EVENT_REQUEST_ACCEPTED,
                             Status::HEALTH_HEALTHY, Status::TERMINAL_NONE,
                             Status::CANCEL_NONE, Status::REASON_NONE, "");
        work_ready_.notify_one();
    }

    void handleCancelMessage(const lcm::ReceiveBuffer* buffer, const std::string&) {
        if (buffer == nullptr || buffer->data == nullptr || buffer->data_size <= 0 ||
            static_cast<std::size_t>(buffer->data_size) > kMaxStatusWireBytes ||
            !preflightStatusWire(buffer->data, buffer->data_size)) {
            protocolError(Status::REASON_PROTOCOL_DECODE_FAILED,
                          "cancel wire preflight failed");
            return;
        }
        Status cancel{};
        int consumed = -1;
        try {
            consumed = cancel.decode(buffer->data, 0, buffer->data_size);
        } catch (const std::exception&) {
            protocolError(Status::REASON_PROTOCOL_DECODE_FAILED,
                          "cancel decode raised an exception");
            return;
        }
        if (consumed < 0 || static_cast<uint32_t>(consumed) != buffer->data_size ||
            !validateCancelMessage(cancel, options_)) {
            protocolError(Status::REASON_PROTOCOL_DECODE_FAILED,
                          "cancel message contract is invalid");
            return;
        }
        std::lock_guard<std::mutex> lock(work_mutex_);
        if (cancel.request_sequence <= retired_request_sequence_through_) return;
        if (!current_request_ ||
            current_request_->request->request_sequence != cancel.request_sequence) {
            protocolError(Status::REASON_PROTOCOL_IDENTITY_MISMATCH,
                          "cancel does not match a retained request");
            return;
        }
        auto& entry = *current_request_;
        if (!sameRequestIdentity(cancel, *entry.request)) {
            protocolError(Status::REASON_PROTOCOL_IDENTITY_MISMATCH,
                          "cancel identity conflicts with its request sequence");
            fatal_.store(true);
            return;
        }
        if (entry.cancel_ack_emitted) return;
        if (entry.phase == RequestPhase::Terminal) {
            enqueueRequestStatus(*entry.request,
                                 Status::EVENT_CANCEL_ACKNOWLEDGED,
                                 Status::HEALTH_HEALTHY, Status::TERMINAL_NONE,
                                 Status::CANCEL_TOO_LATE, Status::REASON_CANCEL_TOO_LATE,
                                 "cancel_too_late");
            entry.cancel_ack_emitted = true;
            return;
        }
        if (!active_request_ || !active_cancel_ ||
            active_request_->request_sequence != cancel.request_sequence) {
            protocolError(Status::REASON_PROTOCOL_IDENTITY_MISMATCH,
                          "cancel request is retained but not active");
            fatal_.store(true);
            return;
        }
        if (entry.phase == RequestPhase::Accepted) {
            enqueueRequestStatus(*entry.request, Status::EVENT_RUNNING,
                                 Status::HEALTH_HEALTHY, Status::TERMINAL_NONE,
                                 Status::CANCEL_NONE, Status::REASON_NONE, "");
            entry.phase = RequestPhase::Running;
        }
        active_cancel_->store(true, std::memory_order_release);
        entry.protocol_cancelled = true;
        entry.cancel_ack_emitted = true;
        entry.phase = RequestPhase::CancelAcknowledged;
        enqueueRequestStatus(*entry.request, Status::EVENT_CANCEL_ACKNOWLEDGED,
                             Status::HEALTH_HEALTHY, Status::TERMINAL_CANCELLED,
                             Status::CANCEL_ACKNOWLEDGED, Status::REASON_CANCELLED,
                             "cancel_acknowledged");
    }

    void workerLoop() {
        bool stopping = false;
        while (!stopping) {
            WorkItem work;
            int64_t last_request_sequence = 0;
            std::optional<std::string> retained_odom_error;
            {
                std::unique_lock<std::mutex> lock(work_mutex_);
                work_ready_.wait(lock,
                                 [&] { return stop_requested_ || pending_work_.has_value(); });
                if (!pending_work_) {
                    if (stop_requested_) break;
                    continue;
                }
                work = std::move(*pending_work_);
                pending_work_.reset();
                if (!current_request_ ||
                    current_request_->request->request_sequence !=
                        work.request->request_sequence) {
                    protocolError(Status::REASON_INVALID_REQUEST,
                                  "worker request is absent from the ledger");
                    fatal_.store(true);
                    break;
                }
                auto& entry = *current_request_;
                if (entry.phase == RequestPhase::Terminal) {
                    releaseRequestPayload(*work.request);
                    if (active_request_ &&
                        active_request_->request_sequence ==
                            work.request->request_sequence) {
                        active_request_.reset();
                        active_cancel_.reset();
                    }
                    busy_ = false;
                    if (stop_requested_) break;
                    continue;
                }
                if (entry.phase == RequestPhase::Accepted) {
                    enqueueRequestStatus(*work.request, Status::EVENT_RUNNING,
                                         Status::HEALTH_HEALTHY,
                                         Status::TERMINAL_NONE,
                                         Status::CANCEL_NONE,
                                         Status::REASON_NONE, "");
                    entry.phase = RequestPhase::Running;
                }
                retained_odom_error =
                    validateAndAdvanceRetainedOdomFence(retained_odom_fence_,
                                                        *work.request);
                last_request_sequence = last_request_sequence_;
            }
            ProcessedRequest processed = retained_odom_error
                                             ? ProcessedRequest{
                                                   rejectedResult(
                                                       *work.request,
                                                       Result::REJECTION_INVALID_GRAPH,
                                                       *retained_odom_error),
                                                   TerminalKind::Rejected,
                                                   Status::REASON_INVALID_REQUEST,
                                                   *retained_odom_error}
                                             : processRequest(
                                                   graph_state_, last_request_sequence,
                                                   *work.request, options_, work.cancelled);
            {
                std::lock_guard<std::mutex> lock(work_mutex_);
                if (!current_request_ ||
                    current_request_->request->request_sequence !=
                        work.request->request_sequence) {
                    protocolError(Status::REASON_INVALID_REQUEST,
                                  "completed request is absent from the ledger");
                    fatal_.store(true);
                } else {
                    auto& entry = *current_request_;
                    if (!entry.terminal_committed) {
                        const bool cancelled =
                            work.cancelled->load(std::memory_order_acquire);
                        if (cancelled) {
                            processed = entry.protocol_cancelled
                                            ? cancelledRequest(*work.request)
                                            : shutdownInterruptedRequest(*work.request);
                        }
                        if (!cancelled && !stop_requested_ &&
                            entry.phase == RequestPhase::Running &&
                            processed.proposed_state) {
                            graph_state_ = std::move(*processed.proposed_state);
                        }
                        if (processed.consumes_request_sequence &&
                            work.request->request_sequence > last_request_sequence_) {
                            last_request_sequence_ = work.request->request_sequence;
                        }
                        if (processed.terminal_kind == TerminalKind::Cancelled) {
                            if (!entry.protocol_cancelled ||
                                !entry.cancel_ack_emitted ||
                                entry.phase != RequestPhase::CancelAcknowledged) {
                                protocolError(
                                    Status::REASON_INVALID_REQUEST,
                                    "cancelled result has no linearized protocol ACK");
                                fatal_.store(true);
                            } else {
                                enqueueResult(std::move(processed.result));
                            }
                        } else {
                            int32_t terminal_state = Status::TERMINAL_SUCCEEDED;
                            int32_t reason_code = Status::REASON_NONE;
                            std::string reason;
                            if (processed.terminal_kind == TerminalKind::Rejected) {
                                terminal_state = Status::TERMINAL_REJECTED;
                                reason_code = processed.reason_code;
                                reason = processed.reason;
                            }
                            enqueueResultAndTerminal(
                                std::move(processed.result), *work.request,
                                terminal_state, Status::CANCEL_NONE,
                                reason_code, reason);
                        }
                        entry.phase = RequestPhase::Terminal;
                        entry.terminal_committed = true;
                    }
                }
                releaseRequestPayload(*work.request);
                if (active_request_ &&
                    active_request_->request_sequence == work.request->request_sequence) {
                    active_request_.reset();
                    active_cancel_.reset();
                }
                busy_ = false;
                stopping = stop_requested_;
            }
        }
        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            worker_done_ = true;
        }
        worker_stopped_.notify_all();
    }

    void unsubscribe() {
        if (request_subscription_ != nullptr) {
            lcm_.unsubscribe(request_subscription_);
            request_subscription_ = nullptr;
        }
        if (cancel_subscription_ != nullptr) {
            lcm_.unsubscribe(cancel_subscription_);
            cancel_subscription_ = nullptr;
        }
    }

    OnlinePgoOptions options_;
    lcm::LCM lcm_;
    std::string request_lcm_channel_;
    std::string result_lcm_channel_;
    std::string status_lcm_channel_;
    std::string cancel_lcm_channel_;
    lcm::Subscription* request_subscription_{nullptr};
    lcm::Subscription* cancel_subscription_{nullptr};
    int64_t status_sequence_{0};
    std::atomic<bool> fatal_{false};
    std::thread worker_;
    std::mutex work_mutex_;
    std::condition_variable work_ready_;
    std::condition_variable worker_stopped_;
    bool stop_requested_{false};
    bool worker_done_{false};
    bool busy_{false};
    std::optional<WorkItem> pending_work_;
    std::shared_ptr<Request> active_request_;
    std::shared_ptr<std::atomic<bool>> active_cancel_;
    std::optional<RequestLedgerEntry> current_request_;
    int64_t retired_request_sequence_through_{0};
    std::mutex outbound_mutex_;
    std::deque<OutboundMessage> outbound_;
    GraphState graph_state_;
    RetainedOdomFence retained_odom_fence_;
    int64_t last_request_sequence_{0};
};

}  // namespace

int runOnlinePgoOwner(const OnlinePgoOptions& options, const std::atomic<bool>& running) {
    try {
        OnlinePgoOwnerImpl owner(options);
        return owner.run(running);
    } catch (const std::exception& error) {
        std::cerr << "PGO: online owner failed: " << error.what() << '\n';
        return 23;
    }
}

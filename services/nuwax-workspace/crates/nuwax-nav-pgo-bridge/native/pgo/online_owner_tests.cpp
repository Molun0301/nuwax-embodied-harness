// Socket-free source fixture. This translation unit includes the implementation so
// request admission, cancellation, retention, and semantic validators stay real.
#include <lcm/lcm-cpp.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

class OnlinePgoRecordingTransport {
public:
    explicit OnlinePgoRecordingTransport(const std::string&) {}
    bool good() const { return true; }
    template <typename... Arguments>
    lcm::Subscription* subscribe(Arguments&&...) { return nullptr; }
    int unsubscribe(lcm::Subscription*) { return 0; }
    int handleTimeout(int) { return 0; }
    template <typename Message>
    int publish(const std::string&, const Message*) {
        ++published_count;
        return 0;
    }
    std::size_t published_count{0};
};

#define NUWAX_NATIVE_PGO_OWNER_TEST
#include "online_owner.cpp"

namespace {

void require(bool condition, const char* detail) {
    if (!condition) throw std::runtime_error(detail);
}

OnlinePgoOptions fixtureOptions() {
    OnlinePgoOptions options;
    options.lcm_url = "test-recording-transport";
    options.runtime_generation = 11;
    options.backend_generation = 13;
    return options;
}

Request fixtureRequest() {
    Request request{};
    request.schema_version = Request::SCHEMA_VERSION_V3;
    request.backend = Request::BACKEND_NATIVE_PGO;
    request.runtime_generation = 11;
    request.backend_generation = 13;
    request.request_sequence = 1;
    request.mapping_session_id = "fixture-session";
    request.mapping_session_generation = 2;
    request.mapping_backend_kind = "voxel";
    request.mapping_backend_generation = 17;
    request.map_frame_generation = 3;
    request.mapping_commit_boot_nonce[0] = 1;
    request.mapping_commit_sequence = 1;
    request.occupancy_revision = 1;
    request.cloud_revision = 1;
    request.committed_map_source_timestamp_ns = 10;
    request.registered_cloud_backend = 1;
    request.registered_cloud_runtime_generation = 29;
    request.registered_cloud_source_sequence = 1;
    request.registered_cloud_source_timestamp_ns = 10;
    request.registered_cloud_point_count = 60;
    request.registered_body_point_count = 30;
    request.registered_body_coordinate_count = 90;
    for (int index = 0; index < 30; ++index) {
        request.registered_body_points_xyz.push_back(static_cast<double>(index));
        request.registered_body_points_xyz.push_back(0.0);
        request.registered_body_points_xyz.push_back(1.0);
    }
    request.correlated_odom_source_sequence = 5;
    request.correlated_odom_source_timestamp_ns = 9;
    request.frozen_odom_rotation_xyzw[3] = 1.0;
    request.committed_map_width = 2;
    request.committed_map_height = 3;
    request.committed_map_resolution_m = 0.1;
    request.source_timestamp_ns = 10;
    request.map_frame_id = "map";
    request.odom_frame_id = "odom";
    request.body_frame_id = "body";
    request.points_frame_id = "body";
    request.correction_lease_boot_nonce[0] = 2;
    request.correction_lease_generation = 7;
    return request;
}

Status fixtureCancel(const Request& request) {
    return requestStatus(request, 1, Status::EVENT_CANCEL_REQUESTED,
                         Status::HEALTH_UNKNOWN, Status::TERMINAL_NONE,
                         Status::CANCEL_REQUESTED, Status::REASON_CANCEL_REQUESTED,
                         "cancel_requested");
}

template <typename Message>
std::vector<uint8_t> fixtureWire(const Message& message) {
    const int size = message.getEncodedSize();
    require(size > 0, "wire size must be positive");
    std::vector<uint8_t> wire(static_cast<std::size_t>(size));
    require(message.encode(wire.data(), 0, size) == size, "generated encoding failed");
    return wire;
}

struct OnlinePgoOwnerTestAccess {
    static void request(OnlinePgoOwnerImpl& owner, const Request& request) {
        auto wire = fixtureWire(request);
        lcm::ReceiveBuffer buffer{};
        buffer.data = wire.data();
        buffer.data_size = wire.size();
        owner.handleRequestMessage(&buffer, "fixture");
    }

    static void cancel(OnlinePgoOwnerImpl& owner, const Status& cancel) {
        auto wire = fixtureWire(cancel);
        lcm::ReceiveBuffer buffer{};
        buffer.data = wire.data();
        buffer.data_size = wire.size();
        owner.handleCancelMessage(&buffer, "fixture");
    }

    static std::deque<OutboundMessage> takeOutbound(OnlinePgoOwnerImpl& owner) {
        std::deque<OutboundMessage> messages;
        messages.swap(owner.outbound_);
        return messages;
    }

    static const RequestLedgerEntry& current(const OnlinePgoOwnerImpl& owner) {
        require(owner.current_request_.has_value(), "current request missing");
        return *owner.current_request_;
    }

    static bool fatal(const OnlinePgoOwnerImpl& owner) { return owner.fatal_.load(); }

    static void completePending(OnlinePgoOwnerImpl& owner) {
        require(owner.pending_work_.has_value(), "pending request missing");
        // No thread or polling is needed: a queued item is processed before the
        // stop flag ends the loop. This runs the actual terminal ownership path.
        owner.stop_requested_ = true;
        owner.workerLoop();
        require(owner.worker_done_, "synchronous worker did not stop");
        require(!owner.active_request_ && !owner.pending_work_ && !owner.busy_,
                "terminal worker retained active execution ownership");
        owner.stop_requested_ = false;
        owner.worker_done_ = false;
    }
};

void verifyWireAndSemanticAdmission() {
    const auto options = fixtureOptions();
    Request request = fixtureRequest();
    require(!validateRequest(request, options), "valid independent domains/counts rejected");
    auto wire = fixtureWire(request);
    require(preflightRequestWire(wire.data(), wire.size()), "V3 request preflight rejected");
    for (std::size_t size = 0; size < wire.size(); ++size) {
        require(!preflightRequestWire(wire.data(), size), "truncated request accepted");
    }
    wire.push_back(0);
    require(!preflightRequestWire(wire.data(), wire.size()), "trailing request byte accepted");
    wire = fixtureWire(request);
    wire[0] ^= 1;
    require(!preflightRequestWire(wire.data(), wire.size()), "wrong schema fingerprint accepted");

    Status cancel = fixtureCancel(request);
    auto cancel_wire = fixtureWire(cancel);
    require(preflightStatusWire(cancel_wire.data(), cancel_wire.size()), "V3 cancel preflight rejected");
    require(validateCancelMessage(cancel, options), "valid cancel event matrix rejected");
    for (std::size_t size = 0; size < cancel_wire.size(); ++size) {
        require(!preflightStatusWire(cancel_wire.data(), size), "truncated cancel accepted");
    }
    cancel_wire.push_back(0);
    require(!preflightStatusWire(cancel_wire.data(), cancel_wire.size()), "trailing cancel byte accepted");
    cancel.identity_present = 0;
    require(!validateCancelMessage(cancel, options), "missing live cancel identity accepted");
    cancel = fixtureCancel(request);
    cancel.terminal_state = Status::TERMINAL_CANCELLED;
    require(!validateCancelMessage(cancel, options), "invalid cancel event matrix accepted");

    Status maximum = fixtureCancel(request);
    for (std::string* field : {&maximum.mapping_session_id, &maximum.mapping_backend_kind,
                              &maximum.map_frame_id, &maximum.odom_frame_id,
                              &maximum.body_frame_id, &maximum.points_frame_id, &maximum.reason}) {
        *field = std::string(kMaxIdentityBytes, 'x');
    }
    const auto maximum_wire = fixtureWire(maximum);
    require(maximum_wire.size() == kMaxStatusWireBytes, "V3 status wire bound drifted");
    require(preflightStatusWire(maximum_wire.data(), maximum_wire.size()),
            "maximum bounded status rejected by preflight");

#define INVALID_REQUEST(field, value) \
    do { auto changed = request; changed.field = value; \
         require(validateRequest(changed, options).has_value(), "invalid " #field " accepted"); } while (false)
    INVALID_REQUEST(schema_version, 2);
    INVALID_REQUEST(mapping_backend_kind, "");
    INVALID_REQUEST(mapping_backend_generation, 0);
    INVALID_REQUEST(registered_cloud_backend, 0);
    INVALID_REQUEST(registered_cloud_backend, 4);
    INVALID_REQUEST(registered_cloud_runtime_generation, 0);
    INVALID_REQUEST(registered_cloud_point_count, 29);
    INVALID_REQUEST(registered_cloud_point_count, Request::MAX_REGISTERED_BODY_POINT_COUNT + 1);
    INVALID_REQUEST(registered_body_coordinate_count, 89);
    INVALID_REQUEST(registered_body_points_xyz[0], std::numeric_limits<double>::infinity());
    INVALID_REQUEST(frozen_odom_rotation_xyzw[3], 2.0);
    INVALID_REQUEST(committed_map_width, std::numeric_limits<int64_t>::max());
    INVALID_REQUEST(committed_map_width, Request::MAX_COMMITTED_MAP_CELL_COUNT);
    INVALID_REQUEST(committed_map_resolution_m, 0.0);
    INVALID_REQUEST(source_timestamp_ns, 11);
    INVALID_REQUEST(points_frame_id, "map");
#undef INVALID_REQUEST
    auto opaque_kind = request;
    opaque_kind.mapping_backend_kind = "future-mapping-owner";
    require(!validateRequest(opaque_kind, options), "opaque mapping provenance label rejected");
    require(sameRequestReceipt(noCandidateResult(opaque_kind), opaque_kind),
            "opaque mapping provenance label was replaced on echo");
    request.committed_map_source_timestamp_ns = 0;
    request.registered_cloud_source_timestamp_ns = 0;
    request.correlated_odom_source_timestamp_ns = 0;
    request.source_timestamp_ns = 0;
    require(!validateRequest(request, options), "present zero timestamps rejected");
}

void verifyExactReceiptAndPayload() {
    const Request request = fixtureRequest();
    require(sameRequestIdentity(request, request), "identical request differs");
    require(sameRequestIdentity(fixtureCancel(request), request), "cancel receipt differs");
    require(sameRequestReceipt(noCandidateResult(request), request), "result receipt differs");
#define RECEIPT_CONFLICT(field, value) \
    do { auto changed = request; changed.field = value; \
         require(!sameRequestIdentity(changed, request), "request omitted " #field); \
         require(!sameRequestIdentity(fixtureCancel(changed), request), "cancel omitted " #field); \
         require(!sameRequestReceipt(noCandidateResult(changed), request), "result omitted " #field); } while (false)
    RECEIPT_CONFLICT(runtime_generation, 12);
    RECEIPT_CONFLICT(backend_generation, 14);
    RECEIPT_CONFLICT(request_sequence, 2);
    RECEIPT_CONFLICT(mapping_session_id, "other-session");
    RECEIPT_CONFLICT(mapping_session_generation, 3);
    RECEIPT_CONFLICT(mapping_backend_kind, "raytracing");
    RECEIPT_CONFLICT(mapping_backend_generation, 18);
    RECEIPT_CONFLICT(map_frame_generation, 4);
    RECEIPT_CONFLICT(mapping_commit_boot_nonce[0], 3);
    RECEIPT_CONFLICT(mapping_commit_sequence, 2);
    RECEIPT_CONFLICT(occupancy_revision, 2);
    RECEIPT_CONFLICT(cloud_revision, 2);
    RECEIPT_CONFLICT(committed_map_source_timestamp_ns, 11);
    RECEIPT_CONFLICT(registered_cloud_backend, 2);
    RECEIPT_CONFLICT(registered_cloud_runtime_generation, 30);
    RECEIPT_CONFLICT(registered_cloud_source_sequence, 2);
    RECEIPT_CONFLICT(registered_cloud_source_timestamp_ns, 11);
    RECEIPT_CONFLICT(registered_cloud_point_count, 61);
    RECEIPT_CONFLICT(registered_body_point_count, 29);
    RECEIPT_CONFLICT(correlated_odom_source_sequence, 6);
    RECEIPT_CONFLICT(correlated_odom_source_timestamp_ns, 10);
    RECEIPT_CONFLICT(frozen_odom_translation_m[0], -0.0);
    RECEIPT_CONFLICT(frozen_odom_translation_m[1], -0.0);
    RECEIPT_CONFLICT(frozen_odom_translation_m[2], -0.0);
    RECEIPT_CONFLICT(frozen_odom_rotation_xyzw[0], -0.0);
    RECEIPT_CONFLICT(frozen_odom_rotation_xyzw[1], -0.0);
    RECEIPT_CONFLICT(frozen_odom_rotation_xyzw[2], -0.0);
    RECEIPT_CONFLICT(frozen_odom_rotation_xyzw[3], -1.0);
    RECEIPT_CONFLICT(committed_map_width, 3);
    RECEIPT_CONFLICT(committed_map_height, 4);
    RECEIPT_CONFLICT(committed_map_resolution_m, 0.2);
    RECEIPT_CONFLICT(committed_map_origin_x_m, -0.0);
    RECEIPT_CONFLICT(committed_map_origin_y_m, -0.0);
    RECEIPT_CONFLICT(source_timestamp_ns, 11);
    RECEIPT_CONFLICT(map_frame_id, "other-map");
    RECEIPT_CONFLICT(odom_frame_id, "other-odom");
    RECEIPT_CONFLICT(body_frame_id, "other-body");
    RECEIPT_CONFLICT(points_frame_id, "other-body");
    RECEIPT_CONFLICT(correction_lease_boot_nonce[0], 3);
    RECEIPT_CONFLICT(correction_lease_generation, 8);
#undef RECEIPT_CONFLICT
    auto changed = request;
    changed.registered_body_points_xyz[0] = -0.0;
    require(sameRequestReceipt(changed, request), "XYZ mutation unexpectedly changes receipt");
    require(!sameRequestIdentity(changed, request), "bitwise XYZ conflict accepted");
    changed = request;
    changed.registered_body_coordinate_count -= 3;
    require(!sameRequestIdentity(changed, request), "coordinate count conflict accepted");
    changed = request;
    changed.registered_body_points_xyz.pop_back();
    require(!sameRequestIdentity(changed, request), "XYZ length conflict accepted");
}

void verifyRetainedOdometryDomain() {
    const auto request = fixtureRequest();
    RetainedOdomFence fence;
    require(!validateAndAdvanceRetainedOdomFence(fence, request), "first odom receipt rejected");
    auto changed = request;
    changed.frozen_odom_translation_m[0] = -0.0;
    require(validateAndAdvanceRetainedOdomFence(fence, changed).has_value(),
            "same-domain same-sequence exact pose conflict accepted");
    require(!validateAndAdvanceRetainedOdomFence(fence, request), "exact odom retry rejected");
    changed = request;
    ++changed.mapping_backend_generation;
    require(!validateAndAdvanceRetainedOdomFence(fence, changed),
            "later mapping backend receipt changed odom lifecycle policy");
    changed.registered_cloud_runtime_generation++;
    changed.correlated_odom_source_sequence = 1;
    changed.correlated_odom_source_timestamp_ns = 0;
    require(!validateAndAdvanceRetainedOdomFence(fence, changed),
            "new LIO runtime sequence restart rejected");
    changed.registered_cloud_backend = 2;
    require(!validateAndAdvanceRetainedOdomFence(fence, changed),
            "new LIO backend sequence domain rejected");
    changed.frozen_odom_rotation_xyzw[0] = -0.0;
    require(validateAndAdvanceRetainedOdomFence(fence, changed).has_value(),
            "new-domain retained pose no longer protected");
    changed.registered_cloud_runtime_generation++;
    changed.odom_frame_id = "unannounced-frame";
    require(validateAndAdvanceRetainedOdomFence(fence, changed).has_value(),
            "LIO restart incorrectly authorized relabeling coordinate frames");
}

void verifyGraphSessionPolicy() {
    const auto options = fixtureOptions();
    GraphState state(options.graph);
    auto request = fixtureRequest();
    require(!advanceSessionBoundary(state, request, options.graph), "initial graph boundary rejected");
    request.mapping_commit_sequence++;
    request.occupancy_revision++;
    request.cloud_revision++;
    request.registered_cloud_source_sequence++;
    request.source_timestamp_ns++;
    request.committed_map_source_timestamp_ns++;
    request.registered_cloud_source_timestamp_ns++;
    request.mapping_backend_generation++;
    request.mapping_backend_kind = "raytracing";
    require(!validateSessionBoundary(state, request),
            "mapping receipt update introduced a graph lifecycle restriction");
    request.odom_frame_id = "unannounced-frame";
    require(validateSessionBoundary(state, request).has_value(),
            "mapping receipt update authorized a coordinate frame relabel");
}

void verifyTerminalRetentionAndRetirement() {
    OnlinePgoOwnerImpl owner(fixtureOptions());
    const Request request = fixtureRequest();
    OnlinePgoOwnerTestAccess::request(owner, request);
    const auto* payload = OnlinePgoOwnerTestAccess::current(owner).request->registered_body_points_xyz.data();
    std::weak_ptr<const Request> retained = OnlinePgoOwnerTestAccess::current(owner).request;
    require(OnlinePgoOwnerTestAccess::takeOutbound(owner).size() == 1, "request accepted more than once");
    OnlinePgoOwnerTestAccess::request(owner, request);
    require(OnlinePgoOwnerTestAccess::takeOutbound(owner).empty(), "accepted retry emitted another event");
    OnlinePgoOwnerTestAccess::completePending(owner);
    const auto terminal = OnlinePgoOwnerTestAccess::takeOutbound(owner);
    require(terminal.size() == 3, "expected running, result, terminal exactly once");
    require(std::get<Status>(terminal[0]).event_kind == Status::EVENT_RUNNING,
            "running event was reordered");
    require(std::holds_alternative<Result>(terminal[1]), "terminal result was reordered");
    require(std::get<Status>(terminal[2]).event_kind == Status::EVENT_TERMINAL,
            "terminal event was reordered");
    require(OnlinePgoOwnerTestAccess::current(owner).phase == RequestPhase::Terminal,
            "request did not become terminal");
    require(OnlinePgoOwnerTestAccess::current(owner).request->registered_body_points_xyz.data() == payload,
            "terminal cleared or copied the retained payload");
    OnlinePgoOwnerTestAccess::request(owner, request);
    require(OnlinePgoOwnerTestAccess::takeOutbound(owner).empty(), "terminal retry emitted another terminal");
    const auto cancel = fixtureCancel(request);
    OnlinePgoOwnerTestAccess::cancel(owner, cancel);
    const auto late = OnlinePgoOwnerTestAccess::takeOutbound(owner);
    require(late.size() == 1 && std::get<Status>(late.front()).cancel_state == Status::CANCEL_TOO_LATE,
            "late cancel was not acknowledged once");
    OnlinePgoOwnerTestAccess::cancel(owner, cancel);
    require(OnlinePgoOwnerTestAccess::takeOutbound(owner).empty(), "duplicate late cancel re-acknowledged");
    auto next = request;
    next.request_sequence = 3;
    OnlinePgoOwnerTestAccess::request(owner, next);
    require(!retained.expired(), "invalid next sequence released current payload");
    OnlinePgoOwnerTestAccess::takeOutbound(owner);
    next.request_sequence = 2;
    next.mapping_commit_sequence++;
    next.occupancy_revision++;
    next.cloud_revision++;
    next.registered_cloud_source_sequence++;
    next.registered_cloud_source_timestamp_ns++;
    next.committed_map_source_timestamp_ns++;
    next.source_timestamp_ns++;
    OnlinePgoOwnerTestAccess::request(owner, next);
    require(retained.expired(), "valid retirement retained an old payload");
    require(OnlinePgoOwnerTestAccess::takeOutbound(owner).size() == 1, "next request admission failed");
    auto old_conflict = request;
    old_conflict.registered_body_points_xyz[0] = -0.0;
    OnlinePgoOwnerTestAccess::request(owner, old_conflict);
    require(OnlinePgoOwnerTestAccess::takeOutbound(owner).empty(), "retired request escaped sequence watermark");
    require(!OnlinePgoOwnerTestAccess::fatal(owner), "retired replay killed owner");
}

void verifyTerminalPayloadConflict() {
    OnlinePgoOwnerImpl owner(fixtureOptions());
    const auto request = fixtureRequest();
    OnlinePgoOwnerTestAccess::request(owner, request);
    OnlinePgoOwnerTestAccess::takeOutbound(owner);
    OnlinePgoOwnerTestAccess::completePending(owner);
    OnlinePgoOwnerTestAccess::takeOutbound(owner);
    auto changed = request;
    changed.registered_body_points_xyz[0] = -0.0;
    OnlinePgoOwnerTestAccess::request(owner, changed);
    const auto conflict = OnlinePgoOwnerTestAccess::takeOutbound(owner);
    require(conflict.size() == 1 && OnlinePgoOwnerTestAccess::fatal(owner),
            "terminal XYZ conflict was not fatal");
    require(std::get<Status>(conflict.front()).reason_code == Status::REASON_PROTOCOL_RECEIPT_MISMATCH,
            "terminal XYZ conflict lost receipt-mismatch reason");
}

void verifyCancelReceiptConflict() {
    OnlinePgoOwnerImpl owner(fixtureOptions());
    const auto request = fixtureRequest();
    OnlinePgoOwnerTestAccess::request(owner, request);
    OnlinePgoOwnerTestAccess::takeOutbound(owner);
    auto changed = fixtureCancel(request);
    changed.frozen_odom_translation_m[0] = -0.0;
    OnlinePgoOwnerTestAccess::cancel(owner, changed);
    const auto conflict = OnlinePgoOwnerTestAccess::takeOutbound(owner);
    require(conflict.size() == 1 && OnlinePgoOwnerTestAccess::fatal(owner),
            "cancel exact pose conflict was not fatal");
    require(std::get<Status>(conflict.front()).reason_code == Status::REASON_PROTOCOL_RECEIPT_MISMATCH,
            "cancel conflict lost receipt-mismatch reason");
    require(!OnlinePgoOwnerTestAccess::current(owner).cancel_ack_emitted,
            "conflicting cancel was acknowledged");
}

void verifyCancellationOrdering() {
    OnlinePgoOwnerImpl owner(fixtureOptions());
    const auto request = fixtureRequest();
    OnlinePgoOwnerTestAccess::request(owner, request);
    OnlinePgoOwnerTestAccess::takeOutbound(owner);
    const auto cancel = fixtureCancel(request);
    OnlinePgoOwnerTestAccess::cancel(owner, cancel);
    const auto ack = OnlinePgoOwnerTestAccess::takeOutbound(owner);
    require(ack.size() == 2, "accepted cancellation did not emit running then ACK");
    require(std::get<Status>(ack[0]).event_kind == Status::EVENT_RUNNING,
            "cancel ACK preceded running");
    require(std::get<Status>(ack[1]).event_kind == Status::EVENT_CANCEL_ACKNOWLEDGED &&
            std::get<Status>(ack[1]).terminal_state == Status::TERMINAL_CANCELLED,
            "cancel ACK lost terminal disposition");
    OnlinePgoOwnerTestAccess::cancel(owner, cancel);
    require(OnlinePgoOwnerTestAccess::takeOutbound(owner).empty(), "cancel ACK emitted twice");
    OnlinePgoOwnerTestAccess::completePending(owner);
    const auto completed = OnlinePgoOwnerTestAccess::takeOutbound(owner);
    require(completed.size() == 1 && std::holds_alternative<Result>(completed.front()),
            "cancelled worker emitted another terminal status");
    require(std::get<Result>(completed.front()).rejection_reason_code == Result::REJECTION_CANCELLED,
            "cancelled worker result changed disposition");
    require(sameRequestIdentity(*OnlinePgoOwnerTestAccess::current(owner).request, request),
            "cancelled terminal discarded exact request identity");
    OnlinePgoOwnerTestAccess::request(owner, request);
    require(OnlinePgoOwnerTestAccess::takeOutbound(owner).empty(), "cancelled terminal retry restarted work");
}

Result fixtureCorrection(const Request& request) {
    Result result = resultBase(request);
    result.result_disposition = Result::DISPOSITION_CORRECTION;
    result.transform_semantics = Result::TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3;
    result.rotation_xyzw[3] = 1.0;
    result.loop_source_node_id = 2;
    result.loop_target_node_id = 0;
    result.loop_inlier_count = 20;
    result.optimized_graph_node_count = 3;
    result.optimized_graph_edge_count = 3;
    result.optimized_graph_loop_edge_count = 1;
    result.optimization_iterations = 1;
    result.optimization_converged = 1;
    result.optimization_initial_cost = 2.0;
    result.optimization_final_cost = 1.0;
    result.trajectory_count = 3;
    result.trajectory_timestamp_ns = {0, 5, 10};
    result.trajectory_translation_m = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    result.trajectory_rotation_xyzw = {{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}};
    return result;
}

void verifyConcreteCorrectionConflicts() {
    const auto request = fixtureRequest();
    const auto result = fixtureCorrection(request);
    require(correctionResultIsValid(request, result), "valid correction rejected");
#define INVALID_CORRECTION(field, value) \
    do { auto changed = result; changed.field = value; \
         require(!correctionResultIsValid(request, changed), "invalid correction " #field " accepted"); } while (false)
    INVALID_CORRECTION(mapping_backend_generation, 18);
    INVALID_CORRECTION(registered_cloud_point_count, 61);
    INVALID_CORRECTION(frozen_odom_translation_m[0], -0.0);
    INVALID_CORRECTION(result_disposition, Result::DISPOSITION_NO_CANDIDATE);
    INVALID_CORRECTION(transform_semantics, 0);
    INVALID_CORRECTION(loop_target_node_id, 2);
    INVALID_CORRECTION(loop_inlier_count, Result::MAX_LOOP_INLIERS + 1);
    INVALID_CORRECTION(optimized_graph_edge_count, 4);
    INVALID_CORRECTION(optimization_iterations, Result::MAX_OPTIMIZATION_ITERATIONS + 1);
    INVALID_CORRECTION(optimization_converged, 0);
    INVALID_CORRECTION(optimization_final_cost, 3.0);
    INVALID_CORRECTION(fitness_score, -1.0);
    INVALID_CORRECTION(inlier_rmse_m, std::numeric_limits<double>::quiet_NaN());
    INVALID_CORRECTION(trajectory_timestamp_ns[1], 0);
    INVALID_CORRECTION(trajectory_timestamp_ns[2], 9);
    INVALID_CORRECTION(trajectory_rotation_xyzw[0][3], 2.0);
    INVALID_CORRECTION(translation_m[0], std::numeric_limits<double>::infinity());
    INVALID_CORRECTION(rejection_reason_code, Result::REJECTION_INVALID_GRAPH);
#undef INVALID_CORRECTION
}

}  // namespace

int main() {
    try {
        verifyWireAndSemanticAdmission();
        verifyExactReceiptAndPayload();
        verifyRetainedOdometryDomain();
        verifyGraphSessionPolicy();
        verifyTerminalRetentionAndRetirement();
        verifyTerminalPayloadConflict();
        verifyCancelReceiptConflict();
        verifyCancellationOrdering();
        verifyConcreteCorrectionConflicts();
    } catch (const std::exception& error) {
        std::cerr << "online owner contract fixture failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}

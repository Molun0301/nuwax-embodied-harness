//! 视觉目标到 Batch02 唯一导航准入链的生产适配。

use std::time::Duration;

use nuwax_nav_types_messages::{NavFrameId, Point3, Quaternion, Timestamp};
use serde::{Deserialize, Serialize};
use tokio::runtime::Runtime;
use tonic::transport::Endpoint;

use crate::geometry_proto::{
    geometry_runtime_client::GeometryRuntimeClient, CancelNavigationRequest, Empty, NavigationGoal,
    NavigationState, Pose, Quat, Vec3,
};
use crate::session::{TargetObservation, TargetObservationState};

const PUBLIC_NAV_OWNER: i32 = 1;

/// 调用者随视觉帧冻结的相机到规划坐标系外参。
#[derive(Debug, Clone, Deserialize)]
pub struct FrozenGoalTransform {
    /// 外参来源坐标系，必须与观察相机坐标系一致。
    pub source_frame_id: String,
    /// 外参目标坐标系，生产仅接受 `map`。
    pub target_frame_id: String,
    /// 外参快照时间戳。
    pub source_timestamp_ns: u64,
    /// 与观察内参快照相同的标定身份。
    pub calibration_id: String,
    /// 目标坐标系中的平移。
    pub translation_xyz: [f64; 3],
    /// 来源坐标系到目标坐标系的旋转，顺序为 `(x,y,z,w)`。
    pub rotation_xyzw: [f64; 4],
}

/// Batch02 公共 wire 准入所需的请求级冻结字段。
#[derive(Debug, Clone, Deserialize)]
pub struct NavigationAdmission {
    /// 相机到权威规划坐标系的冻结外参。
    pub transform: FrozenGoalTransform,
    /// Motion Guard（运动保护器）准入代次。
    pub motion_guard_generation: u64,
    /// Motion Guard 启动 nonce，十六进制 16 字节。
    pub motion_guard_boot_nonce_hex: String,
    /// 调用者观察到的全局停止代次。
    pub motion_stop_generation: u64,
}

/// 已取得的唯一 Batch02 GoalLease（目标租约）身份。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdmittedGoalLease {
    /// Geometry 返回的目标身份。
    pub goal_id: String,
    /// Geometry 返回的父请求身份。
    pub parent_request_id: String,
    /// 唯一仲裁器分配的目标代次。
    pub goal_generation: u64,
    /// Geometry 生产运行时代次。
    pub producer_runtime_generation: u64,
    /// 密封目标载荷哈希；间接绑定目标姿态、frame 与 source timestamp。
    pub goal_payload_sha256: [u8; 32],
}

/// 对外输出的类型化导航准入/终态。
#[derive(Debug, Clone, Serialize)]
pub struct NavigationOutcome {
    /// Geometry 返回的状态名称。
    pub status: String,
    /// 稳定的类型化原因码。
    pub reason_code: String,
    /// 面向运维的结果说明。
    pub message: String,
    /// 终态对应的目标身份。
    pub goal_id: String,
    /// 终态对应的父请求身份。
    pub parent_request_id: String,
    /// 终态对应的目标代次。
    pub goal_generation: u64,
    /// 终态对应的 Geometry 运行时代次。
    pub producer_runtime_generation: u64,
    /// 取消链当前阶段。
    pub cancel_phase: String,
    /// 执行器是否已取得零速证明。
    pub actuator_zeroed: bool,
}

impl NavigationOutcome {
    fn from_state(state: NavigationState) -> Self {
        Self {
            status: state.status,
            reason_code: state.reason_code,
            message: state.message,
            goal_id: state.goal_id,
            parent_request_id: state.parent_request_id,
            goal_generation: state.goal_generation,
            producer_runtime_generation: state.producer_runtime_generation,
            cancel_phase: state.cancel_phase,
            actuator_zeroed: state.actuator_zeroed,
        }
    }
}

/// 真实 Geometry gRPC（远程过程调用）transport 配置。
#[derive(Debug, Clone)]
pub struct GoalTransportConfig {
    /// Geometry gRPC 服务端点。
    pub endpoint: String,
    /// 单次连接或 RPC 的最长等待时间。
    pub rpc_timeout: Duration,
}

impl Default for GoalTransportConfig {
    fn default() -> Self {
        Self {
            endpoint: "http://127.0.0.1:50051".into(),
            rpc_timeout: Duration::from_secs(5),
        }
    }
}

/// 唯一生产目标 transport；不持有规划、队列或执行状态。
pub struct GeometryGoalTransport {
    config: GoalTransportConfig,
    runtime: Runtime,
}

impl GeometryGoalTransport {
    /// 校验端点并创建独立异步运行时；此步骤不连接网络。
    pub fn new(config: GoalTransportConfig) -> Result<Self, String> {
        Endpoint::from_shared(config.endpoint.clone()).map_err(|error| error.to_string())?;
        let runtime = Runtime::new().map_err(|error| error.to_string())?;
        Ok(Self { config, runtime })
    }

    /// 仅在严格观察及全部冻结身份通过后向 Batch02 产生 transport 副作用。
    pub fn submit(
        &self,
        observation: &TargetObservation,
        admission: &NavigationAdmission,
        tracker_swaps_axes: bool,
    ) -> Result<(AdmittedGoalLease, NavigationOutcome), String> {
        let goal = adapt_observation(observation, admission, tracker_swaps_axes)?;
        let endpoint = self.config.endpoint.clone();
        let timeout = self.config.rpc_timeout;
        let state = self.runtime.block_on(async move {
            let mut client =
                tokio::time::timeout(timeout, GeometryRuntimeClient::connect(endpoint))
                    .await
                    .map_err(|_| "Geometry transport connect timed out".to_string())?
                    .map_err(|error| format!("Geometry transport connect failed: {error}"))?;
            tokio::time::timeout(timeout, client.set_navigation_goal(goal))
                .await
                .map_err(|_| "SetNavigationGoal timed out".to_string())?
                .map_err(|error| format!("SetNavigationGoal failed: {error}"))
                .map(|response| response.into_inner())
        })?;
        if state.status != "ACCEPTED" {
            return Err(format!(
                "GoalIntent admission rejected: status={} reason_code={} message={}",
                state.status, state.reason_code, state.message
            ));
        }
        let goal_payload_sha256: [u8; 32] = state
            .goal_payload_sha256
            .as_slice()
            .try_into()
            .map_err(|_| "ACCEPTED navigation state has invalid goal payload hash".to_string())?;
        let lease = AdmittedGoalLease {
            goal_id: state.goal_id.clone(),
            parent_request_id: state.parent_request_id.clone(),
            goal_generation: state.goal_generation,
            producer_runtime_generation: state.producer_runtime_generation,
            goal_payload_sha256,
        };
        Ok((lease, NavigationOutcome::from_state(state)))
    }

    /// 通过 Batch02 exact-lease cancel wire 等待零速确认后的类型化终态。
    pub fn cancel(&self, lease: &AdmittedGoalLease) -> Result<NavigationOutcome, String> {
        let request = CancelNavigationRequest {
            goal_id: lease.goal_id.clone(),
            owner_kind: PUBLIC_NAV_OWNER,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation,
            producer_runtime_generation: lease.producer_runtime_generation,
        };
        let endpoint = self.config.endpoint.clone();
        let timeout = self.config.rpc_timeout;
        let expected = lease.clone();
        let state = self.runtime.block_on(async move {
            let mut client =
                tokio::time::timeout(timeout, GeometryRuntimeClient::connect(endpoint))
                    .await
                    .map_err(|_| "Geometry transport connect timed out".to_string())?
                    .map_err(|error| format!("Geometry transport connect failed: {error}"))?;
            let mut updates =
                tokio::time::timeout(timeout, client.subscribe_navigation_state(Empty {}))
                    .await
                    .map_err(|_| "SubscribeNavigationState timed out".to_string())?
                    .map_err(|error| format!("SubscribeNavigationState failed: {error}"))?
                    .into_inner();
            let initial = tokio::time::timeout(timeout, client.cancel_navigation(request))
                .await
                .map_err(|_| "CancelNavigation timed out".to_string())?
                .map_err(|error| format!("CancelNavigation failed: {error}"))?
                .into_inner();
            if initial.status != "CANCELLED" && initial.status != "CANCEL_PENDING" {
                return Err(cancel_rejected(&initial));
            }
            if !matches_lease(&initial, &expected) {
                return Err("CancelNavigation response does not match exact lease".to_string());
            }
            if initial.status == "CANCELLED" {
                return Ok(initial);
            }
            tokio::time::timeout(timeout, async {
                loop {
                    let update = updates
                        .message()
                        .await
                        .map_err(|error| format!("navigation state stream failed: {error}"))?
                        .ok_or_else(|| {
                            "navigation state stream ended before cancel terminal".to_string()
                        })?;
                    if !matches_lease(&update, &expected) {
                        continue;
                    }
                    match update.status.as_str() {
                        "CANCELLED" => return Ok(update),
                        "FAILED" => return Err(cancel_rejected(&update)),
                        _ => {}
                    }
                }
            })
            .await
            .map_err(|_| "CancelNavigation terminal wait timed out".to_string())?
        })?;
        Ok(NavigationOutcome::from_state(state))
    }
}

fn matches_lease(state: &NavigationState, lease: &AdmittedGoalLease) -> bool {
    state.owner_kind == PUBLIC_NAV_OWNER
        && state.goal_id == lease.goal_id
        && state.parent_request_id == lease.parent_request_id
        && state.goal_generation == lease.goal_generation
        && state.producer_runtime_generation == lease.producer_runtime_generation
        && state.goal_payload_sha256.as_slice() == lease.goal_payload_sha256
}

fn cancel_rejected(state: &NavigationState) -> String {
    format!(
        "CancelNavigation rejected: status={} reason_code={} message={}",
        state.status, state.reason_code, state.message
    )
}

fn adapt_observation(
    observation: &TargetObservation,
    admission: &NavigationAdmission,
    tracker_swaps_axes: bool,
) -> Result<NavigationGoal, String> {
    if observation.state != TargetObservationState::Tracked {
        return Err("only tracked TargetObservation can produce GoalIntent".into());
    }
    if observation.request_id.trim().is_empty()
        || observation.session_id.trim().is_empty()
        || observation.target_id.trim().is_empty()
        || observation.calibration_id.trim().is_empty()
    {
        return Err("observation identity is incomplete".into());
    }
    if observation.camera_frame != NavFrameId::Camera {
        return Err("observation frame is not canonical camera_optical_frame".into());
    }
    let transform = &admission.transform;
    if transform.source_frame_id != observation.camera_frame.as_str()
        || transform.target_frame_id != NavFrameId::Map.as_str()
        || transform.calibration_id != observation.calibration_id
    {
        return Err("frame or calibration identity does not match observation".into());
    }
    if transform.source_timestamp_ns == 0
        || transform.source_timestamp_ns != observation.source_timestamp_ns
    {
        return Err("transform snapshot timestamp does not match observation".into());
    }
    let pose = observation
        .target_pose
        .as_ref()
        .ok_or_else(|| "tracked observation has no target pose".to_string())?;
    if pose.frame_id != observation.camera_frame
        || pose.stamp != Timestamp::from_sensor(observation.source_timestamp_ns as i64)
    {
        return Err("target pose identity does not match observation".into());
    }
    let nonce = decode_nonce(&admission.motion_guard_boot_nonce_hex)?;
    if admission.motion_guard_generation == 0 {
        return Err("motion guard generation must be positive".into());
    }
    let rotation = normalized_quaternion(transform.rotation_xyzw)?;
    let mut point = pose.pose.position;
    if tracker_swaps_axes {
        point = Point3 {
            x: -point.y,
            y: -point.z,
            z: point.x,
        };
    }
    let rotated = rotate_point(rotation, point);
    let target = Point3 {
        x: transform.translation_xyz[0] + rotated.x,
        y: transform.translation_xyz[1] + rotated.y,
        z: transform.translation_xyz[2] + rotated.z,
    };
    if ![target.x, target.y, target.z]
        .iter()
        .all(|value| value.is_finite())
    {
        return Err("transformed goal is not finite".into());
    }
    let yaw =
        (target.y - transform.translation_xyz[1]).atan2(target.x - transform.translation_xyz[0]);
    let orientation = Quaternion::from_yaw(yaw);
    Ok(NavigationGoal {
        pose: Some(Pose {
            position: Some(Vec3 {
                x: target.x,
                y: target.y,
                z: target.z,
            }),
            orientation: Some(Quat {
                x: orientation.x,
                y: orientation.y,
                z: orientation.z,
                w: orientation.w,
            }),
            frame_id: transform.target_frame_id.clone(),
            timestamp_ns: observation.source_timestamp_ns,
        }),
        frame_id: transform.target_frame_id.clone(),
        goal_id: observation.request_id.clone(),
        parent_request_id: observation.session_id.clone(),
        goal_generation: 0,
        owner_kind: PUBLIC_NAV_OWNER,
        motion_guard_generation: admission.motion_guard_generation,
        motion_guard_boot_nonce: nonce.to_vec(),
        motion_stop_generation: admission.motion_stop_generation,
    })
}

fn decode_nonce(value: &str) -> Result<[u8; 16], String> {
    if value.len() != 32 {
        return Err("motion guard boot nonce must contain 32 hex characters".into());
    }
    let mut output = [0u8; 16];
    for (index, byte) in output.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&value[index * 2..index * 2 + 2], 16)
            .map_err(|_| "motion guard boot nonce is not hexadecimal".to_string())?;
    }
    Ok(output)
}

fn normalized_quaternion(values: [f64; 4]) -> Result<Quaternion, String> {
    if !values.iter().all(|value| value.is_finite()) {
        return Err("transform quaternion is not finite".into());
    }
    let norm = values.iter().map(|value| value * value).sum::<f64>().sqrt();
    if norm <= f64::EPSILON || (norm - 1.0).abs() > 1.0e-3 {
        return Err("transform quaternion is not normalized".into());
    }
    Ok(Quaternion {
        x: values[0] / norm,
        y: values[1] / norm,
        z: values[2] / norm,
        w: values[3] / norm,
    })
}

fn rotate_point(q: Quaternion, point: Point3) -> Point3 {
    let vector = Quaternion {
        x: point.x,
        y: point.y,
        z: point.z,
        w: 0.0,
    };
    let conjugate = Quaternion {
        x: -q.x,
        y: -q.y,
        z: -q.z,
        w: q.w,
    };
    let rotated = multiply(multiply(q, vector), conjugate);
    Point3 {
        x: rotated.x,
        y: rotated.y,
        z: rotated.z,
    }
}

fn multiply(left: Quaternion, right: Quaternion) -> Quaternion {
    Quaternion {
        x: left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        y: left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        z: left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
        w: left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{BoundingBox2D, CameraInfo, Pose3, PoseStamped};

    fn observation(state: TargetObservationState) -> TargetObservation {
        TargetObservation {
            request_id: "request-1".into(),
            session_id: "session-1".into(),
            target_id: "chair".into(),
            object_description: "chair".into(),
            source_timestamp_ns: 1_000_000_000,
            timestamp: 1.0,
            camera_frame: NavFrameId::Camera,
            calibration_id: "calibration-1".into(),
            camera_info: CameraInfo::new(2, 2, 1.0, 1.0, 1.0, 1.0),
            image_width: 2,
            image_height: 2,
            bbox: Some(BoundingBox2D::from_xyxy(0.0, 0.0, 2.0, 2.0).unwrap()),
            target_pose: Some(PoseStamped {
                frame_id: NavFrameId::Camera,
                stamp: Timestamp::from_sensor(1_000_000_000),
                pose: Pose3 {
                    position: Point3 {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: Quaternion::IDENTITY,
                },
            }),
            state,
        }
    }

    fn admission() -> NavigationAdmission {
        NavigationAdmission {
            transform: FrozenGoalTransform {
                source_frame_id: "camera_optical_frame".into(),
                target_frame_id: "map".into(),
                source_timestamp_ns: 1_000_000_000,
                calibration_id: "calibration-1".into(),
                translation_xyz: [2.0, 3.0, 0.0],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            motion_guard_generation: 7,
            motion_guard_boot_nonce_hex: "00112233445566778899aabbccddeeff".into(),
            motion_stop_generation: 4,
        }
    }

    #[test]
    fn tracked_observation_maps_identity_into_public_wire() {
        let goal = adapt_observation(
            &observation(TargetObservationState::Tracked),
            &admission(),
            true,
        )
        .unwrap();
        assert_eq!(goal.goal_id, "request-1");
        assert_eq!(goal.parent_request_id, "session-1");
        assert_eq!(goal.frame_id, "map");
        assert_eq!(goal.owner_kind, PUBLIC_NAV_OWNER);
        assert_eq!(goal.pose.unwrap().position.unwrap().x, 2.0);
    }

    #[test]
    fn source_to_target_rotation_uses_declared_direction() {
        let mut transform = admission();
        let half_turn = std::f64::consts::FRAC_PI_4;
        transform.transform.rotation_xyzw = [0.0, 0.0, half_turn.sin(), half_turn.cos()];
        let goal = adapt_observation(
            &observation(TargetObservationState::Tracked),
            &transform,
            false,
        )
        .unwrap();
        let position = goal.pose.unwrap().position.unwrap();
        assert!((position.x - 2.0).abs() < 1.0e-9);
        assert!((position.y - 4.0).abs() < 1.0e-9);
    }

    #[test]
    fn invalid_observations_fail_before_transport() {
        assert!(adapt_observation(
            &observation(TargetObservationState::Lost {
                frames: 1,
                terminal: false,
            }),
            &admission(),
            true,
        )
        .is_err());
        let mut wrong_calibration = admission();
        wrong_calibration.transform.calibration_id = "other".into();
        assert!(adapt_observation(
            &observation(TargetObservationState::Tracked),
            &wrong_calibration,
            true,
        )
        .is_err());
        let mut stale = admission();
        stale.transform.source_timestamp_ns = 1;
        assert!(
            adapt_observation(&observation(TargetObservationState::Tracked), &stale, true,)
                .is_err()
        );
    }

    #[test]
    fn cancel_terminal_requires_public_owner_and_exact_lease() {
        let lease = AdmittedGoalLease {
            goal_id: "request-1".into(),
            parent_request_id: "session-1".into(),
            goal_generation: 7,
            producer_runtime_generation: 8,
            goal_payload_sha256: [9; 32],
        };
        let mut state = NavigationState {
            owner_kind: PUBLIC_NAV_OWNER,
            goal_id: lease.goal_id.clone(),
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation,
            producer_runtime_generation: lease.producer_runtime_generation,
            goal_payload_sha256: lease.goal_payload_sha256.to_vec(),
            ..NavigationState::default()
        };
        assert!(matches_lease(&state, &lease));
        state.owner_kind = 2;
        assert!(!matches_lease(&state, &lease));
        state.owner_kind = PUBLIC_NAV_OWNER;
        state.goal_payload_sha256[0] ^= 1;
        assert!(!matches_lease(&state, &lease));
    }
}

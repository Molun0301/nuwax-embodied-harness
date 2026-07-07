//! Foundation 字段形态便捷构造器
//!
//! 历史 `local_types` 模块的"便捷构造函数"层:不引入任何新类型,仅提供
//! Foundation `PoseStamped` / `Path2D` / `Twist2` 的便捷构造与字段提取。
//!
//! - `make_pose_stamped(frame, x, y, yaw)` → `PoseStamped`(yaw 转为 quaternion)
//! - `make_pose_stamped_xy(frame, x, y)` → 同上(yaw=0)
//! - `make_path2d_empty()` → `Path2D`
//! - `make_path2d(points)` → `Path2D`
//! - `make_twist2_zero()` → `Twist2`(基础零)
//! - `make_twist2(linear_x, angular_z)` → `Twist2`(y=0)
//! - `pose_xy_yaw(pose)` → `(x, y, yaw)`(从 Foundation `PoseStamped` 提取扁平信息)
//! - `pose_xy(pose)` → `(x, y)`(从 Foundation `PoseStamped` 提取扁平信息)
//! - `pose_xyz(pose)` → `(x, y, z)`
//! - `pose_frame(pose)` → frame 字符串
//! - `pose_to_point2(pose)` / `pose_to_point3(pose)` → 位置提取
//! - `pose_distance(a, b)` → 2D 距离(米)

// Foundation 类型 pub re-export(让 `use crate::helpers::Point2` 等可以工作)。
// 当 source file 既需要 helper 函数(本模块)又需要 Foundation 类型时,
// 统一从 `crate::helpers` 引用,避免每个文件写两条 use。
pub use nuwax_nav_base::{NavError, PlannerDiagnostics};
pub use nuwax_nav_types_messages::{
    NavFrameId, NavigationStatus, Path2D, PlannerFailure, PlannerResult, Point2, Point3, Pose2,
    Pose3, PoseStamped, Quaternion, Twist2,
};

/// 由 `frame` 字符串 + `(x, y, yaw)` 构造 Foundation `PoseStamped`
#[must_use]
pub fn make_pose_stamped(frame: &str, x: f64, y: f64, yaw: f64) -> PoseStamped {
    let pose = Pose3::from_pose2(Pose2::new(x, y, yaw));
    PoseStamped::from_pose(pose, parse_frame(frame))
}

/// 由 `frame` + `(x, y)` 构造(yaw=0)
#[must_use]
pub fn make_pose_stamped_xy(frame: &str, x: f64, y: f64) -> PoseStamped {
    make_pose_stamped(frame, x, y, 0.0)
}

/// 空 Path2D
#[must_use]
pub fn make_path2d_empty() -> Path2D {
    Path2D::empty()
}

/// 由点列构造 Path2D
#[must_use]
pub fn make_path2d(points: Vec<Point2>) -> Path2D {
    Path2D::from_points(points)
}

/// 零速 Twist2
#[must_use]
pub fn make_twist2_zero() -> Twist2 {
    Twist2::ZERO
}

/// 由 `linear_x` + `angular_z` 构造(y 默认为 0)
#[must_use]
pub fn make_twist2(linear_x: f64, angular_z: f64) -> Twist2 {
    Twist2 {
        linear: Point2 {
            x: linear_x,
            y: 0.0,
        },
        angular_z,
    }
}

/// 从 Foundation `PoseStamped` 提取 `(x, y, yaw)`
///
/// yaw 取自 quaternion 在 z 轴的旋转;若 quaternion 非纯 z 轴旋转,返回的 yaw 为投影近似。
#[must_use]
pub fn pose_xy_yaw(p: &PoseStamped) -> (f64, f64, f64) {
    let pose2 = p.pose.to_pose2();
    (pose2.position.x, pose2.position.y, pose2.yaw)
}

/// 从 Foundation `PoseStamped` 提取 `(x, y)`(丢弃 yaw)
#[must_use]
pub fn pose_xy(p: &PoseStamped) -> (f64, f64) {
    (p.pose.position.x, p.pose.position.y)
}

/// 从 Foundation `PoseStamped` 提取 `(x, y, z)`
#[must_use]
pub fn pose_xyz(p: &PoseStamped) -> (f64, f64, f64) {
    (p.pose.position.x, p.pose.position.y, p.pose.position.z)
}

/// 从 Foundation `PoseStamped` 提取 frame 字符串
#[must_use]
pub fn pose_frame(p: &PoseStamped) -> String {
    p.frame_id.as_str().to_string()
}

/// Foundation `PoseStamped::position` → `Point2`(丢弃 z)
#[must_use]
pub fn pose_to_point2(p: &PoseStamped) -> Point2 {
    p.pose.position.to_2d()
}

/// Foundation `PoseStamped::position` → `Point3`
#[must_use]
pub fn pose_to_point3(p: &PoseStamped) -> Point3 {
    p.pose.position
}

/// 距离(米)
#[must_use]
pub fn pose_distance(a: &PoseStamped, b: &PoseStamped) -> f64 {
    pose_to_point2(a).distance(pose_to_point2(b))
}

// ============================================================================
// 内部 helpers
// ============================================================================

/// 把 `&str` frame 名解析为 `NavFrameId`(`"map"` → `Map`, `"odom"` → `Odom`,
/// `"base_link"` → `Base`, `"base_footprint"` → `Body`,
/// `"camera_optical_frame"` → `Camera`,其它 → `Custom(s)`)
fn parse_frame(frame: &str) -> NavFrameId {
    NavFrameId::from_static(frame)
}

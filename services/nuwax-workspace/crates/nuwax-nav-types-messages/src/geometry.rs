//! 几何/位姿/时间戳基础类型
//!
//! 全部时间字段一律使用纳秒;位姿和四元数提供 2D/3D 一致转换。

// `AbsDiffEq` 仅为 `abs_diff_eq!` 宏展开提供 trait scope,本身不直接使用。
#[allow(unused_imports)]
use approx::{abs_diff_eq, AbsDiffEq};
use nuwax_types::navigation::{
    ArrivalBoundary, GoalOptions, GoalReference, GoalRegionAssessment, GroundHeightIntent,
    GroundLayerResult,
};
use serde::{Deserialize, Serialize};

use crate::frames::NavFrameId;

/// 时钟种类(用于解释 `Timestamp::nanos` 的语义)。
///
/// - `Unix`:Unix epoch 纳秒(单调递增不严格保证)
///
/// - `Monotonic`:系统单调时钟(仅用于内部时序比较)
/// - `Sensor`:传感器时间(传感器/相机时钟域)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClockKind {
    /// Unix epoch 纳秒
    #[default]
    Unix,
    /// 系统单调时钟
    Monotonic,
    /// 传感器时钟
    Sensor,
}

/// 时间戳,统一纳秒。
///
/// 不暴露 `SystemTime` / `Instant`,序列化使用 i64 纳秒 + 时钟种类。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Timestamp {
    /// 自时钟起点起的纳秒数(`i64` 可表达约 ±292 年的范围)
    pub nanos: i64,
    /// 时钟种类
    pub clock: ClockKind,
}

impl Timestamp {
    /// 构造 Unix epoch 时间戳(秒 + 纳秒分量)
    #[must_use]
    pub const fn from_unix(secs: i64, subsec_nanos: u32) -> Self {
        Self {
            nanos: secs
                .saturating_mul(1_000_000_000)
                .saturating_add(subsec_nanos as i64),
            clock: ClockKind::Unix,
        }
    }

    /// 构造单调时钟时间戳
    #[must_use]
    pub const fn from_monotonic(nanos: i64) -> Self {
        Self {
            nanos,
            clock: ClockKind::Monotonic,
        }
    }

    /// 构造传感器时钟时间戳
    #[must_use]
    pub const fn from_sensor(nanos: i64) -> Self {
        Self {
            nanos,
            clock: ClockKind::Sensor,
        }
    }

    /// Unix 秒分量(向下取整)
    #[must_use]
    pub const fn unix_seconds(self) -> i64 {
        self.nanos / 1_000_000_000
    }

    /// Unix 纳秒分量(0..1e9)
    #[must_use]
    pub const fn unix_subsec_nanos(self) -> u32 {
        (self.nanos.rem_euclid(1_000_000_000)) as u32
    }

    /// 距离另一个时间戳的纳秒差(`self - other`)
    #[must_use]
    pub fn diff_nanos(self, other: Self) -> i64 {
        self.nanos - other.nanos
    }
}

impl Default for Timestamp {
    fn default() -> Self {
        Self {
            nanos: 0,
            clock: ClockKind::Unix,
        }
    }
}

/// 2D 点
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Point2 {
    /// x(米)
    pub x: f64,
    /// y(米)
    pub y: f64,
}

impl Point2 {
    /// 构造点
    #[must_use]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// 到另一个点的欧氏距离
    #[must_use]
    pub fn distance(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// 沿方向线性插值
    #[must_use]
    pub fn lerp(self, other: Self, t: f64) -> Self {
        Self {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
        }
    }
}

/// 数值逐项相等的有限三维位置；不通过距离平方推断相等。
///
/// `+0.0` 与 `-0.0` 表示同一位置。调用方仍负责原坐标系和来源资格；
/// 本函数不定义正容差的开闭边界，也不授予姿态、路径或运动许可。
#[must_use]
pub fn same_finite_position(actual: [f64; 3], goal: [f64; 3]) -> bool {
    actual == goal && actual.iter().all(|coordinate| coordinate.is_finite())
}

/// 零朝向容差的有限提取 yaw 等价；仅合并数值同值与精确的 ±PI 两端点。
///
/// 调用值来自已验证四元数的 `Quaternion::yaw`，范围为 [-PI, PI]。
/// 不先相减或 wrap，避免非零邻值在减法舍入后被当成零；不引入 epsilon。
/// 这不比较完整姿态，也不替代调用方的坐标系、来源、机体或停车资格。
#[must_use]
pub fn same_finite_yaw(actual: f64, goal: f64) -> bool {
    use std::f64::consts::PI;
    (actual == goal && actual.is_finite())
        || (actual == PI && goal == -PI)
        || (actual == -PI && goal == PI)
}

/// 纯到达区域的几何输入；当前尚未接入任何运行或公共目标入口。
///
/// 输入合法性、坐标系与来源资格由原边界负责；yaw 是已验证姿态提取的
/// [-PI, PI] 数值。Ground 分支没有机体 z 或固定高度偏移。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArrivalGeometry {
    /// 机体三维区域的小型数值分支，不消费地面层结论。
    Body {
        /// 当前机体 XYZ。
        actual_xyz: [f64; 3],
        /// 目标机体 XYZ。
        target_xyz: [f64; 3],
        /// 当前提取 yaw。
        actual_yaw: f64,
        /// 目标提取 yaw。
        target_yaw: f64,
    },
    /// 地面 XY/yaw 与原高度意图，层关系由 owner 单独传入。
    Ground {
        /// 当前 XY。
        actual_xy: [f64; 2],
        /// 目标 XY。
        target_xy: [f64; 2],
        /// 当前提取 yaw。
        actual_yaw: f64,
        /// 目标提取 yaw。
        target_yaw: f64,
        /// 原高度 presence 与独立解析参考。
        height: GroundHeightIntent,
    },
}

impl ArrivalGeometry {
    /// 由几何分支派生参考，避免另存一个可能与输入矛盾的 reference。
    #[must_use]
    pub const fn reference(&self) -> GoalReference {
        match self {
            Self::Body { .. } => GoalReference::Body,
            Self::Ground { .. } => GoalReference::Ground,
        }
    }
}

/// 只判断数值区域与 owner 传入的层结论，不创建实际消费或停车见证。
///
/// 零容差复用有限数值精确规则；正容差保留指定的严格/闭边界，yaw 差
/// 采用原停车数值路径的 sin/atan2 规则。此处没有替换或统一现有调用点。
/// Ground 的 resolved 数字不改变原 presence，也不能覆盖 typed 层结论。
/// `WithinRegion` 不等于实际到达或 Succeeded；运行接线留待后续迁移。
#[must_use]
pub fn assess_goal_region(
    geometry: &ArrivalGeometry,
    options: &GoalOptions,
    boundary: ArrivalBoundary,
    layer: GroundLayerResult,
) -> GoalRegionAssessment {
    let within = |error: f64, tolerance: f64| match boundary {
        ArrivalBoundary::Strict => error < tolerance,
        ArrivalBoundary::Closed => error <= tolerance,
    };
    let (position_ok, actual_yaw, target_yaw, height) = match geometry {
        ArrivalGeometry::Body {
            actual_xyz,
            target_xyz,
            actual_yaw,
            target_yaw,
        } => {
            let position_ok = if options.position_tolerance_m == 0.0 {
                same_finite_position(*actual_xyz, *target_xyz)
            } else {
                within(
                    Point3::new(actual_xyz[0], actual_xyz[1], actual_xyz[2])
                        .distance(Point3::new(target_xyz[0], target_xyz[1], target_xyz[2])),
                    options.position_tolerance_m,
                )
            };
            (position_ok, *actual_yaw, *target_yaw, None)
        }
        ArrivalGeometry::Ground {
            actual_xy,
            target_xy,
            actual_yaw,
            target_yaw,
            height,
        } => {
            let position_ok = if options.position_tolerance_m == 0.0 {
                // 第三分量仅复用逐项数值比较，不表示地面高度，也不进入 wire。
                same_finite_position(
                    [actual_xy[0], actual_xy[1], 0.0],
                    [target_xy[0], target_xy[1], 0.0],
                )
            } else {
                within(
                    Point2::new(actual_xy[0], actual_xy[1])
                        .distance(Point2::new(target_xy[0], target_xy[1])),
                    options.position_tolerance_m,
                )
            };
            (position_ok, *actual_yaw, *target_yaw, Some(height))
        }
    };
    let yaw_ok = !options.requires_orientation()
        || if options.yaw_tolerance_rad == 0.0 {
            same_finite_yaw(actual_yaw, target_yaw)
        } else {
            let delta = actual_yaw - target_yaw;
            within(
                delta.sin().atan2(delta.cos()).abs(),
                options.yaw_tolerance_rad,
            )
        };
    if !position_ok || !yaw_ok {
        return GoalRegionAssessment::OutsideRegion;
    }
    match height {
        Some(_) if layer == GroundLayerResult::KnownConflict => {
            GoalRegionAssessment::KnownLayerConflict
        }
        Some(height)
            if height.requested_ground_z.is_some() && layer == GroundLayerResult::Unresolved =>
        {
            GoalRegionAssessment::TargetLayerUnresolved
        }
        _ => GoalRegionAssessment::WithinRegion,
    }
}

/// 3D 点
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Point3 {
    /// x(米)
    pub x: f64,
    /// y(米)
    pub y: f64,
    /// z(米)
    pub z: f64,
}

impl Point3 {
    /// 构造点
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// 到另一个点的欧氏距离
    #[must_use]
    pub fn distance(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// 转为 2D 点(丢弃 z)
    #[must_use]
    pub const fn to_2d(self) -> Point2 {
        Point2 {
            x: self.x,
            y: self.y,
        }
    }
}

/// 四元数(w, x, y, z)，字段与反序列化本身不执行归一化。
///
/// 仅承载数学结构;不直接与 `nuwax-tf` 类型互转(由调用方按需映射)。
///
/// 默认值为全零 `(w=0, x=0, y=0, z=0)`，没有定义朝向；无旋转请使用 `Quaternion::IDENTITY`。
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    /// w 分量
    pub w: f64,
    /// x 分量
    pub x: f64,
    /// y 分量
    pub y: f64,
    /// z 分量
    pub z: f64,
}

impl Quaternion {
    /// 单位四元数
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// 绕 z 轴的 yaw 旋转
    #[must_use]
    pub fn from_yaw(yaw: f64) -> Self {
        let half = yaw * 0.5;
        Self {
            w: half.cos(),
            x: 0.0,
            y: 0.0,
            z: half.sin(),
        }
    }

    /// 提取三维姿态的偏航角（绕 z 轴，弧度）。
    /// 有限非零输入先归一化；全零或非有限输入返回非数，调用方须拒绝无效朝向。
    #[must_use]
    pub fn yaw(self) -> f64 {
        let components = [self.w, self.x, self.y, self.z];
        if components.iter().any(|value| !value.is_finite()) {
            return f64::NAN;
        }
        let scale = components.into_iter().map(f64::abs).fold(0.0, f64::max);
        if scale == 0.0 {
            return f64::NAN;
        }
        // 先缩放到最大分量为一，避免非单位输入在求范数时上溢或下溢。
        let (w, x, y, z) = (
            self.w / scale,
            self.x / scale,
            self.y / scale,
            self.z / scale,
        );
        let norm = (w * w + x * x + y * y + z * z).sqrt();
        let (w, x, y, z) = (w / norm, x / norm, y / norm, z / norm);
        let sin_yaw = 2.0 * (w * z + x * y);
        let cos_yaw = 1.0 - 2.0 * (y * y + z * z);
        sin_yaw.atan2(cos_yaw)
    }
}

/// 2D 位姿(平面坐标 + yaw)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose2 {
    /// 位置
    pub position: Point2,
    /// 朝向(yaw,弧度)
    pub yaw: f64,
}

impl Pose2 {
    /// 构造
    #[must_use]
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self {
            position: Point2 { x, y },
            yaw,
        }
    }
}

impl Default for Pose2 {
    fn default() -> Self {
        Self {
            position: Point2::default(),
            yaw: 0.0,
        }
    }
}

/// 3D 位姿
///
/// 默认值为原点与未定义的全零四元数；有效位姿须显式提供朝向。
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Pose3 {
    /// 位置
    pub position: Point3,
    /// 朝向
    pub orientation: Quaternion,
}

impl Pose3 {
    /// 由 2D 位姿提升(保留 yaw,z=0)
    #[must_use]
    pub fn from_pose2(pose: Pose2) -> Self {
        Self {
            position: Point3 {
                x: pose.position.x,
                y: pose.position.y,
                z: 0.0,
            },
            orientation: Quaternion::from_yaw(pose.yaw),
        }
    }

    /// 投影为 2D 位姿(yaw 来自 quaternion)
    #[must_use]
    pub fn to_pose2(self) -> Pose2 {
        Pose2 {
            position: Point2 {
                x: self.position.x,
                y: self.position.y,
            },
            yaw: self.orientation.yaw(),
        }
    }
}

/// 带坐标系与时间戳的位姿
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseStamped {
    /// 坐标系
    pub frame_id: NavFrameId,
    /// 时间戳
    pub stamp: Timestamp,
    /// 位姿
    pub pose: Pose3,
}

impl PoseStamped {
    /// 构造默认值,frame/map,时间为零
    #[must_use]
    pub fn from_pose(pose: Pose3, frame_id: NavFrameId) -> Self {
        Self {
            frame_id,
            stamp: Timestamp::default(),
            pose,
        }
    }
}

/// 2D 速度(线速度 + yaw 速率)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Twist2 {
    /// 平面线速度(米/秒)
    pub linear: Point2,
    /// yaw 角速度(弧度/秒)
    pub angular_z: f64,
}

impl Twist2 {
    /// 零速度
    pub const ZERO: Self = Self {
        linear: Point2 { x: 0.0, y: 0.0 },
        angular_z: 0.0,
    };

    /// 是否全部接近零
    #[must_use]
    pub fn is_zero(self, eps: f64) -> bool {
        self.linear.x.abs() < eps && self.linear.y.abs() < eps && self.angular_z.abs() < eps
    }

    /// 限幅:`linear` 限到 `max_linear`、`angular_z` 限到 `max_angular`
    #[must_use]
    pub fn clamp(self, max_linear: f64, max_angular: f64) -> Self {
        let m = self.linear.x.hypot(self.linear.y).max(f64::EPSILON);
        let scale = if m > max_linear { max_linear / m } else { 1.0 };
        Self {
            linear: Point2 {
                x: self.linear.x * scale,
                y: self.linear.y * scale,
            },
            angular_z: self.angular_z.clamp(-max_angular, max_angular),
        }
    }
}

impl Default for Twist2 {
    fn default() -> Self {
        Self::ZERO
    }
}

/// 3D 速度
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Twist3 {
    /// 线速度
    pub linear: Point3,
    /// 角速度
    pub angular: Point3,
}

impl Twist3 {
    /// 零速度
    pub const ZERO: Self = Self {
        linear: Point3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        angular: Point3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    };

    /// 是否全部接近零
    #[must_use]
    pub fn is_zero(self, eps: f64) -> bool {
        abs_diff_eq!(self.linear.x, 0.0, epsilon = eps)
            && abs_diff_eq!(self.linear.y, 0.0, epsilon = eps)
            && abs_diff_eq!(self.linear.z, 0.0, epsilon = eps)
            && abs_diff_eq!(self.angular.x, 0.0, epsilon = eps)
            && abs_diff_eq!(self.angular.y, 0.0, epsilon = eps)
            && abs_diff_eq!(self.angular.z, 0.0, epsilon = eps)
    }
}

impl Default for Twist3 {
    fn default() -> Self {
        Self::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timestamp_from_unix_decomposes() {
        let t = Timestamp::from_unix(1, 500_000_000);
        assert_eq!(t.unix_seconds(), 1);
        assert_eq!(t.unix_subsec_nanos(), 500_000_000);
        assert_eq!(t.clock, ClockKind::Unix);
    }

    #[test]
    fn timestamp_diff_nanos_is_signed() {
        let a = Timestamp::from_unix(2, 0);
        let b = Timestamp::from_unix(1, 0);
        assert_eq!(a.diff_nanos(b), 1_000_000_000);
        assert_eq!(b.diff_nanos(a), -1_000_000_000);
    }

    #[test]
    fn point2_distance_and_lerp() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(3.0, 4.0);
        assert!((a.distance(b) - 5.0).abs() < 1e-9);
        assert!((a.lerp(b, 0.5).distance(Point2::new(1.5, 2.0))) < 1e-9);
    }

    #[test]
    fn point3_to_2d_drops_z() {
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(p.to_2d(), Point2::new(1.0, 2.0));
    }

    #[test]
    fn quaternion_yaw_roundtrip() {
        let yaw = 0.7_f64;
        let q = Quaternion::from_yaw(yaw);
        assert!((q.yaw() - yaw).abs() < 1e-9);
        // identity yaw
        assert!(Quaternion::IDENTITY.yaw().abs() < 1e-9);
    }

    #[test]
    fn quaternion_yaw_preserves_tilted_attitude_under_nonzero_scaling() {
        for (roll, pitch, yaw) in [(0.4, -0.5, 1.2), (-1.0, 0.7, -2.4)] {
            // 用已有旋转库生成三轴姿态，期望值直接取原始偏航角。
            let rotation = nalgebra::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
            let unit = rotation.quaternion();
            for scale in [1.0, 3.5, -2.0, 1e300, -1e-300] {
                let quaternion = Quaternion {
                    w: unit.w * scale,
                    x: unit.i * scale,
                    y: unit.j * scale,
                    z: unit.k * scale,
                };
                let actual = quaternion.yaw();
                assert!(
                    (actual - yaw).abs() < 1e-12,
                    "混合姿态或非单位倍率改变偏航：roll={roll}, pitch={pitch}, yaw={yaw}, scale={scale}, actual={actual}"
                );
                let projected = Pose3 {
                    position: Point3::new(1.0, 2.0, 3.0),
                    orientation: quaternion,
                }
                .to_pose2();
                assert!((projected.yaw - yaw).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn quaternion_yaw_marks_undefined_orientation_as_nonfinite() {
        for quaternion in [
            Quaternion::default(),
            Quaternion {
                x: f64::NAN,
                ..Quaternion::IDENTITY
            },
            Quaternion {
                w: f64::INFINITY,
                ..Quaternion::IDENTITY
            },
        ] {
            assert!(quaternion.yaw().is_nan(), "非法姿态不能伪装为零偏航");
        }
    }

    #[test]
    fn pose3_pose2_consistent_xy_yaw() {
        let p2 = Pose2::new(1.5, -2.5, 0.3);
        let p3 = Pose3::from_pose2(p2);
        let back = p3.to_pose2();
        assert!((back.position.x - p2.position.x).abs() < 1e-9);
        assert!((back.position.y - p2.position.y).abs() < 1e-9);
        assert!((back.yaw - p2.yaw).abs() < 1e-9);
    }

    #[test]
    fn twist2_clamp_zero_passthrough() {
        let t = Twist2 {
            linear: Point2::new(2.0, 0.0),
            angular_z: 5.0,
        };
        let c = t.clamp(1.0, 1.0);
        assert!(c.linear.x.abs() <= 1.0 + 1e-9);
        assert!(c.angular_z.abs() <= 1.0 + 1e-9);
    }

    #[test]
    fn twist2_is_zero() {
        assert!(Twist2::ZERO.is_zero(1e-9));
        assert!(!Twist2 {
            linear: Point2::new(0.1, 0.0),
            angular_z: 0.0,
        }
        .is_zero(1e-9));
    }

    #[test]
    fn timestamp_serde_snake_case() {
        let t = Timestamp::from_monotonic(42);
        let s = serde_json::to_string(&t).unwrap();
        assert!(s.contains("\"monotonic\""));
        let back: Timestamp = serde_json::from_str(&s).unwrap();
        assert_eq!(back, t);
    }
}

// DRAFT ONLY: append a private test module to geometry.rs.
#[cfg(test)]
mod exact_position_tests {
    use super::*;
    #[test]
    fn exact_finite_position_never_uses_an_underflowed_norm() {
        let zero = [0.0; 3];
        assert!(same_finite_position(zero, [-0.0; 3]));
        for delta in [f64::from_bits(1), f64::MIN_POSITIVE, 1e-200, f64::EPSILON] {
            for axis in 0..3 {
                let mut other = zero;
                other[axis] = delta;
                assert_ne!(other[axis], zero[axis]);
                assert!(!same_finite_position(zero, other));
                assert!(!same_finite_position(other, zero));
                if delta <= 1e-200 {
                    assert_eq!(Point3::new(other[0], other[1], other[2])
                        .distance(Point3::new(0.0, 0.0, 0.0)), 0.0);
                }
            }
        }
        for invalid in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            assert!(!same_finite_position([invalid, 0.0, 0.0], [invalid, 0.0, 0.0]));
        }
    }
}

#[cfg(test)]
mod exact_yaw_tests {
    use super::*;
    #[test]
    fn exact_finite_yaw_preserves_branch_neighbors_and_nonzero_subnormals() {
        let pi = std::f64::consts::PI;
        assert!(same_finite_yaw(0.0, -0.0));
        assert!(same_finite_yaw(pi, -pi));
        assert!(same_finite_yaw(-pi, pi));
        let inside_pi = f64::from_bits(pi.to_bits() - 1);
        assert!(inside_pi < pi);
        for value in [inside_pi, -inside_pi] {
            assert!(same_finite_yaw(value, value));
            assert!(!same_finite_yaw(value, pi));
            assert!(!same_finite_yaw(pi, value));
            assert!(!same_finite_yaw(value, -pi));
            assert!(!same_finite_yaw(-pi, value));
        }
        // A subtraction-first implementation can round this nonendpoint pair
        // to TAU, then incorrectly call its wrapped difference exact zero.
        assert_eq!(pi - (-inside_pi), 2.0 * pi);
        for value in [f64::from_bits(1), -f64::from_bits(1), f64::MIN_POSITIVE] {
            assert_ne!(value, 0.0);
            assert!(same_finite_yaw(value, value));
            assert!(!same_finite_yaw(value, 0.0));
            assert!(!same_finite_yaw(0.0, value));
        }
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(!same_finite_yaw(invalid, invalid));
            assert!(!same_finite_yaw(invalid, 0.0));
            assert!(!same_finite_yaw(0.0, invalid));
            assert!(!same_finite_yaw(invalid, pi));
            assert!(!same_finite_yaw(-pi, invalid));
        }
        let plus = Quaternion { x: 0.0, y: 0.0, z: 1.0, w: 0.0 };
        for q in [Quaternion::IDENTITY, plus] {
            let antipode = Quaternion { x: -q.x, y: -q.y, z: -q.z, w: -q.w };
            assert_eq!(q.yaw(), antipode.yaw());
            assert!(same_finite_yaw(q.yaw(), antipode.yaw()));
        }
        let minus = Quaternion { x: -0.0, y: 0.0, z: -1.0, w: 0.0 };
        assert_eq!(plus.yaw(), pi);
        assert_eq!(minus.yaw(), -pi);
        assert!(same_finite_yaw(plus.yaw(), minus.yaw()));
        let invalid_yaw = Quaternion::default().yaw();
        assert!(invalid_yaw.is_nan());
        assert!(!same_finite_yaw(invalid_yaw, invalid_yaw));
    }
}

// A01–A12 are pure typed-result oracles. Matched is supplied by these tests;
// no test here claims a real layer observation, consumed segment or stop proof.
#[cfg(test)]
mod ground_region_tests {
    use super::*;

    fn options() -> GoalOptions {
        GoalOptions {
            allow_reverse: false,
            position_tolerance_m: 0.0,
            yaw_tolerance_rad: 0.0,
        }
    }

    fn ground(requested: Option<f64>, resolved: Option<f64>) -> ArrivalGeometry {
        ArrivalGeometry::Ground {
            actual_xy: [2.0, 3.0],
            target_xy: [2.0, 3.0],
            actual_yaw: 0.0,
            target_yaw: 0.0,
            height: GroundHeightIntent {
                requested_ground_z: requested,
                resolved_ground_z: resolved,
            },
        }
    }

    fn height_of(geometry: &ArrivalGeometry) -> &GroundHeightIntent {
        match geometry {
            ArrivalGeometry::Ground { height, .. } => height,
            ArrivalGeometry::Body { .. } => panic!("expected ground test input"),
        }
    }

    fn assess(geometry: &ArrivalGeometry, layer: GroundLayerResult) -> GoalRegionAssessment {
        assess_goal_region(geometry, &options(), ArrivalBoundary::Strict, layer)
    }

    #[test]
    fn a01_omitted_unresolved_height_keeps_xy_region() {
        let geometry = ground(None, None);
        assert_eq!(geometry.reference(), GoalReference::Ground);
        assert_eq!(
            assess(&geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::WithinRegion
        );
        assert_eq!(
            *height_of(&geometry),
            GroundHeightIntent {
                requested_ground_z: None,
                resolved_ground_z: None
            }
        );
    }

    #[test]
    fn a02_resolved_zero_does_not_make_omission_explicit() {
        let geometry = ground(None, Some(0.0));
        assert_eq!(
            assess(&geometry, GroundLayerResult::Matched),
            GoalRegionAssessment::WithinRegion
        );
        // A parsed number must not silently turn the omitted request into a constraint.
        assert_eq!(
            assess(&geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::WithinRegion
        );
        assert_eq!(height_of(&geometry).requested_ground_z, None);
        assert_eq!(
            height_of(&geometry).resolved_ground_z.map(f64::to_bits),
            Some(0.0f64.to_bits())
        );
    }

    #[test]
    fn a03_explicit_zero_unresolved_is_not_within_region() {
        let geometry = ground(Some(0.0), None);
        assert_eq!(
            assess(&geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::TargetLayerUnresolved
        );
        assert_eq!(height_of(&geometry).requested_ground_z, Some(0.0));
        assert_eq!(height_of(&geometry).resolved_ground_z, None);
    }

    #[test]
    fn a04_equal_height_numbers_do_not_resolve_layer() {
        let geometry = ground(Some(3.0), Some(3.0));
        assert_eq!(
            assess(&geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::TargetLayerUnresolved
        );
        assert_eq!(
            height_of(&geometry).requested_ground_z,
            height_of(&geometry).resolved_ground_z
        );
    }

    #[test]
    fn a05_owner_matched_result_requires_no_fresh_observation_field() {
        // Supplied typed result only: this does not construct a carried/executed proof.
        for resolved in [None, Some(3.0)] {
            assert_eq!(
                assess(&ground(Some(3.0), resolved), GroundLayerResult::Matched),
                GoalRegionAssessment::WithinRegion
            );
        }
    }

    #[test]
    fn a06_known_layer_conflict_wins_at_same_xy() {
        for requested in [None, Some(0.0), Some(3.0)] {
            assert_eq!(
                assess(
                    &ground(requested, Some(3.0)),
                    GroundLayerResult::KnownConflict
                ),
                GoalRegionAssessment::KnownLayerConflict
            );
        }
    }

    #[test]
    fn a07_requested_height_presence_and_bits_are_preserved() {
        for requested in [0.0f64, -0.0, -0.25, 2.75] {
            let geometry = ground(Some(requested), Some(7.0));
            assert_eq!(
                assess(&geometry, GroundLayerResult::Unresolved),
                GoalRegionAssessment::TargetLayerUnresolved
            );
            assert_eq!(
                height_of(&geometry).requested_ground_z.map(f64::to_bits),
                Some(requested.to_bits())
            );
            assert_eq!(height_of(&geometry).resolved_ground_z, Some(7.0));
            assert_ne!(height_of(&geometry).requested_ground_z, None);
        }
        assert_ne!(
            GroundHeightIntent {
                requested_ground_z: None,
                resolved_ground_z: Some(0.0)
            },
            GroundHeightIntent {
                requested_ground_z: Some(0.0),
                resolved_ground_z: Some(0.0)
            }
        );
    }

    #[test]
    fn a08_zero_xy_never_uses_an_underflowed_distance() {
        let mut geometry = ground(None, None);
        if let ArrivalGeometry::Ground {
            actual_xy,
            target_xy,
            ..
        } = &mut geometry
        {
            *actual_xy = [-0.0, 0.0];
            *target_xy = [0.0, -0.0];
        }
        assert_eq!(
            assess(&geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::WithinRegion
        );
        for delta in [f64::from_bits(1), 1e-200, f64::EPSILON] {
            for axis in 0..2 {
                let mut actual = [0.0; 2];
                actual[axis] = delta;
                assert_ne!(actual[axis], 0.0);
                if let ArrivalGeometry::Ground {
                    actual_xy,
                    target_xy,
                    ..
                } = &mut geometry
                {
                    *actual_xy = actual;
                    *target_xy = [0.0; 2];
                }
                let distance = Point2::new(actual[0], actual[1]).distance(Point2::new(0.0, 0.0));
                if delta <= 1e-200 {
                    assert_eq!(distance, 0.0);
                }
                for boundary in [ArrivalBoundary::Strict, ArrivalBoundary::Closed] {
                    assert_eq!(
                        assess_goal_region(
                            &geometry,
                            &options(),
                            boundary,
                            GroundLayerResult::Unresolved
                        ),
                        GoalRegionAssessment::OutsideRegion
                    );
                }
                eprintln!(
                    "A08 axis={axis} delta_bits={} distance_bits={}",
                    delta.to_bits(),
                    distance.to_bits()
                );
            }
        }
    }

    #[test]
    fn a09_body_checks_z_while_ground_has_no_body_z() {
        let ground_geometry = ground(None, None);
        let body = ArrivalGeometry::Body {
            actual_xyz: [2.0, 3.0, f64::EPSILON],
            target_xyz: [2.0, 3.0, 0.0],
            actual_yaw: 0.0,
            target_yaw: 0.0,
        };
        assert_eq!(body.reference(), GoalReference::Body);
        assert_eq!(
            assess(&body, GroundLayerResult::Matched),
            GoalRegionAssessment::OutsideRegion
        );
        assert_eq!(
            assess(&ground_geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::WithinRegion
        );
        let matching_body = ArrivalGeometry::Body {
            actual_xyz: [2.0, 3.0, 0.0],
            target_xyz: [2.0, 3.0, 0.0],
            actual_yaw: 0.0,
            target_yaw: 0.0,
        };
        for layer in [
            GroundLayerResult::Matched,
            GroundLayerResult::Unresolved,
            GroundLayerResult::KnownConflict,
        ] {
            assert_eq!(
                assess(&matching_body, layer),
                GoalRegionAssessment::WithinRegion
            );
        }
    }

    #[test]
    fn a10_zero_yaw_reuses_exact_endpoints_without_neighbor_rounding() {
        let pi = std::f64::consts::PI;
        let plus = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 1.0,
            w: 0.0,
        };
        let minus = Quaternion {
            x: -0.0,
            y: 0.0,
            z: -1.0,
            w: 0.0,
        };
        assert_eq!(plus.yaw(), pi);
        assert_eq!(minus.yaw(), -pi);
        let inside_pi = f64::from_bits(pi.to_bits() - 1);
        assert_ne!(inside_pi, pi);
        assert_eq!(pi - (-inside_pi), 2.0 * pi);
        let mut pairs = vec![
            (pi, -pi, true),
            (-pi, pi, true),
            (0.0, -0.0, true),
            (pi, -inside_pi, false),
            (-inside_pi, pi, false),
            (inside_pi, -pi, false),
            (-pi, inside_pi, false),
            (1e-7, 0.0, false),
            (f64::from_bits(1), 0.0, false),
        ];
        // Subnormal is supplied directly: from_yaw would halve and round the smallest one to zero.
        for q in [Quaternion::IDENTITY, plus, minus] {
            let opposite = Quaternion {
                x: -q.x,
                y: -q.y,
                z: -q.z,
                w: -q.w,
            };
            assert_eq!(q.yaw(), opposite.yaw());
            pairs.push((q.yaw(), opposite.yaw(), true));
        }
        for (actual, target, expected) in pairs {
            let mut geometry = ground(None, None);
            if let ArrivalGeometry::Ground {
                actual_yaw,
                target_yaw,
                ..
            } = &mut geometry
            {
                *actual_yaw = actual;
                *target_yaw = target;
            }
            let body = ArrivalGeometry::Body {
                actual_xyz: [0.0; 3],
                target_xyz: [0.0; 3],
                actual_yaw: actual,
                target_yaw: target,
            };
            for input in [geometry, body] {
                for boundary in [ArrivalBoundary::Strict, ArrivalBoundary::Closed] {
                    let result = assess_goal_region(
                        &input,
                        &options(),
                        boundary,
                        GroundLayerResult::Unresolved,
                    );
                    assert_eq!(
                        result,
                        if expected {
                            GoalRegionAssessment::WithinRegion
                        } else {
                            GoalRegionAssessment::OutsideRegion
                        }
                    );
                }
            }
            eprintln!(
                "A10 actual_bits={} target_bits={} equal={expected}",
                actual.to_bits(),
                target.to_bits()
            );
        }
    }

    #[test]
    fn a11_positive_tolerance_keeps_strict_and_closed_boundaries() {
        let mut geometry = ground(None, None);
        if let ArrivalGeometry::Ground { actual_xy, .. } = &mut geometry {
            actual_xy[0] = 2.25;
        }
        let body = ArrivalGeometry::Body {
            actual_xyz: [2.25, 3.0, 0.0],
            target_xyz: [2.0, 3.0, 0.0],
            actual_yaw: 0.0,
            target_yaw: 0.0,
        };
        let positive = GoalOptions {
            position_tolerance_m: 0.25,
            ..options()
        };
        for input in [geometry, body] {
            assert_eq!(
                assess_goal_region(
                    &input,
                    &positive,
                    ArrivalBoundary::Strict,
                    GroundLayerResult::Unresolved
                ),
                GoalRegionAssessment::OutsideRegion
            );
            assert_eq!(
                assess_goal_region(
                    &input,
                    &positive,
                    ArrivalBoundary::Closed,
                    GroundLayerResult::Unresolved
                ),
                GoalRegionAssessment::WithinRegion
            );
        }
        // Also preserve the original positive-yaw wrap and optional-orientation policy.
        let mut yaw_geometry = ground(None, None);
        if let ArrivalGeometry::Ground { actual_yaw, .. } = &mut yaw_geometry {
            *actual_yaw = 0.25;
        }
        let error = 0.25f64.sin().atan2(0.25f64.cos()).abs();
        assert_eq!(error, 0.25);
        let yaw_options = GoalOptions {
            yaw_tolerance_rad: error,
            ..options()
        };
        assert_eq!(
            assess_goal_region(
                &yaw_geometry,
                &yaw_options,
                ArrivalBoundary::Strict,
                GroundLayerResult::Unresolved
            ),
            GoalRegionAssessment::OutsideRegion
        );
        assert_eq!(
            assess_goal_region(
                &yaw_geometry,
                &yaw_options,
                ArrivalBoundary::Closed,
                GroundLayerResult::Unresolved
            ),
            GoalRegionAssessment::WithinRegion
        );
        if let ArrivalGeometry::Ground { actual_yaw, .. } = &mut yaw_geometry {
            *actual_yaw = std::f64::consts::PI;
        }
        for tolerance in [std::f64::consts::PI, std::f64::consts::TAU] {
            let optional = GoalOptions {
                yaw_tolerance_rad: tolerance,
                ..options()
            };
            assert_eq!(
                assess_goal_region(
                    &yaw_geometry,
                    &optional,
                    ArrivalBoundary::Strict,
                    GroundLayerResult::Unresolved
                ),
                GoalRegionAssessment::WithinRegion
            );
        }
    }

    #[test]
    fn a12_geometry_precedes_explicit_unresolved_layer() {
        let mut geometry = ground(Some(3.0), None);
        if let ArrivalGeometry::Ground { actual_xy, .. } = &mut geometry {
            actual_xy[0] = 1.0;
        }
        assert_eq!(
            assess(&geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::OutsideRegion
        );
        if let ArrivalGeometry::Ground {
            actual_xy,
            actual_yaw,
            ..
        } = &mut geometry
        {
            *actual_xy = [2.0, 3.0];
            *actual_yaw = 0.1;
        }
        assert_eq!(
            assess(&geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::OutsideRegion
        );
        if let ArrivalGeometry::Ground { actual_yaw, .. } = &mut geometry {
            *actual_yaw = 0.0;
        }
        assert_eq!(
            assess(&geometry, GroundLayerResult::Unresolved),
            GoalRegionAssessment::TargetLayerUnresolved
        );
        assert_eq!(height_of(&geometry).requested_ground_z, Some(3.0));
        assert_eq!(height_of(&geometry).resolved_ground_z, None);
    }
}

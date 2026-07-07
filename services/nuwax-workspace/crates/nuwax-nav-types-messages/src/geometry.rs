//! 几何/位姿/时间戳基础类型
//!
//! 全部时间字段一律使用纳秒;位姿和四元数提供 2D/3D 一致转换。

// `AbsDiffEq` 仅为 `abs_diff_eq!` 宏展开提供 trait scope,本身不直接使用。
#[allow(unused_imports)]
use approx::{abs_diff_eq, AbsDiffEq};
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

/// 四元数(w, x, y, z),归一化。
///
/// 仅承载数学结构;不直接与 `nuwax-tf` 类型互转(由调用方按需映射)。
///
/// 默认值为全零 `(w=0, x=0, y=0, z=0)`(非单位四元数,需要主动 `Quaternion::IDENTITY` 归一化)。
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

    /// 提取 yaw(z 轴旋转)
    #[must_use]
    pub fn yaw(self) -> f64 {
        let sinp = 2.0 * (self.w * self.z);
        let cosp = 1.0 - 2.0 * (self.z * self.z);
        sinp.atan2(cosp)
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
/// 默认值为原点 + 单位朝向。
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

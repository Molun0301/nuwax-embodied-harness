//! 路径与控制命令
//!
//! - `Path2D/3D` 是规划结果(以及后续 path follower 输入)
//! - `ControllerCommand` 是机器人控制命令(限幅函数可用)

use serde::{Deserialize, Serialize};

use crate::geometry::{Point2, PoseStamped, Twist2, Twist3};

/// 2D 路径(世界坐标点列)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct Path2D {
    /// 点列(按通过顺序)
    pub points: Vec<Point2>,
}

impl Path2D {
    /// 空路径
    #[must_use]
    pub fn empty() -> Self {
        Self { points: Vec::new() }
    }

    /// 是否为空(0 点)
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// 点数
    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// 路径总长度(0/1 点返回 0)
    #[must_use]
    pub fn length(&self) -> f64 {
        if self.points.len() < 2 {
            return 0.0;
        }
        let mut total = 0.0;
        for w in self.points.windows(2) {
            let a = w[0];
            let b = w[1];
            total += a.distance(b);
        }
        total
    }

    /// 由点列构造
    #[must_use]
    pub fn from_points(points: Vec<Point2>) -> Self {
        Self { points }
    }
}

/// 3D 路径(带坐标系与时间戳的 pose 列)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct Path3D {
    /// pose 列
    pub poses: Vec<PoseStamped>,
}

impl Path3D {
    /// 空路径
    #[must_use]
    pub fn empty() -> Self {
        Self { poses: Vec::new() }
    }

    /// 是否为空
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }

    /// pose 数
    #[must_use]
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    /// 路径总长度(累加相邻 pose 的 position 距离)
    #[must_use]
    pub fn length(&self) -> f64 {
        if self.poses.len() < 2 {
            return 0.0;
        }
        let mut total = 0.0;
        for w in self.poses.windows(2) {
            total += w[0].pose.position.distance(w[1].pose.position);
        }
        total
    }
}

/// 控制器命令
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControllerCommand {
    /// 2D twist(平面控制,常用)
    pub twist: Twist2,
    /// 命令持续时间(秒;None 表示单步 / 由调用方决定)
    pub duration_s: Option<f64>,
    /// 坐标系
    pub frame_id: crate::frames::NavFrameId,
    /// 是否为显式停止(zero twist 之外的语义)
    pub stop: bool,
}

impl ControllerCommand {
    /// 构造 2D 命令
    #[must_use]
    pub fn twist_2d(twist: Twist2, frame_id: crate::frames::NavFrameId) -> Self {
        Self {
            twist,
            duration_s: None,
            frame_id,
            stop: false,
        }
    }

    /// 显式停止
    #[must_use]
    pub fn stop(frame_id: crate::frames::NavFrameId) -> Self {
        Self {
            twist: Twist2::ZERO,
            duration_s: None,
            frame_id,
            stop: true,
        }
    }

    /// 对 linear / angular 限幅
    #[must_use]
    pub fn clamped(self, max_linear: f64, max_angular: f64) -> Self {
        Self {
            twist: self.twist.clamp(max_linear, max_angular),
            ..self
        }
    }
}

/// 3D 控制器命令(可选,用于 3D 控制 / MLS 路径跟随)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControllerCommand3D {
    /// 3D twist
    pub twist: Twist3,
    /// 命令持续时间(秒)
    pub duration_s: Option<f64>,
    /// 坐标系
    pub frame_id: crate::frames::NavFrameId,
    /// 显式停止
    pub stop: bool,
}

impl ControllerCommand3D {
    /// 显式停止
    #[must_use]
    pub fn stop(frame_id: crate::frames::NavFrameId) -> Self {
        Self {
            twist: Twist3::ZERO,
            duration_s: None,
            frame_id,
            stop: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::NavFrameId;

    #[test]
    fn path2d_length_zero_for_empty_or_single() {
        let p = Path2D::empty();
        assert_eq!(p.length(), 0.0);
        let p = Path2D::from_points(vec![Point2::new(1.0, 1.0)]);
        assert_eq!(p.length(), 0.0);
    }

    #[test]
    fn path2d_length_accumulates() {
        let p = Path2D::from_points(vec![
            Point2::new(0.0, 0.0),
            Point2::new(3.0, 4.0),
            Point2::new(3.0, 8.0),
        ]);
        assert!((p.length() - 9.0).abs() < 1e-9);
    }

    #[test]
    fn path3d_length_zero_for_empty() {
        let p = Path3D::empty();
        assert_eq!(p.length(), 0.0);
    }

    #[test]
    fn controller_command_clamp() {
        let cmd = ControllerCommand::twist_2d(
            Twist2 {
                linear: Point2::new(5.0, 0.0),
                angular_z: 5.0,
            },
            NavFrameId::Base,
        );
        let c = cmd.clamped(1.0, 1.0);
        assert!(c.twist.linear.x.abs() <= 1.0 + 1e-9);
        assert!(c.twist.angular_z.abs() <= 1.0 + 1e-9);
        assert!(!c.stop);
    }

    #[test]
    fn controller_command_stop_marks_flag() {
        let s = ControllerCommand::stop(NavFrameId::Base);
        assert!(s.stop);
        assert!(s.twist.is_zero(1e-9));
    }

    #[test]
    fn path_serde_roundtrip() {
        let p = Path2D::from_points(vec![Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)]);
        let s = serde_json::to_string(&p).unwrap();
        let back: Path2D = serde_json::from_str(&s).unwrap();
        assert_eq!(back.points.len(), 2);
    }
}

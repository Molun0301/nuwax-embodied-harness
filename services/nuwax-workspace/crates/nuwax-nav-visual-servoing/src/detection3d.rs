//! 3D detection navigation：使用 TF trait 把 3D 目标转换到 robot frame 并产生 twist。
//!
//! 关键约束：
//! - 缺 TF 或 frame 不匹配返回 typed error（`MissingTf` / `FrameMismatch`），**不**用 zero 掩盖。
//! - 单元测试用 fake TF trait 实现。

use serde::{Deserialize, Serialize};

use crate::{PoseStamped, VisualServoingError};

/// TF trait（由调用方注入；测试用 fake）
pub trait TfLike: Send + Sync {
    /// 查询 `target_frame -> source_frame` 的变换，缺失返回 None
    fn lookup(&self, target_frame: &str, source_frame: &str) -> Option<PoseStamped>;
}

/// 3D 目标点（世界 frame）
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TargetPoint3D {
    /// x
    pub x: f64,
    /// y
    pub y: f64,
    /// z
    pub z: f64,
}

/// 3D twist 输出（linear.x + angular.z）
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct NavTwist3D {
    /// linear x y z
    pub linear: [f64; 3],
    /// angular x y z
    pub angular: [f64; 3],
}

/// 3D 配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionNavigationConfig {
    /// 目标距离（m）
    pub target_distance: f64,
    /// 最小距离（m）
    pub min_distance: f64,
    /// 最大线速度
    pub max_linear_speed: f64,
    /// 最大角速度
    pub max_angular_speed: f64,
    /// linear gain
    pub linear_gain: f64,
    /// angular gain
    pub angular_gain: f64,
    /// robot frame
    pub robot_frame: String,
}

impl Default for DetectionNavigationConfig {
    fn default() -> Self {
        Self {
            target_distance: 1.5,
            min_distance: 0.8,
            max_linear_speed: 0.5,
            max_angular_speed: 0.8,
            linear_gain: 0.8,
            angular_gain: 1.5,
            robot_frame: "base_link".to_string(),
        }
    }
}

/// 3D detection navigation controller
pub struct DetectionNavigation {
    tf: Box<dyn TfLike>,
    config: DetectionNavigationConfig,
}

impl DetectionNavigation {
    /// 构造
    pub fn new(tf: Box<dyn TfLike>, config: DetectionNavigationConfig) -> Self {
        Self { tf, config }
    }

    /// 当前配置
    #[must_use]
    pub fn config(&self) -> &DetectionNavigationConfig {
        &self.config
    }

    /// 计算 twist（要求 target 在 world frame）
    pub fn compute_twist(
        &self,
        target_world: TargetPoint3D,
        world_frame: &str,
    ) -> Result<NavTwist3D, VisualServoingError> {
        // 取 world -> robot_frame 变换
        let robot_tf = self
            .tf
            .lookup(world_frame, &self.config.robot_frame)
            .ok_or_else(|| VisualServoingError::MissingTf(self.config.robot_frame.clone()))?;
        if robot_tf.frame_id != world_frame {
            return Err(VisualServoingError::FrameMismatch {
                expected: world_frame.into(),
                actual: robot_tf.frame_id,
            });
        }
        // robot 在 world 中的位置
        let rx = robot_tf.position[0];
        let ry = robot_tf.position[1];
        let dx = target_world.x - rx;
        let dy = target_world.y - ry;
        let distance = (dx * dx + dy * dy).sqrt();
        let angle_to_target = dy.atan2(dx);
        let robot_yaw = yaw_from_quat(&robot_tf.orientation);
        let mut angle_error = angle_to_target - robot_yaw;
        while angle_error > std::f64::consts::PI {
            angle_error -= 2.0 * std::f64::consts::PI;
        }
        while angle_error < -std::f64::consts::PI {
            angle_error += 2.0 * std::f64::consts::PI;
        }
        let angular_z = (self.config.angular_gain * angle_error).clamp(
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
        );
        let linear_x = if distance < self.config.min_distance {
            -self.config.max_linear_speed * 0.6
        } else {
            let turn_factor = 1.0 - (angle_error.abs() / std::f64::consts::PI).min(0.7);
            let v =
                (distance - self.config.target_distance) * self.config.linear_gain * turn_factor;
            v.clamp(-self.config.max_linear_speed, self.config.max_linear_speed)
        };
        Ok(NavTwist3D {
            linear: [linear_x, 0.0, 0.0],
            angular: [0.0, 0.0, angular_z],
        })
    }
}

/// quat (xyzw) -> yaw
#[must_use]
pub fn yaw_from_quat(q: &[f64; 4]) -> f64 {
    // q = (x, y, z, w)
    let (x, y, z, w) = (q[0], q[1], q[2], q[3]);
    // yaw from quaternion
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    siny_cosp.atan2(cosy_cosp)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Fake TF
    struct FakeTf;

    impl TfLike for FakeTf {
        fn lookup(&self, target_frame: &str, source_frame: &str) -> Option<PoseStamped> {
            if target_frame == "world" && source_frame == "base_link" {
                Some(PoseStamped {
                    frame_id: "world".into(),
                    position: [0.0, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                })
            } else {
                None
            }
        }
    }

    struct MissingTf;

    impl TfLike for MissingTf {
        fn lookup(&self, _target: &str, _source: &str) -> Option<PoseStamped> {
            None
        }
    }

    struct MismatchTf;

    impl TfLike for MismatchTf {
        fn lookup(&self, target_frame: &str, source_frame: &str) -> Option<PoseStamped> {
            if target_frame == "world" && source_frame == "base_link" {
                Some(PoseStamped {
                    frame_id: "wrong_frame".into(), // 故意 frame_id 错
                    position: [0.0, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                })
            } else {
                None
            }
        }
    }

    #[test]
    fn missing_tf_returns_typed_error() {
        let n = DetectionNavigation::new(Box::new(MissingTf), DetectionNavigationConfig::default());
        let target = TargetPoint3D {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let r = n.compute_twist(target, "world");
        match r {
            Err(VisualServoingError::MissingTf(_)) => {}
            _ => panic!("expected MissingTf"),
        }
    }

    #[test]
    fn frame_mismatch_returns_typed_error() {
        let n =
            DetectionNavigation::new(Box::new(MismatchTf), DetectionNavigationConfig::default());
        let target = TargetPoint3D {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let r = n.compute_twist(target, "world");
        match r {
            Err(VisualServoingError::FrameMismatch { .. }) => {}
            _ => panic!("expected FrameMismatch"),
        }
    }

    #[test]
    fn forward_target_moves_forward() {
        let n = DetectionNavigation::new(Box::new(FakeTf), DetectionNavigationConfig::default());
        // robot 在 (0,0) 朝 0；target 在 (5,0) -> 距离 5 m
        let target = TargetPoint3D {
            x: 5.0,
            y: 0.0,
            z: 0.0,
        };
        let t = n.compute_twist(target, "world").unwrap();
        // angular ≈ 0（朝向 0，目标在 x 正方向）
        assert_relative_eq!(t.angular[2], 0.0, epsilon = 1e-9);
        // distance_error = 5 - 1.5 = 3.5 -> linear ≈ 3.5 * 0.8 * 1.0 = 2.8 -> clip 到 0.5
        assert_relative_eq!(t.linear[0], 0.5, epsilon = 1e-9);
    }

    #[test]
    fn too_close_backs_up() {
        let n = DetectionNavigation::new(Box::new(FakeTf), DetectionNavigationConfig::default());
        let target = TargetPoint3D {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        }; // 0.5 < min 0.8
        let t = n.compute_twist(target, "world").unwrap();
        assert!(t.linear[0] < 0.0);
    }

    #[test]
    fn side_target_turns() {
        let n = DetectionNavigation::new(Box::new(FakeTf), DetectionNavigationConfig::default());
        // target 在 (5, 5) -> angle = π/4
        let target = TargetPoint3D {
            x: 5.0,
            y: 5.0,
            z: 0.0,
        };
        let t = n.compute_twist(target, "world").unwrap();
        // angular > 0（左转）
        assert!(t.angular[2] > 0.0);
    }

    #[test]
    fn yaw_from_quat_identity() {
        let q = [0.0, 0.0, 0.0, 1.0];
        assert_relative_eq!(yaw_from_quat(&q), 0.0, epsilon = 1e-9);
    }
}

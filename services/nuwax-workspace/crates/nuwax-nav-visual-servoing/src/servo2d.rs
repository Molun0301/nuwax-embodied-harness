//! 2D 视觉伺服（bbox → twist）

use serde::{Deserialize, Serialize};

use crate::{BBox, Twist};

/// 相机内参
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    /// fx (像素)
    pub fx: f64,
    /// fy
    pub fy: f64,
    /// cx (光心 x)
    pub cx: f64,
    /// cy
    pub cy: f64,
}

impl CameraIntrinsics {
    /// K[0,2,4,5,2,5]
    #[must_use]
    pub fn from_k_row(k: &[f64; 9]) -> Self {
        Self {
            fx: k[0],
            fy: k[4],
            cx: k[2],
            cy: k[5],
        }
    }
}

/// 2D 视觉伺服配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualServoing2DConfig {
    /// 目标距离（m）
    pub target_distance: f64,
    /// 最小距离（m），< 此值后退
    pub min_distance: f64,
    /// 最大前进/后退速度（m/s）
    pub max_linear_speed: f64,
    /// 最大角速度（rad/s）
    pub max_angular_speed: f64,
    /// 假定物体宽度（m）
    pub assumed_object_width: f64,
    /// 角速度增益
    pub angular_gain: f64,
    /// 线速度增益
    pub linear_gain: f64,
    /// 后退速度因子（×max_linear_speed）
    pub backup_speed_factor: f64,
    /// turn factor multiplier
    pub turn_factor_multiplier: f64,
    /// turn factor 最大减幅
    pub turn_factor_max_reduction: f64,
    /// rotation requires linear movement（小角度强制前进）
    pub rotation_requires_linear_movement: bool,
}

impl Default for VisualServoing2DConfig {
    fn default() -> Self {
        Self {
            target_distance: 1.5,
            min_distance: 0.8,
            max_linear_speed: 0.5,
            max_angular_speed: 0.8,
            assumed_object_width: 0.45,
            angular_gain: 1.0,
            linear_gain: 0.8,
            backup_speed_factor: 0.6,
            turn_factor_multiplier: 2.0,
            turn_factor_max_reduction: 0.7,
            rotation_requires_linear_movement: false,
        }
    }
}

/// 2D visual servo controller
#[derive(Debug, Clone)]
pub struct VisualServoing2D {
    intrinsics: CameraIntrinsics,
    config: VisualServoing2DConfig,
}

impl VisualServoing2D {
    /// 构造
    #[must_use]
    pub fn new(intrinsics: CameraIntrinsics, config: VisualServoing2DConfig) -> Self {
        Self { intrinsics, config }
    }

    /// 当前配置
    #[must_use]
    pub fn config(&self) -> &VisualServoing2DConfig {
        &self.config
    }

    /// 当前内参
    #[must_use]
    pub fn intrinsics(&self) -> &CameraIntrinsics {
        &self.intrinsics
    }

    /// 计算 twist
    pub fn compute_twist(&self, bbox: BBox) -> Twist {
        if !bbox.is_valid() {
            return Twist::zero();
        }
        let (center_x, _center_y) = bbox.center();
        let x_norm = self.normalized_x(center_x);
        let estimated_distance = match self.estimate_distance(bbox) {
            Some(d) => d,
            None => return Twist::zero(),
        };
        let distance_error = estimated_distance - self.config.target_distance;
        // angular.z = -x_norm * angular_gain
        let angular_z = (-x_norm * self.config.angular_gain).clamp(
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
        );
        let mut linear_x = if estimated_distance < self.config.min_distance {
            // 太近后退
            -self.config.max_linear_speed * self.config.backup_speed_factor
        } else {
            // distance_error * linear_gain * turn_factor
            let turn_factor = 1.0
                - (x_norm.abs() * self.config.turn_factor_multiplier)
                    .min(self.config.turn_factor_max_reduction);
            let v = distance_error * self.config.linear_gain * turn_factor;
            v.clamp(-self.config.max_linear_speed, self.config.max_linear_speed)
        };
        // rotation_requires_linear_movement：小角度时强制前进
        if self.config.rotation_requires_linear_movement && angular_z.abs() < 0.02 {
            linear_x = linear_x.max(0.1);
        }
        Twist {
            linear: [linear_x, 0.0, 0.0],
            angular: [0.0, 0.0, angular_z],
        }
    }

    fn normalized_x(&self, pixel_x: f64) -> f64 {
        (pixel_x - self.intrinsics.cx) / self.intrinsics.fx
    }

    fn estimate_distance(&self, bbox: BBox) -> Option<f64> {
        let w = bbox.width();
        if w <= 0.0 {
            return None;
        }
        Some(self.config.assumed_object_width * self.intrinsics.fx / w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn intrinsics() -> CameraIntrinsics {
        // fx=500, cx=320（典型 640x480 相机）
        CameraIntrinsics {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        }
    }

    fn servo() -> VisualServoing2D {
        VisualServoing2D::new(intrinsics(), VisualServoing2DConfig::default())
    }

    #[test]
    fn invalid_bbox_zero_twist() {
        let s = servo();
        let bbox = BBox::new(10.0, 0.0, 5.0, 10.0); // width = -5
        let t = s.compute_twist(bbox);
        assert_eq!(t, Twist::zero());
    }

    #[test]
    fn center_target_moves_forward() {
        let s = servo();
        // 物体宽度 0.45 m，bbox 150 px：distance = 0.45 * 500 / 150 = 1.5 m（恰好 target）
        let bbox = BBox::new(245.0, 200.0, 395.0, 280.0); // center = 320 (cx), width = 150
        let t = s.compute_twist(bbox);
        // 中心 -> angular ≈ 0；distance = target -> linear ≈ 0
        assert_relative_eq!(t.angular[2], 0.0, epsilon = 1e-9);
        // 距离 ≈ target distance，distance_error ≈ 0
        assert!(t.linear[0].abs() < 0.05);
    }

    #[test]
    fn right_target_turns_left() {
        let s = servo();
        // center = 420 -> x_norm = (420 - 320)/500 = 0.2 -> angular = -0.2 * 1.0 = -0.2 (向左)
        let bbox = BBox::new(345.0, 200.0, 495.0, 280.0);
        let t = s.compute_twist(bbox);
        assert!(t.angular[2] < 0.0); // 负方向 = 左
    }

    #[test]
    fn left_target_turns_right() {
        let s = servo();
        let bbox = BBox::new(145.0, 200.0, 295.0, 280.0); // center 220 -> x_norm = -0.2
        let t = s.compute_twist(bbox);
        assert!(t.angular[2] > 0.0);
    }

    #[test]
    fn too_close_backs_up() {
        let s = servo();
        // distance = 0.45*500/300 = 0.75 m < min 0.8 -> 后退
        let bbox = BBox::new(170.0, 100.0, 470.0, 380.0); // center 320, width 300
        let t = s.compute_twist(bbox);
        assert!(t.linear[0] < 0.0);
        // 后退速度 ≈ -0.5 * 0.6 = -0.3
        assert_relative_eq!(t.linear[0], -0.3, epsilon = 1e-9);
    }

    #[test]
    fn far_target_moves_forward_with_turn_factor() {
        let s = servo();
        // distance = 0.45*500/60 = 3.75 m -> error = 2.25 -> linear ≈ 2.25 * 0.8 * turn
        let bbox = BBox::new(290.0, 200.0, 350.0, 280.0); // center 320, width 60
        let t = s.compute_twist(bbox);
        // 中心 -> turn_factor ≈ 1
        assert!(t.linear[0] > 0.0);
        // 上限 ±0.5
        assert!(t.linear[0] <= 0.5 + 1e-9);
    }

    #[test]
    fn rotation_requires_linear_movement_enforced() {
        let cfg = VisualServoing2DConfig {
            rotation_requires_linear_movement: true,
            ..VisualServoing2DConfig::default()
        };
        let s = VisualServoing2D::new(intrinsics(), cfg);
        // 中心目标 -> angular ≈ 0, < 0.02 -> 强制 linear_x = max(_, 0.1)
        let bbox = BBox::new(245.0, 200.0, 395.0, 280.0);
        let t = s.compute_twist(bbox);
        assert!(t.linear[0] >= 0.1);
    }

    #[test]
    fn fx_cx_affect_normalized_x() {
        let mut intr = intrinsics();
        intr.fx = 1000.0; // 焦距翻倍
        intr.cx = 200.0;
        let s = VisualServoing2D::new(intr, VisualServoing2DConfig::default());
        // center = 400 -> x_norm = (400 - 200) / 1000 = 0.2
        let bbox = BBox::new(325.0, 200.0, 475.0, 280.0);
        let t = s.compute_twist(bbox);
        assert!(t.angular[2] < 0.0);
        assert!(t.angular[2].abs() < 0.3); // 应该较小（x_norm 小）
    }
}

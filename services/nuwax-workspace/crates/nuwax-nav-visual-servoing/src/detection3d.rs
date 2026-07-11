//! 3D detection navigation：使用 TF trait 把 3D 目标转换到 robot frame 并产生 twist。
//!
//! 关键约束：
//! - 缺 TF 或 frame 不匹配返回 typed error（`MissingTf` / `FrameMismatch`），**不**用 zero 掩盖。
//! - 单元测试用 fake TF trait 实现。
//!
//! `extract_3d_from_detection`：从 2D bbox + 相机坐标系 PointCloud2 中提取目标 3D 点。
//! 算法：取 bbox 中心点对应 pointcloud 索引处的 x,y,z,经 TF 转 world frame。
//!
//! `DetectionNavigationModule`：dev mock module I/O 层,封装三路输入(暂用同步 mock)
//! 调用 `DetectionNavigation` 产出 twist。

use serde::{Deserialize, Serialize};

use crate::{BBox, ImageView, PoseStamped, VisualServoingError};

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

/// PointCloud2 like trait（dev mock 用）
///
/// 描述 PointCloud2 风格接口：x/y/z 顺序的平铺 xyz 数据。
/// 真实 ROS2 sensor_msgs/PointCloud2 字段更多,dev 自测只需要 x/y/z,
/// 因此以最小 trait 形式暴露。
pub trait PointCloud2Like: Send + Sync {
    /// xyz flat bytes（小端 f32 x,y,z 顺序）,长度 = N * 12
    fn xyz_bytes(&self) -> &[u8];
    /// 是否 dense（无 NaN / 无 invalid 点）
    fn is_dense(&self) -> bool;
}

/// 简易 PointCloud2 mock：持有 `Vec<(f32,f32,f32)>` + dense flag + bytes cache。
#[derive(Debug, Clone)]
pub struct SimplePointCloud {
    /// 是否 dense
    pub is_dense: bool,
    /// (x,y,z) 元组
    pub points: Vec<(f32, f32, f32)>,
    /// xyz flat bytes cache
    bytes_cache: Vec<u8>,
}

impl SimplePointCloud {
    /// 构造（自动计算 bytes cache）
    #[must_use]
    pub fn new(points: Vec<(f32, f32, f32)>) -> Self {
        let bytes_cache = Self::encode(&points);
        Self {
            is_dense: true,
            points,
            bytes_cache,
        }
    }

    fn encode(points: &[(f32, f32, f32)]) -> Vec<u8> {
        let mut out = Vec::with_capacity(points.len() * 12);
        for &(x, y, z) in points {
            out.extend_from_slice(&x.to_le_bytes());
            out.extend_from_slice(&y.to_le_bytes());
            out.extend_from_slice(&z.to_le_bytes());
        }
        out
    }
}

impl PointCloud2Like for SimplePointCloud {
    fn xyz_bytes(&self) -> &[u8] {
        &self.bytes_cache
    }
    fn is_dense(&self) -> bool {
        self.is_dense
    }
}

/// 相机内参（pinhole,无畸变）
///
/// 仅 dev 自测使用；实装由相机标定 driver 注入。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    /// fx (px)
    pub fx: f64,
    /// fy (px)
    pub fy: f64,
    /// cx (px)
    pub cx: f64,
    /// cy (px)
    pub cy: f64,
    /// image width
    pub width: u32,
    /// image height
    pub height: u32,
}

impl CameraIntrinsics {
    /// 构造
    #[must_use]
    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64, width: u32, height: u32) -> Self {
        Self {
            fx,
            fy,
            cx,
            cy,
            width,
            height,
        }
    }
}

impl Default for CameraIntrinsics {
    fn default() -> Self {
        Self::new(500.0, 500.0, 320.0, 240.0, 640, 480)
    }
}

/// 从 2D bbox + 相机坐标系 PointCloud2 + TF 提取目标 3D 点
///
/// 算法(简化版,dev 自测够用)：
/// 1. bbox 中心 -> 图像坐标 (cx_pixel, cy_pixel)
/// 2. 1:1 简化：索引 = cy_pixel * width + cx_pixel,从 bytes 取 x,y,z
/// 3. 已知 x,y,z 在 camera frame,直接用
/// 4. 用 TF 把 camera_optical -> world_frame
///
/// **TODO(real-machine)**:实机需把 pointcloud 投影到图像平面或做外参对齐；
/// 此处 dev 简化版本假定 pointcloud 与 image 像素 1:1 对应（mock 测试约束）。
pub fn extract_3d_from_detection<P: PointCloud2Like + ?Sized>(
    bbox: BBox,
    image: &ImageView<'_>,
    pointcloud: &P,
    camera_intrinsics: &CameraIntrinsics,
    tf: &dyn TfLike,
    camera_frame: &str,
    world_frame: &str,
) -> Result<TargetPoint3D, VisualServoingError> {
    if !bbox.is_valid() {
        return Err(VisualServoingError::InvalidBbox(format!(
            "bbox non-positive area: ({},{},{},{})",
            bbox.x1, bbox.y1, bbox.x2, bbox.y2
        )));
    }
    if !image.matches(camera_intrinsics.width, camera_intrinsics.height) {
        return Err(VisualServoingError::InvalidBbox(format!(
            "image size {}x{} != camera {}x{}",
            image.width, image.height, camera_intrinsics.width, camera_intrinsics.height
        )));
    }
    let (cx_pixel, cy_pixel) = bbox.center();
    let idx = (cy_pixel as usize) * (camera_intrinsics.width as usize) + (cx_pixel as usize);
    let bytes = pointcloud.xyz_bytes();
    if bytes.len() % 12 != 0 {
        return Err(VisualServoingError::ProjectionFailed(format!(
            "pointcloud bytes length {} not multiple of 12",
            bytes.len()
        )));
    }
    let n_points = bytes.len() / 12;
    if idx >= n_points {
        return Err(VisualServoingError::ProjectionFailed(format!(
            "bbox center pixel index {} >= pointcloud n_points {}",
            idx, n_points
        )));
    }
    let base = idx * 12;
    let x = f32::from_le_bytes([
        bytes[base],
        bytes[base + 1],
        bytes[base + 2],
        bytes[base + 3],
    ]);
    let y = f32::from_le_bytes([
        bytes[base + 4],
        bytes[base + 5],
        bytes[base + 6],
        bytes[base + 7],
    ]);
    let z = f32::from_le_bytes([
        bytes[base + 8],
        bytes[base + 9],
        bytes[base + 10],
        bytes[base + 11],
    ]);
    if !z.is_finite() || z <= 0.0 {
        return Err(VisualServoingError::ProjectionFailed(format!(
            "invalid depth z={} at pixel ({}, {})",
            z, cx_pixel, cy_pixel
        )));
    }
    let cam_target = TargetPoint3D {
        x: f64::from(x),
        y: f64::from(y),
        z: f64::from(z),
    };
    let tf_wc = tf
        .lookup(world_frame, camera_frame)
        .ok_or_else(|| VisualServoingError::MissingTf(format!("{}->{}", world_frame, camera_frame)))?;
    if tf_wc.frame_id != world_frame {
        return Err(VisualServoingError::FrameMismatch {
            expected: world_frame.into(),
            actual: tf_wc.frame_id,
        });
    }
    let yaw = yaw_from_quat(&tf_wc.orientation);
    let (cos_y, sin_y) = (yaw.cos(), yaw.sin());
    let wx = tf_wc.position[0] + cos_y * cam_target.x - sin_y * cam_target.y;
    let wy = tf_wc.position[1] + sin_y * cam_target.x + cos_y * cam_target.y;
    let wz = tf_wc.position[2] + cam_target.z;
    Ok(TargetPoint3D {
        x: wx,
        y: wy,
        z: wz,
    })
}

/// DetectionNavigationModule:dev mock module I/O 层
///
/// 同步 mock 三路输入(image / bbox / pointcloud),调用 DetectionNavigation 产出 twist。
pub struct DetectionNavigationModule {
    nav: DetectionNavigation,
    intrinsics: CameraIntrinsics,
    camera_frame: String,
    world_frame: String,
}

impl DetectionNavigationModule {
    /// 构造
    pub fn new(
        tf: Box<dyn TfLike>,
        config: DetectionNavigationConfig,
        intrinsics: CameraIntrinsics,
        camera_frame: impl Into<String>,
        world_frame: impl Into<String>,
    ) -> Self {
        Self {
            nav: DetectionNavigation::new(tf, config),
            intrinsics,
            camera_frame: camera_frame.into(),
            world_frame: world_frame.into(),
        }
    }

    /// 单步:image + bbox + pointcloud -> twist
    pub fn step<P: PointCloud2Like + ?Sized>(
        &self,
        image: &ImageView<'_>,
        bbox: BBox,
        pointcloud: &P,
        tf: &dyn TfLike,
    ) -> Result<crate::Twist, VisualServoingError> {
        let target_world = extract_3d_from_detection(
            bbox,
            image,
            pointcloud,
            &self.intrinsics,
            tf,
            &self.camera_frame,
            &self.world_frame,
        )?;
        let nav_twist = self.nav.compute_twist(target_world, &self.world_frame)?;
        Ok(crate::Twist {
            linear: nav_twist.linear,
            angular: nav_twist.angular,
        })
    }
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
                    frame_id: "wrong_frame".into(),
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
        let target = TargetPoint3D {
            x: 5.0,
            y: 0.0,
            z: 0.0,
        };
        let t = n.compute_twist(target, "world").unwrap();
        assert_relative_eq!(t.angular[2], 0.0, epsilon = 1e-9);
        assert_relative_eq!(t.linear[0], 0.5, epsilon = 1e-9);
    }

    #[test]
    fn too_close_backs_up() {
        let n = DetectionNavigation::new(Box::new(FakeTf), DetectionNavigationConfig::default());
        let target = TargetPoint3D {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        };
        let t = n.compute_twist(target, "world").unwrap();
        assert!(t.linear[0] < 0.0);
    }

    #[test]
    fn side_target_turns() {
        let n = DetectionNavigation::new(Box::new(FakeTf), DetectionNavigationConfig::default());
        let target = TargetPoint3D {
            x: 5.0,
            y: 5.0,
            z: 0.0,
        };
        let t = n.compute_twist(target, "world").unwrap();
        assert!(t.angular[2] > 0.0);
    }

    #[test]
    fn yaw_from_quat_identity() {
        let q = [0.0, 0.0, 0.0, 1.0];
        assert_relative_eq!(yaw_from_quat(&q), 0.0, epsilon = 1e-9);
    }
}
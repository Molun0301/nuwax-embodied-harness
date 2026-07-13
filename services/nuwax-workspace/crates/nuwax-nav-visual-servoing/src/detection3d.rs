//! 3D detection navigation：使用 TF trait 把 3D 目标转换到 robot frame 并产生 twist。
//!
//! 关键约束：
//! - 缺 TF 或 frame 不匹配返回 typed error（`MissingTf` / `FrameMismatch`），**不**用 zero 掩盖。
//! - 单元测试用 fake TF trait 实现。
//!
//! `extract_3d_from_detection`：把世界坐标系 PointCloud2 投影到图像，收集 bbox
//! 内点，并用距离机器人最近 25% 点的质心作为前表面目标。
//!
//! `DetectionNavigationModule`：同步 I/O 适配层，封装 image / bbox / pointcloud
//! 输入，并调用持有同一 TF provider 的 `DetectionNavigation` 产出 twist。

use serde::{Deserialize, Serialize};

use crate::{BBox, ImageView, PoseStamped, VisualServoingError};

/// TF trait（由调用方注入；测试用 fake）
pub trait TfLike: Send + Sync {
    /// 查询 `target_frame <- source_frame` 的变换，缺失返回 None。
    fn lookup(&self, target_frame: &str, source_frame: &str) -> Option<PoseStamped>;

    /// 按时间查询 `target_frame <- source_frame` 的变换。
    ///
    /// 旧实现只需实现 [`Self::lookup`]；需要时间同步的生产适配器可覆盖本方法。
    fn lookup_at(
        &self,
        target_frame: &str,
        source_frame: &str,
        time_point: Option<f64>,
        time_tolerance: f64,
    ) -> Option<PoseStamped> {
        let _ = (time_point, time_tolerance);
        self.lookup(target_frame, source_frame)
    }
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
        let robot_tf = self
            .tf
            .lookup(world_frame, &self.config.robot_frame)
            .ok_or_else(|| VisualServoingError::MissingTf(self.config.robot_frame.clone()))?;
        self.compute_twist_with_robot_transform(target_world, world_frame, robot_tf)
    }

    /// Project a 2D detection and compute a twist with the controller-owned TF source.
    pub fn compute_twist_for_detection_3d<P: PointCloud2Like + ?Sized>(
        &self,
        bbox: BBox,
        image: &ImageView<'_>,
        pointcloud: &P,
        camera_intrinsics: &CameraIntrinsics,
        camera_frame: &str,
        world_frame: &str,
    ) -> Result<NavTwist3D, VisualServoingError> {
        self.compute_twist_for_detection_3d_at(
            bbox,
            image,
            pointcloud,
            camera_intrinsics,
            camera_frame,
            world_frame,
            None,
        )
    }

    /// Timestamp-aware form of [`Self::compute_twist_for_detection_3d`].
    #[allow(clippy::too_many_arguments)]
    pub fn compute_twist_for_detection_3d_at<P: PointCloud2Like + ?Sized>(
        &self,
        bbox: BBox,
        image: &ImageView<'_>,
        pointcloud: &P,
        camera_intrinsics: &CameraIntrinsics,
        camera_frame: &str,
        world_frame: &str,
        image_timestamp: Option<f64>,
    ) -> Result<NavTwist3D, VisualServoingError> {
        let (target_world, robot_tf) = extract_target_and_robot_at(
            bbox,
            image,
            pointcloud,
            camera_intrinsics,
            self.tf.as_ref(),
            camera_frame,
            world_frame,
            &self.config.robot_frame,
            image_timestamp,
        )?;
        self.compute_twist_with_robot_transform(target_world, world_frame, robot_tf)
    }

    fn compute_twist_with_robot_transform(
        &self,
        target_world: TargetPoint3D,
        world_frame: &str,
        robot_tf: PoseStamped,
    ) -> Result<NavTwist3D, VisualServoingError> {
        if robot_tf.frame_id != world_frame {
            return Err(VisualServoingError::FrameMismatch {
                expected: world_frame.into(),
                actual: robot_tf.frame_id,
            });
        }
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

/// PointCloud2-like projection input.
///
/// Production adapters expose decoded little-endian XYZ values and the header
/// frame through this minimal interface; tests use [`SimplePointCloud`].
pub trait PointCloud2Like: Send + Sync {
    /// xyz flat bytes（小端 f32 x,y,z 顺序）,长度 = N * 12
    fn xyz_bytes(&self) -> &[u8];
    /// 是否 dense（无 NaN / 无 invalid 点）
    fn is_dense(&self) -> bool;
    /// 点云坐标系。旧 mock 未携带 header 时返回 None。
    fn frame_id(&self) -> Option<&str> {
        None
    }
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
    /// Optional PointCloud2 header frame.
    frame_id: Option<String>,
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
            frame_id: None,
        }
    }

    /// Attach the PointCloud2 header frame used by TF projection.
    #[must_use]
    pub fn in_frame(mut self, frame_id: impl Into<String>) -> Self {
        self.frame_id = Some(frame_id.into());
        self
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

    fn frame_id(&self) -> Option<&str> {
        self.frame_id.as_deref()
    }
}

/// 相机内参（pinhole,无畸变）；生产值由共享 `CameraInfo.K` 注入。
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

    /// Construct from the shared ROS `CameraInfo.K` subset.
    #[must_use]
    pub fn from_camera_info(info: nuwax_nav_types_messages::CameraInfo) -> Self {
        Self::new(info.fx, info.fy, info.cx, info.cy, info.width, info.height)
    }

    /// Whether the pinhole model can produce finite image coordinates.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.width > 0
            && self.height > 0
            && self.fx.is_finite()
            && self.fy.is_finite()
            && self.cx.is_finite()
            && self.cy.is_finite()
            && self.fx > 0.0
            && self.fy > 0.0
    }
}

impl From<nuwax_nav_types_messages::CameraInfo> for CameraIntrinsics {
    fn from(info: nuwax_nav_types_messages::CameraInfo) -> Self {
        Self::from_camera_info(info)
    }
}

impl Default for CameraIntrinsics {
    fn default() -> Self {
        Self::new(500.0, 500.0, 320.0, 240.0, 640, 480)
    }
}

const TF_TIME_TOLERANCE_SECONDS: f64 = 1.0;
const BBOX_MARGIN_PIXELS: f64 = 5.0;

/// 从 2D bbox + 世界坐标系 PointCloud2 + TF 提取目标 3D 点
///
/// 算法与原版 `Detection3DPC.from_2d` + robust target 对齐：
/// 1. 查询 `camera_frame <- world_frame`，把每个 world-frame 点投影到 image；
/// 2. 先裁剪图像边界，再选择 bbox 周围 5px margin 内的点；选中点仍保持 world 坐标；
/// 3. 至少需要 10 点，优先过滤 world z <= 0.3 m 的地面点；
/// 4. 取相对 robot XY 距离的 25th percentile 前表面点并求质心。
pub fn extract_3d_from_detection<P: PointCloud2Like + ?Sized>(
    bbox: BBox,
    image: &ImageView<'_>,
    pointcloud: &P,
    camera_intrinsics: &CameraIntrinsics,
    tf: &dyn TfLike,
    camera_frame: &str,
    world_frame: &str,
) -> Result<TargetPoint3D, VisualServoingError> {
    extract_3d_from_detection_at(
        bbox,
        image,
        pointcloud,
        camera_intrinsics,
        tf,
        camera_frame,
        world_frame,
        None,
    )
}

/// Timestamp-aware form of [`extract_3d_from_detection`].
///
/// `image_timestamp` is used for the optical transform, matching the original
/// `LCMTF.get(..., image.ts, time_tolerance=1.0)` contract. Robot pose uses the
/// latest transform with the same tolerance.
#[allow(clippy::too_many_arguments)]
pub fn extract_3d_from_detection_at<P: PointCloud2Like + ?Sized>(
    bbox: BBox,
    image: &ImageView<'_>,
    pointcloud: &P,
    camera_intrinsics: &CameraIntrinsics,
    tf: &dyn TfLike,
    camera_frame: &str,
    world_frame: &str,
    image_timestamp: Option<f64>,
) -> Result<TargetPoint3D, VisualServoingError> {
    extract_target_and_robot_at(
        bbox,
        image,
        pointcloud,
        camera_intrinsics,
        tf,
        camera_frame,
        world_frame,
        "base_link",
        image_timestamp,
    )
    .map(|(target, _)| target)
}

#[allow(clippy::too_many_arguments)]
fn extract_target_and_robot_at<P: PointCloud2Like + ?Sized>(
    bbox: BBox,
    image: &ImageView<'_>,
    pointcloud: &P,
    camera_intrinsics: &CameraIntrinsics,
    tf: &dyn TfLike,
    camera_frame: &str,
    world_frame: &str,
    robot_frame: &str,
    image_timestamp: Option<f64>,
) -> Result<(TargetPoint3D, PoseStamped), VisualServoingError> {
    if !bbox.is_valid() {
        return Err(VisualServoingError::InvalidBbox(format!(
            "bbox non-positive area: ({},{},{},{})",
            bbox.x1, bbox.y1, bbox.x2, bbox.y2
        )));
    }
    if !camera_intrinsics.is_valid() {
        return Err(VisualServoingError::ProjectionFailed(
            "camera intrinsics must have finite positive focal lengths and non-zero dimensions"
                .into(),
        ));
    }
    if !image.matches(camera_intrinsics.width, camera_intrinsics.height) {
        return Err(VisualServoingError::InvalidBbox(format!(
            "image size {}x{} != camera {}x{}",
            image.width, image.height, camera_intrinsics.width, camera_intrinsics.height
        )));
    }
    if let Some(actual) = pointcloud.frame_id().filter(|frame| *frame != world_frame) {
        return Err(VisualServoingError::FrameMismatch {
            expected: world_frame.into(),
            actual: actual.into(),
        });
    }
    let bytes = pointcloud.xyz_bytes();
    if bytes.len() % 12 != 0 {
        return Err(VisualServoingError::ProjectionFailed(format!(
            "pointcloud bytes length {} not multiple of 12",
            bytes.len()
        )));
    }
    let optical_from_world = tf
        .lookup_at(
            camera_frame,
            world_frame,
            image_timestamp,
            TF_TIME_TOLERANCE_SECONDS,
        )
        .ok_or_else(|| VisualServoingError::MissingTf(format!("{camera_frame}<-{world_frame}")))?;
    if optical_from_world.frame_id != camera_frame {
        return Err(VisualServoingError::FrameMismatch {
            expected: camera_frame.into(),
            actual: optical_from_world.frame_id,
        });
    }

    let mut world_points = Vec::new();
    for point in bytes.chunks_exact(12) {
        let x = f64::from(f32::from_le_bytes(point[0..4].try_into().expect("4 bytes")));
        let y = f64::from(f32::from_le_bytes(point[4..8].try_into().expect("4 bytes")));
        let z = f64::from(f32::from_le_bytes(
            point[8..12].try_into().expect("4 bytes"),
        ));
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            continue;
        }
        let point_world = [x, y, z];
        let rotated = rotate_vector(&optical_from_world.orientation, point_world);
        let point_optical = [
            optical_from_world.position[0] + rotated[0],
            optical_from_world.position[1] + rotated[1],
            optical_from_world.position[2] + rotated[2],
        ];
        if !point_optical
            .iter()
            .all(|coordinate| coordinate.is_finite())
            || point_optical[2] <= 0.0
        {
            continue;
        }
        let pixel_x =
            camera_intrinsics.fx * point_optical[0] / point_optical[2] + camera_intrinsics.cx;
        let pixel_y =
            camera_intrinsics.fy * point_optical[1] / point_optical[2] + camera_intrinsics.cy;
        if pixel_x < 0.0
            || pixel_x >= f64::from(camera_intrinsics.width)
            || pixel_y < 0.0
            || pixel_y >= f64::from(camera_intrinsics.height)
        {
            continue;
        }
        if pixel_x < bbox.x1 - BBOX_MARGIN_PIXELS
            || pixel_x > bbox.x2 + BBOX_MARGIN_PIXELS
            || pixel_y < bbox.y1 - BBOX_MARGIN_PIXELS
            || pixel_y > bbox.y2 + BBOX_MARGIN_PIXELS
        {
            continue;
        }
        world_points.push(TargetPoint3D { x, y, z });
    }

    if world_points.is_empty() {
        return Err(VisualServoingError::ProjectionFailed(
            "no world points projected into the detection bbox".into(),
        ));
    }

    let robot_tf = tf
        .lookup_at(world_frame, robot_frame, None, TF_TIME_TOLERANCE_SECONDS)
        .ok_or_else(|| VisualServoingError::MissingTf(format!("{world_frame}<-{robot_frame}")))?;
    if robot_tf.frame_id != world_frame {
        return Err(VisualServoingError::FrameMismatch {
            expected: world_frame.into(),
            actual: robot_tf.frame_id,
        });
    }

    let target = robust_front_target(&world_points, robot_tf.position).ok_or_else(|| {
        VisualServoingError::ProjectionFailed(format!(
            "bbox projection produced {} usable points; need at least 10",
            world_points.len()
        ))
    })?;
    Ok((target, robot_tf))
}

fn rotate_vector(q: &[f64; 4], v: [f64; 3]) -> [f64; 3] {
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if norm <= f64::EPSILON {
        return v;
    }
    let qv = [q[0] / norm, q[1] / norm, q[2] / norm];
    let qw = q[3] / norm;
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let uv = cross(qv, v);
    let uuv = cross(qv, uv);
    [
        v[0] + 2.0 * (qw * uv[0] + uuv[0]),
        v[1] + 2.0 * (qw * uv[1] + uuv[1]),
        v[2] + 2.0 * (qw * uv[2] + uuv[2]),
    ]
}

fn robust_front_target(points: &[TargetPoint3D], robot: [f64; 3]) -> Option<TargetPoint3D> {
    if points.len() < 10 {
        return None;
    }
    let above_floor: Vec<TargetPoint3D> = points.iter().copied().filter(|p| p.z > 0.3).collect();
    let candidates = if above_floor.len() >= 10 {
        above_floor
    } else {
        points.to_vec()
    };
    let mut distances: Vec<f64> = candidates
        .iter()
        .map(|p| ((p.x - robot[0]).powi(2) + (p.y - robot[1]).powi(2)).sqrt())
        .collect();
    distances.sort_by(f64::total_cmp);
    let percentile_position = 0.25 * (distances.len() - 1) as f64;
    let lower = percentile_position.floor() as usize;
    let upper = percentile_position.ceil() as usize;
    let fraction = percentile_position - lower as f64;
    let threshold = distances[lower] * (1.0 - fraction) + distances[upper] * fraction;
    let mut front: Vec<TargetPoint3D> = candidates
        .iter()
        .copied()
        .filter(|p| ((p.x - robot[0]).powi(2) + (p.y - robot[1]).powi(2)).sqrt() <= threshold)
        .collect();
    if front.len() < 3 {
        let middle = distances.len() / 2;
        let median = if distances.len() % 2 == 0 {
            (distances[middle - 1] + distances[middle]) * 0.5
        } else {
            distances[middle]
        };
        front = candidates
            .iter()
            .copied()
            .filter(|p| {
                (((p.x - robot[0]).powi(2) + (p.y - robot[1]).powi(2)).sqrt() - median).abs() < 0.3
            })
            .collect();
    }
    if front.len() < 3 {
        return None;
    }
    let count = front.len() as f64;
    Some(TargetPoint3D {
        x: front.iter().map(|p| p.x).sum::<f64>() / count,
        y: front.iter().map(|p| p.y).sum::<f64>() / count,
        z: front.iter().map(|p| p.z).sum::<f64>() / count,
    })
}

/// DetectionNavigation synchronous module I/O adapter.
///
/// The timestamped path uses the controller-owned TF source for both projection
/// and twist computation. [`Self::step`] retains the legacy injected-TF API.
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
        self.step_with_tf_at(image, bbox, pointcloud, tf, None)
    }

    /// Timestamp-aware single step using the module-owned TF source.
    pub fn step_at<P: PointCloud2Like + ?Sized>(
        &self,
        image: &ImageView<'_>,
        bbox: BBox,
        pointcloud: &P,
        image_timestamp: Option<f64>,
    ) -> Result<crate::Twist, VisualServoingError> {
        let nav_twist = self.nav.compute_twist_for_detection_3d_at(
            bbox,
            image,
            pointcloud,
            &self.intrinsics,
            &self.camera_frame,
            &self.world_frame,
            image_timestamp,
        )?;
        Ok(crate::Twist {
            linear: nav_twist.linear,
            angular: nav_twist.angular,
        })
    }

    fn step_with_tf_at<P: PointCloud2Like + ?Sized>(
        &self,
        image: &ImageView<'_>,
        bbox: BBox,
        pointcloud: &P,
        tf: &dyn TfLike,
        image_timestamp: Option<f64>,
    ) -> Result<crate::Twist, VisualServoingError> {
        let target_world = extract_3d_from_detection_at(
            bbox,
            image,
            pointcloud,
            &self.intrinsics,
            tf,
            &self.camera_frame,
            &self.world_frame,
            image_timestamp,
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

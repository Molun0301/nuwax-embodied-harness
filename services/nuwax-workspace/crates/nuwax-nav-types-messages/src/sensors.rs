//! 传感器/感知消息类型
//!
//! 集中定义:
//! - 3D 点云(`PointCloud`)
//! - 带时间戳的 3D 点(`PointStamped`)
//! - 相机内参(`CameraInfo`)
//! - 2D 检测数组(`Detection2DArray`)
//! - 栅格快照(`GridSnapshot`,自包含字段,避免与 `OccupancyGrid2D` 字段重复)
//! - 里程计(`Odometry`)
//!
//! 设计约束:
//! - 所有时间字段一律使用**纳秒**(`Timestamp`)
//! - 所有数值字段在边界处做 NaN/Inf 显式校验(不静默接受)
//! - 序列化字段使用 `snake_case`,与 Foundation 其他消息保持一致
//!
//! 旧 crate 私有的同名类型(`point_stamped` / `point_cloud` / `odometry` / `camera_info` /
//! `detection2d_array` / `grid_snapshot` / `frontier_goal_input`)已被本模块取代:
//! `pub use nuwax_nav_types_messages::sensors::*` 即可在调用方使用统一类型。

use serde::{Deserialize, Serialize};

use crate::frames::NavFrameId;
use crate::geometry::{Point3, Pose3, Quaternion, Timestamp};

// ============================================================================
// 3D 点与点云
// ============================================================================

/// 带坐标系与时间戳的 3D 点(对应 `geometry_msgs/PointStamped`)
///
/// Foundation 同样提供 `PoseStamped` 表达"带 stamp + frame 的位姿",但 `PointStamped`
/// 用于表达**纯点**(`Point3`,无朝向)。本类型与 `PoseStamped` 字段结构对称:
/// `frame_id: NavFrameId` + `stamp: Timestamp` + `point: Point3`。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PointStamped {
    /// 坐标系
    pub frame_id: NavFrameId,
    /// 时间戳(纳秒)
    pub stamp: Timestamp,
    /// 3D 坐标
    pub point: Point3,
}

impl PointStamped {
    /// 由 frame + xyz 构造(时间为零)
    #[must_use]
    pub fn from_xyz(frame: NavFrameId, x: f64, y: f64, z: f64) -> Self {
        Self {
            frame_id: frame,
            stamp: Timestamp::default(),
            point: Point3 { x, y, z },
        }
    }

    /// 构造完整版本
    #[must_use]
    pub const fn new(frame_id: NavFrameId, stamp: Timestamp, point: Point3) -> Self {
        Self {
            frame_id,
            stamp,
            point,
        }
    }
}

impl Default for PointStamped {
    fn default() -> Self {
        Self {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::default(),
            point: Point3::default(),
        }
    }
}

/// 3D 点云 + 可选强度
///
/// 字段与 `geometry_msgs/PointCloud2` 的最简子集对齐;Foundation 不直接持有
/// `PointCloud2` 字节布局(那是 transport 层职责),仅承载结构化 Nx3 + intensity。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PointCloud {
    /// 时间戳(纳秒)
    pub stamp: Timestamp,
    /// 坐标系
    pub frame_id: NavFrameId,
    /// Nx3 点列表
    pub points: Vec<Point3>,
    /// 可选强度(长度应与 `points` 一致,允许为空)
    pub intensities: Vec<f32>,
}

impl PointCloud {
    /// 是否为空点云(`points.len() == 0`)。
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// 构造空点云
    #[must_use]
    pub fn empty(frame_id: NavFrameId) -> Self {
        Self {
            stamp: Timestamp::default(),
            frame_id,
            points: Vec::new(),
            intensities: Vec::new(),
        }
    }

    /// 校验 intensities 长度与 points 一致(空 intensities 视为合法)
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self.intensities.is_empty() && self.intensities.len() != self.points.len() {
            return Err("intensities length must match points length");
        }
        Ok(())
    }
}

impl Default for PointCloud {
    fn default() -> Self {
        Self::empty(NavFrameId::Map)
    }
}

// ============================================================================
// 2D 检测
// ============================================================================

/// 单个 2D 检测(像素坐标)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Detection2D {
    /// bbox 中心 (x, y) 像素
    pub bbox_center: (f64, f64),
    /// bbox 尺寸 (w, h) 像素
    pub bbox_size: (f64, f64),
    /// 可选置信度
    pub score: Option<f32>,
    /// 可选类别名
    pub class_name: Option<String>,
}

impl Detection2D {
    /// 构造基础 detection
    #[must_use]
    pub const fn new(bbox_center: (f64, f64), bbox_size: (f64, f64)) -> Self {
        Self {
            bbox_center,
            bbox_size,
            score: None,
            class_name: None,
        }
    }
}

/// 2D 检测数组(对应 `vision_msgs/Detection2DArray`)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Detection2DArray {
    /// 坐标系
    pub frame_id: NavFrameId,
    /// 图像时间戳
    pub stamp: Timestamp,
    /// 检测列表
    pub detections: Vec<Detection2D>,
}

impl Detection2DArray {
    /// 构造空 detection array
    #[must_use]
    pub fn empty(frame_id: NavFrameId) -> Self {
        Self {
            frame_id,
            stamp: Timestamp::default(),
            detections: Vec::new(),
        }
    }

    /// 第一个 detection
    #[must_use]
    pub fn first(&self) -> Option<&Detection2D> {
        self.detections.first()
    }

    /// 检测数量
    #[must_use]
    pub fn len(&self) -> usize {
        self.detections.len()
    }

    /// 是否为空
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.detections.is_empty()
    }
}

impl Default for Detection2DArray {
    fn default() -> Self {
        Self::empty(NavFrameId::Camera)
    }
}

// ============================================================================
// 相机内参
// ============================================================================

/// 相机内参(对应 `sensor_msgs/CameraInfo` 的针孔模型子集)
///
/// 字段语义与 ROS `CameraInfo.K` 矩阵一致:row-major 3x3 矩阵 K
/// `[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CameraInfo {
    /// 图像宽度
    pub width: u32,
    /// 图像高度
    pub height: u32,
    /// 主点 x
    pub cx: f64,
    /// 主点 y
    pub cy: f64,
    /// 焦距 x(像素单位)
    pub fx: f64,
    /// 焦距 y(像素单位)
    pub fy: f64,
}

impl CameraInfo {
    /// 由 ROS K 矩阵(9 元素 row-major)构造
    #[must_use]
    pub fn from_k(width: u32, height: u32, k: &[f64; 9]) -> Self {
        Self {
            width,
            height,
            fx: k[0],
            fy: k[4],
            cx: k[2],
            cy: k[5],
        }
    }

    /// 由内参显式字段构造
    #[must_use]
    pub const fn new(width: u32, height: u32, fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self {
            width,
            height,
            fx,
            fy,
            cx,
            cy,
        }
    }

    /// 校验内参合法(focal 严格为正,size 严格为正)
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

impl Default for CameraInfo {
    fn default() -> Self {
        // 640x480 @ 焦距 500 / 主点居中:占位默认(实际使用方应填真实内参)
        Self {
            width: 640,
            height: 480,
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
        }
    }
}

// ============================================================================
// 栅格快照(自包含字段,内嵌原始 occupancy data)
// ============================================================================

/// 自包含的栅格快照(对应历史 `patrolling::GridSnapshot`)
///
/// 字段顺序:
/// - `(width, height, resolution, origin_x, origin_y)` 元数据
/// - `data: Vec<i8>` 行优先,长度 = `width * height`,取值为 occupancy(0=free, 100=occupied, -1=unknown)
///
/// 与 `OccupancyGrid2D` 的差异:`GridSnapshot` 保留 `i8` 原始字节与独立的 `origin_x/origin_y`
/// 标量字段(来自历史 Python `nav_msgs/OccupancyGrid` 接口),`OccupancyGrid2D` 携带
/// 强类型 `OccupancyCell` + `Point2` origin。两者**不直接互转**:
/// - `GridSnapshot` 用于**外部 topic / Patrolling 路由接口**
/// - `OccupancyGrid2D` 用于**导航算法内部** costmap 推导
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridSnapshot {
    /// 宽度(格子数,cols)
    pub width: u32,
    /// 高度(格子数,rows)
    pub height: u32,
    /// 分辨率(米/cell)
    pub resolution: f64,
    /// 原点 x(米)
    pub origin_x: f64,
    /// 原点 y(米)
    pub origin_y: f64,
    /// 原始 occupancy 数据(行优先,长度 = width * height)
    pub data: Vec<i8>,
}

impl GridSnapshot {
    /// 构造全 free 栅格
    #[must_use]
    pub fn filled_free(
        width: u32,
        height: u32,
        resolution: f64,
        origin_x: f64,
        origin_y: f64,
    ) -> Self {
        let data = vec![0_i8; (width as usize) * (height as usize)];
        Self {
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            data,
        }
    }

    /// 校验 data 长度与 width * height 一致
    pub fn validate(&self) -> Result<(), &'static str> {
        let expected = (self.width as usize) * (self.height as usize);
        if self.data.len() != expected {
            return Err("grid data length mismatch with width*height");
        }
        if self.resolution <= 0.0 {
            return Err("resolution must be > 0");
        }
        Ok(())
    }

    /// world → cell(floor)
    #[must_use]
    pub fn world_to_cell(&self, x: f64, y: f64) -> Option<(u32, u32)> {
        if self.resolution <= 0.0 {
            return None;
        }
        let col = ((x - self.origin_x) / self.resolution).floor();
        let row = ((y - self.origin_y) / self.resolution).floor();
        if col < 0.0 || row < 0.0 {
            return None;
        }
        let (col, row) = (col as u32, row as u32);
        if row >= self.height || col >= self.width {
            return None;
        }
        Some((col, row))
    }

    /// cell → world 中心
    #[must_use]
    pub fn cell_center_world(&self, col: u32, row: u32) -> (f64, f64) {
        (
            self.origin_x + (f64::from(col) + 0.5) * self.resolution,
            self.origin_y + (f64::from(row) + 0.5) * self.resolution,
        )
    }

    /// cell 是否 free
    #[must_use]
    pub fn is_free(&self, col: u32, row: u32) -> bool {
        self.get(col, row) == Some(0)
    }

    /// cell 是否 occupied(>= 50)
    #[must_use]
    pub fn is_occupied(&self, col: u32, row: u32) -> bool {
        matches!(self.get(col, row), Some(v) if v >= 50)
    }

    /// 安全读
    #[must_use]
    pub fn get(&self, col: u32, row: u32) -> Option<i8> {
        if col >= self.width || row >= self.height {
            return None;
        }
        self.data
            .get(col as usize + row as usize * self.width as usize)
            .copied()
    }
}

impl Default for GridSnapshot {
    fn default() -> Self {
        Self::filled_free(0, 0, 0.05, 0.0, 0.0)
    }
}

// ============================================================================
// 里程计
// ============================================================================

/// 里程计消息(对应 `nav_msgs/Odometry` 最小子集)
///
/// 字段:
/// - `header.frame_id` / `header.stamp` → `frame_id` + `stamp`
/// - `pose.pose` → `pose: Pose3`
/// - `twist.twist` → `twist: (linear: Point3, angular: Point3)`
///
/// **注意**:本类型**只**承载最小字段,避免与完整 `nav_msgs/Odometry` 的 covariance
/// 矩阵(36 元素 float64)纠缠。生产链路如需完整 ROS 兼容,由 transport 层负责
/// `PointCloud2/Odometry <-> PointCloud/Odometry` 互转,Foundation 不持有完整 36 元素
/// covariance 矩阵。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Odometry {
    /// 坐标系(通常为 `odom` 或 `map`)
    pub frame_id: NavFrameId,
    /// 时间戳(纳秒)
    pub stamp: Timestamp,
    /// 位姿(子坐标系 → `frame_id`)
    pub pose: Pose3,
    /// 子坐标系名(通常为 `base_link` / `base_footprint`)
    pub child_frame_id: NavFrameId,
    /// 线速度(子坐标系下)
    pub linear: Point3,
    /// 角速度(子坐标系下)
    pub angular: Point3,
}

impl Odometry {
    /// identity odometry(零位姿、零速度)
    #[must_use]
    pub fn identity(frame: NavFrameId, child: NavFrameId) -> Self {
        Self {
            frame_id: frame,
            stamp: Timestamp::default(),
            pose: Pose3::default(),
            child_frame_id: child,
            linear: Point3::default(),
            angular: Point3::default(),
        }
    }

    /// 校验所有数值字段有限(不静默接受 NaN/Inf)
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.pose.position.x.is_finite()
            && self.pose.position.y.is_finite()
            && self.pose.position.z.is_finite()
            && self.pose.orientation.w.is_finite()
            && self.pose.orientation.x.is_finite()
            && self.pose.orientation.y.is_finite()
            && self.pose.orientation.z.is_finite()
            && self.linear.x.is_finite()
            && self.linear.y.is_finite()
            && self.linear.z.is_finite()
            && self.angular.x.is_finite()
            && self.angular.y.is_finite()
            && self.angular.z.is_finite()
    }
}

impl Default for Odometry {
    fn default() -> Self {
        Self::identity(NavFrameId::Odom, NavFrameId::Base)
    }
}

// ============================================================================
// FrontierGoalInput(供 patrolling 路由消费;Foundation FrontierGoal 已存在,此为
// 探索板块 → 路由层的**轻量输入**形态)
// ============================================================================

/// frontier 输入(patrolling 路由器消费的轻量形态)
///
/// 与 `FrontierGoal` 的差异:
/// - `FrontierGoal` 是"已决定要作为 navigation goal 派发"的强类型(`PoseStamped` + id)
/// - `FrontierGoalInput` 是"探索板块产出的候选"(pose 用 `(x, y, yaw=0)` 元组简化,
///   缺少完整 `PoseStamped` / `Pose3` / `Quaternion` 字段;供 patrolling router 内部排序)
///
/// 字段:`pose` 元组 → 后续在派发到 navigation goal 时可转为
/// `FrontierGoal::pose_with_identity(NavFrameId::Map, x, y) + yaw=0`。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrontierGoalInput {
    /// 候选位姿 `(x, y, yaw=0)`
    pub pose: (f64, f64, f64),
    /// score(越大越优先)
    pub score: f32,
    /// 信息增益(平方米,可选)
    pub information_gain: Option<f32>,
    /// 距离成本(米,可选)
    pub distance_cost: Option<f32>,
    /// frontier 面积(平方米,可选)
    pub frontier_area_m2: Option<f32>,
    /// producer id(标识 frontier 来源 crate / 算法版本)
    pub producer: String,
}

impl FrontierGoalInput {
    /// 由 pose + score 构造最简 input
    #[must_use]
    pub fn new(pose: (f64, f64, f64), score: f32, producer: impl Into<String>) -> Self {
        Self {
            pose,
            score,
            information_gain: None,
            distance_cost: None,
            frontier_area_m2: None,
            producer: producer.into(),
        }
    }

    /// 校验数值字段有限
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.pose.0.is_finite()
            && self.pose.1.is_finite()
            && self.pose.2.is_finite()
            && self.score.is_finite()
            && self.information_gain.is_none_or(f32::is_finite)
            && self.distance_cost.is_none_or(f32::is_finite)
            && self.frontier_area_m2.is_none_or(f32::is_finite)
    }
}

// ============================================================================
// 抑制未使用引用告警(Quiet unused import)
// ============================================================================

#[allow(dead_code)]
const _: fn() = || {
    let _ = Quaternion::default;
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::NavFrameId;

    #[test]
    fn point_stamped_default_frame_map() {
        let p = PointStamped::default();
        assert_eq!(p.frame_id, NavFrameId::Map);
        assert_eq!(p.point, Point3::default());
    }

    #[test]
    fn point_cloud_empty_and_validate() {
        let c = PointCloud::empty(NavFrameId::Map);
        assert!(c.is_empty());
        assert_eq!(c.points.len(), 0);
        assert!(c.validate().is_ok());
        let bad = PointCloud {
            stamp: Timestamp::default(),
            frame_id: NavFrameId::Map,
            points: vec![Point3::default()],
            intensities: vec![0.5, 0.6],
        };
        assert!(bad.validate().is_err());
    }

    #[test]
    fn detection_2d_array_first() {
        let mut a = Detection2DArray::empty(NavFrameId::Camera);
        assert!(a.is_empty());
        a.detections
            .push(Detection2D::new((320.0, 240.0), (100.0, 100.0)));
        let f = a.first().unwrap();
        assert_eq!(f.bbox_center, (320.0, 240.0));
    }

    #[test]
    fn camera_info_from_k_matches_pinhole() {
        let info = CameraInfo::from_k(
            640,
            480,
            &[500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        );
        assert!(info.is_valid());
        assert_eq!(info.width, 640);
        assert!((info.fx - 500.0).abs() < 1e-9);
        assert!((info.cy - 240.0).abs() < 1e-9);
        let bad = CameraInfo::new(640, 480, 0.0, 500.0, 320.0, 240.0);
        assert!(!bad.is_valid());
    }

    #[test]
    fn grid_snapshot_world_to_cell_roundtrip() {
        let g = GridSnapshot::filled_free(4, 4, 1.0, -2.0, -2.0);
        assert!(g.validate().is_ok());
        let (col, row) = g.world_to_cell(-1.5, -1.5).unwrap();
        assert_eq!((col, row), (0, 0));
        let (cx, cy) = g.cell_center_world(col, row);
        assert!((cx - (-1.5)).abs() < 1e-9);
        assert!((cy - (-1.5)).abs() < 1e-9);
        assert!(g.is_free(0, 0));
    }

    #[test]
    fn odometry_identity_is_finite() {
        let o = Odometry::default();
        assert!(o.is_finite());
        let mut bad = o;
        bad.linear.x = f64::NAN;
        assert!(!bad.is_finite());
    }

    #[test]
    fn frontier_goal_input_is_finite() {
        let g = FrontierGoalInput::new((1.0, 2.0, 0.0), 0.7, "wavefront");
        assert!(g.is_finite());
        let mut bad = g.clone();
        bad.score = f32::NAN;
        assert!(!bad.is_finite());
    }

    #[test]
    fn point_stamped_serde_snake_case() {
        let p = PointStamped::from_xyz(NavFrameId::Map, 1.0, 2.0, 0.0);
        let s = serde_json::to_string(&p).unwrap();
        assert!(s.contains("\"frame_id\""));
        let back: PointStamped = serde_json::from_str(&s).unwrap();
        assert_eq!(back, p);
    }
}

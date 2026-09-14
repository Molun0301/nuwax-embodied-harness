//! 导航板块共享类型与消息
//!
//! 本 crate 是导航板块的"协议面":定义后续所有导航 crate 共用的强类型消息、
//! 几何、网格、路径、目标、视觉与诊断结构。它**仅依赖 workspace shared crate**,
//! 不引用任何具体算法 crate(A* / simple planner / native runner 等)。
//!
//! 设计约束:
//! - 所有时间字段一律使用**纳秒**(`i64 nanos + ClockKind`),不允许混用微秒/毫秒
//! - 所有 bbox 一律使用 **xyxy 像素坐标**(原点左上,`x1 <= x2`、`y1 <= y2`)
//! - 所有 serde 字段使用 `snake_case`
//! - 公开 API 不得包含旧品牌字段
//!
//! ## 模块划分
//! - [`frames`][]:坐标系标识(`NavFrameId`)
//! - [`geometry`][]:几何/位姿/时间戳(`Point2/3`、`Pose2/3`、`Quaternion`、`Twist2/3`、`Timestamp`)
//! - [`grid`][]:占据栅格与代价地图(`OccupancyGrid2D`、`CostMap2D`、`HeightMap`/`TerrainMap`)
//! - [`path`][]:路径与控制命令(`Path2D/3`、`ControllerCommand`)
//! - [`goal`][]:导航目标与状态机(`NavigationGoal`、`NavigationStatus`、`PlannerRequest/Result`)
//! - [`visual`][]:视觉目标与 frontier(`BoundingBox2D`、`VisualTarget`、`FrontierGoal`)
//! - [`sensors`][]:传感器/感知消息(`PointStamped`、`PointCloud`、`CameraInfo`、`Detection2DArray`、`GridSnapshot`、`Odometry`、`FrontierGoalInput`)
//! - [`diagnostics`][]:跨 crate 诊断辅助 trait

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod diagnostics;
pub mod navigation_capabilities;
pub use navigation_capabilities::{MotionLimits, MotionModel, NavigationCapabilities, ObservationKind, ObservationModel, TerrainLimits};
pub mod dimos;
pub mod frames;
pub mod geometry;
pub mod goal;
pub mod grid;
pub mod path;
pub mod robot_geometry;
pub mod sensors;
pub mod visual;

pub use diagnostics::*;
pub use dimos::*;
pub use frames::*;
pub use geometry::*;
pub use goal::*;
pub use grid::*;
pub use path::*;
pub use robot_geometry::*;
pub use sensors::*;
pub use visual::*;

pub use nuwax_types::navigation::GoalOptions;

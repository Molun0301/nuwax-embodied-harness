//! 轻量栅格 A* 全局路径规划器(对应原 Python `simple_planner.py`)。
//!
//! 本 crate 提供:
//! - 通用导航消息类型:通过 `pub use` 转发 [`nuwax_nav_types_messages`] 的
//!   `Point2 / Point3 / Pose3 / PoseStamped / Path / PointCloud` 等。
//! - Costmap(带历史最大高度与脏标记)、膨胀、A*、Stuck 升级、waypoint lookahead、
//!   点云分类等核心算法。
//!
//! 启用 `module` 特性后提供同步模块适配器；消息传输与调度由导航编排层负责。
//!
//! # 用法示例
//!
//! ```no_run
//! use nuwax_nav_simple_planner::{Costmap, plan_on_costmap, Point2};
//! let cm = Costmap::new(0.5, 0.1, 0.2).unwrap();
//! let path = plan_on_costmap(&cm, Point2 { x: 0.0, y: 0.0 }, Point2 { x: 2.0, y: 0.0 }, None, None).unwrap();
//! ```

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod astar;
pub mod config;
pub mod costmap;
pub mod inflation;
pub mod planner;
pub mod pointcloud;
pub mod stuck;
pub mod waypoint;

#[cfg(feature = "module")]
pub mod module;

// ============================================================================
// Foundation re-export(单一真实来源)
// ============================================================================
//
// 通用消息类型:从 Foundation crate 转发,本 crate 不再保留任何 `pub struct` 重复定义。
// 调用方既可用 `crate::Point2` 也可直接 `use nuwax_nav_types_messages::Point2`。
pub use nuwax_nav_types_messages::{Point2, Point3, Pose3, PoseStamped, Quaternion, Timestamp};
// 兼容:simple-planner 早期 `Path` 名称实际是 Foundation `Path3D`
// (带 stamp + frame_id + Vec<PoseStamped>)。
pub use nuwax_nav_types_messages::Path3D as Path;
// sensors
pub use nuwax_nav_types_messages::{PointCloud, PointStamped};

/// 标量布尔消息(对应 `std_msgs/Bool`)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct BoolMsg {
    /// 布尔值
    pub data: bool,
}

/// 统一错误类型(从 Foundation 转发)。
pub use nuwax_nav_base::NavError;

pub use astar::astar;
pub use config::SimplePlannerConfig;
pub use costmap::Costmap;
pub use inflation::{blocked_at_inflation, inflate_obstacles};
pub use planner::{plan_on_costmap, SimplePlanner};
pub use pointcloud::classify_points;
pub use stuck::{progress_tick, StuckState};
pub use waypoint::lookahead;

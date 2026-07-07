//! Stream/RPC 适配器骨架。
//!
//! 本模块仅在启用 `module` feature 时编译；提供：
//! - `subscribe_goal` / `subscribe_stop_movement` / `subscribe_terrain_map` / `subscribe_terrain_map_ext`：
//!   占位函数，由 Foundation lane 完成 `nuwax-nav-base` 与 transport 后替换实现。
//! - 当前只暴露文档与单元占位；不引入 transport 依赖以避免 cycle。

use crate::{NavError, PointStamped, Pose3};

/// `subscribe_goal` 占位。
pub fn subscribe_goal(_callback: fn(PointStamped)) -> Result<(), NavError> {
    Err(NavError::Internal(
        "module feature 待 Foundation lane 接入 transport 后实现".into(),
    ))
}

/// `subscribe_stop_movement` 占位。
pub fn subscribe_stop_movement(_callback: fn(bool)) -> Result<(), NavError> {
    Err(NavError::Internal(
        "module feature 待 Foundation lane 接入 transport 后实现".into(),
    ))
}

/// `subscribe_terrain_map_ext` 占位。
pub fn subscribe_terrain_map_ext(_callback: fn(Vec<crate::Point3>, Pose3)) -> Result<(), NavError> {
    Err(NavError::Internal(
        "module feature 待 Foundation lane 接入 transport 后实现".into(),
    ))
}

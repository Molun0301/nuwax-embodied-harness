//! Stream/RPC 适配器骨架。
//!
//! 仅在启用 `module` feature 时编译；接入 Foundation lane 完成 `nuwax-nav-base`
//! 与 transport 后替换实现。

use crate::{NavError, Point3, Pose3};

/// `subscribe_registered_scan` 占位。
pub fn subscribe_registered_scan(_callback: fn(Vec<Point3>, Vec<f64>)) -> Result<(), NavError> {
    Err(NavError::Internal(
        "module feature 待 Foundation lane 接入 transport 后实现".into(),
    ))
}

/// `subscribe_odometry` 占位。
pub fn subscribe_odometry(_callback: fn(Pose3)) -> Result<(), NavError> {
    Err(NavError::Internal(
        "module feature 待 Foundation lane 接入 transport 后实现".into(),
    ))
}

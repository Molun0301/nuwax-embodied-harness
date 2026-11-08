//! 由现行配置选择固定的三维或二维执行链。
use crate::{NavStackError, NavStackResult};

/// 两条固定生产链。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProductionPipeline {
    /// 三维激光惯性建图、表面探索与地形导航。
    Lio3D,
    /// 二维激光建图、波前探索与平面导航。
    Slam2D,
}

impl ProductionPipeline {
    /// 从合成配置读取实际执行链。
    pub fn from_runtime() -> NavStackResult<Self> {
        match nuwax_config::geometry().map_err(NavStackError::InvalidConfig)?.pipeline.as_str() {
            "lio_3d" => Ok(Self::Lio3D),
            "slam_2d" => Ok(Self::Slam2D),
            value => Err(NavStackError::invalid(format!("未知执行链：{value}"))),
        }
    }

    /// 解析启动器传入的导航配置；不再重复检查执行链配对。
    pub fn from_profile(value: &str) -> NavStackResult<Self> {
        match value {
            "pct-scan-3d" => Ok(Self::Lio3D),
            "simple-2d" => Ok(Self::Slam2D),
            _ => Err(NavStackError::invalid(format!("未知导航配置：{value}"))),
        }
    }

    /// 执行链的配置名称。
    pub fn as_str(self) -> &'static str {
        match self { Self::Lio3D => "lio_3d", Self::Slam2D => "slam_2d" }
    }
}

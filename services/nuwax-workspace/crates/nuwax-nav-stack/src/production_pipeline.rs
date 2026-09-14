//! 建图、探索和导航的固定生产配对，供几何拥有方和导航入口共同捕获。

use crate::{NavStackError, NavStackProfile, NavStackResult};

/// 两条互斥的完整生产链。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProductionPipeline {
    /// 三维激光惯性建图、表面探索和多层地形导航。
    Lio3D,
    /// 二维激光建图、波前探索和平面导航。
    Slam2D,
}

impl ProductionPipeline {
    /// 解析执行链并核对可选的显式导航值；不允许跨维度或静默回退。
    pub fn from_settings(pipeline: Option<&str>, profile: Option<&str>) -> NavStackResult<Self> {
        if pipeline.is_some_and(|value| value != value.trim()) {
            return Err(NavStackError::invalid("NUWAX_GEOMETRY_PIPELINE不能包含首尾空白"));
        }
        let selected = match pipeline.map(str::trim).filter(|value| !value.is_empty()) {
            None | Some("lio_3d") => Self::Lio3D,
            Some("slam_2d") => Self::Slam2D,
            Some(_) => {
                return Err(NavStackError::invalid(
                    "NUWAX_GEOMETRY_PIPELINE必须为lio_3d或slam_2d",
                ));
            }
        };
        if let Some(profile) = profile.map(str::trim).filter(|value| !value.is_empty()) {
            if profile != selected.navigation_profile().as_str() {
                return Err(NavStackError::invalid(format!(
                    "执行链{}固定使用{}导航，NUWAX_NAV_PROFILE={}与执行链冲突；请移除该覆盖或改为{}",
                    selected.as_str(),
                    selected.navigation_profile().as_str(),
                    profile,
                    selected.navigation_profile().as_str(),
                )));
            }
        }
        Ok(selected)
    }

    /// 在创建生产资源前读取并捕获配置；非文本环境值明确拒绝。
    pub fn from_env() -> NavStackResult<Self> {
        let read = |name: &str| match std::env::var(name) {
            Ok(value) => Ok(Some(value)),
            Err(std::env::VarError::NotPresent) => Ok(None),
            Err(std::env::VarError::NotUnicode(_)) => {
                Err(NavStackError::invalid(format!("{name}必须为有效文本")))
            }
        };
        Self::from_settings(
            read("NUWAX_GEOMETRY_PIPELINE")?.as_deref(),
            read("NUWAX_NAV_PROFILE")?.as_deref(),
        )
    }

    /// 执行链的规范名称。
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Lio3D => "lio_3d",
            Self::Slam2D => "slam_2d",
        }
    }

    /// 执行链唯一允许的导航配置。
    pub fn navigation_profile(self) -> NavStackProfile {
        match self {
            Self::Lio3D => NavStackProfile::Mls3D,
            Self::Slam2D => NavStackProfile::Simple2D,
        }
    }

    /// 校验实际将执行的配置，覆盖直接构造命令行选项的调用方。
    pub fn validate_navigation(self, profile: NavStackProfile) -> NavStackResult<()> {
        Self::from_settings(Some(self.as_str()), Some(profile.as_str())).map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn complete_pipelines_derive_and_reject_cross_dimension_execution() {
        assert_eq!(
            ProductionPipeline::from_settings(None, None).unwrap(),
            ProductionPipeline::Lio3D
        );
        for pipeline in [ProductionPipeline::Slam2D, ProductionPipeline::Lio3D] {
            for profile in NavStackProfile::all() {
                let result = ProductionPipeline::from_settings(
                    Some(pipeline.as_str()),
                    Some(profile.as_str()),
                );
                assert_eq!(result.is_ok(), *profile == pipeline.navigation_profile());
            }
            assert_eq!(
                ProductionPipeline::from_settings(Some(pipeline.as_str()), Some(" ")).unwrap(),
                pipeline
            );
        }
        assert!(ProductionPipeline::from_settings(Some("unknown"), None).is_err());
        assert!(ProductionPipeline::from_settings(None, Some("unknown")).is_err());
    }
}

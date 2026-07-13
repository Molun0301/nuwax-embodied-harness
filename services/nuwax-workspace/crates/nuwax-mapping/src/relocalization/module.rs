//! 重定位 Module
//! 基于 nuwax-core Module trait 包装全局特征重定位管线。

use super::relocalize::{
    relocation_pipeline_with_config, IcpConfig, IcpResult, RelocalizationConfig,
};
use async_trait::async_trait;
use nalgebra::{Isometry3, Point3};
use nuwax_core::{Module, ModuleResult};

/// 重定位 Module
pub struct RelocalizationModule {
    config: IcpConfig,
}

impl RelocalizationModule {
    /// 构造
    #[must_use]
    pub fn new(config: IcpConfig) -> Self {
        Self { config }
    }

    /// 一次性调用 FPFH + RANSAC + seeded ICP 重定位管线。
    #[must_use]
    pub fn relocalize(&self, source: &[Point3<f64>], target: &[Point3<f64>]) -> IcpResult {
        let mut config = RelocalizationConfig::default();
        config.icp = self.config;
        relocation_pipeline_with_config(source, target, config).registration
    }

    /// Convert an accepted local-to-map registration into the world-to-map
    /// correction published by the original DimOS module.
    #[must_use]
    pub fn corrected_world_to_map(&self, registration: &IcpResult) -> Option<Isometry3<f64>> {
        registration
            .converged
            .then(|| registration.transformation.inverse())
    }
}

impl Default for RelocalizationModule {
    fn default() -> Self {
        Self::new(IcpConfig::default())
    }
}

#[async_trait]
impl Module for RelocalizationModule {
    async fn start(&self) -> ModuleResult<()> {
        Ok(())
    }

    async fn stop(&self) -> ModuleResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn module_lifecycle() {
        let m = RelocalizationModule::default();
        m.start().await.unwrap();
        m.stop().await.unwrap();
    }

    #[tokio::test]
    async fn module_relocalize_call() {
        let m = RelocalizationModule::default();
        let target: Vec<Point3<f64>> = (0..4)
            .flat_map(|x| {
                (0..3).flat_map(move |y| {
                    (0..2).map(move |z| {
                        Point3::new(
                            x as f64 * 0.41 + (y * y + z) as f64 * 0.013,
                            y as f64 * 0.43 + (x * z + z * z) as f64 * 0.017,
                            z as f64 * 0.37 + (x * x + y) as f64 * 0.011,
                        )
                    })
                })
            })
            .collect();
        let source = target.clone();
        let res = m.relocalize(&source, &target);
        assert!(res.converged);
    }
}

//! 重定位 Module
//! 基于 nuwax-core Module trait 包装 ICP 求解器。

use super::relocalize::{IcpConfig, IcpResult, IcpSolver};
use async_trait::async_trait;
use nalgebra::Point3;
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

    /// 一次性调用 ICP
    #[must_use]
    pub fn relocalize(&self, source: &[Point3<f64>], target: &[Point3<f64>]) -> IcpResult {
        IcpSolver::new(self.config).align(source, target)
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
        let target = vec![
            nalgebra::point![0.0, 0.0, 0.0],
            nalgebra::point![1.0, 0.0, 0.0],
            nalgebra::point![0.0, 1.0, 0.0],
        ];
        let source = target.clone();
        let res = m.relocalize(&source, &target);
        assert!(res.converged);
    }
}

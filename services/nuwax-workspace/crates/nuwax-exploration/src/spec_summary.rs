//! Phase E6 板块 3 typed spec 整合点
//!
//! `NativeSpecSummary` 聚合 `NativeProgramSpec` + `NativeBuildSpec`,
//! 供 `nuwax-nav-stack` / `ModuleNode::from_native_specs` 在不引入具体 bridge
//! crate 的情况下,以类型安全的形式表达 "一个 typed native node"。
//!
//! ## 设计目标
//!
//! - **跨 bridge 统一形状**:TARE / Terrain Analysis / FAR / Local / Path Follower
//!   / PGO 等不同 bridge 的 `build_specs` / `build_spec` 都返回
//!   `(NativeProgramSpec, NativeBuildSpec)` 或 `NativeProgramSpec`;本类型用
//!   `Option<NativeBuildSpec>` 兼容二者。
//! - **避免 facade 暴露具体 bridge 配置字段**:不持有 `TarePlannerBridgeConfig`
//!   / `TerrainAnalysisBridgeConfig` 等具体类型,只持有已渲染的 spec。
//! - **零开销传递**:`NativeSpecSummary` 仅数据,无运行时 hook。
//!
//! ## 不做事项
//!
//! - 不调用具体 bridge 的 `build_specs`(由 caller 自行渲染,以避免 facade
//!   强依赖所有 bridge crate)。
//! - 不实现 typed spec 校验(`NativeProgramSpec::validate` 由 caller 触发)。

use nuwax_nav_native_runner::{NativeBuildSpec, NativeProgramSpec};
use serde::{Deserialize, Serialize};

/// 聚合 typed native spec + build spec,用于 cross-bridge 整合。
///
/// 典型用法:
/// ```ignore
/// let (spec, build) = tare_bridge::build_specs(cfg, &topics)?;
/// let summary = NativeSpecSummary::new(spec, Some(build));
/// // 交给 stack / 平台层统一驱动 spawn / wait。
/// let _ = summary;
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)] // Eq excluded: NativeProgramSpec.executable contains f64
pub struct NativeSpecSummary {
    /// 已渲染的程序 spec(executable / args / topics / env / restart)
    pub spec: NativeProgramSpec,
    /// 可选 build spec;`None` 等价于 "不自动 build"
    pub build: Option<NativeBuildSpec>,
}

impl NativeSpecSummary {
    /// 构造 summary。
    #[must_use]
    pub fn new(spec: NativeProgramSpec, build: Option<NativeBuildSpec>) -> Self {
        Self { spec, build }
    }

    /// 仅 program spec(无 build info)。
    ///
    /// 用于 `build_spec`(单 spec 版本)风格 bridge:PGO / FAR / Local /
    /// Path Follower 返回的 `NativeProgramSpec`,无对应 `NativeBuildSpec`。
    #[must_use]
    pub fn from_spec(spec: NativeProgramSpec) -> Self {
        Self { spec, build: None }
    }

    /// 取出 `executable.display()` 的字符串形式,
    /// 便于 `--dry-run` 打印与跨平台显示。
    #[must_use]
    pub fn binary_display(&self) -> String {
        self.spec.executable.display().to_string()
    }

    /// 是否包含 `NativeBuildSpec`。
    #[must_use]
    pub fn has_build(&self) -> bool {
        self.build.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    use std::path::PathBuf;

    use nuwax_nav_base::native::RestartPolicy;

    fn sample_spec() -> NativeProgramSpec {
        NativeProgramSpec {
            name: "x".to_string(),
            executable: PathBuf::from("result/bin/x"),
            topics: BTreeMap::new(),
            args: vec![],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        }
    }

    #[test]
    fn new_stores_both_spec_and_build() {
        let s = NativeSpecSummary::new(sample_spec(), Some(NativeBuildSpec::default()));
        assert_eq!(s.binary_display(), "result/bin/x");
        assert!(s.has_build());
    }

    #[test]
    fn from_spec_has_no_build() {
        let s = NativeSpecSummary::from_spec(sample_spec());
        assert!(!s.has_build());
    }
}

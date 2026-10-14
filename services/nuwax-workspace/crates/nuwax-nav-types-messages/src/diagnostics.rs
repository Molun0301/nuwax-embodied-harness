//! 跨 crate 诊断辅助 trait / 类型
//!
//! 具体诊断结构(`NavDiagnostics` / `PlannerDiagnostics` / ...)定义在
//! [`nuwax-nav-base`],本 crate 仅提供 trait 与共享枚举,确保序列化协议不重复。

use serde::{Deserialize, Serialize};

/// 诊断严重等级
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DiagnosticLevel {
    /// 跟踪
    Trace,
    /// 调试
    Debug,
    /// 信息
    #[default]
    Info,
    /// 警告
    Warn,
    /// 错误
    Error,
}

/// 通用诊断 trait,各 crate 自行实现
///
/// `component` 必须返回 crate / module 短名(用于日志 / 健康检查),
/// **不得包含旧品牌字段**。
pub trait DiagnosticsSnapshot {
    /// 组件名(例如 `"nuwax-nav-astar"`、`"nuwax-nav-native-runner"`)
    fn component(&self) -> &'static str;
    /// 严重等级
    fn level(&self) -> DiagnosticLevel;
    /// 一行简短描述(供日志 / heartbeat)
    fn summary(&self) -> String;
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Dummy;

    impl DiagnosticsSnapshot for Dummy {
        fn component(&self) -> &'static str {
            "nuwax-nav-test"
        }
        fn level(&self) -> DiagnosticLevel {
            DiagnosticLevel::Info
        }
        fn summary(&self) -> String {
            "ok".to_string()
        }
    }

    #[test]
    fn diagnostic_level_default_info() {
        assert_eq!(DiagnosticLevel::default(), DiagnosticLevel::Info);
    }

    #[test]
    fn diagnostics_trait_basic() {
        let d = Dummy;
        assert_eq!(d.component(), "nuwax-nav-test");
        assert_eq!(d.level(), DiagnosticLevel::Info);
        assert_eq!(d.summary(), "ok");
    }
}

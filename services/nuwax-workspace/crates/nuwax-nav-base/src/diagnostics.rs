//! 诊断结构(`NavDiagnostics` / `PlannerDiagnostics` / `ControllerDiagnostics`)
//!
//! 提供 builder + `DiagnosticsSnapshot` 实现,供 stack / 健康检查使用。

use std::collections::BTreeMap;

use nuwax_nav_types_messages::{
    diagnostics::{DiagnosticLevel, DiagnosticsSnapshot},
    Timestamp,
};
use serde::{Deserialize, Serialize};

/// 通用计数(诊断中常用)
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Counters {
    /// map
    pub values: BTreeMap<String, u64>,
}

impl Counters {
    /// 自增(创建键)
    pub fn inc(&mut self, key: impl Into<String>) {
        *self.values.entry(key.into()).or_insert(0) += 1;
    }

    /// 自增 by delta
    pub fn inc_by(&mut self, key: impl Into<String>, delta: u64) {
        *self.values.entry(key.into()).or_insert(0) += delta;
    }

    /// 取值(默认 0)
    #[must_use]
    pub fn get(&self, key: &str) -> u64 {
        self.values.get(key).copied().unwrap_or(0)
    }
}

/// Planner 诊断
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PlannerDiagnostics {
    /// 组件名
    pub component: String,
    /// 已扩展节点数(最近一次)
    pub last_explored_nodes: usize,
    /// open set 峰值(最近一次)
    pub last_open_set_peak: usize,
    /// 累计 path cost(最近一次成功)
    pub last_path_cost: Option<f64>,
    /// 最近一次 replan 原因
    pub last_replan_reason: Option<String>,
    /// 计数
    pub counters: Counters,
}

impl PlannerDiagnostics {
    /// 新建(给定组件名)
    #[must_use]
    pub fn new(component: impl Into<String>) -> Self {
        Self {
            component: component.into(),
            ..Self::default()
        }
    }
}

impl DiagnosticsSnapshot for PlannerDiagnostics {
    fn component(&self) -> &'static str {
        // 返回 &'static str:借用字面量近似(实际 component 是 String)
        // 这里返回 `"nuwax-nav-planner"` 作为占位短名;
        // 详细 component 通过 self.component() 不在本 trait surface。
        "nuwax-nav-planner"
    }
    fn level(&self) -> DiagnosticLevel {
        if self.last_replan_reason.is_some() && self.counters.get("blocked") > 0 {
            DiagnosticLevel::Warn
        } else {
            DiagnosticLevel::Info
        }
    }
    fn summary(&self) -> String {
        format!(
            "planner explored={} cost={:?}",
            self.last_explored_nodes, self.last_path_cost
        )
    }
}

/// Controller 诊断
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ControllerDiagnostics {
    /// 组件名
    pub component: String,
    /// 当前跟踪的目标 index
    pub target_index: usize,
    /// cross-track error(米)
    pub cross_track_error_m: f64,
    /// heading error(弧度,wrap 到 -pi..pi)
    pub heading_error_rad: f64,
    /// 是否发生 saturation(限幅)
    pub saturated: bool,
    /// 最近一次命令耗时(纳秒)
    pub last_compute_latency_ns: Option<i64>,
    /// 计数
    pub counters: Counters,
}

impl ControllerDiagnostics {
    /// 新建(给定组件名)
    #[must_use]
    pub fn new(component: impl Into<String>) -> Self {
        Self {
            component: component.into(),
            ..Self::default()
        }
    }
}

impl DiagnosticsSnapshot for ControllerDiagnostics {
    fn component(&self) -> &'static str {
        "nuwax-nav-controller"
    }
    fn level(&self) -> DiagnosticLevel {
        if self.saturated {
            DiagnosticLevel::Warn
        } else {
            DiagnosticLevel::Info
        }
    }
    fn summary(&self) -> String {
        format!(
            "controller target={} cte={:.3} heading_err={:.3} saturated={}",
            self.target_index, self.cross_track_error_m, self.heading_error_rad, self.saturated
        )
    }
}

/// 顶层 navigation diagnostics(stack 输出)
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct NavDiagnostics {
    /// 组件名
    pub component: String,
    /// 当前状态(短串)
    pub state: String,
    /// 最近输入延迟(纳秒)
    pub input_age_ns: Option<i64>,
    /// 最近输出延迟(纳秒)
    pub last_output_age_ns: Option<i64>,
    /// 最近一次 tick 延迟(纳秒)
    pub last_tick_latency_ns: Option<i64>,
    /// 最近一次错误(短串)
    pub last_error: Option<String>,
    /// 计数
    pub counters: Counters,
    /// 最近一次时间戳
    pub last_stamp: Option<Timestamp>,
}

impl NavDiagnostics {
    /// 新建(给定组件名)
    #[must_use]
    pub fn new(component: impl Into<String>) -> Self {
        Self {
            component: component.into(),
            ..Self::default()
        }
    }
}

impl DiagnosticsSnapshot for NavDiagnostics {
    fn component(&self) -> &'static str {
        "nuwax-nav-stack"
    }
    fn level(&self) -> DiagnosticLevel {
        if self.last_error.is_some() {
            DiagnosticLevel::Error
        } else if self.input_age_ns.unwrap_or(0) > 1_000_000_000 {
            // 1s 视为 stale
            DiagnosticLevel::Warn
        } else {
            DiagnosticLevel::Info
        }
    }
    fn summary(&self) -> String {
        format!(
            "nav state={} err={:?} tick_lat={:?}",
            self.state, self.last_error, self.last_tick_latency_ns
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counters_inc_and_get() {
        let mut c = Counters::default();
        c.inc("planned");
        c.inc_by("planned", 2);
        assert_eq!(c.get("planned"), 3);
        assert_eq!(c.get("nonexistent"), 0);
    }

    #[test]
    fn planner_diagnostics_level_warn_when_blocked() {
        let mut d = PlannerDiagnostics::new("nuwax-nav-astar");
        d.last_replan_reason = Some("blocked".to_string());
        d.counters.inc("blocked");
        assert_eq!(d.level(), DiagnosticLevel::Warn);
    }

    #[test]
    fn controller_diagnostics_level_warn_when_saturated() {
        let mut d = ControllerDiagnostics::new("nuwax-nav-path-follower");
        d.saturated = true;
        assert_eq!(d.level(), DiagnosticLevel::Warn);
    }

    #[test]
    fn nav_diagnostics_level_error_when_last_error_set() {
        let mut d = NavDiagnostics::new("nuwax-nav-stack");
        d.last_error = Some("boom".to_string());
        assert_eq!(d.level(), DiagnosticLevel::Error);
    }

    #[test]
    fn nav_diagnostics_summary_includes_state() {
        let mut d = NavDiagnostics::new("nuwax-nav-stack");
        d.state = "Moving".to_string();
        assert!(d.summary().contains("Moving"));
    }
}

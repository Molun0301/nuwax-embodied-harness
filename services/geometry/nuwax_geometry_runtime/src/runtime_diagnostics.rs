// Phase 6 (codex-round3 R3-4):schema_hash_version / mapping_ready / nav_ready
// / explore_ready / blockers / mapping_readiness / exploration_readiness /
// format_blockers / readiness_pair 是 platform RuntimeDiagnostics 协议预留
// (Phase 6 后续 Phase 6.1 readiness 协议升级时统一暴露)。
#![allow(dead_code)]

//! Runtime diagnostics —— 统一 health / readiness / blocker 输出。
//!
//! `server.rs` 的 `health_check` / `get_navigation_readiness` 当前内联散落;
//! Phase 2 把所有 diagnostics 输出收敛到本模块,Phase 4 (nav_runtime) 与
//! Phase 5 (explore_runtime) 直接挂接 mapping / nav / explore readiness
//! 字段,不再各自写 ad-hoc Status / 错误码。
//!
//! ## 字段语义
//! - `healthy` —— 进程级 liveness/serviceability;仅由进程生命周期与 LCM
//!   backend 决定,不与 mapping/nav/explore capability readiness 混用
//! - `phase` —— 进程生命周期 phase;默认 `running`,Phase 7 installer
//!   会扩展 `starting / draining / stopped`
//! - `mapping_ready` / `nav_ready` / `explore_ready` —— 板块级 readiness,
//!   `false` 表示该板块未启用 / 上游 crate 未实装 / 等待 typed frame
//! - `blockers` —— 字符串列表,health check / readiness 调用方可 grep
//!   关键字(LCM_BACKEND_UNAVAILABLE / MAPPING_NOT_READY / NAV_STACK_NOT_READY /
//!   EXPLORATION_NOT_READY)做 CI gate
//!
//! ## readiness 判定
//! - mapping:Phase 3 实装前恒 `false`
//! - nav:Phase 4 实装前恒 `false`
//! - explore:Phase 5 实装前恒 `false`
//! - 全部为 `false` 是 P0 阶段正确状态;升级期间不允许假装 ready

use std::sync::Arc;

use tokio::sync::RwLock;

use crate::lcm_runtime::GeometryRuntimeHandle;

/// 板块 readiness 字符串常量(grep gate 关键字)
pub const BLOCKER_LCM_BACKEND: &str = "LCM_BACKEND_UNAVAILABLE";
pub const BLOCKER_MAPPING_NOT_READY: &str = "MAPPING_NOT_READY";
pub const BLOCKER_NAV_STACK_NOT_READY: &str = "NAV_STACK_NOT_READY";
pub const BLOCKER_EXPLORATION_NOT_READY: &str = "EXPLORATION_NOT_READY";

/// 进程级 runtime 状态聚合
#[derive(Clone, Debug)]
pub struct RuntimeDiagnostics {
    /// 进程级 health;`false` = 非 running 生命周期 / LCM backend 不可用
    pub healthy: bool,
    /// 进程 phase(默认 `running`)
    pub phase: String,
    /// 已运行毫秒数
    pub uptime_ms: u64,
    /// RSS(单位 KB;`/proc/self/status` VmRSS 读取)
    pub rss_kb: u32,
    /// LCM backend canonical 名（`real-lcm` 或 `real-lcm-unavailable`）
    pub lcm_backend: String,
    /// schema 字节 layout 的 hash 版本号;Phase 2 当前 = 1(typed codec
    /// 收敛版);P1 切 MurmurHash64A 时升到 2;调用方可借此判断 wire
    /// format 是否会变更
    pub schema_hash_version: u32,
    /// mapping 板块 readiness(Phase 3 接 `nuwax-mapping` 后改 true)
    pub mapping_ready: bool,
    /// navigation 板块 readiness(Phase 4 接 `nuwax-nav-stack` 后改 true)
    pub nav_ready: bool,
    /// exploration 板块 readiness(Phase 5 接 `nuwax-exploration` 后改 true)
    pub explore_ready: bool,
    /// backend 与 capability readiness blocker 字符串列表。能力 blocker 不改变
    /// 进程 health，调用方应结合对应 readiness RPC 判断是否可接受板块请求。
    pub blockers: Vec<String>,
}

impl RuntimeDiagnostics {
    /// 从 `GeometryRuntimeHandle` 构造 diagnostics。
    ///
    /// `started_at_ns` 来自 `server.rs::ServiceState`,用于 uptime 计算;
    /// `mapping_ready / nav_ready / explore_ready` 在 Phase 3/4/5 接入前
    /// 一律为 `false` —— 升级期间允许 `blockers` 暴露。
    pub fn from_handle(handle: &GeometryRuntimeHandle, phase: &str, started_at_ns: u64) -> Self {
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let uptime_ms = now_ns.saturating_sub(started_at_ns) / 1_000_000;
        let rss_kb = read_rss_kb();
        let lcm_backend = handle.transport_backend_name.to_string();

        let mut blockers = Vec::new();
        if !handle.lcm_backend_available {
            blockers.push(BLOCKER_LCM_BACKEND.to_string());
        }
        if !handle.mapping_ready_poll() {
            blockers.push(BLOCKER_MAPPING_NOT_READY.to_string());
        }
        let nav_ready = handle.lcm_backend_available && handle.nav.execution_chain_is_live();
        if !nav_ready {
            blockers.push(BLOCKER_NAV_STACK_NOT_READY.to_string());
        }
        if !handle.explore_ready_poll() {
            blockers.push(BLOCKER_EXPLORATION_NOT_READY.to_string());
        }

        let healthy = handle.lcm_backend_available && phase == "running";
        // schema hash version:Phase 2 收敛到 1(fingerprint 仍 FNV-1a)
        let schema_hash_version = 1u32;

        Self {
            healthy,
            phase: phase.to_string(),
            uptime_ms,
            rss_kb,
            lcm_backend,
            schema_hash_version,
            mapping_ready: handle.mapping_ready_poll(),
            nav_ready,
            explore_ready: handle.explore_ready_poll(),
            blockers,
        }
    }
}

/// 从 `/proc/self/status` 读取 VmRSS(单位 KB);失败返回 0。
pub fn read_rss_kb() -> u32 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("VmRSS"))
                .and_then(|l| l.split_whitespace().nth(1))
                .and_then(|v| v.parse::<u32>().ok())
        })
        .unwrap_or(0)
}

/// 板块级 readiness 报告(给 `get_navigation_readiness` / 后续
/// `get_exploration_readiness` 复用)。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ComponentReadinessCode {
    /// The component is ready.
    Ready,
    /// Real LCM transport is unavailable.
    LcmBackendUnavailable,
    /// Guard, planner input, TF chain, or stack heartbeat is unavailable/stale.
    NavExecutionChainUnavailable,
    /// Mapping capability is unavailable.
    MappingNotReady,
    /// Exploration capability is unavailable.
    ExplorationNotReady,
}

/// Component readiness with a stable typed code plus compatibility text.
#[derive(Clone, Debug)]
pub struct ComponentReadiness {
    /// Whether the component can accept work.
    pub ready: bool,
    /// Human-readable compatibility projection.
    pub reason: String,
    /// Stable legacy blocker labels used by current health tooling.
    pub blockers: Vec<String>,
    /// Typed machine-readable result.
    pub code: ComponentReadinessCode,
}

impl ComponentReadiness {
    /// Build a typed not-ready result.
    pub fn not_ready_with_code(
        code: ComponentReadinessCode,
        reason: impl Into<String>,
        blockers: Vec<String>,
    ) -> Self {
        Self {
            ready: false,
            reason: reason.into(),
            blockers,
            code,
        }
    }

    /// Build a ready result.
    pub fn ready(reason: impl Into<String>) -> Self {
        Self {
            ready: true,
            reason: reason.into(),
            blockers: Vec::new(),
            code: ComponentReadinessCode::Ready,
        }
    }
}

/// Navigation 板块 readiness 报告(Phase 2 收敛;Phase 4 实装 nav-stack
/// 后改 `ready`)。
pub fn navigation_readiness(handle: &GeometryRuntimeHandle) -> ComponentReadiness {
    if !handle.lcm_backend_available {
        return ComponentReadiness::not_ready_with_code(
            ComponentReadinessCode::LcmBackendUnavailable,
            "LCM backend 不可用,nav_stack 无法发布 typed channel",
            vec![BLOCKER_LCM_BACKEND.to_string()],
        );
    }
    let missing = handle.nav.execution_chain_blockers();
    if !missing.is_empty() {
        return ComponentReadiness::not_ready_with_code(
            ComponentReadinessCode::NavExecutionChainUnavailable,
            format!("导航执行链未就绪: {}", missing.iter().map(|code| code.as_str()).collect::<Vec<_>>().join(", ")),
            vec![BLOCKER_NAV_STACK_NOT_READY.to_string()],
        );
    }
    ComponentReadiness::ready("nuwax-nav-stack 已就绪")
}

/// Mapping 板块 readiness 报告(Phase 2 baseline;Phase 3 实装后改 `ready`)。
pub fn mapping_readiness(handle: &GeometryRuntimeHandle) -> ComponentReadiness {
    if !handle.lcm_backend_available {
        return ComponentReadiness::not_ready_with_code(
            ComponentReadinessCode::LcmBackendUnavailable,
            "LCM backend 不可用,mapping adapter 无法接收 odom/pointcloud typed 输入",
            vec![BLOCKER_LCM_BACKEND.to_string()],
        );
    }
    if !handle.mapping_ready_poll() {
        return ComponentReadiness::not_ready_with_code(
            ComponentReadinessCode::MappingNotReady,
            "nuwax-mapping 未就绪:需要 ACTIVE mapping session 与已提交 occupancy revision",
            vec![BLOCKER_MAPPING_NOT_READY.to_string()],
        );
    }
    ComponentReadiness::ready("nuwax-mapping 已就绪")
}

/// Exploration 板块 readiness 报告(Phase 2 baseline;Phase 5 实装后改 `ready`)。
pub fn exploration_readiness(handle: &GeometryRuntimeHandle) -> ComponentReadiness {
    if !handle.lcm_backend_available {
        return ComponentReadiness::not_ready_with_code(
            ComponentReadinessCode::LcmBackendUnavailable,
            "LCM backend 不可用,explore controller 无法订阅 /explore/frontiers 输入",
            vec![BLOCKER_LCM_BACKEND.to_string()],
        );
    }
    if !handle.explore_ready_poll() {
        return ComponentReadiness::not_ready_with_code(
            ComponentReadinessCode::ExplorationNotReady,
            "nuwax-exploration controller 尚未实装 (Phase 5 待办)",
            vec![BLOCKER_EXPLORATION_NOT_READY.to_string()],
        );
    }
    ComponentReadiness::ready("nuwax-exploration 已就绪")
}

/// 把 diagnostics 序列化成可读字符串(给日志 / gRPC Status::failed 用)。
pub fn format_blockers(diag: &RuntimeDiagnostics) -> String {
    if diag.blockers.is_empty() {
        "no_blockers".to_string()
    } else {
        diag.blockers.join(",")
    }
}

/// 简单 helper:把 readiness 转成 (ready, reason) 元组,适配
/// `NavigationReadiness { ready, reason }` 现有 proto 字段(避免重写
/// proto 的同时改契约)。
pub fn readiness_pair(r: &ComponentReadiness) -> (bool, String) {
    (r.ready, r.reason.clone())
}

// ============================================================================
// ServiceState —— 给 server.rs 复用,集中存放 phase / started_at_ns
// ============================================================================

pub struct ServiceState {
    pub phase: String,
    pub message: String,
    pub started_at_ns: u64,
}

impl ServiceState {
    pub fn new_running(message: impl Into<String>) -> Self {
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        Self {
            phase: "running".into(),
            message: message.into(),
            started_at_ns: now_ns,
        }
    }
}

/// `Arc<RwLock<ServiceState>>` 类型别名(给 server.rs 复用)
pub type SharedServiceState = Arc<RwLock<ServiceState>>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_rss_kb_returns_u32() {
        // 不 crash + 返回 0 或正常数字
        let _ = read_rss_kb();
    }

    #[test]
    fn component_readiness_not_ready_has_blockers() {
        let r = ComponentReadiness::not_ready_with_code(
            ComponentReadinessCode::NavExecutionChainUnavailable,
            "test",
            vec!["X".to_string()],
        );
        assert!(!r.ready);
        assert_eq!(r.reason, "test");
        assert_eq!(r.blockers, vec!["X".to_string()]);
        assert_eq!(r.code, ComponentReadinessCode::NavExecutionChainUnavailable);
    }

    #[test]
    fn format_blockers_empty_returns_no_blockers() {
        let diag = RuntimeDiagnostics {
            healthy: true,
            phase: "running".into(),
            uptime_ms: 0,
            rss_kb: 0,
            lcm_backend: "real-lcm".into(),
            schema_hash_version: 1,
            mapping_ready: true,
            nav_ready: true,
            explore_ready: true,
            blockers: Vec::new(),
        };
        assert_eq!(format_blockers(&diag), "no_blockers");
    }

    #[tokio::test]
    async fn running_process_health_is_independent_of_capability_readiness() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");

        let diag = RuntimeDiagnostics::from_handle(&handle, "running", 0);

        assert!(
            diag.healthy,
            "a running process with a live real-LCM backend must stay healthy: {:?}",
            diag.blockers
        );
        assert!(!diag.mapping_ready);
        assert!(!diag.nav_ready);
        assert!(!diag.explore_ready);
        assert!(
            diag.blockers
                .iter()
                .any(|blocker| blocker == BLOCKER_MAPPING_NOT_READY),
            "capability readiness must remain observable"
        );
    }

    #[tokio::test]
    async fn navigation_readiness_requires_navigation_inputs_only() {
        use crate::lcm_runtime::NavState;
        use crate::nav_runtime::{
            MotionSinkObservation, PlannerInputObservation, TfChainObservation,
        };

        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle
            .nav
            .observe_stack_state(NavState {
                status: "IDLE".into(),
                message: "heartbeat".into(),
                goal_id: String::new(),
                distance_remaining: 0.0,
                timestamp_ns: 1,
            })
            .await;
        handle.set_cmd_vel_sink_ready(true);
        handle.nav.observe_motion_sink(MotionSinkObservation { generation: 1, ready: true })
            .expect("执行端观察");

        let missing = navigation_readiness(&handle);
        assert!(!missing.ready);
        assert_eq!(
            missing.code,
            ComponentReadinessCode::NavExecutionChainUnavailable
        );

        assert!(missing.reason.contains("TF_UNOBSERVED"));
        assert!(missing.reason.contains("PLANNER_INPUT_UNOBSERVED"));
        assert!(!missing.reason.contains("MOTION_SINK"), "执行端已就绪，不能伪报缺失");
        assert!(!missing.reason.contains("STACK_STATE"), "心跳已就绪，不能伪报缺失");

        handle
            .nav
            .observe_tf_chain(TfChainObservation {
                generation: 1,
                map_frame: "map".into(),
                odom_frame: "odom".into(),
                base_frame: "base_link".into(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("TF observation");
        handle
            .nav
            .observe_planner_input(PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "map".into(),
                pose_frame: "map".into(),
                ready: true,
            })
            .expect("planner observation");
        handle
            .nav
            .observe_motion_sink(MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("motion sink observation");

        handle.nav.observe_idle_motion_sink(false).expect("守卫拒绝后的空闲观察");
        let ready = navigation_readiness(&handle);
        assert!(ready.ready, "{}", ready.reason);
        assert_eq!(ready.code, ComponentReadinessCode::Ready);

        handle.nav.observe_tf_chain(TfChainObservation {
            generation: 1, map_frame: "map".into(), odom_frame: "odom".into(),
            base_frame: "base_link".into(), map_to_odom: false, odom_to_base: true,
        }).expect("撤销完整坐标链");
        let stale = navigation_readiness(&handle);
        assert!(!stale.ready);
        assert_eq!(stale.reason, "导航执行链未就绪: TF_INCOMPLETE");
        let detailed = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
        assert_eq!(detailed.blockers, vec![crate::nav_runtime::NavigationReadinessCode::TfIncomplete]);
        assert!(!detailed.ready);
    }
}

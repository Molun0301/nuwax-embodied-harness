//! 原生进程健康和导航输入就绪投影；能力依赖由各自真实拥有方判断。

use std::sync::Arc;

use tokio::sync::RwLock;

use crate::lcm_runtime::GeometryRuntimeHandle;

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

}

impl RuntimeDiagnostics {
    /// 从 `GeometryRuntimeHandle` 构造 diagnostics。
    ///
    /// `started_at_ns` 来自 `server.rs::ServiceState`,用于 uptime 计算;
    pub fn from_handle(handle: &GeometryRuntimeHandle, phase: &str, started_at_ns: u64) -> Self {
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let uptime_ms = now_ns.saturating_sub(started_at_ns) / 1_000_000;
        let rss_kb = read_rss_kb();
        let lcm_backend = handle.transport_backend_name.to_string();

        let healthy = handle.lcm_backend_available && phase == "running";

        Self {
            healthy,
            phase: phase.to_string(),
            uptime_ms,
            rss_kb,
            lcm_backend,
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

/// 当前服务真正消费的就绪状态和原因。
#[derive(Clone, Debug)]
pub struct ComponentReadiness {
    pub ready: bool,
    pub reason: String,
}

impl ComponentReadiness {
    pub fn not_ready(reason: impl Into<String>) -> Self {
        Self { ready: false, reason: reason.into() }
    }

    pub fn ready(reason: impl Into<String>) -> Self {
        Self { ready: true, reason: reason.into() }
    }
}

/// Navigation 板块 readiness 报告(Phase 2 收敛;Phase 4 实装 nav-stack
/// 后改 `ready`)。
pub fn navigation_readiness(handle: &GeometryRuntimeHandle) -> ComponentReadiness {
    if !handle.lcm_backend_available {
        return ComponentReadiness::not_ready(
            "LCM backend 不可用,nav_stack 无法发布 typed channel",
        );
    }
    let missing = handle.nav.execution_chain_blockers();
    if !missing.is_empty() {
        return ComponentReadiness::not_ready(
            format!("导航执行链未就绪: {}", missing.iter().map(|code| code.as_str()).collect::<Vec<_>>().join(", ")),
        );
    }
    ComponentReadiness::ready("nuwax-nav-stack 已就绪")
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

    #[tokio::test]
    async fn running_process_health_is_independent_of_capability_readiness() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");

        let diag = RuntimeDiagnostics::from_handle(&handle, "running", 0);

        assert!(diag.healthy, "进程健康与能力输入是否到达分别表达");
        assert!(!handle.mapping_ready_poll());
        assert!(!navigation_readiness(&handle).ready);
        assert!(!handle.explore_ready_poll());
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

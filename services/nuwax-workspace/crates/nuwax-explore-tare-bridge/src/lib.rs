//! TARE Planner C++ native bridge
//!
//! 仅包装 `result/bin/tare_planner` 原生进程,不重写算法。
//! 进程生命周期 / backoff / fatal 由 bridge-local `BridgeSupervisor` 薄层负责。
//! 不订阅/不解码 native `way_point` payload;typed schema `TareWaypoint` 与
//! `TareWaypoint -> FrontierGoal` 转换归属 `nuwax-exploration` facade。
//!
//! 完成定义参见 `docs/子任务/探索模块重写/探索模块重写方案.md` §5 / AC-6。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod bridge;
pub mod spec;
pub mod supervisor;
pub mod waypoint;

pub use bridge::{TareBridgeError, TareBridgeResult, TarePlannerBridge};
pub use spec::{
    build_specs, to_native_args, validate_config, TarePlannerBridgeConfig, TarePlannerTopics,
};
pub use supervisor::{
    BridgeDiagnostics, BridgeSupervisor, BridgeSupervisorState, DEFAULT_RESTART_POLICY,
};
pub use waypoint::{TareWaypoint, TareWaypointError};

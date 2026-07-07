//! Terrain Analysis C++ native bridge
//!
//! 仅包装 `result/bin/terrain_analysis` 原生进程,不重写算法。
//! 该 crate 是 **topic-only bridge**:仅配置 / 监管 native 进程发布 `terrain_map`
//! topic,**不**提供 Rust typed terrain-map point cloud output、不做 point cloud
//! codec / transport 解码、不是 terrain-map-ext、也不做地形扩展算法。
//! 需要 typed terrain-map-ext 数据时,使用 `nuwax-nav-terrain-map-ext` 或
//! `nuwax-explore-terrain-map-bridge` adapter。
//!
//! 进程生命周期 / backoff / fatal 由 bridge-local `BridgeSupervisor` 薄层负责,
//! 与 `nuwax-explore-tare-bridge` 保持同一状态机与测试模板(方案 §5.5)。
//!
//! 完成定义参见 `docs/子任务/探索模块重写/探索模块重写方案.md` §6 / AC-7。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod bridge;
pub mod config;
pub mod spec;
pub mod supervisor;

pub use bridge::{TerrainAnalysisBridge, TerrainBridgeError, TerrainBridgeResult};
pub use config::{cli_exclude, cli_name_override, TerrainAnalysisBridgeConfig};
pub use spec::{build_specs, to_native_args, validate_config, TerrainAnalysisTopics};
pub use supervisor::{
    BridgeDiagnostics, BridgeSupervisor, BridgeSupervisorState, DEFAULT_RESTART_POLICY,
};

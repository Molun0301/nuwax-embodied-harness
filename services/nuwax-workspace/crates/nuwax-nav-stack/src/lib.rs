// Copyright 2026 Nuwax.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! # nuwax-nav-stack
//!
//! 导航 stack 编排 crate,严格遵循方案 §6.8:
//!
//! 1. **不持有算法**:不实现 A*、bridge、visual servoing、movement manager 等逻辑,
//!    只组合上游 crate。
//! 2. **多 profile**:根据 `NavStackProfile` 选择 simple / CMU native / A* / exploration / MLS pipeline。
//! 3. **配置驱动**:`NavTopicConfig` + `NavFeatureFlags` 决定 module graph。
//! 4. **可观测**:每个 profile 输出 module graph,可 dry-run 打印,便于 e2e 验收。
//! 5. **可选可视化**:`--viewer` / rerun / foxglove 只作为可选模块,不影响核心规划。
//! 6. **replay/smoke**:`--replay <rosbag>` 与 fixture 驱动 stack smoke test。
//! 7. **依赖隔离**:默认 features 为空,所有 planner / bridge / visual / explore 都走可选 feature,
//!    避免 stack 默认强依赖探索或其他板块。
//! 8. **进程 owner(rework22 / P0-07)**:`StackRunner` 持有 `tokio::process::Child`
//!    句柄,负责把所有 `NativeBridge` 节点 spawn 成真实 OS 进程并提供 SIGTERM
//!    → 5s → SIGKILL 的优雅关闭链。算法实现仍归上游 crate,stack 不持有算法。
//!
//! ## 平台约束
//!
//! - edition = 2021、rust-version = 1.88
//! - 默认 features 不引入 DDS / ROS2 / CUDA
//! - 不在 build.rs 下载模型 / 二进制 / 访问网络
//!
//! ## 公开 API 速览
//!
//! ```text
//! pub enum NavStackProfile       { Simple2D, CmuNative, ReplanningAStar, ExplorationNative, Mls3D }
//! pub struct NavTopicConfig      { ... }
//! pub struct NavFeatureFlags     { ... }
//! pub struct NavStackBuilder     { profile, topics, features }
//! pub struct ModuleGraph         { nodes: Vec<ModuleNode> }
//! pub struct ModuleNode          { id, kind, feature_gate, binary, inputs, outputs, depends_on }
//! pub enum ModuleKind            { NativeBridge, RustAlgorithm, Visualizer, Recorder, GoalSource, Explorer }
//! pub struct StackRunner         { graph, handles }  // P0-07 process owner
//! ```
//!
//! ## 用法
//!
//! 1. 选择 profile 与 features,构建 `NavStackBuilder`。
//! 2. 调用 `builder.build()` 得到 `ModuleGraph`,可在不启动真实进程的情况下检查拓扑。
//! 3. 调用 `builder.dry_run()` 把 module graph 打印到 stdout。
//! 4. 调用 `builder.build_runner()` 得到 `StackRunner`(P0-07),
//!    `.spawn_all().await` 拉起所有 native bridge 子进程,
//!    `.shutdown().await` 优雅关闭。
//! 5. 调用 `nav_stack_bin::run()` 启动 CLI,支持 `--dry-run` / `--replay <path>` / `--list-features`。
//!
//! ## 错误类型
//!
//! `NavStackError` 使用 `thiserror`,避免上游 algorithm crate 的错误混入。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod bin_support;
pub mod builder;
pub mod dry_run;
pub mod error;
pub mod features;
pub mod graph;
pub mod profile;
pub mod profiles;
pub mod replay;
pub mod runner;
pub mod topics;

pub use builder::{NavStackBuilder, NavStackHandle};
pub use error::{NavStackError, NavStackResult};
pub use features::{NavFeatureFlags, NavStackPreset};
pub use graph::{ModuleGraph, ModuleKind, ModuleNode};
pub use profile::NavStackProfile;
pub use runner::StackRunner;
pub use topics::{NavTopicConfig, TopicName};

/// crate 版本,方便 dry-run 输出时报告。
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// crate 名称。
pub const CRATE_NAME: &str = env!("CARGO_PKG_NAME");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_metadata_is_set() {
        // `env!("CARGO_PKG_VERSION")` 与 `env!("CARGO_PKG_NAME")`
        // 在编译期填充,这里确认编译元数据已被 cargo 注入。
        assert!(VERSION.starts_with(|c: char| c.is_ascii_digit()));
        assert!(CRATE_NAME.starts_with("nuwax-"));
    }

    #[test]
    fn profile_count_matches_spec() {
        // 方案 §6.8 规定 5 个 profile
        assert_eq!(NavStackProfile::all().len(), 5);
    }

    #[test]
    fn empty_features_build_is_still_safe() {
        let builder = NavStackBuilder::new(NavStackProfile::Simple2D);
        let graph = builder
            .build()
            .expect("simple2d profile should build with no extra features");
        // simple2d profile 至少包含一个 Planner 节点
        assert!(!graph.nodes().is_empty());
    }

    #[test]
    fn dry_run_contains_profile_name() {
        let builder = NavStackBuilder::new(NavStackProfile::ReplanningAStar);
        let graph = builder.build().expect("astar profile build");
        let text = dry_run::render(&graph);
        // dry-run 文本使用 kebab-case profile 名,与方案一致
        assert!(text.contains("profile: replanning-astar"));
    }
}

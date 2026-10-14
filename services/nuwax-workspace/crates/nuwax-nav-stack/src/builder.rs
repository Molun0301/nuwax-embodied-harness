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

//! `NavStackBuilder` —— 编排 stack 的主要入口。
//!
//! 用法:
//!
//! ```no_run
//! use nuwax_nav_stack::{NavStackBuilder, NavStackProfile};
//!
//! let builder = NavStackBuilder::new(NavStackProfile::Simple2D);
//! let graph = builder.build().expect("build graph");
//! let text = nuwax_nav_stack::dry_run::render(&graph);
//! println!("{text}");
//! ```
//!
//! ## rework22 / P0-07:`StackRunner` 扩展
//!
//! 方案 §6.8 约束「stack 不持有算法」,rework22 边界澄清:
//!
//! - **算法实现**仍然在 upstream crate(`nuwax-nav-far-planner-bridge` 等);
//!   stack 不实现 A*、FAR、Local、PathFollower、PGO、Terrain 任何算法。
//! - **运行时管理**(`StackRunner`)归 stack:NativeBridge 由真实 OS 进程持有,
//!   Rust / Coordinator 等节点必须注册明确的 in-process owner。未注册节点会在
//!   任何副作用发生前令启动失败。
//!
//! 调用方通过 `builder.build_runner()` 拿到 `StackRunner`,然后
//! `runner.spawn_all().await` 拉起所有已配置并已注册的 graph 节点。

use crate::error::NavStackResult;
use crate::features::{NavFeatureFlags, NavStackPreset};
use crate::graph::ModuleGraph;
use crate::manifest::ProfileManifest;
use crate::profile::NavStackProfile;
use crate::profiles;
use crate::runner::StackRunner;
use crate::topics::NavTopicConfig;

/// Stack builder。
///
/// 通过 `with_topics` / `with_features` / `with_preset` 累积配置,
/// 然后调用 `build()` 得到 `ModuleGraph` 或 `build_runner()` 得到 process owner。
#[derive(Debug, Clone)]
pub struct NavStackBuilder {
    profile: NavStackProfile,
    topics: NavTopicConfig,
    features: NavFeatureFlags,
}

impl NavStackBuilder {
    /// 新建一个 builder,使用空 features / 默认 topics。
    pub fn new(profile: NavStackProfile) -> Self {
        Self {
            profile,
            topics: NavTopicConfig::default(),
            features: NavFeatureFlags::none(),
        }
    }

    /// 新建一个 builder,使用指定 topics。
    pub fn with_topics(profile: NavStackProfile, topics: NavTopicConfig) -> Self {
        Self {
            profile,
            topics,
            features: NavFeatureFlags::none(),
        }
    }

    /// 替换 topics。
    pub fn topics(mut self, topics: NavTopicConfig) -> Self {
        self.topics = topics;
        self
    }

    /// 替换 features。
    pub fn features(mut self, features: NavFeatureFlags) -> Self {
        self.features = features;
        self
    }

    /// 应用 preset(覆盖 features)。
    pub fn preset(mut self, preset: NavStackPreset) -> Self {
        self.features = preset.to_flags();
        self
    }

    /// profile 名。
    pub fn profile(&self) -> NavStackProfile {
        self.profile
    }

    /// 当前 features。
    pub fn current_features(&self) -> NavFeatureFlags {
        self.features
    }

    /// Return the explicit orchestration manifest for the current configuration.
    #[must_use]
    pub fn manifest(&self) -> ProfileManifest {
        ProfileManifest::for_profile(self.profile, &self.topics)
    }

    /// 构建 `ModuleGraph`。
    ///
    /// **不会启动任何进程**,也不会触发 native binary 下载,
    /// 仅生成纯数据 module graph 供 dry-run / 测试使用。
    pub fn build(&self) -> NavStackResult<ModuleGraph> {
        let graph = profiles::build_graph(self.profile, &self.topics, self.features);
        graph
            .validate()
            .map_err(crate::error::NavStackError::invalid)?;
        self.manifest()
            .validate_graph(&graph)
            .map_err(crate::error::NavStackError::invalid)?;
        Ok(graph)
    }

    /// 构建 `StackRunner`(process owner,rework22 / P0-07)。
    ///
    /// 与 `build()` 不同的关键点:
    /// - 内部先调用 `build()` 生成 `ModuleGraph`,并 `validate`。
    /// - 把 graph 包装成尚未启动的 `StackRunner`。
    /// - 调用方先注册 in-process owner,随后 `.spawn_all().await` 启动全部节点。
    ///
    /// 这把「拓扑生成」与「进程生命周期」显式分开,便于上层(CLI / integration
    /// test)在 `spawn_all` 之前对 argv / 拓扑做断言或修改。
    pub fn build_runner(&self) -> NavStackResult<StackRunner> {
        let graph = self.build()?;
        let mut runner = StackRunner::from_graph(graph);
        crate::runtime_owners::register_compiled_owners(&mut runner);
        Ok(runner)
    }

    /// 便捷 dry-run:返回渲染好的纯文本。
    pub fn dry_run(&self) -> NavStackResult<String> {
        let graph = self.build()?;
        Ok(crate::dry_run::render(&graph))
    }
}

/// Stack handle(`ModuleGraph` 的 thin wrapper,语义上表示"已构建完成")。
#[derive(Debug, Clone)]
pub struct NavStackHandle {
    graph: ModuleGraph,
}

impl NavStackHandle {
    /// 包装 graph。
    pub fn new(graph: ModuleGraph) -> Self {
        Self { graph }
    }

    /// 取出 graph。
    pub fn graph(&self) -> &ModuleGraph {
        &self.graph
    }

    /// 渲染 dry-run 文本。
    pub fn dry_run(&self) -> String {
        crate::dry_run::render(&self.graph)
    }
}

impl NavStackBuilder {
    /// 一次性 build + wrap 成 handle 的便捷方法。
    pub fn build_handle(self) -> NavStackResult<NavStackHandle> {
        let graph = self.build()?;
        Ok(NavStackHandle::new(graph))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builder_builds_all_profiles() {
        for p in NavStackProfile::all() {
            let g = NavStackBuilder::new(*p).build().expect("build");
            g.validate().expect("validate");
            assert_eq!(g.profile, p.as_str());
        }
    }

    #[test]
    fn builder_with_preset_overrides_features() {
        let b = NavStackBuilder::new(NavStackProfile::Simple2D).preset(NavStackPreset::All);
        let g = b.build().expect("build");
        // 至少包含 basic-follower
        assert!(g.features.enabled().iter().any(|f| f == "basic-follower"));
    }

    #[test]
    fn dry_run_returns_text() {
        let text = NavStackBuilder::new(NavStackProfile::Simple2D)
            .dry_run()
            .expect("dry_run");
        assert!(text.contains("profile: simple-2d"));
    }

    #[test]
    fn handle_from_builder() {
        let handle = NavStackBuilder::new(NavStackProfile::CmuNative)
            .build_handle()
            .expect("build_handle");
        assert_eq!(handle.graph().profile, "cmu-native");
        let text = handle.dry_run();
        assert!(text.contains("profile: cmu-native"));
    }
}

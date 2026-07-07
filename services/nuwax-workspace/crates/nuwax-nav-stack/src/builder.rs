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

use crate::error::NavStackResult;
use crate::features::{NavFeatureFlags, NavStackPreset};
use crate::graph::ModuleGraph;
use crate::profile::NavStackProfile;
use crate::profiles;
use crate::topics::NavTopicConfig;

/// Stack builder。
///
/// 通过 `with_topics` / `with_features` / `with_preset` 累积配置,
/// 然后调用 `build()` 得到 `ModuleGraph`。
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

    /// 构建 `ModuleGraph`。
    ///
    /// **不会启动任何进程**,也不会触发 native binary 下载,
    /// 仅生成纯数据 module graph 供 dry-run / 测试使用。
    pub fn build(&self) -> NavStackResult<ModuleGraph> {
        let graph = profiles::build_graph(self.profile, &self.topics, self.features);
        graph
            .validate()
            .map_err(crate::error::NavStackError::invalid)?;
        Ok(graph)
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

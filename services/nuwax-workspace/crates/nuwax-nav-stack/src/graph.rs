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

//! Module graph 数据结构。
//!
//! 每个 `NavStackProfile` 在 `build()` 时返回一份 `ModuleGraph`,
//! 描述节点 / 输入 / 输出 / bridge / native binary / feature gate 等元信息。
//! 这些信息用于:
//! - `--dry-run` 打印到 stdout,便于人工核对。
//! - `bin/nuwax-nav-stack` 启动前 sanity check。
//! - 后续 e2e / integration 测试 fixtures。
//!
//! **不持有运行时句柄**:graph 是纯数据,不持有进程、channel、async task。
//! 真正的 spawn / wait 由各 upstream crate 在收到 graph 后自行完成。

use std::collections::{BTreeMap, BTreeSet};

#[cfg(feature = "native-runner")]
use nuwax_exploration::NativeSpecSummary;
use serde::{Deserialize, Serialize};

/// 模块种类。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ModuleKind {
    /// C++ native bridge(由 `nuwax-nav-*-bridge` crate 包装)。
    NativeBridge,
    /// 纯 Rust 算法 crate。
    RustAlgorithm,
    /// 可视化节点(可选,不影响核心规划)。
    Visualizer,
    /// 目标源(click / agent / patrol / frontier 等)。
    GoalSource,
    /// 探索节点(仅当 `explore` Cargo feature 启用)。
    Explorer,
    /// 控制 / 决策流编排节点(movement manager、cmd_vel mux 等)。
    Coordinator,
}

/// 单个 module node 的元信息。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)] // Eq excluded: native_spec 包含 f64
pub struct ModuleNode {
    /// 节点 id,在 graph 内唯一。
    pub id: String,
    /// 显示名称,可与 id 不同,仅用于 dry-run 输出。
    pub display_name: String,
    /// 模块种类。
    pub kind: ModuleKind,
    /// 关联的 Cargo feature 名称(若缺省则表示无论如何都需要)。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feature_gate: Option<String>,
    /// native bridge 时对应的 binary 路径(相对 workspace 根或绝对路径,由配置覆盖)。
    ///
    /// 当 `native_spec` 存在时,`binary` 由 `spec.executable.display()` 派生;
    /// 否则为手填字符串(legacy 模式)。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub binary: Option<String>,
    /// 完整 typed native spec(R5-2 补全):保存 `NativeProgramSpec` 全部字段
    /// (executable / args / topics / env / cwd / restart) + 可选 `NativeBuildSpec`。
    /// `from_native_specs` 写入;`binary` 字段从此派生。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[cfg(feature = "native-runner")]
    pub native_spec: Option<NativeSpecSummary>,
    /// 输入 topic 列表。
    pub inputs: Vec<String>,
    /// 输出 topic 列表。
    pub outputs: Vec<String>,
    /// 拓扑依赖(直接前驱 id)。
    #[serde(default)]
    pub depends_on: Vec<String>,
    /// 备注 / 描述。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

impl ModuleNode {
    /// 构造 helper。
    pub fn new(id: impl Into<String>, display_name: impl Into<String>, kind: ModuleKind) -> Self {
        Self {
            id: id.into(),
            display_name: display_name.into(),
            kind,
            feature_gate: None,
            binary: None,
            #[cfg(feature = "native-runner")]
            native_spec: None,
            inputs: Vec::new(),
            outputs: Vec::new(),
            depends_on: Vec::new(),
            notes: None,
        }
    }

    /// 设置 feature gate。
    pub fn with_feature(mut self, feature: impl Into<String>) -> Self {
        self.feature_gate = Some(feature.into());
        self
    }

    /// 设置 native binary。
    pub fn with_binary(mut self, binary: impl Into<String>) -> Self {
        self.binary = Some(binary.into());
        self
    }

    /// 追加输入 topic。
    pub fn add_input(mut self, topic: impl Into<String>) -> Self {
        self.inputs.push(topic.into());
        self
    }

    /// 追加输出 topic。
    pub fn add_output(mut self, topic: impl Into<String>) -> Self {
        self.outputs.push(topic.into());
        self
    }

    /// 追加依赖。
    pub fn depends_on_node(mut self, node_id: impl Into<String>) -> Self {
        self.depends_on.push(node_id.into());
        self
    }

    /// 设置 notes。
    pub fn with_notes(mut self, notes: impl Into<String>) -> Self {
        self.notes = Some(notes.into());
        self
    }

    /// 从 typed native spec 构造 `ModuleNode`(Phase E6 stack wiring 整合点,R5-2 补全)。
    ///
    /// 完整保存 typed spec:
    /// - `binary` 自动取 `spec.executable` 的字符串表示(便于 `--dry-run` 打印)。
    /// - `native_spec` 写入完整 `NativeSpecSummary`(包含 executable/args/topics/env/cwd/restart/可选 build)供 BridgeSupervisor 在 spawn 时直接读;
    ///   不再走字符串 `with_binary` 路径(legacy fallback 在无 native-runner feature 时保留)。
    /// - `build` 为 `None` 时表示不自动 build(等同于 `NativeBuildPolicy::Never`)。
    ///
    /// 仅在 `native-runner` feature 启用时可用。
    #[cfg(feature = "native-runner")]
    pub fn from_native_specs(
        id: impl Into<String>,
        display_name: impl Into<String>,
        kind: ModuleKind,
        spec: nuwax_nav_native_runner::NativeProgramSpec,
        build: Option<nuwax_nav_native_runner::NativeBuildSpec>,
    ) -> Self {
        Self::from_native_spec_summary(
            id,
            display_name,
            kind,
            nuwax_exploration::NativeSpecSummary::new(spec, build),
        )
    }

    /// 从已聚合的 `NativeSpecSummary` 构造 `ModuleNode`(R5-2 推荐入口)。
    ///
    /// 完整保留 `executable` / `args` / `topics` / `env` / `cwd` / `restart` /
    /// `build` 全部字段到 `native_spec`;`binary` 派生自 `spec.executable`。
    #[cfg(feature = "native-runner")]
    pub fn from_native_spec_summary(
        id: impl Into<String>,
        display_name: impl Into<String>,
        kind: ModuleKind,
        summary: nuwax_exploration::NativeSpecSummary,
    ) -> Self {
        let mut node = Self::new(id, display_name, kind);
        node.binary = Some(summary.binary_display());
        node.native_spec = Some(summary);
        node
    }
}

/// Module graph:有向无环图,节点由 `ModuleNode` 组成。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)] // Eq excluded: ModuleNode.native_spec 包含 f64
pub struct ModuleGraph {
    /// 节点列表,顺序是拓扑序起点 -> 终点。
    pub nodes: Vec<ModuleNode>,
    /// 边集合(冗余存储,便于快速 lookup)。
    pub edges: Vec<GraphEdge>,
    /// 与本 graph 关联的 profile 名。
    pub profile: String,
    /// 关联的 feature flags(便于 dry-run 输出对齐)。
    pub features: NavFeatureFlagsMirror,
}

/// Edge:从 source -> target,通过某个 topic 连接。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GraphEdge {
    /// 起点 node id。
    pub from: String,
    /// 终点 node id。
    pub to: String,
    /// 通过的 topic 名(可与 `NavTopicConfig` 中的字段对应)。
    pub topic: String,
}

/// `NavFeatureFlags` 的 serde-friendly mirror(避免跨 crate 序列化耦合)。
///
/// 这里使用 `Vec<String>` 形式,语义与 `NavFeatureFlags::iter_names` 一致。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct NavFeatureFlagsMirror {
    /// 已启用的运行时 feature 名(kebab-case)。
    pub enabled: Vec<String>,
}

impl NavFeatureFlagsMirror {
    /// 从一组字符串构造。
    pub fn from_names<I, S>(names: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        Self {
            enabled: names.into_iter().map(Into::into).collect(),
        }
    }

    /// 取出 enabled 列表。
    pub fn enabled(&self) -> &[String] {
        &self.enabled
    }
}

impl ModuleGraph {
    /// 新建空 graph。
    pub fn new(profile: impl Into<String>) -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
            profile: profile.into(),
            features: NavFeatureFlagsMirror::default(),
        }
    }

    /// 设置 feature mirror。
    pub fn with_features(mut self, mirror: NavFeatureFlagsMirror) -> Self {
        self.features = mirror;
        self
    }

    /// 推入节点。
    pub fn push_node(&mut self, node: ModuleNode) -> &mut Self {
        self.nodes.push(node);
        self
    }

    /// 推入边。
    pub fn push_edge(&mut self, edge: GraphEdge) -> &mut Self {
        self.edges.push(edge);
        self
    }

    /// 取出所有节点。
    pub fn nodes(&self) -> &[ModuleNode] {
        &self.nodes
    }

    /// 取出所有边。
    pub fn edges(&self) -> &[GraphEdge] {
        &self.edges
    }

    /// 按 id 查找节点。
    pub fn find(&self, id: &str) -> Option<&ModuleNode> {
        self.nodes.iter().find(|n| n.id == id)
    }

    /// 校验拓扑:确保每个 `depends_on` 都指向真实存在的节点 id。
    pub fn validate(&self) -> Result<(), String> {
        let mut unique_ids = BTreeSet::new();
        for node in &self.nodes {
            if !unique_ids.insert(node.id.as_str()) {
                return Err(format!("duplicate node id `{}`", node.id));
            }
        }

        let ids: BTreeMap<&str, ()> = self.nodes.iter().map(|n| (n.id.as_str(), ())).collect();

        for n in &self.nodes {
            for dep in &n.depends_on {
                if !ids.contains_key(dep.as_str()) {
                    return Err(format!("node `{}` depends on missing node `{}`", n.id, dep));
                }
            }
        }

        for e in &self.edges {
            if !ids.contains_key(e.from.as_str()) {
                return Err(format!("edge from missing node `{}`", e.from));
            }
            if !ids.contains_key(e.to.as_str()) {
                return Err(format!("edge to missing node `{}`", e.to));
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builder_helpers_chain() {
        let node = ModuleNode::new("far", "FAR Planner", ModuleKind::NativeBridge)
            .with_feature("native-far")
            .with_binary("result/bin/far_planner_native")
            .add_input("terrain_map_ext")
            .add_output("way_point")
            .depends_on_node("terrain_map_ext");

        assert_eq!(node.id, "far");
        assert_eq!(node.kind, ModuleKind::NativeBridge);
        assert_eq!(node.feature_gate.as_deref(), Some("native-far"));
        assert_eq!(node.inputs, vec!["terrain_map_ext"]);
        assert_eq!(node.outputs, vec!["way_point"]);
        assert_eq!(node.depends_on, vec!["terrain_map_ext"]);
    }

    #[test]
    fn validate_detects_missing_dependency() {
        let mut g = ModuleGraph::new("test");
        g.push_node(ModuleNode::new("a", "A", ModuleKind::RustAlgorithm));
        g.push_node(ModuleNode::new("b", "B", ModuleKind::RustAlgorithm).depends_on_node("ghost"));
        let err = g.validate().unwrap_err();
        assert!(err.contains("ghost"));
    }

    #[test]
    fn validate_rejects_duplicate_node_ids() {
        let mut g = ModuleGraph::new("test");
        g.push_node(ModuleNode::new("same", "First", ModuleKind::RustAlgorithm));
        g.push_node(ModuleNode::new("same", "Second", ModuleKind::Coordinator));
        assert_eq!(g.validate().unwrap_err(), "duplicate node id `same`");
    }

    #[test]
    fn validate_passes_for_simple_chain() {
        let mut g = ModuleGraph::new("test");
        g.push_node(ModuleNode::new("a", "A", ModuleKind::GoalSource));
        g.push_node(ModuleNode::new("b", "B", ModuleKind::RustAlgorithm).depends_on_node("a"));
        g.push_edge(GraphEdge {
            from: "a".into(),
            to: "b".into(),
            topic: "clicked_point".into(),
        });
        assert!(g.validate().is_ok());
    }

    #[test]
    fn serde_roundtrip() {
        let mut g = ModuleGraph::new("simple-2d");
        g.push_node(ModuleNode::new(
            "planner",
            "Simple Planner",
            ModuleKind::RustAlgorithm,
        ));
        let s = serde_json::to_string(&g).unwrap();
        let back: ModuleGraph = serde_json::from_str(&s).unwrap();
        assert_eq!(g, back);
    }

    /// R5-2:`ModuleNode::from_native_spec_summary` 必须完整保存 typed spec,
    /// 不只是把 executable 填到 binary;`args` / `topics` / `restart` 全部在
    /// `native_spec` 字段中可访问。
    #[cfg(feature = "native-runner")]
    #[test]
    fn from_native_spec_summary_preserves_full_spec() {
        use std::collections::BTreeMap;
        use std::path::PathBuf;

        use nuwax_exploration::NativeSpecSummary;
        use nuwax_nav_native_runner::prelude::RestartPolicy;
        use nuwax_nav_native_runner::NativeProgramSpec;

        let mut topics = BTreeMap::new();
        topics.insert("scan".into(), "/scan".to_string());
        topics.insert("odom".into(), "/odom".to_string());

        let spec = NativeProgramSpec {
            name: "tare_planner".to_string(),
            executable: PathBuf::from("result/bin/tare_planner"),
            topics: topics.clone(),
            args: vec!["--config".into(), "/tmp/cfg.yaml".into()],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::OnFailure {
                max_restarts: 3,
                backoff_millis: 1_000,
            },
        };
        let summary = NativeSpecSummary::new(spec.clone(), None);

        let node = ModuleNode::from_native_spec_summary(
            "tare",
            "TARE Planner",
            ModuleKind::NativeBridge,
            summary.clone(),
        );

        // 1. binary 派生自 spec.executable,不再是手填字符串
        assert_eq!(node.binary.as_deref(), Some("result/bin/tare_planner"));
        // 2. native_spec 完整保存 spec 全部字段
        let saved = node
            .native_spec
            .as_ref()
            .expect("native_spec should be Some after from_native_spec_summary");
        assert_eq!(saved.spec.name, "tare_planner");
        assert_eq!(
            saved.spec.executable,
            PathBuf::from("result/bin/tare_planner")
        );
        assert_eq!(saved.spec.args.len(), 2);
        assert_eq!(saved.spec.args[0], "--config");
        assert_eq!(saved.spec.topics.get("scan"), Some(&"/scan".to_string()));
        assert_eq!(
            saved.spec.restart,
            RestartPolicy::OnFailure {
                max_restarts: 3,
                backoff_millis: 1_000
            }
        );
        // 3. roundtrip via JSON(native_spec 不是只读的 stub,可序列化可恢复)
        let s = serde_json::to_string(&node).unwrap();
        let back: ModuleNode = serde_json::from_str(&s).unwrap();
        assert_eq!(back.native_spec.as_ref().unwrap().spec, spec);
    }
}

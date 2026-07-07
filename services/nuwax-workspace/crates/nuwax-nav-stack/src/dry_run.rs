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

//! Dry-run 输出。
//!
//! 把 `ModuleGraph` 渲染为人类可读 + 工具可解析的纯文本格式。
//!
//! 输出格式(YAML-like,刻意不使用 serde_yaml 以减少依赖):
//!
//! ```text
//! profile: simple-2d
//! nodes:
//!   - id: planner
//!     display: Simple Planner
//!     kind: rust-algorithm
//!     feature: simple-planner
//!     inputs: [terrain_map_ext, clicked_point]
//!     outputs: [way_point, goal_path]
//!     depends_on: []
//!   - id: follower
//!     display: Basic Path Follower
//!     kind: rust-algorithm
//!     feature: basic-path-follower
//!     ...
//! edges:
//!   - from: planner
//!     to: follower
//!     topic: way_point
//! features:
//!   - basic-follower
//! ```

use std::fmt::Write;

use crate::graph::{ModuleGraph, ModuleKind};

/// 把 `ModuleGraph` 渲染为可读文本。
pub fn render(graph: &ModuleGraph) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "profile: {}", graph.profile);
    let _ = writeln!(out, "nodes:");
    for n in graph.nodes() {
        let _ = writeln!(out, "  - id: {}", n.id);
        let _ = writeln!(out, "    display: {}", n.display_name);
        let _ = writeln!(out, "    kind: {}", kind_str(n.kind));
        if let Some(f) = &n.feature_gate {
            let _ = writeln!(out, "    feature: {f}");
        }
        if let Some(b) = &n.binary {
            let _ = writeln!(out, "    binary: {b}");
        }
        let _ = writeln!(out, "    inputs: [{}]", n.inputs.join(", "));
        let _ = writeln!(out, "    outputs: [{}]", n.outputs.join(", "));
        let _ = writeln!(out, "    depends_on: [{}]", n.depends_on.join(", "));
        if let Some(notes) = &n.notes {
            let _ = writeln!(out, "    notes: {notes}");
        }
    }
    let _ = writeln!(out, "edges:");
    for e in graph.edges() {
        let _ = writeln!(
            out,
            "  - from: {}\n    to: {}\n    topic: {}",
            e.from, e.to, e.topic
        );
    }
    let _ = writeln!(out, "features:");
    for f in graph.features.enabled() {
        let _ = writeln!(out, "  - {f}");
    }
    out
}

/// `ModuleKind` 的 kebab-case 表示。
pub fn kind_str(k: ModuleKind) -> &'static str {
    match k {
        ModuleKind::NativeBridge => "native-bridge",
        ModuleKind::RustAlgorithm => "rust-algorithm",
        ModuleKind::Visualizer => "visualizer",
        ModuleKind::Recorder => "recorder",
        ModuleKind::GoalSource => "goal-source",
        ModuleKind::Explorer => "explorer",
        ModuleKind::Coordinator => "coordinator",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{ModuleGraph, ModuleNode};

    #[test]
    fn render_includes_all_sections() {
        let mut g = ModuleGraph::new("simple-2d");
        g.push_node(ModuleNode::new(
            "planner",
            "Simple Planner",
            ModuleKind::RustAlgorithm,
        ));
        let text = render(&g);
        assert!(text.contains("profile: simple-2d"));
        assert!(text.contains("nodes:"));
        assert!(text.contains("edges:"));
        assert!(text.contains("features:"));
        assert!(text.contains("id: planner"));
        assert!(text.contains("kind: rust-algorithm"));
    }
}

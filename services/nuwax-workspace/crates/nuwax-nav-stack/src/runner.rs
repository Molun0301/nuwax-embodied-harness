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

//! P0-07:`StackRunner` —— Stack 的 process owner。
//!
//! ## 设计边界(rework22)
//!
//! 方案 §6.8 约束「stack 不持有算法」,本模块严格遵守:
//!
//! - **不实现任何算法**(A* / FAR / Local / PathFollower / PGO / Terrain 全部走上游 crate / native binary)。
//! - **持有进程**:负责把 `ModuleGraph` 翻译为真实 OS 进程,把每个
//!   `NativeBridge` / `RustAlgorithm` 节点 spawn 成 child process。
//! - **可观测**:`running_nodes()` / `nodes_with_native_spec()` 让上层(CLI /
//!   integration test)能立即知道哪些节点真的被拉起。
//!
//! ## argv 构造规则
//!
//! - 若节点有 `native_spec`(typed wiring,cfg-gated `native-runner` feature 开启),
//!   argv = `[executable, --{port} {topic} 字典序, ...config_args]`,与各 bridge
//!   `bridge.argv()` 行为一致。
//! - 否则回退到 `binary` 字符串 + 空 argv(legacy path / dry-run 模式)。
//! - 完全无 `binary` 的节点(如 `RustAlgorithm` 或 `Coordinator`)不出现在本 runner 视野内,
//!   由 stack 上一层 `BridgeSupervisor` 或 in-process 调用管理。

use std::collections::HashMap;
use std::process::Stdio;
use std::time::Duration;

use tokio::process::{Child, Command};

use crate::error::{NavStackError, NavStackResult};
use crate::graph::{ModuleGraph, ModuleKind, ModuleNode};

/// Stack 的 process owner:持有 `ModuleGraph` + 真实子进程 `Child` 句柄。
pub struct StackRunner {
    graph: ModuleGraph,
    /// 已启动的 native bridge 节点 id -> child 句柄。
    ///
    /// 只持有 `ModuleKind::NativeBridge` 节点;`RustAlgorithm` /
    /// `Coordinator` 等不通过 OS 进程表达的节点不在此映射中。
    handles: HashMap<String, Child>,
}

impl StackRunner {
    /// 从 `ModuleGraph` 构造 runner,不启动任何进程。
    pub fn from_graph(graph: ModuleGraph) -> Self {
        Self {
            graph,
            handles: HashMap::new(),
        }
    }

    /// 当前持有的 module graph 引用。
    pub fn graph(&self) -> &ModuleGraph {
        &self.graph
    }

    /// 取出当前已启动(尚未 wait)的节点 id 列表。
    pub fn running_nodes(&self) -> Vec<String> {
        self.handles.keys().cloned().collect()
    }

    /// 列出 **应当** spawn 成子进程但当前还没拉的 native bridge 节点 id。
    ///
    /// 含 `ModuleKind::NativeBridge` 且 `binary` 非空 的节点;
    /// 已存在于 `handles` 的也会列出来(用于「判断预期 vs 实际」的对比)。
    pub fn native_bridge_ids(&self) -> Vec<String> {
        self.graph
            .nodes()
            .iter()
            .filter(|n| matches!(n.kind, ModuleKind::NativeBridge) && n.binary.is_some())
            .map(|n| n.id.clone())
            .collect()
    }

    /// 启动所有 `NativeBridge` 节点为真实子进程。
    ///
    /// 对每个有 `binary` 的 `NativeBridge` 节点:
    /// 1. 构造 argv(executable + topic args;若 typed native_spec 存在则按
    ///    native-runner 渲染顺序追加 config args,否则仅 binary);
    /// 2. `Command::new` + `args` + `kill_on_drop(true)`;
    /// 3. `spawn()` 拿到 `Child` 句柄存到 `handles`。
    ///
    /// 出错时立即返回 `SpawnFailed { node, source }`,已 spawn 的子进程
    /// 不会被自动回收(调用方可继续 `shutdown()` 收尾)。
    pub async fn spawn_all(&mut self) -> NavStackResult<()> {
        for node in self.graph.nodes() {
            if !matches!(node.kind, ModuleKind::NativeBridge) {
                continue;
            }
            let Some(argv) = argv_for(node) else {
                continue;
            };
            // 首项是 executable,后面全是 args
            let (program, args) = argv.split_first().expect("argv non-empty");
            let mut cmd = Command::new(program);
            cmd.args(args);
            cmd.stdout(Stdio::null());
            cmd.stderr(Stdio::null());
            cmd.kill_on_drop(true);
            let child = cmd.spawn().map_err(|source| NavStackError::SpawnFailed {
                node: node.id.clone(),
                source,
            })?;
            self.handles.insert(node.id.clone(), child);
        }
        Ok(())
    }

    /// 优雅关闭:SIGTERM(平台上是 `start_kill`,Windows 上等价 `TerminateProcess`)→
    /// 5 秒等待 → 仍未退出的 SIGKILL(`kill().await`)。
    ///
    /// 等待时间可通过参数覆盖,便于测试。
    pub async fn shutdown_with(&mut self, grace: Duration) -> NavStackResult<()> {
        // Phase 1:对每个 child 发 start_kill(best-effort,忽略单点错误)
        for (_id, child) in self.handles.iter_mut() {
            let _ = child.start_kill();
        }
        // Phase 2:等待 grace,期间 child 可能自然退出
        tokio::time::sleep(grace).await;
        // Phase 3:仍在跑的强制 kill
        for (_id, child) in self.handles.iter_mut() {
            if let Ok(None) = child.try_wait() {
                let _ = child.kill().await;
            }
        }
        Ok(())
    }

    /// 5 秒 grace 期 默认关闭。
    pub async fn shutdown(&mut self) -> NavStackResult<()> {
        self.shutdown_with(Duration::from_secs(5)).await
    }
}

/// 计算单个 `ModuleNode` 的 argv(executable 首项 + 后续 args)。
///
/// 返回 `None` 表示本节点不需要 spawn(无 binary / 非 NativeBridge)。
#[must_use]
pub fn argv_for(node: &ModuleNode) -> Option<Vec<String>> {
    if !matches!(node.kind, ModuleKind::NativeBridge) {
        return None;
    }

    // typed native_spec 路径(cfg-gated `native-runner` feature 启用):
    // argv = [executable, --port topic 字典序, ...config_args],与各 bridge
    // `bridge.argv()` 完全一致。
    #[cfg(feature = "native-runner")]
    {
        if let Some(summary) = &node.native_spec {
            let mut v = vec![summary.spec.executable.display().to_string()];
            for (port, topic) in &summary.spec.topics {
                v.push(format!("--{port}"));
                v.push(topic.clone());
            }
            v.extend(summary.spec.args.iter().cloned());
            return Some(v);
        }
    }

    // legacy 路径:仅有 binary 字符串,argv = [binary]
    let binary = node.binary.as_deref()?;
    Some(vec![binary.to_string()])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::ModuleGraph;

    #[test]
    fn argv_for_returns_none_when_no_binary() {
        let node = ModuleNode::new("rust_only", "Rust", ModuleKind::RustAlgorithm);
        assert!(argv_for(&node).is_none());
    }

    #[test]
    fn argv_for_returns_binary_for_legacy_native_bridge() {
        let node = ModuleNode::new("legacy", "Legacy", ModuleKind::NativeBridge)
            .with_binary("result/bin/far_planner_native");
        let argv = argv_for(&node).expect("argv present");
        assert_eq!(argv, vec!["result/bin/far_planner_native".to_string()]);
    }

    #[tokio::test]
    async fn runner_from_empty_graph_has_no_native_bridges() {
        let graph = ModuleGraph::new("simple-2d");
        let runner = StackRunner::from_graph(graph);
        assert!(runner.running_nodes().is_empty());
        assert!(runner.native_bridge_ids().is_empty());
    }
}
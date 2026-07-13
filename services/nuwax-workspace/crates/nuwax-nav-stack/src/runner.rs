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

//! Runtime ownership for a navigation stack graph.
//!
//! A graph node is not considered started merely because it appears in a
//! [`ModuleGraph`]. Native bridges are owned as child processes. Every other
//! node must have an explicit [`InProcessNodeFactory`] registration before
//! startup. This prevents Rust algorithms and coordinators from being silently
//! skipped while the CLI reports a successful stack launch.

use std::collections::{BTreeMap, HashMap};
use std::future::Future;
use std::path::PathBuf;
use std::pin::Pin;
use std::process::Stdio;
use std::sync::Arc;
use std::time::Duration;

use tokio::process::{Child, Command};

use crate::error::{NavStackError, NavStackResult};
use crate::graph::{ModuleGraph, ModuleKind, ModuleNode};

/// Boxed future returned by an asynchronously managed child process.
pub type ProcessFuture<'a> = Pin<Box<dyn Future<Output = std::io::Result<()>> + Send + 'a>>;

/// Fully rendered request passed to a [`ProcessFactory`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProcessLaunch {
    /// Graph node that owns the process.
    pub node_id: String,
    /// Executable path.
    pub program: PathBuf,
    /// Command-line arguments, excluding the executable.
    pub args: Vec<String>,
    /// Environment overrides from the typed native spec.
    pub env: BTreeMap<String, String>,
    /// Optional working directory from the typed native spec.
    pub cwd: Option<PathBuf>,
}

/// Process handle abstraction used by [`StackRunner`].
///
/// The abstraction keeps lifecycle tests deterministic without weakening the
/// production path, which uses [`tokio::process::Child`].
pub trait ManagedProcess: Send {
    /// Return `true` once the child has exited.
    fn has_exited(&mut self) -> std::io::Result<bool>;

    /// Request termination without blocking for process reaping.
    fn request_shutdown(&mut self) -> std::io::Result<()>;

    /// Force termination after the configured grace period.
    fn force_kill(&mut self) -> ProcessFuture<'_>;

    /// Wait for and reap the process.
    fn wait(&mut self) -> ProcessFuture<'_>;
}

/// Injectable child-process factory.
pub trait ProcessFactory: Send + Sync {
    /// Spawn one rendered native process.
    fn spawn(&self, launch: &ProcessLaunch) -> std::io::Result<Box<dyn ManagedProcess>>;
}

/// Production process factory backed by Tokio.
#[derive(Debug, Default)]
pub struct TokioProcessFactory;

impl ProcessFactory for TokioProcessFactory {
    fn spawn(&self, launch: &ProcessLaunch) -> std::io::Result<Box<dyn ManagedProcess>> {
        let mut command = Command::new(&launch.program);
        command.args(&launch.args);
        command.envs(&launch.env);
        if let Some(cwd) = &launch.cwd {
            command.current_dir(cwd);
        }
        command.stdin(Stdio::null());
        command.stdout(Stdio::null());
        command.stderr(Stdio::null());
        command.kill_on_drop(true);
        let child = command.spawn()?;
        Ok(Box::new(TokioManagedProcess { child }))
    }
}

struct TokioManagedProcess {
    child: Child,
}

impl ManagedProcess for TokioManagedProcess {
    fn has_exited(&mut self) -> std::io::Result<bool> {
        self.child.try_wait().map(|status| status.is_some())
    }

    fn request_shutdown(&mut self) -> std::io::Result<()> {
        #[cfg(unix)]
        if let Some(pid) = self.child.id() {
            let status = std::process::Command::new("/bin/kill")
                .arg("-TERM")
                .arg(pid.to_string())
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .status();
            if matches!(status, Ok(status) if status.success()) {
                return Ok(());
            }
        }

        self.child.start_kill()
    }

    fn force_kill(&mut self) -> ProcessFuture<'_> {
        Box::pin(async move { self.child.kill().await })
    }

    fn wait(&mut self) -> ProcessFuture<'_> {
        Box::pin(async move { self.child.wait().await.map(|_| ()) })
    }
}

/// Lifecycle owned by the stack for a non-native graph node.
pub trait InProcessNode: Send {
    /// Start the concrete node lifecycle.
    fn start(&mut self) -> NavStackResult<()>;

    /// Return whether the owned lifecycle is still running.
    fn is_running(&mut self) -> NavStackResult<bool>;

    /// Stop the lifecycle and release its resources.
    fn shutdown(&mut self) -> NavStackResult<()>;
}

/// Factory for one concrete in-process graph node owner.
pub trait InProcessNodeFactory: Send + Sync {
    /// Create the owner for the supplied graph node.
    fn create(&self, node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>>;
}

impl<F> InProcessNodeFactory for F
where
    F: Fn(&ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> + Send + Sync,
{
    fn create(&self, node: &ModuleNode) -> NavStackResult<Box<dyn InProcessNode>> {
        self(node)
    }
}

/// Stack runtime owner for native processes and registered in-process nodes.
pub struct StackRunner {
    graph: ModuleGraph,
    process_factory: Arc<dyn ProcessFactory>,
    in_process_factories: HashMap<String, Arc<dyn InProcessNodeFactory>>,
    processes: HashMap<String, Box<dyn ManagedProcess>>,
    in_process_nodes: HashMap<String, Box<dyn InProcessNode>>,
}

impl StackRunner {
    /// Construct a runner without starting any node.
    pub fn from_graph(graph: ModuleGraph) -> Self {
        Self {
            graph,
            process_factory: Arc::new(TokioProcessFactory),
            in_process_factories: HashMap::new(),
            processes: HashMap::new(),
            in_process_nodes: HashMap::new(),
        }
    }

    /// Replace the child-process factory before startup.
    pub fn set_process_factory<F>(&mut self, factory: F)
    where
        F: ProcessFactory + 'static,
    {
        self.process_factory = Arc::new(factory);
    }

    /// Register a concrete owner factory for one non-native graph node id.
    ///
    /// Registering the same id again replaces the previous factory. The
    /// factory is retained after shutdown, so a runner can be started again.
    pub fn register_in_process_factory<F>(&mut self, node_id: impl Into<String>, factory: F)
    where
        F: InProcessNodeFactory + 'static,
    {
        self.in_process_factories
            .insert(node_id.into(), Arc::new(factory));
    }

    /// Current module graph.
    pub fn graph(&self) -> &ModuleGraph {
        &self.graph
    }

    /// Sorted ids of all currently owned process and in-process lifecycles.
    pub fn running_nodes(&self) -> Vec<String> {
        let mut ids: Vec<String> = self
            .processes
            .keys()
            .chain(self.in_process_nodes.keys())
            .cloned()
            .collect();
        ids.sort();
        ids
    }

    /// Native bridge ids with a renderable executable.
    pub fn native_bridge_ids(&self) -> Vec<String> {
        self.graph
            .nodes()
            .iter()
            .filter(|node| launch_for(node).is_some())
            .map(|node| node.id.clone())
            .collect()
    }

    /// Nodes that cannot be started with the current registrations.
    pub fn unowned_nodes(&self) -> Vec<String> {
        self.graph
            .nodes()
            .iter()
            .filter_map(|node| match node.kind {
                ModuleKind::NativeBridge if launch_for(node).is_none() => {
                    Some(format!("{} (native bridge has no executable)", node.id))
                }
                ModuleKind::NativeBridge => None,
                _ if self.in_process_factories.contains_key(&node.id) => None,
                _ => Some(format!("{} ({:?})", node.id, node.kind)),
            })
            .collect()
    }

    /// Start every graph node after an all-or-nothing ownership preflight.
    ///
    /// Native bridges are spawned through the configured [`ProcessFactory`].
    /// Other node kinds must have an explicit in-process factory. If any node
    /// is unowned, no process or task is started. A later startup failure
    /// triggers cleanup of every node that was already started.
    pub async fn spawn_all(&mut self) -> NavStackResult<()> {
        if !self.processes.is_empty() || !self.in_process_nodes.is_empty() {
            return Err(NavStackError::AlreadyRunning);
        }

        let unowned = self.unowned_nodes();
        if !unowned.is_empty() {
            return Err(NavStackError::MissingRuntimeOwners { nodes: unowned });
        }

        let nodes = self.graph.nodes().to_vec();
        let result = self.spawn_preflighted(nodes).await;
        if result.is_err() {
            let _ = self.shutdown_with(Duration::ZERO).await;
        }
        result
    }

    async fn spawn_preflighted(&mut self, nodes: Vec<ModuleNode>) -> NavStackResult<()> {
        for node in nodes {
            if matches!(node.kind, ModuleKind::NativeBridge) {
                provision_native_node(&node).await.map_err(|reason| {
                    NavStackError::NodeStartFailed {
                        node: node.id.clone(),
                        reason,
                    }
                })?;
                let launch =
                    launch_for(&node).ok_or_else(|| NavStackError::MissingRuntimeOwners {
                        nodes: vec![format!("{} (native bridge has no executable)", node.id)],
                    })?;
                let process = self.process_factory.spawn(&launch).map_err(|source| {
                    NavStackError::SpawnFailed {
                        node: node.id.clone(),
                        source,
                    }
                })?;
                self.processes.insert(node.id, process);
                continue;
            }

            let factory = self
                .in_process_factories
                .get(&node.id)
                .expect("ownership preflight guarantees a factory");
            let mut owner =
                factory
                    .create(&node)
                    .map_err(|source| NavStackError::NodeStartFailed {
                        node: node.id.clone(),
                        reason: source.to_string(),
                    })?;
            owner
                .start()
                .map_err(|source| NavStackError::NodeStartFailed {
                    node: node.id.clone(),
                    reason: source.to_string(),
                })?;
            self.in_process_nodes.insert(node.id, owner);
        }
        Ok(())
    }

    /// Fail if any owned process or in-process node has stopped unexpectedly.
    pub fn ensure_healthy(&mut self) -> NavStackResult<()> {
        for (node, process) in &mut self.processes {
            if process
                .has_exited()
                .map_err(|source| NavStackError::ProcessInspectionFailed {
                    node: node.clone(),
                    source,
                })?
            {
                return Err(NavStackError::RuntimeNodeExited { node: node.clone() });
            }
        }
        for (node, owner) in &mut self.in_process_nodes {
            if !owner.is_running()? {
                return Err(NavStackError::RuntimeNodeExited { node: node.clone() });
            }
        }
        Ok(())
    }

    /// Stop all in-process owners, terminate all children, and reap them.
    pub async fn shutdown_with(&mut self, grace: Duration) -> NavStackResult<()> {
        let mut first_error = None;

        for (node, mut owner) in self.in_process_nodes.drain() {
            if let Err(source) = owner.shutdown() {
                first_error.get_or_insert_with(|| NavStackError::ShutdownFailed {
                    node,
                    reason: source.to_string(),
                });
            }
        }

        let mut joins = tokio::task::JoinSet::new();
        for (node, process) in self.processes.drain() {
            joins.spawn(async move {
                stop_process(process, grace)
                    .await
                    .map_err(|source| NavStackError::ShutdownFailed {
                        node,
                        reason: source.to_string(),
                    })
            });
        }

        while let Some(result) = joins.join_next().await {
            match result {
                Ok(Ok(())) => {}
                Ok(Err(error)) => {
                    first_error.get_or_insert(error);
                }
                Err(error) => {
                    first_error.get_or_insert_with(|| NavStackError::ShutdownFailed {
                        node: "process-supervisor".to_string(),
                        reason: error.to_string(),
                    });
                }
            }
        }

        match first_error {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    /// Stop every owned node with a five-second grace period.
    pub async fn shutdown(&mut self) -> NavStackResult<()> {
        self.shutdown_with(Duration::from_secs(5)).await
    }
}

#[cfg(feature = "native-runner")]
async fn provision_native_node(node: &ModuleNode) -> Result<(), String> {
    let Some(summary) = node.native_spec.as_ref() else {
        return Ok(());
    };
    summary
        .spec
        .validate()
        .map_err(|error| format!("invalid native program spec: {error}"))?;
    let Some(build) = summary.build.as_ref() else {
        return Ok(());
    };
    let diagnostics = Arc::new(std::sync::Mutex::new(
        nuwax_nav_native_runner::prelude::NativeProcessDiagnostics::new(node.id.clone()),
    ));
    nuwax_nav_native_runner::ensure_built(&summary.spec, build, &diagnostics)
        .await
        .map_err(|error| error.to_string())
}

#[cfg(not(feature = "native-runner"))]
async fn provision_native_node(_node: &ModuleNode) -> Result<(), String> {
    Ok(())
}

async fn stop_process(
    mut process: Box<dyn ManagedProcess>,
    grace: Duration,
) -> std::io::Result<()> {
    if process.has_exited()? {
        return Ok(());
    }

    if let Err(source) = process.request_shutdown() {
        if !process.has_exited()? {
            return Err(source);
        }
        return Ok(());
    }

    match tokio::time::timeout(grace, process.wait()).await {
        Ok(result) => result,
        Err(_) => {
            if process.has_exited()? {
                return Ok(());
            }
            if let Err(source) = process.force_kill().await {
                if !process.has_exited()? {
                    return Err(source);
                }
                return Ok(());
            }
            if process.has_exited()? {
                Ok(())
            } else {
                process.wait().await
            }
        }
    }
}

/// Render the executable, argv, environment, and cwd for a native graph node.
#[must_use]
pub fn launch_for(node: &ModuleNode) -> Option<ProcessLaunch> {
    if !matches!(node.kind, ModuleKind::NativeBridge) {
        return None;
    }

    #[cfg(feature = "native-runner")]
    if let Some(summary) = &node.native_spec {
        let mut args = Vec::with_capacity(summary.spec.topics.len() * 2 + summary.spec.args.len());
        for (port, topic) in &summary.spec.topics {
            args.push(format!("--{port}"));
            args.push(topic.clone());
        }
        args.extend(summary.spec.args.iter().cloned());
        return Some(ProcessLaunch {
            node_id: node.id.clone(),
            program: summary.spec.executable.clone(),
            args,
            env: summary.spec.env.clone(),
            cwd: summary.spec.cwd.clone(),
        });
    }

    Some(ProcessLaunch {
        node_id: node.id.clone(),
        program: PathBuf::from(node.binary.as_deref()?),
        args: Vec::new(),
        env: BTreeMap::new(),
        cwd: None,
    })
}

/// Render a native node as `[executable, args...]`.
#[must_use]
pub fn argv_for(node: &ModuleNode) -> Option<Vec<String>> {
    let launch = launch_for(node)?;
    let mut argv = Vec::with_capacity(launch.args.len() + 1);
    argv.push(launch.program.display().to_string());
    argv.extend(launch.args);
    Some(argv)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn argv_for_returns_none_for_non_native_node() {
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
    async fn runner_from_empty_graph_has_no_owned_nodes() {
        let graph = ModuleGraph::new("simple-2d");
        let runner = StackRunner::from_graph(graph);
        assert!(runner.running_nodes().is_empty());
        assert!(runner.native_bridge_ids().is_empty());
        assert!(runner.unowned_nodes().is_empty());
    }
}

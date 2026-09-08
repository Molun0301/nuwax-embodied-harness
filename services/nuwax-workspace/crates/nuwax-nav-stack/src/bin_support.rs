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

//! bin/nuwax-nav-stack 的支持函数。
//!
//! 把 CLI 参数解析、profile / preset 选择、输出格式与 main 拆分,
//! 便于测试与后续扩展。

use std::future::Future;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::builder::NavStackBuilder;
use crate::error::NavStackResult;
use crate::features::NavStackPreset;
use crate::profile::NavStackProfile;
use crate::runner::StackRunner;
use crate::topics::NavTopicConfig;

const INTERNAL_READY_FILE_ENV: &str = "NUWAX_NAV_STACK_INTERNAL_READY_FILE";
/// Exact token emitted by `--launcher-contract` for deployment compatibility checks.
pub const LAUNCHER_CONTRACT: &str = "nuwax-nav-stack-internal-ready-v1";

/// Return whether this invocation is the side-effect-free launcher contract probe.
#[must_use]
pub fn launcher_contract_requested(args: &[String]) -> bool {
    args == ["--launcher-contract"]
}

/// CLI 选项的内部表示。
#[derive(Debug, Clone)]
pub struct CliOptions {
    /// 选中的 profile。
    pub profile: NavStackProfile,
    /// 可选 preset(若为 None,features 全空)。
    pub preset: Option<NavStackPreset>,
    /// 是否 dry-run。
    pub dry_run: bool,
    /// 是否列出 feature。
    pub list_features: bool,
    /// 是否输出 JSON 形式的 module graph(供机器消费)。
    pub json: bool,
}

impl CliOptions {
    /// 解析命令行参数。
    ///
    /// 支持的 flag:
    /// - `--profile=<name>` / `--profile <name>`
    /// - `--preset=<name>`
    /// - `--dry-run`
    /// - `--list-features`
    /// - `--json`
    /// - `--help` / `-h`
    pub fn parse(args: &[String]) -> NavStackResult<Self> {
        let mut profile: Option<NavStackProfile> = None;
        let mut preset: Option<NavStackPreset> = None;
        let mut dry_run = false;
        let mut list_features = false;
        let mut json = false;

        let mut i = 0;
        while i < args.len() {
            let arg = &args[i];
            let (key, value) = split_kv(arg);

            match key.as_str() {
                "--profile" | "-p" => {
                    let v = require_value("--profile", key_value(args, &mut i, value))?;
                    profile = Some(NavStackProfile::from_cli_str(&v)?);
                }
                "--preset" => {
                    let v = require_value("--preset", key_value(args, &mut i, value))?;
                    preset = NavStackPreset::from_cli_str(&v);
                    if preset.is_none() {
                        return Err(crate::error::NavStackError::invalid(format!(
                            "unknown preset: `{v}` (allowed: none, simple, cmu, explore, mls, all)"
                        )));
                    }
                }
                "--dry-run" => {
                    dry_run = true;
                }
                "--list-features" => {
                    list_features = true;
                }
                "--json" => {
                    json = true;
                }
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                "--version" | "-V" => {
                    println!("{} {}", crate::CRATE_NAME, crate::VERSION);
                    std::process::exit(0);
                }
                other => {
                    return Err(crate::error::NavStackError::invalid(format!(
                        "unknown argument: `{other}`"
                    )));
                }
            }
            i += 1;
        }

        let profile = profile.unwrap_or(NavStackProfile::Simple2D);

        Ok(Self {
            profile,
            preset,
            dry_run,
            list_features,
            json,
        })
    }
}

fn split_kv(arg: &str) -> (String, Option<String>) {
    if let Some(idx) = arg.find('=') {
        let (k, v) = arg.split_at(idx);
        let v = &v[1..]; // skip '='
        (
            k.to_owned(),
            if v.is_empty() {
                None
            } else {
                Some(v.to_owned())
            },
        )
    } else {
        (arg.to_owned(), None)
    }
}

/// 取出 `--key <value>` 或 `--key=<value>` 形式的 value。
///
/// 对于 `--key=<value>` 直接返回;对于 `--key <value>`,需要消费 args 中的下一个元素。
fn key_value(
    args: &[String],
    i: &mut usize,
    inline: Option<String>,
) -> NavStackResult<Option<String>> {
    if let Some(v) = inline {
        return Ok(Some(v));
    }
    let next_idx = *i + 1;
    if next_idx >= args.len() {
        return Ok(None);
    }
    let next = &args[next_idx];
    if next.starts_with("--") {
        // 下一个 arg 是另一个 flag,不允许
        return Ok(None);
    }
    *i = next_idx;
    Ok(Some(next.clone()))
}

fn require_value(flag: &str, value: NavStackResult<Option<String>>) -> NavStackResult<String> {
    match value? {
        Some(v) => Ok(v),
        None => Err(crate::error::NavStackError::invalid(format!(
            "flag `{flag}` requires a value"
        ))),
    }
}

fn print_help() {
    println!(
        "nuwax-nav-stack {}\n\n\
         Navigation stack orchestration.\n\n\
         USAGE:\n    nuwax-nav-stack [OPTIONS]\n\n\
         OPTIONS:\n\
         -p, --profile <NAME>   select stack profile (simple-2d | cmu-native | replanning-astar | exploration-native | mls-3d)\n\
             --preset <NAME>    apply feature preset (none | simple | cmu | explore | mls | all)\n\
             --dry-run         render module graph and exit (no process spawn)\n\
             --list-features   list available features and exit\n\
             --launcher-contract  print the launcher readiness contract and exit\n\
             --json            emit module graph as JSON instead of YAML-like text\n\
         -h, --help             print this help\n\
         -V, --version          print version",
        crate::VERSION
    );
}

/// Execute the CLI until Ctrl-C requests stack shutdown.
///
/// Dry-run and feature-list modes remain synchronous. Every other invocation
/// builds a [`StackRunner`], starts it, monitors owned nodes, and reaps them
/// before returning.
pub fn run(opts: CliOptions) -> NavStackResult<()> {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    runtime.block_on(run_until_shutdown(opts, async {
        tokio::signal::ctrl_c().await.map_err(Into::into)
    }))
}

/// Execute CLI behavior with an injectable shutdown future.
///
/// The future is awaited only after every graph node has passed ownership
/// preflight and started successfully. This is the deterministic test seam for
/// lifecycle supervision; production passes `tokio::signal::ctrl_c()`.
pub async fn run_until_shutdown<F>(opts: CliOptions, shutdown: F) -> NavStackResult<()>
where
    F: Future<Output = NavStackResult<()>> + Send,
{
    if opts.list_features {
        list_features();
        return Ok(());
    }

    let mut builder =
        NavStackBuilder::new(opts.profile).topics(NavTopicConfig::geometry_runtime_defaults());
    if let Some(preset) = opts.preset {
        builder = builder.preset(preset);
    }

    if opts.dry_run {
        let text = builder.dry_run()?;
        println!("{text}");
        return Ok(());
    }

    if !builder.manifest().activation.is_production_guarded() {
        return Err(crate::error::NavStackError::invalid(format!(
            "profile `{}` is not connected to the production guard-backed execution path; use --dry-run for graph inspection",
            opts.profile.as_str()
        )));
    }

    let mut runner = builder.build_runner()?;
    if opts.json {
        let s = serde_json::to_string_pretty(runner.graph())
            .map_err(crate::error::NavStackError::Serde)?;
        println!("{s}");
    } else {
        println!("{}", crate::dry_run::render(runner.graph()));
    }

    supervise_runner_until(&mut runner, shutdown).await
}

/// Start a configured runner, monitor it until shutdown, then clean it up.
///
/// Callers that own concrete Rust modules register their
/// [`crate::runner::InProcessNodeFactory`] implementations on `runner` before
/// calling this function.
pub async fn supervise_runner_until<F>(runner: &mut StackRunner, shutdown: F) -> NavStackResult<()>
where
    F: Future<Output = NavStackResult<()>> + Send,
{
    runner.spawn_all().await?;

    let ready_marker = match RuntimeReadyMarker::publish_from_env() {
        Ok(marker) => marker,
        Err(error) => {
            let _ = runner.shutdown().await;
            return Err(error);
        }
    };

    let mut health = tokio::time::interval(Duration::from_millis(100));
    health.tick().await;
    tokio::pin!(shutdown);

    let supervision = loop {
        tokio::select! {
            result = &mut shutdown => break result,
            _ = health.tick() => {
                #[cfg(feature = "native-runner")]
                let health_result = runner.reconcile_health().await;
                #[cfg(not(feature = "native-runner"))]
                let health_result = runner.ensure_healthy();
                if let Err(error) = health_result {
                    break Err(error);
                }
            }
        }
    };
    let cleanup = runner.shutdown().await;
    drop(ready_marker);

    match (supervision, cleanup) {
        (Err(error), _) => Err(error),
        (Ok(()), Err(error)) => Err(error),
        (Ok(()), Ok(())) => Ok(()),
    }
}

struct RuntimeReadyMarker {
    path: Option<PathBuf>,
}

impl RuntimeReadyMarker {
    fn publish_from_env() -> NavStackResult<Self> {
        let Some(path) = std::env::var_os(INTERNAL_READY_FILE_ENV) else {
            return Ok(Self { path: None });
        };
        Self::publish(PathBuf::from(path))
    }

    fn publish(path: PathBuf) -> NavStackResult<Self> {
        if path.as_os_str().is_empty() {
            return Err(crate::error::NavStackError::invalid(format!(
                "{INTERNAL_READY_FILE_ENV} must not be empty"
            )));
        }
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            std::fs::create_dir_all(parent)?;
        }
        let temporary = ready_temporary_path(&path);
        std::fs::write(&temporary, format!("pid={}\nready=1\n", std::process::id()))?;
        std::fs::rename(&temporary, &path)?;
        Ok(Self { path: Some(path) })
    }
}

impl Drop for RuntimeReadyMarker {
    fn drop(&mut self) {
        if let Some(path) = self.path.as_ref() {
            let _ = std::fs::remove_file(path);
        }
    }
}

fn ready_temporary_path(path: &Path) -> PathBuf {
    let mut name = path.as_os_str().to_os_string();
    name.push(format!(".tmp.{}", std::process::id()));
    PathBuf::from(name)
}

/// 列出所有 Cargo feature,便于 `--list-features` 输出。
pub fn list_features() {
    let features: &[(&str, &str)] = &[
        ("default", "empty (no features enabled)"),
        (
            "foundation",
            "enable nuwax-nav-types-messages + nuwax-nav-base",
        ),
        ("astar", "enable nuwax-nav-astar"),
        ("simple-planner", "enable nuwax-nav-simple-planner"),
        ("terrain-map-ext", "enable nuwax-nav-terrain-map-ext"),
        ("native-far", "enable nuwax-nav-far-planner-bridge"),
        ("native-local", "enable nuwax-nav-local-planner-bridge"),
        (
            "native-path-follower",
            "enable nuwax-nav-path-follower-bridge",
        ),
        ("native-pgo", "enable nuwax-nav-pgo-bridge"),
        (
            "basic-path-follower",
            "enable nuwax-nav-basic-path-follower",
        ),
        ("movement-manager", "enable nuwax-nav-movement-manager"),
        ("patrolling", "enable nuwax-nav-patrolling"),
        ("visual-servoing", "enable nuwax-nav-visual-servoing"),
        ("visual-query", "enable nuwax-nav-visual-query"),
        ("bbox-tracker", "enable nuwax-nav-bbox-tracker"),
        ("click-goal", "enable nuwax-nav-click-goal-bridge"),
        (
            "mls",
            "enable nuwax-nav-mls-planner (3D MLS planner pipeline)",
        ),
        (
            "explore",
            "enable nuwax-explore-wavefront + tare + terrain bridges",
        ),
        (
            "full",
            "convenience preset covering most navigation features",
        ),
    ];
    println!("Available nuwax-nav-stack features:");
    let max = features.iter().map(|(n, _)| n.len()).max().unwrap_or(0);
    for (name, desc) in features {
        println!("  {name:<max$}  {desc}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{ModuleGraph, ModuleKind, ModuleNode};

    #[test]
    fn launcher_contract_probe_is_exact_and_side_effect_free() {
        assert_eq!(LAUNCHER_CONTRACT, "nuwax-nav-stack-internal-ready-v1");
        assert!(launcher_contract_requested(&["--launcher-contract".into()]));
        assert!(!launcher_contract_requested(&[
            "--launcher-contract".into(),
            "--profile=simple-2d".into(),
        ]));
    }

    #[test]
    fn parse_default_profile_is_simple2d() {
        let opts = CliOptions::parse(&[]).unwrap();
        assert_eq!(opts.profile, NavStackProfile::Simple2D);
        assert!(!opts.dry_run);
    }

    #[test]
    fn parse_profile_with_equals() {
        let opts = CliOptions::parse(&["--profile=cmu-native".into()]).unwrap();
        assert_eq!(opts.profile, NavStackProfile::CmuNative);
    }

    #[test]
    fn parse_dry_run() {
        let opts = CliOptions::parse(&["--dry-run".into()]).unwrap();
        assert!(opts.dry_run);
    }

    #[test]
    fn parse_unknown_flag_errors() {
        let err = CliOptions::parse(&["--nonsense".into()]).unwrap_err();
        assert!(err.to_string().contains("unknown argument"));
    }

    #[test]
    fn parse_missing_value_errors() {
        let err = CliOptions::parse(&["--profile".into()]).unwrap_err();
        assert!(err.to_string().contains("requires a value"));
    }

    #[test]
    fn parse_unknown_preset_errors() {
        let err = CliOptions::parse(&["--preset=banana".into()]).unwrap_err();
        assert!(err.to_string().contains("unknown preset"));
    }

    #[tokio::test]
    #[cfg(not(feature = "simple-2d"))]
    async fn non_dry_cli_rejects_unowned_profile_instead_of_reporting_success() {
        let opts = CliOptions::parse(&[]).expect("default CLI options");
        let error = run_until_shutdown(opts, async { Ok(()) })
            .await
            .expect_err("Simple2D has no registered runtime owners");
        assert!(matches!(
            error,
            crate::error::NavStackError::MissingRuntimeOwners { .. }
        ));
    }

    #[tokio::test]
    async fn supervision_owns_native_process_until_shutdown_future_resolves() {
        let mut graph = ModuleGraph::new("supervision-test");
        graph.push_node(
            ModuleNode::new("persistent", "Persistent", ModuleKind::NativeBridge)
                .with_binary("/usr/bin/yes"),
        );
        let mut runner = StackRunner::from_graph(graph);
        #[cfg(feature = "native-runner")]
        runner.register_native_readiness(
            "persistent",
            crate::runner::NativeLifecycleOptions::default(),
            |_generation, _identity| {
                Box::pin(async { Ok(()) }) as nuwax_nav_native_runner::TypedReadyFuture
            },
        );

        supervise_runner_until(&mut runner, async {
            tokio::time::sleep(Duration::from_millis(30)).await;
            Ok(())
        })
        .await
        .expect("supervise and reap persistent native process");
        assert!(runner.running_nodes().is_empty());
    }

    #[tokio::test]
    async fn production_rejects_profiles_without_guarded_execution_owner() {
        for profile in [
            NavStackProfile::CmuNative,
            NavStackProfile::ExplorationNative,
        ] {
            let opts = CliOptions {
                profile,
                preset: None,
                dry_run: false,
                list_features: false,
                json: false,
            };
            let error = run_until_shutdown(opts, async { Ok(()) })
                .await
                .expect_err("unqualified production profile");
            assert!(error.to_string().contains("guard-backed execution path"));
        }
    }

    #[test]
    fn cli_production_admission_comes_from_profile_manifest() {
        for profile in NavStackProfile::all() {
            let builder =
                NavStackBuilder::new(*profile).topics(NavTopicConfig::geometry_runtime_defaults());
            let admitted = builder.manifest().activation.is_production_guarded();
            assert_eq!(
                admitted,
                matches!(
                    profile,
                    NavStackProfile::Simple2D
                        | NavStackProfile::ReplanningAStar
                        | NavStackProfile::Mls3D
                ),
                "{profile}"
            );
        }
    }

    #[test]
    fn runtime_ready_marker_is_atomic_and_removed_on_drop() {
        let path = std::env::temp_dir().join(format!(
            "nuwax-nav-stack-ready-{}.marker",
            std::process::id()
        ));
        let _ = std::fs::remove_file(&path);
        let marker = RuntimeReadyMarker::publish(path.clone()).expect("publish ready marker");
        let content = std::fs::read_to_string(&path).expect("read ready marker");
        assert!(content.contains("ready=1"));
        drop(marker);
        assert!(!path.exists());
    }
}

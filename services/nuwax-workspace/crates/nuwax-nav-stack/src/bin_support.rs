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

use crate::builder::NavStackBuilder;
use crate::error::NavStackResult;
use crate::features::NavStackPreset;
use crate::profile::NavStackProfile;

/// CLI 选项的内部表示。
#[derive(Debug, Clone)]
pub struct CliOptions {
    /// 选中的 profile。
    pub profile: NavStackProfile,
    /// 可选 preset(若为 None,features 全空)。
    pub preset: Option<NavStackPreset>,
    /// 是否 dry-run。
    pub dry_run: bool,
    /// 可选 replay fixture 路径。
    pub replay: Option<String>,
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
    /// - `--replay <path>`
    /// - `--list-features`
    /// - `--json`
    /// - `--help` / `-h`
    pub fn parse(args: &[String]) -> NavStackResult<Self> {
        let mut profile: Option<NavStackProfile> = None;
        let mut preset: Option<NavStackPreset> = None;
        let mut dry_run = false;
        let mut replay: Option<String> = None;
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
                "--replay" => {
                    let v = require_value("--replay", key_value(args, &mut i, value))?;
                    replay = Some(v);
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
            replay,
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
             --replay <PATH>   run smoke replay using the given fixture manifest\n\
             --list-features   list available features and exit\n\
             --json            emit module graph as JSON instead of YAML-like text\n\
         -h, --help             print this help\n\
         -V, --version          print version",
        crate::VERSION
    );
}

/// 执行 CLI。
///
/// 设计为 `Result<(), NavStackError>` 而非 `process::exit`,便于 lib 调用方测试。
pub fn run(opts: CliOptions) -> NavStackResult<()> {
    if opts.list_features {
        list_features();
        return Ok(());
    }

    let mut builder = NavStackBuilder::new(opts.profile);
    if let Some(preset) = opts.preset {
        builder = builder.preset(preset);
    }

    if opts.dry_run {
        let text = builder.dry_run()?;
        println!("{text}");
        return Ok(());
    }

    // 完整 build 后选择输出形式
    let graph = builder.build()?;
    if opts.json {
        let s = serde_json::to_string_pretty(&graph).map_err(crate::error::NavStackError::Serde)?;
        println!("{s}");
    } else {
        println!("{}", crate::dry_run::render(&graph));
    }

    // 真实 replay:仅在显式指定 fixture 时执行,fixture 缺失立即报错
    if let Some(path) = opts.replay {
        crate::replay::run_smoke(std::path::PathBuf::from(path))?;
    }

    Ok(())
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
        ("record", "enable nuwax-nav-record"),
        (
            "mls",
            "enable nuwax-nav-mls-planner (3D MLS planner pipeline)",
        ),
        (
            "explore",
            "enable nuwax-explore-wavefront + tare + terrain bridges",
        ),
        ("replay", "enable --replay smoke entry (compile-time)"),
        ("viewer", "enable visualization modules (compile-time)"),
        ("record-profile", "alias: record"),
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
    fn parse_dry_run_and_replay() {
        let opts = CliOptions::parse(&[
            "--dry-run".into(),
            "--replay".into(),
            "/tmp/foo.json".into(),
        ])
        .unwrap();
        assert!(opts.dry_run);
        assert_eq!(opts.replay.as_deref(), Some("/tmp/foo.json"));
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
}

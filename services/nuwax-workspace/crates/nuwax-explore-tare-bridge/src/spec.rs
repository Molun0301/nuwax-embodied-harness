//! TARE bridge spec / topics / config / argv 渲染
//!
//! - 端口表:`registered_scan` / `odometry` / `way_point`
//! - 渲染职责分离:`to_native_args` 只渲染 config args;
//!   topics 唯一通过 `TarePlannerTopics::to_topic_map()` 填入 `NativeProgramSpec.topics`,
//!   由 native-runner 统一按字典序渲染 `--{port} {topic}`,避免 topic argv 重复。
//! - `build_specs` 必须 `validate_config`,非法配置 → `NavResult::Err`。
//! - 单一 restart policy 来源:`config.restart_policy` 写入 `NativeProgramSpec.restart`,
//!   `BridgeSupervisor` 只读取同一字段(方案 §5.5)。

use std::collections::BTreeMap;
use std::path::PathBuf;

use nuwax_nav_base::error::NavResult;
use nuwax_nav_base::native::RestartPolicy;
use nuwax_nav_native_runner::{NativeBuildPolicy, NativeBuildSpec, NativeProgramSpec};
use serde::{Deserialize, Serialize};

use crate::supervisor::DEFAULT_RESTART_POLICY;

/// TARE bridge 输入/输出 topic 名映射
///
/// 字段名对应 C++ native 二进制的 `--<port>` 参数;值是 topic 名。
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct TarePlannerTopics {
    /// 注册点云 topic (in)
    pub registered_scan: Option<String>,
    /// 里程计 topic (in)
    pub odometry: Option<String>,
    /// 下一 waypoint topic (out)
    pub way_point: Option<String>,
}

impl TarePlannerTopics {
    /// topics -> `NativeProgramSpec.topics` 映射(port 名固定)
    ///
    /// 仅保留 `Some` 的端口;`None` 不进入 argv。
    #[must_use]
    pub fn to_topic_map(&self) -> BTreeMap<String, String> {
        [
            ("registered_scan", &self.registered_scan),
            ("odometry", &self.odometry),
            ("way_point", &self.way_point),
        ]
        .iter()
        .filter_map(|(k, v)| v.as_ref().map(|t| ((*k).to_string(), t.clone())))
        .collect()
    }
}

/// TARE bridge 配置
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct TarePlannerBridgeConfig {
    /// 可执行文件路径
    pub executable: PathBuf,
    /// 工作目录
    pub cwd: Option<PathBuf>,
    /// 额外环境变量
    pub env: BTreeMap<String, String>,
    /// 构建命令(默认 `None`;不自动网络 build)
    pub build_command: Option<String>,
    /// 构建策略(默认 `Never`;`IfMissing` / `Always` 必须配合 `build_command`)
    pub build_policy: NativeBuildPolicy,
    /// 重启策略(默认 `DEFAULT_RESTART_POLICY`,方案 §5.5)
    pub restart_policy: RestartPolicy,
    /// exploration range(meters;默认 20.0)
    pub exploration_range: f64,
    /// update rate(Hz;默认 1.0)
    pub update_rate: f64,
    /// sensor range(meters;默认 20.0)
    pub sensor_range: f64,
}

impl Default for TarePlannerBridgeConfig {
    fn default() -> Self {
        Self {
            executable: PathBuf::from("result/bin/tare_planner"),
            cwd: None,
            env: BTreeMap::new(),
            build_command: None,
            build_policy: NativeBuildPolicy::Never,
            restart_policy: DEFAULT_RESTART_POLICY,
            exploration_range: 20.0,
            update_rate: 1.0,
            sensor_range: 20.0,
        }
    }
}

/// `TarePlannerBridgeConfig` 字段校验
///
/// - `executable` 非空
/// - `exploration_range` / `update_rate` / `sensor_range` 严格 > 0
/// - `build_policy != Never` 必须有 `build_command`
/// - 拒绝 `nix build github:` 等网络自动 build
///
/// # Errors
/// 字段非法时返回 `NavError::InvalidInput`。
pub fn validate_config(config: &TarePlannerBridgeConfig) -> NavResult<()> {
    if config.executable.as_os_str().is_empty() {
        return Err(nuwax_nav_base::error::NavError::InvalidInput {
            field: "executable",
            reason: "executable path is empty".to_string(),
        });
    }
    if !(config.exploration_range.is_finite() && config.exploration_range > 0.0) {
        return Err(nuwax_nav_base::error::NavError::InvalidInput {
            field: "exploration_range",
            reason: format!("must be finite and > 0 (got {})", config.exploration_range),
        });
    }
    if !(config.update_rate.is_finite() && config.update_rate > 0.0) {
        return Err(nuwax_nav_base::error::NavError::InvalidInput {
            field: "update_rate",
            reason: format!("must be finite and > 0 (got {})", config.update_rate),
        });
    }
    if !(config.sensor_range.is_finite() && config.sensor_range > 0.0) {
        return Err(nuwax_nav_base::error::NavError::InvalidInput {
            field: "sensor_range",
            reason: format!("must be finite and > 0 (got {})", config.sensor_range),
        });
    }
    if config.build_policy != NativeBuildPolicy::Never && config.build_command.is_none() {
        return Err(nuwax_nav_base::error::NavError::InvalidInput {
            field: "build_command",
            reason: "build_policy != Never requires build_command".to_string(),
        });
    }
    if let Some(cmd) = &config.build_command {
        if cmd.contains("nix build github:") {
            return Err(nuwax_nav_base::error::NavError::InvalidInput {
                field: "build_command",
                reason: format!("must not auto-run 'nix build github:' (got: {cmd})"),
            });
        }
    }
    Ok(())
}

/// 渲染 config 字段为 native CLI args(不含 topics;topics 走 `NativeProgramSpec.topics`)
///
/// 参数顺序固定:`--explorationRange <x> --updateRate <x> --sensorRange <x>`,便于
/// `spec_argv` 测试断言稳定。
#[must_use]
pub fn to_native_args(config: &TarePlannerBridgeConfig) -> Vec<String> {
    vec![
        "--explorationRange".to_string(),
        format!("{}", config.exploration_range),
        "--updateRate".to_string(),
        format!("{}", config.update_rate),
        "--sensorRange".to_string(),
        format!("{}", config.sensor_range),
    ]
}

/// 构造 `NativeProgramSpec` + `NativeBuildSpec`(供 `BridgeSupervisor` 包装)
/// - `NativeProgramSpec.topics = topics.to_topic_map()`
/// - `NativeProgramSpec.args = to_native_args(&config)`(**不**含 topics)
/// - `NativeProgramSpec.restart = config.restart_policy`
/// - `NativeBuildSpec.policy = config.build_policy`
///
/// # Errors
/// 字段非法返回 `NavError::InvalidInput`。
pub fn build_specs(
    config: TarePlannerBridgeConfig,
    topics: &TarePlannerTopics,
) -> NavResult<(NativeProgramSpec, NativeBuildSpec)> {
    validate_config(&config)?;
    let spec = NativeProgramSpec {
        name: "tare_planner".to_string(),
        executable: config.executable.clone(),
        topics: topics.to_topic_map(),
        args: to_native_args(&config),
        env: config.env.clone(),
        cwd: config.cwd.clone(),
        restart: config.restart_policy.clone(),
    };
    let build = NativeBuildSpec {
        build_command: config.build_command.clone(),
        policy: config.build_policy,
        cwd: config.cwd.clone(),
        env: config.env.clone(),
    };
    Ok((spec, build))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> TarePlannerBridgeConfig {
        TarePlannerBridgeConfig::default()
    }

    #[test]
    fn default_config_validates() {
        assert!(validate_config(&cfg()).is_ok());
    }

    #[test]
    fn topics_to_map_omits_none() {
        let t = TarePlannerTopics {
            registered_scan: Some("/registered_scan".into()),
            odometry: None,
            way_point: Some("/way_point".into()),
        };
        let m = t.to_topic_map();
        assert_eq!(m.len(), 2);
        assert_eq!(
            m.get("registered_scan").map(String::as_str),
            Some("/registered_scan")
        );
        assert_eq!(m.get("way_point").map(String::as_str), Some("/way_point"));
        assert!(!m.contains_key("odometry"));
    }

    #[test]
    fn to_native_args_is_config_only() {
        let args = to_native_args(&cfg());
        assert_eq!(
            args,
            vec![
                "--explorationRange".to_string(),
                "20".to_string(),
                "--updateRate".to_string(),
                "1".to_string(),
                "--sensorRange".to_string(),
                "20".to_string(),
            ]
        );
        // 不应包含任何 topic argv(避免 topic 重复)
        for a in &args {
            assert!(!a.contains("scan"), "topic leaked into args: {a}");
            assert!(!a.contains("odom"), "topic leaked into args: {a}");
            assert!(!a.contains("way"), "topic leaked into args: {a}");
        }
    }

    #[test]
    fn build_specs_embeds_topics_in_spec_not_args() {
        let t = TarePlannerTopics {
            registered_scan: Some("/scan".into()),
            odometry: Some("/odom".into()),
            way_point: Some("/way_point".into()),
        };
        let (spec, build) = build_specs(cfg(), &t).unwrap();
        assert_eq!(spec.topics.len(), 3);
        // topics 渲染由 native-runner 完成;spec.args 只含 config args
        for arg in &spec.args {
            assert!(!arg.starts_with("--registered"), "leaked: {arg}");
            assert!(!arg.starts_with("--odometry"), "leaked: {arg}");
            assert!(!arg.starts_with("--wayPoint"), "leaked: {arg}");
        }
        assert_eq!(build.policy, NativeBuildPolicy::Never);
        // restart policy 单一来源
        assert_eq!(spec.restart, DEFAULT_RESTART_POLICY);
    }

    #[test]
    fn validate_rejects_empty_executable() {
        let mut c = cfg();
        c.executable = PathBuf::new();
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::InvalidInput {
                field: "executable",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_zero_range() {
        let mut c = cfg();
        c.exploration_range = 0.0;
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::InvalidInput {
                field: "exploration_range",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_negative_update_rate() {
        let mut c = cfg();
        c.update_rate = -1.0;
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::InvalidInput {
                field: "update_rate",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_nan_sensor_range() {
        let mut c = cfg();
        c.sensor_range = f64::NAN;
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::InvalidInput {
                field: "sensor_range",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_build_policy_without_command() {
        let mut c = cfg();
        c.build_policy = NativeBuildPolicy::IfMissing;
        c.build_command = None;
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::InvalidInput {
                field: "build_command",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_nix_build_github() {
        let mut c = cfg();
        c.build_policy = NativeBuildPolicy::IfMissing;
        c.build_command = Some("nix build github:foo/bar".into());
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::InvalidInput {
                field: "build_command",
                ..
            }
        ));
    }
}

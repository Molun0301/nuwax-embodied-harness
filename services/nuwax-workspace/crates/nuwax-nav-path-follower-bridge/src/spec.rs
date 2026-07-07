//! Path Follower native bridge spec
//!
//! 端口表（方案 §4.4）:
//! - In (5): `path`、`odometry`、`speed`、`slow_down`、`safety_stop`
//! - Out (1): `cmd_vel`
//!
//! 配置字段单位说明:
//! - `max_yaw_rate`: 历史单位 deg/s；C++ 内部转换为 rad/s。**不在 Rust 层重复转换**。

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use nuwax_nav_base::native::RestartPolicy;
use nuwax_nav_native_runner::{ConfigValue, NativeCliMapping, NativeProgramSpec};
use serde::{Deserialize, Serialize};

/// Path Follower 重启策略。
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PathFollowerRestartPolicy {
    /// 永不自动重启
    #[default]
    Never,
    /// 出错后重启
    OnFailure,
}

impl From<PathFollowerRestartPolicy> for RestartPolicy {
    fn from(p: PathFollowerRestartPolicy) -> Self {
        match p {
            PathFollowerRestartPolicy::Never => Self::Never,
            PathFollowerRestartPolicy::OnFailure => Self::OnFailure {
                max_restarts: 3,
                backoff_millis: 1000,
            },
        }
    }
}

/// 车辆构型（对齐 Python `vehicle_config: Literal["omniDir", "standard"]`）。
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VehicleConfig {
    /// 全向机器人（omniDir）
    #[default]
    OmniDir,
    /// 标准差速
    Standard,
}

/// Path Follower 输入/输出 topic 名映射。
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct PathFollowerTopics {
    // In
    /// 跟踪路径 topic
    pub path: Option<String>,
    /// 当前 odom topic
    pub odometry: Option<String>,
    /// 速度标量 topic
    pub speed: Option<String>,
    /// 减速状态 topic
    pub slow_down: Option<String>,
    /// 安全停止 topic
    pub safety_stop: Option<String>,
    // Out
    /// 速度命令 topic
    pub cmd_vel: Option<String>,
}

impl PathFollowerTopics {
    /// 转为 `NativeProgramSpec.topics`。
    #[must_use]
    pub fn to_topic_map(&self) -> BTreeMap<String, String> {
        let all: &[(&str, &Option<String>)] = &[
            ("path", &self.path),
            ("odometry", &self.odometry),
            ("speed", &self.speed),
            ("slow_down", &self.slow_down),
            ("safety_stop", &self.safety_stop),
            ("cmd_vel", &self.cmd_vel),
        ];
        all.iter()
            .filter_map(|(k, v)| v.as_ref().map(|t| ((*k).to_string(), t.clone())))
            .collect()
    }
}

/// Path Follower 配置。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct PathFollowerBridgeConfig {
    // runner
    /// native 二进制相对路径
    pub executable: PathBuf,
    /// cwd
    pub cwd: Option<PathBuf>,
    /// env
    pub env: BTreeMap<String, String>,
    /// build command（默认 None；不得为 `nix build github:`）
    pub build_command: Option<String>,
    /// restart policy
    pub restart_policy: PathFollowerRestartPolicy,
    /// stdin config（默认 false）
    pub stdin_config: bool,

    // 控制器参数
    /// lookahead 距离 m（与 C++ `lookAheadDis` 对应）
    pub look_ahead_distance: f64,
    /// 最大速度 m/s（与 C++ `maxSpeed` 对应）
    pub max_speed: f64,
    /// 最大 yaw 速率 deg/s（与 C++ `maxYawRate` 对应）
    /// 单位约定: deg/s，**不在 Rust 层做 deg <-> rad 转换**
    pub max_yaw_rate: f64,

    /// 目标容差 m
    pub goal_tolerance: f64,

    /// 车辆构型
    pub vehicle_config: VehicleConfig,
    /// omni 方向 goal threshold m
    pub omni_dir_goal_threshold: f64,
    /// omni 方向 diff threshold rad
    pub omni_dir_diff_threshold: f64,

    /// autonomy_mode（None 不传）
    pub autonomy_mode: Option<bool>,
    /// autonomy speed m/s
    pub autonomy_speed: f64,
    /// joy_to_speed_delay s
    pub joy_to_speed_delay: f64,

    /// two_way_drive
    pub two_way_drive: bool,
    /// max_acceleration m/s^2
    pub max_acceleration: f64,
    /// use_fixed_accel_rate（True=每 tick maxAccel/100；False=maxAccel*dt）
    pub use_fixed_accel_rate: bool,
    /// slow_down_distance_threshold m
    pub slow_down_distance_threshold: f64,
}

impl Default for PathFollowerBridgeConfig {
    fn default() -> Self {
        Self {
            executable: PathBuf::from("result/bin/path_follower"),
            cwd: None,
            env: BTreeMap::new(),
            build_command: None,
            restart_policy: PathFollowerRestartPolicy::Never,
            stdin_config: false,

            look_ahead_distance: 0.5,
            max_speed: 1.0,
            max_yaw_rate: 45.0,

            goal_tolerance: 0.3,

            vehicle_config: VehicleConfig::OmniDir,
            omni_dir_goal_threshold: 1.0,
            omni_dir_diff_threshold: 1.5,

            autonomy_mode: None,
            autonomy_speed: 0.0,
            joy_to_speed_delay: 2.0,

            two_way_drive: true,
            max_acceleration: 1.0,
            use_fixed_accel_rate: true,
            slow_down_distance_threshold: 1.0,
        }
    }
}

/// 把 Path Follower 配置字段映射到 C++ CLI 参数名。
///
/// 完整覆盖 Python `PathFollowerConfig.cli_name_override`：
/// `lookAheadDis`/`maxSpeed`/`maxYawRate`/`autonomyMode`/`autonomySpeed`/`joyToSpeedDelay`/
/// `maxAccel`/`useFixedAccelRate`/`slowDwnDisThre`/`omniDirGoalThre`/`omniDirDiffThre`/`twoWayDrive`
#[must_use]
pub fn cli_name_override() -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    let pairs: &[(&str, &str)] = &[
        ("look_ahead_distance", "lookAheadDis"),
        ("max_speed", "maxSpeed"),
        ("max_yaw_rate", "maxYawRate"),
        ("autonomy_mode", "autonomyMode"),
        ("autonomy_speed", "autonomySpeed"),
        ("joy_to_speed_delay", "joyToSpeedDelay"),
        ("max_acceleration", "maxAccel"),
        ("use_fixed_accel_rate", "useFixedAccelRate"),
        ("slow_down_distance_threshold", "slowDwnDisThre"),
        ("omni_dir_goal_threshold", "omniDirGoalThre"),
        ("omni_dir_diff_threshold", "omniDirDiffThre"),
        ("two_way_drive", "twoWayDrive"),
    ];
    for (k, v) in pairs {
        m.insert((*k).to_string(), (*v).to_string());
    }
    m
}

/// 排除字段。
#[must_use]
pub fn cli_exclude() -> BTreeSet<String> {
    let mut s = BTreeSet::new();
    s.insert("executable".to_string());
    s.insert("cwd".to_string());
    s.insert("env".to_string());
    s.insert("build_command".to_string());
    s.insert("restart_policy".to_string());
    s.insert("stdin_config".to_string());
    s
}

/// 构造 CLI 映射（`NativeCliMapping`）。
#[must_use]
pub fn cli_mapping() -> NativeCliMapping {
    NativeCliMapping {
        exclude: cli_exclude(),
        name_override: cli_name_override(),
        stdin_config: false,
    }
}

/// typed config -> (name, Option<ConfigValue>) 对；按字母升序输出。
fn config_pairs(config: &PathFollowerBridgeConfig) -> Vec<(String, Option<ConfigValue>)> {
    let mut pairs: Vec<(String, Option<ConfigValue>)> = Vec::new();
    let push_f = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: f64| {
        p.push((n.to_string(), Some(v.into())));
    };
    let push_b = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: bool| {
        p.push((n.to_string(), Some(v.into())));
    };
    let push_ob = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: Option<bool>| {
        p.push((n.to_string(), v.map(ConfigValue::Bool)));
    };
    let push_s = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: VehicleConfig| {
        let s: String = match v {
            VehicleConfig::OmniDir => "omniDir".to_string(),
            VehicleConfig::Standard => "standard".to_string(),
        };
        p.push((n.to_string(), Some(s.into())));
    };

    push_f(
        &mut pairs,
        "look_ahead_distance",
        config.look_ahead_distance,
    );
    push_f(&mut pairs, "max_speed", config.max_speed);
    push_f(&mut pairs, "max_yaw_rate", config.max_yaw_rate);
    push_f(&mut pairs, "goal_tolerance", config.goal_tolerance);
    push_s(&mut pairs, "vehicle_config", config.vehicle_config);
    push_f(
        &mut pairs,
        "omni_dir_goal_threshold",
        config.omni_dir_goal_threshold,
    );
    push_f(
        &mut pairs,
        "omni_dir_diff_threshold",
        config.omni_dir_diff_threshold,
    );
    push_ob(&mut pairs, "autonomy_mode", config.autonomy_mode);
    push_f(&mut pairs, "autonomy_speed", config.autonomy_speed);
    push_f(&mut pairs, "joy_to_speed_delay", config.joy_to_speed_delay);
    push_b(&mut pairs, "two_way_drive", config.two_way_drive);
    push_f(&mut pairs, "max_acceleration", config.max_acceleration);
    push_b(
        &mut pairs,
        "use_fixed_accel_rate",
        config.use_fixed_accel_rate,
    );
    push_f(
        &mut pairs,
        "slow_down_distance_threshold",
        config.slow_down_distance_threshold,
    );
    pairs.sort_by(|a, b| a.0.cmp(&b.0));
    pairs
}

/// 把 typed config 渲染为 native CLI `--name value` 参数列表。
#[must_use]
pub fn to_native_args(config: &PathFollowerBridgeConfig) -> Vec<String> {
    let mapping = cli_mapping();
    let mut out = Vec::new();
    for (name, value) in config_pairs(config) {
        out.extend(mapping.render_arg(&name, value));
    }
    out
}

/// Path Follower bridge 顶层句柄。
pub struct PathFollowerBridge {
    /// 内部 NativeRunner
    pub runner: nuwax_nav_native_runner::NativeRunner,
    /// native spec 缓存
    pub spec: NativeProgramSpec,
    /// cwd
    pub cwd: Option<PathBuf>,
    /// env
    pub env: BTreeMap<String, String>,
    /// build command
    pub build_command: Option<String>,
}

impl PathFollowerBridge {
    /// 从配置 + topics 构造 bridge。
    #[must_use]
    pub fn new(config: PathFollowerBridgeConfig, topics: &PathFollowerTopics) -> Self {
        use crate::build_runner;
        let spec = build_spec(config.clone(), topics);
        let runner = build_runner(&config, topics);
        Self {
            runner,
            spec,
            cwd: config.cwd.clone(),
            env: config.env.clone(),
            build_command: config.build_command.clone(),
        }
    }

    /// 渲染后的完整 argv（首项为可执行路径）。
    #[must_use]
    pub fn argv(&self) -> Vec<String> {
        let mut v = vec![self.spec.executable.display().to_string()];
        for (port, topic) in &self.spec.topics {
            v.push(format!("--{port}"));
            v.push(topic.clone());
        }
        v.extend(self.spec.args.iter().cloned());
        v
    }
}

/// 构造 `NativeProgramSpec`。
#[must_use]
pub fn build_spec(
    config: PathFollowerBridgeConfig,
    topics: &PathFollowerTopics,
) -> NativeProgramSpec {
    NativeProgramSpec {
        name: "path_follower".to_string(),
        executable: config.executable.clone(),
        topics: topics.to_topic_map(),
        args: to_native_args(&config),
        env: config.env.clone(),
        cwd: config.cwd.clone(),
        restart: config.restart_policy.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_name_override_look_ahead_dis() {
        let m = cli_name_override();
        assert_eq!(
            m.get("look_ahead_distance").map(String::as_str),
            Some("lookAheadDis")
        );
        assert_eq!(m.get("max_speed").map(String::as_str), Some("maxSpeed"));
        assert_eq!(
            m.get("max_yaw_rate").map(String::as_str),
            Some("maxYawRate")
        );
        assert_eq!(
            m.get("slow_down_distance_threshold").map(String::as_str),
            Some("slowDwnDisThre")
        );
        assert_eq!(
            m.get("omni_dir_goal_threshold").map(String::as_str),
            Some("omniDirGoalThre")
        );
        assert_eq!(
            m.get("omni_dir_diff_threshold").map(String::as_str),
            Some("omniDirDiffThre")
        );
        assert_eq!(
            m.get("two_way_drive").map(String::as_str),
            Some("twoWayDrive")
        );
        assert_eq!(
            m.get("use_fixed_accel_rate").map(String::as_str),
            Some("useFixedAccelRate")
        );
    }

    #[test]
    fn to_native_args_renders_history_names() {
        let cfg = PathFollowerBridgeConfig::default();
        let args = to_native_args(&cfg);
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--lookAheadDis" && w[1] == "0.5"));
        assert!(args.windows(2).any(|w| w[0] == "--maxSpeed" && w[1] == "1"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--maxYawRate" && w[1] == "45"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--slowDwnDisThre" && w[1] == "1"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--omniDirGoalThre" && w[1] == "1"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--omniDirDiffThre" && w[1] == "1.5"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--twoWayDrive" && w[1] == "true"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--useFixedAccelRate" && w[1] == "true"));
        // 排除字段
        assert!(!args.iter().any(|a| a.starts_with("--executable")));
        assert!(!args.iter().any(|a| a.starts_with("--stdinConfig")));
        assert!(!args.iter().any(|a| a.starts_with("--buildCommand")));
    }

    #[test]
    fn to_native_args_skips_none() {
        let cfg = PathFollowerBridgeConfig::default();
        let args = to_native_args(&cfg);
        assert!(!args.iter().any(|a| a.starts_with("--autonomyMode")));
    }

    #[test]
    fn topics_to_topic_map_all_6_ports() {
        let topics = PathFollowerTopics {
            path: Some("/path".to_string()),
            odometry: Some("/odom".to_string()),
            speed: Some("/sp".to_string()),
            slow_down: Some("/sd".to_string()),
            safety_stop: Some("/ss".to_string()),
            cmd_vel: Some("/cv".to_string()),
        };
        let m = topics.to_topic_map();
        assert_eq!(m.len(), 6);
        assert_eq!(m.get("cmd_vel").map(String::as_str), Some("/cv"));
    }

    #[test]
    fn max_yaw_rate_units_are_deg_per_second() {
        let cfg = PathFollowerBridgeConfig::default();
        // 字段 doc 表明单位为 deg/s；不要在此处做单位转换。
        assert!((cfg.max_yaw_rate - 45.0).abs() < 1e-9);
    }

    #[test]
    fn bridge_argv_order_topics_then_config() {
        let cfg = PathFollowerBridgeConfig::default();
        let topics = PathFollowerTopics {
            path: Some("/path".to_string()),
            odometry: Some("/odom".to_string()),
            ..Default::default()
        };
        let bridge = PathFollowerBridge::new(cfg, &topics);
        let argv = bridge.argv();
        assert!(argv[0].ends_with("path_follower"));
        // 没有 cmd_vel topic → 字典序下一个是 --odometry
        assert_eq!(argv[1], "--odometry");
        assert_eq!(argv[2], "/odom");
        assert_eq!(argv[3], "--path");
        assert_eq!(argv[4], "/path");
    }
}

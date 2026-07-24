//! PGO native bridge spec
//!
//! 端口表（方案 §4.5）:
//! - In (2): `registered_scan`、`odometry`
//! - Out (3): `corrected_odometry`、`global_map`、`pgo_tf`
//!
//! 配置字段按方案 §4.5：world_frame、local_frame、loop_search_radius、
//! loop_submap_half_range、global_map_voxel_size、global_map_publish_rate。

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use nuwax_nav_base::native::RestartPolicy;
use nuwax_nav_native_runner::{ConfigValue, NativeCliMapping, NativeProgramSpec};
use serde::{Deserialize, Serialize};

/// PGO 重启策略。
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PgoRestartPolicy {
    /// 永不自动重启
    #[default]
    Never,
    /// 出错后重启
    OnFailure,
}

impl From<PgoRestartPolicy> for RestartPolicy {
    fn from(p: PgoRestartPolicy) -> Self {
        match p {
            PgoRestartPolicy::Never => Self::Never,
            PgoRestartPolicy::OnFailure => Self::OnFailure {
                max_restarts: 3,
                backoff_millis: 1000,
            },
        }
    }
}

/// PGO 输入/输出 topic 名映射。
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct PgoTopics {
    // In
    /// 注册点云 topic
    pub registered_scan: Option<String>,
    /// 里程计 topic
    pub odometry: Option<String>,
    // Out
    /// 优化后 odom topic
    pub corrected_odometry: Option<String>,
    /// 全局地图点云 topic
    pub global_map: Option<String>,
    /// map->odom 修正 topic
    pub pgo_tf: Option<String>,
}

impl PgoTopics {
    /// 转为 `NativeProgramSpec.topics`。
    #[must_use]
    pub fn to_topic_map(&self) -> BTreeMap<String, String> {
        let all: &[(&str, &Option<String>)] = &[
            ("registered_scan", &self.registered_scan),
            ("odometry", &self.odometry),
            ("corrected_odometry", &self.corrected_odometry),
            ("global_map", &self.global_map),
            ("pgo_tf", &self.pgo_tf),
        ];
        all.iter()
            .filter_map(|(k, v)| v.as_ref().map(|t| ((*k).to_string(), t.clone())))
            .collect()
    }
}

/// PGO 配置。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct PgoBridgeConfig {
    // runner
    /// native 二进制相对路径
    pub executable: PathBuf,
    /// cwd（默认 None；启动前若设置会校验存在）
    pub cwd: Option<PathBuf>,
    /// env
    pub env: BTreeMap<String, String>,
    /// build command（默认 None；不得为 `nix build github:`）
    pub build_command: Option<String>,
    /// restart policy
    pub restart_policy: PgoRestartPolicy,
    /// stdin config（默认 false）
    pub stdin_config: bool,

    // 帧
    /// 世界坐标系（默认 `map`）
    pub world_frame: String,
    /// 局部坐标系（默认 `odom`）
    pub local_frame: String,

    // 关键帧检测
    /// 关键帧角度变化阈值 deg
    pub key_pose_delta_deg: f64,
    /// 关键帧平移变化阈值 m
    pub key_pose_delta_trans: f64,

    // 回环
    /// 回环搜索半径 m
    pub loop_search_radius: f64,
    /// 回环时间阈值 s
    pub loop_time_thresh: f64,
    /// 回环分数阈值
    pub loop_score_thresh: f64,
    /// 回环 submap 半范围
    pub loop_submap_half_range: i64,
    /// submap 分辨率
    pub submap_resolution: f64,
    /// 最小回环检测间隔 s
    pub min_loop_detect_duration: f64,

    // 输入处理
    /// 是否把世界坐标系点云转换到 body 坐标系
    pub unregister_input: bool,

    // 全局地图发布
    /// 全局地图体素尺寸 m
    pub global_map_voxel_size: f64,
    /// 全局地图发布频率 Hz
    pub global_map_publish_rate: f64,

    /// debug 模式
    pub debug: bool,
}

impl Default for PgoBridgeConfig {
    fn default() -> Self {
        Self {
            executable: crate::native_executable(),
            cwd: None,
            env: BTreeMap::new(),
            build_command: None,
            restart_policy: PgoRestartPolicy::Never,
            stdin_config: false,

            world_frame: "map".to_string(),
            local_frame: "odom".to_string(),

            key_pose_delta_deg: 10.0,
            key_pose_delta_trans: 0.5,

            loop_search_radius: 1.0,
            loop_time_thresh: 60.0,
            loop_score_thresh: 0.15,
            loop_submap_half_range: 5,
            submap_resolution: 0.1,
            min_loop_detect_duration: 5.0,

            unregister_input: true,

            global_map_voxel_size: 0.1,
            global_map_publish_rate: 1.0,

            debug: false,
        }
    }
}

/// 排除字段（不出现在 CLI 参数中）。
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

/// 当前 PGO 不需要 cli_name_override（配置字段名与 C++ CLI 一致），
/// 但保留 hook 以便后续接入。
#[must_use]
pub fn cli_name_override() -> BTreeMap<String, String> {
    BTreeMap::new()
}

/// 构造 CLI 映射。
#[must_use]
pub fn cli_mapping() -> NativeCliMapping {
    NativeCliMapping {
        exclude: cli_exclude(),
        name_override: cli_name_override(),
        stdin_config: false,
    }
}

/// typed config -> (name, ConfigValue) 对；按字母升序输出。
fn config_pairs(config: &PgoBridgeConfig) -> Vec<(String, ConfigValue)> {
    let mut pairs: Vec<(String, ConfigValue)> = vec![
        ("world_frame".to_string(), config.world_frame.clone().into()),
        ("local_frame".to_string(), config.local_frame.clone().into()),
        (
            "key_pose_delta_deg".to_string(),
            config.key_pose_delta_deg.into(),
        ),
        (
            "key_pose_delta_trans".to_string(),
            config.key_pose_delta_trans.into(),
        ),
        (
            "loop_search_radius".to_string(),
            config.loop_search_radius.into(),
        ),
        (
            "loop_time_thresh".to_string(),
            config.loop_time_thresh.into(),
        ),
        (
            "loop_score_thresh".to_string(),
            config.loop_score_thresh.into(),
        ),
        (
            "loop_submap_half_range".to_string(),
            config.loop_submap_half_range.into(),
        ),
        (
            "submap_resolution".to_string(),
            config.submap_resolution.into(),
        ),
        (
            "min_loop_detect_duration".to_string(),
            config.min_loop_detect_duration.into(),
        ),
        (
            "unregister_input".to_string(),
            config.unregister_input.into(),
        ),
        (
            "global_map_voxel_size".to_string(),
            config.global_map_voxel_size.into(),
        ),
        (
            "global_map_publish_rate".to_string(),
            config.global_map_publish_rate.into(),
        ),
        ("debug".to_string(), config.debug.into()),
    ];
    pairs.sort_by(|a, b| a.0.cmp(&b.0));
    pairs
}

/// 把 typed config 渲染为 native CLI 参数列表。
#[must_use]
pub fn to_native_args(config: &PgoBridgeConfig) -> Vec<String> {
    let mapping = cli_mapping();
    let mut out = Vec::new();
    for (name, value) in config_pairs(config) {
        out.extend(mapping.render_arg(&name, Some(value)));
    }
    out
}

/// PGO bridge 顶层句柄。
pub struct PgoBridge {
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
    /// 当前最新的 pgo_tf 修正(由订阅循环填充)
    pub latest_pgo_tf: Option<super::tf_helper::PgoTfMessage>,
    /// 是否已发送 identity seed TF(与 Python `start()` 行为对齐)
    pub seeded: bool,
    /// TF tree(`handle_pgo_tf` 写入这里;订阅循环和单元测试都通过它读)
    pub tf_tree: std::sync::Arc<nuwax_tf::TfTree>,
    /// world frame 名(默认 "map";`handle_pgo_tf` 用作 transform parent)
    pub world_frame: String,
    /// local frame 名(默认 "odom";`handle_pgo_tf` 用作 transform child)
    pub local_frame: String,
}

impl PgoBridge {
    /// 从配置 + topics 构造 bridge。
    #[must_use]
    pub fn new(config: PgoBridgeConfig, topics: &PgoTopics) -> Self {
        use crate::build_runner;
        let spec = build_spec(config.clone(), topics);
        let runner = build_runner(&config, topics);
        Self {
            runner,
            spec,
            cwd: config.cwd.clone(),
            env: config.env.clone(),
            build_command: config.build_command.clone(),
            latest_pgo_tf: None,
            seeded: false,
            tf_tree: std::sync::Arc::new(nuwax_tf::TfTree::new()),
            world_frame: config.world_frame.clone(),
            local_frame: config.local_frame.clone(),
        }
    }

    /// 渲染后的完整 argv(首项为可执行路径)。
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

    /// 处理一条 PGO transform 订阅消息(bridge subscription handler)。
    ///
    /// 行为:
    /// 1. 用 [`super::tf_helper::apply_pgo_tf_to_tree`] 写入 `self.tf_tree`
    ///    (parent = `self.world_frame`,child = `self.local_frame`)
    /// 2. 更新 `self.latest_pgo_tf` 字段,便于上层在任意时刻读到最新修正
    /// 3. 不返回 Result(TF 写入是 in-process、无 IO 失败语义;如果未来要捕获错误,
    ///    可以把签名改为 `Result<(), TfError>`)
    ///
    /// 该方法不直接订阅 LCM/ROS2 socket;真实订阅由上层 transport 负责把
    /// 反序列化得到的 `PgoTfMessage` 喂进来。
    pub fn handle_pgo_tf(&mut self, msg: super::tf_helper::PgoTfMessage) {
        super::tf_helper::apply_pgo_tf_to_tree(
            &self.tf_tree,
            &msg,
            &self.world_frame,
            &self.local_frame,
        );
        self.latest_pgo_tf = Some(msg);
    }
}

/// 构造 `NativeProgramSpec`。
#[must_use]
pub fn build_spec(config: PgoBridgeConfig, topics: &PgoTopics) -> NativeProgramSpec {
    NativeProgramSpec {
        name: "pgo".to_string(),
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
    fn topics_to_topic_map_all_5_ports() {
        let topics = PgoTopics {
            registered_scan: Some("/scan".to_string()),
            odometry: Some("/odom".to_string()),
            corrected_odometry: Some("/co".to_string()),
            global_map: Some("/gm".to_string()),
            pgo_tf: Some("/tf".to_string()),
        };
        let m = topics.to_topic_map();
        assert_eq!(m.len(), 5);
        assert_eq!(m.get("pgo_tf").map(String::as_str), Some("/tf"));
        assert_eq!(m.get("odometry").map(String::as_str), Some("/odom"));
    }

    #[test]
    fn to_native_args_renders_required_fields() {
        let cfg = PgoBridgeConfig::default();
        let args = to_native_args(&cfg);
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--world_frame" && w[1] == "map"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--local_frame" && w[1] == "odom"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--loop_search_radius" && w[1] == "1"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--loop_submap_half_range" && w[1] == "5"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--global_map_voxel_size" && w[1] == "0.1"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--global_map_publish_rate" && w[1] == "1"));
        // 排除字段
        assert!(!args.iter().any(|a| a.starts_with("--executable")));
        assert!(!args.iter().any(|a| a.starts_with("--cwd")));
        assert!(!args.iter().any(|a| a.starts_with("--stdinConfig")));
    }

    #[test]
    fn default_world_frame_is_map_local_frame_is_odom() {
        let cfg = PgoBridgeConfig::default();
        assert_eq!(cfg.world_frame, "map");
        assert_eq!(cfg.local_frame, "odom");
    }

    #[test]
    fn bridge_argv_order_topics_then_config() {
        let cfg = PgoBridgeConfig::default();
        let topics = PgoTopics {
            odometry: Some("/odom".to_string()),
            pgo_tf: Some("/tf".to_string()),
            ..Default::default()
        };
        let bridge = PgoBridge::new(cfg, &topics);
        let argv = bridge.argv();
        assert!(argv[0].ends_with("pgo"));
        assert_eq!(argv[1], "--odometry");
        assert_eq!(argv[2], "/odom");
        assert_eq!(argv[3], "--pgo_tf");
        assert_eq!(argv[4], "/tf");
        let pos = argv
            .iter()
            .position(|a| a == "--world_frame")
            .expect("world_frame present");
        assert!(pos > 4);
    }
}

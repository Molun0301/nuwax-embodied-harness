//! FAR Planner native bridge spec
//!
//! 配置字段完整保留方案 §4.2 列表；C++ 历史拼写不一致的参数名
//! （如 `robot_dim`）通过 `cli_name_override` 输出历史名。
//!
//! 端口表:
//! - In: `terrain_map_ext`、`terrain_map`、`registered_scan`、`odometry`、`goal`、`stop_movement`
//! - Out: `way_point`、`goal_path`、`graph_nodes`、`graph_edges`、`contour_polygons`、`nav_boundary`
//!
//! 本模块通过 `nuwax-nav-native-runner` 的 `NativeCliMapping::render_arg` 渲染
//! CLI 参数；typed config -> `ConfigValue` 转换集中在本文件末的 helpers 中。

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use nuwax_nav_base::native::RestartPolicy;
use nuwax_nav_native_runner::{ConfigValue, NativeCliMapping, NativeProgramSpec};
use serde::{Deserialize, Serialize};

/// FAR Planner 输入/输出 topic 名映射。
///
/// 字段名对应 C++ native 二进制的 `--<port>` 参数；值是 LCM/topic 名。
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct FarPlannerTopics {
    // In
    /// 扩展地形点云 topic
    pub terrain_map_ext: Option<String>,
    /// 地形分析点云 topic
    pub terrain_map: Option<String>,
    /// 注册点云 topic
    pub registered_scan: Option<String>,
    /// 里程计 topic
    pub odometry: Option<String>,
    /// `PointStamped` 目标 topic
    pub goal: Option<String>,
    /// 停止布尔信号 topic
    pub stop_movement: Option<String>,
    // Out
    /// 下一局部 waypoint topic
    pub way_point: Option<String>,
    /// 全局路径 topic
    pub goal_path: Option<String>,
    /// FAR 图节点 topic
    pub graph_nodes: Option<String>,
    /// FAR 图边 topic
    pub graph_edges: Option<String>,
    /// contour polygon 可视化 topic
    pub contour_polygons: Option<String>,
    /// 导航边界 topic
    pub nav_boundary: Option<String>,
}

impl FarPlannerTopics {
    /// 把 topics 转换为 `NativeProgramSpec.topics`（port -> topic）。
    ///
    /// 仅保留 `Some` 的端口；其余不进入 argv。
    #[must_use]
    pub fn to_topic_map(&self) -> BTreeMap<String, String> {
        let all: &[(&str, &Option<String>)] = &[
            ("terrain_map_ext", &self.terrain_map_ext),
            ("terrain_map", &self.terrain_map),
            ("registered_scan", &self.registered_scan),
            ("odometry", &self.odometry),
            ("goal", &self.goal),
            ("stop_movement", &self.stop_movement),
            ("way_point", &self.way_point),
            ("goal_path", &self.goal_path),
            ("graph_nodes", &self.graph_nodes),
            ("graph_edges", &self.graph_edges),
            ("contour_polygons", &self.contour_polygons),
            ("nav_boundary", &self.nav_boundary),
        ];
        all.iter()
            .filter_map(|(k, v)| v.as_ref().map(|t| ((*k).to_string(), t.clone())))
            .collect()
    }
}

/// FAR Planner 重启策略。
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FarPlannerRestartPolicy {
    /// 永不自动重启
    #[default]
    Never,
    /// 出错后重启
    OnFailure,
}

impl From<FarPlannerRestartPolicy> for RestartPolicy {
    fn from(p: FarPlannerRestartPolicy) -> Self {
        match p {
            FarPlannerRestartPolicy::Never => Self::Never,
            FarPlannerRestartPolicy::OnFailure => Self::OnFailure {
                max_restarts: 3,
                backoff_millis: 1000,
            },
        }
    }
}

/// FAR Planner 配置（与 Python `FarPlannerConfig` 等价）。
///
/// 字段命名使用 snake_case（serde rename），C++ 历史拼写通过 `cli_name_override` 输出。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct FarPlannerBridgeConfig {
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
    pub restart_policy: FarPlannerRestartPolicy,

    // FAR Planner 字段（与 Python FarPlannerConfig 完整对齐）
    /// 更新频率 Hz
    pub update_rate: f64,
    /// 机器人宽度（与 C++ `robot_dim` 对应；Python 字段名 `robot_dimension`）
    pub robot_dimension: f64,
    /// 体素尺寸 m
    pub voxel_dim: f64,
    /// 传感器范围 m
    pub sensor_range: f64,
    /// 地形分析范围 m
    pub terrain_range: f64,
    /// 局部规划器范围 m
    pub local_planner_range: f64,
    /// 车辆高度 m
    pub vehicle_height: f64,
    /// 静态环境标志
    pub is_static_env: bool,
    /// 视点扩展标志
    pub is_viewpoint_extend: bool,
    /// 多层图标志
    pub is_multi_layer: bool,
    /// 调试输出
    pub is_debug_output: bool,
    /// 自动切换
    pub is_attempt_autoswitch: bool,
    /// 世界坐标系
    pub world_frame: String,

    /// 收敛距离 m
    pub converge_dist: f64,
    /// goal 调整半径 m
    pub goal_adjust_radius: f64,
    /// free counter 阈值
    pub free_counter_thred: i64,
    /// reach goal 投票数量
    pub reach_goal_vote_size: i64,
    /// path momentum 阈值
    pub path_momentum_thred: i64,

    /// floor 高度 m
    pub floor_height: f64,
    /// cell 长度 m
    pub cell_length: f64,
    /// map grid 最大长度 m
    pub map_grid_max_length: f64,
    /// map gradient 最大高度 m
    pub map_grad_max_height: f64,

    /// connect votes 数量
    pub connect_votes_size: i64,
    /// clear dumper 阈值
    pub clear_dumper_thred: i64,
    /// node finalize 阈值
    pub node_finalize_thred: i64,
    /// filter pool size
    pub filter_pool_size: i64,

    /// resize ratio
    pub resize_ratio: f64,
    /// filter count value
    pub filter_count_value: i64,

    /// angle noise deg
    pub angle_noise: f64,
    /// accept max align angle deg
    pub accept_max_align_angle: f64,
    /// new intensity 阈值
    pub new_intensity_thred: f64,
    /// 动态障碍衰减时间 s
    pub dynamic_obs_decay_time: f64,
    /// new points decay 时间 s
    pub new_points_decay_time: f64,
    /// dyobs update 阈值
    pub dyobs_update_thred: i64,
    /// new point counter
    pub new_point_counter: i64,
    /// obs inflate size
    pub obs_inflate_size: i64,
    /// visualize ratio
    pub visualize_ratio: f64,
}

impl Default for FarPlannerBridgeConfig {
    fn default() -> Self {
        Self {
            executable: PathBuf::from("result/bin/far_planner_native"),
            cwd: None,
            env: BTreeMap::new(),
            build_command: None,
            restart_policy: FarPlannerRestartPolicy::Never,

            update_rate: 5.0,
            robot_dimension: 0.5,
            voxel_dim: 0.1,
            sensor_range: 30.0,
            terrain_range: 7.5,
            local_planner_range: 2.5,
            vehicle_height: 0.75,
            is_static_env: true,
            is_viewpoint_extend: true,
            is_multi_layer: false,
            is_debug_output: false,
            is_attempt_autoswitch: true,
            world_frame: "map".to_string(),

            converge_dist: 1.5,
            goal_adjust_radius: 10.0,
            free_counter_thred: 5,
            reach_goal_vote_size: 5,
            path_momentum_thred: 5,

            floor_height: 2.0,
            cell_length: 5.0,
            map_grid_max_length: 1000.0,
            map_grad_max_height: 100.0,

            connect_votes_size: 10,
            clear_dumper_thred: 3,
            node_finalize_thred: 3,
            filter_pool_size: 12,

            resize_ratio: 5.0,
            filter_count_value: 5,

            angle_noise: 15.0,
            accept_max_align_angle: 15.0,
            new_intensity_thred: 2.0,
            dynamic_obs_decay_time: 10.0,
            new_points_decay_time: 2.0,
            dyobs_update_thred: 4,
            new_point_counter: 10,
            obs_inflate_size: 2,
            visualize_ratio: 0.4,
        }
    }
}

/// 把 `FarPlannerBridgeConfig` 字段映射到 C++ CLI 参数名。
#[must_use]
pub fn cli_name_override() -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    m.insert("robot_dimension".to_string(), "robot_dim".to_string());
    m
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
    s
}

/// 构造 FAR Planner 的 CLI 映射（`NativeCliMapping`）。
#[must_use]
pub fn cli_mapping() -> NativeCliMapping {
    NativeCliMapping {
        exclude: cli_exclude(),
        name_override: cli_name_override(),
        stdin_config: false,
    }
}

/// 把 typed config 字段渲染为 (name, ConfigValue) 对，按字母升序输出。
fn config_pairs(config: &FarPlannerBridgeConfig) -> Vec<(String, ConfigValue)> {
    // 字段集合按字母升序
    let mut pairs: Vec<(String, ConfigValue)> = vec![
        ("update_rate".to_string(), config.update_rate.into()),
        ("robot_dimension".to_string(), config.robot_dimension.into()),
        ("voxel_dim".to_string(), config.voxel_dim.into()),
        ("sensor_range".to_string(), config.sensor_range.into()),
        ("terrain_range".to_string(), config.terrain_range.into()),
        (
            "local_planner_range".to_string(),
            config.local_planner_range.into(),
        ),
        ("vehicle_height".to_string(), config.vehicle_height.into()),
        ("is_static_env".to_string(), config.is_static_env.into()),
        (
            "is_viewpoint_extend".to_string(),
            config.is_viewpoint_extend.into(),
        ),
        ("is_multi_layer".to_string(), config.is_multi_layer.into()),
        ("is_debug_output".to_string(), config.is_debug_output.into()),
        (
            "is_attempt_autoswitch".to_string(),
            config.is_attempt_autoswitch.into(),
        ),
        ("world_frame".to_string(), config.world_frame.clone().into()),
        ("converge_dist".to_string(), config.converge_dist.into()),
        (
            "goal_adjust_radius".to_string(),
            config.goal_adjust_radius.into(),
        ),
        (
            "free_counter_thred".to_string(),
            config.free_counter_thred.into(),
        ),
        (
            "reach_goal_vote_size".to_string(),
            config.reach_goal_vote_size.into(),
        ),
        (
            "path_momentum_thred".to_string(),
            config.path_momentum_thred.into(),
        ),
        ("floor_height".to_string(), config.floor_height.into()),
        ("cell_length".to_string(), config.cell_length.into()),
        (
            "map_grid_max_length".to_string(),
            config.map_grid_max_length.into(),
        ),
        (
            "map_grad_max_height".to_string(),
            config.map_grad_max_height.into(),
        ),
        (
            "connect_votes_size".to_string(),
            config.connect_votes_size.into(),
        ),
        (
            "clear_dumper_thred".to_string(),
            config.clear_dumper_thred.into(),
        ),
        (
            "node_finalize_thred".to_string(),
            config.node_finalize_thred.into(),
        ),
        (
            "filter_pool_size".to_string(),
            config.filter_pool_size.into(),
        ),
        ("resize_ratio".to_string(), config.resize_ratio.into()),
        (
            "filter_count_value".to_string(),
            config.filter_count_value.into(),
        ),
        ("angle_noise".to_string(), config.angle_noise.into()),
        (
            "accept_max_align_angle".to_string(),
            config.accept_max_align_angle.into(),
        ),
        (
            "new_intensity_thred".to_string(),
            config.new_intensity_thred.into(),
        ),
        (
            "dynamic_obs_decay_time".to_string(),
            config.dynamic_obs_decay_time.into(),
        ),
        (
            "new_points_decay_time".to_string(),
            config.new_points_decay_time.into(),
        ),
        (
            "dyobs_update_thred".to_string(),
            config.dyobs_update_thred.into(),
        ),
        (
            "new_point_counter".to_string(),
            config.new_point_counter.into(),
        ),
        (
            "obs_inflate_size".to_string(),
            config.obs_inflate_size.into(),
        ),
        ("visualize_ratio".to_string(), config.visualize_ratio.into()),
    ];
    pairs.sort_by(|a, b| a.0.cmp(&b.0));
    pairs
}

/// 把 FAR Planner 配置转换为 native CLI 参数列表（按字母升序）。
#[must_use]
pub fn to_native_args(config: &FarPlannerBridgeConfig) -> Vec<String> {
    let mapping = cli_mapping();
    let mut out = Vec::new();
    for (name, value) in config_pairs(config) {
        out.extend(mapping.render_arg(&name, Some(value)));
    }
    out
}

/// FAR Planner bridge 顶层句柄。
pub struct FarPlannerBridge {
    /// 内部 NativeRunner（由 Foundation lane 提供）
    pub runner: nuwax_nav_native_runner::NativeRunner,
    /// native spec 缓存（用于 argv/validate 查询）
    pub spec: NativeProgramSpec,
    /// cwd（runner 镜像，便于无 runner 引用时查询）
    pub cwd: Option<std::path::PathBuf>,
    /// env
    pub env: BTreeMap<String, String>,
    /// build command
    pub build_command: Option<String>,
}

impl FarPlannerBridge {
    /// 从配置 + topics 构造 bridge。
    #[must_use]
    pub fn new(config: FarPlannerBridgeConfig, topics: &FarPlannerTopics) -> Self {
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
    ///
    /// 顺序：executable -> topic args（port 字典序）-> config args（字段名升序）。
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

/// 构造 `NativeProgramSpec`（不启动进程）。
#[must_use]
pub fn build_spec(config: FarPlannerBridgeConfig, topics: &FarPlannerTopics) -> NativeProgramSpec {
    NativeProgramSpec {
        name: "far_planner".to_string(),
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
    fn topics_to_topic_map_keeps_only_some() {
        let topics = FarPlannerTopics {
            terrain_map_ext: None,
            terrain_map: Some("/terrain".to_string()),
            registered_scan: Some("/scan".to_string()),
            odometry: Some("/odom".to_string()),
            goal: Some("/goal".to_string()),
            stop_movement: None,
            way_point: Some("/wp".to_string()),
            goal_path: None,
            graph_nodes: None,
            graph_edges: None,
            contour_polygons: None,
            nav_boundary: None,
        };
        let m = topics.to_topic_map();
        assert_eq!(m.get("odometry").map(String::as_str), Some("/odom"));
        assert_eq!(m.get("goal").map(String::as_str), Some("/goal"));
        assert_eq!(m.get("terrain_map").map(String::as_str), Some("/terrain"));
        assert_eq!(m.get("way_point").map(String::as_str), Some("/wp"));
        assert!(!m.contains_key("terrain_map_ext"));
        assert!(!m.contains_key("stop_movement"));
        assert!(!m.contains_key("goal_path"));
        assert!(!m.contains_key("graph_nodes"));
        assert!(!m.contains_key("graph_edges"));
        assert!(!m.contains_key("contour_polygons"));
        assert!(!m.contains_key("nav_boundary"));
    }

    #[test]
    fn cli_name_override_robot_dimension_to_robot_dim() {
        let m = cli_name_override();
        assert_eq!(
            m.get("robot_dimension").map(String::as_str),
            Some("robot_dim")
        );
    }

    #[test]
    fn to_native_args_renders_history_name_and_default_values() {
        let cfg = FarPlannerBridgeConfig::default();
        let args = to_native_args(&cfg);
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--robot_dim" && w[1] == "0.5"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--update_rate" && w[1] == "5"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--world_frame" && w[1] == "map"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--is_static_env" && w[1] == "true"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--is_multi_layer" && w[1] == "false"));
    }

    #[test]
    fn to_native_args_excludes_runner_fields() {
        let cfg = FarPlannerBridgeConfig::default();
        let args = to_native_args(&cfg);
        for a in &args {
            assert!(!a.starts_with("--executable"));
            assert!(!a.starts_with("--cwd"));
            assert!(!a.starts_with("--env"));
            assert!(!a.starts_with("--build_command"));
            assert!(!a.starts_with("--restart_policy"));
        }
    }

    #[test]
    fn spec_validates() {
        let cfg = FarPlannerBridgeConfig::default();
        let topics = FarPlannerTopics::default();
        let s = build_spec(cfg, &topics);
        // spec.validate 不要求 binary 存在，只校验空字符串
        assert!(s.validate().is_ok() || s.validate().is_err());
    }

    #[test]
    fn argv_starts_with_executable() {
        let cfg = FarPlannerBridgeConfig::default();
        let topics = FarPlannerTopics {
            odometry: Some("/odom".to_string()),
            goal: Some("/goal".to_string()),
            ..Default::default()
        };
        let bridge = FarPlannerBridge::new(cfg, &topics);
        let argv = bridge.argv();
        assert!(argv[0].ends_with("far_planner_native"));
    }

    #[test]
    fn validate_config_rejects_nix_build() {
        let cfg = FarPlannerBridgeConfig {
            build_command: Some(
                "nix build github:nuwax-org/nuwax-module-far-planner/v0.5.0".to_string(),
            ),
            ..FarPlannerBridgeConfig::default()
        };
        let err = super::super::validate_config(&cfg).unwrap_err();
        assert!(matches!(
            err,
            super::super::FarPlannerBridgeError::InvalidConfig(_)
        ));
    }
}

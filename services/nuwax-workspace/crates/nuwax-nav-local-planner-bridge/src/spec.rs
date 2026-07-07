//! Local Planner native bridge spec
//!
//! 端口表（方案 §4.3）:
//! - In (11): `registered_scan`、`odometry`、`terrain_map`、`joy_cmd`、`way_point`、
//!   `goal_pose`、`speed`、`navigation_boundary`、`added_obstacles`、`check_obstacle`、`cancel_goal`
//! - Out (5): `path`、`effective_cmd_vel`、`free_paths`、`slow_down`、`goal_reached`
//!
//! `goal_reached` 是 typed output（[`GoalReached`]），由 `nuwax-nav-stack` 状态机消费。
//! `Option<f64>` None 字段**不**进入 native CLI；不得用 0.0 代替。

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use nuwax_nav_base::native::RestartPolicy;
use nuwax_nav_native_runner::{ConfigValue, NativeCliMapping, NativeProgramSpec};
use serde::{Deserialize, Serialize};

/// `goal_reached` typed output。
///
/// 由 native 输出 topic `goal_reached` 映射而来；Stack lane 根据此信号推进状态机。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GoalReached {
    /// 未到达
    NotReached,
    /// 已到达
    Reached,
}

/// Local Planner 重启策略。
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LocalPlannerRestartPolicy {
    /// 永不自动重启
    #[default]
    Never,
    /// 出错后重启
    OnFailure,
}

impl From<LocalPlannerRestartPolicy> for RestartPolicy {
    fn from(p: LocalPlannerRestartPolicy) -> Self {
        match p {
            LocalPlannerRestartPolicy::Never => Self::Never,
            LocalPlannerRestartPolicy::OnFailure => Self::OnFailure {
                max_restarts: 3,
                backoff_millis: 1000,
            },
        }
    }
}

/// Local Planner 输入/输出 topic 名映射。
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct LocalPlannerTopics {
    // In
    /// 注册点云 topic
    pub registered_scan: Option<String>,
    /// 里程计 topic
    pub odometry: Option<String>,
    /// 地形点云 topic
    pub terrain_map: Option<String>,
    /// 手柄/人工速度 topic
    pub joy_cmd: Option<String>,
    /// 上游 waypoint topic
    pub way_point: Option<String>,
    /// 目标 pose topic
    pub goal_pose: Option<String>,
    /// 速度标量 topic
    pub speed: Option<String>,
    /// 导航边界 polygon topic
    pub navigation_boundary: Option<String>,
    /// 动态/人工障碍点云 topic
    pub added_obstacles: Option<String>,
    /// 障碍检查开关 topic
    pub check_obstacle: Option<String>,
    /// 取消目标 topic
    pub cancel_goal: Option<String>,
    // Out
    /// 局部路径 topic
    pub path: Option<String>,
    /// 有效速度命令 topic
    pub effective_cmd_vel: Option<String>,
    /// 候选自由路径点云 topic
    pub free_paths: Option<String>,
    /// 减速状态 topic
    pub slow_down: Option<String>,
    /// 到达状态 topic
    pub goal_reached: Option<String>,
}

impl LocalPlannerTopics {
    /// 转为 `NativeProgramSpec.topics`（port -> topic）。
    #[must_use]
    pub fn to_topic_map(&self) -> BTreeMap<String, String> {
        let all: &[(&str, &Option<String>)] = &[
            ("registered_scan", &self.registered_scan),
            ("odometry", &self.odometry),
            ("terrain_map", &self.terrain_map),
            ("joy_cmd", &self.joy_cmd),
            ("way_point", &self.way_point),
            ("goal_pose", &self.goal_pose),
            ("speed", &self.speed),
            ("navigation_boundary", &self.navigation_boundary),
            ("added_obstacles", &self.added_obstacles),
            ("check_obstacle", &self.check_obstacle),
            ("cancel_goal", &self.cancel_goal),
            ("path", &self.path),
            ("effective_cmd_vel", &self.effective_cmd_vel),
            ("free_paths", &self.free_paths),
            ("slow_down", &self.slow_down),
            ("goal_reached", &self.goal_reached),
        ];
        all.iter()
            .filter_map(|(k, v)| v.as_ref().map(|t| ((*k).to_string(), t.clone())))
            .collect()
    }
}

/// Local Planner 配置。
///
/// `Option<f64>` 字段 `None` 表示不向 C++ CLI 传该参数。bool 同理。
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct LocalPlannerBridgeConfig {
    // runner
    /// native 二进制相对路径
    pub executable: PathBuf,
    /// cwd
    pub cwd: Option<PathBuf>,
    /// env
    pub env: BTreeMap<String, String>,
    /// build command（默认 None；不得为 `nix build github:`）
    pub build_command: Option<String>,
    /// 重启策略
    pub restart_policy: LocalPlannerRestartPolicy,
    /// stdin config（默认 false）
    pub stdin_config: bool,

    // paths
    /// 路径资源目录（可空；启动前检查存在）
    pub paths_dir: String,

    // vehicle
    /// 车长 m
    pub vehicle_length: f64,
    /// 车宽 m
    pub vehicle_width: f64,
    /// 传感器 x 偏移 m（None 不传）
    pub sensor_offset_x: Option<f64>,
    /// 传感器 y 偏移 m（None 不传）
    pub sensor_offset_y: Option<f64>,

    // speed
    /// 最大速度 m/s
    pub max_speed: f64,
    /// autonomy 速度 m/s
    pub autonomy_speed: f64,
    /// autonomy mode（None 不传）
    pub autonomy_mode: Option<bool>,
    /// use_terrain_analysis（默认 true）
    pub use_terrain_analysis: bool,
    /// check_obstacle（默认 true）
    pub check_obstacle: bool,
    /// check_rot_obstacle（None 不传）
    pub check_rot_obstacle: Option<bool>,
    /// use_cost（None 不传）
    pub use_cost: Option<bool>,

    // thresholds
    /// 障碍高度阈值 m
    pub obstacle_height_threshold: f64,
    /// 地面高度阈值 m
    pub ground_height_threshold: f64,
    /// cost_height_thre1 m（None 不传）
    pub cost_height_thre1: Option<f64>,
    /// cost_height_thre2 m（None 不传）
    pub cost_height_thre2: Option<f64>,
    /// max_relative_z m
    pub max_relative_z: f64,
    /// min_relative_z m
    pub min_relative_z: f64,
    /// adjacent_range m
    pub adjacent_range: f64,
    /// laser_voxel_size m（None 不传）
    pub laser_voxel_size: Option<f64>,
    /// terrain_voxel_size m（None 不传）
    pub terrain_voxel_size: Option<f64>,

    // dir / path scaling
    /// dir weight
    pub dir_weight: f64,
    /// dir threshold deg
    pub dir_thre: f64,
    /// dir_to_vehicle（None 不传）
    pub dir_to_vehicle: Option<bool>,
    /// path_scale
    pub path_scale: f64,
    /// min_path_scale
    pub min_path_scale: f64,
    /// path_scale_step
    pub path_scale_step: f64,
    /// path_scale_by_speed（None 不传）
    pub path_scale_by_speed: Option<bool>,
    /// min_path_range m
    pub min_path_range: f64,
    /// path_range_step m
    pub path_range_step: f64,
    /// path_range_by_speed（None 不传）
    pub path_range_by_speed: Option<bool>,
    /// path_crop_by_goal（None 不传）
    pub path_crop_by_goal: Option<bool>,
    /// point_per_path_thre
    pub point_per_path_thre: i64,
    /// slow_path_num_thre
    pub slow_path_num_thre: i64,
    /// slow_group_num_thre
    pub slow_group_num_thre: i64,
    /// omni_dir_goal_thre m
    pub omni_dir_goal_thre: f64,

    // goal
    /// goal_clearance m
    pub goal_clearance: f64,
    /// goal_reached_threshold m
    pub goal_reached_threshold: f64,
    /// goal_behind_range m
    pub goal_behind_range: f64,
    /// goal_yaw_threshold rad
    pub goal_yaw_threshold: f64,
    /// freeze_ang deg（None 不传；omni-dir 设为 180）
    pub freeze_ang: Option<f64>,
    /// freeze_time s（None 不传）
    pub freeze_time: Option<f64>,
    /// two_way_drive（None 不传）
    pub two_way_drive: Option<bool>,
    /// goal_x m（None 不传）
    pub goal_x: Option<f64>,
    /// goal_y m（None 不传）
    pub goal_y: Option<f64>,

    // joy / publish
    /// joy_to_speed_delay s（None 不传）
    pub joy_to_speed_delay: Option<f64>,
    /// joy_to_check_obstacle_delay s（None 不传）
    pub joy_to_check_obstacle_delay: Option<f64>,
    /// publish_free_paths（None 不传）
    pub publish_free_paths: Option<bool>,

    // penalty
    /// max_momentum_penalty（None 不传）
    pub max_momentum_penalty: Option<f64>,
}

impl Default for LocalPlannerBridgeConfig {
    fn default() -> Self {
        Self {
            executable: PathBuf::from("result/bin/local_planner"),
            cwd: None,
            env: BTreeMap::new(),
            build_command: None,
            restart_policy: LocalPlannerRestartPolicy::Never,
            stdin_config: false,

            paths_dir: String::new(),

            vehicle_length: 0.5,
            vehicle_width: 0.5,
            sensor_offset_x: None,
            sensor_offset_y: None,

            max_speed: 0.75,
            autonomy_speed: 0.75,
            autonomy_mode: None,
            use_terrain_analysis: true,
            check_obstacle: true,
            check_rot_obstacle: None,
            use_cost: None,

            obstacle_height_threshold: 0.1,
            ground_height_threshold: 0.1,
            cost_height_thre1: None,
            cost_height_thre2: None,
            max_relative_z: 0.3,
            min_relative_z: -0.4,
            adjacent_range: 3.5,
            laser_voxel_size: None,
            terrain_voxel_size: None,

            dir_weight: 0.02,
            dir_thre: 90.0,
            dir_to_vehicle: None,
            path_scale: 0.875,
            min_path_scale: 0.675,
            path_scale_step: 0.1,
            path_scale_by_speed: None,
            min_path_range: 0.8,
            path_range_step: 0.6,
            path_range_by_speed: None,
            path_crop_by_goal: None,
            point_per_path_thre: 2,
            slow_path_num_thre: 5,
            slow_group_num_thre: 1,
            omni_dir_goal_thre: 0.5,

            goal_clearance: 0.6,
            goal_reached_threshold: 0.3,
            goal_behind_range: 0.8,
            goal_yaw_threshold: 0.15,
            freeze_ang: None,
            freeze_time: None,
            two_way_drive: None,
            goal_x: None,
            goal_y: None,

            joy_to_speed_delay: None,
            joy_to_check_obstacle_delay: None,
            publish_free_paths: None,

            max_momentum_penalty: None,
        }
    }
}

/// 把 `LocalPlannerBridgeConfig` 字段映射到 C++ CLI 参数名（camelCase）。
#[must_use]
pub fn cli_name_override() -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    let pairs: &[(&str, &str)] = &[
        ("max_speed", "maxSpeed"),
        ("autonomy_speed", "autonomySpeed"),
        ("autonomy_mode", "autonomyMode"),
        ("use_terrain_analysis", "useTerrainAnalysis"),
        ("obstacle_height_threshold", "obstacleHeightThre"),
        ("ground_height_threshold", "groundHeightThre"),
        ("cost_height_thre1", "costHeightThre1"),
        ("cost_height_thre2", "costHeightThre2"),
        ("max_relative_z", "maxRelZ"),
        ("min_relative_z", "minRelZ"),
        ("goal_clearance", "goalClearance"),
        ("goal_reached_threshold", "goalReachedThreshold"),
        ("goal_behind_range", "goalBehindRange"),
        ("goal_yaw_threshold", "goalYawThreshold"),
        ("freeze_ang", "freezeAng"),
        ("freeze_time", "freezeTime"),
        ("two_way_drive", "twoWayDrive"),
        ("goal_x", "goalX"),
        ("goal_y", "goalY"),
        ("vehicle_length", "vehicleLength"),
        ("vehicle_width", "vehicleWidth"),
        ("sensor_offset_x", "sensorOffsetX"),
        ("sensor_offset_y", "sensorOffsetY"),
        ("laser_voxel_size", "laserVoxelSize"),
        ("terrain_voxel_size", "terrainVoxelSize"),
        ("check_obstacle", "checkObstacle"),
        ("check_rot_obstacle", "checkRotObstacle"),
        ("adjacent_range", "adjacentRange"),
        ("use_cost", "useCost"),
        ("slow_path_num_thre", "slowPathNumThre"),
        ("slow_group_num_thre", "slowGroupNumThre"),
        ("point_per_path_thre", "pointPerPathThre"),
        ("dir_weight", "dirWeight"),
        ("dir_thre", "dirThre"),
        ("dir_to_vehicle", "dirToVehicle"),
        ("path_scale", "pathScale"),
        ("min_path_scale", "minPathScale"),
        ("path_scale_step", "pathScaleStep"),
        ("path_scale_by_speed", "pathScaleBySpeed"),
        ("min_path_range", "minPathRange"),
        ("path_range_step", "pathRangeStep"),
        ("path_range_by_speed", "pathRangeBySpeed"),
        ("path_crop_by_goal", "pathCropByGoal"),
        ("joy_to_speed_delay", "joyToSpeedDelay"),
        ("joy_to_check_obstacle_delay", "joyToCheckObstacleDelay"),
        ("omni_dir_goal_thre", "omniDirGoalThre"),
        ("publish_free_paths", "publishFreePaths"),
        ("max_momentum_penalty", "maxMomentumPenalty"),
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
    s.insert("paths_dir".to_string());
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
fn config_pairs(config: &LocalPlannerBridgeConfig) -> Vec<(String, Option<ConfigValue>)> {
    let mut pairs: Vec<(String, Option<ConfigValue>)> = Vec::new();
    let push_f = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: f64| {
        p.push((n.to_string(), Some(v.into())));
    };
    let push_o = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: Option<f64>| {
        p.push((n.to_string(), v.map(ConfigValue::F64)));
    };
    let push_b = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: bool| {
        p.push((n.to_string(), Some(v.into())));
    };
    let push_ob = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: Option<bool>| {
        p.push((n.to_string(), v.map(ConfigValue::Bool)));
    };
    let push_i = |p: &mut Vec<(String, Option<ConfigValue>)>, n: &str, v: i64| {
        p.push((n.to_string(), Some(v.into())));
    };

    push_f(&mut pairs, "vehicle_length", config.vehicle_length);
    push_f(&mut pairs, "vehicle_width", config.vehicle_width);
    push_o(&mut pairs, "sensor_offset_x", config.sensor_offset_x);
    push_o(&mut pairs, "sensor_offset_y", config.sensor_offset_y);
    push_f(&mut pairs, "max_speed", config.max_speed);
    push_f(&mut pairs, "autonomy_speed", config.autonomy_speed);
    push_ob(&mut pairs, "autonomy_mode", config.autonomy_mode);
    push_b(
        &mut pairs,
        "use_terrain_analysis",
        config.use_terrain_analysis,
    );
    push_b(&mut pairs, "check_obstacle", config.check_obstacle);
    push_ob(&mut pairs, "check_rot_obstacle", config.check_rot_obstacle);
    push_ob(&mut pairs, "use_cost", config.use_cost);
    push_f(
        &mut pairs,
        "obstacle_height_threshold",
        config.obstacle_height_threshold,
    );
    push_f(
        &mut pairs,
        "ground_height_threshold",
        config.ground_height_threshold,
    );
    push_o(&mut pairs, "cost_height_thre1", config.cost_height_thre1);
    push_o(&mut pairs, "cost_height_thre2", config.cost_height_thre2);
    push_f(&mut pairs, "max_relative_z", config.max_relative_z);
    push_f(&mut pairs, "min_relative_z", config.min_relative_z);
    push_f(&mut pairs, "adjacent_range", config.adjacent_range);
    push_o(&mut pairs, "laser_voxel_size", config.laser_voxel_size);
    push_o(&mut pairs, "terrain_voxel_size", config.terrain_voxel_size);
    push_f(&mut pairs, "dir_weight", config.dir_weight);
    push_f(&mut pairs, "dir_thre", config.dir_thre);
    push_ob(&mut pairs, "dir_to_vehicle", config.dir_to_vehicle);
    push_f(&mut pairs, "path_scale", config.path_scale);
    push_f(&mut pairs, "min_path_scale", config.min_path_scale);
    push_f(&mut pairs, "path_scale_step", config.path_scale_step);
    push_ob(
        &mut pairs,
        "path_scale_by_speed",
        config.path_scale_by_speed,
    );
    push_f(&mut pairs, "min_path_range", config.min_path_range);
    push_f(&mut pairs, "path_range_step", config.path_range_step);
    push_ob(
        &mut pairs,
        "path_range_by_speed",
        config.path_range_by_speed,
    );
    push_ob(&mut pairs, "path_crop_by_goal", config.path_crop_by_goal);
    push_i(
        &mut pairs,
        "point_per_path_thre",
        config.point_per_path_thre,
    );
    push_i(&mut pairs, "slow_path_num_thre", config.slow_path_num_thre);
    push_i(
        &mut pairs,
        "slow_group_num_thre",
        config.slow_group_num_thre,
    );
    push_f(&mut pairs, "omni_dir_goal_thre", config.omni_dir_goal_thre);
    push_f(&mut pairs, "goal_clearance", config.goal_clearance);
    push_f(
        &mut pairs,
        "goal_reached_threshold",
        config.goal_reached_threshold,
    );
    push_f(&mut pairs, "goal_behind_range", config.goal_behind_range);
    push_f(&mut pairs, "goal_yaw_threshold", config.goal_yaw_threshold);
    push_o(&mut pairs, "freeze_ang", config.freeze_ang);
    push_o(&mut pairs, "freeze_time", config.freeze_time);
    push_ob(&mut pairs, "two_way_drive", config.two_way_drive);
    push_o(&mut pairs, "goal_x", config.goal_x);
    push_o(&mut pairs, "goal_y", config.goal_y);
    push_o(&mut pairs, "joy_to_speed_delay", config.joy_to_speed_delay);
    push_o(
        &mut pairs,
        "joy_to_check_obstacle_delay",
        config.joy_to_check_obstacle_delay,
    );
    push_ob(&mut pairs, "publish_free_paths", config.publish_free_paths);
    push_o(
        &mut pairs,
        "max_momentum_penalty",
        config.max_momentum_penalty,
    );

    pairs.sort_by(|a, b| a.0.cmp(&b.0));
    pairs
}

/// 把 typed config 渲染为 native CLI `--name value` 参数列表。
#[must_use]
pub fn to_native_args(config: &LocalPlannerBridgeConfig) -> Vec<String> {
    let mapping = cli_mapping();
    let mut out = Vec::new();
    for (name, value) in config_pairs(config) {
        out.extend(mapping.render_arg(&name, value));
    }
    out
}

/// Local Planner bridge 顶层句柄。
pub struct LocalPlannerBridge {
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
    /// typed output goal_reached（消费侧使用）
    pub last_goal_reached: Option<GoalReached>,
}

impl LocalPlannerBridge {
    /// 从配置 + topics 构造 bridge。
    #[must_use]
    pub fn new(config: LocalPlannerBridgeConfig, topics: &LocalPlannerTopics) -> Self {
        use crate::build_runner;
        let spec = build_spec(config.clone(), topics);
        let runner = build_runner(&config, topics);
        Self {
            runner,
            spec,
            cwd: config.cwd.clone(),
            env: config.env.clone(),
            build_command: config.build_command.clone(),
            last_goal_reached: None,
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
    config: LocalPlannerBridgeConfig,
    topics: &LocalPlannerTopics,
) -> NativeProgramSpec {
    NativeProgramSpec {
        name: "local_planner".to_string(),
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
    fn cli_name_override_includes_camelcase_pairs() {
        let m = cli_name_override();
        assert_eq!(m.get("max_speed").map(String::as_str), Some("maxSpeed"));
        assert_eq!(
            m.get("obstacle_height_threshold").map(String::as_str),
            Some("obstacleHeightThre")
        );
        assert_eq!(m.get("goal_x").map(String::as_str), Some("goalX"));
        assert_eq!(
            m.get("joy_to_check_obstacle_delay").map(String::as_str),
            Some("joyToCheckObstacleDelay")
        );
    }

    #[test]
    fn to_native_args_skips_none_and_renders_camelcase() {
        let cfg = LocalPlannerBridgeConfig::default();
        let args = to_native_args(&cfg);
        // None 字段不渲染
        assert!(!args.iter().any(|a| a == "--sensorOffsetX"));
        assert!(!args.iter().any(|a| a == "--freezeAng"));
        assert!(!args.iter().any(|a| a == "--goalX"));
        // camelCase 渲染
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--maxSpeed" && w[1] == "0.75"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--useTerrainAnalysis" && w[1] == "true"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--vehicleWidth" && w[1] == "0.5"));
        // 排除字段
        assert!(!args.iter().any(|a| a.starts_with("--executable")));
        assert!(!args.iter().any(|a| a.starts_with("--pathsDir")));
        assert!(!args.iter().any(|a| a.starts_with("--stdinConfig")));
    }

    #[test]
    fn to_native_args_some_option_is_rendered() {
        let cfg = LocalPlannerBridgeConfig {
            sensor_offset_x: Some(0.2),
            freeze_ang: Some(180.0),
            ..LocalPlannerBridgeConfig::default()
        };
        let args = to_native_args(&cfg);
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--sensorOffsetX" && w[1] == "0.2"));
        assert!(args
            .windows(2)
            .any(|w| w[0] == "--freezeAng" && w[1] == "180"));
    }

    #[test]
    fn topics_to_topic_map_all_16_ports() {
        let topics = LocalPlannerTopics {
            registered_scan: Some("/scan".to_string()),
            odometry: Some("/odom".to_string()),
            terrain_map: Some("/tm".to_string()),
            joy_cmd: Some("/joy".to_string()),
            way_point: Some("/wp".to_string()),
            goal_pose: Some("/goal".to_string()),
            speed: Some("/sp".to_string()),
            navigation_boundary: Some("/nb".to_string()),
            added_obstacles: Some("/ao".to_string()),
            check_obstacle: Some("/co".to_string()),
            cancel_goal: Some("/cg".to_string()),
            path: Some("/path".to_string()),
            effective_cmd_vel: Some("/ecv".to_string()),
            free_paths: Some("/fp".to_string()),
            slow_down: Some("/sd".to_string()),
            goal_reached: Some("/gr".to_string()),
        };
        let m = topics.to_topic_map();
        assert_eq!(m.len(), 16);
        assert_eq!(m.get("odometry").map(String::as_str), Some("/odom"));
        assert_eq!(m.get("goal_reached").map(String::as_str), Some("/gr"));
    }

    #[test]
    fn bridge_argv_topic_then_config_order() {
        let cfg = LocalPlannerBridgeConfig::default();
        let topics = LocalPlannerTopics {
            odometry: Some("/odom".to_string()),
            ..Default::default()
        };
        let bridge = LocalPlannerBridge::new(cfg, &topics);
        let argv = bridge.argv();
        assert!(argv[0].ends_with("local_planner"));
        assert_eq!(argv[1], "--odometry");
        assert_eq!(argv[2], "/odom");
        let pos = argv
            .iter()
            .position(|a| a == "--maxSpeed")
            .expect("config present");
        assert!(pos > 2);
    }
}

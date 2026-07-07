//! Terrain Analysis bridge typed config
//!
//! - 30+ 个 typed 字段,完整迁移自历史 `TerrainAnalysisConfig`
//! - CLI 名遵循 C++ 原生 wrapper 的 camelCase 命名(sensorRange / scanVoxelSize / ...)
//! - `cli_name_override()` 返回 BTreeMap<String, String> 用于测试 / 文档
//! - `cli_exclude()` 排除 runner 字段(executable / cwd / env / build_command /
//!   build_policy / restart_policy),避免被 argv 渲染
//!
//! 该 crate 是 topic-only bridge:不做 point cloud 解码,typed 输出为 none。

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use nuwax_nav_base::native::RestartPolicy;
use nuwax_nav_native_runner::NativeBuildPolicy;
use serde::{Deserialize, Serialize};

use crate::supervisor::DEFAULT_RESTART_POLICY;

/// Terrain Analysis bridge 完整 typed 配置
///
/// 字段名 = Rust snake_case,CLI 渲染时通过 [`cli_name_override`] 转 camelCase;
/// `None` 字段不进入 argv(由 `to_native_args` 跳过)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct TerrainAnalysisBridgeConfig {
    // ---- runner 字段(由 BridgeSupervisor 包装,不在 argv 中渲染)----
    /// 可执行文件路径(默认 `result/bin/terrain_analysis`)
    pub executable: PathBuf,
    /// 工作目录
    pub cwd: Option<PathBuf>,
    /// 额外环境变量
    pub env: BTreeMap<String, String>,
    /// 构建命令(默认 `None`;不自动网络 build)
    pub build_command: Option<String>,
    /// 构建策略(默认 `Never`)
    pub build_policy: NativeBuildPolicy,
    /// 重启策略(默认 `DEFAULT_RESTART_POLICY`)
    pub restart_policy: RestartPolicy,

    // ---- typed config 字段(进入 argv)----
    /// 传感器范围(m;默认 20.0)
    pub sensor_range: f64,
    /// scan voxel size(m;默认 0.05)
    pub scan_voxel_size: f64,
    /// terrain voxel size(m;默认 1.0)
    pub terrain_voxel_size: f64,
    /// terrain voxel 半宽(默认 10)
    pub terrain_voxel_half_width: i64,
    /// 障碍高度阈值(默认 0.15)
    pub obstacle_height_threshold: f64,
    /// 地面高度阈值(默认 0.1)
    pub ground_height_threshold: f64,
    /// vehicle height(可选)
    pub vehicle_height: Option<f64>,
    /// min relative z(可选)
    pub min_relative_z: Option<f64>,
    /// max relative z(可选)
    pub max_relative_z: Option<f64>,
    /// use sorting(可选)
    pub use_sorting: Option<bool>,
    /// quantile z(可选)
    pub quantile_z: Option<f64>,
    /// decay time(可选,秒)
    pub decay_time: Option<f64>,
    /// no decay distance(可选,m)
    pub no_decay_distance: Option<f64>,
    /// clearing distance(可选,m)
    pub clearing_distance: Option<f64>,
    /// clear dynamic obstacles(可选)
    pub clear_dynamic_obstacles: Option<bool>,
    /// no data obstacle(可选)
    pub no_data_obstacle: Option<bool>,
    /// no data block skip count(可选)
    pub no_data_block_skip_count: Option<i64>,
    /// min block point count(可选)
    pub min_block_point_count: Option<i64>,
    /// voxel point update threshold(可选)
    pub voxel_point_update_threshold: Option<i64>,
    /// voxel time update threshold(可选)
    pub voxel_time_update_threshold: Option<f64>,
    /// min dynamic obstacle distance(可选)
    pub min_dynamic_obstacle_distance: Option<f64>,
    /// abs dynamic obstacle relative z threshold(可选)
    pub abs_dynamic_obstacle_relative_z_threshold: Option<f64>,
    /// min dynamic obstacle VFOV(可选)
    pub min_dynamic_obstacle_vfov: Option<f64>,
    /// max dynamic obstacle VFOV(可选)
    pub max_dynamic_obstacle_vfov: Option<f64>,
    /// min dynamic obstacle point count(可选)
    pub min_dynamic_obstacle_point_count: Option<i64>,
    /// min out of fov point count(可选)
    pub min_out_of_fov_point_count: Option<i64>,
    /// consider drop(可选)
    pub consider_drop: Option<bool>,
    /// limit ground lift(可选)
    pub limit_ground_lift: Option<bool>,
    /// max ground lift(可选)
    pub max_ground_lift: Option<f64>,
    /// distance ratio z(可选)
    pub distance_ratio_z: Option<f64>,
}

impl Default for TerrainAnalysisBridgeConfig {
    fn default() -> Self {
        Self {
            executable: PathBuf::from("result/bin/terrain_analysis"),
            cwd: None,
            env: BTreeMap::new(),
            build_command: None,
            build_policy: NativeBuildPolicy::Never,
            restart_policy: DEFAULT_RESTART_POLICY,

            sensor_range: 20.0,
            scan_voxel_size: 0.05,
            terrain_voxel_size: 1.0,
            terrain_voxel_half_width: 10,
            obstacle_height_threshold: 0.15,
            ground_height_threshold: 0.1,
            vehicle_height: None,
            min_relative_z: None,
            max_relative_z: None,
            use_sorting: None,
            quantile_z: None,
            decay_time: None,
            no_decay_distance: None,
            clearing_distance: None,
            clear_dynamic_obstacles: None,
            no_data_obstacle: None,
            no_data_block_skip_count: None,
            min_block_point_count: None,
            voxel_point_update_threshold: None,
            voxel_time_update_threshold: None,
            min_dynamic_obstacle_distance: None,
            abs_dynamic_obstacle_relative_z_threshold: None,
            min_dynamic_obstacle_vfov: None,
            max_dynamic_obstacle_vfov: None,
            min_dynamic_obstacle_point_count: None,
            min_out_of_fov_point_count: None,
            consider_drop: None,
            limit_ground_lift: None,
            max_ground_lift: None,
            distance_ratio_z: None,
        }
    }
}

/// Terrain Analysis bridge CLI 名称覆盖表
///
/// 把 Rust snake_case 字段名映射为 C++ 原生 wrapper 的 camelCase 参数名
/// (与历史 `terrain_analysis.py` CLI argv 一致)。
#[must_use]
pub fn cli_name_override() -> BTreeMap<String, String> {
    BTreeMap::from_iter([
        ("sensor_range".to_string(), "sensorRange".to_string()),
        ("scan_voxel_size".to_string(), "scanVoxelSize".to_string()),
        (
            "terrain_voxel_size".to_string(),
            "terrainVoxelSize".to_string(),
        ),
        (
            "terrain_voxel_half_width".to_string(),
            "terrainVoxelHalfWidth".to_string(),
        ),
        (
            "obstacle_height_threshold".to_string(),
            "obstacleHeightThre".to_string(),
        ),
        (
            "ground_height_threshold".to_string(),
            "groundHeightThre".to_string(),
        ),
        ("vehicle_height".to_string(), "vehicleHeight".to_string()),
        ("min_relative_z".to_string(), "minRelZ".to_string()),
        ("max_relative_z".to_string(), "maxRelZ".to_string()),
        ("use_sorting".to_string(), "useSorting".to_string()),
        ("quantile_z".to_string(), "quantileZ".to_string()),
        ("decay_time".to_string(), "decayTime".to_string()),
        ("no_decay_distance".to_string(), "noDecayDis".to_string()),
        ("clearing_distance".to_string(), "clearingDis".to_string()),
        (
            "clear_dynamic_obstacles".to_string(),
            "clearDyObs".to_string(),
        ),
        ("no_data_obstacle".to_string(), "noDataObstacle".to_string()),
        (
            "no_data_block_skip_count".to_string(),
            "noDataBlockSkipNum".to_string(),
        ),
        (
            "min_block_point_count".to_string(),
            "minBlockPointNum".to_string(),
        ),
        (
            "voxel_point_update_threshold".to_string(),
            "voxelPointUpdateThre".to_string(),
        ),
        (
            "voxel_time_update_threshold".to_string(),
            "voxelTimeUpdateThre".to_string(),
        ),
        (
            "min_dynamic_obstacle_distance".to_string(),
            "minDyObsDis".to_string(),
        ),
        (
            "abs_dynamic_obstacle_relative_z_threshold".to_string(),
            "absDyObsRelZThre".to_string(),
        ),
        (
            "min_dynamic_obstacle_vfov".to_string(),
            "minDyObsVFOV".to_string(),
        ),
        (
            "max_dynamic_obstacle_vfov".to_string(),
            "maxDyObsVFOV".to_string(),
        ),
        (
            "min_dynamic_obstacle_point_count".to_string(),
            "minDyObsPointNum".to_string(),
        ),
        (
            "min_out_of_fov_point_count".to_string(),
            "minOutOfFovPointNum".to_string(),
        ),
        ("consider_drop".to_string(), "considerDrop".to_string()),
        (
            "limit_ground_lift".to_string(),
            "limitGroundLift".to_string(),
        ),
        ("max_ground_lift".to_string(), "maxGroundLift".to_string()),
        ("distance_ratio_z".to_string(), "disRatioZ".to_string()),
    ])
}

/// Terrain Analysis bridge CLI 排除字段
///
/// runner 字段(executable / cwd / env / build_command / build_policy /
/// restart_policy)不进 argv;由 `NativeProgramSpec` 自身字段承载。
#[must_use]
pub fn cli_exclude() -> BTreeSet<String> {
    BTreeSet::from_iter([
        "executable".to_string(),
        "cwd".to_string(),
        "env".to_string(),
        "build_command".to_string(),
        "build_policy".to_string(),
        "restart_policy".to_string(),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_has_no_optional_some() {
        let c = TerrainAnalysisBridgeConfig::default();
        // 30+ 字段,默认值都填好
        assert_eq!(c.sensor_range, 20.0);
        assert_eq!(c.scan_voxel_size, 0.05);
        assert_eq!(c.terrain_voxel_size, 1.0);
        assert_eq!(c.terrain_voxel_half_width, 10);
        assert_eq!(c.obstacle_height_threshold, 0.15);
        assert_eq!(c.ground_height_threshold, 0.1);
        // 所有 Option 字段默认 None
        assert!(c.vehicle_height.is_none());
        assert!(c.use_sorting.is_none());
        assert!(c.decay_time.is_none());
        assert!(c.max_ground_lift.is_none());
        assert!(c.distance_ratio_z.is_none());
    }

    #[test]
    fn cli_name_override_has_camel_case_keys() {
        let m = cli_name_override();
        assert_eq!(
            m.get("sensor_range").map(String::as_str),
            Some("sensorRange")
        );
        assert_eq!(
            m.get("scan_voxel_size").map(String::as_str),
            Some("scanVoxelSize")
        );
        assert_eq!(
            m.get("terrain_voxel_half_width").map(String::as_str),
            Some("terrainVoxelHalfWidth")
        );
        assert_eq!(
            m.get("obstacle_height_threshold").map(String::as_str),
            Some("obstacleHeightThre")
        );
        assert_eq!(
            m.get("min_dynamic_obstacle_vfov").map(String::as_str),
            Some("minDyObsVFOV")
        );
        assert_eq!(
            m.get("distance_ratio_z").map(String::as_str),
            Some("disRatioZ")
        );
    }

    #[test]
    fn cli_exclude_contains_runner_fields() {
        let s = cli_exclude();
        for f in &[
            "executable",
            "cwd",
            "env",
            "build_command",
            "build_policy",
            "restart_policy",
        ] {
            assert!(s.contains(*f), "missing from cli_exclude: {f}");
        }
        // 不应包含 typed config 字段
        assert!(!s.contains("sensor_range"));
        assert!(!s.contains("max_ground_lift"));
    }
}

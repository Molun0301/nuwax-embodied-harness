//! Terrain Analysis bridge spec / topics / config / argv 渲染
//!
//! - 端口表:`registered_scan` / `odometry` / `terrain_map`(固定 3 个端口)
//! - 渲染职责分离:`to_native_args` 只渲染 config args;
//!   topics 唯一通过 `TerrainAnalysisTopics::to_topic_map()` 填入
//!   `NativeProgramSpec.topics`,由 native-runner 统一按字典序渲染
//!   `--{port} {topic}`,避免 topic argv 重复。
//! - `build_specs` 必须 `validate_config`,非法配置 → `NavResult::Err`。
//! - 单一 restart policy 来源:`config.restart_policy` 写入
//!   `NativeProgramSpec.restart`,`BridgeSupervisor` 只读取同一字段(方案 §5.5)。

use std::collections::BTreeMap;

use nuwax_nav_base::error::{NavError, NavResult};
use nuwax_nav_native_runner::{NativeBuildSpec, NativeProgramSpec};
use serde::{Deserialize, Serialize};

use crate::config::{cli_exclude, cli_name_override, TerrainAnalysisBridgeConfig};

/// Terrain Analysis bridge topic 名映射
///
/// 字段名对应 C++ native 二进制的 `--<port>` 参数;值是 topic 名。
/// port 名固定为 `registered_scan` / `odometry` / `terrain_map`,
/// `None` 端口不进 `NativeProgramSpec.topics`。
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct TerrainAnalysisTopics {
    /// 注册点云 topic (in)
    pub registered_scan: Option<String>,
    /// 里程计 topic (in)
    pub odometry: Option<String>,
    /// terrain map 输出 topic (out)
    pub terrain_map: Option<String>,
}

impl TerrainAnalysisTopics {
    /// topics -> `NativeProgramSpec.topics` 映射(port 名固定)
    ///
    /// 仅保留 `Some` 的端口;`None` 不进入 argv。
    #[must_use]
    pub fn to_topic_map(&self) -> BTreeMap<String, String> {
        [
            ("registered_scan", &self.registered_scan),
            ("odometry", &self.odometry),
            ("terrain_map", &self.terrain_map),
        ]
        .iter()
        .filter_map(|(k, v)| v.as_ref().map(|t| ((*k).to_string(), t.clone())))
        .collect()
    }
}

/// 字段位置元组:(field_name, required, validator)
///
/// required = true → 字段必须出现在 argv 中;
/// required = false → 字段为可选,只有 `Some` 时才渲染。
type FieldRule = (&'static str, bool);

/// 按固定顺序返回所有 typed config 字段(便于 `to_native_args` 稳定渲染)
///
/// 顺序与历史 wrapper argv 顺序一致,保证 `spec_argv` golden 测试稳定。
fn field_order() -> Vec<FieldRule> {
    vec![
        ("sensor_range", true),
        ("scan_voxel_size", true),
        ("terrain_voxel_size", true),
        ("terrain_voxel_half_width", true),
        ("obstacle_height_threshold", true),
        ("ground_height_threshold", true),
        ("vehicle_height", false),
        ("min_relative_z", false),
        ("max_relative_z", false),
        ("use_sorting", false),
        ("quantile_z", false),
        ("decay_time", false),
        ("no_decay_distance", false),
        ("clearing_distance", false),
        ("clear_dynamic_obstacles", false),
        ("no_data_obstacle", false),
        ("no_data_block_skip_count", false),
        ("min_block_point_count", false),
        ("voxel_point_update_threshold", false),
        ("voxel_time_update_threshold", false),
        ("min_dynamic_obstacle_distance", false),
        ("abs_dynamic_obstacle_relative_z_threshold", false),
        ("min_dynamic_obstacle_vfov", false),
        ("max_dynamic_obstacle_vfov", false),
        ("min_dynamic_obstacle_point_count", false),
        ("min_out_of_fov_point_count", false),
        ("consider_drop", false),
        ("limit_ground_lift", false),
        ("max_ground_lift", false),
        ("distance_ratio_z", false),
    ]
}

/// 取 typed config 字段的字符串值
///
/// `None` 对应 `None` / None-ish 字段(不进入 argv);
/// 返回 `Some(value)` 表示字段应当渲染。
fn config_value(name: &str, c: &TerrainAnalysisBridgeConfig) -> Option<String> {
    let raw: Option<String> = match name {
        "sensor_range" => Some(format!("{}", c.sensor_range)),
        "scan_voxel_size" => Some(format!("{}", c.scan_voxel_size)),
        "terrain_voxel_size" => Some(format!("{}", c.terrain_voxel_size)),
        "terrain_voxel_half_width" => Some(format!("{}", c.terrain_voxel_half_width)),
        "obstacle_height_threshold" => Some(format!("{}", c.obstacle_height_threshold)),
        "ground_height_threshold" => Some(format!("{}", c.ground_height_threshold)),
        "vehicle_height" => c.vehicle_height.map(|v| format!("{v}")),
        "min_relative_z" => c.min_relative_z.map(|v| format!("{v}")),
        "max_relative_z" => c.max_relative_z.map(|v| format!("{v}")),
        "use_sorting" => c.use_sorting.map(|v| v.to_string()),
        "quantile_z" => c.quantile_z.map(|v| format!("{v}")),
        "decay_time" => c.decay_time.map(|v| format!("{v}")),
        "no_decay_distance" => c.no_decay_distance.map(|v| format!("{v}")),
        "clearing_distance" => c.clearing_distance.map(|v| format!("{v}")),
        "clear_dynamic_obstacles" => c.clear_dynamic_obstacles.map(|v| v.to_string()),
        "no_data_obstacle" => c.no_data_obstacle.map(|v| v.to_string()),
        "no_data_block_skip_count" => c.no_data_block_skip_count.map(|v| format!("{v}")),
        "min_block_point_count" => c.min_block_point_count.map(|v| format!("{v}")),
        "voxel_point_update_threshold" => c.voxel_point_update_threshold.map(|v| format!("{v}")),
        "voxel_time_update_threshold" => c.voxel_time_update_threshold.map(|v| format!("{v}")),
        "min_dynamic_obstacle_distance" => c.min_dynamic_obstacle_distance.map(|v| format!("{v}")),
        "abs_dynamic_obstacle_relative_z_threshold" => c
            .abs_dynamic_obstacle_relative_z_threshold
            .map(|v| format!("{v}")),
        "min_dynamic_obstacle_vfov" => c.min_dynamic_obstacle_vfov.map(|v| format!("{v}")),
        "max_dynamic_obstacle_vfov" => c.max_dynamic_obstacle_vfov.map(|v| format!("{v}")),
        "min_dynamic_obstacle_point_count" => {
            c.min_dynamic_obstacle_point_count.map(|v| format!("{v}"))
        }
        "min_out_of_fov_point_count" => c.min_out_of_fov_point_count.map(|v| format!("{v}")),
        "consider_drop" => c.consider_drop.map(|v| v.to_string()),
        "limit_ground_lift" => c.limit_ground_lift.map(|v| v.to_string()),
        "max_ground_lift" => c.max_ground_lift.map(|v| format!("{v}")),
        "distance_ratio_z" => c.distance_ratio_z.map(|v| format!("{v}")),
        _ => None,
    };
    raw.filter(|s| !s.is_empty())
}

/// `TerrainAnalysisBridgeConfig` 字段校验
///
/// - `executable` 非空
/// - 数值字段(voxel/range/threshold)严格 > 0 且 finite
/// - `build_policy != Never` 必须有 `build_command`
/// - 拒绝 `nix build github:` 等网络自动 build
///
/// # Errors
/// 字段非法时返回 `NavError::InvalidInput`。
pub fn validate_config(config: &TerrainAnalysisBridgeConfig) -> NavResult<()> {
    use nuwax_nav_native_runner::NativeBuildPolicy;

    if config.executable.as_os_str().is_empty() {
        return Err(NavError::InvalidInput {
            field: "executable",
            reason: "executable path is empty".to_string(),
        });
    }

    let pos_f64 = |field: &'static str, v: f64| -> NavResult<()> {
        if !(v.is_finite() && v > 0.0) {
            return Err(NavError::InvalidInput {
                field,
                reason: format!("must be finite and > 0 (got {v})"),
            });
        }
        Ok(())
    };
    let pos_i64 = |field: &'static str, v: i64| -> NavResult<()> {
        if v <= 0 {
            return Err(NavError::InvalidInput {
                field,
                reason: format!("must be > 0 (got {v})"),
            });
        }
        Ok(())
    };

    pos_f64("sensor_range", config.sensor_range)?;
    pos_f64("scan_voxel_size", config.scan_voxel_size)?;
    pos_f64("terrain_voxel_size", config.terrain_voxel_size)?;
    pos_i64("terrain_voxel_half_width", config.terrain_voxel_half_width)?;
    pos_f64(
        "obstacle_height_threshold",
        config.obstacle_height_threshold,
    )?;
    pos_f64("ground_height_threshold", config.ground_height_threshold)?;

    if config.build_policy != NativeBuildPolicy::Never && config.build_command.is_none() {
        return Err(NavError::InvalidInput {
            field: "build_command",
            reason: "build_policy != Never requires build_command".to_string(),
        });
    }
    if let Some(cmd) = &config.build_command {
        if cmd.contains("nix build github:") {
            return Err(NavError::InvalidInput {
                field: "build_command",
                reason: format!("must not auto-run 'nix build github:' (got: {cmd})"),
            });
        }
    }
    Ok(())
}

/// 渲染 config 字段为 native CLI args(不含 topics;topics 走
/// `NativeProgramSpec.topics`)
///
/// 参数顺序固定(参见 `field_order()`),`None` 字段不渲染。
/// 字段名通过 [`cli_name_override`] 转 camelCase;runner 字段被
/// [`cli_exclude`] 排除。
#[must_use]
pub fn to_native_args(config: &TerrainAnalysisBridgeConfig) -> Vec<String> {
    let excludes = cli_exclude();
    let overrides = cli_name_override();
    let mut out = Vec::with_capacity(field_order().len() * 2);
    for (name, _required) in field_order() {
        if excludes.contains(name) {
            continue;
        }
        let Some(value) = config_value(name, config) else {
            continue;
        };
        let cli_name = overrides.get(name).map(String::as_str).unwrap_or(name);
        out.push(format!("--{cli_name}"));
        out.push(value);
    }
    out
}

/// 构造 `NativeProgramSpec` + `NativeBuildSpec`(供 `BridgeSupervisor` 包装)
///
/// - `NativeProgramSpec.topics = topics.to_topic_map()`
/// - `NativeProgramSpec.args = to_native_args(&config)`(**不**含 topics)
/// - `NativeProgramSpec.restart = config.restart_policy`
/// - `NativeBuildSpec.policy = config.build_policy`
///
/// # Errors
/// 字段非法返回 `NavError::InvalidInput`。
pub fn build_specs(
    config: TerrainAnalysisBridgeConfig,
    topics: &TerrainAnalysisTopics,
) -> NavResult<(NativeProgramSpec, NativeBuildSpec)> {
    validate_config(&config)?;
    let spec = NativeProgramSpec {
        name: "terrain_analysis".to_string(),
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
    use std::path::PathBuf;

    fn cfg() -> TerrainAnalysisBridgeConfig {
        TerrainAnalysisBridgeConfig::default()
    }

    #[test]
    fn default_config_validates() {
        assert!(validate_config(&cfg()).is_ok());
    }

    #[test]
    fn topics_to_map_omits_none() {
        let t = TerrainAnalysisTopics {
            registered_scan: Some("/registered_scan".into()),
            odometry: None,
            terrain_map: Some("/terrain_map".into()),
        };
        let m = t.to_topic_map();
        assert_eq!(m.len(), 2);
        assert_eq!(
            m.get("registered_scan").map(String::as_str),
            Some("/registered_scan")
        );
        assert_eq!(
            m.get("terrain_map").map(String::as_str),
            Some("/terrain_map")
        );
        assert!(!m.contains_key("odometry"));
    }

    #[test]
    fn to_native_args_is_config_only_no_topics() {
        let args = to_native_args(&cfg());
        let joined = args.join(" ");
        // 默认 6 个 required 字段必须出现
        assert!(joined.contains("--sensorRange"), "args: {joined}");
        assert!(joined.contains("--scanVoxelSize"));
        assert!(joined.contains("--terrainVoxelSize"));
        assert!(joined.contains("--terrainVoxelHalfWidth"));
        assert!(joined.contains("--obstacleHeightThre"));
        assert!(joined.contains("--groundHeightThre"));
        // 不应有 topic argv 泄漏
        assert!(!args.iter().any(|a| a.contains("registered_scan")));
        assert!(!args.iter().any(|a| a.contains("terrain_map")));
    }

    #[test]
    fn to_native_args_uses_camel_case_overrides() {
        let mut c = cfg();
        c.vehicle_height = Some(0.5);
        c.use_sorting = Some(true);
        c.max_ground_lift = Some(0.2);
        c.distance_ratio_z = Some(0.3);
        let args = to_native_args(&c);
        let joined = args.join(" ");
        assert!(joined.contains("--vehicleHeight 0.5"));
        assert!(joined.contains("--useSorting true"));
        assert!(joined.contains("--maxGroundLift 0.2"));
        assert!(joined.contains("--disRatioZ 0.3"));
    }

    #[test]
    fn build_specs_embeds_topics_in_spec_not_args() {
        let t = TerrainAnalysisTopics {
            registered_scan: Some("/scan".into()),
            odometry: Some("/odom".into()),
            terrain_map: Some("/terrain_map".into()),
        };
        let (spec, _build) = build_specs(cfg(), &t).unwrap();
        assert_eq!(spec.topics.len(), 3);
        for arg in &spec.args {
            assert!(!arg.starts_with("--registered"), "leaked: {arg}");
            assert!(!arg.starts_with("--odometry"), "leaked: {arg}");
            assert!(!arg.starts_with("--terrain_map"), "leaked: {arg}");
        }
    }

    #[test]
    fn validate_rejects_empty_executable() {
        let mut c = cfg();
        c.executable = PathBuf::new();
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            NavError::InvalidInput {
                field: "executable",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_zero_sensor_range() {
        let mut c = cfg();
        c.sensor_range = 0.0;
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            NavError::InvalidInput {
                field: "sensor_range",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_nan_scan_voxel_size() {
        let mut c = cfg();
        c.scan_voxel_size = f64::NAN;
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            NavError::InvalidInput {
                field: "scan_voxel_size",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_zero_terrain_voxel_half_width() {
        let mut c = cfg();
        c.terrain_voxel_half_width = 0;
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            NavError::InvalidInput {
                field: "terrain_voxel_half_width",
                ..
            }
        ));
    }

    #[test]
    fn validate_rejects_nix_build_github_command() {
        let mut c = cfg();
        c.build_policy = nuwax_nav_native_runner::NativeBuildPolicy::IfMissing;
        c.build_command = Some("nix build github:foo/bar".into());
        let err = validate_config(&c).unwrap_err();
        assert!(matches!(
            err,
            NavError::InvalidInput {
                field: "build_command",
                ..
            }
        ));
    }
}

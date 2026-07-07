//! `config_validation` 集成测试
//!
//! 验证非法配置被 `validate_config` / `build_specs` 拒绝:
//! - 空 executable
//! - voxel/range/threshold 非法值(0 / NaN / 负数)
//! - `build_policy != Never` 但缺 build_command
//! - 网络自动 build(`nix build github:`)被拒绝

use std::path::PathBuf;

use nuwax_explore_terrain_bridge::{
    build_specs, validate_config, TerrainAnalysisBridgeConfig, TerrainAnalysisTopics,
};
use nuwax_nav_native_runner::NativeBuildPolicy;

fn cfg() -> TerrainAnalysisBridgeConfig {
    TerrainAnalysisBridgeConfig::default()
}

fn topics() -> TerrainAnalysisTopics {
    TerrainAnalysisTopics {
        registered_scan: Some("/registered_scan".into()),
        odometry: Some("/odometry".into()),
        terrain_map: Some("/terrain_map".into()),
    }
}

#[test]
fn rejects_empty_executable() {
    let mut c = cfg();
    c.executable = PathBuf::new();
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("executable"), "msg={msg}");
}

#[test]
fn rejects_zero_sensor_range() {
    let mut c = cfg();
    c.sensor_range = 0.0;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("sensor_range"), "msg={msg}");
}

#[test]
fn rejects_negative_scan_voxel_size() {
    let mut c = cfg();
    c.scan_voxel_size = -0.01;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("scan_voxel_size"), "msg={msg}");
}

#[test]
fn rejects_nan_terrain_voxel_size() {
    let mut c = cfg();
    c.terrain_voxel_size = f64::NAN;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("terrain_voxel_size"), "msg={msg}");
}

#[test]
fn rejects_zero_terrain_voxel_half_width() {
    let mut c = cfg();
    c.terrain_voxel_half_width = 0;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("terrain_voxel_half_width"), "msg={msg}");
}

#[test]
fn rejects_zero_obstacle_height_threshold() {
    let mut c = cfg();
    c.obstacle_height_threshold = 0.0;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("obstacle_height_threshold"), "msg={msg}");
}

#[test]
fn rejects_zero_ground_height_threshold() {
    let mut c = cfg();
    c.ground_height_threshold = -0.1;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("ground_height_threshold"), "msg={msg}");
}

#[test]
fn rejects_build_policy_ifmissing_without_command() {
    let mut c = cfg();
    c.build_policy = NativeBuildPolicy::IfMissing;
    c.build_command = None;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("build_command"), "msg={msg}");
}

#[test]
fn rejects_build_policy_always_without_command() {
    let mut c = cfg();
    c.build_policy = NativeBuildPolicy::Always;
    c.build_command = None;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("build_command"), "msg={msg}");
}

#[test]
fn rejects_nix_build_github_command() {
    let mut c = cfg();
    c.build_policy = NativeBuildPolicy::IfMissing;
    c.build_command = Some("nix build github:foo/bar".into());
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("build_command"), "msg={msg}");
    assert!(msg.contains("nix"), "msg={msg}");
}

#[test]
fn build_specs_returns_err_for_invalid_config() {
    let mut c = cfg();
    c.executable = PathBuf::new();
    let err = build_specs(c, &topics()).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("executable"), "msg={msg}");
}

#[test]
fn build_specs_accepts_valid_default_config() {
    let (spec, build) = build_specs(cfg(), &topics()).unwrap();
    assert!(!spec.executable.as_os_str().is_empty());
    assert_eq!(spec.topics.len(), 3);
    assert_eq!(build.policy, NativeBuildPolicy::Never);
}

#[test]
fn build_specs_accepts_config_with_all_optional_fields() {
    let mut c = cfg();
    c.vehicle_height = Some(0.5);
    c.use_sorting = Some(true);
    c.decay_time = Some(4.0);
    c.max_ground_lift = Some(0.3);
    c.distance_ratio_z = Some(0.1);
    let (spec, _build) = build_specs(c, &topics()).expect("ok");
    assert!(spec
        .args
        .windows(2)
        .any(|w| w == ["--vehicleHeight", "0.5"]));
    assert!(spec.args.windows(2).any(|w| w == ["--useSorting", "true"]));
}

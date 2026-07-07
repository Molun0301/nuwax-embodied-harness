//! `config_validation` 集成测试:验证非法配置被 `validate_config` 拒绝。
//!
//! 验收点(方案 §5.4):空 executable、负 range、0 update_rate、github build command
//! 全部拒绝。

use std::path::PathBuf;

use nuwax_explore_tare_bridge::{
    build_specs, validate_config, TarePlannerBridgeConfig, TarePlannerTopics,
};
use nuwax_nav_native_runner::NativeBuildPolicy;

fn cfg() -> TarePlannerBridgeConfig {
    TarePlannerBridgeConfig::default()
}

fn topics() -> TarePlannerTopics {
    TarePlannerTopics {
        registered_scan: Some("/registered_scan".into()),
        odometry: Some("/odometry".into()),
        way_point: Some("/way_point".into()),
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
fn rejects_negative_exploration_range() {
    let mut c = cfg();
    c.exploration_range = -5.0;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("exploration_range"), "msg={msg}");
}

#[test]
fn rejects_zero_update_rate() {
    let mut c = cfg();
    c.update_rate = 0.0;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("update_rate"), "msg={msg}");
}

#[test]
fn rejects_nan_sensor_range() {
    let mut c = cfg();
    c.sensor_range = f64::NAN;
    let err = validate_config(&c).unwrap_err();
    let msg = format!("{err:?}");
    assert!(msg.contains("sensor_range"), "msg={msg}");
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
    // 透传为 NavError::InvalidInput
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

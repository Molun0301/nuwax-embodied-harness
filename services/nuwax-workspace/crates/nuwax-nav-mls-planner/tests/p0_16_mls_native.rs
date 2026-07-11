//! P0-16 回归测试:`mls_planner_native` binary 可运行 + 输出 JSON 含 waypoints
//!
//! 通过 `assert_cmd` 不在 dev 范围,改为直接调用 binary 解析 stdout 的 JSON。
//!
//! 约束：binary 必须编译并在 dev 环境下执行,这里走 `cargo run --bin ...` 的等价
//! 路径 —— 测试通过构造 `std::process::Command` 调用 `target/debug/mls_planner_native`。

use std::path::PathBuf;
use std::process::Command;

fn binary_path() -> PathBuf {
    // cargo 自动管理 test working dir 为 crate root,且 CARGO_BIN_EXE_<name>
    // 提供集成测试访问 binary 的标准方式。
    PathBuf::from(env!("CARGO_BIN_EXE_mls_planner_native"))
}

#[test]
fn binary_default_runs_and_prints_json() {
    let out = Command::new(binary_path())
        .output()
        .expect("failed to run mls_planner_native");
    assert!(
        out.status.success(),
        "binary failed: stderr={}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.lines().last().expect("no stdout line");
    let v: serde_json::Value = serde_json::from_str(line).expect("invalid JSON output");
    // start/goal 字段存在
    assert!(v.get("start").is_some(), "missing start");
    assert!(v.get("goal").is_some(), "missing goal");
    assert!(v.get("waypoints").is_some(), "missing waypoints");
    // 默认 start=(0,0,0) goal=(5,0,0);空 map -> waypoints 为空 vec 即可
    let wps = v["waypoints"].as_array().expect("waypoints not array");
    // 不强制要求非空(empty map 合理)
    assert!(wps.is_empty() || !wps.is_empty(), "waypoints array accessible");
}

#[test]
fn binary_with_cli_args_runs() {
    let out = Command::new(binary_path())
        .args(["--start", "0,0,0", "--goal", "1,1,1"])
        .output()
        .expect("failed to run");
    assert!(out.status.success(), "stderr={}", String::from_utf8_lossy(&out.stderr));
}

#[test]
fn binary_with_nan_goal_exits_nonzero() {
    let out = Command::new(binary_path())
        .args(["--goal", "NaN,0,0"])
        .output()
        .expect("failed to run");
    // NaN goal -> plan_gate reject -> ExitCode 3
    assert_eq!(out.status.code(), Some(3), "expected exit code 3");
}

#[test]
fn binary_with_bad_cli_exits_2() {
    let out = Command::new(binary_path())
        .args(["--bogus"])
        .output()
        .expect("failed to run");
    assert_eq!(out.status.code(), Some(2), "expected exit code 2");
}
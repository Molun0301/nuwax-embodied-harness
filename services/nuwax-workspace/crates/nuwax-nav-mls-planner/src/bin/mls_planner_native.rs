//! `mls_planner_native` — 自包含 LCM worker 占位 binary
//!
//! **P0-16 重构后的实装策略**(rework22 Lane D):
//! - Phase A 简化版(本 binary):dev 不链 liblcm,使用 mock odom 输入跑 MLS core,
//!   把规划结果以 JSON 单行打印到 stdout(`plan(...)` 输出 waypoints)。
//! - Phase B (后续):接入 nuwax-lcm 真实订阅,在 `src/lcm_worker.rs` 提供
//!   `LatestOnly` stamp pairing + NaN cancel gate(已写在 `stamp_pair`/`nan_cancel`
//!   模块中,dev 自测通过该模块函数验证)。
//!
//! 标准用法:`cargo run --bin mls_planner_native --` 后接可选 mock 参数：
//! `--start x,y,z --goal x,y,z`；默认 start=(0,0,0) goal=(5,0,0)。

use std::process::ExitCode;

use nuwax_nav_mls_planner::mls_planner::{Config, Planner};
use nuwax_nav_mls_planner::stamp_pair::{stamp_pair_latest_only, NaNCancelError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default)]
struct CliArgs {
    start: (f32, f32, f32),
    goal: (f32, f32, f32),
}

fn parse_arg(s: &str) -> Option<(f32, f32, f32)> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 3 {
        return None;
    }
    Some((
        parts[0].parse().ok()?,
        parts[1].parse().ok()?,
        parts[2].parse().ok()?,
    ))
}

fn parse_cli() -> Result<CliArgs, String> {
    let mut args = CliArgs::default();
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--start" => {
                let v = it.next().ok_or("--start requires value")?;
                args.start = parse_arg(&v).ok_or_else(|| format!("bad --start: {}", v))?;
            }
            "--goal" => {
                let v = it.next().ok_or("--goal requires value")?;
                args.goal = parse_arg(&v).ok_or_else(|| format!("bad --goal: {}", v))?;
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other => return Err(format!("unknown flag: {}", other)),
        }
    }
    Ok(args)
}

fn print_usage() {
    eprintln!(
        "usage: mls_planner_native [--start x,y,z] [--goal x,y,z]\n\
         default start=(0,0,0) goal=(5,0,0)\n\
         prints a single JSON line of the planned waypoints"
    );
}

/// Binary 输出 JSON
#[derive(Debug, Clone, Serialize, Deserialize)]
struct PlanOutput {
    /// 输入 start
    pub start: (f32, f32, f32),
    /// 输入 goal
    pub goal: (f32, f32, f32),
    /// 规划结果（空表示未找到）
    pub waypoints: Vec<(f32, f32, f32)>,
}

/// mock odom 输入（最新帧仅）— Phase A 简化占位
fn mock_odom(start: (f32, f32, f32)) -> (f32, f32, f32) {
    start
}

/// Plan gate:NaN cancel + stamp pairing(latest-only)。
///
/// `odom_stamp` 与 `map_stamp` 在 Phase B 走真实 LCM frame header；
/// 本 binary 简化使用 `(odom_stamp, map_stamp) = (1, 1)` 的"最新帧"语义以保证 dev 可运行。
fn plan_gate(
    odom: (f32, f32, f32),
    goal: (f32, f32, f32),
    planner: &mut Planner,
    config: &Config,
) -> Result<Vec<(f32, f32, f32)>, NaNCancelError> {
    // stamp pairing:仅在 odom 和 map 都到最新帧后才规划
    if stamp_pair_latest_only(1, 1).is_none() {
        return Err(NaNCancelError::StampMismatch);
    }
    // NaN cancel
    if odom.0.is_nan() || odom.1.is_nan() || odom.2.is_nan()
        || goal.0.is_nan() || goal.1.is_nan() || goal.2.is_nan()
    {
        return Err(NaNCancelError::NaN);
    }
    Ok(planner.plan(odom, goal, config).unwrap_or_default())
}

fn default_config() -> Config {
    Config {
        world_frame: "map".to_string(),
        voxel_size: 0.1,
        robot_height: 0.5,
        surface_closing_radius: 0.3,
        node_spacing_m: 1.0,
        wall_clearance_m: 0.0,
        wall_buffer_m: 0.3,
        wall_buffer_weight: 1.0,
        step_threshold_m: 0.25,
        step_penalty_weight: 0.0,
        goal_tolerance: 0.3,
        viz_publish_hz: 2.0,
    }
}

fn main() -> ExitCode {
    let cli = match parse_cli() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{}", e);
            print_usage();
            return ExitCode::from(2);
        }
    };
    let odom = mock_odom(cli.start);
    let config = default_config();
    let mut planner = Planner::default();
    let waypoints = match plan_gate(odom, cli.goal, &mut planner, &config) {
        Ok(w) => w,
        Err(e) => {
            eprintln!("plan gate rejected: {:?}", e);
            return ExitCode::from(3);
        }
    };
    let out = PlanOutput {
        start: cli.start,
        goal: cli.goal,
        waypoints,
    };
    match serde_json::to_string(&out) {
        Ok(s) => {
            println!("{}", s);
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("serde_json encode failed: {}", e);
            ExitCode::from(4)
        }
    }
}
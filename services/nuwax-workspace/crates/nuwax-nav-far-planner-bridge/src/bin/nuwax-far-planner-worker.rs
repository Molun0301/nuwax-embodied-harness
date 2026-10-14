use std::collections::BTreeMap;
use std::time::Duration;

use nuwax_nav_far_planner_bridge::core::FarCoreConfig;
use nuwax_nav_far_planner_bridge::worker::{run_worker, WorkerOptions, WorkerTopics};

#[tokio::main]
async fn main() {
    if let Err(error) = run().await {
        eprintln!("{error}");
        std::process::exit(2);
    }
}

async fn run() -> Result<(), String> {
    let args = parse_args(std::env::args().skip(1))?;
    reject_unsupported_args(&args)?;
    let defaults = WorkerTopics::default();
    let topics = WorkerTopics {
        goal: value_or(&args, "goal", defaults.goal),
        cancel: value_or(&args, "stop_movement", defaults.cancel),
        odometry: value_or(&args, "odometry", defaults.odometry),
        terrain_map_ext: value_or(&args, "terrain_map_ext", defaults.terrain_map_ext),
        terrain_map: value_or(&args, "terrain_map", defaults.terrain_map),
        registered_scan: value_or(&args, "registered_scan", defaults.registered_scan),
        planner_result: value_or(&args, "goal_path", defaults.planner_result),
        worker_event: value_or(&args, "way_point", defaults.worker_event),
    };

    let planner_generation = required_env_i64("NUWAX_PLANNER_GENERATION")?;
    let timeout_ms = optional_env_u64("NUWAX_FAR_PLAN_TIMEOUT_MS")?.unwrap_or(2_000);
    if timeout_ms == 0 {
        return Err("NUWAX_FAR_PLAN_TIMEOUT_MS must be positive".into());
    }
    let clearance =
        args.get("robot_dim")
            .map_or(Ok(FarCoreConfig::default().clearance), |value| {
                value
                    .parse::<f64>()
                    .map_err(|error| format!("invalid --robot_dim: {error}"))
            })?;
    let samples_per_obstacle = optional_env_usize("NUWAX_FAR_SAMPLES_PER_OBSTACLE")?
        .unwrap_or(FarCoreConfig::default().samples_per_obstacle);
    let provider_url = std::env::var("LCM_DEFAULT_URL")
        .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=0".into());
    let backend_id =
        std::env::var("NUWAX_FAR_BACKEND_ID").unwrap_or_else(|_| "far-visibility-graph".into());

    run_worker(WorkerOptions {
        provider_url,
        topics,
        planner: FarCoreConfig {
            clearance,
            samples_per_obstacle,
        },
        plan_timeout: Duration::from_millis(timeout_ms),
        planner_generation,
        backend_id,
    })
    .await
    .map_err(|error| error.to_string())
}

fn reject_unsupported_args(args: &BTreeMap<String, String>) -> Result<(), String> {
    for name in args.keys() {
        if !matches!(
            name.as_str(),
            "goal"
                | "stop_movement"
                | "odometry"
                | "terrain_map_ext"
                | "terrain_map"
                | "registered_scan"
                | "goal_path"
                | "way_point"
                | "robot_dim"
        ) {
            return Err(format!("unsupported FAR worker option --{name}"));
        }
    }
    Ok(())
}

fn parse_args(args: impl IntoIterator<Item = String>) -> Result<BTreeMap<String, String>, String> {
    let mut parsed = BTreeMap::new();
    let mut args = args.into_iter();
    while let Some(flag) = args.next() {
        let Some(name) = flag.strip_prefix("--") else {
            return Err(format!("unexpected positional argument {flag}"));
        };
        if name.is_empty() {
            return Err("empty option name".into());
        }
        let value = args
            .next()
            .ok_or_else(|| format!("missing value for {flag}"))?;
        if parsed.insert(name.to_string(), value).is_some() {
            return Err(format!("duplicate option {flag}"));
        }
    }
    Ok(parsed)
}

fn value_or(args: &BTreeMap<String, String>, key: &str, default: String) -> String {
    args.get(key).cloned().unwrap_or(default)
}

fn required_env_i64(name: &str) -> Result<i64, String> {
    let value = std::env::var(name).map_err(|_| format!("{name} is required"))?;
    let parsed = value
        .parse::<i64>()
        .map_err(|error| format!("invalid {name}: {error}"))?;
    if parsed <= 0 {
        return Err(format!("{name} must be positive"));
    }
    Ok(parsed)
}

fn optional_env_u64(name: &str) -> Result<Option<u64>, String> {
    std::env::var(name)
        .ok()
        .map(|value| {
            value
                .parse::<u64>()
                .map_err(|error| format!("invalid {name}: {error}"))
        })
        .transpose()
}

fn optional_env_usize(name: &str) -> Result<Option<usize>, String> {
    std::env::var(name)
        .ok()
        .map(|value| {
            value
                .parse::<usize>()
                .map_err(|error| format!("invalid {name}: {error}"))
        })
        .transpose()
}

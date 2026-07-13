//! Standalone real-LCM process owner for the MLS planner.

use std::process::ExitCode;

use nuwax_nav_mls_planner::lcm_worker::{run_worker, WorkerOptions};

fn value(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    args.next()
        .ok_or_else(|| format!("{flag} requires a value"))
}

fn parse() -> Result<Option<WorkerOptions>, String> {
    let mut options = WorkerOptions::default();
    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        let target = match flag.as_str() {
            "--provider" => &mut options.provider_url,
            "--global_map" | "--global-map" => &mut options.topics.global_map,
            "--local_map" | "--local-map" => &mut options.topics.local_map,
            "--region_bounds" | "--region-bounds" => &mut options.topics.region_bounds,
            "--start_pose" | "--start-pose" => &mut options.topics.start_pose,
            "--goal_pose" | "--goal-pose" => &mut options.topics.goal_pose,
            "--path" => &mut options.topics.path,
            "--surface_map" | "--surface-map" => &mut options.topics.surface_map,
            "--nodes" => &mut options.topics.nodes,
            "--node_edges" | "--node-edges" => &mut options.topics.node_edges,
            "--world_frame" | "--world-frame" => &mut options.planner.world_frame,
            "-h" | "--help" => {
                print_usage();
                return Ok(None);
            }
            _ => {
                let number = value(&mut args, &flag)?
                    .parse::<f32>()
                    .map_err(|_| format!("{flag} requires a finite number"))?;
                if !number.is_finite() {
                    return Err(format!("{flag} requires a finite number"));
                }
                match flag.as_str() {
                    "--voxel_size" | "--voxel-size" => options.planner.voxel_size = number,
                    "--robot_height" | "--robot-height" => options.planner.robot_height = number,
                    "--surface_closing_radius" | "--surface-closing-radius" => {
                        options.planner.surface_closing_radius = number
                    }
                    "--node_spacing_m" | "--node-spacing-m" => {
                        options.planner.node_spacing_m = number
                    }
                    "--wall_clearance_m" | "--wall-clearance-m" => {
                        options.planner.wall_clearance_m = number
                    }
                    "--wall_buffer_m" | "--wall-buffer-m" => options.planner.wall_buffer_m = number,
                    "--wall_buffer_weight" | "--wall-buffer-weight" => {
                        options.planner.wall_buffer_weight = number
                    }
                    "--step_threshold_m" | "--step-threshold-m" => {
                        options.planner.step_threshold_m = number
                    }
                    "--step_penalty_weight" | "--step-penalty-weight" => {
                        options.planner.step_penalty_weight = number
                    }
                    "--goal_tolerance" | "--goal-tolerance" => {
                        options.planner.goal_tolerance = number
                    }
                    "--viz_publish_hz" | "--viz-publish-hz" => {
                        options.planner.viz_publish_hz = number
                    }
                    other => return Err(format!("unknown flag: {other}")),
                }
                continue;
            }
        };
        *target = value(&mut args, &flag)?;
    }
    Ok(Some(options))
}

fn print_usage() {
    println!(
        "usage: mls_planner_native [OPTIONS]\n\
         typed ports: --global_map --local_map --region_bounds --start_pose --goal_pose \\\n         --path --surface_map --nodes --node_edges\n\
         transport: --provider <LCM_URL> (or NUWAX_LCM_URL)\n\
         planner: --world_frame --voxel_size --robot_height --surface_closing_radius \\\n         --node_spacing_m --wall_clearance_m --wall_buffer_m --wall_buffer_weight \\\n         --step_threshold_m --step_penalty_weight --goal_tolerance --viz_publish_hz"
    );
}

async fn shutdown_signal() -> std::io::Result<()> {
    #[cfg(unix)]
    {
        let mut terminate =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
        tokio::select! {
            result = tokio::signal::ctrl_c() => result,
            _ = terminate.recv() => Ok(()),
        }
    }
    #[cfg(not(unix))]
    {
        tokio::signal::ctrl_c().await
    }
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> ExitCode {
    let Some(options) = (match parse() {
        Ok(options) => options,
        Err(error) => {
            eprintln!("{error}");
            print_usage();
            return ExitCode::from(2);
        }
    }) else {
        return ExitCode::SUCCESS;
    };

    tokio::select! {
        result = run_worker(options) => match result {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!("MLS worker failed: {error}");
                ExitCode::from(3)
            }
        },
        signal = shutdown_signal() => match signal {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!("failed to install shutdown signal: {error}");
                ExitCode::from(4)
            }
        }
    }
}

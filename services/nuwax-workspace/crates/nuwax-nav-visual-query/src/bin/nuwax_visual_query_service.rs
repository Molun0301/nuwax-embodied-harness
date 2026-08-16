//! `nuwax-visual-query-service` 公共 JSONL 生产入口。

use nuwax_nav_visual_query::perception::PerceptionWorkerConfig;
use nuwax_nav_visual_query::service::{run_stdio_service, VisualQueryServiceConfig};
use std::path::PathBuf;
use std::time::Duration;

fn main() {
    let config = match parse_config(std::env::args().skip(1)) {
        Ok(Some(config)) => config,
        Ok(None) => return,
        Err(error) => {
            eprintln!("error: {error}");
            eprintln!("Try `--help` for usage.");
            std::process::exit(2);
        }
    };
    if let Err(error) = run_stdio_service(config) {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

fn parse_config(
    args: impl Iterator<Item = String>,
) -> Result<Option<VisualQueryServiceConfig>, String> {
    let mut repo_root = std::env::current_dir().map_err(|error| error.to_string())?;
    let mut python_program = None;
    let mut weights = None;
    let mut device = None;
    let mut timeout_ms = None;
    let mut max_command_bytes = None;

    for argument in args {
        if argument == "--help" || argument == "-h" {
            print_usage();
            return Ok(None);
        }
        let (name, value) = argument
            .split_once('=')
            .ok_or_else(|| format!("expected --name=value, got {argument}"))?;
        match name {
            "--repo-root" => repo_root = PathBuf::from(required_value(name, value)?),
            "--python" => python_program = Some(PathBuf::from(required_value(name, value)?)),
            "--weights" => weights = Some(required_value(name, value)?.to_string()),
            "--device" => device = Some(value.to_string()),
            "--timeout-ms" => {
                timeout_ms = Some(parse_positive_u64(name, value)?);
            }
            "--max-command-bytes" => {
                max_command_bytes = Some(parse_positive_usize(name, value)?);
            }
            _ => return Err(format!("unknown option {name}")),
        }
    }

    let mut worker = PerceptionWorkerConfig::for_repo(repo_root);
    if let Some(value) = python_program {
        worker.python_program = value;
    }
    if let Some(value) = weights {
        worker.weights = value;
    }
    if let Some(value) = device {
        worker.device = value;
    }
    if let Some(value) = timeout_ms {
        worker.timeout = Duration::from_millis(value);
    }
    let mut service = VisualQueryServiceConfig::new(worker);
    if let Some(value) = max_command_bytes {
        service.max_command_bytes = value;
    }
    Ok(Some(service))
}

fn required_value<'a>(name: &str, value: &'a str) -> Result<&'a str, String> {
    if value.is_empty() {
        Err(format!("{name} requires a non-empty value"))
    } else {
        Ok(value)
    }
}

fn parse_positive_u64(name: &str, value: &str) -> Result<u64, String> {
    let parsed = value
        .parse::<u64>()
        .map_err(|_| format!("{name} requires a positive integer"))?;
    if parsed == 0 {
        return Err(format!("{name} requires a positive integer"));
    }
    Ok(parsed)
}

fn parse_positive_usize(name: &str, value: &str) -> Result<usize, String> {
    let parsed = value
        .parse::<usize>()
        .map_err(|_| format!("{name} requires a positive integer"))?;
    if parsed == 0 {
        return Err(format!("{name} requires a positive integer"));
    }
    Ok(parsed)
}

fn print_usage() {
    println!(
        "Usage: nuwax-visual-query-service [OPTIONS]\n\
         \n\
         JSONL stdin commands:\n\
           query    request_id, session_id, target_id, source_timestamp_ns,\n\
                    frame_id, calibration_id, camera_info, width, height,\n\
                    object_description, image\n\
           cancel   request_id\n\
           shutdown\n\
         \n\
         Options:\n\
           --repo-root=PATH          Repository root (default: current directory)\n\
           --python=PATH             Python executable\n\
           --weights=PATH_OR_NAME    YOLO-World weights\n\
           --device=DEVICE           Ultralytics device selector\n\
           --timeout-ms=N            Per-query and preflight hard deadline\n\
           --max-command-bytes=N     Maximum JSONL command bytes\n\
           -h, --help                Show this help"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_rejects_zero_timeout() {
        let error = parse_config(["--timeout-ms=0".to_string()].into_iter()).unwrap_err();
        assert!(error.contains("positive integer"));
    }

    #[test]
    fn cli_builds_production_worker_config() {
        let config = parse_config(
            [
                "--repo-root=/tmp/repo".to_string(),
                "--python=/usr/bin/python3".to_string(),
                "--weights=/tmp/model.pt".to_string(),
                "--device=cpu".to_string(),
                "--timeout-ms=25".to_string(),
            ]
            .into_iter(),
        )
        .unwrap()
        .unwrap();
        assert_eq!(config.worker.repo_root, PathBuf::from("/tmp/repo"));
        assert_eq!(
            config.worker.python_program,
            PathBuf::from("/usr/bin/python3")
        );
        assert_eq!(config.worker.weights, "/tmp/model.pt");
        assert_eq!(config.worker.device, "cpu");
        assert_eq!(config.worker.timeout, Duration::from_millis(25));
    }
}

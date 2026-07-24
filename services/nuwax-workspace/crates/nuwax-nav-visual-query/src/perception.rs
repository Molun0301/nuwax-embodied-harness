//! Repository-aware configuration for the production YOLO-World worker.

use crate::subprocess::{
    run_bounded_process, SubprocessModelConfig, SubprocessVlBBoxModel, STDERR_LIMIT,
};
use crate::VisualQueryError;
use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

/// Configuration for `services/perception/visual_query_worker.py`.
#[derive(Debug, Clone)]
pub struct PerceptionWorkerConfig {
    /// Repository root containing `services/perception`.
    pub repo_root: PathBuf,
    /// Python executable or PATH-resolved program name.
    pub python_program: PathBuf,
    /// YOLO-World `.pt`, TensorRT `.engine`, or Ultralytics model name.
    pub weights: String,
    /// Device selector accepted by Ultralytics, such as `cuda:0` or `cpu`.
    pub device: String,
    /// Detection confidence threshold.
    pub confidence: f32,
    /// NMS IoU threshold.
    pub iou: f32,
    /// Model inference image size.
    pub image_size: u32,
    /// Maximum detections requested from the backend.
    pub max_detections: u32,
    /// Hard one-query process deadline.
    pub timeout: Duration,
    /// Maximum accepted worker stdout bytes.
    pub max_response_bytes: usize,
    /// Maximum raw image bytes accepted by one query.
    pub max_image_bytes: usize,
    /// Maximum decoded image pixels accepted by one query.
    pub max_image_pixels: u64,
    /// Maximum serialized JSON request bytes.
    pub max_request_bytes: usize,
}

impl PerceptionWorkerConfig {
    /// Constructs production defaults rooted at `repo_root`.
    #[must_use]
    pub fn for_repo(repo_root: impl Into<PathBuf>) -> Self {
        let repo_root = repo_root.into();
        let repo_python = repo_root.join(".venv/bin/python");
        Self {
            repo_root,
            python_program: if repo_python.is_file() {
                repo_python
            } else {
                PathBuf::from("python3")
            },
            weights: "yolov8s-worldv2.pt".to_string(),
            device: String::new(),
            confidence: 0.25,
            iou: 0.45,
            image_size: 640,
            max_detections: 100,
            timeout: Duration::from_secs(30),
            max_response_bytes: 16 * 1024,
            max_image_bytes: 16 * 1024 * 1024,
            max_image_pixels: 16 * 1024 * 1024,
            max_request_bytes: 64 * 1024 * 1024,
        }
    }

    /// Absolute or repository-relative worker path.
    #[must_use]
    pub fn worker_path(&self) -> PathBuf {
        self.repo_root
            .join("services/perception/visual_query_worker.py")
    }

    /// Validates repository layout and non-download configuration invariants.
    pub fn preflight(&self) -> Result<(), VisualQueryError> {
        if !self.worker_path().is_file() {
            return Err(model_error(format!(
                "visual query worker is missing: {}",
                self.worker_path().display()
            )));
        }
        let backend = self.repo_root.join("services/perception/detectors.py");
        if !backend.is_file() {
            return Err(model_error(format!(
                "production detector backend is missing: {}",
                backend.display()
            )));
        }
        if self.python_program.as_os_str().is_empty() {
            return Err(model_error("python program path is empty"));
        }
        if self.weights.trim().is_empty() {
            return Err(model_error("YOLO-World weights are empty"));
        }
        let weights = Path::new(&self.weights);
        if (weights.is_absolute()
            || weights
                .parent()
                .is_some_and(|parent| parent != Path::new("")))
            && !weights.is_file()
        {
            return Err(model_error(format!(
                "configured YOLO-World weights are missing: {}",
                weights.display()
            )));
        }
        if !(0.0..=1.0).contains(&self.confidence) || !self.confidence.is_finite() {
            return Err(model_error("confidence must be finite and within [0, 1]"));
        }
        if !(0.0..=1.0).contains(&self.iou) || !self.iou.is_finite() {
            return Err(model_error("iou must be finite and within [0, 1]"));
        }
        if self.image_size < 32 || self.max_detections == 0 {
            return Err(model_error(
                "image_size must be >= 32 and max_detections positive",
            ));
        }
        Ok(())
    }

    /// Builds the existing generic subprocess configuration for this concrete worker.
    pub fn subprocess_config(&self) -> Result<SubprocessModelConfig, VisualQueryError> {
        self.preflight()?;
        let config = SubprocessModelConfig {
            program: self.python_program.clone(),
            args: vec![
                self.worker_path().into_os_string(),
                OsString::from("--weights"),
                OsString::from(&self.weights),
                OsString::from("--device"),
                OsString::from(&self.device),
                OsString::from("--confidence"),
                OsString::from(self.confidence.to_string()),
                OsString::from("--iou"),
                OsString::from(self.iou.to_string()),
                OsString::from("--image-size"),
                OsString::from(self.image_size.to_string()),
                OsString::from("--max-detections"),
                OsString::from(self.max_detections.to_string()),
            ],
            timeout: self.timeout,
            max_response_bytes: self.max_response_bytes,
            max_image_bytes: self.max_image_bytes,
            max_image_pixels: self.max_image_pixels,
            max_request_bytes: self.max_request_bytes,
        };
        config.validate()?;
        Ok(config)
    }

    /// Builds the concrete visual bbox model using the existing subprocess runner.
    pub fn build_model(&self) -> Result<SubprocessVlBBoxModel, VisualQueryError> {
        SubprocessVlBBoxModel::new(self.subprocess_config()?)
    }

    /// Executes the worker's import-only production backend preflight without loading weights.
    pub fn run_process_preflight(&self) -> Result<String, VisualQueryError> {
        self.preflight()?;
        let mut command = Command::new(&self.python_program);
        command
            .arg(self.worker_path())
            .arg("--weights")
            .arg(&self.weights)
            .arg("--device")
            .arg(&self.device)
            .arg("--preflight")
            .current_dir(&self.repo_root);
        let output = run_bounded_process(
            &mut command,
            Vec::new(),
            self.timeout,
            self.max_response_bytes,
            STDERR_LIMIT,
            "worker preflight",
        )?;
        if !output.status.success() {
            return Err(model_error(format!(
                "worker preflight failed with {}: {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            )));
        }
        String::from_utf8(output.stdout)
            .map_err(|error| model_error(format!("worker preflight stdout is not UTF-8: {error}")))
    }
}

fn model_error(message: impl Into<String>) -> VisualQueryError {
    VisualQueryError::ModelFailed(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::Instant;

    static TEMP_ID: AtomicU64 = AtomicU64::new(0);

    fn repo_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(4)
            .expect("crate must be inside repository/services/nuwax-workspace/crates")
            .to_path_buf()
    }

    #[cfg(unix)]
    fn fixture_config(
        script: &str,
        timeout: Duration,
        max_response_bytes: usize,
    ) -> PerceptionWorkerConfig {
        let root = std::env::temp_dir().join(format!(
            "nuwax-visual-preflight-{}-{}",
            std::process::id(),
            TEMP_ID.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(root.join("services/perception")).unwrap();
        fs::write(
            root.join("services/perception/detectors.py"),
            b"# fixture\n",
        )
        .unwrap();
        fs::write(
            root.join("services/perception/visual_query_worker.py"),
            script,
        )
        .unwrap();
        let mut config = PerceptionWorkerConfig::for_repo(root);
        config.python_program = PathBuf::from("/bin/sh");
        config.timeout = timeout;
        config.max_response_bytes = max_response_bytes;
        config
    }

    #[test]
    fn repo_builder_targets_production_worker_with_runtime_parameters() {
        let mut config = PerceptionWorkerConfig::for_repo(repo_root());
        config.python_program = PathBuf::from("python3");
        config.weights = "yolov8s-worldv2.pt".into();
        config.device = "cpu".into();
        config.confidence = 0.4;
        let process = config.subprocess_config().unwrap();
        assert_eq!(process.program, PathBuf::from("python3"));
        assert_eq!(process.args[0], config.worker_path().into_os_string());
        assert!(process.args.iter().any(|arg| arg == "yolov8s-worldv2.pt"));
        assert!(process.args.iter().any(|arg| arg == "0.4"));
    }

    #[test]
    fn repo_builder_prefers_present_repo_venv_and_otherwise_uses_python3() {
        let root =
            std::env::temp_dir().join(format!("nuwax-visual-query-python-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        fs::create_dir_all(root.join(".venv/bin")).unwrap();
        fs::write(root.join(".venv/bin/python"), b"").unwrap();
        assert_eq!(
            PerceptionWorkerConfig::for_repo(&root).python_program,
            root.join(".venv/bin/python")
        );
        fs::remove_dir_all(&root).unwrap();
        assert_eq!(
            PerceptionWorkerConfig::for_repo(&root).python_program,
            PathBuf::from("python3")
        );
    }

    #[test]
    fn preflight_rejects_missing_repo_and_explicit_missing_weights() {
        let missing_repo = PerceptionWorkerConfig::for_repo("/definitely/missing/nuwax-repo");
        assert!(missing_repo.preflight().is_err());

        let mut missing_weights = PerceptionWorkerConfig::for_repo(repo_root());
        missing_weights.weights = "/definitely/missing/yolo-world.pt".into();
        assert!(missing_weights.preflight().is_err());
    }

    #[test]
    fn worker_process_preflight_imports_production_backend_without_loading_weights() {
        let root = repo_root();
        let mut config = PerceptionWorkerConfig::for_repo(&root);
        let model_test_python = root.join(".venv_model_download/bin/python");
        if config.python_program == Path::new("python3") && model_test_python.is_file() {
            config.python_program = model_test_python;
        }
        let output = config.run_process_preflight().unwrap();
        assert!(output.contains("services.perception.detectors.YoloWorldDetectorBackend"));
        assert!(output.contains("\"status\":\"ready\""));
    }

    #[cfg(unix)]
    #[test]
    fn process_preflight_has_a_hard_child_deadline() {
        let config = fixture_config("sleep 5\n", Duration::from_millis(40), 1024);
        let started = Instant::now();
        let error = config.run_process_preflight().unwrap_err().to_string();
        assert!(error.contains("worker preflight timed out"), "{error}");
        assert!(started.elapsed() < Duration::from_secs(1));
        fs::remove_dir_all(config.repo_root).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn process_preflight_bounds_both_output_streams() {
        let stdout_config = fixture_config("head -c 64 /dev/zero\n", Duration::from_secs(1), 8);
        let stdout_error = stdout_config
            .run_process_preflight()
            .unwrap_err()
            .to_string();
        assert!(
            stdout_error.contains("stdout exceeded 8 bytes"),
            "{stdout_error}"
        );
        fs::remove_dir_all(stdout_config.repo_root).unwrap();

        let stderr_config =
            fixture_config("head -c 9000 /dev/zero >&2\n", Duration::from_secs(1), 1024);
        let stderr_error = stderr_config
            .run_process_preflight()
            .unwrap_err()
            .to_string();
        assert!(
            stderr_error.contains("stderr exceeded 8192 bytes"),
            "{stderr_error}"
        );
        fs::remove_dir_all(stderr_config.repo_root).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn process_preflight_deadline_includes_inherited_output_pipes() {
        let config = fixture_config(
            "sleep 5 & printf '{\"status\":\"ready\"}'\n",
            Duration::from_millis(40),
            1024,
        );
        let started = Instant::now();
        let error = config.run_process_preflight().unwrap_err().to_string();
        assert!(error.contains("worker preflight timed out"), "{error}");
        assert!(started.elapsed() < Duration::from_secs(1));
        fs::remove_dir_all(config.repo_root).unwrap();
    }
}

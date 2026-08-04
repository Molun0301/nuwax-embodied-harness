use std::fs;
use std::io::Read;
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Output, Stdio};
use std::sync::{
    atomic::{AtomicU64, Ordering},
    Arc, Mutex,
};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[cfg(feature = "nuwax-lio-test-child")]
use futures::StreamExt;
#[cfg(feature = "nuwax-lio-test-child")]
use nuwax_codec::lcm::{lio, native_pgo, schema};
#[cfg(feature = "nuwax-lio-test-child")]
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame};
#[cfg(feature = "nuwax-lio-test-child")]
use nuwax_transport::{LcmFrameStream, LcmTransport};
#[cfg(feature = "nuwax-lio-test-child")]
use prost::Message;

#[cfg(feature = "nuwax-lio-test-child")]
mod native_fixture_proto {
    tonic::include_proto!("nuwax.geometry");

    impl std::fmt::Debug for ExplorationRecoveryGuardProof {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter.write_str("ExplorationRecoveryGuardProof([REDACTED])")
        }
    }

    impl std::fmt::Debug for RecoverExplorationAfterNavigationRolloverRequest {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter.write_str("RecoverExplorationAfterNavigationRolloverRequest([REDACTED])")
        }
    }

    impl std::fmt::Debug for ReconcileExplorationAfterGlobalStopRequest {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter.write_str("ReconcileExplorationAfterGlobalStopRequest([REDACTED])")
        }
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
use std::os::unix::process::ExitStatusExt;
#[cfg(unix)]
use std::{
    ffi::OsString,
    os::unix::{
        ffi::OsStringExt,
        fs::{MetadataExt, PermissionsExt},
    },
};

const EXIT_TIMEOUT: Duration = Duration::from_secs(3);
const DEFAULT_CLOUD_MAP_SHM_PATH: &str = "/tmp/nuwax_cloudmap.mmap";
const DEFAULT_OCCUPANCY_SHM_PATH: &str = "/tmp/nuwax_occupancy.mmap";
const PRODUCTION_LCM_PROVIDER: &str = "udpm://239.255.76.67:7667?ttl=1";
const INVALID_PREFLIGHT_LCM_PROVIDER: &str = "invalid://preflight-must-win";
const SIDE_EFFECT_ARTIFACTS: [&str; 7] = [
    "geometry.ready",
    "sequence-state.json",
    "exploration-ownership-journal.json",
    "video.shm",
    "video.sock",
    "occupancy.mmap",
    "cloud-map.mmap",
];
const STARTUP_ENV_KEYS: &[&str] = &[
    "NUWAX_GEOMETRY_LIO_BACKEND",
    "NUWAX_GEOMETRY_MAPPING_BACKEND",
    "NUWAX_GEOMETRY_CORRECTION_BACKEND",
    "NUWAX_GEOMETRY_CANONICAL_MAP_WRITER",
    "NUWAX_GEOMETRY_CANONICAL_CORRECTION_WRITER",
    "NUWAX_MAPPING_RAYTRACING_VOXEL_SIZE",
    "NUWAX_MAPPING_RAYTRACING_MAX_RANGE",
    "NUWAX_MAPPING_RAYTRACING_RAY_SUBSAMPLE",
    "NUWAX_MAPPING_RAYTRACING_SHADOW_DEPTH",
    "NUWAX_MAPPING_RAYTRACING_GRACE_DEPTH",
    "NUWAX_MAPPING_RAYTRACING_MIN_HEALTH",
    "NUWAX_MAPPING_RAYTRACING_MAX_HEALTH",
    "NUWAX_MAPPING_RAYTRACING_GRAZE_COS",
    "NUWAX_MAPPING_RAYTRACING_RECENCY_WINDOW",
    "NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL",
    "NUWAX_LCM_CHANNEL_ODOM",
    "NUWAX_GEOMETRY_RUNTIME_TF_CHANNEL",
    "NUWAX_LCM_CHANNEL_TF",
    "NUWAX_GEOMETRY_RUNTIME_POINTCLOUD_CHANNEL",
    "NUWAX_LCM_CHANNEL_POINTCLOUD",
    "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL",
    "NUWAX_LCM_CHANNEL_CLOUD_MAP",
    "NUWAX_GEOMETRY_RUNTIME_OCCUPANCY_CHANNEL",
    "NUWAX_LCM_CHANNEL_OCCUPANCY",
    "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",
    "NUWAX_LCM_CHANNEL_CMD_VEL",
    "NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL",
    "NUWAX_LCM_CHANNEL_GOAL_POSE",
    "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL",
    "NUWAX_LCM_CHANNEL_NAV_STATE",
    "NUWAX_GEOMETRY_RUNTIME_FRONTIERS_CHANNEL",
    "NUWAX_LCM_CHANNEL_FRONTIER",
    "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
    "NUWAX_LCM_CHANNEL_NAV_CANCEL",
    "NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL",
    "NUWAX_LCM_CHANNEL_NAV_PATH",
    "NUWAX_GEOMETRY_MAP_FRAME",
    "NUWAX_MAP_FRAME",
    "NUWAX_GEOMETRY_ODOM_FRAME",
    "NUWAX_ODOM_FRAME",
    "NUWAX_BASE_FRAME",
    "NUWAX_GEOMETRY_BASE_FRAME",
    "NUWAX_GEOMETRY_LIDAR_FRAME",
    "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR",
    "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE",
    "NUWAX_GEOMETRY_NATIVE_LIO_PRIVATE_PREFIX",
    "NUWAX_GEOMETRY_NATIVE_LIO_FIRST_PAIR_TIMEOUT_MS",
    "NUWAX_GEOMETRY_NATIVE_LIO_PAIR_MAX_SKEW_MS",
    "NUWAX_GEOMETRY_NATIVE_LIO_HOST_IP",
    "NUWAX_GEOMETRY_NATIVE_LIO_LIDAR_IP",
    "NUWAX_LIO_TEST_CHILD_RECORDED_FIXTURE",
    "NUWAX_LIO_TEST_CHILD_PUBLISH_PERMIT_FILE",
    "NUWAX_RUNTIME_DATA_DIR",
];
static NEXT_DIR_ID: AtomicU64 = AtomicU64::new(0);

struct ProbeDir {
    path: PathBuf,
    lcm_url: String,
    runtime_port: u16,
    _runtime_port_guard: Option<TcpListener>,
}

impl ProbeDir {
    fn new(label: &str) -> Self {
        Self::with_runtime_port_guard(label, false)
    }

    fn fail_closed(label: &str) -> Self {
        Self::with_runtime_port_guard(label, true)
    }

    fn with_runtime_port_guard(label: &str, hold_runtime_port: bool) -> Self {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock must be after unix epoch")
            .as_nanos();
        let counter = NEXT_DIR_ID.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "nuwax-geometry-lcm-backend-{label}-{}-{nonce}-{counter}",
            std::process::id()
        ));
        fs::create_dir_all(&path).expect("create backend probe directory");
        #[cfg(unix)]
        fs::set_permissions(&path, fs::Permissions::from_mode(0o700))
            .expect("secure backend probe directory");
        let process_id = u64::from(std::process::id());
        let multicast_host = 1 + ((process_id + counter) % 254);
        let port = 20_000 + ((process_id * 97 + counter) % 40_000);
        let lcm_url = format!("udpm://239.255.77.{multicast_host}:{port}?ttl=0");
        let runtime_port_guard =
            TcpListener::bind(("127.0.0.1", 0)).expect("reserve runtime probe port");
        let runtime_port = runtime_port_guard
            .local_addr()
            .expect("read runtime probe port")
            .port();
        Self {
            path,
            lcm_url,
            runtime_port,
            _runtime_port_guard: hold_runtime_port.then_some(runtime_port_guard),
        }
    }

    fn path(&self) -> &Path {
        &self.path
    }

    fn lcm_url(&self) -> &str {
        &self.lcm_url
    }

    fn runtime_port(&self) -> u16 {
        self.runtime_port
    }
}

impl Drop for ProbeDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

struct CapturedChildOutput {
    stdout: Arc<Mutex<Vec<u8>>>,
    stderr: Arc<Mutex<Vec<u8>>>,
    drains: Vec<JoinHandle<()>>,
}

impl CapturedChildOutput {
    fn new(child: &mut Child) -> Self {
        let stdout = Arc::new(Mutex::new(Vec::new()));
        let stderr = Arc::new(Mutex::new(Vec::new()));
        let stdout_pipe = child.stdout.take().expect("child stdout must be piped");
        let stderr_pipe = child.stderr.take().expect("child stderr must be piped");
        let drains = vec![
            Self::spawn_drain(stdout_pipe, Arc::clone(&stdout)),
            Self::spawn_drain(stderr_pipe, Arc::clone(&stderr)),
        ];
        Self {
            stdout,
            stderr,
            drains,
        }
    }

    fn spawn_drain(
        mut pipe: impl Read + Send + 'static,
        captured: Arc<Mutex<Vec<u8>>>,
    ) -> JoinHandle<()> {
        std::thread::spawn(move || {
            let mut chunk = [0_u8; 8 * 1024];
            loop {
                match pipe.read(&mut chunk) {
                    Ok(0) => break,
                    Ok(count) => captured
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner())
                        .extend_from_slice(&chunk[..count]),
                    Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                    Err(_) => break,
                }
            }
        })
    }

    fn snapshot(&self) -> (Vec<u8>, Vec<u8>) {
        let stdout = self
            .stdout
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .clone();
        let stderr = self
            .stderr
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .clone();
        (stdout, stderr)
    }

    fn finish(mut self) -> (Vec<u8>, Vec<u8>) {
        for drain in self.drains.drain(..) {
            let _ = drain.join();
        }
        self.snapshot()
    }
}

impl Drop for CapturedChildOutput {
    fn drop(&mut self) {
        for drain in self.drains.drain(..) {
            let _ = drain.join();
        }
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ProcessGroupCleanupDisposition {
    ExitedWithoutHarnessSignal,
    TerminatedByHarness,
    KilledByHarness,
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
#[derive(Debug, PartialEq, Eq)]
struct ProcessGroupCleanupOutcome {
    pid: u32,
    scenario: String,
    disposition: ProcessGroupCleanupDisposition,
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
#[derive(Clone, Debug)]
struct TrackedProcessGroup {
    pid: u32,
    start_time: u64,
    scenario: String,
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
#[derive(Clone, Debug)]
struct NativePgoDiscovery {
    native_lio_pid: u32,
    executable: PathBuf,
    scenario: String,
}

struct ChildGuard {
    child: Option<Child>,
    output: Option<CapturedChildOutput>,
    #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
    tracked_process_groups: Vec<TrackedProcessGroup>,
    #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
    native_pgo_discovery: Option<NativePgoDiscovery>,
}

impl ChildGuard {
    fn new(mut child: Child) -> Self {
        let output = CapturedChildOutput::new(&mut child);
        Self {
            child: Some(child),
            output: Some(output),
            #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
            tracked_process_groups: Vec::new(),
            #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
            native_pgo_discovery: None,
        }
    }

    #[cfg(feature = "nuwax-lio-test-child")]
    fn empty() -> Self {
        Self {
            child: None,
            output: None,
            #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
            tracked_process_groups: Vec::new(),
            #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
            native_pgo_discovery: None,
        }
    }

    fn child_mut(&mut self) -> &mut Child {
        self.child.as_mut().expect("child guard must own a process")
    }

    #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
    fn discover_native_pgo_on_cleanup(
        &mut self,
        native_lio_pid: u32,
        executable: PathBuf,
        scenario: String,
    ) {
        self.native_pgo_discovery = Some(NativePgoDiscovery {
            native_lio_pid,
            executable,
            scenario,
        });
    }

    #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
    fn track_process_group(&mut self, pid: u32, start_time: u64, scenario: &str) {
        if self
            .tracked_process_groups
            .iter()
            .any(|tracked| tracked.pid == pid && tracked.start_time == start_time)
        {
            return;
        }
        self.tracked_process_groups.push(TrackedProcessGroup {
            pid,
            start_time,
            scenario: scenario.to_string(),
        });
    }

    #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
    fn discover_native_pgo_process_group(&mut self) -> Result<(), String> {
        let Some(discovery) = self.native_pgo_discovery.clone() else {
            return Ok(());
        };
        let Some(geometry_pid) = self.child.as_ref().map(Child::id) else {
            return Ok(());
        };
        for pid in observe_direct_child_pids(geometry_pid)? {
            if pid == discovery.native_lio_pid
                || observe_process_executable(pid)?.as_ref() != Some(&discovery.executable)
            {
                continue;
            }
            let start_time = observe_process_start_time(pid)?.ok_or_else(|| {
                format!(
                    "{}: native PGO child {pid} disappeared during cleanup discovery",
                    discovery.scenario
                )
            })?;
            self.track_process_group(pid, start_time, &discovery.scenario);
        }
        Ok(())
    }

    #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
    fn settle_tracked_process_groups(
        &mut self,
        self_exit_grace: Duration,
    ) -> Result<Vec<ProcessGroupCleanupOutcome>, String> {
        let tracked = std::mem::take(&mut self.tracked_process_groups);
        let mut outcomes = Vec::with_capacity(tracked.len());
        let mut failed = Vec::new();
        let mut errors = Vec::new();
        for process_group in tracked {
            match settle_tracked_process_group(&process_group, self_exit_grace) {
                Ok(disposition) => outcomes.push(ProcessGroupCleanupOutcome {
                    pid: process_group.pid,
                    scenario: process_group.scenario.clone(),
                    disposition,
                }),
                Err(error) => {
                    errors.push(error);
                    failed.push(process_group);
                }
            }
        }
        self.tracked_process_groups = failed;
        if errors.is_empty() {
            Ok(outcomes)
        } else {
            Err(errors.join("; "))
        }
    }

    fn collect(mut self) -> Output {
        #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
        assert!(
            self.tracked_process_groups.is_empty(),
            "tracked process groups must be settled before collecting the child"
        );
        let status = self
            .child
            .take()
            .expect("child guard must own a process")
            .wait()
            .expect("collect child status");
        let (stdout, stderr) = self
            .output
            .take()
            .expect("child guard must own output drains")
            .finish();
        Output {
            status,
            stdout,
            stderr,
        }
    }

    fn terminate_and_collect(mut self) -> Output {
        #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
        self.discover_native_pgo_process_group()
            .expect("discover native PGO child before forced Geometry cleanup");
        let child = self.child.as_mut().expect("child guard must own a process");
        if child
            .try_wait()
            .expect("poll child before termination")
            .is_none()
        {
            child.kill().expect("terminate child process");
        }
        #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
        self.settle_tracked_process_groups(NATIVE_PGO_PARENT_DEATH_GRACE)
            .expect("cleanup tracked process groups after forced Geometry cleanup");
        self.collect()
    }

    #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
    fn sigkill_and_collect_with_process_group_outcomes(
        mut self,
    ) -> (Output, Vec<ProcessGroupCleanupOutcome>) {
        self.discover_native_pgo_process_group()
            .expect("discover native PGO child before Geometry SIGKILL");
        let child = self.child.as_mut().expect("child guard must own a process");
        if child
            .try_wait()
            .expect("poll Geometry before SIGKILL")
            .is_none()
        {
            child.kill().expect("SIGKILL Geometry process");
        }
        let status = self
            .child
            .take()
            .expect("child guard must own a process")
            .wait()
            .expect("collect SIGKILLed Geometry status");
        let outcomes = self
            .settle_tracked_process_groups(NATIVE_PGO_PARENT_DEATH_GRACE)
            .expect("cleanup native PGO process group after Geometry SIGKILL");
        let (stdout, stderr) = self
            .output
            .take()
            .expect("child guard must own output drains")
            .finish();
        (
            Output {
                status,
                stdout,
                stderr,
            },
            outcomes,
        )
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
        if let Err(error) = self.discover_native_pgo_process_group() {
            eprintln!("failed to discover native PGO child during ChildGuard cleanup: {error}");
        }
        if let Some(child) = self.child.as_mut() {
            if !matches!(child.try_wait(), Ok(Some(_))) {
                let _ = child.kill();
            }
            let _ = child.wait();
        }
        #[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
        match self.settle_tracked_process_groups(NATIVE_PGO_PARENT_DEATH_GRACE) {
            Ok(outcomes) => {
                for outcome in outcomes {
                    if outcome.disposition
                        != ProcessGroupCleanupDisposition::ExitedWithoutHarnessSignal
                    {
                        eprintln!(
                            "{}: ChildGuard cleaned native PGO process group {} after {:?}",
                            outcome.scenario, outcome.pid, outcome.disposition
                        );
                    }
                }
            }
            Err(error) => eprintln!("ChildGuard native PGO cleanup failed: {error}"),
        }
        let Some(output) = self.output.take() else {
            return;
        };
        let (stdout, stderr) = output.finish();
        if std::thread::panicking() {
            eprintln!(
                "child stdout retained during panic:\n{}\nchild stderr retained during panic:\n{}",
                String::from_utf8_lossy(&stdout),
                String::from_utf8_lossy(&stderr)
            );
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
struct FileMetadataSnapshot {
    len: u64,
    modified: Option<SystemTime>,
    is_file: bool,
    #[cfg(unix)]
    inode: u64,
}

fn file_metadata_snapshot(path: &Path) -> Option<FileMetadataSnapshot> {
    match fs::metadata(path) {
        Ok(metadata) => Some(FileMetadataSnapshot {
            len: metadata.len(),
            modified: metadata.modified().ok(),
            is_file: metadata.file_type().is_file(),
            #[cfg(unix)]
            inode: metadata.ino(),
        }),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => None,
        Err(error) => panic!("snapshot {} metadata: {error}", path.display()),
    }
}

struct SeededStartupArtifacts {
    files: Vec<(PathBuf, Vec<u8>, FileMetadataSnapshot)>,
}

impl SeededStartupArtifacts {
    fn new(probe: &ProbeDir, scenario: &str) -> Self {
        let files = SIDE_EFFECT_ARTIFACTS
            .into_iter()
            .map(|name| {
                let path = probe.path().join(name);
                let content = format!("sentinel::{scenario}::{name}").into_bytes();
                fs::write(&path, &content)
                    .unwrap_or_else(|error| panic!("seed {}: {error}", path.display()));
                let metadata = file_metadata_snapshot(&path).expect("seeded artifact metadata");
                (path, content, metadata)
            })
            .collect();
        Self { files }
    }

    fn assert_unchanged(&self, scenario: &str) {
        for (path, expected, expected_metadata) in &self.files {
            let metadata = fs::symlink_metadata(path)
                .unwrap_or_else(|error| panic!("{scenario}: stat {}: {error}", path.display()));
            assert!(
                metadata.file_type().is_file(),
                "{scenario}: {} was replaced with another file type",
                path.display()
            );
            assert_eq!(
                fs::read(path)
                    .unwrap_or_else(|error| panic!("{scenario}: read {}: {error}", path.display())),
                *expected,
                "{scenario}: startup mutated {}",
                path.display()
            );
            assert_eq!(
                file_metadata_snapshot(path).as_ref(),
                Some(expected_metadata),
                "{scenario}: startup changed metadata for {}",
                path.display()
            );
        }
    }
}

fn runtime_command(backend: &str, probe: &ProbeDir) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_nuwax-geometry-runtime"));
    for key in STARTUP_ENV_KEYS {
        command.env_remove(key);
    }
    command
        .env("NUWAX_LCM_BACKEND", backend)
        .env("NUWAX_LCM_URL", probe.lcm_url())
        .env(
            "NUWAX_GEOMETRY_RUNTIME_PORT",
            probe.runtime_port().to_string(),
        )
        .env(
            "NUWAX_GEOMETRY_RUNTIME_READY_FILE",
            probe.path().join("geometry.ready"),
        )
        .env(
            "NUWAX_GEOMETRY_SEQUENCE_STATE_PATH",
            probe.path().join("sequence-state.json"),
        )
        .env(
            "NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH",
            probe.path().join("exploration-ownership-journal.json"),
        )
        .env("NUWAX_VIDEO_SHM_PATH", probe.path().join("video.shm"))
        .env("NUWAX_VIDEO_UDS_PATH", probe.path().join("video.sock"))
        .env(
            "NUWAX_OCCUPANCY_SHM_PATH",
            probe.path().join("occupancy.mmap"),
        )
        .env(
            "NUWAX_CLOUDMAP_SHM_PATH",
            probe.path().join("cloud-map.mmap"),
        )
        .env(
            "NUWAX_FORBIDDEN_ZONES_PATH",
            probe.path().join("forbidden-zones.json"),
        )
        .env("NUWAX_REAL_LCM_REQUIRED", "true")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    command
}

fn assert_preflight_failure(
    command: &mut Command,
    artifacts: &SeededStartupArtifacts,
    scenario: &str,
    expected_detail: &str,
) {
    command.env("NUWAX_LCM_URL", INVALID_PREFLIGHT_LCM_PROVIDER);
    let child = ChildGuard::new(command.spawn().expect("spawn preflight failure probe"));
    let output = collect_bounded_output(child, scenario);
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert_eq!(output.status.code(), Some(1), "{scenario}: {stderr}");
    assert!(
        stderr.contains(expected_detail),
        "{scenario} lost expected preflight detail {expected_detail:?}: {stderr}"
    );
    assert!(
        !stderr.contains("LCM_BACKEND_UNAVAILABLE"),
        "{scenario} reached LCM transport before preflight: {stderr}"
    );
    assert!(
        !stdout.contains("启动 nuwax-geometry-runtime")
            && !stdout.contains("Geometry owner profile selected"),
        "{scenario} initialized tracing/runtime before preflight: {stdout}"
    );
    artifacts.assert_unchanged(scenario);
}

#[cfg(unix)]
fn assert_native_preflight_failure(
    command: &mut Command,
    artifacts: &SeededStartupArtifacts,
    scenario: &str,
    expected_detail: &str,
) {
    let child = ChildGuard::new(command.spawn().expect("spawn native preflight probe"));
    let output = collect_bounded_output(child, scenario);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert_eq!(output.status.code(), Some(1), "{scenario}: {stderr}");
    assert!(
        stderr.contains(expected_detail),
        "{scenario} lost expected native preflight detail {expected_detail:?}: {stderr}"
    );
    artifacts.assert_unchanged(scenario);
}

#[cfg(unix)]
fn native_preflight_command(probe: &ProbeDir) -> Command {
    let executable = Path::new("/bin/sh")
        .canonicalize()
        .expect("canonical native preflight executable");
    let mut command = runtime_command("real-lcm", probe);
    command
        .env("NUWAX_GEOMETRY_LIO_BACKEND", "fastlio2")
        .env("NUWAX_GEOMETRY_MAPPING_BACKEND", "voxel")
        .env("NUWAX_GEOMETRY_CORRECTION_BACKEND", "none")
        .env("NUWAX_GEOMETRY_CANONICAL_MAP_WRITER", "geometry")
        .env("NUWAX_GEOMETRY_CANONICAL_CORRECTION_WRITER", "geometry")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE", executable)
        .env(
            "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR",
            concat!(
                r#"{"translation_m":[0.0,0.0,0.0],"#,
                r#""rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#
            ),
        )
        .env("NUWAX_GEOMETRY_NATIVE_LIO_PRIVATE_PREFIX", "/g14")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_FIRST_PAIR_TIMEOUT_MS", "1000")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_PAIR_MAX_SKEW_MS", "10")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_HOST_IP", "192.0.2.10")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_LIDAR_IP", "192.0.2.11");
    command
}

#[cfg(unix)]
fn assert_native_preflight_case(
    scenario: &str,
    configure: impl FnOnce(&mut Command, &ProbeDir),
    expected_detail: &str,
) {
    let probe = ProbeDir::fail_closed(scenario);
    let artifacts = SeededStartupArtifacts::new(&probe, scenario);
    let mut command = native_preflight_command(&probe);
    configure(&mut command, &probe);
    // If native preflight regresses behind transport startup, this provider wins
    // and the expected native bad-config detail below is necessarily absent.
    command.env("NUWAX_LCM_URL", INVALID_PREFLIGHT_LCM_PROVIDER);
    assert_native_preflight_failure(&mut command, &artifacts, scenario, expected_detail);
}

fn wait_until(child: &mut Child, timeout: Duration) -> Option<ExitStatus> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(status) = child.try_wait().expect("poll runtime child") {
            return Some(status);
        }
        if Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn collect_bounded_output(mut child: ChildGuard, scenario: &str) -> Output {
    if wait_until(child.child_mut(), EXIT_TIMEOUT).is_none() {
        let output = child.terminate_and_collect();
        panic!(
            "{scenario} did not fail within {EXIT_TIMEOUT:?}; stdout={}, stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    child.collect()
}

fn wait_for_startup_artifacts(
    child: &mut Child,
    probe: &ProbeDir,
    ready_path: &Path,
    timeout: Duration,
) -> Result<(), String> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(status) = child.try_wait().map_err(|error| error.to_string())? {
            return Err(format!("runtime exited before startup artifacts: {status}"));
        }
        let ready_file_matches = fs::read_to_string(ready_path)
            .is_ok_and(|value| value == probe.runtime_port().to_string());
        let required_artifacts_ready = ready_file_matches
            && probe.path().join("sequence-state.json").exists()
            && probe.path().join("cloud-map.mmap").exists()
            && probe.path().join("occupancy.mmap").exists()
            && probe.path().join("video.shm").exists()
            && probe.path().join("video.sock").exists();
        if required_artifacts_ready {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "startup artifacts were incomplete after {timeout:?}: probe={}",
                probe.path().display()
            ));
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn assert_specific_startup_failure(
    command: &mut Command,
    scenario: &str,
    category: &str,
    detail: &str,
) {
    let child = ChildGuard::new(command.spawn().expect("spawn startup failure probe"));
    let output = collect_bounded_output(child, scenario);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert_eq!(output.status.code(), Some(4), "{scenario}: {stderr}");
    assert!(
        stderr.contains(&format!("category={category}")),
        "{scenario} lost failure category: {stderr}"
    );
    assert!(
        stderr.contains(detail),
        "{scenario} lost failure detail: {stderr}"
    );
    assert!(
        !stderr.contains("liblcm 1.3~1.5 missing"),
        "{scenario} was misreported as a missing liblcm installation: {stderr}"
    );
}

#[test]
fn unsupported_backends_exit_one_before_touching_ready_file() {
    for backend in ["fake", "fake-lcm", "bogus"] {
        let probe = ProbeDir::fail_closed(backend);
        let scenario = format!("unsupported LCM backend {backend}");
        let artifacts = SeededStartupArtifacts::new(&probe, &scenario);
        assert_preflight_failure(
            &mut runtime_command(backend, &probe),
            &artifacts,
            &scenario,
            "LCM_BACKEND_UNSUPPORTED",
        );
    }
}

#[test]
fn conflicting_owner_profile_exits_one_before_startup_side_effects() {
    let probe = ProbeDir::fail_closed("owner-profile-conflict");
    let artifacts = SeededStartupArtifacts::new(&probe, "owner-profile-conflict");

    let mut command = runtime_command("real-lcm", &probe);
    command
        .env("NUWAX_GEOMETRY_LIO_BACKEND", "device,fastlio2")
        .env("NUWAX_GEOMETRY_MAPPING_BACKEND", "voxel")
        .env("NUWAX_GEOMETRY_CORRECTION_BACKEND", "none")
        .env("NUWAX_GEOMETRY_CANONICAL_MAP_WRITER", "geometry")
        .env("NUWAX_GEOMETRY_CANONICAL_CORRECTION_WRITER", "geometry");

    assert_preflight_failure(
        &mut command,
        &artifacts,
        "conflicting Geometry owner profile",
        concat!(
            "invalid Geometry owner profile: lio requires exactly one owner selection, got [\"device\", \"fastlio2\"]"
        ),
    );
}

#[test]
fn padded_owner_profile_exits_one_before_startup_side_effects() {
    let probe = ProbeDir::fail_closed("owner-profile-padded");
    let artifacts = SeededStartupArtifacts::new(&probe, "owner-profile-padded");

    let mut command = runtime_command("real-lcm", &probe);
    command
        .env("NUWAX_GEOMETRY_LIO_BACKEND", " device ")
        .env("NUWAX_GEOMETRY_MAPPING_BACKEND", "voxel")
        .env("NUWAX_GEOMETRY_CORRECTION_BACKEND", "none")
        .env("NUWAX_GEOMETRY_CANONICAL_MAP_WRITER", "geometry")
        .env("NUWAX_GEOMETRY_CANONICAL_CORRECTION_WRITER", "geometry");

    assert_preflight_failure(
        &mut command,
        &artifacts,
        "padded Geometry owner profile",
        "invalid Geometry owner profile: unknown lio owner selection ` device `",
    );
}

#[test]
fn native_owner_profiles_reach_selected_preflight_before_startup_side_effects() {
    for lio in ["fastlio2", "pointlio"] {
        for correction in ["none", "native_pgo"] {
            let scenario = format!("owner-{lio}-voxel-{correction}");
            let probe = ProbeDir::fail_closed(&scenario);
            let artifacts = SeededStartupArtifacts::new(&probe, &scenario);
            let mut command = runtime_command("real-lcm", &probe);
            command
                .env("NUWAX_GEOMETRY_LIO_BACKEND", lio)
                .env("NUWAX_GEOMETRY_MAPPING_BACKEND", "voxel")
                .env("NUWAX_GEOMETRY_CORRECTION_BACKEND", correction)
                .env("NUWAX_GEOMETRY_CANONICAL_MAP_WRITER", "geometry")
                .env("NUWAX_GEOMETRY_CANONICAL_CORRECTION_WRITER", "geometry");
            assert_preflight_failure(
                &mut command,
                &artifacts,
                &scenario,
                "NATIVE_LIO_PREFLIGHT_FAILED env=NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE is_required",
            );
        }
    }
}

#[cfg(unix)]
#[test]
fn native_owner_preflight_matrix_preserves_startup_sentinels() {
    let cases = [
        (
            "native-relative-executable",
            "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE",
            "relative/native-lio",
            "path_must_be_absolute",
        ),
        (
            "native-reflection-extrinsic",
            "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR",
            concat!(
                r#"{"translation_m":[0.0,0.0,0.0],"#,
                r#""rotation_row_major":[-1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#
            ),
            "rotation_must_be_proper_orthonormal",
        ),
        (
            "native-prefix-budget",
            "NUWAX_GEOMETRY_NATIVE_LIO_PRIVATE_PREFIX",
            "/12345678901234",
            "prefix_too_long",
        ),
        (
            "native-prefix-shape",
            "NUWAX_GEOMETRY_NATIVE_LIO_PRIVATE_PREFIX",
            "/",
            "invalid_private_prefix",
        ),
        (
            "native-host-ip",
            "NUWAX_GEOMETRY_NATIVE_LIO_HOST_IP",
            "not-an-ip",
            "invalid_ipv4",
        ),
        (
            "native-first-pair-timeout-zero",
            "NUWAX_GEOMETRY_NATIVE_LIO_FIRST_PAIR_TIMEOUT_MS",
            "0",
            "outside=1..=300000",
        ),
        (
            "native-padded-pair-skew",
            "NUWAX_GEOMETRY_NATIVE_LIO_PAIR_MAX_SKEW_MS",
            " 5",
            "value_is_empty_or_padded",
        ),
    ];
    for (scenario, key, value, expected) in cases {
        assert_native_preflight_case(
            scenario,
            |command, _probe| {
                command.env(key, value);
            },
            expected,
        );
    }

    let invalid_lcm_scenario = "native-invalid-lcm-url";
    let invalid_lcm_probe = ProbeDir::fail_closed(invalid_lcm_scenario);
    let invalid_lcm_artifacts =
        SeededStartupArtifacts::new(&invalid_lcm_probe, invalid_lcm_scenario);
    let mut invalid_lcm_command = native_preflight_command(&invalid_lcm_probe);
    invalid_lcm_command.env("NUWAX_LCM_URL", "not-a-provider");
    assert_native_preflight_failure(
        &mut invalid_lcm_command,
        &invalid_lcm_artifacts,
        invalid_lcm_scenario,
        "invalid_lcm_url",
    );

    assert_native_preflight_case(
        "native-missing-executable",
        |command, probe| {
            command.env(
                "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE",
                probe.path().join("missing-native-lio"),
            );
        },
        "canonicalize_failed",
    );
    assert_native_preflight_case(
        "native-directory-executable",
        |command, probe| {
            command.env("NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE", probe.path());
        },
        "not_regular_file",
    );
    assert_native_preflight_case(
        "native-non-executable-file",
        |command, probe| {
            let artifact = probe.path().join("native-no-exec");
            fs::write(&artifact, "#!/bin/sh\nexit 0\n").expect("write non-executable artifact");
            fs::set_permissions(&artifact, fs::Permissions::from_mode(0o600))
                .expect("remove executable bits");
            command.env("NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE", artifact);
        },
        "not_executable",
    );
    assert_native_preflight_case(
        "native-non-unicode-executable",
        |command, _probe| {
            command.env(
                "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE",
                OsString::from_vec(vec![0xff]),
            );
        },
        "value_is_not_unicode",
    );
    assert_native_preflight_case(
        "native-identical-ip-pair",
        |command, _probe| {
            command
                .env("NUWAX_GEOMETRY_NATIVE_LIO_HOST_IP", "192.0.2.10")
                .env("NUWAX_GEOMETRY_NATIVE_LIO_LIDAR_IP", "192.0.2.10");
        },
        "addresses_must_be_distinct",
    );
    assert_native_preflight_case(
        "native-pair-skew-exceeds-first-pair",
        |command, _probe| {
            command
                .env("NUWAX_GEOMETRY_NATIVE_LIO_FIRST_PAIR_TIMEOUT_MS", "1000")
                .env("NUWAX_GEOMETRY_NATIVE_LIO_PAIR_MAX_SKEW_MS", "1001");
        },
        "exceeds_first_pair_timeout_ms=1000",
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
#[test]
fn production_binary_owner_profile_matrix_covers_all_twenty_four_rows() {
    run_production_binary_owner_profile_matrix();
}

#[test]
fn selected_raytracing_configuration_fails_before_startup_side_effects() {
    let cases = [
        (
            "empty-voxel-size",
            "NUWAX_MAPPING_RAYTRACING_VOXEL_SIZE",
            "",
            "must not be empty",
        ),
        (
            "invalid-max-range",
            "NUWAX_MAPPING_RAYTRACING_MAX_RANGE",
            "far",
            "cannot be parsed",
        ),
        (
            "zero-subsample",
            "NUWAX_MAPPING_RAYTRACING_RAY_SUBSAMPLE",
            "0",
            "ray_subsample",
        ),
        (
            "inverted-health",
            "NUWAX_MAPPING_RAYTRACING_MIN_HEALTH",
            "1",
            "min_health_lt_max_health",
        ),
    ];

    for (case, key, value, detail) in cases {
        let scenario = format!("raytracing-config-{case}");
        let probe = ProbeDir::fail_closed(&scenario);
        let artifacts = SeededStartupArtifacts::new(&probe, &scenario);
        let mut command = runtime_command("real-lcm", &probe);
        command
            .env("NUWAX_GEOMETRY_MAPPING_BACKEND", "raytracing")
            .env(key, value);
        assert_preflight_failure(&mut command, &artifacts, &scenario, detail);
    }

    #[cfg(unix)]
    {
        let scenario = "raytracing-config-non-unicode";
        let probe = ProbeDir::fail_closed(scenario);
        let artifacts = SeededStartupArtifacts::new(&probe, scenario);
        let mut command = runtime_command("real-lcm", &probe);
        command
            .env("NUWAX_GEOMETRY_MAPPING_BACKEND", "raytracing")
            .env(
                "NUWAX_MAPPING_RAYTRACING_VOXEL_SIZE",
                OsString::from_vec(vec![0xff]),
            );
        assert_preflight_failure(
            &mut command,
            &artifacts,
            scenario,
            "must contain valid Unicode",
        );
    }
}

#[test]
fn every_registry_channel_is_preflighted_before_startup_side_effects() {
    let channels = [
        ("odom", "NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL"),
        ("tf", "NUWAX_GEOMETRY_RUNTIME_TF_CHANNEL"),
        ("pointcloud", "NUWAX_GEOMETRY_RUNTIME_POINTCLOUD_CHANNEL"),
        ("cloud_map", "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL"),
        ("occupancy", "NUWAX_GEOMETRY_RUNTIME_OCCUPANCY_CHANNEL"),
        ("cmd_vel", "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL"),
        ("goal_pose", "NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL"),
        ("nav_state", "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL"),
        ("frontiers", "NUWAX_GEOMETRY_RUNTIME_FRONTIERS_CHANNEL"),
        ("cancel", "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL"),
    ];

    for (channel, variable) in channels {
        let scenario = format!("malformed-channel-{channel}");
        let probe = ProbeDir::fail_closed(&scenario);
        let artifacts = SeededStartupArtifacts::new(&probe, &scenario);
        let mut command = runtime_command("real-lcm", &probe);
        command.env(variable, "not-an-lcm-topic#nuwax_msgs.Invalid");
        assert_preflight_failure(
            &mut command,
            &artifacts,
            &scenario,
            &format!("LCM_CHANNEL_PREFLIGHT_FAILED channel={channel}:"),
        );
    }
}

#[test]
fn whitespace_runtime_channel_does_not_fall_through_to_platform() {
    let scenario = "whitespace-runtime-channel";
    let probe = ProbeDir::fail_closed(scenario);
    let artifacts = SeededStartupArtifacts::new(&probe, scenario);
    let mut command = runtime_command("real-lcm", &probe);
    command
        .env("NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL", " \t\n")
        .env("NUWAX_LCM_CHANNEL_ODOM", "/odom#nav_msgs.Odometry");

    assert_preflight_failure(
        &mut command,
        &artifacts,
        scenario,
        concat!(
            "LCM_CHANNEL_PREFLIGHT_FAILED channel=odom: ",
            "env=NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL value_is_empty"
        ),
    );
}

#[cfg(unix)]
#[test]
fn non_unicode_platform_channel_does_not_fall_through_to_default() {
    let scenario = "non-unicode-platform-channel";
    let probe = ProbeDir::fail_closed(scenario);
    let artifacts = SeededStartupArtifacts::new(&probe, scenario);
    let mut command = runtime_command("real-lcm", &probe);
    command.env("NUWAX_LCM_CHANNEL_ODOM", OsString::from_vec(vec![0xff]));

    assert_preflight_failure(
        &mut command,
        &artifacts,
        scenario,
        concat!(
            "LCM_CHANNEL_PREFLIGHT_FAILED channel=odom: ",
            "env=NUWAX_LCM_CHANNEL_ODOM value_is_not_unicode"
        ),
    );
}

#[test]
fn goal_channel_rejects_the_legacy_pose_type_before_startup() {
    let scenario = "goal-channel-legacy-pose-type";
    let probe = ProbeDir::fail_closed(scenario);
    let artifacts = SeededStartupArtifacts::new(&probe, scenario);
    let mut command = runtime_command("real-lcm", &probe);
    command.env(
        "NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL",
        "/nav/goal#geometry_msgs.PoseStamped",
    );
    assert_preflight_failure(
        &mut command,
        &artifacts,
        scenario,
        "message_type=geometry_msgs.PoseStamped expected=nuwax_msgs.GoalEnvelope",
    );
}

#[test]
fn versioned_path_channel_is_preflighted_before_startup() {
    let scenario = "malformed-versioned-path-channel";
    let probe = ProbeDir::fail_closed(scenario);
    let artifacts = SeededStartupArtifacts::new(&probe, scenario);
    let mut command = runtime_command("real-lcm", &probe);
    command.env(
        "NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL",
        "not-an-lcm-topic#nuwax_msgs.PathEnvelope",
    );
    assert_preflight_failure(
        &mut command,
        &artifacts,
        scenario,
        "LCM_PATH_PREFLIGHT_FAILED",
    );
}

#[cfg(unix)]
#[test]
fn non_unicode_runtime_path_does_not_fall_through_to_platform() {
    let scenario = "non-unicode-runtime-path";
    let probe = ProbeDir::fail_closed(scenario);
    let artifacts = SeededStartupArtifacts::new(&probe, scenario);
    let mut command = runtime_command("real-lcm", &probe);
    command
        .env(
            "NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL",
            OsString::from_vec(vec![0xff]),
        )
        .env(
            "NUWAX_LCM_CHANNEL_NAV_PATH",
            "/nav/path#nuwax_msgs.PathEnvelope",
        );

    assert_preflight_failure(
        &mut command,
        &artifacts,
        scenario,
        concat!(
            "LCM_PATH_PREFLIGHT_FAILED: ",
            "env=NUWAX_GEOMETRY_RUNTIME_NAV_PATH_CHANNEL value_is_not_unicode"
        ),
    );
}

#[test]
fn whitespace_platform_path_does_not_fall_through_to_default() {
    let scenario = "whitespace-platform-path";
    let probe = ProbeDir::fail_closed(scenario);
    let artifacts = SeededStartupArtifacts::new(&probe, scenario);
    let mut command = runtime_command("real-lcm", &probe);
    command.env("NUWAX_LCM_CHANNEL_NAV_PATH", " \t\n");

    assert_preflight_failure(
        &mut command,
        &artifacts,
        scenario,
        concat!(
            "LCM_PATH_PREFLIGHT_FAILED: ",
            "env=NUWAX_LCM_CHANNEL_NAV_PATH value_is_empty"
        ),
    );
}

#[test]
fn required_transport_startup_failure_preserves_provider_error() {
    let probe = ProbeDir::new("transport-failure");
    let mut command = runtime_command("real-lcm", &probe);
    command.env("NUWAX_LCM_URL", "invalid://not-an-lcm-provider");

    assert_specific_startup_failure(
        &mut command,
        "invalid LCM provider",
        "transport",
        "invalid://not-an-lcm-provider",
    );
}

#[test]
fn persistence_startup_failure_preserves_sequence_state_error() {
    let probe = ProbeDir::new("persistence-failure");
    let non_directory = probe.path().join("not-a-directory");
    fs::write(&non_directory, "block child path").expect("seed non-directory path");
    let mut command = runtime_command("real-lcm", &probe);
    command.env(
        "NUWAX_GEOMETRY_SEQUENCE_STATE_PATH",
        non_directory.join("sequence-state.json"),
    );

    assert_specific_startup_failure(
        &mut command,
        "invalid sequence state path",
        "persistence",
        "geometry sequence state startup failed closed",
    );
}

#[test]
fn map_odom_and_base_frames_fail_closed_before_startup_side_effects() {
    let cases = [
        ("map", "NUWAX_GEOMETRY_MAP_FRAME", "NUWAX_MAP_FRAME", "map"),
        (
            "odom",
            "NUWAX_GEOMETRY_ODOM_FRAME",
            "NUWAX_ODOM_FRAME",
            "odom",
        ),
        (
            "base",
            "NUWAX_BASE_FRAME",
            "NUWAX_GEOMETRY_BASE_FRAME",
            "base_link",
        ),
    ];

    for (role, preferred, legacy, legacy_value) in cases {
        let scenario = format!("invalid-{role}-frame");
        let probe = ProbeDir::fail_closed(&scenario);
        let artifacts = SeededStartupArtifacts::new(&probe, &scenario);
        let mut command = runtime_command("real-lcm", &probe);
        command
            .env(preferred, "invalid frame")
            .env(legacy, legacy_value);
        assert_preflight_failure(
            &mut command,
            &artifacts,
            &scenario,
            &format!("FRAME_BINDING_INVALID role={role} env={preferred} frame=\"invalid frame\""),
        );
    }
}

#[test]
fn real_lcm_backend_passes_preflight_and_enters_runtime_startup() {
    let probe = ProbeDir::new("real-lcm");
    let ready_path = probe.path().join("geometry.ready");
    let cloud_map_path = probe.path().join("cloud-map.mmap");
    let occupancy_path = probe.path().join("occupancy.mmap");
    let default_cloud_map_path = Path::new(DEFAULT_CLOUD_MAP_SHM_PATH);
    let default_occupancy_path = Path::new(DEFAULT_OCCUPANCY_SHM_PATH);
    let default_cloud_map_before = file_metadata_snapshot(default_cloud_map_path);
    let default_occupancy_before = file_metadata_snapshot(default_occupancy_path);
    assert!(cloud_map_path.starts_with(probe.path()));
    assert!(occupancy_path.starts_with(probe.path()));
    assert_ne!(probe.lcm_url(), PRODUCTION_LCM_PROVIDER);
    assert!(probe.lcm_url().starts_with("udpm://239.255.77."));
    fs::write(&ready_path, "stale-ready-sentinel").expect("seed ready sentinel");

    let mut child = ChildGuard::new(
        runtime_command("real-lcm", &probe)
            .spawn()
            .expect("spawn geometry runtime"),
    );

    if let Err(reason) =
        wait_for_startup_artifacts(child.child_mut(), &probe, &ready_path, EXIT_TIMEOUT)
    {
        let output = child.terminate_and_collect();
        panic!(
            "real-lcm should reach required startup artifacts; reason={reason}, stdout={}, stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    assert_eq!(
        fs::read_to_string(&ready_path).expect("runtime ready file must exist"),
        probe.runtime_port().to_string(),
        "accepted backend should replace the stale ready file after healthcheck"
    );
    assert!(
        probe.path().join("sequence-state.json").exists(),
        "accepted backend should reach runtime state initialization"
    );
    assert!(
        cloud_map_path.exists(),
        "accepted backend should create cloud-map SHM inside its probe"
    );
    assert!(
        occupancy_path.exists(),
        "accepted backend should create occupancy SHM inside its probe"
    );

    let output = child.terminate_and_collect();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stdout.contains(&cloud_map_path.display().to_string()),
        "runtime did not report the isolated cloud-map SHM path: {stdout}"
    );
    assert!(
        stdout.contains(&occupancy_path.display().to_string()),
        "runtime did not report the isolated occupancy SHM path: {stdout}"
    );
    assert!(
        stdout.contains("LCM backend = real-lcm (available=true)"),
        "runtime did not report the required real-LCM backend: {stdout}"
    );
    assert!(
        !stdout.contains(PRODUCTION_LCM_PROVIDER),
        "isolated startup unexpectedly reported the production LCM provider: {stdout}"
    );
    assert_eq!(
        file_metadata_snapshot(default_cloud_map_path),
        default_cloud_map_before,
        "probe touched the default cloud-map SHM path"
    );
    assert_eq!(
        file_metadata_snapshot(default_occupancy_path),
        default_occupancy_before,
        "probe touched the default occupancy SHM path"
    );
    assert!(
        !stderr.contains("LCM_BACKEND_UNSUPPORTED"),
        "real-lcm was rejected: {stderr}"
    );
}

#[cfg(target_os = "linux")]
#[test]
fn video_helper_failure_revokes_ready_and_stops_runtime() {
    let scenario = "video-helper-terminal";
    let probe = ProbeDir::new(scenario);
    let ready_path = probe.path().join("geometry.ready");
    let video_shm_path = probe.path().join("video.shm");
    let video_uds_path = probe.path().join("video.sock");
    let mut child = ChildGuard::new(
        runtime_command("real-lcm", &probe)
            .spawn()
            .expect("spawn geometry runtime for video terminal probe"),
    );

    if let Err(reason) =
        wait_for_startup_artifacts(child.child_mut(), &probe, &ready_path, EXIT_TIMEOUT)
    {
        let output = child.terminate_and_collect();
        panic!(
            "{scenario}: runtime did not become ready; reason={reason}, stdout={}, stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let helper_pid = wait_for_video_relay_helper(child.child_mut().id(), scenario);
    let signal_status = Command::new("kill")
        .arg("-KILL")
        .arg(helper_pid.to_string())
        .status()
        .expect("SIGKILL video relay helper");
    assert!(signal_status.success(), "{scenario}: SIGKILL failed");

    if wait_until(child.child_mut(), Duration::from_secs(8)).is_none() {
        let output = child.terminate_and_collect();
        panic!(
            "{scenario}: runtime remained alive after video terminal; stdout={}, stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let output = child.collect();
    assert_eq!(
        output.status.code(),
        Some(7),
        "{scenario}: stdout={}, stderr={}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!ready_path.exists(), "{scenario}: ready file remained");
    assert!(!video_shm_path.exists(), "{scenario}: video SHM remained");
    assert!(!video_uds_path.exists(), "{scenario}: video UDS remained");
}

#[cfg(target_os = "linux")]
#[test]
fn video_helper_failure_before_ready_stops_runtime_without_publication() {
    let scenario = "video-helper-pre-ready-terminal";
    let probe = ProbeDir::new(scenario);
    let ready_path = probe.path().join("geometry.ready");
    let video_shm_path = probe.path().join("video.shm");
    let video_uds_path = probe.path().join("video.sock");
    let mut child = ChildGuard::new(
        runtime_command("real-lcm", &probe)
            .spawn()
            .expect("spawn geometry runtime for pre-ready video terminal probe"),
    );
    let geometry_pid = child.child_mut().id();
    let geometry_pid =
        nix::unistd::Pid::from_raw(i32::try_from(geometry_pid).expect("geometry pid fits pid_t"));
    nix::sys::signal::kill(geometry_pid, nix::sys::signal::Signal::SIGSTOP)
        .expect("initial SIGSTOP geometry runtime");

    let deadline = Instant::now() + Duration::from_secs(15);
    let helper_pid = loop {
        assert!(
            !ready_path.exists(),
            "{scenario}: ready was published before video terminal injection"
        );
        let helpers = direct_child_pids(u32::try_from(geometry_pid.as_raw()).unwrap())
            .into_iter()
            .filter(|pid| is_video_relay_helper(*pid))
            .collect::<Vec<_>>();
        match helpers.as_slice() {
            [pid] => break *pid,
            [] => {}
            _ => panic!("{scenario}: expected one video relay helper, got {helpers:?}"),
        }
        assert!(
            Instant::now() < deadline,
            "{scenario}: video relay helper did not start before deadline"
        );
        nix::sys::signal::kill(geometry_pid, nix::sys::signal::Signal::SIGCONT)
            .expect("SIGCONT geometry startup slice");
        std::thread::sleep(Duration::from_micros(500));
        nix::sys::signal::kill(geometry_pid, nix::sys::signal::Signal::SIGSTOP)
            .expect("SIGSTOP geometry startup slice");
    };

    nix::sys::signal::kill(
        nix::unistd::Pid::from_raw(i32::try_from(helper_pid).expect("helper pid fits pid_t")),
        nix::sys::signal::Signal::SIGKILL,
    )
    .expect("SIGKILL pre-ready video relay helper");
    nix::sys::signal::kill(geometry_pid, nix::sys::signal::Signal::SIGCONT)
        .expect("resume geometry after video terminal injection");

    if wait_until(child.child_mut(), Duration::from_secs(8)).is_none() {
        let output = child.terminate_and_collect();
        panic!(
            "{scenario}: runtime remained alive after pre-ready video terminal; stdout={}, stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let output = child.collect();
    assert_eq!(
        output.status.code(),
        Some(3),
        "{scenario}: stdout={}, stderr={}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!ready_path.exists(), "{scenario}: ready file appeared");
    assert!(!video_shm_path.exists(), "{scenario}: video SHM remained");
    assert!(!video_uds_path.exists(), "{scenario}: video UDS remained");
}

#[cfg(feature = "nuwax-lio-test-child")]
fn wait_for_file_contents(path: &Path, timeout: Duration) -> Result<String, String> {
    let deadline = Instant::now() + timeout;
    loop {
        match fs::read_to_string(path) {
            Ok(contents) if !contents.is_empty() => return Ok(contents),
            Ok(_) => {}
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(format!("read {}: {error}", path.display())),
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "{} did not become readable within {timeout:?}",
                path.display()
            ));
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
fn private_topic_generation(
    pattern: &str,
    private_prefix: &str,
    backend: &str,
    leaf: &str,
    message_type: &str,
) -> u64 {
    let backend_tag = match backend {
        "fastlio2" => "f2",
        "pointlio" => "pl",
        value => panic!("unsupported native fixture backend: {value}"),
    };
    let leaf_tag = match leaf {
        "odometry" => "o",
        "cloud" => "c",
        value => panic!("unsupported native fixture stream: {value}"),
    };
    let topic = pattern
        .strip_suffix(&format!("#{message_type}"))
        .unwrap_or_else(|| panic!("native topic lost typed suffix: {pattern}"));
    let generation_and_leaf = topic
        .strip_prefix(&format!("{private_prefix}/{backend_tag}/"))
        .unwrap_or_else(|| panic!("native topic escaped generation-private prefix: {pattern}"));
    let (generation, actual_leaf) = generation_and_leaf
        .split_once('/')
        .unwrap_or_else(|| panic!("native topic lost generation/leaf boundary: {pattern}"));
    assert_eq!(
        actual_leaf, leaf_tag,
        "native topic selected the wrong stream"
    );
    let generation = generation
        .parse::<u64>()
        .unwrap_or_else(|error| panic!("invalid native topic generation {generation:?}: {error}"));
    assert!(generation > 0, "native topic generation must be positive");
    generation
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_native_child_reaped(pid: u32, timeout: Duration) {
    let raw_pid = i32::try_from(pid).expect("native test child pid must fit platform pid range");
    let pid = nix::unistd::Pid::from_raw(raw_pid);
    let deadline = Instant::now() + timeout;
    loop {
        match nix::sys::signal::kill(pid, None) {
            Err(nix::errno::Errno::ESRCH) => return,
            Ok(()) | Err(nix::errno::Errno::EPERM) => {}
            Err(error) => panic!("probe native child pid={pid} failed: {error}"),
        }
        if Instant::now() >= deadline {
            panic!("native child pid={pid} remained alive after Geometry exit");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn terminate_native_child_if_present(pid_path: &Path) {
    let Ok(pid) = fs::read_to_string(pid_path) else {
        return;
    };
    let Ok(pid) = pid.parse::<i32>() else {
        return;
    };
    let pid = nix::unistd::Pid::from_raw(pid);
    if matches!(
        nix::sys::signal::kill(pid, None),
        Err(nix::errno::Errno::ESRCH)
    ) {
        return;
    }

    let _ = nix::sys::signal::kill(pid, nix::sys::signal::Signal::SIGTERM);
    let term_deadline = Instant::now() + Duration::from_millis(250);
    while Instant::now() < term_deadline {
        if matches!(
            nix::sys::signal::kill(pid, None),
            Err(nix::errno::Errno::ESRCH)
        ) {
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let _ = nix::sys::signal::kill(pid, nix::sys::signal::Signal::SIGKILL);
    let kill_deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < kill_deadline {
        if matches!(
            nix::sys::signal::kill(pid, None),
            Err(nix::errno::Errno::ESRCH)
        ) {
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
struct NativeChildCleanup {
    pid_path: PathBuf,
    armed: bool,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
impl NativeChildCleanup {
    fn new(pid_path: PathBuf) -> Self {
        Self {
            pid_path,
            armed: true,
        }
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
impl Drop for NativeChildCleanup {
    fn drop(&mut self) {
        if self.armed {
            terminate_native_child_if_present(&self.pid_path);
        }
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
const NATIVE_FIXTURE_TIMEOUT: Duration = Duration::from_secs(12);
#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
const NATIVE_PGO_PARENT_DEATH_GRACE: Duration = Duration::from_millis(250);
#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
const NATIVE_PGO_HARNESS_TERM_GRACE: Duration = Duration::from_millis(250);
#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
const NATIVE_PGO_HARNESS_KILL_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(feature = "nuwax-lio-test-child")]
const OUTPUT_SILENCE_WINDOW: Duration = Duration::from_millis(200);
#[cfg(feature = "nuwax-lio-test-child")]
const RAYTRACING_CANDIDATE_CADENCE_WAIT: Duration = Duration::from_millis(300);
#[cfg(feature = "nuwax-lio-test-child")]
const CORRECTION_CANDIDATE_MAX_BODY_BYTES: usize = 4_096 - 8;
#[cfg(feature = "nuwax-lio-test-child")]
const CORRECTION_FINALIZATION_TARGET: &str = "nuwax_geometry_runtime::correction_finalization";
#[cfg(feature = "nuwax-lio-test-child")]
const CORRECTION_FINALIZATION_EVENT: &str = "nuwax_correction_candidate_finalized_v1";

#[cfg(feature = "nuwax-lio-test-child")]
fn typed_binding(channel: &str, message_type: &str) -> (LcmTopic, LcmMessageType) {
    (
        LcmTopic::new(channel)
            .unwrap_or_else(|error| panic!("invalid fixture topic {channel:?}: {error}")),
        LcmMessageType::new(message_type)
            .unwrap_or_else(|error| panic!("invalid fixture type {message_type:?}: {error}")),
    )
}

#[cfg(feature = "nuwax-lio-test-child")]
fn binding_from_pattern(pattern: &str, expected_type: &str) -> (LcmTopic, LcmMessageType) {
    let (topic, message_type) = LcmTopic::parse_subscription_pattern(pattern)
        .unwrap_or_else(|error| panic!("invalid fixture pattern {pattern:?}: {error}"));
    assert_eq!(
        message_type.as_deref(),
        Some(expected_type),
        "fixture pattern selected the wrong message type"
    );
    (
        topic,
        LcmMessageType::new(expected_type).expect("fixture message type"),
    )
}

#[cfg(feature = "nuwax-lio-test-child")]
struct NativeObservers {
    transport: LcmTransport,
    public_odometry: LcmFrameStream,
    public_cloud: LcmFrameStream,
    status: LcmFrameStream,
    tf: LcmFrameStream,
    global_map: LcmFrameStream,
}

#[cfg(feature = "nuwax-lio-test-child")]
impl NativeObservers {
    fn new(runtime: &tokio::runtime::Runtime, provider: &str) -> Self {
        let transport =
            LcmTransport::new_real(provider).expect("create aggregate real-LCM observer");
        let (public_odometry, public_cloud, status, tf, global_map) = runtime.block_on(async {
            let odometry =
                typed_binding("/geometry/lio/odometry", "nuwax_msgs.LioOdometryEnvelope");
            let cloud = typed_binding("/geometry/lio/cloud", "nuwax_msgs.LioCloudEnvelope");
            let status = typed_binding("/geometry/lio/status", "nuwax_msgs.LioStatus");
            let tf = typed_binding("/tf", "tf2_msgs.TFMessage");
            let global_map = typed_binding("/global_map", "sensor_msgs.PointCloud2");
            (
                transport.subscribe(odometry.0, Some(odometry.1)).await,
                transport.subscribe(cloud.0, Some(cloud.1)).await,
                transport.subscribe(status.0, Some(status.1)).await,
                transport.subscribe(tf.0, Some(tf.1)).await,
                transport.subscribe(global_map.0, Some(global_map.1)).await,
            )
        });
        Self {
            transport,
            public_odometry,
            public_cloud,
            status,
            tf,
            global_map,
        }
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
struct NativePgoObservers {
    transport: LcmTransport,
    request: LcmFrameStream,
    result: LcmFrameStream,
    status: LcmFrameStream,
    cancel: LcmFrameStream,
    library_candidate: LcmFrameStream,
}

#[cfg(feature = "nuwax-lio-test-child")]
impl NativePgoObservers {
    fn new(runtime: &tokio::runtime::Runtime, provider: &str, generation: u64) -> Self {
        let generation = i64::try_from(generation).expect("native PGO generation fits int64");
        let topics = nuwax_nav_pgo_bridge::OnlinePgoTopics::for_runtime(generation, generation)
            .expect("derive native PGO topics");
        let transport =
            LcmTransport::new_real(provider).expect("create native PGO real-LCM observer");
        let (request, result, status, cancel, library_candidate) = runtime.block_on(async {
            let request = binding_from_pattern(&topics.request, "nuwax_msgs.NativePgoRequest");
            let result = binding_from_pattern(&topics.result, "nuwax_msgs.NativePgoResult");
            let status = binding_from_pattern(&topics.status, "nuwax_msgs.NativePgoStatus");
            let cancel = binding_from_pattern(&topics.cancel, "nuwax_msgs.NativePgoStatus");
            let library_candidate = typed_binding(
                &format!("/c/l/{generation}"),
                "nuwax_msgs.CorrectionCandidate",
            );
            (
                transport.subscribe(request.0, Some(request.1)).await,
                transport.subscribe(result.0, Some(result.1)).await,
                transport.subscribe(status.0, Some(status.1)).await,
                transport.subscribe(cancel.0, Some(cancel.1)).await,
                transport
                    .subscribe(library_candidate.0, Some(library_candidate.1))
                    .await,
            )
        });
        Self {
            transport,
            request,
            result,
            status,
            cancel,
            library_candidate,
        }
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
fn next_frame(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    label: &str,
) -> RawLcmFrame {
    runtime.block_on(async {
        tokio::time::timeout(NATIVE_FIXTURE_TIMEOUT, stream.next())
            .await
            .unwrap_or_else(|_| panic!("{label} timed out after {NATIVE_FIXTURE_TIMEOUT:?}"))
            .unwrap_or_else(|| panic!("{label} subscription closed"))
            .unwrap_or_else(|error| panic!("{label} receive failed: {error}"))
    })
}

#[cfg(feature = "nuwax-lio-test-child")]
fn next_frame_within(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    timeout: Duration,
    label: &str,
) -> Option<RawLcmFrame> {
    runtime.block_on(async {
        match tokio::time::timeout(timeout, stream.next()).await {
            Err(_) => None,
            Ok(Some(Ok(frame))) => Some(frame),
            Ok(Some(Err(error))) => panic!("{label} receive failed: {error}"),
            Ok(None) => panic!("{label} subscription closed"),
        }
    })
}

#[cfg(feature = "nuwax-lio-test-child")]
fn assert_stream_silent(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    label: &str,
) {
    runtime.block_on(async {
        match tokio::time::timeout(OUTPUT_SILENCE_WINDOW, stream.next()).await {
            Err(_) => {}
            Ok(Some(Ok(frame))) => panic!(
                "{label} unexpectedly received channel={} type={}",
                frame.topic.channel(),
                frame.msg_type.fqn()
            ),
            Ok(Some(Err(error))) => panic!("{label} receive failed: {error}"),
            Ok(None) => panic!("{label} subscription closed"),
        }
    });
}

#[cfg(feature = "nuwax-lio-test-child")]
fn drain_matching_global_map(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    expected_frame: &RawLcmFrame,
    label: &str,
) {
    let expected = decode_global_map(expected_frame);
    let deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    runtime.block_on(async {
        loop {
            match tokio::time::timeout(OUTPUT_SILENCE_WINDOW, stream.next()).await {
                Err(_) => return,
                Ok(Some(Ok(frame))) => {
                    let actual = decode_global_map(&frame);
                    let canonical_empty = actual.header.frame_id == "map"
                        && actual.height == 1
                        && actual.width == 0
                        && actual.point_step == 12
                        && actual.row_step == 0
                        && actual.data.is_empty();
                    assert!(canonical_empty || actual == expected, "{label}");
                }
                Ok(Some(Err(error))) => panic!("{label} receive failed: {error}"),
                Ok(None) => panic!("{label} subscription closed"),
            }
            assert!(
                Instant::now() < deadline,
                "{label} did not become quiet within {NATIVE_FIXTURE_TIMEOUT:?}"
            );
        }
    });
}

#[cfg(feature = "nuwax-lio-test-child")]
fn drain_equivalent_global_map_geometry(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    expected_frame: &RawLcmFrame,
    label: &str,
) {
    let expected = decode_global_map(expected_frame);
    let deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    runtime.block_on(async {
        loop {
            match tokio::time::timeout(OUTPUT_SILENCE_WINDOW, stream.next()).await {
                Err(_) => return,
                Ok(Some(Ok(frame))) => {
                    let mut actual = decode_global_map(&frame);
                    let canonical_empty = actual.header.frame_id == "map"
                        && actual.height == 1
                        && actual.width == 0
                        && actual.point_step == 12
                        && actual.row_step == 0
                        && actual.data.is_empty();
                    let same_frame = actual.header.frame_id == expected.header.frame_id;
                    actual.header = expected.header.clone();
                    assert!(
                        canonical_empty || (same_frame && actual == expected),
                        "{label}"
                    );
                }
                Ok(Some(Err(error))) => panic!("{label} receive failed: {error}"),
                Ok(None) => panic!("{label} subscription closed"),
            }
            assert!(
                Instant::now() < deadline,
                "{label} did not become quiet within {NATIVE_FIXTURE_TIMEOUT:?}"
            );
        }
    });
}

#[cfg(target_os = "linux")]
fn observe_direct_child_pids(parent_pid: u32) -> Result<Vec<u32>, String> {
    let task_path = format!("/proc/{parent_pid}/task");
    let mut children = Vec::new();
    let entries = match fs::read_dir(&task_path) {
        Ok(entries) => entries,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(children),
        Err(error) => return Err(format!("read {task_path}: {error}")),
    };
    for entry in entries {
        let entry = entry.map_err(|error| format!("read {task_path} entry: {error}"))?;
        let children_path = entry.path().join("children");
        let contents = match fs::read_to_string(&children_path) {
            Ok(contents) => contents,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
            Err(error) => return Err(format!("read {}: {error}", children_path.display())),
        };
        for pid in contents.split_whitespace() {
            children.push(
                pid.parse::<u32>()
                    .map_err(|error| format!("invalid child pid {pid:?}: {error}"))?,
            );
        }
    }
    children.sort_unstable();
    children.dedup();
    Ok(children)
}

#[cfg(target_os = "linux")]
fn direct_child_pids(parent_pid: u32) -> Vec<u32> {
    observe_direct_child_pids(parent_pid)
        .unwrap_or_else(|error| panic!("observe direct children of pid={parent_pid}: {error}"))
}

#[cfg(target_os = "linux")]
fn process_has_environment_key(pid: u32, key: &str) -> bool {
    let path = format!("/proc/{pid}/environ");
    let environment = match fs::read(&path) {
        Ok(environment) => environment,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return false,
        Err(error) => panic!("read {path}: {error}"),
    };
    environment.split(|byte| *byte == 0).any(|entry| {
        entry
            .strip_prefix(key.as_bytes())
            .is_some_and(|suffix| suffix.first() == Some(&b'='))
    })
}

#[cfg(target_os = "linux")]
fn is_video_relay_helper(pid: u32) -> bool {
    process_has_environment_key(pid, "NUWAX_VIDEO_RELAY_HELPER_REQUEST_V1")
}

#[cfg(target_os = "linux")]
fn wait_for_video_relay_helper(parent_pid: u32, scenario: &str) -> u32 {
    let deadline = Instant::now() + EXIT_TIMEOUT;
    loop {
        let helpers = direct_child_pids(parent_pid)
            .into_iter()
            .filter(|pid| is_video_relay_helper(*pid))
            .collect::<Vec<_>>();
        match helpers.as_slice() {
            [pid] => return *pid,
            [] => {}
            _ => panic!("{scenario}: expected one video relay helper, got {helpers:?}"),
        }
        assert!(
            Instant::now() < deadline,
            "{scenario}: video relay helper did not start"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn observe_process_start_time(pid: u32) -> Result<Option<u64>, String> {
    let path = format!("/proc/{pid}/stat");
    let stat = match fs::read_to_string(&path) {
        Ok(stat) => stat,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(format!("read {path}: {error}")),
    };
    let fields = stat
        .rsplit_once(") ")
        .ok_or_else(|| format!("malformed process stat for pid={pid}"))?
        .1;
    let start_time = fields
        .split_whitespace()
        .nth(19)
        .ok_or_else(|| format!("process stat omitted start time for pid={pid}"))?
        .parse()
        .map_err(|error| format!("invalid process start time for pid={pid}: {error}"))?;
    Ok(Some(start_time))
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn process_start_time(pid: u32) -> Option<u64> {
    observe_process_start_time(pid)
        .unwrap_or_else(|error| panic!("observe process start time for pid={pid}: {error}"))
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn observe_process_executable(pid: u32) -> Result<Option<PathBuf>, String> {
    let path = format!("/proc/{pid}/exe");
    match fs::read_link(&path) {
        Ok(executable) => Ok(Some(executable)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(format!("read {path}: {error}")),
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn process_executable(pid: u32) -> Option<PathBuf> {
    observe_process_executable(pid)
        .unwrap_or_else(|error| panic!("observe process executable for pid={pid}: {error}"))
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn tracked_process_group_alive(process_group: &TrackedProcessGroup) -> Result<bool, String> {
    let raw_pid = i32::try_from(process_group.pid)
        .map_err(|_| format!("{}: pid exceeds pid_t", process_group.scenario))?;
    let group_pid = nix::unistd::Pid::from_raw(-raw_pid);
    let group_exists = match nix::sys::signal::kill(group_pid, None) {
        Err(nix::errno::Errno::ESRCH) => false,
        Ok(()) | Err(nix::errno::Errno::EPERM) => true,
        Err(error) => {
            return Err(format!(
                "{}: probe process group {}: {error}",
                process_group.scenario, process_group.pid
            ));
        }
    };
    let observed_start = observe_process_start_time(process_group.pid)?;
    if !group_exists && observed_start.is_none() {
        return Ok(false);
    }
    if observed_start != Some(process_group.start_time) {
        return Err(format!(
            "{}: process group {} identity changed while still observable",
            process_group.scenario, process_group.pid
        ));
    }
    let leader = nix::unistd::Pid::from_raw(raw_pid);
    let observed_group = nix::unistd::getpgid(Some(leader)).map_err(|error| {
        format!(
            "{}: read process group for pid {}: {error}",
            process_group.scenario, process_group.pid
        )
    })?;
    if observed_group.as_raw() != raw_pid {
        return Err(format!(
            "{}: pid {} moved to process group {}",
            process_group.scenario,
            process_group.pid,
            observed_group.as_raw()
        ));
    }
    if !group_exists {
        return Err(format!(
            "{}: leader {} remained but its process group disappeared",
            process_group.scenario, process_group.pid
        ));
    }
    Ok(true)
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn wait_for_tracked_process_group_exit(
    process_group: &TrackedProcessGroup,
    timeout: Duration,
) -> Result<bool, String> {
    let deadline = Instant::now()
        .checked_add(timeout)
        .ok_or_else(|| format!("{}: cleanup deadline overflow", process_group.scenario))?;
    loop {
        if !tracked_process_group_alive(process_group)? {
            return Ok(true);
        }
        if Instant::now() >= deadline {
            return Ok(false);
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn signal_tracked_process_group(
    process_group: &TrackedProcessGroup,
    signal: nix::sys::signal::Signal,
) -> Result<(), String> {
    if !tracked_process_group_alive(process_group)? {
        return Ok(());
    }
    let raw_pid = i32::try_from(process_group.pid)
        .map_err(|_| format!("{}: pid exceeds pid_t", process_group.scenario))?;
    match nix::sys::signal::kill(nix::unistd::Pid::from_raw(-raw_pid), signal) {
        Ok(()) | Err(nix::errno::Errno::ESRCH) => Ok(()),
        Err(error) => Err(format!(
            "{}: signal {signal:?} to process group {}: {error}",
            process_group.scenario, process_group.pid
        )),
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn settle_tracked_process_group(
    process_group: &TrackedProcessGroup,
    self_exit_grace: Duration,
) -> Result<ProcessGroupCleanupDisposition, String> {
    if wait_for_tracked_process_group_exit(process_group, self_exit_grace)? {
        return Ok(ProcessGroupCleanupDisposition::ExitedWithoutHarnessSignal);
    }
    signal_tracked_process_group(process_group, nix::sys::signal::Signal::SIGTERM)?;
    if wait_for_tracked_process_group_exit(process_group, NATIVE_PGO_HARNESS_TERM_GRACE)? {
        return Ok(ProcessGroupCleanupDisposition::TerminatedByHarness);
    }
    signal_tracked_process_group(process_group, nix::sys::signal::Signal::SIGKILL)?;
    if wait_for_tracked_process_group_exit(process_group, NATIVE_PGO_HARNESS_KILL_TIMEOUT)? {
        return Ok(ProcessGroupCleanupDisposition::KilledByHarness);
    }
    Err(format!(
        "{}: process group {} remained after bounded SIGTERM/SIGKILL cleanup",
        process_group.scenario, process_group.pid
    ))
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn wait_for_native_pgo_child(
    geometry_pid: u32,
    native_lio_pid: u32,
    scenario: &str,
) -> (u32, u64, u32, u64) {
    let expected = nuwax_nav_pgo_bridge::native_executable()
        .canonicalize()
        .expect("canonicalize native PGO executable");
    let deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    loop {
        let children = direct_child_pids(geometry_pid);
        let candidates = children
            .iter()
            .copied()
            .filter(|pid| *pid != native_lio_pid)
            .collect::<Vec<_>>();
        let native = candidates
            .iter()
            .copied()
            .filter(|pid| process_executable(*pid).as_ref() == Some(&expected))
            .collect::<Vec<_>>();
        let guardians = candidates
            .iter()
            .copied()
            .filter(|pid| {
                process_executable(*pid)
                    .as_deref()
                    .and_then(Path::file_name)
                    == Some(std::ffi::OsStr::new("nuwax-group-guardian"))
            })
            .collect::<Vec<_>>();
        let video_helpers = candidates
            .iter()
            .copied()
            .filter(|pid| is_video_relay_helper(*pid))
            .collect::<Vec<_>>();
        match (
            native.as_slice(),
            guardians.as_slice(),
            video_helpers.as_slice(),
        ) {
            ([pid], [guardian_pid], [video_pid]) => {
                let start_time = process_start_time(*pid)
                    .unwrap_or_else(|| panic!("{scenario}: native PGO child disappeared"));
                let guardian_start_time = process_start_time(*guardian_pid)
                    .unwrap_or_else(|| panic!("{scenario}: native PGO guardian disappeared"));
                let mut expected_children = vec![native_lio_pid, *pid, *guardian_pid, *video_pid];
                expected_children.sort_unstable();
                assert_eq!(
                    children, expected_children,
                    "{scenario}: voxel/native tuple spawned an unselected child"
                );
                return (*pid, start_time, *guardian_pid, guardian_start_time);
            }
            ([], [], [])
            | ([], [], [_])
            | ([], [_], [])
            | ([], [_], [_])
            | ([_], [], [])
            | ([_], [], [_]) => {}
            _ => {
                let observed = candidates
                    .iter()
                    .map(|pid| (*pid, process_executable(*pid)))
                    .collect::<Vec<_>>();
                panic!(
                    "{scenario}: expected exactly {} plus nuwax-group-guardian, got {observed:?}",
                    expected.display(),
                );
            }
        }
        assert!(
            Instant::now() < deadline,
            "{scenario}: native PGO child did not start"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn wait_for_native_pgo_process_group_exit(pid: u32, start_time: u64, scenario: &str) {
    let raw_pid = i32::try_from(pid).expect("native PGO pid fits pid_t");
    let process_group = nix::unistd::Pid::from_raw(-raw_pid);
    let deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    loop {
        let leader_alive = process_start_time(pid) == Some(start_time);
        let group_alive = match nix::sys::signal::kill(process_group, None) {
            Err(nix::errno::Errno::ESRCH) => false,
            Ok(()) | Err(nix::errno::Errno::EPERM) => true,
            Err(error) => panic!("{scenario}: probe native PGO process group {pid}: {error}"),
        };
        if !leader_alive && !group_alive {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "{scenario}: native PGO process group {pid} remained alive"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn wait_for_mapping_sidecar(parent_pid: u32, native_lio_pid: u32, scenario: &str) -> (u32, u64) {
    let deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    loop {
        let candidates = direct_child_pids(parent_pid)
            .into_iter()
            .filter(|pid| *pid != native_lio_pid && !is_video_relay_helper(*pid))
            .collect::<Vec<_>>();
        match candidates.as_slice() {
            [sidecar_pid] => {
                if let Some(start_time) = process_start_time(*sidecar_pid) {
                    return (*sidecar_pid, start_time);
                }
            }
            [] => {}
            _ => panic!("{scenario}: expected one mapping sidecar, got {candidates:?}"),
        }
        assert!(
            Instant::now() < deadline,
            "{scenario}: mapping sidecar did not start"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn wait_for_child_exit(child_pid: u32, child_start_time: u64, scenario: &str) {
    let deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    while process_start_time(child_pid) == Some(child_start_time) {
        assert!(
            Instant::now() < deadline,
            "{scenario}: mapping sidecar {child_pid} was not reaped"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
fn with_fingerprint(fingerprint: [u8; 8], payload: &[u8]) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(8 + payload.len());
    encoded.extend_from_slice(&fingerprint);
    encoded.extend_from_slice(payload);
    encoded
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_public_odometry(frame: &RawLcmFrame) -> lio::LioOdometryEnvelope {
    lio::decode_lio_odometry_envelope(&with_fingerprint(
        lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
        &frame.payload,
    ))
    .expect("decode public LIO odometry")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_public_cloud(frame: &RawLcmFrame) -> lio::LioCloudEnvelope {
    lio::decode_lio_cloud_envelope(&with_fingerprint(
        lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1,
        &frame.payload,
    ))
    .expect("decode public LIO cloud")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_status(frame: &RawLcmFrame) -> lio::LioStatus {
    lio::decode_lio_status(&with_fingerprint(
        lio::LIO_STATUS_FINGERPRINT_V1,
        &frame.payload,
    ))
    .expect("decode public LIO status")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_native_pgo_request(frame: &RawLcmFrame) -> native_pgo::NativePgoRequest {
    assert_eq!(
        frame.msg_type.fqn(),
        "nuwax_msgs.NativePgoRequest",
        "native PGO request message type drifted"
    );
    native_pgo::decode_native_pgo_request_body(&frame.payload).expect("decode native PGO request")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_native_pgo_result(frame: &RawLcmFrame) -> native_pgo::NativePgoResult {
    assert_eq!(
        frame.msg_type.fqn(),
        "nuwax_msgs.NativePgoResult",
        "native PGO result message type drifted"
    );
    native_pgo::decode_native_pgo_result_body(&frame.payload).expect("decode native PGO result")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_native_pgo_status(frame: &RawLcmFrame) -> native_pgo::NativePgoStatus {
    assert_eq!(
        frame.msg_type.fqn(),
        "nuwax_msgs.NativePgoStatus",
        "native PGO status message type drifted"
    );
    native_pgo::decode_native_pgo_status_body(&frame.payload).expect("decode native PGO status")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn wait_for_native_pgo_status(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    generation: u64,
    expected_event: i32,
    request: Option<&native_pgo::NativePgoRequest>,
    scenario: &str,
) -> native_pgo::NativePgoStatus {
    loop {
        let status = decode_native_pgo_status(&next_frame(runtime, stream, scenario));
        assert_eq!(
            status.schema_version,
            native_pgo::NATIVE_PGO_SCHEMA_VERSION_V1
        );
        assert_eq!(status.backend, native_pgo::NATIVE_PGO_BACKEND);
        assert_eq!(status.runtime_generation, generation as i64);
        assert_eq!(status.backend_generation, generation as i64);
        assert_ne!(
            status.event_kind,
            native_pgo::NATIVE_PGO_EVENT_PROTOCOL_ERROR,
            "{scenario}: native PGO reported protocol error: {}",
            status.reason
        );
        match request {
            Some(request) if status.request.as_ref() == Some(&request.echo()) => {
                native_pgo::validate_native_pgo_status_echo(request, &status)
                    .expect("native PGO status echoed its exact request");
                assert_eq!(
                    status.event_kind, expected_event,
                    "{scenario}: request status order drifted"
                );
                return status;
            }
            Some(_) if status.request.is_none() => {
                assert_eq!(
                    status.event_kind,
                    native_pgo::NATIVE_PGO_EVENT_HEALTH,
                    "{scenario}: unexpected owner-scoped status while awaiting request status"
                );
            }
            Some(_) => panic!("{scenario}: status belongs to a different request"),
            None if status.request.is_none() && status.event_kind == expected_event => {
                return status;
            }
            None if status.request.is_none()
                && status.event_kind == native_pgo::NATIVE_PGO_EVENT_HEALTH => {}
            None => panic!("{scenario}: unexpected request-scoped or out-of-order owner status"),
        }
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
struct NativePgoExchange {
    request_frame: RawLcmFrame,
    request: native_pgo::NativePgoRequest,
    result_frame: RawLcmFrame,
    result: native_pgo::NativePgoResult,
    terminal: native_pgo::NativePgoStatus,
}

#[cfg(feature = "nuwax-lio-test-child")]
fn observe_native_pgo_exchange(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativePgoObservers,
    generation: u64,
    expected_request_sequence: i64,
    expected_source_sequence: i64,
    scenario: &str,
) -> NativePgoExchange {
    let request_frame = next_frame(runtime, &mut observers.request, scenario);
    let request = decode_native_pgo_request(&request_frame);
    assert_eq!(
        request.schema_version,
        native_pgo::NATIVE_PGO_SCHEMA_VERSION_V1
    );
    assert_eq!(request.identity.backend, native_pgo::NATIVE_PGO_BACKEND);
    assert_eq!(request.identity.runtime_generation, generation as i64);
    assert_eq!(request.identity.backend_generation, generation as i64);
    assert_eq!(
        request.identity.request_sequence, expected_request_sequence,
        "{scenario}: request sequence drifted"
    );
    assert_eq!(
        request.identity.registered_cloud.source_sequence, expected_source_sequence,
        "{scenario}: source sequence drifted"
    );
    let accepted = wait_for_native_pgo_status(
        runtime,
        &mut observers.status,
        generation,
        native_pgo::NATIVE_PGO_EVENT_REQUEST_ACCEPTED,
        Some(&request),
        scenario,
    );
    let running = wait_for_native_pgo_status(
        runtime,
        &mut observers.status,
        generation,
        native_pgo::NATIVE_PGO_EVENT_RUNNING,
        Some(&request),
        scenario,
    );
    let result_frame = next_frame(runtime, &mut observers.result, scenario);
    let result = decode_native_pgo_result(&result_frame);
    native_pgo::validate_native_pgo_result_echo(&request, &result)
        .expect("native PGO result echoed its exact request");
    let terminal = wait_for_native_pgo_status(
        runtime,
        &mut observers.status,
        generation,
        native_pgo::NATIVE_PGO_EVENT_TERMINAL,
        Some(&request),
        scenario,
    );
    assert!(
        accepted.status_sequence < running.status_sequence
            && running.status_sequence < terminal.status_sequence,
        "{scenario}: accepted/running/terminal status sequence drifted"
    );
    NativePgoExchange {
        request_frame,
        request,
        result_frame,
        result,
        terminal,
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_tf(frame: &RawLcmFrame) -> schema::TfMessage {
    schema::decode_tf_message(&with_fingerprint(
        schema::TF_MESSAGE_FINGERPRINT,
        &frame.payload,
    ))
    .expect("decode canonical TF")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_global_map(frame: &RawLcmFrame) -> schema::PointCloud2 {
    schema::decode_point_cloud2(&with_fingerprint(
        schema::POINT_CLOUD2_FINGERPRINT,
        &frame.payload,
    ))
    .expect("decode canonical global map")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_private_odometry(frame: &RawLcmFrame) -> schema::Odometry {
    assert_eq!(
        frame.msg_type.fqn(),
        "nav_msgs.Odometry",
        "private source odometry message type drifted"
    );
    schema::decode_odometry(&with_fingerprint(
        schema::ODOMETRY_FINGERPRINT,
        &frame.payload,
    ))
    .expect("decode private source odometry")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_private_source_cloud(frame: &RawLcmFrame) -> schema::PointCloud2 {
    assert_eq!(
        frame.msg_type.fqn(),
        "sensor_msgs.PointCloud2",
        "private source cloud message type drifted"
    );
    schema::decode_point_cloud2(&with_fingerprint(
        schema::POINT_CLOUD2_FINGERPRINT,
        &frame.payload,
    ))
    .expect("decode private source cloud")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn independent_candidate_body_points(
    anchor_odometry: &lio::LioOdometryEnvelope,
    source_odometry: &schema::Odometry,
    source_cloud: &schema::PointCloud2,
) -> Vec<nalgebra::Point3<f64>> {
    assert_eq!(source_cloud.header.frame_id, "g14_lidar");
    assert!(!source_cloud.is_bigendian, "source cloud became big-endian");
    let height = usize::try_from(source_cloud.height).expect("source cloud height is nonnegative");
    let width = usize::try_from(source_cloud.width).expect("source cloud width is nonnegative");
    let point_step =
        usize::try_from(source_cloud.point_step).expect("source cloud point step is nonnegative");
    let row_step =
        usize::try_from(source_cloud.row_step).expect("source cloud row step is nonnegative");
    assert!(height > 0 && width > 0 && point_step >= 12);
    assert_eq!(row_step, width.checked_mul(point_step).unwrap());
    assert_eq!(
        source_cloud.data.len(),
        height.checked_mul(row_step).unwrap()
    );
    for (name, offset) in [("x", 0), ("y", 4), ("z", 8)] {
        let fields = source_cloud
            .fields
            .iter()
            .filter(|field| field.name == name)
            .collect::<Vec<_>>();
        assert_eq!(fields.len(), 1, "source cloud {name} field count drifted");
        assert_eq!(fields[0].offset, offset);
        assert_eq!(fields[0].datatype, 7);
        assert_eq!(fields[0].count, 1);
    }
    let sensor_points = source_cloud
        .data
        .chunks_exact(point_step)
        .map(|bytes| {
            let point = nalgebra::Point3::new(
                f64::from(f32::from_le_bytes(bytes[0..4].try_into().unwrap())),
                f64::from(f32::from_le_bytes(bytes[4..8].try_into().unwrap())),
                f64::from(f32::from_le_bytes(bytes[8..12].try_into().unwrap())),
            );
            assert!(point.coords.iter().all(|value| value.is_finite()));
            point
        })
        .collect::<Vec<_>>();
    assert_eq!(sensor_points.len(), height * width);

    let body_from_sensor_rotation =
        nalgebra::Matrix3::from_row_slice(&[0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
    let body_from_sensor_translation = nalgebra::Vector3::new(0.25, -0.5, 1.0);
    let source_orientation = source_odometry.pose.orientation_xyzw;
    let source_quaternion = nalgebra::Quaternion::new(
        source_orientation[3],
        source_orientation[0],
        source_orientation[1],
        source_orientation[2],
    );
    let source_quaternion_norm = source_quaternion.norm();
    assert!(source_quaternion_norm.is_finite() && (source_quaternion_norm - 1.0).abs() <= 1.0e-3);
    let odom_from_body_rotation = nalgebra::UnitQuaternion::new_normalize(source_quaternion);
    let source_position = source_odometry.pose.position;
    let odom_from_body_translation =
        nalgebra::Vector3::new(source_position[0], source_position[1], source_position[2]);
    let odom_points = sensor_points
        .iter()
        .map(|sensor_point| {
            let body_point = nalgebra::Point3::from(
                body_from_sensor_rotation * sensor_point.coords + body_from_sensor_translation,
            );
            odom_from_body_rotation.transform_point(&body_point) + odom_from_body_translation
        })
        .collect::<Vec<_>>();

    let anchor_pose = &anchor_odometry.odometry.pose;
    let [anchor_qx, anchor_qy, anchor_qz, anchor_qw] = anchor_pose.orientation_xyzw;
    let anchor_norm_squared = anchor_qx * anchor_qx
        + anchor_qy * anchor_qy
        + anchor_qz * anchor_qz
        + anchor_qw * anchor_qw;
    assert!(anchor_norm_squared.is_finite() && anchor_norm_squared > f64::EPSILON);
    let anchor_inverse_norm = anchor_norm_squared.sqrt().recip();
    let anchor_x = anchor_qx * anchor_inverse_norm;
    let anchor_y = anchor_qy * anchor_inverse_norm;
    let anchor_z = anchor_qz * anchor_inverse_norm;
    let anchor_w = anchor_qw * anchor_inverse_norm;
    let anchor_yaw = (2.0 * (anchor_w * anchor_z + anchor_x * anchor_y))
        .atan2(1.0 - 2.0 * (anchor_y * anchor_y + anchor_z * anchor_z));
    assert!(anchor_yaw.is_finite());
    let normalize_yaw = |yaw: f64| {
        (yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
    };
    let anchor_source_yaw = normalize_yaw(anchor_yaw);
    let map_from_odom_yaw = normalize_yaw(-anchor_source_yaw);
    let (anchor_sin, anchor_cos) = map_from_odom_yaw.sin_cos();
    let anchor_position = anchor_pose.position;
    let map_from_odom_translation = [
        -(anchor_cos * anchor_position[0] - anchor_sin * anchor_position[1]),
        -(anchor_sin * anchor_position[0] + anchor_cos * anchor_position[1]),
        -anchor_position[2],
    ];
    let map_points = odom_points
        .iter()
        .map(|point| {
            let (sin_yaw, cos_yaw) = map_from_odom_yaw.sin_cos();
            nalgebra::Point3::new(
                cos_yaw * point.x - sin_yaw * point.y + map_from_odom_translation[0],
                sin_yaw * point.x + cos_yaw * point.y + map_from_odom_translation[1],
                point.z + map_from_odom_translation[2],
            )
        })
        .collect::<Vec<_>>();

    let base_anchor_map_from_odom = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(
            map_from_odom_translation[0],
            map_from_odom_translation[1],
            map_from_odom_translation[2],
        ),
        nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, map_from_odom_yaw),
    );
    let frozen_odom_from_body = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(source_position[0], source_position[1], source_position[2]),
        nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            source_orientation[3],
            source_orientation[0],
            source_orientation[1],
            source_orientation[2],
        )),
    );
    let body_from_map = (base_anchor_map_from_odom * frozen_odom_from_body).inverse();
    map_points
        .iter()
        .map(|point| body_from_map.transform_point(point))
        .collect()
}

#[cfg(feature = "nuwax-lio-test-child")]
fn independent_body_points_sha256(points: &[nalgebra::Point3<f64>]) -> [u8; 32] {
    assert!(!points.is_empty(), "body-point digest source is empty");
    let mut preimage = b"nuwax.correction.body-points.v1\0".to_vec();
    preimage.extend_from_slice(
        &u64::try_from(points.len())
            .expect("body-point count fits u64")
            .to_be_bytes(),
    );
    for point in points {
        assert!(point.coords.iter().all(|value| value.is_finite()));
        preimage.extend_from_slice(&point.x.to_be_bytes());
        preimage.extend_from_slice(&point.y.to_be_bytes());
        preimage.extend_from_slice(&point.z.to_be_bytes());
    }
    nuwax_codec::sha256_bytes(&preimage)
}

#[cfg(feature = "nuwax-lio-test-child")]
#[derive(Clone, Debug)]
struct DecodedCorrectionCandidate {
    backend: i32,
    transform_semantics: i32,
    runtime_generation: u64,
    backend_generation: u64,
    candidate_sequence: u64,
    mapping_session_id: String,
    mapping_session_generation: u64,
    map_frame_generation: u64,
    mapping_commit_boot_nonce: [u8; 16],
    mapping_commit_sequence: u64,
    occupancy_revision: u64,
    cloud_revision: u64,
    committed_map_source_timestamp_ns: u64,
    registered_cloud_source_sequence: u64,
    registered_cloud_source_timestamp_ns: u64,
    registered_cloud_point_count: u64,
    registered_cloud_envelope_sha256: [u8; 32],
    body_points_sha256: [u8; 32],
    correlated_odom_source_sequence: u64,
    correlated_odom_source_timestamp_ns: u64,
    correlated_odom_pose_sha256: [u8; 32],
    committed_map_width: u64,
    committed_map_height: u64,
    committed_map_resolution_m: f64,
    committed_map_origin_x_m: f64,
    committed_map_origin_y_m: f64,
    committed_map_payload_sha256: [u8; 32],
    committed_map_boundary_sha256: [u8; 32],
    source_timestamp_ns: u64,
    map_frame_id: String,
    odom_frame_id: String,
    body_frame_id: String,
    points_frame_id: String,
    correction_lease_boot_nonce: [u8; 16],
    correction_lease_generation: u64,
    translation_m: [f64; 3],
    rotation_xyzw: [f64; 4],
    payload_sha256: [u8; 32],
}

#[cfg(feature = "nuwax-lio-test-child")]
struct CandidateWireCursor<'a> {
    bytes: &'a [u8],
    offset: usize,
}

#[cfg(feature = "nuwax-lio-test-child")]
impl<'a> CandidateWireCursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn take<const N: usize>(&mut self, field: &str) -> Result<[u8; N], String> {
        let end = self
            .offset
            .checked_add(N)
            .ok_or_else(|| format!("{field} offset overflow"))?;
        let value = self
            .bytes
            .get(self.offset..end)
            .ok_or_else(|| format!("{field} truncated at byte {}", self.offset))?;
        self.offset = end;
        value
            .try_into()
            .map_err(|_| format!("{field} had the wrong encoded width"))
    }

    fn read_i32(&mut self, field: &str) -> Result<i32, String> {
        Ok(i32::from_be_bytes(self.take(field)?))
    }

    fn read_positive_u64(&mut self, field: &str) -> Result<u64, String> {
        let value = i64::from_be_bytes(self.take(field)?);
        u64::try_from(value)
            .ok()
            .filter(|value| *value > 0)
            .ok_or_else(|| format!("{field} must be a positive int64, got {value}"))
    }

    fn read_nonnegative_u64(&mut self, field: &str) -> Result<u64, String> {
        let value = i64::from_be_bytes(self.take(field)?);
        u64::try_from(value).map_err(|_| format!("{field} must be nonnegative, got {value}"))
    }

    fn read_f64(&mut self, field: &str) -> Result<f64, String> {
        Ok(f64::from_be_bytes(self.take(field)?))
    }

    fn read_string(&mut self, field: &str) -> Result<String, String> {
        let encoded_len = self.read_i32(&format!("{field} length"))?;
        let encoded_len = usize::try_from(encoded_len)
            .ok()
            .filter(|length| (2..=256).contains(length))
            .ok_or_else(|| format!("{field} has invalid LCM length {encoded_len}"))?;
        let end = self
            .offset
            .checked_add(encoded_len)
            .ok_or_else(|| format!("{field} length overflow"))?;
        let encoded = self
            .bytes
            .get(self.offset..end)
            .ok_or_else(|| format!("{field} is truncated"))?;
        self.offset = end;
        if encoded.last() != Some(&0) || encoded[..encoded.len() - 1].contains(&0) {
            return Err(format!(
                "{field} is not a canonical NUL-terminated LCM string"
            ));
        }
        let value = std::str::from_utf8(&encoded[..encoded.len() - 1])
            .map_err(|error| format!("{field} is not UTF-8: {error}"))?
            .to_owned();
        if value.trim() != value {
            return Err(format!("{field} contains leading or trailing whitespace"));
        }
        Ok(value)
    }

    fn finish(self) -> Result<(), String> {
        if self.offset == self.bytes.len() {
            Ok(())
        } else {
            Err(format!(
                "correction candidate has {} trailing bytes",
                self.bytes.len() - self.offset
            ))
        }
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
fn encode_candidate_lcm_string(value: &str, encoded: &mut Vec<u8>) {
    let length = i32::try_from(value.len() + 1).expect("candidate identity string length fits i32");
    encoded.extend_from_slice(&length.to_be_bytes());
    encoded.extend_from_slice(value.as_bytes());
    encoded.push(0);
}

#[cfg(feature = "nuwax-lio-test-child")]
fn native_candidate_payload_sha256(result: &native_pgo::NativePgoResult) -> [u8; 32] {
    let identity = &result.request.identity;
    let mut encoded = Vec::with_capacity(512);
    encoded.extend_from_slice(&1_i32.to_be_bytes());
    encoded.extend_from_slice(&native_pgo::NATIVE_PGO_BACKEND.to_be_bytes());
    encoded.extend_from_slice(
        &native_pgo::NATIVE_PGO_TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3.to_be_bytes(),
    );
    encoded.extend_from_slice(&identity.runtime_generation.to_be_bytes());
    encoded.extend_from_slice(&identity.backend_generation.to_be_bytes());
    encoded.extend_from_slice(&identity.request_sequence.to_be_bytes());
    encode_candidate_lcm_string(&identity.mapping_session_id, &mut encoded);
    encoded.extend_from_slice(&identity.mapping_session_generation.to_be_bytes());
    encoded.extend_from_slice(&identity.map_frame_generation.to_be_bytes());
    encoded.extend_from_slice(&identity.mapping_commit_id.boot_nonce);
    encoded.extend_from_slice(&identity.mapping_commit_id.sequence.to_be_bytes());
    encoded.extend_from_slice(&identity.occupancy_revision.to_be_bytes());
    encoded.extend_from_slice(&identity.cloud_revision.to_be_bytes());
    encoded.extend_from_slice(&identity.committed_map.source_timestamp_ns.to_be_bytes());
    encoded.extend_from_slice(&identity.registered_cloud.source_sequence.to_be_bytes());
    encoded.extend_from_slice(&identity.registered_cloud.source_timestamp_ns.to_be_bytes());
    encoded.extend_from_slice(&identity.registered_cloud.point_count.to_be_bytes());
    encoded.extend_from_slice(&identity.registered_cloud.envelope_sha256);
    encoded.extend_from_slice(&identity.registered_cloud.body_points_sha256);
    encoded.extend_from_slice(&identity.correlated_odom.source_sequence.to_be_bytes());
    encoded.extend_from_slice(&identity.correlated_odom.source_timestamp_ns.to_be_bytes());
    encoded.extend_from_slice(&identity.correlated_odom.pose_sha256);
    encoded.extend_from_slice(&identity.committed_map.width.to_be_bytes());
    encoded.extend_from_slice(&identity.committed_map.height.to_be_bytes());
    encoded.extend_from_slice(&identity.committed_map.resolution_m.to_be_bytes());
    encoded.extend_from_slice(&identity.committed_map.origin_x_m.to_be_bytes());
    encoded.extend_from_slice(&identity.committed_map.origin_y_m.to_be_bytes());
    encoded.extend_from_slice(&identity.committed_map.payload_sha256);
    encoded.extend_from_slice(&identity.committed_map.boundary_sha256);
    encoded.extend_from_slice(&identity.source_timestamp_ns.to_be_bytes());
    encode_candidate_lcm_string(&identity.map_frame_id, &mut encoded);
    encode_candidate_lcm_string(&identity.odom_frame_id, &mut encoded);
    encode_candidate_lcm_string(&identity.body_frame_id, &mut encoded);
    encode_candidate_lcm_string(&identity.points_frame_id, &mut encoded);
    encoded.extend_from_slice(&identity.correction_lease.boot_nonce);
    encoded.extend_from_slice(&identity.correction_lease.generation.to_be_bytes());
    for value in result
        .translation_m
        .iter()
        .chain(result.rotation_xyzw.iter())
    {
        encoded.extend_from_slice(&value.to_be_bytes());
    }
    nuwax_codec::sha256_bytes(&encoded)
}

#[cfg(feature = "nuwax-lio-test-child")]
fn decode_correction_candidate(frame: &RawLcmFrame) -> DecodedCorrectionCandidate {
    assert_eq!(
        frame.msg_type.fqn(),
        "nuwax_msgs.CorrectionCandidate",
        "correction candidate message type drifted"
    );
    assert!(
        frame.payload.len() <= CORRECTION_CANDIDATE_MAX_BODY_BYTES,
        "correction candidate exceeds the production wire bound"
    );
    let mut cursor = CandidateWireCursor::new(&frame.payload);
    let schema_version = cursor.read_i32("schema_version").unwrap();
    assert_eq!(schema_version, 1, "unsupported correction candidate schema");
    let backend = cursor.read_i32("backend").unwrap();
    assert!(
        (1..=3).contains(&backend),
        "unknown correction backend {backend}"
    );
    let transform_semantics = cursor.read_i32("transform_semantics").unwrap();
    assert_eq!(
        transform_semantics, 1,
        "correction is not absolute-left full SE(3)"
    );
    let runtime_generation = cursor.read_positive_u64("runtime_generation").unwrap();
    let backend_generation = cursor.read_positive_u64("backend_generation").unwrap();
    let candidate_sequence = cursor.read_positive_u64("candidate_sequence").unwrap();
    let mapping_session_id = cursor.read_string("mapping_session_id").unwrap();
    let mapping_session_generation = cursor
        .read_positive_u64("mapping_session_generation")
        .unwrap();
    let map_frame_generation = cursor.read_positive_u64("map_frame_generation").unwrap();
    let mapping_commit_boot_nonce = cursor.take("mapping_commit_boot_nonce").unwrap();
    let mapping_commit_sequence = cursor.read_positive_u64("mapping_commit_sequence").unwrap();
    let occupancy_revision = cursor.read_positive_u64("occupancy_revision").unwrap();
    let cloud_revision = cursor.read_positive_u64("cloud_revision").unwrap();
    let committed_map_source_timestamp_ns = cursor
        .read_nonnegative_u64("committed_map_source_timestamp_ns")
        .unwrap();
    let registered_cloud_source_sequence = cursor
        .read_positive_u64("registered_cloud_source_sequence")
        .unwrap();
    let registered_cloud_source_timestamp_ns = cursor
        .read_nonnegative_u64("registered_cloud_source_timestamp_ns")
        .unwrap();
    let registered_cloud_point_count = cursor
        .read_positive_u64("registered_cloud_point_count")
        .unwrap();
    let registered_cloud_envelope_sha256 = cursor.take("registered_cloud_envelope_sha256").unwrap();
    let body_points_sha256 = cursor.take("body_points_sha256").unwrap();
    let correlated_odom_source_sequence = cursor
        .read_positive_u64("correlated_odom_source_sequence")
        .unwrap();
    let correlated_odom_source_timestamp_ns = cursor
        .read_nonnegative_u64("correlated_odom_source_timestamp_ns")
        .unwrap();
    let correlated_odom_pose_sha256 = cursor.take("correlated_odom_pose_sha256").unwrap();
    let committed_map_width = cursor.read_positive_u64("committed_map_width").unwrap();
    let committed_map_height = cursor.read_positive_u64("committed_map_height").unwrap();
    let committed_map_resolution_m = cursor.read_f64("committed_map_resolution_m").unwrap();
    let committed_map_origin_x_m = cursor.read_f64("committed_map_origin_x_m").unwrap();
    let committed_map_origin_y_m = cursor.read_f64("committed_map_origin_y_m").unwrap();
    let committed_map_payload_sha256 = cursor.take("committed_map_payload_sha256").unwrap();
    let committed_map_boundary_sha256 = cursor.take("committed_map_boundary_sha256").unwrap();
    let source_timestamp_ns = cursor.read_nonnegative_u64("source_timestamp_ns").unwrap();
    let map_frame_id = cursor.read_string("map_frame_id").unwrap();
    let odom_frame_id = cursor.read_string("odom_frame_id").unwrap();
    let body_frame_id = cursor.read_string("body_frame_id").unwrap();
    let points_frame_id = cursor.read_string("points_frame_id").unwrap();
    let correction_lease_boot_nonce = cursor.take("correction_lease_boot_nonce").unwrap();
    let correction_lease_generation = cursor
        .read_positive_u64("correction_lease_generation")
        .unwrap();
    let mut translation_m = [0.0; 3];
    for (axis, value) in translation_m.iter_mut().enumerate() {
        *value = cursor.read_f64(&format!("translation_m[{axis}]")).unwrap();
    }
    let mut rotation_xyzw = [0.0; 4];
    for (axis, value) in rotation_xyzw.iter_mut().enumerate() {
        *value = cursor.read_f64(&format!("rotation_xyzw[{axis}]")).unwrap();
    }
    let payload_sha256 = cursor.take("payload_sha256").unwrap();
    cursor.finish().unwrap();

    for (name, digest) in [
        (
            "mapping_commit_boot_nonce",
            mapping_commit_boot_nonce.as_slice(),
        ),
        (
            "registered_cloud_envelope_sha256",
            registered_cloud_envelope_sha256.as_slice(),
        ),
        ("body_points_sha256", body_points_sha256.as_slice()),
        (
            "correlated_odom_pose_sha256",
            correlated_odom_pose_sha256.as_slice(),
        ),
        (
            "committed_map_payload_sha256",
            committed_map_payload_sha256.as_slice(),
        ),
        (
            "committed_map_boundary_sha256",
            committed_map_boundary_sha256.as_slice(),
        ),
        (
            "correction_lease_boot_nonce",
            correction_lease_boot_nonce.as_slice(),
        ),
        ("payload_sha256", payload_sha256.as_slice()),
    ] {
        assert!(digest.iter().any(|byte| *byte != 0), "{name} is absent");
    }
    assert!(registered_cloud_point_count <= 100_000);
    assert!(correlated_odom_source_timestamp_ns <= registered_cloud_source_timestamp_ns);
    if backend == 3 {
        assert!(
            committed_map_source_timestamp_ns <= registered_cloud_source_timestamp_ns,
            "relocalization committed map is newer than its registered cloud"
        );
    } else {
        assert_eq!(
            committed_map_source_timestamp_ns, registered_cloud_source_timestamp_ns,
            "committed map and registered cloud timestamps diverged"
        );
    }
    assert_eq!(
        source_timestamp_ns, registered_cloud_source_timestamp_ns,
        "candidate source timestamp diverged from registered cloud"
    );
    assert!(
        committed_map_width
            .checked_mul(committed_map_height)
            .is_some(),
        "committed map dimensions overflow"
    );
    assert!(committed_map_resolution_m.is_finite() && committed_map_resolution_m > 0.0);
    assert!(committed_map_origin_x_m.is_finite() && committed_map_origin_y_m.is_finite());
    assert_ne!(map_frame_id, odom_frame_id);
    assert_ne!(map_frame_id, body_frame_id);
    assert_ne!(odom_frame_id, body_frame_id);
    assert_eq!(points_frame_id, body_frame_id);
    assert!(
        translation_m
            .iter()
            .chain(rotation_xyzw.iter())
            .all(|value| value.is_finite()),
        "correction transform contains a non-finite component"
    );
    assert!(
        translation_m.iter().all(|value| value.abs() <= 1_000_000.0),
        "correction translation exceeds the production bound"
    );
    let rotation_norm_squared = rotation_xyzw.iter().map(|value| value * value).sum::<f64>();
    assert!(
        (rotation_norm_squared - 1.0).abs() <= 1.0e-6,
        "correction quaternion is not unit length: norm_squared={rotation_norm_squared}"
    );

    let payload_prefix_len = frame
        .payload
        .len()
        .checked_sub(payload_sha256.len())
        .expect("candidate body includes its payload digest");
    assert_eq!(
        nuwax_codec::sha256_bytes(&frame.payload[..payload_prefix_len]),
        payload_sha256,
        "correction candidate payload SHA-256 mismatch"
    );
    let mut map_boundary = b"nuwax.correction.committed-map-boundary.v1\0".to_vec();
    encode_candidate_lcm_string(&map_frame_id, &mut map_boundary);
    map_boundary.extend_from_slice(
        &i64::try_from(committed_map_width)
            .expect("committed map width fits int64")
            .to_be_bytes(),
    );
    map_boundary.extend_from_slice(
        &i64::try_from(committed_map_height)
            .expect("committed map height fits int64")
            .to_be_bytes(),
    );
    map_boundary.extend_from_slice(&committed_map_resolution_m.to_be_bytes());
    map_boundary.extend_from_slice(&committed_map_origin_x_m.to_be_bytes());
    map_boundary.extend_from_slice(&committed_map_origin_y_m.to_be_bytes());
    assert_eq!(
        nuwax_codec::sha256_bytes(&map_boundary),
        committed_map_boundary_sha256,
        "correction committed-map boundary SHA-256 mismatch"
    );

    DecodedCorrectionCandidate {
        backend,
        transform_semantics,
        runtime_generation,
        backend_generation,
        candidate_sequence,
        mapping_session_id,
        mapping_session_generation,
        map_frame_generation,
        mapping_commit_boot_nonce,
        mapping_commit_sequence,
        occupancy_revision,
        cloud_revision,
        committed_map_source_timestamp_ns,
        registered_cloud_source_sequence,
        registered_cloud_source_timestamp_ns,
        registered_cloud_point_count,
        registered_cloud_envelope_sha256,
        body_points_sha256,
        correlated_odom_source_sequence,
        correlated_odom_source_timestamp_ns,
        correlated_odom_pose_sha256,
        committed_map_width,
        committed_map_height,
        committed_map_resolution_m,
        committed_map_origin_x_m,
        committed_map_origin_y_m,
        committed_map_payload_sha256,
        committed_map_boundary_sha256,
        source_timestamp_ns,
        map_frame_id,
        odom_frame_id,
        body_frame_id,
        points_frame_id,
        correction_lease_boot_nonce,
        correction_lease_generation,
        translation_m,
        rotation_xyzw,
        payload_sha256,
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn next_nonempty_global_map(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    scenario: &str,
) -> RawLcmFrame {
    loop {
        let frame = next_frame(runtime, stream, scenario);
        let global_map = decode_global_map(&frame);
        if global_map.width > 0 && !global_map.data.is_empty() {
            return frame;
        }
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
fn xyz_points(cloud: &schema::PointCloud2) -> Vec<[f64; 3]> {
    assert!(!cloud.is_bigendian, "fixture cloud must be little-endian");
    let point_step = usize::try_from(cloud.point_step).expect("fixture point_step must fit usize");
    assert!(point_step >= 12, "fixture cloud must contain packed XYZ");
    assert_eq!(
        cloud.data.len(),
        usize::try_from(cloud.width).expect("fixture width must fit usize")
            * usize::try_from(cloud.height).expect("fixture height must fit usize")
            * point_step,
        "fixture cloud dimensions drifted from its payload"
    );
    cloud
        .data
        .chunks_exact(point_step)
        .map(|point| {
            [
                f32::from_le_bytes(point[0..4].try_into().expect("x bytes")) as f64,
                f32::from_le_bytes(point[4..8].try_into().expect("y bytes")) as f64,
                f32::from_le_bytes(point[8..12].try_into().expect("z bytes")) as f64,
            ]
        })
        .collect()
}

#[cfg(feature = "nuwax-lio-test-child")]
fn assert_near(actual: f64, expected: f64, tolerance: f64, label: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{label}: expected {expected:.12}, got {actual:.12}, tolerance={tolerance}"
    );
}

#[cfg(feature = "nuwax-lio-test-child")]
fn wait_for_json(
    path: &Path,
    timeout: Duration,
    ready: impl Fn(&serde_json::Value) -> bool,
) -> Result<serde_json::Value, String> {
    let deadline = Instant::now() + timeout;
    loop {
        match fs::read_to_string(path) {
            Ok(contents) => match serde_json::from_str::<serde_json::Value>(&contents) {
                Ok(value) if ready(&value) => return Ok(value),
                Ok(_) | Err(_) => {}
            },
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(format!("read {}: {error}", path.display())),
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "{} did not reach the expected JSON state within {timeout:?}",
                path.display()
            ));
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
type GeometryClient =
    native_fixture_proto::geometry_runtime_client::GeometryRuntimeClient<tonic::transport::Channel>;

#[cfg(feature = "nuwax-lio-test-child")]
fn connect_geometry(
    runtime: &tokio::runtime::Runtime,
    probe: &ProbeDir,
    scenario: &str,
) -> GeometryClient {
    runtime.block_on(async {
        let endpoint = format!("http://127.0.0.1:{}", probe.runtime_port());
        tokio::time::timeout(
            NATIVE_FIXTURE_TIMEOUT,
            GeometryClient::connect(endpoint.clone()),
        )
        .await
        .unwrap_or_else(|_| panic!("{scenario}: connect timed out"))
        .unwrap_or_else(|error| panic!("{scenario}: connect to {endpoint}: {error}"))
    })
}

#[cfg(feature = "nuwax-lio-test-child")]
fn start_mapping(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    scenario: &str,
) -> native_fixture_proto::MappingSessionState {
    runtime.block_on(async {
        tokio::time::timeout(
            NATIVE_FIXTURE_TIMEOUT,
            client.start_mapping(native_fixture_proto::StartMappingRequest {
                session_id: format!("{scenario}-session"),
                map_name: "g014-native-aggregate".to_owned(),
            }),
        )
        .await
        .unwrap_or_else(|_| panic!("{scenario}: StartMapping timed out"))
        .unwrap_or_else(|error| panic!("{scenario}: StartMapping failed: {error}"))
        .into_inner()
    })
}

#[cfg(feature = "nuwax-lio-test-child")]
fn stop_mapping(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    session_id: &str,
    scenario: &str,
) -> native_fixture_proto::MappingSessionState {
    runtime.block_on(async {
        tokio::time::timeout(
            NATIVE_FIXTURE_TIMEOUT,
            client.stop_mapping(native_fixture_proto::StopMappingRequest {
                session_id: session_id.to_owned(),
                reason: "G015 V2 stop fence".to_owned(),
            }),
        )
        .await
        .unwrap_or_else(|_| panic!("{scenario}: StopMapping timed out"))
        .unwrap_or_else(|error| panic!("{scenario}: StopMapping failed: {error}"))
        .into_inner()
    })
}

#[cfg(feature = "nuwax-lio-test-child")]
fn get_mapping_state(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    scenario: &str,
) -> native_fixture_proto::MappingSessionState {
    runtime.block_on(async {
        tokio::time::timeout(
            NATIVE_FIXTURE_TIMEOUT,
            client.get_mapping_state(native_fixture_proto::Empty {}),
        )
        .await
        .unwrap_or_else(|_| panic!("{scenario}: GetMappingState timed out"))
        .unwrap_or_else(|error| panic!("{scenario}: GetMappingState failed: {error}"))
        .into_inner()
    })
}

#[cfg(feature = "nuwax-lio-test-child")]
fn get_frame_tree(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    scenario: &str,
) -> native_fixture_proto::FrameTree {
    runtime.block_on(async {
        tokio::time::timeout(
            NATIVE_FIXTURE_TIMEOUT,
            client.get_frame_tree(native_fixture_proto::Empty {}),
        )
        .await
        .unwrap_or_else(|_| panic!("{scenario}: GetFrameTree timed out"))
        .unwrap_or_else(|error| panic!("{scenario}: GetFrameTree failed: {error}"))
        .into_inner()
    })
}

#[cfg(feature = "nuwax-lio-test-child")]
fn transform_isometry(translation: [f64; 3], rotation_xyzw: [f64; 4]) -> nalgebra::Isometry3<f64> {
    nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(translation[0], translation[1], translation[2]),
        nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            rotation_xyzw[3],
            rotation_xyzw[0],
            rotation_xyzw[1],
            rotation_xyzw[2],
        )),
    )
}

#[cfg(feature = "nuwax-lio-test-child")]
fn assert_transform_matches_isometry(
    translation: [f64; 3],
    rotation_xyzw: [f64; 4],
    expected: &nalgebra::Isometry3<f64>,
    label: &str,
) {
    for (axis, (actual, expected)) in translation
        .into_iter()
        .zip(expected.translation.vector.iter().copied())
        .enumerate()
    {
        assert_near(
            actual,
            expected,
            1.0e-9,
            &format!("{label} translation[{axis}]"),
        );
    }
    let actual_norm = rotation_xyzw
        .iter()
        .map(|component| component * component)
        .sum::<f64>()
        .sqrt();
    assert!(
        actual_norm.is_finite() && actual_norm > f64::EPSILON,
        "{label}: actual quaternion is invalid"
    );
    let expected_quaternion = expected.rotation.quaternion();
    let expected_xyzw = [
        expected_quaternion.i,
        expected_quaternion.j,
        expected_quaternion.k,
        expected_quaternion.w,
    ];
    let expected_norm = expected_xyzw
        .iter()
        .map(|component| component * component)
        .sum::<f64>()
        .sqrt();
    let dot = rotation_xyzw
        .into_iter()
        .zip(expected_xyzw)
        .map(|(actual, expected)| actual / actual_norm * (expected / expected_norm))
        .sum::<f64>()
        .abs()
        .clamp(0.0, 1.0);
    let sign_invariant_angle = 2.0 * dot.acos();
    assert!(
        sign_invariant_angle <= 1.0e-9,
        "{label}: sign-invariant quaternion angle {sign_invariant_angle:.12e} exceeds 1e-9"
    );
}

#[cfg(feature = "nuwax-lio-test-child")]
fn schema_transform_timestamp_ns(transform: &schema::TransformStamped) -> u64 {
    assert!(transform.header.stamp_sec >= 0);
    assert!((0..1_000_000_000).contains(&transform.header.stamp_nsec));
    u64::try_from(transform.header.stamp_sec).expect("TF seconds are nonnegative") * 1_000_000_000
        + u64::try_from(transform.header.stamp_nsec).expect("TF nanoseconds are nonnegative")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn map_from_odom_in_frame(frame: &RawLcmFrame) -> schema::TransformStamped {
    decode_tf(frame)
        .transforms
        .into_iter()
        .find(|transform| transform.header.frame_id == "map" && transform.child_frame_id == "odom")
        .expect("canonical map->odom transform")
}

#[cfg(feature = "nuwax-lio-test-child")]
fn wait_for_exact_map_from_odom(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    expected: &nalgebra::Isometry3<f64>,
    expected_timestamp_ns: u64,
    expected_correction_revision: u32,
    scenario: &str,
) -> RawLcmFrame {
    let deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        assert!(
            !remaining.is_zero(),
            "{scenario}: exact corrected map->odom TF did not arrive"
        );
        let frame = runtime.block_on(async {
            tokio::time::timeout(remaining, stream.next())
                .await
                .unwrap_or_else(|_| panic!("{scenario}: canonical TF timed out"))
                .unwrap_or_else(|| panic!("{scenario}: canonical TF subscription closed"))
                .unwrap_or_else(|error| panic!("{scenario}: canonical TF receive failed: {error}"))
        });
        let tf = decode_tf(&frame);
        let Some(transform) = tf.transforms.iter().find(|transform| {
            transform.header.frame_id == "map"
                && transform.child_frame_id == "odom"
                && schema_transform_timestamp_ns(transform) == expected_timestamp_ns
        }) else {
            continue;
        };
        assert_eq!(
            transform.header.seq,
            i32::try_from(expected_correction_revision).unwrap(),
            "{scenario}: correction revision drifted in canonical TF"
        );
        assert_transform_matches_isometry(
            transform.transform.translation,
            transform.transform.rotation_xyzw,
            expected,
            scenario,
        );
        return frame;
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
fn assert_frame_tree_map_from_odom(
    tree: &native_fixture_proto::FrameTree,
    expected: &nalgebra::Isometry3<f64>,
    expected_timestamp_ns: u64,
    scenario: &str,
) {
    let transform = tree
        .transforms
        .iter()
        .find(|transform| transform.parent_frame == "map" && transform.child_frame == "odom")
        .unwrap_or_else(|| panic!("{scenario}: GetFrameTree omitted map->odom"));
    let translation = transform
        .translation
        .as_ref()
        .unwrap_or_else(|| panic!("{scenario}: GetFrameTree omitted map->odom translation"));
    let rotation = transform
        .rotation
        .as_ref()
        .unwrap_or_else(|| panic!("{scenario}: GetFrameTree omitted map->odom rotation"));
    assert_eq!(
        transform.timestamp_ns, expected_timestamp_ns,
        "{scenario}: GetFrameTree returned the wrong correction timestamp"
    );
    assert_transform_matches_isometry(
        [translation.x, translation.y, translation.z],
        [rotation.x, rotation.y, rotation.z, rotation.w],
        expected,
        scenario,
    );
}

#[cfg(feature = "nuwax-lio-test-child")]
fn correlated_odom_pose_sha256(odometry: &lio::LioOdometryEnvelope) -> [u8; 32] {
    let mut canonical = b"nuwax.correction.odom-from-body.v1\0".to_vec();
    for value in odometry.odometry.pose.position {
        canonical.extend_from_slice(&value.to_be_bytes());
    }
    for value in odometry.odometry.pose.orientation_xyzw {
        canonical.extend_from_slice(&value.to_be_bytes());
    }
    nuwax_codec::sha256_bytes(&canonical)
}

#[cfg(feature = "nuwax-lio-test-child")]
fn digest_hex(digest: &[u8]) -> String {
    use std::fmt::Write as _;

    let mut encoded = String::with_capacity(digest.len() * 2);
    for byte in digest {
        write!(&mut encoded, "{byte:02x}").expect("writing to a String cannot fail");
    }
    encoded
}

#[cfg(feature = "nuwax-lio-test-child")]
#[derive(Debug, serde::Deserialize, PartialEq, Eq)]
enum ObservedCandidateDisposition {
    Applied,
    AlreadyApplied,
    Rejected,
}

#[cfg(feature = "nuwax-lio-test-child")]
#[derive(Debug, serde::Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct ObservedCandidateFinalization {
    event: String,
    runtime_generation: u64,
    backend_generation: u64,
    session_generation: u64,
    candidate_sequence: u64,
    payload_sha256: String,
    disposition: ObservedCandidateDisposition,
    #[serde(default)]
    reject_reason: Option<String>,
    candidate_processed_delivered: bool,
}

#[cfg(feature = "nuwax-lio-test-child")]
fn candidate_finalization_events(output: &Output) -> Vec<ObservedCandidateFinalization> {
    let event_prefix = format!(r#"{{"event":"{CORRECTION_FINALIZATION_EVENT}""#);
    output
        .stdout
        .iter()
        .chain(output.stderr.iter())
        .copied()
        .collect::<Vec<_>>()
        .split(|byte| *byte == b'\n')
        .filter_map(|line| {
            let line = String::from_utf8_lossy(line);
            let start = line.find(&event_prefix)?;
            let end = line[start..]
                .rfind('}')
                .map(|offset| start + offset + 1)
                .unwrap_or_else(|| {
                    panic!(
                        "{CORRECTION_FINALIZATION_TARGET}: structured finalizer event is truncated: {line}"
                    )
                });
            Some(
                serde_json::from_str(&line[start..end]).unwrap_or_else(|error| {
                    panic!(
                        "{CORRECTION_FINALIZATION_TARGET}: decode structured finalizer event: {error}; line={line}"
                    )
                }),
            )
        })
        .collect()
}

#[cfg(feature = "nuwax-lio-test-child")]
fn get_occupancy_data(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    scenario: &str,
) -> native_fixture_proto::OccupancyGridData {
    runtime.block_on(async {
        tokio::time::timeout(
            NATIVE_FIXTURE_TIMEOUT,
            client.get_occupancy_grid_data(native_fixture_proto::Empty {}),
        )
        .await
        .unwrap_or_else(|_| panic!("{scenario}: GetOccupancyGridData timed out"))
        .unwrap_or_else(|error| panic!("{scenario}: GetOccupancyGridData failed: {error}"))
        .into_inner()
    })
}

#[cfg(feature = "nuwax-lio-test-child")]
fn get_occupancy_shm(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    scenario: &str,
    deadline: Instant,
) -> native_fixture_proto::ShmRef {
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        assert!(
            !remaining.is_zero(),
            "{scenario}: GetOccupancyGridShm remained unavailable before deadline"
        );
        match runtime.block_on(async {
            tokio::time::timeout(
                remaining,
                client.get_occupancy_grid_shm(native_fixture_proto::Empty {}),
            )
            .await
        }) {
            Ok(Ok(response)) => break response.into_inner(),
            Ok(Err(error)) if error.code() == tonic::Code::FailedPrecondition => {
                assert!(
                    Instant::now() < deadline,
                    "{scenario}: GetOccupancyGridShm remained unavailable: {error}"
                );
                std::thread::sleep(
                    Duration::from_millis(10)
                        .min(deadline.saturating_duration_since(Instant::now())),
                );
            }
            Ok(Err(error)) => panic!("{scenario}: GetOccupancyGridShm failed: {error}"),
            Err(_) => panic!("{scenario}: GetOccupancyGridShm timed out before deadline"),
        }
    }
}

#[cfg(feature = "nuwax-lio-test-child")]
fn stop_geometry(runtime: &tokio::runtime::Runtime, client: &mut GeometryClient, scenario: &str) {
    let response = runtime.block_on(async {
        tokio::time::timeout(
            NATIVE_FIXTURE_TIMEOUT,
            client.stop(native_fixture_proto::StopRequest {}),
        )
        .await
        .unwrap_or_else(|_| panic!("{scenario}: Stop timed out"))
        .unwrap_or_else(|error| panic!("{scenario}: Stop failed: {error}"))
        .into_inner()
    });
    assert!(response.success, "{scenario}: Stop returned success=false");
}

#[cfg(feature = "nuwax-lio-test-child")]
fn wait_for_status(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    generation: u64,
    lifecycle: i32,
    scenario: &str,
) -> lio::LioStatus {
    loop {
        let status = decode_status(&next_frame(runtime, stream, "public LIO status"));
        if status.runtime_generation == generation as i64 && status.lifecycle_state == lifecycle {
            assert_eq!(
                status.reason_code,
                if lifecycle == lio::LIO_LIFECYCLE_STOPPING
                    || lifecycle == lio::LIO_LIFECYCLE_STOPPED
                {
                    lio::LIO_REASON_SHUTDOWN_REQUESTED
                } else {
                    lio::LIO_REASON_NONE
                },
                "{scenario}: lifecycle reason drifted"
            );
            return status;
        }
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
struct RunningNativeCycle {
    scenario: String,
    backend_code: i32,
    child: ChildGuard,
    pid: u32,
    native_cleanup: NativeChildCleanup,
    ready_path: PathBuf,
    term_path: PathBuf,
    contract_path: PathBuf,
    generation: u64,
    private_odometry: LcmFrameStream,
    private_cloud: LcmFrameStream,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn spawn_native_cycle(
    runtime: &tokio::runtime::Runtime,
    observers: &NativeObservers,
    probe: &ProbeDir,
    profile: (&str, &str, &str),
    fixture_path: &Path,
    permit_path: &Path,
) -> RunningNativeCycle {
    let (backend, mapping_backend, correction_backend) = profile;
    let scenario = format!("g014-real-native-{backend}");
    let ready_path = probe.path().join("geometry.ready");
    let pid_path = probe.path().join(format!("{backend}-native-child.pid"));
    let term_path = probe.path().join(format!("{backend}-native-child.term"));
    let contract_path = probe
        .path()
        .join(format!("{backend}-native-child-contract.json"));
    let native_cleanup = NativeChildCleanup::new(pid_path.clone());
    let private_prefix = "/g14";
    let executable = Path::new(env!("CARGO_BIN_EXE_nuwax-lio-test-child"))
        .canonicalize()
        .expect("canonicalize native LIO test child");

    assert!(
        !ready_path.exists(),
        "{scenario}: stale ready file survived the previous generation"
    );
    let mut command = runtime_command("real-lcm", probe);
    command
        .env("NUWAX_GEOMETRY_LIO_BACKEND", backend)
        .env("NUWAX_GEOMETRY_MAPPING_BACKEND", mapping_backend)
        .env("NUWAX_GEOMETRY_CORRECTION_BACKEND", correction_backend)
        .env("NUWAX_GEOMETRY_CANONICAL_MAP_WRITER", "geometry")
        .env("NUWAX_GEOMETRY_CANONICAL_CORRECTION_WRITER", "geometry")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE", &executable)
        .env(
            "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR",
            concat!(
                r#"{"translation_m":[0.25,-0.5,1.0],"#,
                r#""rotation_row_major":[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]}"#
            ),
        )
        .env("NUWAX_GEOMETRY_NATIVE_LIO_PRIVATE_PREFIX", private_prefix)
        .env("NUWAX_GEOMETRY_NATIVE_LIO_FIRST_PAIR_TIMEOUT_MS", "10000")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_PAIR_MAX_SKEW_MS", "5")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_HOST_IP", "192.0.2.10")
        .env("NUWAX_GEOMETRY_NATIVE_LIO_LIDAR_IP", "192.0.2.11")
        .env("NUWAX_GEOMETRY_LIDAR_FRAME", "g14_lidar")
        .env("NUWAX_LIO_TEST_CHILD_PID_FILE", &pid_path)
        .env("NUWAX_LIO_TEST_CHILD_TERM_FILE", &term_path)
        .env("NUWAX_LIO_TEST_CHILD_CONTRACT_FILE", &contract_path)
        .env("NUWAX_LIO_TEST_CHILD_RECORDED_FIXTURE", fixture_path)
        .env("NUWAX_LIO_TEST_CHILD_PUBLISH_PERMIT_FILE", permit_path);
    if mapping_backend == "raytracing" {
        command
            .env("NUWAX_MAPPING_RAYTRACING_MIN_HEALTH", "0")
            .env("NUWAX_MAPPING_RAYTRACING_MAX_HEALTH", "1");
    }
    let mut child = ChildGuard::new(
        command
            .spawn()
            .unwrap_or_else(|error| panic!("{scenario}: spawn Geometry runtime: {error}")),
    );

    let pid = wait_for_file_contents(&pid_path, NATIVE_FIXTURE_TIMEOUT)
        .unwrap_or_else(|error| panic!("{scenario}: {error}"))
        .parse::<u32>()
        .unwrap_or_else(|error| panic!("{scenario}: invalid native child pid: {error}"));
    #[cfg(target_os = "linux")]
    child.discover_native_pgo_on_cleanup(
        pid,
        nuwax_nav_pgo_bridge::native_executable()
            .canonicalize()
            .expect("canonicalize native PGO executable for panic cleanup"),
        scenario.clone(),
    );
    let contract = wait_for_json(&contract_path, NATIVE_FIXTURE_TIMEOUT, |_| true)
        .unwrap_or_else(|error| panic!("{scenario}: {error}"));
    assert_eq!(contract["backend"], backend, "{scenario}: backend mismatch");
    assert_eq!(
        contract["lcm_url"],
        probe.lcm_url(),
        "{scenario}: child did not inherit the Geometry LCM provider"
    );
    assert_eq!(contract["frame_id"], "odom");
    assert_eq!(contract["child_frame_id"], "base_link");
    assert_eq!(contract["sensor_frame_id"], "g14_lidar");
    assert_eq!(
        contract["extrinsic_translation_m"],
        serde_json::json!([0.25, -0.5, 1.0])
    );
    assert_eq!(
        contract["extrinsic_rotation_row_major"],
        serde_json::json!([0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    );
    assert_eq!(contract["host_ip"], "192.0.2.10");
    assert_eq!(contract["lidar_ip"], "192.0.2.11");
    assert_eq!(
        contract["recorded_fixture"].as_str(),
        fixture_path.to_str(),
        "{scenario}: recorded fixture path drifted"
    );
    assert_eq!(
        contract["publish_permit_file"].as_str(),
        permit_path.to_str(),
        "{scenario}: publish permit path drifted"
    );
    assert_eq!(contract["playback"]["mode"], "recorded");
    let expected_frame_count = serde_json::from_slice::<Vec<serde_json::Value>>(
        &fs::read(fixture_path).expect("read native LIO fixture for expected frame count"),
    )
    .expect("decode native LIO fixture for expected frame count")
    .len();
    assert_eq!(
        contract["playback"]["input_frame_count"].as_u64(),
        Some(expected_frame_count as u64)
    );
    assert_eq!(
        contract["playback"]["processed_output_count"].as_u64(),
        Some(expected_frame_count as u64)
    );
    assert_eq!(contract["published_pair_count"], 0);

    let odometry_pattern = contract["odometry_topic"]
        .as_str()
        .expect("native odometry topic must be a string");
    let cloud_pattern = contract["lidar_topic"]
        .as_str()
        .expect("native cloud topic must be a string");
    let generation = private_topic_generation(
        odometry_pattern,
        private_prefix,
        backend,
        "odometry",
        "nav_msgs.Odometry",
    );
    assert_eq!(
        generation,
        private_topic_generation(
            cloud_pattern,
            private_prefix,
            backend,
            "cloud",
            "sensor_msgs.PointCloud2",
        ),
        "{scenario}: native source streams used different generations"
    );
    let (private_odometry, private_cloud) = runtime.block_on(async {
        let odometry = binding_from_pattern(odometry_pattern, "nav_msgs.Odometry");
        let cloud = binding_from_pattern(cloud_pattern, "sensor_msgs.PointCloud2");
        (
            observers
                .transport
                .subscribe(odometry.0, Some(odometry.1))
                .await,
            observers.transport.subscribe(cloud.0, Some(cloud.1)).await,
        )
    });

    RunningNativeCycle {
        scenario,
        backend_code: match backend {
            "fastlio2" => lio::LIO_BACKEND_FASTLIO2,
            "pointlio" => lio::LIO_BACKEND_POINTLIO,
            value => panic!("unsupported aggregate fixture backend {value}"),
        },
        child,
        pid,
        native_cleanup,
        ready_path,
        term_path,
        contract_path,
        generation,
        private_odometry,
        private_cloud,
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn write_publish_permit(path: &Path, permit: u32) {
    let file_name = path.file_name().expect("publish permit file name");
    let mut temp_name = file_name.to_os_string();
    temp_name.push(format!(".{}.{}.tmp", std::process::id(), permit));
    let temp_path = path.with_file_name(temp_name);
    fs::write(&temp_path, permit.to_string()).unwrap_or_else(|error| {
        panic!(
            "write temporary publish permit {}: {error}",
            temp_path.display()
        )
    });
    fs::rename(&temp_path, path).unwrap_or_else(|error| {
        panic!(
            "atomically replace publish permit {} from {}: {error}",
            path.display(),
            temp_path.display()
        )
    });
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_wait_native_publication_ack(cycle: &RunningNativeCycle, sequence: u32) {
    let expected = u64::from(sequence);
    let contract = wait_for_json(&cycle.contract_path, NATIVE_FIXTURE_TIMEOUT, |value| {
        value["published_pair_count"].as_u64() == Some(expected)
            && value["last_published_sequence"].as_u64() == Some(expected)
    })
    .unwrap_or_else(|error| {
        panic!(
            "{}: native source pair {sequence} was not acknowledged: {error}",
            cycle.scenario
        )
    });
    assert_eq!(
        contract["published_pair_count"].as_u64(),
        Some(expected),
        "{}: native source pair count drifted",
        cycle.scenario
    );
    assert_eq!(
        contract["last_published_sequence"].as_u64(),
        Some(expected),
        "{}: native source sequence acknowledgement drifted",
        cycle.scenario
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn write_library_pgo_recorded_fixture(probe: &ProbeDir) -> PathBuf {
    let points = (0..60)
        .map(|index| {
            let x = index % 5;
            let y = (index / 5) % 4;
            let z = index / 20;
            serde_json::json!({
                "offset_ns": index * 500_000,
                "x": x as f64 * 0.41 + (y * y + z) as f64 * 0.013,
                "y": y as f64 * 0.43 + (x * z + z * z) as f64 * 0.017,
                "z": z as f64 * 0.37 + (x * x + y) as f64 * 0.011,
                "intensity": 10.0 + index as f64,
            })
        })
        .collect::<Vec<_>>();
    let acceleration_x = [
        0.0, 0.25, -0.25, -0.25, 0.25, 0.25, -0.25, -0.25, 0.25, 0.3, -0.15, 0.1,
    ];
    let frames = acceleration_x
        .into_iter()
        .enumerate()
        .map(|(index, acceleration_x)| {
            let timestamp_ns = 1_000_000_000_i64 + index as i64 * 4_000_000_000;
            serde_json::json!({
                "cloud": {
                    "timestamp_ns": timestamp_ns,
                    "frame_id": "mid360_link",
                    "points": points.clone(),
                },
                "imu": [
                    {
                        "timestamp_ns": timestamp_ns - 100_000_000,
                        "accel_mps2": [acceleration_x, 0.0, 0.0],
                        "gyro_rps": [0.0, 0.0, 0.0],
                    },
                    {
                        "timestamp_ns": timestamp_ns - 50_000_000,
                        "accel_mps2": [acceleration_x, 0.0, 0.0],
                        "gyro_rps": [0.0, 0.0, 0.0],
                    }
                ]
            })
        })
        .collect::<Vec<_>>();
    let path = probe.path().join("library-pgo-recorded-scan-imu.json");
    fs::write(
        &path,
        serde_json::to_vec(&frames).expect("encode library PGO fixture"),
    )
    .expect("write library PGO fixture");
    path
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn write_native_pgo_recorded_fixture(probe: &ProbeDir) -> PathBuf {
    let points = (0..60)
        .map(|index| {
            let x = index % 5;
            let y = (index / 5) % 4;
            let z = index / 20;
            serde_json::json!({
                "offset_ns": index * 500_000,
                "x": x as f64 * 0.41 + (y * y + z) as f64 * 0.013,
                "y": y as f64 * 0.43 + (x * z + z * z) as f64 * 0.017,
                "z": z as f64 * 0.37 + (x * x + y) as f64 * 0.011,
                "intensity": 10.0 + index as f64,
            })
        })
        .collect::<Vec<_>>();
    let timestamps_ns = [
        1_000_000_000_i64,
        60_000_000_000,
        119_000_000_000,
        178_000_000_000,
        179_000_000_000,
    ];
    let acceleration_x = [0.0, 0.0, 4.0 / 3_481.0, -11.6 / 3_481.0, 15.2 / 59.0];
    let frames = timestamps_ns
        .into_iter()
        .zip(acceleration_x)
        .enumerate()
        .map(|(index, (timestamp_ns, acceleration_x))| {
            let points = if index == 0 {
                points[..19].to_vec()
            } else {
                points.clone()
            };
            serde_json::json!({
                "cloud": {
                    "timestamp_ns": timestamp_ns,
                    "frame_id": "mid360_link",
                    "points": points,
                },
                "imu": [
                    {
                        "timestamp_ns": timestamp_ns - 100_000_000,
                        "accel_mps2": [acceleration_x, 0.0, 0.0],
                        "gyro_rps": [0.0, 0.0, 0.0],
                    },
                    {
                        "timestamp_ns": timestamp_ns - 50_000_000,
                        "accel_mps2": [acceleration_x, 0.0, 0.0],
                        "gyro_rps": [0.0, 0.0, 0.0],
                    }
                ]
            })
        })
        .collect::<Vec<_>>();
    let path = probe.path().join("native-pgo-recorded-scan-imu.json");
    fs::write(
        &path,
        serde_json::to_vec(&frames).expect("encode native PGO fixture"),
    )
    .expect("write native PGO fixture");
    path
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_cycle_starting_and_ready(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    cycle: &RunningNativeCycle,
) -> lio::LioStatus {
    let starting = wait_for_status(
        runtime,
        &mut observers.status,
        cycle.generation,
        lio::LIO_LIFECYCLE_STARTING,
        &cycle.scenario,
    );
    let ready = wait_for_status(
        runtime,
        &mut observers.status,
        cycle.generation,
        lio::LIO_LIFECYCLE_READY,
        &cycle.scenario,
    );
    assert_eq!(starting.backend, cycle.backend_code);
    assert_eq!(ready.backend, cycle.backend_code);
    assert_eq!(starting.status_sequence, 1);
    assert_eq!(ready.status_sequence, 2);
    ready
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn wait_cycle_ready(cycle: &mut RunningNativeCycle, probe: &ProbeDir) {
    if let Err(reason) = wait_for_startup_artifacts(
        cycle.child.child_mut(),
        probe,
        &cycle.ready_path,
        NATIVE_FIXTURE_TIMEOUT,
    ) {
        let output =
            std::mem::replace(&mut cycle.child, ChildGuard::empty()).terminate_and_collect();
        panic!(
            "{}: native profile did not become ready: {reason}; stdout={}, stderr={}",
            cycle.scenario,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn finish_native_cycle(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    mut client: GeometryClient,
    mut cycle: RunningNativeCycle,
) -> Output {
    stop_geometry(runtime, &mut client, &cycle.scenario);
    let stopping = wait_for_status(
        runtime,
        &mut observers.status,
        cycle.generation,
        lio::LIO_LIFECYCLE_STOPPING,
        &cycle.scenario,
    );
    let stopped = wait_for_status(
        runtime,
        &mut observers.status,
        cycle.generation,
        lio::LIO_LIFECYCLE_STOPPED,
        &cycle.scenario,
    );
    assert!(
        stopped.status_sequence > stopping.status_sequence,
        "{}: STOPPED did not follow STOPPING",
        cycle.scenario
    );
    assert_eq!(stopping.backend, cycle.backend_code);
    assert_eq!(stopped.backend, cycle.backend_code);
    assert_eq!(stopping.status_sequence, 3);
    assert_eq!(stopped.status_sequence, 4);
    if wait_until(cycle.child.child_mut(), NATIVE_FIXTURE_TIMEOUT).is_none() {
        let output = cycle.child.terminate_and_collect();
        panic!(
            "{}: Geometry did not exit after Stop; stdout={}, stderr={}",
            cycle.scenario,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    #[cfg(target_os = "linux")]
    let process_group_outcomes = cycle
        .child
        .settle_tracked_process_groups(NATIVE_FIXTURE_TIMEOUT)
        .unwrap_or_else(|error| panic!("{}: native PGO cleanup failed: {error}", cycle.scenario));
    #[cfg(target_os = "linux")]
    if process_group_outcomes.iter().any(|outcome| {
        outcome.disposition != ProcessGroupCleanupDisposition::ExitedWithoutHarnessSignal
    }) {
        let output = cycle.child.collect();
        panic!(
            "{}: normal Stop required harness native PGO cleanup: {process_group_outcomes:?}; stdout={}, stderr={}",
            cycle.scenario,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let output = cycle.child.collect();
    assert_eq!(
        output.status.code(),
        Some(0),
        "{}: Geometry exit was not clean; stdout={}, stderr={}",
        cycle.scenario,
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert_eq!(
        wait_for_file_contents(&cycle.term_path, NATIVE_FIXTURE_TIMEOUT)
            .unwrap_or_else(|error| panic!("{}: {error}", cycle.scenario)),
        "SIGTERM\n",
        "{}: native child did not observe SIGTERM",
        cycle.scenario
    );
    assert_native_child_reaped(cycle.pid, NATIVE_FIXTURE_TIMEOUT);
    cycle.native_cleanup.disarm();
    assert!(
        !cycle.ready_path.exists(),
        "{}: Geometry retained its ready file after Stop",
        cycle.scenario
    );
    output
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Debug)]
struct CloudMapShmSnapshot {
    revision: u64,
    point_count: u32,
    payload: Vec<u8>,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn read_cloud_map_shm(path: &Path) -> CloudMapShmSnapshot {
    const HEADER_SIZE: usize = 64;
    const SLOT_SIZE: usize = 16 * 1024 * 1024;
    let bytes = fs::read(path)
        .unwrap_or_else(|error| panic!("read cloud-map SHM {}: {error}", path.display()));
    assert!(bytes.len() >= HEADER_SIZE + 2 * SLOT_SIZE);
    assert_eq!(
        u64::from_le_bytes(bytes[0..8].try_into().expect("cloud-map magic")),
        0x4E55_5741_584D_4150
    );
    assert_eq!(
        u32::from_le_bytes(bytes[8..12].try_into().expect("cloud-map version")),
        2
    );
    let active_slot = u32::from_le_bytes(bytes[12..16].try_into().expect("cloud-map active slot"));
    assert!(active_slot <= 1, "cloud-map active slot must be 0 or 1");
    let point_count = u32::from_le_bytes(bytes[16..20].try_into().expect("cloud-map point count"));
    let revision = u64::from_le_bytes(bytes[28..36].try_into().expect("cloud-map revision"));
    let (slot_revision_offset, slot_count_offset) =
        if active_slot == 0 { (40, 48) } else { (52, 60) };
    assert_eq!(
        u64::from_le_bytes(
            bytes[slot_revision_offset..slot_revision_offset + 8]
                .try_into()
                .expect("cloud-map slot revision")
        ),
        revision
    );
    assert_eq!(
        u32::from_le_bytes(
            bytes[slot_count_offset..slot_count_offset + 4]
                .try_into()
                .expect("cloud-map slot count")
        ),
        point_count
    );
    let payload_len = usize::try_from(point_count)
        .expect("cloud-map point count fits usize")
        .checked_mul(12)
        .expect("cloud-map payload length");
    let start =
        HEADER_SIZE + usize::try_from(active_slot).expect("active slot fits usize") * SLOT_SIZE;
    let end = start + payload_len;
    CloudMapShmSnapshot {
        revision,
        point_count,
        payload: bytes[start..end].to_vec(),
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn read_file_prefix(path: &Path, limit: usize) -> Vec<u8> {
    fs::read(path)
        .unwrap_or_else(|error| panic!("read {}: {error}", path.display()))
        .into_iter()
        .take(limit)
        .collect()
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_final_estimator_odometry(
    backend: &str,
    generation: u64,
    envelope: &lio::LioOdometryEnvelope,
) {
    let (backend_code, expected_position, expected_yaw) = match backend {
        "fastlio2" => (
            lio::LIO_BACKEND_FASTLIO2,
            [0.529_996_504_920_526_9, -0.089_982_794_758_981_87, 0.0],
            0.340_013_032_025_788_36,
        ),
        "pointlio" => (
            lio::LIO_BACKEND_POINTLIO,
            [0.615_072_512_078_785_1, -0.020_533_448_815_630_753, 0.0],
            0.275_025_734_303_366_9,
        ),
        value => panic!("unsupported aggregate fixture backend {value}"),
    };
    assert_eq!(envelope.backend, backend_code);
    assert_eq!(envelope.runtime_generation, generation as i64);
    assert_eq!(envelope.source_sequence, 2);
    assert_eq!(envelope.source_timestamp_ns, 2_000_000_000);
    for (axis, (actual, expected)) in envelope
        .odometry
        .pose
        .position
        .into_iter()
        .zip(expected_position)
        .enumerate()
    {
        assert_near(
            actual,
            expected,
            1.0e-9,
            &format!("{backend} position[{axis}]"),
        );
    }
    let quaternion = envelope.odometry.pose.orientation_xyzw;
    let yaw = 2.0 * quaternion[2].atan2(quaternion[3]);
    assert_near(yaw, expected_yaw, 1.0e-9, &format!("{backend} yaw"));
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_recorded_contract_complete(contract: &serde_json::Value, backend: &str) {
    let (health_backend, health_status) = match backend {
        "fastlio2" => ("fast_lio2", "tracking_scan_iterated:3"),
        "pointlio" => ("point_lio", "tracking_point_incremental:3"),
        value => panic!("unsupported aggregate fixture backend {value}"),
    };
    assert_eq!(contract["published_pair_count"], 2);
    assert_eq!(contract["last_published_sequence"], 2);
    assert_eq!(contract["playback"]["input_frame_count"], 2);
    assert_eq!(contract["playback"]["processed_output_count"], 2);
    assert_eq!(
        contract["playback"]["processed_outputs"]
            .as_array()
            .map(Vec::len),
        Some(2)
    );
    let health = &contract["playback"]["final_output"]["health"];
    assert_eq!(health["healthy"], true);
    assert_eq!(health["backend"], health_backend);
    assert_eq!(health["processed_frames"], 2);
    assert_eq!(health["registered_points"], 4);
    assert_eq!(health["status"], health_status);
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_deferred_replay_outputs(
    scenario: &str,
    odometry: &lio::LioOdometryEnvelope,
    cloud: &lio::LioCloudEnvelope,
    tf_frame: &RawLcmFrame,
) {
    assert_eq!(cloud.runtime_generation, odometry.runtime_generation);
    assert_eq!(cloud.source_sequence, 1);
    assert_eq!(cloud.source_timestamp_ns, 1_000_000_000);
    assert_eq!(cloud.correlated_odometry_source_sequence, 1);
    assert_eq!(
        cloud.registration_state,
        lio::LIO_REGISTRATION_REGISTERED_MAP
    );
    assert_eq!(cloud.sensor_frame_id, "g14_lidar");
    assert_eq!(cloud.body_frame_id, "base_link");
    assert_eq!(cloud.odom_frame_id, "odom");
    assert_eq!(cloud.map_frame_id, "map");
    assert!(cloud.source_point_count > 0);
    assert_eq!(cloud.emitted_point_count, cloud.source_point_count);

    let tf = decode_tf(tf_frame);
    assert!(
        tf.transforms.iter().any(|transform| {
            transform.header.frame_id == "map" && transform.child_frame_id == "odom"
        }),
        "{scenario}: deferred replay canonical map->odom TF missing"
    );
    assert!(
        tf.transforms.iter().any(|transform| {
            transform.header.frame_id == "odom" && transform.child_frame_id == "base_link"
        }),
        "{scenario}: deferred replay canonical odom->base TF missing"
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_exactly_once_cloud_transform(
    backend: &str,
    odometry: &lio::LioOdometryEnvelope,
    cloud: &lio::LioCloudEnvelope,
    tf_frame: &RawLcmFrame,
) {
    assert_eq!(cloud.runtime_generation, odometry.runtime_generation);
    assert_eq!(cloud.source_sequence, 2);
    assert_eq!(cloud.source_timestamp_ns, 2_000_000_000);
    assert_eq!(cloud.correlated_odometry_source_sequence, 2);
    assert_eq!(
        cloud.registration_state,
        lio::LIO_REGISTRATION_REGISTERED_MAP
    );
    assert_eq!(cloud.sensor_frame_id, "g14_lidar");
    assert_eq!(cloud.body_frame_id, "base_link");
    assert_eq!(cloud.odom_frame_id, "odom");
    assert_eq!(cloud.map_frame_id, "map");
    assert_eq!(cloud.source_point_count, 4);
    assert_eq!(cloud.emitted_point_count, 4);
    let actual = xyz_points(&cloud.cloud)[0];

    let yaw = 2.0
        * odometry.odometry.pose.orientation_xyzw[2]
            .atan2(odometry.odometry.pose.orientation_xyzw[3]);
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    let body_once = [0.1732_f64, 0.1278, 1.1];
    let expected_once = [
        odometry.odometry.pose.position[0] + cos_yaw * body_once[0] - sin_yaw * body_once[1],
        odometry.odometry.pose.position[1] + sin_yaw * body_once[0] + cos_yaw * body_once[1],
        odometry.odometry.pose.position[2] + body_once[2],
    ];
    let body_twice = [0.1222_f64, -0.3268, 2.1];
    let expected_twice = [
        odometry.odometry.pose.position[0] + cos_yaw * body_twice[0] - sin_yaw * body_twice[1],
        odometry.odometry.pose.position[1] + sin_yaw * body_twice[0] + cos_yaw * body_twice[1],
        odometry.odometry.pose.position[2] + body_twice[2],
    ];
    let tf = decode_tf(tf_frame);
    let map_from_odom = tf
        .transforms
        .iter()
        .find(|transform| transform.header.frame_id == "map" && transform.child_frame_id == "odom")
        .expect("canonical map->odom transform");
    let map_quaternion = map_from_odom.transform.rotation_xyzw;
    let map_yaw = 2.0 * map_quaternion[2].atan2(map_quaternion[3]);
    let (map_sin, map_cos) = map_yaw.sin_cos();
    let apply_map_from_odom = |point: [f64; 3]| {
        [
            map_from_odom.transform.translation[0] + map_cos * point[0] - map_sin * point[1],
            map_from_odom.transform.translation[1] + map_sin * point[0] + map_cos * point[1],
            map_from_odom.transform.translation[2] + point[2],
        ]
    };
    let expected_once = apply_map_from_odom(expected_once);
    let expected_twice = apply_map_from_odom(expected_twice);
    for axis in 0..3 {
        assert_near(
            actual[axis],
            expected_once[axis],
            2.0e-5,
            &format!("{backend} exactly-once cloud[{axis}]"),
        );
    }
    assert!(
        actual
            .into_iter()
            .zip(expected_twice)
            .any(|(left, right)| (left - right).abs() > 0.5),
        "{backend}: cloud is indistinguishable from a double extrinsic transform"
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_mapping_outputs(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    probe: &ProbeDir,
    scenario: &str,
    started: &native_fixture_proto::MappingSessionState,
    tf_frame: &RawLcmFrame,
    global_map_frame: &RawLcmFrame,
) -> native_fixture_proto::MappingSessionState {
    let tf = decode_tf(tf_frame);
    assert!(
        tf.transforms.iter().any(|transform| {
            transform.header.frame_id == "map" && transform.child_frame_id == "odom"
        }),
        "{scenario}: canonical map->odom TF missing"
    );
    assert!(
        tf.transforms.iter().any(|transform| {
            transform.header.frame_id == "odom" && transform.child_frame_id == "base_link"
        }),
        "{scenario}: canonical odom->base TF missing"
    );

    let revision_deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    let mapping = loop {
        let mapping = get_mapping_state(runtime, client, scenario);
        if mapping.success
            && mapping.active
            && mapping.session_generation == started.session_generation
            && mapping.revision > started.revision
        {
            break mapping;
        }
        assert!(
            Instant::now() < revision_deadline,
            "{scenario}: MappingAdapter revision did not advance"
        );
        std::thread::sleep(Duration::from_millis(10));
    };
    assert!(mapping.success && mapping.active);
    assert_eq!(mapping.session_generation, started.session_generation);
    let occupancy = get_occupancy_data(runtime, client, scenario);
    let occupancy_meta = occupancy
        .meta
        .as_ref()
        .expect("committed occupancy metadata");
    assert_eq!(occupancy_meta.revision, mapping.revision);
    assert_eq!(occupancy_meta.frame_id, "map");
    assert!(occupancy_meta.width > 0 && occupancy_meta.height > 0);
    assert_eq!(
        occupancy.data.len(),
        usize::try_from(occupancy_meta.width).expect("occupancy width fits usize")
            * usize::try_from(occupancy_meta.height).expect("occupancy height fits usize")
    );
    let occupancy_shm = get_occupancy_shm(
        runtime,
        client,
        scenario,
        Instant::now() + NATIVE_FIXTURE_TIMEOUT,
    );
    assert_eq!(
        Path::new(&occupancy_shm.key),
        probe.path().join("occupancy.mmap")
    );
    assert_eq!(occupancy_shm.revision, occupancy_meta.revision);
    assert!(occupancy_shm.generation > 0);
    assert_eq!(occupancy_shm.payload_size, occupancy.data.len() as u64);
    assert_eq!(occupancy_shm.layout_version, 2);
    assert_eq!(occupancy_shm.payload_schema_version, 1);

    let cloud_shm = read_cloud_map_shm(&probe.path().join("cloud-map.mmap"));
    assert!(cloud_shm.revision > 0);
    assert!(cloud_shm.point_count > 0);
    let global_map = decode_global_map(global_map_frame);
    assert_eq!(global_map.header.frame_id, "map");
    assert_eq!(global_map.point_step, 12);
    assert_eq!(global_map.height, 1);
    assert_eq!(global_map.width, cloud_shm.point_count as i32);
    assert_eq!(global_map.data, cloud_shm.payload);
    assert_eq!(
        u64::try_from(global_map.header.seq).expect("global-map sequence is positive"),
        cloud_shm.revision
    );
    mapping
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[test]
fn native_owner_profiles_complete_two_cycle_recorded_mapping_and_generation_fence() {
    let runtime = tokio::runtime::Runtime::new().expect("create aggregate fixture runtime");
    let probe = ProbeDir::new("g014-native-aggregate");
    let fixture_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../nuwax-workspace/crates/nuwax-lio-backends/fixtures/recorded_scan_imu.json")
        .canonicalize()
        .expect("canonicalize recorded LIO fixture");
    let permit_path = probe.path().join("recorded-publish-permit");
    write_publish_permit(&permit_path, 0);
    let mut observers = NativeObservers::new(&runtime, probe.lcm_url());

    let mut first = spawn_native_cycle(
        &runtime,
        &observers,
        &probe,
        ("fastlio2", "voxel", "none"),
        &fixture_path,
        &permit_path,
    );
    write_publish_permit(&permit_path, 1);
    let old_private_odometry = next_frame(
        &runtime,
        &mut first.private_odometry,
        "fastlio2 private odometry",
    );
    let old_private_cloud =
        next_frame(&runtime, &mut first.private_cloud, "fastlio2 private cloud");
    wait_cycle_ready(&mut first, &probe);
    let first_ready = assert_cycle_starting_and_ready(&runtime, &mut observers, &first);
    assert_eq!(first_ready.backend, lio::LIO_BACKEND_FASTLIO2);
    let inactive_odometry_frame = next_frame(
        &runtime,
        &mut observers.public_odometry,
        "fastlio2 inactive public odometry",
    );
    let inactive_odometry = decode_public_odometry(&inactive_odometry_frame);
    assert_eq!(
        inactive_odometry.runtime_generation,
        first.generation as i64
    );
    assert_eq!(inactive_odometry.source_sequence, 1);
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "mapping-inactive public cloud",
    );
    assert_stream_silent(&runtime, &mut observers.tf, "mapping-inactive TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "mapping-inactive global map",
    );

    let mut first_client = connect_geometry(&runtime, &probe, &first.scenario);
    let first_mapping = start_mapping(&runtime, &mut first_client, &first.scenario);
    assert!(first_mapping.success && first_mapping.active);
    assert!(first_mapping.session_generation > 0);
    assert_eq!(first_mapping.revision, 1);
    let first_replay_tf = next_frame(&runtime, &mut observers.tf, "fastlio2 deferred replay TF");
    let first_replay_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "fastlio2 deferred replay cloud",
    ));
    assert_deferred_replay_outputs(
        &first.scenario,
        &inactive_odometry,
        &first_replay_cloud,
        &first_replay_tf,
    );
    let _first_replay_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "fastlio2 deferred replay global map",
    );
    assert_stream_silent(
        &runtime,
        &mut observers.public_odometry,
        "fastlio2 deferred replay duplicate odometry",
    );
    write_publish_permit(&permit_path, 2);
    let first_odometry_frame = next_frame(
        &runtime,
        &mut observers.public_odometry,
        "fastlio2 final public odometry",
    );
    let first_odometry = decode_public_odometry(&first_odometry_frame);
    assert_eq!(
        first_odometry.source_sequence,
        2,
        concat!(
            "{}: next live odometry reused the retained cursor; ",
            "startup_timestamp_ns={}, next_timestamp_ns={}, same_payload={}, ",
            "startup_payload_hash={:016x}, next_payload_hash={:016x}"
        ),
        first.scenario,
        inactive_odometry.source_timestamp_ns,
        first_odometry.source_timestamp_ns,
        inactive_odometry_frame.payload == first_odometry_frame.payload,
        nuwax_codec::hash::murmur_hash64a(&inactive_odometry_frame.payload),
        nuwax_codec::hash::murmur_hash64a(&first_odometry_frame.payload),
    );
    assert_final_estimator_odometry("fastlio2", first.generation, &first_odometry);
    let first_tf_frame = next_frame(&runtime, &mut observers.tf, "fastlio2 canonical TF");
    let first_cloud_frame = next_frame(
        &runtime,
        &mut observers.public_cloud,
        "fastlio2 registered public cloud",
    );
    let first_cloud = decode_public_cloud(&first_cloud_frame);
    assert_exactly_once_cloud_transform("fastlio2", &first_odometry, &first_cloud, &first_tf_frame);
    let first_global_map_frame =
        next_nonempty_global_map(&runtime, &mut observers.global_map, "fastlio2 global map");
    let first_committed = assert_mapping_outputs(
        &runtime,
        &mut first_client,
        &probe,
        &first.scenario,
        &first_mapping,
        &first_tf_frame,
        &first_global_map_frame,
    );
    let first_contract = wait_for_json(&first.contract_path, NATIVE_FIXTURE_TIMEOUT, |value| {
        value["published_pair_count"].as_u64() == Some(2)
    })
    .unwrap_or_else(|error| panic!("{}: {error}", first.scenario));
    assert_recorded_contract_complete(&first_contract, "fastlio2");
    let first_generation = first.generation;
    let first_pid = first.pid;
    finish_native_cycle(&runtime, &mut observers, first_client, first);

    write_publish_permit(&permit_path, 0);
    let mut second = spawn_native_cycle(
        &runtime,
        &observers,
        &probe,
        ("pointlio", "voxel", "none"),
        &fixture_path,
        &permit_path,
    );
    assert_eq!(
        second.generation,
        first_generation + 1,
        "restart did not advance the persisted runtime generation exactly once"
    );
    assert_ne!(
        second.pid, first_pid,
        "alternate backend reused the first native child process"
    );
    std::thread::sleep(OUTPUT_SILENCE_WINDOW);
    assert!(
        !second.ready_path.exists(),
        "{}: permit=0 unexpectedly reached READY",
        second.scenario
    );
    assert_stream_silent(
        &runtime,
        &mut observers.public_odometry,
        "pointlio permit=0 public odometry",
    );
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "pointlio permit=0 public cloud",
    );

    runtime
        .block_on(observers.transport.publish(old_private_odometry))
        .expect("replay old private odometry");
    runtime
        .block_on(observers.transport.publish(old_private_cloud))
        .expect("replay old private cloud");
    assert_stream_silent(
        &runtime,
        &mut observers.public_odometry,
        "old private generation public odometry",
    );
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "old private generation public cloud",
    );
    assert!(
        !second.ready_path.exists(),
        "{}: old private generation incorrectly satisfied READY",
        second.scenario
    );

    write_publish_permit(&permit_path, 1);
    let _second_private_odometry = next_frame(
        &runtime,
        &mut second.private_odometry,
        "pointlio private odometry",
    );
    let _second_private_cloud = next_frame(
        &runtime,
        &mut second.private_cloud,
        "pointlio private cloud",
    );
    wait_cycle_ready(&mut second, &probe);
    let second_ready = assert_cycle_starting_and_ready(&runtime, &mut observers, &second);
    assert_eq!(second_ready.backend, lio::LIO_BACKEND_POINTLIO);
    let second_inactive_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "pointlio inactive public odometry",
    ));
    assert_eq!(
        second_inactive_odometry.runtime_generation,
        second.generation as i64
    );
    assert_eq!(second_inactive_odometry.source_sequence, 1);
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "pointlio inactive public cloud",
    );

    drain_matching_global_map(
        &runtime,
        &mut observers.global_map,
        &first_global_map_frame,
        "pointlio restarted observer baseline changed content",
    );
    let mut second_client = connect_geometry(&runtime, &probe, &second.scenario);
    let second_mapping = start_mapping(&runtime, &mut second_client, &second.scenario);
    assert!(second_mapping.success && second_mapping.active);
    assert!(
        second_mapping.session_generation > 0,
        "restart did not allocate a process-local MappingAdapter generation"
    );
    let second_replay_tf = next_frame(&runtime, &mut observers.tf, "pointlio deferred replay TF");
    let second_replay_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "pointlio deferred replay cloud",
    ));
    assert_deferred_replay_outputs(
        &second.scenario,
        &second_inactive_odometry,
        &second_replay_cloud,
        &second_replay_tf,
    );
    let _second_replay_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "pointlio deferred replay global map",
    );
    let mapping_before_stale = get_mapping_state(&runtime, &mut second_client, &second.scenario);
    let occupancy_before_stale = read_file_prefix(&probe.path().join("occupancy.mmap"), 4096);
    let cloud_before_stale = read_file_prefix(&probe.path().join("cloud-map.mmap"), 4096);

    runtime
        .block_on(observers.transport.publish(first_odometry_frame.clone()))
        .expect("replay old public odometry");
    runtime
        .block_on(observers.transport.publish(first_cloud_frame.clone()))
        .expect("replay old public cloud");
    let echoed_old_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "echoed old public odometry",
    ));
    let echoed_old_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "echoed old public cloud",
    ));
    assert_eq!(
        echoed_old_odometry.runtime_generation,
        first_generation as i64
    );
    assert_eq!(echoed_old_cloud.runtime_generation, first_generation as i64);
    assert_stream_silent(&runtime, &mut observers.tf, "stale-generation TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "stale-generation global map",
    );
    let mapping_after_stale = get_mapping_state(&runtime, &mut second_client, &second.scenario);
    assert_eq!(mapping_after_stale, mapping_before_stale);
    assert_eq!(
        read_file_prefix(&probe.path().join("occupancy.mmap"), 4096),
        occupancy_before_stale,
        "stale public generation mutated occupancy SHM"
    );
    assert_eq!(
        read_file_prefix(&probe.path().join("cloud-map.mmap"), 4096),
        cloud_before_stale,
        "stale public generation mutated cloud-map SHM"
    );

    write_publish_permit(&permit_path, 2);
    let second_odometry_frame = next_frame(
        &runtime,
        &mut observers.public_odometry,
        "pointlio final public odometry",
    );
    let second_odometry = decode_public_odometry(&second_odometry_frame);
    assert_final_estimator_odometry("pointlio", second.generation, &second_odometry);
    let second_tf_frame = next_frame(&runtime, &mut observers.tf, "pointlio canonical TF");
    let second_cloud_frame = next_frame(
        &runtime,
        &mut observers.public_cloud,
        "pointlio registered public cloud",
    );
    let second_cloud = decode_public_cloud(&second_cloud_frame);
    assert_exactly_once_cloud_transform(
        "pointlio",
        &second_odometry,
        &second_cloud,
        &second_tf_frame,
    );
    let second_global_map_frame =
        next_nonempty_global_map(&runtime, &mut observers.global_map, "pointlio global map");
    let second_committed = assert_mapping_outputs(
        &runtime,
        &mut second_client,
        &probe,
        &second.scenario,
        &second_mapping,
        &second_tf_frame,
        &second_global_map_frame,
    );
    assert!(
        second_committed.revision > mapping_after_stale.revision,
        "current generation did not advance revision after stale replay"
    );
    assert!(
        second_committed.revision > first_committed.revision,
        "restart did not preserve monotonic MappingAdapter revision"
    );
    let second_contract = wait_for_json(&second.contract_path, NATIVE_FIXTURE_TIMEOUT, |value| {
        value["published_pair_count"].as_u64() == Some(2)
    })
    .unwrap_or_else(|error| panic!("{}: {error}", second.scenario));
    assert_recorded_contract_complete(&second_contract, "pointlio");
    finish_native_cycle(&runtime, &mut observers, second_client, second);
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[test]
fn library_pgo_production_binary_real_lcm_v2_closes_hostile_and_lifecycle_fences() {
    let runtime = tokio::runtime::Runtime::new().expect("create library PGO V2 runtime");
    let probe = ProbeDir::new("g004-library-pgo-v2");
    let fixture_path = write_library_pgo_recorded_fixture(&probe);
    let restart_fixture_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../nuwax-workspace/crates/nuwax-lio-backends/fixtures/recorded_scan_imu.json")
        .canonicalize()
        .expect("canonicalize restart LIO fixture");
    let permit_path = probe.path().join("library-pgo-publish-permit");
    write_publish_permit(&permit_path, 0);
    let mut observers = NativeObservers::new(&runtime, probe.lcm_url());

    let mut first = spawn_native_cycle(
        &runtime,
        &observers,
        &probe,
        ("fastlio2", "voxel", "library_pgo"),
        &fixture_path,
        &permit_path,
    );
    let candidate_transport =
        LcmTransport::new_real(probe.lcm_url()).expect("independent correction observer context");
    let candidate_binding = typed_binding(
        &format!("/c/l/{}", first.generation),
        "nuwax_msgs.CorrectionCandidate",
    );
    let mut first_candidates = runtime.block_on(candidate_transport.subscribe(
        candidate_binding.0.clone(),
        Some(candidate_binding.1.clone()),
    ));

    write_publish_permit(&permit_path, 1);
    let _first_private_odometry = next_frame(
        &runtime,
        &mut first.private_odometry,
        "library PGO first private odometry",
    );
    let _first_private_cloud = next_frame(
        &runtime,
        &mut first.private_cloud,
        "library PGO first private cloud",
    );
    wait_cycle_ready(&mut first, &probe);
    assert_cycle_starting_and_ready(&runtime, &mut observers, &first);
    let inactive_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "library PGO inactive odometry",
    ));
    assert_eq!(inactive_odometry.source_sequence, 1);
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "library PGO inactive cloud",
    );

    let mut first_client = connect_geometry(&runtime, &probe, &first.scenario);
    let first_started = start_mapping(&runtime, &mut first_client, &first.scenario);
    assert_eq!(first_started.revision, 1);
    let first_replay_tf = next_frame(
        &runtime,
        &mut observers.tf,
        "library PGO deferred replay TF",
    );
    let first_replay_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "library PGO deferred replay cloud",
    ));
    assert_deferred_replay_outputs(
        &first.scenario,
        &inactive_odometry,
        &first_replay_cloud,
        &first_replay_tf,
    );
    let _first_replay_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "library PGO deferred replay global map",
    );
    assert_stream_silent(
        &runtime,
        &mut first_candidates,
        "library PGO bootstrap replay candidate",
    );
    let mut candidate_source_snapshot = None;
    for permit in 2..=10 {
        std::thread::sleep(Duration::from_millis(150));
        write_publish_permit(&permit_path, permit);
        let label = |stream: &str| format!("library PGO {stream} sequence {permit}");
        let private_odometry = next_frame(
            &runtime,
            &mut first.private_odometry,
            &label("private odometry"),
        );
        let private_cloud = next_frame(&runtime, &mut first.private_cloud, &label("private cloud"));
        let odometry = next_frame(
            &runtime,
            &mut observers.public_odometry,
            &label("public odometry"),
        );
        let tf = next_frame(&runtime, &mut observers.tf, &label("odometry TF"));
        let cloud = next_frame(
            &runtime,
            &mut observers.public_cloud,
            &label("registered cloud"),
        );
        let global_map =
            next_nonempty_global_map(&runtime, &mut observers.global_map, &label("global map"));
        let candidate_timeout = if permit == 10 {
            NATIVE_FIXTURE_TIMEOUT
        } else {
            OUTPUT_SILENCE_WINDOW
        };
        if let Some(candidate) = next_frame_within(
            &runtime,
            &mut first_candidates,
            candidate_timeout,
            &label("correction candidate"),
        ) {
            let mapping = get_mapping_state(&runtime, &mut first_client, &first.scenario);
            let occupancy = get_occupancy_data(&runtime, &mut first_client, &first.scenario);
            let occupancy_file = fs::read(probe.path().join("occupancy.mmap"))
                .expect("read occupancy at candidate source");
            let cloud_file = fs::read(probe.path().join("cloud-map.mmap"))
                .expect("read cloud map at candidate source");
            let cloud_shm = read_cloud_map_shm(&probe.path().join("cloud-map.mmap"));
            candidate_source_snapshot = Some((
                candidate,
                private_odometry,
                private_cloud,
                odometry,
                cloud,
                tf,
                global_map,
                mapping,
                occupancy,
                occupancy_file,
                cloud_file,
                cloud_shm,
            ));
            break;
        }
    }

    let (
        first_candidate_frame,
        final_private_odometry_frame,
        final_private_cloud_frame,
        final_odometry_frame,
        final_cloud_frame,
        final_uncorrected_tf,
        final_global_map,
        mapping_before_correction,
        occupancy_before_correction,
        occupancy_file_before_correction,
        cloud_file_before_correction,
        cloud_shm_before_correction,
    ) = candidate_source_snapshot.expect("library PGO candidate did not arrive by source 10");
    let final_private_odometry = decode_private_odometry(&final_private_odometry_frame);
    let final_private_cloud = decode_private_source_cloud(&final_private_cloud_frame);
    let final_odometry = decode_public_odometry(&final_odometry_frame);
    let final_cloud = decode_public_cloud(&final_cloud_frame);
    let final_global_map_message = decode_global_map(&final_global_map);
    assert_eq!(first_candidate_frame.topic, candidate_binding.0);
    assert_eq!(first_candidate_frame.msg_type, candidate_binding.1);
    let first_candidate = decode_correction_candidate(&first_candidate_frame);
    assert_eq!(first_candidate.backend, 1);
    assert_eq!(first_candidate.transform_semantics, 1);
    assert_eq!(first_candidate.runtime_generation, first.generation);
    assert_eq!(first_candidate.backend_generation, first.generation);
    assert_eq!(first_candidate.candidate_sequence, 1);
    assert_eq!(first_candidate.mapping_session_id, first_started.session_id);
    assert_eq!(
        first_candidate.mapping_session_generation,
        first_started.session_generation
    );
    assert!(first_candidate.map_frame_generation > 0);
    assert!(first_candidate
        .mapping_commit_boot_nonce
        .iter()
        .any(|byte| *byte != 0));
    assert!(first_candidate.mapping_commit_sequence > 0);
    assert_eq!(
        first_candidate.occupancy_revision,
        mapping_before_correction.revision
    );
    assert_eq!(
        first_candidate.cloud_revision,
        u64::try_from(final_global_map_message.header.seq)
            .expect("global-map revision is nonnegative")
    );
    assert_eq!(
        first_candidate.committed_map_source_timestamp_ns,
        u64::try_from(final_cloud.source_timestamp_ns).expect("cloud timestamp is nonnegative")
    );
    assert_eq!(
        first_candidate.registered_cloud_source_sequence,
        u64::try_from(final_cloud.source_sequence).expect("cloud sequence is positive")
    );
    assert_eq!(
        first_candidate.registered_cloud_source_timestamp_ns,
        u64::try_from(final_cloud.source_timestamp_ns).expect("cloud timestamp is nonnegative")
    );
    assert_eq!(
        first_candidate.registered_cloud_point_count,
        u64::try_from(final_cloud.emitted_point_count).expect("cloud point count is positive")
    );
    assert_eq!(
        final_cloud.source_point_count, final_cloud.emitted_point_count,
        "body digest fixture unexpectedly exercised native downsampling"
    );
    assert_eq!(
        final_private_odometry, final_odometry.odometry,
        "candidate source private odometry diverged from the frozen public observation"
    );
    assert_eq!(
        u64::try_from(final_private_odometry.header.seq)
            .expect("private odometry sequence is positive"),
        first_candidate.correlated_odom_source_sequence
    );
    assert_eq!(
        u64::try_from(final_private_cloud.header.seq).expect("private cloud sequence is positive"),
        first_candidate.registered_cloud_source_sequence
    );
    let private_cloud_timestamp_ns = i64::from(final_private_cloud.header.stamp_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(final_private_cloud.header.stamp_nsec)))
        .and_then(|timestamp| u64::try_from(timestamp).ok())
        .expect("private cloud timestamp is nonnegative");
    assert_eq!(
        private_cloud_timestamp_ns,
        first_candidate.registered_cloud_source_timestamp_ns
    );
    assert_eq!(
        first_candidate.registered_cloud_envelope_sha256,
        nuwax_codec::sha256_bytes(&with_fingerprint(
            lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1,
            &final_cloud_frame.payload,
        )),
        "candidate did not bind the complete registered-cloud envelope"
    );
    let independent_body_points = independent_candidate_body_points(
        &inactive_odometry,
        &final_private_odometry,
        &final_private_cloud,
    );
    assert_eq!(
        u64::try_from(independent_body_points.len()).expect("body-point count fits u64"),
        first_candidate.registered_cloud_point_count,
        "independent body-point reconstruction changed point count/order"
    );
    let independent_body_digest = independent_body_points_sha256(&independent_body_points);
    assert_eq!(
        first_candidate.body_points_sha256,
        independent_body_digest,
        "candidate body-point digest is not bound to the independently decoded source cloud: candidate={} independent={} source_sequence={} source_timestamp_ns={} point_count={} first_body_points={:?}",
        digest_hex(&first_candidate.body_points_sha256),
        digest_hex(&independent_body_digest),
        first_candidate.registered_cloud_source_sequence,
        first_candidate.registered_cloud_source_timestamp_ns,
        independent_body_points.len(),
        independent_body_points.iter().take(3).collect::<Vec<_>>(),
    );
    assert_eq!(
        first_candidate.correlated_odom_source_sequence,
        u64::try_from(final_odometry.source_sequence).expect("odometry sequence is positive")
    );
    assert_eq!(
        first_candidate.correlated_odom_source_timestamp_ns,
        u64::try_from(final_odometry.source_timestamp_ns)
            .expect("odometry timestamp is nonnegative")
    );
    assert_eq!(
        first_candidate.correlated_odom_pose_sha256,
        correlated_odom_pose_sha256(&final_odometry),
        "candidate did not bind the frozen correlated odometry pose"
    );
    let occupancy_meta = occupancy_before_correction
        .meta
        .as_ref()
        .expect("candidate occupancy metadata");
    assert_eq!(
        first_candidate.committed_map_width,
        u64::from(occupancy_meta.width)
    );
    assert_eq!(
        first_candidate.committed_map_height,
        u64::from(occupancy_meta.height)
    );
    assert_near(
        first_candidate.committed_map_resolution_m,
        f64::from(occupancy_meta.resolution),
        1.0e-7,
        "candidate committed-map resolution",
    );
    assert_near(
        first_candidate.committed_map_origin_x_m,
        occupancy_meta.origin_x,
        1.0e-12,
        "candidate committed-map origin x",
    );
    assert_near(
        first_candidate.committed_map_origin_y_m,
        occupancy_meta.origin_y,
        1.0e-12,
        "candidate committed-map origin y",
    );
    assert_eq!(
        first_candidate.committed_map_payload_sha256,
        nuwax_codec::sha256_bytes(&occupancy_before_correction.data),
        "candidate did not bind the committed occupancy payload"
    );
    assert!(first_candidate
        .committed_map_boundary_sha256
        .iter()
        .any(|byte| *byte != 0));
    assert_eq!(
        first_candidate.source_timestamp_ns,
        u64::try_from(final_cloud.source_timestamp_ns).expect("cloud timestamp is nonnegative")
    );
    assert_eq!(first_candidate.map_frame_id, "map");
    assert_eq!(first_candidate.odom_frame_id, "odom");
    assert_eq!(first_candidate.body_frame_id, "base_link");
    assert_eq!(first_candidate.points_frame_id, "base_link");
    assert!(first_candidate
        .correction_lease_boot_nonce
        .iter()
        .any(|byte| *byte != 0));
    assert_eq!(
        first_candidate.correction_lease_generation,
        first.generation
    );

    let before_correction = map_from_odom_in_frame(&final_uncorrected_tf);
    assert_eq!(
        schema_transform_timestamp_ns(&before_correction),
        first_candidate.source_timestamp_ns
    );
    let base_map_from_odom = transform_isometry(
        before_correction.transform.translation,
        before_correction.transform.rotation_xyzw,
    );
    let absolute_left_correction =
        transform_isometry(first_candidate.translation_m, first_candidate.rotation_xyzw);
    let expected_map_from_odom = base_map_from_odom * absolute_left_correction;
    let correction_tf = wait_for_exact_map_from_odom(
        &runtime,
        &mut observers.tf,
        &expected_map_from_odom,
        first_candidate.source_timestamp_ns,
        1,
        "library PGO canonical correction TF",
    );
    assert_frame_tree_map_from_odom(
        &get_frame_tree(&runtime, &mut first_client, &first.scenario),
        &expected_map_from_odom,
        first_candidate.source_timestamp_ns,
        "library PGO corrected frame tree",
    );
    assert_eq!(
        get_mapping_state(&runtime, &mut first_client, &first.scenario),
        mapping_before_correction,
        "correction changed the mapping revision/state"
    );
    assert_eq!(
        get_occupancy_data(&runtime, &mut first_client, &first.scenario),
        occupancy_before_correction,
        "correction changed canonical occupancy"
    );
    assert_eq!(
        fs::read(probe.path().join("occupancy.mmap")).expect("read occupancy after correction"),
        occupancy_file_before_correction,
        "correction changed occupancy SHM"
    );
    assert_eq!(
        fs::read(probe.path().join("cloud-map.mmap")).expect("read cloud map after correction"),
        cloud_file_before_correction,
        "correction changed cloud-map SHM"
    );
    let cloud_shm_after_correction = read_cloud_map_shm(&probe.path().join("cloud-map.mmap"));
    assert_eq!(
        cloud_shm_after_correction.revision,
        cloud_shm_before_correction.revision
    );
    assert_eq!(
        cloud_shm_after_correction.point_count,
        cloud_shm_before_correction.point_count
    );
    assert_eq!(
        cloud_shm_after_correction.payload,
        cloud_shm_before_correction.payload
    );
    drain_matching_global_map(
        &runtime,
        &mut observers.global_map,
        &final_global_map,
        "library PGO correction changed global-map content",
    );
    let first_committed = assert_mapping_outputs(
        &runtime,
        &mut first_client,
        &probe,
        &first.scenario,
        &first_started,
        &correction_tf,
        &final_global_map,
    );

    let mapping_before_hostile = get_mapping_state(&runtime, &mut first_client, &first.scenario);
    let occupancy_before_hostile = read_file_prefix(&probe.path().join("occupancy.mmap"), 4096);
    let cloud_before_hostile = read_file_prefix(&probe.path().join("cloud-map.mmap"), 4096);
    runtime
        .block_on(observers.transport.publish(first_candidate_frame.clone()))
        .expect("publish duplicate correction candidate");
    let duplicate_echo = next_frame(
        &runtime,
        &mut first_candidates,
        "duplicate correction candidate echo",
    );
    assert_eq!(duplicate_echo.payload, first_candidate_frame.payload);
    assert_stream_silent(&runtime, &mut observers.tf, "duplicate candidate TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "duplicate candidate global map",
    );

    let mut malformed_candidate = first_candidate_frame.clone();
    *malformed_candidate
        .payload
        .last_mut()
        .expect("candidate payload is non-empty") ^= 0x01;
    runtime
        .block_on(observers.transport.publish(malformed_candidate))
        .expect("publish malformed correction candidate");
    let _malformed_echo = next_frame(
        &runtime,
        &mut first_candidates,
        "malformed correction candidate echo",
    );
    assert_stream_silent(&runtime, &mut observers.tf, "malformed candidate TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "malformed candidate global map",
    );

    let hostile_occupancy = schema::OccupancyGrid {
        header: schema::Header {
            seq: i32::MAX,
            stamp_sec: 1,
            frame_id: "map".into(),
            ..schema::Header::default()
        },
        info: schema::MapMetaData {
            resolution: 1.0,
            width: 1,
            height: 1,
            ..schema::MapMetaData::default()
        },
        data: vec![100],
    };
    runtime
        .block_on(
            observers.transport.publish(
                nuwax_protocol::RawLcmFrameBuilder::new(
                    LcmTopic::new("/map/occupancy").unwrap(),
                    LcmMessageType::new("nav_msgs.OccupancyGrid").unwrap(),
                )
                .seqno(u32::MAX)
                .payload(schema::encode_occupancy_grid_body(&hostile_occupancy).unwrap())
                .build(),
            ),
        )
        .expect("publish external canonical occupancy injection");
    let hostile_tf = schema::TfMessage {
        transforms: vec![schema::TransformStamped {
            header: schema::Header {
                seq: i32::MAX,
                stamp_sec: 1,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            child_frame_id: "odom".into(),
            transform: schema::Transform {
                translation: [99.0, 98.0, 97.0],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        }],
    };
    runtime
        .block_on(
            observers.transport.publish(
                nuwax_protocol::RawLcmFrameBuilder::new(
                    LcmTopic::new("/tf").unwrap(),
                    LcmMessageType::new("tf2_msgs.TFMessage").unwrap(),
                )
                .seqno(u32::MAX)
                .payload(schema::encode_tf_message_body(&hostile_tf).unwrap())
                .build(),
            ),
        )
        .expect("publish external canonical TF injection");
    let injected_tf = next_frame(&runtime, &mut observers.tf, "external TF injection echo");
    assert_eq!(
        map_from_odom_in_frame(&injected_tf).transform.translation,
        [99.0, 98.0, 97.0]
    );
    std::thread::sleep(OUTPUT_SILENCE_WINDOW);
    assert_eq!(
        get_mapping_state(&runtime, &mut first_client, &first.scenario),
        mapping_before_hostile
    );
    assert_eq!(
        read_file_prefix(&probe.path().join("occupancy.mmap"), 4096),
        occupancy_before_hostile,
        "external occupancy writer mutated Geometry-owned SHM"
    );
    assert_eq!(
        read_file_prefix(&probe.path().join("cloud-map.mmap"), 4096),
        cloud_before_hostile,
        "external TF writer mutated Geometry-owned cloud-map SHM"
    );

    write_publish_permit(&permit_path, 11);
    let post_hostile_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "post-hostile current odometry",
    ));
    let post_hostile_tf = next_frame(&runtime, &mut observers.tf, "post-hostile canonical TF");
    let post_hostile_map_odom = map_from_odom_in_frame(&post_hostile_tf);
    assert_eq!(
        post_hostile_map_odom.header.seq, 1,
        "next live odometry did not reuse correction revision 1"
    );
    assert_eq!(
        schema_transform_timestamp_ns(&post_hostile_map_odom),
        u64::try_from(post_hostile_odometry.source_timestamp_ns)
            .expect("live odometry timestamp is nonnegative")
    );
    assert_transform_matches_isometry(
        post_hostile_map_odom.transform.translation,
        post_hostile_map_odom.transform.rotation_xyzw,
        &expected_map_from_odom,
        "next live odometry correction snapshot",
    );
    let _post_hostile_cloud = next_frame(
        &runtime,
        &mut observers.public_cloud,
        "post-hostile registered cloud",
    );
    let post_hostile_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "post-hostile global map",
    );
    let post_hostile_mapping = get_mapping_state(&runtime, &mut first_client, &first.scenario);
    assert!(post_hostile_mapping.revision > first_committed.revision);

    let first_generation = first.generation;
    let first_odometry_frame = final_odometry_frame;
    let first_cloud_frame = final_cloud_frame;
    let first_output = finish_native_cycle(&runtime, &mut observers, first_client, first);
    let finalization_events = candidate_finalization_events(&first_output);
    assert_eq!(
        finalization_events.len(),
        1,
        "{CORRECTION_FINALIZATION_TARGET}: expected exactly one finalizer event, got {finalization_events:?}"
    );
    let finalized = &finalization_events[0];
    assert_eq!(finalized.event, CORRECTION_FINALIZATION_EVENT);
    assert_eq!(
        finalized.runtime_generation,
        first_candidate.runtime_generation
    );
    assert_eq!(
        finalized.backend_generation,
        first_candidate.backend_generation
    );
    assert_eq!(
        finalized.session_generation,
        first_candidate.mapping_session_generation
    );
    assert_eq!(
        finalized.candidate_sequence,
        first_candidate.candidate_sequence
    );
    assert_eq!(
        finalized.payload_sha256,
        digest_hex(&first_candidate.payload_sha256)
    );
    assert_eq!(finalized.disposition, ObservedCandidateDisposition::Applied);
    assert_eq!(finalized.reject_reason, None);
    assert!(finalized.candidate_processed_delivered);
    runtime
        .block_on(observers.transport.publish(first_candidate_frame.clone()))
        .expect("publish candidate after correction owner shutdown");
    let _late_candidate_echo = next_frame(
        &runtime,
        &mut first_candidates,
        "late candidate echo after shutdown",
    );
    assert_stream_silent(&runtime, &mut observers.tf, "post-stop correction TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "post-stop correction global map",
    );

    write_publish_permit(&permit_path, 0);
    let mut second = spawn_native_cycle(
        &runtime,
        &observers,
        &probe,
        ("pointlio", "voxel", "library_pgo"),
        &restart_fixture_path,
        &permit_path,
    );
    assert_eq!(second.generation, first_generation + 1);
    write_publish_permit(&permit_path, 1);
    let _second_private_odometry = next_frame(
        &runtime,
        &mut second.private_odometry,
        "restarted library PGO private odometry",
    );
    let _second_private_cloud = next_frame(
        &runtime,
        &mut second.private_cloud,
        "restarted library PGO private cloud",
    );
    wait_cycle_ready(&mut second, &probe);
    assert_cycle_starting_and_ready(&runtime, &mut observers, &second);
    let second_inactive_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "restarted library PGO inactive odometry",
    ));
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "restarted library PGO inactive cloud",
    );
    drain_matching_global_map(
        &runtime,
        &mut observers.global_map,
        &post_hostile_global_map,
        "restarted library PGO baseline changed content",
    );
    let mut second_client = connect_geometry(&runtime, &probe, &second.scenario);
    let second_started = start_mapping(&runtime, &mut second_client, &second.scenario);
    let second_replay_tf = next_frame(
        &runtime,
        &mut observers.tf,
        "restarted library PGO deferred replay TF",
    );
    let second_replay_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "restarted library PGO deferred replay cloud",
    ));
    assert_deferred_replay_outputs(
        &second.scenario,
        &second_inactive_odometry,
        &second_replay_cloud,
        &second_replay_tf,
    );
    let _second_replay_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "restarted library PGO deferred replay global map",
    );
    let mapping_before_old_generation =
        get_mapping_state(&runtime, &mut second_client, &second.scenario);
    let new_candidate_binding = typed_binding(
        &format!("/c/l/{}", second.generation),
        "nuwax_msgs.CorrectionCandidate",
    );
    runtime
        .block_on(
            observers.transport.publish(
                nuwax_protocol::RawLcmFrameBuilder::new(
                    new_candidate_binding.0,
                    new_candidate_binding.1,
                )
                .seqno(1)
                .payload(first_candidate_frame.payload.clone())
                .build(),
            ),
        )
        .expect("publish old-generation candidate on new private channel");
    assert_stream_silent(&runtime, &mut observers.tf, "old-generation correction TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "old-generation correction global map",
    );
    assert_eq!(
        get_mapping_state(&runtime, &mut second_client, &second.scenario),
        mapping_before_old_generation
    );

    runtime
        .block_on(observers.transport.publish(first_odometry_frame))
        .expect("replay old public odometry into restarted process");
    runtime
        .block_on(observers.transport.publish(first_cloud_frame))
        .expect("replay old public cloud into restarted process");
    let stale_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "echoed stale public odometry",
    ));
    let stale_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "echoed stale public cloud",
    ));
    assert_eq!(stale_odometry.runtime_generation, first_generation as i64);
    assert_eq!(stale_cloud.runtime_generation, first_generation as i64);
    assert_stream_silent(&runtime, &mut observers.tf, "stale public replay TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "stale public replay global map",
    );

    write_publish_permit(&permit_path, 2);
    let _second_odometry = next_frame(
        &runtime,
        &mut observers.public_odometry,
        "restarted current odometry",
    );
    let second_tf = next_frame(&runtime, &mut observers.tf, "restarted canonical TF");
    let _second_cloud = next_frame(
        &runtime,
        &mut observers.public_cloud,
        "restarted registered cloud",
    );
    let second_global_map =
        next_nonempty_global_map(&runtime, &mut observers.global_map, "restarted global map");
    let second_committed = assert_mapping_outputs(
        &runtime,
        &mut second_client,
        &probe,
        &second.scenario,
        &second_started,
        &second_tf,
        &second_global_map,
    );
    assert!(second_committed.revision > post_hostile_mapping.revision);
    finish_native_cycle(&runtime, &mut observers, second_client, second);
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[test]
fn raytracing_profile_real_lcm_v2_preserves_generation_and_stop_fences() {
    let runtime = tokio::runtime::Runtime::new().expect("create G015 fixture runtime");
    let probe = ProbeDir::new("g015-raytracing-v2");
    let fixture_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../nuwax-workspace/crates/nuwax-lio-backends/fixtures/recorded_scan_imu.json")
        .canonicalize()
        .expect("canonicalize recorded LIO fixture");
    let permit_path = probe.path().join("recorded-publish-permit");
    write_publish_permit(&permit_path, 0);
    let mut observers = NativeObservers::new(&runtime, probe.lcm_url());

    let mut first = spawn_native_cycle(
        &runtime,
        &observers,
        &probe,
        ("fastlio2", "raytracing", "none"),
        &fixture_path,
        &permit_path,
    );
    write_publish_permit(&permit_path, 1);
    let _first_private_odometry = next_frame(
        &runtime,
        &mut first.private_odometry,
        "G015 first private odometry",
    );
    let _first_private_cloud = next_frame(
        &runtime,
        &mut first.private_cloud,
        "G015 first private cloud",
    );
    wait_cycle_ready(&mut first, &probe);
    assert_cycle_starting_and_ready(&runtime, &mut observers, &first);
    let first_inactive_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "G015 first inactive public odometry",
    ));
    assert_eq!(first_inactive_odometry.source_sequence, 1);
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "G015 first inactive public cloud",
    );
    assert_stream_silent(&runtime, &mut observers.tf, "G015 first inactive TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "G015 first inactive global map",
    );

    let mut first_client = connect_geometry(&runtime, &probe, &first.scenario);
    let first_started = start_mapping(&runtime, &mut first_client, &first.scenario);
    let first_replay_tf = next_frame(&runtime, &mut observers.tf, "G015 first deferred replay TF");
    let first_replay_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "G015 first deferred replay cloud",
    ));
    assert_deferred_replay_outputs(
        &first.scenario,
        &first_inactive_odometry,
        &first_replay_cloud,
        &first_replay_tf,
    );
    let _first_replay_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "G015 first deferred replay global map",
    );
    std::thread::sleep(RAYTRACING_CANDIDATE_CADENCE_WAIT);
    write_publish_permit(&permit_path, 2);
    let first_odometry_frame = next_frame(
        &runtime,
        &mut observers.public_odometry,
        "G015 first public odometry",
    );
    let first_odometry = decode_public_odometry(&first_odometry_frame);
    let first_tf_frame = next_frame(&runtime, &mut observers.tf, "G015 first canonical TF");
    let first_cloud_frame = next_frame(
        &runtime,
        &mut observers.public_cloud,
        "G015 first registered cloud",
    );
    let first_cloud = decode_public_cloud(&first_cloud_frame);
    assert_exactly_once_cloud_transform("fastlio2", &first_odometry, &first_cloud, &first_tf_frame);
    let first_global_map_frame =
        next_nonempty_global_map(&runtime, &mut observers.global_map, "G015 first global map");
    let first_committed = assert_mapping_outputs(
        &runtime,
        &mut first_client,
        &probe,
        &first.scenario,
        &first_started,
        &first_tf_frame,
        &first_global_map_frame,
    );
    drain_matching_global_map(
        &runtime,
        &mut observers.global_map,
        &first_global_map_frame,
        "G015 first global-map duplicate changed content",
    );
    let first_generation = first.generation;
    finish_native_cycle(&runtime, &mut observers, first_client, first);

    write_publish_permit(&permit_path, 0);
    let mut second = spawn_native_cycle(
        &runtime,
        &observers,
        &probe,
        ("pointlio", "raytracing", "none"),
        &fixture_path,
        &permit_path,
    );
    assert_eq!(second.generation, first_generation + 1);
    write_publish_permit(&permit_path, 1);
    let _second_private_odometry = next_frame(
        &runtime,
        &mut second.private_odometry,
        "G015 second private odometry",
    );
    let _second_private_cloud = next_frame(
        &runtime,
        &mut second.private_cloud,
        "G015 second private cloud",
    );
    wait_cycle_ready(&mut second, &probe);
    assert_cycle_starting_and_ready(&runtime, &mut observers, &second);
    let second_inactive_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "G015 second inactive public odometry",
    ));
    assert_eq!(second_inactive_odometry.source_sequence, 1);
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "G015 second inactive public cloud",
    );
    assert_stream_silent(&runtime, &mut observers.tf, "G015 second inactive TF");

    drain_matching_global_map(
        &runtime,
        &mut observers.global_map,
        &first_global_map_frame,
        "G015 restarted observer baseline changed content",
    );
    let mut second_client = connect_geometry(&runtime, &probe, &second.scenario);
    let second_started = start_mapping(&runtime, &mut second_client, &second.scenario);
    let second_replay_tf = next_frame(
        &runtime,
        &mut observers.tf,
        "G015 second deferred replay TF",
    );
    let second_replay_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "G015 second deferred replay cloud",
    ));
    assert_deferred_replay_outputs(
        &second.scenario,
        &second_inactive_odometry,
        &second_replay_cloud,
        &second_replay_tf,
    );
    let _second_replay_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "G015 second deferred replay global map",
    );
    std::thread::sleep(RAYTRACING_CANDIDATE_CADENCE_WAIT);
    #[cfg(target_os = "linux")]
    let second_runtime_pid = second.child.child_mut().id();
    #[cfg(target_os = "linux")]
    let (second_sidecar_pid, second_sidecar_start_time) =
        wait_for_mapping_sidecar(second_runtime_pid, second.pid, &second.scenario);
    let mapping_before_stale = get_mapping_state(&runtime, &mut second_client, &second.scenario);
    let occupancy_before_stale = read_file_prefix(&probe.path().join("occupancy.mmap"), 4096);
    let cloud_before_stale = read_file_prefix(&probe.path().join("cloud-map.mmap"), 4096);

    runtime
        .block_on(observers.transport.publish(first_odometry_frame.clone()))
        .expect("replay G015 old-generation odometry");
    runtime
        .block_on(observers.transport.publish(first_cloud_frame.clone()))
        .expect("replay G015 old-generation cloud");
    assert_eq!(
        decode_public_odometry(&next_frame(
            &runtime,
            &mut observers.public_odometry,
            "G015 echoed old odometry",
        ))
        .runtime_generation,
        first_generation as i64
    );
    assert_eq!(
        decode_public_cloud(&next_frame(
            &runtime,
            &mut observers.public_cloud,
            "G015 echoed old cloud",
        ))
        .runtime_generation,
        first_generation as i64
    );
    assert_stream_silent(&runtime, &mut observers.tf, "G015 stale-generation TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "G015 stale-generation global map",
    );
    assert_eq!(
        get_mapping_state(&runtime, &mut second_client, &second.scenario),
        mapping_before_stale
    );
    assert_eq!(
        read_file_prefix(&probe.path().join("occupancy.mmap"), 4096),
        occupancy_before_stale
    );
    assert_eq!(
        read_file_prefix(&probe.path().join("cloud-map.mmap"), 4096),
        cloud_before_stale
    );

    write_publish_permit(&permit_path, 2);
    let second_odometry_frame = next_frame(
        &runtime,
        &mut observers.public_odometry,
        "G015 second public odometry",
    );
    let second_odometry = decode_public_odometry(&second_odometry_frame);
    let second_tf_frame = next_frame(&runtime, &mut observers.tf, "G015 second canonical TF");
    let second_cloud_frame = next_frame(
        &runtime,
        &mut observers.public_cloud,
        "G015 second registered cloud",
    );
    let second_cloud = decode_public_cloud(&second_cloud_frame);
    assert_exactly_once_cloud_transform(
        "pointlio",
        &second_odometry,
        &second_cloud,
        &second_tf_frame,
    );
    let second_global_map_frame = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "G015 second global map",
    );
    let second_committed = assert_mapping_outputs(
        &runtime,
        &mut second_client,
        &probe,
        &second.scenario,
        &second_started,
        &second_tf_frame,
        &second_global_map_frame,
    );
    drain_matching_global_map(
        &runtime,
        &mut observers.global_map,
        &second_global_map_frame,
        "G015 second global-map duplicate changed content",
    );
    assert!(second_committed.revision > first_committed.revision);

    let occupancy_before_stop = read_file_prefix(&probe.path().join("occupancy.mmap"), 4096);
    let cloud_before_stop = read_file_prefix(&probe.path().join("cloud-map.mmap"), 4096);
    let stopped = stop_mapping(
        &runtime,
        &mut second_client,
        &second_started.session_id,
        &second.scenario,
    );
    assert!(stopped.success && !stopped.active);
    assert_eq!(stopped.revision, second_committed.revision);
    #[cfg(target_os = "linux")]
    wait_for_child_exit(
        second_sidecar_pid,
        second_sidecar_start_time,
        &second.scenario,
    );

    runtime
        .block_on(observers.transport.publish(second_odometry_frame))
        .expect("replay stopped-session odometry");
    runtime
        .block_on(observers.transport.publish(second_cloud_frame))
        .expect("replay stopped-session cloud");
    let _echoed_stopped_odometry = next_frame(
        &runtime,
        &mut observers.public_odometry,
        "G015 echoed stopped odometry",
    );
    let _echoed_stopped_cloud = next_frame(
        &runtime,
        &mut observers.public_cloud,
        "G015 echoed stopped cloud",
    );
    assert_stream_silent(&runtime, &mut observers.tf, "G015 stopped-session TF");
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "G015 stopped-session global map",
    );
    let stopped_state = get_mapping_state(&runtime, &mut second_client, &second.scenario);
    assert!(!stopped_state.active);
    assert_eq!(stopped_state.revision, second_committed.revision);
    assert_eq!(
        read_file_prefix(&probe.path().join("occupancy.mmap"), 4096),
        occupancy_before_stop
    );
    assert_eq!(
        read_file_prefix(&probe.path().join("cloud-map.mmap"), 4096),
        cloud_before_stop
    );

    finish_native_cycle(&runtime, &mut observers, second_client, second);
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn assert_native_pgo_geometry_sigkill_cleanup_is_distinguished(runtime: &tokio::runtime::Runtime) {
    let probe = ProbeDir::new("g004-pgo-kill");
    let probe_path = probe.path().to_path_buf();
    let fixture_path = write_native_pgo_recorded_fixture(&probe);
    let permit_path = probe.path().join("native-pgo-publish-permit");
    write_publish_permit(&permit_path, 0);
    let mut observers = NativeObservers::new(runtime, probe.lcm_url());
    let mut native_pgo_observers = NativePgoObservers::new(runtime, probe.lcm_url(), 1);
    let mut cycle = spawn_native_cycle(
        runtime,
        &observers,
        &probe,
        ("fastlio2", "voxel", "native_pgo"),
        &fixture_path,
        &permit_path,
    );
    let geometry_pid = cycle.child.child_mut().id();
    let native_lio_pid = cycle.pid;

    write_publish_permit(&permit_path, 1);
    let _private_odometry = next_frame(
        runtime,
        &mut cycle.private_odometry,
        "Geometry SIGKILL private odometry",
    );
    let _private_cloud = next_frame(
        runtime,
        &mut cycle.private_cloud,
        "Geometry SIGKILL private cloud",
    );
    wait_cycle_ready(&mut cycle, &probe);
    assert_cycle_starting_and_ready(runtime, &mut observers, &cycle);
    let _starting = wait_for_native_pgo_status(
        runtime,
        &mut native_pgo_observers.status,
        1,
        native_pgo::NATIVE_PGO_EVENT_STARTING,
        None,
        "Geometry SIGKILL native PGO STARTING",
    );
    let _ready = wait_for_native_pgo_status(
        runtime,
        &mut native_pgo_observers.status,
        1,
        native_pgo::NATIVE_PGO_EVENT_READY,
        None,
        "Geometry SIGKILL native PGO READY",
    );
    let (native_pgo_pid, native_pgo_start_time, guardian_pid, guardian_start_time) =
        wait_for_native_pgo_child(geometry_pid, native_lio_pid, &cycle.scenario);
    cycle.child.track_process_group(
        native_pgo_pid,
        native_pgo_start_time,
        "Geometry SIGKILL native PGO generation",
    );

    let child = std::mem::replace(&mut cycle.child, ChildGuard::empty());
    let (output, outcomes) = child.sigkill_and_collect_with_process_group_outcomes();
    assert_eq!(
        output.status.signal(),
        Some(nix::libc::SIGKILL),
        "Geometry SIGKILL scenario did not terminate Geometry with SIGKILL"
    );
    assert_eq!(
        outcomes,
        vec![ProcessGroupCleanupOutcome {
            pid: native_pgo_pid,
            scenario: "Geometry SIGKILL native PGO generation".to_string(),
            disposition: ProcessGroupCleanupDisposition::ExitedWithoutHarnessSignal,
        }],
        "native PGO did not self-exit after Geometry SIGKILL without a harness signal"
    );
    assert_eq!(
        process_start_time(geometry_pid),
        None,
        "SIGKILLed Geometry process remained"
    );
    assert_eq!(
        process_start_time(native_pgo_pid),
        None,
        "parent-death-cleaned native PGO process remained"
    );
    assert_ne!(
        process_start_time(guardian_pid),
        Some(guardian_start_time),
        "parent-death guardian process remained"
    );
    assert!(
        cycle.ready_path.exists(),
        "Geometry SIGKILL unexpectedly removed its ready file without an external supervisor"
    );

    drop(cycle);
    assert_native_child_reaped(native_lio_pid, NATIVE_FIXTURE_TIMEOUT);
    drop(native_pgo_observers);
    drop(observers);
    drop(probe);
    assert!(
        !probe_path.exists(),
        "Geometry SIGKILL harness left its probe directory behind"
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
#[test]
fn native_pgo_production_binary_real_lcm_v2_reaches_canonical_applier() {
    let runtime = tokio::runtime::Runtime::new().expect("create native PGO V2 runtime");
    assert_native_pgo_geometry_sigkill_cleanup_is_distinguished(&runtime);
    let probe = ProbeDir::new("g004-native-pgo-v2");
    let fixture_path = write_native_pgo_recorded_fixture(&probe);
    let permit_path = probe.path().join("native-pgo-publish-permit");
    write_publish_permit(&permit_path, 0);
    let mut observers = NativeObservers::new(&runtime, probe.lcm_url());
    let mut first_pgo = NativePgoObservers::new(&runtime, probe.lcm_url(), 1);

    let mut first = spawn_native_cycle(
        &runtime,
        &observers,
        &probe,
        ("fastlio2", "voxel", "native_pgo"),
        &fixture_path,
        &permit_path,
    );
    assert_eq!(first.generation, 1, "fresh native tuple generation drifted");
    let first_geometry_pid = first.child.child_mut().id();

    write_publish_permit(&permit_path, 1);
    let _first_private_odometry = next_frame(
        &runtime,
        &mut first.private_odometry,
        "native PGO first private odometry",
    );
    let _first_private_cloud = next_frame(
        &runtime,
        &mut first.private_cloud,
        "native PGO first private cloud",
    );
    wait_cycle_ready(&mut first, &probe);
    assert_cycle_starting_and_ready(&runtime, &mut observers, &first);
    let native_starting = wait_for_native_pgo_status(
        &runtime,
        &mut first_pgo.status,
        first.generation,
        native_pgo::NATIVE_PGO_EVENT_STARTING,
        None,
        "native PGO STARTING",
    );
    let native_ready = wait_for_native_pgo_status(
        &runtime,
        &mut first_pgo.status,
        first.generation,
        native_pgo::NATIVE_PGO_EVENT_READY,
        None,
        "native PGO READY",
    );
    assert_eq!(native_starting.status_sequence, 1);
    assert_eq!(
        native_starting.health_state,
        native_pgo::NATIVE_PGO_HEALTH_UNKNOWN
    );
    assert_eq!(
        native_starting.terminal_state,
        native_pgo::NATIVE_PGO_TERMINAL_NONE
    );
    assert_eq!(
        native_starting.cancel_state,
        native_pgo::NATIVE_PGO_CANCEL_NONE
    );
    assert_eq!(
        native_starting.reason_code,
        native_pgo::NATIVE_PGO_REASON_NONE
    );
    assert!(native_starting.reason.is_empty());
    assert_eq!(native_ready.status_sequence, 2);
    assert_eq!(
        native_ready.health_state,
        native_pgo::NATIVE_PGO_HEALTH_HEALTHY
    );
    assert_eq!(
        native_ready.terminal_state,
        native_pgo::NATIVE_PGO_TERMINAL_NONE
    );
    assert_eq!(
        native_ready.cancel_state,
        native_pgo::NATIVE_PGO_CANCEL_NONE
    );
    assert_eq!(native_ready.reason_code, native_pgo::NATIVE_PGO_REASON_NONE);
    assert!(native_ready.reason.is_empty());
    let (first_pgo_pid, first_pgo_start_time, first_guardian_pid, first_guardian_start_time) =
        wait_for_native_pgo_child(first_geometry_pid, first.pid, &first.scenario);
    first.child.track_process_group(
        first_pgo_pid,
        first_pgo_start_time,
        "first native PGO generation",
    );

    let inactive_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "native PGO inactive odometry",
    ));
    assert_eq!(inactive_odometry.source_sequence, 1);
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "native PGO inactive cloud",
    );
    let mut first_client = connect_geometry(&runtime, &probe, &first.scenario);
    let first_started = start_mapping(&runtime, &mut first_client, &first.scenario);
    let first_replay_tf = next_frame(&runtime, &mut observers.tf, "native PGO deferred replay TF");
    let first_replay_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "native PGO deferred replay cloud",
    ));
    assert_deferred_replay_outputs(
        &first.scenario,
        &inactive_odometry,
        &first_replay_cloud,
        &first_replay_tf,
    );
    let initial_map_from_odom = map_from_odom_in_frame(&first_replay_tf);
    let uncorrected_map_from_odom = transform_isometry(
        initial_map_from_odom.transform.translation,
        initial_map_from_odom.transform.rotation_xyzw,
    );
    let _first_replay_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "native PGO deferred replay global map",
    );
    let rejected = observe_native_pgo_exchange(
        &runtime,
        &mut first_pgo,
        first.generation,
        1,
        1,
        "native PGO resource rejection",
    );
    assert_eq!(
        rejected.request.identity.mapping_session_id,
        first_started.session_id
    );
    assert_eq!(
        rejected.request.identity.mapping_session_generation,
        first_started.session_generation as i64
    );
    assert_eq!(rejected.request.identity.registered_cloud.point_count, 19);
    assert_eq!(
        rejected.result.result_disposition,
        native_pgo::NATIVE_PGO_DISPOSITION_REJECTED
    );
    assert_eq!(
        rejected.result.rejection_reason_code,
        native_pgo::NATIVE_PGO_REJECTION_RESOURCE_LIMIT
    );
    assert_eq!(
        rejected.result.rejection_reason,
        "registered cloud has too few filtered points"
    );
    assert_eq!(
        rejected.terminal.terminal_state,
        native_pgo::NATIVE_PGO_TERMINAL_REJECTED
    );
    assert_eq!(
        rejected.terminal.reason_code,
        native_pgo::NATIVE_PGO_REASON_INVALID_REQUEST
    );
    assert_eq!(
        rejected.terminal.reason,
        "registered cloud has too few filtered points"
    );
    assert_stream_silent(
        &runtime,
        &mut first_pgo.library_candidate,
        "native tuple library candidate side effect",
    );
    assert_stream_silent(
        &runtime,
        &mut first_pgo.cancel,
        "native tuple unexpected cancel",
    );
    assert_stream_silent(
        &runtime,
        &mut observers.tf,
        "rejected native PGO correction TF",
    );

    for permit in 2..=3 {
        std::thread::sleep(RAYTRACING_CANDIDATE_CADENCE_WAIT);
        write_publish_permit(&permit_path, permit);
        let label = |stream: &str| format!("native PGO {stream} sequence {permit}");
        let _private_odometry = next_frame(
            &runtime,
            &mut first.private_odometry,
            &label("private odometry"),
        );
        let _private_cloud =
            next_frame(&runtime, &mut first.private_cloud, &label("private cloud"));
        let _odometry = next_frame(
            &runtime,
            &mut observers.public_odometry,
            &label("public odometry"),
        );
        let tf = next_frame(&runtime, &mut observers.tf, &label("canonical TF"));
        let map_from_odom = map_from_odom_in_frame(&tf);
        assert_eq!(
            map_from_odom.header.seq, initial_map_from_odom.header.seq,
            "native PGO base transform generation drifted at source sequence {permit}"
        );
        assert_transform_matches_isometry(
            map_from_odom.transform.translation,
            map_from_odom.transform.rotation_xyzw,
            &uncorrected_map_from_odom,
            &format!("native PGO uncorrected TF at source sequence {permit}"),
        );
        let _cloud = next_frame(
            &runtime,
            &mut observers.public_cloud,
            &label("registered cloud"),
        );
        let _global_map =
            next_nonempty_global_map(&runtime, &mut observers.global_map, &label("global map"));
        let exchange = observe_native_pgo_exchange(
            &runtime,
            &mut first_pgo,
            first.generation,
            i64::from(permit),
            i64::from(permit),
            &label("exchange"),
        );
        assert_eq!(
            exchange.result.result_disposition,
            native_pgo::NATIVE_PGO_DISPOSITION_NO_CANDIDATE
        );
        assert_eq!(
            exchange.result.rejection_reason_code,
            native_pgo::NATIVE_PGO_REJECTION_NONE
        );
        assert!(exchange.result.rejection_reason.is_empty());
        assert_eq!(
            exchange.terminal.terminal_state,
            native_pgo::NATIVE_PGO_TERMINAL_SUCCEEDED
        );
        assert_eq!(
            exchange.terminal.reason_code,
            native_pgo::NATIVE_PGO_REASON_NONE
        );
        assert!(exchange.terminal.reason.is_empty());
    }

    std::thread::sleep(RAYTRACING_CANDIDATE_CADENCE_WAIT);
    write_publish_permit(&permit_path, 4);
    let _fourth_private_odometry = next_frame(
        &runtime,
        &mut first.private_odometry,
        "native PGO correction private odometry",
    );
    let _fourth_private_cloud = next_frame(
        &runtime,
        &mut first.private_cloud,
        "native PGO correction private cloud",
    );
    let fourth_odometry_frame = next_frame(
        &runtime,
        &mut observers.public_odometry,
        "native PGO correction public odometry",
    );
    let fourth_odometry = decode_public_odometry(&fourth_odometry_frame);
    let fourth_tf = next_frame(
        &runtime,
        &mut observers.tf,
        "native PGO pre-correction canonical TF",
    );
    let fourth_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "native PGO correction registered cloud",
    ));
    let fourth_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "native PGO correction global map",
    );
    let fourth_global_map_message = decode_global_map(&fourth_global_map);
    let mapping_before_correction = get_mapping_state(&runtime, &mut first_client, &first.scenario);
    let occupancy_before_correction =
        get_occupancy_data(&runtime, &mut first_client, &first.scenario);
    let occupancy_shm_before_correction = get_occupancy_shm(
        &runtime,
        &mut first_client,
        &first.scenario,
        Instant::now() + NATIVE_FIXTURE_TIMEOUT,
    );
    let occupancy_file_before_correction = fs::read(&occupancy_shm_before_correction.key)
        .expect("read native PGO occupancy SHM before correction");
    let cloud_file_before_correction = fs::read(probe.path().join("cloud-map.mmap"))
        .expect("read native PGO cloud-map SHM before correction");
    let cloud_shm_before_correction = read_cloud_map_shm(&probe.path().join("cloud-map.mmap"));

    let correction = observe_native_pgo_exchange(
        &runtime,
        &mut first_pgo,
        first.generation,
        4,
        4,
        "native PGO correction",
    );
    assert_eq!(
        correction.result.result_disposition,
        native_pgo::NATIVE_PGO_DISPOSITION_CORRECTION
    );
    assert_eq!(
        correction.result.transform_semantics,
        native_pgo::NATIVE_PGO_TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3
    );
    assert_eq!(
        correction.terminal.terminal_state,
        native_pgo::NATIVE_PGO_TERMINAL_SUCCEEDED
    );
    assert_eq!(
        correction.terminal.reason_code,
        native_pgo::NATIVE_PGO_REASON_NONE
    );
    assert!(correction.terminal.reason.is_empty());
    assert_eq!(
        correction.request.identity.mapping_session_id,
        first_started.session_id
    );
    assert_eq!(
        correction.request.identity.mapping_session_generation,
        first_started.session_generation as i64
    );
    assert_eq!(
        correction.request.identity.occupancy_revision,
        mapping_before_correction.revision as i64
    );
    assert_eq!(
        correction.request.identity.cloud_revision,
        i64::from(fourth_global_map_message.header.seq)
    );
    assert_eq!(
        correction.request.identity.source_timestamp_ns,
        fourth_cloud.source_timestamp_ns
    );
    assert_eq!(
        correction.request.identity.registered_cloud.source_sequence,
        fourth_cloud.source_sequence
    );
    assert_eq!(
        correction.request.identity.correlated_odom.source_sequence,
        fourth_odometry.source_sequence
    );
    let correction_magnitude = correction
        .result
        .translation_m
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    assert!(
        correction_magnitude > 1.0e-6,
        "native PGO returned an identity correction instead of closing the loop"
    );

    let baseline_map_from_odom = map_from_odom_in_frame(&fourth_tf);
    assert_eq!(
        baseline_map_from_odom.header.seq,
        initial_map_from_odom.header.seq
    );
    assert_transform_matches_isometry(
        baseline_map_from_odom.transform.translation,
        baseline_map_from_odom.transform.rotation_xyzw,
        &uncorrected_map_from_odom,
        "native PGO fourth-frame base transform",
    );
    assert_eq!(
        schema_transform_timestamp_ns(&baseline_map_from_odom),
        u64::try_from(fourth_odometry.source_timestamp_ns)
            .expect("fourth odometry timestamp is nonnegative")
    );
    let base_map_from_odom = transform_isometry(
        baseline_map_from_odom.transform.translation,
        baseline_map_from_odom.transform.rotation_xyzw,
    );
    let native_correction = transform_isometry(
        correction.result.translation_m,
        correction.result.rotation_xyzw,
    );
    let expected_map_from_odom = base_map_from_odom * native_correction;
    let corrected_tf = wait_for_exact_map_from_odom(
        &runtime,
        &mut observers.tf,
        &expected_map_from_odom,
        u64::try_from(fourth_odometry.source_timestamp_ns)
            .expect("fourth odometry timestamp is nonnegative"),
        1,
        "native PGO canonical correction TF",
    );
    assert_frame_tree_map_from_odom(
        &get_frame_tree(&runtime, &mut first_client, &first.scenario),
        &expected_map_from_odom,
        u64::try_from(fourth_odometry.source_timestamp_ns)
            .expect("fourth odometry timestamp is nonnegative"),
        "native PGO corrected frame tree",
    );
    assert_eq!(
        get_mapping_state(&runtime, &mut first_client, &first.scenario),
        mapping_before_correction,
        "native correction changed mapping state/revision"
    );
    assert_eq!(
        get_occupancy_data(&runtime, &mut first_client, &first.scenario),
        occupancy_before_correction,
        "native correction changed occupancy RPC data"
    );
    assert_eq!(
        get_occupancy_shm(
            &runtime,
            &mut first_client,
            &first.scenario,
            Instant::now() + NATIVE_FIXTURE_TIMEOUT,
        ),
        occupancy_shm_before_correction,
        "native correction changed the occupancy SHM descriptor"
    );
    assert_eq!(
        fs::read(&occupancy_shm_before_correction.key)
            .expect("read native PGO occupancy SHM after correction"),
        occupancy_file_before_correction,
        "native correction changed occupancy SHM bytes"
    );
    assert_eq!(
        fs::read(probe.path().join("cloud-map.mmap"))
            .expect("read native PGO cloud-map SHM after correction"),
        cloud_file_before_correction,
        "native correction changed cloud-map SHM bytes"
    );
    let cloud_shm_after_correction = read_cloud_map_shm(&probe.path().join("cloud-map.mmap"));
    assert_eq!(
        cloud_shm_after_correction.revision,
        cloud_shm_before_correction.revision
    );
    assert_eq!(
        cloud_shm_after_correction.point_count,
        cloud_shm_before_correction.point_count
    );
    assert_eq!(
        cloud_shm_after_correction.payload,
        cloud_shm_before_correction.payload
    );
    drain_matching_global_map(
        &runtime,
        &mut observers.global_map,
        &fourth_global_map,
        "native correction changed global-map content",
    );
    assert_stream_silent(
        &runtime,
        &mut first_pgo.library_candidate,
        "native correction used the library candidate channel",
    );

    std::thread::sleep(RAYTRACING_CANDIDATE_CADENCE_WAIT);
    write_publish_permit(&permit_path, 5);
    let _fifth_private_odometry = next_frame(
        &runtime,
        &mut first.private_odometry,
        "native PGO retained correction private odometry",
    );
    let _fifth_private_cloud = next_frame(
        &runtime,
        &mut first.private_cloud,
        "native PGO retained correction private cloud",
    );
    let fifth_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "native PGO retained correction public odometry",
    ));
    let fifth_tf = next_frame(
        &runtime,
        &mut observers.tf,
        "native PGO retained correction TF",
    );
    let fifth_map_from_odom = map_from_odom_in_frame(&fifth_tf);
    assert_eq!(
        fifth_map_from_odom.header.seq, 1,
        "next odometry did not retain native correction revision"
    );
    assert_eq!(
        schema_transform_timestamp_ns(&fifth_map_from_odom),
        u64::try_from(fifth_odometry.source_timestamp_ns)
            .expect("fifth odometry timestamp is nonnegative")
    );
    assert_transform_matches_isometry(
        fifth_map_from_odom.transform.translation,
        fifth_map_from_odom.transform.rotation_xyzw,
        &expected_map_from_odom,
        "next odometry native correction snapshot",
    );
    let _fifth_cloud = next_frame(
        &runtime,
        &mut observers.public_cloud,
        "native PGO retained correction cloud",
    );
    let fifth_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "native PGO retained correction global map",
    );
    let post_correction = observe_native_pgo_exchange(
        &runtime,
        &mut first_pgo,
        first.generation,
        5,
        5,
        "native PGO post-correction non-keyframe",
    );
    assert_eq!(
        post_correction.result.result_disposition,
        native_pgo::NATIVE_PGO_DISPOSITION_NO_CANDIDATE
    );
    assert_eq!(
        post_correction.terminal.terminal_state,
        native_pgo::NATIVE_PGO_TERMINAL_SUCCEEDED
    );

    let mapping_before_hostile = get_mapping_state(&runtime, &mut first_client, &first.scenario);
    let occupancy_before_hostile = get_occupancy_data(&runtime, &mut first_client, &first.scenario);
    let occupancy_file_before_hostile = fs::read(&occupancy_shm_before_correction.key)
        .expect("read occupancy before hostile canonical injection");
    let cloud_file_before_hostile = fs::read(probe.path().join("cloud-map.mmap"))
        .expect("read cloud map before hostile canonical injection");
    let hostile_occupancy = schema::OccupancyGrid {
        header: schema::Header {
            seq: i32::MAX,
            stamp_sec: 1,
            frame_id: "map".into(),
            ..schema::Header::default()
        },
        info: schema::MapMetaData {
            resolution: 1.0,
            width: 1,
            height: 1,
            ..schema::MapMetaData::default()
        },
        data: vec![100],
    };
    runtime
        .block_on(
            observers.transport.publish(
                nuwax_protocol::RawLcmFrameBuilder::new(
                    LcmTopic::new("/map/occupancy").unwrap(),
                    LcmMessageType::new("nav_msgs.OccupancyGrid").unwrap(),
                )
                .seqno(u32::MAX)
                .payload(schema::encode_occupancy_grid_body(&hostile_occupancy).unwrap())
                .build(),
            ),
        )
        .expect("publish hostile canonical occupancy");
    let hostile_tf = schema::TfMessage {
        transforms: vec![schema::TransformStamped {
            header: schema::Header {
                seq: i32::MAX,
                stamp_sec: 1,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            child_frame_id: "odom".into(),
            transform: schema::Transform {
                translation: [99.0, 98.0, 97.0],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        }],
    };
    runtime
        .block_on(
            observers.transport.publish(
                nuwax_protocol::RawLcmFrameBuilder::new(
                    LcmTopic::new("/tf").unwrap(),
                    LcmMessageType::new("tf2_msgs.TFMessage").unwrap(),
                )
                .seqno(u32::MAX)
                .payload(schema::encode_tf_message_body(&hostile_tf).unwrap())
                .build(),
            ),
        )
        .expect("publish hostile canonical TF");
    let injected_tf = next_frame(&runtime, &mut observers.tf, "hostile TF echo");
    assert_eq!(
        map_from_odom_in_frame(&injected_tf).transform.translation,
        [99.0, 98.0, 97.0]
    );
    std::thread::sleep(OUTPUT_SILENCE_WINDOW);
    assert_eq!(
        get_mapping_state(&runtime, &mut first_client, &first.scenario),
        mapping_before_hostile,
        "external canonical traffic changed mapping state"
    );
    assert_eq!(
        get_occupancy_data(&runtime, &mut first_client, &first.scenario),
        occupancy_before_hostile,
        "external canonical traffic changed occupancy state"
    );
    assert_eq!(
        fs::read(&occupancy_shm_before_correction.key)
            .expect("read occupancy after hostile canonical injection"),
        occupancy_file_before_hostile,
        "external canonical traffic changed occupancy SHM"
    );
    assert_eq!(
        fs::read(probe.path().join("cloud-map.mmap"))
            .expect("read cloud map after hostile canonical injection"),
        cloud_file_before_hostile,
        "external canonical traffic changed cloud-map SHM"
    );
    assert_frame_tree_map_from_odom(
        &get_frame_tree(&runtime, &mut first_client, &first.scenario),
        &expected_map_from_odom,
        u64::try_from(fifth_odometry.source_timestamp_ns)
            .expect("fifth odometry timestamp is nonnegative"),
        "hostile TF did not mutate canonical frame tree",
    );
    drain_matching_global_map(
        &runtime,
        &mut observers.global_map,
        &fifth_global_map,
        "hostile traffic changed global-map content",
    );

    let expected_candidate_payload = native_candidate_payload_sha256(&correction.result);
    let old_topics = nuwax_nav_pgo_bridge::OnlinePgoTopics::for_runtime(1, 1)
        .expect("derive old native PGO topics");
    let old_status_binding = binding_from_pattern(&old_topics.status, "nuwax_msgs.NativePgoStatus");
    let old_status_frame =
        nuwax_protocol::RawLcmFrameBuilder::new(old_status_binding.0, old_status_binding.1)
            .seqno(u32::MAX - 1)
            .payload(
                native_pgo::encode_native_pgo_status_body(&correction.terminal)
                    .expect("encode old native PGO terminal status"),
            )
            .build();
    let old_cancel = native_pgo::seal_native_pgo_status(native_pgo::NativePgoStatus {
        schema_version: native_pgo::NATIVE_PGO_SCHEMA_VERSION_V1,
        backend: native_pgo::NATIVE_PGO_BACKEND,
        runtime_generation: 1,
        backend_generation: 1,
        status_sequence: 1,
        status_timestamp_ns: 0,
        request: Some(correction.request.echo()),
        event_kind: native_pgo::NATIVE_PGO_EVENT_CANCEL_REQUESTED,
        health_state: native_pgo::NATIVE_PGO_HEALTH_UNKNOWN,
        terminal_state: native_pgo::NATIVE_PGO_TERMINAL_NONE,
        cancel_state: native_pgo::NATIVE_PGO_CANCEL_REQUESTED,
        reason_code: native_pgo::NATIVE_PGO_REASON_CANCEL_REQUESTED,
        reason: "cancel_requested".into(),
        status_payload_sha256: [0; 32],
    })
    .expect("seal old native PGO cancel");
    let old_cancel_binding = binding_from_pattern(&old_topics.cancel, "nuwax_msgs.NativePgoStatus");
    let old_cancel_frame =
        nuwax_protocol::RawLcmFrameBuilder::new(old_cancel_binding.0, old_cancel_binding.1)
            .seqno(u32::MAX)
            .payload(
                native_pgo::encode_native_pgo_status_body(&old_cancel)
                    .expect("encode old native PGO cancel"),
            )
            .build();
    let old_request_frame = correction.request_frame.clone();
    let old_result_frame = correction.result_frame.clone();

    let first_output = finish_native_cycle(&runtime, &mut observers, first_client, first);
    let native_stopping = wait_for_native_pgo_status(
        &runtime,
        &mut first_pgo.status,
        1,
        native_pgo::NATIVE_PGO_EVENT_STOPPING,
        None,
        "native PGO STOPPING",
    );
    let native_stopped = wait_for_native_pgo_status(
        &runtime,
        &mut first_pgo.status,
        1,
        native_pgo::NATIVE_PGO_EVENT_STOPPED,
        None,
        "native PGO STOPPED",
    );
    assert!(native_stopping.status_sequence < native_stopped.status_sequence);
    assert_eq!(
        native_stopping.reason_code,
        native_pgo::NATIVE_PGO_REASON_SHUTDOWN_REQUESTED
    );
    assert_eq!(
        native_stopped.reason_code,
        native_pgo::NATIVE_PGO_REASON_SHUTDOWN_REQUESTED
    );
    wait_for_native_pgo_process_group_exit(
        first_pgo_pid,
        first_pgo_start_time,
        "first native PGO generation",
    );
    assert_ne!(
        process_start_time(first_guardian_pid),
        Some(first_guardian_start_time),
        "first native PGO guardian remained after Stop response drain"
    );
    assert_eq!(
        process_start_time(first_geometry_pid),
        None,
        "first Geometry process remained after Stop response drain"
    );
    let finalization_events = candidate_finalization_events(&first_output);
    assert_eq!(
        finalization_events.len(),
        1,
        "native direct finalizer did not emit exactly one keyed event: {finalization_events:?}"
    );
    let finalized = &finalization_events[0];
    assert_eq!(finalized.event, CORRECTION_FINALIZATION_EVENT);
    assert_eq!(finalized.runtime_generation, 1);
    assert_eq!(finalized.backend_generation, 1);
    assert_eq!(
        finalized.session_generation,
        correction.request.identity.mapping_session_generation as u64
    );
    assert_eq!(finalized.candidate_sequence, 4);
    assert_eq!(
        finalized.payload_sha256,
        digest_hex(&expected_candidate_payload)
    );
    assert_eq!(finalized.disposition, ObservedCandidateDisposition::Applied);
    assert_eq!(finalized.reject_reason, None);
    assert!(finalized.candidate_processed_delivered);
    assert_stream_silent(
        &runtime,
        &mut first_pgo.cancel,
        "first native PGO generation emitted an unexpected cancel",
    );

    write_publish_permit(&permit_path, 0);
    let mut second_pgo = NativePgoObservers::new(&runtime, probe.lcm_url(), 2);
    let mut second = spawn_native_cycle(
        &runtime,
        &observers,
        &probe,
        ("pointlio", "voxel", "native_pgo"),
        &fixture_path,
        &permit_path,
    );
    assert_eq!(second.generation, 2);
    let second_geometry_pid = second.child.child_mut().id();
    write_publish_permit(&permit_path, 1);
    let _second_private_odometry = next_frame(
        &runtime,
        &mut second.private_odometry,
        "restarted native PGO private odometry",
    );
    let _second_private_cloud = next_frame(
        &runtime,
        &mut second.private_cloud,
        "restarted native PGO private cloud",
    );
    wait_cycle_ready(&mut second, &probe);
    assert_cycle_starting_and_ready(&runtime, &mut observers, &second);
    let second_native_starting = wait_for_native_pgo_status(
        &runtime,
        &mut second_pgo.status,
        2,
        native_pgo::NATIVE_PGO_EVENT_STARTING,
        None,
        "restarted native PGO STARTING",
    );
    let second_native_ready = wait_for_native_pgo_status(
        &runtime,
        &mut second_pgo.status,
        2,
        native_pgo::NATIVE_PGO_EVENT_READY,
        None,
        "restarted native PGO READY",
    );
    assert_eq!(second_native_starting.status_sequence, 1);
    assert_eq!(second_native_ready.status_sequence, 2);
    let (second_pgo_pid, second_pgo_start_time, second_guardian_pid, second_guardian_start_time) =
        wait_for_native_pgo_child(second_geometry_pid, second.pid, &second.scenario);
    second.child.track_process_group(
        second_pgo_pid,
        second_pgo_start_time,
        "second native PGO generation",
    );
    let second_inactive_odometry = decode_public_odometry(&next_frame(
        &runtime,
        &mut observers.public_odometry,
        "restarted native PGO inactive odometry",
    ));
    assert_eq!(second_inactive_odometry.runtime_generation, 2);
    assert_stream_silent(
        &runtime,
        &mut observers.public_cloud,
        "restarted native PGO inactive cloud",
    );
    let mut second_client = connect_geometry(&runtime, &probe, &second.scenario);
    let mapping_before_old_traffic =
        get_mapping_state(&runtime, &mut second_client, &second.scenario);
    for frame in [
        old_request_frame,
        old_result_frame,
        old_status_frame,
        old_cancel_frame,
    ] {
        runtime
            .block_on(second_pgo.transport.publish(frame))
            .expect("replay old native PGO traffic on its old topic");
    }
    assert_stream_silent(
        &runtime,
        &mut second_pgo.request,
        "old request crossed into restarted native PGO topic",
    );
    assert_stream_silent(
        &runtime,
        &mut second_pgo.result,
        "old result crossed into restarted native PGO topic",
    );
    assert_stream_silent(
        &runtime,
        &mut second_pgo.cancel,
        "old cancel crossed into restarted native PGO topic",
    );
    assert_stream_silent(
        &runtime,
        &mut second_pgo.library_candidate,
        "old native traffic activated the library backend",
    );
    assert_stream_silent(
        &runtime,
        &mut observers.tf,
        "old native traffic changed restarted canonical TF",
    );
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "old native traffic changed restarted global map",
    );
    assert_eq!(
        get_mapping_state(&runtime, &mut second_client, &second.scenario),
        mapping_before_old_traffic,
        "old native traffic changed restarted mapping state"
    );

    let second_started = start_mapping(&runtime, &mut second_client, &second.scenario);
    let second_replay_tf = next_frame(
        &runtime,
        &mut observers.tf,
        "restarted native PGO deferred replay TF",
    );
    let second_replay_cloud = decode_public_cloud(&next_frame(
        &runtime,
        &mut observers.public_cloud,
        "restarted native PGO deferred replay cloud",
    ));
    assert_deferred_replay_outputs(
        &second.scenario,
        &second_inactive_odometry,
        &second_replay_cloud,
        &second_replay_tf,
    );
    let _second_replay_global_map = next_nonempty_global_map(
        &runtime,
        &mut observers.global_map,
        "restarted native PGO deferred replay global map",
    );
    let current_rejected = observe_native_pgo_exchange(
        &runtime,
        &mut second_pgo,
        2,
        1,
        1,
        "restarted native PGO current work",
    );
    assert_eq!(
        current_rejected.request.identity.mapping_session_id,
        second_started.session_id
    );
    assert_eq!(
        current_rejected.result.result_disposition,
        native_pgo::NATIVE_PGO_DISPOSITION_REJECTED
    );
    assert_eq!(
        current_rejected.result.rejection_reason_code,
        native_pgo::NATIVE_PGO_REJECTION_RESOURCE_LIMIT
    );
    assert_eq!(
        current_rejected.terminal.terminal_state,
        native_pgo::NATIVE_PGO_TERMINAL_REJECTED
    );
    let second_output = finish_native_cycle(&runtime, &mut observers, second_client, second);
    let second_native_stopping = wait_for_native_pgo_status(
        &runtime,
        &mut second_pgo.status,
        2,
        native_pgo::NATIVE_PGO_EVENT_STOPPING,
        None,
        "restarted native PGO STOPPING",
    );
    let second_native_stopped = wait_for_native_pgo_status(
        &runtime,
        &mut second_pgo.status,
        2,
        native_pgo::NATIVE_PGO_EVENT_STOPPED,
        None,
        "restarted native PGO STOPPED",
    );
    assert!(second_native_stopping.status_sequence < second_native_stopped.status_sequence);
    wait_for_native_pgo_process_group_exit(
        second_pgo_pid,
        second_pgo_start_time,
        "second native PGO generation",
    );
    assert_ne!(
        process_start_time(second_guardian_pid),
        Some(second_guardian_start_time),
        "second native PGO guardian remained after Stop response drain"
    );
    assert_eq!(
        process_start_time(second_geometry_pid),
        None,
        "second Geometry process remained after Stop response drain"
    );
    assert!(
        candidate_finalization_events(&second_output).is_empty(),
        "typed rejection unexpectedly entered the canonical finalizer"
    );
    assert_stream_silent(
        &runtime,
        &mut second_pgo.cancel,
        "second native PGO generation emitted an unexpected cancel",
    );
    let _ = corrected_tf;
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
const MATRIX_ROW_FILTER_ENV: &str = "NUWAX_GEOMETRY_MATRIX_ROW";

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MatrixProfile {
    lio: &'static str,
    mapping: &'static str,
    correction: &'static str,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
impl MatrixProfile {
    fn row_id(self) -> String {
        format!("{}+{}+{}", self.lio, self.mapping, self.correction)
    }

    fn scenario(self) -> String {
        format!("matrix-{}-{}-{}", self.lio, self.mapping, self.correction)
    }

    fn is_native_lio(self) -> bool {
        matches!(self.lio, "fastlio2" | "pointlio")
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Clone)]
struct MatrixDeviceFrame {
    odometry: schema::Odometry,
    cloud: schema::PointCloud2,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
struct MatrixPlayback {
    fixture_path: PathBuf,
    source_frames: Vec<nuwax_lio_backends::LioInputFrame>,
    device_frames: Vec<MatrixDeviceFrame>,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_timestamp_header(frame_id: &str, sequence: u32, timestamp_ns: i64) -> schema::Header {
    assert!(timestamp_ns >= 0, "matrix timestamp must be nonnegative");
    schema::Header {
        seq: i32::try_from(sequence).expect("matrix sequence fits i32"),
        stamp_sec: i32::try_from(timestamp_ns / 1_000_000_000)
            .expect("matrix timestamp seconds fit i32"),
        stamp_nsec: i32::try_from(timestamp_ns % 1_000_000_000)
            .expect("matrix timestamp nanoseconds fit i32"),
        frame_id: frame_id.to_owned(),
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_device_frames(fixture_path: &Path) -> Vec<MatrixDeviceFrame> {
    let inputs = serde_json::from_slice::<Vec<nuwax_lio_backends::LioInputFrame>>(
        &fs::read(fixture_path).expect("read matrix LIO fixture"),
    )
    .expect("decode matrix LIO fixture");
    let mut slot = nuwax_lio_backends::LioBackendSlot::new();
    slot.start(nuwax_lio_backends::BackendKind::FastLio2)
        .expect("start matrix fixture estimator");
    let frames = inputs
        .iter()
        .enumerate()
        .map(|(index, input)| {
            let sequence = u32::try_from(index + 1).expect("matrix fixture sequence fits u32");
            let output = slot.process(input).expect("process matrix fixture frame");
            let half_yaw = output.odometry.yaw_rad / 2.0;
            let odometry = schema::Odometry {
                header: matrix_timestamp_header("odom", sequence, output.odometry.timestamp_ns),
                child_frame_id: "base_link".to_owned(),
                pose: schema::Pose {
                    position: output.odometry.position_m,
                    orientation_xyzw: [0.0, 0.0, half_yaw.sin(), half_yaw.cos()],
                },
                twist_linear: output.odometry.linear_velocity_mps,
                ..schema::Odometry::default()
            };
            let mut data = Vec::with_capacity(output.registered_cloud.points.len() * 16);
            for point in &output.registered_cloud.points {
                for value in [point.x, point.y, point.z, point.intensity] {
                    data.extend_from_slice(&(value as f32).to_le_bytes());
                }
            }
            let width = i32::try_from(output.registered_cloud.points.len())
                .expect("matrix device cloud width fits i32");
            let cloud = schema::PointCloud2 {
                header: matrix_timestamp_header(
                    "odom",
                    sequence,
                    output.registered_cloud.timestamp_ns,
                ),
                height: 1,
                width,
                fields: ["x", "y", "z", "intensity"]
                    .into_iter()
                    .enumerate()
                    .map(|(field_index, name)| schema::PointField {
                        name: name.to_owned(),
                        offset: i32::try_from(field_index * 4)
                            .expect("matrix field offset fits i32"),
                        datatype: 7,
                        count: 1,
                    })
                    .collect(),
                is_bigendian: false,
                point_step: 16,
                row_step: width.checked_mul(16).expect("matrix row step fits i32"),
                data,
                is_dense: true,
            };
            MatrixDeviceFrame { odometry, cloud }
        })
        .collect();
    slot.stop();
    frames
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn write_relocalization_recorded_fixture(probe: &ProbeDir) -> PathBuf {
    let unique_points = (0_usize..512)
        .map(|index| {
            let x = index % 32;
            let y = index / 32;
            let hash = (index as u32).wrapping_mul(0x9e37_79b9).rotate_left(13) ^ 0xa5a5_5a5a;
            let x_voxel = x * 4 + (hash as usize & 1);
            let y_voxel = y * 4 + ((hash as usize >> 1) & 1);
            let z_voxel = (hash as usize >> 2) & 31;
            let voxel_size = f64::from(0.05_f32);
            nuwax_lio_backends::TimedPoint {
                offset_ns: u64::try_from(index).expect("point index fits u64") * 50_000,
                x: (x_voxel as f64 + 0.5) * voxel_size,
                y: (y_voxel as f64 + 0.5) * voxel_size,
                z: (z_voxel as f64 + 0.5) * voxel_size,
                intensity: 10.0 + index as f64,
            }
        })
        .collect::<Vec<_>>();
    let points = (0..50_000)
        .map(|index| unique_points[index % unique_points.len()].clone())
        .collect::<Vec<_>>();
    let frames = [1_000_000_000_i64, 5_000_000_000]
        .into_iter()
        .map(|timestamp_ns| nuwax_lio_backends::LioInputFrame {
            cloud: nuwax_lio_backends::TimestampedCloud {
                timestamp_ns,
                frame_id: "mid360_link".to_owned(),
                points: points.clone(),
            },
            imu: vec![
                nuwax_lio_backends::ImuSample {
                    timestamp_ns: timestamp_ns - 100_000_000,
                    accel_mps2: [0.0, 0.0, 0.0],
                    gyro_rps: [0.0, 0.0, 0.0],
                },
                nuwax_lio_backends::ImuSample {
                    timestamp_ns: timestamp_ns - 50_000_000,
                    accel_mps2: [0.0, 0.0, 0.0],
                    gyro_rps: [0.0, 0.0, 0.0],
                },
            ],
        })
        .collect::<Vec<_>>();
    let path = probe.path().join("relocalization-recorded-scan-imu.json");
    fs::write(
        &path,
        serde_json::to_vec(&frames).expect("encode relocalization fixture"),
    )
    .expect("write relocalization fixture");
    path
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_playback(profile: MatrixProfile, probe: &ProbeDir) -> MatrixPlayback {
    let fixture_path = match profile.correction {
        "library_pgo" => write_library_pgo_recorded_fixture(probe),
        "native_pgo" => write_native_pgo_recorded_fixture(probe),
        "relocalization" => write_relocalization_recorded_fixture(probe),
        "none" => Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../nuwax-workspace/crates/nuwax-lio-backends/fixtures/recorded_scan_imu.json")
            .canonicalize()
            .expect("canonicalize matrix LIO fixture"),
        value => panic!("unsupported matrix correction {value}"),
    };
    let source_frames = serde_json::from_slice::<Vec<nuwax_lio_backends::LioInputFrame>>(
        &fs::read(&fixture_path).expect("read matrix source fixture"),
    )
    .expect("decode matrix source fixture");
    let device_frames = matrix_device_frames(&fixture_path);
    MatrixPlayback {
        fixture_path,
        source_frames,
        device_frames,
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
struct RunningDeviceCycle {
    scenario: String,
    child: ChildGuard,
    ready_path: PathBuf,
    generation: u64,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn spawn_device_cycle(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    probe: &ProbeDir,
    profile: MatrixProfile,
) -> RunningDeviceCycle {
    let scenario = profile.scenario();
    let ready_path = probe.path().join("geometry.ready");
    let mut command = runtime_command("real-lcm", probe);
    command
        .env("NUWAX_GEOMETRY_LIO_BACKEND", profile.lio)
        .env("NUWAX_GEOMETRY_MAPPING_BACKEND", profile.mapping)
        .env("NUWAX_GEOMETRY_CORRECTION_BACKEND", profile.correction)
        .env("NUWAX_GEOMETRY_CANONICAL_MAP_WRITER", "geometry")
        .env("NUWAX_GEOMETRY_CANONICAL_CORRECTION_WRITER", "geometry");
    if profile.mapping == "raytracing" {
        command
            .env("NUWAX_MAPPING_RAYTRACING_MIN_HEALTH", "0")
            .env("NUWAX_MAPPING_RAYTRACING_MAX_HEALTH", "1");
    }
    let mut child = ChildGuard::new(
        command
            .spawn()
            .unwrap_or_else(|error| panic!("{scenario}: spawn Geometry runtime: {error}")),
    );
    if profile.correction == "native_pgo" {
        #[cfg(target_os = "linux")]
        child.discover_native_pgo_on_cleanup(
            u32::MAX,
            nuwax_nav_pgo_bridge::native_executable()
                .canonicalize()
                .expect("canonicalize matrix native PGO executable"),
            scenario.clone(),
        );
    }
    if let Err(reason) = wait_for_startup_artifacts(
        child.child_mut(),
        probe,
        &ready_path,
        NATIVE_FIXTURE_TIMEOUT,
    ) {
        let output = child.terminate_and_collect();
        panic!(
            "{scenario}: device profile did not become ready: {reason}; stdout={}, stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let starting = wait_for_status(
        runtime,
        &mut observers.status,
        1,
        lio::LIO_LIFECYCLE_STARTING,
        &scenario,
    );
    let ready = wait_for_status(
        runtime,
        &mut observers.status,
        1,
        lio::LIO_LIFECYCLE_READY,
        &scenario,
    );
    assert_eq!(starting.backend, lio::LIO_BACKEND_DEVICE);
    assert_eq!(ready.backend, lio::LIO_BACKEND_DEVICE);
    assert_eq!(starting.status_sequence, 1);
    assert_eq!(ready.status_sequence, 2);
    RunningDeviceCycle {
        scenario,
        child,
        ready_path,
        generation: 1,
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
enum MatrixCycle {
    Device(RunningDeviceCycle),
    Native(RunningNativeCycle),
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
impl MatrixCycle {
    fn generation(&self) -> u64 {
        match self {
            Self::Device(cycle) => cycle.generation,
            Self::Native(cycle) => cycle.generation,
        }
    }

    #[cfg(target_os = "linux")]
    fn geometry_pid(&mut self) -> u32 {
        match self {
            Self::Device(cycle) => cycle.child.child_mut().id(),
            Self::Native(cycle) => cycle.child.child_mut().id(),
        }
    }

    #[cfg(target_os = "linux")]
    fn track_native_pgo(&mut self, pid: u32, start_time: u64, scenario: &str) {
        match self {
            Self::Device(cycle) => cycle.child.track_process_group(pid, start_time, scenario),
            Self::Native(cycle) => cycle.child.track_process_group(pid, start_time, scenario),
        }
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn spawn_matrix_cycle(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    probe: &ProbeDir,
    profile: MatrixProfile,
    playback: &MatrixPlayback,
    permit_path: &Path,
) -> MatrixCycle {
    if profile.is_native_lio() {
        MatrixCycle::Native(spawn_native_cycle(
            runtime,
            observers,
            probe,
            (profile.lio, profile.mapping, profile.correction),
            &playback.fixture_path,
            permit_path,
        ))
    } else {
        MatrixCycle::Device(spawn_device_cycle(runtime, observers, probe, profile))
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_publish_device_frame(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    runtime_generation: u64,
    frame: &MatrixDeviceFrame,
) -> (RawLcmFrame, RawLcmFrame) {
    let sequence = u32::try_from(frame.odometry.header.seq).expect("device sequence is positive");
    let odometry = typed_binding("/odom", "nav_msgs.Odometry");
    let cloud = typed_binding("/go2/lidar/points", "sensor_msgs.PointCloud2");
    let odometry_frame = nuwax_protocol::RawLcmFrameBuilder::new(odometry.0, odometry.1)
        .seqno(sequence)
        .payload(schema::encode_odometry_body(&frame.odometry).expect("encode device odometry"))
        .build();
    let cloud_frame = nuwax_protocol::RawLcmFrameBuilder::new(cloud.0, cloud.1)
        .seqno(sequence)
        .payload(schema::encode_point_cloud2_body(&frame.cloud).expect("encode device cloud"))
        .build();
    runtime
        .block_on(observers.transport.publish(odometry_frame))
        .expect("publish matrix device source odometry");

    let public_odometry_frame = next_frame(
        runtime,
        &mut observers.public_odometry,
        "matrix accepted device odometry",
    );
    let public_odometry = decode_public_odometry(&public_odometry_frame);
    let timestamp_ns = i64::from(frame.odometry.header.stamp_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(frame.odometry.header.stamp_nsec)))
        .expect("matrix device timestamp fits i64");
    assert_eq!(public_odometry.schema_version, lio::LIO_SCHEMA_VERSION_V1);
    assert_eq!(public_odometry.backend, lio::LIO_BACKEND_DEVICE);
    assert_eq!(
        public_odometry.runtime_generation,
        i64::try_from(runtime_generation).expect("matrix runtime generation fits i64")
    );
    assert_eq!(public_odometry.source_sequence, i64::from(sequence));
    assert_eq!(public_odometry.source_timestamp_ns, timestamp_ns);
    assert_eq!(public_odometry.odom_frame_id, "odom");
    assert_eq!(public_odometry.body_frame_id, "base_link");
    assert_eq!(public_odometry.map_frame_id, "map");
    assert_eq!(public_odometry.odometry.header, frame.odometry.header);
    assert_eq!(
        public_odometry.odometry.child_frame_id,
        frame.odometry.child_frame_id
    );
    assert_eq!(public_odometry.odometry.pose, frame.odometry.pose);

    let canonical_tf_frame = next_frame(
        runtime,
        &mut observers.tf,
        "matrix accepted device odom-to-base TF",
    );
    let canonical_tf = decode_tf(&canonical_tf_frame);
    let odom_to_base = canonical_tf
        .transforms
        .iter()
        .filter(|transform| {
            transform.header.frame_id == "odom" && transform.child_frame_id == "base_link"
        })
        .collect::<Vec<_>>();
    assert_eq!(
        odom_to_base.len(),
        1,
        "matrix canonical TF must contain exactly one odom-to-base transform"
    );
    let odom_to_base = odom_to_base[0];
    assert!(odom_to_base.header.seq > 0);
    assert_eq!(
        odom_to_base.header.stamp_sec,
        frame.odometry.header.stamp_sec
    );
    assert_eq!(
        odom_to_base.header.stamp_nsec,
        frame.odometry.header.stamp_nsec
    );
    assert_eq!(
        odom_to_base.transform.translation,
        frame.odometry.pose.position
    );
    assert_eq!(
        odom_to_base.transform.rotation_xyzw,
        frame.odometry.pose.orientation_xyzw
    );

    assert_stream_silent(
        runtime,
        &mut observers.public_cloud,
        "matrix device cloud before accepted odometry",
    );
    assert_stream_silent(
        runtime,
        &mut observers.global_map,
        "matrix device global map before accepted odometry",
    );
    runtime
        .block_on(observers.transport.publish(cloud_frame))
        .expect("publish matrix device source cloud after odometry acceptance");
    (public_odometry_frame, canonical_tf_frame)
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_prime_mapping(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    probe: &ProbeDir,
    playback: &MatrixPlayback,
    permit_path: &Path,
    cycle: &mut MatrixCycle,
) -> (GeometryClient, native_fixture_proto::MappingSessionState) {
    match cycle {
        MatrixCycle::Native(native) => {
            write_publish_permit(permit_path, 1);
            matrix_wait_native_publication_ack(native, 1);
            wait_cycle_ready(native, probe);
            assert_cycle_starting_and_ready(runtime, observers, native);
            let inactive_odometry = decode_public_odometry(&next_frame(
                runtime,
                &mut observers.public_odometry,
                "matrix bootstrap inactive odometry",
            ));
            assert_stream_silent(
                runtime,
                &mut observers.public_cloud,
                "matrix bootstrap inactive cloud",
            );
            let mut client = connect_geometry(runtime, probe, native.scenario.as_str());
            let started = start_mapping(runtime, &mut client, native.scenario.as_str());
            let replay_tf = next_frame(runtime, &mut observers.tf, "matrix bootstrap replay TF");
            let replay_cloud = decode_public_cloud(&next_frame(
                runtime,
                &mut observers.public_cloud,
                "matrix bootstrap replay cloud",
            ));
            assert_deferred_replay_outputs(
                &native.scenario,
                &inactive_odometry,
                &replay_cloud,
                &replay_tf,
            );
            let _global_map = next_nonempty_global_map(
                runtime,
                &mut observers.global_map,
                "matrix bootstrap replay global map",
            );
            (client, started)
        }
        MatrixCycle::Device(device) => {
            let mut client = connect_geometry(runtime, probe, &device.scenario);
            let started = start_mapping(runtime, &mut client, &device.scenario);
            let activation_global_map = decode_global_map(&next_frame(
                runtime,
                &mut observers.global_map,
                "matrix device mapping activation global map",
            ));
            assert_eq!(activation_global_map.width, 0);
            assert!(activation_global_map.data.is_empty());
            let (_odometry, _tf) = matrix_publish_device_frame(
                runtime,
                observers,
                device.generation,
                &playback.device_frames[0],
            );
            let _cloud = next_frame(
                runtime,
                &mut observers.public_cloud,
                "matrix device bootstrap cloud",
            );
            let _global_map = next_nonempty_global_map(
                runtime,
                &mut observers.global_map,
                "matrix device bootstrap global map",
            );
            (client, started)
        }
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
struct MatrixPublishedFrame {
    odometry: RawLcmFrame,
    tf: RawLcmFrame,
    cloud: RawLcmFrame,
    global_map: Option<RawLcmFrame>,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Debug, PartialEq)]
struct MatrixObservableSnapshot {
    mapping: native_fixture_proto::MappingSessionState,
    occupancy: native_fixture_proto::OccupancyGridData,
    occupancy_mmap: Vec<u8>,
    cloud_map_mmap: Vec<u8>,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn capture_matrix_observable_snapshot(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    probe: &ProbeDir,
    scenario: &str,
) -> MatrixObservableSnapshot {
    MatrixObservableSnapshot {
        mapping: get_mapping_state(runtime, client, scenario),
        occupancy: get_occupancy_data(runtime, client, scenario),
        occupancy_mmap: fs::read(probe.path().join("occupancy.mmap"))
            .expect("read matrix occupancy snapshot"),
        cloud_map_mmap: fs::read(probe.path().join("cloud-map.mmap"))
            .expect("read matrix cloud-map snapshot"),
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn capture_stable_matrix_observable_snapshot(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    probe: &ProbeDir,
    boundary: &str,
    scenario: &str,
) -> MatrixObservableSnapshot {
    let first = capture_matrix_observable_snapshot(runtime, client, probe, scenario);
    let stable = capture_matrix_observable_snapshot(runtime, client, probe, scenario);
    assert_matrix_observable_snapshots_equal(
        &first,
        &stable,
        &format!("{boundary} consecutive stability"),
        scenario,
    );
    stable
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_read_u32_le(bytes: &[u8], offset: usize, field: &str) -> u32 {
    u32::from_le_bytes(
        bytes
            .get(offset..offset + 4)
            .unwrap_or_else(|| panic!("matrix {field} is truncated"))
            .try_into()
            .expect("matrix u32 field has fixed width"),
    )
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_read_u64_le(bytes: &[u8], offset: usize, field: &str) -> u64 {
    u64::from_le_bytes(
        bytes
            .get(offset..offset + 8)
            .unwrap_or_else(|| panic!("matrix {field} is truncated"))
            .try_into()
            .expect("matrix u64 field has fixed width"),
    )
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_write_u32_le(bytes: &mut [u8], offset: usize, value: u32, field: &str) {
    bytes
        .get_mut(offset..offset + 4)
        .unwrap_or_else(|| panic!("matrix {field} is truncated"))
        .copy_from_slice(&value.to_le_bytes());
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_write_u64_le(bytes: &mut [u8], offset: usize, value: u64, field: &str) {
    bytes
        .get_mut(offset..offset + 8)
        .unwrap_or_else(|| panic!("matrix {field} is truncated"))
        .copy_from_slice(&value.to_le_bytes());
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_matrix_bytes_eq(actual: &[u8], expected: &[u8], label: &str, scenario: &str) {
    let first_mismatch = actual
        .iter()
        .zip(expected)
        .position(|(actual, expected)| actual != expected);
    assert!(
        actual.len() == expected.len() && first_mismatch.is_none(),
        "{scenario}: {label} bytes drifted: actual_len={} expected_len={} first_mismatch={first_mismatch:?}",
        actual.len(),
        expected.len()
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_matrix_observable_snapshots_equal(
    expected: &MatrixObservableSnapshot,
    actual: &MatrixObservableSnapshot,
    boundary: &str,
    scenario: &str,
) {
    assert_eq!(
        actual.mapping, expected.mapping,
        "{scenario}: {boundary} changed MappingSessionState"
    );
    assert_eq!(
        actual.occupancy, expected.occupancy,
        "{scenario}: {boundary} changed OccupancyGridData"
    );
    assert_matrix_bytes_eq(
        &actual.occupancy_mmap,
        &expected.occupancy_mmap,
        &format!("{boundary} occupancy.mmap"),
        scenario,
    );
    assert_matrix_bytes_eq(
        &actual.cloud_map_mmap,
        &expected.cloud_map_mmap,
        &format!("{boundary} cloud-map.mmap"),
        scenario,
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_matrix_occupancy_mmap_frame_transition(
    before: &[u8],
    after: &[u8],
    occupancy: &native_fixture_proto::OccupancyGridData,
    scenario: &str,
) {
    const GLOBAL_HEADER_BYTES: usize = 128;
    const SLOT_HEADER_BYTES: usize = 256;
    const OCCUPANCY_MAGIC: u64 = 0x4E55_5741_584F_4343;
    const SLOT_MAGIC: u64 = 0x4E55_5741_5853_4C54;

    assert_eq!(
        matrix_read_u64_le(before, 0, "occupancy magic"),
        OCCUPANCY_MAGIC
    );
    assert_eq!(matrix_read_u32_le(before, 8, "occupancy layout version"), 2);
    assert_eq!(
        matrix_read_u32_le(before, 12, "occupancy global header size"),
        GLOBAL_HEADER_BYTES as u32
    );
    let slot_capacity = usize::try_from(matrix_read_u64_le(before, 16, "occupancy slot capacity"))
        .expect("matrix occupancy slot capacity fits usize");
    assert_eq!(
        before.len(),
        GLOBAL_HEADER_BYTES + 2 * slot_capacity,
        "{scenario}: unexpected occupancy SHM size"
    );
    assert_eq!(
        after.len(),
        before.len(),
        "{scenario}: occupancy SHM resized"
    );

    let before_sequence = matrix_read_u64_le(before, 24, "occupancy write sequence");
    let before_published = matrix_read_u64_le(before, 32, "occupancy published state");
    let before_generation = before_published >> 1;
    let before_slot = usize::try_from(before_published & 1).expect("occupancy slot fits usize");
    assert!(
        before_generation > 0,
        "{scenario}: occupancy SHM is unpublished"
    );
    assert_eq!(before_sequence, before_generation * 2, "{scenario}");

    let generation = before_generation;
    let active_slot = before_slot;
    let meta = occupancy
        .meta
        .as_ref()
        .expect("matrix committed occupancy metadata");
    let payload_checksum = nuwax_codec::sha256_bytes(&occupancy.data);
    assert!(
        meta.frame_id.len() <= 128,
        "{scenario}: occupancy frame id is too long"
    );
    assert!(
        SLOT_HEADER_BYTES + occupancy.data.len() <= slot_capacity,
        "{scenario}: occupancy fixture unexpectedly grew the SHM layout"
    );

    let slot_offset = GLOBAL_HEADER_BYTES + active_slot * slot_capacity;
    let payload_offset = slot_offset + SLOT_HEADER_BYTES;
    let mut expected = before.to_vec();
    expected[slot_offset..slot_offset + SLOT_HEADER_BYTES].fill(0);
    matrix_write_u64_le(
        &mut expected,
        slot_offset,
        SLOT_MAGIC,
        "occupancy slot magic",
    );
    matrix_write_u32_le(
        &mut expected,
        slot_offset + 8,
        2,
        "occupancy slot layout version",
    );
    matrix_write_u32_le(
        &mut expected,
        slot_offset + 12,
        1,
        "occupancy payload schema version",
    );
    matrix_write_u32_le(
        &mut expected,
        slot_offset + 16,
        SLOT_HEADER_BYTES as u32,
        "occupancy slot header size",
    );
    matrix_write_u32_le(
        &mut expected,
        slot_offset + 20,
        u32::try_from(meta.frame_id.len()).expect("occupancy frame id length fits u32"),
        "occupancy frame id length",
    );
    matrix_write_u64_le(
        &mut expected,
        slot_offset + 24,
        generation,
        "occupancy generation",
    );
    matrix_write_u64_le(
        &mut expected,
        slot_offset + 32,
        meta.revision,
        "occupancy revision",
    );
    matrix_write_u64_le(
        &mut expected,
        slot_offset + 40,
        meta.timestamp_ns,
        "occupancy timestamp",
    );
    matrix_write_u64_le(
        &mut expected,
        slot_offset + 48,
        u64::try_from(occupancy.data.len()).expect("occupancy payload length fits u64"),
        "occupancy payload length",
    );
    matrix_write_u32_le(
        &mut expected,
        slot_offset + 56,
        meta.width,
        "occupancy width",
    );
    matrix_write_u32_le(
        &mut expected,
        slot_offset + 60,
        meta.height,
        "occupancy height",
    );
    matrix_write_u32_le(
        &mut expected,
        slot_offset + 64,
        meta.resolution.to_bits(),
        "occupancy resolution",
    );
    matrix_write_u64_le(
        &mut expected,
        slot_offset + 72,
        meta.origin_x.to_bits(),
        "occupancy origin x",
    );
    matrix_write_u64_le(
        &mut expected,
        slot_offset + 80,
        meta.origin_y.to_bits(),
        "occupancy origin y",
    );
    expected[slot_offset + 88..slot_offset + 120].copy_from_slice(&payload_checksum);
    expected[slot_offset + 120..slot_offset + 120 + meta.frame_id.len()]
        .copy_from_slice(meta.frame_id.as_bytes());
    expected[payload_offset..payload_offset + occupancy.data.len()]
        .copy_from_slice(&occupancy.data);
    matrix_write_u64_le(
        &mut expected,
        24,
        generation * 2,
        "occupancy write sequence",
    );
    matrix_write_u64_le(
        &mut expected,
        32,
        (generation << 1) | u64::try_from(active_slot).expect("occupancy slot fits u64"),
        "occupancy published state",
    );
    assert_matrix_bytes_eq(after, &expected, "occupancy SHM frame transition", scenario);
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_matrix_cloud_mmap_frame_transition(
    before: &[u8],
    after: &[u8],
    candidate: &DecodedCorrectionCandidate,
    global_map_frame: &RawLcmFrame,
    scenario: &str,
) {
    const HEADER_SIZE: usize = 64;
    const SLOT_SIZE: usize = 16 * 1024 * 1024;
    const CLOUD_MAP_MAGIC: u64 = 0x4E55_5741_584D_4150;

    assert_eq!(before.len(), HEADER_SIZE + 2 * SLOT_SIZE, "{scenario}");
    assert_eq!(
        after.len(),
        before.len(),
        "{scenario}: cloud-map SHM resized"
    );
    assert_eq!(
        matrix_read_u64_le(before, 0, "cloud-map magic"),
        CLOUD_MAP_MAGIC
    );
    assert_eq!(matrix_read_u32_le(before, 8, "cloud-map version"), 2);
    let before_revision = matrix_read_u64_le(before, 28, "cloud-map revision");
    assert_eq!(
        candidate.cloud_revision, before_revision,
        "{scenario}: candidate changed the activation cloud-map revision"
    );
    let active_slot = usize::try_from((candidate.cloud_revision - 1) % 2)
        .expect("matrix cloud-map active slot fits usize");
    let global_map = decode_global_map(global_map_frame);
    assert_eq!(
        global_map.header.frame_id, candidate.map_frame_id,
        "{scenario}"
    );
    assert_eq!(global_map.height, 1, "{scenario}");
    assert_eq!(global_map.point_step, 12, "{scenario}");
    let point_count = u32::try_from(global_map.width)
        .unwrap_or_else(|_| panic!("{scenario}: global-map width is negative"));
    assert_eq!(
        global_map.data.len(),
        usize::try_from(point_count).expect("global-map width fits usize") * 12,
        "{scenario}: global-map payload length drifted"
    );
    assert_eq!(
        u64::try_from(global_map.header.seq).expect("global-map revision is positive"),
        candidate.cloud_revision,
        "{scenario}: published global map is not the candidate boundary"
    );
    let published_timestamp_ns = i64::from(global_map.header.stamp_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(global_map.header.stamp_nsec)))
        .and_then(|timestamp| u64::try_from(timestamp).ok())
        .expect("matrix global-map timestamp is nonnegative");
    assert_eq!(
        published_timestamp_ns, candidate.committed_map_source_timestamp_ns,
        "{scenario}: global-map timestamp is not candidate-bound"
    );

    let before_wall_timestamp = matrix_read_u64_le(before, 20, "cloud-map wall timestamp");
    let after_wall_timestamp = matrix_read_u64_le(after, 20, "cloud-map wall timestamp");
    assert_eq!(
        after_wall_timestamp, before_wall_timestamp,
        "{scenario}: APPLIED changed the cloud-map publication timestamp"
    );
    assert!(
        after_wall_timestamp > 0,
        "{scenario}: cloud-map is unpublished"
    );
    assert_eq!(
        &after[36..40],
        &before[36..40],
        "{scenario}: active frame changed cloud-map voxel size"
    );

    let (slot_revision_offset, slot_count_offset) =
        if active_slot == 0 { (40, 48) } else { (52, 60) };
    let payload_offset = HEADER_SIZE + active_slot * SLOT_SIZE;
    let mut expected = before.to_vec();
    expected[payload_offset..payload_offset + global_map.data.len()]
        .copy_from_slice(&global_map.data);
    matrix_write_u64_le(
        &mut expected,
        slot_revision_offset,
        candidate.cloud_revision,
        "cloud-map slot revision",
    );
    matrix_write_u32_le(
        &mut expected,
        slot_count_offset,
        point_count,
        "cloud-map slot point count",
    );
    matrix_write_u32_le(
        &mut expected,
        12,
        u32::try_from(active_slot).expect("cloud-map slot fits u32"),
        "cloud-map active slot",
    );
    matrix_write_u32_le(&mut expected, 16, point_count, "cloud-map point count");
    matrix_write_u64_le(
        &mut expected,
        20,
        after_wall_timestamp,
        "cloud-map wall timestamp",
    );
    matrix_write_u64_le(
        &mut expected,
        28,
        candidate.cloud_revision,
        "cloud-map revision",
    );
    assert_matrix_bytes_eq(after, &expected, "cloud-map SHM frame transition", scenario);
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_matrix_relocalization_observable_transition(
    before: &MatrixObservableSnapshot,
    after: &MatrixObservableSnapshot,
    candidate: &DecodedCorrectionCandidate,
    global_map_frame: &RawLcmFrame,
    scenario: &str,
) {
    assert_matrix_observable_snapshots_equal(before, after, "APPLIED", scenario);
    let before_meta = before
        .occupancy
        .meta
        .as_ref()
        .expect("matrix activation occupancy metadata");
    let after_meta = after
        .occupancy
        .meta
        .as_ref()
        .expect("matrix active-frame occupancy metadata");
    assert_eq!(before.mapping.revision, before_meta.revision, "{scenario}");
    assert_eq!(
        candidate.occupancy_revision, before.mapping.revision,
        "{scenario}: candidate changed the activation mapping revision"
    );
    let mut expected_mapping = before.mapping.clone();
    expected_mapping.revision = candidate.occupancy_revision;
    assert_eq!(
        after.mapping, expected_mapping,
        "{scenario}: APPLIED changed mapping state beyond the candidate-bound active frame"
    );
    assert_eq!(
        after_meta.revision, candidate.occupancy_revision,
        "{scenario}"
    );
    assert_eq!(
        after_meta.timestamp_ns, candidate.committed_map_source_timestamp_ns,
        "{scenario}"
    );
    assert_eq!(
        u64::from(after_meta.width),
        candidate.committed_map_width,
        "{scenario}"
    );
    assert_eq!(
        u64::from(after_meta.height),
        candidate.committed_map_height,
        "{scenario}"
    );
    assert_eq!(
        f64::from(after_meta.resolution).to_bits(),
        candidate.committed_map_resolution_m.to_bits(),
        "{scenario}"
    );
    assert_eq!(
        after_meta.origin_x.to_bits(),
        candidate.committed_map_origin_x_m.to_bits(),
        "{scenario}"
    );
    assert_eq!(
        after_meta.origin_y.to_bits(),
        candidate.committed_map_origin_y_m.to_bits(),
        "{scenario}"
    );
    assert_eq!(after_meta.frame_id, candidate.map_frame_id, "{scenario}");
    assert_eq!(
        nuwax_codec::sha256_bytes(&after.occupancy.data),
        candidate.committed_map_payload_sha256,
        "{scenario}: occupancy payload is not the candidate-bound committed map"
    );
    assert_matrix_occupancy_mmap_frame_transition(
        &before.occupancy_mmap,
        &after.occupancy_mmap,
        &after.occupancy,
        scenario,
    );
    assert_matrix_cloud_mmap_frame_transition(
        &before.cloud_map_mmap,
        &after.cloud_map_mmap,
        candidate,
        global_map_frame,
        scenario,
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_next_global_map_revision(
    runtime: &tokio::runtime::Runtime,
    stream: &mut LcmFrameStream,
    expected_revision: u64,
    scenario: &str,
) -> RawLcmFrame {
    loop {
        let frame = next_frame(runtime, stream, "matrix candidate-bound global map");
        let global_map = decode_global_map(&frame);
        assert_eq!(global_map.header.frame_id, "map", "{scenario}");
        let revision = u64::try_from(global_map.header.seq)
            .unwrap_or_else(|_| panic!("{scenario}: global-map revision is negative"));
        assert!(
            revision <= expected_revision,
            "{scenario}: global-map revision skipped the candidate boundary: expected={expected_revision} actual={revision}"
        );
        if revision == expected_revision {
            return frame;
        }
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_matrix_activation_global_map(
    checkpoint: &native_fixture_proto::MappingBackendCheckpoint,
    installed_revision: u64,
    frame: &RawLcmFrame,
    scenario: &str,
) {
    let cloud = checkpoint
        .canonical_cloud_checkpoint
        .as_ref()
        .expect("matrix activation cloud checkpoint");
    let receipt = checkpoint
        .cloud_receipt
        .as_ref()
        .expect("matrix activation cloud receipt");
    assert_eq!(frame.topic.channel(), "/global_map", "{scenario}");
    assert_eq!(
        frame.msg_type.fqn(),
        "sensor_msgs.PointCloud2",
        "{scenario}"
    );
    assert_eq!(
        receipt.session_generation, checkpoint.source_session_generation,
        "{scenario}: activation cloud receipt changed session generation"
    );
    assert_eq!(
        receipt.cloud_revision, cloud.source_revision,
        "{scenario}: activation cloud receipt changed revision"
    );
    assert_eq!(
        receipt.point_count, cloud.point_count,
        "{scenario}: activation cloud receipt changed point count"
    );

    let global_map = decode_global_map(frame);
    assert_eq!(global_map.header.frame_id, cloud.frame_id, "{scenario}");
    assert_eq!(
        u64::try_from(global_map.header.seq)
            .unwrap_or_else(|_| panic!("{scenario}: activation map revision is negative")),
        installed_revision,
        "{scenario}: activation map revision is not installed-canonical"
    );
    let timestamp_ns = i64::from(global_map.header.stamp_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(global_map.header.stamp_nsec)))
        .and_then(|timestamp| u64::try_from(timestamp).ok())
        .expect("matrix activation global-map timestamp is nonnegative");
    assert_eq!(
        timestamp_ns, receipt.source_timestamp_ns,
        "{scenario}: activation map timestamp is not receipt-bound"
    );
    assert_eq!(global_map.height, 1, "{scenario}");
    assert_eq!(
        global_map.width,
        i32::try_from(cloud.point_count).expect("matrix activation point count fits i32"),
        "{scenario}: activation map width changed"
    );
    assert_eq!(global_map.fields.len(), 3, "{scenario}");
    for (field, name, offset) in global_map
        .fields
        .iter()
        .zip(["x", "y", "z"])
        .zip([0, 4, 8])
        .map(|((field, name), offset)| (field, name, offset))
    {
        assert_eq!(field.name, name, "{scenario}");
        assert_eq!(field.offset, offset, "{scenario}");
        assert_eq!(field.datatype, 7, "{scenario}");
        assert_eq!(field.count, 1, "{scenario}");
    }
    assert!(!global_map.is_bigendian, "{scenario}");
    assert_eq!(global_map.point_step, 12, "{scenario}");
    assert_eq!(
        global_map.row_step,
        i32::try_from(cloud.xyz_f32_le.len()).expect("matrix activation row size fits i32"),
        "{scenario}: activation map row size changed"
    );
    assert_eq!(global_map.data, cloud.xyz_f32_le, "{scenario}");
    assert_eq!(
        nuwax_codec::sha256_bytes(&global_map.data),
        matrix_hex_digest(&cloud.xyz_sha256, "activation cloud xyz_sha256"),
        "{scenario}: activation map payload digest changed"
    );
    assert!(global_map.is_dense, "{scenario}");
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_publish_active_frame(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    profile: MatrixProfile,
    playback: &MatrixPlayback,
    permit_path: &Path,
    cycle: &mut MatrixCycle,
    sequence: u32,
) -> MatrixPublishedFrame {
    if profile.mapping == "raytracing" || profile.correction != "none" {
        std::thread::sleep(RAYTRACING_CANDIDATE_CADENCE_WAIT);
    }
    let accepted_device_outputs = match cycle {
        MatrixCycle::Native(native) => {
            write_publish_permit(permit_path, sequence);
            matrix_wait_native_publication_ack(native, sequence);
            None
        }
        MatrixCycle::Device(device) => {
            let index = usize::try_from(sequence - 1).expect("matrix sequence fits usize");
            Some(matrix_publish_device_frame(
                runtime,
                observers,
                device.generation,
                playback
                    .device_frames
                    .get(index)
                    .unwrap_or_else(|| panic!("matrix fixture omitted source sequence {sequence}")),
            ))
        }
    };
    let (odometry, tf) = accepted_device_outputs.unwrap_or_else(|| {
        (
            next_frame(
                runtime,
                &mut observers.public_odometry,
                "matrix public odometry",
            ),
            next_frame(runtime, &mut observers.tf, "matrix canonical TF"),
        )
    });
    let cloud = next_frame(
        runtime,
        &mut observers.public_cloud,
        "matrix registered cloud",
    );
    let global_map = if profile.correction == "relocalization" {
        None
    } else {
        Some(next_nonempty_global_map(
            runtime,
            &mut observers.global_map,
            "matrix canonical global map",
        ))
    };
    MatrixPublishedFrame {
        odometry,
        tf,
        cloud,
        global_map,
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn finish_device_cycle(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    mut client: GeometryClient,
    mut cycle: RunningDeviceCycle,
) -> Output {
    stop_geometry(runtime, &mut client, &cycle.scenario);
    let stopping = wait_for_status(
        runtime,
        &mut observers.status,
        cycle.generation,
        lio::LIO_LIFECYCLE_STOPPING,
        &cycle.scenario,
    );
    let stopped = wait_for_status(
        runtime,
        &mut observers.status,
        cycle.generation,
        lio::LIO_LIFECYCLE_STOPPED,
        &cycle.scenario,
    );
    assert_eq!(stopping.backend, lio::LIO_BACKEND_DEVICE);
    assert_eq!(stopped.backend, lio::LIO_BACKEND_DEVICE);
    assert!(stopped.status_sequence > stopping.status_sequence);
    if wait_until(cycle.child.child_mut(), NATIVE_FIXTURE_TIMEOUT).is_none() {
        let output = cycle.child.terminate_and_collect();
        panic!(
            "{}: Geometry did not exit after Stop; stdout={}, stderr={}",
            cycle.scenario,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    #[cfg(target_os = "linux")]
    let process_group_outcomes = cycle
        .child
        .settle_tracked_process_groups(NATIVE_FIXTURE_TIMEOUT)
        .unwrap_or_else(|error| panic!("{}: native PGO cleanup failed: {error}", cycle.scenario));
    #[cfg(target_os = "linux")]
    assert!(
        process_group_outcomes.iter().all(|outcome| {
            outcome.disposition == ProcessGroupCleanupDisposition::ExitedWithoutHarnessSignal
        }),
        "{}: normal Stop required harness native PGO cleanup: {process_group_outcomes:?}",
        cycle.scenario
    );
    let output = cycle.child.collect();
    assert_eq!(
        output.status.code(),
        Some(0),
        "{}: Geometry exit was not clean; stdout={}, stderr={}",
        cycle.scenario,
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        !cycle.ready_path.exists(),
        "{}: Geometry retained its ready file after Stop",
        cycle.scenario
    );
    output
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn assert_matrix_shutdown_stdout(output: &Output, runtime_generation: u64, scenario: &str) {
    let stdout = std::str::from_utf8(&output.stdout)
        .unwrap_or_else(|error| panic!("{scenario}: Geometry stdout is not UTF-8: {error}"));
    let expected = [
        (
            "nuwax.geometry.runtime.shutdown.correction_settled.v1",
            "correction_settled",
            1_u64,
        ),
        (
            "nuwax.geometry.runtime.shutdown.mapping_retired.v1",
            "mapping_retired",
            2_u64,
        ),
        (
            "nuwax.geometry.runtime.shutdown.lio_joined.v1",
            "lio_joined",
            3_u64,
        ),
        (
            "nuwax.geometry.runtime.shutdown.resource_cleanup_complete.v1",
            "resource_cleanup_complete",
            4_u64,
        ),
    ];
    let event_lines = stdout
        .lines()
        .enumerate()
        .filter(|(_, line)| line.contains("geometry runtime shutdown stage completed"))
        .collect::<Vec<_>>();
    assert_eq!(
        event_lines.len(),
        expected.len(),
        "{scenario}: shutdown event count drifted; stdout={stdout}"
    );
    for ((_, line), (event_name, stage, ordinal)) in event_lines.iter().zip(expected) {
        let tokens = line.split_whitespace().collect::<Vec<_>>();
        for token in [
            format!("event_name=\"{event_name}\""),
            "event_schema_version=1".to_owned(),
            format!("runtime_generation={runtime_generation}"),
            format!("stage_ordinal={ordinal}"),
            format!("stage=\"{stage}\""),
            "status=\"completed\"".to_owned(),
        ] {
            assert_eq!(
                tokens
                    .iter()
                    .filter(|value| **value == token.as_str())
                    .count(),
                1,
                "{scenario}: shutdown event token {token:?} was not exact once; line={line}"
            );
        }
    }
    for (event_name, _, _) in expected {
        assert_eq!(
            stdout.matches(event_name).count(),
            1,
            "{scenario}: shutdown event {event_name} was not emitted exactly once"
        );
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn finish_matrix_cycle(
    runtime: &tokio::runtime::Runtime,
    observers: &mut NativeObservers,
    client: GeometryClient,
    mut cycle: MatrixCycle,
    probe: &ProbeDir,
) -> Output {
    let runtime_generation = cycle.generation();
    let scenario = match &cycle {
        MatrixCycle::Device(cycle) => cycle.scenario.clone(),
        MatrixCycle::Native(cycle) => cycle.scenario.clone(),
    };
    let geometry_pid = cycle.geometry_pid();
    let mut process_identities = vec![(
        geometry_pid,
        process_start_time(geometry_pid)
            .unwrap_or_else(|| panic!("{scenario}: Geometry disappeared before Stop")),
    )];
    for pid in direct_child_pids(geometry_pid) {
        let start_time = process_start_time(pid).unwrap_or_else(|| {
            panic!("{scenario}: direct Geometry child {pid} disappeared before Stop")
        });
        process_identities.push((pid, start_time));
    }
    let output = match cycle {
        MatrixCycle::Device(cycle) => finish_device_cycle(runtime, observers, client, cycle),
        MatrixCycle::Native(cycle) => finish_native_cycle(runtime, observers, client, cycle),
    };
    assert_matrix_shutdown_stdout(&output, runtime_generation, &scenario);
    for (pid, start_time) in process_identities {
        assert_ne!(
            process_start_time(pid),
            Some(start_time),
            "{scenario}: process identity pid={pid} start_time={start_time} survived Stop"
        );
    }
    for artifact in [
        "geometry.ready",
        "video.shm",
        "video.sock",
        "occupancy.mmap",
        "cloud-map.mmap",
    ] {
        let path = probe.path().join(artifact);
        assert!(
            !path.exists(),
            "{scenario}: shutdown retained {}",
            path.display()
        );
    }
    output
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_push_string(out: &mut Vec<u8>, value: &str) {
    out.extend_from_slice(
        &u32::try_from(value.len())
            .expect("matrix canonical string length fits u32")
            .to_be_bytes(),
    );
    out.extend_from_slice(value.as_bytes());
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_push_optional_digest(out: &mut Vec<u8>, digest: Option<[u8; 32]>) {
    match digest {
        Some(digest) => {
            out.push(1);
            out.extend_from_slice(&digest);
        }
        None => out.push(0),
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_digest32(value: &[u8], label: &str) -> [u8; 32] {
    value
        .try_into()
        .unwrap_or_else(|_| panic!("{label} must contain exactly 32 bytes"))
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_hex_digest(value: &str, label: &str) -> [u8; 32] {
    assert_eq!(
        value.len(),
        64,
        "{label} must contain 64 lowercase hex bytes"
    );
    let mut digest = [0_u8; 32];
    for (index, pair) in value.as_bytes().chunks_exact(2).enumerate() {
        assert!(
            pair.iter()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(byte)),
            "{label} is not canonical lowercase hex"
        );
        let encoded = std::str::from_utf8(pair).expect("hex pair is UTF-8");
        digest[index] = u8::from_str_radix(encoded, 16).expect("decode canonical hex pair");
    }
    digest
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_occupancy_sha256(checkpoint: &native_fixture_proto::OccupancyGridData) -> [u8; 32] {
    let meta = checkpoint.meta.as_ref().expect("matrix occupancy metadata");
    let mut out = b"nuwax.geometry.occupancy-checkpoint.v1\0".to_vec();
    out.extend_from_slice(&1_u32.to_be_bytes());
    out.extend_from_slice(&meta.width.to_be_bytes());
    out.extend_from_slice(&meta.height.to_be_bytes());
    out.extend_from_slice(&meta.resolution.to_be_bytes());
    out.extend_from_slice(
        &(if meta.origin_x == 0.0 {
            0.0
        } else {
            meta.origin_x
        })
        .to_be_bytes(),
    );
    out.extend_from_slice(
        &(if meta.origin_y == 0.0 {
            0.0
        } else {
            meta.origin_y
        })
        .to_be_bytes(),
    );
    matrix_push_string(&mut out, &meta.frame_id);
    out.extend_from_slice(&meta.revision.to_be_bytes());
    out.extend_from_slice(&meta.timestamp_ns.to_be_bytes());
    out.extend_from_slice(
        &u64::try_from(checkpoint.data.len())
            .expect("matrix occupancy data length fits u64")
            .to_be_bytes(),
    );
    out.extend_from_slice(&nuwax_codec::sha256_bytes(&checkpoint.data));
    nuwax_codec::sha256_bytes(&out)
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_cloud_sha256(checkpoint: &native_fixture_proto::CloudMapCheckpoint) -> [u8; 32] {
    let mut out = b"nuwax.geometry.cloud-checkpoint.v1\0".to_vec();
    matrix_push_string(&mut out, &checkpoint.frame_id);
    out.extend_from_slice(&checkpoint.source_revision.to_be_bytes());
    out.extend_from_slice(&checkpoint.point_count.to_be_bytes());
    out.extend_from_slice(&checkpoint.voxel_size.to_be_bytes());
    out.extend_from_slice(
        &u64::try_from(checkpoint.xyz_f32_le.len())
            .expect("matrix cloud data length fits u64")
            .to_be_bytes(),
    );
    out.extend_from_slice(&matrix_hex_digest(
        &checkpoint.xyz_sha256,
        "cloud checkpoint xyz_sha256",
    ));
    out.extend_from_slice(&checkpoint.payload_schema_version.to_be_bytes());
    nuwax_codec::sha256_bytes(&out)
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_append_commit_identity(
    out: &mut Vec<u8>,
    identity: Option<&native_fixture_proto::MappingCommitIdentity>,
) {
    let identity = identity.expect("matrix mapping commit identity");
    assert_eq!(identity.boot_nonce.len(), 16);
    out.extend_from_slice(&identity.boot_nonce);
    out.extend_from_slice(&identity.sequence.to_be_bytes());
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_backend_sha256(checkpoint: &native_fixture_proto::MappingBackendCheckpoint) -> [u8; 32] {
    let occupancy_receipt = checkpoint
        .occupancy_receipt
        .as_ref()
        .expect("matrix checkpoint occupancy receipt");
    let cloud_receipt = checkpoint
        .cloud_receipt
        .as_ref()
        .expect("matrix checkpoint cloud receipt");
    let occupancy = checkpoint
        .canonical_occupancy_checkpoint
        .as_ref()
        .expect("matrix canonical occupancy checkpoint");
    let cloud = checkpoint
        .canonical_cloud_checkpoint
        .as_ref()
        .expect("matrix canonical cloud checkpoint");
    let mut out = b"nuwax.geometry.backend-checkpoint.v1\0".to_vec();
    out.extend_from_slice(&checkpoint.schema_version.to_be_bytes());
    matrix_push_string(&mut out, &checkpoint.backend_kind);
    out.extend_from_slice(&checkpoint.source_session_generation.to_be_bytes());
    out.extend_from_slice(&matrix_digest32(
        &checkpoint.content_sha256,
        "backend content_sha256",
    ));
    out.extend_from_slice(&checkpoint.payload_size.to_be_bytes());
    out.extend_from_slice(&checkpoint.created_at_ns.to_be_bytes());
    out.extend_from_slice(&nuwax_codec::sha256_bytes(&checkpoint.payload));
    matrix_push_string(&mut out, &checkpoint.source_session_id);
    matrix_append_commit_identity(&mut out, occupancy_receipt.commit_id.as_ref());
    matrix_push_string(&mut out, &occupancy_receipt.session_id);
    out.extend_from_slice(&occupancy_receipt.session_generation.to_be_bytes());
    out.extend_from_slice(&occupancy_receipt.occupancy_revision.to_be_bytes());
    out.extend_from_slice(&occupancy_receipt.source_timestamp_ns.to_be_bytes());
    matrix_push_string(&mut out, &occupancy_receipt.backend_kind);
    out.extend_from_slice(&occupancy_receipt.backend_generation.to_be_bytes());
    matrix_append_commit_identity(&mut out, cloud_receipt.commit_id.as_ref());
    out.extend_from_slice(&cloud_receipt.session_generation.to_be_bytes());
    out.extend_from_slice(&cloud_receipt.occupancy_revision.to_be_bytes());
    out.extend_from_slice(&cloud_receipt.cloud_revision.to_be_bytes());
    out.extend_from_slice(&cloud_receipt.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&cloud_receipt.point_count.to_be_bytes());
    out.extend_from_slice(&matrix_occupancy_sha256(occupancy));
    out.extend_from_slice(&matrix_cloud_sha256(cloud));
    nuwax_codec::sha256_bytes(&out)
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_asset_bundle_sha256(
    manifest: [u8; 32],
    occupancy: [u8; 32],
    cloud: [u8; 32],
    backend: [u8; 32],
) -> [u8; 32] {
    let mut out = b"nuwax.geometry.asset-bundle.v1\0".to_vec();
    out.extend_from_slice(&1_u32.to_be_bytes());
    out.extend_from_slice(&manifest);
    out.extend_from_slice(&occupancy);
    matrix_push_optional_digest(&mut out, Some(cloud));
    matrix_push_optional_digest(&mut out, Some(backend));
    nuwax_codec::sha256_bytes(&out)
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn export_matrix_checkpoint(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    scenario: &str,
) -> native_fixture_proto::MappingBackendCheckpoint {
    runtime.block_on(async {
        tokio::time::timeout(
            NATIVE_FIXTURE_TIMEOUT,
            client.export_mapping_backend_checkpoint(native_fixture_proto::Empty {}),
        )
        .await
        .unwrap_or_else(|_| panic!("{scenario}: ExportMappingBackendCheckpoint timed out"))
        .unwrap_or_else(|error| {
            panic!("{scenario}: ExportMappingBackendCheckpoint failed: {error}")
        })
        .into_inner()
    })
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn activate_matrix_checkpoint(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    profile: MatrixProfile,
    runtime_generation: u64,
    checkpoint: &native_fixture_proto::MappingBackendCheckpoint,
) -> native_fixture_proto::MappingSessionState {
    let scenario = profile.scenario();
    let occupancy = checkpoint
        .canonical_occupancy_checkpoint
        .clone()
        .expect("matrix activation occupancy checkpoint");
    let cloud = checkpoint
        .canonical_cloud_checkpoint
        .clone()
        .expect("matrix activation cloud checkpoint");
    let manifest = nuwax_codec::sha256_bytes(
        format!("nuwax.matrix.asset-manifest.v1\0{}", profile.row_id()).as_bytes(),
    );
    let bundle = matrix_asset_bundle_sha256(
        manifest,
        matrix_occupancy_sha256(&occupancy),
        matrix_cloud_sha256(&cloud),
        matrix_backend_sha256(checkpoint),
    );
    let request = native_fixture_proto::ActivateMapRequest {
        session_id: format!("{scenario}-activated"),
        map_name: format!("{scenario}-map"),
        checkpoint: Some(occupancy),
        cloud_checkpoint: Some(cloud),
        mapping_backend_checkpoint: Some(checkpoint.clone()),
        activation_request_id: format!("{scenario}-activate"),
        asset_manifest_sha256: manifest.to_vec(),
        asset_bundle_sha256: bundle.to_vec(),
        expected_runtime_generation: runtime_generation,
        activation_contract_version: 1,
    };
    let activated = runtime.block_on(async {
        tokio::time::timeout(NATIVE_FIXTURE_TIMEOUT, client.activate_map(request))
            .await
            .unwrap_or_else(|_| panic!("{scenario}: ActivateMap timed out"))
            .unwrap_or_else(|error| panic!("{scenario}: ActivateMap failed: {error}"))
            .into_inner()
    });
    assert!(activated.success, "{scenario}: {activated:?}");
    assert!(activated.active, "{scenario}: activated map is not active");
    assert_eq!(
        activated.activation_receipt_disposition,
        native_fixture_proto::ActivationReceiptDisposition::Sealed as i32,
        "{scenario}: activation did not seal"
    );
    assert!(activated.relocalization_target_available);
    assert_eq!(activated.runtime_generation, runtime_generation);
    assert_eq!(activated.mapping_backend_kind, profile.mapping);
    assert_eq!(activated.asset_manifest_sha256, manifest);
    assert_eq!(activated.asset_bundle_sha256, bundle);
    activated
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_policy_digest(value: &str, label: &str) -> Vec<u8> {
    matrix_hex_digest(value, label).to_vec()
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_relocalize_request(
    profile: MatrixProfile,
    activated: &native_fixture_proto::MappingSessionState,
    checkpoint: &native_fixture_proto::MappingBackendCheckpoint,
) -> native_fixture_proto::RelocalizeRequest {
    let cloud = checkpoint
        .canonical_cloud_checkpoint
        .as_ref()
        .expect("matrix relocalization cloud checkpoint");
    native_fixture_proto::RelocalizeRequest {
        schema_version: 1,
        request_id: format!("{}-relocalize", profile.scenario()),
        runtime_generation: activated.runtime_generation,
        session_id: activated.session_id.clone(),
        map_name: activated.map_name.clone(),
        session_generation: activated.session_generation,
        backend_generation: activated.backend_generation,
        activation_identity_sha256: activated.activation_identity_sha256.clone(),
        target_receipt_sha256: activated.target_receipt_sha256.clone(),
        target_cloud_revision: cloud.source_revision,
        target_cloud_sha256: matrix_hex_digest(
            &cloud.xyz_sha256,
            "relocalization target cloud digest",
        )
        .to_vec(),
        asset_manifest_sha256: activated.asset_manifest_sha256.clone(),
        asset_bundle_sha256: activated.asset_bundle_sha256.clone(),
        source_policy_version: 1,
        source_policy_sha256: matrix_policy_digest(
            "b0e3c49266e73179cee77207a64680ee719e497886db18ea2267093c94a76aa8",
            "source policy digest",
        ),
        deadline_policy_version: 1,
        deadline_policy_sha256: matrix_policy_digest(
            "cf2793085656ef81fadf12a80fb1147973ec71a77735a4a3e600ac1345b59c70",
            "deadline policy digest",
        ),
        logical_timeout_ms: 30_000,
        algorithm_options_version: 1,
        algorithm_options_sha256: matrix_policy_digest(
            "f8b5ef8b359eeb0129f6d523244ea8bca35102143f3bce28e619598749276090",
            "algorithm policy digest",
        ),
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_relocalization_failure_diagnostic(
    checkpoint: &native_fixture_proto::MappingBackendCheckpoint,
    published: &MatrixPublishedFrame,
) -> String {
    let checkpoint = checkpoint
        .canonical_cloud_checkpoint
        .as_ref()
        .expect("matrix relocalization cloud checkpoint");
    let target = checkpoint
        .xyz_f32_le
        .chunks_exact(12)
        .map(|bytes| {
            nalgebra::Point3::new(
                f64::from(f32::from_le_bytes(
                    bytes[0..4].try_into().expect("target x"),
                )),
                f64::from(f32::from_le_bytes(
                    bytes[4..8].try_into().expect("target y"),
                )),
                f64::from(f32::from_le_bytes(
                    bytes[8..12].try_into().expect("target z"),
                )),
            )
        })
        .collect::<Vec<_>>();
    let source = xyz_points(&decode_public_cloud(&published.cloud).cloud)
        .into_iter()
        .map(nalgebra::Point3::from)
        .collect::<Vec<_>>();
    let algorithm = nuwax_mapping::relocalization::relocation_pipeline_with_config(
        &source,
        &target,
        nuwax_mapping::relocalization::RelocalizationConfig::default(),
    );
    let odometry = decode_public_odometry(&published.odometry);
    let fine_source = nuwax_mapping::relocalization::voxel_downsample(&source, 0.1);
    let fine_target = nuwax_mapping::relocalization::voxel_downsample(&target, 0.1);
    let identity_distances = fine_source
        .iter()
        .map(|source_point| {
            fine_target
                .iter()
                .map(|target_point| nalgebra::distance_squared(source_point, target_point))
                .fold(f64::INFINITY, f64::min)
        })
        .collect::<Vec<_>>();
    let identity_inliers = identity_distances
        .iter()
        .filter(|distance| **distance <= 0.15_f64.powi(2))
        .count();
    let identity_fitness = identity_inliers as f64 / fine_source.len() as f64;
    let identity_rmse = (identity_distances
        .iter()
        .filter(|distance| **distance <= 0.15_f64.powi(2))
        .sum::<f64>()
        / identity_inliers.max(1) as f64)
        .sqrt();
    let scale_diagnostics = [0.2_f64, 0.3, 0.8]
        .into_iter()
        .map(|voxel_size| {
            let source_points =
                nuwax_mapping::relocalization::voxel_downsample(&source, voxel_size);
            let target_points =
                nuwax_mapping::relocalization::voxel_downsample(&target, voxel_size);
            let source_normals = nuwax_mapping::relocalization::estimate_normals(
                &source_points,
                voxel_size * 2.0,
                100,
            );
            let target_normals = nuwax_mapping::relocalization::estimate_normals(
                &target_points,
                voxel_size * 2.0,
                100,
            );
            let source_features = nuwax_mapping::relocalization::compute_fpfh(
                &source_points,
                &source_normals,
                voxel_size * 5.0,
                100,
            );
            let target_features = nuwax_mapping::relocalization::compute_fpfh(
                &target_points,
                &target_normals,
                voxel_size * 5.0,
                100,
            );
            let matches =
                nuwax_mapping::relocalization::match_fpfh(&source_features, &target_features);
            let ransac = nuwax_mapping::relocalization::ransac_align(
                &source_points,
                &target_points,
                &matches,
                nuwax_mapping::relocalization::RansacConfig {
                    max_iterations: 4_096,
                    inlier_threshold: voxel_size * 1.5,
                    min_inliers: 3,
                    min_inlier_ratio: 0.45,
                    ..nuwax_mapping::relocalization::RansacConfig::default()
                },
            );
            format!(
                "scale={voxel_size}:source={}:target={}:matches={}:inliers={}:converged={}:tilt={}",
                source_points.len(),
                target_points.len(),
                matches.len(),
                ransac.inlier_count,
                ransac.converged,
                nuwax_mapping::relocalization::gravity_tilt_deg(&ransac.transformation),
            )
        })
        .collect::<Vec<_>>()
        .join(";");
    format!(
        "source_points={} target_points={} fine_source={} fine_target={} identity_fitness={} identity_rmse={} odom_position={:?} odom_orientation={:?} converged={} feature_matches={} ransac_inliers={} fitness={} mse={} scales=[{}]",
        source.len(),
        target.len(),
        fine_source.len(),
        fine_target.len(),
        identity_fitness,
        identity_rmse,
        odometry.odometry.pose.position,
        odometry.odometry.pose.orientation_xyzw,
        algorithm.registration.converged,
        algorithm.feature_matches,
        algorithm.ransac_inliers,
        algorithm.fitness,
        algorithm.registration.mse,
        scale_diagnostics,
    )
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
const MATRIX_EVIDENCE_DOMAIN_V1: &[u8] = b"nuwax.geometry.relocalize.evidence.v1\0";
#[cfg(all(feature = "nuwax-lio-test-child", unix))]
const MATRIX_EVIDENCE_DISCLOSURE_DOMAIN_V1: &[u8] =
    b"nuwax.geometry.relocalize.evidence-disclosure.v1\0";
#[cfg(all(feature = "nuwax-lio-test-child", unix))]
const MATRIX_EVIDENCE_COMPOSITE_DOMAIN_V1: &[u8] =
    b"nuwax.geometry.relocalize.evidence-composite.v1\0";
#[cfg(all(feature = "nuwax-lio-test-child", unix))]
const MATRIX_SOURCE_COMMITMENT_DOMAIN_V1: &[u8] =
    b"nuwax.geometry.relocalize.source-commitment.v1\0";
#[cfg(all(feature = "nuwax-lio-test-child", unix))]
const MATRIX_TERMINAL_PROOF_DOMAIN_V1: &[u8] = b"nuwax.geometry.relocalize.terminal-proof.v1\0";

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
struct MatrixProofCursor<'a> {
    bytes: &'a [u8],
    offset: usize,
    label: &'static str,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
impl<'a> MatrixProofCursor<'a> {
    fn new(bytes: &'a [u8], label: &'static str) -> Self {
        Self {
            bytes,
            offset: 0,
            label,
        }
    }

    fn take_slice(&mut self, length: usize, field: &str) -> &'a [u8] {
        let end = self
            .offset
            .checked_add(length)
            .unwrap_or_else(|| panic!("matrix proof {} {field} offset overflow", self.label));
        let value = self.bytes.get(self.offset..end).unwrap_or_else(|| {
            panic!(
                "matrix proof {} {field} truncated at byte {}",
                self.label, self.offset
            )
        });
        self.offset = end;
        value
    }

    fn take<const N: usize>(&mut self, field: &str) -> [u8; N] {
        self.take_slice(N, field)
            .try_into()
            .unwrap_or_else(|_| panic!("matrix proof {} {field} width drifted", self.label))
    }

    fn literal(&mut self, expected: &[u8], field: &str) {
        assert_eq!(
            self.take_slice(expected.len(), field),
            expected,
            "matrix proof {} {field} literal drifted",
            self.label
        );
    }

    fn u8(&mut self, field: &str) -> u8 {
        self.take::<1>(field)[0]
    }

    fn flag(&mut self, field: &str) -> bool {
        match self.u8(field) {
            0 => false,
            1 => true,
            value => panic!(
                "matrix proof {} {field} flag is not canonical: {value}",
                self.label
            ),
        }
    }

    fn u32(&mut self, field: &str) -> u32 {
        u32::from_be_bytes(self.take(field))
    }

    fn i32(&mut self, field: &str) -> i32 {
        i32::from_be_bytes(self.take(field))
    }

    fn u64(&mut self, field: &str) -> u64 {
        u64::from_be_bytes(self.take(field))
    }

    fn f32(&mut self, field: &str) -> f32 {
        f32::from_be_bytes(self.take(field))
    }

    fn f64(&mut self, field: &str) -> f64 {
        f64::from_be_bytes(self.take(field))
    }

    fn digest(&mut self, field: &str) -> [u8; 32] {
        let digest = self.take(field);
        assert!(
            digest.iter().any(|byte| *byte != 0),
            "matrix proof {} {field} digest is absent",
            self.label
        );
        digest
    }

    fn string(&mut self, field: &str) -> String {
        let length = usize::try_from(self.u32(&format!("{field} length")))
            .expect("matrix proof string length fits usize");
        assert!(
            length <= 4_096,
            "matrix proof {} {field} string exceeds v1 bounds",
            self.label
        );
        std::str::from_utf8(self.take_slice(length, field))
            .unwrap_or_else(|error| {
                panic!("matrix proof {} {field} is not UTF-8: {error}", self.label)
            })
            .to_owned()
    }

    fn tag(&mut self, expected: u32) {
        assert_eq!(
            self.u32(&format!("field tag {expected}")),
            expected,
            "matrix proof {} field tag order drifted",
            self.label
        );
    }

    fn finish(self) {
        assert_eq!(
            self.offset,
            self.bytes.len(),
            "matrix proof {} has {} trailing bytes",
            self.label,
            self.bytes.len() - self.offset
        );
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct MatrixRelocalizationFrameKey {
    mapping_commit_boot_nonce: [u8; 16],
    mapping_commit_sequence: u64,
    occupancy_revision: u64,
    cloud_revision: u64,
    registered_cloud_source_timestamp_ns: u64,
    registered_cloud_source_sequence: u64,
    correlated_odom_source_sequence: u64,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_read_frame_key(
    cursor: &mut MatrixProofCursor<'_>,
    field: &str,
) -> MatrixRelocalizationFrameKey {
    let key = MatrixRelocalizationFrameKey {
        mapping_commit_boot_nonce: cursor.take(&format!("{field} mapping_commit_boot_nonce")),
        mapping_commit_sequence: cursor.u64(&format!("{field} mapping_commit_sequence")),
        occupancy_revision: cursor.u64(&format!("{field} occupancy_revision")),
        cloud_revision: cursor.u64(&format!("{field} cloud_revision")),
        registered_cloud_source_timestamp_ns: cursor
            .u64(&format!("{field} registered_cloud_source_timestamp_ns")),
        registered_cloud_source_sequence: cursor
            .u64(&format!("{field} registered_cloud_source_sequence")),
        correlated_odom_source_sequence: cursor
            .u64(&format!("{field} correlated_odom_source_sequence")),
    };
    assert!(key.mapping_commit_boot_nonce.iter().any(|byte| *byte != 0));
    assert!(key.mapping_commit_sequence > 0);
    assert!(key.occupancy_revision > 0);
    assert!(key.cloud_revision > 0);
    assert!(key.registered_cloud_source_sequence > 0);
    assert!(key.correlated_odom_source_sequence > 0);
    key
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_read_optional_frame_key(
    cursor: &mut MatrixProofCursor<'_>,
    field: &str,
) -> Option<MatrixRelocalizationFrameKey> {
    cursor
        .flag(&format!("{field} present"))
        .then(|| matrix_read_frame_key(cursor, field))
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Clone, Copy, Debug, PartialEq)]
struct MatrixProofTransform {
    translation_m: [f64; 3],
    rotation_xyzw: [f64; 4],
    encoded: [u8; 56],
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
impl MatrixProofTransform {
    fn isometry(self) -> nalgebra::Isometry3<f64> {
        nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(
                self.translation_m[0],
                self.translation_m[1],
                self.translation_m[2],
            ),
            nalgebra::UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(
                self.rotation_xyzw[3],
                self.rotation_xyzw[0],
                self.rotation_xyzw[1],
                self.rotation_xyzw[2],
            )),
        )
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_read_proof_transform(
    cursor: &mut MatrixProofCursor<'_>,
    field: &str,
) -> MatrixProofTransform {
    let encoded = cursor.take::<56>(field);
    let mut translation_m = [0.0; 3];
    let mut rotation_xyzw = [0.0; 4];
    for (index, value) in translation_m.iter_mut().enumerate() {
        let start = index * 8;
        *value = f64::from_be_bytes(encoded[start..start + 8].try_into().unwrap());
    }
    for (index, value) in rotation_xyzw.iter_mut().enumerate() {
        let start = 24 + index * 8;
        *value = f64::from_be_bytes(encoded[start..start + 8].try_into().unwrap());
    }
    assert!(
        translation_m
            .iter()
            .chain(rotation_xyzw.iter())
            .all(|value| value.is_finite()),
        "matrix proof {field} contains a non-finite transform"
    );
    let norm_squared = rotation_xyzw.iter().map(|value| value * value).sum::<f64>();
    assert!(
        (norm_squared - 1.0).abs() <= 1.0e-6,
        "matrix proof {field} quaternion is not unit length"
    );
    MatrixProofTransform {
        translation_m,
        rotation_xyzw,
        encoded,
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_reencode_candidate_preimage(candidate: &DecodedCorrectionCandidate) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(512);
    encoded.extend_from_slice(&1_i32.to_be_bytes());
    encoded.extend_from_slice(&candidate.backend.to_be_bytes());
    encoded.extend_from_slice(&candidate.transform_semantics.to_be_bytes());
    encoded.extend_from_slice(&candidate.runtime_generation.to_be_bytes());
    encoded.extend_from_slice(&candidate.backend_generation.to_be_bytes());
    encoded.extend_from_slice(&candidate.candidate_sequence.to_be_bytes());
    encode_candidate_lcm_string(&candidate.mapping_session_id, &mut encoded);
    encoded.extend_from_slice(&candidate.mapping_session_generation.to_be_bytes());
    encoded.extend_from_slice(&candidate.map_frame_generation.to_be_bytes());
    encoded.extend_from_slice(&candidate.mapping_commit_boot_nonce);
    encoded.extend_from_slice(&candidate.mapping_commit_sequence.to_be_bytes());
    encoded.extend_from_slice(&candidate.occupancy_revision.to_be_bytes());
    encoded.extend_from_slice(&candidate.cloud_revision.to_be_bytes());
    encoded.extend_from_slice(&candidate.committed_map_source_timestamp_ns.to_be_bytes());
    encoded.extend_from_slice(&candidate.registered_cloud_source_sequence.to_be_bytes());
    encoded.extend_from_slice(&candidate.registered_cloud_source_timestamp_ns.to_be_bytes());
    encoded.extend_from_slice(&candidate.registered_cloud_point_count.to_be_bytes());
    encoded.extend_from_slice(&candidate.registered_cloud_envelope_sha256);
    encoded.extend_from_slice(&candidate.body_points_sha256);
    encoded.extend_from_slice(&candidate.correlated_odom_source_sequence.to_be_bytes());
    encoded.extend_from_slice(&candidate.correlated_odom_source_timestamp_ns.to_be_bytes());
    encoded.extend_from_slice(&candidate.correlated_odom_pose_sha256);
    encoded.extend_from_slice(&candidate.committed_map_width.to_be_bytes());
    encoded.extend_from_slice(&candidate.committed_map_height.to_be_bytes());
    encoded.extend_from_slice(&candidate.committed_map_resolution_m.to_be_bytes());
    encoded.extend_from_slice(&candidate.committed_map_origin_x_m.to_be_bytes());
    encoded.extend_from_slice(&candidate.committed_map_origin_y_m.to_be_bytes());
    encoded.extend_from_slice(&candidate.committed_map_payload_sha256);
    encoded.extend_from_slice(&candidate.committed_map_boundary_sha256);
    encoded.extend_from_slice(&candidate.source_timestamp_ns.to_be_bytes());
    encode_candidate_lcm_string(&candidate.map_frame_id, &mut encoded);
    encode_candidate_lcm_string(&candidate.odom_frame_id, &mut encoded);
    encode_candidate_lcm_string(&candidate.body_frame_id, &mut encoded);
    encode_candidate_lcm_string(&candidate.points_frame_id, &mut encoded);
    encoded.extend_from_slice(&candidate.correction_lease_boot_nonce);
    encoded.extend_from_slice(&candidate.correction_lease_generation.to_be_bytes());
    for value in candidate
        .translation_m
        .iter()
        .chain(candidate.rotation_xyzw.iter())
    {
        encoded.extend_from_slice(&value.to_be_bytes());
    }
    encoded
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_validate_candidate_proof(
    result: &native_fixture_proto::RelocalizeResult,
    scenario: &str,
) -> DecodedCorrectionCandidate {
    let preimage = result.candidate_payload_preimage_v1.as_slice();
    assert!(
        !preimage.is_empty() && preimage.len() <= 4_096,
        "{scenario}: field 29 is outside the candidate preimage bound"
    );
    let expected_digest = matrix_digest32(
        &result.candidate_payload_sha256,
        "matrix candidate payload digest",
    );
    assert_eq!(
        nuwax_codec::sha256_bytes(preimage),
        expected_digest,
        "{scenario}: field 24 does not hash the complete field 29"
    );
    let mut body = preimage.to_vec();
    body.extend_from_slice(&expected_digest);
    let binding = typed_binding(
        "/matrix/relocalization/candidate-proof",
        "nuwax_msgs.CorrectionCandidate",
    );
    let frame = nuwax_protocol::RawLcmFrameBuilder::new(binding.0, binding.1)
        .seqno(1)
        .payload(body)
        .build();
    let candidate = decode_correction_candidate(&frame);
    assert_eq!(candidate.backend, 3, "{scenario}");
    assert_eq!(candidate.transform_semantics, 1, "{scenario}");
    assert!(candidate.committed_map_width <= u64::from(u32::MAX));
    assert!(candidate.committed_map_height <= u64::from(u32::MAX));
    assert_eq!(
        matrix_reencode_candidate_preimage(&candidate),
        preimage,
        "{scenario}: field 29 is not the canonical literal LCM prefix"
    );
    assert_eq!(candidate.payload_sha256, expected_digest, "{scenario}");
    assert_eq!(
        candidate.runtime_generation, result.runtime_generation,
        "{scenario}"
    );
    assert_eq!(
        candidate.candidate_sequence, result.candidate_sequence,
        "{scenario}"
    );
    assert_eq!(
        candidate.mapping_session_id, result.session_id,
        "{scenario}"
    );
    assert_eq!(
        candidate.mapping_session_generation, result.session_generation,
        "{scenario}"
    );
    candidate
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_candidate_frame_key(
    candidate: &DecodedCorrectionCandidate,
) -> MatrixRelocalizationFrameKey {
    MatrixRelocalizationFrameKey {
        mapping_commit_boot_nonce: candidate.mapping_commit_boot_nonce,
        mapping_commit_sequence: candidate.mapping_commit_sequence,
        occupancy_revision: candidate.occupancy_revision,
        cloud_revision: candidate.cloud_revision,
        registered_cloud_source_timestamp_ns: candidate.registered_cloud_source_timestamp_ns,
        registered_cloud_source_sequence: candidate.registered_cloud_source_sequence,
        correlated_odom_source_sequence: candidate.correlated_odom_source_sequence,
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_map_boundary_sha256(
    map_frame_id: &str,
    width: u64,
    height: u64,
    resolution_m: f64,
    origin_x_m: f64,
    origin_y_m: f64,
) -> [u8; 32] {
    let mut preimage = b"nuwax.correction.committed-map-boundary.v1\0".to_vec();
    encode_candidate_lcm_string(map_frame_id, &mut preimage);
    preimage.extend_from_slice(
        &i64::try_from(width)
            .expect("matrix map width fits signed LCM integer")
            .to_be_bytes(),
    );
    preimage.extend_from_slice(
        &i64::try_from(height)
            .expect("matrix map height fits signed LCM integer")
            .to_be_bytes(),
    );
    preimage.extend_from_slice(&resolution_m.to_be_bytes());
    preimage.extend_from_slice(&origin_x_m.to_be_bytes());
    preimage.extend_from_slice(&origin_y_m.to_be_bytes());
    nuwax_codec::sha256_bytes(&preimage)
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Debug)]
struct MatrixEvidenceTarget {
    runtime_generation: u64,
    session_id: String,
    map_name: String,
    session_generation: u64,
    backend_generation: u64,
    activation_identity_sha256: [u8; 32],
    target_receipt_sha256: [u8; 32],
    asset_manifest_sha256: Option<[u8; 32]>,
    asset_bundle_sha256: Option<[u8; 32]>,
    frame_id: String,
    source_revision: u64,
    xyz_sha256: [u8; 32],
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Debug)]
struct MatrixEvidenceFrame {
    key: MatrixRelocalizationFrameKey,
    mapping_session_id: String,
    mapping_session_generation: u64,
    map_frame_generation: u64,
    point_count: u64,
    envelope_sha256: [u8; 32],
    body_points_sha256: [u8; 32],
    odom_source_timestamp_ns: u64,
    odom_pose_sha256: [u8; 32],
    map_source_timestamp_ns: u64,
    map_width: u64,
    map_height: u64,
    map_resolution_m: f64,
    map_origin_x_m: f64,
    map_origin_y_m: f64,
    map_payload_sha256: [u8; 32],
    map_boundary_sha256: [u8; 32],
    points_frame_id: String,
    odom_from_body: MatrixProofTransform,
    base_anchor_map_from_odom: MatrixProofTransform,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Debug)]
struct MatrixEvidenceAlgorithm {
    accepted: bool,
    correction: Option<MatrixProofTransform>,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Debug)]
struct MatrixEvidenceCandidate {
    backend: i32,
    transform_semantics: i32,
    runtime_generation: u64,
    backend_generation: u64,
    candidate_sequence: u64,
    mapping_session_id: String,
    mapping_session_generation: u64,
    map_frame_generation: u64,
    frame_key: MatrixRelocalizationFrameKey,
    correction_lease_boot_nonce: [u8; 16],
    correction_lease_generation: u64,
    transform: MatrixProofTransform,
    payload_sha256: [u8; 32],
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
#[derive(Debug)]
struct MatrixEvidenceAnchors {
    request_identity_sha256: [u8; 32],
    admission_ordinal: Option<u64>,
    normalized_logical_timeout_ms: Option<u64>,
    runtime_generation: u64,
    activated_mapping_backend_generation: u64,
    correction_owner_backend_generation: u64,
    session_id: String,
    map_name: String,
    session_generation: u64,
    map_frame_id: String,
    odom_frame_id: String,
    body_frame_id: String,
    activation_identity_sha256: [u8; 32],
    target_receipt_sha256: [u8; 32],
    asset_manifest_sha256: [u8; 32],
    asset_bundle_sha256: [u8; 32],
    source_policy_version: u32,
    source_policy_sha256: [u8; 32],
    deadline_policy_version: u32,
    deadline_policy_sha256: [u8; 32],
    algorithm_options_version: u32,
    algorithm_options_sha256: [u8; 32],
    target: MatrixEvidenceTarget,
    frames: Vec<MatrixEvidenceFrame>,
    aggregate_input_points: u64,
    post_transform_points: u64,
    emitted_points: u64,
    boundary_key: MatrixRelocalizationFrameKey,
    source_length: usize,
    source_sha256: [u8; 32],
    algorithm: MatrixEvidenceAlgorithm,
    candidate: Option<MatrixEvidenceCandidate>,
    terminal_disposition: i32,
    correction_revision: u64,
    detail: String,
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_parse_mode1_evidence(
    prefix: &[u8],
    source_length: usize,
    source_sha256: [u8; 32],
    suffix: &[u8],
) -> MatrixEvidenceAnchors {
    let mut prefix_cursor = MatrixProofCursor::new(prefix, "prefix");
    prefix_cursor.literal(MATRIX_EVIDENCE_DOMAIN_V1, "domain");
    assert_eq!(prefix_cursor.u32("schema_version"), 1);

    prefix_cursor.tag(1);
    let request_identity_sha256 = prefix_cursor.digest("request_identity_sha256");
    prefix_cursor.tag(2);
    let admission_ordinal = prefix_cursor
        .flag("admission_ordinal present")
        .then(|| prefix_cursor.u64("admission_ordinal"));
    prefix_cursor.tag(3);
    let normalized_logical_timeout_ms = prefix_cursor
        .flag("normalized_logical_timeout_ms present")
        .then(|| prefix_cursor.u64("normalized_logical_timeout_ms"));
    prefix_cursor.tag(4);
    let runtime_generation = prefix_cursor.u64("runtime_generation");
    prefix_cursor.tag(5);
    let activated_mapping_backend_generation =
        prefix_cursor.u64("activated_mapping_backend_generation");
    prefix_cursor.tag(6);
    let correction_owner_backend_generation =
        prefix_cursor.u64("correction_owner_backend_generation");
    prefix_cursor.tag(7);
    let session_id = prefix_cursor.string("session_id");
    prefix_cursor.tag(8);
    let map_name = prefix_cursor.string("map_name");
    prefix_cursor.tag(9);
    let session_generation = prefix_cursor.u64("session_generation");
    prefix_cursor.tag(10);
    let map_frame_id = prefix_cursor.string("map_frame_id");
    prefix_cursor.tag(11);
    let odom_frame_id = prefix_cursor.string("odom_frame_id");
    prefix_cursor.tag(12);
    let body_frame_id = prefix_cursor.string("body_frame_id");
    assert!(!map_frame_id.is_empty() && !odom_frame_id.is_empty() && !body_frame_id.is_empty());
    assert_ne!(map_frame_id, odom_frame_id);
    assert_ne!(map_frame_id, body_frame_id);
    assert_ne!(odom_frame_id, body_frame_id);
    prefix_cursor.tag(13);
    let activation_identity_sha256 = prefix_cursor.digest("activation_identity_sha256");
    prefix_cursor.tag(14);
    let target_receipt_sha256 = prefix_cursor.digest("target_receipt_sha256");
    prefix_cursor.tag(15);
    let asset_manifest_sha256 = prefix_cursor.digest("asset_manifest_sha256");
    prefix_cursor.tag(16);
    let asset_bundle_sha256 = prefix_cursor.digest("asset_bundle_sha256");
    prefix_cursor.tag(17);
    let source_policy_version = prefix_cursor.u32("source_policy_version");
    let source_policy_sha256 = prefix_cursor.digest("source_policy_sha256");
    prefix_cursor.tag(18);
    let deadline_policy_version = prefix_cursor.u32("deadline_policy_version");
    let deadline_policy_sha256 = prefix_cursor.digest("deadline_policy_sha256");
    prefix_cursor.tag(19);
    let algorithm_options_version = prefix_cursor.u32("algorithm_options_version");
    let algorithm_options_sha256 = prefix_cursor.digest("algorithm_options_sha256");

    prefix_cursor.tag(20);
    assert!(prefix_cursor.flag("target present"));
    prefix_cursor.literal(b"target.v1\0", "target domain");
    let target_runtime_generation = prefix_cursor.u64("target runtime_generation");
    let target_lifecycle_epoch = prefix_cursor.u64("target lifecycle_epoch");
    let target_session_id = prefix_cursor.string("target session_id");
    let target_map_name = prefix_cursor.string("target map_name");
    let target_session_generation = prefix_cursor.u64("target session_generation");
    let target_backend_generation = prefix_cursor.u64("target backend_generation");
    let _activation_request_id = prefix_cursor.string("activation_request_id");
    let _activation_request_identity_sha256 =
        prefix_cursor.digest("activation_request_identity_sha256");
    let target_activation_identity_sha256 =
        prefix_cursor.digest("target activation_identity_sha256");
    let target_target_receipt_sha256 = prefix_cursor.digest("target target_receipt_sha256");
    let target_asset_manifest_sha256 = prefix_cursor
        .flag("target asset_manifest_sha256 present")
        .then(|| prefix_cursor.digest("target asset_manifest_sha256"));
    let target_asset_bundle_sha256 = prefix_cursor
        .flag("target asset_bundle_sha256 present")
        .then(|| prefix_cursor.digest("target asset_bundle_sha256"));
    let target_frame_id = prefix_cursor.string("target frame_id");
    let target_source_revision = prefix_cursor.u64("target source_revision");
    let target_point_count = prefix_cursor.u32("target point_count");
    let target_voxel_size = prefix_cursor.f32("target voxel_size");
    let target_payload_schema_version = prefix_cursor.u32("target payload_schema_version");
    let target_xyz_length = prefix_cursor.u64("target xyz length");
    let target_xyz_sha256 = prefix_cursor.digest("target xyz_sha256");
    let target_decoded_point_count = prefix_cursor.u64("target decoded_point_count");
    let target_decoded_f64_bytes = prefix_cursor.u64("target decoded_f64_bytes");
    let _target_decoded_sha256 = prefix_cursor.digest("target decoded_sha256");
    let publication_source_timestamp_ns =
        prefix_cursor.u64("target publication source_timestamp_ns");
    let publication_cloud_revision = prefix_cursor.u64("target publication cloud_revision");
    let publication_point_count = prefix_cursor.u32("target publication point_count");
    let publication_voxel_size = prefix_cursor.f32("target publication voxel_size");
    let publication_mapping_commit_boot_nonce =
        prefix_cursor.take::<16>("target publication mapping_commit_boot_nonce");
    let publication_mapping_commit_sequence =
        prefix_cursor.u64("target publication mapping_commit_sequence");
    let publication_session_generation = prefix_cursor.u64("target publication session_generation");
    let publication_occupancy_revision = prefix_cursor.u64("target publication occupancy_revision");
    let publication_receipt_cloud_revision =
        prefix_cursor.u64("target publication receipt cloud_revision");
    let publication_receipt_source_timestamp_ns =
        prefix_cursor.u64("target publication receipt source_timestamp_ns");
    let publication_receipt_point_count =
        prefix_cursor.u32("target publication receipt point_count");
    let publication_target_receipt_sha256 =
        prefix_cursor.digest("target publication target_receipt_sha256");
    assert!(target_runtime_generation > 0);
    assert!(target_lifecycle_epoch > 0);
    assert!(target_session_generation > 0);
    assert!(target_backend_generation > 0);
    assert!(target_source_revision > 0);
    assert!(target_point_count > 0);
    assert!(target_voxel_size.is_finite() && target_voxel_size > 0.0);
    assert_eq!(target_payload_schema_version, 1);
    assert_eq!(target_xyz_length, u64::from(target_point_count) * 12);
    assert_eq!(target_decoded_point_count, u64::from(target_point_count));
    assert_eq!(target_decoded_f64_bytes, u64::from(target_point_count) * 24);
    assert!(publication_cloud_revision > 0);
    assert_eq!(publication_point_count, target_point_count);
    assert_eq!(
        publication_voxel_size.to_bits(),
        target_voxel_size.to_bits()
    );
    assert!(publication_mapping_commit_boot_nonce
        .iter()
        .any(|byte| *byte != 0));
    assert!(publication_mapping_commit_sequence > 0);
    assert_eq!(publication_session_generation, target_session_generation);
    assert!(publication_occupancy_revision > 0);
    assert_eq!(
        publication_receipt_cloud_revision,
        publication_cloud_revision
    );
    assert_eq!(
        publication_receipt_source_timestamp_ns,
        publication_source_timestamp_ns
    );
    assert_eq!(publication_receipt_point_count, publication_point_count);
    assert_eq!(
        publication_target_receipt_sha256,
        target_target_receipt_sha256
    );
    let target = MatrixEvidenceTarget {
        runtime_generation: target_runtime_generation,
        session_id: target_session_id,
        map_name: target_map_name,
        session_generation: target_session_generation,
        backend_generation: target_backend_generation,
        activation_identity_sha256: target_activation_identity_sha256,
        target_receipt_sha256: target_target_receipt_sha256,
        asset_manifest_sha256: target_asset_manifest_sha256,
        asset_bundle_sha256: target_asset_bundle_sha256,
        frame_id: target_frame_id,
        source_revision: target_source_revision,
        xyz_sha256: target_xyz_sha256,
    };

    prefix_cursor.tag(21);
    assert!(prefix_cursor.flag("source present"));
    prefix_cursor.literal(b"source.v1\0", "source domain");
    let _arm_watermark = matrix_read_optional_frame_key(&mut prefix_cursor, "source arm_watermark");
    let cutoff_reason = prefix_cursor.u32("source cutoff_reason");
    let last_accepted_key =
        matrix_read_optional_frame_key(&mut prefix_cursor, "source last_accepted_key");
    let frame_count = usize::try_from(prefix_cursor.u32("source frame_count"))
        .expect("matrix source frame count fits usize");
    assert!((1..=8).contains(&frame_count));
    let mut frames = Vec::with_capacity(frame_count);
    for index in 0..frame_count {
        let frame_name = format!("source frame {index}");
        prefix_cursor.literal(b"frame.v1\0", &format!("{frame_name} domain"));
        let key = matrix_read_frame_key(&mut prefix_cursor, &format!("{frame_name} key"));
        let mapping_session_id = prefix_cursor.string(&format!("{frame_name} mapping_session_id"));
        let mapping_session_generation =
            prefix_cursor.u64(&format!("{frame_name} mapping_session_generation"));
        let map_frame_generation = prefix_cursor.u64(&format!("{frame_name} map_frame_generation"));
        let point_count = prefix_cursor.u64(&format!("{frame_name} point_count"));
        let envelope_sha256 = prefix_cursor.digest(&format!("{frame_name} envelope_sha256"));
        let body_points_sha256 = prefix_cursor.digest(&format!("{frame_name} body_points_sha256"));
        let odom_source_timestamp_ns =
            prefix_cursor.u64(&format!("{frame_name} odom_source_timestamp_ns"));
        let odom_pose_sha256 = prefix_cursor.digest(&format!("{frame_name} odom_pose_sha256"));
        let map_source_timestamp_ns =
            prefix_cursor.u64(&format!("{frame_name} map_source_timestamp_ns"));
        let map_width = prefix_cursor.u64(&format!("{frame_name} map_width"));
        let map_height = prefix_cursor.u64(&format!("{frame_name} map_height"));
        let map_resolution_m = prefix_cursor.f64(&format!("{frame_name} map_resolution_m"));
        let map_origin_x_m = prefix_cursor.f64(&format!("{frame_name} map_origin_x_m"));
        let map_origin_y_m = prefix_cursor.f64(&format!("{frame_name} map_origin_y_m"));
        let map_payload_sha256 = prefix_cursor.digest(&format!("{frame_name} map_payload_sha256"));
        let map_boundary_sha256 =
            prefix_cursor.digest(&format!("{frame_name} map_boundary_sha256"));
        let points_frame_id = prefix_cursor.string(&format!("{frame_name} points_frame_id"));
        let odom_from_body = matrix_read_proof_transform(
            &mut prefix_cursor,
            &format!("{frame_name} odom_from_body"),
        );
        let base_anchor_map_from_odom = matrix_read_proof_transform(
            &mut prefix_cursor,
            &format!("{frame_name} base_anchor_map_from_odom"),
        );
        let _frame_evidence_sha256 = prefix_cursor.digest(&format!("{frame_name} evidence_sha256"));
        assert_eq!(mapping_session_id, session_id);
        assert_eq!(mapping_session_generation, session_generation);
        assert!(map_frame_generation > 0);
        assert!((1..=100_000).contains(&point_count));
        assert!(odom_source_timestamp_ns <= key.registered_cloud_source_timestamp_ns);
        assert!(map_source_timestamp_ns <= key.registered_cloud_source_timestamp_ns);
        assert!((1..=u64::from(u32::MAX)).contains(&map_width));
        assert!((1..=u64::from(u32::MAX)).contains(&map_height));
        assert!(map_width.checked_mul(map_height).is_some());
        assert!(map_resolution_m.is_finite() && map_resolution_m > 0.0);
        assert!(map_origin_x_m.is_finite() && map_origin_y_m.is_finite());
        assert_eq!(points_frame_id, body_frame_id);
        assert_eq!(
            matrix_map_boundary_sha256(
                &map_frame_id,
                map_width,
                map_height,
                map_resolution_m,
                map_origin_x_m,
                map_origin_y_m,
            ),
            map_boundary_sha256
        );
        frames.push(MatrixEvidenceFrame {
            key,
            mapping_session_id,
            mapping_session_generation,
            map_frame_generation,
            point_count,
            envelope_sha256,
            body_points_sha256,
            odom_source_timestamp_ns,
            odom_pose_sha256,
            map_source_timestamp_ns,
            map_width,
            map_height,
            map_resolution_m,
            map_origin_x_m,
            map_origin_y_m,
            map_payload_sha256,
            map_boundary_sha256,
            points_frame_id,
            odom_from_body,
            base_anchor_map_from_odom,
        });
    }
    let aggregate_input_points = prefix_cursor.u64("source aggregate_input_points");
    let aggregate_input_f64_bytes = prefix_cursor.u64("source aggregate_input_f64_bytes");
    let post_transform_points = prefix_cursor.u64("source post_transform_points");
    let post_transform_f64_bytes = prefix_cursor.u64("source post_transform_f64_bytes");
    let emitted_points = prefix_cursor.u64("source emitted_points");
    prefix_cursor.literal(
        MATRIX_SOURCE_COMMITMENT_DOMAIN_V1,
        "source commitment domain",
    );
    assert_eq!(prefix_cursor.u32("source commitment version"), 1);
    let committed_source_length = usize::try_from(prefix_cursor.u64("source commitment length"))
        .expect("matrix committed source length fits usize");
    let committed_source_sha256 = prefix_cursor.digest("source commitment sha256");
    prefix_cursor.finish();

    let frame_keys = frames.iter().map(|frame| frame.key).collect::<Vec<_>>();
    let mut sorted_keys = frame_keys.clone();
    sorted_keys.sort_unstable();
    sorted_keys.dedup();
    assert_eq!(frame_keys, sorted_keys);
    assert!((1..=14).contains(&cutoff_reason));
    assert_eq!(last_accepted_key, frame_keys.last().copied());
    assert_eq!(
        aggregate_input_points,
        frames.iter().map(|frame| frame.point_count).sum::<u64>()
    );
    assert_eq!(aggregate_input_f64_bytes, aggregate_input_points * 24);
    assert_eq!(post_transform_points, aggregate_input_points);
    assert_eq!(post_transform_f64_bytes, post_transform_points * 24);
    assert!((50_000..=100_000).contains(&emitted_points));
    assert!(emitted_points <= post_transform_points);
    assert_eq!(committed_source_length, source_length);
    assert_eq!(
        source_length,
        8 + usize::try_from(emitted_points).unwrap() * 24
    );
    assert_eq!(committed_source_sha256, source_sha256);

    let mut suffix_cursor = MatrixProofCursor::new(suffix, "suffix");
    let boundary_key = matrix_read_frame_key(&mut suffix_cursor, "source boundary_key");
    let boundary_base_anchor_map_from_odom = matrix_read_proof_transform(
        &mut suffix_cursor,
        "source boundary_base_anchor_map_from_odom",
    );
    let boundary_frame = frames.last().expect("matrix evidence boundary frame");
    assert_eq!(boundary_key, boundary_frame.key);
    assert_eq!(
        boundary_base_anchor_map_from_odom.encoded,
        boundary_frame.base_anchor_map_from_odom.encoded
    );

    suffix_cursor.tag(22);
    assert!(suffix_cursor.flag("algorithm present"));
    suffix_cursor.literal(b"algorithm.v1\0", "algorithm domain");
    let converged = suffix_cursor.flag("algorithm converged");
    let mse = suffix_cursor.f64("algorithm mse");
    let _iterations = suffix_cursor.u64("algorithm iterations");
    let _feature_matches = suffix_cursor.u64("algorithm feature_matches");
    let _ransac_inliers = suffix_cursor.u64("algorithm ransac_inliers");
    let fitness = suffix_cursor.f64("algorithm fitness");
    let _algorithm_transform =
        matrix_read_proof_transform(&mut suffix_cursor, "algorithm transform");
    let accepted = suffix_cursor.flag("algorithm accepted");
    let correction = suffix_cursor
        .flag("algorithm correction present")
        .then(|| matrix_read_proof_transform(&mut suffix_cursor, "algorithm correction"));
    assert!(!mse.is_nan() && mse >= 0.0);
    assert!(mse.is_finite() || (!converged && !accepted));
    assert!(fitness.is_finite() && (0.0..=1.0).contains(&fitness));
    assert_eq!(accepted, correction.is_some());
    assert!(!accepted || converged);
    let algorithm = MatrixEvidenceAlgorithm {
        accepted,
        correction,
    };

    suffix_cursor.tag(23);
    let candidate = suffix_cursor.flag("candidate present").then(|| {
        suffix_cursor.literal(b"candidate.v1\0", "candidate domain");
        let candidate = MatrixEvidenceCandidate {
            backend: suffix_cursor.i32("candidate backend"),
            transform_semantics: suffix_cursor.i32("candidate transform_semantics"),
            runtime_generation: suffix_cursor.u64("candidate runtime_generation"),
            backend_generation: suffix_cursor.u64("candidate backend_generation"),
            candidate_sequence: suffix_cursor.u64("candidate candidate_sequence"),
            mapping_session_id: suffix_cursor.string("candidate mapping_session_id"),
            mapping_session_generation: suffix_cursor.u64("candidate mapping_session_generation"),
            map_frame_generation: suffix_cursor.u64("candidate map_frame_generation"),
            frame_key: matrix_read_frame_key(&mut suffix_cursor, "candidate frame_key"),
            correction_lease_boot_nonce: suffix_cursor
                .take("candidate correction_lease_boot_nonce"),
            correction_lease_generation: suffix_cursor.u64("candidate correction_lease_generation"),
            transform: matrix_read_proof_transform(&mut suffix_cursor, "candidate transform"),
            payload_sha256: suffix_cursor.digest("candidate payload_sha256"),
        };
        assert_eq!(candidate.backend, 3);
        assert_eq!(candidate.transform_semantics, 1);
        assert!(candidate.runtime_generation > 0);
        assert!(candidate.backend_generation > 0);
        assert!(candidate.candidate_sequence > 0);
        assert!(candidate.mapping_session_generation > 0);
        assert!(candidate.map_frame_generation > 0);
        assert!(candidate.correction_lease_generation > 0);
        assert!(candidate
            .correction_lease_boot_nonce
            .iter()
            .any(|byte| *byte != 0));
        assert!(accepted);
        candidate
    });
    suffix_cursor.tag(24);
    let terminal_disposition = suffix_cursor.i32("terminal disposition");
    suffix_cursor.tag(25);
    let correction_revision = suffix_cursor.u64("correction revision");
    suffix_cursor.tag(26);
    let detail = suffix_cursor.string("detail");
    suffix_cursor.tag(27);
    suffix_cursor.literal(b"terminal-proof-follows\0", "terminal proof marker");
    suffix_cursor.tag(28);
    assert_eq!(suffix_cursor.u8("terminal proof placeholder"), 0);
    suffix_cursor.finish();

    MatrixEvidenceAnchors {
        request_identity_sha256,
        admission_ordinal,
        normalized_logical_timeout_ms,
        runtime_generation,
        activated_mapping_backend_generation,
        correction_owner_backend_generation,
        session_id,
        map_name,
        session_generation,
        map_frame_id,
        odom_frame_id,
        body_frame_id,
        activation_identity_sha256,
        target_receipt_sha256,
        asset_manifest_sha256,
        asset_bundle_sha256,
        source_policy_version,
        source_policy_sha256,
        deadline_policy_version,
        deadline_policy_sha256,
        algorithm_options_version,
        algorithm_options_sha256,
        target,
        frames,
        aggregate_input_points,
        post_transform_points,
        emitted_points,
        boundary_key,
        source_length,
        source_sha256,
        algorithm,
        candidate,
        terminal_disposition,
        correction_revision,
        detail,
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_canonical_source_from_playback(
    playback: &MatrixPlayback,
    frames: &[MatrixEvidenceFrame],
) -> Vec<u8> {
    let by_sequence = playback
        .source_frames
        .iter()
        .enumerate()
        .map(|(index, input)| {
            (
                u64::try_from(index + 1).expect("matrix fixture sequence fits u64"),
                input,
            )
        })
        .collect::<std::collections::BTreeMap<_, _>>();
    let mut ordered = Vec::new();
    for frame in frames {
        let input = by_sequence
            .get(&frame.key.registered_cloud_source_sequence)
            .unwrap_or_else(|| {
                panic!(
                    "matrix fixture omitted evidence frame sequence {}",
                    frame.key.registered_cloud_source_sequence
                )
            });
        assert_eq!(
            u64::try_from(input.cloud.points.len()).expect("matrix point count fits u64"),
            frame.point_count
        );
        for point in &input.cloud.points {
            let sensor_point = nalgebra::Point3::new(
                f64::from(point.x as f32),
                f64::from(point.y as f32),
                f64::from(point.z as f32),
            );
            let body_point = nalgebra::Point3::new(
                -sensor_point.y + 0.25,
                sensor_point.x - 0.5,
                sensor_point.z + 1.0,
            );
            let odom_point = frame.odom_from_body.isometry().transform_point(&body_point);
            assert!(odom_point.coords.iter().all(|value| value.is_finite()));
            ordered.push(odom_point);
        }
    }
    let emitted_count = ordered.len().min(100_000);
    assert!((50_000..=100_000).contains(&emitted_count));
    let mut canonical = Vec::with_capacity(8 + emitted_count * 24);
    canonical.extend_from_slice(
        &u64::try_from(emitted_count)
            .expect("matrix emitted point count fits u64")
            .to_be_bytes(),
    );
    for index in 0..emitted_count {
        let source_index = index
            .checked_mul(ordered.len())
            .expect("matrix source sampling index does not overflow")
            / emitted_count;
        let point = ordered[source_index];
        canonical.extend_from_slice(&point.x.to_be_bytes());
        canonical.extend_from_slice(&point.y.to_be_bytes());
        canonical.extend_from_slice(&point.z.to_be_bytes());
    }
    canonical
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_composite_evidence_sha256(
    prefix: &[u8],
    source_length: usize,
    source_sha256: [u8; 32],
    suffix: &[u8],
) -> [u8; 32] {
    let full_length = prefix
        .len()
        .checked_add(source_length)
        .and_then(|length| length.checked_add(suffix.len()))
        .expect("matrix evidence full length does not overflow");
    let mut composite = MATRIX_EVIDENCE_COMPOSITE_DOMAIN_V1.to_vec();
    composite.extend_from_slice(&1_u32.to_be_bytes());
    composite.extend_from_slice(&0_u32.to_be_bytes());
    composite.extend_from_slice(&u64::try_from(full_length).unwrap().to_be_bytes());
    composite.extend_from_slice(&1_u32.to_be_bytes());
    composite.extend_from_slice(&u64::try_from(prefix.len()).unwrap().to_be_bytes());
    composite.extend_from_slice(prefix);
    composite.extend_from_slice(&2_u32.to_be_bytes());
    composite.extend_from_slice(&u64::try_from(source_length).unwrap().to_be_bytes());
    composite.extend_from_slice(&source_sha256);
    composite.extend_from_slice(&3_u32.to_be_bytes());
    composite.extend_from_slice(&u64::try_from(suffix.len()).unwrap().to_be_bytes());
    composite.extend_from_slice(suffix);
    nuwax_codec::sha256_bytes(&composite)
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_validate_evidence_proof(
    result: &native_fixture_proto::RelocalizeResult,
    candidate: &DecodedCorrectionCandidate,
    playback: &MatrixPlayback,
    scenario: &str,
) -> [u8; 32] {
    let expected = matrix_digest32(&result.evidence_sha256, "matrix evidence digest");
    let disclosure = result.evidence_disclosure_v1.as_slice();
    assert!(
        !disclosure.is_empty() && disclosure.len() <= 20_736,
        "{scenario}: field 30 is outside the retained disclosure bound"
    );
    let mut cursor = MatrixProofCursor::new(disclosure, "disclosure");
    cursor.literal(MATRIX_EVIDENCE_DISCLOSURE_DOMAIN_V1, "domain");
    assert_eq!(cursor.u32("version"), 1, "{scenario}");
    match cursor.u8("mode") {
        0 => {
            let preimage_length = usize::try_from(cursor.u64("inline preimage length"))
                .expect("matrix inline preimage length fits usize");
            assert!(preimage_length <= 4_096, "{scenario}");
            let preimage = cursor.take_slice(preimage_length, "inline preimage");
            assert_eq!(nuwax_codec::sha256_bytes(preimage), expected, "{scenario}");
            cursor.finish();
        }
        1 => {
            let full_length = usize::try_from(cursor.u64("full preimage length"))
                .expect("matrix full evidence length fits usize");
            let prefix_length = usize::try_from(cursor.u64("prefix length"))
                .expect("matrix evidence prefix length fits usize");
            let source_length = usize::try_from(cursor.u64("source length"))
                .expect("matrix evidence source length fits usize");
            let suffix_length = usize::try_from(cursor.u64("suffix length"))
                .expect("matrix evidence suffix length fits usize");
            assert!(prefix_length <= 16_384, "{scenario}");
            assert!((1..=2_400_008).contains(&source_length), "{scenario}");
            assert!(suffix_length <= 4_096, "{scenario}");
            assert_eq!(full_length, prefix_length + source_length + suffix_length);
            let prefix = cursor.take_slice(prefix_length, "prefix");
            let disclosed_source_sha256 = cursor.take::<32>("source digest");
            let suffix = cursor.take_slice(suffix_length, "suffix");
            cursor.finish();
            let mut source_commitment = MATRIX_SOURCE_COMMITMENT_DOMAIN_V1.to_vec();
            source_commitment.extend_from_slice(&1_u32.to_be_bytes());
            source_commitment
                .extend_from_slice(&u64::try_from(source_length).unwrap().to_be_bytes());
            source_commitment.extend_from_slice(&disclosed_source_sha256);
            assert!(
                prefix.ends_with(&source_commitment),
                "{scenario}: evidence prefix does not end in the literal source commitment"
            );
            let anchors =
                matrix_parse_mode1_evidence(prefix, source_length, disclosed_source_sha256, suffix);
            let source = matrix_canonical_source_from_playback(playback, &anchors.frames);
            assert_eq!(source.len(), source_length, "{scenario}");
            let rebuilt_source_sha256 = nuwax_codec::sha256_bytes(&source);
            assert_eq!(
                rebuilt_source_sha256, disclosed_source_sha256,
                "{scenario}: disclosure source digest is not bound to the real fixture frames"
            );
            assert_eq!(anchors.source_length, source_length);
            assert_eq!(anchors.source_sha256, rebuilt_source_sha256);
            assert_eq!(
                usize::try_from(anchors.aggregate_input_points).unwrap(),
                anchors
                    .frames
                    .iter()
                    .map(|frame| usize::try_from(frame.point_count).unwrap())
                    .sum::<usize>()
            );
            assert_eq!(
                anchors.post_transform_points,
                anchors.aggregate_input_points
            );
            assert_eq!(
                usize::try_from(anchors.emitted_points).unwrap(),
                (source_length - 8) / 24
            );
            assert_eq!(
                anchors.request_identity_sha256,
                matrix_digest32(&result.request_identity_sha256, "result request identity")
            );
            assert!(anchors.admission_ordinal.is_some());
            assert_eq!(
                anchors.normalized_logical_timeout_ms,
                Some(result.logical_timeout_ms)
            );
            assert_eq!(anchors.runtime_generation, result.runtime_generation);
            assert_eq!(
                anchors.activated_mapping_backend_generation,
                result.backend_generation
            );
            assert_eq!(anchors.session_id, result.session_id);
            assert_eq!(anchors.map_name, result.map_name);
            assert_eq!(anchors.session_generation, result.session_generation);
            assert_eq!(
                anchors.activation_identity_sha256,
                matrix_digest32(
                    &result.activation_identity_sha256,
                    "result activation identity"
                )
            );
            assert_eq!(
                anchors.target_receipt_sha256,
                matrix_digest32(&result.target_receipt_sha256, "result target receipt")
            );
            assert_eq!(
                anchors.asset_manifest_sha256,
                matrix_digest32(&result.asset_manifest_sha256, "result asset manifest")
            );
            assert_eq!(
                anchors.asset_bundle_sha256,
                matrix_digest32(&result.asset_bundle_sha256, "result asset bundle")
            );
            assert_eq!(anchors.source_policy_version, result.source_policy_version);
            assert_eq!(
                anchors.source_policy_sha256,
                matrix_digest32(&result.source_policy_sha256, "result source policy")
            );
            assert_eq!(
                anchors.deadline_policy_version,
                result.deadline_policy_version
            );
            assert_eq!(
                anchors.deadline_policy_sha256,
                matrix_digest32(&result.deadline_policy_sha256, "result deadline policy")
            );
            assert_eq!(
                anchors.algorithm_options_version,
                result.algorithm_options_version
            );
            assert_eq!(
                anchors.algorithm_options_sha256,
                matrix_digest32(&result.algorithm_options_sha256, "result algorithm policy")
            );
            assert_eq!(anchors.terminal_disposition, result.disposition);
            assert_eq!(anchors.correction_revision, result.correction_revision);
            assert_eq!(anchors.detail, result.detail);
            assert_eq!(anchors.target.runtime_generation, result.runtime_generation);
            assert_eq!(anchors.target.session_id, result.session_id);
            assert_eq!(anchors.target.map_name, result.map_name);
            assert_eq!(anchors.target.session_generation, result.session_generation);
            assert_eq!(anchors.target.backend_generation, result.backend_generation);
            assert_eq!(
                anchors.target.activation_identity_sha256,
                matrix_digest32(
                    &result.activation_identity_sha256,
                    "target activation identity"
                )
            );
            assert_eq!(
                anchors.target.target_receipt_sha256,
                matrix_digest32(&result.target_receipt_sha256, "target receipt")
            );
            assert_eq!(
                anchors.target.asset_manifest_sha256,
                Some(matrix_digest32(
                    &result.asset_manifest_sha256,
                    "target manifest"
                ))
            );
            assert_eq!(
                anchors.target.asset_bundle_sha256,
                Some(matrix_digest32(
                    &result.asset_bundle_sha256,
                    "target bundle"
                ))
            );
            assert_eq!(anchors.target.frame_id, anchors.map_frame_id);
            assert_eq!(anchors.target.source_revision, result.target_cloud_revision);
            assert_eq!(
                anchors.target.xyz_sha256,
                matrix_digest32(&result.target_cloud_sha256, "target cloud")
            );
            assert!(anchors.algorithm.accepted);
            let evidence_candidate = anchors
                .candidate
                .as_ref()
                .expect("matrix applied evidence must contain a candidate");
            let candidate_key = matrix_candidate_frame_key(candidate);
            assert_eq!(evidence_candidate.backend, candidate.backend);
            assert_eq!(
                evidence_candidate.transform_semantics,
                candidate.transform_semantics
            );
            assert_eq!(
                evidence_candidate.runtime_generation,
                candidate.runtime_generation
            );
            assert_eq!(
                evidence_candidate.backend_generation,
                candidate.backend_generation
            );
            assert_eq!(
                anchors.correction_owner_backend_generation,
                candidate.backend_generation
            );
            assert_eq!(
                evidence_candidate.candidate_sequence,
                candidate.candidate_sequence
            );
            assert_eq!(
                evidence_candidate.mapping_session_id,
                candidate.mapping_session_id
            );
            assert_eq!(
                evidence_candidate.mapping_session_generation,
                candidate.mapping_session_generation
            );
            assert_eq!(
                evidence_candidate.map_frame_generation,
                candidate.map_frame_generation
            );
            assert_eq!(evidence_candidate.frame_key, candidate_key);
            assert_eq!(anchors.boundary_key, candidate_key);
            assert_eq!(
                evidence_candidate.correction_lease_boot_nonce,
                candidate.correction_lease_boot_nonce
            );
            assert_eq!(
                evidence_candidate.correction_lease_generation,
                candidate.correction_lease_generation
            );
            assert_eq!(
                evidence_candidate.transform.translation_m,
                candidate.translation_m
            );
            assert_eq!(
                evidence_candidate.transform.rotation_xyzw,
                candidate.rotation_xyzw
            );
            assert_eq!(evidence_candidate.payload_sha256, candidate.payload_sha256);
            let algorithm_correction = anchors
                .algorithm
                .correction
                .expect("matrix applied algorithm must disclose its correction");
            assert_eq!(
                algorithm_correction.encoded,
                evidence_candidate.transform.encoded
            );
            let boundary = anchors
                .frames
                .last()
                .expect("matrix evidence boundary frame");
            assert_eq!(boundary.key, candidate_key);
            assert_eq!(boundary.mapping_session_id, candidate.mapping_session_id);
            assert_eq!(
                boundary.mapping_session_generation,
                candidate.mapping_session_generation
            );
            assert_eq!(
                boundary.map_frame_generation,
                candidate.map_frame_generation
            );
            assert_eq!(boundary.point_count, candidate.registered_cloud_point_count);
            assert_eq!(
                boundary.envelope_sha256,
                candidate.registered_cloud_envelope_sha256
            );
            assert_eq!(boundary.body_points_sha256, candidate.body_points_sha256);
            assert_eq!(
                boundary.odom_source_timestamp_ns,
                candidate.correlated_odom_source_timestamp_ns
            );
            assert_eq!(
                boundary.odom_pose_sha256,
                candidate.correlated_odom_pose_sha256
            );
            assert_eq!(
                boundary.map_source_timestamp_ns,
                candidate.committed_map_source_timestamp_ns
            );
            assert_eq!(boundary.map_width, candidate.committed_map_width);
            assert_eq!(boundary.map_height, candidate.committed_map_height);
            assert_eq!(
                boundary.map_resolution_m.to_bits(),
                candidate.committed_map_resolution_m.to_bits()
            );
            assert_eq!(
                boundary.map_origin_x_m.to_bits(),
                candidate.committed_map_origin_x_m.to_bits()
            );
            assert_eq!(
                boundary.map_origin_y_m.to_bits(),
                candidate.committed_map_origin_y_m.to_bits()
            );
            assert_eq!(
                boundary.map_payload_sha256,
                candidate.committed_map_payload_sha256
            );
            assert_eq!(
                boundary.map_boundary_sha256,
                candidate.committed_map_boundary_sha256
            );
            assert_eq!(boundary.points_frame_id, candidate.points_frame_id);
            assert_eq!(candidate.map_frame_id, anchors.map_frame_id);
            assert_eq!(candidate.odom_frame_id, anchors.odom_frame_id);
            assert_eq!(candidate.body_frame_id, anchors.body_frame_id);
            assert_eq!(
                matrix_composite_evidence_sha256(
                    prefix,
                    source_length,
                    rebuilt_source_sha256,
                    suffix,
                ),
                expected,
                "{scenario}: independently rebuilt evidence composite did not match field 25"
            );
        }
        mode => panic!("{scenario}: unsupported field 30 disclosure mode {mode}"),
    }
    expected
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_assert_evidence_golden() -> [u8; 32] {
    const EXPECTED_SOURCE_SHA256: [u8; 32] = [
        0x38, 0x7a, 0xb2, 0xa6, 0x58, 0x5f, 0x41, 0xb1, 0x13, 0x53, 0xaf, 0xc1, 0x15, 0x3a, 0x68,
        0x20, 0x18, 0x5c, 0x29, 0xe8, 0x2c, 0xff, 0x1f, 0x14, 0x19, 0xe6, 0xb2, 0xcc, 0x2a, 0xb7,
        0x9a, 0x98,
    ];
    const EXPECTED_FIELD25: [u8; 32] = [
        0x9b, 0xcf, 0x73, 0x0f, 0xd6, 0x9d, 0xc9, 0x4f, 0xf8, 0x20, 0x71, 0x24, 0x4b, 0x25, 0x1e,
        0xf8, 0xdf, 0xb9, 0x6a, 0x52, 0xe8, 0xa1, 0xfa, 0x3c, 0x4e, 0x5e, 0x61, 0xf4, 0xe8, 0x1b,
        0x1d, 0xc1,
    ];
    let mut source = Vec::with_capacity(8 + 50_000 * 24);
    source.extend_from_slice(&50_000_u64.to_be_bytes());
    for index in 0..50_000_usize {
        let unique_index = index % 512;
        let x = unique_index % 32;
        let y = unique_index / 32;
        let hash = (unique_index as u32)
            .wrapping_mul(0x9e37_79b9)
            .rotate_left(13)
            ^ 0xa5a5_5a5a;
        let x_voxel = x * 4 + (hash as usize & 1);
        let y_voxel = y * 4 + ((hash as usize >> 1) & 1);
        let z_voxel = (hash as usize >> 2) & 31;
        let sensor_x = f64::from(((x_voxel as f64 + 0.5) * f64::from(0.05_f32)) as f32);
        let sensor_y = f64::from(((y_voxel as f64 + 0.5) * f64::from(0.05_f32)) as f32);
        let sensor_z = f64::from(((z_voxel as f64 + 0.5) * f64::from(0.05_f32)) as f32);
        for value in [-sensor_y + 0.25, sensor_x - 0.5, sensor_z + 1.0] {
            source.extend_from_slice(&value.to_be_bytes());
        }
    }
    let source_sha256 = nuwax_codec::sha256_bytes(&source);
    assert_eq!(
        source_sha256, EXPECTED_SOURCE_SHA256,
        "matrix 50k real-source recipe golden drifted"
    );
    let mut prefix = MATRIX_EVIDENCE_DOMAIN_V1.to_vec();
    prefix.extend_from_slice(&1_u32.to_be_bytes());
    prefix.extend_from_slice(b"matrix-stage-a-50000-source-golden\0");
    prefix.extend_from_slice(MATRIX_SOURCE_COMMITMENT_DOMAIN_V1);
    prefix.extend_from_slice(&1_u32.to_be_bytes());
    prefix.extend_from_slice(&u64::try_from(source.len()).unwrap().to_be_bytes());
    prefix.extend_from_slice(&source_sha256);
    let suffix = b"matrix-stage-a-evidence-suffix-golden\0";
    let field25 = matrix_composite_evidence_sha256(&prefix, source.len(), source_sha256, suffix);
    assert_eq!(field25, EXPECTED_FIELD25, "matrix field 25 golden drifted");
    field25
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_push_present_bytes(out: &mut Vec<u8>, value: &[u8]) {
    if value.is_empty() {
        out.push(0);
    } else {
        out.push(1);
        out.extend_from_slice(&u64::try_from(value.len()).unwrap().to_be_bytes());
        out.extend_from_slice(value);
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_terminal_proof_preimage(result: &native_fixture_proto::RelocalizeResult) -> Vec<u8> {
    let mut out = MATRIX_TERMINAL_PROOF_DOMAIN_V1.to_vec();
    out.extend_from_slice(&1_u32.to_be_bytes());
    macro_rules! tag {
        ($value:expr) => {
            out.extend_from_slice(&$value.to_be_bytes())
        };
    }
    tag!(1_u32);
    out.extend_from_slice(&result.schema_version.to_be_bytes());
    tag!(2_u32);
    matrix_push_string(&mut out, &result.request_id);
    tag!(3_u32);
    matrix_push_present_bytes(&mut out, &result.request_identity_sha256);
    tag!(4_u32);
    out.extend_from_slice(&result.runtime_generation.to_be_bytes());
    tag!(5_u32);
    matrix_push_string(&mut out, &result.session_id);
    tag!(6_u32);
    matrix_push_string(&mut out, &result.map_name);
    tag!(7_u32);
    out.extend_from_slice(&result.session_generation.to_be_bytes());
    tag!(8_u32);
    out.extend_from_slice(&result.backend_generation.to_be_bytes());
    tag!(9_u32);
    matrix_push_present_bytes(&mut out, &result.activation_identity_sha256);
    tag!(10_u32);
    matrix_push_present_bytes(&mut out, &result.target_receipt_sha256);
    tag!(11_u32);
    out.extend_from_slice(&result.target_cloud_revision.to_be_bytes());
    tag!(12_u32);
    matrix_push_present_bytes(&mut out, &result.target_cloud_sha256);
    tag!(13_u32);
    matrix_push_present_bytes(&mut out, &result.asset_manifest_sha256);
    tag!(14_u32);
    matrix_push_present_bytes(&mut out, &result.asset_bundle_sha256);
    tag!(15_u32);
    out.extend_from_slice(&result.source_policy_version.to_be_bytes());
    tag!(16_u32);
    matrix_push_present_bytes(&mut out, &result.source_policy_sha256);
    tag!(17_u32);
    out.extend_from_slice(&result.deadline_policy_version.to_be_bytes());
    tag!(18_u32);
    matrix_push_present_bytes(&mut out, &result.deadline_policy_sha256);
    tag!(19_u32);
    out.extend_from_slice(&result.logical_timeout_ms.to_be_bytes());
    tag!(20_u32);
    out.extend_from_slice(&result.algorithm_options_version.to_be_bytes());
    tag!(21_u32);
    matrix_push_present_bytes(&mut out, &result.algorithm_options_sha256);
    tag!(22_u32);
    out.extend_from_slice(&result.disposition.to_be_bytes());
    tag!(23_u32);
    out.extend_from_slice(&result.candidate_sequence.to_be_bytes());
    tag!(24_u32);
    matrix_push_present_bytes(&mut out, &result.candidate_payload_sha256);
    tag!(25_u32);
    matrix_push_present_bytes(&mut out, &result.evidence_sha256);
    tag!(26_u32);
    out.extend_from_slice(&result.correction_revision.to_be_bytes());
    tag!(27_u32);
    out.push(0);
    tag!(28_u32);
    matrix_push_string(&mut out, &result.detail);
    out
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_assert_terminal_proof_golden() -> [u8; 32] {
    const EXPECTED: [u8; 32] = [
        0xc6, 0x4d, 0xac, 0x3b, 0xa8, 0x05, 0x65, 0x5b, 0x26, 0x24, 0x90, 0xa0, 0xdb, 0x37, 0x65,
        0x52, 0xbf, 0xe4, 0x1c, 0x10, 0x43, 0xa7, 0x49, 0xea, 0xf4, 0x81, 0x9e, 0xf3, 0x06, 0xa0,
        0x03, 0x58,
    ];
    let result = native_fixture_proto::RelocalizeResult {
        schema_version: 1,
        request_id: "req-golden".to_owned(),
        request_identity_sha256: vec![1; 32],
        runtime_generation: 2,
        session_id: "session-golden".to_owned(),
        map_name: "map-golden".to_owned(),
        session_generation: 3,
        backend_generation: 4,
        activation_identity_sha256: vec![5; 32],
        target_receipt_sha256: vec![6; 32],
        target_cloud_revision: 7,
        target_cloud_sha256: vec![8; 32],
        asset_manifest_sha256: vec![9; 32],
        asset_bundle_sha256: vec![10; 32],
        source_policy_version: 1,
        source_policy_sha256: vec![11; 32],
        deadline_policy_version: 1,
        deadline_policy_sha256: vec![12; 32],
        logical_timeout_ms: 30_000,
        algorithm_options_version: 1,
        algorithm_options_sha256: vec![13; 32],
        disposition: native_fixture_proto::RelocalizeDisposition::Applied as i32,
        candidate_sequence: 14,
        candidate_payload_sha256: vec![15; 32],
        evidence_sha256: vec![16; 32],
        correction_revision: 17,
        terminal_proof_sha256: vec![0xaa; 32],
        detail: "APPLIED".to_owned(),
        candidate_payload_preimage_v1: vec![0x29; 19],
        evidence_disclosure_v1: vec![0x30; 23],
    };
    let preimage = matrix_terminal_proof_preimage(&result);
    assert_eq!(
        preimage.len(),
        745,
        "matrix terminal proof golden length drifted"
    );
    assert_eq!(
        nuwax_codec::sha256_bytes(&preimage),
        EXPECTED,
        "matrix terminal proof cross-language golden drifted"
    );
    EXPECTED
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn matrix_validate_terminal_proof(
    result: &native_fixture_proto::RelocalizeResult,
    scenario: &str,
) -> [u8; 32] {
    let expected = nuwax_codec::sha256_bytes(&matrix_terminal_proof_preimage(result));
    assert_eq!(
        expected,
        matrix_digest32(
            &result.terminal_proof_sha256,
            "matrix terminal proof digest"
        ),
        "{scenario}: independently recomputed field 27 does not match"
    );
    expected
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_matrix_relocalize_identity_echo(
    request: &native_fixture_proto::RelocalizeRequest,
    result: &native_fixture_proto::RelocalizeResult,
    scenario: &str,
) {
    assert_eq!(result.schema_version, request.schema_version, "{scenario}");
    assert_eq!(result.request_id, request.request_id, "{scenario}");
    assert_eq!(
        result.runtime_generation, request.runtime_generation,
        "{scenario}"
    );
    assert_eq!(result.session_id, request.session_id, "{scenario}");
    assert_eq!(result.map_name, request.map_name, "{scenario}");
    assert_eq!(
        result.session_generation, request.session_generation,
        "{scenario}"
    );
    assert_eq!(
        result.backend_generation, request.backend_generation,
        "{scenario}"
    );
    assert_eq!(
        result.activation_identity_sha256, request.activation_identity_sha256,
        "{scenario}"
    );
    assert_eq!(
        result.target_receipt_sha256, request.target_receipt_sha256,
        "{scenario}"
    );
    assert_eq!(
        result.target_cloud_revision, request.target_cloud_revision,
        "{scenario}"
    );
    assert_eq!(
        result.target_cloud_sha256, request.target_cloud_sha256,
        "{scenario}"
    );
    assert_eq!(
        result.asset_manifest_sha256, request.asset_manifest_sha256,
        "{scenario}"
    );
    assert_eq!(
        result.asset_bundle_sha256, request.asset_bundle_sha256,
        "{scenario}"
    );
    assert_eq!(
        result.source_policy_version, request.source_policy_version,
        "{scenario}"
    );
    assert_eq!(
        result.source_policy_sha256, request.source_policy_sha256,
        "{scenario}"
    );
    assert_eq!(
        result.deadline_policy_version, request.deadline_policy_version,
        "{scenario}"
    );
    assert_eq!(
        result.deadline_policy_sha256, request.deadline_policy_sha256,
        "{scenario}"
    );
    assert_eq!(
        result.logical_timeout_ms, request.logical_timeout_ms,
        "{scenario}"
    );
    assert_eq!(
        result.algorithm_options_version, request.algorithm_options_version,
        "{scenario}"
    );
    assert_eq!(
        result.algorithm_options_sha256, request.algorithm_options_sha256,
        "{scenario}"
    );
    assert_eq!(result.request_identity_sha256.len(), 32, "{scenario}");
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn start_proven_active_relocalization(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    request: &native_fixture_proto::RelocalizeRequest,
    scenario: &str,
) -> tokio::task::JoinHandle<native_fixture_proto::RelocalizeResult> {
    let mut active_command = None;
    runtime.block_on(async {
        for attempt in 0..8_u32 {
            let mut command_client = client.clone();
            let command_request = request.clone();
            let mut command = tokio::spawn(async move {
                command_client
                    .relocalize(command_request)
                    .await
                    .expect("matrix Relocalize command RPC")
                    .into_inner()
            });
            tokio::task::yield_now().await;
            tokio::task::yield_now().await;
            let mut probe = request.clone();
            probe.request_id = format!("{}-busy-{attempt}", request.request_id);
            match tokio::time::timeout(Duration::from_millis(500), client.relocalize(probe)).await {
                Ok(Ok(response)) => {
                    let result = response.into_inner();
                    if result.disposition
                        == native_fixture_proto::RelocalizeDisposition::Busy as i32
                    {
                        assert_eq!(result.candidate_sequence, 0, "{scenario}");
                        assert!(result.terminal_proof_sha256.is_empty(), "{scenario}");
                        active_command = Some(command);
                        return;
                    }
                    panic!(
                        "{scenario}: BUSY probe returned disposition={} detail={}",
                        result.disposition, result.detail
                    );
                }
                Ok(Err(error)) => panic!("{scenario}: BUSY probe RPC failed: {error}"),
                Err(_) => {
                    let raced = tokio::time::timeout(Duration::from_millis(500), &mut command)
                        .await
                        .unwrap_or_else(|_| {
                            panic!("{scenario}: neither command nor BUSY probe settled")
                        })
                        .unwrap_or_else(|error| panic!("{scenario}: command join failed: {error}"));
                    assert_eq!(
                        raced.disposition,
                        native_fixture_proto::RelocalizeDisposition::Busy as i32,
                        "{scenario}: probe won admission but command was not BUSY"
                    );
                    tokio::task::yield_now().await;
                }
            }
        }
        panic!("{scenario}: could not prove an active Relocalize command")
    });
    active_command.expect("active Relocalize command")
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn matrix_track_selected_native_pgo(
    cycle: &mut MatrixCycle,
    scenario: &str,
) -> (u32, u64, u32, u64) {
    let geometry_pid = cycle.geometry_pid();
    let expected = nuwax_nav_pgo_bridge::native_executable()
        .canonicalize()
        .expect("canonicalize selected native PGO executable");
    let deadline = Instant::now() + NATIVE_FIXTURE_TIMEOUT;
    loop {
        let children = direct_child_pids(geometry_pid);
        let native = children
            .iter()
            .copied()
            .find(|pid| process_executable(*pid).as_ref() == Some(&expected));
        let guardian = children.iter().copied().find(|pid| {
            process_executable(*pid)
                .as_deref()
                .and_then(Path::file_name)
                == Some(std::ffi::OsStr::new("nuwax-group-guardian"))
        });
        if let (Some(pid), Some(guardian_pid)) = (native, guardian) {
            let start_time = process_start_time(pid)
                .unwrap_or_else(|| panic!("{scenario}: native PGO child disappeared"));
            let guardian_start_time = process_start_time(guardian_pid)
                .unwrap_or_else(|| panic!("{scenario}: native PGO guardian disappeared"));
            cycle.track_native_pgo(pid, start_time, scenario);
            return (pid, start_time, guardian_pid, guardian_start_time);
        }
        assert!(
            Instant::now() < deadline,
            "{scenario}: selected native PGO child did not start"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn assert_matrix_native_pgo_not_spawned(cycle: &mut MatrixCycle, scenario: &str) {
    let geometry_pid = cycle.geometry_pid();
    let expected = nuwax_nav_pgo_bridge::native_executable()
        .canonicalize()
        .expect("canonicalize unselected native PGO executable");
    assert!(
        direct_child_pids(geometry_pid)
            .into_iter()
            .all(|pid| process_executable(pid).as_ref() != Some(&expected)),
        "{scenario}: unselected native PGO executable was spawned"
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", unix))]
fn assert_relocalization_owner_unselected(
    runtime: &tokio::runtime::Runtime,
    client: &mut GeometryClient,
    scenario: &str,
) {
    let error = runtime
        .block_on(client.relocalize(native_fixture_proto::RelocalizeRequest::default()))
        .expect_err("unselected relocalization owner must reject the RPC");
    assert_eq!(error.code(), tonic::Code::FailedPrecondition, "{scenario}");
    assert_eq!(
        error.message(),
        "RELOCALIZATION_OWNER_NOT_SELECTED",
        "{scenario}"
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn run_non_relocalization_matrix_profile(profile: MatrixProfile) {
    let scenario = profile.scenario();
    let runtime = tokio::runtime::Runtime::new().expect("create matrix runtime");
    let probe = ProbeDir::new(&scenario);
    let playback = matrix_playback(profile, &probe);
    let permit_path = probe.path().join("matrix-publish-permit");
    write_publish_permit(&permit_path, 0);
    let mut observers = NativeObservers::new(&runtime, probe.lcm_url());
    let mut pgo = NativePgoObservers::new(&runtime, probe.lcm_url(), 1);
    let mut cycle = spawn_matrix_cycle(
        &runtime,
        &mut observers,
        &probe,
        profile,
        &playback,
        &permit_path,
    );
    assert_eq!(cycle.generation(), 1, "{scenario}");
    let native_process = if profile.correction == "native_pgo" {
        Some(matrix_track_selected_native_pgo(&mut cycle, &scenario))
    } else {
        assert_matrix_native_pgo_not_spawned(&mut cycle, &scenario);
        None
    };
    let (mut client, _started) = matrix_prime_mapping(
        &runtime,
        &mut observers,
        &probe,
        &playback,
        &permit_path,
        &mut cycle,
    );
    assert_relocalization_owner_unselected(&runtime, &mut client, &scenario);

    let mapping_before_effect;
    let occupancy_before_effect;
    let occupancy_file_before_effect;
    let cloud_file_before_effect;
    let mut selected_candidate = None;
    let mut selected_native_exchange = None;
    match profile.correction {
        "none" => {
            let published = matrix_publish_active_frame(
                &runtime,
                &mut observers,
                profile,
                &playback,
                &permit_path,
                &mut cycle,
                2,
            );
            let _ = (published.tf, published.global_map);
            mapping_before_effect = get_mapping_state(&runtime, &mut client, &scenario);
            occupancy_before_effect = get_occupancy_data(&runtime, &mut client, &scenario);
            occupancy_file_before_effect =
                fs::read(probe.path().join("occupancy.mmap")).expect("read matrix occupancy");
            cloud_file_before_effect =
                fs::read(probe.path().join("cloud-map.mmap")).expect("read matrix cloud map");
            assert_stream_silent(&runtime, &mut pgo.request, "none selected native request");
            assert_stream_silent(&runtime, &mut pgo.result, "none selected native result");
            assert_stream_silent(
                &runtime,
                &mut pgo.library_candidate,
                "none selected library candidate",
            );
        }
        "library_pgo" => {
            assert_stream_silent(
                &runtime,
                &mut pgo.library_candidate,
                "library bootstrap candidate",
            );
            for sequence in 2..=10_u32 {
                let _published = matrix_publish_active_frame(
                    &runtime,
                    &mut observers,
                    profile,
                    &playback,
                    &permit_path,
                    &mut cycle,
                    sequence,
                );
                let timeout = if sequence == 10 {
                    NATIVE_FIXTURE_TIMEOUT
                } else {
                    Duration::from_millis(25)
                };
                if let Some(frame) = next_frame_within(
                    &runtime,
                    &mut pgo.library_candidate,
                    timeout,
                    "matrix library candidate",
                ) {
                    selected_candidate = Some(frame);
                    break;
                }
            }
            let candidate_frame = selected_candidate.as_ref().unwrap_or_else(|| {
                panic!("{scenario}: selected library PGO produced no candidate")
            });
            let candidate = decode_correction_candidate(candidate_frame);
            assert_eq!(candidate.backend, 1, "{scenario}");
            assert_eq!(
                candidate.runtime_generation,
                cycle.generation(),
                "{scenario}"
            );
            mapping_before_effect = get_mapping_state(&runtime, &mut client, &scenario);
            occupancy_before_effect = get_occupancy_data(&runtime, &mut client, &scenario);
            occupancy_file_before_effect =
                fs::read(probe.path().join("occupancy.mmap")).expect("read matrix occupancy");
            cloud_file_before_effect =
                fs::read(probe.path().join("cloud-map.mmap")).expect("read matrix cloud map");
            let corrected = next_frame(&runtime, &mut observers.tf, "matrix library corrected TF");
            assert!(
                map_from_odom_in_frame(&corrected).header.seq > 0,
                "{scenario}"
            );
            assert_stream_silent(&runtime, &mut pgo.request, "library native request");
            assert_stream_silent(&runtime, &mut pgo.result, "library native result");
            assert_stream_silent(&runtime, &mut pgo.status, "library native status");
        }
        "native_pgo" => {
            let first = observe_native_pgo_exchange(
                &runtime,
                &mut pgo,
                cycle.generation(),
                1,
                1,
                "matrix native PGO bootstrap",
            );
            assert_eq!(
                first.result.result_disposition,
                native_pgo::NATIVE_PGO_DISPOSITION_REJECTED,
                "{scenario}"
            );
            for sequence in 2..=4_u32 {
                let _published = matrix_publish_active_frame(
                    &runtime,
                    &mut observers,
                    profile,
                    &playback,
                    &permit_path,
                    &mut cycle,
                    sequence,
                );
                let exchange = observe_native_pgo_exchange(
                    &runtime,
                    &mut pgo,
                    cycle.generation(),
                    i64::from(sequence),
                    i64::from(sequence),
                    "matrix selected native PGO exchange",
                );
                if sequence == 4 {
                    assert_eq!(
                        exchange.result.result_disposition,
                        native_pgo::NATIVE_PGO_DISPOSITION_CORRECTION,
                        "{scenario}: selected native PGO did not correct"
                    );
                    selected_native_exchange = Some(exchange);
                } else {
                    assert_eq!(
                        exchange.result.result_disposition,
                        native_pgo::NATIVE_PGO_DISPOSITION_NO_CANDIDATE,
                        "{scenario}"
                    );
                }
            }
            mapping_before_effect = get_mapping_state(&runtime, &mut client, &scenario);
            occupancy_before_effect = get_occupancy_data(&runtime, &mut client, &scenario);
            occupancy_file_before_effect =
                fs::read(probe.path().join("occupancy.mmap")).expect("read matrix occupancy");
            cloud_file_before_effect =
                fs::read(probe.path().join("cloud-map.mmap")).expect("read matrix cloud map");
            let corrected = next_frame(&runtime, &mut observers.tf, "matrix native corrected TF");
            assert!(
                map_from_odom_in_frame(&corrected).header.seq > 0,
                "{scenario}"
            );
            assert_stream_silent(
                &runtime,
                &mut pgo.library_candidate,
                "native selected library candidate",
            );
        }
        value => panic!("unsupported non-relocalization correction {value}"),
    }

    assert_eq!(
        get_mapping_state(&runtime, &mut client, &scenario),
        mapping_before_effect,
        "{scenario}: correction changed mapping state"
    );
    assert_eq!(
        get_occupancy_data(&runtime, &mut client, &scenario),
        occupancy_before_effect,
        "{scenario}: correction changed occupancy"
    );
    assert_eq!(
        fs::read(probe.path().join("occupancy.mmap")).expect("read matrix occupancy after effect"),
        occupancy_file_before_effect,
        "{scenario}: correction changed occupancy SHM"
    );
    assert_eq!(
        fs::read(probe.path().join("cloud-map.mmap")).expect("read matrix cloud after effect"),
        cloud_file_before_effect,
        "{scenario}: correction changed cloud-map SHM"
    );

    if let Some(candidate) = selected_candidate {
        runtime
            .block_on(observers.transport.publish(candidate.clone()))
            .expect("replay matrix library candidate");
        let echoed = next_frame(
            &runtime,
            &mut pgo.library_candidate,
            "matrix duplicate library candidate echo",
        );
        assert_eq!(echoed.payload, candidate.payload, "{scenario}");
        assert_stream_silent(
            &runtime,
            &mut observers.tf,
            "duplicate library correction TF",
        );
    }
    if let Some(exchange) = selected_native_exchange {
        runtime
            .block_on(pgo.transport.publish(exchange.result_frame))
            .expect("replay matrix native result");
        assert_stream_silent(
            &runtime,
            &mut observers.tf,
            "duplicate native correction TF",
        );
    }

    let output = finish_matrix_cycle(&runtime, &mut observers, client, cycle, &probe);
    let finalizations = candidate_finalization_events(&output);
    match profile.correction {
        "none" => assert!(finalizations.is_empty(), "{scenario}: {finalizations:?}"),
        "library_pgo" | "native_pgo" => {
            assert_eq!(finalizations.len(), 1, "{scenario}: {finalizations:?}");
            assert_eq!(
                finalizations[0].disposition,
                ObservedCandidateDisposition::Applied,
                "{scenario}"
            );
        }
        _ => unreachable!(),
    }
    if let Some((pid, start_time, guardian_pid, guardian_start_time)) = native_process {
        wait_for_native_pgo_process_group_exit(pid, start_time, &scenario);
        assert_ne!(
            process_start_time(guardian_pid),
            Some(guardian_start_time),
            "{scenario}: native PGO guardian remained after Stop"
        );
    }
    drop(pgo);
    drop(observers);
    drop(runtime);
    let probe_path = probe.path().to_path_buf();
    drop(probe);
    assert!(
        !probe_path.exists(),
        "{scenario}: ProbeDir cleanup retained {}",
        probe_path.display()
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn run_relocalization_matrix_profile(profile: MatrixProfile) {
    let scenario = profile.scenario();
    let _evidence_golden = matrix_assert_evidence_golden();
    let _terminal_golden = matrix_assert_terminal_proof_golden();
    let runtime = tokio::runtime::Runtime::new().expect("create relocalization matrix runtime");
    let probe = ProbeDir::new(&scenario);
    let playback = matrix_playback(profile, &probe);
    let permit_path = probe.path().join("matrix-publish-permit");
    write_publish_permit(&permit_path, 0);
    let mut observers = NativeObservers::new(&runtime, probe.lcm_url());
    let mut pgo = NativePgoObservers::new(&runtime, probe.lcm_url(), 1);
    let mut cycle = spawn_matrix_cycle(
        &runtime,
        &mut observers,
        &probe,
        profile,
        &playback,
        &permit_path,
    );
    assert_matrix_native_pgo_not_spawned(&mut cycle, &scenario);
    let (mut client, started) = matrix_prime_mapping(
        &runtime,
        &mut observers,
        &probe,
        &playback,
        &permit_path,
        &mut cycle,
    );
    assert_eq!(started.runtime_generation, cycle.generation(), "{scenario}");
    let checkpoint = export_matrix_checkpoint(&runtime, &mut client, &scenario);
    assert_eq!(checkpoint.backend_kind, profile.mapping, "{scenario}");
    assert!(
        checkpoint
            .canonical_cloud_checkpoint
            .as_ref()
            .is_some_and(|cloud| cloud.point_count >= 3),
        "{scenario}: exported relocalization target is empty"
    );
    let stopped = stop_mapping(&runtime, &mut client, &started.session_id, &scenario);
    assert!(stopped.success && !stopped.active, "{scenario}");
    let activated = activate_matrix_checkpoint(
        &runtime,
        &mut client,
        profile,
        cycle.generation(),
        &checkpoint,
    );
    let source_cloud_revision = checkpoint
        .canonical_cloud_checkpoint
        .as_ref()
        .expect("matrix activation cloud checkpoint")
        .source_revision;
    let installed_cloud_map = read_cloud_map_shm(&probe.path().join("cloud-map.mmap"));
    let installed_cloud_revision = installed_cloud_map.revision;
    assert_eq!(
        installed_cloud_revision,
        source_cloud_revision
            .checked_add(1)
            .expect("matrix activation source cloud revision exhausted"),
        "{scenario}: activation installed cloud revision did not advance exactly once from the persisted source revision"
    );
    let activation_global_map = matrix_next_global_map_revision(
        &runtime,
        &mut observers.global_map,
        installed_cloud_revision,
        &scenario,
    );
    assert_matrix_activation_global_map(
        &checkpoint,
        installed_cloud_revision,
        &activation_global_map,
        &scenario,
    );
    drain_equivalent_global_map_geometry(
        &runtime,
        &mut observers.global_map,
        &activation_global_map,
        "matrix activation committed global-map backlog",
    );
    let before_apply = capture_stable_matrix_observable_snapshot(
        &runtime,
        &mut client,
        &probe,
        "before_apply",
        &scenario,
    );
    let request = matrix_relocalize_request(profile, &activated, &checkpoint);
    let command = start_proven_active_relocalization(&runtime, &mut client, &request, &scenario);
    let published = matrix_publish_active_frame(
        &runtime,
        &mut observers,
        profile,
        &playback,
        &permit_path,
        &mut cycle,
        2,
    );
    assert!(
        published.global_map.is_none(),
        "{scenario}: relocalization active frame consumed a new global map"
    );
    let result = runtime.block_on(async {
        tokio::time::timeout(Duration::from_secs(45), command)
            .await
            .unwrap_or_else(|_| panic!("{scenario}: Relocalize did not settle"))
            .unwrap_or_else(|error| panic!("{scenario}: Relocalize task failed: {error}"))
    });
    assert_matrix_relocalize_identity_echo(&request, &result, &scenario);
    assert_eq!(
        result.disposition,
        native_fixture_proto::RelocalizeDisposition::Applied as i32,
        "{scenario}: Relocalize failed disposition={} detail={} diagnostic={}",
        result.disposition,
        result.detail,
        matrix_relocalization_failure_diagnostic(&checkpoint, &published),
    );
    assert_eq!(result.candidate_sequence, 1, "{scenario}");
    assert_eq!(result.candidate_payload_sha256.len(), 32, "{scenario}");
    assert_eq!(result.evidence_sha256.len(), 32, "{scenario}");
    assert_eq!(result.correction_revision, 1, "{scenario}");
    assert_eq!(result.terminal_proof_sha256.len(), 32, "{scenario}");
    let candidate = matrix_validate_candidate_proof(&result, &scenario);
    let _evidence_proof = matrix_validate_evidence_proof(&result, &candidate, &playback, &scenario);
    let _terminal_proof = matrix_validate_terminal_proof(&result, &scenario);

    let corrected = next_frame(
        &runtime,
        &mut observers.tf,
        "matrix relocalization corrected TF",
    );
    let corrected_map_from_odom = map_from_odom_in_frame(&corrected);
    assert_eq!(
        corrected_map_from_odom.header.seq,
        i32::try_from(result.correction_revision).expect("correction revision fits i32"),
        "{scenario}"
    );
    let corrected_isometry = transform_isometry(
        corrected_map_from_odom.transform.translation,
        corrected_map_from_odom.transform.rotation_xyzw,
    );
    assert_frame_tree_map_from_odom(
        &get_frame_tree(&runtime, &mut client, &scenario),
        &corrected_isometry,
        schema_transform_timestamp_ns(&corrected_map_from_odom),
        &scenario,
    );
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "relocalization APPLIED global map",
    );
    let after_apply = capture_stable_matrix_observable_snapshot(
        &runtime,
        &mut client,
        &probe,
        "after_apply",
        &scenario,
    );
    assert_matrix_relocalization_observable_transition(
        &before_apply,
        &after_apply,
        &candidate,
        &activation_global_map,
        &scenario,
    );

    let first_result_bytes = result.encode_to_vec();
    let replay = runtime.block_on(async {
        client
            .relocalize(request.clone())
            .await
            .expect("replay matrix Relocalize")
            .into_inner()
    });
    assert_eq!(
        replay, result,
        "{scenario}: same-ID replay was not byte-stable"
    );
    assert_eq!(
        replay.encode_to_vec(),
        first_result_bytes,
        "{scenario}: same-ID replay Prost message bytes drifted"
    );
    assert_stream_silent(
        &runtime,
        &mut observers.tf,
        "same-ID relocalization replay TF",
    );
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "same-ID relocalization replay global map",
    );
    let after_replay = capture_stable_matrix_observable_snapshot(
        &runtime,
        &mut client,
        &probe,
        "after_replay",
        &scenario,
    );
    assert_matrix_observable_snapshots_equal(
        &before_apply,
        &after_replay,
        "same-ID replay relative to before_apply",
        &scenario,
    );
    assert_matrix_observable_snapshots_equal(
        &after_apply,
        &after_replay,
        "same-ID replay relative to after_apply",
        &scenario,
    );
    let mut conflict = request.clone();
    conflict.logical_timeout_ms -= 1;
    let conflict = runtime.block_on(async {
        client
            .relocalize(conflict)
            .await
            .expect("conflicting matrix Relocalize")
            .into_inner()
    });
    assert_eq!(
        conflict.disposition,
        native_fixture_proto::RelocalizeDisposition::ReplayConflict as i32,
        "{scenario}"
    );
    assert_eq!(conflict.candidate_sequence, 0, "{scenario}");
    let mut stale = request.clone();
    stale.request_id = format!("{}-stale", request.request_id);
    stale.session_generation -= 1;
    let stale = runtime.block_on(async {
        client
            .relocalize(stale)
            .await
            .expect("stale matrix Relocalize")
            .into_inner()
    });
    assert_eq!(
        stale.disposition,
        native_fixture_proto::RelocalizeDisposition::StaleSession as i32,
        "{scenario}: stale request was not silent"
    );
    assert_eq!(stale.candidate_sequence, 0, "{scenario}");
    assert_eq!(
        get_mapping_state(&runtime, &mut client, &scenario),
        after_apply.mapping
    );
    assert_eq!(
        get_occupancy_data(&runtime, &mut client, &scenario),
        after_apply.occupancy
    );
    assert_eq!(
        fs::read(probe.path().join("occupancy.mmap")).expect("read occupancy after replay"),
        after_apply.occupancy_mmap,
        "{scenario}: replay/stale traffic changed occupancy SHM"
    );
    assert_eq!(
        fs::read(probe.path().join("cloud-map.mmap")).expect("read cloud map after replay"),
        after_apply.cloud_map_mmap,
        "{scenario}: replay/stale traffic changed cloud-map SHM"
    );
    assert_stream_silent(
        &runtime,
        &mut observers.tf,
        "relocalization replay/stale TF",
    );
    assert_stream_silent(
        &runtime,
        &mut observers.global_map,
        "relocalization replay/stale global map",
    );
    assert_stream_silent(
        &runtime,
        &mut pgo.request,
        "relocalization native PGO request",
    );
    assert_stream_silent(
        &runtime,
        &mut pgo.result,
        "relocalization native PGO result",
    );
    assert_stream_silent(
        &runtime,
        &mut pgo.status,
        "relocalization native PGO status",
    );
    assert_stream_silent(
        &runtime,
        &mut pgo.library_candidate,
        "relocalization library PGO candidate",
    );

    let output = finish_matrix_cycle(&runtime, &mut observers, client, cycle, &probe);
    let finalizations = candidate_finalization_events(&output);
    assert!(finalizations.is_empty(), "{scenario}: {finalizations:?}");
    let _ = published.tf;
    drop(pgo);
    drop(observers);
    drop(runtime);
    let probe_path = probe.path().to_path_buf();
    drop(probe);
    assert!(
        !probe_path.exists(),
        "{scenario}: ProbeDir cleanup retained {}",
        probe_path.display()
    );
}

#[cfg(all(feature = "nuwax-lio-test-child", target_os = "linux"))]
fn run_production_binary_owner_profile_matrix() {
    let filter = std::env::var(MATRIX_ROW_FILTER_ENV).ok();
    let lio = ["device", "fastlio2", "pointlio"];
    let mapping = ["voxel", "raytracing"];
    let correction = ["none", "library_pgo", "native_pgo", "relocalization"];
    let mut declared_rows = 0;
    let mut declared_relocalization_rows = 0;
    let mut executed_rows = 0;
    for lio in lio {
        for mapping in mapping {
            for correction in correction {
                let profile = MatrixProfile {
                    lio,
                    mapping,
                    correction,
                };
                declared_rows += 1;
                if correction == "relocalization" {
                    declared_relocalization_rows += 1;
                }
                if filter
                    .as_deref()
                    .is_some_and(|selected| selected != profile.row_id())
                {
                    continue;
                }
                executed_rows += 1;
                if correction == "relocalization" {
                    run_relocalization_matrix_profile(profile);
                } else {
                    run_non_relocalization_matrix_profile(profile);
                }
            }
        }
    }
    assert_eq!(declared_rows, 24);
    assert_eq!(declared_relocalization_rows, 6);
    if let Some(filter) = filter {
        assert_eq!(
            executed_rows, 1,
            "{MATRIX_ROW_FILTER_ENV} selected no exact matrix row: {filter}"
        );
    } else {
        assert_eq!(executed_rows, 24);
    }
}

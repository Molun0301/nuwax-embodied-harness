use super::*;
use std::ffi::OsString;
use std::sync::{Mutex as StdMutex, MutexGuard};

static ENV_LOCK: StdMutex<()> = StdMutex::new(());

struct EnvRestore {
    values: Vec<(&'static str, Option<OsString>)>,
}

impl EnvRestore {
    fn set(values: &[(&'static str, Option<&str>)]) -> (MutexGuard<'static, ()>, Self) {
        let lock = ENV_LOCK
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let previous = values
            .iter()
            .map(|(key, _)| (*key, env::var_os(key)))
            .collect();
        for (key, value) in values {
            match value {
                Some(value) => env::set_var(key, value),
                None => env::remove_var(key),
            }
        }
        (lock, Self { values: previous })
    }
}

impl Drop for EnvRestore {
    fn drop(&mut self) {
        for (key, value) in self.values.drain(..) {
            match value {
                Some(value) => env::set_var(key, value),
                None => env::remove_var(key),
            }
        }
    }
}

fn identity_json() -> &'static str {
    r#"{"translation_m":[1.0,2.0,3.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#
}

fn rotated_json() -> &'static str {
    r#"{"translation_m":[1.0,2.0,3.0],"rotation_row_major":[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]}"#
}

struct TestExecutable(PathBuf);

impl TestExecutable {
    fn new(script: &str) -> Self {
        static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let directory = env::temp_dir().join(format!(
            "nuwax-native-executable-{}-{}",
            std::process::id(),
            NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
        ));
        std::fs::create_dir(&directory).unwrap();
        let path = directory.join("renamed-program");
        std::fs::write(&path, format!("#!/bin/sh\n{script}\n")).unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o700)).unwrap();
        Self(path)
    }
}

impl Drop for TestExecutable {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
        let _ = std::fs::remove_dir(self.0.parent().unwrap());
    }
}

fn preflight(backend: NativeBackend) -> NativeLioPreflight {
    let probe = TestExecutable::new("exit 77");
    let shell = probe.0.to_str().unwrap();
    let (_lock, _env) = EnvRestore::set(&[
        (NATIVE_EXECUTABLE_ENV, Some(shell)),
        (BODY_FROM_SENSOR_ENV, Some(rotated_json())),
        (BODY_FROM_IMU_ENV, Some(identity_json())),
        (POINTLIO_OPTIONS_ENV, None),
        (LCM_URL_ENV, Some("memq://native-lio-test")),
        (PRIVATE_PREFIX_ENV, Some("/g13")),
        (PAIR_MAX_SKEW_MS_ENV, Some("25")),
        (SCAN_VOXEL_SIZE_M_ENV, None),
        (MAP_VOXEL_SIZE_M_ENV, None),
        (INITIAL_MAP_FRAMES_ENV, None),
    ]);
    NativeLioPreflight::from_env(backend).expect("native preflight")
}

#[test]
fn parses_and_applies_proper_body_from_sensor_rotation() {
    let transform = BodyFromSensorTransform::from_json(rotated_json()).expect("transform");
    assert_eq!(
        transform.transform_point(&Point3::new(4.0, 5.0, 6.0)),
        Point3::new(-4.0, 6.0, 9.0)
    );

    let reflection = r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[-1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#;
    assert!(BodyFromSensorTransform::from_json(reflection).is_err());
    let scaled = r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[2.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#;
    assert!(BodyFromSensorTransform::from_json(scaled).is_err());
}

#[test]
fn independent_ray_origin_preserves_payload_transform_and_legacy_default() {
    let old = BodyFromSensorTransform::from_json(rotated_json()).unwrap();
    assert_eq!(
        old.ray_origin_body(),
        old.transform_point(&Point3::origin())
    );
    let mut json: serde_json::Value = serde_json::from_str(rotated_json()).unwrap();
    json["ray_origin_in_input_m"] = serde_json::json!([0.31502, 0.0, -0.089145]);
    let independent = BodyFromSensorTransform::from_json(&json.to_string()).unwrap();
    assert_eq!(independent.translation_m(), old.translation_m());
    assert_eq!(independent.rotation_row_major(), old.rotation_row_major());
    assert_eq!(
        independent.transform_point(&Point3::new(4., 5., 6.)),
        old.transform_point(&Point3::new(4., 5., 6.))
    );
    assert_eq!(
        independent.ray_origin_body(),
        Point3::new(1., 2.31502, 2.910855)
    );
    for bad in [
        serde_json::json!([1, 2]),
        serde_json::json!([1, 2, null]),
        serde_json::json!(null),
    ] {
        json["ray_origin_in_input_m"] = bad;
        assert!(BodyFromSensorTransform::from_json(&json.to_string()).is_err());
    }
}

#[test]
fn point_options_and_imu_origin_are_captured_before_binding() {
    let probe = TestExecutable::new("exit 77");
    let imu = r#"{"translation_m":[0.1,-0.2,0.3],"rotation_row_major":[1,0,0,0,1,0,0,0,1]}"#;
    let (_lock, _env) = EnvRestore::set(&[
        (NATIVE_EXECUTABLE_ENV, probe.0.to_str()),
        (BODY_FROM_SENSOR_ENV, Some(identity_json())),
        (BODY_FROM_IMU_ENV, Some(imu)),
        (
            POINTLIO_OPTIONS_ENV,
            Some(
                r#"{"point_filter_num":1,"lidar_meas_cov":0.001,"publish_odometry_without_downsample":true}"#,
            ),
        ),
        (LCM_URL_ENV, Some("memq://point-options")),
        (PRIVATE_PREFIX_ENV, Some("/point")),
    ]);
    let captured = NativeLioPreflight::from_env(NativeBackend::PointLio).unwrap();
    env::set_var(POINTLIO_OPTIONS_ENV, r#"{"point_filter_num":7}"#);
    env::set_var(BODY_FROM_IMU_ENV, identity_json());
    let bound = captured.bind("sensor", "body", "odom").unwrap();
    for expected in [
        ["--point_filter_num", "1"],
        ["--imu_origin_in_body", "0.1,-0.2,0.3"],
        ["--publish_odometry_without_downsample", "true"],
    ] {
        assert!(bound.runtime.args.windows(2).any(|x| x == expected));
    }
    env::set_var(POINTLIO_OPTIONS_ENV, r#"{"det_range":0}"#);
    assert!(NativeLioPreflight::from_env(NativeBackend::PointLio).is_err());
    env::set_var(POINTLIO_OPTIONS_ENV, "{}");
    env::remove_var(BODY_FROM_IMU_ENV);
    assert!(NativeLioPreflight::from_env(NativeBackend::PointLio).is_err());
}

#[test]
#[ignore = "由保存配置和本机真实程序驱动，输出实际生产bind的命令行"]
fn export_saved_point_configuration_for_replay() {
    let bound = NativeLioPreflight::from_env(NativeBackend::PointLio)
        .unwrap()
        .bind("utlidar_lidar", "base_link", "odom")
        .unwrap();
    println!(
        "POINT_RUNTIME_ARGS {}",
        serde_json::to_string(&bound.runtime.args).unwrap()
    );
}

#[test]
fn executable_artifact_validation_fails_closed() {
    let (_lock, _env) = EnvRestore::set(&[
        (NATIVE_EXECUTABLE_ENV, Some("relative/native-lio")),
        (BODY_FROM_SENSOR_ENV, Some(identity_json())),
        (BODY_FROM_IMU_ENV, Some(identity_json())),
        (POINTLIO_OPTIONS_ENV, None),
    ]);
    let error = NativeLioPreflight::from_env(NativeBackend::FastLio2)
        .expect_err("relative artifact must fail");
    assert!(error.contains("path_must_be_absolute"));
}

#[test]
fn bind_renders_native_argv_and_lcm_environment() {
    let bound = preflight(NativeBackend::FastLio2)
        .bind("mid360", "base", "odom")
        .expect("bound runtime");
    assert_eq!(
        bound.runtime.env.get("LCM_DEFAULT_URL").map(String::as_str),
        Some("memq://native-lio-test")
    );
    assert_eq!(bound.pair_max_skew, Duration::from_millis(25));
    assert_eq!(
        bound
            .body_from_sensor
            .transform_point(&Point3::new(0.0, 0.0, 0.0)),
        Point3::new(1.0, 2.0, 3.0)
    );
    let args = &bound.runtime.args;
    assert!(args
        .windows(2)
        .any(|pair| { pair == ["--lidar", "/g13/f2/c#sensor_msgs.PointCloud2",] }));
    assert!(args.windows(2).any(|pair| pair == ["--frame_id", "odom"]));
    assert!(args
        .windows(2)
        .any(|pair| pair == ["--child_frame_id", "base"]));
    assert!(args
        .windows(2)
        .any(|pair| pair == ["--sensor_frame_id", "mid360"]));
    assert!(args
        .windows(2)
        .any(|pair| pair == ["--extrinsic_t", "1,2,3"]));
    assert!(args
        .windows(2)
        .any(|pair| pair == ["--extrinsic_r", "0,-1,0,1,0,0,0,0,1"]));
}

#[test]
fn optional_sampling_is_captured_before_binding() {
    let shell = Path::new("/bin/sh").canonicalize().unwrap();
    let (_lock, _env) = EnvRestore::set(&[
        (NATIVE_EXECUTABLE_ENV, shell.to_str()),
        (BODY_FROM_SENSOR_ENV, Some(identity_json())),
        (BODY_FROM_IMU_ENV, Some(identity_json())),
        (POINTLIO_OPTIONS_ENV, None),
        (LCM_URL_ENV, Some("memq://native-sampling-test")),
        (PRIVATE_PREFIX_ENV, Some("/sample")),
        (PAIR_MAX_SKEW_MS_ENV, Some("100")),
        (SCAN_VOXEL_SIZE_M_ENV, Some("0.1")),
        (MAP_VOXEL_SIZE_M_ENV, Some("0.2")),
        (INITIAL_MAP_FRAMES_ENV, Some("7")),
    ]);
    let captured = [NativeBackend::FastLio2, NativeBackend::PointLio].map(|backend| {
        let probe = TestExecutable::new("exit 77");
        env::set_var(NATIVE_EXECUTABLE_ENV, &probe.0);
        NativeLioPreflight::from_env(backend).unwrap()
    });
    env::set_var(SCAN_VOXEL_SIZE_M_ENV, "invalid-after-preflight");
    env::set_var(MAP_VOXEL_SIZE_M_ENV, "0");
    env::set_var(INITIAL_MAP_FRAMES_ENV, "9");
    for (index, preflight) in captured.into_iter().enumerate() {
        let bound = preflight.bind("sensor", "body", "odom").unwrap();
        for pair in [
            ["--filter_size_surf", "0.1"],
            ["--filter_size_map", "0.2"],
            ["--initial_map_frames", "7"],
        ] {
            assert!(bound.runtime.args.windows(2).any(|args| args == pair));
        }
    }
    assert!(NativeLioPreflight::from_env(NativeBackend::PointLio).is_err());
}

#[test]
fn optional_sampling_empty_values_preserve_native_defaults() {
    let probe = TestExecutable::new("exit 77");
    let shell = &probe.0;
    for value in [None, Some(""), Some("   ")] {
        let (_lock, _env) = EnvRestore::set(&[
            (NATIVE_EXECUTABLE_ENV, shell.to_str()),
            (BODY_FROM_SENSOR_ENV, Some(identity_json())),
            (BODY_FROM_IMU_ENV, Some(identity_json())),
            (POINTLIO_OPTIONS_ENV, None),
            (LCM_URL_ENV, Some("memq://native-sampling-test")),
            (PRIVATE_PREFIX_ENV, Some("/sample")),
            (PAIR_MAX_SKEW_MS_ENV, Some("100")),
            (SCAN_VOXEL_SIZE_M_ENV, value),
            (MAP_VOXEL_SIZE_M_ENV, value),
            (INITIAL_MAP_FRAMES_ENV, value),
        ]);
        let bound = NativeLioPreflight::from_env(NativeBackend::PointLio)
            .unwrap()
            .bind("sensor", "body", "odom")
            .unwrap();
        for option in [
            "--filter_size_surf",
            "--filter_size_map",
            "--initial_map_frames",
        ] {
            assert!(!bound.runtime.args.iter().any(|arg| arg == option));
        }
    }
}

#[test]
fn optional_sampling_rejects_invalid_environment_values() {
    let shell = Path::new("/bin/sh").canonicalize().unwrap();
    let (_lock, _env) = EnvRestore::set(&[
        (NATIVE_EXECUTABLE_ENV, shell.to_str()),
        (BODY_FROM_SENSOR_ENV, Some(identity_json())),
        (BODY_FROM_IMU_ENV, Some(identity_json())),
        (POINTLIO_OPTIONS_ENV, None),
        (LCM_URL_ENV, Some("memq://native-sampling-test")),
        (PRIVATE_PREFIX_ENV, Some("/sample")),
        (PAIR_MAX_SKEW_MS_ENV, Some("100")),
        (SCAN_VOXEL_SIZE_M_ENV, None),
        (MAP_VOXEL_SIZE_M_ENV, None),
        (INITIAL_MAP_FRAMES_ENV, None),
    ]);
    for key in [SCAN_VOXEL_SIZE_M_ENV, MAP_VOXEL_SIZE_M_ENV] {
        for value in ["0", "-1", "NaN", "inf", "1e-40", "1e40", "0.1tail", " 0.1 "] {
            env::set_var(key, value);
            assert!(
                NativeLioPreflight::from_env(NativeBackend::PointLio).is_err(),
                "{key}={value}"
            );
        }
        env::remove_var(key);
    }
    for value in ["0", "-1", "1.5", "2147483648", "7tail"] {
        env::set_var(INITIAL_MAP_FRAMES_ENV, value);
        assert!(
            NativeLioPreflight::from_env(NativeBackend::FastLio2).is_err(),
            "initial frames={value}"
        );
    }
}

#[test]
#[ignore = "由临时配置保存和共同环境加载专项驱动，不启动原生进程"]
fn saved_sampling_configuration_reaches_actual_bind() {
    let shell = Path::new("/bin/sh").canonicalize().unwrap();
    let (_lock, _env) = EnvRestore::set(&[
        (NATIVE_EXECUTABLE_ENV, shell.to_str()),
        (BODY_FROM_SENSOR_ENV, Some(identity_json())),
        (BODY_FROM_IMU_ENV, Some(identity_json())),
        (POINTLIO_OPTIONS_ENV, None),
        (LCM_URL_ENV, Some("memq://saved-native-config")),
        (PRIVATE_PREFIX_ENV, Some("/saved")),
        (PAIR_MAX_SKEW_MS_ENV, Some("100")),
    ]);
    // 三个采样值故意不在本测试写入，只接收真实设置保存/环境加载的输出。
    for backend in [NativeBackend::FastLio2, NativeBackend::PointLio] {
        let probe = TestExecutable::new("exit 77");
        env::set_var(NATIVE_EXECUTABLE_ENV, &probe.0);
        let bound = NativeLioPreflight::from_env(backend)
            .unwrap()
            .bind("sensor", "body", "odom")
            .unwrap();
        for pair in [
            ["--filter_size_surf", "0.1"],
            ["--filter_size_map", "0.2"],
            ["--initial_map_frames", "7"],
        ] {
            assert!(bound.runtime.args.windows(2).any(|args| args == pair));
        }
        println!(
            "NATIVE_CONFIG_BIND_JSON {}",
            serde_json::json!({"backend": backend, "args": bound.runtime.args})
        );
    }
}

#[test]
fn preflight_and_bind_do_not_execute_or_require_backend_identity() {
    // 不支持身份查询的程序也应完成配置绑定，且准备阶段不能提前执行它。
    let executable = TestExecutable::new("touch \"$0.executed\"; exit 77");
    let marker = executable.0.with_extension("executed");
    let (_lock, _env) = EnvRestore::set(&[
        (NATIVE_EXECUTABLE_ENV, executable.0.to_str()),
        (BODY_FROM_SENSOR_ENV, Some(identity_json())),
        (BODY_FROM_IMU_ENV, Some(identity_json())),
        (POINTLIO_OPTIONS_ENV, None),
        (LCM_URL_ENV, Some("memq://no-identity-probe")),
        (PRIVATE_PREFIX_ENV, Some("/plain")),
        (PAIR_MAX_SKEW_MS_ENV, Some("100")),
        (SCAN_VOXEL_SIZE_M_ENV, None),
        (MAP_VOXEL_SIZE_M_ENV, None),
        (INITIAL_MAP_FRAMES_ENV, None),
    ]);
    for backend in [NativeBackend::FastLio2, NativeBackend::PointLio] {
        let bound = NativeLioPreflight::from_env(backend)
            .unwrap()
            .bind("sensor", "body", "odom")
            .unwrap();
        assert_eq!(bound.runtime.executable, executable.0);
    }
    assert!(!marker.exists(), "准备阶段不应执行原生程序");
}

fn shell_runtime(script: &str) -> NativeRuntimeSpec {
    NativeRuntimeSpec {
        backend: NativeBackend::FastLio2,
        cwd: PathBuf::from("/"),
        executable: PathBuf::from("/bin/sh"),
        args: vec!["-c".to_owned(), script.to_owned()],
        env: BTreeMap::new(),
    }
}

async fn wait_for_stdout(owner: &NativeProcessOwner, expected: &str) {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(1);
    while !String::from_utf8_lossy(&owner.stdout_tail()).contains(expected) {
        assert!(
            tokio::time::Instant::now() < deadline,
            "child did not publish readiness marker"
        );
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

#[cfg(unix)]
#[tokio::test]
async fn shutdown_uses_term_and_reaps_child() {
    let runtime = shell_runtime("trap 'exit 0' TERM; printf 'ready\\n'; while :; do :; done");
    let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn shell");
    assert!(owner.pid() > 0);
    wait_for_stdout(&owner, "ready").await;
    let status = owner
        .shutdown(Duration::from_secs(1), Duration::from_secs(1))
        .await
        .expect("TERM shutdown");
    assert!(status.success(), "TERM trap status: {status}");
    assert!(owner
        .poll_unexpected_exit()
        .expect("post-reap poll")
        .is_some());

    // A cached shutdown must not probe or signal the numeric PGID again.
    // An out-of-range replacement makes any such regression deterministic.
    owner.pid = u32::MAX;
    let repeated = owner
        .shutdown(Duration::ZERO, Duration::ZERO)
        .await
        .expect("cached shutdown");
    assert_eq!(repeated, status);
}

#[cfg(unix)]
#[tokio::test]
async fn shutdown_falls_back_to_kill_and_reaps_child() {
    use std::os::unix::process::ExitStatusExt;

    let runtime = shell_runtime("trap '' TERM; printf 'ready\\n'; while :; do :; done");
    let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn shell");
    wait_for_stdout(&owner, "ready").await;
    let status = owner
        .shutdown(Duration::from_millis(25), Duration::from_secs(1))
        .await
        .expect("KILL fallback shutdown");
    assert_eq!(status.signal(), Some(nix::libc::SIGKILL));
}

#[cfg(unix)]
#[tokio::test]
async fn shutdown_terminates_descendants_after_parent_reap() {
    let runtime = shell_runtime(
            "trap 'exit 0' TERM; sh -c 'trap \"\" TERM; exec >/dev/null 2>&1; while :; do sleep 1; done' & descendant=$!; printf 'descendant=%s\\n' \"$descendant\"; while :; do sleep 1; done",
        );
    let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn process tree");
    wait_for_stdout(&owner, "descendant=").await;
    let output = String::from_utf8(owner.stdout_tail()).expect("UTF-8 descendant marker");
    let descendant_pid = output
        .lines()
        .find_map(|line| line.strip_prefix("descendant="))
        .expect("descendant marker")
        .parse::<u32>()
        .expect("descendant pid");

    let status = owner
        .shutdown(Duration::from_millis(25), Duration::from_secs(2))
        .await
        .expect("process-group shutdown");
    assert!(status.success(), "parent TERM trap status: {status}");
    assert!(
        !process_group_exists(owner.pid()).expect("probe process group"),
        "native process group survived acknowledged shutdown"
    );
    assert!(
        !Path::new(&format!("/proc/{descendant_pid}")).exists(),
        "descendant pid {descendant_pid} survived acknowledged shutdown"
    );
}

#[cfg(unix)]
#[tokio::test]
async fn unexpected_exit_is_observed_before_group_identity_is_reaped() {
    let runtime = shell_runtime("exit 7");
    let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn exiting shell");
    let pid = owner.pid();
    let observed = tokio::time::timeout(Duration::from_secs(1), owner.wait_unexpected_exit())
        .await
        .expect("unexpected-exit observation timeout")
        .expect("observe unexpected exit");
    assert_eq!(observed.code(), Some(7));
    assert!(owner.child.is_some(), "waitid(WNOWAIT) reaped the leader");
    assert!(owner.reaped_status.is_none());
    assert!(
        process_group_exists(pid).expect("probe pinned process group"),
        "unreaped leader no longer pinned its PGID"
    );

    let reaped = owner
        .shutdown(Duration::ZERO, Duration::from_secs(1))
        .await
        .expect("cleanup observed exit");
    assert_eq!(reaped.code(), Some(7));
    assert!(owner.reaped_status.is_some());
    assert!(owner.group_cleanup_confirmed);
    assert!(!process_group_exists(pid).expect("probe reaped process group"));
}

#[cfg(unix)]
#[tokio::test]
async fn output_is_drained_concurrently_and_retained_with_a_bound() {
    let runtime = shell_runtime(
            "i=0; while [ $i -lt 10000 ]; do printf 'stdout-%08d-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\\n' $i; printf 'stderr-%08d-yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy\\n' $i >&2; i=$((i+1)); done",
        );
    let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn writer");
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    loop {
        if owner.poll_unexpected_exit().expect("poll writer").is_some() {
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "writer timed out");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    owner
        .shutdown(Duration::ZERO, Duration::from_secs(1))
        .await
        .expect("finish drains");

    let stdout = owner.stdout_tail();
    let stderr = owner.stderr_tail();
    assert!(stdout.len() <= OUTPUT_TAIL_BYTES);
    assert!(stderr.len() <= OUTPUT_TAIL_BYTES);
    assert!(String::from_utf8_lossy(&stdout).contains("stdout-00009999"));
    assert!(String::from_utf8_lossy(&stderr).contains("stderr-00009999"));
}

use nuwax_codec::lcm::schema::{
    encode_odometry_body, encode_point_cloud2_body, Header, Odometry, PointCloud2, PointField, Pose,
};
use nuwax_lio_backends::native::{NativeBackend, NativeLioConfig};
use nuwax_lio_backends::{BackendKind, LioBackendSlot, LioInputFrame, LioOutputFrame};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
use nuwax_transport::LcmTransport;
use serde_json::{json, Value};
use std::collections::BTreeSet;
use std::env;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Duration;

const LCM_URL_ENV: &str = "LCM_DEFAULT_URL";
const BACKEND_ENV: &str = "NUWAX_GEOMETRY_LIO_BACKEND";
const PID_FILE_ENV: &str = "NUWAX_LIO_TEST_CHILD_PID_FILE";
const TERM_FILE_ENV: &str = "NUWAX_LIO_TEST_CHILD_TERM_FILE";
const CONTRACT_FILE_ENV: &str = "NUWAX_LIO_TEST_CHILD_CONTRACT_FILE";
const RECORDED_FIXTURE_ENV: &str = "NUWAX_LIO_TEST_CHILD_RECORDED_FIXTURE";
const PUBLISH_PERMIT_FILE_ENV: &str = "NUWAX_LIO_TEST_CHILD_PUBLISH_PERMIT_FILE";
const ODOMETRY_TYPE: &str = "nav_msgs.Odometry";
const POINT_CLOUD_TYPE: &str = "sensor_msgs.PointCloud2";
const PUBLISH_PERIOD: Duration = Duration::from_millis(25);

struct EncodedPair {
    odometry: Vec<u8>,
    cloud: Vec<u8>,
}

struct ProcessedRecordedFrame {
    sequence: u32,
    source_timestamp_ns: i64,
    output: LioOutputFrame,
    pair: EncodedPair,
}

struct RecordedPlayback {
    input_frame_count: usize,
    frames: Vec<ProcessedRecordedFrame>,
}

struct NativeChildArgs {
    backend: NativeBackend,
    config: NativeLioConfig,
    odometry: (LcmTopic, LcmMessageType),
    cloud: (LcmTopic, LcmMessageType),
}

impl NativeChildArgs {
    fn parse() -> Result<Self, String> {
        let backend = match required_env(BACKEND_ENV)?.as_str() {
            "fastlio2" => NativeBackend::FastLio2,
            "pointlio" => NativeBackend::PointLio,
            value => return Err(format!("unsupported {BACKEND_ENV}={value:?}")),
        };
        let mut config = NativeLioConfig::for_backend(backend);
        let argv = env::args().skip(1).collect::<Vec<_>>();
        if argv.len() % 2 != 0 {
            return Err("native argv must contain flag/value pairs".to_owned());
        }

        let mut seen = BTreeSet::new();
        for pair in argv.chunks_exact(2) {
            let flag = pair[0].as_str();
            let value = pair[1].as_str();
            if !seen.insert(flag.to_owned()) {
                return Err(format!("duplicate native flag {flag}"));
            }
            match flag {
                "--lidar" => config.lidar_topic = value.to_owned(),
                "--odometry" => config.odometry_topic = value.to_owned(),
                "--frame_id" => config.frame_id = value.to_owned(),
                "--child_frame_id" => config.child_frame_id = value.to_owned(),
                "--sensor_frame_id" => config.sensor_frame_id = value.to_owned(),
                "--extrinsic_t" => {
                    config.extrinsic_translation_m = parse_numbers(flag, value)?;
                }
                "--extrinsic_r" => {
                    config.extrinsic_rotation_row_major = parse_numbers(flag, value)?;
                }
                "--host_ip" => config.host_ip = value.to_owned(),
                "--lidar_ip" => config.lidar_ip = value.to_owned(),
                "--frequency" => {
                    config.lidar_frequency_hz = parse_number(flag, value)?;
                }
                "--pointcloud_freq" => {
                    config.pointcloud_frequency_hz = parse_number(flag, value)?;
                }
                "--odom_freq" => {
                    config.odometry_frequency_hz = parse_number(flag, value)?;
                }
                "--debug" => {
                    config.debug = value
                        .parse::<bool>()
                        .map_err(|error| format!("invalid {flag}={value:?}: {error}"))?;
                }
                _ => return Err(format!("unsupported native flag {flag}")),
            }
        }

        let expected_flags = [
            "--child_frame_id",
            "--debug",
            "--extrinsic_r",
            "--extrinsic_t",
            "--frame_id",
            "--frequency",
            "--host_ip",
            "--lidar",
            "--lidar_ip",
            "--odometry",
            "--odom_freq",
            "--pointcloud_freq",
            "--sensor_frame_id",
        ];
        if seen.len() != expected_flags.len()
            || expected_flags.iter().any(|flag| !seen.contains(*flag))
        {
            return Err(format!("native argv is incomplete: flags={seen:?}"));
        }
        config.validate()?;

        let odometry = typed_binding(&config.odometry_topic, ODOMETRY_TYPE)?;
        let cloud = typed_binding(&config.lidar_topic, POINT_CLOUD_TYPE)?;
        Ok(Self {
            backend,
            config,
            odometry,
            cloud,
        })
    }
}

fn parse_number(flag: &str, value: &str) -> Result<f64, String> {
    value
        .parse::<f64>()
        .map_err(|error| format!("invalid {flag}={value:?}: {error}"))
}

fn parse_numbers<const N: usize>(flag: &str, value: &str) -> Result<[f64; N], String> {
    let values = value
        .split(',')
        .map(|item| parse_number(flag, item))
        .collect::<Result<Vec<_>, _>>()?;
    values.try_into().map_err(|values: Vec<f64>| {
        format!(
            "invalid {flag}={value:?}: expected {N} comma-separated values, got {}",
            values.len()
        )
    })
}

fn required_env(key: &str) -> Result<String, String> {
    match env::var(key) {
        Ok(value) if !value.is_empty() && value == value.trim() => Ok(value),
        Ok(_) => Err(format!("{key} must be non-empty and unpadded")),
        Err(error) => Err(format!("{key} is required: {error}")),
    }
}

fn optional_strict_env(key: &str) -> Result<Option<String>, String> {
    match env::var(key) {
        Ok(value) if !value.is_empty() && value == value.trim() => Ok(Some(value)),
        Ok(_) => Err(format!("{key} must be non-empty and unpadded when set")),
        Err(env::VarError::NotPresent) => Ok(None),
        Err(error) => Err(format!("read optional {key} failed: {error}")),
    }
}

fn typed_binding(pattern: &str, expected_type: &str) -> Result<(LcmTopic, LcmMessageType), String> {
    let (topic, message_type) = LcmTopic::parse_subscription_pattern(pattern)
        .map_err(|error| format!("invalid typed topic {pattern:?}: {error}"))?;
    if message_type.as_deref() != Some(expected_type) {
        return Err(format!(
            "typed topic {pattern:?} must use message type {expected_type}"
        ));
    }
    let message_type = LcmMessageType::new(expected_type)
        .map_err(|error| format!("invalid message type {expected_type}: {error}"))?;
    Ok((topic, message_type))
}

fn native_test_messages(config: &NativeLioConfig, sequence: u32) -> Result<EncodedPair, String> {
    let stamp_sec = 1_000_i32
        .checked_add(i32::try_from(sequence).map_err(|_| "sequence exceeds i32".to_owned())?)
        .ok_or_else(|| "test timestamp overflow".to_owned())?;
    let header = Header {
        seq: i32::try_from(sequence).map_err(|_| "sequence exceeds i32".to_owned())?,
        stamp_sec,
        stamp_nsec: 123_456_789,
        frame_id: config.frame_id.clone(),
    };
    let odometry = Odometry {
        header,
        child_frame_id: config.child_frame_id.clone(),
        pose: Pose {
            position: [0.0, 0.0, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        },
        ..Odometry::default()
    };

    let mut point_data = Vec::with_capacity(12);
    for coordinate in [1.0_f32, 2.0, 3.0] {
        point_data.extend_from_slice(&coordinate.to_le_bytes());
    }
    let cloud = PointCloud2 {
        header: Header {
            frame_id: config.sensor_frame_id.clone(),
            ..odometry.header.clone()
        },
        height: 1,
        width: 1,
        fields: ["x", "y", "z"]
            .into_iter()
            .enumerate()
            .map(|(index, name)| PointField {
                name: name.to_owned(),
                offset: i32::try_from(index * 4).expect("three point fields fit i32"),
                datatype: 7,
                count: 1,
            })
            .collect(),
        is_bigendian: false,
        point_step: 12,
        row_step: 12,
        data: point_data,
        is_dense: true,
    };

    let odometry = encode_odometry_body(&odometry)
        .map_err(|error| format!("encode test odometry failed: {error}"))?;
    let cloud = encode_point_cloud2_body(&cloud)
        .map_err(|error| format!("encode test cloud failed: {error}"))?;
    Ok(EncodedPair { odometry, cloud })
}

fn timestamp_header(frame_id: &str, sequence: u32, timestamp_ns: i64) -> Result<Header, String> {
    if timestamp_ns < 0 {
        return Err(format!("negative recorded timestamp {timestamp_ns}"));
    }
    let stamp_sec = i32::try_from(timestamp_ns / 1_000_000_000)
        .map_err(|_| format!("recorded timestamp seconds exceed i32: {timestamp_ns}"))?;
    let stamp_nsec = i32::try_from(timestamp_ns % 1_000_000_000)
        .map_err(|_| format!("recorded timestamp nanoseconds exceed i32: {timestamp_ns}"))?;
    Ok(Header {
        seq: i32::try_from(sequence).map_err(|_| "sequence exceeds i32".to_owned())?,
        stamp_sec,
        stamp_nsec,
        frame_id: frame_id.to_owned(),
    })
}

fn recorded_test_messages(
    config: &NativeLioConfig,
    sequence: u32,
    input: &LioInputFrame,
    output: &LioOutputFrame,
) -> Result<EncodedPair, String> {
    let half_yaw = output.odometry.yaw_rad / 2.0;
    let odometry = Odometry {
        header: timestamp_header(&config.frame_id, sequence, output.odometry.timestamp_ns)?,
        child_frame_id: config.child_frame_id.clone(),
        pose: Pose {
            position: output.odometry.position_m,
            orientation_xyzw: [0.0, 0.0, half_yaw.sin(), half_yaw.cos()],
        },
        twist_linear: output.odometry.linear_velocity_mps,
        ..Odometry::default()
    };

    let point_data_capacity = input
        .cloud
        .points
        .len()
        .checked_mul(16)
        .ok_or_else(|| "recorded point-cloud byte size overflow".to_owned())?;
    let mut point_data = Vec::with_capacity(point_data_capacity);
    for point in &input.cloud.points {
        for value in [point.x, point.y, point.z, point.intensity] {
            point_data.extend_from_slice(&(value as f32).to_le_bytes());
        }
    }
    let width = i32::try_from(input.cloud.points.len())
        .map_err(|_| "recorded point count exceeds i32".to_owned())?;
    let row_step = width
        .checked_mul(16)
        .ok_or_else(|| "recorded PointCloud2 row_step overflow".to_owned())?;
    let cloud = PointCloud2 {
        header: timestamp_header(&config.sensor_frame_id, sequence, input.cloud.timestamp_ns)?,
        height: 1,
        width,
        fields: ["x", "y", "z", "intensity"]
            .into_iter()
            .enumerate()
            .map(|(index, name)| PointField {
                name: name.to_owned(),
                offset: i32::try_from(index * 4).expect("four point fields fit i32"),
                datatype: 7,
                count: 1,
            })
            .collect(),
        is_bigendian: false,
        point_step: 16,
        row_step,
        data: point_data,
        is_dense: true,
    };

    let odometry = encode_odometry_body(&odometry)
        .map_err(|error| format!("encode recorded odometry failed: {error}"))?;
    let cloud = encode_point_cloud2_body(&cloud)
        .map_err(|error| format!("encode recorded cloud failed: {error}"))?;
    Ok(EncodedPair { odometry, cloud })
}

fn backend_kind(backend: NativeBackend) -> BackendKind {
    match backend {
        NativeBackend::FastLio2 => BackendKind::FastLio2,
        NativeBackend::PointLio => BackendKind::PointLio,
    }
}

fn backend_name(backend: NativeBackend) -> &'static str {
    match backend {
        NativeBackend::FastLio2 => "fastlio2",
        NativeBackend::PointLio => "pointlio",
    }
}

fn load_recorded_playback(
    path: &Path,
    backend: NativeBackend,
    config: &NativeLioConfig,
) -> Result<RecordedPlayback, String> {
    let bytes = std::fs::read(path)
        .map_err(|error| format!("read recorded fixture {path:?} failed: {error}"))?;
    let inputs: Vec<LioInputFrame> = serde_json::from_slice(&bytes)
        .map_err(|error| format!("deserialize recorded fixture {path:?} failed: {error}"))?;

    let mut slot = LioBackendSlot::new();
    slot.start(backend_kind(backend))
        .map_err(|error| format!("start recorded LIO backend failed: {error}"))?;
    let processed = inputs
        .iter()
        .enumerate()
        .map(|(index, input)| {
            let sequence = u32::try_from(index + 1)
                .map_err(|_| "recorded fixture frame count exceeds u32".to_owned())?;
            let output = slot
                .process(input)
                .map_err(|error| format!("process recorded frame {sequence} failed: {error}"))?;
            let pair = recorded_test_messages(config, sequence, input, &output)?;
            Ok(ProcessedRecordedFrame {
                sequence,
                source_timestamp_ns: input.cloud.timestamp_ns,
                output,
                pair,
            })
        })
        .collect::<Result<Vec<_>, String>>();
    slot.stop();

    Ok(RecordedPlayback {
        input_frame_count: inputs.len(),
        frames: processed?,
    })
}

fn read_publish_permit(path: &Path) -> Result<u32, String> {
    let bytes = std::fs::read(path)
        .map_err(|error| format!("read publish permit file {path:?} failed: {error}"))?;
    let raw = std::str::from_utf8(&bytes)
        .map_err(|error| format!("publish permit file {path:?} is not UTF-8: {error}"))?;
    let permit = raw
        .parse::<u32>()
        .map_err(|error| format!("publish permit file {path:?} is not a u32: {error}"))?;
    if permit.to_string() != raw {
        return Err(format!(
            "publish permit file {path:?} must contain canonical unpadded u32 text"
        ));
    }
    Ok(permit)
}

fn contract_value(
    args: &NativeChildArgs,
    lcm_url: &str,
    recorded_fixture: Option<&str>,
    publish_permit_file: Option<&str>,
    playback: Option<&RecordedPlayback>,
    published_pair_count: u32,
    last_published_sequence: u32,
) -> Value {
    let processed_outputs = playback
        .map(|recorded| {
            recorded
                .frames
                .iter()
                .map(|frame| {
                    json!({
                        "sequence": frame.sequence,
                        "source_timestamp_ns": frame.source_timestamp_ns,
                        "health": &frame.output.health,
                        "output": &frame.output,
                    })
                })
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    let final_output = playback
        .and_then(|recorded| recorded.frames.last())
        .map(|frame| &frame.output);
    let input_frame_count = playback.map_or(0, |recorded| recorded.input_frame_count);
    let processed_output_count = playback.map_or(0, |recorded| recorded.frames.len());
    let playback_mode = if playback.is_some() {
        "recorded"
    } else {
        "synthetic"
    };

    json!({
        "backend": backend_name(args.backend),
        "lcm_url": lcm_url,
        "lidar_topic": &args.config.lidar_topic,
        "odometry_topic": &args.config.odometry_topic,
        "frame_id": &args.config.frame_id,
        "child_frame_id": &args.config.child_frame_id,
        "sensor_frame_id": &args.config.sensor_frame_id,
        "extrinsic_translation_m": args.config.extrinsic_translation_m,
        "extrinsic_rotation_row_major": args.config.extrinsic_rotation_row_major,
        "host_ip": &args.config.host_ip,
        "lidar_ip": &args.config.lidar_ip,
        "recorded_fixture": recorded_fixture,
        "publish_permit_file": publish_permit_file,
        "playback": {
            "mode": playback_mode,
            "recorded_fixture": recorded_fixture,
            "input_frame_count": input_frame_count,
            "processed_output_count": processed_output_count,
            "processed_outputs": processed_outputs,
            "final_output": final_output,
        },
        "published_pair_count": published_pair_count,
        "last_published_sequence": last_published_sequence,
    })
}

fn write_contract_atomic(path: &Path, contract: &Value) -> Result<(), String> {
    let file_name = path
        .file_name()
        .ok_or_else(|| format!("contract path {path:?} has no file name"))?;
    let mut temp_name = file_name.to_os_string();
    temp_name.push(format!(".{}.tmp", std::process::id()));
    let temp_path = path.with_file_name(temp_name);
    let encoded = serde_json::to_vec(contract)
        .map_err(|error| format!("serialize contract {path:?} failed: {error}"))?;
    std::fs::write(&temp_path, encoded)
        .map_err(|error| format!("write temporary contract {temp_path:?} failed: {error}"))?;
    std::fs::rename(&temp_path, path)
        .map_err(|error| format!("replace contract {path:?} from {temp_path:?} failed: {error}"))
}

fn frame(binding: &(LcmTopic, LcmMessageType), sequence: u32, payload: Vec<u8>) -> RawLcmFrame {
    RawLcmFrameBuilder::new(binding.0.clone(), binding.1.clone())
        .seqno(sequence)
        .payload(payload)
        .build()
}

async fn publish_pair(
    transport: &LcmTransport,
    args: &NativeChildArgs,
    sequence: u32,
    pair: &EncodedPair,
) -> Result<(), String> {
    transport
        .publish(frame(&args.odometry, sequence, pair.odometry.clone()))
        .await
        .map_err(|error| format!("publish test odometry failed: {error}"))?;
    transport
        .publish(frame(&args.cloud, sequence, pair.cloud.clone()))
        .await
        .map_err(|error| format!("publish test cloud failed: {error}"))
}

fn write_marker(key: &str, contents: impl AsRef<[u8]>) -> Result<(), String> {
    let path = required_env(key)?;
    std::fs::write(Path::new(&path), contents)
        .map_err(|error| format!("write {key}={path:?} failed: {error}"))
}

#[cfg(target_os = "linux")]
fn install_parent_death_signal() -> Result<(), String> {
    let parent = nix::unistd::getppid();
    nix::sys::prctl::set_pdeathsig(nix::sys::signal::Signal::SIGKILL)
        .map_err(|error| format!("install parent-death signal failed: {error}"))?;
    if nix::unistd::getppid() != parent {
        return Err("Geometry parent exited while the test child was starting".to_owned());
    }
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn install_parent_death_signal() -> Result<(), String> {
    Ok(())
}

#[cfg(unix)]
async fn wait_for_test_termination() -> Result<&'static str, String> {
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .map_err(|error| format!("install SIGTERM handler failed: {error}"))?;
    terminate
        .recv()
        .await
        .ok_or_else(|| "SIGTERM stream closed before a signal arrived".to_owned())?;
    Ok("SIGTERM\n")
}

#[cfg(not(unix))]
async fn wait_for_test_termination() -> Result<&'static str, String> {
    tokio::signal::ctrl_c()
        .await
        .map_err(|error| format!("wait for termination failed: {error}"))?;
    Ok("CTRL_C\n")
}

async fn run_native_lio_test_child() -> Result<(), String> {
    install_parent_death_signal()?;
    let args = NativeChildArgs::parse()?;
    let lcm_url = required_env(LCM_URL_ENV)?;
    let recorded_fixture = optional_strict_env(RECORDED_FIXTURE_ENV)?;
    let publish_permit_file = optional_strict_env(PUBLISH_PERMIT_FILE_ENV)?;
    let contract_path = PathBuf::from(required_env(CONTRACT_FILE_ENV)?);
    if let Some(path) = publish_permit_file.as_deref() {
        read_publish_permit(Path::new(path))?;
    }
    let playback = recorded_fixture
        .as_deref()
        .map(|path| load_recorded_playback(Path::new(path), args.backend, &args.config))
        .transpose()?;
    let transport = LcmTransport::new_real(&lcm_url)
        .map_err(|error| format!("create real LCM transport failed: {error}"))?;

    write_marker(PID_FILE_ENV, std::process::id().to_string())?;
    let mut published_pair_count = 0_u32;
    let mut last_published_sequence = 0_u32;
    let contract = contract_value(
        &args,
        &lcm_url,
        recorded_fixture.as_deref(),
        publish_permit_file.as_deref(),
        playback.as_ref(),
        published_pair_count,
        last_published_sequence,
    );
    write_contract_atomic(&contract_path, &contract)?;

    let termination = wait_for_test_termination();
    tokio::pin!(termination);
    let mut interval = tokio::time::interval(PUBLISH_PERIOD);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut sequence = 1_u32;
    let update_contract_after_publish = playback.is_some() || publish_permit_file.is_some();
    loop {
        tokio::select! {
            biased;
            marker = &mut termination => {
                write_marker(TERM_FILE_ENV, marker?)?;
                return Ok(());
            }
            _ = interval.tick() => {
                if let Some(path) = publish_permit_file.as_deref() {
                    let permit = read_publish_permit(Path::new(path))?;
                    if sequence > permit {
                        continue;
                    }
                }

                let generated_pair;
                let pair = if let Some(recorded) = playback.as_ref() {
                    let index = usize::try_from(sequence - 1)
                        .map_err(|_| "recorded sequence exceeds usize".to_owned())?;
                    let Some(frame) = recorded.frames.get(index) else {
                        continue;
                    };
                    &frame.pair
                } else {
                    generated_pair = native_test_messages(&args.config, sequence)?;
                    &generated_pair
                };

                tokio::select! {
                    biased;
                    marker = &mut termination => {
                        write_marker(TERM_FILE_ENV, marker?)?;
                        return Ok(());
                    }
                    result = publish_pair(&transport, &args, sequence, pair) => result?,
                }

                published_pair_count = published_pair_count
                    .checked_add(1)
                    .ok_or_else(|| "published pair count exhausted".to_owned())?;
                last_published_sequence = sequence;
                if update_contract_after_publish {
                    let contract = contract_value(
                        &args,
                        &lcm_url,
                        recorded_fixture.as_deref(),
                        publish_permit_file.as_deref(),
                        playback.as_ref(),
                        published_pair_count,
                        last_published_sequence,
                    );
                    write_contract_atomic(&contract_path, &contract)?;
                }
                sequence = sequence
                    .checked_add(1)
                    .ok_or_else(|| "test sequence exhausted".to_owned())?;
            }
        }
    }
}

fn main() -> ExitCode {
    let runtime = match tokio::runtime::Runtime::new() {
        Ok(runtime) => runtime,
        Err(error) => {
            eprintln!("native LIO test child runtime failed: {error}");
            return ExitCode::FAILURE;
        }
    };
    match runtime.block_on(run_native_lio_test_child()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("native LIO test child failed: {error}");
            ExitCode::FAILURE
        }
    }
}

// Restored assigned-map composition. Only the sensor and physical actuator are
// simulated; admission, fine-map transport, planning, control and guard are real.
use super::*;
use futures::FutureExt;
use nuwax_nav_stack::{register_compiled_in_process_owners, NavStackBuilder, NavStackPreset,
    NavStackProfile, NavTopicConfig, ProductionPipeline, StackRunner};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};

const CHILD_MODE: &str = "NUWAX_ASSIGNED_NAVIGATION_TEST_CHILD";
const POSITION_TOLERANCE: f64 = 0.06;
const YAW_TOLERANCE: f64 = 0.08;
const OBSERVATION_TTL: f64 = 1.0;
const ODOM_ORIGIN: [f64; 3] = [4.0, 2.0, 1.0];

// These are explicitly simulated generic 0.3 m body/actuator capabilities, not
// claims about any device. The map frame is established at the initial body,
// so its support floor is below zero and the body target Z is zero.
const BODY: &str = r#"{"footprint_xy_m":[[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],"z_min_m":-0.35,"z_max_m":0.2,"source":"assigned-map generic simulated 0.3 m body"}"#;
const CAPABILITIES: &str = r#"{
 "schema_version":1,"calibration_id":"assigned-map-generic-simulation",
 "motion":{"model":"differential","max_forward_mps":0.3,"max_reverse_mps":0.0,
 "max_lateral_mps":0.0,"max_angular_rps":0.7,"linear_acceleration_mps2":0.6,
 "guaranteed_braking_mps2":0.8,"angular_acceleration_rps2":1.4,
 "guaranteed_angular_braking_rps2":1.4,"reaction_time_sec":0.1,
 "minimum_turning_radius_m":0.0,"can_rotate_in_place":true},
 "terrain":{"max_step_up_m":0.0,"max_step_down_m":0.0,"min_tread_depth_m":0.2,
 "max_slope_deg":15.0,"max_cross_slope_deg":15.0,"support_height_uncertainty_m":0.01,
 "max_observation_age_sec":1.0,"stairs_supported":false}}
"#;

const GUARD_PROCESS: &str = r#"
import json, os, signal, sys, threading, time, uuid
from pathlib import Path
from services.navigation.motion_safety_guard.actor import GuardActor
from services.navigation.motion_safety_guard.core import MotionGuardCore
from services.navigation.motion_safety_guard.epoch import EpochRecord, EpochStore, ProvisionRecord, atomic_write_record
from services.navigation.motion_safety_guard.server import MotionGuardUnixServer

root = Path(sys.argv[1])
digest = "sha256:" + "4" * 64

def append(name, value):
    with (root / name).open("a", encoding="ascii") as stream:
        stream.write(json.dumps(value) + "\n")
        stream.flush()

class SimulatedActuator:
    is_configured = True
    actuator_symbols_sha256 = digest
    def apply_motion(self, request):
        assert request.HasField("twist"), "fixture only supports navigation twist"
        t = request.twist
        append("actuator.jsonl", {"at":time.monotonic_ns(), "twist":[t.linear_x_mps,t.linear_y_mps,t.angular_z_rps],
            "operation_id":request.envelope.operation_id,"lease_id":request.envelope.tuple.lease_id})
    def zero(self):
        append("actuator.jsonl", {"at":time.monotonic_ns(),"twist":[0.0,0.0,0.0],"operation_id":"","lease_id":0})
    def execute_sport(self, command):
        raise AssertionError("navigation must not invoke sport")
    def switch_mode(self, command):
        raise AssertionError("generic fixture has no vendor mode")

provision = root / "provision.json"
epoch_path = root / "epoch.json"
atomic_write_record(provision, ProvisionRecord.create(instance_id=str(uuid.uuid4())).to_document(), mode=0o400)
atomic_write_record(epoch_path, EpochRecord.initial(actuator_symbols_sha256=digest).to_document(), mode=0o600)
store = EpochStore(provision_path=provision, epoch_path=epoch_path)
core = MotionGuardCore(epoch=store.advance_boot(), epoch_store=store, actuator=SimulatedActuator())
core.initialize_zero_fence()
server = MotionGuardUnixServer(socket_path=root / "guard.sock",actor=GuardActor(core),socket_gid=os.getegid())
stop = threading.Event()
signal.signal(signal.SIGTERM, lambda *_: stop.set())
signal.signal(signal.SIGINT, lambda *_: stop.set())
server.start()
try:
    while not stop.wait(0.05):
        pass
finally:
    server.stop()
"#;

struct GuardProcess { child: Child, root: PathBuf }
impl GuardProcess {
    async fn start(root: &Path) -> Self {
        let root = root.join("guard");
        std::fs::create_dir(&root).unwrap();
        let repository = Path::new(env!("CARGO_MANIFEST_DIR")).ancestors().nth(3).unwrap();
        let python = std::env::var_os("NUWAX_TEST_GUARD_PYTHON").unwrap_or_else(|| "python3".into());
        let stderr = std::fs::File::create(root.join("stderr.log")).unwrap();
        let child = Command::new(python).args(["-u", "-c", GUARD_PROCESS]).arg(&root)
            .current_dir(repository).stdin(Stdio::null()).stdout(Stdio::null()).stderr(stderr)
            .spawn().expect("start actual Python MotionGuardCore/GuardActor/UnixServer");
        let mut guard = Self { child, root };
        let deadline = tokio::time::Instant::now() + Duration::from_secs(8);
        while !guard.socket().exists() {
            if let Some(status) = guard.child.try_wait().unwrap() {
                panic!("production guard startup {status}: {}", guard.stderr());
            }
            assert!(tokio::time::Instant::now() < deadline, "guard startup timeout: {}", guard.stderr());
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        guard
    }
    fn socket(&self) -> PathBuf { self.root.join("guard.sock") }
    fn stderr(&self) -> String { std::fs::read_to_string(self.root.join("stderr.log")).unwrap_or_default() }
    async fn stop(&mut self) {
        let result = unsafe { libc::kill(self.child.id() as i32, libc::SIGTERM) };
        assert_eq!(result, 0, "signal guard");
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                if let Some(status) = self.child.try_wait().unwrap() {
                    assert!(status.success(), "guard shutdown {status}: {}", self.stderr());
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }).await.expect("guard child joined");
        assert!(!self.socket().exists(), "guard unlinks socket");
    }
}
impl Drop for GuardProcess {
    fn drop(&mut self) {
        if self.child.try_wait().ok().flatten().is_none() { let _ = self.child.kill(); let _ = self.child.wait(); }
    }
}

#[derive(Clone, Debug, serde::Deserialize)]
struct Actuation { at: u64, twist: [f64; 3], operation_id: String, lease_id: u64 }
fn read_records<T: serde::de::DeserializeOwned>(path: &Path) -> Vec<T> {
    let bytes = std::fs::read(path).unwrap_or_default();
    // A concurrent writer may have started the final line, which is not an
    // event until the trailing newline commits it.
    bytes.split_inclusive(|byte| *byte == b'\n').filter(|line| line.last() == Some(&b'\n'))
        .map(|line| serde_json::from_slice(line).expect("complete guard evidence JSON")).collect()
}
fn monotonic_ns() -> u64 {
    let mut value = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    assert_eq!(unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut value) }, 0);
    value.tv_sec as u64 * 1_000_000_000 + value.tv_nsec as u64
}
#[derive(Debug)]
struct IntegratedActuator {
    x: f64, y: f64, yaw: f64, command: [f64; 3], at: u64, consumed: usize,
    nonzero: usize, distance: f64,
}
impl IntegratedActuator {
    fn new() -> Self { Self { x:0.0,y:0.0,yaw:0.0,command:[0.0;3],at:0,consumed:0,nonzero:0,distance:0.0 } }
    fn advance(&mut self, at: u64) {
        assert!(at >= self.at, "actuator evidence cannot go backward");
        if self.at != 0 {
            let dt = (at - self.at) as f64 / 1e9;
            let [vx, vy, omega] = self.command;
            let mid = self.yaw + omega * dt * 0.5;
            let arc = if omega.abs() < 1e-10 { dt } else { 2.0*(omega*dt*0.5).sin()/omega };
            self.x += (vx*mid.cos()-vy*mid.sin())*arc;
            self.y += (vx*mid.sin()+vy*mid.cos())*arc;
            self.yaw += omega*dt;
            self.distance += vx.hypot(vy)*dt;
        }
        self.at = at;
    }
    fn consume(&mut self, path: &Path) {
        let events: Vec<Actuation> = read_records(path);
        assert!(events.len() >= self.consumed, "actuator ledger was truncated");
        for event in &events[self.consumed..] {
            self.advance(event.at);
            self.command = event.twist;
            self.nonzero += usize::from(event.twist.iter().any(|v| *v != 0.0));
        }
        self.consumed = events.len();
        // Integrate only closed event intervals. Predict a separate snapshot
        // to the sensor's capture time; never overwrite evidence chronology.
    }
    fn pose_at(&self, now: u64) -> [f64; 3] {
        let mut snapshot = Self {x:self.x,y:self.y,yaw:self.yaw,command:self.command,at:self.at,
            consumed:self.consumed,nonzero:self.nonzero,distance:self.distance};
        snapshot.advance(now); [snapshot.x,snapshot.y,snapshot.yaw]
    }
}

#[derive(Default)]
struct Ledger {
    states: Vec<schema::NavigationState>, paths: Vec<schema::PathEnvelope>,
    maps: Vec<schema::NavigationMap3D>, goals: Vec<schema::GoalEnvelope>,
}
struct EvidenceReaders { ledger: Arc<StdMutex<Ledger>>, tasks: Vec<tokio::task::JoinHandle<()>> }
impl EvidenceReaders {
    async fn start(transport: &LcmTransport) -> Self {
        let ledger = Arc::new(StdMutex::new(Ledger::default()));
        let mut tasks = Vec::new();
        for (topic, kind) in [("/nav/state","nuwax_msgs.NavigationState"),
            ("/nav/path","nuwax_msgs.PathEnvelope"),("/nav/map3d","nuwax_msgs.NavigationMap3D"),
            ("/nav/goal","nuwax_msgs.GoalEnvelope")] {
            let mut frames = transport.subscribe(LcmTopic::new(topic).unwrap(), Some(LcmMessageType::new(kind).unwrap())).await;
            let ledger = ledger.clone();
            tasks.push(tokio::spawn(async move {
                while let Some(frame) = frames.next().await {
                    let frame = frame.expect("real-LCM evidence stream");
                    let mut ledger = ledger.lock().unwrap();
                    match kind {
                        "nuwax_msgs.NavigationState" => ledger.states.push(schema::decode_navigation_state_body(&frame.payload).unwrap()),
                        "nuwax_msgs.PathEnvelope" => ledger.paths.push(schema::decode_path_envelope_body(&frame.payload).unwrap()),
                        "nuwax_msgs.NavigationMap3D" => ledger.maps.push(schema::decode_navigation_map_3d_body(&frame.payload).unwrap()),
                        _ => ledger.goals.push(schema::decode_goal_envelope_body(&frame.payload).unwrap()),
                    }
                    assert!(ledger.states.len() < 8192 && ledger.maps.len() < 8192 && ledger.paths.len() < 8192,
                        "bounded composition evidence overflow");
                }
            }));
        }
        Self { ledger, tasks }
    }
    async fn finish(mut self) {
        for task in &self.tasks { task.abort(); }
        for task in self.tasks.drain(..) { if let Err(error) = task.await { assert!(error.is_cancelled(), "evidence reader: {error}"); } }
    }
}
impl Drop for EvidenceReaders { fn drop(&mut self) { for task in &self.tasks { task.abort(); } } }

fn scene() -> Vec<Point3<f64>> {
    // Fine resolution is 0.05 m; all floor cells across the full swept body
    // have real hit observations, including during the final yaw alignment.
    let mut points = Vec::new();
    for x in -30..=60 { for y in -30..=30 {
        points.push(Point3::new((x as f64+0.5)*0.05,(y as f64+0.5)*0.05,-0.4));
    }}
    for (x,y,height) in [(-1.2,0.95,0.65),(2.5,1.15,0.9),(2.1,-1.2,0.45)] {
        for z in 0..=(height/0.05) as i32 { for dx in 0..3 { for dy in 0..4 {
            points.push(Point3::new(x+dx as f64*0.05,y+dy as f64*0.05,-0.35+z as f64*0.05));
        }}}
    }
    points
}

struct LiveSensor {
    stop: Arc<AtomicBool>, cloud_enabled: Arc<AtomicBool>,
    integrated: Arc<StdMutex<IntegratedActuator>>, task: Option<tokio::task::JoinHandle<()>>,
}
impl LiveSensor {
    fn start(handle: GeometryRuntimeHandle, common: Arc<crate::lio_ingress::CommonLioIngress>,
        correction: crate::correction_coordinator::CorrectionFrameIngress,
        applier: Arc<crate::lcm_runtime::GeometryCanonicalCorrectionApplier>,
        actuator_path: PathBuf, world: Vec<Point3<f64>>, sequence_start: u64) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let cloud_enabled = Arc::new(AtomicBool::new(true));
        let integrated = Arc::new(StdMutex::new(IntegratedActuator::new()));
        let (stopping, clouds, simulation) = (stop.clone(),cloud_enabled.clone(),integrated.clone());
        let task = tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_millis(50));
            tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            let mut sequence = sequence_start;
            while !stopping.load(Ordering::Acquire) {
                tick.tick().await;
                let pose = {
                    let mut integrated = simulation.lock().unwrap();
                    integrated.consume(&actuator_path);
                    integrated.pose_at(monotonic_ns())
                };
                sequence += 1;
                sensor_frame(&handle, &common, &correction, &applier, &world, pose, sequence,
                    clouds.load(Ordering::Acquire)).await;
            }
        });
        Self { stop, cloud_enabled, integrated, task:Some(task) }
    }
    fn pose(&self) -> [f64;3] { self.integrated.lock().unwrap().pose_at(monotonic_ns()) }
    async fn finish(mut self) -> Result<(),String> {
        self.stop.store(true,Ordering::Release);
        self.task.take().unwrap().await.map_err(|error|format!("live sensor failed: {error}"))
    }
}
impl Drop for LiveSensor { fn drop(&mut self) { self.stop.store(true,Ordering::Release); if let Some(task)=&self.task {task.abort();} } }

async fn sensor_frame(handle: &GeometryRuntimeHandle, common: &crate::lio_ingress::CommonLioIngress,
    correction: &crate::correction_coordinator::CorrectionFrameIngress,
    applier: &crate::lcm_runtime::GeometryCanonicalCorrectionApplier,
    world: &[Point3<f64>], integrated: [f64;3], sequence: u64, cloud: bool) {
    let stamp = now_ns();
    let [x,y,yaw] = integrated;
    let mut odometry = schema::Odometry::default();
    odometry.header.frame_id="odom".into(); odometry.child_frame_id="base_link".into();
    odometry.pose.position=[ODOM_ORIGIN[0]+x,ODOM_ORIGIN[1]+y,ODOM_ORIGIN[2]];
    odometry.pose.orientation_xyzw=[0.0,0.0,(yaw/2.0).sin(),(yaw/2.0).cos()];
    let envelope = common.prepare_legacy_odometry(stamp as i64,odometry).unwrap();
    let correlated = crate::lio_ingress::CorrelatedOdometry {source_sequence:envelope.source_sequence,
        source_timestamp_ns:envelope.source_timestamp_ns};
    let identity = crate::lio_ingress::SourceIdentity {backend:envelope.backend,runtime_generation:envelope.runtime_generation,
        source_sequence:sequence as i64,source_timestamp_ns:stamp as i64+1};
    common.accept_public_odometry(envelope.clone(),stamp as i64).unwrap();
    {
        let _lifecycle = handle.mapping_lifecycle_lock.lock().await;
        assert!(crate::lcm_runtime::handoff_lio_odometry_with_correction(handle,&PoseState {
            x:ODOM_ORIGIN[0]+x,y:ODOM_ORIGIN[1]+y,z:ODOM_ORIGIN[2],
            qz:(yaw/2.0).sin(),qw:(yaw/2.0).cos(),frame_id:"odom".into(),timestamp_ns:stamp,..Default::default()
        },"base_link",&(LcmTopic::new("/tf").unwrap(),LcmMessageType::new("tf2_msgs.TFMessage").unwrap()),applier)
            .await.expect("canonical corrected odometry/TF"));
    }
    // The production map-pose adapter consumes public odometry and canonical
    // corrected TF. No direct map_pose or readiness observation is injected.
    handle.transport.publish(crate::typed_codec::build_typed_frame(LcmTopic::new("/odom").unwrap(),
        LcmMessageType::new("nav_msgs.Odometry").unwrap(),schema::encode_odometry_body(&envelope.odometry).unwrap(),sequence))
        .await.unwrap();
    if cloud {
        let odom_points = world.iter().map(|point| Point3::new(point.x+ODOM_ORIGIN[0],
            point.y+ODOM_ORIGIN[1],point.z+ODOM_ORIGIN[2])).collect::<Vec<_>>();
        let prepared = {
            let _lifecycle = handle.mapping_lifecycle_lock.lock().await;
            common.register_odom_points(identity,correlated,stamp as i64+1,odom_points.len(),odom_points,
                &handle.mapping_adapter.as_ref().unwrap().lock()).unwrap()
        };
        let accepted = common.accept_public_cloud(prepared.encoded_for_test().as_ref(),stamp as i64+2).unwrap();
        crate::lcm_runtime::handoff_registered_lio_cloud_with_correction(handle,&accepted,correction)
            .await.expect("fresh registered sensor cloud through production correction ingress");
    }
}

async fn guard_query(socket: &Path, operation: nuwax_motion_safety_guard::wire::motion_guard_request::Operation)
    -> nuwax_motion_safety_guard::wire::motion_guard_response::Outcome {
    use nuwax_motion_safety_guard::wire as guard;
    use tokio::io::{AsyncReadExt,AsyncWriteExt};
    let request = guard::MotionGuardRequest {api_version:guard::API_VERSION,
        request_id:uuid::Uuid::new_v4().to_string(),operation:Some(operation)};
    tokio::time::timeout(Duration::from_secs(2),async {
        let mut stream = tokio::net::UnixStream::connect(socket).await.unwrap();
        let bytes = request.encode_to_vec();
        stream.write_u32(bytes.len() as u32).await.unwrap(); stream.write_all(&bytes).await.unwrap();
        let length = stream.read_u32().await.unwrap() as usize;
        assert!(length > 0 && length <= 1_048_576);
        let mut bytes = vec![0;length]; stream.read_exact(&mut bytes).await.unwrap();
        let response = guard::MotionGuardResponse::decode(bytes.as_slice()).unwrap();
        assert_eq!(response.api_version,guard::API_VERSION);
        assert_eq!(response.request_id,request.request_id);
        response.outcome.expect("correlated guard query outcome")
    }).await.expect("actual guard read-only evidence query")
}
async fn assert_applied(socket: &Path, event: &Actuation) {
    use nuwax_motion_safety_guard::wire as guard;
    assert!(event.lease_id > 0 && !event.operation_id.is_empty());
    let outcome = guard_query(socket,guard::motion_guard_request::Operation::GetOperationResult(
        guard::GetOperationResultRequest {source:guard::GuardSource::Nav as i32,
            operation_namespace:guard::OperationNamespace::Lease as i32,operation_id:event.operation_id.clone()})).await;
    let guard::motion_guard_response::Outcome::GetOperationResult(result) = outcome else {panic!("wrong query outcome")};
    assert!(result.resolved,"actual actuator twist has no settled guard result: {result:?}");
    assert_eq!(result.target_operation_id,event.operation_id);
    assert_eq!(result.target_ack,guard::AckCode::Applied as i32);
}
async fn assert_exact_zero(socket: &Path, state: &schema::NavigationState) {
    use nuwax_motion_safety_guard::wire as guard;
    assert!(!state.zero_ack_id.is_empty() && state.zero_applied_monotonic_ns > 0,"missing exact terminal zero: {state:?}");
    let outcome = guard_query(socket,guard::motion_guard_request::Operation::GetGuardState(
        guard::GetGuardStateRequest {source:guard::GuardSource::Nav as i32})).await;
    let guard::motion_guard_response::Outcome::GetGuardState(result) = outcome else {panic!("wrong guard state outcome")};
    let zero = result.last_zero_confirmation.expect("actual guard exact ZeroAck");
    assert_eq!(zero.zero_ack_id,state.zero_ack_id);
    assert_eq!(zero.applied_monotonic_ns,state.zero_applied_monotonic_ns as u64);
}

fn goal_state_matches(state: &schema::NavigationState, goal: &schema::GoalEnvelope) -> bool {
    state.session_id == goal.session_id && state.owner_kind == goal.owner_kind
        && state.parent_request_id == goal.parent_request_id && state.goal_generation == goal.goal_generation
        && state.goal_payload_sha256 == goal.payload_sha256
}
fn diagnostic(ledger: &Arc<StdMutex<Ledger>>, sensor: &LiveSensor) -> String {
    let ledger = ledger.lock().unwrap();
    format!("integrated={:?}; paths={}; maps={}; last states={:?}",sensor.pose(),ledger.paths.len(),ledger.maps.len(),
        ledger.states.iter().rev().take(8).collect::<Vec<_>>())
}
async fn wait_ready(client: &mut crate::proto::geometry_runtime_client::GeometryRuntimeClient<tonic::transport::Channel>) {
    let deadline = tokio::time::Instant::now()+Duration::from_secs(12);
    loop {
        let detail = client.get_navigation_readiness(Empty{}).await.unwrap().into_inner();
        if detail.ready {return;}
        assert!(tokio::time::Instant::now()<deadline,"real consumer readiness timeout: {detail:?}");
        tokio::time::sleep(Duration::from_millis(25)).await;
    }
}
async fn submit_goal(client: &mut crate::proto::geometry_runtime_client::GeometryRuntimeClient<tonic::transport::Channel>,
    guard_socket:&Path,id:&str,x:f64,yaw:f64) -> NavigationState {
    let mut guard = nuwax_motion_safety_guard::MotionGuardClient::connect(guard_socket).await.unwrap();
    let ready = guard.get_readiness().await.unwrap(); assert!(ready.ready);
    let admitted = client.set_navigation_goal(NavigationGoal {
        pose:Some(Pose {position:Some(Vec3{x,y:0.0,z:0.0}),
            orientation:Some(Quat{x:0.0,y:0.0,z:(yaw/2.0).sin(),w:(yaw/2.0).cos()}),
            frame_id:"map".into(),timestamp_ns:now_ns(),..Default::default()}),
        frame_id:"map".into(),goal_id:id.into(),
        motion_guard_generation:ready.generation,motion_guard_boot_nonce:ready.boot_nonce,
        motion_stop_generation:ready.motion_stop_generation,
        position_tolerance_m:Some(POSITION_TOLERANCE),yaw_tolerance_rad:Some(YAW_TOLERANCE),
        ..Default::default()
    }).await.unwrap().into_inner();
    assert_eq!(admitted.status,"ACCEPTED","actual tonic admission: {admitted:?}");
    admitted
}
async fn observed_goal(ledger:&Arc<StdMutex<Ledger>>,id:&str) -> schema::GoalEnvelope {
    tokio::time::timeout(Duration::from_secs(3),async {
        loop {
            if let Some(goal)=ledger.lock().unwrap().goals.iter().find(|g|g.session_id==id).cloned() {return goal;}
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await.expect("actual goal envelope observed")
}
async fn moving_evidence(ledger:&Arc<StdMutex<Ledger>>,sensor:&LiveSensor,guard:&GuardProcess,
    goal:&schema::GoalEnvelope,after_event:usize) -> Actuation {
    let deadline=tokio::time::Instant::now()+Duration::from_secs(25);
    loop {
        let events:Vec<Actuation>=read_records(&guard.root.join("actuator.jsonl"));
        let actual=events.iter().skip(after_event).rev().find(|event|event.twist[0].abs()>1e-5).cloned();
        let matching=ledger.lock().unwrap().states.iter().any(|state|
            goal_state_matches(state,goal) && state.state==2 && state.path_revision>0 && state.message=="twist APPLIED");
        if let Some(event)=actual.filter(|_|matching) {assert_applied(&guard.socket(),&event).await;return event;}
        assert!(tokio::time::Instant::now()<deadline,"actual nonzero/APPLIED timeout: {}",diagnostic(ledger,sensor));
        let failed=ledger.lock().unwrap().states.iter().any(|s|goal_state_matches(s,goal)&&s.state==4);
        assert!(!failed,"terminal failure before motion: {}",diagnostic(ledger,sensor));
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

async fn run_composition(local_planning:bool,root:&Path) {
    use crate::lcm_runtime::{prepare_geometry_runtime_for_pipeline,spawn_geometry_runtime,LcmBackendKind,ValidatedFrameBindings};
    use crate::proto::geometry_runtime_client::GeometryRuntimeClient;
    use crate::proto::geometry_runtime_server::GeometryRuntimeServer;
    let guard = GuardProcess::start(root).await;
    std::env::set_var("NUWAX_MOTION_GUARD_SOCKET",guard.socket());
    let prepared=prepare_geometry_runtime_for_pipeline(LcmBackendKind::Real,ProductionPipeline::Lio3D).unwrap();
    let mut destination=spawn_geometry_runtime(prepared,crate::mapping_backend::MappingBackendSpec::Voxel,
        crate::channel_registry::ValidatedChannelBindings::from_env().unwrap(),crate::nav_runtime::path_envelope_binding().unwrap(),
        ValidatedFrameBindings::from_env().unwrap()).await.unwrap();
    let handle=destination.handle.clone();
    assert_eq!(handle.transport_backend_name,"real-lcm");
    assert!(!handle.cmd_vel_sink_ready.load(Ordering::Acquire),"production spawn must not inject motion readiness");
    let mut service=GeometryRuntimeService::new(handle.clone());
    let identity=crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
        runtime_generation:handle.nav.producer_runtime_generation(),backend_generation:1,
        correction_lease:crate::correction_coordinator::CorrectionLeaseId {boot_nonce:uuid::Uuid::new_v4().as_u128(),generation:1}};
    let spec=crate::correction_coordinator::CorrectionOwnerSpec::Relocalization {
        map_frame_id:"map".into(),odom_frame_id:"odom".into(),body_frame_id:"base_link".into()};
    let applier=Arc::new(crate::lcm_runtime::GeometryCanonicalCorrectionApplier::new(identity.clone(),"map","odom","base_link",
        handle.state.clone(),handle.transport.clone(),(LcmTopic::new("/tf").unwrap(),LcmMessageType::new("tf2_msgs.TFMessage").unwrap())).unwrap());
    applier.bind_relocalization_mapping(&handle).unwrap();
    let sink=Arc::new(crate::correction_coordinator::GeometryCorrectionSink::new_for_spec(identity.clone(),&spec,"map","odom","base_link",applier.clone()).unwrap());
    let owner=crate::correction_coordinator::CorrectionTransportOwner::start(spec,identity,"",sink).await.unwrap();
    service.activation_receipts=owner.phase_a_control().activated_target_receipts();
    service.relocalization_ingress=owner.relocalization_ingress();
    service.canonical_correction=Some(applier.clone());
    let source=CanonicalResetFixture::new().await;
    let observer=LcmTransport::new_real(&std::env::var("NUWAX_LCM_URL").unwrap()).unwrap();
    let readers=EvidenceReaders::start(&observer).await;
    let graph=NavStackBuilder::with_topics(NavStackProfile::Mls3D,NavTopicConfig::geometry_runtime_defaults())
        .preset(NavStackPreset::Mls).build().unwrap();
    assert_eq!(graph.nodes().iter().map(|n|n.id.as_str()).collect::<Vec<_>>(),
        ["map_pose_adapter","mls_planner","surface_nav_scheduler"]);
    let mut runner=StackRunner::from_graph(graph);
    register_compiled_in_process_owners(&mut runner);
    runner.spawn_all().await.expect("production Mls3D owners start");
    let listener=tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address=listener.local_addr().unwrap();
    let incoming=futures::stream::unfold(listener,|listener|async {Some((listener.accept().await.map(|(s,_)|s),listener))});
    let (rpc_stop,rpc_stopped)=tokio::sync::oneshot::channel();
    let rpc_service=service.clone();
    let rpc=tokio::spawn(async move {tonic::transport::Server::builder().add_service(GeometryRuntimeServer::new(rpc_service))
        .serve_with_incoming_shutdown(incoming,async {let _=rpc_stopped.await;}).await});
    let mut sensor:Option<LiveSensor>=None;
    let outcome=std::panic::AssertUnwindSafe(async {
        let mut client=GeometryRuntimeClient::connect(format!("http://{address}")).await.unwrap();
        let world=scene();
        assert!(start_mapping_session(&source.service,"composition-source","floor-landmarks").await.active);
        commit_checkpoint_frame_test_pose(&source.handle,now_ns()).await;
        for _ in 0..3 {
            let stamp=now_ns();
            assert!(crate::lcm_runtime::handoff_registered_lio_cloud(&source.handle,&world,12,stamp,stamp,"map").await.unwrap());
            tokio::time::sleep(Duration::from_millis(110)).await;
        }
        let checkpoint=GeometryRuntime::export_mapping_backend_checkpoint(&source.service,Request::new(
            ExportMappingBackendCheckpointRequest {checkpoint_schema_version:2})).await.unwrap().into_inner();
        let fine=checkpoint.fine_map_artifact.as_ref().unwrap();
        let dense=checkpoint.dense_cloud_artifact.as_ref().unwrap();
        transfer_assigned_checkpoint_member(&source.service,&service,fine,b"NUWAXAR2").await;
        transfer_assigned_checkpoint_member(&source.service,&service,dense,b"NUWAXDC1").await;
        let activated=client.activate_map(ActivateMapRequest {
            activation_contract_version:2,activation_request_id:"composition-activate".into(),
            session_id:"composition-restored".into(),map_name:"floor-landmarks".into(),
            expected_runtime_generation:service.activation_receipts.runtime_generation(),
            asset_identity:Some(AssetIdentityV2 {asset_id:"b".repeat(32)}),
            mapping_backend_checkpoint:Some(checkpoint),..Default::default()
        }).await.unwrap().into_inner();
        assert!(activated.success && activated.relocalization_target_available,"{activated:?}");
        assert!(handle.mapping_adapter.as_ref().unwrap().lock().checkpoint_localization_required);
        let common=Arc::new(crate::lio_ingress::CommonLioIngress::new(
            crate::lio_ingress::DeviceLioIngressSpec::new(1,"sensor","base_link","odom","map").unwrap()));
        common.bind_relocalization_mode_fence(owner.ingress().relocalization_mode_fence()).unwrap();
        sensor_frame(&handle,&common,&owner.ingress(),&applier,&world,[0.0;3],1,false).await;
        let request=RelocalizeRequest {schema_version:2,request_id:"composition-relocalize".into(),
            assigned_target:Some(relocalize_request::AssignedTarget::AssignedCloudV2(AssignedCloudRelocalizeRequestV2 {
                target:activated.activation_receipt_v2.as_ref().unwrap().target.clone(),
                policy:activated.relocalization_policy_v2.clone(),logical_timeout_ms:30_000})),..Default::default()};
        let mut relocalize_client=client.clone();let command=request.clone();
        let pending=tokio::spawn(async move {relocalize_client.relocalize(command).await});
        tokio::time::timeout(Duration::from_secs(3),async {
            while !owner.ingress().relocalization_mode_snapshot().active {tokio::task::yield_now().await;}
        }).await.expect("real Relocalize command admitted");
        let registration_scan=world.iter().cycle().take(50_000).copied().collect::<Vec<_>>();
        sensor_frame(&handle,&common,&owner.ingress(),&applier,&registration_scan,[0.0;3],2,true).await;
        let terminal=tokio::time::timeout(Duration::from_secs(30),pending).await.unwrap().unwrap().unwrap().into_inner();
        assert_eq!(terminal.assigned_cloud_v2.as_ref().unwrap().disposition,RelocalizeDisposition::Applied as i32,"{terminal:?}");
        assert_eq!(applier.snapshot().unwrap().correction_revision,1);
        let sample=client.get_assigned_localization_sample(request).await.unwrap().into_inner();
        assert_eq!(sample.correction_revision,1);
        assert!(sample.map_from_odom.is_some() && sample.odom_from_body.is_some());
        let pose=sample.pose.unwrap().position.unwrap();
        assert!(pose.x.abs()<0.02 && pose.y.abs()<0.02 && pose.z.abs()<0.02,"known scene relocalization: {pose:?}");
        sensor=Some(LiveSensor::start(handle.clone(),common,owner.ingress(),applier.clone(),guard.root.join("actuator.jsonl"),world,2));
        let live=sensor.as_ref().unwrap();
        wait_ready(&mut client).await;
        // Two fresh sensor observations of each footprint cell are mandatory;
        // readiness alone can become true before the second observed hit.
        tokio::time::sleep(Duration::from_millis(350)).await;
        let id=if local_planning {"restored-on-arrival"} else {"restored-off-arrival"};
        let baseline=read_records::<Actuation>(&guard.root.join("actuator.jsonl")).len();
        let _admitted=submit_goal(&mut client,&guard.socket(),id,1.2,0.35).await;
        let goal=observed_goal(&readers.ledger,id).await;
        assert!((goal.goal.pose.position[0]-sample_position_x(&handle).await).abs()>=1.0,
            "arrival goal must require actual metre-scale movement");
        moving_evidence(&readers.ledger,live,&guard,&goal,baseline).await;
        let deadline=tokio::time::Instant::now()+Duration::from_secs(45);
        let arrived=loop {
            if let Some(state)=readers.ledger.lock().unwrap().states.iter().rev()
                .find(|s|goal_state_matches(s,&goal)&&s.state==3).cloned() {break state;}
            let failed=readers.ledger.lock().unwrap().states.iter().any(|s|goal_state_matches(s,&goal)&&s.state==4);
            assert!(!failed,"production navigation failed: {}",diagnostic(&readers.ledger,live));
            assert!(tokio::time::Instant::now()<deadline,"metre-scale arrival timeout: {}",diagnostic(&readers.ledger,live));
            runner.ensure_healthy().unwrap();
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        assert_exact_zero(&guard.socket(),&arrived).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        live.integrated.lock().unwrap().consume(&guard.root.join("actuator.jsonl"));
        let actual=live.pose();
        assert!((actual[0]-1.2).hypot(actual[1])<=POSITION_TOLERANCE+0.005,"integrated full XY arrival: {actual:?}");
        assert!((actual[2]-0.35).sin().atan2((actual[2]-0.35).cos()).abs()<=YAW_TOLERANCE+0.005,
            "integrated yaw arrival: {actual:?}");
        let actual_pose=handle.state.read().await.pose.clone();
        assert!(actual_pose.z.abs()<=POSITION_TOLERANCE,"full XYZ arrival includes canonical body Z: {:?}",
            (actual_pose.x,actual_pose.y,actual_pose.z));
        assert!(actual[0]>1.0 && live.integrated.lock().unwrap().distance>1.0,"real applied commands must cause simulated progress");
        assert_eq!(live.integrated.lock().unwrap().command,[0.0;3]);
        {
            let ledger=readers.ledger.lock().unwrap();
            let matching=ledger.states.iter().filter(|s|goal_state_matches(s,&goal)).collect::<Vec<_>>();
            assert_eq!(matching.iter().any(|s|!s.local_path.is_empty()),local_planning,
                "ON must admit an actual PlannerWorker/Scan candidate; OFF cannot display one");
            assert!(ledger.paths.iter().any(|p|p.session_id==id && p.goal_payload_sha256==goal.payload_sha256
                && p.path_revision==arrived.path_revision && p.path_payload_sha256==arrived.path_payload_sha256),
                "arrival consumes the exact production path identity");
            assert!(ledger.maps.iter().any(|m|m.fine.is_some() && !m.cleared && m.committed_map.session_id=="composition-restored"),
                "planner consumes restored fine-root publications");
        }
        let (path_count,map_count)={let ledger=readers.ledger.lock().unwrap();(ledger.paths.len(),ledger.maps.len())};
        eprintln!("ASSIGNED_COMPOSITION arrival local_planning={local_planning} simulated_pose={actual:?} zero_ack={} paths={path_count} maps={map_count}",arrived.zero_ack_id);

        // A separate moving goal proves expiry stopping before cancellation.
        wait_ready(&mut client).await;
        let baseline=read_records::<Actuation>(&guard.root.join("actuator.jsonl")).len();
        let admitted=submit_goal(&mut client,&guard.socket(),"restored-expiry",2.4,0.35).await;
        let expiry_goal=observed_goal(&readers.ledger,"restored-expiry").await;
        moving_evidence(&readers.ledger,live,&guard,&expiry_goal,baseline).await;
        let cut_time=now_ns();
        let cut_monotonic=monotonic_ns();
        live.cloud_enabled.store(false,Ordering::Release);
        let cut=read_records::<Actuation>(&guard.root.join("actuator.jsonl")).len();
        let deadline=tokio::time::Instant::now()+Duration::from_secs_f64(OBSERVATION_TTL+5.0);
        // Production receipt TTLs use a zero/indefinite sentinel. The finite
        // observation guard can correctly hold an APPLIED zero twist without
        // retiring the goal. Require that actual hold before requesting cancel.
        let (held,zero_event)=loop {
            let events:Vec<Actuation>=read_records(&guard.root.join("actuator.jsonl"));
            let (maturity,state)={
                let ledger=readers.ledger.lock().unwrap();
                // Re-read the actual publication ledger each time: a final
                // sensor frame already in flight at cutoff may publish later.
                let last_source=ledger.maps.iter().filter(|map|!map.cleared
                    && map.committed_map.session_id=="composition-restored" && map.source_timestamp_ns>0)
                    .map(|map|map.source_timestamp_ns as u64).max().expect("restored-map source evidence");
                let maturity=last_source.saturating_add((OBSERVATION_TTL*1e9) as u64);
                let state=ledger.states.iter().rev().find(|state| {
                    let stamp=state.stamp_sec as u64*1_000_000_000+state.stamp_nsec as u64;
                    let expiry=state.message.to_lowercase().contains("stale") || state.message.contains("过期")
                        || state.message.contains("时效") || state.message.contains("观察");
                    goal_state_matches(state,&expiry_goal) && stamp>=cut_time && stamp>=maturity && expiry
                        && (state.message.contains("零速") || state.message.contains("等待") || state.state==4)
                }).cloned();
                (maturity,state)
            };
            let mature_monotonic=cut_monotonic.saturating_add(maturity.saturating_sub(cut_time));
            let zero=events.iter().skip(cut).rev().find(|event|event.twist==[0.0;3]
                && !event.operation_id.is_empty() && event.at>=mature_monotonic).cloned();
            if now_ns()>=maturity {
                if let (Some(state),Some(zero))=(state,zero) {break (state,zero);}
            }
            assert!(tokio::time::Instant::now()<deadline,"freshness expiry must hold zero before cancellation: {}",diagnostic(&readers.ledger,live));
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        assert_applied(&guard.socket(),&zero_event).await;
        tokio::time::sleep(Duration::from_millis(150)).await;
        live.integrated.lock().unwrap().consume(&guard.root.join("actuator.jsonl"));
        assert_eq!(live.integrated.lock().unwrap().command,[0.0;3],"expiry holds zero before cancel");
        let quiet_events:Vec<Actuation>=read_records(&guard.root.join("actuator.jsonl"));
        assert!(quiet_events.iter().filter(|event|event.at>=zero_event.at)
            .all(|event|event.twist==[0.0;3]),"no nonzero burst is allowed after expiry hold and before cancellation");
        assert!((live.pose()[0]-2.4).abs()>POSITION_TOLERANCE,"expiry goal must not be mistaken for arrival");
        let cancel=CancelNavigationRequest {goal_id:admitted.goal_id,
            owner_kind:admitted.owner_kind,parent_request_id:admitted.parent_request_id,
            goal_generation:admitted.goal_generation,producer_runtime_generation:admitted.producer_runtime_generation};
        let pending=client.cancel_navigation(cancel.clone()).await.unwrap().into_inner();
        assert!(pending.cancel_generation>0,"actual held goal must enter cancellation: {pending:?}");
        let deadline=tokio::time::Instant::now()+Duration::from_secs(5);
        let terminal=loop {
            if let Some(state)=readers.ledger.lock().unwrap().states.iter().rev().find(|state|
                goal_state_matches(state,&expiry_goal) && state.state==0 && state.message.starts_with("CANCELLED:")
                && state.cancel_generation==pending.cancel_generation as i64
                && !state.zero_ack_id.is_empty() && state.zero_applied_monotonic_ns>0).cloned() {break state;}
            assert!(tokio::time::Instant::now()<deadline,"cancel must produce exact terminal fence after prior zero hold: {}",diagnostic(&readers.ledger,live));
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        assert_exact_zero(&guard.socket(),&terminal).await;
        let cancelled=client.cancel_navigation(cancel).await.unwrap().into_inner();
        assert!(cancelled.actuator_zeroed,"cancel replay resolves only with actual zero: {cancelled:?}");
        assert_eq!(cancelled.zero_ack_id,terminal.zero_ack_id,"cancel replays exact terminal fence");
        assert_eq!(cancelled.zero_applied_monotonic_ns,terminal.zero_applied_monotonic_ns as u64);
        eprintln!("ASSIGNED_COMPOSITION observation_expiry_hold local_planning={local_planning} reason={:?} applied_zero_operation={} cancellation_exact_zero={}",
            held.message,zero_event.operation_id,terminal.zero_ack_id);
    }).catch_unwind().await;
    // Always settle the production resources, including on assertion failure.
    let mut cleanup_errors=Vec::new();
    if let Some(sensor)=sensor.take() {if let Err(error)=sensor.finish().await {cleanup_errors.push(error);}}
    let _=rpc_stop.send(());
    match rpc.await {Ok(Ok(()))=>{},other=>cleanup_errors.push(format!("tonic shutdown: {other:?}"))}
    if let Err(error)=runner.shutdown().await {cleanup_errors.push(format!("navigation shutdown: {error}"));}
    if let Err(error)=owner.shutdown().await {cleanup_errors.push(format!("correction shutdown: {error}"));}
    let shutdown=destination.task_owner.shutdown().await;
    if !shutdown.producers_settled() {cleanup_errors.push("geometry producers did not settle".into());}
    if let Err(error)=shutdown.result {cleanup_errors.push(error);}
    // A live backend worker is a separate owned resource from the root tasks.
    let mapping=handle.mapping_adapter.as_ref().unwrap().clone();
    let retirement={
        let _lifecycle=handle.mapping_lifecycle_lock.lock().await;
        let mut adapter=mapping.lock();let current=adapter.snapshot();
        if current.active {adapter.stop_session_detached(&current.session_id).unwrap().retirement} else {None}
    };
    if let Err(error)=finish_backend_retirement(&mapping,&handle.mapping_lifecycle_lock,retirement).await {
        cleanup_errors.push(format!("mapping backend shutdown: {error}"));
    }
    readers.finish().await;
    source.finish().await;
    drop(mapping);drop(service);drop(handle);drop(destination);drop(observer);
    let mut guard=guard;guard.stop().await;
    if !cleanup_errors.is_empty() {eprintln!("ASSIGNED_COMPOSITION cleanup errors: {cleanup_errors:?}");}
    if let Err(panic)=outcome {std::panic::resume_unwind(panic);}
    assert!(cleanup_errors.is_empty(),"composition cleanup: {cleanup_errors:?}");
}
async fn sample_position_x(handle:&GeometryRuntimeHandle)->f64 {handle.state.read().await.pose.x}

fn isolated_case(name:&str,local_planning:bool) {
    if std::env::var(CHILD_MODE).as_deref()==Ok(name) {
        let _=tracing_subscriber::fmt().with_env_filter("warn,nuwax_nav_runtime=info,nuwax_nav_stack=info").try_init();
        let root=std::env::temp_dir().join(format!("assigned-nav-{}",uuid::Uuid::new_v4()));
        std::fs::create_dir(&root).unwrap();
        let config=[("NUWAX_LCM_URL",crate::lcm_runtime::test_lcm_url()),
            ("NUWAX_GEOMETRY_PIPELINE","lio_3d".into()),("NUWAX_NAV_PROFILE","pct-scan-3d".into()),
            ("NUWAX_GEOMETRY_SEQUENCE_STATE_PATH",root.join("sequence.json").display().to_string()),
            ("NUWAX_CLOUDMAP_SHM_PATH",root.join("cloud.mmap").display().to_string()),
            ("NUWAX_OCCUPANCY_SHM_PATH",root.join("occupancy.mmap").display().to_string()),
            ("NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH",root.join("explore.json").display().to_string()),
            ("NUWAX_FORBIDDEN_ZONES_PATH",root.join("forbidden.json").display().to_string()),
            ("NUWAX_NAV_BODY_TO_SUPPORT_Z_M","0.4".into()),("NUWAX_NAV_ROBOT_WIDTH_M","0.3".into()),
            ("NUWAX_NAV_ROBOT_HEIGHT_M","0.55".into()),("NUWAX_NAV_SURFACE_VOXEL_SIZE_M","0.1".into()),
            ("NUWAX_ROBOT_GEOMETRY_JSON",BODY.into()),("NUWAX_NAV_CAPABILITIES_JSON",CAPABILITIES.into()),
            ("NUWAX_NAV_LOCAL_PLANNING_ENABLED",local_planning.to_string())];
        for (key,value) in config {std::env::set_var(key,value);}
        let result=std::panic::catch_unwind(||tokio::runtime::Builder::new_multi_thread().worker_threads(4).enable_all().build().unwrap()
            .block_on(run_composition(local_planning,&root)));
        if result.is_ok() {std::fs::remove_dir_all(&root).unwrap();}
        else {eprintln!("ASSIGNED_COMPOSITION failure evidence retained at {}",root.display());}
        if let Err(panic)=result {std::panic::resume_unwind(panic);}
        return;
    }
    let mut command=Command::new(std::env::current_exe().unwrap());
    for (key,_) in std::env::vars_os() {
        if (key.to_string_lossy().starts_with("NUWAX_") && key!="NUWAX_TEST_GUARD_PYTHON")
            || key.to_string_lossy().starts_with("_NUWAX_NAV_ACTUATOR_") {command.env_remove(key);}
    }
    let full=format!("server::lifecycle_tests::assigned_navigation_composition::{name}");
    let result=command.args(["--exact",&full,"--nocapture"]).env(CHILD_MODE,name).output().unwrap();
    eprintln!("{}",String::from_utf8_lossy(&result.stdout));
    eprintln!("{}",String::from_utf8_lossy(&result.stderr));
    assert!(result.status.success(),"restored production composition subprocess failed: {}",result.status);
}
#[test]
fn assigned_v2_restored_production_navigation_local_off_arrives_and_expiry_stops() {
    isolated_case("assigned_v2_restored_production_navigation_local_off_arrives_and_expiry_stops",false);
}
#[test]
fn assigned_v2_restored_production_navigation_local_on_arrives_and_expiry_stops() {
    isolated_case("assigned_v2_restored_production_navigation_local_on_arrives_and_expiry_stops",true);
}

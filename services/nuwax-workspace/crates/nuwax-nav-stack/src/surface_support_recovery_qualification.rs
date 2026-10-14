// Opt-in software qualification: real same-context liblcm memq, map cache,
// production owners and PlannerWorker. Only the motion boundary is synthetic.
// The plant is an ideal velocity holder, not a physical braking/lease model.
// Run each ignored case exactly once, individually, in the full opt3 profile.
// This scenario lowers observed ground and restores it; it does not require
// unknown terrain to be an obstacle. Feature compilation alone is not a pass
// of either 60-second live qualification below.
use super::*;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::OnceLock;
use nuwax_map_chunks::{Domain, Manifest, Repository, Transfer, Writer, MAX_BLOCK_BYTES};
use nuwax_nav_runtime::{AppliedProof, BoundedTwistOutcome, CommandDispatchBound,
    FenceLeaseState, FenceProof, LeaseProof, MotionPortError, NavMotionAdmission,
    NavMotionPort, SchedulerConfig, SchedulerStatus};
use nuwax_nav_types_messages::{MotionLimits, MotionModel, NavigationCapabilities,
    RobotGeometry, TerrainLimits};

const SAMPLE_PERIOD: Duration = Duration::from_millis(200);
const QUALIFICATION_WINDOW: Duration = Duration::from_secs(60);
const SUPPORT_WAIT: &str = "当前机体几何检查未通过";
const RESOLUTION: f32 = 0.05;
type BodyPose = ([f64; 3], [f64; 4]);

fn calibrated_config(local_planning_enabled: bool) -> SurfaceNavigationConfig {
    let motion = MotionLimits {
        model: MotionModel::Differential, max_forward_mps: 0.5, max_reverse_mps: 0.0,
        max_lateral_mps: 0.0, max_angular_rps: 0.8, linear_acceleration_mps2: 1.0,
        guaranteed_braking_mps2: 1.0, angular_acceleration_rps2: 2.0,
        guaranteed_angular_braking_rps2: 2.0, reaction_time_sec: 0.05,
        minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
    };
    let caps = NavigationCapabilities {
        schema_version: 1, calibration_id: "generic-synthetic-memq-recovery".into(),
        motion: motion.clone(), observation: None,
        terrain: Some(TerrainLimits {
            max_step_up_m: 0.15, max_step_down_m: 0.15, min_tread_depth_m: 0.2,
            max_slope_deg: 18.0, max_cross_slope_deg: 18.0,
            support_height_uncertainty_m: 0.0, max_observation_age_sec: 1.0,
            stairs_supported: true, required_locomotion_mode: None,
            mode_transition_timeout_sec: 5.0,
        }),
    };
    let body = RobotGeometry {
        footprint_xy_m: vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],
        z_min_m: -0.05, z_max_m: 0.2, source: "generic synthetic qualification".into(),
    };
    let mut config = SurfaceNavigationConfig::new(
        "map".into(), 0.4, 0.3, 0.6, 0.1, 0.15, 18.0, 0.2).unwrap();
    config.apply_calibration(Some(body), Some(caps)).unwrap();
    config.follower.local_planning_enabled = local_planning_enabled;
    assert_eq!(config.follower.motion_limits.as_ref(), Some(&motion));
    assert_eq!(config.follower.max_observation_age_sec, Some(1.0));
    assert_eq!(config.follower.controller.control_frequency, 20.0);
    assert!(!config.follower.controller.simulation);
    config
}

#[derive(Clone, Debug)]
struct MotionEvent {
    seq: u64, at: Instant, unix_ns: i64, monotonic_ns: u64,
    kind: &'static str, operation: String, twist: Twist2,
    deadline: Option<Instant>, source: i64, session: Option<String>, revision: u64,
    body: BodyPose, ack: Option<String>,
}
struct Plant {
    origin: Instant, integrated_at: Instant, position: [f64; 3], yaw: f64,
    applied: Twist2, events: Vec<MotionEvent>,
}
impl Plant {
    fn new() -> Self {
        let now = Instant::now();
        Self { origin: now, integrated_at: now, position: [1.0,1.5,0.5], yaw: 0.0,
            applied: Twist2::ZERO, events: Vec::new() }
    }
    fn advance(&mut self, now: Instant) {
        let dt = now.duration_since(self.integrated_at).as_secs_f64();
        let (vx,vy,w) = (self.applied.linear.x,self.applied.linear.y,self.applied.angular_z);
        let end = self.yaw + w * dt;
        if w.abs() > 1e-12 {
            self.position[0] += (vx*(end.sin()-self.yaw.sin()) + vy*(end.cos()-self.yaw.cos()))/w;
            self.position[1] += (vx*(self.yaw.cos()-end.cos()) + vy*(end.sin()-self.yaw.sin()))/w;
        } else {
            self.position[0] += (vx*self.yaw.cos()-vy*self.yaw.sin())*dt;
            self.position[1] += (vx*self.yaw.sin()+vy*self.yaw.cos())*dt;
        }
        self.yaw = end;
        self.integrated_at = now;
    }
    fn body(&self) -> BodyPose {
        let q = Quaternion::from_yaw(self.yaw);
        (self.position,[q.x,q.y,q.z,q.w])
    }
    fn sample(&mut self) -> BodyPose { self.advance(Instant::now()); self.body() }
    fn record(&mut self, at: Instant, kind: &'static str, operation: &str,
        deadline: Option<Instant>, status: SchedulerStatus, ack: Option<String>) -> u64 {
        let seq = self.events.len() as u64 + 1;
        let ns = at.duration_since(self.origin).as_nanos().min(u128::from(u64::MAX)) as u64;
        assert!(ns > 0);
        let event = MotionEvent { seq, at, unix_ns: current_unix_timestamp_ns().unwrap(),
            monotonic_ns: ns, kind, operation: operation.into(), twist: self.applied, deadline,
            source: status.costmap_source_timestamp_ns, session: status.session_id,
            revision: status.path_revision, body: self.body(), ack };
        self.events.push(event);
        ns
    }
}
struct RealTimeMotion {
    plant: Arc<Mutex<Plant>>, scheduler: Arc<OnceLock<NavSchedulerHandle>>,
}
impl RealTimeMotion {
    fn status(&self) -> SchedulerStatus { self.scheduler.get().unwrap().status() }
    fn apply(&mut self, operation: &str, twist: Twist2, bound: Option<CommandDispatchBound>)
        -> BoundedTwistOutcome {
        let status = self.status();
        let mut plant = self.plant.lock().unwrap();
        let deadline = bound.and_then(CommandDispatchBound::monotonic_deadline);
        let at = Instant::now();
        plant.advance(at);
        // Last receiving check, after integrating the prior actual command.
        if bound.is_some_and(CommandDispatchBound::expired_now) {
            plant.record(Instant::now(), "expired", operation, deadline, status, None);
            return BoundedTwistOutcome::ExpiredNotApplied;
        }
        plant.applied = twist;
        let ns = plant.record(at, "applied", operation, deadline, status, None);
        BoundedTwistOutcome::Applied(AppliedProof {
            operation_id: operation.into(), applied_monotonic_ns: ns })
    }
}
#[async_trait::async_trait]
impl NavMotionPort for RealTimeMotion {
    async fn acquire(&mut self, operation: &str, admission: &NavMotionAdmission)
        -> Result<LeaseProof, MotionPortError> {
        let status = self.status(); let mut plant = self.plant.lock().unwrap();
        let at = Instant::now(); plant.advance(at);
        plant.record(at,"acquire",operation,None,status,None);
        Ok(LeaseProof { lease_id: 1, guard_generation: admission.guard_generation })
    }
    async fn renew(&mut self, operation: &str) -> Result<(), MotionPortError> {
        let status = self.status(); let mut plant = self.plant.lock().unwrap();
        let at = Instant::now(); plant.advance(at);
        plant.record(at,"renew",operation,None,status,None); Ok(())
    }
    async fn submit_twist(&mut self, operation: &str, twist: Twist2)
        -> Result<AppliedProof, MotionPortError> {
        match self.apply(operation,twist,None) {
            BoundedTwistOutcome::Applied(proof) => Ok(proof),
            BoundedTwistOutcome::ExpiredNotApplied => unreachable!(),
        }
    }
    async fn submit_twist_bounded(&mut self, operation: &str, twist: Twist2,
        bound: Option<CommandDispatchBound>) -> Result<BoundedTwistOutcome, MotionPortError> {
        Ok(self.apply(operation,twist,bound))
    }
    async fn revoke_and_zero(&mut self, operation: &str) -> Result<FenceProof, MotionPortError> {
        let status = self.status(); let mut plant = self.plant.lock().unwrap();
        let at = Instant::now(); plant.advance(at); plant.applied = Twist2::ZERO;
        let ack = format!("memq-fence-{}",plant.events.len()+1);
        let ns = plant.record(at,"fence",operation,None,status,Some(ack.clone()));
        assert_eq!(plant.applied,Twist2::ZERO);
        Ok(FenceProof { zero_ack_id: ack, applied_monotonic_ns: ns, lease: FenceLeaseState::Idle })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stage { Full, Lowered, Restored }
#[derive(Clone, Debug)]
struct SourceRecord {
    stage: Stage, sample: Instant, published: Instant, source: i64,
    authority: GuardedMapAuthority, root: nuwax_map_chunks::RootIdentity,
    root_geometry_revision: u64, retained_cells: usize,
    probe_key: [i32;3], probe_hits: Option<u16>, probe_observed_ns: Option<u64>,
}
#[derive(Clone)]
struct SensorControl { stage: Stage, lowered_region: Option<([f64;2],f64)> }
impl SensorControl {
    fn in_lowered_region(&self, key: &[i32;3]) -> bool {
        self.lowered_region.is_some_and(|(center,radius)| {
            let x=(key[0] as f64+0.5)*f64::from(RESOLUTION);
            let y=(key[1] as f64+0.5)*f64::from(RESOLUTION);
            (x-center[0]).abs() <= radius && (y-center[1]).abs() <= radius
        })
    }
    fn sample_keys(&self,ground:&[[i32;3]])->Vec<[i32;3]> {
        ground.iter().map(|&key|if self.stage==Stage::Lowered && self.in_lowered_region(&key) {
            [key[0],key[1],-8] // 45 cm below the original 5-cm voxel top.
        } else {key}).collect()
    }
}
// The existing wire contract uses either inline coarse points or a fine root.
// A live fine-root message must carry an explicitly empty coarse cloud.
fn map_from_fine(root: &Manifest, repository: &Repository) -> schema::NavigationMap3D {
    let mut map=map_message(root.revision as i64,&root.domain.session);
    map.source_voxel_size=root.voxel_size_m;
    map.source_timestamp_ns=root.source_timestamp_ns as i64;
    map.committed_map.source_timestamp_ns=map.source_timestamp_ns;
    map.cloud.header.stamp_sec=(map.source_timestamp_ns/1_000_000_000) as i32;
    map.cloud.header.stamp_nsec=(map.source_timestamp_ns%1_000_000_000) as i32;
    map.cloud.data.clear(); map.cloud.width=0; map.cloud.row_step=0;
    map.fine=Some(Transfer {root:root.encode().unwrap(),included:root.blocks.iter()
        .map(|block|(block.object,repository.read_object(block.object,MAX_BLOCK_BYTES).unwrap())).collect()});
    assert!(!map.cleared);
    assert_eq!(schema::navigation_map_3d_domain(&map),root.domain);
    map
}

#[test]
fn fine_map_known_drop_restore_codec_contract_roundtrips() {
    use nuwax_nav_mls_planner::surface::BodyGroundRelation;
    let repository=Repository::temporary().unwrap();
    let mut writer=Writer::new(repository.clone(),RESOLUTION,4*1024*1024).unwrap();
    let config=calibrated_config(false);
    let mut cache=SurfaceGeometryCache::configured(&config);
    let cells:Vec<_>=(0..100).flat_map(|x|(0..60).map(move |y|[x,y,1])).collect();
    for (index,stage) in [Stage::Full,Stage::Lowered,Stage::Restored].into_iter().enumerate() {
        let control=SensorControl {stage,lowered_region:Some(([1.1,1.5],1.1))};
        if stage==Stage::Lowered {writer.remove(cells.iter().copied().filter(|key|control.in_lowered_region(key))).unwrap();}
        if stage==Stage::Restored {writer.remove(cells.iter().filter(|key|control.in_lowered_region(key)).map(|key|[key[0],key[1],-8])).unwrap();}
        let retained=control.sample_keys(&cells);
        let stamp=current_unix_timestamp_ns().unwrap();
        writer.observe(retained.iter().copied(),stamp as u64).unwrap();
        let root=writer.snapshot(Domain {session:"memq-support-map".into(),generation:1,
            frame_generation:1,producer_generation:17},index as u64+1,stamp as u64).unwrap();
        let map=map_from_fine(&root,&repository);
        let encoded=schema::encode_navigation_map_3d_body(&map).unwrap();
        let decoded=schema::decode_navigation_map_3d_body(&encoded).unwrap();
        assert_eq!(decoded,map);
        assert!(!decoded.cleared && decoded.fine.is_some());
        assert!(decoded.cloud.data.is_empty());
        assert_eq!((decoded.cloud.width,decoded.cloud.row_step),(0,0));
        assert_eq!(decoded.cloud_revision,index as i64+1);
        assert_eq!(decoded.committed_map.revision,decoded.cloud_revision);
        assert_eq!(decoded.commit_sequence,decoded.committed_map.commit_sequence);
        assert_eq!(decoded.source_timestamp_ns,stamp);
        assert_eq!(decoded.committed_map.source_timestamp_ns,stamp);
        assert_eq!(decoded.source_voxel_size,RESOLUTION);
        let geometry=cache.build(&decoded).unwrap();
        let pose=([1.1,1.5,0.5],[0.0,0.0,0.0,1.0]);
        let expected=if stage==Stage::Lowered {BodyGroundRelation::KnownIncompatible}else{BodyGroundRelation::KnownCompatible};
        assert_eq!(geometry.body_ground_relation(pose.0,pose.1),Ok(expected));
        assert_eq!(geometry.validate_body_pose(pose.0,pose.1),Ok(()),
            "已知下方落差与恢复均不凭空制造机体占据碰撞");
        eprintln!("CODEC_SMOKE stage={stage:?} revision={} source_ns={stamp} fine_cells={} bytes={} cleared={} coarse_width={} coarse_row_step={}",
            decoded.cloud_revision,retained.len(),encoded.len(),decoded.cleared,decoded.cloud.width,decoded.cloud.row_step);
    }
    let control=SensorControl{stage:Stage::Lowered,lowered_region:Some(([1.1,1.5],1.1))};
    let points:Vec<_>=cells.iter().filter(|key|!control.in_lowered_region(key)).map(|key|(
        (key[0] as f32+0.5)*RESOLUTION,(key[1] as f32+0.5)*RESOLUTION,(key[2] as f32+0.5)*RESOLUTION)).collect();
    let unknown=nuwax_nav_mls_planner::SurfaceSnapshot::from_points(&points,config.geometry,RESOLUTION).unwrap()
        .with_body_envelope(config.body_envelope.unwrap()).unwrap();
    let pose=([1.1,1.5,0.5],[0.0,0.0,0.0,1.0]);
    assert_eq!(unknown.body_ground_relation(pose.0,pose.1),Ok(BodyGroundRelation::Unknown));
    assert_eq!(unknown.validate_body_pose(pose.0,pose.1),Ok(()),"Unknown不是已知落差，也不是碰撞");
    // This checks legal messages only, not 200ms sensor maturity or navigation.
}

struct Evidence {
    maps: BTreeMap<i64,SourceRecord>, built: BTreeMap<i64,PreparedSurfaceMap>,
    latest_map: Option<schema::NavigationMap3D>, paths: Vec<schema::PathEnvelope>,
    states: Vec<schema::NavigationState>,
}
struct RecoveryHarness {
    transport: LcmTransport, node: ModuleNode, scheduler: NavSchedulerHandle,
    planner: Option<Box<dyn InProcessNode>>, executor: Option<Box<dyn InProcessNode>>,
    plant: Arc<Mutex<Plant>>, sensor: Arc<Mutex<SensorControl>>,
    evidence: Arc<Mutex<Evidence>>, feeder: tokio::task::JoinHandle<()>,
    observers: Vec<tokio::task::JoinHandle<()>>, goal: Option<schema::GoalEnvelope>,
    started: Instant, local_planning_enabled: bool,
}
impl RecoveryHarness {
    async fn start(local_planning_enabled: bool) -> Self {
        Self::start_with_recent(local_planning_enabled,
            Arc::new(std::sync::atomic::AtomicBool::new(true))).await
    }
    async fn start_with_recent(local_planning_enabled: bool,
        publish_recent: Arc<std::sync::atomic::AtomicBool>) -> Self {
        use crate::topics::{NavTopicConfig,TopicName};
        let config=calibrated_config(local_planning_enabled);
        let transport=LcmTransport::new_real("memq://").unwrap();
        let planner_transport=transport.clone(); let executor_transport=transport.clone();
        let context=transport.shared_lcm_context().unwrap();
        assert!(Arc::ptr_eq(&context,&planner_transport.shared_lcm_context().unwrap()));
        assert!(Arc::ptr_eq(&context,&executor_transport.shared_lcm_context().unwrap()));
        let mut topics=NavTopicConfig::empty();
        for name in ["navigation_map3d","map_pose","goal_pose","nav_cancel","goal_path",
            "cmd_vel","nav_state","goal_reached","stop_movement"] {
            topics.topics.insert(name.into(),TopicName::new(format!("/memq-recovery/{name}")));
        }
        let graph=crate::profiles::build_graph(crate::NavStackProfile::Mls3D,&topics,
            crate::features::NavStackPreset::Mls.to_flags());
        let node=graph.find("mls_planner").unwrap().clone();
        let executor_node=graph.find("surface_nav_scheduler").unwrap().clone();
        let mut paths=typed_stream(&transport,&node.outputs[0],"nuwax_msgs.PathEnvelope").await.unwrap();
        let mut states=typed_stream(&transport,&node.inputs[4],"nuwax_msgs.NavigationState").await.unwrap();
        let hub=SurfaceMapHub::new(Arc::from("base_link")); let mut built=hub.events.subscribe();
        let evidence=Arc::new(Mutex::new(Evidence { maps:BTreeMap::new(),built:BTreeMap::new(),
            latest_map:None,paths:Vec::new(),states:Vec::new() }));
        let path_evidence=evidence.clone(); let state_evidence=evidence.clone(); let map_evidence=evidence.clone();
        let observers=vec![
            tokio::spawn(async move { while let Some(frame)=paths.next().await {
                let path=schema::decode_path_envelope_body(&frame.unwrap().payload).unwrap();
                path_evidence.lock().unwrap().paths.push(path);
            }}),
            tokio::spawn(async move { while let Some(frame)=states.next().await {
                let state=schema::decode_navigation_state_body(&frame.unwrap().payload).unwrap();
                state_evidence.lock().unwrap().states.push(state);
            }}),
            tokio::spawn(async move { while let Ok(map)=built.recv().await {
                if !map.awaiting_build { map_evidence.lock().unwrap().built.insert(map.source_timestamp_ns,map); }
            }}),
        ];
        let planner_hub=hub.clone(); let planner_node=node.clone(); let planner_config=config.clone();
        let mut planner=boxed_owner("memq-recovery-real-planner",Box::new(move |shutdown,ready| {
            Box::pin(run_surface_planner_with_transport(planner_node,planner_hub,planner_config,
                planner_transport,shutdown,ready))
        }));
        planner.start().unwrap();
        let plant=Arc::new(Mutex::new(Plant::new())); let handle_slot=Arc::new(OnceLock::new());
        let motion=RealTimeMotion {plant:plant.clone(),scheduler:handle_slot.clone()};
        let (sender,receiver)=std_mpsc::sync_channel(1);
        let mut scheduler_config=SchedulerConfig {robot_width_m:config.robot_width_m,..SchedulerConfig::default()};
        if let Some(age)=config.follower.max_observation_age_sec {
            scheduler_config.costmap_ttl=scheduler_config.costmap_ttl.min(Duration::from_secs_f64(age));
        }
        assert_eq!(scheduler_config.costmap_ttl,Duration::ZERO);
        let mut executor=boxed_owner("memq-recovery-real-executor",Box::new(move |shutdown,ready| {
            Box::pin(run_surface_scheduler_with_transport(executor_node,hub,executor_transport,
                shutdown,ready,config.follower.body_to_support_z_m,
                config.follower.max_observation_age_sec.map(Duration::from_secs_f64),async move {
                    let backend=nuwax_nav_runtime::qualification_surface_backend(config.follower)?;
                    let scheduler=nuwax_nav_runtime::NavScheduler::new(motion,backend,scheduler_config)?;
                    let pair=nuwax_nav_runtime::spawn_scheduler(scheduler);
                    sender.send(pair.0.clone()).unwrap(); Ok(pair)
                }))
        }));
        executor.start().unwrap();
        let scheduler=receiver.recv_timeout(Duration::from_secs(2)).unwrap();
        assert!(handle_slot.set(scheduler.clone()).is_ok());
        let sensor=Arc::new(Mutex::new(SensorControl {stage:Stage::Full,lowered_region:None}));
        let feed_sensor=sensor.clone(); let feed_plant=plant.clone(); let feed_evidence=evidence.clone();
        let feed_transport=transport.clone(); let feed_node=node.clone();
        let feeder=tokio::spawn(async move {
            let repository=Repository::temporary().unwrap();
            let mut writer=Writer::new(repository.clone(),RESOLUTION,4*1024*1024).unwrap();
            let all:Vec<_>=(0..100).flat_map(|x|(0..60).map(move |y|[x,y,1])).collect();
            let mut previous_stage=Stage::Full; let mut revision=0i64; let mut sequence=0;
            let mut next_sample=Instant::now();
            let mut ticker=tokio::time::interval(Duration::from_millis(25));
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                ticker.tick().await; sequence+=1;
                let body=feed_plant.lock().unwrap().sample(); let stamp=current_unix_timestamp_ns().unwrap();
                let pose=schema::PoseStamped {header:schema::Header {seq:sequence,
                    stamp_sec:(stamp/1_000_000_000) as i32,stamp_nsec:(stamp%1_000_000_000) as i32,
                    frame_id:"map".into()},pose:schema::Pose {position:body.0,orientation_xyzw:body.1}};
                publish_body(&feed_transport,&feed_node.inputs[1],"geometry_msgs.PoseStamped",
                    schema::encode_pose_stamped_body(&pose).unwrap()).await.unwrap();
                let pose_stamp=stamp;
                if Instant::now()<next_sample {continue;}
                let sample=Instant::now(); let stamp=current_unix_timestamp_ns().unwrap();
                next_sample=sample+SAMPLE_PERIOD; // no catch-up/backdated sensor frames
                let control=feed_sensor.lock().unwrap().clone();
                if control.stage==Stage::Lowered && previous_stage!=Stage::Lowered {
                    writer.remove(all.iter().copied().filter(|key|control.in_lowered_region(key))).unwrap();
                }
                if control.stage==Stage::Restored && previous_stage==Stage::Lowered {
                    writer.remove(all.iter().filter(|key|control.in_lowered_region(key)).map(|key|[key[0],key[1],-8])).unwrap();
                }
                let retained=control.sample_keys(&all);
                writer.observe(retained.iter().copied(),stamp as u64).unwrap(); revision+=1;
                let root=writer.snapshot(Domain {session:"memq-support-map".into(),generation:1,
                    frame_generation:1,producer_generation:17},revision as u64,stamp as u64).unwrap();
                let map=map_from_fine(&root,&repository);
                let payload=schema::encode_navigation_map_3d_body(&map).unwrap();
                let center=control.lowered_region.map_or([1.0,1.5],|lowered_region|lowered_region.0);
                let probe_key=[(center[0]/f64::from(RESOLUTION)).floor() as i32,
                    (center[1]/f64::from(RESOLUTION)).floor() as i32,if control.stage==Stage::Lowered {-8}else{1}];
                let probe=writer.region(probe_key,probe_key,1).unwrap().first().copied();
                if control.stage==Stage::Lowered {assert!(probe.is_some(),"lowered ground must be actually observed, not missing");}
                if control.stage==Stage::Restored && previous_stage==Stage::Lowered {
                    assert_eq!(probe.map(|(_,evidence)|evidence.hits),Some(1),
                        "restored deleted fine cell must begin with one real acquisition");
                }
                let record=SourceRecord {stage:control.stage,sample,published:Instant::now(),source:stamp,
                    authority:map_authority(&map),root:root.identity(),
                    root_geometry_revision:root.geometry_revision,retained_cells:retained.len(),probe_key,
                    probe_hits:probe.map(|(_,evidence)|evidence.hits),
                    probe_observed_ns:probe.map(|(_,evidence)|evidence.observed_ns)};
                publish_body(&feed_transport,&feed_node.inputs[0],"nuwax_msgs.NavigationMap3D",
                    payload).await.unwrap();
                if publish_recent.load(std::sync::atomic::Ordering::Relaxed) {
                    publish_sampled_recent(&feed_transport,&map,sequence,stamp,pose_stamp,
                        retained.iter().copied().map(|key|(key,true))).await;
                }
                // Only a successfully encoded and published source enters the ledger.
                { let mut evidence=feed_evidence.lock().unwrap();
                    evidence.maps.insert(stamp,record); evidence.latest_map=Some(map); }
                previous_stage=control.stage;
            }
        });
        Self {transport,node,scheduler,planner:Some(planner),executor:Some(executor),plant,sensor,
            evidence,feeder,observers,goal:None,started:Instant::now(),local_planning_enabled}
    }
    fn check_live(&self,stage:&str)->Result<(),String> {
        if self.started.elapsed()>QUALIFICATION_WINDOW {return Err(format!("{stage}: 60s qualification window exhausted"));}
        if self.feeder.is_finished() || self.observers.iter().any(|task|task.is_finished()) {
            return Err(format!("{stage}: feeder/observer exited"));
        }
        let status=self.scheduler.status();
        if let Some(terminal)=status.last_terminal {
            if terminal!=TerminalOutcome::Succeeded {return Err(format!("{stage}: unexpected terminal {status:?}"));}
        }
        Ok(())
    }
    fn source(&self,stamp:i64,stage:Stage)->bool {
        self.evidence.lock().unwrap().maps.get(&stamp).is_some_and(|row|row.stage==stage)
    }
    fn fresh_body(&self,source:i64,body:BodyPose)->bool {
        let evidence=self.evidence.lock().unwrap();
        let Some(built)=evidence.built.get(&source) else {return false;};
        let Ok(map)=&built.snapshot else {return false;};
        let Some(record)=evidence.maps.get(&source) else {return false;};
        // This is acquisition health, not per-cell ground permission.
        if record.sample.elapsed()>Duration::from_secs(1) {return false;}
        let mut ahead=body; ahead.0[0]+=0.05;
        map.validate_body_pose(body.0,body.1).is_ok() && map.validate_body_motion(body,ahead,&[]).is_ok()
    }
    fn current_body_supported(&self,event:&MotionEvent)->bool {
        let evidence=self.evidence.lock().unwrap();
        let Some(built)=evidence.built.get(&event.source) else {return false;};
        let Ok(map)=&built.snapshot else {return false;};
        if map.body_ground_relation(event.body.0,event.body.1)!=Ok(nuwax_nav_mls_planner::surface::BodyGroundRelation::KnownCompatible) {return false;}
        // Match refresh_current_body_view: use its derived view when available,
        // otherwise retain the source snapshot. This is geometry, not proof replay.
        match map.with_current_body_support(event.body.0,event.body.1) {
            Ok(view)=>view.validate_body_pose(event.body.0,event.body.1).is_ok(),
            Err(_)=>map.validate_body_pose(event.body.0,event.body.1).is_ok(),
        }
    }
    fn unsupported(&self,event:&MotionEvent)->bool {
        let evidence=self.evidence.lock().unwrap();
        let Some(built)=evidence.built.get(&event.source) else {return false;};
        let Ok(map)=&built.snapshot else {return false;};
        map.body_ground_relation(event.body.0,event.body.1)==Ok(nuwax_nav_mls_planner::surface::BodyGroundRelation::KnownIncompatible)
    }
    fn validate_paths(&self,goal:&schema::GoalEnvelope)->Result<(),String> {
        let events=self.plant.lock().unwrap().events.clone();
        let evidence=self.evidence.lock().unwrap(); let mut previous:Option<&schema::PathEnvelope>=None;
        for path in &evidence.paths {
            validate_path_goal_correlation(path,goal)?;
            if let Some(old)=previous {
                if path.path_revision<old.path_revision || path.planner_runtime_generation<old.planner_runtime_generation {
                    return Err("production path identity/revision regressed".into());
                }
                if path.path_revision==old.path_revision && path!=old {return Err("same revision changed payload".into());}
            }
            previous=Some(path);
        }
        // Success clears SchedulerStatus.path_revision. Use actual call-time
        // accepted revisions and the original goal's published execution states.
        let mut previous_accepted=0;
        let accepted:Vec<_>=events.iter().filter(|event|
            event.session.as_deref()==Some(goal.session_id.as_str()) && event.revision>0).collect();
        if accepted.is_empty() {return Err("no accepted execution revision recorded".into());}
        for event in accepted {
            if event.revision<previous_accepted {return Err("actual accepted revision regressed".into());}
            if !evidence.paths.iter().any(|path|path.path_revision as u64==event.revision) {
                return Err(format!("motion event {} accepted revision {} absent from wire evidence",event.seq,event.revision));
            }
            previous_accepted=event.revision;
        }
        for state in evidence.states.iter().filter(|state|state_matches_goal(state,goal) && state.path_revision>0) {
            if !evidence.paths.iter().any(|path|path.path_revision==state.path_revision) {
                return Err(format!("nav_state accepted revision {} absent from wire evidence",state.path_revision));
            }
        }
        Ok(())
    }
    async fn exercise(&mut self)->Result<(),String> {
        // Read the actual built/consumed fine source, never scheduler freshness alone.
        loop {
            self.check_live("mature start")?;
            let source=self.scheduler.status().costmap_source_timestamp_ns;
            let body=self.plant.lock().unwrap().sample();
            if self.source(source,Stage::Full) && self.fresh_body(source,body) {break;}
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let map=self.evidence.lock().unwrap().latest_map.clone().unwrap();
        let requested_options=nuwax_nav_types_messages::GoalOptions {
            allow_reverse:false,position_tolerance_m:0.1,yaw_tolerance_rad:0.15 };
        let goal=goal_for_with_options(&map,1,[3.5,1.5,0.5],requested_options);
        assert_eq!(goal.options,requested_options); // one original protocol sealing call
        assert!(goal.options.requires_orientation());
        let original_goal=goal.clone(); self.goal=Some(goal.clone());
        eprintln!("GOAL at={:?} unix_ns={} options={:?} original_goal={}",Instant::now(),
            current_unix_timestamp_ns().unwrap(),goal.options,goal.session_id);
        publish_body(&self.transport,&self.node.inputs[2],"nuwax_msgs.GoalEnvelope",
            schema::encode_goal_envelope_body(&goal).unwrap()).await.unwrap();
        let initial=self.plant.lock().unwrap().sample();
        loop {
            self.check_live("initial applied movement")?;
            let plant=self.plant.lock().unwrap();
            let moved=distance3(plant.body().0,initial.0)>0.10;
            let nonzero=plant.events.iter().any(is_nonzero_applied);
            drop(plant);
            if moved && nonzero && !self.evidence.lock().unwrap().paths.is_empty() {break;}
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        if self.scheduler.status().last_terminal.is_some() {return Err("goal terminated before lowering".into());}
        let (lowered_body,lowered_seq)={let mut plant=self.plant.lock().unwrap();
            (plant.sample(),plant.events.len() as u64)};
        if distance3(lowered_body.0,goal.goal.pose.position)<=goal.options.position_tolerance_m {
            return Err("lowering was not before arrival".into());
        }
        // Lower the actually observed ground across the footprint + original
        // 5-cell halo + 0.35m input-adoption drift margin; this is not a blind gap.
        let config=calibrated_config(self.local_planning_enabled);
        let halo=((config.follower.body_to_support_z_m+f64::from(RESOLUTION)) /
            f64::from(config.geometry.voxel_size)).ceil()*f64::from(config.geometry.voxel_size);
        let radius=config.body_envelope.as_ref().unwrap().radius()+halo+0.35;
        {let mut sensor=self.sensor.lock().unwrap();sensor.stage=Stage::Lowered;
            sensor.lowered_region=Some(([lowered_body.0[0],lowered_body.0[1]],radius));}
        eprintln!("LOWERED_GROUND at={:?} unix_ns={} cursor={lowered_seq} body={lowered_body:?} radius={radius}",
            Instant::now(),current_unix_timestamp_ns().unwrap());
        let stopped=loop {
            self.check_live("post-lowering support failure and fresh zero")?;
            if self.scheduler.status().last_terminal.is_some() {return Err("unexpected terminal during known drop".into());}
            let events=self.plant.lock().unwrap().events.clone();
            let found=events.into_iter().find(|event| {
                if event.seq<=lowered_seq || !is_zero_event(event) || !self.source(event.source,Stage::Lowered)
                    || event.session.as_deref()!=Some(goal.session_id.as_str()) || !self.unsupported(event) {return false;}
                let evidence=self.evidence.lock().unwrap(); let row=&evidence.maps[&event.source];
                event.at>=row.published && evidence.states.iter().any(|state|
                    state_matches_goal(state,&goal) && state.costmap_source_timestamp_ns==event.source
                    && state.message.contains(SUPPORT_WAIT))
            });
            if let Some(event)=found {break event;}
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        let (zero_pose,hold_start)={let mut plant=self.plant.lock().unwrap();(plant.sample(),Instant::now())};
        while hold_start.elapsed()<SAMPLE_PERIOD*3 {
            self.check_live("known-drop stationary hold")?;
            let mut plant=self.plant.lock().unwrap(); let pose=plant.sample();
            if plant.applied!=Twist2::ZERO || distance3(pose.0,zero_pose.0)>1e-12 ||
                plant.events.iter().any(|event|event.seq>stopped.seq && is_nonzero_applied(event)) {
                return Err("post-lowering plant moved or accepted another nonzero command".into());
            }
            drop(plant);
            let status=self.scheduler.status();
            if status.session_id.as_deref()!=Some(goal.session_id.as_str()) || status.last_terminal.is_some() {
                return Err(format!("original goal did not remain waiting: {status:?}"));
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let evidence=self.evidence.lock().unwrap();
        let lowered_samples=evidence.maps.values().filter(|row|row.stage==Stage::Lowered && row.published>=stopped.at).count();
        if lowered_samples<2 {return Err("fewer than two actual post-lowering samples during hold".into());}
        let publish_to_zero_ms=stopped.at.duration_since(evidence.maps[&stopped.source].published).as_secs_f64()*1000.0;
        drop(evidence);
        eprintln!("STOP at={:?} unix_ns={} source={} publish_to_zero_ms={publish_to_zero_ms} zero_seq={} wait_reason={} hold_ms={} samples={lowered_samples}",
            stopped.at,stopped.unix_ns,stopped.source,stopped.seq,SUPPORT_WAIT,hold_start.elapsed().as_millis());
        let restore_seq=self.plant.lock().unwrap().events.len() as u64;
        self.sensor.lock().unwrap().stage=Stage::Restored; // only map and ordinary measured pose continue
        eprintln!("RESTORE at={:?} unix_ns={} cursor={restore_seq}",Instant::now(),current_unix_timestamp_ns().unwrap());
        let resumed=loop {
            self.check_live("original goal natural recovery")?;
            let events=self.plant.lock().unwrap().events.clone();
            let found=events.into_iter().find(|event| event.seq>restore_seq && is_nonzero_applied(event)
                && event.session.as_deref()==Some(goal.session_id.as_str()) && self.source(event.source,Stage::Restored));
            if let Some(event)=found {break event;}
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        // Restored measured height must be compatible and real geometry clear;
        // neither a cell hit quorum nor a ground expiry lease grants recovery.
        if !self.current_body_supported(&resumed) {return Err("restored current body lacks original geometric support".into());}
        eprintln!("RESTORED_APPLIED at={:?} unix_ns={} applied_seq={} consumed_source={} original_goal={} revision={} deadline={:?} current_body_geometry=valid exact_selected_proof_cells=not_recorded",
            resumed.at,resumed.unix_ns,resumed.seq,resumed.source,goal.session_id,resumed.revision,resumed.deadline);
        loop {
            self.check_live("restored net progress")?;
            let body=self.plant.lock().unwrap().sample();
            if distance3(body.0,zero_pose.0)>0.10 && distance3(body.0,goal.goal.pose.position)<distance3(zero_pose.0,goal.goal.pose.position) {break;}
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        eprintln!("RECOVERY at={:?} unix_ns={} applied_seq={} consumed_source={} original_goal={} revision={}",
            resumed.at,resumed.unix_ns,resumed.seq,resumed.source,goal.session_id,resumed.revision);
        loop {
            self.check_live("strict original goal arrival")?;
            let status=self.scheduler.status();
            let wire_success=self.evidence.lock().unwrap().states.iter().any(|state|
                state_matches_goal(state,&goal) && state.state==3 && state.zero_ack_id==status.last_zero_ack_id.clone().unwrap_or_default()
                && state.zero_applied_monotonic_ns as u64==status.last_fence_monotonic_ns);
            if status.last_terminal==Some(TerminalOutcome::Succeeded) && wire_success {break;}
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        self.validate_paths(&goal)?;
        if self.goal.as_ref()!=Some(&original_goal) {return Err("original goal mutated".into());}
        let status=self.scheduler.status(); let mut plant=self.plant.lock().unwrap(); let arrived=plant.sample();
        let xyz=distance3(arrived.0,goal.goal.pose.position);
        let goal_q=goal.goal.pose.orientation_xyzw;
        let goal_yaw=Quaternion {x:goal_q[0],y:goal_q[1],z:goal_q[2],w:goal_q[3]}.yaw();
        let yaw=(plant.yaw-goal_yaw).sin().atan2((plant.yaw-goal_yaw).cos()).abs();
        if xyz>=goal.options.position_tolerance_m || (goal.options.requires_orientation() && yaw>=goal.options.yaw_tolerance_rad)
            || plant.applied!=Twist2::ZERO {return Err(format!("strict arrival failed xyz={xyz} yaw={yaw} velocity={:?}",plant.applied));}
        let fence=plant.events.iter().find(|event|event.kind=="fence" && event.seq>resumed.seq &&
            event.ack==status.last_zero_ack_id && event.monotonic_ns==status.last_fence_monotonic_ns)
            .ok_or("success lacks its new actual fence event")?;
        let fence_seq=fence.seq;
        drop(plant);
        eprintln!("QUALIFIED at={:?} unix_ns={} local_planning_enabled={} xyz_error={xyz} yaw_error={yaw} position_tolerance={} yaw_tolerance={} fence_seq={} elapsed_s={} context=shared_real_memq new_global_planning_input_revision=not_independently_proven physical_braking=not_proven uds=not_covered",
            Instant::now(),current_unix_timestamp_ns().unwrap(),self.local_planning_enabled,goal.options.position_tolerance_m,goal.options.yaw_tolerance_rad,
            fence_seq,self.started.elapsed().as_secs_f64());
        Ok(())
    }
    fn dump_evidence(&self) {
        let events=self.plant.lock().unwrap().events.clone();
        let (sources,paths,states)={let evidence=self.evidence.lock().unwrap();
            (evidence.maps.values().cloned().collect::<Vec<_>>(),evidence.paths.clone(),evidence.states.clone())};
        // No per-call I/O while holding the motion boundary or sensor locks.
        for event in events {eprintln!("MOTION {event:?}");}
        for source in sources {eprintln!("SOURCE {source:?}");}
        for path in paths {eprintln!("PATH {path:?}");}
        for state in states {eprintln!("STATE {state:?}");}
    }
    async fn shutdown(&mut self)->Result<(),String> {
        let cancel_cursor=self.plant.lock().unwrap().events.len() as u64;
        let before=self.scheduler.status();
        if before.session_id.is_some() {
            let goal=self.goal.as_ref().ok_or("active cleanup session lacks its original goal")?;
            publish_body(&self.transport,&self.node.inputs[3],"nuwax_msgs.CancelNavigation",
                schema::encode_cancel_navigation_body(&cancel_for(goal)).unwrap()).await.map_err(|e|e.to_string())?;
            tokio::time::timeout(Duration::from_secs(3),async {
                loop {
                    let status=self.scheduler.status();
                    if status.session_id.is_none() && matches!(status.last_terminal,
                        Some(TerminalOutcome::Cancelled | TerminalOutcome::Succeeded)) {
                        let wire=self.evidence.lock().unwrap().states.iter().any(|state|
                            state_matches_goal(state,goal) && state.zero_ack_id==status.last_zero_ack_id.clone().unwrap_or_default()
                            && state.zero_applied_monotonic_ns as u64==status.last_fence_monotonic_ns
                            && ((status.last_terminal==Some(TerminalOutcome::Cancelled) && state.message.starts_with("CANCELLED:"))
                                || (status.last_terminal==Some(TerminalOutcome::Succeeded) && state.state==3)));
                        if wire {break;}
                    }
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            }).await.map_err(|_|"failure cleanup did not reach correlated Cancelled/Succeeded".to_string())?;
            let status=self.scheduler.status(); let mut plant=self.plant.lock().unwrap(); plant.sample();
            let fence=plant.events.iter().find(|event|event.kind=="fence" && event.seq>cancel_cursor
                && event.ack==status.last_zero_ack_id && event.monotonic_ns==status.last_fence_monotonic_ns
                && event.session.as_deref()==Some(goal.session_id.as_str()) && event.twist==Twist2::ZERO);
            if fence.is_none() || plant.applied!=Twist2::ZERO {return Err("cleanup lacks new correlated actual zero fence".into());}
            let fence_seq=fence.unwrap().seq; let actual_velocity=plant.applied;
            drop(plant);
            eprintln!("CLEANUP original_goal={} terminal={:?} cancel_cursor={} fence_seq={} actual_velocity={actual_velocity:?}",
                goal.session_id,status.last_terminal,cancel_cursor,fence_seq);
        }
        self.feeder.abort(); let _=(&mut self.feeder).await;
        for observer in &mut self.observers {observer.abort();let _=observer.await;}
        for owner in [&mut self.executor,&mut self.planner] {
            if let Some(mut running)=owner.take() {
                running.shutdown().map_err(|e|e.to_string())?;
                if running.is_running().map_err(|e|e.to_string())? {return Err("owner failed to stop".into());}
            }
        }
        Ok(())
    }
}
impl Drop for RecoveryHarness {
    fn drop(&mut self) {
        self.feeder.abort(); for task in &self.observers {task.abort();}
        for owner in [&mut self.executor,&mut self.planner] {
            if let Some(mut running)=owner.take() {let _=running.shutdown();}
        }
    }
}
fn distance3(a:[f64;3],b:[f64;3])->f64 {a.into_iter().zip(b).map(|(x,y)|(x-y).powi(2)).sum::<f64>().sqrt()}
fn is_nonzero_applied(event:&MotionEvent)->bool {event.kind=="applied" && event.twist!=Twist2::ZERO}
fn is_zero_event(event:&MotionEvent)->bool {(event.kind=="applied" || event.kind=="fence") && event.twist==Twist2::ZERO}
async fn qualify(enabled:bool) {
    let mut harness=RecoveryHarness::start(enabled).await;
    let result=harness.exercise().await;
    let status=harness.scheduler.status();
    let cleanup=harness.shutdown().await;
    harness.dump_evidence();
    eprintln!("FINAL local_planning_enabled={enabled} result={result:?} status={status:?} cleanup={cleanup:?}");
    assert!(result.is_ok(),"{result:?}; cleanup={cleanup:?}");
    assert!(cleanup.is_ok(),"{cleanup:?}");
}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="opt-in real-clock memq software qualification; full opt3, one recorded run"]
async fn original_goal_known_drop_recovers_and_arrives_local_planning_off() {qualify(false).await;}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="opt-in real-clock memq software qualification; full opt3, one recorded run"]
async fn original_goal_known_drop_recovers_and_arrives_local_planning_on() {qualify(true).await;}

// The synthetic sensor acquires one return at each sampled voxel center. Feed
// those same current returns to the live channel, never the accumulated Writer
// or a map replay. Static map publication alone cannot establish source health.
pub(super) async fn publish_sampled_recent(
    transport: &LcmTransport, map: &schema::NavigationMap3D, sequence: i32, stamp: i64, pose_stamp: i64,
    returns: impl IntoIterator<Item=([i32;3],bool)>,
) {
    use nuwax_codec::lcm::{lio,recent_cloud};
    let mut data=Vec::new(); let mut ground_point_indices=Vec::new(); let mut count=0;
    for (index,(key,ground)) in returns.into_iter().enumerate() {
        if ground {ground_point_indices.push(index as u32);}
        for coordinate in key {data.extend_from_slice(&((coordinate as f32+0.5)*RESOLUTION).to_le_bytes());}
        count+=1;
    }
    assert!(count>0,"a live scan must contain actual newly sampled returns");
    let cloud=schema::PointCloud2 {
        header:schema::Header {seq:sequence,stamp_sec:(stamp/1_000_000_000) as i32,
            stamp_nsec:(stamp%1_000_000_000) as i32,frame_id:"map".into()},
        height:1,width:count,fields:["x","y","z"].into_iter().enumerate().map(|(i,name)|schema::PointField {
            name:name.into(),offset:(i*4) as i32,datatype:7,count:1,
        }).collect(),is_bigendian:false,point_step:12,row_step:count*12,data,is_dense:true,
    };
    let packet=recent_cloud::RecentCloud3D {
        producer_runtime_generation:map.producer_runtime_generation,
        session_id:map.committed_map.session_id.clone(),
        session_generation:map.committed_map.session_generation,
        map_frame_generation:map.map_frame_generation,commit_boot_nonce:map.committed_map.commit_boot_nonce,
        ground_point_indices,cloud:Some(lio::LioCloudEnvelope {
            schema_version:1,backend:lio::LIO_BACKEND_DEVICE,runtime_generation:17,
            source_sequence:i64::from(sequence),source_timestamp_ns:stamp,
            correlated_odometry_source_sequence:i64::from(sequence),correlated_odometry_source_timestamp_ns:pose_stamp,
            sensor_frame_id:"synthetic_sensor".into(),body_frame_id:"base_link".into(),
            odom_frame_id:"odom".into(),map_frame_id:"map".into(),
            registration_state:lio::LIO_REGISTRATION_REGISTERED_MAP,
            source_point_count:i64::from(count),emitted_point_count:i64::from(count),cloud,
        }),
    };
    publish_body(transport,recent_cloud::DEFAULT_TOPIC,recent_cloud::TYPE_NAME,
        recent_cloud::encode_body(&packet).unwrap()).await.unwrap();
}

#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="bounded real-owner startup regression; run in full opt3"]
async fn live_source_is_required_before_goal_admission() {
    let live=Arc::new(std::sync::atomic::AtomicBool::new(false));
    let mut harness=RecoveryHarness::start_with_recent(false,live.clone()).await;
    let result=async {
        tokio::time::timeout(Duration::from_secs(3),async {
            loop {
                let status=harness.scheduler.status();
                if status.costmap_source_timestamp_ns>0 && status.pose_source_timestamp_ns>0
                    && harness.evidence.lock().unwrap().maps.len()>=3 {break;}
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }).await.map_err(|_|"map/pose startup timeout")?;
        assert!(!harness.scheduler.status().planner_inputs_fresh);
        let map=harness.evidence.lock().unwrap().latest_map.clone().unwrap();
        let goal=goal_for_with_options(&map,1,[3.5,1.5,0.5],nuwax_nav_types_messages::GoalOptions {
            allow_reverse:false,position_tolerance_m:0.1,yaw_tolerance_rad:0.15 });
        harness.goal=Some(goal.clone());
        publish_body(&harness.transport,&harness.node.inputs[2],"nuwax_msgs.GoalEnvelope",
            schema::encode_goal_envelope_body(&goal).unwrap()).await.unwrap();
        tokio::time::sleep(Duration::from_millis(500)).await;
        let missing=harness.scheduler.status();
        assert!(!missing.planner_inputs_fresh && missing.session_id.is_none());
        assert_eq!(missing.applied_commands,0);
        assert!(harness.evidence.lock().unwrap().paths.is_empty());
        live.store(true,std::sync::atomic::Ordering::Relaxed);
        tokio::time::timeout(Duration::from_secs(8),async {
            loop {
                if harness.plant.lock().unwrap().events.iter().any(is_nonzero_applied) {break;}
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }).await.map_err(|_|"live source failed to release buffered original goal")?;
        assert_eq!(harness.scheduler.status().session_id.as_deref(),Some(goal.session_id.as_str()));
        Ok::<(),&str>(())
    }.await;
    let cleanup=harness.shutdown().await;
    harness.dump_evidence();
    assert!(result.is_ok() && cleanup.is_ok(),"startup={result:?} cleanup={cleanup:?}");
}

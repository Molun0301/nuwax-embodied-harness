// Opt-in software qualification: real same-context liblcm memq, map cache,
// production owners and PlannerWorker. Only the motion boundary is synthetic.
// The plant is an ideal velocity holder, not a physical braking/lease model.
// Run each ignored case exactly once, individually, in the full opt3 profile.
use super::*;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::OnceLock;
use nuwax_map_chunks::{Domain, Manifest, Repository, Transfer, Writer, MAX_BLOCK_BYTES};
use nuwax_nav_runtime::{AppliedProof, BoundedTwistOutcome, CommandDispatchBound,
    FenceLeaseState, FenceProof, LeaseProof, MotionPortError, NavMotionAdmission,
    NavMotionPort, SchedulerConfig, SchedulerStatus};
use nuwax_nav_types_messages::{MotionLimits, MotionModel, NavigationCapabilities,
    RobotGeometry, TerrainLimits};

mod dense_recent { include!("surface_dense_recent_qualification.rs"); }

const SAMPLE_PERIOD: Duration = Duration::from_millis(200);
const QUALIFICATION_WINDOW: Duration = Duration::from_secs(60);
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
    applied: Twist2, events: Vec<MotionEvent>, segments: Vec<ActualSegment>, inserted: Option<Instant>,
}
impl Plant {
    fn new() -> Self {
        let now = Instant::now();
        Self { origin: now, integrated_at: now, position: [1.0,1.5,0.5], yaw: 0.0,
            applied: Twist2::ZERO, events: Vec::new(), segments: Vec::new(), inserted: None }
    }
    fn advance(&mut self, now: Instant) {
        let from=self.integrated_at;
        let start=OraclePose {position:self.position,yaw:self.yaw};
        let dt = now.duration_since(from).as_secs_f64();
        let (position,yaw)=stable_plant_integrate(self.position,self.yaw,self.applied,dt);
        self.position=position;self.yaw=yaw;
        self.integrated_at = now;
        if now>from {self.segments.push(ActualSegment {from,to:now,
            motion:OracleSegment {start,end:OraclePose {position:self.position,yaw:self.yaw},
                twist:self.applied,duration_s:dt,obstacle_active:self.inserted.is_some_and(|at|from>=at)}});}
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
// Test-only ideal plant integration. This uses body-local sinc/cosc increments,
// independently of the oracle's midpoint-yaw/half-angle formulation. There is
// no motion gate, timing change, or modification of the applied command.
fn stable_plant_integrate(position:[f64;3],yaw:f64,twist:Twist2,dt:f64)->([f64;3],f64) {
    let a=twist.angular_z*dt;
    let (sinc,cosc)=if a.abs()<1e-4 {
        let a2=a*a;
        (1.0-a2/6.0+a2*a2/120.0-a2*a2*a2/5040.0,
            a*(0.5-a2/24.0+a2*a2/720.0-a2*a2*a2/40320.0))
    } else {
        // 2*sin(a/2)^2 is 1-cos(a) without cancellation.
        (a.sin()/a,2.0*(a/2.0).sin().powi(2)/a)
    };
    let dx=dt*(twist.linear.x*sinc-twist.linear.y*cosc);
    let dy=dt*(twist.linear.x*cosc+twist.linear.y*sinc);
    let (sin_yaw,cos_yaw)=yaw.sin_cos();
    ([position[0]+cos_yaw*dx-sin_yaw*dy,position[1]+sin_yaw*dx+cos_yaw*dy,position[2]],yaw+a)
}
#[test]
fn stable_plant_preserves_recorded_near_zero_counterexample_and_normal_arcs() {
    // Immutable v1 box ON segment292, reproduced literally; v1 logs are retained.
    let position=[1.656949728023706,1.2389364713478088,0.5];let yaw=-0.7819257774735757_f64;
    let dt=0.000344165_f64;let mut twist=Twist2::ZERO;twist.linear.x=0.5;twist.angular_z=-1.600517838137915e-12;
    let (stable,heading)=stable_plant_integrate(position,yaw,twist,dt);
    let expected=oracle_integrate(OraclePose {position,yaw},twist,dt).unwrap();
    assert!(distance3(stable,expected.position)<1e-12);assert!((heading-expected.yaw).abs()<1e-12);
    let end=yaw+twist.angular_z*dt;
    let old=[position[0]+twist.linear.x*(end.sin()-yaw.sin())/twist.angular_z,
        position[1]+twist.linear.x*(yaw.cos()-end.cos())/twist.angular_z,0.5];
    let old_error=(0..3).map(|i|(old[i]-expected.position[i]).abs()).fold(0.0,f64::max);
    assert!(old_error>1e-6,"the preserved counterexample must expose old cancellation");
    // Normal finite turns also match the previous closed form; small-angle and
    // zero tests include actual dt values instead of replacing motion with 50ms.
    let mut cases=0;
    for w in [0.0,-1e-14,1e-14,-1.600517838137915e-12,1.600517838137915e-12,-1e-10,1e-10,-0.1,0.1,-0.8,0.8] {
        for dt in [0.0,1e-7,0.000344165,0.025,0.05,0.117] {
            for (vx,vy) in [(0.0,0.0),(0.5,0.0),(0.3,-0.2)] {
                let mut twist=Twist2::ZERO;twist.linear.x=vx;twist.linear.y=vy;twist.angular_z=w;
                let (position2,yaw2)=stable_plant_integrate(position,yaw,twist,dt);
                let expected=oracle_integrate(OraclePose {position,yaw},twist,dt).unwrap();
                assert!(distance3(position2,expected.position)<1e-12);assert!((yaw2-expected.yaw).abs()<1e-12);
                if w.abs()>=0.1 && dt>=0.025 {
                    let end=yaw+w*dt;
                    let legacy=[position[0]+(vx*(end.sin()-yaw.sin())+vy*(end.cos()-yaw.cos()))/w,
                        position[1]+(vx*(yaw.cos()-end.cos())+vy*(end.sin()-yaw.sin()))/w,position[2]];
                    assert!(distance3(position2,legacy)<1e-12,"normal arc changed");
                }
                cases+=1;
            }
        }
    }
    // Check both sides of the fixed numerical switch, never change oracle EPS.
    for angle in [0.0000999999,0.0001000001,-0.0000999999,-0.0001000001] {
        let dt=0.05;let mut twist=Twist2::ZERO;twist.linear.x=0.5;twist.angular_z=angle/dt;
        let (result,_)=stable_plant_integrate(position,yaw,twist,dt);
        let expected=oracle_integrate(OraclePose {position,yaw},twist,dt).unwrap();
        assert!(distance3(result,expected.position)<1e-12);
    }
    assert_eq!(ORACLE_EPS,1e-8);
    eprintln!("STABLE_PLANT_REGRESSION preserved_v1_old_max_error_m={old_error:.15} equivalent_cases={cases} numerical_switch_cases=4 oracle_epsilon_unchanged={ORACLE_EPS}");
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
enum Scene { Box, Wall }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stage { Clear, Inserted }
#[derive(Clone, Debug)]
struct ActualSegment {from:Instant,to:Instant,motion:OracleSegment}
#[derive(Clone, Debug)]
struct SourceRecord {
    stage:Stage,sample:Instant,published:Instant,source:i64,
    authority:GuardedMapAuthority,root:nuwax_map_chunks::RootIdentity,
    root_geometry_revision:u64,retained_cells:usize,ground_count:usize,obstacle_count:usize,
    all_keys_exact:bool,all_observed_ns_current:bool,obstacle_hits_min:u16,obstacle_hits_max:u16,
}
#[derive(Clone)]
struct SensorControl {stage:Stage,scene:Scene}
fn ground_keys()->BTreeSet<[i32;3]> {(0..100).flat_map(|x|(0..60).map(move|y|[x,y,1])).collect()}
fn obstacle_keys(scene:Scene)->BTreeSet<[i32;3]> {
    let ys=if scene==Scene::Box {24..36}else{0..60};
    (40..48).flat_map(|x|ys.clone().flat_map(move|y|(2..21).map(move|z|[x,y,z]))).collect()
}
fn scene_keys(scene:Scene,stage:Stage)->BTreeSet<[i32;3]> {
    let mut cells=ground_keys();if stage==Stage::Inserted {cells.extend(obstacle_keys(scene));}cells
}
fn body_pose(x:f64,y:f64,yaw:f64)->BodyPose {([x,y,0.5],[0.0,0.0,(yaw/2.0).sin(),(yaw/2.0).cos()])}
fn witness_poses()->Vec<BodyPose> {
    // Fixed construction agreed before the first build. No path injection.
    let xy:[[f64;2];6]=[[1.0,1.5],[1.5,1.5],[1.5,0.6],[2.9,0.6],[2.9,1.5],[3.5,1.5]];
    let mut poses=vec![body_pose(xy[0][0],xy[0][1],0.0)];
    for edge in xy.windows(2) {
        let yaw=(edge[1][1]-edge[0][1]).atan2(edge[1][0]-edge[0][0]);
        poses.push(body_pose(edge[0][0],edge[0][1],yaw));
        poses.push(body_pose(edge[1][0],edge[1][1],yaw));
    }
    poses.push(body_pose(3.5,1.5,0.0));poses
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


fn audit_observations(map:&SurfaceSnapshot,scene:Scene,stage:Stage,stamp:i64)->Result<(),String> {
    let obs=map.observations().ok_or("missing fine observations")?;
    let expected=scene_keys(scene,stage);
    for x in 0..100 {for y in 0..60 {for z in 1..21 {
        let key=[x,y,z];let value=obs.get(key);
        if value.is_some()!=expected.contains(&key) {return Err(format!("built membership mismatch {key:?}"));}
        if value.is_some_and(|v|v.observed_ns!=stamp as u64) {return Err(format!("built source timestamp mismatch {key:?}"));}
    }}}
    Ok(())
}
fn original_geometry_queries(map:&SurfaceSnapshot,scene:Scene,stage:Stage)->Result<(),String> {
    let start=body_pose(1.0,1.5,0.0);let goal=body_pose(3.5,1.5,0.0);
    for (name,pose) in [("start",start),("goal",goal)] {
        let mut witness=None;let result=map.validate_body_pose_with_witness(pose.0,pose.1,&mut witness);
        eprintln!("PREFLIGHT_BODY scene={scene:?} stage={stage:?} at={name} result={result:?} witness={witness:?}");
        result.map_err(|e|format!("{scene:?}/{stage:?}/{name}: {e:?}, {witness:?}"))?;
    }
    let mut witness=None;let straight=map.validate_body_motion_with_witness(start,goal,&[],&mut witness);
    eprintln!("PREFLIGHT_STRAIGHT scene={scene:?} stage={stage:?} result={straight:?} first_witness={witness:?}");
    if straight.is_ok()!=(stage==Stage::Clear) {return Err(format!("unexpected original straight query {straight:?} {witness:?}"));}
    if stage==Stage::Inserted {
        let view=map.with_current_body_support(start.0,start.1).map_err(|e|format!("actual-start overlay: {e:?}"))?;
        let mut overlay_witness=None;
        let result=view.validate_body_motion_with_witness(start,goal,&[],&mut overlay_witness);
        eprintln!("PREFLIGHT_ACTUAL_START_OVERLAY scene={scene:?} result={result:?} witness={overlay_witness:?}");
        if result.is_ok() {return Err("actual-start self-support overlay admitted obstacle crossing".into());}
    }
    if scene==Scene::Box {
        for (index,edge) in witness_poses().windows(2).enumerate() {
            let mut witness=None;let result=map.validate_body_motion_with_witness(edge[0],edge[1],&[],&mut witness);
            eprintln!("PREFLIGHT_WITNESS stage={stage:?} edge={index} from={:?} to={:?} result={result:?} first_witness={witness:?}",edge[0],edge[1]);
            result.map_err(|e|format!("original witness rejected edge {index}: {e:?}; {witness:?}"))?;
        }
    }
    Ok(())
}
fn independent_geometry_preflight()->Result<(),String> {
    let r=f64::from(RESOLUTION);let radius=0.15_f64.hypot(0.15);
    // Fixed waypoint decimals are intentional; conservative distances use the exact f32-origin obstacle.
    let box_gap=(40.0*r-1.5).min(24.0*r-0.6).min(2.9-48.0*r)-radius;
    let support_gap=(0.6-radius).min(60.0*r-1.5-radius);
    if box_gap<=0.28786 || support_gap<=0.38786 {return Err("fixed independent witness lacks the promised positive clearance".into());}
    let cut=44.0*r;
    if !(1.0<cut && cut<3.5 && 40.0*r<cut && cut<48.0*r && 2.0*r<0.45 && 0.70<21.0*r) {
        return Err("fixed-z wall separator arithmetic failed".into());
    }
    eprintln!("PREFLIGHT_GEOMETRY resolution={r:.17} ground=6000 box_added=1824 wall_added=9120 box_gap={box_gap:.17} support_gap={support_gap:.17} wall_cut_x={cut:.17} wall_domain_y=[0,{:.17}] fixed_body_z=[0.45,0.70]",60.0*r);
    for edge in witness_poses().windows(2) {
        let a=edge[0];let b=edge[1];let yaw_a=2.0*a.1[2].atan2(a.1[3]);let yaw_b=2.0*b.1[2].atan2(b.1[3]);
        let distance=distance3(a.0,b.0);let yaw_delta=(yaw_b-yaw_a).sin().atan2((yaw_b-yaw_a).cos());
        let duration=if distance>1e-12 {distance/0.5}else{yaw_delta.abs()/0.8};
        let mut twist=Twist2::ZERO;
        if distance>1e-12 {twist.linear.x=0.5;} else if duration>0.0 {twist.angular_z=yaw_delta/duration;}
        let segment=OracleSegment {start:OraclePose {position:a.0,yaw:yaw_a},end:OraclePose {position:b.0,yaw:yaw_a+yaw_delta},
            twist,duration_s:duration,obstacle_active:true};
        check_segment(&segment,Scene::Box)?;
    }
    Ok(())
}
#[test]
#[ignore="opt-in preflight only: original codec, measured surface body queries, independent geometry"]
fn original_codec_geometry_and_body_witness_preflight() {
    let mut decoded_scenes=Vec::new();
    // Actual wall clock scans, no forged evidence or repeated same-frame observe.
    for scene in [Scene::Box,Scene::Wall] {
        let repository=Repository::temporary().unwrap();
        let mut writer=Writer::new(repository.clone(),RESOLUTION,4*1024*1024).unwrap();
        let mut cache=SurfaceGeometryCache::configured(&calibrated_config(false));
        let mut previous_sample=None;let mut previous_geometry=None;
        for (index,stage) in [Stage::Clear,Stage::Inserted,Stage::Inserted].into_iter().enumerate() {
            if let Some(previous)=previous_sample {let elapsed:Duration=Instant::now()-previous;
                if elapsed<SAMPLE_PERIOD {std::thread::sleep(SAMPLE_PERIOD-elapsed);}}
            let sample=Instant::now();previous_sample=Some(sample);let stamp=current_unix_timestamp_ns().unwrap();
            let expected=scene_keys(scene,stage);writer.observe(expected.iter().copied(),stamp as u64).unwrap();
            let root=writer.snapshot(Domain {session:format!("dynamic-preflight-{scene:?}"),generation:1,
                frame_generation:1,producer_generation:17},index as u64+1,stamp as u64).unwrap();
            let values=writer.region([0,0,1],[99,59,20],16000).unwrap();
            let keys:BTreeSet<_>=values.iter().map(|(key,_)|*key).collect();assert_eq!(keys,expected);
            assert_eq!(keys.intersection(&ground_keys()).count(),6000);
            assert!(values.iter().all(|(_,v)|v.observed_ns==stamp as u64));
            if index==1 {for key in obstacle_keys(scene) {assert_eq!(values.iter().find(|(k,_)|*k==key).unwrap().1.hits,1);}}
            if index==1 {assert!(root.geometry_revision>previous_geometry.unwrap());}
            if index==2 {assert_eq!(root.geometry_revision,previous_geometry.unwrap());}
            previous_geometry=Some(root.geometry_revision);
            let map=map_from_fine(&root,&repository);let bytes=schema::encode_navigation_map_3d_body(&map).unwrap();
            let decoded=schema::decode_navigation_map_3d_body(&bytes).unwrap();assert_eq!(decoded,map);
            assert!(!decoded.cleared && decoded.fine.is_some() && decoded.cloud.data.is_empty());
            assert_eq!((decoded.cloud.width,decoded.cloud.row_step),(0,0));
            assert_eq!(decoded.source_timestamp_ns,stamp);assert_eq!(decoded.committed_map.source_timestamp_ns,stamp);
            assert_eq!(decoded.committed_map.revision,index as i64+1);assert_eq!(decoded.source_voxel_size,RESOLUTION);
            let built=cache.build(&decoded).unwrap();audit_observations(&built,scene,stage,stamp).unwrap();
            eprintln!("PREFLIGHT_CODEC scene={scene:?} stage={stage:?} source={stamp} root={:?} geometry_revision={} cells={} bytes={}",root.identity(),root.geometry_revision,keys.len(),bytes.len());
            decoded_scenes.push((scene,stage,built));
        }
    }
    stable_plant_preserves_recorded_near_zero_counterexample_and_normal_arcs();
    continuous_oracle_tests::preflight_regressions();
    independent_geometry_preflight().unwrap();
    for (scene,stage,built) in decoded_scenes {original_geometry_queries(&built,scene,stage).unwrap();}
    eprintln!("PREFLIGHT_PASS original_codec=true independent_geometry=true original_body_witness=true live_scenes_run=0");
}


// Test-only snippet: insert in a module that already imports Twist2 and defines
// Scene::{Box, Wall} and RESOLUTION: f32. No production collision, integration,
// planner, or controller API is used by this offline trajectory assessor.

#[derive(Clone, Copy, Debug)]
struct OraclePose {
    position: [f64; 3],
    yaw: f64,
}

#[derive(Clone, Copy, Debug)]
struct OracleSegment {
    start: OraclePose,
    end: OraclePose,
    twist: Twist2,
    duration_s: f64,
    // The caller splits actual motion at the exact occupancy insertion time.
    obstacle_active: bool,
}

const ORACLE_HALF_SIDE_M: f64 = 0.15;
const ORACLE_CENTER_Z_M: f64 = 0.5;
const ORACLE_LOCAL_Z_M: [f64; 2] = [-0.05, 0.20];
// Positive margins must exceed this tolerance; touching is never certified.
const ORACLE_EPS: f64 = 1.0e-8;
const ORACLE_MIN_INTERVAL_S: f64 = 0.001;
const ORACLE_MAX_DEPTH: usize = 20;
const ORACLE_MAX_NODES: usize = 4095;

#[derive(Clone, Copy)]
struct OracleInterval {
    lo_s: f64,
    hi_s: f64,
    depth: usize,
}

#[derive(Clone, Copy)]
struct OracleGeometry {
    support_max: [f64; 2],
    obstacle_min: [f64; 3],
    obstacle_max: [f64; 3],
}

#[derive(Clone, Copy)]
struct OracleMargins {
    support: f64,
    obstacle: f64,
}

fn oracle_geometry(scene: &Scene) -> Result<OracleGeometry, String> {
    let r = f64::from(RESOLUTION);
    if !r.is_finite() || r <= 0.0 || !(100.0 * r).is_finite() {
        return Err("invalid oracle grid resolution".into());
    }
    let (min_y, max_y) = match scene {
        Scene::Box => (24.0 * r, 36.0 * r),
        Scene::Wall => (0.0, 60.0 * r),
    };
    Ok(OracleGeometry {
        support_max: [100.0 * r, 60.0 * r],
        obstacle_min: [40.0 * r, min_y, 2.0 * r],
        obstacle_max: [48.0 * r, max_y, 21.0 * r],
    })
}

fn oracle_finite_pose(pose: OraclePose) -> bool {
    pose.position.into_iter().all(f64::is_finite) && pose.yaw.is_finite()
}

// Exact constant body-frame SE(2) twist integration, including lateral speed:
// delta p = t*sinc(w*t/2) R(yaw + w*t/2) [vx, vy]. The short Taylor expansion
// avoids the cancellation in (1 - cos(w*t))/w for nearly straight motion.
fn oracle_integrate(
    start: OraclePose,
    twist: Twist2,
    time_s: f64,
) -> Result<OraclePose, String> {
    let angle = twist.angular_z * time_s;
    let half_angle = 0.5 * angle;
    let mid_yaw = start.yaw + half_angle;
    let end_yaw = start.yaw + angle;
    if !time_s.is_finite()
        || time_s < 0.0
        || !angle.is_finite()
        || !mid_yaw.is_finite()
        || !end_yaw.is_finite()
    {
        return Err("Unknown: non-finite SE(2) integration arithmetic".into());
    }
    // Conservatively reject extreme argument reduction rather than claiming
    // a nanometre-scale geometric certificate for arbitrarily large angles.
    if start.yaw.abs() > 1.0e6 || angle.abs() > 1.0e6 || end_yaw.abs() > 1.0e6 {
        return Err("Unknown: angular numeric range exceeded".into());
    }
    let sinc = if half_angle.abs() < 1.0e-4 {
        let x2 = half_angle * half_angle;
        1.0 + x2 * (-1.0 / 6.0 + x2 * (1.0 / 120.0 - x2 / 5040.0))
    } else {
        half_angle.sin() / half_angle
    };
    let scale = time_s * sinc;
    let (sin_yaw, cos_yaw) = mid_yaw.sin_cos();
    let local_x = scale * twist.linear.x;
    let local_y = scale * twist.linear.y;
    let pose = OraclePose {
        position: [
            start.position[0] + cos_yaw * local_x - sin_yaw * local_y,
            start.position[1] + sin_yaw * local_x + cos_yaw * local_y,
            start.position[2],
        ],
        yaw: end_yaw,
    };
    if !scale.is_finite() || !oracle_finite_pose(pose) {
        return Err("Unknown: non-finite integrated pose".into());
    }
    Ok(pose)
}

fn oracle_dot_2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

// Static OBB/AABB SAT. All XY axes are explicitly normalized, so each gap
// and the support-domain clearance are in metres and can be compared with h.
fn oracle_margins(
    pose: OraclePose,
    geometry: OracleGeometry,
) -> Result<OracleMargins, String> {
    let (sin_yaw, cos_yaw) = pose.yaw.sin_cos();
    let norm = cos_yaw.hypot(sin_yaw);
    if !norm.is_finite() || norm <= 0.0 {
        return Err("Unknown: invalid SAT body axes".into());
    }
    let body_x = [cos_yaw / norm, sin_yaw / norm];
    let body_y = [-body_x[1], body_x[0]];
    let body_radius_on = |axis: [f64; 2]| {
        ORACLE_HALF_SIDE_M
            * (oracle_dot_2(body_x, axis).abs() + oracle_dot_2(body_y, axis).abs())
    };
    let radius_x = body_radius_on([1.0, 0.0]);
    let radius_y = body_radius_on([0.0, 1.0]);
    let support = [
        pose.position[0] - radius_x,
        geometry.support_max[0] - pose.position[0] - radius_x,
        pose.position[1] - radius_y,
        geometry.support_max[1] - pose.position[1] - radius_y,
    ]
    .into_iter()
    .fold(f64::INFINITY, f64::min);

    let box_center = [
        0.5 * (geometry.obstacle_min[0] + geometry.obstacle_max[0]),
        0.5 * (geometry.obstacle_min[1] + geometry.obstacle_max[1]),
    ];
    let box_half = [
        0.5 * (geometry.obstacle_max[0] - geometry.obstacle_min[0]),
        0.5 * (geometry.obstacle_max[1] - geometry.obstacle_min[1]),
    ];
    let delta = [
        pose.position[0] - box_center[0],
        pose.position[1] - box_center[1],
    ];
    let mut obstacle = f64::NEG_INFINITY;
    for axis in [[1.0, 0.0], [0.0, 1.0], body_x, body_y] {
        let box_radius = box_half[0] * axis[0].abs() + box_half[1] * axis[1].abs();
        let gap = oracle_dot_2(delta, axis).abs() - body_radius_on(axis) - box_radius;
        obstacle = obstacle.max(gap);
    }
    // Both bodies are upright prisms. Z is fixed for this SE(2) fixture, but
    // account for it explicitly instead of silently treating every voxel as tall.
    obstacle = obstacle
        .max(geometry.obstacle_min[2] - (pose.position[2] + ORACLE_LOCAL_Z_M[1]))
        .max((pose.position[2] + ORACLE_LOCAL_Z_M[0]) - geometry.obstacle_max[2]);
    if !support.is_finite() || !obstacle.is_finite() {
        return Err("Unknown: non-finite SAT margin".into());
    }
    Ok(OracleMargins { support, obstacle })
}

fn oracle_check_sample(
    pose: OraclePose,
    time_s: f64,
    geometry: OracleGeometry,
    obstacle_active: bool,
) -> Result<OracleMargins, String> {
    let margins = oracle_margins(pose, geometry)?;
    if margins.support < -ORACLE_EPS {
        return Err(format!(
            "unsupported footprint at t={time_s:.9}s (clearance={:.12}m)",
            margins.support
        ));
    }
    if margins.support <= ORACLE_EPS {
        return Err(format!(
            "Unknown: near-zero support margin at t={time_s:.9}s (clearance={:.12}m)",
            margins.support
        ));
    }
    if obstacle_active && margins.obstacle < -ORACLE_EPS {
        return Err(format!(
            "occupied obstacle collision at t={time_s:.9}s (SAT gap={:.12}m)",
            margins.obstacle
        ));
    }
    if obstacle_active && margins.obstacle <= ORACLE_EPS {
        return Err(format!(
            "Unknown: near-zero obstacle margin at t={time_s:.9}s (SAT gap={:.12}m)",
            margins.obstacle
        ));
    }
    Ok(margins)
}

// Returns the number of examined interval nodes, including subdivisions and
// certified leaves. Any unresolved interval is Err(Unknown), never safe.
fn check_segment(segment: &OracleSegment, scene: Scene) -> Result<usize, String> {
    if !oracle_finite_pose(segment.start)
        || !oracle_finite_pose(segment.end)
        || !segment.twist.linear.x.is_finite()
        || !segment.twist.linear.y.is_finite()
        || !segment.twist.angular_z.is_finite()
        || !segment.duration_s.is_finite()
        || segment.duration_s < 0.0
    {
        return Err("invalid/non-finite actual trajectory segment".into());
    }
    if (segment.start.position[2] - ORACLE_CENTER_Z_M).abs() > ORACLE_EPS
        || (segment.end.position[2] - ORACLE_CENTER_Z_M).abs() > ORACLE_EPS
    {
        return Err("actual trajectory violates fixed z=0.5m SE(2) fixture".into());
    }
    if segment.end.yaw.abs() > 1.0e6 {
        return Err("Unknown: recorded endpoint angular numeric range exceeded".into());
    }
    let geometry = oracle_geometry(&scene)?;
    // Reintegrate the executed twist independently; an endpoint chord is not
    // the robot's path. Wrapped yaw agreement permits a recorded +/-pi wrap.
    let expected_end = oracle_integrate(segment.start, segment.twist, segment.duration_s)?;
    let yaw_delta = expected_end.yaw - segment.end.yaw;
    if !yaw_delta.is_finite() {
        return Err("invalid/non-finite endpoint angular difference".into());
    }
    let yaw_error = yaw_delta.sin().atan2(yaw_delta.cos()).abs();
    let position_error = expected_end
        .position
        .into_iter()
        .zip(segment.end.position)
        .map(|(expected, actual)| (expected - actual).abs())
        .fold(0.0, f64::max);
    if position_error > ORACLE_EPS || yaw_error > ORACLE_EPS {
        return Err(format!(
            "actual endpoint disagrees with independent SE(2) integration: position={position_error:.12}m, yaw={yaw_error:.12}rad"
        ));
    }
    // Check the actual recorded endpoints, not only the independently
    // reconstructed endpoint (which is allowed a small numeric tolerance).
    oracle_check_sample(segment.start, 0.0, geometry, segment.obstacle_active)?;
    oracle_check_sample(
        segment.end,
        segment.duration_s,
        geometry,
        segment.obstacle_active,
    )?;

    // For every point of the square, |p_dot| <= |v| + rho*|w|, where
    // rho=sqrt(2)*0.15. Thus every point moves at most h over half an interval.
    // A fixed midpoint separating axis with gap > h+eps remains separating;
    // support clearance > h+eps keeps the entire footprint in the domain.
    let point_speed = segment.twist.linear.x.hypot(segment.twist.linear.y)
        + std::f64::consts::SQRT_2 * ORACLE_HALF_SIDE_M * segment.twist.angular_z.abs();
    if !point_speed.is_finite() {
        return Err("Unknown: non-finite point-speed bound".into());
    }
    let root = OracleInterval {
        lo_s: 0.0,
        hi_s: segment.duration_s,
        depth: 0,
    };
    // Depth-first traversal needs at most MAX_DEPTH+1 pending intervals.
    // No recursion, unbounded allocation, or controller feedback is involved.
    let mut stack = [root; ORACLE_MAX_DEPTH + 1];
    let mut stack_len = 1usize;
    let mut nodes = 0usize;
    while stack_len != 0 {
        if nodes == ORACLE_MAX_NODES {
            return Err(format!("Unknown: interval-node budget exhausted ({nodes})"));
        }
        stack_len -= 1;
        let interval = stack[stack_len];
        nodes += 1;
        let width_s = interval.hi_s - interval.lo_s;
        let half_width_s = 0.5 * width_s;
        let midpoint_s = interval.lo_s + half_width_s;
        let mut midpoint = None;
        for time_s in [interval.lo_s, midpoint_s, interval.hi_s] {
            let pose = oracle_integrate(segment.start, segment.twist, time_s)?;
            let margins = oracle_check_sample(pose, time_s, geometry, segment.obstacle_active)?;
            if time_s == midpoint_s {
                midpoint = Some(margins);
            }
        }
        let midpoint = midpoint.ok_or_else(|| "Unknown: missing midpoint evidence".to_owned())?;
        let h = point_speed * half_width_s;
        if !h.is_finite() || !(h + ORACLE_EPS).is_finite() {
            return Err("Unknown: non-finite swept displacement bound".into());
        }
        if midpoint.support > h + ORACLE_EPS
            && (!segment.obstacle_active || midpoint.obstacle > h + ORACLE_EPS)
        {
            continue;
        }
        if width_s <= ORACLE_MIN_INTERVAL_S {
            return Err(format!(
                "Unknown: unresolved interval at minimum width ({width_s:.12}s, nodes={nodes})"
            ));
        }
        if interval.depth >= ORACLE_MAX_DEPTH {
            return Err(format!("Unknown: subdivision depth exhausted (nodes={nodes})"));
        }
        if midpoint_s <= interval.lo_s || midpoint_s >= interval.hi_s {
            return Err("Unknown: subdivision cannot make numeric progress".into());
        }
        if stack_len + 2 > stack.len() {
            return Err("Unknown: fixed interval stack exhausted".into());
        }
        // Push right first so chronological left intervals are examined first.
        stack[stack_len] = OracleInterval {
            lo_s: midpoint_s,
            hi_s: interval.hi_s,
            depth: interval.depth + 1,
        };
        stack[stack_len + 1] = OracleInterval {
            lo_s: interval.lo_s,
            hi_s: midpoint_s,
            depth: interval.depth + 1,
        };
        stack_len += 2;
    }
    Ok(nodes)
}

#[cfg(test)]
mod continuous_oracle_tests {
    use super::*;

    // Allows the enclosing ignored preflight to run these same ordinary
    // regressions without duplicating their assertions or changing the oracle.
    pub(super) fn preflight_regressions() {
        straight_thin_crossing_is_found_between_coarse_samples();
        curved_collision_is_found_even_when_endpoint_chord_is_stationary();
        safe_box_bypass_produces_subdivision_evidence();
        safe_wall_side_witness_and_zero_duration_are_certified();
        support_excursion_is_found_between_clear_coarse_samples();
        unresolved_near_boundary_motion_fails_closed_at_minimum_width();
        node_budget_exhaustion_fails_closed();
        depth_budget_exhaustion_fails_closed();
        near_zero_contact_and_clearance_are_unknown();
        lateral_twist_and_small_angle_have_independent_analytic_endpoints();
        actual_endpoint_mismatch_nonfinite_negative_time_and_z_drift_fail();
        yaw_wrap_is_accepted_but_angular_endpoint_mismatch_is_not();
        inactive_obstacle_does_not_change_support_requirement();
    }

    fn pose(x: f64, y: f64, yaw: f64) -> OraclePose {
        OraclePose { position: [x, y, ORACLE_CENTER_Z_M], yaw }
    }

    fn twist(vx: f64, vy: f64, omega: f64) -> Twist2 {
        let mut result = Twist2::ZERO;
        result.linear.x = vx;
        result.linear.y = vy;
        result.angular_z = omega;
        result
    }

    fn assert_three_coarse_samples_are_clear(segment: &OracleSegment, scene: Scene) {
        let geometry = oracle_geometry(&scene).unwrap();
        for t in [0.0, segment.duration_s * 0.5, segment.duration_s] {
            oracle_check_sample(
                oracle_integrate(segment.start, segment.twist, t).unwrap(),
                t,
                geometry,
                segment.obstacle_active,
            )
            .unwrap();
        }
    }

    #[test]
    fn straight_thin_crossing_is_found_between_coarse_samples() {
        let segment = OracleSegment {
            start: pose(1.6, 1.5, 0.0),
            end: pose(4.6, 1.5, 0.0),
            twist: twist(3.0, 0.0, 0.0),
            duration_s: 1.0,
            obstacle_active: true,
        };
        assert_three_coarse_samples_are_clear(&segment, Scene::Box);
        assert!(check_segment(&segment, Scene::Box).unwrap_err().contains("occupied obstacle collision"));
    }

    #[test]
    fn curved_collision_is_found_even_when_endpoint_chord_is_stationary() {
        let yaw = -std::f64::consts::FRAC_PI_2;
        let segment = OracleSegment {
            start: pose(1.7, 1.8, yaw),
            end: pose(1.7, 1.8, yaw + std::f64::consts::TAU),
            twist: twist(0.5, 0.0, 1.0),
            duration_s: std::f64::consts::TAU,
            obstacle_active: true,
        };
        assert_three_coarse_samples_are_clear(&segment, Scene::Box);
        assert!(check_segment(&segment, Scene::Box).unwrap_err().contains("occupied obstacle collision"));
    }

    #[test]
    fn safe_box_bypass_produces_subdivision_evidence() {
        let segment = OracleSegment {
            start: pose(1.0, 0.6, 0.0),
            end: pose(4.0, 0.6, 0.0),
            twist: twist(0.75, 0.0, 0.0),
            duration_s: 4.0,
            obstacle_active: true,
        };
        let nodes = check_segment(&segment, Scene::Box).unwrap();
        assert!(nodes > 1 && nodes <= ORACLE_MAX_NODES);
    }

    #[test]
    fn safe_wall_side_witness_and_zero_duration_are_certified() {
        let segment = OracleSegment {
            start: pose(0.5, 0.8, 0.0),
            end: pose(1.5, 0.8, 0.0),
            twist: twist(1.0, 0.0, 0.0),
            duration_s: 1.0,
            obstacle_active: true,
        };
        assert!(check_segment(&segment, Scene::Wall).is_ok());
        let stationary = OracleSegment {
            end: segment.start,
            duration_s: 0.0,
            ..segment
        };
        assert_eq!(check_segment(&stationary, Scene::Wall).unwrap(), 1);
    }

    #[test]
    fn support_excursion_is_found_between_clear_coarse_samples() {
        let yaw = std::f64::consts::PI;
        let segment = OracleSegment {
            start: pose(0.5, 1.95, yaw),
            end: pose(0.5, 1.95, yaw + std::f64::consts::TAU),
            twist: twist(0.45, 0.0, 1.0),
            duration_s: std::f64::consts::TAU,
            obstacle_active: false,
        };
        assert_three_coarse_samples_are_clear(&segment, Scene::Box);
        assert!(check_segment(&segment, Scene::Box).unwrap_err().contains("unsupported footprint"));
    }

    #[test]
    fn unresolved_near_boundary_motion_fails_closed_at_minimum_width() {
        // This path is in fact supported. The isotropic point-speed bound is
        // deliberately too weak to certify its tiny lateral clearance at 1ms.
        let segment = OracleSegment {
            start: pose(0.15001, 1.0, 0.0),
            end: pose(0.15001, 1.01, 0.0),
            twist: twist(0.0, 1.0, 0.0),
            duration_s: 0.01,
            obstacle_active: false,
        };
        assert_three_coarse_samples_are_clear(&segment, Scene::Box);
        let error = check_segment(&segment, Scene::Box).unwrap_err();
        assert!(error.starts_with("Unknown:") && error.contains("minimum width"));
    }

    #[test]
    fn node_budget_exhaustion_fails_closed() {
        let segment = OracleSegment {
            start: pose(0.1501, 0.5, 0.0),
            end: pose(0.1501, 1.5, 0.0),
            twist: twist(0.0, 0.001, 0.0),
            duration_s: 1000.0,
            obstacle_active: false,
        };
        let error = check_segment(&segment, Scene::Box).unwrap_err();
        assert!(error.starts_with("Unknown:") && error.contains("node budget"));
    }

    #[test]
    fn depth_budget_exhaustion_fails_closed() {
        let segment = OracleSegment {
            start: pose(0.1500004, 0.5, 0.0),
            end: pose(0.1500004, 1.5, 0.0),
            twist: twist(0.0, 0.000001, 0.0),
            duration_s: 1_000_000.0,
            obstacle_active: false,
        };
        let error = check_segment(&segment, Scene::Box).unwrap_err();
        assert!(error.starts_with("Unknown:") && error.contains("depth exhausted"));
    }

    #[test]
    fn near_zero_contact_and_clearance_are_unknown() {
        let r = f64::from(RESOLUTION);
        // Tiny positive separation, contact, and tiny apparent penetration all
        // lie inside numerical uncertainty. Every case fails without claiming
        // a definite collision or a definitely unsupported footprint.
        for margin in [-0.5 * ORACLE_EPS, 0.0, 0.5 * ORACLE_EPS] {
            let near_obstacle = pose(
                40.0 * r - ORACLE_HALF_SIDE_M - margin,
                30.0 * r,
                0.0,
            );
            let segment = OracleSegment {
                start: near_obstacle,
                end: near_obstacle,
                twist: Twist2::ZERO,
                duration_s: 0.0,
                obstacle_active: true,
            };
            let error = check_segment(&segment, Scene::Box).unwrap_err();
            assert!(error.starts_with("Unknown:") && error.contains("obstacle margin"));

            let near_support = pose(ORACLE_HALF_SIDE_M + margin, 1.0, 0.0);
            let segment = OracleSegment {
                start: near_support,
                end: near_support,
                obstacle_active: false,
                ..segment
            };
            let error = check_segment(&segment, Scene::Box).unwrap_err();
            assert!(error.starts_with("Unknown:") && error.contains("support margin"));
        }
    }

    #[test]
    fn lateral_twist_and_small_angle_have_independent_analytic_endpoints() {
        let start = pose(2.0, 1.0, 0.0);
        let lateral_arc = oracle_integrate(
            start,
            twist(0.0, 0.5, 1.0),
            std::f64::consts::FRAC_PI_2,
        )
        .unwrap();
        assert!((lateral_arc.position[0] - 1.5).abs() < 1.0e-12);
        assert!((lateral_arc.position[1] - 1.5).abs() < 1.0e-12);
        let near_straight = oracle_integrate(start, twist(0.2, 0.1, 1.0e-12), 1.0).unwrap();
        assert!((near_straight.position[0] - 2.2).abs() < 1.0e-12);
        assert!((near_straight.position[1] - 1.1).abs() < 1.0e-12);
    }

    #[test]
    fn actual_endpoint_mismatch_nonfinite_negative_time_and_z_drift_fail() {
        let good = OracleSegment {
            start: pose(1.0, 0.6, 0.0),
            end: pose(1.1, 0.6, 0.0),
            twist: twist(0.1, 0.0, 0.0),
            duration_s: 1.0,
            obstacle_active: false,
        };
        let mut bad = good;
        bad.end.position[0] += 0.001;
        assert!(check_segment(&bad, Scene::Box).unwrap_err().contains("endpoint disagrees"));
        bad = good;
        bad.twist.linear.y = f64::NAN;
        assert!(check_segment(&bad, Scene::Box).is_err());
        bad = good;
        bad.duration_s = -1.0;
        assert!(check_segment(&bad, Scene::Box).is_err());
        bad = good;
        bad.end.position[2] += 0.001;
        assert!(check_segment(&bad, Scene::Box).unwrap_err().contains("fixed z"));
    }

    #[test]
    fn yaw_wrap_is_accepted_but_angular_endpoint_mismatch_is_not() {
        let segment = OracleSegment {
            start: pose(1.0, 0.6, std::f64::consts::PI - 0.1),
            end: pose(1.0, 0.6, -std::f64::consts::PI + 0.1),
            twist: twist(0.0, 0.0, 0.2),
            duration_s: 1.0,
            obstacle_active: false,
        };
        assert!(check_segment(&segment, Scene::Box).is_ok());
        let mut bad = segment;
        bad.end.yaw += 0.01;
        assert!(check_segment(&bad, Scene::Box).unwrap_err().contains("endpoint disagrees"));
    }

    #[test]
    fn inactive_obstacle_does_not_change_support_requirement() {
        let middle_of_box = pose(2.2, 1.5, 0.0);
        let segment = OracleSegment {
            start: middle_of_box,
            end: middle_of_box,
            twist: Twist2::ZERO,
            duration_s: 1.0,
            obstacle_active: false,
        };
        assert_eq!(check_segment(&segment, Scene::Box).unwrap(), 1);
        let active = OracleSegment { obstacle_active: true, ..segment };
        assert!(check_segment(&active, Scene::Box).unwrap_err().contains("occupied obstacle collision"));
    }
}

struct Evidence {
    maps: BTreeMap<i64,SourceRecord>, built: BTreeMap<i64,PreparedSurfaceMap>,
    latest_map: Option<schema::NavigationMap3D>, paths: Vec<schema::PathEnvelope>,
    states: Vec<schema::NavigationState>, path_receipts:Vec<(Instant,i64,i64)>, consumed:Vec<(Instant,i64)>,
}
struct OccupancyHarness {
    transport: LcmTransport, node: ModuleNode, scheduler: NavSchedulerHandle,
    planner: Option<Box<dyn InProcessNode>>, executor: Option<Box<dyn InProcessNode>>,
    plant: Arc<Mutex<Plant>>, sensor: Arc<Mutex<SensorControl>>,
    evidence: Arc<Mutex<Evidence>>, feeder: tokio::task::JoinHandle<()>,
    observers: Vec<tokio::task::JoinHandle<()>>, goal: Option<schema::GoalEnvelope>,
    started: Instant, local_planning_enabled: bool, scene:Scene, insertion:Option<(Instant,BodyPose,u64)>, wall_hold:Option<(MotionEvent,Instant)>,
}
impl OccupancyHarness {
    async fn start(local_planning_enabled: bool,scene:Scene) -> Self {
        Self::start_with_dense_recent(local_planning_enabled,scene,None).await
    }
    // An explicit new load case; original entry points always select None.
    async fn start_with_dense_recent(local_planning_enabled: bool,scene:Scene,
        dense_log:Option<Arc<Mutex<Vec<dense_recent::PublishedDenseScan>>>>) -> Self {
        let started=Instant::now();
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
            topics.topics.insert(name.into(),TopicName::new(format!("/memq-dynamic-occupancy/{name}")));
        }
        let graph=crate::profiles::build_graph(crate::NavStackProfile::Mls3D,&topics,
            crate::features::NavStackPreset::Mls.to_flags());
        let node=graph.find("mls_planner").unwrap().clone();
        let executor_node=graph.find("surface_nav_scheduler").unwrap().clone();
        let mut paths=typed_stream(&transport,&node.outputs[0],"nuwax_msgs.PathEnvelope").await.unwrap();
        let mut states=typed_stream(&transport,&node.inputs[4],"nuwax_msgs.NavigationState").await.unwrap();
        let hub=SurfaceMapHub::new(Arc::from("base_link")); let mut built=hub.events.subscribe();
        let evidence=Arc::new(Mutex::new(Evidence { maps:BTreeMap::new(),built:BTreeMap::new(),
            latest_map:None,paths:Vec::new(),states:Vec::new(),path_receipts:Vec::new(),consumed:Vec::new() }));
        let path_evidence=evidence.clone(); let state_evidence=evidence.clone(); let map_evidence=evidence.clone();
        let observers=vec![
            tokio::spawn(async move { while let Some(frame)=paths.next().await {
                let path=schema::decode_path_envelope_body(&frame.unwrap().payload).unwrap();
                let mut evidence=path_evidence.lock().unwrap();
                evidence.path_receipts.push((Instant::now(),path.path_revision,current_unix_timestamp_ns().unwrap()));evidence.paths.push(path);
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
        let mut planner=boxed_owner("memq-dynamic-occupancy-real-planner",Box::new(move |shutdown,ready| {
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
        let mut executor=boxed_owner("memq-dynamic-occupancy-real-executor",Box::new(move |shutdown,ready| {
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
        let sensor=Arc::new(Mutex::new(SensorControl {stage:Stage::Clear,scene}));
        let feed_sensor=sensor.clone(); let feed_plant=plant.clone(); let feed_evidence=evidence.clone();
        let feed_transport=transport.clone(); let feed_node=node.clone();
        let feeder=tokio::spawn(async move {
            let repository=Repository::temporary().unwrap();
            let mut writer=Writer::new(repository.clone(),RESOLUTION,4*1024*1024).unwrap();

            let mut previous_stage=Stage::Clear; let mut revision=0i64; let mut sequence=0;
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
                let (sample,stamp,control)={let guard=feed_sensor.lock().unwrap();
                    (Instant::now(),current_unix_timestamp_ns().unwrap(),guard.clone())};
                next_sample=sample+SAMPLE_PERIOD; // no catch-up/backdated sensor frames
                let retained=scene_keys(control.scene,control.stage);
                writer.observe(retained.iter().copied(),stamp as u64).unwrap(); revision+=1;
                let root=writer.snapshot(Domain {session:"memq-dynamic-occupancy-map".into(),generation:1,
                    frame_generation:1,producer_generation:17},revision as u64,stamp as u64).unwrap();
                let map=map_from_fine(&root,&repository);
                let payload=schema::encode_navigation_map_3d_body(&map).unwrap();
                let values=writer.region([0,0,1],[99,59,20],16000).unwrap();
                let actual:BTreeSet<_>=values.iter().map(|(k,_)|*k).collect();
                let obstacles=obstacle_keys(control.scene);
                let hits:Vec<_>=values.iter().filter(|(k,_)|obstacles.contains(k)).map(|(_,v)|v.hits).collect();
                let record=SourceRecord {stage:control.stage,sample,published:Instant::now(),source:stamp,
                    authority:map_authority(&map),root:root.identity(),root_geometry_revision:root.geometry_revision,
                    retained_cells:retained.len(),ground_count:actual.intersection(&ground_keys()).count(),
                    obstacle_count:hits.len(),all_keys_exact:actual==retained,
                    all_observed_ns_current:values.iter().all(|(_,v)|v.observed_ns==stamp as u64),
                    obstacle_hits_min:hits.iter().copied().min().unwrap_or(0),
                    obstacle_hits_max:hits.iter().copied().max().unwrap_or(0)};
                assert!(record.all_keys_exact && record.all_observed_ns_current);
                assert_eq!(record.ground_count,6000);
                if control.stage==Stage::Inserted && previous_stage==Stage::Clear {
                    assert_eq!((record.obstacle_hits_min,record.obstacle_hits_max),(1,1));
                }
                publish_body(&feed_transport,&feed_node.inputs[0],"nuwax_msgs.NavigationMap3D",
                    payload).await.unwrap();
                if let Some(log)=&dense_log {
                    let record=dense_recent::publish(&feed_transport,&map,sequence,stamp,pose_stamp,
                        control.scene,control.stage).await;
                    log.lock().unwrap().push(record);
                } else {
                    super::support_recovery_qualification::publish_sampled_recent(
                        &feed_transport,&map,sequence,stamp,pose_stamp,
                        retained.iter().copied().map(|key|(key,key[2]==1))).await;
                }
                // Only a successfully encoded and published source enters the ledger.
                { let mut evidence=feed_evidence.lock().unwrap();
                    evidence.maps.insert(stamp,record); evidence.latest_map=Some(map); }
                previous_stage=control.stage;
            }
        });
        Self {transport,node,scheduler,planner:Some(planner),executor:Some(executor),plant,sensor,
            evidence,feeder,observers,goal:None,started,local_planning_enabled,scene,insertion:None,wall_hold:None}
    }
    fn check_live(&self,stage:&str)->Result<(),String> {
        if self.started.elapsed()>QUALIFICATION_WINDOW {return Err(format!("{stage}: 60s qualification window exhausted"));}
        if self.feeder.is_finished() || self.observers.iter().any(|task|task.is_finished()) {
            return Err(format!("{stage}: feeder/observer exited"));
        }
        let status=self.scheduler.status();
        self.evidence.lock().unwrap().consumed.push((Instant::now(),status.costmap_source_timestamp_ns));
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
        // Keep real sensor acquisition health separate from ground geometry.
        if record.sample.elapsed()>Duration::from_secs(1) {return false;}
        let mut ahead=body; ahead.0[0]+=0.05;
        map.validate_body_pose(body.0,body.1).is_ok() && map.validate_body_motion(body,ahead,&[]).is_ok()
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
        loop {
            self.check_live("mature initial clear source")?;
            let source=self.scheduler.status().costmap_source_timestamp_ns;
            let body=self.plant.lock().unwrap().sample();
            if self.source(source,Stage::Clear) && self.fresh_body(source,body) {break;}
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let map=self.evidence.lock().unwrap().latest_map.clone().ok_or("no initial map")?;
        let options=nuwax_nav_types_messages::GoalOptions {allow_reverse:false,position_tolerance_m:0.1,yaw_tolerance_rad:0.15};
        let goal=goal_for_with_options(&map,1,[3.5,1.5,0.5],options);
        assert_eq!(goal.options,options);assert!(goal.options.requires_orientation());
        let original=goal.clone();self.goal=Some(goal.clone());
        eprintln!("GOAL original={goal:?} scene={:?} local_planning_enabled={} seals=1 publishes=1",self.scene,self.local_planning_enabled);
        publish_body(&self.transport,&self.node.inputs[2],"nuwax_msgs.GoalEnvelope",
            schema::encode_goal_envelope_body(&goal).unwrap()).await.map_err(|e|e.to_string())?;
        loop {
            self.check_live("initial actual movement and unique insertion")?;
            let mut plant=self.plant.lock().unwrap();let body=plant.sample();
            if distance3(body.0,[1.0,1.5,0.5])>0.10 && plant.events.iter().any(is_nonzero_applied) {
                if body.0[0]>1.25 || !(1.45..=1.55).contains(&body.0[1]) || self.scheduler.status().last_terminal.is_some() {
                    return Err(format!("unique insertion window missed: {body:?}"));
                }
                let accepted_revision=plant.events.iter().rev().find(|e|is_nonzero_applied(e))
                    .map(|e|e.revision).ok_or("movement lacks actual accepted revision")?;
                let path=self.evidence.lock().unwrap().paths.iter().find(|p|p.path_revision as u64==accepted_revision)
                    .cloned().ok_or("actual accepted revision lacks matching real wire path")?;
                validate_path_goal_correlation(&path,&goal)?;
                if !future_path_hits_box(&path,body) {return Err("initial real path future part does not intersect fixed box".into());}
                // Serialize insertion with the actual plant boundary. The prior command
                // remains applied; its next interval is checked against the new body.
                let mut sensor=self.sensor.lock().unwrap();
                let insert=Instant::now();plant.advance(insert);let body=plant.body();
                if body.0[0]>1.25 || !(1.45..=1.55).contains(&body.0[1]) {return Err("insertion crossed fixed pose bound".into());}
                let cursor=plant.events.len() as u64;assert!(plant.inserted.is_none());plant.inserted=Some(insert);
                sensor.stage=Stage::Inserted;
                self.insertion=Some((insert,body,cursor));
                let prior_command=plant.applied;drop(sensor);drop(plant);
                eprintln!("INSERT scene={:?} at={insert:?} cursor={cursor} body={body:?} actual_prior_command={:?} initial_revision={} future_path_intersects=true",self.scene,prior_command,path.path_revision);
                break;
            }
            drop(plant);tokio::time::sleep(Duration::from_millis(10)).await;
        }
        if self.scene==Scene::Wall {
            // Passive observation only: temporary zero during map adoption is not
            // assumed to be the final wall hold. Any actual nonzero resets only
            // this candidate window, never the original sixty-second deadline.
            loop {
                self.check_live("wall passive continuous stationary-window observation")?;
                let status=self.scheduler.status();
                if status.session_id.as_deref()!=Some(goal.session_id.as_str()) || status.last_terminal.is_some() {
                    return Err(format!("wall did not preserve active original goal: {status:?}"));
                }
                let (body,applied,events)={let mut plant=self.plant.lock().unwrap();
                    (plant.sample(),plant.applied,plant.events.clone())};
                let (_,_,cursor)=self.insertion.unwrap();
                let last_nonzero=events.iter().rev().find(|e|is_nonzero_applied(e)).map_or(cursor,|e|e.seq);
                let candidate=events.iter().find(|e|e.seq>last_nonzero && e.seq>cursor && is_zero_event(e)
                    && e.session.as_deref()==Some(goal.session_id.as_str()) && self.source(e.source,Stage::Inserted));
                if let Some(stopped)=candidate {
                    if applied==Twist2::ZERO && distance3(body.0,stopped.body.0)<=1e-12 && stopped.at.elapsed()>=Duration::from_secs(5) {
                        let end=Instant::now();let evidence=self.evidence.lock().unwrap();
                        let samples:Vec<_>=evidence.maps.values().filter(|r|r.stage==Stage::Inserted && r.sample>=stopped.at && r.sample<=end).collect();
                        let consumed:BTreeSet<_>=evidence.consumed.iter().filter(|(at,_)|*at>=stopped.at&&*at<=end).map(|(_,s)|*s).collect();
                        let built_consumed=samples.iter().filter(|r|evidence.built.contains_key(&r.source)&&consumed.contains(&r.source)).count();
                        let continuous=samples.first().is_some_and(|r|r.sample.duration_since(stopped.at)<=Duration::from_secs(1))
                            && samples.last().is_some_and(|r|end.duration_since(r.sample)<=Duration::from_secs(1))
                            && samples.windows(2).all(|w|w[1].sample.duration_since(w[0].sample)<=Duration::from_secs(1));
                        if samples.len()>=20 && built_consumed>=2 && continuous {
                            let sample_count=samples.len();drop(evidence);
                            eprintln!("WALL_ACTIVE_HOLD seconds={} samples={} built_consumed={} stopped={stopped:?}",end.duration_since(stopped.at).as_secs_f64(),sample_count,built_consumed);
                            self.wall_hold=Some((stopped.clone(),end));break;
                        }
                    }
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            self.check_live("wall hold completed within original sixty seconds")?;
        } else {
            loop {
                self.check_live("box strict original goal arrival")?;
                let status=self.scheduler.status();
                let wire=self.evidence.lock().unwrap().states.iter().any(|state|
                    state_matches_goal(state,&goal) && state.state==3
                    && state.zero_ack_id==status.last_zero_ack_id.clone().unwrap_or_default()
                    && state.zero_applied_monotonic_ns as u64==status.last_fence_monotonic_ns);
                if status.last_terminal==Some(TerminalOutcome::Succeeded) && wire {break;}
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            let status=self.scheduler.status();let mut plant=self.plant.lock().unwrap();let arrived=plant.sample();
            let xyz=distance3(arrived.0,goal.goal.pose.position);let q=goal.goal.pose.orientation_xyzw;
            let goal_yaw=Quaternion {x:q[0],y:q[1],z:q[2],w:q[3]}.yaw();
            let yaw=(plant.yaw-goal_yaw).sin().atan2((plant.yaw-goal_yaw).cos()).abs();
            if xyz>=0.1 || yaw>=0.15 || plant.applied!=Twist2::ZERO {return Err(format!("strict box arrival xyz={xyz} yaw={yaw} applied={:?}",plant.applied));}
            let (_,_,cursor)=self.insertion.unwrap();
            let fence=plant.events.iter().find(|e|e.kind=="fence" && e.seq>cursor && e.ack==status.last_zero_ack_id
                && e.monotonic_ns==status.last_fence_monotonic_ns).ok_or("strict arrival lacks matching actual fence")?;
            let fence_seq=fence.seq;drop(plant);
            eprintln!("STRICT_ARRIVAL xyz={xyz} yaw={yaw} fence_seq={fence_seq} elapsed_s={}",self.started.elapsed().as_secs_f64());
        }
        self.validate_paths(&goal)?;
        if self.goal.as_ref()!=Some(&original) {return Err("original sealed goal mutated".into());}
        Ok(())
    }
    fn audit_after_cleanup(&self)->Result<(),String> {
        let goal=self.goal.as_ref().ok_or("no original goal")?;
        let (insert,insert_body,cursor)=self.insertion.ok_or("unique insertion not established")?;
        let plant=self.plant.lock().unwrap();let events=plant.events.clone();let segments=plant.segments.clone();drop(plant);
        if segments.is_empty() {return Err("missing actual trajectory intervals".into());}
        let mut oracle_nodes=0usize;
        for (index,segment) in segments.iter().enumerate() {
            if index>0 && segments[index-1].to!=segment.from {return Err(format!("trajectory gap at {index}"));}
            if segment.from<insert && segment.to>insert {return Err(format!("unsplit insertion interval {index}"));}
            if segment.motion.obstacle_active!=(segment.from>=insert) {return Err(format!("wrong physical obstacle epoch at {index}"));}
            oracle_nodes+=check_segment(&segment.motion,self.scene).map_err(|e|format!("actual segment {index}: {e}; {segment:?}"))?;
        }
        if !segments.iter().any(|s|s.from==insert && s.motion.obstacle_active) {return Err("prior applied command's insertion boundary was not recorded".into());}
        eprintln!("OFFLINE_ORACLE_PASS segments={} nodes={oracle_nodes} exact_insertion={insert:?} support_domain=full_footprint collision=strict_contact outcome=continuous_se2_only",segments.len());
        for event in events.iter().filter(|e|is_nonzero_applied(e)) {
            if event.twist.linear.x<0.0 || event.twist.linear.x>0.5+1e-12 || event.twist.linear.y!=0.0 || event.twist.angular_z.abs()>0.8+1e-12 {
                return Err(format!("actual applied motion violates original limits: {event:?}"));
            }
            if event.deadline.is_some_and(|deadline|event.at>deadline) {return Err(format!("bounded nonzero application missed realtime deadline: {event:?}"));}
        }
        let unbounded_nonzero=events.iter().filter(|e|is_nonzero_applied(e)&&e.deadline.is_none()).count();
        eprintln!("MOTION_ADMISSION_AUDIT bounded_deadlines=checked unbounded_nonzero={unbounded_nonzero} original_current_footprint_exemption=unchanged");
        let evidence=self.evidence.lock().unwrap();let sources:Vec<_>=evidence.maps.values().cloned().collect();
        let mut first_inserted=None;let mut last_clear_geometry=None;let mut inserted_geometry=None;let mut previous:Option<&SourceRecord>=None;
        for row in &sources {
            if !row.all_keys_exact || !row.all_observed_ns_current || row.ground_count!=6000 {return Err(format!("source ledger mismatch {row:?}"));}
            if let Some(prior)=previous {if row.sample.duration_since(prior.sample)<SAMPLE_PERIOD || row.source<=prior.source {return Err("non-real-time or regressing sensor samples".into());}}
            previous=Some(row);
            if row.stage==Stage::Clear {if inserted_geometry.is_some() {return Err("obstacle was removed".into());}last_clear_geometry=Some(row.root_geometry_revision);}
            else {
                if row.sample<insert || row.obstacle_count!=obstacle_keys(self.scene).len() {return Err("bad insertion source timing/count".into());}
                if first_inserted.is_none() {first_inserted=Some(row.source);
                    if row.obstacle_hits_min!=1 || row.obstacle_hits_max!=1 || Some(row.root_geometry_revision)<=last_clear_geometry {return Err("first obstacle frame geometry/hits wrong".into());}
                    inserted_geometry=Some(row.root_geometry_revision);
                } else if inserted_geometry!=Some(row.root_geometry_revision) {return Err("evidence-only scan changed geometry".into());}
            }
        }
        let consumed:BTreeSet<_>=evidence.consumed.iter().filter(|(at,_)|*at>=insert).map(|(_,source)|*source)
            .chain(events.iter().filter(|e|e.at>=insert).map(|e|e.source)).collect();
        let built_inserted:Vec<_>=sources.iter().filter(|row|row.stage==Stage::Inserted && consumed.contains(&row.source))
            .filter_map(|row|evidence.built.get(&row.source).map(|built|(row,built.clone()))).collect();
        if built_inserted.is_empty() {return Err("no built AND scheduler-consumed inserted source".into());}
        // Original queries are separate diagnostics. They never replace the oracle.
        for (row,built) in &built_inserted {
            let map=built.snapshot.as_ref().map_err(|e|format!("built source {} failed: {e}",row.source))?;
            audit_observations(map,self.scene,Stage::Inserted,row.source)?;
        }
        let (row,built)=&built_inserted[0];let map=built.snapshot.as_ref().map_err(|e|e.to_string())?;
        let mut witness=None;let direct=map.validate_body_motion_with_witness(insert_body,body_pose(3.5,1.5,0.0),&[],&mut witness);
        eprintln!("CONSUMED_ORIGINAL_DIRECT source={} result={direct:?} first_witness={witness:?} independent_entity_geometry=true",row.source);
        if direct.is_ok() {return Err("original direct old corridor remains admitted on inserted source".into());}
        let view=map.with_current_body_support(insert_body.0,insert_body.1).map_err(|e|format!("actual insertion body support: {e:?}"))?;
        let mut witness=None;let direct=view.validate_body_motion_with_witness(insert_body,body_pose(3.5,1.5,0.0),&[],&mut witness);
        eprintln!("CONSUMED_ACTUAL_START_OVERLAY source={} result={direct:?} first_witness={witness:?}",row.source);
        if direct.is_ok() {return Err("current-body support overlay permits crossing occupied body".into());}
        let initial_revision=events.iter().filter(|e|e.seq<=cursor && is_nonzero_applied(e)).map(|e|e.revision).max().ok_or("no initial accepted nonzero")?;
        let stopped=events.iter().find(|e|e.at>=insert && is_zero_event(e) && e.session.as_deref()==Some(goal.session_id.as_str())
            && sources.iter().any(|r|r.source==e.source && r.stage==Stage::Inserted));
        let global_recovery=stopped.and_then(|stop|events.iter().find(|e|e.seq>stop.seq && is_nonzero_applied(e) && e.revision>initial_revision
            && evidence.path_receipts.iter().any(|(at,revision,receipt_unix)|*at>=stop.at && *at<=e.at && *revision as u64==e.revision
                && evidence.states.iter().any(|s| {
                    let stamp=s.stamp_sec*1_000_000_000+i64::from(s.stamp_nsec);
                    state_matches_goal(s,goal)&&s.state==1&&s.path_revision as u64<=e.revision
                        && stamp>=stop.unix_ns && stamp<=*receipt_unix && stamp<=e.unix_ns
                }))));
        // OFF soft refresh keeps the old reference usable under the original
        // guard. A zero/AwaitingPath chain is optional, but publication alone
        // is insufficient: require a later actual command on the new revision.
        let inserted_published=sources.iter().filter(|row|row.stage==Stage::Inserted)
            .map(|row|(row.published,row.source))
            .min_by_key(|(at,_)|*at);
        let soft_global_refresh=inserted_published.and_then(|(published_at,_)|events.iter().find(|e|
            e.at>=published_at && is_nonzero_applied(e) && e.revision>initial_revision
            && e.session.as_deref()==Some(goal.session_id.as_str())
            && built_inserted.iter().any(|(row,_)|row.source==e.source)
            && evidence.paths.iter().any(|path|path.path_revision>0 && path.path_revision as u64==e.revision
                && validate_path_goal_correlation(path,goal).is_ok())
            && evidence.path_receipts.iter().any(|(at,revision,_)|*at>=published_at && *at<=e.at
                && *revision>0 && *revision as u64==e.revision)));
        // Path delivery may precede the scheduler's first observed consumption;
        // require an inserted source at actual execution, not before receipt.
        // Receipt ordering proves observed delivery/execution, not the precise
        // planner snapshot. PathEnvelope.committed_map remains the sealed goal's
        // identity; owner tests must separately cover the captured input source.
        if self.scene==Scene::Wall {
            let (stop,hold_end)=self.wall_hold.as_ref().ok_or("wall has no completed passive stationary window")?;
            if hold_end.duration_since(stop.at)<Duration::from_secs(5) {return Err("wall stationary window too short".into());}
            if events.iter().any(|e|e.seq>stop.seq && e.at<=*hold_end && is_nonzero_applied(e)) {
                return Err("wall accepted a nonzero within the completed five-second window".into());
            }
            if segments.iter().filter(|s|s.from>=stop.at&&s.from<*hold_end).any(|s|
                distance3(s.motion.end.position,stop.body.0)>1e-12 || s.motion.twist!=Twist2::ZERO) {
                return Err("wall completed stationary window trace moved".into());
            }
            if evidence.states.iter().any(|s|state_matches_goal(s,goal)&&s.state==3) {
                return Err("wall falsely published success through cleanup".into());
            }
            if segments.iter().filter(|s|s.motion.obstacle_active).any(|s|s.motion.end.position[0]>=44.0*f64::from(RESOLUTION)) {
                return Err("wall crossed its analytic separation plane".into());
            }
        }
        let same_global_revision=events.iter().filter(|e|e.at>=insert&&is_nonzero_applied(e)).all(|e|e.revision==initial_revision);
        let local_display=evidence.states.iter().any(|s|state_matches_goal(s,goal)&&!s.local_path.is_empty()
            && sources.iter().any(|r|r.stage==Stage::Inserted&&r.source==s.costmap_source_timestamp_ns));
        let actual_detour=segments.iter().filter(|s|s.motion.obstacle_active).any(|s|(s.motion.end.position[1]-1.5).abs()>0.3);
        if self.scene==Scene::Box && !self.local_planning_enabled && soft_global_refresh.is_none() {
            return Err(format!("OFF coverage not established (separate from continuous safety and strict arrival): inserted-source publication/new correlated wire revision/actual nonzero execution on inserted source chain missing; inserted_published={inserted_published:?} stopped={stopped:?} global_resume={global_recovery:?} same_global_revision={same_global_revision} local_display={local_display} actual_detour={actual_detour}"));
        }
        // This audit also runs after a failed exercise. Only STRICT_ARRIVAL in
        // exercise() reports terminal arrival; these labels classify the trace.
        eprintln!("EVIDENCE_AUDIT scene={:?} local_planning_enabled={} classification={} initial_revision={initial_revision} inserted_source={} built_consumed={} inserted_published={inserted_published:?} soft_global_execution={soft_global_refresh:?} natural_zero={stopped:?} global_resume={global_recovery:?} same_global_revision={same_global_revision} local_display={local_display} actual_detour={actual_detour} precise_new_global_input_identity=not_independently_proven selected_command_proof_cells=not_recorded physical_braking=not_proven uds=not_covered",
            self.scene,self.local_planning_enabled,if self.scene==Scene::Wall {"bounded_active_five_second_wall_hold"}else if !self.local_planning_enabled {"off_soft_global_reference_executed"}else if global_recovery.is_some() {"natural_stop_global_replan_executed"}else if same_global_revision {"on_continuous_trace_same_global_revision"}else {"on_continuous_trace_global_order_not_fully_attributed"},first_inserted.unwrap(),built_inserted.len());
        Ok(())
    }

    fn dump_evidence(&self) {
        let (events,segments)={let plant=self.plant.lock().unwrap();(plant.events.clone(),plant.segments.clone())};
        let (sources,paths,states,consumed,receipts)={let evidence=self.evidence.lock().unwrap();
            (evidence.maps.values().cloned().collect::<Vec<_>>(),evidence.paths.clone(),evidence.states.clone(),evidence.consumed.clone(),evidence.path_receipts.clone())};
        // No per-call I/O while holding the motion boundary or sensor locks.
        for event in events {eprintln!("MOTION {event:?}");}
        for segment in segments {eprintln!("SEGMENT {segment:?}");}
        for consumed in consumed {eprintln!("CONSUMED {consumed:?}");}
        for source in sources {eprintln!("SOURCE {source:?}");}
        for receipt in receipts {eprintln!("PATH_RECEIPT {receipt:?}");}
        for path in paths {eprintln!("PATH {path:?}");}
        for state in states {eprintln!("STATE {state:?}");}
    }
    async fn shutdown(&mut self)->Result<(),String> {
        let mut errors=Vec::new();
        if let Err(error)=self.shutdown_inner().await {errors.push(error);}
        // Even failed cancellation must not skip feeder/observer/owner cleanup.
        self.feeder.abort();let _=(&mut self.feeder).await;
        for task in &mut self.observers {task.abort();let _=task.await;}
        for owner in [&mut self.executor,&mut self.planner] {
            if let Some(mut running)=owner.take() {
                if let Err(e)=running.shutdown() {errors.push(e.to_string());}
                match running.is_running() {Ok(false)=>{},Ok(true)=>errors.push("owner still running".into()),Err(e)=>errors.push(e.to_string())}
            }
        }
        if errors.is_empty() {Ok(())}else{Err(errors.join("; "))}
    }
    async fn shutdown_inner(&mut self)->Result<(),String> {
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
        Ok(())
    }
}
impl Drop for OccupancyHarness {
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
// Future means the suffix from the nearest projection on the actually accepted
// original path. The first segment is clipped at that projection, in XYZ.
fn future_path_hits_box(path:&schema::PathEnvelope,body:BodyPose)->bool {
    let points:Vec<_>=path.path.poses.iter().map(|p|p.pose.position).collect();
    let Some((nearest,t,_))=points.windows(2).enumerate().map(|(i,e)| {
        let d=[e[1][0]-e[0][0],e[1][1]-e[0][1],e[1][2]-e[0][2]];
        let len=d.iter().map(|v|v*v).sum::<f64>();
        let t=if len>0.0 {(0..3).map(|j|(body.0[j]-e[0][j])*d[j]).sum::<f64>()/len}else{0.0};
        let t=t.clamp(0.0,1.0);let projected=[e[0][0]+d[0]*t,e[0][1]+d[1]*t,e[0][2]+d[2]*t];
        (i,t,distance3(projected,body.0))
    }).min_by(|a,b|a.2.total_cmp(&b.2)) else {return false;};
    let r=f64::from(RESOLUTION);let lower=[40.0*r,24.0*r,2.0*r];let upper=[48.0*r,36.0*r,21.0*r];
    points[nearest..].windows(2).enumerate().any(|(i,e)| {
        let from=if i==0 {std::array::from_fn(|j|e[0][j]+(e[1][j]-e[0][j])*t)}else{e[0]};
        let mut lo=0.0_f64;let mut hi=1.0_f64;
        for axis in 0..3 {let delta=e[1][axis]-from[axis];
            if delta.abs()<1e-15 {if from[axis]<lower[axis] || from[axis]>upper[axis] {return false;}}
            else {let a=(lower[axis]-from[axis])/delta;let b=(upper[axis]-from[axis])/delta;
                lo=lo.max(a.min(b));hi=hi.min(a.max(b));if hi<lo {return false;}}
        } hi>=lo
    })
}
async fn qualify(scene:Scene,enabled:bool) {
    let mut harness=OccupancyHarness::start(enabled,scene).await;
    let result=harness.exercise().await;let status=harness.scheduler.status();
    let cleanup=harness.shutdown().await;
    // Include every interval through success/cancel/owner shutdown; never gate motion.
    harness.plant.lock().unwrap().sample();
    let audit=harness.audit_after_cleanup();harness.dump_evidence();
    eprintln!("FINAL scene={scene:?} local_planning_enabled={enabled} result={result:?} status={status:?} cleanup={cleanup:?} audit={audit:?}");
    assert!(result.is_ok() && cleanup.is_ok() && audit.is_ok(),"result={result:?}; cleanup={cleanup:?}; audit={audit:?}");
}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="opt-in exact once only after saved and passing preflight"]
async fn dynamic_box_local_planning_off() {qualify(Scene::Box,false).await;}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="opt-in exact once only after saved and passing preflight"]
async fn dynamic_box_local_planning_on() {qualify(Scene::Box,true).await;}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="opt-in exact once only after saved and passing preflight"]
async fn dynamic_wall_local_planning_off() {qualify(Scene::Wall,false).await;}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="opt-in exact once only after saved and passing preflight"]
async fn dynamic_wall_local_planning_on() {qualify(Scene::Wall,true).await;}

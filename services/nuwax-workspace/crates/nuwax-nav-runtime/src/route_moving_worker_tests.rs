//! Source-only, opt-in wall-clock experiments. No robot/device port is created.
//! Register from lib.rs as `#[cfg(test)] mod route_moving_worker_tests;`.
//! Run individually with --ignored --nocapture --test-threads=1. These are
//! qualification experiments: a timeout/deadline miss is a failure, not permission
//! to increase observation ages, change simulation mode, or fabricate feedback.
//!
//! The dependency direction is nav-stack -> nav-runtime, so importing
//! SurfaceNavigationConfig here would create a cycle. `fixture` mirrors
//! SurfaceNavigationConfig::new("map", .4, .3, .6, .1, .15, 18, .2), then
//! apply_calibration with the explicit generic body/motion/terrain below.
//! Every geometry field is spelled out; follower defaults come from the same
//! SurfaceFollowerConfig::new used by the factory. Factory mutations are 20Hz,
//! motion_limits, speed, terrain observation age, and local-planning ON/OFF.
//! No actuator env overrides, basic-kinematics override or simulation=true.
//! The fixed-window replacement matrix is supplementary to deterministic owner
//! lifecycle tests; it does not prove the exact late-completion receipt ordering.
//! Source resolution is separately 5cm (current FineCloud default), with 1cm
//! explicitly labeled stress. This is generic synthetic calibration, not Go2.

use super::*;
#[path="route_moving_cadence_trace.rs"]
mod cadence_trace;
use cadence_trace::{SchedRead, SchedReader};
use crate::control_profile::{self, Counter, Record as StageRecord, Stage};
use nuwax_nav_control::{integrate, PlanarPose};
use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
#[cfg(feature="body-query-profile")]
use nuwax_nav_mls_planner::body_query_profile::{self, Counter as BodyCounter, Record as BodyRecord, Stage as BodyStage};
use nuwax_nav_types_messages::{GoalOptions, NavFrameId, MotionLimits, MotionModel, Point3, Pose3, Quaternion, Timestamp};
use std::collections::BTreeMap;
use std::sync::{atomic::{AtomicBool, AtomicUsize, Ordering}, mpsc, Mutex, Weak};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const PERIOD: Duration = Duration::from_millis(50);
const SENSOR_PERIOD: Duration = Duration::from_millis(200);
const WIDTH: f64 = 0.3;
const OBS_AGE: f64 = 1.0; // explicit live sensor health age, not ground expiry
const MAX_TICKS: usize = 1_200;
// The compute owner can finish a little more work than consumer tick count.
// Standard runs retain MAX_TICKS + 16 samples; existing 120s runs retain
// 2 * MAX_TICKS + 16. Further samples are counted, never grow this new buffer.
const PROFILE_DRAIN_ALLOWANCE: usize = 16;

type Body = ([f64; 3], [f64; 4]);
fn unix_ns() -> u64 { SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos().try_into().unwrap() }
fn stamp() -> Timestamp { let ns=unix_ns(); Timestamp::from_unix((ns/1_000_000_000) as i64,(ns%1_000_000_000) as u32) }
fn pose(x:f64,y:f64,yaw:f64)->PoseStamped { PoseStamped {frame_id:NavFrameId::Map,stamp:stamp(),pose:Pose3 {position:Point3::new(x,y,0.5),orientation:Quaternion::from_yaw(yaw)}} }
fn planar(p:&PoseStamped)->PlanarPose {PlanarPose{x:p.pose.position.x,y:p.pose.position.y,yaw:p.pose.orientation.yaw()}}
fn body(p:PlanarPose)->Body {let q=Quaternion::from_yaw(p.yaw);([p.x,p.y,0.5],[q.x,q.y,q.z,q.w])}
fn nonzero(v:Twist2)->bool {v.linear.x.hypot(v.linear.y)>1e-9 || v.angular_z.abs()>1e-9}
fn limits()->MotionLimits {MotionLimits {
    model:MotionModel::Differential,max_forward_mps:0.5,max_reverse_mps:0.0,
    max_lateral_mps:0.0,max_angular_rps:0.8,linear_acceleration_mps2:1.0,
    guaranteed_braking_mps2:1.0,angular_acceleration_rps2:2.0,
    guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
    minimum_turning_radius_m:0.0,can_rotate_in_place:true,
}}
pub(crate) fn fixture(enabled:bool, resolution:f32, length:f32)->(SurfaceFollowerConfig,Arc<SurfaceSnapshot>,Arc<Vec<[i32;3]>>) {
    let envelope=SurfaceBodyEnvelope {fixed_collision:None,footprint_xy_m:vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],
        z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    // Exact new/apply_calibration geometry mutations: body radius + .1 soft
    // clearance, voxel-height extraction and zero hard centre-line clearance.
    let geometry=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,
        surface_closing_radius:0.0,node_spacing_m:(0.1_f64*3.0) as f32,
        wall_clearance_m:0.0,wall_buffer_m:(envelope.radius()+f64::from(0.1_f32)) as f32,
        wall_buffer_weight:2.0,step_threshold_m:0.15,step_penalty_weight:2.0,
        max_slope_deg:18.0,min_step_tread_depth_m:0.2,goal_tolerance:0.2,viz_publish_hz:0.0};
    let mut config=SurfaceFollowerConfig::new(0.4);
    config.controller.control_frequency=1.0/DEFAULT_TICK_INTERVAL.as_secs_f64();
    config.controller.speed=limits().max_forward_mps;
    config.motion_limits=Some(limits());
    config.max_observation_age_sec=Some(OBS_AGE);
    config.local_planning_enabled=enabled;
    assert!(!config.controller.simulation);
    assert_eq!(DEFAULT_TICK_INTERVAL,PERIOD);
    let nx=(length/resolution).round() as i32;
    let ny=(3.0/resolution).round() as i32;
    let z=(0.1/resolution).round() as i32-1;
    let keys:Vec<_>=(0..nx).flat_map(|x|(0..ny).map(move|y|[x,y,z])).collect();
    let points:Vec<_>=keys.iter().map(|k|((k[0] as f32+0.5)*resolution,(k[1] as f32+0.5)*resolution,(k[2] as f32+0.5)*resolution)).collect();
    let map=Arc::new(SurfaceSnapshot::from_points(&points,geometry,resolution).unwrap().with_body_envelope(envelope).unwrap());
    (config,map,Arc::new(keys))
}

type EvidenceBlocks=BTreeMap<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>;
// Frozen whole-plane builder remains the default and independent reference.
fn whole_plane_evidence(keys:&[[i32;3]],sample_ns:u64,hits:u16)->EvidenceBlocks {
    let mut blocks=EvidenceBlocks::new();
    for &key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
        .insert(key,SurfaceObservation{observed_ns:sample_ns,hits});}
    blocks
}
// Fixture-only layout: integer indices into one immutable key owner, no evidence
// or permission. Membership changes require a new owner and a rebuilt layout.
struct PreparedEvidenceLayout {
    keys:Arc<Vec<[i32;3]>>,
    blocks:Vec<([i32;3],Vec<usize>)>,
}
impl PreparedEvidenceLayout {
    fn new(keys:Arc<Vec<[i32;3]>>)->Self {
        let mut blocks=BTreeMap::<[i32;3],Vec<usize>>::new();
        for (index,key) in keys.iter().enumerate() {blocks.entry(key.map(|v|v.div_euclid(32))).or_default().push(index);}
        for indices in blocks.values_mut() {indices.sort_unstable_by_key(|&index|keys[index]);}
        Self {keys,blocks:blocks.into_iter().collect()}
    }
    fn materialize(&self,keys:&Arc<Vec<[i32;3]>>,sample_ns:u64,hits:u16)->Option<EvidenceBlocks> {
        if !Arc::ptr_eq(&self.keys,keys) {return None;}
        let evidence=SurfaceObservation{observed_ns:sample_ns,hits};
        Some(self.blocks.iter().map(|(block,indices)|(*block,
            indices.iter().map(|&index|(self.keys[index],evidence)).collect())).collect())
    }
}

#[derive(Clone,Copy,Debug)]
struct SourceAssembly {group_ns:u64,replace_ns:u64,snapshot_ns:u64,cells:usize,blocks:usize,prepared:bool,layout_setup_ns:u64}
struct Frame {map:Arc<SurfaceSnapshot>,sample_ns:u64,ready_ns:u64,sequence:u16,assembly:SourceAssembly}
struct Sensor {receive:mpsc::Receiver<Frame>,stop:Arc<AtomicBool>,join:Option<std::thread::JoinHandle<()>>,dropped:Arc<AtomicUsize>}
impl Sensor {
    fn new(base:Arc<SurfaceSnapshot>,keys:Arc<Vec<[i32;3]>>)->Self {Self::with_prepared_layout(base,keys,false)}
    fn with_prepared_layout(base:Arc<SurfaceSnapshot>,keys:Arc<Vec<[i32;3]>>,prepared:bool)->Self {
        let (send,receive)=mpsc::sync_channel(1); // exactly one producer / one queued frame
        let stop=Arc::new(AtomicBool::new(false));let done=stop.clone();
        let dropped=Arc::new(AtomicUsize::new(0));let skipped=dropped.clone();
        let join=std::thread::spawn(move|| {
            let mut observations=SurfaceObservations::new(base.source_voxel_size_m(),32).unwrap();
            let layout_started=Instant::now();
            let layout=prepared.then(||PreparedEvidenceLayout::new(keys.clone()));
            let layout_setup_ns=if prepared {layout_started.elapsed().as_nanos() as u64}else{0};
            let mut sequence=0u16;
            let mut deadline=Instant::now();
            while !done.load(Ordering::Acquire) {
                if let Some(delay)=deadline.checked_duration_since(Instant::now()){std::thread::sleep(delay);}
                if done.load(Ordering::Acquire){break;}
                // One actual synthetic sensor sample now, before assembly. No
                // pre-dated frames; hits count distinct wall-spaced acquisitions.
                let sample_ns=unix_ns();sequence=sequence.saturating_add(1);
                let grouping=Instant::now();
                let blocks=match &layout {
                    Some(layout)=>layout.materialize(&keys,sample_ns,sequence).expect("immutable fixture key owner changed"),
                    None=>whole_plane_evidence(&keys,sample_ns,sequence),
                };
                let group_ns=grouping.elapsed().as_nanos() as u64;
                let block_count=blocks.len();let replacement=Instant::now();
                for (key,block) in blocks {observations.replace_block(key,block).unwrap();}
                let replace_ns=replacement.elapsed().as_nanos() as u64;
                let snapshot=Instant::now();
                observations.mark_complete_geometry_source();
                // Clone the previous persistent observation roots before replacing
                // blocks: same occupied columns retain their actual identity.
                let map=Arc::new(base.with_observations(Some(Arc::new(observations.clone()))));
                let snapshot_ns=snapshot.elapsed().as_nanos() as u64;
                let frame=Frame{map,sample_ns,ready_ns:unix_ns(),sequence,
                    assembly:SourceAssembly{group_ns,replace_ns,snapshot_ns,cells:keys.len(),blocks:block_count,prepared,layout_setup_ns}};
                match send.try_send(frame){Ok(())=>{},Err(mpsc::TrySendError::Full(_))=>{skipped.fetch_add(1,Ordering::Relaxed);},Err(mpsc::TrySendError::Disconnected(_))=>break}
                deadline+=SENSOR_PERIOD;
                if deadline<Instant::now(){deadline=Instant::now()+SENSOR_PERIOD;} // missed samples are dropped, never backfilled
            }
        });
        Self{receive,stop,join:Some(join),dropped}
    }
}
impl Drop for Sensor {fn drop(&mut self){self.stop.store(true,Ordering::Release);if let Some(t)=self.join.take(){t.join().unwrap();}}}

// No source payloads or query views are retained by these diagnostic records.
struct SourceFrameSample {sample_ns:u64,ready_ns:u64,delivered_ns:u64,assembly:SourceAssembly}
struct SourceInstallSample {source_view_seq:u64,sample_ns:Option<u64>,completed_ns:u64,probe_wait:Duration}

struct ControlStepSample {
    step_seq: u64,
    control_ms: f64,
    sched_before: SchedRead, sched_after: SchedRead,
    profile_begin: Duration, profile_end: Duration, probe_wait: Duration,
    stages: StageRecord,
    #[cfg(feature="body-query-profile")]
    body_queries: BodyRecord,
    source_view_seq: u64,
    source_present: bool,
    first_after_view_change: bool,
    pose_stamp_ns: Option<i64>,
    source_sample_ns: Option<u64>,
    step_entry_ns: u64,
    step_exit_ns: u64,
    outcome_reason: Option<PlannerStep>,
    pending_wait_reason: Option<String>,
    submitted_before: u64,
    submitted_after: u64,
    admitted_before: u64,
    admitted_after: u64,
    jobs_before: usize,
    jobs_after: usize,
    held_before: bool,
    held_after: bool,
    held_phase_before: usize,
    held_phase_after: usize,
    outcome: &'static str,
}

#[derive(Default)]
struct Probe {
    control_ms:Vec<f64>,map_update_ms:Vec<f64>,submitted:u64,admitted:u64,
    recheck_ms:Vec<f64>,recheck_overflow:usize,
    negative:usize,pending:usize,endpoints:Vec<[f64;3]>,
    control_samples:Vec<ControlStepSample>,control_sample_limit:usize,control_sample_overflow:usize,
    source_frames:Vec<SourceFrameSample>,source_frame_overflow:usize,
    source_installs:Vec<SourceInstallSample>,source_install_overflow:usize,
}
impl Probe {
    fn with_control_sample_budget(max_ticks: usize) -> Self {
        let limit = max_ticks.min(MAX_TICKS * 2) + PROFILE_DRAIN_ALLOWANCE;
        Self {
            control_ms: Vec::with_capacity(MAX_TICKS * 2),
            control_samples: Vec::with_capacity(limit),
            source_frames: Vec::with_capacity(limit),
            source_installs: Vec::with_capacity(limit),
            control_sample_limit: limit,
            ..Self::default()
        }
    }
    fn record_frame(&mut self,frame:&Frame,delivered_ns:u64) {
        if self.source_frames.len()<self.control_sample_limit {
            self.source_frames.push(SourceFrameSample{sample_ns:frame.sample_ns,ready_ns:frame.ready_ns,delivered_ns,assembly:frame.assembly});
        }else{self.source_frame_overflow=self.source_frame_overflow.saturating_add(1);}
    }
}

struct Measured {
    inner:RouteFollower,probe:Arc<Mutex<Probe>>,
    source:Option<Weak<SurfaceSnapshot>>,source_view_seq:u64,first_after_view_change:bool,
    pose_stamp_ns:Option<i64>,source_sample_ns:Option<u64>,step_seq:u64,
    sched:Option<SchedReader>, // opened lazily on the actual compute owner
    jobs:Arc<AtomicUsize>,held:Arc<AtomicBool>,held_phase:Arc<AtomicUsize>,
}
impl Measured {
    fn new(inner:RouteFollower,probe:Arc<Mutex<Probe>>,jobs:Arc<AtomicUsize>,held:Arc<AtomicBool>,held_phase:Arc<AtomicUsize>)->Self {
        Self {inner,probe,source:None,source_view_seq:0,first_after_view_change:false,
            pose_stamp_ns:None,source_sample_ns:None,step_seq:0,sched:None,jobs,held,held_phase}
    }
    fn clear_source_marker(&mut self) {
        self.source_sample_ns=None;
        if self.source.take().is_some() {
            self.source_view_seq=self.source_view_seq.saturating_add(1);
            self.first_after_view_change=true;
        }
    }
    fn recheck(&mut self,command:Twist2,current_state:bool)->Option<PlannerStep> {
        let started=Instant::now();
        let outcome=if current_state {self.inner.revalidate_selected_command_at_current_state(command)}
            else {self.inner.revalidate_selected_command(command)};
        let elapsed=started.elapsed().as_secs_f64()*1e3;
        let mut probe=self.probe.lock().unwrap();
        if probe.recheck_ms.len()<probe.control_sample_limit {probe.recheck_ms.push(elapsed);}
        else {probe.recheck_overflow+=1;}
        outcome
    }
}
impl NavPlanner for Measured {
    fn revalidate_selected_command(&mut self,c:Twist2)->Option<PlannerStep>{self.recheck(c,false)}
    fn revalidate_selected_command_at_current_state(&mut self,c:Twist2)->Option<PlannerStep>{self.recheck(c,true)}
    fn take_command_dispatch_bound(&mut self)->Option<crate::CommandDispatchBound> {self.inner.take_command_dispatch_bound()}
    fn configure_goal(&mut self,g:&PoseStamped,o:GoalOptions)->Result<(),RuntimeError>{self.inner.configure_goal(g,o)}
    fn configure_boundary(&mut self,b:&[[f64;2]]){self.inner.configure_boundary(b)}
    fn replace_path(&mut self,p:Path2D){self.inner.replace_path(p)}
    fn replace_execution_path(&mut self,p:ExecutionPath,y:Option<Vec<f64>>,g:PoseStamped)->Result<(),RuntimeError>{self.inner.replace_execution_path(p,y,g)}
    fn update_odometry(&mut self,p:PoseStamped){self.pose_stamp_ns=Some(p.stamp.nanos);self.inner.update_odometry(p)}
    fn update_costmap(&mut self,m:CostMap2D){self.inner.update_costmap(m);self.clear_source_marker();}
    fn update_execution_map(&mut self,m:ExecutionMap)->Result<(),RuntimeError>{
        // The marker follows the actual source view delivered to this compute
        // owner. Identity comes from the existing comparison, never a new hash,
        // an Arc address, the consumer's newer map, or a fabricated timestamp.
        // A weak marker does not pin prior source payloads or their query
        // caches. Its temporary upgrade is dropped before the inner update;
        // if the source is already gone, conservatively mark a new view.
        let (source,changed,sample_ns)=match &m {
            ExecutionMap::Surface(next) | ExecutionMap::SurfaceUpdate(crate::surface_changes::SurfaceMapUpdate {snapshot:next,..})=>{
                let changed=self.source.as_ref().and_then(Weak::upgrade)
                    .is_none_or(|old|!old.same_body_query_view(next));
                // All fixture observations share the actual frame timestamp.
                // One sentinel lookup at installation; no per-step map lookup,
                // source-timestamp API, source pinning, or timestamp rewriting.
                let key=[0,0,(0.1/next.source_voxel_size_m()).round()as i32-1];
                let sample_ns=next.observations().and_then(|observations|observations.get(key))
                    .map(|observation|observation.observed_ns).filter(|&ns|ns!=0);
                (Some(Arc::downgrade(next)),changed,sample_ns)
            },
            ExecutionMap::Planar(_)=>(None,false,None),
        };
        let t=Instant::now();let result=self.inner.update_execution_map(m);let ms=t.elapsed().as_secs_f64()*1e3;
        let completed_ns=unix_ns();
        if result.is_ok() {
            if changed {self.source_view_seq=self.source_view_seq.saturating_add(1);self.first_after_view_change=true;}
            self.source=source;self.source_sample_ns=sample_ns;
        }
        let probe_wait_started=Instant::now();let mut p=self.probe.lock().unwrap();
        let probe_wait=probe_wait_started.elapsed();p.map_update_ms.push(ms);
        if result.is_ok() {
            if p.source_installs.len()<p.control_sample_limit {
                p.source_installs.push(SourceInstallSample{source_view_seq:self.source_view_seq,sample_ns,completed_ns,probe_wait});
            }else{p.source_install_overflow=p.source_install_overflow.saturating_add(1);}
        }
        result
    }
    fn invalidate_execution_map(&mut self){self.inner.invalidate_execution_map();self.clear_source_marker();}
    fn observe_applied_command(&mut self,c:Twist2){self.inner.observe_applied_command(c)}
    fn execution_wait_reason(&self)->Option<String>{self.inner.execution_wait_reason()}
    fn local_path_for_display(&self)->Vec<[f64;3]>{self.inner.local_path_for_display()}
    fn step(&mut self,w:f64)->PlannerStep {
        self.step_seq=self.step_seq.saturating_add(1);
        let sched=self.sched.get_or_insert_with(SchedReader::open);
        let submitted_before=self.inner.local_planning_calls();let admitted_before=self.inner.local_plans();
        let jobs_before=self.jobs.load(Ordering::Acquire);let held_phase_before=self.held_phase.load(Ordering::Acquire);
        let held_before=self.held.load(Ordering::Acquire);
        let profile_begin_started=Instant::now();
        control_profile::begin();
        // This wrapper executes on the real compute-owner thread. MLS records
        // only this step's TLS work, not the concurrent planning owner's work,
        // map installation, or the independent post-run safety oracles.
        #[cfg(feature="body-query-profile")]
        body_query_profile::begin();
        let profile_begin=profile_begin_started.elapsed();
        let sched_before=sched.sample();
        let step_entry_ns=unix_ns();
        let t=Instant::now();let result=self.inner.step(w);let ms=t.elapsed().as_secs_f64()*1e3;
        let step_exit_ns=unix_ns();
        let sched_after=sched.sample();
        let profile_end_started=Instant::now();
        let stages=control_profile::end();
        #[cfg(feature="body-query-profile")]
        let body_queries=body_query_profile::end();
        let profile_end=profile_end_started.elapsed();
        let submitted_after=self.inner.local_planning_calls();let admitted_after=self.inner.local_plans();
        let jobs_after=self.jobs.load(Ordering::Acquire);let held_phase_after=self.held_phase.load(Ordering::Acquire);
        let held_after=self.held.load(Ordering::Acquire);
        let mut sample=ControlStepSample {step_seq:self.step_seq,control_ms:ms,stages,
            sched_before,sched_after,profile_begin,profile_end,probe_wait:Duration::ZERO,
            #[cfg(feature="body-query-profile")]
            body_queries,
            source_view_seq:self.source_view_seq,source_present:self.source.is_some(),
            first_after_view_change:std::mem::take(&mut self.first_after_view_change),pose_stamp_ns:self.pose_stamp_ns,
            source_sample_ns:self.source_sample_ns,step_entry_ns,step_exit_ns,outcome_reason:None,pending_wait_reason:None,
            submitted_before,submitted_after,admitted_before,admitted_after,jobs_before,jobs_after,
            held_before,held_after,held_phase_before,held_phase_after,
            outcome:match &result {PlannerStep::Command(v)=>if nonzero(*v){"command_nonzero"}else{"command_zero"},
                PlannerStep::Pending(_)=>"pending",PlannerStep::Succeeded=>"succeeded",
                PlannerStep::ReplanRequired(_)=>"replan_required",PlannerStep::Failed(_)=>"failed"}};
        // One existing stats lock after the measured step. New records never
        // reallocate; overflow is visible and cannot silently qualify a run.
        let probe_wait_started=Instant::now();let mut p=self.probe.lock().unwrap();
        sample.probe_wait=probe_wait_started.elapsed();p.control_ms.push(ms);p.submitted=submitted_after;
        if p.control_samples.len()<p.control_sample_limit {
            // Clone only retained non-command outcomes after timing. The record
            // count is bounded; fixed-fixture reason payload bytes are not capped.
            // Format after drain and never infer a TTL cause from this text.
            if matches!(&result,PlannerStep::Pending(_)|PlannerStep::ReplanRequired(_)|PlannerStep::Failed(_)) {
                sample.outcome_reason=Some(result.clone());
                if matches!(&result,PlannerStep::Pending(_)){sample.pending_wait_reason=self.inner.execution_wait_reason();}
            }
            p.control_samples.push(sample);
        }
        else {p.control_sample_overflow=p.control_sample_overflow.saturating_add(1);}
        if admitted_after>p.admitted {if let Some(end)=self.inner.local_path().and_then(|path|path.poses.last()){let v=end.pose.position;p.endpoints.push([v.x,v.y,v.z]);}}
        p.admitted=admitted_after;
        p.negative+=usize::from(matches!(result,PlannerStep::Failed(_)|PlannerStep::ReplanRequired(_)));
        p.pending+=usize::from(matches!(result,PlannerStep::Pending(_)));result
    }
    fn stop(&mut self){self.inner.stop()}
}

#[derive(Clone,Copy,Debug,PartialEq)]
enum Scenario {Goal,HoldFirst,HoldSecond,HoldSecondContinuity,ReplaceGoal,ReplacePath,ReplaceBoundary,ReplaceGeometry,Cancel}
struct RawReceipt {tick:usize,map:Arc<SurfaceSnapshot>,boundary:Arc<Vec<[f64;2]>>,pose:PlanarPose,before:Twist2,command:Twist2,ns:u64,
    source_sample_ns:u64,source_sequence:u16,pose_stamp_ns:i64,worker_result_age_ms:f64,worker_remaining_ms:f64,
    receipt_pose:crate::command_proof_trace::PoseTrace,proof_context:Option<crate::command_proof_trace::DeliveredCommandContext>}
struct RawSegment {map:Arc<SurfaceSnapshot>,boundary:Arc<Vec<[f64;2]>>,from:PlanarPose,to:PlanarPose,ns:u64,dt:f64}
fn source_healthy_at(sample_ns:u64,checked_ns:u64)->bool {
    sample_ns!=0 && checked_ns.checked_sub(sample_ns).is_some_and(|age|age<=(OBS_AGE*1e9)as u64)
}
// Acquisition health is checked at receipt, separately from every geometric
// sample of the complete stopping trajectory. Ground has no expiry lease.
fn receipt_oracle_at(r:&RawReceipt,resolution:f32,anchor_ns:u64)->bool {
    if !source_healthy_at(r.source_sample_ns,anchor_ns) {return false;}
    let make_check=|| |a:PlanarPose,b:PlanarPose,_elapsed:f64| {
        r.map.validate_body_motion(body(a),body(b),&r.boundary)
            .and_then(|_|r.map.body_query_with_current_support(body(a).0,body(a).1)
                .and_then(|view|view.validate_body_motion(body(a),body(b),&r.boundary))).is_ok()
    };
    nuwax_nav_control::check_stopping_trajectory_with_period_check(r.pose,r.before,r.command,&limits(),
        PERIOD.as_secs_f64(),f64::from(resolution)*0.5,r.map.body_envelope().unwrap().radius(),make_check(),make_check()).is_some()
}
fn report_proof_receipts(receipts:&[RawReceipt]) {
    let mut guard_age=Vec::new();let mut guard_delay=Vec::new();let mut compute_delay=Vec::new();let mut publication_delay=Vec::new();
    let mut pose_to_receipt=Vec::new();let mut compute_to_receipt=Vec::new();
    let mut missing=0;let mut state_mismatch=0;let mut worker_guard_mismatch=0;let mut clock_invalid=0;let mut previous_id=None;
    for receipt in receipts {
        let Some(context)=&receipt.proof_context else {missing+=1;continue;};
        assert_eq!(context.proof.guard.selected_command_bits,crate::command_proof_trace::command_bits(receipt.command));
        assert!(previous_id.is_none_or(|previous|context.proof.input.result_id>previous),"one result must never be delivered twice");
        previous_id=Some(context.proof.input.result_id);
        // 本夹具的输入位姿使用Unix源时间，按实际发布结果中的同一输入配对。
        // 未跟踪的回执保持缺失，不以最近一拍的位姿代替它的真实输入。
        if let Some(pose)=context.proof.input.pose.as_ref().filter(|p|p.stamp.nanos>0) {
            if let Some(delay)=receipt.ns.checked_sub(pose.stamp.nanos as u64) {
                pose_to_receipt.push(delay as f64/1e6);
            } else {clock_invalid+=1;}
        }
        if let Some(delay)=receipt.ns.checked_sub(context.proof.input.compute_start_unix_ns) {
            compute_to_receipt.push(delay as f64/1e6);
        } else {clock_invalid+=1;}
        let guard=&context.proof.guard;
        worker_guard_mismatch+=usize::from(!context.proof.worker_guard_pose_bits_equal
            || !context.proof.worker_guard_pose_stamp_equal || !context.proof.worker_guard_pose_frame_equal
            || !context.proof.worker_guard_applied_bits_equal);
        state_mismatch+=usize::from(guard.pose.bits!=receipt.receipt_pose.bits
            || guard.pose.frame_id!=receipt.receipt_pose.frame_id
            || guard.applied_bits!=crate::command_proof_trace::command_bits(receipt.before));
        // 当前守卫无地面有效期；用其真实诊断时刻与精确回执来源配对。
        // 该时刻只统计年龄，不补造守卫中的观察许可。
        {
            let checked=guard.checked_unix_ns;
            if let (Some(age),Some(delay))=(checked.checked_sub(receipt.source_sample_ns),receipt.ns.checked_sub(checked)) {
                guard_age.push(age as f64/1e6);guard_delay.push(delay as f64/1e6);
            }else{clock_invalid+=1;}
        }
        if let Some(delay)=context.proof.compute_completion_unix_ns.checked_sub(context.proof.input.compute_start_unix_ns){compute_delay.push(delay as f64/1e6);}else{clock_invalid+=1;}
        if let Some(delay)=context.retrieval_unix_ns.checked_sub(context.publication_unix_ns){publication_delay.push(delay as f64/1e6);}else{clock_invalid+=1;}
    }
    eprintln!("PROOF_RECEIPTS receipts={} traced={} missing={missing} worker_guard_basis_mismatches={worker_guard_mismatch} guard_receipt_state_mismatches={state_mismatch} invalid_clock_pairs={clock_invalid} source_age_at_selected_guard_ms={} guard_to_receipt_ms={} compute_ms={} publication_to_retrieval_ms={} result_identity=exact_stored_publication no_motion_authority_from_diagnostics=true",
        receipts.len(),receipts.len()-missing,measured_distribution(&guard_age),measured_distribution(&guard_delay),measured_distribution(&compute_delay),measured_distribution(&publication_delay));
    // 两次非零回执之间缺失的消费拍在原夹具中明确施加零速。末尾停车另算，
    // 此处只统计再次运动之前的停顿，不将末态或初始等待计入恢复事件。
    let pauses:Vec<_>=receipts.windows(2).filter(|p|p[1].tick>p[0].tick+1)
        .map(|p|(p[1].tick-p[0].tick-1) as f64*PERIOD.as_secs_f64()*1000.0).collect();
    eprintln!("NAVIGATION_TIMING pose_to_receipt_ms={} compute_to_receipt_ms={} paired_receipts={} missing_receipts={missing} pause_events={} pause_ms={} scope=synthetic_pose_sample_to_local_command_receipt",
        measured_distribution(&pose_to_receipt),measured_distribution(&compute_to_receipt),pose_to_receipt.len(),
        pauses.len(),measured_distribution(&pauses));
}
fn distribution(values:&[f64])->(f64,f64,f64) {let mut v=values.to_vec();v.sort_by(f64::total_cmp);if v.is_empty(){return (0.0,0.0,0.0);} (v[v.len()/2],v[((v.len()-1)as f64*0.95).ceil()as usize],*v.last().unwrap())}
fn measured_distribution(values:&[f64])->String {
    if values.is_empty(){"unavailable".into()}else{format!("{:?}",distribution(values))}
}

fn source_age_ms(sample_ns:Option<u64>,at_ns:u64)->Result<f64,&'static str> {
    let sample_ns=sample_ns.filter(|&ns|ns!=0).ok_or("unknown")?;
    at_ns.checked_sub(sample_ns).map(|age|age as f64/1e6).ok_or("future")
}

fn report_source_ages(p:&Probe,factory:bool,context:&str) {
    // All diagnostics are printed after worker drain. Clock regressions and
    // unknown timestamps remain visible; neither is clamped to fresh age zero.
    let report_ages=|metric:&str,pairs:Vec<(Option<u64>,u64)>,omitted:usize| {
        let mut values=Vec::with_capacity(pairs.len());let mut unknown=0;let mut future=0;
        for (sample,at) in pairs {
            match sample.filter(|&ns|ns!=0) {
                None=>unknown+=1,
                Some(sample)=>match at.checked_sub(sample) {
                    Some(age)=>values.push(age as f64/1e6),None=>future+=1,
                },
            }
        }
        let age_ge_ttl=values.iter().filter(|&&age|age>=OBS_AGE*1e3).count();
        eprintln!("MOVING_SOURCE_AGE {context} metric={metric} known={} unknown={unknown} future={future} omitted={omitted} age_ms_median_p95_max={} initial_source_age_ge_ttl={age_ge_ttl} ttl_ms={} age_ge_ttl_is_not_actual_rejection=true",
            values.len(),measured_distribution(&values),OBS_AGE*1e3);
    };
    let mut spacing=Vec::with_capacity(p.source_frames.len());let mut nonincreasing=0;
    for pair in p.source_frames.windows(2) {
        match pair[1].sample_ns.checked_sub(pair[0].sample_ns) {
            Some(ns) if ns>0=>spacing.push(ns as f64/1e6),_=>nonincreasing+=1,
        }
    }
    eprintln!("MOVING_SOURCE_CADENCE {context} scope=consumer_delivered_frames_including_priming retained_frames={} record_limit={} omitted_frames={} delivered_frame_sample_spacing_ms_median_p95_max={} nonincreasing_sample_pairs={nonincreasing} configured_sensor_period_ms={} configured_period_is_not_cadence_evidence=true dropped_frame_samples_excluded=true producer_drop_count=MOVING_summary",
        p.source_frames.len(),p.control_sample_limit,p.source_frame_overflow,measured_distribution(&spacing),SENSOR_PERIOD.as_secs_f64()*1e3);
    eprintln!("SOURCE_LAYOUT {context} prepared_fixture_layout={:?} initial_layout_setup_ms={:?} setup_before_first_acquisition=true source_membership=fixed_immutable_key_owner evidence=all_cells_refreshed_each_actual_sample production_delta_path_unchanged=true",
        p.source_frames.first().map(|f|f.assembly.prepared),p.source_frames.first().map(|f|f.assembly.layout_setup_ns as f64/1e6));
    for (label,field) in [("full_coverage_evidence_materialization",0),("observation_block_replacement",1),("metadata_stamp_and_snapshot_construction",2)] {
        let values:Vec<_>=p.source_frames.iter().map(|frame|match field {
            0=>frame.assembly.group_ns,1=>frame.assembly.replace_ns,_=>frame.assembly.snapshot_ns,
        }as f64/1e6).collect();
        eprintln!("SOURCE_ASSEMBLY {context} stage={label} duration=wall_ms median_p95_max={} frames={} omitted={} scope=consumer_delivered_frames_including_first_build full_coverage=true geometry=fixed_observation_only source_profile=runtime_test_package_see_recorded_cargo_flags mls_profile=separately_reported acquisition_timestamp=before_all_stages excludes=channel_send_wait_drop_and_consumer_work cells_min_max={:?} blocks_min_max={:?}",
            measured_distribution(&values),p.source_frames.len(),p.source_frame_overflow,
            (p.source_frames.iter().map(|f|f.assembly.cells).min(),p.source_frames.iter().map(|f|f.assembly.cells).max()),
            (p.source_frames.iter().map(|f|f.assembly.blocks).min(),p.source_frames.iter().map(|f|f.assembly.blocks).max()));
    }
    report_ages("source_at_frame_ready",p.source_frames.iter().map(|frame|(Some(frame.sample_ns),frame.ready_ns)).collect(),p.source_frame_overflow);
    report_ages("source_at_consumer_delivery",p.source_frames.iter().map(|frame|(Some(frame.sample_ns),frame.delivered_ns)).collect(),p.source_frame_overflow);
    if factory {
        eprintln!("MOVING_SOURCE_OWNER {context} available=false reason=unprobed_factory");return;
    }
    eprintln!("MOVING_SOURCE_OWNER {context} available=true source_stamp=actual_installed_map_fixture_sentinel successful_installs={} omitted_installs={} retained_steps={} omitted_steps={} step_entry_is_not_selected_guard_entry=true actual_selected_guard_entry=unavailable_in_step_record available_when_exact_receipt_trace_present=true rejection_cause=exact_result_and_wait_only reason_storage=bounded_step_count_payload_bytes_not_capped",
        p.source_installs.len(),p.source_install_overflow,p.control_samples.len(),p.control_sample_overflow);
    report_ages("source_at_compute_owner_install_completion",p.source_installs.iter().map(|sample|(sample.sample_ns,sample.completed_ns)).collect(),p.source_install_overflow);
    report_ages("installed_source_at_compute_step_entry",p.control_samples.iter().map(|sample|(sample.source_sample_ns,sample.step_entry_ns)).collect(),p.control_sample_overflow);
    report_ages("installed_source_at_compute_step_exit",p.control_samples.iter().map(|sample|(sample.source_sample_ns,sample.step_exit_ns)).collect(),p.control_sample_overflow);
    for outcome in ["command_nonzero","command_zero","pending","replan_required","failed","succeeded"] {
        report_ages(&format!("installed_source_at_compute_step_entry_{outcome}"),p.control_samples.iter().filter(|sample|sample.outcome==outcome).map(|sample|(sample.source_sample_ns,sample.step_entry_ns)).collect(),p.control_sample_overflow);
    }
    for outcome in ["pending","replan_required","failed"] {
        let count=p.control_samples.iter().filter(|sample|sample.outcome==outcome).count();
        eprintln!("MOVING_STEP_REASON_SCOPE {context} outcome={outcome} retained_steps={count} printed_steps={} omitted_examples={} omitted_unrecorded_steps={} selection=first_two_and_last_three_retained_steps exact_payload=true ttl_cause_not_inferred=true",
            count.min(5),count.saturating_sub(5),p.control_sample_overflow);
        for (index,sample) in p.control_samples.iter().filter(|sample|sample.outcome==outcome).enumerate() {
            if index>=2&&index+3<count {continue;}
            let source_install=p.source_installs.iter().rev().find(|install|install.source_view_seq==sample.source_view_seq&&install.completed_ns<=sample.step_entry_ns);
            let stage_ms_calls=Stage::ALL.map(|stage|{let index=stage as usize;(stage.label(),sample.stages.wall_nanos[index]as f64/1e6,sample.stages.calls[index])});
            eprintln!("MOVING_STEP_REASON {context} step_seq={} outcome={outcome} reason={:?} pending_wait={:?} source_view_seq={} source_sample_ns={:?} source_install_completion_ns={:?} step_entry_ns={} step_exit_ns={} step_entry_source_age_ms={:?} step_exit_source_age_ms={:?} control_wall_ms={} inclusive_stage_ms_calls={stage_ms_calls:?}",
                sample.step_seq,sample.outcome_reason,sample.pending_wait_reason,sample.source_view_seq,sample.source_sample_ns,source_install.map(|install|install.completed_ns),sample.step_entry_ns,sample.step_exit_ns,source_age_ms(sample.source_sample_ns,sample.step_entry_ns),source_age_ms(sample.source_sample_ns,sample.step_exit_ns),sample.control_ms);
        }
    }
}

fn report_control_stages(p:&Probe,factory:bool,scenario:Scenario,enabled:bool,resolution:f32,startup:&str) {
    let context=format!("scenario={scenario:?} enabled={enabled} source_cm={} startup={startup}",resolution*100.0);
    report_source_ages(p,factory,&context);
    if factory {
        eprintln!("CONTROL_STAGE_SCOPE {context} available=false reason=unprobed_factory wall_not_thread_cpu=true");
        eprintln!("BODY_QUERY_SCOPE {context} available=false reason=unprobed_factory wall_not_thread_cpu=true");
        return;
    }
    eprintln!("SELECTED_RECHECK {context} count={} omitted={} wall_ms_median_p95_max={} scope=selected_stopping_trajectory_without_candidate_ranking",
        p.recheck_ms.len(),p.recheck_overflow,measured_distribution(&p.recheck_ms));
    let sample_scope=if p.control_sample_overflow==0 {"all_recorded_steps"}else{"retained_prefix_only_incomplete"};
    eprintln!("CONTROL_STAGE_SCOPE {context} available=true sample_scope={sample_scope} retained_steps={} record_limit={} omitted_steps={} attribution_complete={} nested_stages=inclusive_do_not_sum duration=wall_not_thread_cpu source_marker=successful_compute_owner_same_body_query_view_changes first_after_view_change=first_step_after_input_view_change metadata_cold_is_not_geometry_cold=true pose_stamp=actual_compute_owner_update_odometry_stamp planning_and_held_markers=independent_step_boundary_samples exact_transition_time=unavailable held_phase_0=not_started held_phase_1=held held_phase_2=released derived_body_view_identity=unavailable generated_candidate_count=unavailable candidate_eligibility_is_not_generated_candidate_count=true",
        p.control_samples.len(),p.control_sample_limit,p.control_sample_overflow,p.control_sample_overflow==0);
    // Reporting occurs after worker drain, outside every timed step. The scratch
    // allocation is bounded by the already bounded retained sample count.
    let mut values=Vec::with_capacity(p.control_samples.len());
    for stage in Stage::ALL {
        let index=stage as usize;
        values.clear();
        values.extend(p.control_samples.iter().filter(|sample|sample.stages.calls[index]>0)
            .map(|sample|sample.stages.wall_nanos[index]as f64/1e6));
        let calls:u64=p.control_samples.iter().map(|sample|u64::from(sample.stages.calls[index])).sum();
        eprintln!("CONTROL_STAGE {context} sample_scope={sample_scope} stage={} inclusive=true called_steps={} calls={calls} called_step_ms_median_p95_max={}",
            stage.label(),values.len(),measured_distribution(&values));
    }
    // These counts are actual call-site events across every retained step,
    // including zero-event steps. Never interpret predicate calls as the number
    // of candidates generated internally by nav-control.
    for counter in Counter::ALL {
        let index=counter as usize;
        values.clear();
        values.extend(p.control_samples.iter().map(|sample|sample.stages.counters[index]as f64));
        let total=p.control_samples.iter().fold(0_u64,|sum,sample|sum.saturating_add(sample.stages.counters[index]));
        let nonzero_steps=p.control_samples.iter().filter(|sample|sample.stages.counters[index]>0).count();
        eprintln!("CONTROL_COUNTER {context} sample_scope={sample_scope} counter={} total={total} nonzero_steps={nonzero_steps} retained_steps={} retained_step_count_median_p95_max={}",
            counter.label(),p.control_samples.len(),measured_distribution(&values));
    }
    #[cfg(feature="body-query-profile")]
    report_body_query_stages(p,&context,sample_scope);
    #[cfg(not(feature="body-query-profile"))]
    eprintln!("BODY_QUERY_SCOPE {context} available=false reason=feature_disabled required_feature=body-query-profile wall_not_thread_cpu=true");
    let mut slowest:[Option<&ControlStepSample>;5]=[None;5];
    for sample in &p.control_samples {
        if let Some(index)=slowest.iter().position(|old|old.is_none_or(|old|sample.control_ms>old.control_ms)) {
            for destination in ((index+1)..slowest.len()).rev() {slowest[destination]=slowest[destination-1];}
            slowest[index]=Some(sample);
        }
    }
    for (rank,sample) in slowest.into_iter().flatten().enumerate() {
        // All fields join on this compute-owner step, not on an asynchronously
        // newer consumer receipt. Absent stages retain calls=0 for that step.
        let stage_ms_calls=Stage::ALL.map(|stage|{
            let index=stage as usize;
            (stage.label(),sample.stages.wall_nanos[index]as f64/1e6,sample.stages.calls[index])
        });
        let counter_counts=Counter::ALL.map(|counter|(counter.label(),sample.stages.counters[counter as usize]));
        eprintln!("CONTROL_SLOW_STEP {context} sample_scope={sample_scope} rank={} step_seq={} control_wall_ms={:.6} source_view_seq={} source_present={} first_after_view_change={} pose_stamp_ns={:?} source_sample_ns={:?} step_entry_ns={} step_exit_ns={} step_entry_source_age_ms={:?} step_exit_source_age_ms={:?} submitted_before={} submitted_after={} admitted_before={} admitted_after={} jobs_before={} jobs_after={} held_before={} held_after={} held_phase_before={} held_phase_after={} outcome={} inclusive_stage_ms_calls={stage_ms_calls:?} counter_counts={counter_counts:?}",
            rank+1,sample.step_seq,sample.control_ms,sample.source_view_seq,sample.source_present,
            sample.first_after_view_change,sample.pose_stamp_ns,sample.source_sample_ns,sample.step_entry_ns,sample.step_exit_ns,
            source_age_ms(sample.source_sample_ns,sample.step_entry_ns),source_age_ms(sample.source_sample_ns,sample.step_exit_ns),sample.submitted_before,sample.submitted_after,
            sample.admitted_before,sample.admitted_after,sample.jobs_before,sample.jobs_after,
            sample.held_before,sample.held_after,sample.held_phase_before,sample.held_phase_after,sample.outcome);
        #[cfg(feature="body-query-profile")]
        report_body_query_slow_step(&context,sample_scope,"all_control_steps",rank+1,sample);
    }
}

#[cfg(feature="body-query-profile")]
fn dense_initial_table_payload_bytes_per_slot()->usize {
    std::mem::size_of::<bool>()+std::mem::size_of::<Option<Arc<Vec<[f64;2]>>>>()
        +std::mem::size_of::<Option<[f64;2]>>()
}

#[cfg(feature="body-query-profile")]
fn dense_initial_table_payload_bytes(record:&BodyRecord)->u64 {
    record.counters[BodyCounter::DenseAllocatedSlots as usize]
        .saturating_mul(dense_initial_table_payload_bytes_per_slot()as u64)
        .saturating_add(record.counters[BodyCounter::DenseVisitedTableSlots as usize]
            .saturating_mul(std::mem::size_of::<bool>()as u64))
}

#[cfg(feature="body-query-profile")]
fn report_body_query_stages(p:&Probe,context:&str,sample_scope:&str) {
    let query_steps=p.control_samples.iter().filter(|sample|sample.body_queries.calls.iter().any(|&calls|calls>0)).count();
    let first_after_view_change_steps=p.control_samples.iter().filter(|sample|sample.first_after_view_change).count();
    let query_steps_after_view_change=p.control_samples.iter().filter(|sample|sample.first_after_view_change
        &&sample.body_queries.calls.iter().any(|&calls|calls>0)).count();
    let nonzero_command_steps=p.control_samples.iter().filter(|sample|sample.outcome=="command_nonzero").count();
    eprintln!("BODY_QUERY_SCOPE {context} available=true feature=nondefault sample_scope={sample_scope} retained_steps={} record_limit={} omitted_steps={} attribution_complete={} query_steps={query_steps} first_after_view_change_steps={first_after_view_change_steps} query_steps_after_view_change={query_steps_after_view_change} nonzero_command_steps={nonzero_command_steps} thread=compute_owner interval=complete_inner_step nested_stages=inclusive_do_not_sum duration=wall_not_thread_cpu planning_owner_excluded=true map_installation_excluded=true post_run_oracles_excluded=true metadata_cold_is_not_geometry_cold=true source_view_marker=successful_compute_owner_same_body_query_view_changes nonzero_command_is_not_receipt_or_displacement=true actual_motion_evidence=MOVING_summary dense_cache_present_columns=initial_batch_hits_singletons_copied_non_singletons_arc_cloned_not_pointcloud_copies dense_cache_absent_columns=initial_batch_absence_not_guaranteed_later_misses",
        p.control_samples.len(),p.control_sample_limit,p.control_sample_overflow,p.control_sample_overflow==0);
    // The same bounded per-step records back all cohorts. A first step after an
    // input-view change may itself return Pending; it does not prove a body-query
    // cache miss or geometry rebuild. Subsequent steps may still have cold work.
    let mut values=Vec::with_capacity(p.control_samples.len());
    for (cohort,first_after_change) in [("all_steps",None),("first_after_view_change",Some(true)),("not_first_after_view_change",Some(false))] {
        let samples=||p.control_samples.iter().filter(move |sample|first_after_change.is_none_or(|first|sample.first_after_view_change==first));
        let retained_steps=samples().count();
        for stage in BodyStage::ALL {
            let index=stage as usize;
            values.clear();
            values.extend(samples().filter(|sample|sample.body_queries.calls[index]>0)
                .map(|sample|sample.body_queries.wall_nanos[index]as f64/1e6));
            let calls=samples().fold(0_u64,|sum,sample|sum.saturating_add(u64::from(sample.body_queries.calls[index])));
            let total_ms=samples().map(|sample|sample.body_queries.wall_nanos[index]as f64/1e6).sum::<f64>();
            eprintln!("BODY_QUERY_STAGE {context} sample_scope={sample_scope} cohort={cohort} stage={} inclusive=true retained_steps={retained_steps} called_steps={} calls={calls} total_ms={total_ms:.6} called_step_ms_median_p95_max={}",
                stage.label(),values.len(),measured_distribution(&values));
        }
        for counter in BodyCounter::ALL {
            let index=counter as usize;
            values.clear();
            values.extend(samples().map(|sample|sample.body_queries.counters[index]as f64));
            let total=samples().fold(0_u64,|sum,sample|sum.saturating_add(sample.body_queries.counters[index]));
            let nonzero_steps=samples().filter(|sample|sample.body_queries.counters[index]>0).count();
            eprintln!("BODY_QUERY_COUNTER {context} sample_scope={sample_scope} cohort={cohort} counter={} total={total} nonzero_steps={nonzero_steps} retained_steps={retained_steps} retained_step_count_median_p95_max={}",
                counter.label(),measured_distribution(&values));
        }
        values.clear();
        values.extend(samples().map(|sample|dense_initial_table_payload_bytes(&sample.body_queries)as f64));
        let total=samples().fold(0_u64,|sum,sample|sum.saturating_add(dense_initial_table_payload_bytes(&sample.body_queries)));
        eprintln!("BODY_QUERY_DERIVED {context} sample_scope={sample_scope} cohort={cohort} metric=initial_dense_table_payload_bytes basis=base_and_actual_visited_table_slots base_bytes_per_slot={} visited_bytes_per_slot=1 cumulative_requested_payload_not_peak_memory_or_allocator_bytes=true excludes=allocator_overhead_queues_output_cache_contents_and_other_allocations retained_steps={retained_steps} total={total} retained_step_bytes_median_p95_max={}",
            dense_initial_table_payload_bytes_per_slot(),measured_distribution(&values));
    }
    // Retain the existing top-five all-step join below and separately expose
    // command-producing steps so startup/Pending work cannot hide moving control.
    // A returned command is only a compute-owner outcome, not an applied receipt.
    let mut slowest:[Option<&ControlStepSample>;5]=[None;5];
    for sample in p.control_samples.iter().filter(|sample|sample.outcome=="command_nonzero") {
        if let Some(index)=slowest.iter().position(|old|old.is_none_or(|old|sample.control_ms>old.control_ms)) {
            for destination in ((index+1)..slowest.len()).rev() {slowest[destination]=slowest[destination-1];}
            slowest[index]=Some(sample);
        }
    }
    for (rank,sample) in slowest.into_iter().flatten().enumerate() {
        report_body_query_slow_step(context,sample_scope,"nonzero_command_steps",rank+1,sample);
    }
}

#[cfg(feature="body-query-profile")]
fn report_body_query_slow_step(context:&str,sample_scope:&str,ranking:&str,rank:usize,sample:&ControlStepSample) {
    // Format only after worker drain. Both profiles and every marker refer to
    // this exact step; storing the MLS record retains no source or query view.
    let stage_ms_calls=BodyStage::ALL.map(|stage|{
        let index=stage as usize;
        (stage.label(),sample.body_queries.wall_nanos[index]as f64/1e6,sample.body_queries.calls[index])
    });
    let counter_counts=BodyCounter::ALL.map(|counter|(counter.label(),sample.body_queries.counters[counter as usize]));
    let dense_initial_table_payload_bytes=dense_initial_table_payload_bytes(&sample.body_queries);
    eprintln!("BODY_QUERY_SLOW_STEP {context} sample_scope={sample_scope} ranking={ranking} rank={rank} step_seq={} control_wall_ms={:.6} source_view_seq={} source_present={} first_after_view_change={} pose_stamp_ns={:?} source_sample_ns={:?} step_entry_ns={} step_exit_ns={} step_entry_source_age_ms={:?} step_exit_source_age_ms={:?} submitted_before={} submitted_after={} admitted_before={} admitted_after={} jobs_before={} jobs_after={} held_before={} held_after={} held_phase_before={} held_phase_after={} outcome={} inclusive_stage_ms_calls={stage_ms_calls:?} counter_counts={counter_counts:?} dense_initial_table_payload_bytes={dense_initial_table_payload_bytes}",
        sample.step_seq,sample.control_ms,sample.source_view_seq,sample.source_present,
        sample.first_after_view_change,sample.pose_stamp_ns,sample.source_sample_ns,sample.step_entry_ns,sample.step_exit_ns,
            source_age_ms(sample.source_sample_ns,sample.step_entry_ns),source_age_ms(sample.source_sample_ns,sample.step_exit_ns),sample.submitted_before,sample.submitted_after,
        sample.admitted_before,sample.admitted_after,sample.jobs_before,sample.jobs_after,
        sample.held_before,sample.held_after,sample.held_phase_before,sample.held_phase_after,sample.outcome);
}
// Fixed scalar categories describe the consumer's actual Pending reasons; an
// empty 5Hz sensor queue during a 20Hz tick is expected and counted separately.
#[derive(Default,Debug)]
struct ConsumerWaits {pending:usize,source_fence:usize,expired_result:usize,pose_or_receipt:usize,arrival_pose:usize,planning:usize,other:usize,expired_at_receipt:usize,dispatch_expired_publication:usize,dispatch_expired_retrieval:usize,dispatch_expired_at_receipt:usize,finite_bound_nonzero:usize,unbounded_nonzero:usize}
impl ConsumerWaits {
    fn record(&mut self,reason:Option<String>){
        self.pending+=1;
        match reason.as_deref() {
            Some("地图或边界已更新，保持零速并重新计算")=>self.source_fence+=1,
            Some("局部速度计算结果已过期，保持零速并重新计算")=>self.expired_result+=1,
            Some("观察扫掠凭据在发布前已过期，保持零速并重新计算")=>self.dispatch_expired_publication+=1,
            Some("观察扫掠凭据在交付前已过期，保持零速并重新计算")=>self.dispatch_expired_retrieval+=1,
            Some("位姿或执行回执已更新，保持零速并重新计算")=>self.pose_or_receipt+=1,
            Some("位姿已更新，保持零速并重新核对到达状态")=>self.arrival_pose+=1,
            Some("局部轨迹正在计算，保持零速等待")=>self.planning+=1,
            _=>self.other+=1,
        }
    }
}
fn cpu_snapshot()->String {format!("stat={:?} status_threads={:?}",std::fs::read_to_string("/proc/self/stat").ok(),std::fs::read_to_string("/proc/self/status").ok().and_then(|s|s.lines().find(|l|l.starts_with("Threads:")).map(str::to_owned)))}

// Existing entrypoints intentionally remain the cold first-acquisition arm.
fn run(enabled:bool,resolution:f32,scenario:Scenario,factory:bool) {
    run_startup(enabled,resolution,scenario,factory,false);
}
fn run_startup(enabled:bool,resolution:f32,scenario:Scenario,factory:bool,primed:bool) {
    run_with_budget(enabled,resolution,scenario,factory,primed,MAX_TICKS);
}
fn run_with_budget(enabled:bool,resolution:f32,scenario:Scenario,factory:bool,primed:bool,max_ticks:usize) {
    run_with_source_layout(enabled,resolution,scenario,factory,primed,max_ticks,false);
}
fn run_with_source_layout(enabled:bool,resolution:f32,scenario:Scenario,factory:bool,primed:bool,max_ticks:usize,prepared:bool) {
    let (config,base,keys)=fixture(enabled,resolution,if matches!(scenario,Scenario::HoldSecond|Scenario::HoldSecondContinuity){10.0}else{6.0});
    eprintln!("MOVING_POLICY compiled_default=translation_advice prepared_fixture_layout={prepared}");
    let priming_started=Instant::now();
    let sensor=Sensor::with_prepared_layout(base.clone(),keys,prepared);
    let probe=Arc::new(Mutex::new(Probe::with_control_sample_budget(max_ticks)));
    let jobs=Arc::new(AtomicUsize::new(0));let held=Arc::new(AtomicBool::new(false));
    // Test-hook phase only: one selected job is held in each existing scenario.
    // This records a completed hold even if it begins/ends between step samples.
    // It is diagnostic; the existing held flag still drives the same assertions.
    let held_phase=Arc::new(AtomicUsize::new(0));
    let (release,receive)=mpsc::sync_channel(1);
    let mut worker=if factory {assert_eq!(scenario,Scenario::Goal);production_surface_backend(config.clone()).unwrap()} else {
        let count=jobs.clone();let active=held.clone();let phase=held_phase.clone();
        let inner=RouteFollower::new_with_planning_hook(config.clone(),Box::new(move |_| {
            let job=count.fetch_add(1,Ordering::SeqCst)+1;
            let hold=match scenario{Scenario::Goal=>false,Scenario::HoldSecond|Scenario::HoldSecondContinuity=>job==2,_=>job==1};
            if hold {active.store(true,Ordering::Release);phase.store(1,Ordering::Release);let _=receive.recv_timeout(Duration::from_secs(10));active.store(false,Ordering::Release);phase.store(2,Ordering::Release);}
        })).unwrap();
        planner_worker::PlannerWorker::new(Measured::new(inner,probe.clone(),jobs.clone(),held.clone(),held_phase.clone())).unwrap()
    };
    let mut map=base;let mut boundary=Arc::new(Vec::new());let mut frames=0usize;let mut sensor_no_new_frame=0usize;let mut consumer_waits=ConsumerWaits::default();
    let mut previous_frame=None::<u64>;let mut previous_source=None::<Arc<SurfaceSnapshot>>;let mut sequence=0u16;let mut installed_sample_ns=0u64;
    if primed {
        // Same live 5Hz producer and one-frame queue as the cold arm. Install two
        // separately acquired frames before any goal/path; never edit their hits
        // or timestamps. The producer continues normally after priming.
        for _ in 0..2 {
            let frame=sensor.receive.recv_timeout(Duration::from_secs(5)).expect("two real priming frames");
            let delivered_ns=unix_ns();probe.lock().unwrap().record_frame(&frame,delivered_ns);
            assert!(frame.sample_ns<=unix_ns());
            if let Some(previous)=previous_frame {
                assert!(frame.sample_ns>previous);
                let old=previous_source.as_ref().unwrap();
                assert!(nuwax_nav_pct::same_source_geometry(old,&frame.map));
                assert!(!old.same_body_query_view(&frame.map));
            }
            previous_frame=Some(frame.sample_ns);previous_source=Some(frame.map.clone());sequence=frame.sequence;frames+=1;
            installed_sample_ns=frame.sample_ns;map=frame.map;worker.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();
        }
        assert!(sequence>=2);
    }
    let priming_duration=if primed {priming_started.elapsed()}else{Duration::ZERO};
    let startup=if primed {"primed_two_real_frames"}else{"cold_first_acquisition"};
    let mut current=pose(0.75,1.5,0.0);
    let goal=pose(if matches!(scenario,Scenario::HoldSecond|Scenario::HoldSecondContinuity){7.75}else{1.75},1.5,std::f64::consts::FRAC_PI_2);
    let options=GoalOptions {position_tolerance_m:0.15,yaw_tolerance_rad:0.15,allow_reverse:false};
    worker.configure_goal(&goal,options).unwrap();worker.update_odometry(current.clone());
    worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![current.clone(),goal.clone()]}),None,goal.clone()).unwrap();
    let mut receipts=Vec::with_capacity(MAX_TICKS);let mut segments=Vec::with_capacity(MAX_TICKS);
    let mut gaps=Vec::with_capacity(MAX_TICKS);let mut consume_ms=Vec::with_capacity(MAX_TICKS);
    let mut command=Twist2::ZERO;let mut command_at=Instant::now();let mut command_deadline=command_at;let mut segment_ns=unix_ns();
    let mut nonzero_count=0;let mut accepted=0;let mut held_nonzero=0;let mut succeeded=false;let mut delivered_negative=0usize;let mut delivered_negative_reason=None::<String>;let mut expected_current_rejection=false;
    let mut hold_start=None;let mut hold_frame_count=0;let mut hold_admitted_count=0;let mut released_at=None;let mut continuity_complete=false;let mut changed=false;let mut retired_deadline=None;
    let mut cadence=cadence_trace::Trace::new(max_ticks);let mut consumer_sched=SchedReader::open();
    let mut timeout_wait=None;
    let started=Instant::now();let mut last_tick=started;let mut deadline=started;
    let cpu_before=cpu_snapshot();
    for tick in 0..max_ticks {
        let sched_before_sleep=consumer_sched.sample();
        let incoming_deadline=deadline;let before_sleep=Instant::now();
        let requested_sleep=deadline.checked_duration_since(before_sleep);
        if let Some(delay)=requested_sleep{std::thread::sleep(delay);}
        let now=Instant::now();let elapsed=now.duration_since(last_tick);gaps.push(elapsed.as_secs_f64()*1e3);
        let sched_after_wake=consumer_sched.sample();
        let mut phase_ends=[now;8];let mut probe_wait=[None;3];let mut map_call=None;
        // Ideal kinematic actuator: only previously accepted commands cause
        // displacement. Expiry caps their lifetime; no waypoint/goal teleport.
        let live=command_deadline.saturating_duration_since(last_tick.max(command_at));
        let since_receipt=now.saturating_duration_since(last_tick.max(command_at));
        let dt=since_receipt.min(live).as_secs_f64();
        if nonzero(command)&&dt>0.0 {
            let from=planar(&current);let to=integrate(from,command,dt);
            segments.push(RawSegment{map:map.clone(),boundary:boundary.clone(),from,to,ns:segment_ns,dt});
            current.pose.position.x=to.x;current.pose.position.y=to.y;current.pose.orientation=Quaternion::from_yaw(to.yaw);
        }
        if now>=command_deadline {command=Twist2::ZERO;}
        last_tick=now;current.stamp=stamp();phase_ends[0]=Instant::now();
        if let Ok(frame)=sensor.receive.try_recv(){
            let delivered_ns=unix_ns();
            let lock_started=Instant::now();let mut p=probe.lock().unwrap();probe_wait[0]=Some(lock_started.elapsed());
            p.record_frame(&frame,delivered_ns);drop(p);
            assert!(frame.sample_ns<=unix_ns());
            if let Some(previous)=previous_frame {assert!(frame.sample_ns>previous);let previous_map=previous_source.as_ref().unwrap();assert!(nuwax_nav_pct::same_source_geometry(previous_map,&frame.map));assert!(!previous_map.same_body_query_view(&frame.map));}
            previous_frame=Some(frame.sample_ns);previous_source=Some(frame.map.clone());sequence=frame.sequence;frames+=1;
            if !changed || !matches!(scenario,Scenario::ReplaceGeometry|Scenario::Cancel){installed_sample_ns=frame.sample_ns;map=frame.map;
                let map_started=Instant::now();worker.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();map_call=Some(map_started.elapsed());}
        }else{sensor_no_new_frame+=1;}
        phase_ends[1]=Instant::now();worker.update_odometry(current.clone());phase_ends[2]=Instant::now();
        if held.load(Ordering::Acquire)&&hold_start.is_none(){hold_start=Some(now);hold_frame_count=frames;
            let lock_started=Instant::now();let p=probe.lock().unwrap();probe_wait[1]=Some(lock_started.elapsed());hold_admitted_count=p.admitted;}
        if let Some(hold)=hold_start {
            if !changed && !matches!(scenario,Scenario::HoldFirst|Scenario::HoldSecond|Scenario::HoldSecondContinuity) {
                changed=true;
                match scenario {
                    Scenario::ReplaceGoal=>{let g=pose(current.pose.position.x,2.2,0.0);worker.configure_goal(&g,options).unwrap();},
                    Scenario::ReplacePath=>{let g=pose(current.pose.position.x,2.2,0.0);worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![current.clone(),g.clone()]}),None,g).unwrap();},
                    Scenario::ReplaceBoundary=>{boundary=Arc::new(vec![[4.0,0.0],[5.0,0.0],[5.0,1.0],[4.0,1.0]]);worker.configure_boundary(&boundary);},
                    Scenario::ReplaceGeometry=>{map=Arc::new(map.with_virtual_obstacles([0.0,0.0],0.1,60,30,&vec![true;60*30]).unwrap());worker.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();},
                    Scenario::Cancel=>worker.stop(),_=>{}
                }
                retired_deadline=Some(now+Duration::from_secs(2));
            }
            if released_at.is_none()&&now.duration_since(hold)>=Duration::from_millis(650)&&release.try_send(()).is_ok(){released_at=Some(now);}
        }
        phase_ends[3]=Instant::now();
        let t=Instant::now();let result=worker.step(WIDTH);let dispatch_bound=worker.take_command_dispatch_bound();consume_ms.push(t.elapsed().as_secs_f64()*1e3);
        phase_ends[4]=Instant::now();
        if matches!(&result,PlannerStep::Pending(_)){consumer_waits.record(worker.execution_wait_reason());}
        let receipt_at=Instant::now();
        // Local receipt admission uses the exact result's original bound. Keep
        // the later unchanged receipt oracle to expose any remaining time gap.
        let source_bound=CommandDispatchBound::from_source_timestamp(installed_sample_ns,Duration::from_secs_f64(OBS_AGE));
        let dispatch_expired=dispatch_bound.is_some_and(CommandDispatchBound::expired_now)||source_bound.expired_now();
        if dispatch_expired {consumer_waits.dispatch_expired_at_receipt+=1;}
        // Original worker result expiry is captured under its selection lock.
        // Replaying the same candidate never renews its permission horizon.
        let original_deadline=worker.delivered_command_deadline_for_test().unwrap_or(receipt_at);
        let permitted_until=original_deadline.min(receipt_at+PERIOD);
        if matches!(&result,PlannerStep::Command(_))&&permitted_until<=receipt_at{consumer_waits.expired_at_receipt+=1;}
        let before=command;command=match result {
            PlannerStep::Command(v) if permitted_until>receipt_at&&!dispatch_expired=>{accepted+=1;if nonzero(v){if dispatch_bound.is_some(){consumer_waits.finite_bound_nonzero+=1;}else{consumer_waits.unbounded_nonzero+=1;}nonzero_count+=1;if held.load(Ordering::Acquire)&&frames>hold_frame_count&&hold_start.is_some_and(|start|now.duration_since(start)>DEFAULT_CONTROL_HORIZON+PERIOD){held_nonzero+=1;}}v},
            PlannerStep::Succeeded=>{succeeded=true;Twist2::ZERO},
            negative @ (PlannerStep::ReplanRequired(_)|PlannerStep::Failed(_))=>{
                expected_current_rejection=changed&&matches!(scenario,Scenario::ReplaceBoundary|Scenario::ReplaceGeometry)
                    &&matches!(&negative,PlannerStep::ReplanRequired(_));
                delivered_negative+=1;delivered_negative_reason=Some(format!("{negative:?}"));
                // A real scheduler leaves this execution state on a terminal or
                // replan outcome. Retire the route now; never keep stepping it.
                worker.stop();Twist2::ZERO
            },_=>Twist2::ZERO,
        };
        // One actual acquisition is sufficient geometry; live stream health
        // must come from its real sample time, never a per-cell hit quorum.
        if nonzero(command){assert!(sequence>=1);assert!(source_healthy_at(installed_sample_ns,unix_ns()));}
        if changed&&matches!(scenario,Scenario::ReplaceGoal|Scenario::ReplaceBoundary|Scenario::ReplaceGeometry|Scenario::Cancel){assert!(!nonzero(command),"stale command after {scenario:?}");}
        command_at=receipt_at;command_deadline=permitted_until;segment_ns=unix_ns();
        if nonzero(command){receipts.push(RawReceipt{tick,map:map.clone(),boundary:boundary.clone(),pose:planar(&current),before,command,ns:segment_ns,source_sample_ns:installed_sample_ns,source_sequence:sequence,pose_stamp_ns:current.stamp.nanos,
            worker_result_age_ms:DEFAULT_CONTROL_HORIZON.saturating_sub(original_deadline.saturating_duration_since(receipt_at)).as_secs_f64()*1e3,
            worker_remaining_ms:original_deadline.saturating_duration_since(receipt_at).as_secs_f64()*1e3,
            receipt_pose:crate::command_proof_trace::PoseTrace::capture(&current),proof_context:worker.delivered_command_context_for_test().cloned()});}
        worker.observe_applied_command(command);phase_ends[5]=Instant::now();
        if scenario==Scenario::HoldSecondContinuity&&held_nonzero>=2&&released_at.is_some()
            &&!held.load(Ordering::Acquire) {
            let lock_started=Instant::now();let p=probe.lock().unwrap();probe_wait[2]=Some(lock_started.elapsed());
            if p.admitted>hold_admitted_count {continuity_complete=true;}
        }
        phase_ends[6]=Instant::now();
        let terminal=delivered_negative_reason.is_some()||continuity_complete||succeeded||retired_deadline.is_some_and(|d|Instant::now()>=d);
        let mut reset=false;
        if !terminal {
            deadline+=PERIOD;
            if deadline<Instant::now(){deadline=Instant::now();reset=true;} // no fake catch-up feedback
            if tick+1==max_ticks {timeout_wait=Some(worker.execution_wait_reason());}
        }
        phase_ends[7]=Instant::now();
        let sched_body_end=consumer_sched.sample();
        cadence.push(cadence_trace::Tick{tick,incoming_deadline,before_sleep,requested_sleep,start:now,gap:elapsed,
            ends:phase_ends,outgoing_deadline:(!terminal).then_some(deadline),reset,probe_wait,map_call,
            sched_before_sleep,sched_after_wake,sched_body_end});
        if terminal{break;}
    }
    let time_to_goal=started.elapsed();worker.stop();let _=release.try_send(());
    drop(worker); // control/planning samples settled before reading metrics
    let p=probe.lock().unwrap();
    if let Some(wait)=timeout_wait {eprintln!("TIMEOUT scenario={scenario:?} enabled={enabled} pose={current:?} wait={wait:?}");}
    let planning_metrics=if factory {"unavailable (unprobed factory)".to_owned()}else{format!("jobs_started={} submitted={} admitted={} control_pending={} control_negative={}",jobs.load(Ordering::Relaxed),p.submitted,p.admitted,p.pending,p.negative)};
    let distance_m: f64=segments.iter().map(|s|(s.to.x-s.from.x).hypot(s.to.y-s.from.y)).sum();
    let displacement_m=(current.pose.position.x-0.75).hypot(current.pose.position.y-1.5);
    let tick_gaps=&gaps[1..];
    let remaining_goal_m=current.pose.position.distance(goal.pose.position);
    eprintln!("MOVING scenario={scenario:?} enabled={enabled} source_cm={} factory={factory} startup={startup} priming_s={:.3} elapsed_s={:.3} succeeded={succeeded} continuity_complete={continuity_complete} remaining_goal_m={remaining_goal_m:.6} max_ticks={max_ticks} delivered_negative={delivered_negative} delivered_negative_reason={delivered_negative_reason:?} expected_current_rejection={expected_current_rejection} accepted={accepted} nonzero={nonzero_count} held_nonzero={held_nonzero} sensor_frames={frames} sensor_no_new_frame={sensor_no_new_frame} dropped_frames={} consumer_waits={consumer_waits:?} planning_metrics=[{planning_metrics}] final_pose_xyzyaw={:?} actual_displacement_m={displacement_m:.6} actual_travel_m={distance_m:.6} tick_gap_ms_median_p95_max={} tick_gaps_over_50ms={} tick_gaps_over_75ms={} consumer_ms_median_p95_max={} control_step_ms_median_p95_max={} map_update_ms_median_p95_max={}\nCPU BEFORE {cpu_before}\nCPU AFTER {}",
        resolution*100.0,priming_duration.as_secs_f64(),time_to_goal.as_secs_f64(),sensor.dropped.load(Ordering::Relaxed),
        [current.pose.position.x,current.pose.position.y,current.pose.position.z,current.pose.orientation.yaw()],
        measured_distribution(tick_gaps),tick_gaps.iter().filter(|ms|**ms>50.0).count(),tick_gaps.iter().filter(|ms|**ms>75.0).count(),
        measured_distribution(&consume_ms),measured_distribution(&p.control_ms),measured_distribution(&p.map_update_ms),cpu_snapshot());
    report_control_stages(&p,factory,scenario,enabled,resolution,startup);
    cadence.report(started,&receipts,&p,factory);
    // Report CPU raw /proc values; no false per-thread CPU attribution. Factory
    // runs deliberately have no inner probe: consumer latency is NOT control CPU.
    drop(p);drop(sensor);
    validate_motion_history(&receipts,&segments,resolution);
    assert!(delivered_negative_reason.is_none()||expected_current_rejection,"delivered_negative={delivered_negative}: {}",delivered_negative_reason.as_deref().unwrap_or("none"));
    let p=probe.lock().unwrap();
    if matches!(scenario,Scenario::Goal|Scenario::HoldFirst|Scenario::HoldSecond){assert!(succeeded,"full position plus final yaw required");assert!(nonzero_count>0);assert!(!segments.is_empty());assert!(current.pose.position.x>1.5,"actual accepted-command displacement required");assert!(current.pose.position.distance(goal.pose.position)<options.position_tolerance_m);assert!(nuwax_nav_astar::controllers::angle_diff(current.pose.orientation.yaw(),goal.pose.orientation.yaw()).abs()<options.yaw_tolerance_rad);}
    if matches!(scenario,Scenario::HoldSecond|Scenario::HoldSecondContinuity){assert!(held_nonzero>=2,"must continue fresh guarded motion on previously accepted path while real planning owner is held");}
    if scenario==Scenario::HoldSecondContinuity {
        assert!(continuity_complete,"held-planning continuity needs fresh held commands, 650ms release and a newer actual admission");
        assert!(released_at.is_some()&&p.admitted>hold_admitted_count);
        assert!(!segments.is_empty()&&distance_m>0.0,"actual motion required; this arm makes no full-goal claim");
    }
    if scenario==Scenario::HoldFirst{assert!(frames>=4&&p.admitted>0,"metadata churn must allow a current job to finish/admit");}
    if matches!(scenario,Scenario::ReplaceGoal|Scenario::ReplaceBoundary|Scenario::ReplaceGeometry|Scenario::Cancel){assert!(changed&&hold_start.is_some());assert_eq!(p.admitted,0,"old held candidate must not become admitted");}
    if scenario==Scenario::ReplacePath {assert!(changed);assert!(p.endpoints.iter().all(|end|(end[0]-0.75).abs()<0.15&&end[1]>1.9),"old eastbound endpoint survived northbound path replacement: {:?}",p.endpoints);}
    if !enabled&&!factory{assert_eq!(jobs.load(Ordering::Relaxed),0);assert_eq!(p.submitted,0);}
    drop(p);
    // This consumer-cadence test allows 25ms scheduling jitter. Passing its
    // 75ms threshold is NOT proof of a strict 50ms hard deadline; the log also
    // reports every observed >50ms gap. Inner control/map limits remain 50ms.
    assert!(gaps[1..].iter().all(|ms|*ms<=75.0),"wall consumer tick gaps missed 50ms cadence (+25ms test scheduler allowance)");
    let p=probe.lock().unwrap();
    if !factory {assert!(p.map_update_ms.iter().all(|ms|*ms<=50.0),"map installation exceeded control deadline");assert!(p.control_ms.iter().all(|ms|*ms<=50.0),"real control step exceeded 50ms; planning split does not fix residual guard cost");}
}

#[test] #[ignore="wall-paced real factory qualification, native owner runs explicitly"]
fn moving_factory_5cm_on_off_full_goal(){for enabled in [false,true]{run(enabled,0.05,Scenario::Goal,true);}}
#[test] #[ignore="wall-paced measured control qualification"]
fn moving_measured_5cm_on_off_full_goal(){for enabled in [false,true]{run(enabled,0.05,Scenario::Goal,false);}}
#[test] #[ignore="wall-paced metadata-churn qualification"]
fn moving_5cm_metadata_churn_finishes_current_job(){run(true,0.05,Scenario::HoldFirst,false);}
#[test] #[ignore="wall-paced committed-path continuity qualification"]
fn moving_5cm_committed_control_continues_held_planning(){run(true,0.05,Scenario::HoldSecond,false);}
#[test] #[ignore="wall-paced stale-result retirement matrix"]
fn moving_5cm_replacements_retire_old_planning(){for scenario in [Scenario::ReplaceGoal,Scenario::ReplacePath,Scenario::ReplaceBoundary,Scenario::ReplaceGeometry,Scenario::Cancel]{run(true,0.05,scenario,false);}}
#[test] #[ignore="1cm stress is not current 5cm FineCloud default"]
fn moving_1cm_stress_on_off_full_goal(){for enabled in [false,true]{run(enabled,0.01,Scenario::Goal,false);}}


#[test] #[ignore="run before timing; actual fresh then actually expired source evidence"]
fn moving_5cm_source_positive_and_expired_negative_probe() {
    let (_,base,keys)=fixture(false,0.05,6.0);
    let sensor=Sensor::new(base,keys);
    let first=sensor.receive.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(first.sequence,1);
    let second=sensor.receive.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(second.sequence,2);
    let from=body(PlanarPose{x:0.75,y:1.5,yaw:0.0});
    let to=body(PlanarPose{x:0.95,y:1.5,yaw:0.0});
    first.map.validate_body_motion(from,to,&[]).expect("one real sample already supplies geometric ground");
    second.map.validate_body_motion(from,to,&[]).expect("fresh source geometry must permit independent segment");
    let bound=CommandDispatchBound::from_source_timestamp(second.sample_ns,Duration::from_secs_f64(OBS_AGE));
    assert!(!bound.expired_now());
    assert!(source_healthy_at(second.sample_ns,unix_ns()));
    drop(sensor); // Stop the real producer; never rewrite its captured timestamp.
    std::thread::sleep(Duration::from_secs_f64(OBS_AGE+0.05));
    assert!(bound.expired_now(),"stopped sensor source must lose execution health");
    assert!(!source_healthy_at(second.sample_ns,unix_ns()));
    second.map.validate_body_motion(from,to,&[]).expect("old ground geometry itself has no TTL");
    let blocked=second.map.with_recent_obstacle_points(1,second.sample_ns,0,&[],[(0,[1.025,1.5,0.5])]).unwrap();
    assert!(blocked.validate_body_motion(from,to,&[]).is_err(),"old ground cannot hide a real occupied obstacle");
}


#[test] #[ignore="independent OFF measured run; failure cannot hide ON evidence"]
fn moving_measured_5cm_off_full_goal(){run(false,0.05,Scenario::Goal,false);}
#[test] #[ignore="independent ON measured run; failure cannot hide OFF evidence"]
fn moving_measured_5cm_on_full_goal(){run(true,0.05,Scenario::Goal,false);}

#[test] #[ignore="independent 1cm OFF stress; failure cannot hide ON evidence"]
fn moving_1cm_stress_off_full_goal(){run(false,0.01,Scenario::Goal,false);}
#[test] #[ignore="independent 1cm ON stress; failure cannot hide OFF evidence"]
fn moving_1cm_stress_on_full_goal(){run(true,0.01,Scenario::Goal,false);}


#[test] #[ignore="primed real factory OFF; two genuine frames before goal/path"]
fn moving_factory_5cm_primed_off_full_goal(){run_startup(false,0.05,Scenario::Goal,true,true);}
#[test] #[ignore="primed real factory ON; two genuine frames before goal/path"]
fn moving_factory_5cm_primed_on_full_goal(){run_startup(true,0.05,Scenario::Goal,true,true);}
#[test] #[ignore="primed measured OFF; two genuine frames before goal/path"]
fn moving_measured_5cm_primed_off_full_goal(){run_startup(false,0.05,Scenario::Goal,false,true);}
#[test] #[ignore="primed measured ON; two genuine frames before goal/path"]
fn moving_measured_5cm_primed_on_full_goal(){run_startup(true,0.05,Scenario::Goal,false,true);}


#[test] #[ignore="continuity only: fresh held commands then actual post-release admission; not full goal"]
fn moving_5cm_held_planning_continuity_only(){run(true,0.05,Scenario::HoldSecondContinuity,false);}
#[test] #[ignore="separate 120s full 7m goal arm; original 60s arm remains unchanged"]
fn moving_5cm_held_planning_7m_full_goal_120s(){run_with_budget(true,0.05,Scenario::HoldSecond,false,false,MAX_TICKS*2);}

#[test] #[ignore="primed continuity only; genuine samples before goal, no full-goal claim"]
fn moving_5cm_primed_held_planning_continuity_only(){run_startup(true,0.05,Scenario::HoldSecondContinuity,false,true);}
#[test] #[ignore="primed metadata-churn arm; two real samples before goal"]
fn moving_5cm_primed_metadata_churn_finishes_current_job(){run_startup(true,0.05,Scenario::HoldFirst,false,true);}
#[test] #[ignore="primed separate 120s full 7m goal; original cold 60s arm retained"]
fn moving_5cm_primed_held_planning_7m_full_goal_120s(){run_with_budget(true,0.05,Scenario::HoldSecond,false,true,MAX_TICKS*2);}
#[test] #[ignore="primed 1cm OFF stress; genuine samples before goal"]
fn moving_1cm_stress_primed_off_full_goal(){run_startup(false,0.01,Scenario::Goal,false,true);}
#[test] #[ignore="primed 1cm ON stress; genuine samples before goal"]
fn moving_1cm_stress_primed_on_full_goal(){run_startup(true,0.01,Scenario::Goal,false,true);}

// Selected-candidate microbenchmark, deliberately not a controller or movement
// qualification. Each call is an independent synthetic pose/velocity fixture.
// Metadata-cold means a new genuine observation object, not cold geometry roots.
// Warm and moved calls keep that exact source; their naturally increasing source
// age is reported and must never be refreshed away just to obtain a pass.
#[derive(Default)]
struct SelectedGuardSamples {
    c_ms:Vec<f64>,guard_ms:Vec<f64>,pose_age_ms:Vec<f64>,source_entry_age_ms:Vec<f64>,
    source_completion_age_ms:Vec<f64>,accepted:usize,rejected:usize,
}
fn benchmark_selected_guard(resolution:f32) {
    const SAMPLES:usize=12;
    let (config,base,keys)=fixture(false,resolution,6.0);
    let sensor=Sensor::new(base,keys);
    let priming_started=Instant::now();
    let first=sensor.receive.recv_timeout(Duration::from_secs(5)).expect("first real source acquisition");
    let second=sensor.receive.recv_timeout(Duration::from_secs(5)).expect("second real source acquisition");
    assert!(first.sample_ns<second.sample_ns&&second.sample_ns<=unix_ns());
    assert!(first.sequence>=1&&second.sequence>=2);
    assert!(nuwax_nav_pct::same_source_geometry(&first.map,&second.map));
    assert!(!first.map.same_body_query_view(&second.map));
    eprintln!("SELECTED_GUARD_PRIMING source_cm={} duration_s={:.6} first_sequence={} second_sequence={} first_sample_ns={} second_sample_ns={} actual_sample_spacing_ms={:.6}",
        resolution*100.0,priming_started.elapsed().as_secs_f64(),first.sequence,second.sequence,
        first.sample_ns,second.sample_ns,(second.sample_ns-first.sample_ns)as f64/1e6);
    let mut ready=Some(second);
    let mut prior_source=None::<Arc<SurfaceSnapshot>>;
    let mut prior_sample_ns=first.sample_ns;
    let mut all_rejections=0;
    let mut all_over_50ms=0;
    for (case,before_speed,candidate_speed) in [("steady_0.1",0.1_f64,0.1_f64),("steady_0.3",0.3,0.3),("ramp_0.05_to_0.1",0.05,0.1)] {
        let applied=Twist2{linear:nuwax_nav_types_messages::Point2::new(before_speed,0.0),angular_z:0.0};
        let candidate=Twist2{linear:nuwax_nav_types_messages::Point2::new(candidate_speed,0.0),angular_z:0.0};
        assert!(nonzero(applied)&&nonzero(candidate));
        assert!(candidate_speed<=limits().max_forward_mps);
        assert!((candidate_speed-before_speed).abs()<=limits().linear_acceleration_mps2*PERIOD.as_secs_f64()+1e-12);
        let mut follower=SurfaceFollower::new(config.clone()).unwrap();
        let mut samples:[SelectedGuardSamples;3]=std::array::from_fn(|_|SelectedGuardSamples::default());
        let mut sample_spacings_ms=Vec::with_capacity(SAMPLES);
        for sample in 0..SAMPLES {
            let frame=ready.take().unwrap_or_else(||sensor.receive.recv_timeout(Duration::from_secs(5)).expect("next bounded real source frame"));
            assert!(frame.sequence>=2&&frame.sample_ns>prior_sample_ns&&frame.sample_ns<=unix_ns());
            sample_spacings_ms.push((frame.sample_ns-prior_sample_ns)as f64/1e6);
            if let Some(old)=&prior_source {
                assert!(nuwax_nav_pct::same_source_geometry(old,&frame.map));
                assert!(!old.same_body_query_view(&frame.map));
            }
            prior_sample_ns=frame.sample_ns;prior_source=Some(frame.map.clone());
            // Each batch changes its baseline slightly too. These are explicitly
            // separate geometric fixtures; they are not claimed applied motion.
            let baseline_x=0.75+sample as f64*0.001;
            for (phase,label) in ["metadata_cold","warm_same_pose","moved_pose_5mm"].into_iter().enumerate() {
                let capture=Instant::now();
                let captured=pose(baseline_x+if phase==2{0.005}else{0.0},1.5,0.0);
                let pose_stamp_ns=captured.stamp.nanos as u64;
                if phase==0 {follower.update_execution_map(ExecutionMap::Surface(frame.map.clone())).unwrap();}
                follower.update_odometry(captured.clone());
                follower.observe_applied_command(applied);
                let guard_entry_ns=unix_ns();
                assert!(frame.sample_ns<=guard_entry_ns&&pose_stamp_ns<=guard_entry_ns);
                let guard_start=Instant::now();
                let accepted=follower.check_selected_candidate_for_test(candidate);
                // The method has returned: shared production checker setup,
                // swept stopping checks and its local cleanup are all included.
                let guard_elapsed=guard_start.elapsed();
                let completion_ns=unix_ns();
                let c=capture.elapsed();
                let record=&mut samples[phase];
                record.c_ms.push(c.as_secs_f64()*1e3);
                record.guard_ms.push(guard_elapsed.as_secs_f64()*1e3);
                record.pose_age_ms.push(completion_ns.saturating_sub(pose_stamp_ns)as f64/1e6);
                record.source_entry_age_ms.push(guard_entry_ns.saturating_sub(frame.sample_ns)as f64/1e6);
                record.source_completion_age_ms.push(completion_ns.saturating_sub(frame.sample_ns)as f64/1e6);
                if accepted {record.accepted+=1;}else {
                    record.rejected+=1;
                    // Do not replace failed nonzero candidates with a zero or
                    // retry against fresher metadata. The original false remains.
                    eprintln!("SELECTED_GUARD_REJECT source_cm={} case={case} phase={label} sample={sample} source_sequence={} source_sample_ns={} pose_stamp_ns={pose_stamp_ns} guard_entry_ns={guard_entry_ns} completion_ns={completion_ns} pose={:?} applied={applied:?} candidate={candidate:?} C_ms={:.6} guard_ms={:.6} source_age_entry_ms={:.6} source_age_completion_ms={:.6} detailed_guard_reason=unavailable_from_bool_api",
                        resolution*100.0,frame.sequence,frame.sample_ns,captured.pose,c.as_secs_f64()*1e3,guard_elapsed.as_secs_f64()*1e3,
                        guard_entry_ns.saturating_sub(frame.sample_ns)as f64/1e6,completion_ns.saturating_sub(frame.sample_ns)as f64/1e6);
                }
            }
        }
        for (phase,label) in ["metadata_cold","warm_same_pose","moved_pose_5mm"].into_iter().enumerate() {
            let result=&samples[phase];
            let over_50ms=result.c_ms.iter().filter(|ms|**ms>50.0).count();
            all_over_50ms+=over_50ms;all_rejections+=result.rejected;
            eprintln!("SELECTED_GUARD_COST source_cm={} case={case} phase={label} samples={} accepted_nonzero={} rejected_nonzero={} C_ms_median_p95_max={} guard_ms_median_p95_max={} captured_pose_age_completion_ms_median_p95_max={} source_age_entry_ms_median_p95_max={} source_age_completion_ms_median_p95_max={} actual_source_spacing_ms_median_p95_max={} C_over_50ms={over_50ms} control_period_ms=50 observation_age_limit_ms=1000",
                resolution*100.0,result.c_ms.len(),result.accepted,result.rejected,measured_distribution(&result.c_ms),
                measured_distribution(&result.guard_ms),measured_distribution(&result.pose_age_ms),
                measured_distribution(&result.source_entry_age_ms),measured_distribution(&result.source_completion_age_ms),measured_distribution(&sample_spacings_ms));
        }
    }
    eprintln!("SELECTED_GUARD_TOTAL source_cm={} rejected_nonzero={all_rejections} C_over_50ms={all_over_50ms} producer_dropped_frames={}",resolution*100.0,sensor.dropped.load(Ordering::Relaxed));
    drop(sensor);
    assert_eq!(all_rejections,0,"original selected-candidate safety failures; see per-sample diagnostics, no zero substituted");
    assert_eq!(all_over_50ms,0,"selected-candidate capture-to-completion cost exceeds 50ms; no deadline qualification");
}

#[test] #[ignore="run 5cm first: exact shared selected-candidate guard cost, no route/ranking"]
fn selected_candidate_guard_cost_5cm(){benchmark_selected_guard(0.05);}
#[test] #[ignore="separate optional 1cm stress; truthful source ages and rejections retained"]
fn selected_candidate_guard_cost_1cm_stress(){benchmark_selected_guard(0.01);}


// Separate from the historical H50 straight-only benchmark above. A larger H
// does not widen the 50ms control period, observation TTL or real-time criterion.
// H minus (capture cost + one consumer phase) is reported without pretending
// unknown dispatch/actuator delay is zero or that positive headroom is a lease.
fn benchmark_selected_guard_prefix(periods:usize) {
    assert!(matches!(periods,2|3));
    let resolution=0.05_f32;
    let horizon_ms=periods as f64*PERIOD.as_secs_f64()*1e3;
    eprintln!("SELECTED_PREFIX_GUARD_SCOPE periods={periods} H_ms={horizon_ms:.0} samples_per_cell=12 empirical_p95_equals_max=true pose_capture=near_call_fixture_only real_capture_transport_bound=unestablished headroom_formula=H-(C+50ms) dispatch_budget=unknown sample_age_budget=not_in_headroom positive_headroom_is_not_qualification=true");
    const SAMPLES:usize=12;
    let (config,base,keys)=fixture(false,resolution,6.0);
    let sensor=Sensor::new(base,keys);
    let priming_started=Instant::now();
    let first=sensor.receive.recv_timeout(Duration::from_secs(5)).expect("first real source acquisition");
    let second=sensor.receive.recv_timeout(Duration::from_secs(5)).expect("second real source acquisition");
    assert!(first.sample_ns<second.sample_ns&&second.sample_ns<=unix_ns());
    assert!(first.sequence>=1&&second.sequence>=2);
    assert!(nuwax_nav_pct::same_source_geometry(&first.map,&second.map));
    assert!(!first.map.same_body_query_view(&second.map));
    eprintln!("SELECTED_PREFIX_GUARD_PRIMING periods={periods} H_ms={horizon_ms:.0} source_cm={} duration_s={:.6} first_sequence={} second_sequence={} first_sample_ns={} second_sample_ns={} actual_sample_spacing_ms={:.6}",
        resolution*100.0,priming_started.elapsed().as_secs_f64(),first.sequence,second.sequence,
        first.sample_ns,second.sample_ns,(second.sample_ns-first.sample_ns)as f64/1e6);
    let mut ready=Some(second);
    let mut prior_source=None::<Arc<SurfaceSnapshot>>;
    let mut prior_sample_ns=first.sample_ns;
    let mut all_rejections=0;
    let mut all_over_50ms=0;
    let mut all_nonpositive_headroom=0;
    let twist=|linear:f64,angular:f64|Twist2{linear:nuwax_nav_types_messages::Point2::new(linear,0.0),angular_z:angular};
    for (case,applied,candidate) in [
        ("steady_0.1",twist(0.1,0.0),twist(0.1,0.0)),
        ("steady_0.3",twist(0.3,0.0),twist(0.3,0.0)),
        ("ramp_0.05_to_0.1",twist(0.05,0.0),twist(0.1,0.0)),
        ("from_rest_0.05",Twist2::ZERO,twist(0.05,0.0)),
        ("steady_rotation_0.2",twist(0.0,0.2),twist(0.0,0.2)),
        ("steady_curve_0.2_plus_0.2",twist(0.2,0.2),twist(0.2,0.2)),
    ] {
        assert!(nonzero(candidate),"zero cannot qualify a positive selected candidate");
        assert!(candidate.linear.x>=0.0&&candidate.linear.x<=limits().max_forward_mps);
        assert!(candidate.angular_z.abs()<=limits().max_angular_rps);
        assert!((candidate.linear.x-applied.linear.x).abs()<=limits().linear_acceleration_mps2*PERIOD.as_secs_f64()+1e-12);
        assert!((candidate.angular_z-applied.angular_z).abs()<=limits().angular_acceleration_rps2*PERIOD.as_secs_f64()+1e-12);
        let mut follower=SurfaceFollower::new(config.clone()).unwrap();
        let mut samples:[SelectedGuardSamples;3]=std::array::from_fn(|_|SelectedGuardSamples::default());
        let mut sample_spacings_ms=Vec::with_capacity(SAMPLES);
        for sample in 0..SAMPLES {
            let frame=ready.take().unwrap_or_else(||sensor.receive.recv_timeout(Duration::from_secs(5)).expect("next bounded real source frame"));
            assert!(frame.sequence>=2&&frame.sample_ns>prior_sample_ns&&frame.sample_ns<=unix_ns());
            sample_spacings_ms.push((frame.sample_ns-prior_sample_ns)as f64/1e6);
            if let Some(old)=&prior_source {
                assert!(nuwax_nav_pct::same_source_geometry(old,&frame.map));
                assert!(!old.same_body_query_view(&frame.map));
            }
            prior_sample_ns=frame.sample_ns;prior_source=Some(frame.map.clone());
            // Each batch changes its baseline slightly too. These are explicitly
            // separate geometric fixtures; they are not claimed applied motion.
            let baseline_x=0.75+sample as f64*0.001;
            for (phase,label) in ["metadata_cold","warm_same_pose","moved_pose_5mm"].into_iter().enumerate() {
                let capture=Instant::now();
                let captured=pose(baseline_x+if phase==2{0.005}else{0.0},1.5,0.0);
                let pose_stamp_ns=captured.stamp.nanos as u64;
                if phase==0 {follower.update_execution_map(ExecutionMap::Surface(frame.map.clone())).unwrap();}
                follower.update_odometry(captured.clone());
                follower.observe_applied_command(applied);
                let guard_entry_ns=unix_ns();
                assert!(frame.sample_ns<=guard_entry_ns&&pose_stamp_ns<=guard_entry_ns);
                let guard_start=Instant::now();
                let accepted=follower.check_selected_candidate_periods_for_test(candidate,periods);
                // The method has returned: shared production checker setup,
                // swept stopping checks and its local cleanup are all included.
                let guard_elapsed=guard_start.elapsed();
                let completion_ns=unix_ns();
                let c=capture.elapsed();
                let record=&mut samples[phase];
                record.c_ms.push(c.as_secs_f64()*1e3);
                record.guard_ms.push(guard_elapsed.as_secs_f64()*1e3);
                record.pose_age_ms.push(completion_ns.saturating_sub(pose_stamp_ns)as f64/1e6);
                record.source_entry_age_ms.push(guard_entry_ns.saturating_sub(frame.sample_ns)as f64/1e6);
                record.source_completion_age_ms.push(completion_ns.saturating_sub(frame.sample_ns)as f64/1e6);
                if accepted {record.accepted+=1;}else {
                    record.rejected+=1;
                    // Do not replace failed nonzero candidates with a zero or
                    // retry against fresher metadata. The original false remains.
                    eprintln!("SELECTED_PREFIX_GUARD_REJECT periods={periods} H_ms={horizon_ms:.0} source_cm={} case={case} phase={label} sample={sample} source_sequence={} source_sample_ns={} pose_stamp_ns={pose_stamp_ns} guard_entry_ns={guard_entry_ns} completion_ns={completion_ns} pose={:?} applied={applied:?} candidate={candidate:?} C_ms={:.6} guard_ms={:.6} source_age_entry_ms={:.6} source_age_completion_ms={:.6} detailed_guard_reason=unavailable_from_bool_api",
                        resolution*100.0,frame.sequence,frame.sample_ns,captured.pose,c.as_secs_f64()*1e3,guard_elapsed.as_secs_f64()*1e3,
                        guard_entry_ns.saturating_sub(frame.sample_ns)as f64/1e6,completion_ns.saturating_sub(frame.sample_ns)as f64/1e6);
                }
            }
        }
        for (phase,label) in ["metadata_cold","warm_same_pose","moved_pose_5mm"].into_iter().enumerate() {
            let result=&samples[phase];
            let over_50ms=result.c_ms.iter().filter(|ms|**ms>50.0).count();
            all_over_50ms+=over_50ms;all_rejections+=result.rejected;
            let headroom:Vec<_>=result.c_ms.iter().map(|c|horizon_ms-(c+50.0)).collect();
            let nonpositive_headroom=headroom.iter().filter(|ms|**ms<=0.0).count();
            let minimum_headroom=headroom.iter().copied().fold(f64::INFINITY,f64::min);
            all_nonpositive_headroom+=nonpositive_headroom;
            eprintln!("SELECTED_PREFIX_GUARD_COST periods={periods} H_ms={horizon_ms:.0} source_cm={} case={case} phase={label} samples={} accepted_nonzero={} rejected_nonzero={} C_ms_median_p95_max={} guard_ms_median_p95_max={} captured_pose_age_completion_ms_median_p95_max={} source_age_entry_ms_median_p95_max={} source_age_completion_ms_median_p95_max={} actual_source_spacing_ms_median_p95_max={} C_over_50ms={over_50ms} H_minus_C_plus_50ms_headroom_median_p95_max={} minimum_headroom_ms={minimum_headroom:.6} nonpositive_headroom={nonpositive_headroom} dispatch_delay=unknown sample_age_budget=not_in_headroom headroom_is_not_delivery_permission=true control_period_ms=50 observation_age_limit_ms=1000",
                resolution*100.0,result.c_ms.len(),result.accepted,result.rejected,measured_distribution(&result.c_ms),
                measured_distribution(&result.guard_ms),measured_distribution(&result.pose_age_ms),
                measured_distribution(&result.source_entry_age_ms),measured_distribution(&result.source_completion_age_ms),measured_distribution(&sample_spacings_ms),measured_distribution(&headroom));
        }
    }
    eprintln!("SELECTED_PREFIX_GUARD_TOTAL periods={periods} H_ms={horizon_ms:.0} source_cm={} rejected_nonzero={all_rejections} C_over_50ms={all_over_50ms} nonpositive_H_minus_C_plus_50ms_headroom={all_nonpositive_headroom} dispatch_delay=unknown sample_age_budget=not_in_headroom producer_dropped_frames={}",resolution*100.0,sensor.dropped.load(Ordering::Relaxed));
    drop(sensor);
    assert_eq!(all_rejections,0,"original selected-candidate safety failures; see per-sample diagnostics, no zero substituted");
    assert_eq!(all_over_50ms,0,"selected-candidate capture-to-completion cost exceeds strict 50ms; larger H does not qualify 20Hz");
    assert_eq!(all_nonpositive_headroom,0,"no positive headroom after capture cost plus 50ms consumer phase; dispatch still unknown");
}

#[test] #[ignore="separate H100 genuine two-prefix check at fixed50ms; 5cm six motion cases"]
fn selected_candidate_guard_cost_5cm_h100(){benchmark_selected_guard_prefix(2);}
#[test] #[ignore="separate H150 genuine three-prefix check at fixed50ms; 5cm six motion cases"]
fn selected_candidate_guard_cost_5cm_h150(){benchmark_selected_guard_prefix(3);}

#[test] #[ignore="experimental advice, primed actual factory OFF"]
fn moving_advice_factory_5cm_primed_off_full_goal(){run_with_budget(false,0.05,Scenario::Goal,true,true,MAX_TICKS);}
#[test] #[ignore="experimental advice, primed actual factory ON"]
fn moving_advice_factory_5cm_primed_on_full_goal(){run_with_budget(true,0.05,Scenario::Goal,true,true,MAX_TICKS);}
#[test] #[ignore="experimental advice, primed current control during held planning"]
fn moving_advice_5cm_primed_held_planning_continuity(){run_with_budget(true,0.05,Scenario::HoldSecondContinuity,false,true,MAX_TICKS);}

#[tokio::test]
async fn dispatch_real_observed_backends_preserve_bound_to_scheduler() {
    // A stationary-state composition fixture with truthful two-frame priming.
    // This checks metadata ownership, not closed-loop goal completion.
    for variant in 0..4 {
        let (config,base,keys)=fixture(variant>=2,0.05,3.0);
        let sensor=Sensor::new(base,keys);
        let first=sensor.receive.recv_timeout(Duration::from_secs(2)).unwrap();
        let frame=sensor.receive.recv_timeout(Duration::from_secs(2)).unwrap();
        assert!(frame.sequence>=2 && frame.sample_ns>first.sample_ns);
        let mut source_timestamp_ns=frame.sample_ns;
        let mut worker=match variant {
            0=>planner_worker::PlannerWorker::new(SurfaceFollower::new(config).unwrap()).unwrap(),
            1|2=>production_surface_backend(config).unwrap(),
            _=>planner_worker::PlannerWorker::new(Measured::new(RouteFollower::new(config).unwrap(),
                Arc::new(Mutex::new(Probe::with_control_sample_budget(20))),Arc::new(AtomicUsize::new(0)),
                Arc::new(AtomicBool::new(false)),Arc::new(AtomicUsize::new(0)))).unwrap(),
        };
        let start=pose(0.75,1.5,0.0);let goal=pose(1.75,1.5,0.0);
        worker.configure_goal(&goal,GoalOptions::default()).unwrap();
        worker.update_execution_map(ExecutionMap::Surface(frame.map)).unwrap();
        worker.update_odometry(start.clone());
        worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![start,goal.clone()]}),None,goal).unwrap();
        worker.observe_applied_command(Twist2{linear:nuwax_nav_types_messages::Point2::new(0.3,0.0),angular_z:0.0});
        let deadline=Instant::now()+Duration::from_secs(3);
        let step=loop {
            if let Ok(frame)=sensor.receive.try_recv() {
                source_timestamp_ns=frame.sample_ns;
                worker.update_execution_map(ExecutionMap::Surface(frame.map)).unwrap();
            }
            match worker.step(WIDTH) {
                step@PlannerStep::Command(command) if command!=Twist2::ZERO=>break step,
                PlannerStep::Pending(_)|PlannerStep::Command(_)=>{},
                other=>panic!("real backend {variant} rejected metadata fixture: {other:?}"),
            }
            assert!(Instant::now()<deadline,"real backend {variant} did not produce a finite observed command");
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        crate::tests::assert_real_source_bound_reaches_scheduler(
            worker,step,source_timestamp_ns,Duration::from_secs_f64(OBS_AGE)).await;
    }
}

#[test]
fn prepared_fixture_layout_preserves_exact_evidence_and_rejects_changed_membership_owner() {
    let scenes=[vec![],vec![[0,0,0]],vec![[-33,0,-1],[-32,-1,32],[31,32,0],[32,31,0],[31,32,0]],
        (0..120).rev().map(|i|[i-60,(i%17)-8,(i%3)-1]).collect()];
    for keys in scenes {
        let keys=Arc::new(keys);let layout=PreparedEvidenceLayout::new(keys.clone());
        for (sample,hits) in [(0,0),(1,1),(2,2),(u64::MAX,u16::MAX)] {
            assert_eq!(layout.materialize(&keys,sample,hits).unwrap(),whole_plane_evidence(&keys,sample,hits));
        }
        let identical_new_owner=Arc::new(keys.as_ref().clone());
        assert!(layout.materialize(&identical_new_owner,3,3).is_none());
        let mut changed=keys.clone();let values=Arc::make_mut(&mut changed);
        if !values.is_empty(){values.remove(0);}values.push([65,-65,33]);
        assert!(layout.materialize(&changed,3,3).is_none(),"old coverage cannot survive a different key owner");
        let rebuilt=PreparedEvidenceLayout::new(changed.clone());
        assert_eq!(rebuilt.materialize(&changed,3,3).unwrap(),whole_plane_evidence(&changed,3,3));
        assert_eq!(layout.materialize(&keys,4,4).unwrap(),whole_plane_evidence(&keys,4,4),"old immutable owner remains exact");
    }
}
#[test]
#[ignore="separate fixed-coverage prepared-source experiment; original full-plane arm retained"]
fn moving_prepared_source_1cm_primed_off_full_goal(){run_with_source_layout(false,0.01,Scenario::Goal,false,true,MAX_TICKS,true);}
#[test]
#[ignore="separate fixed-coverage prepared-source experiment; original full-plane arm retained"]
fn moving_prepared_source_1cm_primed_on_full_goal(){run_with_source_layout(true,0.01,Scenario::Goal,false,true,MAX_TICKS,true);}

// Shared unchanged actual-edge and complete delivered braking-tail oracle.
fn validate_motion_history(receipts:&[RawReceipt],segments:&[RawSegment],resolution:f32) {
    report_proof_receipts(receipts);
    // Run expensive independent oracles after wall-timed control. Retained immutable
    // snapshots preserve geometry and the actual source/receipt timestamps.
    for (i,s) in segments.iter().enumerate(){
        s.map.validate_body_motion(body(s.from),body(s.to),&s.boundary).unwrap_or_else(|e|panic!("segment {i} body: {e:?}"));
    }
    for (i,r) in receipts.iter().enumerate(){
        assert!(source_healthy_at(r.source_sample_ns,r.ns),"receipt {i} has stale or invalid sensor acquisition");
        // Retain the independent flat-body and future-view geometric oracle.
        // Production projection alternatives remain diagnostic only.
        let first_failure=std::cell::RefCell::new(None::<String>);
        let make_check=|kind:&'static str| {
            let first_failure=&first_failure;
            move |a:PlanarPose,b:PlanarPose,elapsed:f64| {
                let checked_ns=r.ns+(elapsed*1e9)as u64;
                let result=r.map.validate_body_motion(body(a),body(b),&r.boundary)
                    .map_err(|e|format!("source geometry sweep: {e:?}"))
                    .and_then(|_|r.map.body_query_with_current_support(body(a).0,body(a).1)
                        .and_then(|view|view.validate_body_motion(body(a),body(b),&r.boundary))
                        .map_err(|e|format!("future-footprint geometry sweep: {e:?}")));
                if let Err(reason)=result {
                    if first_failure.borrow().is_none() {
                        // Alternatives are printed only, never used as permission.
                        let source_projection=r.map.body_pose_after_motion(body(a),[b.x,b.y],b.yaw);
                        let view_projection=r.map.body_query_with_current_support(body(a).0,body(a).1)
                            .and_then(|view|view.body_pose_after_motion(body(a),[b.x,b.y],b.yaw));
                        *first_failure.borrow_mut()=Some(format!("callback={kind} elapsed_s={elapsed:.9} from={:?} to={:?} checked_ns={checked_ns} source_age_at_check_ms={:.6} reason={reason} source_projection={source_projection:?} future_view_projection={view_projection:?}",
                            body(a),body(b),checked_ns.saturating_sub(r.source_sample_ns)as f64/1e6));
                    }
                    return false;
                }
                true
            }
        };
        let choice=nuwax_nav_control::check_stopping_trajectory_with_period_check(r.pose,r.before,r.command,&limits(),PERIOD.as_secs_f64(),f64::from(resolution)*0.5,r.map.body_envelope().unwrap().radius(),make_check("geometric-subdivision"),make_check("physical-period"));
        if choice.is_none() {
            let current_body_result=r.map.validate_body_pose(body(r.pose).0,body(r.pose).1);
            let current_view_same=r.map.with_current_body_support(body(r.pose).0,body(r.pose).1).map(|view|view.same_body_query_view(&r.map));
            if let Some(context)=&r.proof_context {
                let guard=&context.proof.guard;
                let same_pose=guard.pose.bits==r.receipt_pose.bits && guard.pose.frame_id==r.receipt_pose.frame_id;
                let same_applied=guard.applied_bits==crate::command_proof_trace::command_bits(r.before);
                let historical=guard.observation_anchor_ns.map(|anchor|receipt_oracle_at(r,resolution,anchor));
                eprintln!("PROOF_TIME_REPLAY receipt={i} result_id={} guard_pose_equals_receipt={same_pose} guard_applied_equals_receipt={same_applied} guard_anchor_receipt_basis_pass={historical:?} original_receipt_basis_pass=false original_ttl_ns={:?} fixture_ttl_ns={} historical_anchor_is_captured_not_fabricated=true",
                    context.proof.input.result_id,guard.observation_max_age_ns,(OBS_AGE*1e9)as u64);
            }

            eprintln!("BRAKING_ORACLE_FAILURE receipt={i} source_cm={} receipt_ns={} pose_stamp_ns={} source_sample_ns={} source_sequence={} source_age_at_receipt_ms={:.6} worker_result_age_ms={:.6} worker_remaining_ms={:.6} selected_proof_context={:?} receipt_pose_raw={:?} pose={:?} before={:?} command={:?} current_body={current_body_result:?} current_view_same={current_view_same:?} first_failure={:?}",
                resolution*100.0,r.ns,r.pose_stamp_ns,r.source_sample_ns,r.source_sequence,r.ns.saturating_sub(r.source_sample_ns)as f64/1e6,r.worker_result_age_ms,r.worker_remaining_ms,r.proof_context,r.receipt_pose,r.pose,r.before,r.command,first_failure.borrow());
        }
        assert!(choice.is_some(),"receipt {i} independent full braking oracle rejected: {:?}",first_failure.into_inner());
    }
}

#[path="route_sustained_worker_tests.rs"]
mod sustained_worker;
// Independent synthetic selected-command fixtures; no applied-motion claim.
#[cfg(feature="body-query-profile")]
#[test]
#[ignore="serial source-owner and exact-pose allocation attribution"]
fn selected_guard_contact_domain_profile_5cm_10cm() {
    for resolution in [0.05_f32,0.1_f32] {
        let (config,base,keys)=fixture(false,resolution,6.0);
        let sensor=Sensor::new(base,keys);
        let first=sensor.receive.recv_timeout(Duration::from_secs(5)).unwrap();
        let mut previous=first.map;let mut previous_stamp=first.sample_ns;
        let mut follower=SurfaceFollower::new(config).unwrap();
        for (case,before,candidate) in [("steady_01",0.1,0.1),("steady_03",0.3,0.3),("ramp_005_01",0.05,0.1)] {
            for sample in 0..12 {
                let frame=sensor.receive.recv_timeout(Duration::from_secs(5)).unwrap();
                assert!(frame.sample_ns>previous_stamp);
                assert!(nuwax_nav_pct::same_source_geometry(&previous,&frame.map));
                assert!(!previous.same_body_query_view(&frame.map));
                previous=frame.map.clone();previous_stamp=frame.sample_ns;
                for (phase,dx,yaw) in [("new_source",0.0,0.0),("warm_same_pose",0.0,0.0),("moved_xy_yaw",0.005,0.01)] {
                    if phase=="new_source" {follower.update_execution_map(ExecutionMap::Surface(frame.map.clone())).unwrap();}
                    follower.update_odometry(pose(0.75+sample as f64*0.001+dx,1.5,yaw));
                    follower.observe_applied_command(Twist2{linear:nuwax_nav_types_messages::Point2::new(before,0.0),angular_z:0.0});
                    let entry=unix_ns();assert!(entry>=frame.sample_ns);
                    body_query_profile::begin();
                    let start=Instant::now();
                    let accepted=follower.check_selected_candidate_for_test(Twist2{linear:nuwax_nav_types_messages::Point2::new(candidate,0.0),angular_z:0.0});
                    let elapsed=start.elapsed();
                    let record=body_query_profile::end();
                    let end=unix_ns();
                    eprintln!("DOMAIN_SAMPLE source_cm={} case={case} sample={sample} phase={phase} accepted={accepted} guard_ns={} source_entry_age_ns={} source_completion_age_ns={} dense_calls={} plane_hits={} plane_misses={} allocated_slots={} visited_slots={} table_payload_bytes={}",
                        resolution*100.0,elapsed.as_nanos(),entry-frame.sample_ns,end-frame.sample_ns,
                        record.counters[BodyCounter::DenseDomainCalls as usize],record.counters[BodyCounter::PlaneCacheHits as usize],
                        record.counters[BodyCounter::PlaneCacheMisses as usize],record.counters[BodyCounter::DenseAllocatedSlots as usize],
                        record.counters[BodyCounter::DenseVisitedTableSlots as usize],dense_initial_table_payload_bytes(&record));
                    for stage in BodyStage::ALL {
                        eprintln!("DOMAIN_STAGE source_cm={} case={case} sample={sample} phase={phase} stage={} calls={} ns={}",
                            resolution*100.0,stage.label(),record.calls[stage as usize],record.wall_nanos[stage as usize]);
                    }
                    assert!(accepted,"no retry or zero substitution");
                    assert!(end-frame.sample_ns<=1_000_000_000,"actual source health age exceeded");
                }
            }
        }
        eprintln!("DOMAIN_SOURCE source_cm={} dropped={}",resolution*100.0,sensor.dropped.load(Ordering::Relaxed));
        drop(sensor);
    }
}

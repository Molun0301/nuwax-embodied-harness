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
use crate::control_profile::{self, Counter, Record as StageRecord, Stage};
use nuwax_nav_control::{integrate, PlanarPose};
use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
use nuwax_nav_types_messages::{GoalOptions, NavFrameId, MotionLimits, MotionModel, Point3, Pose3, Quaternion, Timestamp};
use std::collections::BTreeMap;
use std::sync::{atomic::{AtomicBool, AtomicUsize, Ordering}, mpsc, Mutex, Weak};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const PERIOD: Duration = Duration::from_millis(50);
const SENSOR_PERIOD: Duration = Duration::from_millis(200);
const WIDTH: f64 = 0.3;
const OBS_AGE: f64 = 1.0; // explicit synthetic terrain capability, never extended
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
fn fixture(enabled:bool, resolution:f32, length:f32)->(SurfaceFollowerConfig,Arc<SurfaceSnapshot>,Arc<Vec<[i32;3]>>) {
    let envelope=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],
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

struct Frame {map:Arc<SurfaceSnapshot>,sample_ns:u64,sequence:u16}
struct Sensor {receive:mpsc::Receiver<Frame>,stop:Arc<AtomicBool>,join:Option<std::thread::JoinHandle<()>>,dropped:Arc<AtomicUsize>}
impl Sensor {
    fn new(base:Arc<SurfaceSnapshot>,keys:Arc<Vec<[i32;3]>>)->Self {
        let (send,receive)=mpsc::sync_channel(1); // exactly one producer / one queued frame
        let stop=Arc::new(AtomicBool::new(false));let done=stop.clone();
        let dropped=Arc::new(AtomicUsize::new(0));let skipped=dropped.clone();
        let join=std::thread::spawn(move|| {
            let mut observations=SurfaceObservations::new(base.source_voxel_size_m(),32).unwrap();
            let mut sequence=0u16;
            let mut deadline=Instant::now();
            while !done.load(Ordering::Acquire) {
                if let Some(delay)=deadline.checked_duration_since(Instant::now()){std::thread::sleep(delay);}
                if done.load(Ordering::Acquire){break;}
                // One actual synthetic sensor sample now, before assembly. No
                // pre-dated frames; hits count distinct wall-spaced acquisitions.
                let sample_ns=unix_ns();sequence=sequence.saturating_add(1);
                let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
                for &key in keys.iter(){blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:sample_ns,hits:sequence});}
                for (key,block) in blocks {observations.replace_block(key,block).unwrap();}
                observations.set_source_timestamp_ns(sample_ns);observations.mark_complete_geometry_source();
                // Clone the previous persistent observation roots before replacing
                // blocks: same occupied columns retain their actual identity.
                let frame=Frame{map:Arc::new(base.with_observations(Some(Arc::new(observations.clone())))),sample_ns,sequence};
                match send.try_send(frame){Ok(())=>{},Err(mpsc::TrySendError::Full(_))=>{skipped.fetch_add(1,Ordering::Relaxed);},Err(mpsc::TrySendError::Disconnected(_))=>break}
                deadline+=SENSOR_PERIOD;
                if deadline<Instant::now(){deadline=Instant::now()+SENSOR_PERIOD;} // missed samples are dropped, never backfilled
            }
        });
        Self{receive,stop,join:Some(join),dropped}
    }
}
impl Drop for Sensor {fn drop(&mut self){self.stop.store(true,Ordering::Release);if let Some(t)=self.join.take(){t.join().unwrap();}}}

struct ControlStepSample {
    step_seq: u64,
    control_ms: f64,
    stages: StageRecord,
    source_view_seq: u64,
    source_present: bool,
    first_after_view_change: bool,
    pose_stamp_ns: Option<i64>,
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
    negative:usize,pending:usize,endpoints:Vec<[f64;3]>,
    control_samples:Vec<ControlStepSample>,control_sample_limit:usize,control_sample_overflow:usize,
}
impl Probe {
    fn with_control_sample_budget(max_ticks: usize) -> Self {
        let limit = max_ticks.min(MAX_TICKS * 2) + PROFILE_DRAIN_ALLOWANCE;
        Self {
            control_ms: Vec::with_capacity(MAX_TICKS * 2),
            control_samples: Vec::with_capacity(limit),
            control_sample_limit: limit,
            ..Self::default()
        }
    }
}

struct Measured {
    inner:RouteFollower,probe:Arc<Mutex<Probe>>,
    source:Option<Weak<SurfaceSnapshot>>,source_view_seq:u64,first_after_view_change:bool,
    pose_stamp_ns:Option<i64>,step_seq:u64,
    jobs:Arc<AtomicUsize>,held:Arc<AtomicBool>,held_phase:Arc<AtomicUsize>,
}
impl Measured {
    fn new(inner:RouteFollower,probe:Arc<Mutex<Probe>>,jobs:Arc<AtomicUsize>,held:Arc<AtomicBool>,held_phase:Arc<AtomicUsize>)->Self {
        Self {inner,probe,source:None,source_view_seq:0,first_after_view_change:false,
            pose_stamp_ns:None,step_seq:0,jobs,held,held_phase}
    }
    fn clear_source_marker(&mut self) {
        if self.source.take().is_some() {
            self.source_view_seq=self.source_view_seq.saturating_add(1);
            self.first_after_view_change=true;
        }
    }
}
impl NavPlanner for Measured {
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
        let (source,changed)=match &m {
            ExecutionMap::Surface(next)=>{
                let changed=self.source.as_ref().and_then(Weak::upgrade)
                    .is_none_or(|old|!old.same_body_query_view(next));
                (Some(Arc::downgrade(next)),changed)
            },
            ExecutionMap::Planar(_)=>(None,false),
        };
        let t=Instant::now();let result=self.inner.update_execution_map(m);let ms=t.elapsed().as_secs_f64()*1e3;
        if result.is_ok() {
            if changed {self.source_view_seq=self.source_view_seq.saturating_add(1);self.first_after_view_change=true;}
            self.source=source;
        }
        self.probe.lock().unwrap().map_update_ms.push(ms);result
    }
    fn invalidate_execution_map(&mut self){self.inner.invalidate_execution_map();self.clear_source_marker();}
    fn observe_applied_command(&mut self,c:Twist2){self.inner.observe_applied_command(c)}
    fn execution_wait_reason(&self)->Option<String>{self.inner.execution_wait_reason()}
    fn local_path_for_display(&self)->Vec<[f64;3]>{self.inner.local_path_for_display()}
    fn step(&mut self,w:f64)->PlannerStep {
        self.step_seq=self.step_seq.saturating_add(1);
        let submitted_before=self.inner.local_planning_calls();let admitted_before=self.inner.local_plans();
        let jobs_before=self.jobs.load(Ordering::Acquire);let held_phase_before=self.held_phase.load(Ordering::Acquire);
        let held_before=self.held.load(Ordering::Acquire);
        control_profile::begin();
        let t=Instant::now();let result=self.inner.step(w);let ms=t.elapsed().as_secs_f64()*1e3;
        let stages=control_profile::end();
        let submitted_after=self.inner.local_planning_calls();let admitted_after=self.inner.local_plans();
        let jobs_after=self.jobs.load(Ordering::Acquire);let held_phase_after=self.held_phase.load(Ordering::Acquire);
        let held_after=self.held.load(Ordering::Acquire);
        let sample=ControlStepSample {step_seq:self.step_seq,control_ms:ms,stages,
            source_view_seq:self.source_view_seq,source_present:self.source.is_some(),
            first_after_view_change:std::mem::take(&mut self.first_after_view_change),pose_stamp_ns:self.pose_stamp_ns,
            submitted_before,submitted_after,admitted_before,admitted_after,jobs_before,jobs_after,
            held_before,held_after,held_phase_before,held_phase_after,
            outcome:match &result {PlannerStep::Command(v)=>if nonzero(*v){"command_nonzero"}else{"command_zero"},
                PlannerStep::Pending(_)=>"pending",PlannerStep::Succeeded=>"succeeded",
                PlannerStep::ReplanRequired(_)=>"replan_required",PlannerStep::Failed(_)=>"failed"}};
        // One existing stats lock after the measured step. New records never
        // reallocate; overflow is visible and cannot silently qualify a run.
        let mut p=self.probe.lock().unwrap();p.control_ms.push(ms);p.submitted=submitted_after;
        if p.control_samples.len()<p.control_sample_limit {p.control_samples.push(sample);}
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
struct RawReceipt {map:Arc<SurfaceSnapshot>,boundary:Arc<Vec<[f64;2]>>,pose:PlanarPose,before:Twist2,command:Twist2,ns:u64,
    source_sample_ns:u64,source_sequence:u16,pose_stamp_ns:i64,worker_result_age_ms:f64,worker_remaining_ms:f64}
struct RawSegment {map:Arc<SurfaceSnapshot>,boundary:Arc<Vec<[f64;2]>>,from:PlanarPose,to:PlanarPose,ns:u64,dt:f64}
fn distribution(values:&[f64])->(f64,f64,f64) {let mut v=values.to_vec();v.sort_by(f64::total_cmp);if v.is_empty(){return (0.0,0.0,0.0);} (v[v.len()/2],v[((v.len()-1)as f64*0.95).ceil()as usize],*v.last().unwrap())}
fn measured_distribution(values:&[f64])->String {
    if values.is_empty(){"unavailable".into()}else{format!("{:?}",distribution(values))}
}

fn report_control_stages(p:&Probe,factory:bool,scenario:Scenario,enabled:bool,resolution:f32,startup:&str) {
    let context=format!("scenario={scenario:?} enabled={enabled} source_cm={} startup={startup}",resolution*100.0);
    if factory {
        eprintln!("CONTROL_STAGE_SCOPE {context} available=false reason=unprobed_factory wall_not_thread_cpu=true");
        return;
    }
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
        eprintln!("CONTROL_SLOW_STEP {context} sample_scope={sample_scope} rank={} step_seq={} control_wall_ms={:.6} source_view_seq={} source_present={} first_after_view_change={} pose_stamp_ns={:?} submitted_before={} submitted_after={} admitted_before={} admitted_after={} jobs_before={} jobs_after={} held_before={} held_after={} held_phase_before={} held_phase_after={} outcome={} inclusive_stage_ms_calls={stage_ms_calls:?} counter_counts={counter_counts:?}",
            rank+1,sample.step_seq,sample.control_ms,sample.source_view_seq,sample.source_present,
            sample.first_after_view_change,sample.pose_stamp_ns,sample.submitted_before,sample.submitted_after,
            sample.admitted_before,sample.admitted_after,sample.jobs_before,sample.jobs_after,
            sample.held_before,sample.held_after,sample.held_phase_before,sample.held_phase_after,sample.outcome);
    }
}
// Fixed scalar categories describe the consumer's actual Pending reasons; an
// empty 5Hz sensor queue during a 20Hz tick is expected and counted separately.
#[derive(Default,Debug)]
struct ConsumerWaits {pending:usize,source_fence:usize,expired_result:usize,pose_or_receipt:usize,arrival_pose:usize,planning:usize,other:usize,expired_at_receipt:usize}
impl ConsumerWaits {
    fn record(&mut self,reason:Option<String>){
        self.pending+=1;
        match reason.as_deref() {
            Some("地图或边界已更新，保持零速并重新计算")=>self.source_fence+=1,
            Some("局部速度计算结果已过期，保持零速并重新计算")=>self.expired_result+=1,
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
    let (config,base,keys)=fixture(enabled,resolution,if matches!(scenario,Scenario::HoldSecond|Scenario::HoldSecondContinuity){10.0}else{6.0});
    eprintln!("MOVING_POLICY compiled_default=translation_advice");
    let priming_started=Instant::now();
    let sensor=Sensor::new(base.clone(),keys);
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
    let started=Instant::now();let mut last_tick=started;let mut deadline=started;
    let cpu_before=cpu_snapshot();
    for tick in 0..max_ticks {
        if let Some(delay)=deadline.checked_duration_since(Instant::now()){std::thread::sleep(delay);}
        let now=Instant::now();let elapsed=now.duration_since(last_tick);gaps.push(elapsed.as_secs_f64()*1e3);
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
        last_tick=now;current.stamp=stamp();
        if let Ok(frame)=sensor.receive.try_recv(){
            assert!(frame.sample_ns<=unix_ns());
            if let Some(previous)=previous_frame {assert!(frame.sample_ns>previous);let previous_map=previous_source.as_ref().unwrap();assert!(nuwax_nav_pct::same_source_geometry(previous_map,&frame.map));assert!(!previous_map.same_body_query_view(&frame.map));}
            previous_frame=Some(frame.sample_ns);previous_source=Some(frame.map.clone());sequence=frame.sequence;frames+=1;
            if !changed || !matches!(scenario,Scenario::ReplaceGeometry|Scenario::Cancel){installed_sample_ns=frame.sample_ns;map=frame.map;worker.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();}
        }else{sensor_no_new_frame+=1;}
        worker.update_odometry(current.clone());
        if held.load(Ordering::Acquire)&&hold_start.is_none(){hold_start=Some(now);hold_frame_count=frames;hold_admitted_count=probe.lock().unwrap().admitted;}
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
        let t=Instant::now();let result=worker.step(WIDTH);consume_ms.push(t.elapsed().as_secs_f64()*1e3);
        if matches!(&result,PlannerStep::Pending(_)){consumer_waits.record(worker.execution_wait_reason());}
        let receipt_at=Instant::now();
        // Original worker result expiry is captured under its selection lock.
        // Replaying the same candidate never renews its permission horizon.
        let original_deadline=worker.delivered_command_deadline_for_test().unwrap_or(receipt_at);
        let permitted_until=original_deadline.min(receipt_at+PERIOD);
        if matches!(&result,PlannerStep::Command(_))&&permitted_until<=receipt_at{consumer_waits.expired_at_receipt+=1;}
        let before=command;command=match result {
            PlannerStep::Command(v) if permitted_until>receipt_at=>{accepted+=1;if nonzero(v){nonzero_count+=1;if held.load(Ordering::Acquire)&&frames>hold_frame_count&&hold_start.is_some_and(|start|now.duration_since(start)>DEFAULT_CONTROL_HORIZON+PERIOD){held_nonzero+=1;}}v},
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
        // First acquisition has one hit, never fabricate independent evidence.
        if nonzero(command){assert!(sequence>=2);}
        if changed&&matches!(scenario,Scenario::ReplaceGoal|Scenario::ReplaceBoundary|Scenario::ReplaceGeometry|Scenario::Cancel){assert!(!nonzero(command),"stale command after {scenario:?}");}
        command_at=receipt_at;command_deadline=permitted_until;segment_ns=unix_ns();
        if nonzero(command){receipts.push(RawReceipt{map:map.clone(),boundary:boundary.clone(),pose:planar(&current),before,command,ns:segment_ns,source_sample_ns:installed_sample_ns,source_sequence:sequence,pose_stamp_ns:current.stamp.nanos,
            worker_result_age_ms:DEFAULT_CONTROL_HORIZON.saturating_sub(original_deadline.saturating_duration_since(receipt_at)).as_secs_f64()*1e3,
            worker_remaining_ms:original_deadline.saturating_duration_since(receipt_at).as_secs_f64()*1e3});}
        worker.observe_applied_command(command);
        if scenario==Scenario::HoldSecondContinuity&&held_nonzero>=2&&released_at.is_some()
            &&!held.load(Ordering::Acquire)&&probe.lock().unwrap().admitted>hold_admitted_count {
            continuity_complete=true;
        }
        if delivered_negative_reason.is_some()||continuity_complete||succeeded||retired_deadline.is_some_and(|d|Instant::now()>=d){break;}
        deadline+=PERIOD;
        if deadline<Instant::now(){deadline=Instant::now();} // no fake catch-up feedback
        if tick+1==max_ticks {eprintln!("TIMEOUT scenario={scenario:?} enabled={enabled} pose={current:?} wait={:?}",worker.execution_wait_reason());}
    }
    let time_to_goal=started.elapsed();worker.stop();let _=release.try_send(());
    drop(worker); // control/planning samples settled before reading metrics
    let p=probe.lock().unwrap();
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
    // Report CPU raw /proc values; no false per-thread CPU attribution. Factory
    // runs deliberately have no inner probe: consumer latency is NOT control CPU.
    drop(p);drop(sensor);
    // Run expensive independent oracles after wall-timed control. Retained immutable
    // snapshots preserve hits, observation source stamp and actual receipt time.
    for (i,s) in segments.iter().enumerate(){
        let occupied=s.map.body_observation_footprint(body(s.from)).unwrap();
        s.map.validate_body_motion(body(s.from),body(s.to),&s.boundary).unwrap_or_else(|e|panic!("segment {i} body: {e:?}"));
        s.map.validate_observed_body_motion_in_boundary(body(s.from),body(s.to),&occupied,s.ns+(s.dt*1e9)as u64,(OBS_AGE*1e9)as u64,&s.boundary).unwrap_or_else(|e|panic!("segment {i} observation: {e:?}"));
    }
    for (i,r) in receipts.iter().enumerate(){
        let occupied=r.map.body_observation_footprint(body(r.pose)).unwrap();
        // Diagnostic-only additions: retain the exact prior flat-body oracle,
        // receipt clock, initial footprint exemption and unchanged age limit.
        // In particular a production-projected alternative cannot make it pass.
        let first_failure=std::cell::RefCell::new(None::<String>);
        let make_check=|kind:&'static str| {
            let first_failure=&first_failure;let occupied=&occupied;
            move |a:PlanarPose,b:PlanarPose,elapsed:f64| {
                let checked_ns=r.ns+(elapsed*1e9)as u64;
                let result=r.map.validate_observed_body_motion_in_boundary(body(a),body(b),occupied,checked_ns,(OBS_AGE*1e9)as u64,&r.boundary)
                    .map_err(|e|format!("initial-footprint observed sweep: {e:?}"))
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
            let footprint_evidence:Vec<_>=occupied.iter().take(4).map(|key|(*key,r.map.observations().and_then(|o|o.get(*key)))).collect();
            eprintln!("BRAKING_ORACLE_FAILURE receipt={i} source_cm={} receipt_ns={} pose_stamp_ns={} source_sample_ns={} source_sequence={} source_age_at_receipt_ms={:.6} worker_result_age_ms={:.6} worker_remaining_ms={:.6} actual_guard_entry_time=unavailable computed_pose_applied_context=unavailable pose={:?} before={:?} command={:?} initial_footprint_cells={} footprint_evidence_first4={footprint_evidence:?} current_body={current_body_result:?} current_view_same={current_view_same:?} first_failure={:?}",
                resolution*100.0,r.ns,r.pose_stamp_ns,r.source_sample_ns,r.source_sequence,r.ns.saturating_sub(r.source_sample_ns)as f64/1e6,r.worker_result_age_ms,r.worker_remaining_ms,r.pose,r.before,r.command,occupied.len(),first_failure.borrow());
        }
        assert!(choice.is_some(),"receipt {i} independent full braking oracle rejected: {:?}",first_failure.into_inner());
    }
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
    let occupied=second.map.body_observation_footprint(from).unwrap();
    assert!(first.map.validate_observed_body_motion(from,to,&occupied,unix_ns(),(OBS_AGE*1e9)as u64).is_err(),"one sample must not invent two hits");
    second.map.validate_observed_body_motion(from,to,&occupied,unix_ns(),(OBS_AGE*1e9)as u64).expect("fresh actual source evidence must permit independent segment");
    std::thread::sleep(Duration::from_secs_f64(OBS_AGE+0.05));
    assert!(second.map.validate_observed_body_motion(from,to,&occupied,unix_ns(),(OBS_AGE*1e9)as u64).is_err(),"genuinely aged captured source must reject; do not rewrite its timestamps");
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

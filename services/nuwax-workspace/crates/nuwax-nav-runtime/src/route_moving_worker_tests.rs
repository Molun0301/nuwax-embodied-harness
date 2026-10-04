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
use nuwax_nav_control::{integrate, PlanarPose};
use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
use nuwax_nav_types_messages::{GoalOptions, NavFrameId, MotionLimits, MotionModel, Point3, Pose3, Quaternion, Timestamp};
use std::collections::BTreeMap;
use std::sync::{atomic::{AtomicBool, AtomicUsize, Ordering}, mpsc, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const PERIOD: Duration = Duration::from_millis(50);
const SENSOR_PERIOD: Duration = Duration::from_millis(200);
const WIDTH: f64 = 0.3;
const OBS_AGE: f64 = 1.0; // explicit synthetic terrain capability, never extended
const MAX_TICKS: usize = 1_200;

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

#[derive(Default)]
struct Probe {control_ms:Vec<f64>,map_update_ms:Vec<f64>,submitted:u64,admitted:u64,negative:usize,pending:usize,endpoints:Vec<[f64;3]>}
struct Measured {inner:RouteFollower,probe:Arc<Mutex<Probe>>}
impl NavPlanner for Measured {
    fn configure_goal(&mut self,g:&PoseStamped,o:GoalOptions)->Result<(),RuntimeError>{self.inner.configure_goal(g,o)}
    fn configure_boundary(&mut self,b:&[[f64;2]]){self.inner.configure_boundary(b)}
    fn replace_path(&mut self,p:Path2D){self.inner.replace_path(p)}
    fn replace_execution_path(&mut self,p:ExecutionPath,y:Option<Vec<f64>>,g:PoseStamped)->Result<(),RuntimeError>{self.inner.replace_execution_path(p,y,g)}
    fn update_odometry(&mut self,p:PoseStamped){self.inner.update_odometry(p)}
    fn update_costmap(&mut self,m:CostMap2D){self.inner.update_costmap(m)}
    fn update_execution_map(&mut self,m:ExecutionMap)->Result<(),RuntimeError>{let t=Instant::now();let result=self.inner.update_execution_map(m);self.probe.lock().unwrap().map_update_ms.push(t.elapsed().as_secs_f64()*1e3);result}
    fn invalidate_execution_map(&mut self){self.inner.invalidate_execution_map()}
    fn observe_applied_command(&mut self,c:Twist2){self.inner.observe_applied_command(c)}
    fn execution_wait_reason(&self)->Option<String>{self.inner.execution_wait_reason()}
    fn local_path_for_display(&self)->Vec<[f64;3]>{self.inner.local_path_for_display()}
    fn step(&mut self,w:f64)->PlannerStep {
        let t=Instant::now();let result=self.inner.step(w);let ms=t.elapsed().as_secs_f64()*1e3;
        let mut p=self.probe.lock().unwrap();p.control_ms.push(ms);p.submitted=self.inner.local_planning_calls();
        if self.inner.local_plans()>p.admitted {if let Some(end)=self.inner.local_path().and_then(|path|path.poses.last()){let v=end.pose.position;p.endpoints.push([v.x,v.y,v.z]);}}
        p.admitted=self.inner.local_plans();
        p.negative+=usize::from(matches!(result,PlannerStep::Failed(_)|PlannerStep::ReplanRequired(_)));
        p.pending+=usize::from(matches!(result,PlannerStep::Pending(_)));result
    }
    fn stop(&mut self){self.inner.stop()}
}

#[derive(Clone,Copy,Debug,PartialEq)]
enum Scenario {Goal,HoldFirst,HoldSecond,ReplaceGoal,ReplacePath,ReplaceBoundary,ReplaceGeometry,Cancel}
struct RawReceipt {map:Arc<SurfaceSnapshot>,boundary:Arc<Vec<[f64;2]>>,pose:PlanarPose,before:Twist2,command:Twist2,ns:u64}
struct RawSegment {map:Arc<SurfaceSnapshot>,boundary:Arc<Vec<[f64;2]>>,from:PlanarPose,to:PlanarPose,ns:u64,dt:f64}
fn distribution(values:&[f64])->(f64,f64,f64) {let mut v=values.to_vec();v.sort_by(f64::total_cmp);if v.is_empty(){return (0.0,0.0,0.0);} (v[v.len()/2],v[((v.len()-1)as f64*0.95).ceil()as usize],*v.last().unwrap())}
fn measured_distribution(values:&[f64])->String {
    if values.is_empty(){"unavailable".into()}else{format!("{:?}",distribution(values))}
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

fn run(enabled:bool,resolution:f32,scenario:Scenario,factory:bool) {
    let (config,base,keys)=fixture(enabled,resolution,if scenario==Scenario::HoldSecond{10.0}else{6.0});
    let sensor=Sensor::new(base.clone(),keys);
    let probe=Arc::new(Mutex::new(Probe{control_ms:Vec::with_capacity(MAX_TICKS*2),..Default::default()}));
    let jobs=Arc::new(AtomicUsize::new(0));let held=Arc::new(AtomicBool::new(false));
    let (release,receive)=mpsc::sync_channel(1);
    let mut worker=if factory {assert_eq!(scenario,Scenario::Goal);production_surface_backend(config.clone()).unwrap()} else {
        let count=jobs.clone();let active=held.clone();
        let inner=RouteFollower::new_with_planning_hook(config.clone(),Box::new(move |_| {
            let job=count.fetch_add(1,Ordering::SeqCst)+1;
            let hold=match scenario{Scenario::Goal=>false,Scenario::HoldSecond=>job==2,_=>job==1};
            if hold {active.store(true,Ordering::Release);let _=receive.recv_timeout(Duration::from_secs(10));active.store(false,Ordering::Release);}
        })).unwrap();
        planner_worker::PlannerWorker::new(Measured{inner,probe:probe.clone()}).unwrap()
    };
    let mut current=pose(0.75,1.5,0.0);
    let goal=pose(if scenario==Scenario::HoldSecond{7.75}else{1.75},1.5,std::f64::consts::FRAC_PI_2);
    let options=GoalOptions {position_tolerance_m:0.15,yaw_tolerance_rad:0.15,allow_reverse:false};
    worker.configure_goal(&goal,options).unwrap();worker.update_odometry(current.clone());
    worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![current.clone(),goal.clone()]}),None,goal.clone()).unwrap();
    let mut map=base;let mut boundary=Arc::new(Vec::new());let mut frames=0usize;let mut sensor_no_new_frame=0usize;let mut consumer_waits=ConsumerWaits::default();
    let mut previous_frame=None::<u64>;let mut previous_source=None::<Arc<SurfaceSnapshot>>;let mut sequence=0u16;
    let mut receipts=Vec::with_capacity(MAX_TICKS);let mut segments=Vec::with_capacity(MAX_TICKS);
    let mut gaps=Vec::with_capacity(MAX_TICKS);let mut consume_ms=Vec::with_capacity(MAX_TICKS);
    let mut command=Twist2::ZERO;let mut command_at=Instant::now();let mut command_deadline=command_at;let mut segment_ns=unix_ns();
    let mut nonzero_count=0;let mut accepted=0;let mut held_nonzero=0;let mut succeeded=false;
    let mut hold_start=None;let mut hold_frame_count=0;let mut changed=false;let mut retired_deadline=None;
    let started=Instant::now();let mut last_tick=started;let mut deadline=started;
    let cpu_before=cpu_snapshot();
    for tick in 0..MAX_TICKS {
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
            if !changed || !matches!(scenario,Scenario::ReplaceGeometry|Scenario::Cancel){map=frame.map;worker.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();}
        }else{sensor_no_new_frame+=1;}
        worker.update_odometry(current.clone());
        if held.load(Ordering::Acquire)&&hold_start.is_none(){hold_start=Some(now);hold_frame_count=frames;}
        if let Some(hold)=hold_start {
            if !changed && !matches!(scenario,Scenario::HoldFirst|Scenario::HoldSecond) {
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
            if now.duration_since(hold)>=Duration::from_millis(650){let _=release.try_send(());}
        }
        let t=Instant::now();let result=worker.step(WIDTH);consume_ms.push(t.elapsed().as_secs_f64()*1e3);
        if matches!(&result,PlannerStep::Pending(_)){consumer_waits.record(worker.execution_wait_reason());}
        let receipt_at=Instant::now();
        // Original worker result expiry is captured under its selection lock.
        // Replaying the same candidate never renews its permission horizon.
        let permitted_until=worker.delivered_command_deadline_for_test().unwrap_or(receipt_at).min(receipt_at+PERIOD);
        if matches!(&result,PlannerStep::Command(_))&&permitted_until<=receipt_at{consumer_waits.expired_at_receipt+=1;}
        let before=command;command=match result {
            PlannerStep::Command(v) if permitted_until>receipt_at=>{accepted+=1;if nonzero(v){nonzero_count+=1;if held.load(Ordering::Acquire)&&frames>hold_frame_count&&hold_start.is_some_and(|start|now.duration_since(start)>DEFAULT_CONTROL_HORIZON+PERIOD){held_nonzero+=1;}}v},
            PlannerStep::Succeeded=>{succeeded=true;Twist2::ZERO},_=>Twist2::ZERO,
        };
        // First acquisition has one hit, never fabricate independent evidence.
        if nonzero(command){assert!(sequence>=2);}
        if changed&&matches!(scenario,Scenario::ReplaceGoal|Scenario::ReplaceBoundary|Scenario::ReplaceGeometry|Scenario::Cancel){assert!(!nonzero(command),"stale command after {scenario:?}");}
        command_at=receipt_at;command_deadline=permitted_until;segment_ns=unix_ns();
        if nonzero(command){receipts.push(RawReceipt{map:map.clone(),boundary:boundary.clone(),pose:planar(&current),before,command,ns:segment_ns});}
        worker.observe_applied_command(command);
        if succeeded||retired_deadline.is_some_and(|d|Instant::now()>=d){break;}
        deadline+=PERIOD;
        if deadline<Instant::now(){deadline=Instant::now();} // no fake catch-up feedback
        if tick+1==MAX_TICKS {eprintln!("TIMEOUT scenario={scenario:?} enabled={enabled} pose={current:?} wait={:?}",worker.execution_wait_reason());}
    }
    let time_to_goal=started.elapsed();worker.stop();let _=release.try_send(());
    drop(worker); // control/planning samples settled before reading metrics
    let p=probe.lock().unwrap();
    let planning_metrics=if factory {"unavailable (unprobed factory)".to_owned()}else{format!("jobs_started={} submitted={} admitted={} control_pending={} control_negative={}",jobs.load(Ordering::Relaxed),p.submitted,p.admitted,p.pending,p.negative)};
    let distance_m: f64=segments.iter().map(|s|(s.to.x-s.from.x).hypot(s.to.y-s.from.y)).sum();
    let displacement_m=(current.pose.position.x-0.75).hypot(current.pose.position.y-1.5);
    let tick_gaps=&gaps[1..];
    eprintln!("MOVING scenario={scenario:?} enabled={enabled} source_cm={} factory={factory} elapsed_s={:.3} succeeded={succeeded} accepted={accepted} nonzero={nonzero_count} held_nonzero={held_nonzero} sensor_frames={frames} sensor_no_new_frame={sensor_no_new_frame} dropped_frames={} consumer_waits={consumer_waits:?} planning_metrics=[{planning_metrics}] final_pose_xyzyaw={:?} actual_displacement_m={displacement_m:.6} actual_travel_m={distance_m:.6} tick_gap_ms_median_p95_max={} tick_gaps_over_50ms={} tick_gaps_over_75ms={} consumer_ms_median_p95_max={} control_step_ms_median_p95_max={} map_update_ms_median_p95_max={}\nCPU BEFORE {cpu_before}\nCPU AFTER {}",
        resolution*100.0,time_to_goal.as_secs_f64(),sensor.dropped.load(Ordering::Relaxed),
        [current.pose.position.x,current.pose.position.y,current.pose.position.z,current.pose.orientation.yaw()],
        measured_distribution(tick_gaps),tick_gaps.iter().filter(|ms|**ms>50.0).count(),tick_gaps.iter().filter(|ms|**ms>75.0).count(),
        measured_distribution(&consume_ms),measured_distribution(&p.control_ms),measured_distribution(&p.map_update_ms),cpu_snapshot());
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
        let check=|a:PlanarPose,b:PlanarPose,elapsed:f64| {
            r.map.validate_observed_body_motion_in_boundary(body(a),body(b),&occupied,r.ns+(elapsed*1e9)as u64,(OBS_AGE*1e9)as u64,&r.boundary).is_ok()
                && r.map.body_query_with_current_support(body(a).0,body(a).1).and_then(|view|view.validate_body_motion(body(a),body(b),&r.boundary)).is_ok()
        };
        assert!(nuwax_nav_control::check_stopping_trajectory_with_period_check(r.pose,r.before,r.command,&limits(),PERIOD.as_secs_f64(),f64::from(resolution)*0.5,r.map.body_envelope().unwrap().radius(),check,check).is_some(),"receipt {i} independent full braking oracle rejected");
    }
    let p=probe.lock().unwrap();
    if matches!(scenario,Scenario::Goal|Scenario::HoldFirst|Scenario::HoldSecond){assert!(succeeded,"full position plus final yaw required");assert!(nonzero_count>0);assert!(!segments.is_empty());assert!(current.pose.position.x>1.5,"actual accepted-command displacement required");assert!(current.pose.position.distance(goal.pose.position)<options.position_tolerance_m);assert!(nuwax_nav_astar::controllers::angle_diff(current.pose.orientation.yaw(),goal.pose.orientation.yaw()).abs()<options.yaw_tolerance_rad);}
    if scenario==Scenario::HoldSecond{assert!(held_nonzero>=2,"must continue fresh guarded motion on previously accepted path while real planning owner is held");}
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

//! Persistent synthetic-worker endurance, deliberately narrower than memq/robot-port qualification.
//! Oracle drains run only with an acknowledged stopped worker and expired plant command.
use super::*;
use crate::surface_changes::{SurfaceGeometryChanges, SurfaceMapUpdate};

const POSE_PERIOD: Duration = Duration::from_millis(25);
const SEED: u64 = 20261013;
// 60s of 40Hz poses + 20Hz consumer requests + 5Hz map updates, rounded
// conservatively to 80Hz. This does not change the original fixture cap.
const LEG_RECORD_LIMIT: usize = MAX_TICKS*4+PROFILE_DRAIN_ALLOWANCE;
const PHASES: [&str; 5] = ["baseline", "recent_remote", "pose_jitter", "combined", "replacement_cancel_restart"];

// Keep the existing measured fixture unchanged. Forward the production current-state
// recheck hooks here too: NavPlanner's default None would suppress that real path.
struct SustainedMeasured {
    measured: Measured,
    stopped: Arc<AtomicUsize>,
    rechecks: Arc<Mutex<Vec<f64>>>,
}
impl NavPlanner for SustainedMeasured {
    fn configure_goal(&mut self,g:&PoseStamped,o:GoalOptions)->Result<(),RuntimeError>{self.measured.configure_goal(g,o)}
    fn configure_boundary(&mut self,b:&[[f64;2]]){self.measured.configure_boundary(b)}
    fn replace_path(&mut self,p:Path2D){self.measured.replace_path(p)}
    fn replace_execution_path(&mut self,p:ExecutionPath,y:Option<Vec<f64>>,g:PoseStamped)->Result<(),RuntimeError>{self.measured.replace_execution_path(p,y,g)}
    fn update_odometry(&mut self,p:PoseStamped){self.measured.update_odometry(p)}
    fn update_costmap(&mut self,m:CostMap2D){self.measured.update_costmap(m)}
    fn update_execution_map(&mut self,m:ExecutionMap)->Result<(),RuntimeError>{self.measured.update_execution_map(m)}
    fn invalidate_execution_map(&mut self){self.measured.invalidate_execution_map()}
    fn observe_applied_command(&mut self,c:Twist2){self.measured.observe_applied_command(c)}
    fn execution_wait_reason(&self)->Option<String>{self.measured.execution_wait_reason()}
    fn local_path_for_display(&self)->Vec<[f64;3]>{self.measured.local_path_for_display()}
    fn step(&mut self,w:f64)->PlannerStep{self.measured.step(w)}
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound>{self.measured.take_command_dispatch_bound()}
    fn revalidate_selected_command(&mut self,c:Twist2)->Option<PlannerStep>{
        let t=Instant::now();let r=self.measured.inner.revalidate_selected_command(c);
        self.rechecks.lock().unwrap().push(t.elapsed().as_secs_f64()*1e3);r
    }
    fn revalidate_selected_command_at_current_state(&mut self,c:Twist2)->Option<PlannerStep>{
        let t=Instant::now();let r=self.measured.inner.revalidate_selected_command_at_current_state(c);
        self.rechecks.lock().unwrap().push(t.elapsed().as_secs_f64()*1e3);r
    }
    fn stop(&mut self){self.measured.stop();self.stopped.fetch_add(1,Ordering::Release);}
}

fn rss_status()->String {
    std::fs::read_to_string("/proc/self/status").unwrap_or_default().lines()
        .filter(|s|s.starts_with("VmRSS:")||s.starts_with("VmHWM:")||s.starts_with("Threads:"))
        .collect::<Vec<_>>().join("; ")
}

// Opt-in stopped-boundary diagnostics only: no allocator replacement, production
// cache clearing, extra hot-path counters, or changes to the motion oracles.
// mallinfo2's in-use count includes allocator internals/tcache effects; it is NOT
// an exact live Rust payload count. The sensor can allocate during A/B/C sampling.
mod memory_diagnostic {
    use super::*;

    #[cfg(all(target_os="linux",target_env="gnu"))]
    // This entire enclosing fixture is cfg(test). Only these optional GNU C ABI
    // calls need unsafe; the crate's production deny(unsafe_code) is unchanged.
    #[allow(unsafe_code)]
    mod gnu {
        use std::ffi::{c_char,c_int,c_void};
        #[repr(C)]
        struct Mallinfo2 {
            arena:usize,ordblks:usize,smblks:usize,hblks:usize,hblkhd:usize,
            usmblks:usize,fsmblks:usize,uordblks:usize,fordblks:usize,keepcost:usize,
        }
        // Resolve optionally: older GNU libc may not export mallinfo2. These are
        // the system malloc.h signatures, with size_t represented by usize.
        #[link(name="dl")]
        unsafe extern "C" {fn dlsym(handle:*mut c_void,symbol:*const c_char)->*mut c_void;}
        type Info=unsafe extern "C" fn()->Mallinfo2;
        type Trim=unsafe extern "C" fn(usize)->c_int;
        fn permitted()->bool {
            std::env::var_os("LD_PRELOAD").is_none_or(|value|value.is_empty())
        }
        pub(super) fn info()->Option<[usize;6]> {
            if !permitted(){return None;}
            // RTLD_DEFAULT is null on GNU/Linux. No owned handle is created.
            let symbol=unsafe {dlsym(std::ptr::null_mut(),c"mallinfo2".as_ptr())};
            if symbol.is_null(){return None;}
            let call:Info=unsafe {std::mem::transmute(symbol)};
            let value=unsafe {call()};
            Some([value.arena,value.uordblks,value.fordblks,value.hblkhd,value.hblks,value.keepcost])
        }
        pub(super) fn trim()->Option<i32> {
            if !permitted(){return None;}
            let symbol=unsafe {dlsym(std::ptr::null_mut(),c"malloc_trim".as_ptr())};
            if symbol.is_null(){return None;}
            let call:Trim=unsafe {std::mem::transmute(symbol)};
            Some(unsafe {call(0)})
        }
    }

    fn proc_value(text:&str,key:&str)->Option<u64> {
        text.lines().find_map(|line|line.strip_prefix(key)
            .and_then(|value|value.split_whitespace().next())
            .and_then(|value|value.parse().ok()))
    }
    pub(super) fn checkpoint(enabled:bool,leg:usize,phase:&str,point:&str,sensor_live:bool) {
        // Capture allocator statistics before allocating procfs read buffers.
        #[cfg(all(target_os="linux",target_env="gnu"))]
        let allocator=gnu::info();
        #[cfg(not(all(target_os="linux",target_env="gnu")))]
        let allocator:Option<[usize;6]>=None;
        let status=std::fs::read_to_string("/proc/self/status").unwrap_or_default();
        let rollup=std::fs::read_to_string("/proc/self/smaps_rollup").unwrap_or_default();
        eprintln!("SUSTAINED_MEMORY enabled={enabled} leg={leg} phase={phase} point={point} sensor_live={sensor_live} rss_kib={:?} hwm_kib={:?} threads={:?} anonymous_kib={:?} private_dirty_kib={:?} allocator_arena_inuse_free_mmapbytes_mmapcount_releasable={allocator:?} allocator_units=bytes_except_mmapcount missing_metric=None inuse_is_not_exact_live_rust_bytes=true samples_not_atomic=true",
            proc_value(&status,"VmRSS:"),proc_value(&status,"VmHWM:"),proc_value(&status,"Threads:"),
            proc_value(&rollup,"Anonymous:"),proc_value(&rollup,"Private_Dirty:"));
    }
    fn vector<T>(enabled:bool,leg:usize,point:&str,name:&str,value:&Vec<T>) {
        eprintln!("SUSTAINED_MEMORY_VECTOR enabled={enabled} leg={leg} point={point} name={name} len={} capacity={} element_bytes={} capacity_bytes={} nested_payload_excluded=true",
            value.len(),value.capacity(),std::mem::size_of::<T>(),value.capacity()*std::mem::size_of::<T>());
    }
    pub(super) fn persistent(enabled:bool,leg:usize,point:&str,p:&Probe,checks:&Vec<f64>) {
        vector(enabled,leg,point,"control_ms",&p.control_ms);
        vector(enabled,leg,point,"map_update_ms",&p.map_update_ms);
        vector(enabled,leg,point,"endpoints",&p.endpoints);
        vector(enabled,leg,point,"control_samples",&p.control_samples);
        vector(enabled,leg,point,"source_frames",&p.source_frames);
        vector(enabled,leg,point,"source_installs",&p.source_installs);
        vector(enabled,leg,point,"rechecks",checks);
    }
    pub(super) fn history<E>(enabled:bool,leg:usize,receipts:&Vec<RawReceipt>,segments:&Vec<RawSegment>,gaps:&Vec<f64>,events:&Vec<E>) {
        vector(enabled,leg,"A_pre_oracle","receipts",receipts);
        vector(enabled,leg,"A_pre_oracle","segments",segments);
        vector(enabled,leg,"A_pre_oracle","gaps",gaps);
        vector(enabled,leg,"A_pre_oracle","events",events);
    }
    pub(super) fn terminal_trim(enabled:bool) {
        #[cfg(all(target_os="linux",target_env="gnu"))]
        let result=gnu::trim();
        #[cfg(not(all(target_os="linux",target_env="gnu")))]
        let result:Option<i32>=None;
        eprintln!("SUSTAINED_MEMORY_TRIM enabled={enabled} result={result:?} after_all_motion_and_owner_joins=true diagnostic_intervention_only=true missing_metric=None");
    }
    #[test]
    fn proc_metrics_are_optional_and_exactly_named() {
        let status="VmHWM:\t42 kB\nVmRSS:\t37 kB\nThreads:\t13\n";
        assert_eq!(proc_value(status,"VmRSS:"),Some(37));
        assert_eq!(proc_value(status,"Threads:"),Some(13));
        assert_eq!(proc_value(status,"Rss:"),None);
        assert_eq!(proc_value("VmRSS: unavailable","VmRSS:"),None);
        assert_eq!(proc_value("","VmRSS:"),None);
    }
}
#[derive(Default,Debug,PartialEq)]
struct Lifecycle {epoch:u64,goal:u64,path:u64}
impl Lifecycle {
    fn install(&mut self){self.epoch+=2;self.goal+=1;self.path+=2;}
    fn stop(&mut self){self.epoch+=1;self.goal+=1;self.path+=1;}
}
fn advance_plant(current:&mut PoseStamped,integrated:&mut Instant,now:Instant,command:Twist2,
    command_at:Instant,deadline:Instant,map:&Arc<SurfaceSnapshot>,boundary:&Arc<Vec<[f64;2]>>,
    segment_ns:u64,segments:&mut Vec<RawSegment>) {
    let start=(*integrated).max(command_at);
    let dt=now.saturating_duration_since(start).min(deadline.saturating_duration_since(start)).as_secs_f64();
    if nonzero(command)&&dt>0.0 {
        let from=planar(current);let to=integrate(from,command,dt);
        segments.push(RawSegment{map:map.clone(),boundary:boundary.clone(),from,to,ns:segment_ns,dt});
        current.pose.position.x=to.x;current.pose.position.y=to.y;current.pose.orientation=Quaternion::from_yaw(to.yaw);
    }
    *integrated=now;
}
fn install_leg(worker:&mut planner_worker::PlannerWorker,current:&PoseStamped,goal:&PoseStamped,lifecycle:&mut Lifecycle) {
    lifecycle.install();
    worker.configure_goal(goal,GoalOptions{position_tolerance_m:0.15,yaw_tolerance_rad:0.15,allow_reverse:false}).unwrap();
    worker.update_odometry(current.clone());
    // Explicit orientation at the start handles the reverse direction without
    // permitting reverse drive or teleporting the physical plant.
    let mut aligned=current.clone();
    aligned.pose.orientation=Quaternion::from_yaw((goal.pose.position.y-current.pose.position.y).atan2(goal.pose.position.x-current.pose.position.x));
    worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![current.clone(),aligned,goal.clone()]}),None,goal.clone()).unwrap();
}
fn stop_ack(worker:&mut planner_worker::PlannerWorker,stopped:&AtomicUsize) {
    let before=stopped.load(Ordering::Acquire);worker.stop();
    let deadline=Instant::now()+Duration::from_secs(2);
    while stopped.load(Ordering::Acquire)==before {
        assert!(Instant::now()<deadline,"compute-owner stop acknowledgement timed out");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn sustained(enabled:bool,short:bool) {
    let memory=std::env::var_os("NEH_SUSTAINED_MEMORY_DIAGNOSTIC").is_some_and(|value|value=="1");
    if memory {memory_diagnostic::checkpoint(enabled,0,"setup","before_fixture",false);}
    let (config,base,keys)=fixture(enabled,0.05,6.0);
    let sensor=Sensor::new(base.clone(),keys);
    let probe=Arc::new(Mutex::new(Probe{
        control_ms:Vec::with_capacity(LEG_RECORD_LIMIT),map_update_ms:Vec::with_capacity(MAX_TICKS),
        control_samples:Vec::with_capacity(LEG_RECORD_LIMIT),source_frames:Vec::with_capacity(MAX_TICKS),
        source_installs:Vec::with_capacity(MAX_TICKS),control_sample_limit:LEG_RECORD_LIMIT,..Probe::default()
    }));
    let jobs=Arc::new(AtomicUsize::new(0));let held=Arc::new(AtomicBool::new(false));let held_phase=Arc::new(AtomicUsize::new(0));
    let stopped=Arc::new(AtomicUsize::new(0));let rechecks=Arc::new(Mutex::new(Vec::with_capacity(MAX_TICKS*3)));
    let count=jobs.clone();
    let inner=RouteFollower::new_with_planning_hook(config,Box::new(move |_|{
        count.fetch_add(1,Ordering::SeqCst);
    })).unwrap();
    let mut worker=planner_worker::PlannerWorker::new(SustainedMeasured{
        measured:Measured::new(inner,probe.clone(),jobs.clone(),held.clone(),held_phase.clone()),stopped:stopped.clone(),rechecks:rechecks.clone()}).unwrap();
    let mut map=base;let boundary=Arc::new(Vec::new());
    let mut source_ns=0;let mut sequence=0;let mut frames=0usize;
    for _ in 0..2 {
        let frame=sensor.receive.recv_timeout(Duration::from_secs(5)).expect("two real source priming frames");
        source_ns=frame.sample_ns;sequence=frame.sequence;frames+=1;map=frame.map;
        worker.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();
    }
    let mut current=pose(0.75,1.5,0.0);
    let mut lifecycle=Lifecycle::default();let mut previous_id=None;let mut completed=0;let mut active_time=Duration::ZERO;let mut oracle_time=Duration::ZERO;
    let mut phase_legs=[0usize;5];let mut phase_seconds=[0f64;5];
    let mut total_receipts=0;let mut total_segments=0;let mut total_distance=0.0;let mut total_motion_seconds=0.0;
    let mut replacements=0;let mut cancellations=0;let started=Instant::now();
    eprintln!("SUSTAINED_BEGIN enabled={enabled} short={short} seed={SEED} phase_seconds=60 full_arm_minimum_active_seconds=300 goal_budget_seconds=60 source_hz=5 pose_hz=40 command_hz=20 jitter_m=0.004 jitter_rad=0.01 remote_return=non_ground_obstacle rss_diagnostic_only=true");
    while if short {completed<5} else {active_time<Duration::from_secs(300)||phase_legs[4]==0} {
        // Drain pauses may leave one old queued frame. While stopped, consume
        // until an actually fresh acquisition arrives; never renew its timestamp.
        loop {
            let frame=sensor.receive.recv_timeout(Duration::from_secs(5)).expect("live source after oracle drain");
            if !source_healthy_at(frame.sample_ns,unix_ns()){continue;}
            assert!(frame.sample_ns>source_ns);source_ns=frame.sample_ns;sequence=frame.sequence;frames+=1;
            map=frame.map;worker.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();break;
        }
        let phase=if short {completed}else{(active_time.as_secs()/60).min(4)as usize};
        let jitter=matches!(phase,2|3);let recent=matches!(phase,1|3);
        let east=completed%2==0;let mut goal=pose(if east{1.75}else{0.75},1.5,if east{0.0}else{std::f64::consts::PI});
        install_leg(&mut worker,&current,&goal,&mut lifecycle);
        let mut receipts=Vec::with_capacity(MAX_TICKS);let mut segments=Vec::with_capacity(MAX_TICKS*4+PROFILE_DRAIN_ALLOWANCE);
        let mut events=Vec::with_capacity(MAX_TICKS*3);
        let mut gaps=Vec::with_capacity(MAX_TICKS);let mut command=Twist2::ZERO;
        let leg_start=Instant::now();let mut next_pose=leg_start;let mut integrated=leg_start;
        let mut next_control=leg_start+Duration::from_millis(12);let mut last_control=None;
        let mut command_at=leg_start;let mut command_deadline=leg_start;let mut segment_ns=unix_ns();
        let mut poses=0usize;let mut controls=0usize;let mut replaced=false;let mut cancelled=false;let mut succeeded=false;
        let mut measured=current.clone();
        eprintln!("SUSTAINED_LEG_BEGIN enabled={enabled} leg={completed} phase={} active_s={:.6} unix_ns={} target_xyyaw={:?} replacement_at_s=1 cancel_restart_at_s=2",PHASES[phase],active_time.as_secs_f64(),unix_ns(),[goal.pose.position.x,goal.pose.position.y,goal.pose.orientation.yaw()]);
        while leg_start.elapsed()<Duration::from_secs(60) {
            let now=Instant::now();
            advance_plant(&mut current,&mut integrated,now,command,command_at,command_deadline,&map,&boundary,segment_ns,&mut segments);
            if now>=command_deadline{command=Twist2::ZERO;}
            if let Ok(frame)=sensor.receive.try_recv() {
                assert!(frame.sample_ns>source_ns&&frame.sample_ns<=unix_ns());
                probe.lock().unwrap().record_frame(&frame,unix_ns());
                source_ns=frame.sample_ns;sequence=frame.sequence;frames+=1;
                events.push((source_ns,"source",u64::from(sequence),if recent{if (u64::from(sequence)+SEED)%2==0{1i8}else{-1i8}}else{0i8}));
                let next=if recent {
                    let sign=if (u64::from(sequence)+SEED)%2==0{1.0}else{-1.0};
                    Arc::new(frame.map.with_recent_obstacle_points(u64::from(sequence),source_ns,0,&[],[(0,[4.5+sign*0.01,0.5,0.5])]).unwrap())
                }else{frame.map};
                assert!(nuwax_nav_pct::same_source_geometry(&map,&next));
                worker.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate{planar_reference:None,snapshot:next.clone(),change_basis:Some(map.clone()),changes:Arc::new(SurfaceGeometryChanges::Empty)})).unwrap();
                map=next;
            }
            if now>=next_pose {
                current.stamp=stamp();measured=current.clone();
                if jitter {let sign=if (poses as u64+SEED)%2==0{1.0}else{-1.0};measured.pose.position.x+=sign*0.004;
                    measured.pose.orientation=Quaternion::from_yaw(current.pose.orientation.yaw()+sign*0.01);}
                events.push((unix_ns(),"pose",poses as u64,if jitter{if (poses as u64+SEED)%2==0{1i8}else{-1i8}}else{0i8}));
                worker.update_odometry(measured.clone());poses+=1;next_pose+=POSE_PERIOD;
                if next_pose<Instant::now(){next_pose=Instant::now()+POSE_PERIOD;}
            }
            if now>=next_control {
                if let Some(last)=last_control {gaps.push(now.duration_since(last).as_secs_f64()*1e3);}last_control=Some(now);
                if phase==4&&!replaced&&leg_start.elapsed()>=Duration::from_secs(1) {
                    goal.pose.position.y=1.7;install_leg(&mut worker,&measured,&goal,&mut lifecycle);replaced=true;replacements+=1;
                    eprintln!("SUSTAINED_EVENT kind=replace_goal_path leg={completed} unix_ns={}",unix_ns());
                }
                if phase==4&&!cancelled&&leg_start.elapsed()>=Duration::from_secs(2) {
                    advance_plant(&mut current,&mut integrated,Instant::now(),command,command_at,command_deadline,&map,&boundary,segment_ns,&mut segments);
                    worker.stop();lifecycle.stop();command=Twist2::ZERO;worker.observe_applied_command(command);
                    assert!(!matches!(worker.step(WIDTH),PlannerStep::Command(c) if nonzero(c)),"old result survived cancel");
                    install_leg(&mut worker,&measured,&goal,&mut lifecycle);cancelled=true;cancellations+=1;
                    eprintln!("SUSTAINED_EVENT kind=cancel_restart leg={completed} unix_ns={}",unix_ns());
                }
                let result=worker.step(WIDTH);let bound=worker.take_command_dispatch_bound();let receipt_at=Instant::now();
                let original=worker.delivered_command_deadline_for_test().unwrap_or(receipt_at);
                let permitted=original.min(receipt_at+PERIOD);
                let dispatch_expired=bound.is_some_and(CommandDispatchBound::expired_now)
                    ||CommandDispatchBound::from_source_timestamp(source_ns,Duration::from_secs_f64(OBS_AGE)).expired_now();
                advance_plant(&mut current,&mut integrated,receipt_at,command,command_at,command_deadline,&map,&boundary,segment_ns,&mut segments);
                if receipt_at>=command_deadline {command=Twist2::ZERO;}
                let before=command;
                command=match result {
                    PlannerStep::Command(c) if permitted>receipt_at&&!dispatch_expired=>c,
                    PlannerStep::Succeeded=>{succeeded=true;Twist2::ZERO},
                    bad @ (PlannerStep::Failed(_)|PlannerStep::ReplanRequired(_))=>panic!("sustained terminal enabled={enabled} leg={completed} phase={} elapsed={:?} pose={current:?} result={bad:?}",PHASES[phase],leg_start.elapsed()),
                    _=>Twist2::ZERO,
                };
                command_at=receipt_at;command_deadline=permitted;segment_ns=unix_ns();
                if nonzero(command) {
                    assert!(source_healthy_at(source_ns,segment_ns));
                    let context=worker.delivered_command_context_for_test().cloned().expect("nonzero receipt needs exact result identity");
                    let input=&context.proof.input;
                    assert_eq!((input.epoch,input.goal_version,input.path_version),(lifecycle.epoch,lifecycle.goal,lifecycle.path),"stale lifecycle receipt");
                    let id=input.result_id;
                    assert!(previous_id.is_none_or(|previous|id>previous),"result identity repeated across drain");previous_id=Some(id);
                    receipts.push(RawReceipt{tick:controls,map:map.clone(),boundary:boundary.clone(),pose:planar(&current),before,command,ns:segment_ns,
                        source_sample_ns:source_ns,source_sequence:sequence,pose_stamp_ns:measured.stamp.nanos,
                        worker_result_age_ms:DEFAULT_CONTROL_HORIZON.saturating_sub(original.saturating_duration_since(receipt_at)).as_secs_f64()*1e3,
                        worker_remaining_ms:original.saturating_duration_since(receipt_at).as_secs_f64()*1e3,
                        receipt_pose:crate::command_proof_trace::PoseTrace::capture(&measured),proof_context:Some(context)});
                }
                worker.observe_applied_command(command);controls+=1;
                next_control+=PERIOD;if next_control<Instant::now(){next_control=Instant::now()+PERIOD;}
                if succeeded{break;}
            }
            // Wake on actual 40Hz/20Hz events. No catch-up samples or commands.
            let wake=next_pose.min(next_control);
            if let Some(delay)=wake.checked_duration_since(Instant::now()){std::thread::sleep(delay);}
        }
        let leg_elapsed=leg_start.elapsed();active_time+=leg_elapsed;phase_seconds[phase]+=leg_elapsed.as_secs_f64();
        assert!(leg_elapsed<=Duration::from_secs(60),"goal elapsed beyond original 60s budget");
        assert!(succeeded,"60s full-goal deadline missed: enabled={enabled} phase={} pose={current:?} wait={:?}",PHASES[phase],worker.execution_wait_reason());
        assert!(current.pose.position.distance(goal.pose.position)<0.15);
        assert!(nuwax_nav_astar::controllers::angle_diff(current.pose.orientation.yaw(),goal.pose.orientation.yaw()).abs()<0.15);
        if phase==4 {assert!(replaced&&cancelled,"phase must execute both scheduled lifecycle changes");}
        assert!(!nonzero(command));stop_ack(&mut worker,&stopped);lifecycle.stop();
        let drain_start=Instant::now();
        if memory {
            memory_diagnostic::checkpoint(enabled,completed,PHASES[phase],"A_pre_oracle",true);
            memory_diagnostic::history(enabled,completed,&receipts,&segments,&gaps,&events);
            memory_diagnostic::persistent(enabled,completed,"A_pre_oracle",&probe.lock().unwrap(),&rechecks.lock().unwrap());
        }
        for (ns,kind,index,sign) in &events {eprintln!("SUSTAINED_INPUT leg={completed} phase={} kind={kind} index={index} unix_ns={ns} perturbation_sign={sign}",PHASES[phase]);}
        report_sustained_mismatches(&receipts,completed,PHASES[phase]);
        validate_motion_history(&receipts,&segments,0.05);
        if memory {memory_diagnostic::checkpoint(enabled,completed,PHASES[phase],"B_post_oracle_histories_live",true);}
        assert!(!receipts.is_empty()&&!segments.is_empty());
        assert!(gaps.iter().all(|ms|*ms<=75.0),"consumer gap exceeded unchanged 75ms allowance");
        let distance: f64=segments.iter().map(|s|(s.to.x-s.from.x).hypot(s.to.y-s.from.y)).sum();
        let motion_seconds: f64=segments.iter().map(|s|s.dt).sum();total_motion_seconds+=motion_seconds;
        total_distance+=distance;total_receipts+=receipts.len();total_segments+=segments.len();
        let mut p=probe.lock().unwrap();let mut checks=rechecks.lock().unwrap();
        eprintln!("SUSTAINED_LEG enabled={enabled} leg={completed} phase={} elapsed_s={:.6} poses={poses} controls={controls} receipts={} edges={} travel_m={distance:.6} accepted_motion_s={motion_seconds:.6} gap_ms={} step_ms={} map_ms={} recheck_ms={} profile_overflow={} source_frame_overflow={} source_install_overflow={}",PHASES[phase],leg_elapsed.as_secs_f64(),receipts.len(),segments.len(),measured_distribution(&gaps),measured_distribution(&p.control_ms),measured_distribution(&p.map_update_ms),measured_distribution(&checks),p.control_sample_overflow,p.source_frame_overflow,p.source_install_overflow);
        assert!(p.control_ms.iter().chain(&p.map_update_ms).chain(checks.iter()).all(|ms|*ms<=50.0),"inner step/map/recheck exceeded unchanged 50ms limit");
        assert_eq!(p.control_sample_overflow,0);assert_eq!(p.source_frame_overflow,0);assert_eq!(p.source_install_overflow,0);
        assert!(p.control_samples.len()<=LEG_RECORD_LIMIT);
        assert!(receipts.len()<=MAX_TICKS&&segments.len()<=MAX_TICKS*4+PROFILE_DRAIN_ALLOWANCE,"per-leg evidence exceeded bounded event budget");
        assert!(events.len()<=MAX_TICKS*3&&checks.len()<=MAX_TICKS*4,"per-leg diagnostic budget exceeded");
        // Release snapshot owners and all diagnostic histories before sampling RSS.
        drop(receipts);drop(segments);drop(gaps);drop(events);
        p.control_ms.clear();p.map_update_ms.clear();p.endpoints.clear();p.control_samples.clear();p.source_frames.clear();p.source_installs.clear();checks.clear();
        if memory {memory_diagnostic::persistent(enabled,completed,"C_post_clear",&p,&checks);}
        drop(checks);drop(p);
        if memory {memory_diagnostic::checkpoint(enabled,completed,PHASES[phase],"C_post_drop_clear",true);}
        oracle_time+=drain_start.elapsed();phase_legs[phase]+=1;completed+=1;
        eprintln!("SUSTAINED_DRAIN enabled={enabled} completed_legs={completed} active_s={:.6} oracle_s={:.6} last_result_id={previous_id:?} retained_receipts=0 retained_edges=0 retained_endpoints=0 sensor_dropped={} {}",active_time.as_secs_f64(),oracle_time.as_secs_f64(),sensor.dropped.load(Ordering::Relaxed),rss_status());
        // The sensor remains alive while stopped. Reject stale queued acquisition
        // naturally on the next leg; never change its timestamp or health age.
    }
    let admitted=probe.lock().unwrap().admitted;
    if !enabled {assert_eq!(jobs.load(Ordering::Acquire),0);}
    if !short {assert!(phase_legs.iter().all(|count|*count>0));assert!(replacements>0&&cancellations>0);}
    eprintln!("SUSTAINED_DONE enabled={enabled} short={short} seed={SEED} active_s={:.6} oracle_s={:.6} wall_s={:.6} completed_legs={completed} phase_legs={phase_legs:?} phase_active_s={phase_seconds:?} receipts={total_receipts} edges={total_segments} travel_m={total_distance:.6} accepted_motion_s={total_motion_seconds:.6} held_continuity=separate_existing_7m_continuity_arm admitted={admitted} sensor_frames={frames} replacements={replacements} cancellations={cancellations} final_result_id={previous_id:?} rss_is_not_leak_proof=true",active_time.as_secs_f64(),oracle_time.as_secs_f64(),started.elapsed().as_secs_f64());
    if memory {
        // All acceptance checks precede this terminal teardown. No trim occurs
        // during a leg, between goals, or while sensor/compute owners are alive.
        drop(worker);
        memory_diagnostic::checkpoint(enabled,completed,"terminal","worker_joined",true);
        drop(sensor);
        memory_diagnostic::checkpoint(enabled,completed,"terminal","sensor_joined_queue_dropped",false);
        drop((map,boundary));
        memory_diagnostic::checkpoint(enabled,completed,"terminal","map_dropped",false);
        drop((probe,rechecks,jobs,held,held_phase,stopped));
        memory_diagnostic::checkpoint(enabled,completed,"terminal","all_fixture_owners_dropped_pretrim",false);
        memory_diagnostic::terminal_trim(enabled);
        memory_diagnostic::checkpoint(enabled,completed,"terminal","all_fixture_owners_dropped_posttrim",false);
    }
}

#[test] #[ignore="short persistent OFF five-phase smoke validates bounded oracle drains before endurance"]
fn sustained_worker_short_off(){sustained(false,true);}
#[test] #[ignore="short persistent ON five-phase smoke validates bounded oracle drains and lifecycle replacement"]
fn sustained_worker_short_on(){sustained(true,true);}
#[test] #[ignore="at least 5min live persistent OFF; serial exclusive CPU only"]
fn sustained_worker_5min_off(){sustained(false,false);}
#[test] #[ignore="at least 5min live persistent ON; serial exclusive CPU only"]
fn sustained_worker_5min_on(){sustained(true,false);}

#[test]
fn sustained_plant_accounts_for_receipt_latency_and_caps_original_expiry() {
    let (_,map,_)=fixture(false,0.05,6.0);let boundary=Arc::new(Vec::new());
    let mut current=pose(0.75,1.5,0.0);let start=Instant::now();let mut integrated=start;
    let command=Twist2{linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0};
    let mut edges=Vec::new();let expires=start+Duration::from_millis(50);
    for millis in [12,15,25,50,80] {
        advance_plant(&mut current,&mut integrated,start+Duration::from_millis(millis),command,start,expires,&map,&boundary,0,&mut edges);
    }
    assert!((current.pose.position.x-0.775).abs()<1e-12);
    assert!((edges.iter().map(|s|s.dt).sum::<f64>()-0.05).abs()<1e-12);
    assert_eq!(edges.len(),4,"receipt-latency edge must exist, expired interval must not");
    for edge in edges {edge.map.validate_body_motion(body(edge.from),body(edge.to),&edge.boundary).unwrap();}
}
#[test]
fn sustained_lifecycle_counters_do_not_reset_at_drain() {
    let mut state=Lifecycle::default();state.install();
    assert_eq!((state.epoch,state.goal,state.path),(2,1,2));
    state.stop();state.install();
    assert_eq!((state.epoch,state.goal,state.path),(5,3,5));
    state.install();state.stop();state.install();
    assert_eq!((state.epoch,state.goal,state.path),(10,6,10));
}

// Bounded by one 60s leg and emitted only while stopped, before dropping the
// historical source owners. These rows are provenance, never motion authority.
fn report_sustained_mismatches(receipts:&[RawReceipt],leg:usize,phase:&str) {
    assert!(receipts.len()<=MAX_TICKS);
    let mut counts=[0usize;6];
    for r in receipts {
        let context=r.proof_context.as_ref().expect("retained exact result proof");
        let proof=&context.proof;let guard=&proof.guard;
        let mismatch=[!proof.worker_guard_pose_bits_equal,!proof.worker_guard_pose_stamp_equal,
            !proof.worker_guard_pose_frame_equal,!proof.worker_guard_applied_bits_equal,
            guard.pose.bits!=r.receipt_pose.bits||guard.pose.frame_id!=r.receipt_pose.frame_id,
            guard.applied_bits!=crate::command_proof_trace::command_bits(r.before)];
        for (count,changed) in counts.iter_mut().zip(mismatch){*count+=usize::from(changed);}
        if mismatch.iter().any(|changed|*changed) {
            eprintln!("SUSTAINED_PROOF_MISMATCH leg={leg} phase={phase} tick={} result_id={} component_order=worker_pose_bits,worker_pose_stamp,worker_pose_frame,worker_applied,receipt_pose,receipt_applied mismatch={mismatch:?} receipt_ns={} source_sample_ns={} source_sequence={} receipt_measured_pose={:?} receipt_actual_pose={:?} receipt_applied_bits={:?} receipt_command_bits={:?} context={context:?}",
                r.tick,proof.input.result_id,r.ns,r.source_sample_ns,r.source_sequence,r.receipt_pose,r.pose,
                crate::command_proof_trace::command_bits(r.before),crate::command_proof_trace::command_bits(r.command));
        }
    }
    eprintln!("SUSTAINED_PROOF_COMPONENTS leg={leg} phase={phase} worker_pose_bits={} worker_pose_stamp={} worker_pose_frame={} worker_applied={} receipt_pose={} receipt_applied={} historical_input_is_not_rewritten_after_current_state_catchup=true",
        counts[0],counts[1],counts[2],counts[3],counts[4],counts[5]);
}

// 使用真实原生扫掠、异步计算线程与调度器，只在计算入口注入耗时。
use super::*;
use std::sync::atomic::{AtomicBool, AtomicUsize, AtomicU64, Ordering};

// 测试只订阅现有来源事件，不增加依赖、准入条件或行为门禁。
struct CommandDiagnostics { origin:std::time::Instant, next_span:AtomicU64 }
impl tracing::Subscriber for CommandDiagnostics {
    fn enabled(&self,m:&tracing::Metadata<'_>)->bool {
        matches!(m.target(),"navigation.command"|"navigation.local")
    }
    fn new_span(&self,_:&tracing::span::Attributes<'_>)->tracing::span::Id {
        tracing::span::Id::from_u64(self.next_span.fetch_add(1,Ordering::Relaxed)+1)
    }
    fn record(&self,_:&tracing::span::Id,_:&tracing::span::Record<'_>) {}
    fn record_follows_from(&self,_:&tracing::span::Id,_:&tracing::span::Id) {}
    fn enter(&self,_:&tracing::span::Id) {}
    fn exit(&self,_:&tracing::span::Id) {}
    fn event(&self,event:&tracing::Event<'_>) {
        struct Fields(String);
        impl tracing::field::Visit for Fields {
            fn record_debug(&mut self,f:&tracing::field::Field,v:&dyn std::fmt::Debug) {
                use std::fmt::Write as _;
                let _=write!(self.0," {}={v:?}",f.name());
            }
        }
        let mut fields=Fields(String::new());event.record(&mut fields);
        eprintln!("COMMAND_DIAGNOSTIC at_ms={}{}",self.origin.elapsed().as_secs_f64()*1000.,fields.0);
    }
}

struct SlowSurface {
    inner: SurfaceFollower,
    calls: usize,
    slow: Arc<AtomicBool>,
    native_max_ns: Arc<AtomicU64>,
    rechecks: Arc<AtomicUsize>,
    observed_frames: Arc<AtomicUsize>,
    last_frame: Option<RegisteredFrameIdentity>,
}
impl NavPlanner for SlowSurface {
    fn set_control_period(&mut self,p:Duration) {self.inner.set_control_period(p);}
    fn configure_goal(&mut self,p:&PoseStamped,o:nuwax_nav_types_messages::GoalOptions)->Result<(),RuntimeError> {self.inner.configure_goal(p,o)}
    fn configure_boundary(&mut self,b:&[[f64;2]]) {self.inner.configure_boundary(b);}
    fn replace_path(&mut self,p:Path2D) {self.inner.replace_path(p);}
    fn replace_execution_path(&mut self,p:ExecutionPath,y:Option<Vec<f64>>,g:PoseStamped)->Result<(),RuntimeError> {self.inner.replace_execution_path(p,y,g)}
    fn update_odometry(&mut self,p:PoseStamped) {self.inner.update_odometry(p);}
    fn update_costmap(&mut self,m:CostMap2D) {self.inner.update_costmap(m);}
    fn update_execution_map(&mut self,m:ExecutionMap)->Result<(),RuntimeError> {self.inner.update_execution_map(m)}
    fn observe_applied_command(&mut self,c:Twist2) {self.inner.observe_applied_command(c);}
    fn observe_measured_motion(&mut self,m:Option<MeasuredMotion>) {self.inner.observe_measured_motion(m);}
    fn observe_registered_frame(&mut self,frame:Option<Arc<RegisteredFrame>>) {
        let identity=frame.as_ref().map(|f|f.identity.clone());
        if identity!=self.last_frame {
            if identity.is_some() {self.observed_frames.fetch_add(1,Ordering::Relaxed);}
            self.last_frame=identity;
        }
        self.inner.observe_registered_frame(frame);
    }
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {self.inner.take_command_dispatch_bound()}
    fn take_command_execution_window(&mut self)->Option<(u64,Duration)> {self.inner.take_command_execution_window()}
    fn execution_wait_reason(&self)->Option<String> {self.inner.execution_wait_reason()}
    fn step(&mut self,w:f64)->PlannerStep {
        self.calls+=1;
        let started=std::time::Instant::now();
        let step=self.inner.step(w);
        if self.slow.load(Ordering::Relaxed) {
            self.native_max_ns.fetch_max(started.elapsed().as_nanos() as u64,Ordering::Relaxed);
            // 总计算周期周期性为70毫秒，包含原算法耗时；原算法本身不能被缩短。
            let period=Duration::from_millis(if self.calls%3==0 {70}else{8});
            std::thread::sleep(period.saturating_sub(started.elapsed()));
        }
        step
    }
    fn recheck_command(&mut self,w:f64,command:Twist2)->Option<PlannerStep> {
        self.rechecks.fetch_add(1,Ordering::Relaxed);
        let started=std::time::Instant::now();
        let step=self.inner.recheck_command(w,command);
        self.native_max_ns.fetch_max(started.elapsed().as_nanos() as u64,Ordering::Relaxed);
        step
    }
    fn stop(&mut self) {self.inner.stop();}
}

struct ContinuousMotion {
    inner: FakeMotion,
    command: Arc<Mutex<Twist2>>,
    measured: Arc<Mutex<MeasuredMotion>>,
    output: Arc<Mutex<Vec<(std::time::Instant,Twist2)>>>,
}
#[async_trait]
impl NavMotionPort for ContinuousMotion {
    fn measured_motion(&self)->Option<MeasuredMotion> {Some(self.measured.lock().unwrap().clone())}
    async fn acquire(&mut self,id:&str,a:&NavMotionAdmission)->Result<LeaseProof,MotionPortError> {self.inner.acquire(id,a).await}
    async fn renew(&mut self,id:&str)->Result<(),MotionPortError> {self.inner.renew(id).await}
    async fn submit_twist(&mut self,id:&str,c:Twist2)->Result<AppliedProof,MotionPortError> {
        let proof=self.inner.submit_twist(id,c).await?;
        *self.command.lock().unwrap()=c;
        self.output.lock().unwrap().push((std::time::Instant::now(),c));
        Ok(proof)
    }
    async fn submit_twist_bounded(&mut self,id:&str,c:Twist2,b:Option<CommandDispatchBound>)->Result<BoundedTwistOutcome,MotionPortError> {
        if b.is_some_and(CommandDispatchBound::expired_now) {return Ok(BoundedTwistOutcome::ExpiredNotApplied);}
        self.submit_twist(id,c).await.map(BoundedTwistOutcome::Applied)
    }
    async fn revoke_and_zero(&mut self,id:&str)->Result<FenceProof,MotionPortError> {
        *self.command.lock().unwrap()=Twist2::ZERO;
        self.output.lock().unwrap().push((std::time::Instant::now(),Twist2::ZERO));
        self.inner.revoke_and_zero(id).await
    }
}

// 同一执行器采样冻结点云、原点和机体位姿，保留独立注册输入身份。
fn continuity_frame(pose:&PoseStamped,sequence:i64)->Arc<RegisteredFrame> {
    let mut frame=(*crate::registered_frame::test_frame(sequence)).clone();
    let p=pose.pose.position;let q=pose.pose.orientation;
    frame.identity.timestamp_ns=pose.stamp.nanos;
    frame.body_position_world=[p.x,p.y,p.z];frame.sensor_origin_world=[p.x,p.y,p.z+0.35];
    frame.body_orientation_xyzw=[q.x,q.y,q.z,q.w];
    frame.points_world=vec![[p.x,p.y,0.05]].into();frame.ground_point_indices=vec![0].into();
    frame.paired_odometry_sequence=sequence;frame.paired_odometry_timestamp_ns=pose.stamp.nanos;
    Arc::new(frame)
}

#[tokio::test]
async fn command_continuity_real_surface_at_point_three_rate() {
    use std::os::unix::fs::FileExt;
    use nuwax_nav_types_messages::{Point3,Quaternion,Timestamp};
    // 来源时钟为进程环境变量，沿现有测试方式用子进程隔离。
    if std::env::var_os("NUWAX_CONTINUITY_CHILD").is_none() {
        let dir=std::env::temp_dir().join(format!("行走连续性-{}",std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut results=Vec::new();
        // 保留快速位姿回归，再按录制的10赫兹来源位姿周期验证实际低频输入。
        for period in [100,334] {
            let clock=dir.join(format!("来源时钟-{period}.bin"));
            let result=std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact","tests::command_continuity::command_continuity_real_surface_at_point_three_rate","--nocapture"])
                .env("NUWAX_CONTINUITY_CHILD","1").env("NUWAX_CONTINUITY_POSE_MS",period.to_string())
                .env("NUWAX_SOURCE_TIME_FILE",&clock).status().unwrap();
            results.push(result);
        }
        std::fs::remove_dir_all(dir).unwrap();
        assert!(results.iter().all(|r|r.success()));return;
    }
    // 只订阅既有诊断，零段失败时保留实际原因；日志初始化不作为行为门禁。
    let _=tracing::subscriber::set_global_default(CommandDiagnostics{
        origin:std::time::Instant::now(),next_span:AtomicU64::new(0)});
    let pose_period_ms=std::env::var("NUWAX_CONTINUITY_POSE_MS").unwrap().parse::<u64>().unwrap();
    let file=Arc::new(std::fs::OpenOptions::new().create(true).truncate(true).read(true).write(true)
        .open(std::env::var_os("NUWAX_SOURCE_TIME_FILE").unwrap()).unwrap());
    file.set_len(16+16384*32).unwrap();file.write_all_at(b"NEHTIME1",0).unwrap();
    let source_origin=crate::command_dispatch::unix_ns().unwrap()-10_000_000_000;
    let origin=std::time::Instant::now();
    let source_now=move ||source_origin+(origin.elapsed().as_secs_f64()*0.3*1e9)as u64;
    let write_clock=move |seq:u64,stamp:u64| {
        let row=[seq,stamp,0,crate::motion_feedback::monotonic_ns()];
        let bytes:Vec<_>=row.into_iter().flat_map(u64::to_le_bytes).collect();
        file.write_all_at(&bytes,16+(seq-1)%16384*32).unwrap();
        file.write_all_at(&seq.to_le_bytes(),8).unwrap();
    };
    write_clock(1,source_now());
    let (config,map)=crate::native_test_helpers::fixture();
    let slow=Arc::new(AtomicBool::new(false));
    let native_max_ns=Arc::new(AtomicU64::new(0));
    let rechecks=Arc::new(AtomicUsize::new(0));
    let observed_frames=Arc::new(AtomicUsize::new(0));
    let follower=SlowSurface{inner:SurfaceFollower::new(config).unwrap(),calls:0,slow:slow.clone(),native_max_ns:native_max_ns.clone(),rechecks:rechecks.clone(),observed_frames:observed_frames.clone(),last_frame:None};
    let worker=planner_worker::PlannerWorker::new(follower).unwrap();
    let command=Arc::new(Mutex::new(Twist2::ZERO));
    let mut motion=crate::motion_feedback::tests::measured([0.;3],[0.;3]);
    motion.source_timestamp_ns=source_now();
    let measured=Arc::new(Mutex::new(motion));let output=Arc::new(Mutex::new(Vec::new()));
    let port=ContinuousMotion{inner:FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
        command:command.clone(),measured:measured.clone(),output:output.clone()};
    let mut scheduler=NavScheduler::new(port,worker,SchedulerConfig::default()).unwrap();
    let mut start=map_pose();start.pose.position=Point3::new(0.65,1.05,0.45);
    start.pose.orientation=Quaternion::from_yaw(0.);start.stamp=Timestamp {nanos:source_now()as i64,clock:nuwax_nav_types_messages::ClockKind::Unix};
    let mut goal=start.clone();goal.pose.position.x=4.5;
    let mut request=session();request.goal=ExecutionGoal::Body(goal.clone());
    let (status,_)=watch::channel(scheduler.status.clone());
    scheduler.observe_odometry(&status,start.clone(),None).unwrap();
    scheduler.observe_costmap(&status,CostmapObservation{costmap:ExecutionMap::Surface(map),source_timestamp_ns:source_now()as i64},Instant::now()).unwrap();
    scheduler.planner.observe_registered_frame(Some(continuity_frame(&start,1)));
    scheduler.start_session(&status,request).await.unwrap();
    let mut route=path();route.path=ExecutionPath::Surface(Path3D{poses:vec![start.clone(),goal]});
    scheduler.submit_path(&status,route).await.unwrap();
    let (handle,task)=spawn_scheduler(scheduler);
    let running=Arc::new(AtomicBool::new(true));
    let samples=Arc::new(AtomicUsize::new(0));
    let producer={let running=running.clone();let measured=measured.clone();let samples=samples.clone();
        let handle=handle.clone();let command=command.clone();
        std::thread::spawn(move || {
            let mut seq=1;let mut pose=start;let mut last=std::time::Instant::now();let mut last_pose=last;let mut last_motion=last;let mut last_frame=last;
            let mut actual=Twist2::ZERO;let mut yaw=0.0_f64;
            while running.load(Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(5));seq+=1;
                let stamp=source_now();write_clock(seq,stamp);
                let now=std::time::Instant::now();let dt=now.duration_since(last).as_secs_f64()*0.3;last=now;
                let c=*command.lock().unwrap();
                // 独立执行器按原物理加速度响应；反馈取积分状态，不把接纳回执当测速。
                let ramp=|from:f64,to:f64,step:f64|from+(to-from).clamp(-step,step);
                let next=Twist2{linear:Point2::new(ramp(actual.linear.x,c.linear.x,0.5*dt),
                    ramp(actual.linear.y,c.linear.y,0.5*dt)),angular_z:ramp(actual.angular_z,c.angular_z,dt)};
                let vx=(actual.linear.x+next.linear.x)*0.5;let vy=(actual.linear.y+next.linear.y)*0.5;
                pose.pose.position.x+=(yaw.cos()*vx-yaw.sin()*vy)*dt;
                pose.pose.position.y+=(yaw.sin()*vx+yaw.cos()*vy)*dt;
                yaw+=(actual.angular_z+next.angular_z)*0.5*dt;
                pose.pose.orientation=Quaternion::from_yaw(yaw);actual=next;
                if last_motion.elapsed()>=Duration::from_millis(67) {
                    let mut m=measured.lock().unwrap();m.source_timestamp_ns=stamp;
                    m.received_monotonic_ns=crate::motion_feedback::monotonic_ns();m.valid_until_monotonic_ns=m.received_monotonic_ns+200_000_000;
                    m.body_linear_xyz=[actual.linear.x,actual.linear.y,0.];m.body_angular_xyz=[0.,0.,actual.angular_z];
                    m.orientation_wxyz=Some([(yaw/2.).cos(),0.,0.,(yaw/2.).sin()]);last_motion=now;
                }
                if last_pose.elapsed()>=Duration::from_millis(pose_period_ms) {
                    pose.stamp=Timestamp {nanos:stamp as i64,clock:nuwax_nav_types_messages::ClockKind::Unix};
                    let (reply,_)=oneshot::channel();
                    if handle.input_tx.blocking_send(InputCommand{kind:InputKind::UpdateOdometry(pose.clone(),None,Instant::now()),reply}).is_err(){break;}
                    samples.fetch_add(1,Ordering::Relaxed);last_pose=now;
                }
                if last_frame.elapsed()>=Duration::from_millis(334) {
                    let mut frozen=pose.clone();frozen.stamp.nanos=stamp as i64;
                    let frame=continuity_frame(&frozen,seq as i64);
                    let (reply,_)=oneshot::channel();
                    if handle.input_tx.blocking_send(InputCommand{kind:InputKind::UpdateObstacleSource(None,true,None,Some(Some(frame))),reply}).is_err(){break;}
                    last_frame=now;
                }
            }
        })};
    let deadline=Instant::now()+Duration::from_secs(3);
    while handle.status().last_applied_twist.is_none_or(|c|c==Twist2::ZERO) {
        assert!(Instant::now()<deadline,"真实原生规划未起步: {:?}",handle.status());
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    eprintln!("WALK_START {:?}",handle.status().last_applied_twist);
    let walking_started=std::time::Instant::now();
    output.lock().unwrap().clear();slow.store(true,Ordering::Relaxed);
    tokio::time::sleep(Duration::from_secs(if pose_period_ms==100 {3}else{8})).await;
    let walking=output.lock().unwrap().clone();
    let snapshot=handle.status();
    let cancel_start=std::time::Instant::now();handle.cancel("goal-1").await.unwrap();let cancel_elapsed=cancel_start.elapsed();
    let stopped=*command.lock().unwrap();
    running.store(false,Ordering::Relaxed);producer.join().unwrap();
    handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    for (i,(at,c)) in walking.iter().enumerate().filter(|(_,(_,c))|*c==Twist2::ZERO) {
        eprintln!("CONTINUITY_ZERO at_s={} previous={:?}",at.duration_since(walking_started).as_secs_f64(),i.checked_sub(1).map(|j|walking[j].1));
    }
    let zeros=walking.iter().filter(|(_,c)|*c==Twist2::ZERO).count();
    eprintln!("CONTINUITY rate=0.3 pose_period_wall_ms={pose_period_ms} commands={} zeros={zeros} pose_samples={} cancel_ms={} native_max_ms={} status={}",walking.len(),samples.load(Ordering::Relaxed),cancel_elapsed.as_secs_f64()*1000.,native_max_ns.load(Ordering::Relaxed) as f64/1e6,snapshot.message);
    eprintln!("CONTINUITY_RECHECKS {} FRAMES {}",rechecks.load(Ordering::Relaxed),observed_frames.load(Ordering::Relaxed));
    assert!(observed_frames.load(Ordering::Relaxed)>2,"必须实际消费新注册帧");
    assert!(walking.len()>10);assert!(samples.load(Ordering::Relaxed)>20);
    assert!(rechecks.load(Ordering::Relaxed)>0,"必须实际执行原生命令复核");
    assert_eq!(zeros,0,"开阔直线行进出现短暂零段");
    assert_eq!(stopped,Twist2::ZERO);assert!(cancel_elapsed<Duration::from_millis(20));
}

// 此夹具固定一拍已证明命令，只用阻塞计算区分“没有新结果”和明确停车结果。
struct WindowPlanner {
    pose: PoseStamped,
    window: Option<(u64,Duration)>,
    first: bool,
    applied: Twist2,
    following: Option<PlannerStep>,
    rechecked: Arc<Mutex<Vec<Twist2>>>,
    bound_lifetime: Option<Duration>,
    initial_bound: Option<CommandDispatchBound>,
    bound: Option<CommandDispatchBound>,
    entered: std::sync::mpsc::Sender<()>,
    release: std::sync::mpsc::Receiver<PlannerStep>,
}
impl NavPlanner for WindowPlanner {
    fn replace_path(&mut self,_:Path2D) {}
    fn update_odometry(&mut self,p:PoseStamped) {self.pose=p;}
    fn update_costmap(&mut self,_:CostMap2D) {}
    fn observe_applied_command(&mut self,command:Twist2) {self.applied=command;}
    fn take_command_execution_window(&mut self)->Option<(u64,Duration)> {self.window.take()}
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {self.bound.take()}
    fn step(&mut self,_:f64)->PlannerStep {
        if self.first {
            self.first=false;
            self.initial_bound=self.bound_lifetime.map(|age|handoff_bound(age.as_nanos() as u64));
        }
        if self.applied==Twist2::ZERO {
            // 原接纳回执真正到达后才阻塞下一轮，重复初始查询不刷新原截止。
            self.bound=self.initial_bound;
            self.window=Some((self.pose.stamp.nanos as u64,Duration::from_millis(50)));
            PlannerStep::Command(delivery_gap_command())
        } else if let Some(step)=&self.following {
            step.clone()
        } else {
            self.entered.send(()).unwrap();
            let step=self.release.recv_timeout(Duration::from_secs(2)).expect("释放本次计算");
            self.following=Some(step.clone());
            step
        }
    }
    fn recheck_command(&mut self,_:f64,command:Twist2)->Option<PlannerStep> {
        self.rechecked.lock().unwrap().push(command);
        Some(PlannerStep::Command(command))
    }
    fn stop(&mut self) {}
}
async fn window_fixture(bound_lifetime:Option<Duration>)->(NavScheduler<ContinuousMotion,planner_worker::PlannerWorker>,watch::Sender<SchedulerStatus>,std::sync::mpsc::Sender<PlannerStep>,Arc<Mutex<Vec<Twist2>>>) {
    let (entered,starts)=std::sync::mpsc::channel();let (release,finish)=std::sync::mpsc::channel();
    let mut pose=map_pose();pose.stamp.nanos=crate::command_dispatch::unix_ns().unwrap()as i64;
    let rechecked=Arc::new(Mutex::new(Vec::new()));
    let planner=WindowPlanner{pose:pose.clone(),window:None,first:true,applied:Twist2::ZERO,following:None,rechecked:rechecked.clone(),bound_lifetime,initial_bound:None,bound:None,entered,release:finish};
    let worker=planner_worker::PlannerWorker::new(planner).unwrap();
    let port=ContinuousMotion{inner:FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
        command:Arc::new(Mutex::new(Twist2::ZERO)),measured:Arc::new(Mutex::new(crate::motion_feedback::tests::measured([0.;3],[0.;3]))),output:Arc::new(Mutex::new(Vec::new()))};
    let mut scheduler=NavScheduler::new(port,worker,SchedulerConfig::default()).unwrap();
    let (status,_)=watch::channel(scheduler.status.clone());
    scheduler.observe_odometry(&status,pose,None).unwrap();scheduler.observe_costmap(&status,costmap_at(1),Instant::now()).unwrap();
    scheduler.start_session(&status,session()).await.unwrap();scheduler.submit_path(&status,path()).await.unwrap();
    let deadline=Instant::now()+Duration::from_secs(1);
    while scheduler.status.last_applied_twist!=Some(delivery_gap_command()) {
        assert!(Instant::now()<deadline);
        scheduler.control_tick(&status).await.unwrap();tokio::task::yield_now().await;
    }
    starts.recv_timeout(Duration::from_secs(1)).unwrap();
    (scheduler,status,release,rechecked)
}

#[tokio::test]
async fn command_continuity_original_period_expires_without_reissue() {
    let (mut scheduler,status,release,_)=window_fixture(None).await;
    let before=scheduler.motion.output.lock().unwrap().len();
    scheduler.control_tick(&status).await.unwrap();
    assert_eq!(scheduler.motion.output.lock().unwrap().len(),before,"保留原命令不能重发或续期");
    assert_eq!(scheduler.status.last_applied_twist,Some(delivery_gap_command()));
    tokio::time::sleep(Duration::from_millis(55)).await;
    scheduler.control_tick(&status).await.unwrap();
    assert_eq!(scheduler.status.last_applied_twist,Some(Twist2::ZERO),"原50毫秒证明周期耗尽必须归零");
    release.send(PlannerStep::Pending(Vec::new())).unwrap();
    drop(release);scheduler.planner.stop();
}

#[tokio::test]
async fn command_continuity_retained_wait_delivers_ready_result_once() {
    let (mut scheduler,status,release,_)=window_fixture(None).await;
    let before=scheduler.status.applied_commands;
    scheduler.control_tick(&status).await.unwrap();
    let mut publication=scheduler.retained_command_publication.take().expect("保持期间订阅下一结果");
    release.send(PlannerStep::Command(delivery_gap_command())).unwrap();
    publication.changed().await.unwrap();
    assert!(scheduler.planner.prepare_control_step(0.3,true).is_none(),"已就绪结果必须立即交付，不能反复订阅");
    scheduler.control_tick(&status).await.unwrap();
    assert_eq!(scheduler.status.applied_commands,before+1);
    assert_eq!(*scheduler.motion.command.lock().unwrap(),delivery_gap_command());
    drop(release);scheduler.planner.stop();
}

#[tokio::test]
async fn command_continuity_map_change_and_stop_results_override_active_window() {
    for outcome in [PlannerStep::Command(Twist2::ZERO),PlannerStep::Pending(Vec::new()),
        PlannerStep::ReplanRequired("碰撞与制动扫掠失败".into()),PlannerStep::Failed("计算失败".into()),PlannerStep::Succeeded] {
        let (mut scheduler,status,release,_)=window_fixture(None).await;
        let mut publication=scheduler.planner.prepare_control_step(0.3,false).unwrap();
        assert!(matches!(scheduler.planner.step(0.3),PlannerStep::Pending(_)));
        assert!(scheduler.planner.retain_applied_command());
        release.send(outcome.clone()).unwrap();
        publication.changed().await.unwrap();
        assert!(!scheduler.planner.retain_applied_command(),"迟到停车结果必须覆盖已读取的空档: {outcome:?}");
        scheduler.control_tick(&status).await.unwrap();
        let applied=*scheduler.motion.command.lock().unwrap();
        drop(release);scheduler.planner.stop();
        assert_eq!(applied,Twist2::ZERO,"真实停车结论必须覆盖原命令: {outcome:?}");
    }
    let (mut scheduler,status,release,_)=window_fixture(None).await;
    scheduler.observe_costmap(&status,costmap_at(2),Instant::now()).unwrap();
    scheduler.control_tick(&status).await.unwrap();
    assert_eq!(*scheduler.motion.command.lock().unwrap(),Twist2::ZERO,"未复核新地图不能保持旧命令");
    release.send(PlannerStep::Pending(Vec::new())).unwrap();
    drop(release);scheduler.planner.stop();
}


#[tokio::test]
async fn command_continuity_observation_expiry_and_guard_rejection_still_zero() {
    let (mut scheduler,status,release,_)=window_fixture(Some(Duration::from_millis(25))).await;
    scheduler.control_tick(&status).await.unwrap();
    assert_eq!(scheduler.status.last_applied_twist,Some(delivery_gap_command()));
    tokio::time::sleep(Duration::from_millis(28)).await;
    scheduler.control_tick(&status).await.unwrap();
    assert_eq!(*scheduler.motion.command.lock().unwrap(),Twist2::ZERO,"来源原截止不能被新的执行周期续期");
    release.send(PlannerStep::Pending(Vec::new())).unwrap();
    drop(release);scheduler.planner.stop();

    let (mut scheduler,status,release,_)=window_fixture(None).await;
    let mut publication=scheduler.planner.prepare_control_step(0.3,false).unwrap();
    scheduler.motion.inner.0.lock().unwrap().fail_twist=true;
    release.send(PlannerStep::Command(delivery_gap_command())).unwrap();
    publication.changed().await.unwrap();
    assert!(scheduler.control_tick(&status).await.is_err());
    assert_eq!(*scheduler.motion.command.lock().unwrap(),Twist2::ZERO,"守卫拒绝必须沿原失败停车路径归零");
    drop(release);scheduler.planner.stop();
}


#[tokio::test]
async fn command_continuity_rechecks_selected_candidate_only_for_original_applied_version() {
    for changed_applied in [false,true] {
        let (mut scheduler,status,release,rechecked)=window_fixture(None).await;
        let mut publication=scheduler.planner.prepare_control_step(0.3,false).unwrap();
        let mut pose=map_pose();pose.pose.position.x=0.01;
        pose.stamp.nanos=crate::command_dispatch::unix_ns().unwrap()as i64;
        scheduler.observe_odometry(&status,pose,None).unwrap();
        if changed_applied {
            let next=Twist2{linear:Point2::new(0.15,0.),angular_z:0.};
            scheduler.apply_controller_twist(&status,next).await.unwrap();
        }
        let selected=Twist2{linear:Point2::new(0.2,0.),angular_z:0.};
        release.send(PlannerStep::Command(selected)).unwrap();
        publication.changed().await.unwrap();
        let rechecked=rechecked.lock().unwrap().clone();
        scheduler.control_tick(&status).await.unwrap();
        drop(release);scheduler.planner.stop();
        if changed_applied {
            assert!(rechecked.is_empty(),"真实执行版本已变时不能复核旧候选");
        } else {
            assert_eq!(rechecked,vec![selected],"应复核已选新指令，不能占用交付拍维持旧速度");
        }
    }
}


#[tokio::test]
async fn command_continuity_recheck_preserves_original_selection_horizon() {
    let (mut scheduler,_,release,rechecked)=window_fixture(None).await;
    let mut publication=scheduler.planner.prepare_control_step(0.3,false).unwrap();
    let mut pose=map_pose();pose.pose.position.x=0.01;
    scheduler.planner.update_odometry(pose);
    tokio::time::sleep(Duration::from_millis(180)).await;
    release.send(PlannerStep::Command(delivery_gap_command())).unwrap();
    publication.changed().await.unwrap();
    assert_eq!(rechecked.lock().unwrap().len(),1);
    tokio::time::sleep(Duration::from_millis(90)).await;
    let step=scheduler.planner.step(0.3);
    drop(release);scheduler.planner.stop();
    assert!(matches!(step,PlannerStep::Pending(_)),"复核不能重置原250毫秒许可: {step:?}");
}

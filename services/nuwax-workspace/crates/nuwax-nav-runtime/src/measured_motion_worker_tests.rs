// 使用真实 PlannerWorker，只阻塞合成计算；不授予几何许可。
struct MeasuredWorkerProbe {
    measured: Option<MeasuredMotion>,
    applied: Twist2,
    entered: std::sync::mpsc::Sender<(Option<MeasuredMotion>,Twist2)>,
    release: std::sync::mpsc::Receiver<()>,
}
impl NavPlanner for MeasuredWorkerProbe {
    fn replace_path(&mut self,_:Path2D) {}
    fn update_odometry(&mut self,_:PoseStamped) {}
    fn update_costmap(&mut self,_:CostMap2D) {}
    fn observe_measured_motion(&mut self,m:Option<MeasuredMotion>) {self.measured=m;}
    fn observe_applied_command(&mut self,c:Twist2) {self.applied=c;}
    fn step(&mut self,_:f64)->PlannerStep {
        self.entered.send((self.measured.clone(),self.applied)).unwrap();
        self.release.recv_timeout(Duration::from_secs(3)).unwrap();
        PlannerStep::Command(Twist2{linear:nuwax_nav_types_messages::Point2::new(
            self.measured.as_ref().map_or(0.01,|m|m.body_linear_xyz[0]),0.),angular_z:0.})
    }
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {
        self.measured.as_ref().map(|sample|CommandDispatchBound::from_same_host_monotonic_deadline(
            sample.valid_until_monotonic_ns))
    }
    fn stop(&mut self) {}
}
fn measured_probe()->(PlannerWorker,std::sync::mpsc::Receiver<(Option<MeasuredMotion>,Twist2)>,std::sync::mpsc::Sender<()>) {
    let (entered,start)=std::sync::mpsc::channel();let (release,finish)=std::sync::mpsc::channel();
    (PlannerWorker::new(MeasuredWorkerProbe{measured:None,applied:Twist2::ZERO,entered,release:finish}).unwrap(),start,release)
}
#[test]
fn measured_change_without_pose_or_command_change_retires_inflight_result() {
    let (mut worker,started,release)=measured_probe();
    let mut a=crate::motion_feedback::tests::measured([0.1,0.,0.],[0.;3]);
    // 合成计算允许等待，保留原测试样本的固定有效期限。
    a.valid_until_monotonic_ns+=5_000_000_000;
    let applied=Twist2{linear:nuwax_nav_types_messages::Point2::new(0.8,0.),angular_z:0.};
    worker.observe_applied_command(applied);worker.observe_measured_motion(Some(a.clone()));worker.step(0.3);
    let first=started.recv_timeout(Duration::from_secs(1)).unwrap();
    assert_eq!(first,(Some(a.clone()),applied));
    let mut b=a.clone();b.source_timestamp_ns+=1;b.body_linear_xyz[0]=0.2;
    worker.observe_measured_motion(Some(b.clone()));release.send(()).unwrap();
    let second=started.recv_timeout(Duration::from_secs(1)).unwrap();assert_eq!(second,(Some(b),applied));
    assert!(worker.shared.0.lock().unwrap().result.is_none(),"旧测速结果不得晚到发布");
    release.send(()).unwrap();wait_for_source_result(&worker);
    assert!(matches!(worker.step(0.3),PlannerStep::Command(c) if c.linear.x==0.2));
    worker.stop();
}
#[test]
fn motion_loss_and_recovery_never_substitute_the_applied_command() {
    let (mut worker,started,release)=measured_probe();
    let applied=Twist2{linear:nuwax_nav_types_messages::Point2::new(0.8,0.),angular_z:0.};
    worker.observe_applied_command(applied);worker.step(0.3);
    let sample=started.recv_timeout(Duration::from_secs(1)).unwrap();assert_eq!(sample,(None,applied));
    release.send(()).unwrap();wait_for_source_result(&worker);let _=worker.step(0.3);
    let mut measured=crate::motion_feedback::tests::measured([0.2,0.,0.],[0.;3]);
    measured.valid_until_monotonic_ns+=5_000_000_000;
    worker.observe_measured_motion(Some(measured.clone()));
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap(),(Some(measured),applied));
    release.send(()).unwrap();wait_for_source_result(&worker);let _=worker.step(0.3);
    worker.observe_measured_motion(None);
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap(),(None,applied));
    release.send(()).unwrap();wait_for_source_result(&worker);
    assert!(matches!(worker.step(0.3),PlannerStep::Command(c) if c.linear.x==0.01));worker.stop();
}

/// 生产异步时序回归：20 Hz 的同物理状态新观测不应无条件饿死
/// 一次 170 ms 的初次计算；保留首个样本的原有效期限，不扩控制预算。
#[test]
fn timestamp_only_measurement_refresh_during_slow_compute_must_not_starve_result() {
    let (mut worker,started,release)=measured_probe();
    let first=crate::motion_feedback::tests::measured([0.1,0.,0.],[0.;3]);
    worker.observe_measured_motion(Some(first.clone()));worker.step(0.3);
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap().0,Some(first.clone()));
    let initial_version=worker.shared.0.lock().unwrap().motion_version;
    for index in 1..=3 {
        std::thread::sleep(Duration::from_millis(50));
        let mut refreshed=first.clone();
        refreshed.source_timestamp_ns+=index*50_000_000;
        refreshed.received_monotonic_ns=crate::motion_feedback::monotonic_ns();
        // This regression deliberately keeps the original deadline fixed.
        worker.observe_measured_motion(Some(refreshed));
    }
    std::thread::sleep(Duration::from_millis(20));
    let final_version=worker.shared.0.lock().unwrap().motion_version;
    release.send(()).unwrap();
    // If the original result was retired, release the following queued compute
    // before assertion so the worker can shut down without a blocked test thread.
    let restarted=started.recv_timeout(Duration::from_millis(20)).is_ok();
    if restarted { release.send(()).unwrap(); }
    let published_first={worker.shared.0.lock().unwrap().result.is_some()};
    let latest=worker.shared.0.lock().unwrap().measured_motion.clone().unwrap();
    let selected=if !restarted {Some(worker.step(0.3))} else {None};
    worker.stop();
    assert_eq!(latest.source_timestamp_ns,first.source_timestamp_ns+150_000_000);
    assert_eq!(latest.valid_until_monotonic_ns,first.valid_until_monotonic_ns);
    assert!(matches!(selected,Some(PlannerStep::Command(c)) if c.linear.x==0.1));
    assert_eq!(final_version,initial_version,"仅时间改变却使相同状态计算退休");
    assert!(published_first&&!restarted,"同状态新时点导致慢首帧结果被饿死");
}

#[test]
fn fresh_same_state_cannot_renew_the_consumed_samples_deadline() {
    let (mut worker,started,release)=measured_probe();
    let mut old=crate::motion_feedback::tests::measured([0.1,0.,0.],[0.;3]);
    old.valid_until_monotonic_ns=old.received_monotonic_ns+100_000_000;
    worker.observe_measured_motion(Some(old.clone()));worker.step(0.3);
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap().0,Some(old.clone()));
    let version=worker.shared.0.lock().unwrap().motion_version;
    std::thread::sleep(Duration::from_millis(50));
    let mut fresh=old.clone();fresh.source_timestamp_ns+=50_000_000;
    fresh.received_monotonic_ns=crate::motion_feedback::monotonic_ns();
    fresh.valid_until_monotonic_ns=fresh.received_monotonic_ns+1_000_000_000;
    worker.observe_measured_motion(Some(fresh.clone()));
    std::thread::sleep(Duration::from_millis(80));
    release.send(()).unwrap();wait_for_source_result(&worker);
    {
        let input=worker.shared.0.lock().unwrap();
        assert_eq!(input.motion_version,version);
        assert_eq!(input.measured_motion.as_ref(),Some(&fresh));
        assert!(fresh.is_current());
        assert!(matches!(input.result.as_ref().map(|v|&v.1),Some(PlannerStep::Pending(_))),
            "新同状态样本不能替旧消费样本延长命令许可");
    }
    worker.stop();
}

#[test]
fn motion_boundary_identity_axes_orientation_and_presence_still_version() {
    for change in 0..8 {
        let (mut worker,_,_)=measured_probe();
        let old=crate::motion_feedback::tests::measured([0.1,0.,0.],[0.;3]);
        worker.observe_measured_motion(Some(old.clone()));
        let version=worker.shared.0.lock().unwrap().motion_version;
        let mut changed=old.clone();changed.source_timestamp_ns+=1;
        match change {
            0=>changed.body_linear_xyz[1]=0.2,
            1=>changed.body_angular_xyz[0]=0.3,
            2=>changed.orientation_wxyz=Some([0.999,0.01,0.,0.]),
            3=>changed.orientation_wxyz=None,
            4=>changed.source_id.push_str(":new-source"),
            5=>changed.frame_id.push_str(":new-frame"),
            6=>changed.body_linear_xyz[2]=-0.,
            _=>{},
        }
        worker.observe_measured_motion((change!=7).then_some(changed));
        assert_eq!(worker.shared.0.lock().unwrap().motion_version,version+1,"change {change}");
        worker.stop();
    }
}

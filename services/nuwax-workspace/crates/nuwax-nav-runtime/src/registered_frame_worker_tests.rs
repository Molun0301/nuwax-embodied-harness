// Real worker version/publication path with a deliberately gated synthetic computation.
struct RegisteredWorkerProbe {
    frame:Option<Arc<RegisteredFrame>>,
    entered:std::sync::mpsc::Sender<Option<RegisteredFrameIdentity>>,
    release:std::sync::mpsc::Receiver<()>,
}
impl NavPlanner for RegisteredWorkerProbe {
    fn replace_path(&mut self,_:Path2D) {}
    fn update_odometry(&mut self,_:PoseStamped) {}
    fn update_costmap(&mut self,_:CostMap2D) {}
    fn observe_registered_frame(&mut self,frame:Option<Arc<RegisteredFrame>>) {self.frame=frame;}
    fn step(&mut self,_:f64)->PlannerStep {
        self.entered.send(self.frame.as_ref().map(|frame|frame.identity.clone())).unwrap();
        self.release.recv_timeout(Duration::from_secs(3)).unwrap();
        PlannerStep::Command(Twist2 {linear:nuwax_nav_types_messages::Point2::new(0.1,0.),angular_z:0.})
    }
    fn stop(&mut self) {}
}
fn registered_probe()->(PlannerWorker,std::sync::mpsc::Receiver<Option<RegisteredFrameIdentity>>,std::sync::mpsc::Sender<()>) {
    let (entered,started)=std::sync::mpsc::channel();
    let (release,wait)=std::sync::mpsc::channel();
    (PlannerWorker::new(RegisteredWorkerProbe{frame:None,entered,release:wait}).unwrap(),started,release)
}
#[test]
fn duplicate_registered_identity_does_not_replace_points_or_extend_original_deadline() {
    let (mut worker,_,_)=registered_probe();
    let original=crate::registered_frame::test_frame(1);
    worker.observe_registered_frame(Some(original.clone()));
    let version=worker.shared.0.lock().unwrap().map_version;
    let mut duplicate=(*original).clone();
    duplicate.points_world=vec![[900.,900.,900.]].into();
    duplicate.dispatch_bound=Some(CommandDispatchBound::from_source_timestamp(1,Duration::ZERO));
    worker.observe_registered_frame(Some(Arc::new(duplicate)));
    let input=worker.shared.0.lock().unwrap();
    assert_eq!(input.map_version,version);
    assert!(Arc::ptr_eq(input.registered_frame.as_ref().unwrap(),&original));
}
#[test]
fn registered_generation_change_and_revocation_retire_inflight_results() {
    let (mut worker,started,release)=registered_probe();
    let original=crate::registered_frame::test_frame(1);
    worker.observe_registered_frame(Some(original.clone()));
    worker.step(0.3);
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap(),Some(original.identity.clone()));
    let mut replacement=(*crate::registered_frame::test_frame(2)).clone();
    replacement.identity.domain.runtime_generation+=1;
    let identity=replacement.identity.clone();
    worker.observe_registered_frame(Some(Arc::new(replacement)));
    release.send(()).unwrap();
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap(),Some(identity));
    assert!(worker.shared.0.lock().unwrap().result.is_none());
    worker.observe_registered_frame(None);
    release.send(()).unwrap();
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap(),None);
    assert!(worker.shared.0.lock().unwrap().result.is_none());
    release.send(()).unwrap();
    wait_for_source_result(&worker);
    worker.stop();
}

#[test]
fn newer_scan_with_identical_points_retires_old_inflight_map_without_claiming_equal_probability() {
    let (mut worker,started,release)=registered_probe();
    let first=crate::registered_frame::test_frame(1);
    worker.observe_registered_frame(Some(first.clone()));worker.step(0.3);
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap(),Some(first.identity.clone()));
    let initial=worker.shared.0.lock().unwrap().map_version;
    let mut last=first.clone();
    for seq in 2..=4 {
        std::thread::sleep(Duration::from_millis(50));
        last=crate::registered_frame::test_frame(seq);
        assert_eq!(last.points_world,first.points_world);
        worker.observe_registered_frame(Some(last.clone()));
    }
    std::thread::sleep(Duration::from_millis(20));
    release.send(()).unwrap();
    assert_eq!(started.recv_timeout(Duration::from_secs(1)).unwrap(),Some(last.identity.clone()));
    assert_eq!(worker.shared.0.lock().unwrap().map_version,initial+3);
    assert!(worker.shared.0.lock().unwrap().result.is_none());
    release.send(()).unwrap();wait_for_source_result(&worker);
    assert!(matches!(worker.step(0.3),PlannerStep::Command(_)));
    worker.stop();
}

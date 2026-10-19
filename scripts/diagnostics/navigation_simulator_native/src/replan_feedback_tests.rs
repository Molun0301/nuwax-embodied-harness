// Synthetic feedback adapter test. The refusal is explicitly injected; this
// is not a terrain acceptance result and does not modify the C report verdicts.
use super::*;
fn fixture() -> (Scene,Simulation,PathBuf) {
    let mut config=SurfaceNavigationConfig::new("map".into(),0.4,0.3,0.6,0.05,0.15,18.,0.2).unwrap();
    config.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
        self_collision:None,footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        z_min_m:-0.05,z_max_m:0.2,source:"same-input sim adapter test".into(),
    }),None).unwrap();
    let points:Vec<_>=(-10..50).flat_map(|x|(-10..10).map(move |y|
        ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,-0.025))).collect();
    let directory=std::env::temp_dir().join(format!("neh-replan-feedback-{}",std::process::id()));
    fs::create_dir_all(&directory).unwrap();
    let scene=Scene::make("same-input-feedback","adapter fixture",points,vec![],config,true,
        GroundEvidence::MeasuredSurface,&directory).unwrap();
    let mut session=Simulation::new(&scene,&json!({"start":[0.,0.,0.4,0.],"goal":[1.5,0.,0.4,0.],
        "local_planning_enabled":false})).unwrap();
    session.plan(&scene).unwrap();
    session.rejected_plan=Some(session.plans);
    session.rejected_context=session.follower.refresh_execution_context(as_twist(session.applied));
    assert!(session.rejected_context.is_some());
    session.need_plan=true; session.status="planning".into(); session.message="injected execution refusal".into();
    (scene,session,directory)
}
#[test]
fn identical_stopped_refusal_reuses_success_without_stepping_but_real_pose_change_retries() {
    let (scene,mut session,directory)=fixture();
    let plans=session.plans;
    // stop withdraws the controller path. Calling step on the waiting adapter
    // would therefore be observable rather than silently re-executing the route.
    session.follower.stop();
    for _ in 0..5 {session.tick(&scene).unwrap();}
    assert_eq!(session.plans,plans);
    assert!(session.need_plan);
    assert_eq!(session.status,"planning");
    assert_eq!(session.message,"injected execution refusal");
    assert_eq!(session.command,[0.;3]);
    CANCEL.store(true,Ordering::Relaxed);
    let cancelled=session.plan(&scene);
    CANCEL.store(false,Ordering::Relaxed);
    assert!(cancelled.unwrap_err().contains("层析规划已取消"),"memo cannot swallow existing cancellation");
    assert_eq!(session.plans,plans+1,"preserve original failed-attempt accounting");
    session.current.0[2]=f64::from_bits(session.current.0[2].to_bits()+1);
    session.follower.update_odometry(stamped(session.current,session.time));
    // The assertion concerns dispatch, not whether this synthetic pose can be
    // planned; an original error remains an error rather than becoming no-path.
    let _=session.plan(&scene);
    assert_eq!(session.plans,plans+2);
    let _=fs::remove_file(directory.join("same-input-feedback.bin"));
    let _=fs::remove_dir(directory);
}

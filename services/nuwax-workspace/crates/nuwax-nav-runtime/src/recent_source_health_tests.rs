fn recent_health_scheduler()->(NavSchedulerHandle,JoinHandle<Result<(),RuntimeError>>) {
    let template=dispatch_scheduler();
    let scheduler=NavScheduler::new(template.motion,FakePlanner::commanding(),SchedulerConfig {
        tick_interval:Duration::from_millis(5),renew_interval:Duration::from_millis(20),
        ..SchedulerConfig::default()
    }).unwrap();
    spawn_scheduler(scheduler)
}
// Included inside runtime tests to use the existing real scheduler/fence fixtures.
#[tokio::test]
async fn frozen_cloud_pauses_despite_live_pose_and_map_updates() {
    let (handle,task)=recent_health_scheduler();
    handle.update_costmap(costmap_at(1_000_000_000)).await.unwrap();
    handle.update_odometry(map_pose()).await.unwrap();
    let stamp=crate::command_dispatch::unix_ns().unwrap();
    handle.update_obstacle_source(None,true,Some(CommandDispatchBound::from_source_timestamp(
        stamp,Duration::from_millis(150)))).await.unwrap();
    handle.start_session(session()).await.unwrap();
    handle.submit_path(path()).await.unwrap();
    for sequence in 2..50 {
        if handle.status().last_terminal.is_some() {break;}
        handle.update_odometry(map_pose_at(sequence)).await.unwrap();
        handle.update_costmap(costmap_at(sequence*1_000_000_000)).await.unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let terminal=wait_for_input_pause(&handle).await;
    assert!(terminal.applied_commands>0);
    assert!(terminal.message.contains("recent obstacle source deadline"));
    assert_eq!(terminal.last_applied_twist,Some(Twist2::ZERO));
    assert!(terminal.last_zero_ack_id.is_none());
    handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
}

#[tokio::test]
async fn live_cloud_renews_execution_health_and_explicit_revoke_pauses() {
    let (handle,task)=recent_health_scheduler();
    handle.update_costmap(costmap_at(1_000_000_000)).await.unwrap();
    handle.update_odometry(map_pose()).await.unwrap();
    handle.start_session(session()).await.unwrap();handle.submit_path(path()).await.unwrap();
    for sequence in 2..12 {
        handle.update_odometry(map_pose_at(sequence)).await.unwrap();
        handle.update_obstacle_source(None,true,Some(CommandDispatchBound::from_source_timestamp(
            crate::command_dispatch::unix_ns().unwrap(),Duration::from_millis(150)))).await.unwrap();
        tokio::time::sleep(Duration::from_millis(25)).await;
        assert!(handle.status().last_terminal.is_none());
    }
    handle.update_obstacle_source(None,false,None).await.unwrap();
    let terminal=wait_for_input_pause(&handle).await;
    assert!(terminal.message.contains("recent obstacle source revoked"));
    assert!(terminal.last_zero_ack_id.is_none());
    handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
}

#[test]
fn unconfigured_cloud_age_adds_no_default_stop_gate() {
    let mut scheduler=NavScheduler::new(FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
        FakePlanner::commanding(),SchedulerConfig::default()).unwrap();
    let (status,_)=watch::channel(scheduler.status.clone());
    scheduler.observe_odometry(&status,map_pose(),None).unwrap();
    scheduler.observe_costmap(&status,costmap_at(1_000_000_000),Instant::now()).unwrap();
    assert!(scheduler.stale_input_reason(Instant::now()).is_none());
    assert!(scheduler.obstacle_source_bound.is_none());
}

#[test]
fn mask_only_surface_update_changes_map_without_renewing_source_health() {
    let follower=SurfaceFollower::new(SurfaceFollowerConfig::new(0.3)).unwrap();
    let mut scheduler=NavScheduler::new(FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
        follower,SchedulerConfig::default()).unwrap();
    let (status,_)=watch::channel(scheduler.status.clone());
    let map=crate::surface_follower::test_snapshot(0.0,false);
    let receipt=Instant::now();
    scheduler.observe_costmap(&status,CostmapObservation {
        costmap:ExecutionMap::Surface(map.clone()),source_timestamp_ns:1_000_000_000,
    },receipt).unwrap();
    let bound=CommandDispatchBound::from_source_timestamp(1,Duration::from_nanos(1));
    scheduler.obstacle_source_bound=Some(bound);
    scheduler.observe_costmap(&status,CostmapObservation {
        costmap:ExecutionMap::SurfaceUpdate(crate::surface_changes::SurfaceMapUpdate {
            snapshot:map.clone(),change_basis:Some(map),
            changes:Arc::new(crate::surface_changes::SurfaceGeometryChanges::Unknown),
        }),source_timestamp_ns:1_000_000_000,
    },Instant::now()).unwrap();
    assert_eq!(scheduler.costmap_receipt.unwrap().received_at,receipt);
    assert_eq!(scheduler.obstacle_source_bound,Some(bound));
    assert!(scheduler.stale_input_reason(Instant::now()).unwrap().contains("recent obstacle source deadline"));
}

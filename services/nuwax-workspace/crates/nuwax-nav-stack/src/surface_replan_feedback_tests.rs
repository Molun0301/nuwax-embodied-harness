// Adapter tests only; these do not claim transport/controller closure.
use super::*;
use nuwax_nav_types_messages::Twist2;

#[tokio::test]
async fn repeated_current_path_refusals_are_rate_limited_and_cancellable() {
    let mut harness=Harness::start().await;
    let mut states=typed_stream(&harness.transport,&harness.planner_node.inputs[4],
        "nuwax_msgs.NavigationState").await.unwrap();
    harness.wait_ready().await;
    let goal=goal_for(&harness.feed.lock().unwrap().map,1,[3.2,1.55,2.5]);
    harness.goal(&goal).await;
    let mut previous=harness.path_for(&goal).await;
    harness.wait_controlling().await;
    let mut waiting=tokio::time::timeout(Duration::from_secs(2),async {
        loop {
            let frame=states.next().await.unwrap().unwrap();
            let state=schema::decode_navigation_state_body(&frame.payload).unwrap();
            if state_matches_goal(&state,&goal) && state.state==2 {break state;}
        }
    }).await.unwrap();
    waiting.state=1;
    for _ in 0..2 {
        waiting.path_revision=previous.path_revision;
        for _ in 0..15 {
            publish_body(&harness.transport,&harness.planner_node.inputs[4],
                "nuwax_msgs.NavigationState",schema::encode_navigation_state_body(&waiting).unwrap()).await.unwrap();
        }
        let next=harness.path_for(&goal).await;
        let stamp=|path:&schema::PathEnvelope| i64::from(path.path.header.stamp_sec)*1_000_000_000
            +i64::from(path.path.header.stamp_nsec);
        assert!(stamp(&next)-stamp(&previous)>=1_000_000_000,
            "重复局部等待不能立即刷新全局路线");
        previous=next;
        harness.wait_controlling().await;
    }
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}
fn fixture() -> (Arc<SurfaceSnapshot>,PoseStamped,schema::GoalEnvelope,SurfaceNavigationConfig,SurfaceReplanFeedback) {
    let mut config=config();
    config.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
        self_collision:None,footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        z_min_m:-0.05,z_max_m:0.2,source:"replan adapter fixture".into(),
    }),None).unwrap();
    let points:Vec<_>=(0..30).flat_map(|x|(0..20).map(move |y|
        ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
    let surface=Arc::new(SurfaceSnapshot::from_points(&points,config.geometry.clone(),0.1).unwrap()
        .with_body_envelope(config.body_envelope.clone().unwrap()).unwrap());
    let goal=goal_for(&map_message(1,"feedback"),1,[2.,1.,0.5]);
    let mut start=pose_from_schema(&goal.goal).unwrap();start.pose.position.x=0.5;
    let actual=Arc::new(ActualExecutionContext {measured_motion:None,source:surface.clone(),odometry:start.clone(),
        applied:Twist2::ZERO});
    let feedback=SurfaceReplanFeedback {path_identity:Arc::new(published_surface_identity(&goal,31,7)),
        rejected_context:Some(actual.clone()),current_context:Some(actual),zero_confirmed_rejection:true,rejection_reason:Some("Current body pose is occupied or outside the local map".into())};
    (surface,start,goal,config,feedback)
}
#[test]
fn revisions_are_correlation_not_search_input_and_foreign_goal_context_is_not_used() {
    let (source,start,goal,config,mut feedback)=fixture();
    let original=surface_replan_input(&source,&start,&goal,&config,Some(&feedback)).unwrap();
    feedback.path_identity=Arc::new(published_surface_identity(&goal,31,8));
    let next=surface_replan_input(&source,&start,&goal,&config,Some(&feedback)).unwrap();
    assert!(original.same_complete_input(&next),"output revision is not a work input");
    let mut foreign=goal.clone();foreign.goal_generation+=1;
    let next=surface_replan_input(&source,&start,&foreign,&config,Some(&feedback)).unwrap();
    assert!(next.actual.is_none(),"do not borrow another goal's live context");
    assert!(!original.same_complete_input(&next));
}
#[test]
fn adapter_keeps_current_recent_obstacles_as_real_dependencies() {
    let (source,start,goal,config,mut feedback)=fixture();
    let original=surface_replan_input(&source,&start,&goal,&config,Some(&feedback)).unwrap();
    let actual=Arc::make_mut(feedback.current_context.as_mut().unwrap());
    actual.source=Arc::new(source.with_recent_obstacle_points(1,1,0,&[],[(0,[1.,1.,0.7])]).unwrap());
    let recent=surface_replan_input(&source,&start,&goal,&config,Some(&feedback)).unwrap();
    assert!(recent.source.same_body_query_view(&original.source));
    assert!(!original.same_complete_input(&recent));
}

#[test]
fn adapter_preserves_rejected_b_when_current_and_published_inputs_return_to_a() {
    let (source,start,goal,config,mut feedback)=fixture();
    let a=surface_replan_input(&source,&start,&goal,&config,Some(&feedback)).unwrap();
    let mut memo=SuccessfulReplanMemo::default();
    memo.published(feedback.path_identity.as_ref().clone(),a.clone());
    assert!(memo.should_reuse(surface_replan_rejection(Some(&feedback),&goal),&a));
    // Only the refusing actual observation becomes B; publication and latest
    // current actual remain A. A single pose bit is still a real dependency.
    let rejected=Arc::make_mut(feedback.rejected_context.as_mut().unwrap());
    rejected.odometry.pose.position.z=f64::from_bits(rejected.odometry.pose.position.z.to_bits()+1);
    let restored=surface_replan_input(&source,&start,&goal,&config,Some(&feedback)).unwrap();
    assert!(a.same_complete_input(&restored));
    assert!(!memo.should_reuse(surface_replan_rejection(Some(&feedback),&goal),&restored));
    feedback.rejected_context=None;
    assert!(!memo.should_reuse(surface_replan_rejection(Some(&feedback),&goal),&restored));
    feedback.rejected_context=feedback.current_context.clone();
    feedback.zero_confirmed_rejection=false;
    assert!(!memo.should_reuse(surface_replan_rejection(Some(&feedback),&goal),&restored));
    feedback.zero_confirmed_rejection=true;
    assert!(memo.should_reuse(surface_replan_rejection(Some(&feedback),&goal),&restored));
    let mut foreign=goal.clone(); foreign.goal_generation+=1;
    assert!(!memo.should_reuse(surface_replan_rejection(Some(&feedback),&foreign),&restored));
}

#[test]
fn recomputation_progress_keeps_original_refusal_without_borrowing_another_goal() {
    let (_,_,goal,_,mut feedback)=fixture();
    // 后端没有可复用的实际上下文，也不能丢掉已确认的拒绝事实。
    feedback.rejected_context=None;
    feedback.current_context=None;
    let message=surface_planning_diagnostic(Some(&feedback),&goal,"正在计算PCT全局路线");
    assert!(message.contains("Current body pose is occupied or outside the local map"));
    assert!(message.contains("正在计算PCT全局路线"));
    assert!(message.contains("已归零"));
    let mut replacement=goal.clone();replacement.goal_generation+=1;
    assert_eq!(surface_planning_diagnostic(Some(&feedback),&replacement,"正在计算PCT全局路线"),
        "正在计算PCT全局路线");
    feedback.zero_confirmed_rejection=false;
    assert_eq!(surface_planning_diagnostic(Some(&feedback),&goal,"正在计算PCT全局路线"),
        "正在计算PCT全局路线");
}

// Adapter tests only; these do not claim transport/controller closure.
use super::*;
use nuwax_nav_types_messages::Twist2;
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
    let actual=Arc::new(ActualExecutionContext {source:surface.clone(),odometry:start.clone(),
        ground:surface.body_query().start_ground_state(([0.5,1.,0.5],[0.,0.,0.,1.])).unwrap(),applied:Twist2::ZERO});
    let feedback=SurfaceReplanFeedback {path_identity:Arc::new(published_surface_identity(&goal,31,7)),
        rejected_context:Some(actual.clone()),current_context:Some(actual),zero_confirmed_rejection:true};
    (surface,start,goal,config,feedback)
}
#[test]
fn revisions_are_correlation_not_search_input_and_foreign_goal_context_is_not_used() {
    let (source,start,goal,config,mut feedback)=fixture();
    let penalties=Arc::new(vec![]);
    let original=surface_replan_input(&source,&start,&goal,&config,256,penalties.clone(),Some(&feedback)).unwrap();
    feedback.path_identity=Arc::new(published_surface_identity(&goal,31,8));
    let next=surface_replan_input(&source,&start,&goal,&config,256,penalties.clone(),Some(&feedback)).unwrap();
    assert!(original.same_complete_input(&next),"output revision is not a work input");
    let mut foreign=goal.clone();foreign.goal_generation+=1;
    let next=surface_replan_input(&source,&start,&foreign,&config,256,penalties,Some(&feedback)).unwrap();
    assert!(next.actual.is_none(),"do not borrow another goal's live context");
    assert!(!original.same_complete_input(&next));
}
#[test]
fn adapter_keeps_recent_overlay_costs_and_original_budget_as_independent_dependencies() {
    let (source,start,goal,config,mut feedback)=fixture();
    let a=Arc::new(vec![]);
    let original=surface_replan_input(&source,&start,&goal,&config,256,a.clone(),Some(&feedback)).unwrap();
    let actual=Arc::make_mut(feedback.current_context.as_mut().unwrap());
    actual.source=Arc::new(source.with_recent_obstacle_points(1,1,0,&[],[(0,[1.,1.,0.7])]).unwrap());
    let recent=surface_replan_input(&source,&start,&goal,&config,256,a.clone(),Some(&feedback)).unwrap();
    assert!(recent.source.same_body_query_view(&original.source));
    assert!(!original.same_complete_input(&recent));
    let quick=surface_replan_input(&source,&start,&goal,&config,0,a,Some(&feedback)).unwrap();
    assert!(!recent.same_complete_input(&quick),"keep existing quick/full work semantics");
    let b=Arc::new(vec![nuwax_nav_pct::RoutePenalty{center:[1.,1.,0.1],radius:0.2,weight:16.}]);
    let changed=surface_replan_input(&source,&start,&goal,&config,256,b.clone(),Some(&feedback)).unwrap();
    assert!(Arc::ptr_eq(&changed.penalties,&b));
    assert!(!recent.same_complete_input(&changed));
}

#[test]
fn adapter_preserves_rejected_b_when_current_and_published_inputs_return_to_a() {
    let (source,start,goal,config,mut feedback)=fixture();
    let penalties=Arc::new(vec![]);
    let a=surface_replan_input(&source,&start,&goal,&config,256,penalties.clone(),Some(&feedback)).unwrap();
    let mut memo=SuccessfulReplanMemo::default();
    memo.published(feedback.path_identity.as_ref().clone(),a.clone());
    assert!(memo.should_reuse(surface_replan_rejection(Some(&feedback),&goal),&a));
    // Only the refusing actual observation becomes B; publication and latest
    // current actual remain A. A single pose bit is still a real dependency.
    let rejected=Arc::make_mut(feedback.rejected_context.as_mut().unwrap());
    rejected.odometry.pose.position.z=f64::from_bits(rejected.odometry.pose.position.z.to_bits()+1);
    let restored=surface_replan_input(&source,&start,&goal,&config,256,penalties,Some(&feedback)).unwrap();
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

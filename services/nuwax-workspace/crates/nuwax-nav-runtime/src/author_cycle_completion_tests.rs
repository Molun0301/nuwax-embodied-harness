// Typed completion feedback is not reusable geometric refusal evidence.
struct CycleFeedbackProbe { actual:Arc<ActualExecutionContext>, complete:bool }
impl NavPlanner for CycleFeedbackProbe {
    fn replace_path(&mut self,_:Path2D) {}
    fn update_odometry(&mut self,_:PoseStamped) {}
    fn update_costmap(&mut self,_:CostMap2D) {}
    fn refresh_execution_context(&mut self,_:Twist2)->Option<Arc<ActualExecutionContext>> {Some(self.actual.clone())}
    fn execution_cycle_complete(&self)->bool {self.complete}
    fn step(&mut self)->PlannerStep {PlannerStep::ReplanRequired("typed lifecycle result".into())}
    fn stop(&mut self) {}
}
#[test]
fn completed_author_cycle_preserves_current_context_but_not_geometric_refusal_proof() {
    for complete in [false,true] {
        let actual=Arc::new(ActualExecutionContext {
            source:crate::surface_follower::test_snapshot(0.,false),odometry:map_pose(),
            applied:Twist2::ZERO,measured_motion:None,
        });
        let mut scheduler=NavScheduler::new(FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
            CycleFeedbackProbe{actual:actual.clone(),complete},SchedulerConfig::default()).unwrap();
        scheduler.current_execution_identity=Some(Arc::new(path().execution_identity));scheduler.status.last_applied_twist=Some(Twist2::ZERO);
        let step=scheduler.planner.step();let intent=scheduler.wrap_local_intent(step).unwrap();
        assert_eq!(intent.execution_context.is_some(),!complete,"完成不能变为几何拒绝缓存");
        assert!(scheduler.planner.refresh_execution_context(Twist2::ZERO).is_some());
    }
}

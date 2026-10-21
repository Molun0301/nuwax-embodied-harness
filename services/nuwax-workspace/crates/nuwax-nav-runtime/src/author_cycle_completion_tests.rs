// Typed completion feedback is not reusable geometric refusal evidence.
struct CycleFeedbackProbe { actual:Arc<ActualExecutionContext>, complete:bool }
impl NavPlanner for CycleFeedbackProbe {
    fn replace_path(&mut self,_:Path2D) {}
    fn update_odometry(&mut self,_:PoseStamped) {}
    fn update_costmap(&mut self,_:CostMap2D) {}
    fn refresh_execution_context(&mut self,_:Twist2)->Option<Arc<ActualExecutionContext>> {Some(self.actual.clone())}
    fn execution_cycle_complete(&self)->bool {self.complete}
    fn step(&mut self,_:f64)->PlannerStep {PlannerStep::ReplanRequired("typed lifecycle result".into())}
    fn stop(&mut self) {}
}
#[test]
fn completed_author_cycle_preserves_current_context_but_not_geometric_refusal_proof() {
    for complete in [false,true] {
        let actual=Arc::new(ActualExecutionContext {
            source:crate::surface_follower::test_snapshot(0.,false),odometry:route_pose(0.5,0),
            applied:Twist2::ZERO,measured_motion:None,
        });
        let mut worker=PlannerWorker::new(CycleFeedbackProbe{actual:actual.clone(),complete}).unwrap();
        worker.step(0.3);wait_for_source_result(&worker);
        assert!(matches!(worker.step(0.3),PlannerStep::ReplanRequired(_)));
        assert_eq!(worker.take_step_execution_context(Twist2::ZERO).is_some(),!complete,
            "完成不能变为几何拒绝缓存；普通拒绝仍保留其原证据");
        assert!(worker.refresh_execution_context(Twist2::ZERO).is_some());
        worker.stop();
        assert!(worker.shared.0.lock().unwrap().result.is_none());
    }
}

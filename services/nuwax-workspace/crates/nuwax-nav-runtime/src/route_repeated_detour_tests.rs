// Included inside the existing diagnostic fixture module. Actual PlanKernel::compute,
// exact logged stopped poses, original budget and source; no injected failure/candidate.
mod repeated_detour_tests {
    use super::*;
    use nuwax_nav_scan::LocalPlanFailureKind;
    use crate::planning_owner::Cancellation;

    fn frozen_stopped(case: Case) -> (nuwax_nav_scan::BodyPose, f64) {
        match case {
            Case::WideApproach => (([-0.33401961360183036, 0.15117473073791413, 0.4],
                [0.0, 0.0, 0.13954311464423594, 0.9902159962126372]), 0.8659803863981697),
            Case::ShortApproach => (([-0.33607583041988814, 0.17963204671760727, 0.4],
                [0.0, 0.0, 0.16058573835323378, 0.9870218947103183]), 0.41392416958011186),
        }
    }
    fn compute(kernel: &mut PlanKernel, input: &PlanRequest, cancel: &Cancellation)
        -> (PlanResult, probe::Frame) {
        probe::begin();
        let result=kernel.compute(duplicate_request(input), cancel);
        let frame=probe::end();
        (result,frame)
    }
    fn failure_bits(result: &PlanResult) -> Vec<(LocalPlanFailureKind,
        Option<([u64;3],[u64;4])>, Option<[u64;3]>, String)> {
        let failures=match &result.outcome {Err(f)=>f,Ok(_)=>panic!("expected actual local failure")};
        failures.iter().map(|f|(f.kind,f.closest_pose.map(pose_bits),
            f.blocked.map(|p|p.map(f64::to_bits)),f.reason.clone())).collect()
    }
    fn detour_reference(frame: &probe::Frame) -> &[[f64;3]] {
        &frame.branches.iter().find(|b|b.detour).expect("actual detour branch").reference
    }
    fn real_repeat_and_recovery(case: Case, change_costs: bool) {
        let mut follower=prepare_case(case);
        let (stopped,progress)=frozen_stopped(case);
        follower.pose=Some(stamped(stopped,Duration::ZERO));
        follower.progress=progress;
        let request=snapshot_request(&follower,false);
        request.source.validate_body_motion(stopped,stopped,&request.boundary)
            .expect("recorded stopped body must remain geometrically valid");
        let mut kernel=PlanKernel::new(follower.config.clone());
        let mut cancel=Cancellation {epoch:Arc::new(AtomicU64::new(1)),expected:1};
        let (first,warm)=compute(&mut kernel,&request,&cancel);
        let failures=failure_bits(&first);
        assert_eq!(failures.len(),2);
        assert!(failures.iter().all(|f|f.0==LocalPlanFailureKind::BudgetExhausted));
        assert_eq!(warm.pct.len(),1,"warm request must call actual PCT");
        assert!(warm.pct[0].stats.as_ref().is_some_and(|s|s.expanded>0),
            "fixture requires successful PCT reference plus local Scan failure");
        assert_eq!(warm.branches.len(),2);
        assert!(warm.branches.iter().all(|b|!b.cached_negative));
        let (second,repeat)=compute(&mut kernel,&request,&cancel);
        assert_eq!(failure_bits(&second),failures,"exact repeat must preserve typed diagnostic output bitwise");
        eprintln!("REPEATED_DETOUR case={case:?} warm_pct={} repeated_pct={} warm_expanded={} repeat_work={repeat:?}",
            warm.pct.len(),repeat.pct.len(),warm.pct[0].stats.as_ref().unwrap().expanded);
        // Keep evidence from positive changes even on the expected red baseline.
        if change_costs {
            let center=detour_reference(&warm)[1];
            follower.config.pct.report_local_blockage(center,0.5);
            assert_eq!(cancel.expected,1,"cost changes deliberately do not cancel the owner");
            let (penalized,penalty_work)=compute(&mut kernel,&request,&cancel);
            assert_eq!(penalty_work.pct.len(),1,"same-epoch changed costs must rerun actual PCT");
            assert!(penalty_work.reused_failures.is_empty());
            assert!(!penalty_work.branches.iter().find(|b|b.detour).unwrap().cached_negative);
            assert!(!exact_points(detour_reference(&warm),detour_reference(&penalty_work)),
                "reported cost must actually change this PCT reference; otherwise fixture is inconclusive");
            // Whatever the new finite plan concludes, it must equal a fresh computation
            // at the same snapshot and original budget, not the old cached local result.
            let mut fresh=PlanKernel::new(follower.config.clone());
            let (fresh_result,fresh_work)=compute(&mut fresh,&request,&cancel);
            assert_eq!(fresh_work.pct.len(),1);
            match (&penalized.outcome,&fresh_result.outcome) {
                (Err(_),Err(_))=>assert_eq!(failure_bits(&penalized),failure_bits(&fresh_result)),
                (Ok(a),Ok(b))=>{
                    assert_eq!(a.partial,b.partial);
                    assert_eq!(a.path.poses.iter().map(|p|pose_bits(body(p))).collect::<Vec<_>>(),
                        b.path.poses.iter().map(|p|pose_bits(body(p))).collect::<Vec<_>>());
                }
                _=>panic!("changed-cost cached owner diverged from fresh actual compute"),
            }
            follower.config.pct.clear_local_costs();
            let (cleared,clear_work)=compute(&mut kernel,&request,&cancel);
            assert_eq!(clear_work.pct.len(),1,"clearing costs at same epoch must re-evaluate");
            assert!(clear_work.reused_failures.is_empty());
            assert!(exact_points(detour_reference(&warm),detour_reference(&clear_work)));
            assert_eq!(failure_bits(&cleared),failures);
            eprintln!("REPEATED_DETOUR_COST_CHANGE report_pct={} clear_pct={} changed_reference=true epoch={}",
                penalty_work.pct.len(),clear_work.pct.len(),cancel.expected);
        }
        if change_costs {
            let mut larger=duplicate_request(&request);
            larger.scan.search_expansion_limit+=1;
            let (changed_budget,budget_work)=compute(&mut kernel,&larger,&cancel);
            assert_eq!(budget_work.pct.len(),1);
            assert!(budget_work.reused_failures.is_empty());
            assert!(budget_work.branches.iter().all(|b|!b.cached_negative),
                "a larger finite budget must retry actual Scan, never inherit old exhaustion");
            match &changed_budget.outcome {
                Err(_)=>assert_ne!(failure_bits(&changed_budget),failures,
                    "fixture must show the new finite-budget result, not replay old diagnostics"),
                Ok(candidate)=>assert!(!candidate.path.poses.is_empty()),
            }
            eprintln!("REPEATED_DETOUR_BUDGET old={} new={} pct={} actual_scan_branches={}",
                request.scan.search_expansion_limit,larger.scan.search_expansion_limit,
                budget_work.pct.len(),budget_work.branches.len());
        }
        // Real recovery: same source/goal/body/budget, actual start changed back to
        // the observed safe approach; detour must again be computed and executable.
        let mut moved=duplicate_request(&request);
        moved.pose=pose(case.start_x(),1_000_000_000);
        moved.progress=0.0;
        moved.reference_source.as_mut().unwrap().progress=0.0;
        moved.detour=true;
        let (recovered,moved_work)=compute(&mut kernel,&moved,&cancel);
        assert_eq!(moved_work.pct.len(),1);
        assert!(moved_work.reused_failures.is_empty());
        let candidate=recovered.outcome.as_ref().unwrap_or_else(|e|panic!("real safe-pose recovery failed: {e:?}"));
        independently_check_full_candidate(&recovered.request,candidate).unwrap();
        // Epoch retirement and cancellation are exercised without adding gates.
        cancel.epoch.store(2,std::sync::atomic::Ordering::Release);
        let (cancelled,cancel_work)=compute(&mut kernel,&request,&cancel);
        let errors=cancelled.outcome.err().expect("cancelled work cannot publish a candidate");
        assert!(errors.iter().all(|f|f.kind==LocalPlanFailureKind::Cancelled));
        assert!(cancel_work.pct.is_empty() && cancel_work.reused_failures.is_empty());
        cancel.expected=2;
        let (new_epoch,new_epoch_work)=compute(&mut kernel,&moved,&cancel);
        assert_eq!(new_epoch_work.pct.len(),1);
        assert!(new_epoch_work.reused_failures.is_empty());
        independently_check_full_candidate(&new_epoch.request,new_epoch.outcome.as_ref().unwrap()).unwrap();
        assert_eq!(repeat.pct.len(),0,"exact repeated local failure must not recompute its global reference");
        assert_eq!(repeat.reused_failures.len(),2,"both original input branches reused their exact failures");
    }
    #[test]
    fn wide_exact_failure_reuses_pct_work_and_recovers_after_actual_pose_change() {
        real_repeat_and_recovery(Case::WideApproach,false);
    }
    #[test]
    fn short_exact_failure_preserves_same_epoch_cost_changes_and_real_recovery() {
        real_repeat_and_recovery(Case::ShortApproach,true);
    }
}

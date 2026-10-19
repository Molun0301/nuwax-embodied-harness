use super::*;
use crate::reference::Reference;

fn adjacent_observed_map() -> Tomogram {
    Tomogram::from_columns_with_evidence(1.0,1.0,
        [([0,0],vec![[-1.0,0.0]]),([1,0],vec![[-1.0,0.0]])],
        0.4,0.0,0.1,0.1,20.0,0.0,true).unwrap()
}

fn distant_penalty() -> [crate::RoutePenalty;1] {
    [crate::RoutePenalty {center:[10.0,10.0,0.0],radius:0.1,weight:1.0}]
}

#[test]
fn adjacent_observed_goal_survives_deferred_unknown_admission() {
    let map=adjacent_observed_map();
    let start=[0.5,0.5,0.0];let goal=[1.5,0.5,0.0];
    let initial=map.reference_start(start).unwrap();
    let target=map.reference_start(goal).unwrap();
    assert!(matches!(initial,Reference::Known(_)));
    assert!(matches!(target,Reference::Known(_)));
    assert_eq!(map.reference_neighbor(initial,[1,0]),Some(target));
    assert!(map.reference_segment(start,goal,&[],None).is_some(),
        "独立核对这两个真实观测状态的完整参考邻边");
    for (start,goal) in [(start,goal),(goal,start)] {
        for maximum in [2,4096] {
            let route=map.plan_bounded_with_limits_typed(start,goal,0.0,&[],None,None,
                &distant_penalty(),SearchLimits {max_states:maximum}).unwrap();
            assert_eq!(route.points,vec![start,goal]);
            assert_eq!(route.stats.expanded,2);
            assert!(route.points.windows(2).all(|p|map.segment_clear(p[0],p[1],&[])));
            let traced=map.plan_bounded_traced(start,goal,0.0,&[],None,None,&distant_penalty(),
                &|_,_|true,SearchLimits {max_states:maximum}).unwrap();
            assert_eq!(traced.route.points,route.points);
            assert_eq!(traced.route.stats.expanded,route.stats.expanded);
        }
    }
}

#[test]
fn full_state_pool_keeps_an_already_admitted_unknown_goal() {
    let map=Tomogram::from_columns(1.0,[],0.4,0.0,0.1,0.1,20.0).unwrap();
    let start=[0.5,0.5,0.0];let goal=[1.5,0.5,0.0];
    assert!(matches!(map.reference_start(goal),Some(Reference::Unknown {..})));
    // 原邻居次序中目标是第七个新状态；第八个新状态不能抢先终结整次搜索。
    let route=map.plan_bounded_with_limits_typed(start,goal,0.0,&[],None,None,
        &distant_penalty(),SearchLimits {max_states:8}).unwrap();
    assert_eq!(route.points,vec![start,goal]);
    assert_eq!(route.stats.expanded,2);
    let error=map.plan_bounded_with_limits_typed(start,goal,0.0,&[],None,None,
        &distant_penalty(),SearchLimits {max_states:1}).unwrap_err();
    assert!(matches!(error,PlanningFailure::BudgetExhausted(_)),"{error}");
}

#[test]
fn observed_endpoints_still_use_a_required_unknown_bridge_within_budget() {
    let map=Tomogram::from_columns_with_evidence(1.0,1.0,
        [([0,0],vec![[-1.0,0.0]]),([2,0],vec![[-1.0,0.0]])],
        0.4,0.0,0.1,0.1,20.0,0.0,true).unwrap();
    let boundary=[[0.0,0.0],[3.0,0.0],[3.0,1.0],[0.0,1.0]];
    let start=[0.5,0.5,0.0];let goal=[2.5,0.5,0.0];
    let unknown=map.reference_neighbor(map.reference_start(start).unwrap(),[1,0]).unwrap();
    assert!(matches!(unknown,Reference::Unknown {..}));
    assert_eq!(map.reference_neighbor(unknown,[2,0]),map.reference_start(goal));
    let route=map.plan_bounded_with_limits_typed(start,goal,0.0,&boundary,None,None,
        &distant_penalty(),SearchLimits {max_states:3}).unwrap();
    assert_eq!(route.stats.expanded,3);
    assert_eq!(route.points.first(),Some(&start));
    assert_eq!(route.points.last(),Some(&goal));
    assert!(route.points.windows(2).all(|p|map.segment_clear(p[0],p[1],&boundary)));
    let error=map.plan_bounded_with_limits_typed(start,goal,0.0,&boundary,None,None,
        &distant_penalty(),SearchLimits {max_states:2}).unwrap_err();
    assert_eq!(error,PlanningFailure::BudgetExhausted(
        "层析请求状态预算耗尽（展开2格，状态2，上限2）；未证明无路".into()));
}

#[test]
fn rejected_candidates_do_not_manufacture_state_budget_exhaustion() {
    let map=adjacent_observed_map();
    let start=[0.5,0.5,0.0];let goal=[1.5,0.5,0.0];
    let calls=std::cell::RefCell::new(Vec::new());
    let rejects=|_: [f64;3],b: [f64;3]| { calls.borrow_mut().push(b);false };
    let error=map.plan_bounded_with_segment_filter_and_limits_typed(start,goal,0.0,&[],None,None,
        &distant_penalty(),&rejects,SearchLimits {max_states:1}).unwrap_err();
    assert_eq!(error,PlanningFailure::Other("层析参考在请求范围内没有可连接路线（展开1格）".into()));
    assert_eq!(*calls.borrow(),vec![[-0.5,-0.5,0.0],[-0.5,0.5,0.0],[-0.5,1.5,0.0],
        [0.5,-0.5,0.0],[0.5,1.5,0.0],[1.5,-0.5,0.0],[1.5,0.5,0.0],[1.5,1.5,0.0]],
        "观测优先不能改变每次展开的过滤回调次序");
}

#[test]
fn a_known_wall_exhausts_the_finite_graph_without_a_budget_error() {
    let map=Tomogram::from_columns_with_evidence(1.0,1.0,
        [([0,0],vec![[-1.0,0.0]]),([1,0],vec![[0.0,1.0]]),([2,0],vec![[-1.0,0.0]])],
        0.4,0.0,0.1,0.1,20.0,0.0,true).unwrap();
    let boundary=[[0.0,0.0],[3.0,0.0],[3.0,1.0],[0.0,1.0]];
    let start=[0.5,0.5,0.0];let goal=[2.5,0.5,0.0];
    assert!(map.reference_neighbor(map.reference_start(start).unwrap(),[1,0]).is_none());
    assert!(map.reference_segment(start,goal,&boundary,None).is_none());
    let error=map.plan_bounded_with_limits_typed(start,goal,0.0,&boundary,None,None,
        &distant_penalty(),SearchLimits {max_states:1}).unwrap_err();
    assert_eq!(error,PlanningFailure::Other("层析参考在请求范围内没有可连接路线（展开1格）".into()));
}

#[test]
fn cancellation_remains_active_while_draining_admitted_states() {
    let map=adjacent_observed_map();
    let start=[0.5,0.5,0.0];let goal=[1.5,0.5,0.0];
    let cancelled=AtomicBool::new(false);
    let accepts=|a: [f64;3],_: [f64;3]| {
        if a==goal {cancelled.store(true,AtomicOrdering::Release);}
        true
    };
    let error=map.plan_bounded_with_segment_filter_and_limits_typed(start,goal,0.0,&[],
        Some(&cancelled),None,&distant_penalty(),&accepts,SearchLimits {max_states:2}).unwrap_err();
    assert!(cancelled.load(AtomicOrdering::Acquire),"必须确实处理已接纳的目标状态");
    assert_eq!(error,PlanningFailure::Other("层析规划已取消".into()));
}

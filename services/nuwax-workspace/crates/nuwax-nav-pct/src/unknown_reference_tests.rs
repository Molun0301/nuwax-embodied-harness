use crate::{SearchLimits, Tomogram};
fn map(columns: impl IntoIterator<Item=([i32;2],Vec<[f64;2]>)>) -> Tomogram {
    Tomogram::from_columns(0.1,columns,0.4,0.0,0.1,0.1,20.0).unwrap()
}
const BOUNDARY: [[f64;2];4] = [[0.,0.],[2.,0.],[2.,0.3],[0.,0.3]];
#[test]
fn unknown_start_gap_endpoint_share_unmodified_reference_geometry() {
    for columns in [vec![],vec![([0,1],vec![[-0.1,0.0]])],
        vec![([0,1],vec![[-0.1,0.0]]),([19,1],vec![[-0.1,0.0]])],
        vec![([19,1],vec![[-0.1,0.0]])]] {
        let map=map(columns);
        let before=format!("{:?}",map.cells);
        let route=map.plan([0.05,0.15,0.0],[1.95,0.15,0.0],0.0,&BOUNDARY,None).unwrap();
        assert!(route.points.iter().all(|p|p[2]==0.0));
        assert!(route.points.windows(2).all(|p|map.segment_clear(p[0],p[1],&BOUNDARY)));
        assert_eq!(before,format!("{:?}",map.cells));
        assert!(map.cell_at([1.,0.15,0.0]).is_none());
    }
}
#[test]
fn same_unknown_scene_still_rejects_real_wall_and_low_ceiling() {
    for bands in [vec![[-0.1,1.0]],vec![[0.2,0.3]]] {
        let map=map((0..3).map(|y|([10,y],bands.clone())));
        assert!(!map.segment_clear([0.05,0.15,0.0],[1.95,0.15,0.0],&BOUNDARY));
        assert!(map.plan([0.05,0.15,0.0],[1.95,0.15,0.0],0.0,&BOUNDARY,None).is_err());
    }
    let high=map((0..3).map(|y|([10,y],vec![[0.8,0.9]])));
    assert!(high.plan([0.05,0.15,0.0],[1.95,0.15,0.0],0.0,&BOUNDARY,None).is_ok(),
        "overhead-only evidence does not prove a floor or block sufficient clearance");
}
#[test]
fn known_drop_or_other_floor_cannot_be_crossed_as_unknown() {
    for z in [-0.5] {
        let map=map((0..3).map(|y|([10,y],vec![[z-0.1,z]])));
        assert!(map.plan([0.05,0.15,0.0],[1.95,0.15,0.0],0.0,&BOUNDARY,None).is_err(),
            "known drop or raised solid surface remains incompatible at z={z}");
    }
    let empty=map([]);
    let error=empty.plan_bounded_with_limits([0.05,0.15,0.0],[1.95,0.15,1.0],0.0,
        &[],None,None,&[],SearchLimits{max_states:128}).unwrap_err();
    assert!(error.starts_with("UnsupportedReferenceHeight:"));
    assert!(empty.cells.is_empty());
}
#[test]
fn unknown_diagonal_and_shortcut_cannot_cut_a_known_corner() {
    let map=map([([1,0],vec![[-0.1,1.0]]),([0,1],vec![[-0.1,1.0]])]);
    let boundary=[[0.,0.],[0.2,0.],[0.2,0.2],[0.,0.2]];
    assert!(!map.segment_clear([0.05,0.05,0.0],[0.15,0.15,0.0],&boundary));
    assert!(map.plan([0.05,0.05,0.0],[0.15,0.15,0.0],0.0,&boundary,None).is_err());
}
#[test]
fn long_all_unknown_is_linear_and_cancellable_without_admitting_an_area() {
    let map=map([]);
    for length in [1.0,10.0,100.0] {
        let route=map.plan_bounded_with_limits([0.05,0.05,0.0],[length,0.05,0.0],0.0,
            &[],None,None,&[],SearchLimits{max_states:1}).unwrap();
        assert_eq!(route.points.len(),2);
        assert_eq!(route.stats.expanded,0);
        assert_eq!(route.stats.cells,0);
        eprintln!("unknown_direct length_m={length} expanded={} cells={} elapsed_ms={}",
            route.stats.expanded,route.stats.cells,route.stats.elapsed_ms);
    }
    let error=crate::cancellation::after_checkpoints(20,||
        map.plan([0.05,0.05,0.0],[100.,0.05,0.0],0.0,&[],None)).unwrap_err();
    assert!(error.contains("取消"));
}
#[test]
fn enclosed_target_has_inconclusive_budget_error_and_request_limit_is_retryable() {
    let map=map((-2..=2).flat_map(|x|(-2..=2).filter_map(move|y|
        ((x== -2||x==2)||(y== -2||y==2)).then(||([x,y],vec![[-0.1,1.0]])))));
    let start=[1.05,0.05,0.0];let goal=[0.05,0.05,0.0];
    let error=map.plan_bounded_with_limits(start,goal,0.0,&[],None,None,&[],SearchLimits{max_states:64}).unwrap_err();
    assert!(error.starts_with("BudgetExhausted:"),"{error}");
    let bounded=[[-2.,-2.],[2.,-2.],[2.,2.],[-2.,2.]];
    let error=map.plan_bounded_with_limits(start,goal,0.0,&bounded,None,None,&[],SearchLimits{max_states:4096}).unwrap_err();
    assert!(!error.starts_with("BudgetExhausted:"),"finite requested domain should drain: {error}");
}
#[test]
fn artificial_domain_expands_without_restarting_and_filters_cover_unknown_endpoints() {
    let map=map([]);let start=[0.05,0.05,0.0];let goal=[0.55,0.05,0.0];
    let accepts=|a:[f64;3],b:[f64;3]| {
        if a[0].min(b[0])<=0.3&&a[0].max(b[0])>=0.3 {
            (a[1]+(0.3-a[0])/(b[0]-a[0])*(b[1]-a[1])).abs()>0.7
        } else {true}
    };
    let route=map.plan_bounded_with_segment_filter(start,goal,0.0,&[],None,None,&[],&accepts).unwrap();
    assert!(route.points.iter().any(|p|p[1].abs()>0.7));
    assert!(route.points.windows(2).all(|p|accepts(p[0],p[1])));
    assert!(map.plan_bounded_with_segment_filter(start,goal,0.0,&BOUNDARY,None,None,&[],&|a,_|a!=start).is_err());
    assert!(map.plan_bounded_with_segment_filter(start,goal,0.0,&BOUNDARY,None,None,&[],&|_,b|b!=goal).is_err());
}

#[test]
fn explicit_empty_occupancy_columns_remain_unknown_not_forbidden() {
    let map=map((0..20).flat_map(|x|(0..3).map(move|y|([x,y],vec![]))));
    let route=map.plan([0.05,0.15,0.0],[1.95,0.15,0.0],0.0,&BOUNDARY,None).unwrap();
    assert_eq!(route.points.len(),2);
    assert!(map.cells.is_empty());
}
#[test]
fn unknown_corridor_respects_body_width_without_missing_ground_inflation() {
    for (half_gap,allowed) in [(0.1,false),(0.3,true)] {
        let walls=(-2..=20).flat_map(|x|[-half_gap-0.05,half_gap+0.05].map(move|y|
            ([x,(y/0.1_f64).floor() as i32],vec![[-0.1,1.0]])));
        let map=Tomogram::from_columns(0.1,walls,0.4,0.16,0.1,0.1,20.0).unwrap();
        let boundary=[[0.,-half_gap],[2.,-half_gap],[2.,half_gap],[0.,half_gap]];
        let route=map.plan([0.05,0.0,0.0],[1.95,0.0,0.0],0.0,&boundary,None);
        assert_eq!(route.is_ok(),allowed,"half_gap={half_gap}: {route:?}");
    }
}

#[test]
fn unknown_arrival_ball_can_end_inside_boundary_when_exact_xy_is_outside() {
    let map=map([]);let boundary=[[0.,0.],[0.57,0.],[0.57,0.3],[0.,0.3]];
    let route=map.plan([0.05,0.15,0.0],[0.59,0.15,0.0],0.05,&boundary,None).unwrap();
    let end=*route.points.last().unwrap();
    assert!(end[0]<=0.57);assert!(crate::distance(end,[0.59,0.15,0.0])<=0.05);
}

#[test]
fn zero_budget_is_explicit_direct_only_and_inconclusive_for_detours_or_penalties() {
    let empty=map([]);let start=[0.05,0.15,0.0];let goal=[1.95,0.15,0.0];
    assert!(empty.plan_bounded_with_limits(start,goal,0.0,&BOUNDARY,None,None,&[],SearchLimits{max_states:0}).is_ok());
    let penalty=[crate::RoutePenalty{center:[1.,0.15,0.0],radius:0.2,weight:4.0}];
    let error=empty.plan_bounded_with_limits(start,goal,0.0,&BOUNDARY,None,None,&penalty,SearchLimits{max_states:0}).unwrap_err();
    assert!(error.starts_with("BudgetExhausted:"));
    let wall=map([([10,1],vec![[-0.1,1.0]])]);
    let error=wall.plan_bounded_with_limits(start,goal,0.0,&[],None,None,&[],SearchLimits{max_states:0}).unwrap_err();
    assert!(error.starts_with("BudgetExhausted:"));
    assert!(wall.plan_bounded_with_limits(start,goal,0.0,&[],None,None,&[],SearchLimits{max_states:4096}).is_ok());
}

#[test]
fn progressive_domain_revisits_share_the_same_expansion_work_allowance() {
    let map=map([]);let start=[0.05,0.05,0.0];let goal=[0.55,0.05,0.0];
    let accepts=|a:[f64;3],b:[f64;3]| !(a[0].min(b[0])<=0.3&&a[0].max(b[0])>=0.3);
    let mut reached_expansion_limit=false;
    for maximum in [32,64,128,256,512] {
        let error=map.plan_bounded_with_segment_filter_and_limits(start,goal,0.0,&[],None,None,
            &[],&accepts,SearchLimits{max_states:maximum}).unwrap_err();
        assert!(error.starts_with("BudgetExhausted:"),"{error}");
        if error.contains("展开工作预算耗尽") {
            assert!(error.contains(&format!("展开{maximum}格")),"{error}");
            reached_expansion_limit=true;
        }
    }
    assert!(reached_expansion_limit,"progressive edge revisits must exercise the independent work guard");
}


#[test]
fn query_radius_is_inconclusive_only_at_an_admissible_artificial_frontier() {
    let wall=map((-8..8).map(|y|([3,y],vec![[-0.1,1.0]])));
    let start=[0.05,0.05,0.0];let goal=[0.55,0.05,0.0];
    let boundary=[[-0.2,-1.0],[1.0,-1.0],[1.0,1.0],[-0.2,1.0]];
    let query=|radius|wall.plan_bounded_with_limits_typed(start,goal,0.0,&boundary,None,Some(radius),
        &[],SearchLimits::default());
    assert!(matches!(query(0.7),Err(crate::PlanningFailure::SearchDomainLimited(_))),
        "a route outside the query disk has not been excluded by physical geometry");
    let route=query(1.5).expect("a larger bounded disk reaches the valid wall detour");
    assert!(route.points.iter().any(|point|point[1].abs()>0.8));
    assert!(route.points.windows(2).all(|pair|wall.segment_clear(pair[0],pair[1],&boundary)));
    let enclosed=map((-2..=2).flat_map(|x|(-2..=2).filter_map(move|y|
        ((x== -2||x==2)||(y== -2||y==2)).then(||([x,y],vec![[-0.1,1.0]])))));
    let error=enclosed.plan_bounded_with_limits_typed(start,goal,0.0,&boundary,None,Some(0.15),
        &[],SearchLimits::default()).unwrap_err();
    assert!(matches!(error,crate::PlanningFailure::Other(_)),
        "all would-be outside-radius successors are physically blocked: {error}");
    let tight=[[-0.2,-0.5],[1.0,-0.5],[1.0,0.5],[-0.2,0.5]];
    let error=wall.plan_bounded_with_limits_typed(start,goal,0.0,&tight,None,Some(1.5),
        &[],SearchLimits::default()).unwrap_err();
    assert!(matches!(error,crate::PlanningFailure::Other(_)),
        "a physical mission boundary is not an artificial horizon: {error}");
}

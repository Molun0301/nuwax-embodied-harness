use super::*;

fn config() -> Config {
    Config {
        resolution: 0.1, point_resolution: 0.025, robot_height: 0.45,
        body_to_ground: 0.4, max_step: 0.16, max_step_down: 0.08,
        footprint: vec![[-0.24,-0.14],[0.24,-0.14],[0.24,0.14],[-0.24,0.14]],
        search_margin: 0.3, ..Default::default()
    }
}
fn flat() -> Vec<Cell> {
    (-5..55).flat_map(|x| (-5..35).map(move |y| Cell {x,y,ground:0.0,ceiling:f64::INFINITY})).collect()
}
fn summarize(route: &Route) -> (f64,[f64;3],f64) {
    let cost=route.path.windows(2).map(|e|(e[1][0]-e[0][0]).hypot(e[1][1]-e[0][1]).hypot(e[1][2]-e[0][2])).sum();
    let yaw=route.path.windows(2).last().map_or(0.,|e|(e[1][1]-e[0][1]).atan2(e[1][0]-e[0][0]));
    (cost,*route.path.last().unwrap(),yaw)
}
fn rank(search: &mut MultiGoalSearch<'_>,start: [f64;3],goal: [f64;3]) -> Result<RouteCost,String> {
    search.rank_costs(start,&[goal]).remove(0)
}
#[test]
fn one_source_search_ranks_many_goals_like_separate_queries_and_reads_measured_heights() {
    let bands:Vec<_>=(0..23).map(|y|Band{x:20,y,low:0.,high:1.5}).collect();
    let mut cells:Vec<_>=flat().into_iter().filter(|c|c.x<30 || c.x>33).collect();
    cells.push(Cell{x:3,y:3,ground:0.9,ceiling:f64::INFINITY});
    let start=[0.55,0.55,0.4];
    let goals:Vec<_>=(0..40).map(|i|[0.35+(i%10) as f64*0.5,0.45+(i/10) as f64*0.7,0.4]).collect();
    let mut planner=Planner::from_cells(config(),&cells,&bands).unwrap();
    let mut search=planner.multi_goal_search().unwrap();
    let batch=search.rank_costs(start,&goals);
    assert_eq!(search.stats().source_searches,1,"同一起点只做一次单源搜索");
    assert!(batch.iter().any(Result::is_ok) && batch.iter().any(Result::is_err));
    assert_eq!(search.measured_heights([0.35,0.35]),vec![0.,0.9]);
    assert!(search.measured_heights([3.15,0.35]).is_empty(),"缺测列不能补造地面");
    drop(search);
    for (goal,batched) in goals.iter().zip(batch) {
        let mut separate=Planner::from_cells(config(),&cells,&bands).unwrap();
        let single=rank(&mut separate.multi_goal_search().unwrap(),start,*goal);
        assert_eq!(single.is_ok(),batched.is_ok(),"goal={goal:?}");
        if let (Ok(single),Ok(batched))=(single,batched) {
            assert_eq!((single.cost,single.endpoint,single.yaw),(batched.cost,batched.endpoint,batched.yaw));
        }
    }
    with_planning_cancellation(Arc::new(AtomicBool::new(true)),|| {
        assert!(planner.multi_goal_search().unwrap().rank_costs(start,&goals[..2]).iter()
            .all(|cost|cost.as_ref().unwrap_err().contains("取消")));
    });
}
fn compare(cfg: Config,cells: &[Cell],bands: &[Band],queries: &[([f64;3],[f64;3])]) -> MultiGoalStats {
    let mut independent=Planner::from_cells(cfg.clone(),cells,bands).unwrap();
    let mut shared=Planner::from_cells(cfg,cells,bands).unwrap();
    let mut search=shared.multi_goal_search().unwrap();
    // 重复与改变来源交错，覆盖区域候选之间的查询，避免仅验证同一目标缓存。
    for &(start,goal) in queries.iter().chain(queries.iter().rev()) {
        let expected=independent.plan(start,goal);
        let actual=search.cost(start,goal);
        assert_eq!(actual.is_ok(),expected.is_ok(),"start={start:?} goal={goal:?}, actual={actual:?}, expected={expected:?}");
        if let (Ok(actual),Ok(route))=(actual,expected) {
            let (cost,endpoint,yaw)=summarize(&route);
            assert!((actual.cost-cost).abs()<1e-10,"cost: {actual:?}, {cost}");
            assert_eq!(actual.endpoint,endpoint);
            assert_eq!(actual.yaw,yaw);
            assert_eq!(actual.expanded,route.expanded);
        }
    }
    search.stats()
}
#[test]
fn multi_goal_matches_independent_queries_in_soft_clearance_and_offset_endpoints() {
    let bands:Vec<_>=(0..23).map(|y|Band{x:20,y,low:0.,high:1.5}).collect();
    let queries=[([0.52,0.48,0.4],[4.37,0.53,0.4]),([0.52,0.48,0.4],[1.37,2.28,0.4]),
        ([0.52,0.48,0.4],[4.33,2.73,0.4]),([4.33,2.73,0.4],[0.52,0.48,0.4]),
        ([0.52,0.48,0.4],[0.56,0.49,0.4])];
    let stats=compare(config(),&flat(),&bands,&queries);
    assert!(stats.hits>0);
    assert_eq!(stats.source_searches,0,"完整查询不做单源搜索");
}
#[test]
fn multi_goal_matches_independent_queries_across_distinct_floors() {
    let cells:Vec<_>=flat().into_iter().flat_map(|c|[Cell{ceiling:0.95,..c},Cell{ground:1.,..c}]).collect();
    let bands:Vec<_>=flat().iter().map(|c|Band{x:c.x,y:c.y,low:0.95,high:1.}).collect();
    let stats=compare(config(),&cells,&bands,&[
        ([0.55,0.55,0.4],[4.45,0.55,1.4]),([0.55,0.55,0.4],[4.45,0.55,0.4]),
        ([0.55,0.55,1.4],[4.45,0.55,1.4])]);
    assert_eq!(stats.exact_queries,6);
}
#[test]
fn multi_goal_matches_independent_queries_at_narrow_doors() {
    for door in [1,5] {
        let bands:Vec<_>=(-5..35).filter(|y|*y<14 || *y>=14+door)
            .map(|y|Band{x:20,y,low:0.,high:2.}).collect();
        compare(config(),&flat(),&bands,&[([0.55,1.55,0.4],[4.45,1.55,0.4]),
            ([0.55,1.55,0.4],[1.76,1.57,0.4]),([0.55,1.55,0.4],[1.84,1.59,0.4])]);
    }
}
#[test]
fn multi_goal_never_crosses_missing_ground() {
    let cells:Vec<_>=flat().into_iter().filter(|c|c.x<18 || c.x>27).collect();
    let stats=compare(config(),&cells,&[],&[([0.55,0.55,0.4],[4.45,0.55,0.4]),
        ([0.55,0.55,0.4],[1.45,0.55,0.4]),([0.55,0.55,0.4],[2.25,0.55,0.4])]);
    assert_eq!(stats.exact_queries,6);
}
#[test]
fn multi_goal_matches_independent_directional_steps_and_continuous_slopes() {
    for gradient in [false,true] {
        let cells:Vec<_>=flat().into_iter().map(|c|Cell{ground:if gradient{c.x as f64*0.025}else if c.x>=20{0.12}else{0.},..c}).collect();
        let mut cfg=config();cfg.max_slope=10_f64.to_radians();
        let height=|x:f64|if gradient{(x/0.1).floor()*0.025+0.4}else if x>=2.{0.52}else{0.4};
        compare(cfg,&cells,&[],&[([0.55,0.55,height(0.55)],[4.45,0.55,height(4.45)]),
            ([4.45,0.55,height(4.45)],[0.55,0.55,height(0.55)])]);
    }
}
#[test]
fn multi_goal_cancellation_reaches_cold_and_warm_search_then_recovers() {
    let mut planner=Planner::from_cells(config(),&flat(),&[]).unwrap();
    let start=[0.55,0.55,0.4];let goal=[4.45,0.55,0.4];
    let flag=AtomicBool::new(true);
    {
        let mut search=planner.multi_goal_search().unwrap();
        assert!(search.cost_cancellable(start,goal,&flag).unwrap_err().contains("取消"));
        flag.store(false,Ordering::Release);
        assert!(search.cost_cancellable(start,goal,&flag).is_ok());
        flag.store(true,Ordering::Release);
        assert!(search.cost_cancellable(start,goal,&flag).unwrap_err().contains("取消"));
        flag.store(false,Ordering::Release);
        assert!(search.cost_cancellable(start,goal,&flag).is_ok());
        with_planning_cancellation(Arc::new(AtomicBool::new(true)),|| {
            assert!(search.cost(start,goal).unwrap_err().contains("取消"));
        });
    }
    assert!(planner.plan(start,goal).is_ok());
    assert_eq!(planner.plan_progressive(start,[8.,0.55,0.4],None).unwrap().kind,RouteKind::Approach);
}

#[test]
fn single_source_expansion_stops_during_search_and_resets_for_navigation() {
    unsafe extern "C" { fn pct_single_source_cancellation_regression() -> bool; }
    assert!(unsafe { pct_single_source_cancellation_regression() });
}
#[test]
fn multi_goal_falls_back_when_source_snap_or_surface_representation_changes() {
    let mut cfg=config();cfg.search_margin=0.2;
    let cells:Vec<_>=flat().into_iter().filter(|c|c.x>=3).flat_map(|c|[
        Cell {ground:0.19,ceiling:0.55,..c},Cell{ground:0.63,..c}
    ]).collect();
    compare(cfg,&cells,&[],&[([0.04,0.04,0.61],[4.44,0.46,1.03]),
        ([0.04,0.04,0.61],[0.35,0.45,0.61]),([0.04,0.04,0.61],[4.44,0.46,-0.8])]);
}

#[test]
fn source_tree_bounds_many_reachable_goals_and_preserves_exact_navigation() {
    let mut planner=Planner::from_cells(config(),&flat(),&[]).unwrap();
    let start=[0.55,0.55,0.4];
    {
        let mut search=planner.multi_goal_search().unwrap();
        let goals:Vec<_>=std::iter::once([1.55,1.55,0.4]).chain((0..128).map(|i|
            [1.05+(i%35) as f64*0.1,0.65+(i%20) as f64*0.1,0.4])).collect();
        let routes=search.rank_costs(start,&goals);
        assert!(routes[0].as_ref().unwrap().cost>=2_f64.sqrt()-1e-10);
        for (goal,route) in goals.iter().zip(routes) {
            let route=route.unwrap();
            assert_eq!(route.endpoint,*goal);
            assert!(route.cost+1e-10>=((goal[0]-start[0]).powi(2)+(goal[1]-start[1]).powi(2)).sqrt());
        }
        assert_eq!(search.stats().source_searches,1);
        assert_eq!(search.stats().exact_queries,0);
        search.cost(start,[3.55,1.55,0.4]).unwrap();
        assert_eq!(search.stats().exact_queries,1);
    }
    assert!(planner.plan(start,[3.55,1.55,0.4]).is_ok());
}

#[test]
fn source_tree_uses_real_detours_missing_ground_floors_and_endpoint_offsets() {
    let cfg=config();
    let bands:Vec<_>=(0..23).map(|y|Band{x:20,y,low:0.,high:1.5}).collect();
    let mut planner=Planner::from_cells(cfg.clone(),&flat(),&bands).unwrap();
    let mut search=planner.multi_goal_search().unwrap();
    let route=rank(&mut search,[0.55,0.55,0.4],[4.37,0.53,0.4]).unwrap();
    // 墙的下端在y=0，上端在y=2.3；任意绕行都至少到达其中一个端点高度。
    let minimum_detour=1.45_f64.hypot(0.55)+2.37_f64.hypot(0.53);
    assert!(route.cost>=minimum_detour,"必须绕过真实墙体：{route:?}");
    assert_eq!(route.endpoint,[4.37,0.53,0.4]);
    let cells:Vec<_>=flat().into_iter().filter(|c|c.x<18||c.x>27).collect();
    let mut planner=Planner::from_cells(cfg.clone(),&cells,&[]).unwrap();
    assert!(rank(&mut planner.multi_goal_search().unwrap(),[0.55,0.55,0.4],[4.45,0.55,0.4]).is_err());
    let cells:Vec<_>=flat().into_iter().flat_map(|c|[Cell{ceiling:0.95,..c},Cell{ground:1.,..c}]).collect();
    let mut planner=Planner::from_cells(cfg,&cells,&[]).unwrap();
    let mut search=planner.multi_goal_search().unwrap();
    assert!(rank(&mut search,[0.55,0.55,0.4],[4.45,0.55,1.4]).is_err());
    assert!(rank(&mut search,[0.55,0.55,1.4],[4.45,0.55,1.4]).is_ok());
}

#[test]
fn source_tree_keeps_directional_step_limits_and_owner_cancellation() {
    let cells:Vec<_>=flat().into_iter().map(|c|Cell{ground:if c.x>=20 {0.12}else{0.},..c}).collect();
    let mut cfg=config();cfg.max_slope=0.;cfg.max_step=0.15;cfg.max_step_down=0.;
    let mut planner=Planner::from_cells(cfg,&cells,&[]).unwrap();
    let mut search=planner.multi_goal_search().unwrap();
    assert!(rank(&mut search,[0.55,0.55,0.4],[4.45,0.55,0.52]).is_ok());
    assert!(rank(&mut search,[4.45,0.55,0.52],[0.55,0.55,0.4]).is_err());
    with_planning_cancellation(Arc::new(AtomicBool::new(true)),|| {
        assert!(rank(&mut search,[4.45,0.55,0.52],[3.45,0.55,0.52]).unwrap_err().contains("取消"));
    });
    assert!(rank(&mut search,[4.45,0.55,0.52],[3.45,0.55,0.52]).is_ok());
}

#[test]
fn source_search_settles_goals_in_path_cost_order_and_expands_only_as_far_as_needed() {
    let bands:Vec<_>=(0..23).map(|y|Band{x:20,y,low:0.,high:1.5}).collect();
    let cells:Vec<_>=flat().into_iter().filter(|c|c.x<40 || c.x>42).collect();
    let start=[0.55,0.55,0.4];
    // 近处、绕墙、缺测带对岸、起点格内、缺测端点。
    let goals=[[1.05,0.95,0.4],[0.55,1.95,0.4],[3.05,0.45,0.4],[4.75,2.95,0.4],[0.56,0.56,0.4],[20.,20.,0.4],[2.55,2.95,0.4]];
    let mut planner=Planner::from_cells(config(),&cells,&bands).unwrap();
    let mut search=planner.multi_goal_search().unwrap();
    assert_eq!(search.begin_source(start,&goals).unwrap(),vec![true,true,true,true,true,false,true],
        "缺测端点没有节点");
    let mut order=Vec::new();
    while let Some(settled)=search.next_settled(f64::INFINITY).unwrap() {
        order.push((settled,search.stats().source_expanded));
    }
    let mut indices:Vec<_>=order.iter().map(|(settled,_)|settled.index).collect();
    assert_eq!(indices[0],4,"起点格内目标最先定值");
    indices.sort();
    assert_eq!(indices,vec![0,1,2,4,6],"可达目标各定值一次，缺测带对岸展开完也不定值");
    assert!(order.windows(2).all(|pair|pair[0].0.weighted<=pair[1].0.weighted),"按加权路径代价从小到大定值");
    let total=search.stats().source_expanded;
    let (near,at)=order.iter().find(|(settled,_)|settled.index==0).unwrap();
    assert!(at*4<total,"近处目标定值时只展开了附近：{at}/{total}");
    for (settled,_) in &order {
        let goal=goals[settled.index];
        assert_eq!(settled.route.endpoint,goal);
        assert!(settled.route.cost+1e-10>=(goal[0]-start[0]).hypot(goal[1]-start[1]));
        let mut separate=Planner::from_cells(config(),&cells,&bands).unwrap();
        let single=rank(&mut separate.multi_goal_search().unwrap(),start,goal).unwrap();
        assert_eq!((single.cost,single.yaw),(settled.route.cost,settled.route.yaw));
    }
    assert!(near.route.cost<order.iter().find(|(settled,_)|settled.index==2).unwrap().0.route.cost);
    // 墙后目标必须绕过墙。
    let behind=order.iter().find(|(settled,_)|settled.index==2).unwrap().0.route.cost;
    assert!(behind>=1.45_f64.hypot(2.25)+1.05_f64.hypot(2.3-0.45));
    // 完整查询不受单源搜索影响。
    let mut independent=Planner::from_cells(config(),&cells,&bands).unwrap();
    let expected=summarize(&independent.plan(start,goals[2]).unwrap());
    let exact=search.cost(start,goals[2]).unwrap();
    assert!((exact.cost-expected.0).abs()<1e-10);
    assert_eq!((exact.endpoint,exact.yaw),(expected.1,expected.2));
}

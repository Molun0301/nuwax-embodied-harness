//! 现有表面地图到独立层析核心的适配与缓存。
use crate::{distance, RoutePenalty, Tomogram};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use std::sync::{Arc, Mutex};

/// 每个导航拥有方只保存当前一份派生图；观察时间变化不会重新构图。
#[derive(Default)]
pub struct PctPlanner {
    cached: Mutex<Option<(SurfaceSnapshot, Arc<Tomogram>)>>,
    local_costs: Mutex<Vec<RoutePenalty>>,
    pub(crate) search_limits: crate::SearchLimits,
}

#[cfg(test)]
mod body_route_cancellation_tests {
    use super::*;
    use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
    use nuwax_nav_types_messages::GoalOptions;
    use std::sync::atomic::{AtomicBool, Ordering};

    fn source() -> SurfaceSnapshot {
        crate::sampling_tests::sampled_room(true, false).with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.4,
            max_step_up_m: 0.15, max_step_down_m: 0.15,
            max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
        }).unwrap()
    }

    #[test]
    fn prepared_reference_identity_includes_geometry_config_and_materialization_domain() {
        let source = source();
        let map = Tomogram::from_surface(&source).unwrap();
        assert!(map.matches_surface(&source.clone()));
        let recent = source.with_recent_obstacle_points(1,100,0,&[],[(0,[0.0,0.0,0.6])]).unwrap();
        assert!(map.matches_surface(&recent),"recent geometry is a separate live execution check");
        let cropped = Tomogram::local_from_surface(&source,[0.0,0.0],1.0).unwrap();
        assert!(!cropped.matches_surface(&source),"a crop cannot stand in for the full prepared source");
        let mut model = source.body_envelope().unwrap().clone();model.max_step_up_m *= 0.5;
        let changed = source.clone().with_body_envelope(model).unwrap();
        assert!(!map.matches_surface(&changed),"immutable config/model identity is part of geometry admission");
    }

    #[test]
    fn tilted_start_and_goal_use_one_body_center_guide_transform() {
        let source = source();let planner = PctPlanner::default();
        let start = ([-0.03,0.0,0.462],[(0.35_f64*0.5).sin(),0.0,0.0,(0.35_f64*0.5).cos()]);
        let goal = ([0.03,0.0,0.462],[0.0,(-0.3_f64*0.5).sin(),0.0,(-0.3_f64*0.5).cos()]);
        let options = GoalOptions {position_tolerance_m:0.001,..GoalOptions::default()};
        let route = planner.body_route(&source,start,goal,options,&[],None).unwrap();
        assert_eq!(route[0],start,"Scan receives the actual complete initial pose");
        assert!(crate::distance(route.last().unwrap().0,goal.0)<=options.position_tolerance_m+1e-9);
        assert!(route.iter().all(|p|p.0[0]>=start.0[0]-1e-9&&p.0[0]<=goal.0[0]+1e-9));
        assert!(route.iter().all(|p|p.0[1].abs()<1e-9&&(p.0[2]-start.0[2]).abs()<1e-9));
    }

    #[test]
    fn body_route_preserves_calibrated_bias_and_original_tight_goal_sphere() {
        let source = source();
        let planner = PctPlanner::default();
        let start = ([-0.5,0.0,0.462],[0.0,0.0,0.0,1.0]);
        let goal = ([0.5,0.0,0.462],start.1);
        let options = GoalOptions { position_tolerance_m: 0.001, ..GoalOptions::default() };
        let route = planner.body_route(&source,start,goal,options,&[],None).unwrap();
        assert!(crate::distance(route.last().unwrap().0,goal.0)<=options.position_tolerance_m+1e-9);
        assert!(route.iter().all(|pose|(pose.0[2]-start.0[2]).abs()<1e-9));
        let wrong_layer = ([goal.0[0],goal.0[1],goal.0[2]+0.2],goal.1);
        assert!(planner.body_route(&source,start,wrong_layer,options,&[],None).is_err(),
            "calibration transport cannot widen the original 3D goal sphere");
    }

    #[test]
    fn body_route_uses_configured_and_explicit_request_budgets() {
        let source = source();
        let planner = PctPlanner::default().with_search_limits(crate::SearchLimits { max_states: 1 });
        planner.report_local_blockage([0.0,0.0,0.05],0.2);
        let start = ([-0.5,0.0,0.45],[0.0,0.0,0.0,1.0]);
        let goal = ([0.5,0.0,0.45],start.1);
        let error = planner.body_route(&source,start,goal,GoalOptions::default(),&[],None).unwrap_err();
        assert!(error.starts_with("BudgetExhausted:"),"{error}");
        assert!(planner.body_route_with_limits(&source,start,goal,GoalOptions::default(),&[],None,
            crate::SearchLimits { max_states: 4096 }).is_ok());
    }

    #[test]
    fn already_cancelled_near_and_distant_goals_do_not_build_or_replace_cache() {
        let source = source();
        let cancelled = AtomicBool::new(true);
        let start = ([-0.5,0.0,0.45], [0.0,0.0,0.0,1.0]);
        for x in [-0.49, 0.5] {
            let planner = PctPlanner::default();
            let goal = ([x,0.0,0.45], start.1);
            let error = planner.body_route(&source, start, goal, GoalOptions::default(), &[], Some(&cancelled)).unwrap_err();
            assert_eq!(error, "层析规划已取消");
            assert!(planner.cached.lock().unwrap().is_none());
            let cached = planner.map(&source).unwrap();
            assert!(planner.body_route(&source, start, goal, GoalOptions::default(), &[], Some(&cancelled)).is_err());
            assert!(Arc::ptr_eq(&cached, &planner.cached.lock().unwrap().as_ref().unwrap().1));
        }
    }

    #[test]
    fn flag_cancelled_during_build_rejects_route_after_preserving_complete_cache() {
        let source = source();
        let planner = PctPlanner::default();
        let cancelled = Arc::new(AtomicBool::new(false));
        let flip = cancelled.clone();
        // Entry, map and source checks precede the countdown's tenth point;
        // it fires while from_columns_with_evidence consumes actual columns.
        let result = crate::cancellation::after_checkpoint_action(10,
            move || flip.store(true, Ordering::Release), ||
            planner.body_route(&source, ([-0.5,0.0,0.45],[0.0,0.0,0.0,1.0]),
                ([0.5,0.0,0.45],[0.0,0.0,0.0,1.0]), GoalOptions::default(), &[], Some(&cancelled)));
        assert!(cancelled.load(Ordering::Acquire));
        assert_eq!(result.unwrap_err(), "层析规划已取消");
        // Existing limitation: AtomicBool does not interrupt tomogram construction.
        let cached = planner.cached.lock().unwrap().as_ref().unwrap().1.clone();
        assert!(!cached.cells.is_empty());
        assert!(Arc::ptr_eq(&cached, &planner.map(&source).unwrap()));
        cancelled.store(false, Ordering::Release);
        assert!(planner.body_route(&source, ([-0.5,0.0,0.45],[0.0,0.0,0.0,1.0]),
            ([0.5,0.0,0.45],[0.0,0.0,0.0,1.0]), GoalOptions::default(), &[], Some(&cancelled)).is_ok());
    }

    #[test]
    fn scoped_owned_flag_aborts_real_columns_without_replacing_complete_cache() {
        let source = source();
        let previous = source.clone().with_ground_evidence(
            nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume);
        assert!(!same_source_geometry(&source, &previous));
        for populated in [false, true] {
            let planner = PctPlanner::default();
            let old = populated.then(|| planner.map(&previous).unwrap());
            let cancelled = Arc::new(AtomicBool::new(false));
            let flip = cancelled.clone();
            let token = crate::PlanningCancellation::from_flag(cancelled.clone());
            // Same real column checkpoint as the legacy limitation test above.
            let result = crate::cancellation::after_checkpoint_action(10,
                move || flip.store(true, Ordering::Release), ||
                crate::with_planning_cancellation(&token, ||
                    planner.body_route(&source, ([-0.5,0.0,0.45],[0.0,0.0,0.0,1.0]),
                        ([0.5,0.0,0.45],[0.0,0.0,0.0,1.0]), GoalOptions::default(), &[], None)));
            assert!(cancelled.load(Ordering::Acquire));
            assert_eq!(result.unwrap_err(), "层析规划已取消");
            let cached = planner.cached.lock().unwrap();
            match old.as_ref() {
                Some(old) => {
                    let (retained_source, retained_map) = cached.as_ref().unwrap();
                    assert!(same_source_geometry(retained_source, &previous));
                    assert!(Arc::ptr_eq(old, retained_map));
                }
                None => assert!(cached.is_none()),
            }
            drop(cached);
            let fresh = crate::PlanningCancellation::from_flag(Arc::new(AtomicBool::new(false)));
            let rebuilt = crate::with_planning_cancellation(&fresh, || planner.map(&source)).unwrap();
            assert!(!rebuilt.cells.is_empty());
            assert!(Arc::ptr_eq(&rebuilt, &planner.map(&source).unwrap()));
            assert!(!crate::planning_cancelled());
        }
    }

    #[test]
    fn scoped_owned_flag_aborts_connect_and_costs_without_cache_publication() {
        let source = source();
        let previous = source.clone().with_ground_evidence(
            nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume);
        // Freeze the fixture's exact checkpoint layout, without adding production probes.
        let (_, columns) = source.navigation_source_columns(None);
        let input_checks: usize = columns.map(|(_, mut bands)| {
            bands.sort_unstable_by(|a, b| a[0].total_cmp(&b[0]));
            assert!(bands.iter().all(|b| b[0].is_finite() && b[1].is_finite() && b[0] < b[1]));
            assert!(bands.windows(2).all(|p| p[1][0] > p[0][1] + 1e-6));
            1 + 2 * bands.len()
        }).sum();
        let cells = Tomogram::from_surface(&source).unwrap().cells.len();
        assert!(cells > 17);
        // map entry, three surface checks, from_columns entry, post-input precede connect.
        let connect = 6 + input_checks;
        let costs = connect + cells + 1;
        for (phase, count) in [("connect", connect + 17), ("costs", costs + 17)] {
            for populated in [false, true] {
                let planner = PctPlanner::default();
                let old = populated.then(|| planner.map(&previous).unwrap());
                let flag = Arc::new(AtomicBool::new(false));
                let flip = flag.clone();
                let token = crate::PlanningCancellation::from_flag(flag.clone());
                let result = crate::cancellation::after_checkpoint_action(count,
                    move || flip.store(true, Ordering::Release), ||
                    crate::with_planning_cancellation(&token, || planner.map(&source)));
                assert!(flag.load(Ordering::Acquire), "{phase} checkpoint was not reached");
                assert_eq!(result.unwrap_err(), "层析规划已取消", "{phase}");
                let cached = planner.cached.lock().unwrap();
                match old.as_ref() {
                    Some(old) => {
                        let (retained_source, retained_map) = cached.as_ref().unwrap();
                        assert!(same_source_geometry(retained_source, &previous));
                        assert!(Arc::ptr_eq(old, retained_map), "{phase}");
                    }
                    None => assert!(cached.is_none(), "{phase}"),
                }
                assert!(!crate::planning_cancelled());
            }
        }
    }
}
impl std::fmt::Debug for PctPlanner {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("PctPlanner")
    }
}
/// 占据几何相同即可复用计算，真实观察时间仍由执行端独立消费。
pub fn same_source_geometry(a: &SurfaceSnapshot, b: &SurfaceSnapshot) -> bool {
    a.same_geometry(b)
        && match (a.observations(), b.observations()) {
            (Some(a), Some(b)) => a.same_geometry(b),
            (None, None) => true,
            _ => false,
        }
}
impl PctPlanner {
    /// Configure the actual global body-route request budget. This bounds work,
    /// not physical reachability; BudgetExhausted is safe to retry with more work.
    pub fn with_search_limits(mut self, limits: crate::SearchLimits) -> Self {
        self.search_limits = limits;
        self
    }
    /// The configured per-query resource ceiling, independent of physical reachability.
    pub fn search_limits(&self) -> crate::SearchLimits { self.search_limits }
    pub fn clear_local_costs(&self) {
        self.local_costs
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
    }
    pub fn local_costs(&self) -> Vec<RoutePenalty> {
        self.local_costs
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
    pub fn report_local_blockage(&self, center: [f64; 3], radius: f64) {
        let mut costs = self.local_costs.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(previous) = costs
            .iter_mut()
            .find(|p| distance(p.center, center) < radius * 0.5)
        {
            previous.weight = (previous.weight * 2.0).min(256.0);
        } else {
            costs.push(RoutePenalty {
                center,
                radius,
                weight: 16.0,
            });
        }
        tracing::debug!(target:"navigation.pct",event="navigation.pct.local_cost",?center,radius,
            regions=costs.len(),"局部接续反馈已加入当前目标的路线代价");
    }
    pub fn map(&self, source: &SurfaceSnapshot) -> Result<Arc<Tomogram>, String> {
        crate::cancellation::checkpoint()?;
        {
            let cached = self.cached.lock().unwrap_or_else(|e| e.into_inner());
            if let Some((previous, map)) = cached
                .as_ref()
                .filter(|(previous, _)| same_source_geometry(previous, source))
            {
                let _ = previous;
                crate::cancellation::checkpoint()?;
                return Ok(Arc::clone(map));
            }
        }
        // 构建在锁外执行；缓存只负责交付，不阻塞另一个已取得快照的搜索。
        let began = std::time::Instant::now();
        let map = Arc::new(Tomogram::from_surface(source)?);
        tracing::info!(target: "navigation.pct", event="navigation.pct.map", cells=map.cells.len(),
            elapsed_ms=began.elapsed().as_secs_f64()*1000.0, "层析地图构建完成");
        crate::cancellation::checkpoint()?;
        let old = self
            .cached
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .replace((source.clone(), Arc::clone(&map)));
        drop(old);
        Ok(map)
    }
}

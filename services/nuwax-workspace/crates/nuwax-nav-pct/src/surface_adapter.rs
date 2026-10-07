//! 现有表面地图到独立层析核心的适配与缓存。
use crate::{distance, RoutePenalty, Tomogram};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use std::sync::{Arc, Mutex};

/// 每个导航拥有方只保存当前一份派生图；观察时间变化不会重新构图。
#[derive(Default)]
pub struct PctPlanner {
    cached: Mutex<Option<(SurfaceSnapshot, Arc<Tomogram>)>>,
    local_costs: Mutex<Vec<RoutePenalty>>,
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

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

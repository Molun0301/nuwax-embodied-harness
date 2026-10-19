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
            fixed_collision: None,
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
    nuwax_nav_mls_planner::surface::same_source_geometry(a, b)
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
        self.map_with_delta(source, crate::SurfaceDelta::Rebuild)
    }
    /// Consume the source owner's complete delta using the existing immutable cache.
    pub fn map_with_delta(&self, source: &SurfaceSnapshot, delta: crate::SurfaceDelta<'_>)
        -> Result<Arc<Tomogram>, String>
    {
        crate::cancellation::checkpoint()?;
        let previous = {
            let cached = self.cached.lock().unwrap_or_else(|e| e.into_inner());
            if let Some((_, map)) = cached
                .as_ref()
                .filter(|(previous, _)| same_source_geometry(previous, source))
            {
                crate::cancellation::checkpoint()?;
                return Ok(Arc::clone(map));
            }
            cached.as_ref().map(|(_, map)| Arc::clone(map))
        };
        // 构建在锁外执行；缓存只负责交付，不阻塞另一个已取得快照的搜索。
        let began = std::time::Instant::now();
        let map = Arc::new(match previous {
            Some(previous) => previous.updated_from_surface(source, delta)?.0,
            None => Tomogram::from_surface(source)?,
        });
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

#[cfg(test)]
mod delta_cache_tests {
    use super::*;
    use crate::{Cell, PlanningCancellation, SurfaceDelta};
    use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations, mls_planner::Config};
    use nuwax_nav_mls_planner::surface::GroundKey;
    use std::collections::{BTreeMap, BTreeSet};
    use std::sync::atomic::{AtomicBool, Ordering};

    const Q: f32 = 0.125;
    type Values = BTreeMap<[i32; 3], SurfaceObservation>;
    type Identity = ([i32; 2], u64, Option<GroundKey>);

    fn fixture() -> (SurfaceSnapshot, SurfaceObservations, Values) {
        let values: Values = (0..8).flat_map(|x| (0..8).map(move |y| {
            ([x,y,0], SurfaceObservation {observed_ns:10,hits:2})
        })).collect();
        let points: Vec<_> = values.keys().map(|key| (
            (key[0] as f32+0.5)*Q,(key[1] as f32+0.5)*Q,(key[2] as f32+0.5)*Q,
        )).collect();
        let config = Config {
            world_frame:"map".into(),voxel_size:0.25,robot_height:0.5,
            surface_closing_radius:0.0,node_spacing_m:0.5,wall_clearance_m:0.0,
            wall_buffer_m:0.0,wall_buffer_weight:0.0,max_slope_deg:0.0,
            step_threshold_m:0.0,min_step_tread_depth_m:0.5,step_penalty_weight:1.0,
            goal_tolerance:0.1,viz_publish_hz:0.0,
        };
        let base=SurfaceSnapshot::from_points(&points,config,Q).unwrap();
        let mut observations=SurfaceObservations::new(Q,32).unwrap();
        observations.replace_block([0,0,0],values.clone()).unwrap();
        observations.mark_complete_geometry_source();
        (base.with_observations(Some(Arc::new(observations.clone()))),observations,values)
    }

    fn replace(source:&SurfaceSnapshot,observations:&mut SurfaceObservations,values:&Values)
        -> (SurfaceSnapshot,Vec<[i32;2]>) {
        let changed=observations.replace_block_with_geometry_changes([0,0,0],values.clone()).unwrap();
        observations.mark_complete_geometry_source();
        (source.with_observations(Some(Arc::new(observations.clone()))),changed)
    }
    fn identity(cell:&Cell)->Identity {(cell.column,cell.ground.to_bits(),cell.source_ground)}
    fn identities(map:&Tomogram)->Vec<Identity> {map.cells.iter().map(identity).collect()}

    // Dense IDs may move during deletion. Compare all graph values by source identity,
    // preserving column/neighbor/FIFO order and exact floating bits, then the real route.
    fn exact(actual:&Tomogram,cold:&Tomogram) {
        let parameters=|m:&Tomogram|[m.resolution,m.source_quantum,m.body_height,m.half_width,
            m.step_up,m.step_down,m.slope,m.clearance_radius,m.minimum_cost].map(f64::to_bits);
        assert_eq!(parameters(actual),parameters(cold));
        assert_eq!(actual.occupied_volume,cold.occupied_volume);
        assert_eq!(actual.known_columns,cold.known_columns);
        assert_eq!(actual.xy_bounds,cold.xy_bounds);
        assert_eq!(actual.ground_bounds.map(|v|v.map(f64::to_bits)),cold.ground_bounds.map(|v|v.map(f64::to_bits)));
        assert_eq!(actual.window,cold.window);
        let a=identities(actual);let b=identities(cold);
        let indexed=|ids:&[Identity]|ids.iter().copied().enumerate().map(|(id,key)|(key,id)).collect::<BTreeMap<_,_>>();
        let ai=indexed(&a);let bi=indexed(&b);
        assert_eq!(ai.len(),a.len());assert_eq!(bi.len(),b.len());
        assert_eq!(ai.keys().collect::<Vec<_>>(),bi.keys().collect::<Vec<_>>());
        for (key,&id) in &ai {
            let left=&actual.cells[id];let right=&cold.cells[bi[key]];
            let bits=|c:&Cell|[c.ground,c.ground_interval[0],c.ground_interval[1],c.ceiling,
                c.clearance,c.support_clearance,c.cost,c.terrain,c.terrain_upper].map(f64::to_bits);
            assert_eq!((left.layer,left.ground_approximation,bits(left)),
                (right.layer,right.ground_approximation,bits(right)),"{key:?}");
            assert_eq!(left.neighbors.iter().map(|&id|a[id]).collect::<Vec<_>>(),
                right.neighbors.iter().map(|&id|b[id]).collect::<Vec<_>>(),"{key:?}");
        }
        let columns=|m:&Tomogram,ids:&[Identity]|m.columns.iter().map(|(&xy,values)|
            (xy,values.iter().map(|&id|ids[id]).collect::<Vec<_>>())).collect::<BTreeMap<_,_>>();
        assert_eq!(columns(actual,&a),columns(cold,&b));
        let occupancy=|m:&Tomogram|m.occupancy.iter().map(|(&xy,bands)|
            (xy,bands.iter().map(|v|v.map(f64::to_bits)).collect::<Vec<_>>())).collect::<BTreeMap<_,_>>();
        assert_eq!(occupancy(actual),occupancy(cold));
        for (map,ids) in [(actual,&a),(cold,&b)] {
            assert_eq!(map.seed_order.ids().collect::<BTreeSet<_>>(),(0..ids.len()).collect());
        }
        assert_eq!(actual.seed_order.ids().map(|id|a[id]).collect::<Vec<_>>(),
            cold.seed_order.ids().map(|id|b[id]).collect::<Vec<_>>());
        let point=|xy:[i32;2]|[(f64::from(xy[0])+0.5)*f64::from(Q),
            (f64::from(xy[1])+0.5)*f64::from(Q),f64::from(Q)];
        let start=point([1,1]);let goal=point([6,6]);
        let ar=actual.plan(start,goal,0.001,&[],None).unwrap();
        let br=cold.plan(start,goal,0.001,&[],None).unwrap();
        assert_eq!(ar.points.iter().map(|p|p.map(f64::to_bits)).collect::<Vec<_>>(),
            br.points.iter().map(|p|p.map(f64::to_bits)).collect::<Vec<_>>());
        assert_eq!((ar.stats.expanded,ar.stats.cells),(br.stats.expanded,br.stats.cells));
        for (map,route) in [(actual,&ar),(cold,&br)] {
            assert!(route.points.windows(2).all(|pair|map.segment_clear(pair[0],pair[1],&[])));
        }
    }

    fn retained(planner:&PctPlanner,source:&SurfaceSnapshot,map:&Arc<Tomogram>) {
        let cached=planner.cached.lock().unwrap();
        let (cached_source,cached_map)=cached.as_ref().expect("完整旧缓存仍在");
        assert!(same_source_geometry(cached_source,source));
        assert!(Arc::ptr_eq(cached_map,map));
    }

    #[test]
    fn cache_complete_column_delta_includes_deletion_and_matches_cold_graph_and_route() {
        let (source,mut observations,mut values)=fixture();
        let planner=PctPlanner::default();let old=planner.map(&source).unwrap();
        let frozen=old.as_ref().clone();
        let last=identity(&old.cells[old.cells.len()-1]);
        // Dense IDs follow the actual build, not a fixed XY order. Select two
        // existing non-tail columns while preserving the route endpoints. The
        // first column is retired first; its slot survives both removals, so the
        // original last cell must really move there (a cold rebuild cannot pass).
        let mut candidates=old.cells.iter().enumerate()
            .filter(|(id,cell)| *id<old.cells.len()-2
                && cell.column!=[1,1] && cell.column!=[6,6])
            .map(|(id,cell)|(cell.column,id)).collect::<Vec<_>>();
        candidates.sort_unstable();
        let (deleted_xy,deleted_id)=candidates[0];
        let (occupied_xy,_)=candidates[1];
        assert_ne!(last.0,deleted_xy);assert_ne!(last.0,occupied_xy);
        eprintln!("dense relocation: delete={deleted_xy:?}/id={deleted_id}, change={occupied_xy:?}, last={last:?}");
        values.remove(&[deleted_xy[0],deleted_xy[1],0]);
        values.insert([occupied_xy[0],occupied_xy[1],2],SurfaceObservation{observed_ns:20,hits:2});
        let (next_source,changed)=replace(&source,&mut observations,&values);
        assert_eq!(changed,vec![deleted_xy,occupied_xy]);
        let next=planner.map_with_delta(&next_source,SurfaceDelta::Columns {
            basis:&source,resolution:f64::from(Q),columns:&changed,
        }).unwrap();
        assert!(!Arc::ptr_eq(&old,&next));
        assert_eq!(identity(&next.cells[deleted_id]),last,
            "适配器必须消费旧图的真实删除/ID搬移，不能总是静默冷建");
        assert!(!next.known_columns.contains(&deleted_xy));
        assert!(!next.occupancy.contains_key(&deleted_xy));
        assert_eq!(next_source.observations().unwrap().column_levels(deleted_xy[0],deleted_xy[1]),None);
        assert_eq!(next_source.observations().unwrap().column_levels(occupied_xy[0],occupied_xy[1]),Some([0,2].as_slice()));
        assert!(next.matches_surface(&next_source));
        exact(&next,&Tomogram::from_surface(&next_source).unwrap());
        exact(&old,&frozen);
        retained(&planner,&next_source,&next);
        assert!(Arc::ptr_eq(&next,&planner.map(&next_source).unwrap()));
    }

    #[test]
    fn cache_metadata_only_delta_reuses_arc_then_accepts_the_current_basis() {
        let (source,mut observations,mut values)=fixture();
        let planner=PctPlanner::default();let old=planner.map(&source).unwrap();
        for value in values.values_mut() {value.observed_ns=900;value.hits=17;}
        let (timed,changed)=replace(&source,&mut observations,&values);
        assert!(changed.is_empty());assert!(same_source_geometry(&source,&timed));
        assert_eq!(timed.observations().unwrap().get([3,3,0]).unwrap().observed_ns,900);
        let reused=planner.map_with_delta(&timed,SurfaceDelta::Unchanged{basis:&source}).unwrap();
        assert!(Arc::ptr_eq(&old,&reused));
        exact(&reused,&Tomogram::from_surface(&timed).unwrap());
        values.insert([3,3,2],SurfaceObservation{observed_ns:901,hits:2});
        let (next_source,changed)=replace(&timed,&mut observations,&values);
        let next=planner.map_with_delta(&next_source,SurfaceDelta::Columns {
            basis:&timed,resolution:f64::from(Q),columns:&changed,
        }).unwrap();
        assert!(!Arc::ptr_eq(&reused,&next));
        exact(&next,&Tomogram::from_surface(&next_source).unwrap());
        retained(&planner,&next_source,&next);
    }

    #[test]
    fn cache_wrong_delta_basis_rebuilds_all_current_columns() {
        let (source,mut observations,mut values)=fixture();
        let planner=PctPlanner::default();let old=planner.map(&source).unwrap();
        values.insert([3,3,2],SurfaceObservation{observed_ns:20,hits:2});
        let (skipped,changed)=replace(&source,&mut observations,&values);
        assert_eq!(changed,vec![[3,3]]);
        assert!(!old.matches_surface(&skipped));
        values.remove(&[0,0,0]);
        let (next_source,changed)=replace(&skipped,&mut observations,&values);
        assert_eq!(changed,vec![[0,0]],"delta相对被跳过基底是完整的，但不包含此前变化");
        let next=planner.map_with_delta(&next_source,SurfaceDelta::Columns {
            basis:&skipped,resolution:f64::from(Q),columns:&changed,
        }).unwrap();
        let cold=Tomogram::from_surface(&next_source).unwrap();
        assert_eq!(identities(&next),identities(&cold),"错误基底必须实际冷建");
        assert_eq!(next.occupancy[&[3,3]].len(),2,"不能丢掉不在本次delta内的先前变化");
        assert!(!next.known_columns.contains(&[0,0]));
        exact(&next,&cold);
        retained(&planner,&next_source,&next);
    }

    #[test]
    fn cache_cancelled_delta_keeps_complete_previous_map_and_fresh_retry_matches_cold() {
        let (source,mut observations,mut values)=fixture();
        values.remove(&[0,0,0]);
        values.insert([3,3,2],SurfaceObservation{observed_ns:20,hits:2});
        let (next_source,changed)=replace(&source,&mut observations,&values);
        let cold=Tomogram::from_surface(&next_source).unwrap();
        // 0 是入口；8/64 确定性覆盖已消费 delta 后的真实构图工作。
        // 不把依赖实现的计数冒称某一固定内部阶段。
        for count in [0,8,64] {
            let planner=PctPlanner::default();let old=planner.map(&source).unwrap();
            let frozen=old.as_ref().clone();
            let flag=Arc::new(AtomicBool::new(false));let flip=flag.clone();
            let token=PlanningCancellation::from_flag(flag.clone());
            let result=crate::cancellation::after_checkpoint_action(count,
                move||flip.store(true,Ordering::Release),||crate::with_planning_cancellation(&token,||
                    planner.map_with_delta(&next_source,SurfaceDelta::Columns {
                        basis:&source,resolution:f64::from(Q),columns:&changed,
                    })));
            assert!(flag.load(Ordering::Acquire),"未到达取消检查点 {count}");
            assert_eq!(result.unwrap_err(),"层析规划已取消","checkpoint={count}");
            assert!(!crate::planning_cancelled());
            retained(&planner,&source,&old);exact(&old,&frozen);
            let fresh=PlanningCancellation::from_flag(Arc::new(AtomicBool::new(false)));
            let retry=crate::with_planning_cancellation(&fresh,||planner.map_with_delta(&next_source,
                SurfaceDelta::Columns{basis:&source,resolution:f64::from(Q),columns:&changed})).unwrap();
            assert!(!Arc::ptr_eq(&old,&retry));
            exact(&retry,&cold);exact(&old,&frozen);
            retained(&planner,&next_source,&retry);
            assert!(Arc::ptr_eq(&retry,&planner.map(&next_source).unwrap()));
        }
    }
}

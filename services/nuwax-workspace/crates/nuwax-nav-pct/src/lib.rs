//! PCT式点云层析：地面/顶面与通行代价在建图时生成，搜索不含机体偏航。
//! 采用CPU稀疏列实现，未移植原论文的CUDA及GPMP优化器。
#[cfg(feature = "mls-adapter")]
mod body_route;
mod boundary;
#[cfg(feature = "mls-adapter")]
mod surface_adapter;
#[cfg(feature = "mls-adapter")]
pub use surface_adapter::{PctPlanner, same_source_geometry};
mod search;
mod tomogram;
pub use search::{Route, SearchStats};
pub use tomogram::{Cell, Tomogram};

/// 仅当前目标使用的局部接续软代价，不修改原地图或宣称该处永久不可通行。
#[derive(Clone, Debug)]
pub struct RoutePenalty {
    pub center: [f64; 3],
    pub radius: f64,
    pub weight: f64,
}
pub(crate) fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}
pub use boundary::{inside_boundary, segment_inside_boundary};

#[cfg(all(test, feature = "mls-adapter"))]
mod tests {
    use super::*;
    use std::sync::Arc;
    use nuwax_nav_mls_planner::SurfaceSnapshot;
    use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceObservation, SurfaceObservations};
    use std::collections::BTreeMap;
    #[test]
    fn ground_evidence_changes_invalidate_the_same_occupancy_cache() {
        let source=super::sampling_tests::sampled_room(true,false);
        let planner=PctPlanner::default();let measured=planner.map(&source).unwrap();
        let occupied=source.clone().with_ground_evidence(nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume);
        assert!(!same_source_geometry(&source,&occupied));
        assert!(!source.same_body_query_view(&occupied));
        assert!(!Arc::ptr_eq(&measured,&planner.map(&occupied).unwrap()));
        let observed=occupied.with_observations(occupied.observations().cloned());
        assert_eq!(observed.ground_evidence(),nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume);
    }
    #[test]
    fn fine_obstacle_moves_inside_one_coarse_cell_but_time_updates_reuse_tomogram() {
        let fine = 0.05_f32;
        let values = |x, stamp| {
            let mut cells: BTreeMap<_, _> = (0..8)
                .flat_map(|x| {
                    (0..8).map(move |y| {
                        (
                            [x, y, 0],
                            SurfaceObservation {
                                observed_ns: stamp,
                                hits: 2,
                            },
                        )
                    })
                })
                .collect();
            cells.insert(
                [x, 0, 5],
                SurfaceObservation {
                    observed_ns: stamp,
                    hits: 2,
                },
            );
            cells
        };
        let first_values = values(0, 1);
        let points: Vec<_> = first_values
            .keys()
            .map(|p| {
                (
                    (p[0] as f32 + 0.5) * fine,
                    (p[1] as f32 + 0.5) * fine,
                    (p[2] as f32 + 0.5) * fine,
                )
            })
            .collect();
        let config = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.3,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.0,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 2.0,
            max_slope_deg: 20.0,
            step_threshold_m: 0.1,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        let source = SurfaceSnapshot::from_points(&points, config, fine).unwrap();
        let mut evidence = SurfaceObservations::new(fine, 32).unwrap();
        evidence.replace_block([0, 0, 0], first_values).unwrap();
        evidence.mark_complete_geometry_source();
        let first = source.with_observations(Some(Arc::new(evidence.clone())));
        let planner = PctPlanner::default();
        let a = planner.map(&first).unwrap();
        assert!(a.cell_at([0.025, 0.025, 0.05]).is_none());
        assert!(a.cell_at([0.075, 0.025, 0.05]).is_some());
        evidence.replace_block([0, 0, 0], values(0, 2)).unwrap();
        evidence.mark_complete_geometry_source();
        let timed = source.with_observations(Some(Arc::new(evidence.clone())));
        assert!(
            Arc::ptr_eq(&a, &planner.map(&timed).unwrap()),
            "纯时间更新应保留层析图"
        );
        evidence.replace_block([0, 0, 0], values(1, 3)).unwrap();
        evidence.mark_complete_geometry_source();
        let moved = source.with_observations(Some(Arc::new(evidence)));
        assert!(first.same_geometry(&moved), "该反例保持粗格几何相同");
        let b = planner.map(&moved).unwrap();
        assert!(!Arc::ptr_eq(&a, &b));
        assert!(b.cell_at([0.025, 0.025, 0.05]).is_some());
        assert!(b.cell_at([0.075, 0.025, 0.05]).is_none());
    }
    #[test]
    fn local_feedback_changes_preference_without_erasing_a_passage() {
        let map = Tomogram::from_columns(
            0.1,
            (0..50).flat_map(|x| (-10..11).map(move |y| ([x, y], vec![[-0.1, 0.0]]))),
            0.3,
            0.1,
            0.1,
            0.1,
            20.0,
        )
        .unwrap();
        let start = [0.25, 0.05, 0.0];
        let goal = [4.75, 0.05, 0.0];
        let penalties = [RoutePenalty {
            center: [2.5, 0.05, 0.0],
            radius: 0.6,
            weight: 32.0,
        }];
        let direct = map.plan(start, goal, 0.01, &[], None).unwrap();
        let repaired = map
            .plan_bounded(start, goal, 0.01, &[], None, None, &penalties)
            .unwrap();
        assert!(direct.points.iter().all(|p| p[1].abs() < 0.1));
        assert!(repaired.points.iter().any(|p| p[1].abs() > 0.4));
        assert!(map.plan(start, goal, 0.01, &[], None).is_ok());
    }
}

#[cfg(all(test, feature = "mls-adapter"))]
mod sampling_tests {
    use super::*;
    use std::sync::Arc;
    use nuwax_nav_mls_planner::SurfaceSnapshot;
    use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceObservation, SurfaceObservations};
    use std::collections::BTreeMap;

    pub(super) fn sampled_room(centered: bool, missing_strip: bool) -> SurfaceSnapshot {
        let size = 0.05_f32;
        // 对齐原np.arange的实际双精度步长，再按原发布方式转float32。
        let delta = (-2.5_f64 + 0.05) - (-2.5);
        let axis: Vec<f32> = if centered {
            (-50..50)
                .map(|i| ((f64::from(i) + 0.5) * f64::from(size)) as f32)
                .collect()
        } else {
            (0..101)
                .map(|i| (-2.5 + f64::from(i) * delta) as f32)
                .collect()
        };
        let mut points = Vec::new();
        let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
        for &x in &axis {
            for &y in &axis {
                for z in [0.0_f32, 1.0] {
                    if f64::from(x).powi(2) + f64::from(y).powi(2) + (f64::from(z) - 0.35).powi(2)
                        > 1.6_f64.powi(2)
                    {
                        continue;
                    }
                    let key =
                        [x, y, z].map(|v| (f64::from(v) * (1.0 / f64::from(size))).floor() as i32);
                    if missing_strip && key[0] == 2 {
                        continue;
                    }
                    points.push((x, y, z));
                    blocks
                        .entry(key.map(|v| v.div_euclid(32)))
                        .or_default()
                        .insert(
                            key,
                            SurfaceObservation {
                                observed_ns: 0,
                                hits: 0,
                            },
                        );
                }
            }
        }
        let config = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.3,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.0,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 2.0,
            max_slope_deg: 18.0,
            step_threshold_m: 0.15,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 2.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        let mut observations = SurfaceObservations::new(size, 32).unwrap();
        for (key, values) in blocks {
            observations.replace_block(key, values).unwrap();
        }
        observations.mark_complete_geometry_source();
        SurfaceSnapshot::from_points(&points, config, size)
            .unwrap()
            .with_observations(Some(Arc::new(observations)))
    }

    #[test]
    fn boundary_sampled_missing_columns_stay_disconnected_while_complete_centres_connect() {
        let start = [0.0, 0.0, 0.05];
        let goal = [0.8, 0.8, 0.05];
        let boundary = sampled_room(false, false);
        let map = Tomogram::from_surface(&boundary).unwrap();
        assert_eq!(
            map.cells.len(),
            3102,
            "原生产快照的完整fine占据数量必须精确复现"
        );
        assert!(
            map.cell_at(start).is_none(),
            "原点所在的fine列确实没有观测，不能由完整提交标记填出地面"
        );
        assert_eq!(
            map.plan(start, goal, 0.05, &[], None).unwrap_err(),
            "层析地面之间没有可连接路线（展开4格）"
        );
        let complete = Tomogram::from_surface(&sampled_room(true, false)).unwrap();
        let route = complete.plan(start, goal, 0.05, &[], None).unwrap();
        assert!(route
            .points
            .windows(2)
            .all(|p| complete.segment_clear(p[0], p[1], &[])));
        let gap = Tomogram::from_surface(&sampled_room(true, true)).unwrap();
        assert!(
            gap.plan([-0.2, 0.0, 0.05], goal, 0.05, &[], None).is_err(),
            "真实缺列不能被全局路线补桥"
        );
        assert!(
            complete
                .plan(start, [0.8, 0.8, 1.0], 0.05, &[], None)
                .is_err(),
            "地面和顶面不是可跨越的相邻楼层"
        );
    }
}

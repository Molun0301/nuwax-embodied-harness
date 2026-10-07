//! 禁区源代价优化的同快照精确对照。
//! 仅冻结旧表面包装函数；未修改的底层构造器继续共用。
use super::*;
use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceObservation, SurfaceObservations};
use std::{collections::BTreeMap, hint::black_box, sync::Arc, time::{Duration, Instant}};

trait FrozenSurfaceWindow: Sized {
    fn from_surface_window(
        source: &SurfaceSnapshot,
        window: Option<([f64; 2], f64)>,
    ) -> Result<Self, String>;
}

// 逐字节复制优化前的包装函数，原 tomogram.rs SHA-256：
// 448aa791f764a812dac4c8c88804cbc44c92a5ed97acfd7a2183c063c64bc899.
// 独立 trait 保留原方法名和 Self 引用。
impl FrozenSurfaceWindow for Tomogram {
    #[cfg(feature = "mls-adapter")]
    fn from_surface_window(
        source: &SurfaceSnapshot,
        window: Option<([f64; 2], f64)>,
    ) -> Result<Self, String> {
        checkpoint()?;
        let config = source.config();
        let body = source.body_envelope();
        let required_height = body.map_or(f64::from(config.robot_height), |m| {
            m.body_to_support_z_m + m.z_bounds_m[1]
        });
        let half_width = body.map_or(f64::from(config.wall_clearance_m), |m| {
            let (lo, hi) = m
                .footprint_xy_m
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                    (lo.min(p[1]), hi.max(p[1]))
                });
            (hi - lo) * 0.5
        });
        // The source supplies one iterator; its creation and individual next() calls
        // remain indivisible, so check on both sides of that MLS boundary.
        checkpoint()?;
        let (resolution, columns) = source.navigation_source_columns(window);
        checkpoint()?;
        let mut blocked = AHashSet::new();
        let mut result = Self::from_columns_with_evidence(
            resolution,
            f64::from(source.source_voxel_size_m()),
            columns.take_while(|_| !crate::planning_cancelled()).filter(|(column, _)| {
                let allowed = source.navigation_column_allowed(*column, resolution);
                if !allowed { blocked.insert(*column); }
                allowed
            }),
            required_height,
            half_width,
            body.map_or(f64::from(config.step_threshold_m), |m| m.max_step_up_m),
            body.map_or(f64::from(config.step_threshold_m), |m| m.max_step_down_m),
            f64::from(config.max_slope_deg),
            body.map_or(half_width, |body| body.radius()),
            source.ground_evidence()==nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume,
        )?;
        if !blocked.is_empty() {
            for column in blocked {
                checkpoint()?;
                result.known_columns.insert(column);
            }
            result.compute_costs()?;
        }
        checkpoint()?;
        Ok(result)
    }
}

const FINE: f32 = 0.05;
const LOCAL: ([f64; 2], f64) = ([0.9, 0.65], 0.8);

fn fixture(complete_fine: bool) -> SurfaceSnapshot {
    let size = f64::from(FINE);
    let mut points = Vec::new();
    let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
    for x in -2_i32..38 {
        for y in -2..28 {
            // 真正缺测的两精细列条带；粗列回退也不能补出它。
            if (8..10).contains(&x) && (20..24).contains(&y) { continue; }
            for z in [0, 20] {
                let p = [x, y, z].map(|v| ((f64::from(v) + 0.5) * size) as f32);
                points.push((p[0], p[1], p[2]));
                blocks.entry([x, y, z].map(|v| v.div_euclid(32))).or_default()
                    .insert([x, y, z], SurfaceObservation { observed_ns: 1, hits: 2 });
            }
        }
    }
    let config = Config {
        world_frame: "map".into(), voxel_size: 0.1, robot_height: 0.3,
        surface_closing_radius: 0.0, node_spacing_m: 0.3, wall_clearance_m: 0.0,
        wall_buffer_m: 0.1, wall_buffer_weight: 2.0, max_slope_deg: 20.0,
        step_threshold_m: 0.1, min_step_tread_depth_m: 0.2, step_penalty_weight: 1.0,
        goal_tolerance: 0.1, viz_publish_hz: 0.0,
    };
    let mut source = SurfaceSnapshot::from_points(&points, config, FINE).unwrap();
    if complete_fine {
        let mut observations = SurfaceObservations::new(FINE, 32).unwrap();
        for (block, values) in blocks { observations.replace_block(block, values).unwrap(); }
        observations.mark_complete_geometry_source();
        source = source.with_observations(Some(Arc::new(observations)));
    }
    // 禁区范围小于测量房间；虚拟线的末端保留绕行通道。
    let mut blocked = vec![false; 36 * 26];
    for x in 18..20 { for y in 0..16 { blocked[y * 36 + x] = true; } }
    source.with_virtual_obstacles([0.0, 0.0], size, 36, 26, &blocked).unwrap()
}

fn current(source: &SurfaceSnapshot, window: Option<([f64; 2], f64)>) -> Result<Tomogram, String> {
    match window {
        None => Tomogram::from_surface(source),
        Some((center, radius)) => Tomogram::local_from_surface(source, center, radius),
    }
}

fn assert_exact(reference: &Tomogram, actual: &Tomogram) {
    let parameters = |m: &Tomogram| [m.resolution, m.body_height, m.half_width,
        m.step_up, m.step_down, m.slope, m.clearance_radius].map(f64::to_bits);
    assert_eq!(parameters(reference), parameters(actual));
    assert_eq!(reference.cells.len(), actual.cells.len());
    for (id, (a, b)) in reference.cells.iter().zip(&actual.cells).enumerate() {
        let bits = |c: &Cell| [c.ground, c.ground_interval[0], c.ground_interval[1],
            c.ceiling, c.clearance, c.support_clearance, c.cost].map(f64::to_bits);
        assert_eq!((a.column, a.layer, bits(a), &a.neighbors),
            (b.column, b.layer, bits(b), &b.neighbors), "stored cell/order {id}");
    }
    // 哈希容器无迭代顺序契约；其中存储的索引向量必须保持原顺序。
    assert_eq!(reference.columns, actual.columns);
    assert_eq!(reference.known_columns, actual.known_columns);
    for z in [0.05, 1.05] {
        let start = [0.275, 0.425, z];
        let goal = [1.525, 0.425, z];
        let old = reference.plan(start, goal, 0.01, &[], None).unwrap();
        let new = actual.plan(start, goal, 0.01, &[], None).unwrap();
        assert_eq!(old.points.iter().map(|p| p.map(f64::to_bits)).collect::<Vec<_>>(),
            new.points.iter().map(|p| p.map(f64::to_bits)).collect::<Vec<_>>());
        assert_eq!((old.stats.expanded, old.stats.cells), (new.stats.expanded, new.stats.cells));
        assert!(!actual.segment_clear(start, goal, &[]));
        assert!(new.points.iter().any(|p| p[1] > 0.8), "route must go around the mask line");
        assert!(new.points.windows(2).all(|p| actual.segment_clear(p[0], p[1], &[])));
    }
    let cross_floor = |m: &Tomogram| m.plan([0.275, 0.425, 0.05],
        [1.525, 0.425, 1.05], 0.01, &[], None).unwrap_err();
    assert_eq!(cross_floor(reference), cross_floor(actual));
}

fn assert_fixture(source: &SurfaceSnapshot, map: &Tomogram, fine: bool, local: bool) {
    let (resolution, columns) = source.navigation_source_columns(local.then_some(LOCAL));
    let columns: AHashSet<_> = columns.map(|(column, _)| column).collect();
    let expected_resolution = f64::from(if fine { FINE } else { 0.1_f32 });
    assert_eq!(resolution.to_bits(), expected_resolution.to_bits());
    assert_eq!(map.resolution.to_bits(), expected_resolution.to_bits());
    assert_eq!(columns.len(), match (fine, local) {
        (true, false) => 1192, (true, true) => 952,
        (false, false) => 298, (false, true) => 238,
    });
    let divisor = if fine { 1 } else { 2 };
    let blocked = [18, 8].map(|v| v / divisor);
    let outside = [4, 27].map(|v| v / divisor);
    let missing = [8, 20].map(|v| v / divisor);
    let allowed = [12, 8].map(|v| v / divisor);
    for column in [blocked, outside] {
        assert!(columns.contains(&column));
        assert!(!source.navigation_column_allowed(column, resolution));
        assert!(map.known_columns.contains(&column));
        assert!(!map.columns.contains_key(&column));
    }
    assert!(!columns.contains(&missing));
    assert!(source.navigation_column_allowed(missing, resolution));
    assert!(!map.known_columns.contains(&missing));
    assert!(!map.columns.contains_key(&missing));
    assert!(source.navigation_column_allowed(allowed, resolution));
    let floors = map.columns.get(&allowed).unwrap();
    assert_eq!(floors.len(), 2);
    assert!(map.cells[floors[0]].ground < 0.1 && map.cells[floors[1]].ground > 1.0);
}

#[test]
fn masked_surface_matches_frozen_wrapper_exactly() {
    for fine in [false, true] {
        let source = fixture(fine);
        let mut global_cells = 0;
        for window in [None, Some(LOCAL)] {
            let reference = <Tomogram as FrozenSurfaceWindow>::from_surface_window(&source, window).unwrap();
            let actual = current(&source, window).unwrap();
            assert_fixture(&source, &actual, fine, window.is_some());
            assert_exact(&reference, &actual);
            if window.is_none() { global_cells = actual.cells.len(); }
            else { assert!(actual.cells.len() < global_cells, "local path must really crop cells"); }
        }
        let blocked = source.with_virtual_obstacles([0.0, 0.0], f64::from(FINE),
            36, 26, &vec![true; 36 * 26]).unwrap();
        for window in [None, Some(LOCAL)] {
            let old = <Tomogram as FrozenSurfaceWindow>::from_surface_window(&blocked, window).unwrap_err();
            let new = current(&blocked, window).unwrap_err();
            assert_eq!(old, "层析地图没有可用地面");
            assert_eq!(old, new);
        }
    }
}

#[test]
fn masked_surface_cancellation_uses_real_epoch() {
    use std::{cell::Cell as Counter, sync::atomic::{AtomicU64, Ordering}};
    let source = fixture(true);
    let epoch = Arc::new(AtomicU64::new(1));
    let token = crate::PlanningCancellation::new(epoch.clone(), 0);
    let global = crate::with_planning_cancellation(&token, || Tomogram::from_surface(&source)).unwrap_err();
    let local = Tomogram::local_from_surface_cancellable(&source, LOCAL.0, LOCAL.1, &token).unwrap_err();
    assert_eq!(global, "层析规划已取消");
    assert_eq!(global, local);
    assert!(!crate::planning_cancelled());

    // 范围：确定性验证候选空高度带转换与真实构造器、取消令牌；
    // 不声称通过时序竞争覆盖公开入口中途取消。
    epoch.store(0, Ordering::Release);
    let visited = Counter::new(0);
    let resolution = f64::from(FINE);
    let columns = (0..10_000).map(|_| ([18, 8], vec![[0.0, resolution]]))
        .take_while(|_| !crate::planning_cancelled())
        .map(|(column, mut bands)| {
            assert!(!source.navigation_column_allowed(column, resolution));
            bands.clear();
            (column, bands)
        }).inspect(|_| {
            visited.set(visited.get() + 1);
            if visited.get() == 8 { epoch.store(1, Ordering::Release); }
        });
    let error = crate::with_planning_cancellation(&token, ||
        Tomogram::from_columns_with_evidence(resolution, resolution, columns,
            0.3, 0.0, 0.1, 0.1, 20.0, 0.0, false)).unwrap_err();
    assert_eq!(error, "层析规划已取消");
    assert_eq!(visited.get(), 8, "cancellation must stop the blocked input early");
    assert!(!crate::planning_cancelled());
}

#[test]
#[ignore = "同进程冻结版/当前版构造基线；以 --ignored --nocapture 单独运行"]
fn masked_surface_alternating_baseline() {
    const WARMUPS: usize = 4;
    const REPS: usize = 32;
    for fine in [false, true] {
        // 源准备与精确断言均位于计时区域外。
        let source = fixture(fine);
        for window in [None, Some(LOCAL)] {
            let reference = <Tomogram as FrozenSurfaceWindow>::from_surface_window(&source, window).unwrap();
            let actual = current(&source, window).unwrap();
            assert_fixture(&source, &actual, fine, window.is_some());
            assert_exact(&reference, &actual);
            let mut elapsed = [Duration::ZERO; 2];
            for iteration in 0..WARMUPS + REPS {
                // 每对轮换先运行的版本，降低先后顺序偏差。
                for version in [iteration % 2, 1 - iteration % 2] {
                    let began = Instant::now();
                    let built = if version == 0 {
                        <Tomogram as FrozenSurfaceWindow>::from_surface_window(black_box(&source), window)
                    } else { current(black_box(&source), window) }.unwrap();
                    black_box(&built);
                    if iteration >= WARMUPS { elapsed[version] += began.elapsed(); }
                    // 返回地图的析构不计时；消耗输入时的清理仍在计时内。
                    drop(built);
                }
            }
            assert_exact(&reference, &current(&source, window).unwrap());
            println!("masked_surface source={} scope={} resolution_bits={} cells={} known_columns={} warmups={} reps={} frozen_total_ns={} current_total_ns={} frozen_ns_per_iter={:.3} current_ns_per_iter={:.3}",
                if fine { "complete_fine" } else { "coarse_fallback" },
                if window.is_some() { "local" } else { "global" },
                actual.resolution.to_bits(), actual.cells.len(), actual.known_columns.len(), WARMUPS, REPS,
                elapsed[0].as_nanos(), elapsed[1].as_nanos(),
                elapsed[0].as_secs_f64() * 1e9 / REPS as f64,
                elapsed[1].as_secs_f64() * 1e9 / REPS as f64);
        }
    }
    println!("masked_surface source_preparation=excluded exact_checks=excluded returned_map_destruction=excluded consumed_input_cleanup=included route_elapsed_ms=excluded_from_equality");
}

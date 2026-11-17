//! 三维选点规则的性质：候选按路程由近到远发现、簇代表、搜到8个簇即停、规模无关、重试优先。
use super::tests::{map, volume_model};
use super::*;
use nuwax_nav_runtime::native_map::PctTerrainSource;
use nuwax_nav_types_messages::geometry::{Point2, Quaternion, Timestamp};
use nuwax_nav_types_messages::NavFrameId;
use std::collections::{BTreeMap, BTreeSet};

const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

/// 0.1米格平地（地面高0.05米），`size_m`见方，去掉`missing`选中的格。
fn floor(size_m: f64, missing: impl Fn(i32, i32) -> bool) -> Arc<SurfaceSnapshot> {
    let cells = (size_m / 0.1).round() as i32;
    let points: Vec<_> = (0..cells)
        .flat_map(|x| (0..cells).map(move |y| (x, y)))
        .filter(|&(x, y)| !missing(x, y))
        .map(|(x, y)| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05))
        .collect();
    Arc::new(SurfaceSnapshot::from_points(&points, map().config().clone(), 0.1).unwrap())
}

/// 以格`center`为中心的螺线上`count`个4格（0.4米）见方缺测，半径从2.5米每个加0.5米，各自形成前沿。
/// 用整数格偏移，嵌入不同大小的地图时形状完全相同。
fn spiral(center: i32, count: usize) -> Vec<(i32, i32)> {
    (0..count)
        .map(|i| {
            let (radius, angle) = (25.0 + 5.0 * i as f64, 0.9 * i as f64);
            (center + (radius * angle.cos()).round() as i32, center + (radius * angle.sin()).round() as i32)
        })
        .collect()
}

fn in_hole(holes: &[(i32, i32)], x: i32, y: i32) -> bool {
    holes.iter().any(|&(hx, hy)| (x - hx).abs() < 2 && (y - hy).abs() < 2)
}

/// 地图四周向内0.3米的探索边界，地图边缘不形成前沿。
fn inset(size_m: f64) -> ExplorationBoundary {
    ExplorationBoundary::from_vertices(vec![
        Point2::new(0.3, 0.3),
        Point2::new(size_m - 0.3, 0.3),
        Point2::new(size_m - 0.3, size_m - 0.3),
        Point2::new(0.3, size_m - 0.3),
    ])
    .unwrap()
}

fn vertices(boundary: &ExplorationBoundary) -> Vec<[f64; 2]> {
    boundary.vertices().iter().map(|p| [p.x, p.y]).collect()
}

fn adapter(receiver: &SurfaceInputReceiver, boundary: Option<ExplorationBoundary>, model: bool) -> SurfaceGoalSourceAdapter {
    let terrain = PctTerrainSource::from_runtime().unwrap();
    let mut source = SurfaceGoalSourceAdapter::new("surface", 0.3, receiver.clone(), Arc::new(PctPlanner::new(terrain))).unwrap();
    source.configure_constraints(boundary, 1, 0.3).unwrap();
    if model {
        source.set_observation_model(volume_model()).unwrap();
    }
    source.begin().unwrap();
    source
}

fn pose(x: f64, y: f64, sequence: i64) -> ExplorationStateSnapshot<'static> {
    let now = Timestamp::from_unix(sequence, 0);
    ExplorationStateSnapshot::new(
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: now,
            pose: Pose3 { position: Point3::new(x, y, 0.35), orientation: Quaternion::IDENTITY },
        },
        now,
    )
}

/// 一次发现与同一起点展开到可达范围尽头的对照：对照给出每个可达候选（前沿格、簇）的加权路径代价。
struct Run {
    discovery: Discovery,
    expanded: usize,
    truth: Vec<([i32; 3], u32, f64)>,
    truth_expanded: usize,
    spans: BTreeMap<u32, f64>,
}

/// 只做一次发现，返回发现结果、展开节点数与基础每米代价。
fn discover_only(source: &SurfaceGoalSourceAdapter, surface: &SurfaceSnapshot, boundary: &[[f64; 2]], start: [f64; 3])
    -> (Discovery, usize, f64) {
    surface.prepare_frontier_geometry(boundary).unwrap();
    let mut routes = source.planner.route_costs(surface, 0.3, boundary).unwrap();
    let mut search = routes.search().unwrap();
    let geometry = frontier_geometry(surface, boundary).unwrap();
    let mut candidates = geometry.candidates;
    snap_support_heights(&search, &mut candidates);
    let robot_support = surface.support_position_from_body(start, IDENTITY, 0.3).unwrap();
    let vertical_radius = f64::from(surface.config().step_threshold_m) + f64::from(surface.config().voxel_size);
    let discovery = source
        .discover(surface, &mut search, candidates, &geometry.spans, &geometry.targets, (start, IDENTITY),
            robot_support, vertical_radius, true, Default::default())
        .unwrap();
    (discovery, search.stats().source_expanded, search.cost_per_meter())
}

fn run(source: &SurfaceGoalSourceAdapter, surface: &SurfaceSnapshot, boundary: &[[f64; 2]], start: [f64; 3]) -> Run {
    surface.prepare_frontier_geometry(boundary).unwrap();
    let mut routes = source.planner.route_costs(surface, 0.3, boundary).unwrap();
    let mut search = routes.search().unwrap();
    let geometry = frontier_geometry(surface, boundary).unwrap();
    let mut candidates = geometry.candidates;
    snap_support_heights(&search, &mut candidates);
    let robot_support = surface.support_position_from_body(start, IDENTITY, 0.3).unwrap();
    let vertical_radius = f64::from(surface.config().step_threshold_m) + f64::from(surface.config().voxel_size);
    let discovery = source
        .discover(surface, &mut search, candidates.clone(), &geometry.spans, &geometry.targets, (start, IDENTITY),
            robot_support, vertical_radius, true, Default::default())
        .unwrap();
    let expanded = search.stats().source_expanded;
    let minimum = f64::from(surface.config().goal_tolerance.max(surface.config().voxel_size));
    let kept: Vec<_> = candidates.into_iter().filter(|c| distance(robot_support, c.support_xyz) > minimum).collect();
    let goals: Vec<_> = kept.iter().map(|c| [c.support_xyz[0], c.support_xyz[1], c.support_xyz[2] + 0.3]).collect();
    search.begin_source(start, &goals).unwrap();
    let mut truth = Vec::new();
    while let Some(settled) = search.next_settled(f64::INFINITY).unwrap() {
        let candidate = &kept[settled.index];
        truth.push((candidate.frontier_cell, candidate.cluster, settled.weighted));
    }
    Run { discovery, expanded, truth, truth_expanded: search.stats().source_expanded - expanded, spans: geometry.spans }
}

fn cheapest_by_cluster(truth: &[([i32; 3], u32, f64)]) -> BTreeMap<u32, f64> {
    let mut best = BTreeMap::new();
    for &(_, cluster, weighted) in truth {
        let entry = best.entry(cluster).or_insert(f64::INFINITY);
        *entry = f64::min(*entry, weighted);
    }
    best
}

fn representatives(run: &Run) -> BTreeMap<u32, f64> {
    let weighted: BTreeMap<_, _> = run.truth.iter().map(|&(cell, _, w)| (cell, w)).collect();
    run.discovery
        .admissible
        .iter()
        .map(|(candidate, _, _)| (candidate.cluster, weighted[&candidate.frontier_cell]))
        .collect()
}

#[test]
fn representative_is_cheapest_reachable_candidate_and_selected_clusters_are_the_eight_cheapest() {
    let center = 12.05;
    let holes = spiral(120, 14);
    let surface = floor(24.0, |x, y| in_hole(&holes, x, y));
    let boundary = inset(24.0);
    let receiver = SurfaceInputReceiver::new();
    let source = adapter(&receiver, Some(boundary.clone()), false);
    let run = run(&source, &surface, &vertices(&boundary), [center, center, 0.35]);
    let best = cheapest_by_cluster(&run.truth);
    assert!(best.len() > 8, "夹具应有8个以上可达簇：{}", best.len());
    let chosen = representatives(&run);
    assert_eq!(chosen.len(), 8, "每个簇一个代表，凑满8个簇");
    // 1. 每个簇的代表是该簇全部可达候选里路径代价最小的。
    for (cluster, weighted) in &chosen {
        assert_eq!(*weighted, best[cluster], "簇{cluster}");
    }
    // 2. 被选入的8个簇正是代表路径代价最小的8个。
    let mut ranked: Vec<_> = best.iter().map(|(cluster, w)| (*w, *cluster)).collect();
    ranked.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(ranked[7].0 < ranked[8].0, "夹具第8与第9个簇代价不应相同");
    let cheapest: BTreeSet<_> = ranked.iter().take(8).map(|(_, cluster)| *cluster).collect();
    assert_eq!(chosen.keys().copied().collect::<BTreeSet<_>>(), cheapest);
    assert!(run.expanded < run.truth_expanded, "凑满8个簇即停，未展开到可达范围尽头：{} 对 {}",
        run.expanded, run.truth_expanded);
}

#[test]
fn far_unreachable_candidates_neither_change_the_result_nor_expand_the_whole_map() {
    let center = 12.05;
    let holes = spiral(120, 14);
    // 远处角落一块被一圈缺测隔开的平台：它的边缘是前沿候选，但不可达。
    let island = |x: i32, y: i32| {
        let outside = |a: i32| (205..=235).contains(&a);
        let gap = |a: i32| (200..=240).contains(&a);
        gap(x) && gap(y) && !(outside(x) && outside(y))
    };
    let boundary = inset(24.0);
    let polygon = vertices(&boundary);
    let receiver = SurfaceInputReceiver::new();
    let source = adapter(&receiver, Some(boundary.clone()), false);
    let start = [center, center, 0.35];
    let near = run(&source, &floor(24.0, |x, y| in_hole(&holes, x, y)), &polygon, start);
    let with_island = run(&source, &floor(24.0, |x, y| in_hole(&holes, x, y) || island(x, y)), &polygon, start);
    let summary = |run: &Run| run.discovery.admissible.iter()
        .map(|(c, body, _)| (c.frontier_cell, c.travel_cost.to_bits(), body.0.map(f64::to_bits))).collect::<Vec<_>>();
    assert_eq!(summary(&near), summary(&with_island), "远处不可达候选不影响结果");
    assert_eq!(near.expanded, with_island.expanded);
    let total = 240 * 240;
    assert!(with_island.expanded * 4 < total, "展开{}个节点，全图{total}", with_island.expanded);
    // 可达范围只有一个封闭房间（可达簇不足8个）：搜索展开完房间即停，不碰隔开的远处平台。
    let room = |x: i32, y: i32| {
        let d = (x - 120).abs().max((y - 120).abs());
        (31..=33).contains(&d) || (d < 31 && in_hole(&[(100, 120), (140, 125)], x, y))
    };
    let closed = run(&source, &floor(24.0, |x, y| room(x, y) || island(x, y)), &polygon, start);
    assert!(!closed.discovery.admissible.is_empty());
    assert_eq!(closed.expanded, closed.truth_expanded, "可达簇不足8个时展开完可达范围");
    assert!(closed.expanded <= 61 * 61, "只展开房间内：{}", closed.expanded);
}

#[test]
fn with_view_model_selected_cluster_candidates_settle_up_to_the_representative_bound() {
    let mut sampled_total = 0;
    let mut within_total = 0;
    let mut beyond_total = 0;
    for (name, surface, boundary, center) in [
        ("螺线缺测", floor(24.0, |x, y| in_hole(&spiral(120, 14), x, y)), Some(inset(24.0)), 12.05),
        ("多房间", rooms(50.0, 10.0), None, 25.25),
    ] {
        let polygon = boundary.as_ref().map_or_else(Vec::new, vertices);
        let receiver = SurfaceInputReceiver::new();
        let source = adapter(&receiver, boundary, true);
        let run = run(&source, &surface, &polygon, [center, center, 0.35]);
        let weighted: BTreeMap<_, _> = run.truth.iter().map(|&(cell, _, w)| (cell, w)).collect();
        // 每个簇第一个收下的候选是该簇的代表；上限 = 代表最大代价 + 8个簇里最大的平面跨度 × 每米代价（0.1米格，每米10）。
        let mut representatives = BTreeMap::new();
        for (candidate, _, _) in &run.discovery.admissible {
            representatives.entry(candidate.cluster).or_insert(weighted[&candidate.frontier_cell]);
        }
        assert_eq!(representatives.len(), 8);
        let span = representatives.keys().map(|cluster| run.spans[cluster]).fold(0.0, f64::max);
        assert!(span > 0.0);
        let bound = representatives.values().copied().fold(f64::NEG_INFINITY, f64::max) + span * 10.0;
        let width = surface.frontier_body_width();
        let square = |p: [f64; 3]| [(p[0] / width).floor() as i64, (p[1] / width).floor() as i64];
        let squares: BTreeMap<_, _> = frontier_geometry(&surface, &polygon).unwrap().candidates.into_iter()
            .map(|c| (c.frontier_cell, (c.cluster, square(c.support_xyz)))).collect();
        let filled: BTreeSet<_> = run.discovery.admissible.iter().map(|(c, _, _)| squares[&c.frontier_cell]).collect();
        assert_eq!(filled.len(), run.discovery.admissible.len(), "每簇每方格至多收下一个候选");
        let mut reachable_squares = BTreeMap::<_, Vec<_>>::new();
        let (mut within, mut beyond, mut skipped) = (0, 0, 0);
        for &(cell, cluster, w) in &run.truth {
            if !representatives.contains_key(&cluster) { continue; }
            reachable_squares.entry(squares[&cell]).or_default().push((cell, w));
            if w <= bound {
                within += 1;
                if !run.discovery.examined.contains(&cell) { skipped += 1; }
            } else {
                beyond += 1;
                assert!(!run.discovery.examined.contains(&cell), "超出上限的候选{cell:?}按不可达处理");
            }
        }
        for (square, candidates) in &reachable_squares {
            assert!(filled.contains(square) || candidates.iter().all(|(cell, w)| {
                run.discovery.examined.contains(cell) || *w > bound
            }), "方格{square:?}应已有候选，或可达候选都已定值，或剩余候选超出上限");
        }
        // 不含可达格的方格自然完成；有可达格的方格按上述三条路径结束。
        assert!(run.discovery.admissible.len() > representatives.len(), "每簇可从多个方格的候选选择代表");
        eprintln!("{name}选中簇内候选：上限内{within}，超出{beyond}，上限内未定值{skipped}，方格{}，已有候选{}",
            reachable_squares.len(), filled.len());
        sampled_total += run.discovery.admissible.len();
        within_total += within;
        beyond_total += beyond;
    }
    assert!(sampled_total < within_total, "抽稀后只收下每方格首个通过检查的候选");
    assert!(beyond_total > 0, "必须覆盖代价上限截断，超出上限的候选不得定值");
}

#[test]
fn view_model_with_unreachable_wall_top_cluster_does_not_expand_with_map_area() {
    let mut runs = Vec::new();
    for (size, cell) in [(50.0, 250), (100.0, 490)] {
        let center = (f64::from(cell) + 0.5) * 0.1;
        let holes = spiral(cell, 14);
        // 机器人东侧3米一道1米高、4米长的墙：墙顶自成一个达标簇，但不可达。
        let wall = move |x: i32, y: i32| x == cell + 30 && (cell - 20..cell + 20).contains(&y);
        let cells = (size / 0.1_f64).round() as i32;
        let points: Vec<_> = (0..cells)
            .flat_map(|x| (0..cells).map(move |y| (x, y)))
            .filter(|&(x, y)| !in_hole(&holes, x, y))
            .flat_map(|(x, y)| {
                let (px, py) = ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1);
                let height = if wall(x, y) { 10 } else { 1 };
                (0..height).map(move |z| (px, py, z as f32 * 0.1 + 0.05))
            })
            .collect();
        let surface = Arc::new(SurfaceSnapshot::from_points(&points, map().config().clone(), 0.1).unwrap());
        let boundary = inset(size);
        let polygon = vertices(&boundary);
        let receiver = SurfaceInputReceiver::new();
        let source = adapter(&receiver, Some(boundary), true);
        let (discovery, expanded, _) = discover_only(&source, &surface, &polygon, [center, center, 0.35]);
        let clusters: BTreeSet<_> = discovery.admissible.iter().map(|(c, _, _)| c.cluster).collect();
        let tops: BTreeSet<_> = surface.frontier_clusters(&polygon).unwrap().iter()
            .filter(|cluster| cluster.frontiers.iter().all(|f| f.support_xyz[2] > 0.5)).map(|cluster| cluster.id).collect();
        eprintln!("{size}米：展开{expanded}，簇{}，墙顶簇{tops:?}", clusters.len());
        assert!(!tops.is_empty(), "墙顶应自成达标簇");
        assert!(clusters.is_disjoint(&tops), "不可达的墙顶簇没有代表，不会被选入8个簇");
        assert_eq!(clusters.len(), 8);
        let relative: Vec<_> = discovery.admissible.iter()
            .map(|(c, _, _)| [c.support_xyz[0] - center, c.support_xyz[1] - center].map(|v| (v * 10.0).round() as i64)).collect();
        runs.push((expanded, relative, (size / 0.1 * size / 0.1) as usize));
    }
    assert_eq!(runs[0].1, runs[1].1, "同一场景选出相同的候选");
    assert_eq!(runs[0].0, runs[1].0, "展开节点数不随地图面积增长");
    assert!(runs[0].0 * 4 < runs[0].2, "不展开到全图：{} / {}", runs[0].0, runs[0].2);
}

#[test]
fn single_selection_effort_does_not_grow_with_map_area_when_frontiers_are_near() {
    let mut runs = Vec::new();
    // 两张图上场景的整数格位置相对中心相同（中心相差24米）。
    for (size, cell) in [(50.0, 250), (100.0, 490)] {
        let center = (f64::from(cell) + 0.5) * 0.1;
        let holes = spiral(cell, 14);
        let surface = floor(size, |x, y| in_hole(&holes, x, y));
        let boundary = inset(size);
        let polygon = vertices(&boundary);
        surface.prepare_frontier_geometry(&polygon).unwrap();
        let receiver = SurfaceInputReceiver::new();
        let mut source = adapter(&receiver, Some(boundary), true);
        // 第一拍建原生地图（与地图面积相关、只建一次），第二拍计时。
        let mut timings = Vec::new();
        let mut goals = Vec::new();
        for round in 0..2 {
            receiver.observe(Arc::clone(&surface)).unwrap();
            let started = std::time::Instant::now();
            let ExplorationPoll::Goal(goal) = source.poll_goal(&pose(center, center, round + 1)).unwrap() else {
                panic!("{size}米图应选出目标");
            };
            timings.push(started.elapsed().as_secs_f64() * 1000.0);
            source.report_goal_feedback(&goal.id, ExplorationGoalFeedback::Reached).unwrap();
            let p = goal.pose.pose.position;
            goals.push(([p.x - center, p.y - center], goal.metadata["route_expanded_nodes"].clone()));
        }
        eprintln!("{size}米：单次选点 冷{:.1}ms 热{:.1}ms，展开节点{:?}", timings[0], timings[1],
            goals.iter().map(|g| g.1.clone()).collect::<Vec<_>>());
        runs.push((timings[1], goals));
    }
    let ((small, small_goals), (large, large_goals)) = (&runs[0], &runs[1]);
    for (a, b) in small_goals.iter().zip(large_goals) {
        // 地面点是f32，绝对坐标50米处的f32间隔约3.8e-6米，同一相对目标在两张图上的差在这个量级。
        assert!((a.0[0] - b.0[0]).abs() < 1e-5 && (a.0[1] - b.0[1]).abs() < 1e-5, "同一场景选出同一相对目标：{a:?} {b:?}");
        assert_eq!(a.1, b.1, "展开节点数相同");
    }
    assert!(*large < small * 1.5 + 50.0, "耗时不随地图面积增长：50米{small:.1}ms，100米{large:.1}ms");
}

#[test]
fn failed_retry_candidate_is_still_selected_first_even_when_other_regions_are_nearer() {
    let center = 12.05;
    let holes = spiral(120, 14);
    let surface = floor(24.0, |x, y| in_hole(&holes, x, y));
    let receiver = SurfaceInputReceiver::new();
    let mut source = adapter(&receiver, Some(inset(24.0)), false);
    receiver.observe(Arc::clone(&surface)).unwrap();
    let first = goal_of(source.poll_goal(&pose(center, center, 1)).unwrap());
    source.report_goal_feedback(&first.id, ExplorationGoalFeedback::Failed("受阻".into())).unwrap();
    // 机器人移到远离该目标的一侧，附近另有更近的区域。
    let target = first.pose.pose.position;
    let (x, y) = (2.0 * center - target.x, 2.0 * center - target.y);
    let other = holes.iter()
        .map(|&(hx, hy)| ((f64::from(hx) + 0.5) * 0.1 - x).hypot((f64::from(hy) + 0.5) * 0.1 - y))
        .fold(f64::INFINITY, f64::min);
    assert!(other < (target.x - x).hypot(target.y - y), "夹具中应有比重试目标更近的前沿");
    receiver.observe(Arc::clone(&surface)).unwrap();
    let retry = goal_of(source.poll_goal(&pose(x, y, 2)).unwrap());
    assert_eq!(retry.metadata["selection_method"], "navigation_retry");
    assert!((retry.pose.pose.position.x - target.x).hypot(retry.pose.pose.position.y - target.y) < 0.15);
}

fn goal_of(poll: ExplorationPoll) -> FrontierGoal {
    let ExplorationPoll::Goal(goal) = poll else { panic!("预期目标，实际{poll:?}") };
    goal
}


/// 方形房间之间隔一格缺测地面，只有门洞相连（E2耗时对比所用50米图）。
fn rooms(size_m: f64, room_m: f64) -> Arc<SurfaceSnapshot> {
    let room = (room_m / 0.1).round() as i32;
    floor(size_m, |x, y| {
        let gap_x = x % room == 0 && x > 0 && (y % room - room / 2).abs() > 5;
        let gap_y = y % room == 0 && y > 0 && (x % room - room / 2).abs() > 5;
        gap_x || gap_y
    })
}

/// 空地；`holes`时机器人周围半径6米一圈12个0.4米见方缺测（与改前基线相同）。
fn open_floor(size_m: f64, holes: bool) -> Arc<SurfaceSnapshot> {
    let center = ((size_m / 0.1).round() as i32) / 2;
    floor(size_m, |x, y| {
        holes && (0..12).any(|i| {
            let a = i as f64 * std::f64::consts::TAU / 12.0;
            let (hx, hy) = (center as f64 + 60.0 * a.cos(), center as f64 + 60.0 * a.sin());
            (x as f64 - hx).abs() < 2.0 && (y as f64 - hy).abs() < 2.0
        })
    })
}

/// 只收集 `exploration.selection.stages` 的耗时与计数字段。
#[derive(Clone, Default)]
struct Stages(Arc<std::sync::Mutex<Vec<(String, String)>>>);
struct StageVisitor<'a>(&'a mut Vec<(String, String)>);
impl tracing::field::Visit for StageVisitor<'_> {
    fn record_f64(&mut self, field: &tracing::field::Field, value: f64) {
        self.0.push((field.name().into(), format!("{value:.1}")));
    }
    fn record_u64(&mut self, field: &tracing::field::Field, value: u64) {
        self.0.push((field.name().into(), value.to_string()));
    }
    fn record_debug(&mut self, _: &tracing::field::Field, _: &dyn std::fmt::Debug) {}
}
impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for Stages {
    fn on_event(&self, event: &tracing::Event<'_>, _: tracing_subscriber::layer::Context<'_, S>) {
        if event.metadata().fields().field("geometry_ms").is_some() {
            let mut fields = Vec::new();
            event.record(&mut StageVisitor(&mut fields));
            *self.0.lock().unwrap() = fields;
        }
    }
}

/// 耗时：与改前基线（`批次/E3a/改前基线_01dbc3d2.log`）相同的四个场景，空间视域标定，每场景三拍。
/// `cargo test --release -p nuwax-exploration --all-features selection_timing -- --ignored --nocapture`
#[test]
#[ignore = "耗时对比，手动运行"]
fn selection_timing_against_baseline_scenes() {
    use tracing_subscriber::layer::SubscriberExt;
    let stages = Stages::default();
    let _guard = tracing::subscriber::set_default(tracing_subscriber::registry().with(stages.clone()));
    for (name, surface, size, center, bounded) in [
        ("50米多房间（E2图）", rooms(50.0, 10.0), 50.0, 25.25, false),
        ("只剩远处前沿（50米空地，只有四边）", open_floor(50.0, false), 50.0, 25.05, false),
        ("规模：50米，前沿在10米内", open_floor(50.0, true), 50.0, 25.05, true),
        ("规模：100米，前沿在10米内", open_floor(100.0, true), 100.0, 50.05, true),
    ] {
        let boundary = bounded.then(|| inset(size));
        let polygon = boundary.as_ref().map_or_else(Vec::new, vertices);
        surface.prepare_frontier_geometry(&polygon).unwrap();
        let receiver = SurfaceInputReceiver::new();
        let mut source = adapter(&receiver, boundary, true);
        for round in 0..3_i64 {
            receiver.observe(Arc::clone(&surface)).unwrap();
            let started = std::time::Instant::now();
            let mut polls = 0;
            let mut outcome = ExplorationPoll::Pending;
            while polls < 16 && outcome == ExplorationPoll::Pending {
                polls += 1;
                outcome = source.poll_goal(&pose(center, center, round + 1)).unwrap();
            }
            let elapsed = started.elapsed().as_secs_f64() * 1000.0;
            let ExplorationPoll::Goal(goal) = outcome else { panic!("{name}: {outcome:?}") };
            let fields = stages.0.lock().unwrap().iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
            eprintln!("{name} 第{round}拍（{polls}次轮询）：总耗时{elapsed:.1}ms {fields}\n  簇{}/{} 目标{:?} 代价{:?}",
                goal.metadata["route_clusters"], goal.metadata["available_clusters"],
                [goal.pose.pose.position.x, goal.pose.pose.position.y], goal.distance_cost);
            source.report_goal_feedback(&goal.id, ExplorationGoalFeedback::Reached).unwrap();
        }
    }
}

/// 只收集前沿索引重建事件（`navigation.surface.frontier.rebuild`）的计数与聚类耗时。
#[derive(Clone, Default)]
struct Rebuilds(Arc<std::sync::Mutex<Vec<(String, f64)>>>);
struct RebuildVisitor<'a>(&'a mut Vec<(String, f64)>);
impl tracing::field::Visit for RebuildVisitor<'_> {
    fn record_f64(&mut self, field: &tracing::field::Field, value: f64) {
        self.0.push((field.name().into(), value));
    }
    fn record_u64(&mut self, field: &tracing::field::Field, value: u64) {
        self.0.push((field.name().into(), value as f64));
    }
    fn record_debug(&mut self, _: &tracing::field::Field, _: &dyn std::fmt::Debug) {}
}
impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for Rebuilds {
    fn on_event(&self, event: &tracing::Event<'_>, _: tracing_subscriber::layer::Context<'_, S>) {
        if event.metadata().fields().field("cluster_ms").is_some() {
            let mut fields = Vec::new();
            event.record(&mut RebuildVisitor(&mut fields));
            *self.0.lock().unwrap() = fields;
        }
    }
}

/// 聚类维护每拍耗时：同一构建器逐拍去掉少量地面列（模拟机器人附近的新观测改变前沿），每拍取一次前沿索引，
/// 记录重新评估的格数、重新聚类的格数与聚类耗时（索引重建事件里的`cluster_ms`）。第0行是首次全图建索引。
/// `cargo test --release -p nuwax-exploration --all-features cluster_maintenance_timing -- --ignored --nocapture`
#[test]
#[ignore = "耗时，手动运行"]
fn cluster_maintenance_timing() {
    use tracing_subscriber::layer::SubscriberExt;
    let rebuilds = Rebuilds::default();
    let _guard = tracing::subscriber::set_default(tracing_subscriber::registry().with(rebuilds.clone()));
    let room = |x: i32, y: i32| {
        let gap_x = x % 100 == 0 && x > 0 && (y % 100 - 50).abs() > 5;
        let gap_y = y % 100 == 0 && y > 0 && (x % 100 - 50).abs() > 5;
        gap_x || gap_y
    };
    let holes = |center: i32| move |x: i32, y: i32| (0..12).any(|i| {
        let a = i as f64 * std::f64::consts::TAU / 12.0;
        let (hx, hy) = (center as f64 + 60.0 * a.cos(), center as f64 + 60.0 * a.sin());
        (x as f64 - hx).abs() < 2.0 && (y as f64 - hy).abs() < 2.0
    });
    let near50 = holes(250);
    let near100 = holes(500);
    // 场景：名称、边长、缺测、是否用内缩边界、每拍新去掉的格。
    let scenes: Vec<(&str, f64, Box<dyn Fn(i32, i32) -> bool>, bool, Vec<(i32, i32)>)> = vec![
        ("50米多房间：每拍机器人所在房间墙边少一格（触及房间一圈前沿簇）", 50.0, Box::new(room), false,
            (0..10).map(|i| (201, 210 + 7 * i)).collect()),
        ("50米，前沿在10米内：每拍东侧缺测扩大一格", 50.0, Box::new(near50), true,
            (0..10).map(|i| (312 + i, 250)).collect()),
        ("100米，前沿在10米内：每拍东侧缺测扩大一格", 100.0, Box::new(near100), true,
            (0..10).map(|i| (562 + i, 500)).collect()),
        ("50米空地只有四边（上限参考）：每拍地图边缘少一格（触及整圈前沿簇）", 50.0, Box::new(|_, _| false), false,
            (0..10).map(|i| (0, 100 + 7 * i)).collect()),
    ];
    for (name, size, missing, bounded, ticks) in scenes {
        let cells = (size / 0.1_f64).round() as i32;
        let polygon = if bounded { vertices(&inset(size)) } else { Vec::new() };
        let mut removed = BTreeSet::new();
        let cloud = |removed: &BTreeSet<(i32, i32)>| -> Vec<(f32, f32, f32)> {
            (0..cells).flat_map(|x| (0..cells).map(move |y| (x, y)))
                .filter(|&(x, y)| !missing(x, y) && !removed.contains(&(x, y)))
                .map(|(x, y)| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05))
                .collect()
        };
        let mut builder = nuwax_nav_mls_planner::SurfaceMapBuilder::new(map().config().clone());
        eprintln!("{name}");
        for tick in 0..=ticks.len() {
            if tick > 0 {
                removed.insert(ticks[tick - 1]);
            }
            let surface = builder.build(&cloud(&removed), 0.1).unwrap().snapshot;
            let started = std::time::Instant::now();
            let stats = surface.prepare_frontier_geometry(&polygon).unwrap();
            let total = started.elapsed().as_secs_f64() * 1000.0;
            let fields: BTreeMap<_, _> = rebuilds.0.lock().unwrap().iter().cloned().collect();
            eprintln!("  第{tick}拍：前沿索引{total:.3}ms 聚类{:.3}ms 评估{}格 重新聚类{}格 前沿格{} 达标簇{} 增量{}",
                fields["cluster_ms"], stats.evaluated_cells, stats.reclustered_cells, stats.frontier_count,
                stats.candidate_clusters, stats.incremental);
        }
    }
}

/// 8米×4米地面，探索边界向上越过地面上缘：只有上缘一条长前沿线，横跨原来4米世界栅格的 x=4 米线。
#[test]
fn long_frontier_line_across_the_old_four_meter_grid_line_is_one_candidate() {
    let surface = floor(8.0, |_, y| y >= 40);
    let boundary = ExplorationBoundary::from_vertices(vec![
        Point2::new(0.3, 0.3),
        Point2::new(7.7, 0.3),
        Point2::new(7.7, 6.0),
        Point2::new(0.3, 6.0),
    ])
    .unwrap();
    let polygon = vertices(&boundary);
    let clusters = surface.frontier_clusters(&polygon).unwrap();
    assert_eq!(clusters.len(), 1, "上缘前沿线是一个簇");
    let sides: BTreeSet<_> = clusters[0].frontiers.iter().map(|f| (f.support_xyz[0] / 4.0).floor() as i64).collect();
    assert_eq!(sides.len(), 2, "簇横跨原来的4米栅格线");
    let receiver = SurfaceInputReceiver::new();
    let mut source = adapter(&receiver, Some(boundary), false);
    let (discovery, _, _) = discover_only(&source, &surface, &polygon, [4.05, 2.05, 0.35]);
    assert_eq!(discovery.admissible.len(), 1, "一个簇只有一个代表");
    receiver.observe(Arc::clone(&surface)).unwrap();
    let goal = goal_of(source.poll_goal(&pose(4.05, 2.05, 1)).unwrap());
    assert_eq!(goal.metadata["route_clusters"], "1");
    assert_eq!(goal.metadata["available_clusters"], "1");
    assert_eq!(goal.information_gain, Some(clusters[0].gain as f32), "收益取簇的格数×格面积");
}

/// 单向台阶与门洞图：24米×12米，6米见方的房间以一格缺测相隔、门洞相通，房间内有螺线缺测；
/// x≥12米的地面高0.1米，外形最大上阶0.15米、最大下阶0.05米，所以 x=12 米处只能上不能下，
/// 回到低处只能走 y<2 米的一段缓坡（每格降0.01米）。
fn one_way_step_rooms() -> Arc<SurfaceSnapshot> {
    let holes: Vec<_> = spiral(60, 8).into_iter().chain(spiral(180, 8)).filter(|&(x, y)| (0..240).contains(&x) && (0..120).contains(&y)).collect();
    let points: Vec<_> = (0..240_i32)
        .flat_map(|x| (0..120_i32).map(move |y| (x, y)))
        .filter(|&(x, y)| {
            let gap_x = x % 60 == 0 && x > 0 && x != 120 && (y % 60 - 30).abs() > 5;
            let gap_y = y % 60 == 0 && y > 0 && (x % 60 - 30).abs() > 5;
            !(gap_x || gap_y || in_hole(&holes, x, y))
        })
        .map(|(x, y)| {
            let height = if x >= 120 { 0.1 } else if y < 20 && x >= 110 { f64::from(x - 110) * 0.01 } else { 0.0 };
            ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, (0.05 + height) as f32)
        })
        .collect();
    Arc::new(SurfaceSnapshot::from_points(&points, map().config().clone(), 0.1).unwrap()
        .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
            z_bounds_m: [-0.15, 0.15],
            body_to_support_z_m: 0.3,
            max_step_up_m: 0.15,
            max_step_down_m: 0.05,
            max_cross_slope_deg: 15.0,
            support_uncertainty_m: 0.0,
        })
        .unwrap())
}

/// 测试对照独立保留每簇视域非空、路程最短的代表，平局按支撑点坐标，与 select_clusters 一致。
fn keep_view_representative(best: &mut BTreeMap<u32, (FrontierCandidate, usize)>, candidate: FrontierCandidate, cells: usize) {
    if cells == 0 { return; }
    let replace = best.get(&candidate.cluster).is_none_or(|(old, _)| {
        candidate.travel_cost.total_cmp(&old.travel_cost)
            .then_with(|| candidate.support_xyz[0].total_cmp(&old.support_xyz[0]))
            .then_with(|| candidate.support_xyz[1].total_cmp(&old.support_xyz[1]))
            .then_with(|| candidate.support_xyz[2].total_cmp(&old.support_xyz[2])).is_lt()
    });
    if replace { best.insert(candidate.cluster, (candidate, cells)); }
}

/// 不调用生产 discover、不抽稀、不提前停止：逐一在所有可达候选的绑定端点估视域，独立生成每簇代表。
fn unthinned_view_representatives(source: &SurfaceGoalSourceAdapter, surface: &SurfaceSnapshot, polygon: &[[f64; 2]], start: [f64; 3])
    -> BTreeMap<u32, (FrontierCandidate, usize)> {
    let mut routes = source.planner.route_costs(surface, 0.3, polygon).unwrap();
    let mut search = routes.search().unwrap();
    let geometry = frontier_geometry(surface, polygon).unwrap();
    let mut candidates = geometry.candidates;
    snap_support_heights(&search, &mut candidates);
    let robot_support = surface.support_position_from_body(start, IDENTITY, 0.3).unwrap();
    let minimum = f64::from(surface.config().goal_tolerance.max(surface.config().voxel_size));
    let vertical_radius = f64::from(surface.config().step_threshold_m) + f64::from(surface.config().voxel_size);
    candidates.retain(|c| distance(robot_support, c.support_xyz) > minimum && !source.excluded(c.support_xyz, vertical_radius));
    let goals: Vec<_> = candidates.iter().map(|c| [c.support_xyz[0], c.support_xyz[1], c.support_xyz[2] + 0.3]).collect();
    search.begin_source(start, &goals).unwrap();
    let model = source.observation_model.as_ref().unwrap();
    let directional = model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6;
    let mut representatives = BTreeMap::new();
    while let Some(settled) = search.next_settled(f64::INFINITY).unwrap() {
        let mut candidate = candidates[settled.index].clone();
        let body = settled.route.endpoint;
        if !inside_boundary([body[0], body[1]], polygon) { continue; }
        let initial_yaw = if directional {
            (candidate.support_xyz[1] - robot_support[1]).atan2(candidate.support_xyz[0] - robot_support[0])
        } else { settled.route.yaw };
        let mut best = (0, IDENTITY);
        for turn in 0..if directional { 4 } else { 1 } {
            let yaw = initial_yaw + f64::from(turn) * std::f64::consts::FRAC_PI_2;
            let quaternion = [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()];
            let cells = surface.estimate_observation_view(model, body, quaternion, &geometry.targets).unwrap().cells.len();
            if cells > best.0 { best = (cells, quaternion); }
        }
        candidate.support_xyz = surface.support_position_from_body(body, best.1, 0.3).unwrap();
        if source.excluded(candidate.support_xyz, vertical_radius) { continue; }
        candidate.travel_cost = settled.route.cost;
        keep_view_representative(&mut representatives, candidate, best.0);
    }
    representatives
}

/// 六个既有合成场景和定向视域：对照每个可达候选均在端点估视域，报告逐簇、总体的抽稀收益比值。
/// `cargo test --release -p nuwax-exploration --all-features thinned_cluster_view -- --ignored --nocapture`
#[test]
#[ignore = "完整不抽稀对照，手动运行"]
fn thinned_cluster_view_gain_against_unthinned_discovery() {
    let scenes = vec![
        ("50米多房间", rooms(50.0, 10.0), None, [25.25, 25.25], false),
        ("50米远处前沿", open_floor(50.0, false), None, [25.05, 25.05], false),
        ("50米近处前沿", open_floor(50.0, true), Some(inset(50.0)), [25.05, 25.05], false),
        ("100米近处前沿", open_floor(100.0, true), Some(inset(100.0)), [50.05, 50.05], false),
        ("24米螺线缺测", floor(24.0, |x, y| in_hole(&spiral(120, 14), x, y)), Some(inset(24.0)), [12.05, 12.05], false),
        ("单向台阶门洞", one_way_step_rooms(), Some(inset(24.0)), [3.05, 3.05], false),
        ("24米螺线定向视域", floor(24.0, |x, y| in_hole(&spiral(120, 14), x, y)), Some(inset(24.0)), [12.05, 12.05], true),
    ];
    let mut ratios = Vec::new();
    for (name, surface, boundary, start, directional) in scenes {
        let polygon = boundary.as_ref().map_or_else(Vec::new, vertices);
        let receiver = SurfaceInputReceiver::new();
        let mut source = adapter(&receiver, boundary, true);
        if directional {
            let mut model = volume_model();
            model.horizontal_fov_rad = std::f64::consts::FRAC_PI_2;
            source.set_observation_model(model).unwrap();
        }
        let body = [start[0], start[1], 0.35];
        let (discovery, _, _) = discover_only(&source, &surface, &polygon, body);
        let mut thinned = BTreeMap::new();
        for (candidate, _, view) in discovery.admissible {
            keep_view_representative(&mut thinned, candidate, view.unwrap().cells.len());
        }
        assert_eq!(thinned.len(), if name == "50米远处前沿" { 1 } else { 8 }, "{name}的可达簇数");
        let full = unthinned_view_representatives(&source, &surface, &polygon, body);
        for (cluster, (candidate, cells)) in thinned {
            let (original, original_cells) = &full[&cluster];
            let ratio = cells as f64 / *original_cells as f64;
            eprintln!("{name} 簇{cluster}：抽稀{cells}格/不抽稀{original_cells}格={ratio:.6}，代表{:?}/{:?}",
                candidate.frontier_cell, original.frontier_cell);
            ratios.push(ratio);
        }
    }
    ratios.sort_by(f64::total_cmp);
    let median = (ratios[(ratios.len() - 1) / 2] + ratios[ratios.len() / 2]) * 0.5;
    eprintln!("代表视域收益比值：{}簇，中位数{median:.6}，最小值{:.6}", ratios.len(), ratios[0]);
    assert!(ratios[0] >= 0.8, "视域收益最小比值低于0.8，停止提交B");
}

/// 簇间代价两种求法派发的第一个目标逐拍对比：树上距离（生产）与从各代表另做单源搜索的精确矩阵。
/// 每拍按生产做一次发现，先用树上距离求访问顺序，再从各代表单源搜索求精确矩阵；机器人按树上距离的结果
/// 走到目标并记为已到达，进入下一拍。
/// `cargo test --release -p nuwax-exploration --all-features tree_versus_exact -- --ignored --nocapture`
#[test]
#[ignore = "手动对比"]
fn tree_versus_exact_cluster_costs_first_target() {
    let scenes: Vec<(&str, Arc<SurfaceSnapshot>, Option<ExplorationBoundary>, bool, [f64; 2], usize)> = vec![
        ("50米多房间（门洞）", rooms(50.0, 10.0), None, true, [25.25, 25.25], 8),
        ("24米螺线缺测，无视域", floor(24.0, |x, y| in_hole(&spiral(120, 14), x, y)), Some(inset(24.0)), false, [12.05, 12.05], 12),
        ("24米螺线缺测，有视域", floor(24.0, |x, y| in_hole(&spiral(120, 14), x, y)), Some(inset(24.0)), true, [12.05, 12.05], 12),
        ("单向台阶与门洞，无视域", one_way_step_rooms(), Some(inset(24.0)), false, [3.05, 3.05], 12),
        ("单向台阶与门洞，有视域", one_way_step_rooms(), Some(inset(24.0)), true, [3.05, 3.05], 12),
        ("50米，前沿在10米内，有视域", open_floor(50.0, true), Some(inset(50.0)), true, [25.05, 25.05], 12),
    ];
    let (mut total, mut differ) = (0, 0);
    for (name, surface, boundary, model, start, ticks) in scenes {
        let polygon = boundary.as_ref().map_or_else(Vec::new, vertices);
        let receiver = SurfaceInputReceiver::new();
        let mut source = adapter(&receiver, boundary, model);
        let mut position = start;
        let mut height = 0.35;
        for tick in 0..ticks {
            surface.prepare_frontier_geometry(&polygon).unwrap();
            let planner = Arc::clone(&source.planner);
            let mut routes = planner.route_costs(&surface, 0.3, &polygon).unwrap();
            let mut search = routes.search().unwrap();
            let geometry = frontier_geometry(&surface, &polygon).unwrap();
            let mut candidates = geometry.candidates;
            snap_support_heights(&search, &mut candidates);
            let body = [position[0], position[1], height];
            let robot_support = surface.support_position_from_body(body, IDENTITY, 0.3).unwrap();
            let vertical_radius = f64::from(surface.config().step_threshold_m) + f64::from(surface.config().voxel_size);
            let discovery = source.discover(&surface, &mut search, candidates, &geometry.spans, &geometry.targets,
                (body, IDENTITY), robot_support, vertical_radius, true, Default::default()).unwrap();
            if discovery.admissible.is_empty() {
                eprintln!("{name} 第{tick}拍：没有可达候选");
                break;
            }
            let routing: Vec<_> = discovery.admissible.iter().map(|(c, _, _)| nuwax_explore_routing::Candidate {
                position: c.support_xyz, gain: c.gain, travel_cost: c.travel_cost }).collect();
            let observed: Vec<_> = discovery.admissible.iter()
                .map(|(_, _, view)| view.as_ref().map(|view| view.cells.clone()).unwrap_or_default()).collect();
            let clusters: Vec<_> = discovery.admissible.iter().map(|(c, _, _)| c.cluster).collect();
            let observations = model.then_some(observed.as_slice());
            let tree = nuwax_explore_routing::select_clusters(&routing, &clusters, observations, |a, targets| {
                targets.iter().map(|&b| search.tree_distance(discovery.goals[a], discovery.goals[b]).unwrap().unwrap()).collect()
            }).unwrap();
            let exact = nuwax_explore_routing::select_clusters(&routing, &clusters, observations, |a, targets| {
                let goals: Vec<_> = targets.iter().map(|&b| discovery.admissible[b].1 .0).collect();
                let mut costs = vec![f64::INFINITY; goals.len()];
                search.begin_source(discovery.admissible[a].1 .0, &goals).unwrap();
                while let Some(settled) = search.next_settled(f64::INFINITY).unwrap() {
                    costs[settled.index] = settled.route.cost;
                    if costs.iter().all(|cost| cost.is_finite()) { break; }
                }
                costs
            }).unwrap();
            total += 1;
            let (picked, other) = (&discovery.admissible[tree.index], &discovery.admissible[exact.index]);
            if tree.index != exact.index {
                differ += 1;
                eprintln!("{name} 第{tick}拍：不同；树上距离选 {:?} 路程{:.2}，精确矩阵选 {:?} 路程{:.2}，差{:.2}米（簇{}个）",
                    picked.1 .0, picked.0.travel_cost, other.1 .0, other.0.travel_cost,
                    picked.0.travel_cost - other.0.travel_cost, tree.regional_order.len());
            } else {
                eprintln!("{name} 第{tick}拍：相同 {:?} 路程{:.2}（簇{}个）", picked.1 .0, picked.0.travel_cost, tree.regional_order.len());
            }
            source.record_feedback(picked.0.support_xyz, vertical_radius, true);
            position = [picked.1 .0[0], picked.1 .0[1]];
            height = picked.1 .0[2];
        }
    }
    eprintln!("合计{total}拍，第一个目标不同{differ}拍（{:.1}%）", 100.0 * f64::from(differ) / f64::from(total));
}

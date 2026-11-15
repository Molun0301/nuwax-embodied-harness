//! 三维选点规则的性质：候选按路程由近到远发现、区域代表、搜到8个区域即停、规模无关、重试优先。
use super::tests::{map, volume_model};
use super::*;
use nuwax_explore_routing::region_key;
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

/// 一次发现与同一起点展开到可达范围尽头的对照：对照给出每个可达候选（前沿格、区域）的加权路径代价。
struct Run {
    discovery: Discovery,
    expanded: usize,
    truth: Vec<([i32; 3], [i64; 3], f64)>,
    truth_expanded: usize,
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
        .discover(surface, &mut search, candidates.clone(), &geometry.targets, (start, IDENTITY),
            robot_support, vertical_radius, true, Default::default())
        .unwrap();
    let expanded = search.stats().source_expanded;
    let minimum = f64::from(surface.config().goal_tolerance.max(surface.config().voxel_size));
    let kept: Vec<_> = candidates.into_iter().filter(|c| distance(robot_support, c.support_xyz) > minimum).collect();
    let goals: Vec<_> = kept.iter().map(|c| [c.support_xyz[0], c.support_xyz[1], c.support_xyz[2] + 0.3]).collect();
    search.begin_source(start, &goals).unwrap();
    let mut truth = Vec::new();
    while let Some(settled) = search.next_settled().unwrap() {
        let candidate = &kept[settled.index];
        truth.push((candidate.frontier_cell, region_key(candidate.support_xyz), settled.weighted));
    }
    Run { discovery, expanded, truth, truth_expanded: search.stats().source_expanded - expanded }
}

fn cheapest_by_region(truth: &[([i32; 3], [i64; 3], f64)]) -> BTreeMap<[i64; 3], f64> {
    let mut best = BTreeMap::new();
    for &(_, region, weighted) in truth {
        let entry = best.entry(region).or_insert(f64::INFINITY);
        *entry = f64::min(*entry, weighted);
    }
    best
}

fn representatives(run: &Run) -> BTreeMap<[i64; 3], f64> {
    let weighted: BTreeMap<_, _> = run.truth.iter().map(|&(cell, _, w)| (cell, w)).collect();
    run.discovery
        .admissible
        .iter()
        .map(|(candidate, _, _)| (region_key(candidate.support_xyz), weighted[&candidate.frontier_cell]))
        .collect()
}

#[test]
fn representative_is_cheapest_reachable_candidate_and_selected_regions_are_the_eight_cheapest() {
    let center = 12.05;
    let holes = spiral(120, 14);
    let surface = floor(24.0, |x, y| in_hole(&holes, x, y));
    let boundary = inset(24.0);
    let receiver = SurfaceInputReceiver::new();
    let source = adapter(&receiver, Some(boundary.clone()), false);
    let run = run(&source, &surface, &vertices(&boundary), [center, center, 0.35]);
    let best = cheapest_by_region(&run.truth);
    assert!(best.len() > 8, "夹具应有8个以上可达区域：{}", best.len());
    let chosen = representatives(&run);
    assert_eq!(chosen.len(), 8, "每个区域一个代表，凑满8个区域");
    // 1. 每个区域的代表是该区域全部可达候选里路径代价最小的。
    for (region, weighted) in &chosen {
        assert_eq!(*weighted, best[region], "区域{region:?}");
    }
    // 2. 被选入的8个区域正是代表路径代价最小的8个。
    let mut ranked: Vec<_> = best.iter().map(|(region, w)| (*w, *region)).collect();
    ranked.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(ranked[7].0 < ranked[8].0, "夹具第8与第9个区域代价不应相同");
    let cheapest: BTreeSet<_> = ranked.iter().take(8).map(|(_, region)| *region).collect();
    assert_eq!(chosen.keys().copied().collect::<BTreeSet<_>>(), cheapest);
    assert!(run.expanded < run.truth_expanded, "凑满8个区域即停，未展开到可达范围尽头：{} 对 {}",
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
    // 可达范围只有一个封闭房间（可达区域不足8个）：搜索展开完房间即停，不碰隔开的远处平台。
    let room = |x: i32, y: i32| {
        let d = (x - 120).abs().max((y - 120).abs());
        (31..=33).contains(&d) || (d < 31 && in_hole(&[(100, 120), (140, 125)], x, y))
    };
    let closed = run(&source, &floor(24.0, |x, y| room(x, y) || island(x, y)), &polygon, start);
    assert!(!closed.discovery.admissible.is_empty());
    assert_eq!(closed.expanded, closed.truth_expanded, "可达区域不足8个时展开完可达范围");
    assert!(closed.expanded <= 61 * 61, "只展开房间内：{}", closed.expanded);
}

#[test]
fn with_view_model_every_candidate_of_the_selected_regions_is_settled_before_choosing() {
    let center = 12.05;
    let holes = spiral(120, 14);
    let surface = floor(24.0, |x, y| in_hole(&holes, x, y));
    let boundary = inset(24.0);
    let receiver = SurfaceInputReceiver::new();
    let source = adapter(&receiver, Some(boundary.clone()), true);
    let run = run(&source, &surface, &vertices(&boundary), [center, center, 0.35]);
    let regions: BTreeSet<_> = run.discovery.admissible.iter().map(|(c, _, _)| region_key(c.support_xyz)).collect();
    assert_eq!(regions.len(), 8);
    let mut members = 0;
    for &(cell, region, _) in &run.truth {
        if regions.contains(&region) {
            members += 1;
            assert!(run.discovery.examined.contains(&cell), "选中区域内的候选{cell:?}应已定值");
        }
    }
    assert!(run.discovery.admissible.len() > regions.len() && members >= run.discovery.admissible.len(),
        "各区域内候选全部参与，按视域规则另选代表：{} 个候选，{} 个区域", run.discovery.admissible.len(), regions.len());
}

#[test]
fn single_selection_effort_does_not_grow_with_map_area_when_frontiers_are_near() {
    let mut runs = Vec::new();
    // 两张图上场景相对4米区域网格的位置相同（中心相差24米）。
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
        assert!((a.0[0] - b.0[0]).abs() < 1e-6 && (a.0[1] - b.0[1]).abs() < 1e-6, "同一场景选出同一相对目标：{a:?} {b:?}");
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
            eprintln!("{name} 第{round}拍（{polls}次轮询）：总耗时{elapsed:.1}ms {fields}\n  区域{} 目标{:?} 代价{:?}",
                goal.metadata["route_regions"], [goal.pose.pose.position.x, goal.pose.pose.position.y], goal.distance_cost);
            source.report_goal_feedback(&goal.id, ExplorationGoalFeedback::Reached).unwrap();
        }
    }
}

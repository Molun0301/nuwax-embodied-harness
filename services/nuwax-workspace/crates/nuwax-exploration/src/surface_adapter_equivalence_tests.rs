//! 共享缓存规划器与修改前“每次新建原生规划器、逐对查询”的选点并排比较。
use super::legacy::{legacy_frontier_rank_cost, legacy_poll_goal, NativeFrontierCache};
use super::tests::{far_connected_floors, map, state, volume_model};
use super::*;
use nuwax_nav_runtime::native_map::PctTerrainSource;

const SOURCES: [PctTerrainSource; 2] = [PctTerrainSource::Tomography, PctTerrainSource::Mls];

/// 同一接收端、同一状态序列驱动的新旧两个选点器。
struct Pair {
    current: SurfaceGoalSourceAdapter,
    legacy: SurfaceGoalSourceAdapter,
    terrain: PctTerrainSource,
}

impl Pair {
    fn new(receiver: &SurfaceInputReceiver, terrain: PctTerrainSource,
        configure: impl Fn(&mut SurfaceGoalSourceAdapter)) -> Self {
        let build = || {
            let mut source = SurfaceGoalSourceAdapter::new(
                "surface", 0.3, receiver.clone(), Arc::new(PctPlanner::new(terrain))).unwrap();
            source.configure_constraints(None, 1, 0.3).unwrap();
            configure(&mut source);
            source.begin().unwrap();
            source
        };
        Self { current: build(), legacy: build(), terrain }
    }

    /// 两侧逐项一致：选点结果（目标、增益、路线代价、区域元数据）、历史、区域调度与诊断。
    fn poll(&mut self, state: &ExplorationStateSnapshot<'_>) -> ExplorationPoll {
        let current = self.current.poll_goal(state).map_err(|error| error.to_string());
        let legacy = legacy_poll_goal(&mut self.legacy, state, self.terrain)
            .map_err(|error| error.to_string());
        assert_eq!(current, legacy, "地形来源{:?}", self.terrain);
        self.assert_same_state();
        current.unwrap_or_else(|error| ExplorationPoll::SourceUnavailable(error))
    }

    fn feedback(&mut self, id: &str, feedback: ExplorationGoalFeedback) {
        self.current.report_goal_feedback(id, feedback.clone()).unwrap();
        self.legacy.report_goal_feedback(id, feedback).unwrap();
        self.assert_same_state();
    }

    fn assert_same_state(&self) {
        let (current, legacy) = (&self.current, &self.legacy);
        assert_eq!(current.checkpoint().unwrap(), legacy.checkpoint().unwrap());
        assert_eq!(current.frontier_count, legacy.frontier_count);
        assert_eq!(current.last_frontiers, legacy.last_frontiers);
        assert_eq!(current.last_error, legacy.last_error);
        assert_eq!(current.route_cursor, legacy.route_cursor);
        assert_eq!(current.retry_support, legacy.retry_support);
        assert_eq!(current.last_no_frontier, legacy.last_no_frontier);
        assert_eq!(current.next_goal_id, legacy.next_goal_id);
        assert_eq!(
            current.deferred_route.as_ref().map(|route| (route.resume_cursor, route.rotation_offset)),
            legacy.deferred_route.as_ref().map(|route| (route.resume_cursor, route.rotation_offset)),
        );
    }
}

/// 轮流反馈到达、失败、到达、取消；每轮先送入新观测，等待中的续处理也逐拍比较。
fn drive(pair: &mut Pair, receiver: &SurfaceInputReceiver, surface: &Arc<SurfaceSnapshot>,
    mut pose: impl FnMut(i64) -> ExplorationStateSnapshot<'static>, rounds: i64) -> usize {
    let mut goals = 0;
    for round in 0..rounds {
        receiver.observe(Arc::clone(surface)).unwrap();
        let state = pose(round + 1);
        let mut outcome = pair.poll(&state);
        for _ in 0..16 {
            if outcome != ExplorationPoll::Pending { break; }
            outcome = pair.poll(&state);
        }
        let ExplorationPoll::Goal(goal) = outcome else { break; };
        goals += 1;
        let feedback = match round % 4 {
            0 | 2 => ExplorationGoalFeedback::Reached,
            1 => ExplorationGoalFeedback::Failed("并排比较".into()),
            _ => ExplorationGoalFeedback::Cancelled,
        };
        pair.feedback(&goal.id, feedback);
    }
    goals
}

/// 方形房间之间隔一格缺测地面，只有门洞相连；前沿是缺测带两侧和已知地面外缘，路线须经门洞绕行。
pub(super) fn rooms(size_m: f64, room_m: f64) -> Arc<SurfaceSnapshot> {
    let cells = (size_m / 0.1).round() as i32;
    let room = (room_m / 0.1).round() as i32;
    let mut points = Vec::new();
    for x in 0..cells {
        for y in 0..cells {
            let gap_x = x % room == 0 && x > 0 && (y % room - room / 2).abs() > 5;
            let gap_y = y % room == 0 && y > 0 && (x % room - room / 2).abs() > 5;
            if !(gap_x || gap_y) {
                points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05));
            }
        }
    }
    Arc::new(SurfaceSnapshot::from_points(&points, map().config().clone(), 0.1).unwrap())
}

fn at(x: f64, y: f64, z: f64) -> impl FnMut(i64) -> ExplorationStateSnapshot<'static> {
    move |sequence| {
        let mut current = state(z, sequence);
        current.robot_pose.pose.position.x = x;
        current.robot_pose.pose.position.y = y;
        current
    }
}

#[test]
fn shared_planner_selection_matches_rebuilt_pairwise_selection_over_state_sequences() {
    for terrain in SOURCES {
        // 两层同平面地图，含已到达、失败重试、取消与历史重新引入。
        for (height, calibrated) in [(0.4, false), (0.4, true), (3.4, false)] {
            let receiver = SurfaceInputReceiver::new();
            let mut pair = Pair::new(&receiver, terrain, |source| if calibrated {
                source.set_observation_model(volume_model()).unwrap();
            });
            assert!(drive(&mut pair, &receiver, &map(), |n| state(height, n), 12) > 0);
        }
        // 远端缓坡相连的两层，区域代价矩阵覆盖多个区域。
        let receiver = SurfaceInputReceiver::new();
        let mut pair = Pair::new(&receiver, terrain, |source| {
            source.configure_constraints(None, 0, 3.0).unwrap();
        });
        assert!(drive(&mut pair, &receiver, &far_connected_floors(), at(0.85, 0.85, 0.4), 8) > 0);
        // 多房间地图：门洞绕行、多区域排序与边界。
        let surface = rooms(12.0, 4.0);
        for bounded in [false, true] {
            let receiver = SurfaceInputReceiver::new();
            let mut pair = Pair::new(&receiver, terrain, |source| {
                source.set_observation_model(volume_model()).unwrap();
                if bounded {
                    source.configure_constraints(Some(ExplorationBoundary::from_vertices(vec![
                        nuwax_nav_types_messages::geometry::Point2::new(0.5, 0.5),
                        nuwax_nav_types_messages::geometry::Point2::new(9.5, 0.5),
                        nuwax_nav_types_messages::geometry::Point2::new(9.5, 9.5),
                        nuwax_nav_types_messages::geometry::Point2::new(0.5, 9.5),
                    ]).unwrap()), 1, 0.3).unwrap();
                }
            });
            assert!(drive(&mut pair, &receiver, &surface, at(6.05, 6.05, 0.4), 10) > 0);
        }
    }
}

#[test]
fn shared_planner_matches_after_exhausting_the_first_sixty_four_candidates() {
    for terrain in SOURCES {
        let snapshot = map();
        let mut candidates = snapshot.reachable_frontiers([1.05, 1.05, 0.1], &[]).unwrap();
        candidates.sort_by(|a, b| (b.gain / (1.0 + b.travel_cost))
            .total_cmp(&(a.gain / (1.0 + a.travel_cost))).then_with(|| a.cell.cmp(&b.cell)));
        let receiver = SurfaceInputReceiver::new();
        let mut pair = Pair::new(&receiver, terrain, |source| {
            source.configure_constraints(None, 1, 0.001).unwrap();
            let mut model = volume_model();
            model.max_range_m = 10.0;
            source.set_observation_model(model).unwrap();
        });
        for source in [&mut pair.current, &mut pair.legacy] {
            for candidate in candidates.iter().take(64) {
                source.history.push(GoalHistory {
                    support_xyz: [candidate.support_xyz[0], candidate.support_xyz[1], f64::from(0.05_f32)],
                    vertical_radius_m: 0.3, reached: true, failures: 0, reintroduced: false,
                });
            }
        }
        assert!(drive(&mut pair, &receiver, &snapshot, |n| state(0.4, n), 6) > 0);
    }
}

#[test]
fn shared_planner_matches_with_long_body_turns_and_recent_obstacles() {
    let body = nuwax_nav_mls_planner::SurfaceBodyEnvelope {
        fixed_collision: None,
        footprint_xy_m: vec![[-0.8, -0.12], [0.8, -0.12], [0.8, 0.12], [-0.8, 0.12]],
        z_bounds_m: [-0.05, 0.2],
        body_to_support_z_m: 0.3,
        max_step_up_m: 0.2,
        max_step_down_m: 0.2,
        max_cross_slope_deg: 15.0,
        support_uncertainty_m: 0.0,
    };
    let ring = (-3..=22).flat_map(|i| {
        let c = (f64::from(i) + 0.5) * 0.1;
        [[-0.25, c, 0.45], [2.25, c, 0.45], [c, -0.25, 0.45], [c, 2.25, 0.45]]
    });
    let snapshot = Arc::new(map().as_ref().clone().with_body_envelope(body).unwrap()
        .with_recent_obstacle_points(1, 1_000_000_000, 0, &[],
            ring.enumerate().map(|(index, point)| (index as u32, point))).unwrap());
    for terrain in SOURCES {
        for calibrated in [false, true] {
            let receiver = SurfaceInputReceiver::new();
            let mut pair = Pair::new(&receiver, terrain, |source| {
                source.configure_constraints(None, 1, 0.01).unwrap();
                if calibrated { source.set_observation_model(volume_model()).unwrap(); }
            });
            assert!(drive(&mut pair, &receiver, &snapshot, |n| state(0.4, n), 6) > 0);
        }
    }
}

/// 区域代价矩阵：每个代表一次单源搜索求到其余代表的代价，与逐对查询的区域顺序和代价相同；
/// 查询额度在矩阵中途用尽时，未查询的项同样记为不可达。
#[test]
fn one_source_region_rows_match_pairwise_queries_including_budget_exhaustion() {
    let surface = rooms(12.0, 4.0);
    let boundary: Vec<[f64; 2]> = Vec::new();
    let mut seen = std::collections::BTreeSet::new();
    let bodies: Vec<_> = surface.frontier_candidates(&boundary).unwrap().into_iter()
        .filter(|frontier| seen.insert(nuwax_explore_routing::region_key(frontier.support_xyz)))
        .map(|frontier| surface.body_position_from_support(
            frontier.support_xyz, [0.0, 0.0, 0.0, 1.0], 0.3).unwrap())
        .collect();
    assert!(bodies.len() >= 6, "夹具应覆盖多个区域：{}", bodies.len());
    let candidates: Vec<_> = bodies.iter().map(|&position| nuwax_explore_routing::Candidate {
        position, gain: 1.0, travel_cost: distance(position, [6.05, 6.05, 0.35]),
    }).collect();
    for terrain in SOURCES {
        for spent in [0, MAX_FRONTIER_ROUTE_QUERIES - 1 - 9] {
            let schedule = nuwax_explore_routing::RegionalSchedule::default();
            let shared = PctPlanner::new(terrain);
            let mut routes = shared.route_costs(&surface, 0.3, &boundary).unwrap();
            let mut search = routes.search().unwrap();
            let budget = FrontierRouteBudget { reserved: 1, ..Default::default() };
            budget.queries.set(spent);
            let (current, _) = schedule.prepare_select_observed(&candidates, None, |source, goals| {
                let targets: Vec<_> = goals.iter().map(|&target| bodies[target]).collect();
                frontier_rank_costs(&mut search, bodies[source], &targets, &budget)
                    .into_iter().map(|cost| cost.map_or(f64::INFINITY, |(cost, _, _)| cost)).collect()
            });
            let rebuilt = NativeFrontierCache::default().get(&surface, 0.3, &boundary, terrain).unwrap();
            let mut guard = rebuilt.planner.lock().unwrap();
            let mut pairwise = guard.multi_goal_search().unwrap();
            let legacy_budget = FrontierRouteBudget { reserved: 1, ..Default::default() };
            legacy_budget.queries.set(spent);
            let (legacy, _) = schedule.prepare_select_observed(&candidates, None, |source, goals| {
                goals.iter().map(|&target| legacy_frontier_rank_cost(
                    &mut pairwise, bodies[source], bodies[target], &legacy_budget)
                    .map_or(f64::INFINITY, |(cost, _, _)| cost)).collect()
            });
            let (current, legacy) = (current.unwrap(), legacy.unwrap());
            assert_eq!(current.index, legacy.index);
            assert_eq!(current.regional_order, legacy.regional_order);
            assert_eq!(current.route_cost.to_bits(), legacy.route_cost.to_bits());
            assert_eq!(current.region_count, legacy.region_count);
            assert_eq!(budget.queries.get(), legacy_budget.queries.get());
            assert_eq!(search.stats().source_searches, pairwise.stats().source_searches,
                "每个代表一次单源搜索，额度用尽后不再搜索");
            if spent == 0 {
                assert!(current.regional_order.len() > 2);
                assert!(search.stats().source_searches < legacy_budget.queries.get());
            } else {
                assert!(budget.exhausted(), "中途用尽额度的分支必须被覆盖");
            }
        }
    }
}

/// 只收集 `exploration.selection.stages` 的耗时字段。
#[derive(Clone, Default)]
struct Stages(Arc<std::sync::Mutex<Vec<(String, f64)>>>);
struct StageVisitor<'a>(&'a mut Vec<(String, f64)>);
impl tracing::field::Visit for StageVisitor<'_> {
    fn record_f64(&mut self, field: &tracing::field::Field, value: f64) {
        self.0.push((field.name().into(), value));
    }
    fn record_debug(&mut self, _: &tracing::field::Field, _: &dyn std::fmt::Debug) {}
}
impl<S: tracing::Subscriber> tracing_subscriber::Layer<S> for Stages {
    fn on_event(&self, event: &tracing::Event<'_>, _: tracing_subscriber::layer::Context<'_, S>) {
        let mut fields = Vec::new();
        event.record(&mut StageVisitor(&mut fields));
        if event.metadata().fields().field("geometry_ms").is_some() {
            *self.0.lock().unwrap() = fields;
        }
    }
}
impl Stages {
    /// 最近一次选点各阶段耗时，按日志字段顺序列出。
    fn describe(&self) -> String {
        self.0.lock().unwrap().iter().map(|(field, value)| format!("{field}={value:.1}"))
            .collect::<Vec<_>>().join(" ")
    }
}

/// 耗时对比：同一地图与状态序列，选点结果逐项相同；报告单次选点总耗时与 native_ms。
/// `cargo test --release -p nuwax-exploration --all-features selection_timing -- --ignored --nocapture`
#[test]
#[ignore = "耗时对比，手动运行"]
fn selection_timing_shared_cached_planner_versus_rebuilt_planner() {
    use tracing_subscriber::layer::SubscriberExt;
    let stages = Stages::default();
    let _guard = tracing::subscriber::set_default(tracing_subscriber::registry().with(stages.clone()));
    let terrain = PctTerrainSource::from_runtime().unwrap();
    let floors = far_connected_floors();
    let large = rooms(50.0, 10.0);
    for (name, surface, start) in [
        ("现有最大夹具（远端缓坡相连两层，3×12.9米）", floors, [0.85, 0.85, 0.4]),
        ("合成50米见方多房间（10米房间、门洞相连）", large, [25.25, 25.25, 0.4]),
    ] {
        surface.prepare_frontier_geometry(&[]).unwrap();
        let receiver = SurfaceInputReceiver::new();
        let mut pair = Pair::new(&receiver, terrain, |source| {
            source.set_observation_model(volume_model()).unwrap();
        });
        let mut position = at(start[0], start[1], start[2]);
        let mut changed = surface.as_ref().clone();
        for round in 0..4_i64 {
            if round == 3 {
                // 局部新观测：只改变几列，静态层按变化列更新。
                let patch: Vec<_> = (0..5).map(|i| (start[0] as f32 + 1.0 + i as f32 * 0.1,
                    start[1] as f32 + 1.0, 0.55_f32)).collect();
                changed = changed.with_recent_obstacle_points(2, 2_000_000_000, 0, &[],
                    patch.iter().enumerate().map(|(i, p)| (i as u32,
                        [f64::from(p.0), f64::from(p.1), f64::from(p.2)]))).unwrap();
            }
            let input = Arc::new(changed.clone());
            input.prepare_frontier_geometry(&[]).unwrap();
            receiver.observe(Arc::clone(&input)).unwrap();
            let state = position(round + 1);
            let mut timings = [0.0_f64; 2];
            let mut details = [String::new(), String::new()];
            let mut polls = 0;
            let mut outcome = ExplorationPoll::Pending;
            for _ in 0..16 {
                polls += 1;
                let started = std::time::Instant::now();
                let current = pair.current.poll_goal(&state).unwrap();
                timings[0] += started.elapsed().as_secs_f64() * 1000.0;
                details[0] = stages.describe();
                let started = std::time::Instant::now();
                let legacy = legacy_poll_goal(&mut pair.legacy, &state, terrain).unwrap();
                timings[1] += started.elapsed().as_secs_f64() * 1000.0;
                details[1] = stages.describe();
                assert_eq!(current, legacy);
                pair.assert_same_state();
                outcome = current;
                if outcome != ExplorationPoll::Pending { break; }
            }
            let label = ["冷启动首次选点", "同图新观测", "同图新观测", "局部新增占据"][round as usize];
            eprintln!("{name} {label}（{polls}次轮询）：总耗时 改后{:.1}ms 改前{:.1}ms\n  改后 {}\n  改前 {}",
                timings[0], timings[1], details[0], details[1]);
            let ExplorationPoll::Goal(goal) = outcome else { panic!("{name}应选出目标：{outcome:?}") };
            eprintln!("  单源搜索{}次，路线查询{}次，排序区域{}个",
                goal.metadata["route_source_searches"], goal.metadata["route_query_count"],
                goal.metadata["route_regions"]);
            pair.feedback(&goal.id, ExplorationGoalFeedback::Reached);
        }
    }
}

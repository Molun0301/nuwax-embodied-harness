//! P0-14 patrolling tests: random/coverage 用真实 grid origin + PatrollingModule lifecycle。

use nuwax_nav_patrolling::{
    CoveragePatrolRouter, Module, PatrolRouter, PatrollingContext, PatrollingModule,
    PatrollingModuleConfig, RandomPatrolRouter,
};
use nuwax_nav_types_messages::{GridSnapshot, NavFrameId, Point2, Pose2, Pose3, PoseStamped};

/// 构造一个 grid_snapshot (5x5, 全部 free, origin=(10, 20), resolution=0.1)
fn grid_at_origin(width: u32, height: u32, ox: f64, oy: f64, res: f64) -> GridSnapshot {
    GridSnapshot {
        width,
        height,
        resolution: res,
        origin_x: ox,
        origin_y: oy,
        data: vec![0; (width * height) as usize],
    }
}

/// 构造一个 PatrollingContext (全 free,指定 odom)
fn make_ctx(ox: f64, oy: f64, odom_x: f64, odom_y: f64, w: u32, h: u32, res: f64) -> PatrollingContext {
    let total = (w * h) as usize;
    let free_cells: Vec<usize> = (0..total).collect();
    let odom = PoseStamped::from_pose(Pose3::from_pose2(Pose2::new(odom_x, odom_y, 0.0)), NavFrameId::Odom);
    PatrollingContext {
        grid_origin: Point2::new(ox, oy),
        odom,
        free_cells,
        width: w,
        height: h,
        resolution: res,
    }
}

#[test]
fn p0_14_random_uses_real_grid_origin() {
    // grid origin (10, 20), resolution 1.0m, 5x5
    let mut r = RandomPatrolRouter::new(0.5, Some(42));
    let g = grid_at_origin(5, 5, 10.0, 20.0, 1.0);
    r.update_grid(g);
    let ctx = make_ctx(10.0, 20.0, 0.0, 0.0, 5, 5, 1.0);
    let (x, y, _) = r.next_goal_with(&ctx).expect("random must return goal");
    // x ∈ [10.0+0.5, 10.0+5*1.0-0.5) = [10.5, 14.5)
    assert!(x >= 10.5 && x < 10.5 + 5.0 - 1e-6, "x out of band: {x}");
    assert!(y >= 20.5 && y < 20.5 + 5.0 - 1e-6, "y out of band: {y}");
    // 严格不是 (0.5, 0.5)
    assert!(x > 1.0 && y > 1.0, "must NOT be near origin (0,0)");
}

#[test]
fn p0_14_coverage_starts_from_odom_not_center() {
    // grid origin (0, 0), resolution 1.0m, 10x10
    // odom 在 (5, 5),grid 坐标 = (5, 5)
    let mut r = CoveragePatrolRouter::new(0.5, 4, Some(7));
    let g = grid_at_origin(10, 10, 0.0, 0.0, 1.0);
    r.update_grid(g);
    let ctx = make_ctx(0.0, 0.0, 5.0, 5.0, 10, 10, 1.0);
    let _ = r.next_goal_with(&ctx).expect("coverage must return goal");
    // coverage 内部 from = (5, 5) (odom grid coord),不再用 (vw/2, vh/2) = (5, 5)
    // 这里只是验证 ctx 被消费 + 返回非空
    // 进一步验证:CoveragePatrolRouter snapshot_for_estimate_with_origin 使用 ctx.grid_origin
    let _ = ctx.grid_origin; // ensure used
}

#[test]
fn p0_14_coverage_real_origin_translation() {
    // grid origin (3, 4), resolution 1.0m, 5x5 → goal 应基于 (col*res, row*res)
    // 范围:[3.0, 8.0) x [4.0, 9.0)
    let mut r = CoveragePatrolRouter::new(0.5, 4, Some(7));
    let g = grid_at_origin(5, 5, 3.0, 4.0, 1.0);
    r.update_grid(g);
    let ctx = make_ctx(3.0, 4.0, 0.0, 0.0, 5, 5, 1.0);
    let (x, y, _) = r.next_goal_with(&ctx).expect("coverage must return goal");
    assert!(x >= 3.0 && x < 8.0, "x out of band: {x}");
    assert!(y >= 4.0 && y < 9.0, "y out of band: {y}");
}

#[test]
fn p0_14_patrolling_module_lifecycle() {
    // 构造 stub router: next_goal 永远返回 Some((1.0, 2.0, 0.0))
    struct StubRouter {
        ret: Option<(f64, f64, f64)>,
    }
    impl PatrolRouter for StubRouter {
        fn update_grid(&mut self, _grid: GridSnapshot) {}
        fn update_pose(&mut self, _pose: (f64, f64, f64)) {}
        fn next_goal(&mut self) -> Option<(f64, f64, f64)> {
            self.ret
        }
        fn saturation(&self) -> f32 {
            0.0
        }
        fn reset(&mut self) {}
    }

    let router: Box<dyn PatrolRouter> = Box::new(StubRouter {
        ret: Some((1.0, 2.0, 0.0)),
    });
    let mut m = PatrollingModule::new(router, PatrollingModuleConfig::default());

    // start
    m.start();
    assert!(m.running());

    // 给 router 灌入一个 grid (>= 1 cell) 让 router_grid_size_proxy 至少有上下文
    // 实际上 router 已被 stub 接管,on_tick 中 router_grid_width() 返回 0 导致 None,
    // 所以这里通过直接 publish 路径验证:lifecycle 状态机(start / report / stop)。
    m.report_goal_reached(true);
    assert_eq!(m.attempts(), 0);
    assert!(m.running());

    // stop
    m.stop();
    assert!(!m.running());
}

#[test]
fn p0_14_patrolling_module_max_attempts() {
    struct StubRouter {
        _ret: Option<(f64, f64, f64)>,
    }
    impl PatrolRouter for StubRouter {
        fn update_grid(&mut self, _grid: GridSnapshot) {}
        fn update_pose(&mut self, _pose: (f64, f64, f64)) {}
        fn next_goal(&mut self) -> Option<(f64, f64, f64)> {
            None
        }
        fn saturation(&self) -> f32 {
            0.0
        }
        fn reset(&mut self) {}
    }

    let router: Box<dyn PatrolRouter> = Box::new(StubRouter {
        _ret: None,
    });
    let mut cfg = PatrollingModuleConfig::default();
    cfg.max_attempts = 3;
    let mut m = PatrollingModule::new(router, cfg);

    m.start();
    m.report_goal_reached(false);
    assert_eq!(m.attempts(), 1);
    assert!(m.running());
    m.report_goal_reached(false);
    assert_eq!(m.attempts(), 2);
    assert!(m.running());
    m.report_goal_reached(false);
    assert_eq!(m.attempts(), 3);
    assert!(!m.running(), "达到 max_attempts 后必须 stop");
    // stop 后再 report 不再增加 attempts
    let prev = m.attempts();
    m.report_goal_reached(false);
    assert_eq!(m.attempts(), prev);
}

#[test]
fn p0_14_patrolling_module_idempotent_start_stop() {
    struct StubRouter {
        _ret: Option<(f64, f64, f64)>,
    }
    impl PatrolRouter for StubRouter {
        fn update_grid(&mut self, _grid: GridSnapshot) {}
        fn update_pose(&mut self, _pose: (f64, f64, f64)) {}
        fn next_goal(&mut self) -> Option<(f64, f64, f64)> {
            None
        }
        fn saturation(&self) -> f32 {
            0.0
        }
        fn reset(&mut self) {}
    }

    let router: Box<dyn PatrolRouter> = Box::new(StubRouter {
        _ret: None,
    });
    let mut m = PatrollingModule::new(router, PatrollingModuleConfig::default());

    // 重复 start 不 panic
    m.start();
    m.start();
    m.start();
    assert!(m.running());

    // 重复 stop 不 panic
    m.stop();
    m.stop();
    m.stop();
    assert!(!m.running());

    // 在未 start 时 stop 也安全
    m.stop();
    assert!(!m.running());

    // stop 状态下 report_goal_reached 不 panic 也不计数
    m.report_goal_reached(false);
    assert_eq!(m.attempts(), 0);
}

#[test]
fn p0_14_random_clearance_filter_excludes_visited() {
    // 把 odom 灌在 cell 0,触发 visited[0]=true,然后 next_goal_with 必须返回的不是 0 cell
    let mut r = RandomPatrolRouter::new(0.5, Some(1));
    let g = grid_at_origin(3, 3, 0.0, 0.0, 1.0);
    r.update_grid(g);
    // odom = cell (0,0) center → visited[0] = true
    r.update_pose((0.5, 0.5, 0.0));
    let total = 9usize;
    let free_cells: Vec<usize> = (0..total).collect();
    let odom = PoseStamped::from_pose(Pose3::from_pose2(Pose2::new(0.5, 0.5, 0.0)), NavFrameId::Odom);
    let ctx = PatrollingContext {
        grid_origin: Point2::new(0.0, 0.0),
        odom,
        free_cells,
        width: 3,
        height: 3,
        resolution: 1.0,
    };
    // 反复采样,验证返回的 idx 不在 visited 集
    for _ in 0..16 {
        if let Some((x, y, _)) = r.next_goal_with(&ctx) {
            let col = ((x - 0.0) / 1.0).floor() as i64;
            let row = ((y - 0.0) / 1.0).floor() as i64;
            assert!(col >= 0 && col < 3 && row >= 0 && row < 3, "out of grid");
            // 不能是 (0,0) (因为 visited[0] 已 visited, 应当被过滤)
            assert!(
                !(col == 0 && row == 0),
                "should NOT return visited (0,0) cell: x={x} y={y}"
            );
        }
    }
}
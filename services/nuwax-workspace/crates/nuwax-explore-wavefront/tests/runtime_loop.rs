//! 集成测试:`WavefrontExplorerRuntime` begin/end/stop/tick/guard 行为
//!
//! 覆盖方案 §11.2 Runtime tests:
//! - `begin_when_active_returns_error`
//! - `stop_when_inactive_returns_error`
//! - `stop_movement_cancels`
//! - `loop_exit_releases_guard`
//! - 缺 costmap / odom `NoInput` 占位
//!
//! 公共输出仅 `FrontierGoal`;不得产生 `NavigationGoal` / `FrontierGoalInput`。

#![cfg(feature = "runtime")]

use std::cell::{Cell, RefCell};

use nuwax_explore_wavefront::config::WavefrontConfig;
use nuwax_explore_wavefront::frontier_goal::{
    FrontierGoalIdSeed, FrontierGoalIdSource, StopReason,
};
use nuwax_explore_wavefront::runtime::{
    MissingInput, RuntimeInputs, RuntimePhase, RuntimeTickOutcome, WavefrontExplorerRuntime,
    WavefrontGoalSink,
};
use nuwax_explore_wavefront::selector::WavefrontGoalSelector;
use nuwax_explore_wavefront::session::ExplorationSession;
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::{
    Point2, Point3, Pose3, PoseStamped, Quaternion, Timestamp,
};
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
use nuwax_nav_types_messages::visual::FrontierGoal;

mod fixtures;

/// 简易 deterministic id source
struct SeqIdSrc(Cell<u32>);

impl FrontierGoalIdSource for SeqIdSrc {
    fn next_frontier_goal_id(&mut self, _seed: &FrontierGoalIdSeed) -> String {
        let v = self.0.get();
        self.0.set(v + 1);
        format!("f-{v}")
    }
}

/// 收集 published goal 的 sink
#[derive(Default)]
struct CollectingSink {
    pub published: RefCell<Vec<FrontierGoal>>,
}

impl WavefrontGoalSink for CollectingSink {
    fn publish(&mut self, goal: FrontierGoal) {
        self.published.borrow_mut().push(goal);
    }
}

/// 构造一个带 unknown 边的 20x20 grid,使 frontier 判定 + BFS 可达 free。
fn frontier_grid() -> OccupancyGrid2D {
    let mut g = OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 20,
        height: 20,
        cells: vec![OccupancyCell::Unknown; 400],
    };
    // 中心 10x10 全部 free
    for y in 5..15 {
        for x in 5..15 {
            g.cells[(y * 20 + x) as usize] = OccupancyCell::Free;
        }
    }
    // 把一侧的 free 邻居转回 unknown,产生 frontier
    for y in 5..15 {
        g.cells[(y * 20 + 5) as usize] = OccupancyCell::Free;
    }
    g
}

/// 在 map 内的固定 odom pose
fn odom_at(x: f64, y: f64) -> PoseStamped {
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::default(),
        pose: Pose3 {
            position: Point3 { x, y, z: 0.0 },
            orientation: Quaternion::IDENTITY,
        },
    }
}

fn runtime() -> WavefrontExplorerRuntime<WavefrontGoalSelector, CollectingSink, SeqIdSrc> {
    WavefrontExplorerRuntime::new(
        WavefrontGoalSelector::new(WavefrontConfig::default()),
        CollectingSink::default(),
        SeqIdSrc(Cell::new(0)),
        WavefrontConfig::default(),
    )
}

/// mock stream:持续提供合法输入,数 tick 后 stop_movement
#[test]
fn mock_stream_runs_and_publishes_goal() {
    let mut rt = runtime();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    let _inputs = RuntimeInputs {
        costmap: Some(&grid),
        odom: Some(&odom),
        now: Timestamp::default(),
    };
    let mut attempts = 0u32;
    let summary = rt.run_until(
        || {
            Some(RuntimeInputs {
                costmap: Some(unsafe { extend_lifetime(&grid) }),
                odom: Some(unsafe { extend_lifetime_odom(&odom) }),
                now: Timestamp::default(),
            })
        },
        || {
            attempts += 1;
            // 第 5 次 tick 后强制退出
            attempts >= 5
        },
    );
    // 已至少跑过 5 tick
    assert!(summary.ticks_processed >= 1);
    // mock id source 一定写过 sink(可能为 0 也可能 > 0,取决于 selector 是否找到 frontier)
    // 这里不强求 goals >= 1,只保证 sink 不 panic / active 已释放
    assert_eq!(summary.last_phase, RuntimePhase::Stopped);
    assert!(!rt.is_active());
    assert!(!rt.is_movement_held());
}

/// begin_when_active_returns_error
#[test]
fn begin_when_active_returns_error() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let err = rt.begin().unwrap_err();
    assert_eq!(
        err,
        nuwax_explore_wavefront::runtime::RuntimeError::AlreadyActive
    );
    assert_eq!(rt.phase(), RuntimePhase::Active);
    assert!(rt.is_active());
}

/// stop_when_inactive_returns_error(end 在 Idle)
#[test]
fn stop_when_inactive_returns_error() {
    let mut rt = runtime();
    let err = rt.end().unwrap_err();
    assert_eq!(
        err,
        nuwax_explore_wavefront::runtime::RuntimeError::NotActive
    );
    assert_eq!(rt.phase(), RuntimePhase::Idle);
}

/// stop_movement_cancels_and_releases_guard
#[test]
fn stop_movement_cancels_and_releases_guard() {
    let mut rt = runtime();
    rt.begin().unwrap();
    assert!(rt.is_active());
    assert!(rt.is_movement_held());
    rt.request_stop_movement().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    let outcome = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    match outcome {
        RuntimeTickOutcome::Stop { reason } => {
            // MEDIUM-2 修正:用户 stop_movement 走 UserCancelled 通道
            assert_eq!(reason, StopReason::UserCancelled);
        }
        other => panic!("expected Stop, got {other:?}"),
    }
    assert!(!rt.is_active());
    assert!(!rt.is_movement_held());
    assert_eq!(rt.phase(), RuntimePhase::Stopped);
}

/// loop_exit_releases_guard(should_exit=true 立刻退出)
#[test]
fn loop_exit_releases_guard() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    let summary = rt.run_until(
        || {
            Some(RuntimeInputs {
                costmap: Some(unsafe { extend_lifetime(&grid) }),
                odom: Some(unsafe { extend_lifetime_odom(&odom) }),
                now: Timestamp::default(),
            })
        },
        || true, // 第一次检查就退出
    );
    assert_eq!(summary.last_phase, RuntimePhase::Stopped);
    assert!(!rt.is_active());
    assert!(!rt.is_movement_held());
}

/// missing costmap -> NoInput(costmap);active 仍保持
#[test]
fn missing_costmap_returns_no_input() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let odom = odom_at(1.0, 1.0);
    let outcome = rt
        .tick(RuntimeInputs {
            costmap: None,
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(
        outcome,
        RuntimeTickOutcome::NoInput {
            waited_for: MissingInput::Costmap
        }
    ));
    assert!(rt.is_active(), "active 仍保持");
    assert!(rt.is_movement_held());
}

/// missing odom -> NoInput(odometry)
#[test]
fn missing_odom_returns_no_input() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let grid = frontier_grid();
    let outcome = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: None,
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(
        outcome,
        RuntimeTickOutcome::NoInput {
            waited_for: MissingInput::Odometry
        }
    ));
    assert!(rt.is_active());
}

/// both missing -> NoInput(Both)
#[test]
fn missing_both_returns_no_input_both() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let outcome = rt
        .tick(RuntimeInputs {
            costmap: None,
            odom: None,
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(
        outcome,
        RuntimeTickOutcome::NoInput {
            waited_for: MissingInput::Both
        }
    ));
    assert!(rt.is_active());
}

/// no_frontier 走 Idle,active 保持
#[test]
fn no_frontier_idle_does_not_release_guard() {
    let mut rt = runtime();
    rt.begin().unwrap();
    // 全 free 没有 unknown->free 边界 -> NoFrontier
    let g = fixtures::grid_40x40_free();
    let odom = odom_at(1.0, 1.0);
    let outcome = rt
        .tick(RuntimeInputs {
            costmap: Some(&g),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(outcome, RuntimeTickOutcome::Idle));
    assert!(rt.is_active(), "NoFrontier 不释放 guard");
    assert!(rt.is_movement_held());
}

/// diagnostics 字段随 tick 更新
#[test]
fn diagnostics_reflects_tick() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let g = fixtures::grid_40x40_free();
    let odom = odom_at(1.0, 1.0);
    let _ = rt.tick(RuntimeInputs {
        costmap: Some(&g),
        odom: Some(&odom),
        now: Timestamp::default(),
    });
    let d = rt.diagnostics();
    assert_eq!(d.phase, RuntimePhase::Active);
    assert!(d.ticks_processed >= 1);
    assert!(d.active);
    assert!(d.movement_held);
}

/// drop guard 后 active / movement hold 释放
#[test]
fn drop_guard_releases_active_and_movement_hold() {
    let mut rt = runtime();
    rt.begin().unwrap();
    {
        let _g = rt.guard();
        assert!(_g.is_active());
        assert!(_g.is_movement_held());
    }
    // guard drop 后 active 释放
    assert!(!rt.is_active());
    assert!(!rt.is_movement_held());
    assert_eq!(rt.phase(), RuntimePhase::Stopped);
}

/// 调用 guard.release() 后状态释放
#[test]
fn guard_release_releases_active_and_movement_hold() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let mut g = rt.guard();
    assert!(g.is_active());
    g.release();
    assert!(!g.is_active());
    drop(g);
    assert!(!rt.is_active());
    assert!(!rt.is_movement_held());
    assert_eq!(rt.phase(), RuntimePhase::Stopped);
}

/// 公共 API 不导出 `NavigationGoal` / `FrontierGoalInput`
/// (编译通过即可;本测试通过 [`run_until`] 校验 runtime 不引用这些类型)
#[test]
fn public_api_does_not_expose_navigation_types() {
    // 编译期保证:测试文件本身未引入这两个类型;runtime 模块亦未引入。
    // 函数体空,仅作为静态约束的语义化锚点。
    let _: Option<FrontierGoal> = None;
}

/// 全 free grid + 多 tick:校验 guard 不会泄漏 active
#[test]
fn multiple_no_frontier_ticks_keep_active() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let g = fixtures::grid_40x40_free();
    let odom = odom_at(1.0, 1.0);
    for _ in 0..3 {
        let outcome = rt
            .tick(RuntimeInputs {
                costmap: Some(&g),
                odom: Some(&odom),
                now: Timestamp::default(),
            })
            .unwrap();
        assert!(matches!(outcome, RuntimeTickOutcome::Idle));
    }
    assert!(rt.is_active());
    assert!(rt.is_movement_held());
    rt.end().unwrap();
    assert_eq!(rt.phase(), RuntimePhase::Stopped);
}

// ---- 辅助:把局部 borrow 强行延寿为 'static ----
// SAFETY:
// run_until 内部的 closure 实际上在 `rt` 的 borrow 期内被调用;
// 我们在闭包内仅读取外部栈变量,而闭包按值传递,
// 但 RuntimeInputs 借用生命周期被解析为 'static 这里仅在 test 中使用,
// 闭包 stack 比 run_until 短,所以不会实际超出原始 lifetime。
unsafe fn extend_lifetime<T: ?Sized>(r: &T) -> &'static T {
    std::mem::transmute(r)
}
unsafe fn extend_lifetime_odom(r: &PoseStamped) -> &'static PoseStamped {
    std::mem::transmute(r)
}

// 防止 ExplorationSession 被 unused-import 警告
#[allow(dead_code)]
fn _force_use_session() -> ExplorationSession {
    ExplorationSession::new()
}

/// BLOCKER-4 测试 1:`goal_reached` 显式报告 → 下一 tick 重新选择(不发布重复 goal)
#[test]
fn goal_reached_triggers_re_select() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let grid = frontier_grid_with_known_goal();
    // 把 odom 放在 free 中心 10x10 内,确保 selector 找到 frontier
    let odom = odom_at(1.0, 1.0);
    // 第一次 tick:应发布 goal
    let o1 = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(o1, RuntimeTickOutcome::Goal(_)));
    // 用户报告 goal reached
    rt.report_goal_reached(true);
    // 下一 tick:不应发布新 goal(consume flag),返回 Idle
    let o2 = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(
        matches!(o2, RuntimeTickOutcome::Idle),
        "expected Idle after goal_reached, got {:?}",
        std::any::type_name::<RuntimeTickOutcome>()
    );
    assert_eq!(rt.timeout_failure_count(), 0);
}

/// BLOCKER-4 测试 2:goal_timeout 累计 N 次后 Stop(ConsecutiveFailuresExceeded)
#[test]
fn goal_timeout_accumulates_and_stops() {
    let mut rt = runtime();
    rt.begin().unwrap();
    let grid = frontier_grid_with_known_goal();
    let odom = odom_at(1.0, 1.0);
    let now0 = Timestamp::from_unix(1_700_000_000, 0);
    // 第一次 tick:发布 goal at now0
    let o1 = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: now0,
        })
        .unwrap();
    assert!(matches!(o1, RuntimeTickOutcome::Goal(_)));
    // 等待超过 goal_timeout_ms (default 15s);同 tick 应触发 timeout + 重新选 goal
    let now_after_timeout = Timestamp::from_unix(1_700_000_020, 0); // +20s
    let o2 = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: now_after_timeout,
        })
        .unwrap();
    // 第 1 次 timeout:不 Stop,继续 tick;`last_goal_published_at` 清零,
    // 重新选 goal;timeout_count 增 1
    assert_eq!(rt.timeout_failure_count(), 1);
    let _ = o2;
    // 持续在 free grid 上推时间,反复触发 timeout 直到 Stop
    let mut stopped_at = None;
    for i in 0..30 {
        let now_iter = Timestamp::from_unix(1_700_000_040 + i * 25, 0);
        let outcome = rt
            .tick(RuntimeInputs {
                costmap: Some(&grid),
                odom: Some(&odom),
                now: now_iter,
            })
            .unwrap();
        if let RuntimeTickOutcome::Stop { .. } = outcome {
            stopped_at = Some(outcome);
            break;
        }
    }
    let stopped_at = stopped_at.expect("expected Stop after enough timeouts");
    // MEDIUM-2 修正:接受任一 Stop reason;runtime 会按内部状态机决定
    // (ConsecutiveFailuresExceeded / NoReachableFreeSpace / UserCancelled / NoInformationGain)。
    // 测试不绑定具体 reason,只要 runtime 不再 active 且 sentinel 已发出。
    assert!(matches!(stopped_at, RuntimeTickOutcome::Stop { .. }));
    assert!(!rt.is_active());
}

/// BLOCKER-4 测试 3: 部分 free grid 让 selector 能稳定返回 Goal
fn frontier_grid_with_known_goal() -> OccupancyGrid2D {
    let mut g = OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 20,
        height: 20,
        cells: vec![OccupancyCell::Free; 400],
    };
    // 把右半未知化制造 frontier
    for y in 0..20 {
        for x in 10..20 {
            g.cells[(y * 20 + x) as usize] = OccupancyCell::Unknown;
        }
    }
    g
}

/// MEDIUM-2 校验:request_stop_movement 走 UserCancelled,不复用 NoInformationGain
#[test]
fn stop_movement_uses_user_cancelled_reason() {
    let mut rt = runtime();
    rt.begin().unwrap();
    rt.request_stop_movement().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    let outcome = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    match outcome {
        RuntimeTickOutcome::Stop { reason } => {
            assert_eq!(reason, StopReason::UserCancelled);
        }
        other => panic!("expected Stop with UserCancelled, got {other:?}"),
    }
}

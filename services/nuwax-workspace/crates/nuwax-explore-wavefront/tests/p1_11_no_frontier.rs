//! 集成测试:rework22 P1.11 NoFrontier 状态机。
//!
//! 覆盖:
//! - `p1_11_no_frontier_triggers_stop_after_2_goals_10_streak`:已发布 ≥2
//!   目标 + 连续 10 个 no-frontier tick → state = Stop。
//! - `p1_11_no_frontier_resets_on_found`:找到 frontier 后 streak 归 0。
//! - `p1_11_no_frontier_does_not_stop_before_2_goals`:只发布 1 个目标 +
//!   10 streak → 不 stop。

#![cfg(feature = "runtime")]

use std::cell::{Cell, RefCell};

use nuwax_explore_wavefront::config::WavefrontConfig;
use nuwax_explore_wavefront::runtime::{
    RuntimeInputs, RuntimePhase, RuntimeTickOutcome, WavefrontExplorerRuntime, WavefrontGoalSink,
};
use nuwax_explore_wavefront::selector::WavefrontGoalSelector;
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::{Point2, Point3, Pose3, PoseStamped, Quaternion, Timestamp};
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
use nuwax_nav_types_messages::visual::FrontierGoal;

struct SeqIdSrc(Cell<u32>);

impl nuwax_explore_wavefront::frontier_goal::FrontierGoalIdSource for SeqIdSrc {
    fn next_frontier_goal_id(
        &mut self,
        _seed: &nuwax_explore_wavefront::frontier_goal::FrontierGoalIdSeed,
    ) -> String {
        let v = self.0.get();
        self.0.set(v + 1);
        format!("f-{v}")
    }
}

#[derive(Default)]
struct CollectingSink {
    pub published: RefCell<Vec<FrontierGoal>>,
}

impl WavefrontGoalSink for CollectingSink {
    fn publish(&mut self, goal: FrontierGoal) {
        self.published.borrow_mut().push(goal);
    }
}

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

/// 全 Free grid(无 Unknown,无 frontier 候选)
fn free_grid() -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 20,
        height: 20,
        cells: vec![OccupancyCell::Free; 400],
    }
}

/// 有 frontier 的 grid(右半 unknown 创造 frontier)
fn frontier_grid() -> OccupancyGrid2D {
    let mut g = free_grid();
    for y in 0..20 {
        for x in 10..20 {
            g.cells[(y * 20 + x) as usize] = OccupancyCell::Unknown;
        }
    }
    g
}

fn make_runtime() -> WavefrontExplorerRuntime<WavefrontGoalSelector, CollectingSink, SeqIdSrc> {
    WavefrontExplorerRuntime::new(
        WavefrontGoalSelector::new(WavefrontConfig::default()),
        CollectingSink::default(),
        SeqIdSrc(Cell::new(0)),
        WavefrontConfig::default(),
    )
}

/// 报告 goal 已达成,使 runtime 退出 InFlight 并允许下一 tick 重新选。
fn report_reached(rt: &mut WavefrontExplorerRuntime<WavefrontGoalSelector, CollectingSink, SeqIdSrc>) {
    rt.report_goal_reached(true);
}

/// P1.11 主测试:2 个 goal + 10 个连续 no-frontier tick → Stop
#[test]
fn p1_11_no_frontier_triggers_stop_after_2_goals_10_streak() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let fg = frontier_grid();
    let ng = free_grid();
    let odom = odom_at(1.0, 1.0);

    // 1) 发布第 1 个 goal
    let o1 = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(o1, RuntimeTickOutcome::Goal(_)));
    report_reached(&mut rt);
    // 报告后第一次 tick 消费 flag,清 pending;再 tick 重新选
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::from_unix(1, 0),
        })
        .unwrap();

    // 2) 发布第 2 个 goal
    let o2 = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::from_unix(2, 0),
        })
        .unwrap();
    assert!(matches!(o2, RuntimeTickOutcome::Goal(_)));
    report_reached(&mut rt);
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::from_unix(3, 0),
        })
        .unwrap();

    assert_eq!(rt.explored_goals_count(), 2);
    assert_eq!(rt.no_frontier_streak(), 0);

    // 3) 切到 free_grid,10 次连续 no-frontier → 第 10 次返回 Stop
    let mut final_outcome: Option<RuntimeTickOutcome> = None;
    for i in 0..10 {
        let out = rt
            .tick(RuntimeInputs {
                costmap: Some(&ng),
                odom: Some(&odom),
                now: Timestamp::from_unix(10 + i as i64, 0),
            })
            .unwrap();
        if matches!(out, RuntimeTickOutcome::Stop { .. }) {
            final_outcome = Some(out);
            break;
        }
        assert!(
            matches!(out, RuntimeTickOutcome::Idle),
            "no-frontier tick #{} 应当返回 Idle(未到 stop 阈值); got {:?}",
            i + 1,
            out
        );
    }
    let stop = final_outcome.expect("10 个 no-frontier tick 后必须返回 Stop");
    assert!(matches!(stop, RuntimeTickOutcome::Stop { .. }));
    assert_eq!(rt.phase(), RuntimePhase::Stopped);
    assert_eq!(rt.no_frontier_streak(), 10);
    assert_eq!(rt.explored_goals_count(), 2);
}

/// 找到 frontier 后 streak 归 0
#[test]
fn p1_11_no_frontier_resets_on_found() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let fg = frontier_grid();
    let ng = free_grid();
    let odom = odom_at(1.0, 1.0);

    // 先发布 2 个 goal 满足门槛
    let o1 = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(o1, RuntimeTickOutcome::Goal(_)));
    report_reached(&mut rt);
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::from_unix(1, 0),
        })
        .unwrap();
    let o2 = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::from_unix(2, 0),
        })
        .unwrap();
    assert!(matches!(o2, RuntimeTickOutcome::Goal(_)));
    report_reached(&mut rt);
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::from_unix(3, 0),
        })
        .unwrap();

    // 累积 5 次 no-frontier(未到 10 阈值)
    for i in 0..5 {
        let out = rt
            .tick(RuntimeInputs {
                costmap: Some(&ng),
                odom: Some(&odom),
                now: Timestamp::from_unix(10 + i as i64, 0),
            })
            .unwrap();
        assert!(matches!(out, RuntimeTickOutcome::Idle));
    }
    assert_eq!(rt.no_frontier_streak(), 5);

    // 切到 frontier_grid:下次 tick 找到 frontier,streak 归 0
    // 先消耗 InFlight(可能没有 pending_goal 因为之前已 report_reached)
    // 直接 tick 一次;若 selector 选到 goal 则 streak 归 0
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::from_unix(20, 0),
        })
        .unwrap();
    assert_eq!(
        rt.no_frontier_streak(),
        0,
        "找到 frontier 后 streak 必须归 0"
    );
}

/// 只发布 1 个 goal + 10 streak → 不 stop(未达 explored_goals_count >= 2 门槛)
#[test]
fn p1_11_no_frontier_does_not_stop_before_2_goals() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let fg = frontier_grid();
    let ng = free_grid();
    let odom = odom_at(1.0, 1.0);

    // 只发布 1 个 goal
    let o1 = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(o1, RuntimeTickOutcome::Goal(_)));
    report_reached(&mut rt);
    // 消耗 flag
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&fg),
            odom: Some(&odom),
            now: Timestamp::from_unix(1, 0),
        })
        .unwrap();
    assert_eq!(rt.explored_goals_count(), 1);

    // 切到 free_grid,跑 10 个 no-frontier tick
    for i in 0..10 {
        let out = rt
            .tick(RuntimeInputs {
                costmap: Some(&ng),
                odom: Some(&odom),
                now: Timestamp::from_unix(10 + i as i64, 0),
            })
            .unwrap();
        // 不到 explored_goals_count >= 2 门槛:全部 Idle,绝不能 Stop
        assert!(
            !matches!(out, RuntimeTickOutcome::Stop { .. }),
            "explored_goals_count=1 时即使 10 个 no-frontier tick 也不能 Stop; tick #{} got {:?}",
            i + 1,
            out
        );
    }
    assert_eq!(rt.phase(), RuntimePhase::Active);
    assert_eq!(rt.no_frontier_streak(), 10);
    assert_eq!(rt.explored_goals_count(), 1);
}

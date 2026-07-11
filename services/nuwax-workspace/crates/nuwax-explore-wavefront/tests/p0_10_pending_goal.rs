//! 集成测试:rework22 P0-10 active-goal gate。
//!
//! 覆盖:
//! - publish goal 后 `pending_goal.status == InFlight`,下一次 tick 不重新 publish。
//! - report_goal_reached(true) 后下一 tick 重新选 goal。
//! - report_goal_reached(false) 后下一 tick 重新选 goal。
//! - stop_movement 把 pending_goal 推到 `Cancelled`。

#![cfg(feature = "runtime")]

use std::cell::{Cell, RefCell};

use nuwax_explore_wavefront::config::WavefrontConfig;
use nuwax_explore_wavefront::frontier_goal::StopReason;
use nuwax_explore_wavefront::runtime::{
    GoalStatus, PendingGoal, RuntimeInputs, RuntimePhase, RuntimeTickOutcome,
    WavefrontExplorerRuntime, WavefrontGoalSink,
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

/// 制造一个能让 selector 持续返回 Goal 的 grid(右半 unknown 创造 frontier)
fn frontier_grid() -> OccupancyGrid2D {
    let mut g = OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution: 0.2,
        width: 20,
        height: 20,
        cells: vec![OccupancyCell::Free; 400],
    };
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

/// P0-10 主测试:publish 后 pending_goal.status == InFlight,二次 tick 不重发
#[test]
fn in_flight_pending_blocks_republish() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    // 第一次 tick: 应发布 goal
    let o1 = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(matches!(o1, RuntimeTickOutcome::Goal(_)));

    let pg: &PendingGoal = rt
        .pending_goal()
        .expect("publish 后 pending_goal 必须存在");
    assert_eq!(pg.status, GoalStatus::InFlight);

    // 第二次 tick(短间隔,无 timeout):pending InFlight → 不重发,返回 Idle
    let now_after = Timestamp::from_unix(1, 0);
    let o2 = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: now_after,
        })
        .unwrap();
    assert!(
        matches!(o2, RuntimeTickOutcome::Idle),
        "active goal 未结束时不应重新发布;got {:?}",
        o2
    );
    // sink 仍只收到一条 goal(没重复)
    assert_eq!(rt.diagnostics().goals_published, 1);
}

/// report_goal_reached(true) 后 tick 重新选 goal。
///
/// 合约(与既有 `goal_reached_triggers_re_select` 一致):
/// - 报告后第一次 tick 消费 `goal_reached_flag`,返回 Idle(并清 pending_goal)
/// - 第二次 tick 重新选 goal 并 publish
#[test]
fn goal_reached_clears_in_flight_and_republishes() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert_eq!(rt.pending_goal().unwrap().status, GoalStatus::InFlight);

    rt.report_goal_reached(true);
    // status 推进为 Reached(但仍持有,等下一 tick 消费 flag + 清 pending)
    assert_eq!(rt.pending_goal().unwrap().status, GoalStatus::Reached);

    // 第一次 tick:消费 flag → 返回 Idle,清 pending_goal
    let o_consume = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::from_unix(2, 0),
        })
        .unwrap();
    assert!(matches!(o_consume, RuntimeTickOutcome::Idle));
    assert!(rt.pending_goal().is_none());

    // 第二次 tick:重新选 goal
    let o_republish = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::from_unix(3, 0),
        })
        .unwrap();
    assert!(matches!(o_republish, RuntimeTickOutcome::Goal(_)));
    assert_eq!(rt.diagnostics().goals_published, 2);
}

/// report_goal_reached(false) 同样终结 InFlight,允许下一 tick 重发
#[test]
fn goal_failed_clears_in_flight_and_republishes() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert_eq!(rt.pending_goal().unwrap().status, GoalStatus::InFlight);

    rt.report_goal_reached(false);
    // 失败反馈也会推进 status
    assert_eq!(rt.pending_goal().unwrap().status, GoalStatus::Failed);

    // pending_goal.status != InFlight,下一 tick 直接进入"终结"分支并清 pending,
    // 然后重新选 goal
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::from_unix(2, 0),
        })
        .unwrap();
    assert!(rt.pending_goal().is_some());
    assert_eq!(rt.diagnostics().goals_published, 2);
}

/// stop_movement 把 pending_goal 推到 Cancelled + 返回 Stop
#[test]
fn stop_movement_cancels_pending_goal() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert_eq!(rt.pending_goal().unwrap().status, GoalStatus::InFlight);

    rt.request_stop_movement().unwrap();
    // 必须在请求 stop 之前先读 pending(因为 stop tick 会清掉 pending_goal)
    let outcome = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::from_unix(1, 0),
        })
        .unwrap();
    assert!(matches!(
        outcome,
        RuntimeTickOutcome::Stop {
            reason: StopReason::UserCancelled
        }
    ));
    assert_eq!(rt.phase(), RuntimePhase::Stopped);
    // stop_movement 分支释放 guard 后 pending_goal 已被清空(release_guards),
    // 后续 begin 是新会话,无 stale pending。
    assert!(rt.pending_goal().is_none());
}
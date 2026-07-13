//! 集成测试:rework22 P1.10 momentum 在 publish goal 后更新 last_direction。
//!
//! 覆盖:
//! - `p1_10_momentum_updates_after_goal_publish`:publish 后 `last_direction()`
//!   等于 `(goal - robot).normalize()`。
//! - `p1_10_momentum_resets_on_begin`:begin 后 `last_direction()` 为 None
//!   (历史会话残留不应影响新一轮)。

#![cfg(feature = "runtime")]

use std::cell::{Cell, RefCell};

use nuwax_explore_wavefront::config::WavefrontConfig;
use nuwax_explore_wavefront::frontier_goal::StopReason;
use nuwax_explore_wavefront::runtime::{
    RuntimeInputs, RuntimePhase, RuntimeTickOutcome, WavefrontExplorerRuntime, WavefrontGoalSink,
};
use nuwax_explore_wavefront::selector::WavefrontGoalSelector;
use nuwax_explore_wavefront::session::Direction2;
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

/// P1.10 主测试:publish 后 last_direction = normalize(goal - robot)
#[test]
fn p1_10_momentum_updates_after_goal_publish() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    // 初始 last_direction 应为 None
    assert!(rt.last_direction().is_none());

    // 第一次 tick:应发布 goal
    let o1 = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    let goal_pos: Point2 = match o1 {
        RuntimeTickOutcome::Goal(g) => g.pose.pose.position.to_2d(),
        other => panic!("first tick should publish goal; got {other:?}"),
    };
    // publish 后,last_direction 应更新为 (goal - robot) 单位向量
    let last_dir = rt
        .last_direction()
        .expect("publish 后 last_direction 必须被更新");
    let expected = Direction2::from_delta(goal_pos.x - 1.0, goal_pos.y - 1.0)
        .expect("delta 必须非零且 finite");
    assert!(
        (last_dir.x - expected.x).abs() < 1e-9 && (last_dir.y - expected.y).abs() < 1e-9,
        "last_direction 应当是 (goal - robot) 单位向量; got ({}, {}), expected ({}, {})",
        last_dir.x,
        last_dir.y,
        expected.x,
        expected.y
    );
    // 长度必须 = 1
    let len = (last_dir.x * last_dir.x + last_dir.y * last_dir.y).sqrt();
    assert!(
        (len - 1.0).abs() < 1e-9,
        "last_direction 必须是单位向量; got len={}",
        len
    );
}

/// begin 后 last_direction 为 None(不应受上一会话残留影响)
#[test]
fn p1_10_momentum_resets_on_begin() {
    let mut rt = make_runtime();
    rt.begin().unwrap();
    let grid = frontier_grid();
    let odom = odom_at(1.0, 1.0);
    // 先跑一次让 last_direction 写上
    let _ = rt
        .tick(RuntimeInputs {
            costmap: Some(&grid),
            odom: Some(&odom),
            now: Timestamp::default(),
        })
        .unwrap();
    assert!(rt.last_direction().is_some());

    // end + begin 应当重置
    rt.end().unwrap();
    assert_eq!(rt.phase(), RuntimePhase::Stopped);
    rt.begin().unwrap();
    assert!(
        rt.last_direction().is_none(),
        "begin 后 last_direction 必须为 None,实测为 {:?}",
        rt.last_direction()
    );

    // 顺手清空以满足 stop reason 引用
    let _ = StopReason::UserCancelled;
}

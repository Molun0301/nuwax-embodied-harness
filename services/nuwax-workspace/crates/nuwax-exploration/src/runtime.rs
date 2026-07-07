//! Runtime adapter 最小 sink 合同(方案 §8.5)
//!
//! `runtime` feature 下 `nuwax-exploration` 提供:
//! - 三个 sink trait:`WavefrontInputReceiver` / `ExplorationGoalSink` / `StopMotionSink`
//! - `#[cfg(feature = "tare")]` 下 re-export `TareWaypoint` + `TareWaypointGoalMapping`
//! - `ExplorationRuntimeAdapter<C, G, S>` 包装 controller + goal sink + stop sink
//!
//! 跨 navigation 转换在 `crate::conversions`;native bridge 生命周期在
//! `nuwax-explore-tare-bridge::BridgeSupervisor`,facade 不直接 spawn native process。

use nuwax_nav_types_messages::geometry::PoseStamped;
use nuwax_nav_types_messages::goal::NavigationGoal;
use nuwax_nav_types_messages::grid::OccupancyGrid2D;
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::{ExplorationController, ExplorationStateSnapshot, ExploreError, ExploreResult};

/// 接收 wavefront 输入(costmap / odom)
pub trait WavefrontInputReceiver {
    /// 注入 costmap(用于 source adapter 缓存)
    fn ingest_costmap(&mut self, costmap: OccupancyGrid2D) -> ExploreResult<()>;
    /// 注入 odom pose
    fn ingest_odometry(&mut self, pose: PoseStamped) -> ExploreResult<()>;
}

/// 发布 exploration goal
pub trait ExplorationGoalSink {
    /// 发布 frontier goal(原始)
    fn publish_frontier_goal(&mut self, goal: FrontierGoal) -> ExploreResult<()>;
    /// 发布 navigation goal(`crate::conversions` 转换后)
    fn publish_navigation_goal(&mut self, goal: NavigationGoal) -> ExploreResult<()>;
}

/// 通知运动层在当前位姿停下
pub trait StopMotionSink {
    /// 在当前位姿停下
    fn stop_at_current_pose(&mut self, pose: PoseStamped) -> ExploreResult<()>;
}

#[cfg(feature = "tare")]
pub use nuwax_explore_tare_bridge::TareWaypoint;

/// TARE waypoint ingestion trait(方案 §8.5)
#[cfg(all(feature = "runtime", feature = "tare"))]
pub trait TareWaypointReceiver {
    /// 注入 TARE bridge 输出的 waypoint
    fn ingest_waypoint(&mut self, waypoint: TareWaypoint) -> ExploreResult<()>;
}

/// `TareWaypoint -> FrontierGoal` 字段映射配置(方案 §8.5)
///
/// 默认 score = 1.0,默认 producer = `nuwax-explore-tare-bridge/<env! version>`。
#[cfg(feature = "tare")]
#[derive(Debug, Clone)]
pub struct TareWaypointGoalMapping {
    /// frontier score(默认 1.0)
    pub score: f32,
    /// producer 字符串(默认 `nuwax-explore-tare-bridge/<version>`)
    pub producer: String,
}

#[cfg(feature = "tare")]
impl Default for TareWaypointGoalMapping {
    fn default() -> Self {
        Self {
            score: 1.0,
            producer: format!("nuwax-explore-tare-bridge/{}", env!("CARGO_PKG_VERSION")),
        }
    }
}

#[cfg(feature = "tare")]
impl TareWaypointGoalMapping {
    /// 构造自定义 mapping
    #[must_use]
    pub fn new(score: f32, producer: impl Into<String>) -> Self {
        Self {
            score,
            producer: producer.into(),
        }
    }

    /// 把 TARE waypoint 转为 `FrontierGoal`。
    ///
    /// 字段映射(方案 §8.5 固定):
    /// - `id = "tare:<frame>:<stamp.nanos>:<x>:<y>:<z>"` 的 deterministic stable hash
    ///   (使用 `std::hash::Hasher` + 固定种子,不是进程随机 `RandomState`)
    /// - `pose = PoseStamped { frame_id, stamp, position(x,y,z), orientation IDENTITY }`
    /// - `score = self.score`(默认 1.0)
    /// - `information_gain / distance_cost / frontier_area_m2 = None`
    /// - `producer = self.producer`
    /// - `metadata["source"] = "tare_waypoint"`
    /// - `TareWaypoint::validate()` 失败 -> `ExploreError::Conversion(...)`
    ///
    /// # Errors
    /// `TareWaypoint::validate()` 失败时返回 `ExploreError::Conversion`。
    pub fn waypoint_to_frontier_goal(
        &self,
        waypoint: &TareWaypoint,
    ) -> ExploreResult<FrontierGoal> {
        waypoint
            .validate()
            .map_err(|e| ExploreError::Conversion(format!("TareWaypoint invalid: {e}")))?;

        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        // 固定 id_seed = `tare:<frame>:<stamp.nanos>:<x>:<y>:<z>`(方案 v18 §6.1);
        // 同坐标 + 同 stamp 的同一 waypoint 在重启/不同进程下产生**不同** id,
        // 用于跨重启去重(若需 stable,可由 caller 在 metadata 注入 hash)。
        let id_seed = format!(
            "tare:{}:{}:{}:{}:{}",
            waypoint.frame_id.as_str(),
            waypoint.stamp.nanos,
            waypoint.x,
            waypoint.y,
            waypoint.z,
        );
        // deterministic SipHash(seed=0) → hex 截断;两次调用结果一致
        let mut h = DefaultHasher::new();
        0u8.hash(&mut h);
        id_seed.hash(&mut h);
        let id_hash = h.finish();
        let id = format!("tare-{id_hash:016x}");

        use nuwax_nav_types_messages::geometry::{Point3, Pose3, Quaternion};
        let pose = PoseStamped {
            frame_id: waypoint.frame_id.clone(),
            stamp: waypoint.stamp,
            pose: Pose3 {
                position: Point3 {
                    x: waypoint.x,
                    y: waypoint.y,
                    z: waypoint.z,
                },
                orientation: Quaternion::IDENTITY,
            },
        };
        let mut goal = FrontierGoal::new(id, pose, self.score);
        goal.information_gain = None;
        goal.distance_cost = None;
        goal.frontier_area_m2 = None;
        goal.producer = self.producer.clone();
        goal.stamp = waypoint.stamp;
        goal.metadata
            .insert("source".to_string(), "tare_waypoint".to_string());
        Ok(goal)
    }
}

/// Runtime adapter:包装 controller + goal sink + stop sink
///
/// 不直接持有 source adapter 句柄;`begin_exploration` / `poll_next_goal` /
/// `stop` 全部委托到内部 `ExplorationController`。Goal / Stop publish 走 sink
/// 回调。`runtime` feature 提供同步 controller;若上层需要 tokio 调度,在
/// 各自的 stack wiring 处做。
pub struct ExplorationRuntimeAdapter<C, G, S>
where
    C: AsRef<ExplorationController> + AsMut<ExplorationController>,
    G: ExplorationGoalSink,
    S: StopMotionSink,
{
    controller: C,
    goal_sink: G,
    stop_sink: S,
}

impl<C, G, S> ExplorationRuntimeAdapter<C, G, S>
where
    C: AsRef<ExplorationController> + AsMut<ExplorationController>,
    G: ExplorationGoalSink,
    S: StopMotionSink,
{
    /// 构造 adapter
    #[must_use]
    pub fn new(controller: C, goal_sink: G, stop_sink: S) -> Self {
        Self {
            controller,
            goal_sink,
            stop_sink,
        }
    }

    /// 获取 controller 引用
    #[must_use]
    pub fn controller(&self) -> &ExplorationController {
        self.controller.as_ref()
    }

    /// 获取 controller 可变引用
    pub fn controller_mut(&mut self) -> &mut ExplorationController {
        self.controller.as_mut()
    }

    /// 单次 tick:把 controller 输出转发到 goal sink / stop sink。
    ///
    /// - `Goal(goal)`:同时 publish `FrontierGoal` 与 `NavigationGoal`
    ///   (后者由 `crate::conversions::frontier_goal_to_navigation_goal` 转换)。
    /// - `Stop(reason)`:在 `ExplorationStateSnapshot::robot_pose` 位置调用 stop sink,
    ///   并调用 `controller.cancel()` 释放 active。
    /// - `Pending` / `SourceUnavailable`:不调用 sink,直接返回原始 `ExplorationPoll`。
    ///
    /// # Errors
    /// 内部 controller error 或 sink error 透传。
    pub fn tick(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<crate::ExplorationPoll> {
        let poll = self.controller.as_mut().poll_next_goal(state)?;
        match &poll {
            crate::ExplorationPoll::Goal(goal) => {
                self.goal_sink.publish_frontier_goal(goal.clone())?;
                let nav = crate::conversions::frontier_goal_to_navigation_goal(goal);
                self.goal_sink.publish_navigation_goal(nav)?;
            }
            crate::ExplorationPoll::Stop(_reason) => {
                self.stop_sink
                    .stop_at_current_pose(state.robot_pose.clone())?;
                self.controller.as_mut().cancel()?;
            }
            _ => {}
        }
        Ok(poll)
    }
}

#[cfg(feature = "tare")]
#[cfg(test)]
mod tare_mapping_tests {
    use super::*;
    use nuwax_nav_types_messages::frames::NavFrameId;
    use nuwax_nav_types_messages::geometry::Timestamp;

    fn sample_waypoint() -> TareWaypoint {
        TareWaypoint::new(
            NavFrameId::Map,
            Timestamp::from_unix(1_700_000_000, 0),
            1.0,
            2.0,
            0.0,
        )
    }

    #[test]
    fn mapping_default_fields() {
        let m = TareWaypointGoalMapping::default();
        assert!((m.score - 1.0).abs() < 1e-9);
        assert!(m.producer.starts_with("nuwax-explore-tare-bridge/"));
    }

    #[test]
    fn waypoint_to_goal_field_mapping() {
        let m = TareWaypointGoalMapping::default();
        let g = m.waypoint_to_frontier_goal(&sample_waypoint()).unwrap();
        assert!((g.pose.pose.position.x - 1.0).abs() < 1e-9);
        assert!((g.pose.pose.position.y - 2.0).abs() < 1e-9);
        assert!((g.pose.pose.position.z - 0.0).abs() < 1e-9);
        assert_eq!(g.pose.frame_id, NavFrameId::Map);
        assert_eq!(g.score, 1.0);
        assert!(g.information_gain.is_none());
        assert!(g.distance_cost.is_none());
        assert!(g.frontier_area_m2.is_none());
        assert!(g.producer.starts_with("nuwax-explore-tare-bridge/"));
        assert_eq!(
            g.metadata.get("source").map(String::as_str),
            Some("tare_waypoint")
        );
    }

    #[test]
    fn waypoint_to_goal_is_deterministic_id() {
        let m = TareWaypointGoalMapping::default();
        let wp = sample_waypoint();
        let g1 = m.waypoint_to_frontier_goal(&wp).unwrap();
        let g2 = m.waypoint_to_frontier_goal(&wp).unwrap();
        assert_eq!(g1.id, g2.id);
    }

    #[test]
    fn waypoint_to_goal_validation_error_returns_conversion() {
        let m = TareWaypointGoalMapping::default();
        let mut wp = sample_waypoint();
        wp.x = f64::NAN;
        let err = m.waypoint_to_frontier_goal(&wp).unwrap_err();
        assert!(matches!(err, ExploreError::Conversion(_)));
    }
}

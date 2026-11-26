//! 沿当前地图内已实测走过的路线脱离局部阻塞。
use crate::sampling_controller::{PoseGoal, Route, Sampler, PREDICTION_SECONDS};
use nuwax_nav_scan_native::{CollisionSnapshot, Config};
use nuwax_nav_types_messages::{Path3D, Point3, PoseStamped, Twist2};

/// 当前执行器用于恢复的同一碰撞快照与实际运动限制。
pub struct RecoverySurface {
    pub(crate) snapshot: CollisionSnapshot,
    pub(crate) forward: Config,
    pub(crate) reverse: Config,
}

#[derive(Default)]
pub(crate) struct WalkedPath {
    poses: Vec<PoseStamped>,
    phase: TracePhase,
}

#[derive(Default)]
enum TracePhase {
    #[default]
    Recording,
    Retreating(Point3),
    Replanning,
}

pub(crate) enum RetreatStep {
    Move(Twist2),
    DepartureAvailable,
    Exhausted,
}

impl WalkedPath {
    pub(crate) fn record(&mut self, pose: PoseStamped) {
        if matches!(self.phase, TracePhase::Recording)
            && self.poses.last().is_none_or(|last| last.pose.position != pose.pose.position)
        {
            self.poses.push(pose);
        }
    }

    pub(crate) fn begin(&mut self, pose: &PoseStamped, surface: &RecoverySurface) -> bool {
        self.record(pose.clone());
        if self.poses.len() < 2 || surface.reverse.max_reverse_velocity <= 0. {
            return false;
        }
        self.phase = TracePhase::Retreating(pose.pose.position);
        true
    }

    pub(crate) fn active(&self) -> bool {
        matches!(self.phase, TracePhase::Retreating(_))
    }

    pub(crate) fn end(&mut self) {
        self.phase = TracePhase::Recording;
    }

    pub(crate) fn replan(&mut self) {
        // 重规划和刹车期间继续保留倒退游标，不能把刚退过的轨迹重新接到末尾。
        self.phase = TracePhase::Replanning;
    }

    pub(crate) fn replan_advanced(
        &self,
        path: &Path3D,
        from: Point3,
        here: Point3,
        resolution: f64,
    ) -> bool {
        if !matches!(self.phase, TracePhase::Replanning) {
            return from.distance(here) >= resolution;
        }
        let route = Route::new(path.poses.iter().map(coordinates).collect());
        let remaining = |p: Point3| {
            let (arc, nearest) = route.nearest([p.x, p.y, p.z]);
            route.length() - arc + (p.x - nearest[0]).hypot(p.y - nearest[1])
        };
        remaining(from) - remaining(here) >= resolution
    }

    pub(crate) fn clear(&mut self) {
        self.poses.clear();
        self.end();
    }

    pub(crate) fn step(
        &mut self,
        pose: &PoseStamped,
        surface: &RecoverySurface,
        requested: &Path3D,
        applied: Twist2,
    ) -> RetreatStep {
        let here = pose.pose.position;
        let xyz = coordinates(pose);
        let yaw = pose.pose.orientation.yaw();
        let TracePhase::Retreating(blocked) = self.phase else {
            unreachable!("倒退控制只在实测轨迹的倒退阶段执行");
        };
        // 在碰撞图仍分辨不出位移时，不能把受阻位姿已有的动作当作新的出发机会。
        if (here.x - blocked.x).hypot(here.y - blocked.y) >= surface.forward.resolution {
            let reference = Route::new(requested.poses.iter().map(coordinates).collect());
            if Sampler::default()
                .select_departure(&surface.snapshot, &surface.forward, &reference, xyz, yaw, Twist2::ZERO, None, false)
                .is_some()
            {
                return RetreatStep::DepartureAvailable;
            }
        }

        // 用实测位姿移去已经退过的轨迹点，不按下发指令或固定步数推进。
        while let [.., older, newer] = self.poses.as_slice() {
            if here.distance(older.pose.position) > here.distance(newer.pose.position) {
                break;
            }
            self.poses.pop();
        }
        if self.poses.len() < 2 {
            return RetreatStep::Exhausted;
        }
        let mut route = vec![xyz];
        let mut heading = yaw;
        let lookahead = PREDICTION_SECONDS * surface.reverse.max_reverse_velocity;
        for past in self.poses.iter().rev().skip(1) {
            route.push(coordinates(past));
            heading = past.pose.orientation.yaw();
            // 步态摆动的累计弧长不能占满前视，否则会一直追着窄处的摆动选点。
            if here.distance(past.pose.position) >= lookahead {
                break;
            }
        }
        let target = *route.last().unwrap();
        let reference = Route::new(route);
        let mut reverse = surface.reverse;
        reverse.max_velocity = 0.;
        let selection = Sampler::default().select_retreat(
            &surface.snapshot, &reverse, &reference, xyz, yaw, applied,
            PoseGoal {
                position: target,
                yaw: heading,
                position_tolerance_m: 0.,
                position_reached: true,
                approach_surface: None,
            },
        );
        match selection {
            Some(selection) if selection.command != Twist2::ZERO => RetreatStep::Move(selection.command),
            _ => RetreatStep::Exhausted,
        }
    }
}

fn coordinates(pose: &PoseStamped) -> [f64; 3] {
    let point = pose.pose.position;
    [point.x, point.y, point.z]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{path_for, session_for, FakeMotion, FakeMotionState};
    use crate::{
        CostmapObservation, ExecutionMap, ExecutionPath, InputCommand, InputKind, NavPlanner,
        NavScheduler, PlannerStep, RuntimeError, SchedulerConfig, SchedulerPhase, TerminalOutcome,
    };
    use nuwax_nav_astar::navigation_map::CostMap2D;
    use nuwax_nav_scan_native::MapOwner;
    use nuwax_nav_types_messages::{NavFrameId, Path2D, Pose3, Quaternion, Timestamp};
    use std::sync::{Arc, Mutex};
    use tokio::sync::{oneshot, watch};
    use tokio::time::Instant;

    struct BlockedPlanner(RecoverySurface);

    impl NavPlanner for BlockedPlanner {
        fn replace_path(&mut self, _: Path2D) {}
        fn replace_execution_path(
            &mut self, _: ExecutionPath, _: Option<Vec<f64>>, _: PoseStamped,
        ) -> Result<(), RuntimeError> {
            Ok(())
        }
        fn update_odometry(&mut self, _: PoseStamped) {}
        fn update_costmap(&mut self, _: CostMap2D) {}
        fn step(&mut self) -> PlannerStep {
            PlannerStep::Blocked("局部没有可行动作".into())
        }
        fn stop(&mut self) {}
        fn recovery_surface(&mut self) -> Option<RecoverySurface> {
            Some(RecoverySurface {
                snapshot: self.0.snapshot.clone(),
                forward: self.0.forward,
                reverse: self.0.reverse,
            })
        }
    }

    fn pose(x: f64) -> PoseStamped {
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1, 0),
            pose: Pose3 {
                position: Point3::new(x, 0., 0.4),
                orientation: Quaternion::IDENTITY,
            },
        }
    }

    fn surface(obstacles: &[[f64; 3]]) -> RecoverySurface {
        let mut forward = crate::SurfaceFollowerConfig::new(0.4, 0.15).scan;
        forward.resolution = 0.05;
        forward.max_acceleration = 1.;
        forward.body_length = 0.40;
        forward.max_yaw_rate = 0.;
        let mut reverse = forward;
        reverse.max_reverse_velocity = 0.2;
        let mut map = MapOwner::new(&forward).unwrap();
        map.seed_occupied(obstacles, [0., 0., 0.4]);
        map.update_surface(&[], &[], &[], &[]).unwrap();
        RecoverySurface { snapshot: map.snapshot(), forward, reverse }
    }

    #[test]
    fn retraces_observed_motion_then_yields_when_departure_becomes_possible() {
        let mut walked = WalkedPath::default();
        let front = surface(&[[0.20, 0.01, 0.425]]);
        let requested = Path3D { poses: vec![pose(0.), pose(3.)] };
        assert!(!walked.begin(&pose(0.), &front), "只有一个实测点，不能倒退");
        walked.clear();
        for index in -20..=0 {
            walked.record(pose(f64::from(index) * 0.02));
        }
        assert!(walked.begin(&pose(0.), &front));
        let mut current = pose(0.);
        let mut applied = Twist2::ZERO;
        loop {
            match walked.step(&current, &front, &requested, applied) {
                RetreatStep::Move(command) => {
                    assert!((-0.2..0.).contains(&command.linear.x));
                    assert_eq!(command.linear.y, 0.);
                    assert_eq!(command.angular_z, 0.);
                    applied = command;
                    current.pose.position.x += command.linear.x * front.reverse.sample_dt;
                    assert!(current.pose.position.x > -0.2, "可出发后应交回规划");
                }
                RetreatStep::DepartureAvailable => {
                    assert!(current.pose.position.x < 0.);
                    break;
                }
                RetreatStep::Exhausted => panic!("原实测轨迹还有安全的倒退空间"),
            }
        }
    }

    #[test]
    fn an_obstacle_in_the_recorded_trail_still_blocks_retreat() {
        let mut walked = WalkedPath::default();
        let occupied = surface(&[[-0.20, 0.01, 0.425], [0.20, 0.01, 0.425]]);
        walked.record(pose(-0.4));
        assert!(walked.begin(&pose(0.), &occupied));
        let requested = Path3D { poses: vec![pose(0.), pose(3.)] };
        assert!(!matches!(walked.step(&pose(0.), &occupied, &requested, Twist2::ZERO),
                         RetreatStep::Move(_)));
    }

    #[test]
    fn recent_pose_oscillation_does_not_hide_the_earlier_exit() {
        let mut walked = WalkedPath::default();
        let front = surface(&[[0.20, 0.01, 0.425]]);
        for index in -25..=-1 {
            walked.record(pose(f64::from(index) * 0.02));
        }
        // 小范围反复摆动的累计路程远超当拍的物理前视范围。
        for _ in 0..30 {
            walked.record(pose(-0.01));
            walked.record(pose(0.01));
        }
        assert!(walked.begin(&pose(0.), &front));
        let requested = Path3D { poses: vec![pose(0.), pose(3.)] };
        let RetreatStep::Move(command) = walked.step(&pose(0.), &front, &requested, Twist2::ZERO) else {
            panic!("仍有实测退出路线，需要开始倒退");
        };
        assert!(command.linear.x < -0.03, "先前走入段仍应指导倒退 {command:?}");
    }

    #[test]
    fn retreat_uses_a_safe_motion_even_if_waiting_scores_better() {
        let open = surface(&[]);
        let mut config = open.reverse;
        config.max_velocity = 0.;
        let origin = coordinates(&pose(0.));
        let target = coordinates(&pose(-0.001));
        let route = Route::new(vec![origin, target]);
        let goal = || PoseGoal {
            position: target,
            yaw: 0.,
            position_tolerance_m: 0.,
            position_reached: true,
            approach_surface: None,
        };
        let ordinary = Sampler::default()
            .select(&open.snapshot, &config, &route, origin, 0., Twist2::ZERO, Some(goal()))
            .unwrap();
        assert_eq!(ordinary.command, Twist2::ZERO);
        assert!(!ordinary.only_stop);
        let retreat = Sampler::default()
            .select_retreat(&open.snapshot, &config, &route, origin, 0., Twist2::ZERO, goal())
            .unwrap();
        assert!(retreat.command.linear.x < 0.);
    }

    #[test]
    fn departure_after_retreat_can_take_a_direction_other_than_the_rejected_route() {
        let mut walked = WalkedPath::default();
        let open = surface(&[]);
        walked.record(pose(-0.4));
        assert!(walked.begin(&pose(0.), &open));
        let rejected = Path3D { poses: vec![pose(0.), pose(-3.)] };
        assert!(matches!(walked.step(&pose(-0.001), &open, &rejected, Twist2::ZERO),
                        RetreatStep::Move(_)));
        assert!(matches!(walked.step(&pose(-0.10), &open, &rejected, Twist2::ZERO),
                        RetreatStep::DepartureAvailable));
    }

    #[test]
    fn replan_preserves_the_retreat_cursor_until_the_new_route_advances() {
        let mut walked = WalkedPath::default();
        let open = surface(&[]);
        for x in [-0.6, -0.4, -0.2, 0.] {
            walked.record(pose(x));
        }
        assert!(walked.begin(&pose(0.), &open));
        walked.replan();
        walked.record(pose(-0.08));
        walked.record(pose(-0.12));
        assert_eq!(walked.poses.len(), 4);

        let revised = Path3D { poses: vec![pose(-0.08), pose(3.)] };
        assert!(!walked.replan_advanced(&revised, pose(-0.08).pose.position,
                                        pose(-0.2).pose.position, open.forward.resolution));
        assert!(walked.replan_advanced(&revised, pose(-0.08).pose.position,
                                       pose(0.).pose.position, open.forward.resolution));
        assert!(walked.begin(&pose(-0.12), &open));
        assert_eq!(walked.poses.len(), 4);
    }

    #[test]
    fn a_stationary_candidate_does_not_hide_that_every_motion_is_blocked() {
        let front = surface(&[[0.20, 0.01, 0.425]]);
        let origin = coordinates(&pose(0.));
        let route = Route::new(vec![origin, coordinates(&pose(3.))]);
        let selection = Sampler::default()
            .select(&front.snapshot, &front.forward, &route, origin, 0., Twist2::ZERO, None)
            .expect("原位停止仍可行");
        assert_eq!(selection.command, Twist2::ZERO);
        assert!(selection.only_stop);
    }

    #[tokio::test]
    async fn scheduler_retries_global_planning_before_retreat_and_honors_cancel() {
        let motion = FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
        let mut scheduler = NavScheduler::new(
            motion, BlockedPlanner(surface(&[[0.20, 0.01, 0.425]])), SchedulerConfig::default(),
        ).unwrap();
        let (status, _) = watch::channel(scheduler.status.clone());
        scheduler.observe_costmap(&status, CostmapObservation {
            costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 80, 80, 0.05)),
            source_timestamp_ns: 1,
        }, Instant::now()).unwrap();
        scheduler.observe_odometry(&status, pose(-0.3), None).unwrap();
        scheduler.session_tick(&status).await.unwrap();
        for x in [-0.2, -0.1, 0.] {
            scheduler.observe_odometry(&status, pose(x), None).unwrap();
        }
        let session = session_for("retreat");
        let goal_key = session.execution_identity.goal_key();
        scheduler.start_session(&status, session).await.unwrap();
        let mut path = path_for("retreat", 1);
        path.path = ExecutionPath::Surface(Path3D { poses: vec![pose(0.), pose(3.)] });
        scheduler.submit_path(&status, path).await.unwrap();
        scheduler.session_tick(&status).await.unwrap();
        assert_eq!(scheduler.status.phase, SchedulerPhase::AwaitingPath);
        assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
        assert!(!scheduler.walked_path.active());

        let (reply, result) = oneshot::channel();
        scheduler.handle_input(&status, InputCommand {
            kind: InputKind::PlanningDiagnostic(
                "retreat".into(), goal_key, "没有可行全局路线".into(), false, true,
            ),
            reply,
        }).await;
        result.await.unwrap().unwrap();
        assert!(scheduler.walked_path.active());
        scheduler.session_tick(&status).await.unwrap();
        assert!(scheduler.status.last_applied_twist.unwrap().linear.x < 0.);
        assert!(!scheduler.status.no_feasible_path);
        scheduler.cancel(&status, "retreat").await.unwrap();
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Cancelled));
        assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
        assert!(!scheduler.walked_path.active());
    }
}

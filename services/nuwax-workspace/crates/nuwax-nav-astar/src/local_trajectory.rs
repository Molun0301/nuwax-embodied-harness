//! 局部控制的完整机体与制动尾段检查，不复制整张地图。
use super::*;
use nuwax_nav_control::{choose_control, PlanarPose};
use nuwax_nav_types_messages::RobotGeometry;
use std::collections::BTreeSet;

impl LocalPlanner {
    pub(super) fn choose_bounded_control(
        &self,
        preferred: Twist2,
        robot_width: f64,
    ) -> Option<Twist2> {
        let limits = self.motion_limits.as_ref()?;
        let odom = self.current_odom.as_ref()?;
        let grid = self.costmap.as_ref()?;
        let path = self.path_distancer.as_ref()?;
        let (x, y, yaw) = pose_xy_yaw(odom);
        let position = Point2::new(x, y);
        let target = if self.oriented_path {
            self.tracking_target?.0
        } else if matches!(
            self.state,
            LocalPlannerState::InitialRotation | LocalPlannerState::FinalRotation
        ) {
            position
        } else {
            path.find_lookahead_from_position(position)
        };
        let target_yaw = if let Some(yaw) = self.rotation_target_yaw() {
            yaw
        } else if self.oriented_path {
            self.tracking_target?.1
        } else {
            match self.state {
                LocalPlannerState::InitialRotation => {
                    self.first_segment_yaw_or_first_yaw().unwrap_or(yaw)
                }
                LocalPlannerState::FinalRotation => {
                    self.last_segment_yaw_or_last_yaw().unwrap_or(yaw)
                }
                _ => (target.y - y).atan2(target.x - x),
            }
        };
        let geometry = if let Some(geometry) = &self.robot_geometry {
            geometry.clone()
        } else {
            let radius = robot_width * 0.5 / std::f64::consts::FRAC_PI_8.cos();
            if !radius.is_finite() || radius <= 0.0 {
                return None;
            }
            RobotGeometry {
                self_collision: None,
                footprint_xy_m: (0..8)
                    .map(|i| {
                        let angle = i as f64 * std::f64::consts::FRAC_PI_4;
                        [radius * angle.cos(), radius * angle.sin()]
                    })
                    .collect(),
                z_min_m: -0.5,
                z_max_m: 0.5,
                source: "算法圆形外形".into(),
            }
        };
        let footprint = geometry
            .grid_footprint(
                grid.resolution(),
                yaw,
                u32::try_from(grid.width()).ok()?,
                u32::try_from(grid.height()).ok()?,
            )
            .ok()?;
        let first = grid.cell_center_world(0, 0);
        let origin = Point2::new(
            first.x - grid.resolution() * 0.5,
            first.y - grid.resolution() * 0.5,
        );
        let (cx, cy) = grid.world_to_cell(position);
        if cx < 0 || cy < 0 || cx as usize >= grid.width() || cy as usize >= grid.height() {
            return None;
        }
        let occupied_by_body: BTreeSet<_> = footprint
            .at_position(origin, position)
            .ok()?
            .offsets
            .iter()
            .map(|&(dx, dy)| (i64::from(cx) + dx, i64::from(cy) + dy))
            .collect();
        let radius = geometry
            .footprint_xy_m
            .iter()
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f64::max);
        // 保留已观察到的惯性运动，不能把刚发出的零速度当作机器人已经静止。
        let bound = |command: f64, observed: f64| {
            if observed.abs() > command.abs() {
                observed
            } else {
                command
            }
        };
        let current = Twist2 {
            linear: Point2::new(
                bound(self.last_command.linear.x, self.observed_velocity.linear.x),
                bound(self.last_command.linear.y, self.observed_velocity.linear.y),
            ),
            angular_z: bound(
                self.last_command.angular_z,
                self.observed_velocity.angular_z,
            ),
        };
        choose_control(
            PlanarPose { x, y, yaw },
            current,
            preferred,
            PlanarPose {
                x: target.x,
                y: target.y,
                yaw: target_yaw,
            },
            limits,
            1.0 / self.config.control_frequency,
            grid.resolution() * 0.5,
            radius,
            self.allow_reverse,
            |from, to, _| {
                footprint.visit_motion(
                    origin,
                    (Point2::new(from.x, from.y), from.yaw),
                    (Point2::new(to.x, to.y), to.yaw),
                    |x, y| {
                        (x as usize) < grid.width()
                            && (y as usize) < grid.height()
                            && (occupied_by_body.contains(&(i64::from(x), i64::from(y)))
                                || grid.get(x as usize, y as usize) < 50)
                    },
                )
            },
        )
        .map(|choice| choice.command)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::{make_path2d, make_pose_stamped};
    use nuwax_nav_types_messages::{MotionLimits, MotionModel, Timestamp};
    fn limits() -> MotionLimits {
        MotionLimits {
            model: MotionModel::Differential,
            max_forward_mps: 0.55,
            max_reverse_mps: 0.25,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.8,
            linear_acceleration_mps2: 0.5,
            guaranteed_braking_mps2: 0.5,
            angular_acceleration_rps2: 1.5,
            guaranteed_angular_braking_rps2: 1.5,
            reaction_time_sec: 0.2,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        }
    }
    fn geometry() -> RobotGeometry {
        RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.2,-0.1],[0.2,-0.1],[0.2,0.1],[-0.2,0.1]],"z_min_m":-0.2,"z_max_m":0.2}"#).unwrap()
    }
    #[test]
    fn calibrated_controller_reaches_turning_path_without_exceeding_acceleration() {
        let mut planner = LocalPlanner::new(0.5, 20.0, 0.2);
        planner.set_robot_geometry(geometry());
        planner.set_motion_limits(limits()).unwrap();
        planner.start_planning(make_path2d(vec![
            Point2::new(1.0, 1.0),
            Point2::new(2.5, 1.0),
            Point2::new(2.5, 3.0),
            Point2::new(4.0, 3.0),
        ]));
        let grid = CostMap2D::new("map", 60, 60, 0.1);
        planner.update_costmap(grid.clone());
        let mut pose = PlanarPose {
            x: 1.0,
            y: 1.0,
            yaw: 0.0,
        };
        let mut previous = Twist2::ZERO;
        for tick in 0..1600 {
            let mut odom = make_pose_stamped("map", pose.x, pose.y, pose.yaw);
            odom.stamp = Timestamp::from_unix(1 + tick / 20, (tick % 20 * 50_000_000) as u32);
            planner.handle_odom(odom);
            let (command, stop) = planner.step(0.4);
            if stop == Some(StopMessage::Arrived) {
                assert!((pose.x - 4.0).hypot(pose.y - 3.0) < 0.2);
                return;
            }
            assert!(stop.is_none(), "tick={tick} pose={pose:?} stop={stop:?}");
            assert!(command.linear.x.abs() - previous.linear.x.abs() <= 0.5 * 0.05 + 1e-6);
            assert!(command.angular_z.abs() - previous.angular_z.abs() <= 1.5 * 0.05 + 1e-6);
            pose = nuwax_nav_control::integrate(pose, command, 0.05);
            assert!(pose.x > 0.2 && pose.x < 5.8 && pose.y > 0.2 && pose.y < 5.8);
            previous = command;
        }
        panic!("有标定控制器未在合成期限内到达");
    }
    #[test]
    fn unobserved_stopping_tail_does_not_allow_motion_into_unknown() {
        let mut planner = LocalPlanner::new(0.5, 20.0, 0.1);
        planner.set_robot_geometry(geometry());
        planner.set_motion_limits(limits()).unwrap();
        planner.start_planning(make_path2d(vec![
            Point2::new(1.0, 1.0),
            Point2::new(3.0, 1.0),
        ]));
        let mut grid = CostMap2D::new("map", 40, 30, 0.1);
        for y in 0..30 {
            for x in 14..40 {
                grid.set_unknown(x, y);
            }
        }
        planner.update_costmap(grid);
        planner.handle_odom(make_pose_stamped("map", 1.0, 1.0, 0.0));
        planner.last_command = Twist2 {
            linear: Point2::new(0.5, 0.0),
            angular_z: 0.0,
        };
        planner.braking_latch = false;
        let (command, stop) = planner.step(0.4);
        assert_eq!(command, Twist2::ZERO);
        assert_eq!(stop, Some(StopMessage::ObstacleFound));
    }
}

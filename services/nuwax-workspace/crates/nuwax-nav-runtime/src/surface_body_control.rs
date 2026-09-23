//! 完整三维机体路径跟随与共享制动检查，仍由原调度器唯一写入运动守卫。
use super::*;
use nuwax_nav_control::{choose_control, PlanarPose};
use nuwax_nav_types_messages::MotionModel;
fn body(p: &PoseStamped) -> ([f64; 3], [f64; 4]) {
    let q = p.pose.orientation;
    (xyz(p.pose.position), [q.x, q.y, q.z, q.w])
}
fn planar(p: &PoseStamped) -> PlanarPose {
    PlanarPose {
        x: p.pose.position.x,
        y: p.pose.position.y,
        yaw: p.pose.orientation.yaw(),
    }
}
impl SurfaceFollower {
    pub(super) fn begin_braking(&mut self) {
        // 地图位姿包含定位校正，不能差分成真实速度或另一份静止证明。
        // 仅在实际生成过非零控制时保留标定制动上界；重复零速不延长等待。
        if self.last_command != Twist2::ZERO {
            if let Some(limits) = &self.config.motion_limits {
                self.braking_deadline = Some(std::time::Instant::now()
                    + std::time::Duration::from_secs_f64(limits.maximum_stopping_time_sec()));
            }
        }
        self.last_command = Twist2::ZERO;
    }
    pub(super) fn follow_body_step(&mut self, robot_width: f64) -> PlannerStep {
        if let Some(error) = &self.invalid_input {
            return PlannerStep::Failed(error.clone());
        }
        if let Some(deadline) = self.braking_deadline {
            if std::time::Instant::now() < deadline {
                return PlannerStep::Command(Twist2::ZERO);
            }
            self.braking_deadline = None;
        }
        let (Some(map), Some(pose), Some(path), Some(goal)) = (
            self.map.as_ref(),
            self.odometry.as_ref(),
            self.path.as_ref(),
            self.goal.as_ref(),
        ) else {
            return PlannerStep::ReplanRequired("机体执行输入缺失".into());
        };
        if !robot_width.is_finite() || robot_width <= 0.0 {
            return PlannerStep::Failed("机器人宽度无效".into());
        }
        if let Err(e) = map.validate_body_pose(body(pose).0, body(pose).1) {
            return PlannerStep::ReplanRequired(format!("当前三维机体包络无效: {e}"));
        }
        let position = pose.pose.position;
        let yaw = pose.pose.orientation.yaw();
        let actual_support = map.locate_support_from_body(
            body(pose).0, body(pose).1, self.config.body_to_support_z_m,
        ).ok();
        let goal_support = map.locate_goal_support_from_body(
            body(goal).0, body(goal).1, self.config.body_to_support_z_m,
        ).ok();
        let same_goal_surface = actual_support.zip(goal_support).is_some_and(|(a,b)|
            (a.surface_xyz[2]-b.surface_xyz[2]).abs() <= f64::from(map.source_voxel_size_m())+1e-6);
        let final_distance = if same_goal_surface {
            (position.x-goal.pose.position.x).hypot(position.y-goal.pose.position.y)
        } else { position.distance(goal.pose.position) };
        let final_yaw = angle_diff(goal.pose.orientation.yaw(), yaw);
        if final_distance < self.config.controller.goal_tolerance
            && (!self.options.requires_orientation() || final_yaw.abs() < self.config.controller.orientation_tolerance)
        {
            // 这里只确认到达规划终点；调度器随后撤租归零，等待守卫的物理
            // 停稳证明后才发布成功，避免地图噪声形成第二道停车门槛。
            return PlannerStep::Succeeded;
        }
        // 逐点保留转向停留；曲线点只在到达或沿已确认小段越过时消耗。
        let tolerance = (f64::from(map.config().voxel_size) * 0.7).min(0.07);
        while self.path_index < path.poses.len() {
            let target = &path.poses[self.path_index];
            let previous = &path.poses[self.path_index - 1];
            let rotation = target.pose.position.distance(previous.pose.position) < 1e-5;
            let error = angle_diff(target.pose.orientation.yaw(), yaw);
            let same_support = actual_support.zip(map.locate_support_from_body(
                body(target).0, body(target).1, self.config.body_to_support_z_m,
            ).ok()).is_some_and(|(a,b)| {
                (a.surface_xyz[2]-b.surface_xyz[2]).abs()
                    <= f64::from(map.source_voxel_size_m())+1e-6
            });
            // 地面厚层允许实际足端参考位于层内。已通过完整机体检查且关联
            // 同一高度层时，按水平进度消耗路径点；平面速度无法补偿层顶偏差。
            // 最终到达仍使用上方完整三维距离，不跨楼层或扩大用户容差。
            let distance = if same_support {
                (position.x-target.pose.position.x).hypot(position.y-target.pose.position.y)
            } else {position.distance(target.pose.position)};
            let passed = if rotation {
                false
            } else {
                let a = previous.pose.position;
                let b = target.pose.position;
                let dot = (position.x - b.x) * (b.x - a.x) + (position.y - b.y) * (b.y - a.y);
                dot >= 0.0 && distance < tolerance * 2.0
            };
            let goal_position = (target.pose.position.x-goal.pose.position.x)
                .hypot(target.pose.position.y-goal.pose.position.y) < 1e-5;
            let reached_position = !goal_position || final_distance < self.config.controller.goal_tolerance;
            if reached_position && (distance < tolerance && (!rotation || error.abs() < 0.05) || passed) {
                self.progress_m = self.cumulative[self.path_index];
                self.path_index += 1;
            } else {
                break;
            }
        }
        if self.path_index >= path.poses.len() {
            if final_distance >= self.config.controller.goal_tolerance {
                return PlannerStep::ReplanRequired("三维安全路径结束，原目标未到达".into());
            }
            self.control_target = Some(PlanarPose {
                x: position.x,
                y: position.y,
                yaw: goal.pose.orientation.yaw(),
            });
            return self.bounded_command(Twist2 {
                linear: Point2::new(0.0, 0.0),
                angular_z: final_yaw.clamp(-0.5, 0.5),
            });
        }
        let target = &path.poses[self.path_index];
        let previous = &path.poses[self.path_index - 1];
        if position
            .distance(previous.pose.position)
            .min(position.distance(target.pose.position))
            > self.config.max_path_deviation_m
        {
            return PlannerStep::ReplanRequired("机体偏离连续三维路线".into());
        }
        let rotation = target.pose.position.distance(previous.pose.position) < 1e-5;
        let mut target_index = self.path_index;
        if rotation {
            // 同方向细分转向共用前瞻，避免每个离散角度都减速；不跨越反转或平移。
            let direction = angle_diff(target.pose.orientation.yaw(), previous.pose.orientation.yaw());
            while target_index + 1 < path.poses.len() {
                let a = &path.poses[target_index];
                let b = &path.poses[target_index + 1];
                let step = angle_diff(b.pose.orientation.yaw(), a.pose.orientation.yaw());
                let ahead = angle_diff(b.pose.orientation.yaw(), yaw);
                if b.pose.position.distance(previous.pose.position) >= 1e-5
                    || step * direction <= 0.0
                    || ahead * direction <= 0.0
                    || ahead.abs() > std::f64::consts::FRAC_PI_2
                {
                    break;
                }
                target_index += 1;
            }
        } else {
            while target_index + 1 < path.poses.len() {
                let a = &path.poses[target_index];
                let b = &path.poses[target_index + 1];
                if a.pose.position.distance(b.pose.position) < 1e-5
                    || angle_diff(b.pose.orientation.yaw(), target.pose.orientation.yaw()).abs()
                        > 0.25
                    || position.distance(b.pose.position) > self.config.lookahead_m
                {
                    break;
                }
                target_index += 1;
            }
        }
        let target = &path.poses[target_index];
        let heading_error = angle_diff(target.pose.orientation.yaw(), yaw);
        self.control_target = Some(planar(target));
        let mut preferred = Twist2 {
            linear: Point2::new(0.0, 0.0),
            angular_z: heading_error.clamp(-0.6, 0.6),
        };
        if !rotation {
            let dx = target.pose.position.x - position.x;
            let dy = target.pose.position.y - position.y;
            let forward = dx * yaw.cos() + dy * yaw.sin();
            let lateral = -dx * yaw.sin() + dy * yaw.cos();
            let speed = self.config.controller.speed.min(dx.hypot(dy) * 1.5);
            let omni = self
                .config
                .motion_limits
                .as_ref()
                .is_some_and(|m| m.model == MotionModel::Omnidirectional);
            let reverse = self.options.allow_reverse && forward < 0.0;
            let direction = if reverse { -1.0 } else { 1.0 };
            let travel_yaw = (dy).atan2(dx) + if reverse { std::f64::consts::PI } else { 0.0 };
            let error = angle_diff(travel_yaw, yaw);
            let rotate_allowed = self
                .config
                .motion_limits
                .as_ref()
                .is_none_or(|m| m.can_rotate_in_place);
            if omni {
                preferred.linear = Point2::new(forward, lateral);
                let n = preferred.linear.x.hypot(preferred.linear.y);
                if n > speed {
                    preferred.linear = Point2::new(forward * speed / n, lateral * speed / n);
                }
            } else if error.abs() < 0.6 || !rotate_allowed {
                preferred.linear.x = direction * speed;
                preferred.angular_z = (2.0 * error).clamp(-0.6, 0.6);
            } else {
                preferred.angular_z = error.clamp(-0.6, 0.6);
            }
        }
        self.bounded_command(preferred)
    }
    pub(super) fn bounded_body_command(&mut self, preferred: Twist2) -> PlannerStep {
        let (Some(map), Some(pose), Some(limits)) = (
            self.map.as_ref(),
            self.odometry.as_ref(),
            self.config.motion_limits.as_ref(),
        ) else {
            return PlannerStep::ReplanRequired("缺少机体控制标定".into());
        };
        let Some(model) = map.body_envelope() else {
            return PlannerStep::ReplanRequired("缺少机体包络".into());
        };
        let target = self.control_target.unwrap_or_else(|| planar(pose));
        // 用已下发控制及标定动态界预测制动扫掠；map位姿仍用于实际
        // 位置、障碍和偏离检查，其坐标校正不充当执行器速度。
        let current = self.last_command;
        let Ok(initial) =
            map.locate_support_from_body(body(pose).0, body(pose).1, model.body_to_support_z_m).map(|s|s.support_xyz)
        else {
            return PlannerStep::ReplanRequired("机体支撑缺失".into());
        };
        let mut previous_support = initial;
        let mut previous_xy = [pose.pose.position.x, pose.pose.position.y];
        let mut previous_yaw=pose.pose.orientation.yaw();
        let observation_check = if let Some(age) = self.config.max_observation_age_sec {
            let Ok(occupied) = map.body_observation_footprint(body(pose)) else {
                return PlannerStep::ReplanRequired("三维近场实际观察缺失".into());
            };
            let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos().min(u128::from(u64::MAX)) as u64).unwrap_or(0);
            Some((occupied, now, (age * 1e9) as u64))
        } else { None };
        let mut last_failure = None;
        let result = choose_control(
            planar(pose),
            current,
            preferred,
            target,
            limits,
            1.0 / self.config.controller.control_frequency,
            f64::from(map.source_voxel_size_m()) * 0.5,
            model.radius(),
            self.options.allow_reverse,
            |from, to, elapsed| {
                // 每个速度候选及其独立紧急制动轨迹都从实际支撑重置。
                if (from.x - previous_xy[0]).hypot(from.y - previous_xy[1]) > 1e-5 || angle_diff(from.yaw,previous_yaw).abs()>1e-5 {
                    previous_support = initial;
                }
                let Ok(a) = map.body_pose_on_surface([from.x, from.y], from.yaw, previous_support)
                else {
                    return false;
                };
                let Ok(support) =
                    map.support_position_from_body(a.0, a.1, model.body_to_support_z_m)
                else {
                    return false;
                };
                let Ok(b) = map.body_pose_on_surface([to.x, to.y], to.yaw, support) else {
                    return false;
                };
                let valid = match &observation_check {
                    Some((occupied, now, age)) => map.validate_observed_body_motion(a, b, occupied,
                        now.saturating_add((elapsed * 1e9) as u64), *age),
                    None => map.validate_body_motion(a, b, &[]),
                };
                if let Err(error) = valid {
                    last_failure = Some(error);
                    return false;
                }
                previous_support = map
                    .support_position_from_body(b.0, b.1, model.body_to_support_z_m)
                    .unwrap_or(support);
                previous_xy = [to.x, to.y];
                previous_yaw=to.yaw;
                true
            },
        );
        match result {
            Some(choice) if choice.command.linear.x.hypot(choice.command.linear.y) > 1e-9
                || choice.command.angular_z.abs() > 1e-9 => {
                self.last_command = choice.command;
                PlannerStep::Command(choice.command)
            }
            _ => {
                self.begin_braking();
                PlannerStep::ReplanRequired(format!("三维制动扫掠受阻，停车后重新接续{}",
                    last_failure.map(|error|format!(": {error}")).unwrap_or_default()))
            }
        }
    }
}

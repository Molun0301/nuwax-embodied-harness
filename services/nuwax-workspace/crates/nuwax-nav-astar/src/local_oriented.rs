//! 联合朝向路径的连续进度和旋转停留点，防止跳过回环或改变转向方向。
use super::*;
use nuwax_nav_types_messages::MotionModel;
impl LocalPlanner {
    pub(super) fn initialize_oriented_path(&mut self) {
        let (Some(path), Some(yaws)) = (&self.path, &self.path_yaws) else {
            return;
        };
        self.path_cumulative = vec![0.0; path.points.len()];
        for i in 1..path.points.len() {
            let a = path.points[i - 1];
            let b = path.points[i];
            let length = a.distance(b);
            self.path_cumulative[i] = self.path_cumulative[i - 1] + length;
            if length < 1e-9 && angle_diff(yaws[i], yaws[i - 1]).abs() > 1e-6 {
                self.rotation_stops.push((i, b, yaws[i]));
                self.oriented_path = true;
            } else if length > 1e-9
                && angle_diff((b.y - a.y).atan2(b.x - a.x), yaws[i - 1]).abs() > 1e-3
            {
                self.oriented_path = true;
            }
        }
        self.progress_credit = self
            .current_odom
            .as_ref()
            .zip(path.points.first())
            .map_or(0.0, |(pose, first)| pose_to_point2(pose).distance(*first));
    }
    pub(super) fn rotation_target_yaw(&self) -> Option<f64> {
        let stop = self.rotation_stops.get(self.rotation_cursor)?;
        let current = pose_to_point2(self.current_odom.as_ref()?);
        (current.distance(stop.1) <= 0.05).then_some(stop.2)
    }
    pub(super) fn update_oriented_tracking(&mut self) {
        let (Some(path), Some(yaws), Some(odom)) =
            (&self.path, &self.path_yaws, &self.current_odom)
        else {
            return;
        };
        let position = pose_to_point2(odom);
        let yaw = odom.pose.orientation.yaw();
        while let Some(stop) = self.rotation_stops.get(self.rotation_cursor) {
            if position.distance(stop.1) <= 0.05
                && angle_diff(stop.2, yaw).abs() <= self.config.orientation_tolerance.min(0.15)
                && self.last_command.linear.x.hypot(self.last_command.linear.y) < 0.03
            {
                self.rotation_cursor += 1;
            } else {
                break;
            }
        }
        let stop_distance = self
            .rotation_stops
            .get(self.rotation_cursor)
            .map(|stop| self.path_cumulative[stop.0])
            .unwrap_or(f64::INFINITY);
        let upper = (self.path_progress + self.progress_credit + 0.02).min(stop_distance);
        let mut best_distance = f64::INFINITY;
        let mut progress = self.path_progress;
        let mut segment = self.pose_index;
        for i in self.pose_index.saturating_sub(1)..path.points.len().saturating_sub(1) {
            if self.path_cumulative[i] > upper + 1e-9 {
                break;
            }
            let a = path.points[i];
            let b = path.points[i + 1];
            let length = a.distance(b);
            if length < 1e-9 {
                continue;
            }
            let fraction = (((position.x - a.x) * (b.x - a.x) + (position.y - a.y) * (b.y - a.y))
                / (length * length))
                .clamp(0.0, 1.0);
            let along = self.path_cumulative[i] + fraction * length;
            if along + 0.02 < self.path_progress || along > upper + 1e-9 {
                continue;
            }
            let projected = Point2::new(a.x + (b.x - a.x) * fraction, a.y + (b.y - a.y) * fraction);
            let distance = position.distance(projected);
            if distance < best_distance - 1e-9 {
                best_distance = distance;
                progress = along.max(self.path_progress);
                segment = i;
            }
        }
        self.progress_credit = (self.progress_credit - (progress - self.path_progress)).max(0.0);
        self.path_progress = progress;
        self.pose_index = segment;
        let target_distance = (progress + 0.5).min(stop_distance);
        let index = self
            .path_cumulative
            .partition_point(|distance| *distance < target_distance)
            .min(path.points.len().saturating_sub(1));
        let previous = index.saturating_sub(1);
        let length = self.path_cumulative[index] - self.path_cumulative[previous];
        let fraction = if length > 1e-9 {
            (target_distance - self.path_cumulative[previous]) / length
        } else {
            0.0
        };
        self.tracking_target = Some((
            Point2::new(
                path.points[previous].x
                    + (path.points[index].x - path.points[previous].x) * fraction.clamp(0.0, 1.0),
                path.points[previous].y
                    + (path.points[index].y - path.points[previous].y) * fraction.clamp(0.0, 1.0),
            ),
            yaws[previous] + angle_diff(yaws[index], yaws[previous]) * fraction.clamp(0.0, 1.0),
        ));
    }
    pub(super) fn oriented_preference(&mut self) -> Option<Twist2> {
        let (target, target_yaw) = self.tracking_target?;
        let odom = self.current_odom.as_ref()?.clone();
        let position = pose_to_point2(&odom);
        let yaw = odom.pose.orientation.yaw();
        if let Some(stop_yaw) = self.rotation_target_yaw() {
            if self.last_command.linear.x.hypot(self.last_command.linear.y) > 0.03
                || self
                    .observed_velocity
                    .linear
                    .x
                    .hypot(self.observed_velocity.linear.y)
                    > 0.03
            {
                return Some(Twist2::ZERO);
            }
            return Some(self.controller.rotate(angle_diff(stop_yaw, yaw)));
        }
        let remaining = self.path_cumulative.last().copied().unwrap_or(0.0) - self.path_progress;
        if remaining <= 0.05 && self.rotation_cursor >= self.rotation_stops.len() {
            return None;
        }
        if self.state != LocalPlannerState::PathFollowing {
            self.change_state(LocalPlannerState::PathFollowing);
        }
        let mut command = if self
            .motion_limits
            .as_ref()
            .is_some_and(|limits| limits.model == MotionModel::Omnidirectional)
        {
            let dx = target.x - position.x;
            let dy = target.y - position.y;
            let scale = self.config.speed / dx.hypot(dy).max(1e-6);
            Twist2 {
                linear: Point2::new(
                    (dx * yaw.cos() + dy * yaw.sin()) * scale,
                    (-dx * yaw.sin() + dy * yaw.cos()) * scale,
                ),
                angular_z: self
                    .controller
                    .rotate(angle_diff(target_yaw, yaw))
                    .angular_z,
            }
        } else if self.allow_reverse
            && angle_diff(
                (target.y - position.y).atan2(target.x - position.x),
                target_yaw,
            )
            .abs()
                > std::f64::consts::FRAC_PI_2
        {
            Twist2 {
                linear: Point2::new(-self.config.speed, 0.0),
                angular_z: self
                    .controller
                    .rotate(angle_diff(target_yaw, yaw))
                    .angular_z,
            }
        } else {
            self.controller.advance((target.x, target.y), odom)
        };
        if let Some(limits) = &self.motion_limits {
            let distance = self
                .rotation_stops
                .get(self.rotation_cursor)
                .map_or(remaining, |stop| {
                    self.path_cumulative[stop.0] - self.path_progress
                })
                .max(0.0);
            let delay = limits.reaction_time_sec + 1.0 / self.config.control_frequency;
            let maximum = ((limits.guaranteed_braking_mps2 * delay).powi(2)
                + 2.0 * limits.guaranteed_braking_mps2 * distance)
                .sqrt()
                - limits.guaranteed_braking_mps2 * delay;
            let scale = (maximum / command.linear.x.hypot(command.linear.y).max(1e-9)).min(1.0);
            command.linear.x *= scale;
            command.linear.y *= scale;
        }
        Some(command)
    }
}

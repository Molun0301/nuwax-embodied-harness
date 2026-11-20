//! 在一拍可达窗口中比较恒速圆弧，同一快照完成整批机身查询。
use nuwax_core::logging::CollisionEvidence;
use nuwax_nav_scan_native::{clearance_speed_cap, BodySample, CollisionSnapshot, Config};
use nuwax_nav_types_messages::{Point2, Twist2};

pub(crate) const PREDICTION_SECONDS: f64 = 1.5;
const SAMPLE_SECONDS: f64 = 0.1;
const PREDICTION_STEPS: usize = 15;
const HEADING_LOOKAHEAD_M: f64 = 0.6;
const PROJECTION_MARGIN_M: f64 = 0.5;
const LINEAR_INTERVALS: usize = 4;
const ANGULAR_INTERVALS: usize = 8;
// 得分统一为等效米数，偏航按机身半长折算，净空差额按一米等于一米计。
const CLEARANCE_WEIGHT: f64 = 1.0;

#[derive(Default)]
pub(crate) struct Route {
    points: Vec<[f64; 3]>,
    arc: Vec<f64>,
}

impl Route {
    pub(crate) fn new(points: Vec<[f64; 3]>) -> Self {
        let mut arc = vec![0.; points.len()];
        for (index, pair) in points.windows(2).enumerate() {
            arc[index + 1] = arc[index] + (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]);
        }
        Self { points, arc }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub(crate) fn length(&self) -> f64 {
        *self.arc.last().unwrap()
    }

    pub(crate) fn nearest(&self, position: [f64; 3]) -> (f64, [f64; 3]) {
        let mut best = (0., self.points[0]);
        let mut distance = squared_distance(best.1, position);
        for (index, pair) in self.points.windows(2).enumerate() {
            let delta = std::array::from_fn::<_, 3, _>(|i| pair[1][i] - pair[0][i]);
            let norm = delta.iter().map(|v| v * v).sum::<f64>();
            if norm == 0. {
                continue;
            }
            let along = (0..3)
                .map(|i| (position[i] - pair[0][i]) * delta[i])
                .sum::<f64>();
            let fraction = (along / norm).clamp(0., 1.);
            let projected = std::array::from_fn(|i| pair[0][i] + fraction * delta[i]);
            let candidate = squared_distance(projected, position);
            if candidate < distance {
                distance = candidate;
                best = (
                    self.arc[index] + fraction * (self.arc[index + 1] - self.arc[index]),
                    projected,
                );
            }
        }
        best
    }

    /// 候选终点只投影到当前弧长之后、本次可达范围内的路线。
    fn project_endpoint(&self, position: [f64; 4], start: f64, end: f64) -> (f64, [f64; 3]) {
        let mut best = (start, self.point_at(start));
        let mut distance = xy_distance_squared(best.1, position);
        let first = self.arc.partition_point(|&s| s <= start).saturating_sub(1);
        for index in first..self.points.len() - 1 {
            let arc = self.arc[index];
            if arc >= end {
                break;
            }
            let length = self.arc[index + 1] - arc;
            if length == 0. {
                continue;
            }
            let a = self.points[index];
            let b = self.points[index + 1];
            let dx = b[0] - a[0];
            let dy = b[1] - a[1];
            let fraction = (((position[0] - a[0]) * dx + (position[1] - a[1]) * dy)
                / (length * length))
                .clamp(
                    ((start - arc) / length).max(0.),
                    ((end - arc) / length).min(1.),
                );
            let point = std::array::from_fn(|i| a[i] + (b[i] - a[i]) * fraction);
            let candidate = xy_distance_squared(point, position);
            if candidate < distance {
                distance = candidate;
                best = (arc + fraction * length, point);
            }
        }
        best
    }

    fn heading_at(&self, arc: f64, point: [f64; 3], from: [f64; 3]) -> f64 {
        let forward = self.point_at(arc + HEADING_LOOKAHEAD_M);
        let delta = [forward[0] - point[0], forward[1] - point[1]];
        if delta != [0., 0.] {
            return delta[1].atan2(delta[0]);
        }
        // 到达路线末点时以最后一个非零路段的方向延续。
        if let Some(last) = self.points.iter().rev().find(|p| p[..2] != forward[..2]) {
            return (forward[1] - last[1]).atan2(forward[0] - last[0]);
        }
        (forward[1] - from[1]).atan2(forward[0] - from[0])
    }

    fn point_at(&self, distance: f64) -> [f64; 3] {
        let index = self.arc.partition_point(|&v| v <= distance);
        if index == self.arc.len() {
            return *self.points.last().unwrap();
        }
        let before = index - 1;
        let fraction = (distance - self.arc[before]) / (self.arc[index] - self.arc[before]);
        std::array::from_fn(|i| {
            self.points[before][i] + fraction * (self.points[index][i] - self.points[before][i])
        })
    }
}

fn xy_distance_squared(a: [f64; 3], b: [f64; 4]) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

fn squared_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum()
}

fn angle_delta(to: f64, from: f64) -> f64 {
    let difference = to - from;
    difference.sin().atan2(difference.cos())
}

/// 对称区间包含两端、上一拍能维持的值，以及可达的精确零速。
fn samples(min: f64, max: f64, previous: f64, intervals: usize) -> Vec<f64> {
    let mut out = (0..=intervals)
        .map(|index| min + (max - min) * index as f64 / intervals as f64)
        .collect::<Vec<_>>();
    out.push(previous.clamp(min, max));
    if min <= 0. && max >= 0. {
        out.push(0.);
    }
    out.sort_by(f64::total_cmp);
    out.dedup();
    out
}

fn arc_pose(position: [f64; 3], yaw: f64, v: f64, w: f64, time: f64) -> [f64; 4] {
    let half = w * time / 2.;
    let travel = v * time * if half == 0. { 1. } else { half.sin() / half };
    [
        position[0] + travel * (yaw + half).cos(),
        position[1] + travel * (yaw + half).sin(),
        position[2],
        yaw + 2. * half,
    ]
}

pub(crate) struct Selection {
    pub(crate) command: Twist2,
    pub(crate) poses: Vec<[f64; 3]>,
}

fn blocked(sample: &BodySample, here: BodySample) -> bool {
    sample.occupied < 0
        || (sample.occupied != 0 && (here.occupied != 1 || sample.depth > here.depth))
}

struct Candidate {
    score: f64,
    same_turn: bool,
    index: usize,
    v: f64,
    w: f64,
}

#[derive(Default)]
pub(crate) struct Sampler {
    poses: Vec<[f64; 4]>,
    samples: Vec<BodySample>,
    refined_poses: Vec<[f64; 4]>,
    refined_samples: Vec<BodySample>,
    pub(crate) yaw_error: Option<f64>,
}

impl Sampler {
    pub(crate) fn select(
        &mut self,
        snapshot: &CollisionSnapshot,
        config: &Config,
        reference: &Route,
        position: [f64; 3],
        yaw: f64,
        previous: Twist2,
        turn_to: Option<f64>,
    ) -> Option<Selection> {
        let (arc, nearest) = reference.nearest(position);
        let end = (arc + config.max_velocity * PREDICTION_SECONDS + PROJECTION_MARGIN_M)
            .min(reference.length());
        let heading = turn_to.unwrap_or_else(|| reference.heading_at(arc, nearest, position));
        self.yaw_error = Some(angle_delta(heading, yaw));

        let mut here = [BodySample::default()];
        snapshot.sample_poses(
            config,
            &[[position[0], position[1], position[2], yaw]],
            &mut here,
            None,
        );
        let here = here[0];
        let speed_cap = clearance_speed_cap(config, here.clearance);
        let linear_step = config.max_acceleration * config.sample_dt;
        let angular_step = config.max_yaw_acceleration * config.sample_dt;
        let min_linear = (previous.linear.x - linear_step).max(-config.max_reverse_velocity);
        let max_linear = (previous.linear.x + linear_step).min(config.max_velocity);
        // 净空下调时先在可达窗边缘减速。
        let min_linear = min_linear.max((-speed_cap).min(max_linear));
        let max_linear = max_linear.min(speed_cap.max(min_linear));
        let min_yaw = (previous.angular_z - angular_step).max(-config.max_yaw_rate);
        let max_yaw = (previous.angular_z + angular_step).min(config.max_yaw_rate);
        let forward = if turn_to.is_some() {
            vec![0.]
        } else {
            samples(min_linear, max_linear, previous.linear.x, LINEAR_INTERVALS)
        };
        let angular = samples(min_yaw, max_yaw, previous.angular_z, ANGULAR_INTERVALS);
        self.poses.clear();
        for &v in &forward {
            for &w in &angular {
                for step in 1..=PREDICTION_STEPS {
                    let mut pose = arc_pose(position, yaw, v, w, step as f64 * SAMPLE_SECONDS);
                    if v != 0. {
                        // 三维机体高度随同一条参考路线，保留当拍实测的高度偏移。
                        let (_, projected) = reference.nearest([pose[0], pose[1], pose[2]]);
                        pose[2] += projected[2] - nearest[2];
                    }
                    self.poses.push(pose);
                }
            }
        }
        self.samples.resize(self.poses.len(), BodySample::default());
        snapshot.sample_poses(config, &self.poses, &mut self.samples, None);

        let mut ranked = Vec::new();
        for ((index, samples), (v, w)) in
            self.samples.chunks_exact(PREDICTION_STEPS).enumerate().zip(
                forward
                    .iter()
                    .flat_map(|&v| angular.iter().map(move |&w| (v, w))),
            )
        {
            if samples.iter().any(|sample| blocked(sample, here)) {
                continue;
            }
            let endpoint = self.poses[(index + 1) * PREDICTION_STEPS - 1];
            let score = if let Some(target_yaw) = turn_to {
                angle_delta(target_yaw, endpoint[3]).abs()
            } else {
                let (projected_arc, projected) = reference.project_endpoint(endpoint, arc, end);
                let route_heading = reference.heading_at(projected_arc, projected, position);
                let clearance = samples
                    .iter()
                    .map(|s| s.clearance)
                    .fold(f64::INFINITY, f64::min);
                reference.length() - projected_arc
                    + xy_distance_squared(projected, endpoint).sqrt()
                    + config.body_length / 2. * angle_delta(route_heading, endpoint[3]).abs()
                    + CLEARANCE_WEIGHT * (config.obstacle_clearance_m - clearance).max(0.)
            };
            ranked.push(Candidate {
                score,
                same_turn: w * previous.angular_z > 0.,
                index,
                v,
                w,
            });
        }
        ranked.sort_by(|a, b| {
            a.score
                .total_cmp(&b.score)
                .then_with(|| b.same_turn.cmp(&a.same_turn))
        });
        self.refined_poses.clear();
        self.refined_samples.clear();
        let radius = (config.body_length / 2. + config.body_center_x.abs())
            .hypot(config.body_width / 2. + config.body_center_y.abs());
        for Candidate { index, v, w, .. } in ranked {
            let coarse = &self.poses[index * PREDICTION_STEPS..(index + 1) * PREDICTION_STEPS];
            self.refined_poses.clear();
            let mut start_z = position[2];
            for (step, point) in coarse.iter().enumerate() {
                let dz = point[2] - start_z;
                // 平移、转角扫过的最远机身点及沿路线的高度变化共同限定步距。
                let travel = SAMPLE_SECONDS * (v.abs() + radius * w.abs()) + dz.abs();
                let subdivisions = (2. * travel / config.resolution).ceil().max(1.) as usize;
                for part in 1..=subdivisions {
                    let fraction = part as f64 / subdivisions as f64;
                    let mut sample = arc_pose(
                        position,
                        yaw,
                        v,
                        w,
                        (step as f64 + fraction) * SAMPLE_SECONDS,
                    );
                    sample[2] = start_z + fraction * dz;
                    self.refined_poses.push(sample);
                }
                start_z = point[2];
            }
            self.refined_samples
                .resize(self.refined_poses.len(), BodySample::default());
            snapshot.check_poses(config, &self.refined_poses, &mut self.refined_samples, None);
            if self
                .refined_samples
                .iter()
                .any(|sample| blocked(sample, here))
            {
                continue;
            }
            return Some(Selection {
                command: Twist2 {
                    linear: Point2::new(v, 0.),
                    angular_z: w,
                },
                poses: coarse.iter().map(|p| [p[0], p[1], p[2]]).collect(),
            });
        }
        None
    }

    pub(crate) fn blocking_cells(
        &mut self,
        snapshot: &CollisionSnapshot,
        config: &Config,
        position: [f64; 3],
        yaw: f64,
    ) -> CollisionEvidence {
        let mut evidence = CollisionEvidence::new(position, config.body_to_ground);
        snapshot.sample_poses(
            config,
            &[[position[0], position[1], position[2], yaw]],
            &mut [BodySample::default()],
            Some(&mut evidence),
        );
        snapshot.sample_poses(config, &self.poses, &mut self.samples, Some(&mut evidence));
        snapshot.check_poses(
            config,
            &self.refined_poses,
            &mut self.refined_samples,
            Some(&mut evidence),
        );
        evidence
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_scan_native::MapOwner;

    const START: [f64; 3] = [0., 0., 0.4];

    fn fixture(obstacles: &[[f64; 3]]) -> (Config, CollisionSnapshot) {
        let mut config = crate::SurfaceFollowerConfig::new(0.4, 0.15).scan;
        config.resolution = 0.05;
        config.max_velocity = 1.;
        config.max_yaw_rate = 0.8;
        config.max_acceleration = 1.;
        config.max_yaw_acceleration = 11.;
        let mut map = MapOwner::new(&config).unwrap();
        map.seed_occupied(obstacles, START);
        map.update_surface(&[], &[], &[], &[]).unwrap();
        (config, map.snapshot())
    }

    #[test]
    fn straight_route_uses_reachable_forward_command_only() {
        let (config, map) = fixture(&[]);
        let route = Route::new(vec![START, [3., 0., START[2]]]);
        let chosen = Sampler::default()
            .select(&map, &config, &route, START, 0., Twist2::ZERO, None)
            .unwrap();
        assert_eq!(
            chosen.command.linear.x,
            config.max_acceleration * config.sample_dt
        );
        assert_eq!(chosen.command.linear.y, 0.);
        assert_eq!(chosen.command.angular_z, 0.);
    }

    #[test]
    fn along_route_progress_keeps_accelerating_past_the_old_speed_plateau() {
        let (config, map) = fixture(&[]);
        let route = Route::new(vec![START, [7., 0., START[2]]]);
        let mut sampler = Sampler::default();
        let mut twist = Twist2::ZERO;
        let mut position = START;
        for _ in 0..20 {
            twist = sampler
                .select(&map, &config, &route, position, 0., twist, None)
                .unwrap()
                .command;
            position[0] += twist.linear.x * config.sample_dt;
        }
        assert_eq!(twist.linear.x, config.max_velocity);
        assert_eq!(twist.angular_z, 0.);
    }

    #[test]
    fn endpoint_projection_cannot_jump_to_a_later_fold() {
        let route = Route::new(vec![
            START,
            [3., 0., START[2]],
            [3., 0.1, START[2]],
            [0., 0.1, START[2]],
        ]);
        let (arc, point) = route.project_endpoint([0.8, 0.1, START[2], 0.], 0.4, 2.4);
        assert!((arc - 0.8).abs() < 1e-12);
        assert!(squared_distance(point, [0.8, 0., START[2]]) < 1e-24);
    }

    #[test]
    fn dense_review_rejects_a_cell_between_clear_coarse_poses() {
        let (mut config, _) = fixture(&[]);
        config.resolution = 0.02;
        config.body_length = 0.02;
        config.body_width = 0.02;
        let mut owner = MapOwner::new(&config).unwrap();
        owner.seed_occupied(&[[0.05, 0.01, 0.425]], START);
        owner.update_surface(&[], &[], &[], &[]).unwrap();
        let map = owner.snapshot();
        assert_eq!(map.pose_clearance(START, 0.).occupied, 0);
        assert_eq!(map.pose_clearance([0.1, 0., START[2]], 0.).occupied, 0);
        assert_eq!(map.pose_clearance([0.05, 0., START[2]], 0.).occupied, 1);
        let route = Route::new(vec![START, [3., 0., START[2]]]);
        let previous = Twist2 {
            linear: Point2::new(1., 0.),
            ..Twist2::ZERO
        };
        assert!(Sampler::default()
            .select(&map, &config, &route, START, 0., previous, None)
            .is_none());
    }

    #[test]
    fn occupied_straight_arc_is_replaced_by_a_clear_arc() {
        let (config, map) = fixture(&[[0.775, 0.125, 0.425]]);
        let route = Route::new(vec![START, [3., 0., START[2]]]);
        assert_eq!(map.pose_clearance([0.6, 0., START[2]], 0.).occupied, 1);
        let previous = Twist2 {
            linear: Point2::new(0.4, 0.),
            ..Twist2::ZERO
        };
        let chosen = Sampler::default()
            .select(&map, &config, &route, START, 0., previous, None)
            .unwrap();
        assert!(chosen.command.linear.x > 0.);
        assert!(chosen.command.angular_z < 0., "{:?}", chosen.command);
        let last = *chosen.poses.last().unwrap();
        assert_eq!(
            map.pose_clearance(last, chosen.command.angular_z * PREDICTION_SECONDS)
                .occupied,
            0
        );
    }

    #[test]
    fn goal_behind_turns_toward_route_when_reverse_is_disabled() {
        let (config, map) = fixture(&[]);
        let route = Route::new(vec![START, [-2., 0.3, START[2]]]);
        let chosen = Sampler::default()
            .select(&map, &config, &route, START, 0., Twist2::ZERO, None)
            .unwrap();
        assert_eq!(chosen.command.linear, Point2::new(0., 0.));
        assert!(chosen.command.angular_z > 0.);
    }

    #[test]
    fn touching_body_can_advance_without_deepening_the_overlap() {
        let (config, map) = fixture(&[[-0.225, 0.025, 0.425]]);
        let route = Route::new(vec![START, [2., 0., START[2]]]);
        let mut here = [BodySample::default()];
        map.sample_poses(&config, &[[0., 0., START[2], 0.]], &mut here, None);
        assert_eq!(here[0].occupied, 1);
        assert!(here[0].depth > 0.);
        let mut sampler = Sampler::default();
        let result = sampler
            .select(&map, &config, &route, START, 0., Twist2::ZERO, None)
            .unwrap();
        assert!(result.command.linear.x > 0.);
        let mut checked = vec![BodySample::default(); PREDICTION_STEPS];
        let poses: Vec<_> = result
            .poses
            .iter()
            .enumerate()
            .map(|(i, p)| {
                [
                    p[0],
                    p[1],
                    p[2],
                    result.command.angular_z * (i + 1) as f64 * SAMPLE_SECONDS,
                ]
            })
            .collect();
        map.sample_poses(&config, &poses, &mut checked, None);
        assert!(checked.iter().all(|sample| sample.depth <= here[0].depth));
    }
}

//! 在一拍可达窗口中比较恒速圆弧，同一快照完成整批机身查询。
use nuwax_core::logging::CollisionEvidence;
use nuwax_nav_scan_native::{clearance_speed_cap, BodySample, CollisionSnapshot, Config};
use nuwax_nav_types_messages::{Point2, Twist2};

pub(crate) const PREDICTION_SECONDS: f64 = 1.5;
const SAMPLE_SECONDS: f64 = 0.1;
const PREDICTION_STEPS: usize = 15;
const LOOKAHEAD_MIN_M: f64 = 0.6;
const LINEAR_INTERVALS: usize = 4;
const ANGULAR_INTERVALS: usize = 8;
// 得分统一为等效米数，偏航按机身半长折算，净空差额按一米等于一米计。
const CLEARANCE_WEIGHT: f64 = 1.0;
// 同等走向的小速度差用少量代价区分。
const UNUSED_SPEED_WEIGHT_M: f64 = 0.02;

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

fn arc_pose(position: [f64; 3], yaw: f64, linear: Point2, w: f64, time: f64) -> [f64; 4] {
    let half = w * time / 2.;
    let scale = if half == 0. { 1. } else { half.sin() / half };
    let forward = linear.x * time * scale;
    let lateral = linear.y * time * scale;
    let (s, c) = (yaw + half).sin_cos();
    [
        position[0] + forward * c - lateral * s,
        position[1] + forward * s + lateral * c,
        position[2],
        yaw + 2. * half,
    ]
}

pub(crate) struct Selection {
    pub(crate) command: Twist2,
    pub(crate) poses: Vec<[f64; 3]>,
}

pub(crate) struct PoseGoal {
    pub(crate) position: [f64; 3],
    pub(crate) yaw: f64,
    pub(crate) position_tolerance_m: f64,
    pub(crate) yaw_tolerance_rad: f64,
    pub(crate) position_reached: bool,
}

fn blocked(sample: &BodySample, here: BodySample) -> bool {
    sample.occupied < 0
        || (sample.occupied != 0 && (here.occupied != 1 || sample.depth > here.depth))
}

struct Candidate {
    score: f64,
    same_turn: bool,
    index: usize,
    linear: Point2,
    w: f64,
}

#[derive(Default)]
pub(crate) struct Sampler {
    poses: Vec<[f64; 4]>,
    samples: Vec<BodySample>,
    refined_poses: Vec<[f64; 4]>,
    refined_samples: Vec<BodySample>,
    terminal: bool,
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
        pose_goal: Option<PoseGoal>,
    ) -> Option<Selection> {
        let (arc, nearest) = reference.nearest(position);
        let lookahead = LOOKAHEAD_MIN_M.max(PREDICTION_SECONDS * config.max_velocity);
        self.terminal = pose_goal.as_ref().is_some_and(|goal| {
            let exit = goal.position_tolerance_m + config.body_length / 2.;
            let near = squared_distance(position, goal.position) <= exit * exit;
            // 朝向已经合适时，全向机体可以在末段横移，不必朝侧向位置误差转身。
            let aligned_approach = config.max_lateral_velocity > 0.
                && reference.length() - arc <= exit
                && angle_delta(goal.yaw, yaw).abs() <= goal.yaw_tolerance_rad;
            goal.position_reached
                || near && (self.terminal || aligned_approach)
        });
        let turn_to = pose_goal.filter(|_| self.terminal).map(|goal| goal.yaw);
        let target_arc = if self.terminal {
            reference.length()
        } else {
            arc + lookahead
        };
        let target = reference.point_at(target_arc);
        let delta = [target[0] - position[0], target[1] - position[1]];
        let heading = turn_to.unwrap_or_else(|| delta[1].atan2(delta[0]));
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
        // 朝向修正也保留可达的平移动作，碰撞扫掠决定是否能边退边转。
        let forward = samples(min_linear, max_linear, previous.linear.x, LINEAR_INTERVALS);
        let (sin, cos) = yaw.sin_cos();
        let desired_lateral = (-delta[0] * sin + delta[1] * cos) / PREDICTION_SECONDS;
        let mut translations = Vec::with_capacity(forward.len() * 2);
        for vx in forward {
            let x_acceleration = vx - previous.linear.x;
            let y_step = (linear_step.powi(2) - x_acceleration.powi(2)).max(0.).sqrt();
            let lateral_cap = if self.terminal {
                config.max_lateral_velocity.min((speed_cap.powi(2) - vx.powi(2)).max(0.).sqrt())
            } else {
                0.
            };
            let reachable_min = previous.linear.y - y_step;
            let reachable_max = previous.linear.y + y_step;
            let minimum = reachable_min.max(-lateral_cap).min(reachable_max);
            let maximum = reachable_max.min(lateral_cap).max(minimum);
            let zero = 0_f64.clamp(minimum, maximum);
            translations.push(Point2::new(vx, zero));
            let toward_goal = desired_lateral.clamp(minimum, maximum);
            if toward_goal != zero {
                translations.push(Point2::new(vx, toward_goal));
            }
        }
        let angular = samples(min_yaw, max_yaw, previous.angular_z, ANGULAR_INTERVALS);
        self.poses.clear();
        for &linear in &translations {
            for &w in &angular {
                for step in 1..=PREDICTION_STEPS {
                    let mut pose = arc_pose(position, yaw, linear, w, step as f64 * SAMPLE_SECONDS);
                    if linear.x != 0. || linear.y != 0. {
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
        for ((index, samples), (linear, w)) in
            self.samples.chunks_exact(PREDICTION_STEPS).enumerate().zip(
                translations
                    .iter()
                    .flat_map(|&linear| angular.iter().map(move |&w| (linear, w))),
            )
        {
            if samples.iter().any(|sample| blocked(sample, here)) {
                continue;
            }
            let endpoint = self.poses[(index + 1) * PREDICTION_STEPS - 1];
            let clearance = samples
                .iter()
                .map(|sample| sample.clearance)
                .fold(f64::INFINITY, f64::min);
            let speed_fraction =
                linear.x.hypot(linear.y) / config.max_velocity.max(config.max_reverse_velocity);
            let score = (endpoint[0] - target[0]).hypot(endpoint[1] - target[1])
                + config.body_length / 2. * angle_delta(heading, endpoint[3]).abs()
                + CLEARANCE_WEIGHT * (config.obstacle_clearance_m - clearance).max(0.)
                + if self.terminal {
                    0.
                } else {
                    UNUSED_SPEED_WEIGHT_M * (1. - speed_fraction)
                };
            ranked.push(Candidate {
                score,
                same_turn: w * previous.angular_z > 0.,
                index,
                linear,
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
        for Candidate { index, linear, w, .. } in ranked {
            let coarse = &self.poses[index * PREDICTION_STEPS..(index + 1) * PREDICTION_STEPS];
            self.refined_poses.clear();
            let mut start_z = position[2];
            for (step, point) in coarse.iter().enumerate() {
                let dz = point[2] - start_z;
                // 平移、转角扫过的最远机身点及沿路线的高度变化共同限定步距。
                let travel = SAMPLE_SECONDS * (linear.x.hypot(linear.y) + radius * w.abs()) + dz.abs();
                let subdivisions = (2. * travel / config.resolution).ceil().max(1.) as usize;
                for part in 1..=subdivisions {
                    let fraction = part as f64 / subdivisions as f64;
                    let mut sample = arc_pose(
                        position,
                        yaw,
                        linear,
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
                    linear,
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
    fn fixed_lookahead_keeps_accelerating_past_the_old_speed_plateau() {
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
    fn terminal_turn_does_not_move_along_the_route_when_rotation_is_clear() {
        let (config, map) = fixture(&[]);
        let route = Route::new(vec![START]);
        let oriented = Sampler::default()
            .select(
                &map, &config, &route, START, 0., Twist2::ZERO,
                Some(PoseGoal {
                    position: START, yaw: 1., position_tolerance_m: 0.06,
                    yaw_tolerance_rad: 0.15, position_reached: true,
                }),
            )
            .unwrap();
        assert_eq!(oriented.command.linear, Point2::new(0., 0.));
        assert!(oriented.command.angular_z > 0.);
    }

    #[test]
    fn terminal_heading_and_translation_survive_drift_past_arrival_tolerance() {
        let (mut config, map) = fixture(&[]);
        config.body_length = 0.30;
        config.max_reverse_velocity = 0.20;
        let route = Route::new(vec![[2., 0., START[2]], START]);
        let mut sampler = Sampler::default();
        let goal = |position_reached| Some(PoseGoal {
            position: START, yaw: 0., position_tolerance_m: 0.06,
            yaw_tolerance_rad: 0.15, position_reached,
        });
        let approach = sampler
            .select(
                &map, &config, &route, [0.18, 0., START[2]], 0.55, Twist2::ZERO, goal(false),
            )
            .unwrap();
        assert!(approach.command.angular_z > 0., "{:?}", approach.command);
        for (x, position_reached) in [(0.04, true), (0.07, false), (0.18, false)] {
            let position = [x, 0., START[2]];
            let chosen = sampler
                .select(
                    &map, &config, &route, position, 0.55, Twist2::ZERO,
                    goal(position_reached),
                )
                .unwrap();
            assert!(chosen.command.angular_z < 0., "{:?}", chosen.command);
            assert!(chosen.poses.last().unwrap()[0].abs() < x);
        }
        let rejoin = sampler
            .select(
                &map, &config, &route, [0.34, 0., START[2]], 0.55, Twist2::ZERO,
                goal(false),
            )
            .unwrap();
        assert!(rejoin.command.angular_z > 0., "{:?}", rejoin.command);
    }

    #[test]
    fn sideward_pose_error_uses_lateral_motion_when_the_device_can_strafe() {
        let (mut config, map) = fixture(&[]);
        let target = [0., 0.09, START[2]];
        let route = Route::new(vec![START, target]);
        let goal = || Some(PoseGoal {
            position: target, yaw: 0., position_tolerance_m: 0.06,
            yaw_tolerance_rad: 0.15, position_reached: false,
        });
        let without_lateral = Sampler::default()
            .select(&map, &config, &route, START, 0., Twist2::ZERO, goal())
            .unwrap();
        assert_eq!(without_lateral.command.linear.y, 0.);
        assert!(without_lateral.command.angular_z > 0.);
        config.max_lateral_velocity = 0.2;
        let lateral = Sampler::default()
            .select(&map, &config, &route, START, 0., Twist2::ZERO, goal())
            .unwrap();
        assert_eq!(lateral.command.angular_z, 0.);
        assert!(lateral.command.linear.y > 0.);
        assert!(lateral.command.linear.x.hypot(lateral.command.linear.y)
            <= config.max_acceleration * config.sample_dt);
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

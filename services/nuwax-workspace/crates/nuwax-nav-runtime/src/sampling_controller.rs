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

#[derive(Default)]
pub(crate) struct Sampler {
    poses: Vec<[f64; 4]>,
    samples: Vec<BodySample>,
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
        let lookahead = LOOKAHEAD_MIN_M.max(PREDICTION_SECONDS * previous.linear.x);
        let target = if turn_to.is_some() {
            position
        } else {
            reference.point_at(arc + lookahead)
        };
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

        let mut best: Option<(f64, bool, usize, f64, f64)> = None;
        for ((index, samples), (v, w)) in
            self.samples.chunks_exact(PREDICTION_STEPS).enumerate().zip(
                forward
                    .iter()
                    .flat_map(|&v| angular.iter().map(move |&w| (v, w))),
            )
        {
            if samples.iter().any(|sample| {
                sample.occupied < 0
                    || (sample.occupied != 0 && (here.occupied != 1 || sample.depth > here.depth))
            }) {
                continue;
            }
            let endpoint = self.poses[(index + 1) * PREDICTION_STEPS - 1];
            let clearance = samples
                .iter()
                .map(|sample| sample.clearance)
                .fold(f64::INFINITY, f64::min);
            let speed_fraction = v.abs() / config.max_velocity.max(config.max_reverse_velocity);
            let score = (endpoint[0] - target[0]).hypot(endpoint[1] - target[1])
                + config.body_length / 2. * angle_delta(heading, endpoint[3]).abs()
                + CLEARANCE_WEIGHT * (config.obstacle_clearance_m - clearance).max(0.)
                + UNUSED_SPEED_WEIGHT_M * (1. - speed_fraction);
            let same_turn = w * previous.angular_z > 0.;
            if best.is_none_or(|(old, old_turn, ..)| {
                score < old || (score == old && same_turn && !old_turn)
            }) {
                best = Some((score, same_turn, index, v, w));
            }
        }
        best.map(|(_, _, index, v, w)| Selection {
            command: Twist2 {
                linear: Point2::new(v, 0.),
                angular_z: w,
            },
            poses: self.poses[index * PREDICTION_STEPS..(index + 1) * PREDICTION_STEPS]
                .iter()
                .map(|pose| [pose[0], pose[1], pose[2]])
                .collect(),
        })
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

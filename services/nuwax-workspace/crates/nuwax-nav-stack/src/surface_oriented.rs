//! 按连续机体位姿搜索三维路径，平面运动基元在真实支撑面上逐段核验。
use nuwax_nav_control::{integrate, PlanarPose};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_types_messages::{GoalOptions, MotionLimits, MotionModel, Point2, Twist2};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

type BodyPose = ([f64; 3], [f64; 4]);
fn angle(v: f64) -> f64 {
    (v + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
fn planar(p: BodyPose) -> PlanarPose {
    let [x, y, z, w] = p.1;
    PlanarPose {
        x: p.0[0],
        y: p.0[1],
        yaw: (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)),
    }
}
#[derive(PartialEq)]
struct Queue(f64, usize);
impl Eq for Queue {}
impl PartialOrd for Queue {
    fn partial_cmp(&self, b: &Self) -> Option<Ordering> {
        Some(self.cmp(b))
    }
}
impl Ord for Queue {
    fn cmp(&self, b: &Self) -> Ordering {
        b.0.total_cmp(&self.0).then_with(|| b.1.cmp(&self.1))
    }
}
struct Node {
    pose: BodyPose,
    cost: f64,
    parent: Option<usize>,
    samples: Vec<BodyPose>,
}

fn action(
    map: &SurfaceSnapshot,
    from: BodyPose,
    velocity: Twist2,
    duration: f64,
    boundary: &[[f64; 2]],
) -> Option<Vec<BodyPose>> {
    let model = map.body_envelope()?;
    let start = planar(from);
    let n = ((velocity.linear.x.hypot(velocity.linear.y)
        + model.radius() * velocity.angular_z.abs())
        * duration
        / (f64::from(map.config().voxel_size) * 0.5))
        .max(velocity.angular_z.abs() * duration / 0.08)
        .ceil()
        .max(1.0) as usize;
    if n > 1024 {
        return None;
    }
    let mut samples = Vec::with_capacity(n);
    let mut previous = from;
    for i in 1..=n {
        let next = integrate(start, velocity, duration * i as f64 / n as f64);
        let support = map
            .locate_support_from_body(previous.0, previous.1, model.body_to_support_z_m)
            .ok()?.support_xyz;
        let lifted = map
            .body_pose_on_surface([next.x, next.y], next.yaw, support)
            .inspect_err(|error| tracing::trace!(?error,?previous,?next,?support,"三维基元投影被拒绝"))
            .ok()?;
        map.validate_body_motion(previous, lifted, boundary)
            .inspect_err(|error| tracing::trace!(?error,?previous,?lifted,"三维基元扫掠被拒绝"))
            .ok()?;
        samples.push(lifted);
        previous = lifted;
    }
    Some(samples)
}
fn connect(
    map: &SurfaceSnapshot,
    from: BodyPose,
    to: BodyPose,
    boundary: &[[f64; 2]],
    rotate: bool,
    reverse: bool,
    omni: bool,
) -> Option<Vec<BodyPose>> {
    let a = planar(from);
    let b = planar(to);
    let distance = (a.x - b.x).hypot(a.y - b.y);
    let heading = (b.y - a.y).atan2(b.x - a.x);
    let mut headings = vec![heading];
    if reverse {
        headings.push(angle(heading + std::f64::consts::PI));
    }
    if omni || distance < 1e-9 {
        headings.insert(0, a.yaw);
    }
    headings.sort_by(|x, y| {
        (angle(*x - a.yaw).abs() + angle(b.yaw - *x).abs())
            .total_cmp(&(angle(*y - a.yaw).abs() + angle(b.yaw - *y).abs()))
    });
    for heading in headings {
        if !rotate && (angle(heading - a.yaw).abs() > 1e-5 || angle(b.yaw - heading).abs() > 1e-5) {
            continue;
        }
        let candidate = (|| {
            let mut out = vec![from];
            let delta = angle(heading - a.yaw);
            if delta.abs() > 1e-6 {
                out.extend(action(
                    map,
                    from,
                    Twist2 {
                        linear: Point2::new(0.0, 0.0),
                        angular_z: delta.signum(),
                    },
                    delta.abs(),
                    boundary,
                )?);
            }
            let aligned = *out.last()?;
            let velocity = Twist2 {
                linear: Point2::new(
                    (b.x - a.x) * heading.cos() + (b.y - a.y) * heading.sin(),
                    -(b.x - a.x) * heading.sin() + (b.y - a.y) * heading.cos(),
                ),
                angular_z: 0.0,
            };
            if distance > 1e-9 {
                out.extend(action(map, aligned, velocity, 1.0, boundary)?);
            }
            let current = *out.last()?;
            let delta = angle(b.yaw - planar(current).yaw);
            if delta.abs() > 1e-6 {
                out.extend(action(
                    map,
                    current,
                    Twist2 {
                        linear: Point2::new(0.0, 0.0),
                        angular_z: delta.signum(),
                    },
                    delta.abs(),
                    boundary,
                )?);
            }
            map.validate_body_motion(*out.last()?, to, boundary).ok()?;
            out.push(to);
            Some(out)
        })();
        if candidate.is_some() {
            return candidate;
        }
    }
    None
}

/// 复用生产机体规划查询；返回真实三维机体路径，不包含运动副作用。
pub fn plan(
    map: &SurfaceSnapshot,
    start: BodyPose,
    mut goal: BodyPose,
    options: GoalOptions,
    limits: Option<&MotionLimits>,
    boundary: &[[f64; 2]],
    budget: usize,
) -> Result<Vec<BodyPose>, String> {
    let began=std::time::Instant::now();
    options.validate()?;
    if let Some(limits) = limits {
        limits.validate()?;
    }
    map.validate_body_pose(start.0, start.1)
        .map_err(|e| format!("起点机体包络: {e}"))?;
    let model = map.body_envelope().ok_or("三维完整机体标定缺失")?;
    if map.locate_support_from_body(goal.0, goal.1, model.body_to_support_z_m).is_err() {
        let support = map.locate_goal_support_from_body(goal.0, goal.1, model.body_to_support_z_m)
            .map_err(|e| format!("目标支撑层: {e}"))?;
        goal = map.body_pose_on_surface([goal.0[0], goal.0[1]], planar(goal).yaw, support.surface_xyz)
            .map_err(|e| format!("目标支撑面投影: {e}"))?;
    }
    // 起终点与沿途扫掠共用同一当前占据视图；局部补足的足迹边界仍由内核检查。
    map.validate_body_pose(goal.0, goal.1)
        .map_err(|e| format!("目标机体包络: {e}"))?;
    let rotate = limits.is_none_or(|m| m.can_rotate_in_place);
    let reverse = options.allow_reverse && limits.is_some_and(|m| m.max_reverse_mps > 0.0);
    if options.allow_reverse && !reverse {
        return Err("倒车缺少标定".into());
    }
    let omni =
        limits.is_some_and(|m| m.model == MotionModel::Omnidirectional && m.max_lateral_mps > 0.0);
    if let Some(path) = connect(map, start, goal, boundary, rotate, reverse, omni) {
        return Ok(path);
    }
    let resolution = f64::from(map.config().voxel_size);
    let yaw_step = std::f64::consts::TAU / 32.0;
    let radius = limits
        .map_or(0.0, |m| m.minimum_turning_radius_m)
        .max(model.radius())
        .max(0.2);
    let length = resolution * 2.0;
    let key = |p: BodyPose| {
        let s = map
            .locate_support_from_body(p.0, p.1, model.body_to_support_z_m)
            .ok()?;
        Some((
            s.cell,
            ((planar(p).yaw / yaw_step).round() as i32).rem_euclid(32),
        ))
    };
    let distance = |p: BodyPose| {
        (p.0[0] - goal.0[0])
            .hypot(p.0[1] - goal.0[1])
            .hypot(p.0[2] - goal.0[2])
    };
    let mut nodes = vec![Node {
        pose: start,
        cost: 0.0,
        parent: None,
        samples: vec![],
    }];
    let mut best = HashMap::from([(key(start).ok_or("起点支撑缺失")?, (0.0, 0))]);
    let mut queue = BinaryHeap::from([Queue(distance(start), 0)]);
    let mut expanded = 0;
    while let Some(Queue(_, index)) = queue.pop() {
        let here = nodes[index].pose;
        let cost = nodes[index].cost;
        if best
            .get(&key(here).ok_or("搜索状态支撑缺失")?)
            .is_none_or(|&(_, known)| known != index)
        {
            continue;
        }
        expanded += 1;
        if began.elapsed()>std::time::Duration::from_secs(2) {return Err("三维朝向搜索超过两秒计算预算".into());}
        if expanded > budget || nodes.len() > budget * 4 {
            return Err("三维朝向搜索预算耗尽".into());
        }
        let close = distance(here) <= options.position_tolerance_m.min(resolution).max(1e-6)
            && angle(planar(here).yaw - planar(goal).yaw).abs()
                <= options.yaw_tolerance_rad.min(yaw_step).max(1e-6);
        let tail = if close {
            Some(vec![here])
        } else if distance(here) < length * 3.0 || expanded % 64 == 0 {
            connect(map, here, goal, boundary, rotate, reverse, omni)
        } else {
            None
        };
        if let Some(tail) = tail {
            let mut chain = vec![];
            let mut cursor = index;
            while let Some(parent) = nodes[cursor].parent {
                chain.push(cursor);
                cursor = parent;
            }
            chain.reverse();
            let mut path = vec![start];
            for i in chain {
                path.extend_from_slice(&nodes[i].samples);
            }
            path.extend(tail.into_iter().skip(1));
            return Ok(path);
        }
        let mut actions = vec![];
        for sign in [1.0, -1.0] {
            if sign < 0.0 && !reverse {
                continue;
            }
            for curvature in [0.0, -1.0 / radius, 1.0 / radius] {
                actions.push((
                    Twist2 {
                        linear: Point2::new(sign, 0.0),
                        angular_z: sign * curvature,
                    },
                    if curvature == 0.0 {
                        length
                    } else {
                        radius * yaw_step
                    },
                ));
            }
        }
        if rotate {
            for sign in [-1.0, 1.0] {
                actions.push((
                    Twist2 {
                        linear: Point2::new(0.0, 0.0),
                        angular_z: sign,
                    },
                    yaw_step,
                ));
            }
        }
        if omni {
            for sign in [-1.0, 1.0] {
                actions.push((
                    Twist2 {
                        linear: Point2::new(0.0, sign),
                        angular_z: 0.0,
                    },
                    length,
                ));
            }
        }
        if distance(here) < radius + length * 3.0 {
            let delta = angle(planar(goal).yaw - planar(here).yaw).clamp(-yaw_step, yaw_step);
            if delta.abs() > 1e-4 {
                actions.push((
                    Twist2 {
                        linear: Point2::new(1.0, 0.0),
                        angular_z: delta.signum() / radius,
                    },
                    radius * delta.abs(),
                ));
            }
            actions.push((
                Twist2 {
                    linear: Point2::new(1.0, 0.0),
                    angular_z: 0.0,
                },
                distance(here).clamp(resolution * 0.25, length),
            ));
        }
        for (velocity, duration) in actions {
            let Some(samples) = action(map, here, velocity, duration, boundary) else {
                continue;
            };
            let Some(&end) = samples.last() else {
                continue;
            };
            let Some(k) = key(end) else {
                continue;
            };
            if Some(k) == key(here) {
                continue;
            }
            let next = cost
                + velocity.linear.x.hypot(velocity.linear.y) * duration
                + model.radius() * velocity.angular_z.abs() * duration * 0.2
                + 0.001;
            if best.get(&k).is_some_and(|&(old, _)| old <= next) {
                continue;
            }
            let id = nodes.len();
            nodes.push(Node {
                pose: end,
                cost: next,
                parent: Some(index),
                samples,
            });
            best.insert(k, (next, id));
            queue.push(Queue(next + distance(end), id));
        }
    }
    Err("不存在满足完整机体与运动约束的三维路线".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope};
    use nuwax_nav_runtime::{
        ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, SurfaceFollower,
        SurfaceFollowerConfig,
    };
    use nuwax_nav_types_messages::{Path3D, Point3, Pose3, PoseStamped, Quaternion, Timestamp};
    use std::sync::Arc;
    fn scene(obstacle: bool) -> SurfaceSnapshot {
        let mut points: Vec<_> = (-35..35)
            .flat_map(|x| {
                (-20..20).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
            })
            .collect();
        if obstacle {
            for x in -2..2 {
                for y in -2..2 {
                    points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.55));
                }
            }
        }
        let cfg = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.35,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.1,
            wall_buffer_m: 0.0,
            wall_buffer_weight: 0.0,
            max_slope_deg: 20.0,
            step_threshold_m: 0.2,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        SurfaceSnapshot::from_points(&points, cfg, 0.1)
            .unwrap()
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.3, -0.12], [0.3, -0.12], [0.3, 0.12], [-0.3, 0.12]],
                z_bounds_m: [-0.05, 0.2],
                body_to_support_z_m: 0.4,
                max_step_up_m: 0.2,
                max_step_down_m: 0.2,
                max_cross_slope_deg: 15.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap()
    }
    fn pose(x: f64, y: f64, yaw: f64) -> BodyPose {
        (
            [x, y, 0.5],
            [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()],
        )
    }
    fn stamped(p: BodyPose, nanos: i64) -> PoseStamped {
        PoseStamped {
            frame_id: nuwax_nav_types_messages::NavFrameId::Map,
            stamp: Timestamp::from_unix(nanos / 1_000_000_000, (nanos % 1_000_000_000) as u32),
            pose: Pose3 {
                position: Point3::new(p.0[0], p.0[1], p.0[2]),
                orientation: Quaternion {
                    x: p.1[0],
                    y: p.1[1],
                    z: p.1[2],
                    w: p.1[3],
                },
            },
        }
    }
    fn limits() -> MotionLimits {
        MotionLimits {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.3,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.7,
            linear_acceleration_mps2: 1.0,
            guaranteed_braking_mps2: 1.0,
            angular_acceleration_rps2: 2.0,
            guaranteed_angular_braking_rps2: 2.0,
            reaction_time_sec: 0.1,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        }
    }
    #[test]
    fn complete_body_search_detours_and_preserves_rotation() {
        let map = scene(true);
        let start = pose(-1.55, 0.05, 0.0);
        let goal = pose(1.55, 0.05, 0.0);
        let path = plan(
            &map,
            start,
            goal,
            GoalOptions::default(),
            Some(&limits()),
            &[],
            8000,
        )
        .unwrap();
        assert!(path.iter().any(|p| p.0[1].abs() > 0.3));
        for p in path.windows(2) {
            map.validate_body_motion(p[0], p[1], &[]).unwrap();
        }
        let turn = plan(
            &scene(false),
            pose(0.05, 0.05, 0.0),
            pose(0.05, 0.05, 1.2),
            GoalOptions::default(),
            Some(&limits()),
            &[],
            100,
        )
        .unwrap();
        assert!(turn.len() > 2);
        assert!(turn
            .iter()
            .all(|p| (p.0[0] - 0.05).hypot(p.0[1] - 0.05) < 1e-5));
    }
    #[test]
    fn blind_start_planning_and_following_reach_observed_ground() {
        let base = scene(false);
        let start = pose(0.027, 0.013, 0.0);
        let goal = pose(0.827, 0.013, 0.0);
        let points: Vec<_> = (-40..40).flat_map(|x| (-30..30).map(move |y|
            ((x as f32 + 0.5) * 0.05, (y as f32 + 0.5) * 0.05, 0.075)))
            .filter(|&(x, y, _)| (f64::from(x)-start.0[0]).abs() > 0.3
                || (f64::from(y)-start.0[1]).abs() > 0.12).collect();
        let map = Arc::new(SurfaceSnapshot::from_points(&points, base.config().clone(), 0.05)
            .unwrap().with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        assert!(map.validate_body_pose(start.0, start.1).is_err());
        let view = map.with_current_body_support(start.0, start.1).unwrap();
        let motion = limits();
        let options = GoalOptions { allow_reverse: false, position_tolerance_m: 0.08,
            yaw_tolerance_rad: 0.08 };
        let path = plan(&view, start, goal, options, Some(&motion), &[], 2000).unwrap();
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.motion_limits = Some(motion);
        config.controller.control_frequency = 20.0;
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
        let mut current = start;
        let mut nanos = 1_000_000_000;
        follower.update_odometry(stamped(current, nanos));
        follower.configure_goal(&stamped(goal, nanos), options).unwrap();
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
        }), None, stamped(goal, nanos)).unwrap();
        let mut reached = false;
        for _ in 0..1000 {
            let command = match follower.step(0.24) {
                PlannerStep::Command(v) => v,
                PlannerStep::Succeeded => { reached = true; break; }
                other => panic!("盲区起步执行失败: {other:?} at {current:?}"),
            };
            let next = integrate(planar(current), command, 0.05);
            // 反馈来自独立平面运动积分，真实高度保持不变。
            current = ([next.x, next.y, start.0[2]],
                [0.0, 0.0, (next.yaw*0.5).sin(), (next.yaw*0.5).cos()]);
            nanos += 50_000_000;
            follower.update_odometry(stamped(current, nanos));
        }
        assert!(reached, "盲区起步未到达目标: {current:?}");
        assert!((current.0[0]-goal.0[0]).abs() < 0.08);
        assert!(map.validate_body_pose(start.0, start.1).is_err(), "执行不能补写原始盲区");
    }

    #[test]
    fn current_occupied_goal_uses_the_same_view_as_start_and_path() {
        let base = scene(false);
        let start = pose(0.027, 0.013, 0.0);
        let points: Vec<_> = (-40..40).flat_map(|x| (-30..30).map(move |y|
            ((x as f32 + 0.5) * 0.05, (y as f32 + 0.5) * 0.05, 0.075)))
            .filter(|&(x, y, _)| (f64::from(x)-start.0[0]).abs() > 0.3
                || (f64::from(y)-start.0[1]).abs() > 0.12).collect();
        let map = SurfaceSnapshot::from_points(&points, base.config().clone(), 0.05)
            .unwrap().with_body_envelope(base.body_envelope().unwrap().clone()).unwrap();
        let view = map.with_current_body_support(start.0, start.1).unwrap();
        assert!(map.validate_body_pose(start.0, start.1).is_err());
        let options = GoalOptions { allow_reverse: false, position_tolerance_m: 0.08,
            yaw_tolerance_rad: 0.08 };
        plan(&view, start, start, options, Some(&limits()), &[], 2000)
            .expect("当前占据区内的目标不能重新用未处理自体的原图拒绝");
    }

    #[test]
    fn observed_ground_band_height_error_does_not_stall_before_final_rotation() {
        let base = scene(false);
        let points: Vec<_> = (-20..20)
            .flat_map(|x| {
                (-15..15).flat_map(move |y| {
                    [0.025, 0.075, 0.125]
                        .into_iter()
                        .map(move |z| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z))
                })
            })
            .collect();
        let map = Arc::new(
            SurfaceSnapshot::from_points(&points, base.config().clone(), 0.05)
                .unwrap()
                .with_body_envelope(base.body_envelope().unwrap().clone())
                .unwrap(),
        );
        let start = ([-0.55, 0.05, 0.46], [0.0, 0.0, 0.0, 1.0]);
        let goal = ([0.55, 0.05, 0.70], [0.0, 0.0, 0.3_f64.sin(), 0.3_f64.cos()]);
        let options = GoalOptions {
            allow_reverse: false,
            position_tolerance_m: 0.1,
            yaw_tolerance_rad: 0.08,
        };
        let motion = limits();
        let path = plan(&map, start, goal, options, Some(&motion), &[], 3000).unwrap();
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.motion_limits = Some(motion);
        config.controller.control_frequency = 20.0;
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower
            .update_execution_map(ExecutionMap::Surface(map))
            .unwrap();
        let mut current = start;
        let mut nanos = 1_000_000_000;
        follower.update_odometry(stamped(current, nanos));
        follower
            .configure_goal(&stamped(goal, nanos), options)
            .unwrap();
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                }),
                None,
                stamped(goal, nanos),
            )
            .unwrap();
        let mut reached = false;
        for _ in 0..1200 {
            let command = match follower.step(0.24) {
                PlannerStep::Command(v) => v,
                PlannerStep::Succeeded => {
                    reached = true;
                    break;
                }
                other => panic!("{other:?} at {current:?}"),
            };
            let next = integrate(planar(current), command, 0.05);
            // 独立平面模型保持真实高度，不让规划高度成为模拟反馈。
            current = (
                [next.x, next.y, start.0[2]],
                [0.0, 0.0, (next.yaw * 0.5).sin(), (next.yaw * 0.5).cos()],
            );
            nanos += 50_000_000;
            follower.update_odometry(stamped(current, nanos));
        }
        assert!(reached, "同一支撑层高度偏差导致停止推进: {current:?}");
        assert!((current.0[0] - goal.0[0]).hypot(current.0[1] - goal.0[1])
            < options.position_tolerance_m);
        assert!(angle(planar(current).yaw - planar(goal).yaw).abs() < 0.08);
    }

    #[test]
    fn same_xy_on_another_observed_floor_never_completes_body_goal() {
        let base = scene(false);
        let points: Vec<_> = (-20..20).flat_map(|x| (-15..15).flat_map(move |y|
            [0.025, 3.025].map(|z| ((x as f32 + 0.5) * 0.1,
                (y as f32 + 0.5) * 0.1, z)))).collect();
        let map = Arc::new(SurfaceSnapshot::from_points(&points, base.config().clone(), 0.1)
            .unwrap().with_body_envelope(base.body_envelope().unwrap().clone()).unwrap());
        let start = pose(0.05, 0.05, 0.0);
        let mut goal = start;
        goal.0[2] += 3.0;
        map.validate_body_pose(start.0, start.1).unwrap();
        map.validate_body_pose(goal.0, goal.1).unwrap();
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.motion_limits = Some(limits());
        let mut follower = SurfaceFollower::new(config).unwrap();
        follower.update_execution_map(ExecutionMap::Surface(map)).unwrap();
        follower.update_odometry(stamped(start, 1_000_000_000));
        follower.configure_goal(&stamped(goal, 1_000_000_000), GoalOptions {
            allow_reverse: false, position_tolerance_m: 0.1, yaw_tolerance_rad: 0.08,
        }).unwrap();
        // 当前层的短路径不能借水平距离为零而宣称到达另一层。
        follower.replace_execution_path(ExecutionPath::Surface(Path3D {
            poses: vec![stamped(start, 1_000_000_000), stamped(start, 1_000_000_000)],
        }), None, stamped(goal, 1_000_000_000)).unwrap();
        for index in 1..=4 {
            follower.update_odometry(stamped(start, 1_000_000_000 + index * 50_000_000));
            assert!(!matches!(follower.step(0.24), PlannerStep::Succeeded));
        }
    }

    #[test]
    fn shared_surface_control_reaches_forward_reverse_and_final_yaw() {
        for reverse in [false, true] {
            let map = Arc::new(scene(false));
            let start = pose(-0.55, 0.05, 0.0);
            let goal = pose(
                if reverse { -1.55 } else { 0.55 },
                0.05,
                if reverse { 0.0 } else { 1.0 },
            );
            let options = GoalOptions {
                allow_reverse: reverse,
                position_tolerance_m: 0.08,
                yaw_tolerance_rad: 0.08,
            };
            let limits = limits();
            let path = plan(&map, start, goal, options, Some(&limits), &[], 2000).unwrap();
            if reverse {
                assert!(path.iter().all(|p| planar(*p).yaw.abs() < 0.1));
            }
            let mut config = SurfaceFollowerConfig::new(0.4);
            config.motion_limits = Some(limits.clone());
            config.controller.control_frequency = 20.0;
            let mut follower = SurfaceFollower::new(config).unwrap();
            follower
                .update_execution_map(ExecutionMap::Surface(Arc::clone(&map)))
                .unwrap();
            let mut current = start;
            let mut nanos = 1_000_000_000;
            follower.update_odometry(stamped(current, nanos));
            follower
                .configure_goal(&stamped(goal, nanos), options)
                .unwrap();
            follower
                .replace_execution_path(
                    ExecutionPath::Surface(Path3D {
                        poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                    }),
                    None,
                    stamped(goal, nanos),
                )
                .unwrap();
            let mut reached = false;
            for _ in 0..1000 {
                let command = match follower.step(0.2) {
                    PlannerStep::Command(v) => v,
                    PlannerStep::Succeeded => {
                        reached = true;
                        break;
                    }
                    other => panic!("{other:?} at {current:?}"),
                };
                let next = integrate(planar(current), command, 0.05);
                let support = map
                    .support_position_from_body(current.0, current.1, 0.4)
                    .unwrap();
                let next = map
                    .body_pose_on_surface([next.x, next.y], next.yaw, support)
                    .unwrap();
                map.validate_body_motion(current, next, &[]).unwrap();
                current = next;
                nanos += 50_000_000;
                follower.update_odometry(stamped(current, nanos));
            }
            assert!(reached, "倒车={reverse},终态={current:?}");
        }
    }
    #[test]
    fn body_terrain_planner_and_control_cross_real_stair_heights_both_directions() {
        let base = scene(false);
        let mut cfg = base.config().clone();
        cfg.min_step_tread_depth_m = 0.3;
        let mut points = Vec::new();
        for x in 0..65 {
            for y in -15..15 {
                let h = if x < 20 {
                    0.0
                } else if x < 38 {
                    ((x - 20) / 6 + 1) as f32 * 0.15
                } else {
                    0.45
                };
                points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, h + 0.0125));
            }
        }
        let map = Arc::new(
            SurfaceSnapshot::from_points(&points, cfg, 0.025)
                .unwrap()
                .with_body_envelope(base.body_envelope().unwrap().clone())
                .unwrap(),
        );
        for descending in [false, true] {
            let a = ([0.55, 0.05, 0.425], [0.0, 0.0, 0.0, 1.0]);
            let b = ([4.55, 0.05, 0.875], [0.0, 0.0, 0.0, 1.0]);
            let (start, goal) = if descending { (b, a) } else { (a, b) };
            let options = GoalOptions {
                allow_reverse: descending,
                position_tolerance_m: 0.08,
                yaw_tolerance_rad: 0.08,
            };
            let calibrated = limits();
            let mut probe=start;
            for n in 1..=80 {
                let t=n as f64/80.0;let xy=[start.0[0]*(1.0-t)+goal.0[0]*t,0.05];
                let support=map.support_position_from_body(probe.0,probe.1,0.4).unwrap();
                let next=map.body_pose_on_surface(xy,0.0,support).unwrap_or_else(|e|panic!("地形采样 {n} {xy:?} from {probe:?}: {e}"));
                assert!((next.0[0]-xy[0]).hypot(next.0[1]-xy[1])<1e-8,"姿态拟合不得改变机体水平采样位置");
                map.validate_body_motion(probe,next,&[]).unwrap_or_else(|e|panic!("地形扫掠 {n}: {e}"));probe=next;
            }
            eprintln!("踏面逐点验证通过，下阶={descending}");
            let path = plan(&map, start, goal, options, Some(&calibrated), &[], 2000).unwrap();
            eprintln!("路径完成 {} 点，下阶={descending}",path.len());
            let mut config = SurfaceFollowerConfig::new(0.4);
            config.motion_limits = Some(calibrated);
            config.controller.control_frequency = 20.0;
            let mut follower = SurfaceFollower::new(config).unwrap();
            follower
                .update_execution_map(ExecutionMap::Surface(Arc::clone(&map)))
                .unwrap();
            let mut current = start;
            let mut nanos = 1_000_000_000;
            follower.update_odometry(stamped(current, nanos));
            follower
                .configure_goal(&stamped(goal, nanos), options)
                .unwrap();
            follower
                .replace_execution_path(
                    ExecutionPath::Surface(Path3D {
                        poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                    }),
                    None,
                    stamped(goal, nanos),
                )
                .unwrap();
            let mut reached = false;
            let mut replans = 0;
            for tick in 0..1500 {
                if tick%100==0 {eprintln!("控制 {tick},下阶={descending},位姿={current:?}");}
                let command = match follower.step(0.2) {
                    PlannerStep::Command(v) => v,
                    PlannerStep::Succeeded => {
                        reached = true;
                        break;
                    }
                    PlannerStep::ReplanRequired(reason) => {
                        replans += 1;
                        assert!(replans <= 8, "{reason} at {current:?}");
                        let path =
                            plan(&map, current, goal, options, Some(&limits()), &[], 2000).unwrap();
                        follower
                            .replace_execution_path(
                                ExecutionPath::Surface(Path3D {
                                    poses: path.into_iter().map(|p| stamped(p, nanos)).collect(),
                                }),
                                None,
                                stamped(goal, nanos),
                            )
                            .unwrap();
                        Twist2::ZERO
                    }
                    other => panic!("{other:?}"),
                };
                let next = integrate(planar(current), command, 0.05);
                let support = map
                    .support_position_from_body(current.0, current.1, 0.4)
                    .unwrap();
                let next = map
                    .body_pose_on_surface([next.x, next.y], next.yaw, support)
                    .unwrap();
                map.validate_body_motion(current, next, &[]).unwrap();
                current = next;
                nanos += 50_000_000;
                follower.update_odometry(stamped(current, nanos));
            }
            assert!(reached, "下阶={descending},终态={current:?}");
        }
    }
}

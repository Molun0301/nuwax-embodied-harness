//! 带完整机体扫掠的连续位姿搜索；固定预算，位置与偏航共同区分状态。
use crate::NavError;
use nuwax_nav_control::{integrate, PlanarPose};
use nuwax_nav_types_messages::{
    GoalOptions, MotionLimits, MotionModel, OccupancyGrid2D, Point2, RobotGeometry, Twist2,
};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

fn angle(value: f64) -> f64 {
    (value + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}
#[derive(Clone, Copy)]
struct Action {
    velocity: Twist2,
    duration: f64,
}
struct Node {
    pose: PlanarPose,
    cost: f64,
    parent: Option<usize>,
    action: Option<Action>,
}
#[derive(PartialEq)]
struct Queue(f64, usize);
impl Eq for Queue {}
impl PartialOrd for Queue {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Queue {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .0
            .total_cmp(&self.0)
            .then_with(|| other.1.cmp(&self.1))
    }
}

/// 按真实位置和朝向验证转向/平移扫掠；未知和图外不提供通行证据。
pub fn motion_is_clear(
    grid: &OccupancyGrid2D,
    geometry: &RobotGeometry,
    from: PlanarPose,
    to: PlanarPose,
) -> bool {
    let Ok(footprint) = geometry.grid_footprint(grid.resolution, from.yaw, grid.width, grid.height)
    else {
        return false;
    };
    footprint.visit_motion(
        grid.origin,
        (Point2::new(from.x, from.y), from.yaw),
        (Point2::new(to.x, to.y), to.yaw),
        |x, y| {
            grid.get(x, y)
                .is_some_and(|cell| cell.is_known_traversable())
        },
    )
}
fn action_samples(from: PlanarPose, action: Action, resolution: f64) -> Vec<PlanarPose> {
    let distance = action.velocity.linear.x.hypot(action.velocity.linear.y) * action.duration;
    let turn = action.velocity.angular_z.abs() * action.duration;
    let count = (distance / (resolution * 0.5))
        .max(turn / 0.05)
        .ceil()
        .max(1.0) as usize;
    if count > 4096 {
        return Vec::new();
    }
    (1..=count)
        .map(|i| {
            integrate(
                from,
                action.velocity,
                action.duration * i as f64 / count as f64,
            )
        })
        .collect()
}
fn straight_connection(
    grid: &OccupancyGrid2D,
    geometry: &RobotGeometry,
    from: PlanarPose,
    goal: PlanarPose,
    rotate: bool,
    omni: bool,
    reverse: bool,
    limits: Option<&MotionLimits>,
) -> Option<Vec<PlanarPose>> {
    let distance = (goal.x - from.x).hypot(goal.y - from.y);
    if distance < 1e-9 {
        return ((rotate || angle(goal.yaw - from.yaw).abs() < 1e-6)
            && motion_is_clear(grid, geometry, from, goal))
        .then(|| vec![from, goal]);
    }
    let heading = (goal.y - from.y).atan2(goal.x - from.x);
    let mut headings = vec![heading];
    if omni {
        headings.insert(0, from.yaw);
    }
    if reverse {
        headings.push(angle(heading + std::f64::consts::PI));
    }
    let mut best = None;
    let mut best_cost = f64::INFINITY;
    for movement_yaw in headings {
        let relative = angle(heading - movement_yaw);
        let forward = relative.cos();
        let lateral = relative.sin();
        if forward < -1e-8 && !reverse || lateral.abs() > 1e-8 && !omni {
            continue;
        }
        if !rotate
            && (angle(movement_yaw - from.yaw).abs() > 1e-6
                || angle(goal.yaw - movement_yaw).abs() > 1e-6)
        {
            continue;
        }
        let forward_speed = limits.map_or(0.55, |limits| {
            if forward >= 0.0 {
                limits.max_forward_mps
            } else {
                limits.max_reverse_mps
            }
        });
        let lateral_speed = limits.map_or(0.0, |limits| limits.max_lateral_mps);
        let speed = (if forward.abs() > 1e-8 {
            forward_speed / forward.abs()
        } else {
            f64::INFINITY
        })
        .min(if lateral.abs() > 1e-8 {
            lateral_speed / lateral.abs()
        } else {
            f64::INFINITY
        });
        if speed <= 0.0 {
            continue;
        }
        let angular = limits.map_or(0.55, |limits| limits.max_angular_rps);
        let cost = distance / speed
            + (angle(movement_yaw - from.yaw).abs() + angle(goal.yaw - movement_yaw).abs())
                / angular;
        if cost >= best_cost {
            continue;
        }
        let aligned = PlanarPose {
            yaw: movement_yaw,
            ..from
        };
        let arrived = PlanarPose {
            x: goal.x,
            y: goal.y,
            yaw: movement_yaw,
        };
        if motion_is_clear(grid, geometry, from, aligned)
            && motion_is_clear(grid, geometry, aligned, arrived)
            && motion_is_clear(grid, geometry, arrived, goal)
        {
            let mut result = vec![from];
            for pose in [aligned, arrived, goal] {
                if result.last() != Some(&pose) {
                    result.push(pose);
                }
            }
            best = Some(result);
            best_cost = cost;
        }
    }
    best
}

/// 混合位姿搜索：运动基元保留连续坐标，栅格/偏航桶只用于限定搜索状态。
/// 返回路径保留必要的原地旋转点；实际速度由原局部控制器与守卫负责。
pub fn plan_oriented(
    grid: &OccupancyGrid2D,
    geometry: &RobotGeometry,
    start: PlanarPose,
    goal: PlanarPose,
    options: GoalOptions,
    limits: Option<&MotionLimits>,
    max_expansions: usize,
) -> Result<Option<Vec<PlanarPose>>, NavError> {
    plan_oriented_interruptible(
        grid,
        geometry,
        start,
        goal,
        options,
        limits,
        max_expansions,
        &|| false,
    )
}

/// 受管搜索在每次扩展及运动基元前响应所属任务取消。
pub(crate) fn plan_oriented_interruptible(
    grid: &OccupancyGrid2D,
    geometry: &RobotGeometry,
    start: PlanarPose,
    goal: PlanarPose,
    options: GoalOptions,
    limits: Option<&MotionLimits>,
    max_expansions: usize,
    cancelled: &impl Fn() -> bool,
) -> Result<Option<Vec<PlanarPose>>, NavError> {
    let invalid = |reason: String| NavError::InvalidInput {
        field: "oriented_planner",
        reason,
    };
    options.validate().map_err(invalid)?;
    geometry.validate().map_err(invalid)?;
    if let Some(limits) = limits {
        limits.validate().map_err(invalid)?;
    }
    if ![
        start.x,
        start.y,
        start.yaw,
        goal.x,
        goal.y,
        goal.yaw,
        grid.resolution,
    ]
    .into_iter()
    .all(f64::is_finite)
        || grid.resolution <= 0.0
        || grid.cells.len() != grid.width as usize * grid.height as usize
    {
        return Err(invalid("地图或位姿无效".into()));
    }
    let reverse =
        options.allow_reverse && limits.is_some_and(|limits| limits.max_reverse_mps > 0.0);
    if options.allow_reverse && !reverse {
        return Err(invalid("倒车请求缺少能力标定".into()));
    }
    let rotate = limits.is_none_or(|limits| limits.can_rotate_in_place);
    let omni = limits.is_some_and(|limits| {
        limits.model == MotionModel::Omnidirectional && limits.max_lateral_mps > 0.0
    });
    if !motion_is_clear(grid, geometry, start, start)
        || !motion_is_clear(grid, geometry, goal, goal)
    {
        return Ok(None);
    }
    if let Some(direct) =
        straight_connection(grid, geometry, start, goal, rotate, omni, reverse, limits)
    {
        return Ok(Some(direct));
    }
    let radius = geometry
        .footprint_xy_m
        .iter()
        .map(|p| p[0].hypot(p[1]))
        .fold(0.0, f64::max)
        .max(grid.resolution);
    let minimum_radius = limits.map_or(0.0, |limits| limits.minimum_turning_radius_m);
    let curve_radius = minimum_radius.max(radius).max(0.2);
    let length = (grid.resolution * 2.0)
        .max((2.0 * grid.resolution * minimum_radius).sqrt())
        .min(4.0);
    let yaw_step = std::f64::consts::TAU / 32.0;
    let key = |pose: PlanarPose| {
        grid.world_to_cell(Point2::new(pose.x, pose.y))
            .map(|(x, y)| {
                (
                    x,
                    y,
                    (((angle(pose.yaw) + std::f64::consts::PI) / yaw_step).round() as i32)
                        .rem_euclid(32),
                )
            })
    };
    let Some(start_key) = key(start) else {
        return Ok(None);
    };
    let mut nodes = vec![Node {
        pose: start,
        cost: 0.0,
        parent: None,
        action: None,
    }];
    let mut best = HashMap::from([(start_key, (0.0, 0usize))]);
    let mut queue = BinaryHeap::from([Queue((start.x - goal.x).hypot(start.y - goal.y), 0)]);
    let mut expanded = 0usize;
    while let Some(Queue(_, index)) = queue.pop() {
        if cancelled() {
            return Ok(None);
        }
        let here = nodes[index].pose;
        let cost = nodes[index].cost;
        if best
            .get(&key(here).unwrap())
            .is_none_or(|&(_, best)| best != index)
        {
            continue;
        }
        expanded += 1;
        if expanded > max_expansions || nodes.len() > max_expansions.saturating_mul(4).max(1) {
            return Ok(None);
        }
        let distance = (here.x - goal.x).hypot(here.y - goal.y);
        let close = distance <= options.position_tolerance_m.min(grid.resolution).max(1e-6)
            && angle(here.yaw - goal.yaw).abs()
                <= options.yaw_tolerance_rad.min(yaw_step).max(1e-6);
        let connection = if close {
            Some(vec![here])
        } else if expanded % 64 == 0 || distance < length * 3.0 {
            straight_connection(grid, geometry, here, goal, rotate, omni, reverse, limits)
        } else {
            None
        };
        if let Some(connection) = connection {
            let mut route = Vec::new();
            let mut cursor = index;
            while let Some(parent) = nodes[cursor].parent {
                route.push((parent, nodes[cursor].action.unwrap()));
                cursor = parent;
            }
            route.reverse();
            let mut output = vec![start];
            for (parent, action) in route {
                output.extend(action_samples(nodes[parent].pose, action, grid.resolution));
            }
            output.extend(connection.into_iter().skip(1));
            // 小段原地旋转合并为一个明确停留点，保留完整转向扫掠的验证结果。
            let mut compact: Vec<PlanarPose> = Vec::new();
            for pose in output {
                if compact.len() >= 2 {
                    let a = compact[compact.len() - 2];
                    let b = *compact.last().unwrap();
                    if (a.x - b.x).hypot(a.y - b.y) < 1e-9
                        && (b.x - pose.x).hypot(b.y - pose.y) < 1e-9
                        && angle(pose.yaw - a.yaw).abs() <= std::f64::consts::FRAC_PI_2
                        && angle(b.yaw - a.yaw) * angle(pose.yaw - b.yaw) >= 0.0
                    {
                        compact.pop();
                    }
                }
                compact.push(pose);
            }
            return Ok(Some(compact));
        }
        let mut actions = Vec::new();
        for direction in [1.0, -1.0] {
            if direction < 0.0 && !reverse {
                continue;
            }
            for curvature in [0.0, -1.0 / curve_radius, 1.0 / curve_radius] {
                actions.push(Action {
                    velocity: Twist2 {
                        linear: Point2::new(direction, 0.0),
                        angular_z: curvature * direction,
                    },
                    duration: if curvature == 0.0 {
                        length
                    } else {
                        curve_radius * yaw_step
                    },
                });
            }
        }
        if distance < length * 4.0 + curve_radius {
            let error = angle(goal.yaw - here.yaw);
            for wanted in [
                error,
                error - error.signum() * options.yaw_tolerance_rad * 0.5,
            ] {
                let delta = wanted.clamp(-yaw_step, yaw_step);
                if delta.abs() > 1e-4 {
                    actions.push(Action {
                        velocity: Twist2 {
                            linear: Point2::new(1.0, 0.0),
                            angular_z: delta.signum() / curve_radius,
                        },
                        duration: curve_radius * delta.abs(),
                    });
                }
            }
            actions.push(Action {
                velocity: Twist2 {
                    linear: Point2::new(1.0, 0.0),
                    angular_z: 0.0,
                },
                duration: distance.max(grid.resolution * 0.25).min(length),
            });
        }
        if rotate {
            for sign in [-1.0, 1.0] {
                actions.push(Action {
                    velocity: Twist2 {
                        linear: Point2::new(0.0, 0.0),
                        angular_z: sign,
                    },
                    duration: yaw_step,
                });
            }
        }
        if omni {
            for (x, y) in [(0.0, 1.0), (0.0, -1.0), (0.707, 0.707), (0.707, -0.707)] {
                actions.push(Action {
                    velocity: Twist2 {
                        linear: Point2::new(x, y),
                        angular_z: 0.0,
                    },
                    duration: length,
                });
            }
        }
        for action in actions {
            if cancelled() {
                return Ok(None);
            }
            let end = integrate(here, action.velocity, action.duration);
            let Some(next_key) = key(end) else {
                continue;
            };
            if Some(next_key) == key(here) {
                continue;
            }
            let next_cost = cost
                + action.velocity.linear.x.hypot(action.velocity.linear.y) * action.duration
                + radius * action.velocity.angular_z.abs() * action.duration * 0.2
                + 0.001;
            if best
                .get(&next_key)
                .is_some_and(|&(known, _)| known <= next_cost)
            {
                continue;
            }
            let samples = action_samples(here, action, grid.resolution);
            if samples.is_empty() {
                continue;
            }
            let mut previous = here;
            let clear = samples.iter().all(|&next| {
                let clear = motion_is_clear(grid, geometry, previous, next);
                previous = next;
                clear
            });
            if !clear {
                continue;
            }
            let next_index = nodes.len();
            nodes.push(Node {
                pose: end,
                cost: next_cost,
                parent: Some(index),
                action: Some(action),
            });
            best.insert(next_key, (next_cost, next_index));
            queue.push(Queue(
                next_cost + (end.x - goal.x).hypot(end.y - goal.y),
                next_index,
            ));
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{OccupancyCell, Timestamp};
    fn geometry(length: f64) -> RobotGeometry {
        RobotGeometry {
            footprint_xy_m: vec![
                [-length / 2.0, -0.15],
                [length / 2.0, -0.15],
                [length / 2.0, 0.15],
                [-length / 2.0, 0.15],
            ],
            z_min_m: -0.2,
            z_max_m: 0.2,
            source: "仿真".into(),
        }
    }
    fn limits() -> MotionLimits {
        MotionLimits {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.25,
            max_lateral_mps: 0.25,
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
    fn differential() -> MotionLimits {
        let mut result = limits();
        result.max_lateral_mps = 0.0;
        result
    }
    fn execute(
        grid: &OccupancyGrid2D,
        geometry: &RobotGeometry,
        route: Vec<PlanarPose>,
        goal: PlanarPose,
        options: GoalOptions,
        limits: MotionLimits,
    ) {
        use nuwax_nav_astar::{
            helpers::{make_path2d, make_pose_stamped},
            local_planner::{LocalPlanner, StopMessage},
            navigation_map::CostMap2D,
        };
        let mut local = LocalPlanner::new(0.5, 20.0, options.position_tolerance_m.max(0.05));
        local.set_robot_geometry(geometry.clone());
        local.set_motion_limits(limits.clone()).unwrap();
        local
            .configure_goal(make_pose_stamped("map", goal.x, goal.y, goal.yaw), options)
            .unwrap();
        local.start_planning_with_orientations(
            make_path2d(route.iter().map(|p| Point2::new(p.x, p.y)).collect()),
            Some(route.iter().map(|p| p.yaw).collect()),
        );
        let mut costmap = CostMap2D::new(
            "map",
            grid.width as usize,
            grid.height as usize,
            grid.resolution,
        );
        costmap.set_origin(grid.origin);
        for y in 0..grid.height {
            for x in 0..grid.width {
                costmap.set(
                    x as usize,
                    y as usize,
                    if grid.get(x, y).unwrap().is_known_traversable() {
                        0
                    } else {
                        100
                    },
                );
            }
        }
        local.update_costmap(costmap);
        let mut pose = route[0];
        let mut velocity = Twist2::ZERO;
        let mut replan = false;
        let mut replans = 0;
        let mut brake_delay = None;
        for tick in 0..3200 {
            let mut odom = make_pose_stamped("map", pose.x, pose.y, pose.yaw);
            odom.stamp = Timestamp::from_unix(1 + tick / 20, (tick % 20 * 50_000_000) as u32);
            local.handle_odom(odom);
            if replan
                && velocity.linear.x.hypot(velocity.linear.y) + velocity.angular_z.abs() < 1e-8
            {
                let replacement =
                    plan_oriented(grid, geometry, pose, goal, options, Some(&limits), 200_000)
                        .unwrap()
                        .expect("停车后应能补规划");
                local.start_planning_with_orientations(
                    make_path2d(replacement.iter().map(|p| Point2::new(p.x, p.y)).collect()),
                    Some(replacement.iter().map(|p| p.yaw).collect()),
                );
                replan = false;
            }
            let (command, stop) = if replan {
                (Twist2::ZERO, None)
            } else {
                local.step(0.3)
            };
            if stop == Some(StopMessage::Arrived) {
                assert!(
                    (pose.x - goal.x).hypot(pose.y - goal.y) <= options.position_tolerance_m + 1e-9
                );
                assert!(angle(pose.yaw - goal.yaw).abs() <= options.yaw_tolerance_rad + 1e-9);
                assert!(
                    velocity.linear.x.hypot(velocity.linear.y) + velocity.angular_z.abs() < 0.02
                );
                return;
            }
            if stop == Some(StopMessage::ObstacleFound) {
                replans += 1;
                assert!(replans <= 12, "持续无法执行 pose={pose:?}");
                replan = true;
            } else {
                assert!(stop.is_none(), "控制错误 {stop:?}");
            }
            let next_velocity = if command == Twist2::ZERO && velocity != Twist2::ZERO {
                let delay = brake_delay.get_or_insert(limits.reaction_time_sec);
                if *delay > 1e-9 {
                    *delay = (*delay - 0.05).max(0.0);
                    velocity
                } else {
                    let ratio = (1.0
                        - limits.guaranteed_braking_mps2 * 0.05
                            / velocity.linear.x.hypot(velocity.linear.y).max(1e-9))
                    .max(0.0);
                    let angular = if velocity.angular_z > 0.0 {
                        (velocity.angular_z - limits.guaranteed_angular_braking_rps2 * 0.05)
                            .max(0.0)
                    } else {
                        (velocity.angular_z + limits.guaranteed_angular_braking_rps2 * 0.05)
                            .min(0.0)
                    };
                    Twist2 {
                        linear: Point2::new(velocity.linear.x * ratio, velocity.linear.y * ratio),
                        angular_z: angular,
                    }
                }
            } else {
                brake_delay = None;
                command
            };
            let applied = if command == Twist2::ZERO {
                Twist2 {
                    linear: Point2::new(
                        (velocity.linear.x + next_velocity.linear.x) * 0.5,
                        (velocity.linear.y + next_velocity.linear.y) * 0.5,
                    ),
                    angular_z: (velocity.angular_z + next_velocity.angular_z) * 0.5,
                }
            } else {
                next_velocity
            };
            let next = integrate(pose, applied, 0.05);
            assert!(motion_is_clear(grid,geometry,pose,next),"实际响应或制动期间不得进入墙体/未知区域 tick={tick} pose={pose:?} velocity={velocity:?}");
            pose = next;
            velocity = next_velocity;
        }
        panic!("联合规划与控制未在仿真期限内到达: {pose:?} goal={goal:?}");
    }
    #[test]
    fn long_body_turns_into_narrow_corridor_with_oriented_search_and_execution() {
        let mut grid =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 70, 70, OccupancyCell::Occupied);
        for y in 7..17 {
            for x in 5..45 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        for y in 7..25 {
            for x in 32..46 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        for y in 20..60 {
            for x in 35..43 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        let body = geometry(1.2);
        let start = PlanarPose {
            x: 1.25,
            y: 1.25,
            yaw: 0.0,
        };
        let goal = PlanarPose {
            x: 3.95,
            y: 5.25,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let options = GoalOptions {
            position_tolerance_m: 0.15,
            yaw_tolerance_rad: 0.2,
            ..Default::default()
        };
        let map = crate::Costmap::from_occupancy(grid.clone(), 0.2)
            .unwrap()
            .planning_view(Point2::new(start.x, start.y), start.yaw, &body)
            .unwrap();
        assert!(crate::plan_on_costmap(
            &map,
            Point2::new(start.x, start.y),
            Point2::new(goal.x, goal.y),
            None,
            None
        )
        .unwrap()
        .is_none());
        let route = plan_oriented(
            &grid,
            &body,
            start,
            goal,
            options,
            Some(&differential()),
            200_000,
        )
        .unwrap()
        .expect("朝向联合路径");
        execute(&grid, &body, route, goal, options, differential());
    }
    #[test]
    fn reverse_exit_requires_goal_permission_and_declared_capability() {
        let mut grid = OccupancyGrid2D::filled(
            Point2::new(0.0, 0.0),
            0.05,
            100,
            40,
            OccupancyCell::Occupied,
        );
        for y in 16..24 {
            for x in 5..95 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        let body = geometry(0.8);
        let start = PlanarPose {
            x: 3.0,
            y: 1.0,
            yaw: 0.0,
        };
        let goal = PlanarPose { x: 1.0, ..start };
        let mut options = GoalOptions {
            position_tolerance_m: 0.1,
            yaw_tolerance_rad: 0.1,
            ..Default::default()
        };
        assert!(plan_oriented(
            &grid,
            &body,
            start,
            goal,
            options,
            Some(&differential()),
            5000
        )
        .unwrap()
        .is_none());
        options.allow_reverse = true;
        assert!(plan_oriented(&grid, &body, start, goal, options, None, 5000).is_err());
        let route = plan_oriented(
            &grid,
            &body,
            start,
            goal,
            options,
            Some(&differential()),
            5000,
        )
        .unwrap()
        .unwrap();
        execute(&grid, &body, route, goal, options, differential());
    }
    #[test]
    fn ackermann_path_obeys_curvature_without_rotation_in_place() {
        let grid =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 100, 100, OccupancyCell::Free);
        let body = geometry(0.6);
        let start = PlanarPose {
            x: 2.0,
            y: 2.0,
            yaw: 0.0,
        };
        let goal = PlanarPose {
            x: 5.0,
            y: 5.0,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let mut limits = differential();
        limits.model = MotionModel::Ackermann;
        limits.can_rotate_in_place = false;
        limits.minimum_turning_radius_m = 1.0;
        let options = GoalOptions {
            position_tolerance_m: 0.15,
            yaw_tolerance_rad: 0.2,
            ..Default::default()
        };
        let route = plan_oriented(&grid, &body, start, goal, options, Some(&limits), 100_000)
            .unwrap()
            .expect("曲率受限路径");
        for segment in route.windows(2) {
            let distance = (segment[0].x - segment[1].x).hypot(segment[0].y - segment[1].y);
            let turn = angle(segment[1].yaw - segment[0].yaw).abs();
            assert!(distance + 1e-4 >= limits.minimum_turning_radius_m * turn);
        }
        execute(&grid, &body, route, goal, options, limits);
    }
}

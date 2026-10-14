//! Pure-pursuit path follower with bounded linear and yaw commands.

/// Planar pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2 {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
    /// Heading radians.
    pub yaw: f64,
}

/// Path point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
}

/// Velocity command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// Forward speed.
    pub linear: f64,
    /// Yaw rate.
    pub angular: f64,
    /// Goal reached flag.
    pub reached: bool,
}

/// Pure-pursuit parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowerConfig {
    /// Lookahead distance.
    pub lookahead: f64,
    /// Goal tolerance.
    pub goal_tolerance: f64,
    /// Maximum speed.
    pub max_speed: f64,
    /// Maximum yaw rate.
    pub max_yaw_rate: f64,
    /// Heading proportional gain.
    pub yaw_gain: f64,
}

impl Default for FollowerConfig {
    fn default() -> Self {
        Self {
            lookahead: 0.8,
            goal_tolerance: 0.25,
            max_speed: 1.0,
            max_yaw_rate: 1.2,
            yaw_gain: 1.5,
        }
    }
}

/// Compute one command from a pose and path.
pub fn follow(pose: Pose2, path: &[Point2], config: FollowerConfig) -> Option<Command> {
    if path.is_empty()
        || !path
            .iter()
            .all(|point| point.x.is_finite() && point.y.is_finite())
        || ![
            pose.x,
            pose.y,
            pose.yaw,
            config.lookahead,
            config.goal_tolerance,
            config.max_speed,
            config.max_yaw_rate,
            config.yaw_gain,
        ]
        .into_iter()
        .all(f64::is_finite)
        || config.lookahead <= 0.0
        || config.goal_tolerance < 0.0
        || config.max_speed < 0.0
        || config.max_yaw_rate < 0.0
        || config.yaw_gain < 0.0
    {
        return None;
    }
    let goal = *path.last()?;
    let goal_distance = (goal.x - pose.x).hypot(goal.y - pose.y);
    if goal_distance <= config.goal_tolerance {
        return Some(Command {
            linear: 0.0,
            angular: 0.0,
            reached: true,
        });
    }
    let target = path
        .iter()
        .copied()
        .find(|point| (point.x - pose.x).hypot(point.y - pose.y) >= config.lookahead)
        .unwrap_or(goal);
    let heading = (target.y - pose.y).atan2(target.x - pose.x);
    let error = normalize(heading - pose.yaw);
    let angular = (config.yaw_gain * error).clamp(-config.max_yaw_rate, config.max_yaw_rate);
    let turn_factor = (1.0 - error.abs() / std::f64::consts::PI).clamp(0.2, 1.0);
    Some(Command {
        linear: config.max_speed * turn_factor,
        angular,
        reached: false,
    })
}

fn normalize(angle: f64) -> f64 {
    let angle = angle % std::f64::consts::TAU;
    if angle > std::f64::consts::PI {
        angle - std::f64::consts::TAU
    } else if angle < -std::f64::consts::PI {
        angle + std::f64::consts::TAU
    } else {
        angle
    }
}

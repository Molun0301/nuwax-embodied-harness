//! Candidate trajectory collision filtering and cost selection.

/// Planar point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
}

/// Circular obstacle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obstacle {
    /// Centre point.
    pub center: Point2,
    /// Radius.
    pub radius: f64,
}

/// One precomputed local trajectory.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    /// Ordered path samples.
    pub points: Vec<Point2>,
}

/// Selection configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalCoreConfig {
    /// Robot collision radius.
    pub robot_radius: f64,
    /// Path length contribution to cost.
    pub length_weight: f64,
}

impl Default for LocalCoreConfig {
    fn default() -> Self {
        Self {
            robot_radius: 0.35,
            length_weight: 0.1,
        }
    }
}

/// Return the lowest-cost collision-free trajectory index and cost.
pub fn select_trajectory(
    goal: Point2,
    candidates: &[Trajectory],
    obstacles: &[Obstacle],
    config: LocalCoreConfig,
) -> Option<(usize, f64)> {
    if !finite(goal)
        || !config.robot_radius.is_finite()
        || config.robot_radius < 0.0
        || !config.length_weight.is_finite()
        || config.length_weight < 0.0
    {
        return None;
    }
    candidates
        .iter()
        .enumerate()
        .filter(|(_, trajectory)| {
            !trajectory.points.is_empty()
                && collision_free(trajectory, obstacles, config.robot_radius)
        })
        .map(|(index, trajectory)| {
            let end = *trajectory.points.last().expect("non-empty");
            let length: f64 = trajectory
                .points
                .windows(2)
                .map(|pair| distance(pair[0], pair[1]))
                .sum();
            (index, distance(end, goal) + config.length_weight * length)
        })
        .min_by(|left, right| left.1.total_cmp(&right.1))
}

/// Check every sampled segment against inflated obstacles.
#[must_use]
pub fn collision_free(trajectory: &Trajectory, obstacles: &[Obstacle], robot_radius: f64) -> bool {
    trajectory.points.iter().all(|point| finite(*point))
        && obstacles.iter().all(|obstacle| {
            finite(obstacle.center)
                && obstacle.radius.is_finite()
                && obstacle.radius >= 0.0
                && trajectory.points.iter().all(|point| {
                    distance(*point, obstacle.center) >= obstacle.radius + robot_radius
                })
                && trajectory.points.windows(2).all(|pair| {
                    segment_distance(obstacle.center, pair[0], pair[1])
                        >= obstacle.radius + robot_radius
                })
        })
}

fn segment_distance(point: Point2, a: Point2, b: Point2) -> f64 {
    let delta = Point2 {
        x: b.x - a.x,
        y: b.y - a.y,
    };
    let denominator = delta.x * delta.x + delta.y * delta.y;
    if denominator <= f64::EPSILON {
        return distance(point, a);
    }
    let t = (((point.x - a.x) * delta.x + (point.y - a.y) * delta.y) / denominator).clamp(0.0, 1.0);
    distance(
        point,
        Point2 {
            x: a.x + t * delta.x,
            y: a.y + t * delta.y,
        },
    )
}

fn distance(a: Point2, b: Point2) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}
fn finite(point: Point2) -> bool {
    point.x.is_finite() && point.y.is_finite()
}

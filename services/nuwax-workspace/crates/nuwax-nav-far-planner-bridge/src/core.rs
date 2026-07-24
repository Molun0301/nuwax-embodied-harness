//! Dependency-free 2D visibility-graph planner used when the locked CMU artifact is unavailable.

/// Two-dimensional point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    /// X coordinate in metres.
    pub x: f64,
    /// Y coordinate in metres.
    pub y: f64,
}

/// Circular obstacle approximation used by the auditable fallback.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircleObstacle {
    /// Obstacle centre.
    pub center: Point2,
    /// Occupied radius in metres.
    pub radius: f64,
}

/// Visibility-graph configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FarCoreConfig {
    /// Robot clearance added around each obstacle.
    pub clearance: f64,
    /// Number of evenly spaced boundary samples per obstacle.
    pub samples_per_obstacle: usize,
}

impl Default for FarCoreConfig {
    fn default() -> Self {
        Self {
            clearance: 0.3,
            samples_per_obstacle: 16,
        }
    }
}

/// Plan a collision-free shortest path over sampled obstacle boundaries.
pub fn plan(
    start: Point2,
    goal: Point2,
    obstacles: &[CircleObstacle],
    config: FarCoreConfig,
) -> Option<Vec<Point2>> {
    if !valid_point(start)
        || !valid_point(goal)
        || !config.clearance.is_finite()
        || config.clearance < 0.0
        || config.samples_per_obstacle < 4
        || obstacles.iter().any(|obstacle| {
            !valid_point(obstacle.center) || !obstacle.radius.is_finite() || obstacle.radius <= 0.0
        })
    {
        return None;
    }
    let mut nodes = vec![start, goal];
    for obstacle in obstacles {
        // A chord between samples on the exact boundary cuts through the circle.
        // Keep the graph ring outside the collision boundary so adjacent chords remain visible.
        let collision_radius = obstacle.radius + config.clearance;
        let radius = collision_radius * 1.05 + 1.0e-6;
        for sample in 0..config.samples_per_obstacle {
            let angle = std::f64::consts::TAU * sample as f64 / config.samples_per_obstacle as f64;
            nodes.push(Point2 {
                x: obstacle.center.x + radius * angle.cos(),
                y: obstacle.center.y + radius * angle.sin(),
            });
        }
    }

    let mut distance = vec![f64::INFINITY; nodes.len()];
    let mut previous = vec![None; nodes.len()];
    let mut visited = vec![false; nodes.len()];
    distance[0] = 0.0;
    for _ in 0..nodes.len() {
        let current = (0..nodes.len())
            .filter(|&index| !visited[index])
            .min_by(|&left, &right| distance[left].total_cmp(&distance[right]))?;
        if !distance[current].is_finite() {
            break;
        }
        if current == 1 {
            break;
        }
        visited[current] = true;
        for next in 0..nodes.len() {
            if current == next
                || visited[next]
                || !visible(nodes[current], nodes[next], obstacles, config.clearance)
            {
                continue;
            }
            let candidate = distance[current] + euclidean(nodes[current], nodes[next]);
            if candidate < distance[next] {
                distance[next] = candidate;
                previous[next] = Some(current);
            }
        }
    }
    if !distance[1].is_finite() {
        return None;
    }
    let mut indices = vec![1];
    let mut cursor = 1;
    while cursor != 0 {
        cursor = previous[cursor]?;
        indices.push(cursor);
    }
    indices.reverse();
    Some(indices.into_iter().map(|index| nodes[index]).collect())
}

/// Whether a segment respects all inflated obstacles.
#[must_use]
pub fn visible(a: Point2, b: Point2, obstacles: &[CircleObstacle], clearance: f64) -> bool {
    obstacles.iter().all(|obstacle| {
        segment_distance(obstacle.center, a, b) + 1.0e-9 >= obstacle.radius + clearance
    })
}

fn segment_distance(point: Point2, a: Point2, b: Point2) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let denominator = dx * dx + dy * dy;
    if denominator <= f64::EPSILON {
        return euclidean(point, a);
    }
    let t = (((point.x - a.x) * dx + (point.y - a.y) * dy) / denominator).clamp(0.0, 1.0);
    euclidean(
        point,
        Point2 {
            x: a.x + t * dx,
            y: a.y + t * dy,
        },
    )
}

fn euclidean(a: Point2, b: Point2) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn valid_point(point: Point2) -> bool {
    point.x.is_finite() && point.y.is_finite()
}

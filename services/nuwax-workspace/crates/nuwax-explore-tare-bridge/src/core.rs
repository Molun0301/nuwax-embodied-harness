//! Frontier clustering and information-gain waypoint selection.

/// Candidate frontier sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrontierPoint {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
    /// Information gain.
    pub gain: f64,
}

/// Exploration robot pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotPose {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
}

/// Selected exploration waypoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Centroid X.
    pub x: f64,
    /// Centroid Y.
    pub y: f64,
    /// Utility score.
    pub score: f64,
    /// Cluster sample count.
    pub support: usize,
}

/// TARE-equivalent fallback parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TareCoreConfig {
    /// Frontier clustering radius.
    pub cluster_radius: f64,
    /// Maximum exploration radius.
    pub exploration_range: f64,
    /// Travel-distance penalty.
    pub distance_weight: f64,
}

impl Default for TareCoreConfig {
    fn default() -> Self {
        Self {
            cluster_radius: 1.5,
            exploration_range: 20.0,
            distance_weight: 0.2,
        }
    }
}

/// Cluster reachable frontiers and choose the highest gain-distance utility centroid.
pub fn select_waypoint(
    robot: RobotPose,
    frontiers: &[FrontierPoint],
    config: TareCoreConfig,
) -> Option<Waypoint> {
    if ![
        robot.x,
        robot.y,
        config.cluster_radius,
        config.exploration_range,
        config.distance_weight,
    ]
    .into_iter()
    .all(f64::is_finite)
        || config.cluster_radius <= 0.0
        || config.exploration_range <= 0.0
        || config.distance_weight < 0.0
    {
        return None;
    }
    let valid: Vec<_> = frontiers
        .iter()
        .copied()
        .filter(|p| {
            [p.x, p.y, p.gain].into_iter().all(f64::is_finite)
                && p.gain >= 0.0
                && (p.x - robot.x).hypot(p.y - robot.y) <= config.exploration_range
        })
        .collect();
    let mut used = vec![false; valid.len()];
    let mut best: Option<Waypoint> = None;
    for seed in 0..valid.len() {
        if used[seed] {
            continue;
        }
        let mut stack = vec![seed];
        used[seed] = true;
        let mut cluster = Vec::new();
        while let Some(index) = stack.pop() {
            cluster.push(valid[index]);
            for next in 0..valid.len() {
                if !used[next]
                    && (valid[next].x - valid[index].x).hypot(valid[next].y - valid[index].y)
                        <= config.cluster_radius
                {
                    used[next] = true;
                    stack.push(next);
                }
            }
        }
        let support = cluster.len();
        let x = cluster.iter().map(|p| p.x).sum::<f64>() / support as f64;
        let y = cluster.iter().map(|p| p.y).sum::<f64>() / support as f64;
        let gain = cluster.iter().map(|p| p.gain).sum::<f64>();
        let score = gain - config.distance_weight * (x - robot.x).hypot(y - robot.y);
        let waypoint = Waypoint {
            x,
            y,
            score,
            support,
        };
        if best.is_none_or(|current| waypoint.score > current.score) {
            best = Some(waypoint);
        }
    }
    best
}

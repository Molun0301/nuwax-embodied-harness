//! 导航边界几何：多边形边界与路径相交判定。

use super::*;

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) const BOUNDARY_EPSILON: f64 = 1e-9;

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
#[derive(Debug, Clone, PartialEq)]
pub(super) struct NavigationBoundary {
    pub(super) vertices: Vec<Point2>,
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
impl NavigationBoundary {
    pub(super) fn new(points: &[[f64; 3]]) -> NavStackResult<Self> {
        let mut vertices: Vec<Point2> = points
            .iter()
            .map(|point| Point2::new(point[0], point[1]))
            .collect();
        if vertices
            .first()
            .zip(vertices.last())
            .is_some_and(|(first, last)| first.distance(*last) <= BOUNDARY_EPSILON)
        {
            vertices.pop();
        }
        if vertices.len() < 3 {
            return Err(NavStackError::InvalidConfig(
                "navigation boundary requires at least three distinct vertices".into(),
            ));
        }
        if vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .take(vertices.len())
            .any(|(left, right)| left.distance(*right) <= BOUNDARY_EPSILON)
        {
            return Err(NavStackError::InvalidConfig(
                "navigation boundary contains duplicate adjacent vertices".into(),
            ));
        }
        let signed_area_twice: f64 = vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .take(vertices.len())
            .map(|(left, right)| left.x * right.y - right.x * left.y)
            .sum();
        if signed_area_twice.abs() <= BOUNDARY_EPSILON {
            return Err(NavStackError::InvalidConfig(
                "navigation boundary polygon area must be non-zero".into(),
            ));
        }
        for left_index in 0..vertices.len() {
            let left_next = (left_index + 1) % vertices.len();
            for right_index in (left_index + 1)..vertices.len() {
                let right_next = (right_index + 1) % vertices.len();
                if left_index == right_index || left_next == right_index || right_next == left_index
                {
                    continue;
                }
                if segments_intersect(
                    vertices[left_index],
                    vertices[left_next],
                    vertices[right_index],
                    vertices[right_next],
                ) {
                    return Err(NavStackError::InvalidConfig(
                        "navigation boundary polygon must not self-intersect".into(),
                    ));
                }
            }
        }
        Ok(Self { vertices })
    }

    pub(super) fn contains(&self, point: Point2) -> bool {
        if self
            .edges()
            .any(|(start, end)| point_on_segment(point, start, end))
        {
            return true;
        }
        let mut inside = false;
        for (start, end) in self.edges() {
            if (start.y > point.y) != (end.y > point.y)
                && point.x < (end.x - start.x) * (point.y - start.y) / (end.y - start.y) + start.x
            {
                inside = !inside;
            }
        }
        inside
    }

    pub(super) fn segment_is_inside(&self, start: Point2, end: Point2, sample_step: f64) -> bool {
        if !self.contains(start) || !self.contains(end) {
            return false;
        }
        let mut parameters = vec![0.0, 1.0];
        for (edge_start, edge_end) in self.edges() {
            if let Some(parameter) =
                segment_intersection_parameter(start, end, edge_start, edge_end)
            {
                parameters.push(parameter.clamp(0.0, 1.0));
            }
        }
        parameters.sort_by(f64::total_cmp);
        parameters.dedup_by(|left, right| (*left - *right).abs() <= BOUNDARY_EPSILON);
        if parameters.windows(2).any(|window| {
            let midpoint = (window[0] + window[1]) * 0.5;
            !self.contains(start.lerp(end, midpoint))
        }) {
            return false;
        }

        let step = if sample_step.is_finite() && sample_step > 0.0 {
            sample_step
        } else {
            0.05
        };
        let samples = (start.distance(end) / step).ceil().max(1.0) as usize;
        (0..=samples).all(|index| self.contains(start.lerp(end, index as f64 / samples as f64)))
    }

    pub(super) fn edges(&self) -> impl Iterator<Item = (Point2, Point2)> + '_ {
        self.vertices
            .iter()
            .copied()
            .zip(self.vertices.iter().copied().cycle().skip(1))
            .take(self.vertices.len())
    }
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) fn navigation_boundary_from_goal(
    goal: &schema::NavigationGoal,
) -> NavStackResult<Option<NavigationBoundary>> {
    if goal.boundary_points.is_empty() {
        return Ok(None);
    }
    let boundary = NavigationBoundary::new(&goal.boundary_points)?;
    let point = Point2::new(goal.goal.pose.position[0], goal.goal.pose.position[1]);
    if !boundary.contains(point) {
        return Err(NavStackError::InvalidConfig("探索目标不在边界内".into()));
    }
    Ok(Some(boundary))
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar", feature = "cmu-native",
    feature = "explore", feature = "mls"))]
pub(super) fn path_respects_navigation_boundary(
    path: &Path2D,
    boundary: Option<&NavigationBoundary>,
    sample_step: f64,
) -> bool {
    let Some(boundary) = boundary else {
        return true;
    };
    !path.points.is_empty()
        && path
            .points
            .iter()
            .copied()
            .all(|point| boundary.contains(point))
        && path
            .points
            .windows(2)
            .all(|segment| boundary.segment_is_inside(segment[0], segment[1], sample_step))
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) fn point_on_segment(point: Point2, start: Point2, end: Point2) -> bool {
    let cross = cross_product(start, end, point).abs();
    let scale = start.distance(end).max(1.0);
    if cross > BOUNDARY_EPSILON * scale {
        return false;
    }
    point.x >= start.x.min(end.x) - BOUNDARY_EPSILON
        && point.x <= start.x.max(end.x) + BOUNDARY_EPSILON
        && point.y >= start.y.min(end.y) - BOUNDARY_EPSILON
        && point.y <= start.y.max(end.y) + BOUNDARY_EPSILON
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) fn cross_product(start: Point2, end: Point2, point: Point2) -> f64 {
    (end.x - start.x) * (point.y - start.y) - (end.y - start.y) * (point.x - start.x)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) fn segments_intersect(
    left_start: Point2,
    left_end: Point2,
    right_start: Point2,
    right_end: Point2,
) -> bool {
    let left_a = cross_product(left_start, left_end, right_start);
    let left_b = cross_product(left_start, left_end, right_end);
    let right_a = cross_product(right_start, right_end, left_start);
    let right_b = cross_product(right_start, right_end, left_end);
    if ((left_a > BOUNDARY_EPSILON && left_b < -BOUNDARY_EPSILON)
        || (left_a < -BOUNDARY_EPSILON && left_b > BOUNDARY_EPSILON))
        && ((right_a > BOUNDARY_EPSILON && right_b < -BOUNDARY_EPSILON)
            || (right_a < -BOUNDARY_EPSILON && right_b > BOUNDARY_EPSILON))
    {
        return true;
    }
    (left_a.abs() <= BOUNDARY_EPSILON && point_on_segment(right_start, left_start, left_end))
        || (left_b.abs() <= BOUNDARY_EPSILON && point_on_segment(right_end, left_start, left_end))
        || (right_a.abs() <= BOUNDARY_EPSILON
            && point_on_segment(left_start, right_start, right_end))
        || (right_b.abs() <= BOUNDARY_EPSILON && point_on_segment(left_end, right_start, right_end))
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) fn segment_intersection_parameter(
    start: Point2,
    end: Point2,
    edge_start: Point2,
    edge_end: Point2,
) -> Option<f64> {
    let segment = Point2::new(end.x - start.x, end.y - start.y);
    let edge = Point2::new(edge_end.x - edge_start.x, edge_end.y - edge_start.y);
    let denominator = segment.x * edge.y - segment.y * edge.x;
    if denominator.abs() <= BOUNDARY_EPSILON {
        return None;
    }
    let offset = Point2::new(edge_start.x - start.x, edge_start.y - start.y);
    let parameter = (offset.x * edge.y - offset.y * edge.x) / denominator;
    let edge_parameter = (offset.x * segment.y - offset.y * segment.x) / denominator;
    let segment_range = -BOUNDARY_EPSILON..=1.0 + BOUNDARY_EPSILON;
    (segment_range.contains(&parameter) && segment_range.contains(&edge_parameter))
        .then_some(parameter)
}

//! 匹配期间不变的三维最近邻索引，允许共面、共轴及重复坐标。

use nalgebra::Point3;

struct IndexedPoint {
    point: Point3<f64>,
    source_index: usize,
    axis: usize,
}

pub(crate) struct NearestPointIndex {
    points: Vec<IndexedPoint>,
}

impl NearestPointIndex {
    pub(crate) fn new(points: &[Point3<f64>]) -> Self {
        let mut indexed: Vec<_> = points
            .iter()
            .enumerate()
            .map(|(source_index, point)| IndexedPoint {
                point: point.map(|v| if v == 0.0 { 0.0 } else { v }),
                source_index,
                axis: 0,
            })
            .collect();
        indexed.sort_unstable_by(|a, b| {
            a.point
                .x
                .total_cmp(&b.point.x)
                .then(a.point.y.total_cmp(&b.point.y))
                .then(a.point.z.total_cmp(&b.point.z))
                .then(a.source_index.cmp(&b.source_index))
        });
        // 同一空间位置只保留最早索引，重复采样不增加匹配的独立几何证据。
        indexed.dedup_by(|a, b| a.point == b.point);
        build_balanced(&mut indexed);
        Self { points: indexed }
    }

    pub(crate) fn nearest(&self, point: &Point3<f64>) -> Option<(usize, f64)> {
        if point.coords.iter().any(|value| !value.is_finite()) {
            return None;
        }
        let mut best = None;
        search(&self.points, point, &mut best);
        best
    }
}

fn build_balanced(points: &mut [IndexedPoint]) {
    if points.is_empty() {
        return;
    }
    let mut minimum = points[0].point;
    let mut maximum = minimum;
    for point in points.iter() {
        for axis in 0..3 {
            minimum[axis] = minimum[axis].min(point.point[axis]);
            maximum[axis] = maximum[axis].max(point.point[axis]);
        }
    }
    let axis = (0..3)
        .max_by(|a, b| (maximum[*a] - minimum[*a]).total_cmp(&(maximum[*b] - minimum[*b])))
        .unwrap_or(0);
    let middle = points.len() / 2;
    points.select_nth_unstable_by(middle, |a, b| {
        a.point[axis]
            .total_cmp(&b.point[axis])
            .then(a.source_index.cmp(&b.source_index))
    });
    let (left, rest) = points.split_at_mut(middle);
    let (node, right) = rest.split_first_mut().expect("非空分区包含中点");
    node.axis = axis;
    // 按元素中位数分区，即使分割坐标相同也保证递归规模减半。
    build_balanced(left);
    build_balanced(right);
}

fn search(points: &[IndexedPoint], query: &Point3<f64>, best: &mut Option<(usize, f64)>) {
    if points.is_empty() {
        return;
    }
    let middle = points.len() / 2;
    let node = &points[middle];
    let distance_squared = (node.point - query).norm_squared();
    if distance_squared.is_finite()
        && best.is_none_or(|(index, distance)| {
            distance_squared < distance
                || (distance_squared == distance && node.source_index < index)
        })
    {
        *best = Some((node.source_index, distance_squared));
    }
    let split_delta = query[node.axis] - node.point[node.axis];
    let (near, far) = if split_delta <= 0.0 {
        (&points[..middle], &points[middle + 1..])
    } else {
        (&points[middle + 1..], &points[..middle])
    };
    search(near, query, best);
    // 分割平面另一侧仍可能有更近点；等距时也搜索以稳定选择最早原始索引。
    if split_delta * split_delta <= best.map_or(f64::INFINITY, |(_, distance)| distance) {
        search(far, query, best);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn balanced_index_matches_exhaustive_search_for_degenerate_and_spatial_clouds() {
        let mut points: Vec<_> = (0..257)
            .map(|i| {
                Point3::new(
                    ((i * 37) % 211) as f64 * 0.03,
                    ((i * 53) % 173) as f64 * 0.04,
                    ((i * 71) % 137) as f64 * 0.05,
                )
            })
            .collect();
        points.extend((0..128).map(|i| Point3::new(1.0, i as f64 * 0.02, 0.0)));
        points.extend(vec![Point3::origin(); 300]);
        let index = NearestPointIndex::new(&points);
        for query in points.iter().copied().chain((0..200).map(|i| {
            Point3::new(
                ((i * 13) % 119) as f64 * 0.08 - 1.0,
                ((i * 17) % 97) as f64 * 0.1 - 2.0,
                ((i * 19) % 79) as f64 * 0.1 - 1.0,
            )
        })) {
            let expected = points
                .iter()
                .enumerate()
                .map(|(i, p)| (i, (p - query).norm_squared()))
                .min_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
            assert_eq!(index.nearest(&query), expected);
        }
        assert!(NearestPointIndex::new(&[])
            .nearest(&Point3::origin())
            .is_none());
    }
}

use crate::{distance, inside_boundary, Tomogram};
use std::{
    cmp::Ordering,
    collections::BinaryHeap,
    sync::atomic::{AtomicBool, Ordering as AtomicOrdering},
};
#[derive(Clone, Debug, Default)]
pub struct SearchStats {
    pub expanded: usize,
    pub elapsed_ms: f64,
    pub cells: usize,
}
#[derive(Clone, Debug)]
pub struct Route {
    pub points: Vec<[f64; 3]>,
    pub stats: SearchStats,
}
#[derive(Clone, Copy, PartialEq)]
struct Queue {
    score: f64,
    cost: f64,
    id: usize,
    order: [i32; 3],
}
impl Eq for Queue {}
impl PartialOrd for Queue {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Queue {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .score
            .total_cmp(&self.score)
            .then_with(|| other.order.cmp(&self.order))
    }
}
pub(crate) fn checkpoint(cancelled: Option<&AtomicBool>) -> Result<(), String> {
    if cancelled.is_some_and(|flag| flag.load(AtomicOrdering::Acquire)) {
        return Err("层析规划已取消".into());
    }
    crate::cancellation::checkpoint()
}
impl Tomogram {
    /// 起终点为支撑坐标；容差仅用于选择原目标球内的真实地面点。
    pub fn plan(
        &self,
        start: [f64; 3],
        goal: [f64; 3],
        tolerance: f64,
        boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>,
    ) -> Result<Route, String> {
        self.plan_bounded(start, goal, tolerance, boundary, cancelled, None, &[])
    }
    /// 局部绕障限制在机器人附近；全局调用不设置空间范围。
    pub fn plan_bounded(
        &self,
        start: [f64; 3],
        goal: [f64; 3],
        tolerance: f64,
        boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>,
        radius: Option<f64>,
        penalties: &[crate::RoutePenalty],
    ) -> Result<Route, String> {
        checkpoint(cancelled)?;
        let result = self.plan_bounded_impl(start, goal, tolerance, boundary, cancelled, radius, penalties, &|_, _| true);
        checkpoint(cancelled)?;
        result
    }
    /// Request-local guide restrictions without rebuilding or mutating the static tomogram.
    /// The predicate must also accept shortcuts and actual start/end connectors.
    /// This is reference geometry only; it does not grant executable body clearance.
    pub fn plan_bounded_with_segment_filter(
        &self, start: [f64; 3], goal: [f64; 3], tolerance: f64,
        boundary: &[[f64; 2]], cancelled: Option<&AtomicBool>, radius: Option<f64>,
        penalties: &[crate::RoutePenalty], accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
    ) -> Result<Route, String> {
        checkpoint(cancelled)?;
        let result = self.plan_bounded_impl(start, goal, tolerance, boundary, cancelled, radius, penalties, accepts);
        checkpoint(cancelled)?;
        result
    }
    /// Explicit token convenience API; the legacy AtomicBool and scoped tokens both apply.
    pub fn plan_bounded_cancellable(
        &self, start: [f64; 3], goal: [f64; 3], tolerance: f64,
        boundary: &[[f64; 2]], cancelled: Option<&AtomicBool>, radius: Option<f64>,
        penalties: &[crate::RoutePenalty], cancellation: &crate::PlanningCancellation,
    ) -> Result<Route, String> {
        crate::with_planning_cancellation(cancellation, ||
            self.plan_bounded(start, goal, tolerance, boundary, cancelled, radius, penalties))
    }
    fn plan_bounded_impl(
        &self,
        start: [f64; 3],
        goal: [f64; 3],
        tolerance: f64,
        boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>,
        radius: Option<f64>,
        penalties: &[crate::RoutePenalty],
        accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
    ) -> Result<Route, String> {
        let began = std::time::Instant::now();
        if !start.into_iter().chain(goal).all(f64::is_finite)
            || !tolerance.is_finite()
            || tolerance < 0.0
        {
            return Err("层析起终点或到达容差无效".into());
        }
        if !inside_boundary([start[0], start[1]], boundary) {
            return Err("层析起点不在任务边界内".into());
        }
        let from = self
            .cell_at(start)
            .or_else(|| self.nearest(start, self.resolution * 2.0))
            .ok_or("当前位姿附近没有层析地面")?;
        let mut targets = Vec::new();
        if let Some(id) = self.cell_at(goal) {
            let point = [goal[0], goal[1], self.cells[id].ground];
            if distance(point, goal) <= tolerance + 1e-6
                && inside_boundary([point[0], point[1]], boundary)
            {
                targets.push((id, point));
            }
        }
        let [gx, gy] = [
            (goal[0] / self.resolution).floor() as i32,
            (goal[1] / self.resolution).floor() as i32,
        ];
        let r = (tolerance / self.resolution).ceil() as i32;
        for x in gx - r..=gx + r {
            checkpoint(cancelled)?;
            for y in gy - r..=gy + r {
                checkpoint(cancelled)?;
                if let Some(ids) = self.columns.get(&[x, y]) {
                    for &id in ids {
                        checkpoint(cancelled)?;
                        let point = self.position(id);
                        if distance(point, goal) <= tolerance + 1e-6
                            && inside_boundary([point[0], point[1]], boundary)
                        {
                            targets.push((id, point));
                        }
                    }
                }
            }
        }
        checkpoint(cancelled)?;
        targets.sort_unstable_by(|a, b| {
            distance(a.1, goal)
                .total_cmp(&distance(b.1, goal))
                .then_with(|| a.0.cmp(&b.0))
        });
        checkpoint(cancelled)?;
        if targets.is_empty() {
            return Err("原目标到达范围内没有层析地面".into());
        }
        let mut ends = ahash::AHashMap::new();
        for (id, p) in targets {
            checkpoint(cancelled)?;
            ends.entry(id).or_insert(p);
        }
        let heuristic = |id| (distance(self.position(id), goal) - tolerance).max(0.0);
        let order = |id: usize| {
            let cell = &self.cells[id];
            [cell.column[0], cell.column[1], cell.layer]
        };
        let mut costs = vec![f64::INFINITY; self.cells.len()];
        let mut parents = vec![usize::MAX; self.cells.len()];
        let mut queue = BinaryHeap::new();
        costs[from] = 0.0;
        queue.push(Queue {
            score: heuristic(from),
            cost: 0.0,
            id: from,
            order: order(from),
        });
        let mut expanded = 0;
        let mut best: Option<(usize, [f64; 3], f64)> = None;
        while let Some(Queue {
            cost, id, score, ..
        }) = queue.pop()
        {
            checkpoint(cancelled)?;
            if cost > costs[id] {
                continue;
            }
            if best.is_some_and(|(_, _, total)| score >= total) {
                break;
            }
            expanded += 1;
            if let Some(&endpoint) = ends.get(&id).filter(|&&endpoint|
                accepts(if id == from { start } else { self.position(id) }, endpoint)) {
                // 终端代价偏好到达区域内部，避免每次都停在容差球的数值边缘。
                let total = cost + 2.0 * distance(endpoint, goal);
                if best.is_none_or(|(_, _, old)| total < old) {
                    best = Some((id, endpoint, total));
                }
            }
            let a = self.position(id);
            for &next in &self.cells[id].neighbors {
                checkpoint(cancelled)?;
                let b = self.position(next);
                if !crate::segment_inside_boundary([a[0], a[1]], [b[0], b[1]], boundary)
                    || radius.is_some_and(|r| (b[0] - start[0]).hypot(b[1] - start[1]) > r)
                    || !accepts(if id == from { start } else { a }, b)
                {
                    continue;
                }
                let local_cost = penalties
                    .iter()
                    .map(|p| {
                        let d = distance(b, p.center);
                        if d < p.radius {
                            p.weight * (1.0 - d / p.radius).powi(2)
                        } else {
                            0.0
                        }
                    })
                    .sum::<f64>();
                let next_cost = cost
                    + distance(a, b)
                        * ((self.cells[id].cost + self.cells[next].cost) * 0.5 + local_cost);
                if next_cost < costs[next] {
                    costs[next] = next_cost;
                    parents[next] = id;
                    queue.push(Queue {
                        score: next_cost + heuristic(next),
                        cost: next_cost,
                        id: next,
                        order: order(next),
                    });
                }
            }
        }
        let Some((id, endpoint, _)) = best else {
            return Err(format!("层析地面之间没有可连接路线（展开{expanded}格）"));
        };
        let mut ids = vec![id];
        let mut current = id;
        while current != from {
            checkpoint(cancelled)?;
            current = parents[current];
            ids.push(current);
        }
        ids.reverse();
        let mut points = Vec::with_capacity(ids.len() + 2);
        points.push(start);
        for id in ids.into_iter().skip(1) {
            checkpoint(cancelled)?;
            let p = self.position(id);
            if distance(*points.last().unwrap(), p) > 1e-6 {
                points.push(p);
            }
        }
        if distance(*points.last().unwrap(), endpoint) > 1e-6 {
            points.push(endpoint);
        }
        let mut stats = SearchStats {
            expanded,
            elapsed_ms: 0.0,
            cells: self.cells.len(),
        };
        let points = self.reduce(points, boundary, penalties, cancelled, accepts)?;
        stats.elapsed_ms = began.elapsed().as_secs_f64() * 1000.0;
        Ok(Route { points, stats })
    }

    fn segment_cost(
        &self,
        a: [f64; 3],
        b: [f64; 3],
        boundary: &[[f64; 2]],
        penalties: &[crate::RoutePenalty],
        cancelled: Option<&AtomicBool>,
        accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
    ) -> Option<f64> {
        if !accepts(a, b) || !crate::segment_inside_boundary([a[0], a[1]], [b[0], b[1]], boundary)
            || !self.segment_clear_cancelled(a, b, boundary, cancelled)
        {
            return None;
        }
        let length = distance(a, b);
        let n = (length / (self.resolution * 0.5)).ceil().max(1.0) as usize;
        let mut sum = 0.0;
        for i in 0..n {
            checkpoint(cancelled).ok()?;
            let t = (i as f64 + 0.5) / n as f64;
            let point = std::array::from_fn(|j| a[j] + t * (b[j] - a[j]));
            let id = self.cell_at(point)?;
            let penalty = penalties
                .iter()
                .map(|p| {
                    let d = distance(point, p.center);
                    if d < p.radius {
                        p.weight * (1.0 - d / p.radius).powi(2)
                    } else {
                        0.0
                    }
                })
                .sum::<f64>();
            sum += (self.cells[id].cost + penalty) * length / n as f64;
        }
        Some(sum)
    }
    fn reduce(
        &self,
        points: Vec<[f64; 3]>,
        boundary: &[[f64; 2]],
        penalties: &[crate::RoutePenalty],
        cancelled: Option<&AtomicBool>,
        accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
    ) -> Result<Vec<[f64; 3]>, String> {
        checkpoint(cancelled)?;
        if points.len() < 3 {
            return Ok(points);
        }
        let mut prefix = vec![0.0];
        for pair in points.windows(2) {
            checkpoint(cancelled)?;
            let cost = self
                .segment_cost(pair[0], pair[1], boundary, penalties, cancelled, accepts)
                .unwrap_or_else(|| distance(pair[0], pair[1]));
            prefix.push(prefix.last().unwrap() + cost);
        }
        let mut output = vec![points[0]];
        let mut anchor = 0;
        while anchor + 1 < points.len() {
            checkpoint(cancelled)?;
            let mut far = anchor + 1;
            while far + 1 < points.len() && distance(points[anchor], points[far + 1]) <= 3.0 {
                checkpoint(cancelled)?;
                far += 1;
            }
            let mut chosen = anchor + 1;
            for next in (anchor + 2..=far).rev() {
                checkpoint(cancelled)?;
                if self
                    .segment_cost(points[anchor], points[next], boundary, penalties, cancelled, accepts)
                    .is_some_and(|cost| cost <= (prefix[next] - prefix[anchor]) * 1.1 + 1e-6)
                {
                    chosen = next;
                    break;
                }
            }
            output.push(points[chosen]);
            anchor = chosen;
        }
        checkpoint(cancelled)?;
        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn map(hole: bool) -> Tomogram {
        let columns = (-15..100).flat_map(|x| {
            (-15..16).filter_map(move |y| {
                if hole && x == 40 {
                    None
                } else {
                    Some(([x, y], vec![[-0.1, 0.0], [1.9, 2.0]]))
                }
            })
        });
        Tomogram::from_columns(0.1, columns, 0.5, 0.15, 0.2, 0.2, 20.0).unwrap()
    }
    #[test]
    fn request_segment_filter_keeps_static_map_and_survives_reduction() {
        let map = map(false);
        let start = [0.05,0.05,0.0]; let goal = [2.05,0.05,0.0];
        let before: Vec<_> = map.cells.iter().map(|cell|
            (cell.column,cell.ground,cell.cost,cell.neighbors.clone())).collect();
        let accepts = |a:[f64;3],b:[f64;3]| {
            // A finite wall crossing x=.8, including endpoint contact.
            if a[0].min(b[0])<=0.8 && a[0].max(b[0])>=0.8 {
                let t = (0.8-a[0])/(b[0]-a[0]);
                (a[1]+t*(b[1]-a[1])).abs()>0.35
            } else { true }
        };
        let plain = map.plan(start,goal,0.01,&[],None).unwrap();
        assert!(plain.points.windows(2).any(|p|!accepts(p[0],p[1])));
        let route = map.plan_bounded_with_segment_filter(start,goal,0.01,&[],None,Some(3.),&[],&accepts).unwrap();
        assert!(route.points.windows(2).all(|p|accepts(p[0],p[1])));
        assert!(route.points.iter().any(|p|p[1].abs()>0.35));
        assert!(map.plan_bounded_with_segment_filter(start,goal,0.01,&[],None,Some(3.),&[],
            &|a,_|a!=start).is_err(),"actual start connector must obey the overlay");
        assert!(map.plan_bounded_with_segment_filter(start,goal,0.0,&[],None,Some(3.),&[],
            &|_,b|b!=goal).is_err(),"actual goal connector must obey the overlay");
        let after: Vec<_> = map.cells.iter().map(|cell|
            (cell.column,cell.ground,cell.cost,cell.neighbors.clone())).collect();
        assert_eq!(before,after);
        assert_eq!(plain.points,map.plan(start,goal,0.01,&[],None).unwrap().points);
    }
    #[test]
    fn epoch_cancellation_interrupts_search_and_reduction_without_changing_the_map() {
        let map = map(false);
        let before: Vec<_> = map.cells.iter().map(|cell|
            (cell.column, cell.ground, cell.cost, cell.neighbors.clone())).collect();
        for checkpoints in [0, 20, 80] {
            let error = crate::cancellation::after_checkpoints(checkpoints, ||
                map.plan([0.05, 0.05, 0.0], [8.05, 0.05, 0.0], 0.01, &[], None)).unwrap_err();
            assert!(error.contains("取消"), "{error}");
        }
        let points: Vec<_> = (0..80).map(|x| [f64::from(x) * 0.1 + 0.05, 0.05, 0.0]).collect();
        let error = crate::cancellation::after_checkpoints(12, ||
            map.reduce(points, &[], &[], None, &|_, _| true)).unwrap_err();
        assert!(error.contains("取消"));
        let after: Vec<_> = map.cells.iter().map(|cell|
            (cell.column, cell.ground, cell.cost, cell.neighbors.clone())).collect();
        assert_eq!(before, after);
        let baseline = map.plan([0.05, 0.05, 0.0], [8.05, 0.05, 0.0], 0.01, &[], None).unwrap();
        let token = crate::PlanningCancellation::new(std::sync::Arc::new(std::sync::atomic::AtomicU64::new(9)), 9);
        let scoped = map.plan_bounded_cancellable([0.05, 0.05, 0.0], [8.05, 0.05, 0.0],
            0.01, &[], None, None, &[], &token).unwrap();
        assert_eq!(baseline.points, scoped.points);
        assert_eq!(baseline.stats.expanded, scoped.stats.expanded);
    }
    #[test]
    fn explicit_occupied_volume_stairs_keep_the_exposed_step_heights() {
        let columns=||(-20..100).flat_map(|x|(-8..9).map(move|y| {
            let height=if x<0 {0.0}else{((x/10)+1).min(8) as f64*0.1};
            ([x,y],vec![[-0.05,height]])
        }));
        let solid=Tomogram::from_columns_with_evidence(0.05,0.05,columns(),0.5,0.16,0.15,0.15,18.0,0.34,true).unwrap();
        let start=[-0.5,0.025,0.0];let goal=[4.75,0.025,0.8];
        assert!(solid.plan(start,goal,0.05,&[],None).is_ok());
        assert!(solid.plan(goal,start,0.05,&[],None).is_ok());
        assert!(solid.cells.iter().all(|c|c.ground==c.ground_interval[1]),"实体内部不能生成更低的名义地面");
    }
    #[test]
    fn overlapping_floors_do_not_connect() {
        let map = map(false);
        assert!(map
            .plan([0.05, 0.05, 0.0], [8.05, 0.05, 0.0], 0.05, &[], None)
            .is_ok());
        assert!(map
            .plan([0.05, 0.05, 2.0], [8.05, 0.05, 2.0], 0.05, &[], None)
            .is_ok());
        assert!(map
            .plan([0.05, 0.05, 0.0], [8.05, 0.05, 2.0], 0.05, &[], None)
            .is_err());
    }
    #[test]
    fn missing_ground_is_not_interpolated_and_cancellation_is_observed() {
        assert!(map(true)
            .plan([0.05, 0.05, 0.0], [8.05, 0.05, 0.0], 0.05, &[], None)
            .is_err());
        assert!(map(false)
            .plan(
                [0.05, 0.05, 0.0],
                [8.05, 0.05, 0.0],
                0.05,
                &[],
                Some(&AtomicBool::new(true))
            )
            .unwrap_err()
            .contains("取消"));
    }
    #[test]
    fn sloping_ground_crosses_slice_boundaries() {
        let columns = (0..120).flat_map(|x| {
            (-5..6).map(move |y| {
                (
                    [x, y],
                    vec![[f64::from(x) * 0.02 - 0.1, f64::from(x) * 0.02]],
                )
            })
        });
        let map = Tomogram::from_columns(0.1, columns, 0.5, 0.15, 0.15, 0.15, 20.0).unwrap();
        let path = map
            .plan([0.05, 0.05, 0.0], [11.05, 0.05, 2.2], 0.05, &[], None)
            .unwrap();
        assert!(path.points.last().unwrap()[2] > 2.0);
        assert!(path.stats.expanded < map.cells.len());
    }
    #[test]
    fn reference_shortcuts_keep_a_concave_task_boundary() {
        let map = Tomogram::from_columns(
            0.1,
            (0..40).flat_map(|x| (0..40).map(move |y| ([x, y], vec![[-0.1, 0.0]]))),
            0.3,
            0.1,
            0.1,
            0.1,
            20.0,
        )
        .unwrap();
        let boundary = [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 1.0],
            [1.0, 1.0],
            [1.0, 4.0],
            [0.0, 4.0],
        ];
        let route = map
            .plan([0.5, 3.5, 0.0], [3.5, 0.5, 0.0], 0.01, &boundary, None)
            .unwrap();
        for pair in route.points.windows(2) {
            for i in 0..=100 {
                let t = i as f64 / 100.0;
                let x = pair[0][0] + t * (pair[1][0] - pair[0][0]);
                let y = pair[0][1] + t * (pair[1][1] - pair[0][1]);
                assert!(
                    x <= 1.0 + 1e-9 || y <= 1.0 + 1e-9,
                    "全局参考不能穿过L形边界的凹角"
                );
            }
        }
    }
    #[test]
    fn rough_edge_prefers_body_room_without_blocking_a_required_step() {
        let columns = || {
            (0..55).flat_map(|x| {
                (-15..16).map(move |y| {
                    let z = if (20..=30).contains(&x) && (0..=3).contains(&y) {
                        0.15
                    } else {
                        0.0
                    };
                    ([x, y], vec![[z - 0.05, z]])
                })
            })
        };
        let map = Tomogram::from_columns(0.1, columns(), 0.5, 0.3, 0.2, 0.2, 20.0).unwrap();
        let smaller = Tomogram::from_columns(0.1, columns(), 0.5, 0.05, 0.2, 0.2, 20.0).unwrap();
        let edge_clearance = |map: &Tomogram| {
            let route = map
                .plan([0.55, -0.05, 0.0], [4.55, -0.05, 0.0], 0.02, &[], None)
                .unwrap();
            let mut clearance = f64::INFINITY;
            for pair in route.points.windows(2) {
                for i in 0..=100 {
                    let t = i as f64 / 100.0;
                    let x = pair[0][0] + t * (pair[1][0] - pair[0][0]);
                    let y = pair[0][1] + t * (pair[1][1] - pair[0][1]);
                    if (2.0..=3.1).contains(&x) {
                        clearance = clearance.min(-y);
                    }
                }
            }
            clearance
        };
        let large_room = edge_clearance(&map);
        let small_room = edge_clearance(&smaller);
        assert!(
            large_room > small_room + 0.05,
            "相同阶边应随机体尺寸留出更多空间，大机体{large_room}米，小机体{small_room}米"
        );
        // 同一地图以任务边界要求跨过台阶，软代价不能变成新的通行门槛。
        let boundary = [[0.0, 0.0], [5.5, 0.0], [5.5, 0.4], [0.0, 0.4]];
        assert!(map
            .plan([0.55, 0.15, 0.0], [4.55, 0.15, 0.0], 0.02, &boundary, None)
            .is_ok());
    }

    #[test]
    fn insufficient_ceiling_and_one_way_steps() {
        let columns = (0..50).flat_map(|x| {
            (-5..6).map(move |y| {
                let z = if x >= 25 { 0.15 } else { 0.0 };
                ([x, y], vec![[z - 0.05, z]])
            })
        });
        let map = Tomogram::from_columns(0.1, columns, 0.5, 0.15, 0.2, 0.05, 10.0).unwrap();
        assert!(map
            .plan([0.5, 0.05, 0.0], [4.5, 0.05, 0.15], 0.05, &[], None)
            .is_ok());
        assert!(map
            .plan([4.5, 0.05, 0.15], [0.5, 0.05, 0.0], 0.05, &[], None)
            .is_err());
        let map = Tomogram::from_columns(
            0.1,
            [([0, 0], vec![[-0.1, 0.0], [0.2, 0.3]])],
            0.5,
            0.15,
            0.2,
            0.2,
            20.0,
        )
        .unwrap();
        assert!(map.cells.iter().all(|c| c.ground >= 0.3));
    }
}

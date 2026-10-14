use crate::{distance, inside_boundary, Tomogram, PlanningFailure};
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
            .then_with(|| other.id.cmp(&self.id))
    }
}
pub(crate) fn checkpoint(cancelled: Option<&AtomicBool>) -> Result<(), String> {
    if cancelled.is_some_and(|flag| flag.load(AtomicOrdering::Acquire)) {
        return Err("层析规划已取消".into());
    }
    crate::cancellation::checkpoint()
}
/// A resource bound, not a terrain permission threshold. The default permits
/// 65,536 lazily admitted states and expansions, including domain-edge revisits;
/// callers may explicitly raise it and retry.
/// Zero permits only a validated direct connection, with no A* state expansion.
#[derive(Clone, Copy, Debug)]
pub struct SearchLimits { pub max_states: usize }
impl Default for SearchLimits { fn default() -> Self { Self { max_states: 65_536 } } }
struct SearchState { reference: crate::reference::Reference, cost: f64, parent: usize }
fn penalty_at(point: [f64; 3], p: &crate::RoutePenalty) -> f64 {
    let d = distance(point, p.center);
    if d < p.radius { p.weight * (1.0 - d / p.radius).powi(2) } else { 0.0 }
}
/// Finite initial domain with progressive growth; hard user/local boundaries do
/// not grow. The padding is derived from the physical footprint, not resolution
/// changes or an unrelated route-shortcut constant.
struct SearchDomain { lo: [i32; 2], hi: [i32; 2], margin: i32, hard: bool, cropped: Option<[[i32; 2]; 2]> }
impl SearchDomain {
    fn new(map: &Tomogram, start: [f64; 3], goal: [f64; 3], boundary: &[[f64; 2]], radius: Option<f64>) -> Self {
        let a = map.point_column(start); let b = map.point_column(goal);
        let mut lo = [a[0].min(b[0]), a[1].min(b[1])];
        let mut hi = [a[0].max(b[0]), a[1].max(b[1])];
        if let Some([mlo, mhi]) = map.xy_bounds { for axis in 0..2 { lo[axis] = lo[axis].min(mlo[axis]); hi[axis] = hi[axis].max(mhi[axis]); } }
        let margin = (map.clearance_radius.max(map.half_width).max(map.resolution) / map.resolution).ceil() as i32;
        let mut result = Self { lo, hi, margin, hard: radius.is_some() || !boundary.is_empty(), cropped: None };
        if !boundary.is_empty() {
            for axis in 0..2 {
                result.lo[axis] = (boundary.iter().map(|p|p[axis]).fold(f64::INFINITY,f64::min) / map.resolution).floor() as i32;
                result.hi[axis] = (boundary.iter().map(|p|p[axis]).fold(f64::NEG_INFINITY,f64::max) / map.resolution).floor() as i32;
            }
        } else if let Some(radius) = radius {
            for axis in 0..2 { result.lo[axis] = ((start[axis]-radius)/map.resolution).floor() as i32;
                result.hi[axis] = ((start[axis]+radius)/map.resolution).floor() as i32; }
        } else { result.expand(); }
        if let Some((center, window_radius)) = map.window {
            let usable = (window_radius - map.half_width - map.resolution * 0.5).max(0.0);
            let lo = center.map(|v| ((v-usable)/map.resolution-0.5).ceil() as i32);
            let hi = center.map(|v| ((v+usable)/map.resolution-0.5).floor() as i32);
            if boundary.is_empty() && radius.is_none() {
                result.lo = lo; result.hi = hi;
            } else {
                for axis in 0..2 { result.lo[axis] = result.lo[axis].max(lo[axis]); result.hi[axis] = result.hi[axis].min(hi[axis]); }
            }
            result.cropped = Some([lo,hi]); result.hard = true;
        }
        result
    }
    fn contains(&self, column: [i32; 2]) -> bool { (0..2).all(|i| column[i] >= self.lo[i] && column[i] <= self.hi[i]) }
    fn inconclusive_edge(&self, column: [i32; 2]) -> bool {
        !self.hard || self.cropped.is_some_and(|[lo,hi]|
            (0..2).any(|axis| column[axis] < lo[axis] || column[axis] > hi[axis]))
    }
    fn expand(&mut self) { for axis in 0..2 { self.lo[axis] = self.lo[axis].saturating_sub(self.margin); self.hi[axis] = self.hi[axis].saturating_add(self.margin); } }
    fn grow(&mut self) -> bool {
        if self.hard { return false; }
        let old = (self.lo, self.hi);
        self.margin = self.margin.saturating_mul(2); self.expand();
        old != (self.lo, self.hi)
    }
}
impl Tomogram {
    /// 起终点为支撑参考坐标；缺测沿当前层参考延续，容差仍限定原三维目标。
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
    /// 局部绕障限制在机器人附近；全局请求使用渐进扩张的有限搜索域。
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
        let result = self.plan_bounded_impl(start, goal, tolerance, boundary, cancelled, radius, penalties, &|_, _| true, SearchLimits::default());
        checkpoint(cancelled)?;
        result.map_err(|error|error.to_string())
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
        let result = self.plan_bounded_impl(start, goal, tolerance, boundary, cancelled, radius, penalties, accepts, SearchLimits::default());
        checkpoint(cancelled)?;
        result.map_err(|error|error.to_string())
    }
    /// Bounded filtered queries share the exact same admission as global routes.
    /// Useful for candidate batches with a total owner-controlled work budget.
    pub fn plan_bounded_with_segment_filter_and_limits(
        &self, start: [f64; 3], goal: [f64; 3], tolerance: f64,
        boundary: &[[f64; 2]], cancelled: Option<&AtomicBool>, radius: Option<f64>,
        penalties: &[crate::RoutePenalty], accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
        limits: SearchLimits,
    ) -> Result<Route, String> {
        checkpoint(cancelled)?;
        let result = self.plan_bounded_impl(start, goal, tolerance, boundary, cancelled, radius,
            penalties, accepts, limits);
        checkpoint(cancelled)?;
        result.map_err(|error|error.to_string())
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
    /// Explicit per-request resource control. Exhaustion is inconclusive and
    /// callers can retry with a larger budget; it never labels unknown as blocked.
    pub fn plan_bounded_with_limits(
        &self, start: [f64; 3], goal: [f64; 3], tolerance: f64,
        boundary: &[[f64; 2]], cancelled: Option<&AtomicBool>, radius: Option<f64>,
        penalties: &[crate::RoutePenalty], limits: SearchLimits,
    ) -> Result<Route, String> {
        checkpoint(cancelled)?;
        let result = self.plan_bounded_impl(start, goal, tolerance, boundary, cancelled, radius,
            penalties, &|_, _| true, limits);
        checkpoint(cancelled)?;
        result.map_err(|error|error.to_string())
    }
    /// Typed producer outcome for request owners that distinguish resource
    /// limits from geometric/input failures without parsing diagnostic wording.
    pub fn plan_bounded_with_limits_typed(
        &self, start: [f64;3], goal: [f64;3], tolerance: f64,
        boundary: &[[f64;2]], cancelled: Option<&AtomicBool>, radius: Option<f64>,
        penalties: &[crate::RoutePenalty], limits: SearchLimits,
    ) -> Result<Route,PlanningFailure> {
        self.plan_bounded_with_segment_filter_and_limits_typed(start,goal,tolerance,boundary,
            cancelled,radius,penalties,&|_,_|true,limits)
    }
    /// Typed equivalent of the filtered bounded compatibility query.
    pub fn plan_bounded_with_segment_filter_and_limits_typed(
        &self, start: [f64;3], goal: [f64;3], tolerance: f64,
        boundary: &[[f64;2]], cancelled: Option<&AtomicBool>, radius: Option<f64>,
        penalties: &[crate::RoutePenalty], accepts: &dyn Fn([f64;3],[f64;3])->bool,
        limits: SearchLimits,
    ) -> Result<Route,PlanningFailure> {
        checkpoint(cancelled)?;
        let result=self.plan_bounded_impl(start,goal,tolerance,boundary,cancelled,radius,penalties,accepts,limits);
        checkpoint(cancelled)?;
        result
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
        limits: SearchLimits,
    ) -> Result<Route, PlanningFailure> {
        let began = std::time::Instant::now();
        if !start.into_iter().chain(goal).all(f64::is_finite)
            || !tolerance.is_finite() || tolerance < 0.0
            || radius.is_some_and(|r| !r.is_finite() || r <= 0.0)
            || boundary.iter().flatten().any(|v| !v.is_finite()) {
            return Err("层析起终点、范围或到达容差无效".into());
        }
        if !inside_boundary([start[0], start[1]], boundary) {
            return Err("层析起点不在任务边界内".into());
        }
        if !self.reference_inside_window([start[0],start[1]]) {
            return Err(PlanningFailure::SearchDomainLimited("起点超出本次地图物化范围；未证明物理无路".into()));
        }
        let initial = self.reference_start(start).ok_or("层析起点存在已知障碍或不兼容地形")?;
        let goal_column = self.point_column(goal);
        let initial_height = self.reference_position(initial)[2];
        let [low, high] = self.ground_bounds.unwrap_or([initial_height; 2]);
        if goal[2] + tolerance + 1e-6 < low.min(initial_height)
            || goal[2] - tolerance - 1e-6 > high.max(initial_height) {
            // Every reachable height is either an observed cell reference or
            // inherited from one/start. No unknown node can invent a new height.
            // A crop can conclude only that its own reference domain is limited.
            return Err(if self.window.is_some() {
                PlanningFailure::SearchDomainLimited("本次地图物化范围未覆盖目标高程参考；范围外地形尚未检查".into())
            } else {
                PlanningFailure::Other("UnsupportedReferenceHeight: 当前标定及已知地形没有覆盖目标高程；缺测不能生成新楼层".into())
            });
        }
        // Unknown endpoints inherit the current reference. The requested goal z
        // remains an arrival constraint, never evidence for a second floor.
        let direct_height = self.cell_at(goal).map(|id| self.cells[id].ground).unwrap_or(initial_height);
        let direct = [goal[0], goal[1], direct_height];
        if distance(direct, goal) <= tolerance + 1e-6
            && radius.is_none_or(|r| (direct[0] - start[0]).hypot(direct[1] - start[1]) <= r)
            && penalties.is_empty() && accepts(start, direct)
            && self.reference_segment_evidence(start, direct, boundary, cancelled)
                .is_some_and(|(cost, observed)| limits.max_states == 0 || !observed || cost <= distance(start, direct) + 1e-6) {
            checkpoint(cancelled)?;
            return Ok(Route { points: if distance(start, direct) <= 1e-6 { vec![start] } else { vec![start, direct] },
                stats: SearchStats { expanded: 0, elapsed_ms: began.elapsed().as_secs_f64() * 1000.0,
                    cells: self.cells.len() } });
        }
        if limits.max_states == 0 {
            return Err(PlanningFailure::BudgetExhausted("快速检查仅验证直连接续，绕行或局部代价需要完整搜索；未证明无路".into()));
        }
        let mut domain = SearchDomain::new(self, start, goal, boundary, radius);
        let order = |reference| {
            let column = self.reference_column(reference);
            [column[0], column[1], (self.reference_position(reference)[2] / self.body_height.max(self.resolution)).floor() as i32]
        };
        let heuristic = |reference| (distance(self.reference_position(reference), goal) - tolerance).max(0.0) * self.minimum_cost;
        let mut states = vec![SearchState { reference: initial, cost: 0.0, parent: usize::MAX }];
        let mut indexes = ahash::AHashMap::new();
        indexes.insert(initial, 0);
        let mut queue = BinaryHeap::new();
        queue.push(Queue { score: heuristic(initial), cost: 0.0, id: 0, order: order(initial) });
        let mut edge_states: ahash::AHashSet<usize> = ahash::AHashSet::new();
        let mut expanded = 0;
        let mut radius_clipped = false;
        let mut best: Option<(usize, [f64; 3], f64)> = None;
        'search: loop {
            checkpoint(cancelled)?;
            let Some(Queue { cost, id, score, .. }) = queue.pop() else {
                if best.is_some() || edge_states.is_empty() || !domain.grow() { break; }
                // Continue the same search, retaining every admitted state/cost.
                // Only states clipped by the previous artificial edge are revisited.
                for id in edge_states.drain() {
                    let state = &states[id];
                    queue.push(Queue { score: state.cost + heuristic(state.reference), cost: state.cost,
                        id, order: order(state.reference) });
                }
                continue;
            };
            if cost > states[id].cost { continue; }
            if best.is_some_and(|(_, _, total)| score >= total) { break; }
            if expanded >= limits.max_states {
                if best.is_some() { break; }
                return Err(PlanningFailure::BudgetExhausted(format!("层析请求展开工作预算耗尽（展开{expanded}格，状态{}，上限{}）；未证明无路",
                    states.len(), limits.max_states)));
            }
            expanded += 1;
            let from = states[id].reference;
            let a = self.reference_position(from);
            let actual_a = if id == 0 { start } else { a };
            let precise = (self.reference_column(from) == goal_column)
                .then_some([goal[0], goal[1], a[2]]);
            // Keep the cell-center arrival candidate as well: the exact goal XY
            // can be just outside a boundary while its arrival ball overlaps it.
            let center = if precise.is_some_and(|p| distance(p,a) <= 1e-6) { None } else { Some(a) };
            for candidate in [center, precise].into_iter().flatten() {
                if distance(candidate, goal) <= tolerance + 1e-6
                    && radius.is_none_or(|r| (candidate[0] - start[0]).hypot(candidate[1] - start[1]) <= r)
                    && accepts(actual_a, candidate)
                    && self.reference_segment(actual_a, candidate, boundary, cancelled).is_some() {
                    let total = cost + 2.0 * distance(candidate, goal);
                    if best.is_none_or(|(_, _, old)| total < old) { best = Some((id, candidate, total)); }
                }
            }
            let column = self.reference_column(from);
            for dx in -1..=1 { for dy in -1..=1 {
                checkpoint(cancelled)?;
                if dx == 0 && dy == 0 { continue; }
                let Some(x) = column[0].checked_add(dx) else { continue; };
                let Some(y) = column[1].checked_add(dy) else { continue; };
                let xy = [x,y].map(|v|(f64::from(v)+0.5)*self.resolution);
                if !crate::segment_inside_boundary([actual_a[0], actual_a[1]], xy, boundary) { continue; }
                let outside_radius = radius.is_some_and(|r| (xy[0]-start[0]).hypot(xy[1]-start[1]) > r);
                if !domain.contains([x,y]) {
                    if domain.inconclusive_edge([x,y]) {
                        // Unmaterialized cells remain uninspected, never unknown.
                        edge_states.insert(id);continue;
                    }
                    if !outside_radius { continue; }
                }
                let Some(next) = self.reference_neighbor(from, [x, y]) else { continue; };
                let b = self.reference_position(next);
                if !accepts(actual_a, b) { continue; }
                // The real start connector may cross more than one grid edge.
                if id == 0 && self.reference_segment(start, b, boundary, cancelled).is_none() { continue; }
                if outside_radius {
                    // Only an otherwise admissible successor establishes an
                    // artificial horizon cutoff. A wall/forbidden/physical-boundary
                    // enclosure remains a geometric conclusion inside the disk.
                    radius_clipped = true;continue;
                }
                let local_cost = penalties.iter().map(|p| penalty_at(b, p)).sum::<f64>();
                let next_cost = cost + distance(a, b)
                    * ((self.reference_cost(from) + self.reference_cost(next)) * 0.5 + local_cost);
                let next_id = match indexes.get(&next).copied() {
                    Some(id) => id,
                    None => {
                        if states.len() >= limits.max_states {
                            if best.is_some() { break 'search; }
                            return Err(PlanningFailure::BudgetExhausted(format!("层析请求状态预算耗尽（展开{expanded}格，状态{}，上限{}）；未证明无路",
                                states.len(), limits.max_states)));
                        }
                        let id = states.len();
                        states.push(SearchState { reference: next, cost: f64::INFINITY, parent: usize::MAX });
                        indexes.insert(next, id);
                        id
                    }
                };
                if next_cost < states[next_id].cost {
                    states[next_id].cost = next_cost;
                    states[next_id].parent = id;
                    if queue.len() >= limits.max_states {
                        queue.retain(|entry| entry.cost == states[entry.id].cost);
                        checkpoint(cancelled)?;
                    }
                    queue.push(Queue { score: next_cost + heuristic(next), cost: next_cost,
                        id: next_id, order: order(next) });
                }
            }}
        }
        let Some((id, endpoint, _)) = best else {
            return Err(if !edge_states.is_empty() || radius_clipped {
                PlanningFailure::SearchDomainLimited(format!("层析搜索范围已耗尽（展开{expanded}格）；未证明物理无路"))
            } else { PlanningFailure::Other(format!("层析参考在请求范围内没有可连接路线（展开{expanded}格）")) });
        };
        let mut ids = vec![id];
        let mut current = id;
        while current != 0 {
            checkpoint(cancelled)?;
            current = states[current].parent;
            ids.push(current);
        }
        ids.reverse();
        let mut points = Vec::with_capacity(ids.len() + 2);
        points.push(start);
        for id in ids.into_iter().skip(1) {
            checkpoint(cancelled)?;
            let p = self.reference_position(states[id].reference);
            if distance(*points.last().unwrap(), p) > 1e-6 { points.push(p); }
        }
        if distance(*points.last().unwrap(), endpoint) > 1e-6 { points.push(endpoint); }
        let points = self.reduce(points, boundary, penalties, cancelled, accepts)?;
        Ok(Route { points, stats: SearchStats { expanded,
            elapsed_ms: began.elapsed().as_secs_f64() * 1000.0, cells: self.cells.len() } })
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
        {
            return None;
        }
        let base = self.reference_segment(a, b, boundary, cancelled)?;
        let length = distance(a, b);
        let n = (length / (self.resolution * 0.5)).ceil().max(1.0) as usize;
        let mut extra = 0.0;
        for i in 0..n {
            checkpoint(cancelled).ok()?;
            let t = (i as f64 + 0.5) / n as f64;
            let point = std::array::from_fn(|j| a[j] + t * (b[j] - a[j]));
            extra += penalties.iter().map(|p| penalty_at(point, p)).sum::<f64>() * length / n as f64;
        }
        Some(base + extra)
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
            let Some(cost) = self.segment_cost(pair[0], pair[1], boundary, penalties, cancelled, accepts) else {
                checkpoint(cancelled)?;
                return Err("层析参考相邻段未通过一致的障碍/地形验证".into());
            };
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
    fn missing_ground_uses_reference_without_map_interpolation_and_observes_cancellation() {
        assert!(map(true)
            .plan([0.05, 0.05, 0.0], [8.05, 0.05, 0.0], 0.05, &[], None)
            .is_ok());
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
        eprintln!("ramp_reference expanded={} known_cells={} elapsed_ms={} points={} unknown_points={}",
            path.stats.expanded,map.cells.len(),path.stats.elapsed_ms,path.points.len(),
            path.points.iter().filter(|&&p|map.cell_at(p).is_none()).count());
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

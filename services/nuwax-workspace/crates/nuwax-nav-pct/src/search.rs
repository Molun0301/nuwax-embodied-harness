use crate::{distance, inside_boundary, Tomogram, PlanningFailure};
use crate::reference::{Reference, SegmentTrace, TraceChange};
use std::{ops::Range, sync::Arc};
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
/// Planned reference samples for this Tomogram only. No source ownership or
/// execution/arrival authority is implied by a Known cell id.
#[derive(Clone, Debug)]
pub(crate) struct ConnectionRecord {
    pub point_span: [usize; 2],
    pub evaluated_from: [f64; 3],
    pub evaluated_to: [f64; 3],
    pub steps: usize,
    pub changes: Range<usize>,
}
#[derive(Debug, Default)]
pub(crate) struct RouteReferenceTrace {
    pub connections: Vec<ConnectionRecord>,
    pub changes: Vec<TraceChange>,
}
impl RouteReferenceTrace {
    fn append(&mut self, span: [usize; 2], from: [f64; 3], to: [f64; 3],
        steps: usize, changes: impl IntoIterator<Item = TraceChange>) {
        let first = self.changes.len();
        #[cfg(test)]
        let previous_capacity = self.changes.capacity();
        self.changes.extend(changes);
        #[cfg(test)]
        {
            trace_tests::record_trace_work("arena", 1);
            trace_tests::record_trace_work("growth", usize::from(self.changes.capacity() != previous_capacity)
                + usize::from(self.connections.len() == self.connections.capacity()));
        }
        self.connections.push(ConnectionRecord { point_span: span,
            evaluated_from: from, evaluated_to: to, steps, changes: first..self.changes.len() });
    }
    fn append_record(&mut self, source: &Self, record: &ConnectionRecord, span: [usize; 2]) {
        self.append(span, record.evaluated_from, record.evaluated_to, record.steps,
            source.changes[record.changes.clone()].iter().copied());
    }
}
#[derive(Clone, Debug)]
pub(crate) struct TracedRoute {
    pub route: Route,
    pub trace: Arc<RouteReferenceTrace>,
}
struct SearchOutput { route: Route, trace: Option<Arc<RouteReferenceTrace>> }
/// Request-local interpretation; no source mutation, second search or ground
/// height is inferred from an Unanchored physical coordinate.
#[derive(Clone, Copy)]
enum SearchGeometry {
    Body,
    Ground { unanchored_z: Option<f64>, target: Option<usize> },
}
impl SearchGeometry {
    fn ground(self) -> bool { matches!(self,Self::Ground { .. }) }
    fn initial(self,map:&Tomogram,start:[f64;3])->Option<Reference> {
        match self {
            Self::Ground {unanchored_z:Some(_),..}=>map.unanchored_at(map.point_column(start)),
            _=>map.reference_start(start),
        }
    }
    fn position(self,map:&Tomogram,reference:Reference)->[f64;3] {
        match (self,reference) {
            (Self::Ground {unanchored_z:Some(z),..},Reference::Unanchored {column})=>[
                (f64::from(column[0])+0.5)*map.resolution,
                (f64::from(column[1])+0.5)*map.resolution,z],
            _=>map.reference_position(reference),
        }
    }
    fn height(self,map:&Tomogram,reference:Reference,xy:[f64;2])->Option<f64> {
        match (self,reference) {
            (Self::Ground {unanchored_z:Some(z),..},Reference::Unanchored {..})=>Some(z),
            _=>map.reference_height_at(reference,xy),
        }
    }
    fn goal_height(self,map:&Tomogram,goal:[f64;3])->Option<f64> {
        match self {
            Self::Ground {unanchored_z:Some(_),..}=>None,
            Self::Ground {target:Some(id),..}=>map.ground_height_at(id,[goal[0],goal[1]]),
            _=>map.cell_at(goal).and_then(|id|map.ground_height_at(id,[goal[0],goal[1]])),
        }
    }
    fn arrival_distance(self,a:[f64;3],b:[f64;3])->f64 {
        if self.ground() {(a[0]-b[0]).hypot(a[1]-b[1])} else {distance(a,b)}
    }
    fn initial_for_segment(self,reference:Reference)->Option<Reference> {
        self.ground().then_some(reference)
    }
    fn goal_matches(self,reference:Reference)->bool {
        match self { Self::Ground {target:Some(id),..}=>reference==Reference::Known(id), _=>true }
    }
}

// The original output de-duplication, shared by real backtracking and its
// metadata oracle. No geometry query is performed here.
fn push_reference_point<const RECORD: bool>(points: &mut Vec<[f64; 3]>, point: [f64; 3],
    previous_raw: &mut [f64; 3], connections_after: usize, tail_start: &mut usize)
    -> ([usize; 2], [f64; 3]) {
    let before = points.len() - 1;
    if distance(*points.last().unwrap(), point) > 1e-6 { points.push(point); }
    let from = *previous_raw;
    if RECORD {
        *previous_raw = point;
        if before != points.len() - 1 { *tail_start = connections_after; }
    }
    ([before, points.len() - 1], from)
}
/// Only the current admitted parent edge. Overflow changes representation,
/// never search admission, budgets, or the original sample count.
enum ParentEvidence {
    Inline { references: [Reference; 3], samples: [u8; 3], len: u8, steps: u8 },
    Overflow { steps: usize, changes: Vec<TraceChange> },
}
impl ParentEvidence {
    fn from_segment(segment: &SegmentTrace) -> Self {
        if segment.changes.len() <= 3 && segment.steps <= u8::MAX as usize
            && segment.changes.iter().all(|change| change.sample_index <= u8::MAX as usize) {
            let mut references = [segment.changes[0].reference; 3];
            let mut samples = [0; 3];
            for (index, change) in segment.changes.iter().enumerate() {
                references[index] = change.reference;
                samples[index] = change.sample_index as u8;
            }
            Self::Inline { references, samples, len: segment.changes.len() as u8, steps: segment.steps as u8 }
        } else {
            #[cfg(test)]
            trace_tests::record_trace_work("growth", 1);
            Self::Overflow { steps: segment.steps, changes: segment.changes.clone() }
        }
    }
    fn append(&self, trace: &mut RouteReferenceTrace, span: [usize; 2], from: [f64; 3], to: [f64; 3]) {
        match self {
            Self::Inline { references, samples, len, steps } => trace.append(span, from, to, *steps as usize,
                (0..usize::from(*len)).map(|i| TraceChange { sample_index: samples[i] as usize, reference: references[i] })),
            Self::Overflow { steps, changes } => trace.append(span, from, to, *steps, changes.iter().copied()),
        }
    }
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
    #[cfg(test)]
    trace_tests::record_checkpoint();
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
        let result = self.plan_bounded_impl::<false>(start, goal, tolerance, boundary, cancelled, radius, penalties, &|_, _| true, SearchLimits::default());
        checkpoint(cancelled)?;
        result.map(|output| output.route).map_err(|error|error.to_string())
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
        let result = self.plan_bounded_impl::<false>(start, goal, tolerance, boundary, cancelled, radius, penalties, accepts, SearchLimits::default());
        checkpoint(cancelled)?;
        result.map(|output| output.route).map_err(|error|error.to_string())
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
        let result = self.plan_bounded_impl::<false>(start, goal, tolerance, boundary, cancelled, radius,
            penalties, accepts, limits);
        checkpoint(cancelled)?;
        result.map(|output| output.route).map_err(|error|error.to_string())
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
        let result = self.plan_bounded_impl::<false>(start, goal, tolerance, boundary, cancelled, radius,
            penalties, &|_, _| true, limits);
        checkpoint(cancelled)?;
        result.map(|output| output.route).map_err(|error|error.to_string())
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
        let result=self.plan_bounded_impl::<false>(start,goal,tolerance,boundary,cancelled,radius,penalties,accepts,limits);
        checkpoint(cancelled)?;
        result.map(|output| output.route)
    }
    /// Internal opt-in planning provenance; public body/reference wrappers keep
    /// their original Route and do not allocate reference traces.
    pub(crate) fn plan_bounded_traced(
        &self, start: [f64; 3], goal: [f64; 3], tolerance: f64,
        boundary: &[[f64; 2]], cancelled: Option<&AtomicBool>, radius: Option<f64>,
        penalties: &[crate::RoutePenalty], accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
        limits: SearchLimits,
    ) -> Result<TracedRoute, PlanningFailure> {
        checkpoint(cancelled)?;
        let result = self.plan_bounded_impl::<true>(start, goal, tolerance, boundary, cancelled,
            radius, penalties, accepts, limits);
        checkpoint(cancelled)?;
        result.map(|output| TracedRoute { route: output.route, trace: output.trace.expect("recorded successful route") })
    }
    fn plan_bounded_impl<const RECORD: bool>(
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
    ) -> Result<SearchOutput, PlanningFailure> {
        self.plan_bounded_impl_in::<RECORD>(start,goal,tolerance,boundary,cancelled,radius,
            penalties,accepts,limits,SearchGeometry::Body)
    }
    pub(crate) fn plan_ground_traced(
        &self,start:[f64;3],goal:[f64;3],tolerance:f64,boundary:&[[f64;2]],
        cancelled:Option<&AtomicBool>,radius:Option<f64>,penalties:&[crate::RoutePenalty],
        accepts:&dyn Fn([f64;3],[f64;3])->bool,limits:SearchLimits,
        unanchored_z:Option<f64>,target:Option<usize>,
    )->Result<TracedRoute,PlanningFailure> {
        checkpoint(cancelled)?;
        let result=self.plan_bounded_impl_in::<true>(start,goal,tolerance,boundary,cancelled,radius,
            penalties,accepts,limits,SearchGeometry::Ground {unanchored_z,target});
        checkpoint(cancelled)?;
        result.map(|output|TracedRoute {route:output.route,trace:output.trace.expect("Ground records its admitted trace")})
    }
    fn plan_bounded_impl_in<const RECORD: bool>(
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
        geometry: SearchGeometry,
    ) -> Result<SearchOutput, PlanningFailure> {
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
        let initial = geometry.initial(self,start).ok_or("层析起点存在已知障碍或不兼容地形")?;
        let goal_column = self.point_column(goal);
        let initial_height = geometry.height(self,initial,[start[0],start[1]])
            .ok_or("层析起点地表来源与实际查询列不一致")?;
        let goal_height = geometry.goal_height(self,goal);
        let [low, high] = self.ground_bounds.unwrap_or([initial_height; 2]);
        let low = low.min(initial_height).min(goal_height.unwrap_or(initial_height));
        let high = high.max(initial_height).max(goal_height.unwrap_or(initial_height));
        if !geometry.ground() && (goal[2] + tolerance + 1e-6 < low
            || goal[2] - tolerance - 1e-6 > high) {
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
        let direct_height = goal_height.unwrap_or(initial_height);
        let direct = [goal[0], goal[1], direct_height];
        let mut scratch = RECORD.then(SegmentTrace::default);
        #[cfg(test)]
        if RECORD { trace_tests::record_trace_work("scratch", 1); }
        if geometry.arrival_distance(direct, goal) <= tolerance + 1e-6
            && radius.is_none_or(|r| (direct[0] - start[0]).hypot(direct[1] - start[1]) <= r)
            && penalties.is_empty() && accepts(start, direct)
            && self.reference_segment_from_recorded::<RECORD>(start, direct, boundary, cancelled, &mut scratch,
                geometry.initial_for_segment(initial)).is_some_and(|(cost, observed, end)| geometry.goal_matches(end)
                    && (limits.max_states == 0 || !observed || cost <= distance(start, direct) + 1e-6)) {
            checkpoint(cancelled)?;
            let points = if distance(start, direct) <= 1e-6 { vec![start] } else { vec![start, direct] };
            let trace = if RECORD {
                let segment = scratch.take().expect("recording scratch");
                let mut trace = RouteReferenceTrace::default();
                trace.append([0, points.len() - 1], start, direct, segment.steps, segment.changes);
                #[cfg(test)]
                trace_tests::record_trace_work("arc", 1);
                Some(Arc::new(trace))
            } else { None };
            return Ok(SearchOutput { route: Route { points,
                stats: SearchStats { expanded: 0, elapsed_ms: began.elapsed().as_secs_f64() * 1000.0,
                    cells: self.cells.len() } }, trace });
        }
        if limits.max_states == 0 {
            return Err(PlanningFailure::BudgetExhausted("快速检查仅验证直连接续，绕行或局部代价需要完整搜索；未证明无路".into()));
        }
        let mut domain = SearchDomain::new(self, start, goal, boundary, radius);
        let order = |reference| {
            let column = self.reference_column(reference);
            [column[0], column[1], (geometry.position(self,reference)[2] / self.body_height.max(self.resolution)).floor() as i32]
        };
        let heuristic = |reference| (geometry.arrival_distance(geometry.position(self,reference), goal) - tolerance).max(0.0) * self.minimum_cost;
        let mut states = vec![SearchState { reference: initial, cost: 0.0, parent: usize::MAX }];
        let mut parents: Option<Vec<Option<ParentEvidence>>> = RECORD.then(|| vec![None]);
        let mut best_trace = RECORD.then(SegmentTrace::default);
        #[cfg(test)]
        if RECORD {
            trace_tests::record_trace_work("sidecar", 1);
            trace_tests::record_trace_work("growth", 1);
        }
        let mut indexes = ahash::AHashMap::new();
        indexes.insert(initial, 0);
        let mut queue = BinaryHeap::new();
        queue.push(Queue { score: heuristic(initial), cost: 0.0, id: 0, order: order(initial) });
        // 起终点都有观测参考时，先沿有限的已观测连接寻找首条路线。
        // 同层噪声也会产生多个缺测高程平面，不能按端点高差或半数预算
        // 提前放弃观测进展。缺测候选共用状态、代价和预算，仍可桥接断点；
        // 找到首条路线后合并队列，继续按原代价改进。未知目标仍走混合搜索。
        let mut prefer_observed = matches!(initial, crate::reference::Reference::Known(_))
            && self.cell_at(goal).is_some();
        let mut deferred = BinaryHeap::new();
        let mut edge_states: ahash::AHashSet<usize> = ahash::AHashSet::new();
        let mut expanded = 0;
        let mut radius_clipped = false;
        let mut state_budget_exhausted = false;
        let mut best: Option<(usize, [f64; 3], f64)> = None;
        'search: loop {
            checkpoint(cancelled)?;
            if prefer_observed && best.is_some() {
                prefer_observed = false;
                queue.append(&mut deferred);
            }
            if prefer_observed && queue.is_empty() {
                // 已观测坡道中也可能有缺测缝；取一个未知候选继续连接，
                // 新发现的观测状态仍有机会在当前共享预算内展开。
                if let Some(next) = deferred.pop() { queue.push(next); }
            }
            let Some(Queue { cost, id, score, .. }) = queue.pop() else {
                if best.is_some() || state_budget_exhausted || edge_states.is_empty() || !domain.grow() { break; }
                // Continue the same search, retaining every admitted state/cost.
                // Only states clipped by the previous artificial edge are revisited.
                for id in edge_states.drain() {
                    let state = &states[id];
                    queue.push(Queue { score: state.cost + heuristic(state.reference), cost: state.cost,
                        id, order: order(state.reference) });
                }
                continue;
            };
            if cost > states[id].cost {
                #[cfg(test)]
                trace_tests::record_stale_queue(id, cost, states[id].cost, states[id].parent);
                #[cfg(test)]
                if RECORD {
                    trace_tests::record_stale_parent(id, states[id].parent, parents.as_ref().unwrap()[id].as_ref().unwrap());
                }
                continue;
            }
            if best.is_some_and(|(_, _, total)| score >= total) { break; }
            if expanded >= limits.max_states {
                if best.is_some() { break; }
                return Err(PlanningFailure::BudgetExhausted(format!("层析请求展开工作预算耗尽（展开{expanded}格，状态{}，上限{}）；未证明无路",
                    states.len(), limits.max_states)));
            }
            expanded += 1;
            let from = states[id].reference;
            let a = geometry.position(self,from);
            let actual_a = if id == 0 { start } else { a };
            let precise = (self.reference_column(from) == goal_column)
                .then(||geometry.height(self,from,[goal[0],goal[1]]))
                .flatten().map(|height|[goal[0],goal[1],height]);
            // Keep the cell-center arrival candidate as well: the exact goal XY
            // can be just outside a boundary while its arrival ball overlaps it.
            let center = if precise.is_some_and(|p| distance(p,a) <= 1e-6) { None } else { Some(a) };
            for candidate in [center, precise].into_iter().flatten() {
                if geometry.arrival_distance(candidate, goal) <= tolerance + 1e-6
                    && radius.is_none_or(|r| (candidate[0] - start[0]).hypot(candidate[1] - start[1]) <= r)
                    && accepts(actual_a, candidate)
                    && self.reference_segment_from_recorded::<RECORD>(actual_a, candidate, boundary, cancelled, &mut scratch,
                        geometry.initial_for_segment(from)).is_some_and(|(_,_,end)|geometry.goal_matches(end)) {
                    let total = cost + 2.0 * geometry.arrival_distance(candidate, goal);
                    if best.is_none_or(|(_, _, old)| total < old) {
                        best = Some((id, candidate, total));
                        if RECORD { std::mem::swap(&mut best_trace, &mut scratch); }
                        #[cfg(test)]
                        trace_tests::record_best_cost(total, precise == Some(candidate), actual_a, candidate);
                    }
                }
            }
            let column = self.reference_column(from);
            let mut candidates = [None; 8];
            let mut candidate_count = 0;
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
                let b = geometry.position(self,next);
                if !accepts(actual_a, b) { continue; }
                if outside_radius {
                    // 只有整段可接纳的越界边才证明人工半径截断；仅有邻点
                    // 不能把真实障碍或高度不连续改写成搜索范围不足。
                    #[cfg(test)]
                    tests::record_neighbor_validation(b);
                    if self.reference_segment_from_recorded::<false>(actual_a,b,boundary,cancelled,&mut None,
                        geometry.initial_for_segment(from)).is_some() {
                        radius_clipped = true;
                    }
                    continue;
                }
                let local_cost = penalties.iter().map(|p| penalty_at(b, p)).sum::<f64>();
                let next_cost = cost + distance(a, b)
                    * ((self.reference_cost(from) + self.reference_cost(next)) * 0.5 + local_cost);
                let existing = indexes.get(&next).copied();
                // 只跳过同一精确参考状态的不改善边，保留过滤回调的原调用顺序。
                // 新状态仍在完整段验证之后才占用状态预算。
                if existing.is_some_and(|next_id| !(next_cost < states[next_id].cost)) { continue; }
                candidates[candidate_count] = Some((next, b, next_cost, existing));
                candidate_count += 1;
            }}
            // 先保持过滤回调的原邻居顺序，再让观测优先也作用于状态接纳。
            // 最多八个栈上候选；同类保持原次序，未知仍由原延后队列接续。
            if prefer_observed {
                candidates[..candidate_count].sort_by_key(|candidate| candidate.is_some_and(
                    |(next, _, _, _)| matches!(next, crate::reference::Reference::Unknown { .. })));
            }
            for (next, b, next_cost, existing) in candidates.into_iter().flatten() {
                checkpoint(cancelled)?;
                // 邻接成立不代表整段成立：缺测格继承的高度与斜向段内经过的
                // 侧格可能不同。所有可能改善路线的边仍复用输出段检查。
                #[cfg(test)]
                tests::record_neighbor_validation(b);
                if self.reference_segment_from_recorded::<RECORD>(actual_a,b,boundary,cancelled,&mut scratch,
                    geometry.initial_for_segment(from)).is_none_or(|(_,_,end)|geometry.ground()&&end!=next) { continue; }
                let next_id = match existing {
                    Some(id) => id,
                    None => {
                        if states.len() >= limits.max_states {
                            if best.is_some() { break 'search; }
                            // 新状态被截断不等于已接纳队列没有路线；继续处理原
                            // 队列及父链，最终无解时仍报告资源不足而非物理无路。
                            state_budget_exhausted = true;
                            continue;
                        }
                        let id = states.len();
                        states.push(SearchState { reference: next, cost: f64::INFINITY, parent: usize::MAX });
                        if RECORD {
                            let parents = parents.as_mut().expect("recorded parents");
                            #[cfg(test)]
                            if parents.len() == parents.capacity() { trace_tests::record_trace_work("growth", 1); }
                            parents.push(None);
                        }
                        indexes.insert(next, id);
                        id
                    }
                };
                if next_cost < states[next_id].cost {
                    #[cfg(test)]
                    if existing.is_some() {
                        trace_tests::record_parent_update(next_id, states[next_id].cost, next_cost,
                            states[next_id].parent, id, next);
                    }
                    states[next_id].cost = next_cost;
                    states[next_id].parent = id;
                    if RECORD {
                        parents.as_mut().expect("recorded parents")[next_id] =
                            Some(ParentEvidence::from_segment(scratch.as_ref().expect("recording scratch")));
                        #[cfg(test)]
                        trace_tests::record_parent_trace(next_id, id, parents.as_ref().unwrap()[next_id].as_ref().unwrap(), scratch.as_ref().unwrap());
                    }
                    if queue.len() + deferred.len() >= limits.max_states {
                        queue.retain(|entry| entry.cost == states[entry.id].cost);
                        deferred.retain(|entry: &Queue| entry.cost == states[entry.id].cost);
                        checkpoint(cancelled)?;
                    }
                    let target = if prefer_observed && matches!(next, crate::reference::Reference::Unknown { .. }) {
                        &mut deferred
                    } else { &mut queue };
                    target.push(Queue { score: next_cost + heuristic(next), cost: next_cost,
                        id: next_id, order: order(next) });
                }
            }
        }
        let Some((id, endpoint, _)) = best else {
            return Err(if state_budget_exhausted {
                PlanningFailure::BudgetExhausted(format!("层析请求状态预算耗尽（展开{expanded}格，状态{}，上限{}）；未证明无路",
                    states.len(), limits.max_states))
            } else if !edge_states.is_empty() || radius_clipped {
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
        let mut raw_trace = RECORD.then(RouteReferenceTrace::default);
        let mut tail_start = 0;
        let mut previous_raw = start;
        points.push(start);
        let mut point_references=geometry.ground().then(||vec![initial]);
        for id in ids.into_iter().skip(1) {
            checkpoint(cancelled)?;
            let p = geometry.position(self,states[id].reference);
            let (span, from) = push_reference_point::<RECORD>(&mut points, p, &mut previous_raw,
                raw_trace.as_ref().map_or(0, |trace| trace.connections.len() + 1), &mut tail_start);
            if let Some(references)=point_references.as_mut() {
                if span[0]!=span[1] {references.push(states[id].reference);}
            }
            if RECORD {
                let trace = raw_trace.as_mut().unwrap();
                parents.as_ref().unwrap()[id].as_ref().expect("admitted parent evidence")
                    .append(trace, span, from, p);
            }
        }
        let (span, from) = push_reference_point::<RECORD>(&mut points, endpoint, &mut previous_raw,
            raw_trace.as_ref().map_or(0, |trace| trace.connections.len() + 1), &mut tail_start);
        if RECORD {
            let segment = best_trace.take().expect("best connector evidence");
            if let Some(references)=point_references.as_mut() {
                if span[0]!=span[1] {references.push(segment.changes.last().unwrap().reference);}
            }
            let trace = raw_trace.as_mut().unwrap();
            trace.append(span, from, endpoint, segment.steps, segment.changes);
            // Parent evidence is no longer needed; do not overlap its peak with
            // reducer prefix storage. The false specialization retains old life spans.
            drop(parents.take());
            drop(states);
        }
        let (points, trace) = self.reduce_recorded_in::<RECORD>(points, boundary, penalties, cancelled,
            accepts, raw_trace, tail_start, &mut scratch,point_references.as_deref())?;
        #[cfg(test)]
        if RECORD { trace_tests::record_trace_work("arc", 1); }
        Ok(SearchOutput { route: Route { points, stats: SearchStats { expanded,
            elapsed_ms: began.elapsed().as_secs_f64() * 1000.0, cells: self.cells.len() } }, trace: trace.map(Arc::new) })
    }

    fn segment_cost_recorded<const RECORD: bool>(
        &self,
        a: [f64; 3],
        b: [f64; 3],
        boundary: &[[f64; 2]],
        penalties: &[crate::RoutePenalty],
        cancelled: Option<&AtomicBool>,
        accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
        scratch: &mut Option<SegmentTrace>,
        references: Option<[Reference;2]>,
    ) -> Option<f64> {
        if !accepts(a, b) || !crate::segment_inside_boundary([a[0], a[1]], [b[0], b[1]], boundary)
        {
            return None;
        }
        let (base,_,end)=self.reference_segment_from_recorded::<RECORD>(a,b,boundary,cancelled,scratch,
            references.map(|r|r[0]))?;
        if references.is_some_and(|r|r[1]!=end) {return None;}
        let length = distance(a, b);
        let n = (length / (self.resolution * 0.5)).ceil().max(1.0) as usize;
        let mut extra = 0.0;
        for i in 0..n {
            checkpoint(cancelled).ok()?;
            let t = (i as f64 + 0.5) / n as f64;
            let point = std::array::from_fn(|j| a[j] + t * (b[j] - a[j]));
            extra += penalties.iter().map(|p| penalty_at(point, p)).sum::<f64>() * length / n as f64;
        }
        #[cfg(test)]
        trace_tests::record_reduced_cost(base + extra);
        Some(base + extra)
    }

    #[cfg(test)]
    fn reduce(
        &self,
        points: Vec<[f64; 3]>,
        boundary: &[[f64; 2]],
        penalties: &[crate::RoutePenalty],
        cancelled: Option<&AtomicBool>,
        accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
    ) -> Result<Vec<[f64; 3]>, String> {
        self.reduce_recorded::<false>(points, boundary, penalties, cancelled, accepts, None, 0, &mut None)
            .map(|(points, _)| points)
    }
    fn reduce_recorded<const RECORD: bool>(
        &self, points: Vec<[f64; 3]>, boundary: &[[f64; 2]], penalties: &[crate::RoutePenalty],
        cancelled: Option<&AtomicBool>, accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
        raw_trace: Option<RouteReferenceTrace>, tail_start: usize, scratch: &mut Option<SegmentTrace>,
    ) -> Result<(Vec<[f64; 3]>, Option<RouteReferenceTrace>), String> {
        self.reduce_recorded_in::<RECORD>(points,boundary,penalties,cancelled,accepts,
            raw_trace,tail_start,scratch,None)
    }
    fn reduce_recorded_in<const RECORD: bool>(
        &self, points: Vec<[f64; 3]>, boundary: &[[f64; 2]], penalties: &[crate::RoutePenalty],
        cancelled: Option<&AtomicBool>, accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
        raw_trace: Option<RouteReferenceTrace>, tail_start: usize, scratch: &mut Option<SegmentTrace>,
        point_references: Option<&[Reference]>,
    ) -> Result<(Vec<[f64; 3]>, Option<RouteReferenceTrace>), String> {
        checkpoint(cancelled)?;
        if points.len() < 3 {
            return Ok((points, raw_trace));
        }
        let mut prefix_trace = RECORD.then(RouteReferenceTrace::default);
        let mut output_trace = RECORD.then(RouteReferenceTrace::default);
        let mut prefix = vec![0.0];
        for (index,pair) in points.windows(2).enumerate() {
            checkpoint(cancelled)?;
            let Some(cost) = self.segment_cost_recorded::<RECORD>(pair[0], pair[1], boundary, penalties, cancelled, accepts, scratch,
                point_references.map(|r|[r[index],r[index+1]])) else {
                checkpoint(cancelled)?;
                return Err(format!("层析参考相邻段未通过一致的障碍/地形验证：{:?} -> {:?}", pair[0], pair[1]));
            };
            if RECORD {
                let segment = scratch.as_ref().unwrap();
                let index = prefix.len() - 1;
                prefix_trace.as_mut().unwrap().append([index, index + 1], pair[0], pair[1],
                    segment.steps, segment.changes.iter().copied());
            }
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
            let mut shortcut = false;
            for next in (anchor + 2..=far).rev() {
                checkpoint(cancelled)?;
                if self
                    .segment_cost_recorded::<RECORD>(points[anchor], points[next], boundary, penalties, cancelled, accepts, scratch,
                        point_references.map(|r|[r[anchor],r[next]]))
                    .is_some_and(|cost| cost <= (prefix[next] - prefix[anchor]) * 1.1 + 1e-6)
                {
                    chosen = next;
                    shortcut = true;
                    break;
                }
            }
            if RECORD {
                let span = [output.len() - 1, output.len()];
                let trace = output_trace.as_mut().unwrap();
                if shortcut {
                    let segment = scratch.as_ref().unwrap();
                    trace.append(span, points[anchor], points[chosen], segment.steps, segment.changes.iter().copied());
                } else {
                    let prefix = prefix_trace.as_ref().unwrap();
                    trace.append_record(prefix, &prefix.connections[anchor], span);
                }
            }
            output.push(points[chosen]);
            anchor = chosen;
        }
        checkpoint(cancelled)?;
        if RECORD {
            if let Some(raw) = raw_trace {
                let last = output.len() - 1;
                for record in &raw.connections[tail_start..] {
                    output_trace.as_mut().unwrap().append_record(&raw, record, [last, last]);
                }
            }
        }
        Ok((output, output_trace))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    thread_local! {
        static NEIGHBOR_VALIDATIONS: std::cell::Cell<Option<([f64;3],usize,usize)>> = const {
            std::cell::Cell::new(None)
        };
    }
    pub(super) fn record_neighbor_validation(endpoint: [f64;3]) {
        trace_tests::record_neighbor(endpoint);
        if let Some((start,total,returns)) = NEIGHBOR_VALIDATIONS.get() {
            NEIGHBOR_VALIDATIONS.set(Some((start,total+1,returns+usize::from(endpoint==start))));
        }
    }
    #[test]
    fn dominated_return_edges_skip_segment_validation_but_keep_filter_calls() {
        for known in [false,true] {
            let columns: Vec<_> = if known {(0..5).map(|x|([x,0],vec![[-0.1,0.0]])).collect()}
                else {Vec::new()};
            let map=Tomogram::from_columns(0.1,columns,0.4,0.0,0.1,0.1,20.0).unwrap();
            let start=[0.05,0.05,0.0];let goal=[0.45,0.05,0.0];
            let boundary=[[0.0,0.0],[0.5,0.0],[0.5,0.1],[0.0,0.1]];
            // 非空但远离路线的软代价只关闭直连快路，不改变任何邻边代价。
            let penalties=[crate::RoutePenalty{center:[10.0,0.0,0.0],radius:0.1,weight:1.0}];
            let filter_returns=std::cell::Cell::new(0);
            let accepts=|_: [f64;3],b: [f64;3]| {
                if b==start {filter_returns.set(filter_returns.get()+1);}
                true
            };
            NEIGHBOR_VALIDATIONS.set(Some((start,0,0)));
            let result=map.plan_bounded_with_segment_filter(start,goal,0.0,&boundary,None,
                Some(1.0),&penalties,&accepts);
            let (_,validations,returns)=NEIGHBOR_VALIDATIONS.take().unwrap();
            let route=result.unwrap();
            assert_eq!(route.stats.expanded,5);
            assert!(filter_returns.get()>0,"既有过滤回调仍观察到返回起点的候选");
            assert_eq!(returns,0,"已接纳起点的更贵回边不应重复扫描整段，known={known}");
            assert_eq!(validations,4,"只完整核验四条实际改善代价的邻边");
            assert_eq!(route.points.first(),Some(&start));
            assert_eq!(route.points.last(),Some(&goal));
            assert!(route.points.windows(2).all(|p|map.segment_clear(p[0],p[1],&boundary)));
        }
    }
    #[test]
    fn radius_cutoff_requires_full_off_center_segment_admission() {
        let map=Tomogram::from_columns(0.1,[([0,1],vec![[0.0,0.1]])],
            0.4,0.0,0.15,0.15,20.0).unwrap();
        let start=[0.01,0.09,0.0];let goal=[0.55,0.55,0.0];
        let initial=map.reference_start(start).unwrap();
        let next=map.reference_neighbor(initial,[1,1]).expect("格邻接允许这个候选");
        let endpoint=map.reference_position(next);
        assert_eq!(endpoint[2],0.0);
        assert!(map.reference_segment(start,endpoint,&[],None).is_none(),
            "实际斜线先跨过较高侧格，不能再在缺测格虚构下降平面");
        let error=map.plan_bounded_with_segment_filter_and_limits_typed(start,goal,0.0,&[],None,
            Some(0.05),&[],&|_,b|b==endpoint,SearchLimits::default()).unwrap_err();
        assert!(matches!(error,PlanningFailure::Other(_)),
            "越界邻点存在但整段不合法，不能谎报搜索半径截断：{error}");
    }
    #[test]
    fn irregular_thick_ground_keeps_search_and_segment_admission_consistent() {
        let q = f64::from(0.05_f32);
        let columns = (-20_i32..180).flat_map(|ix| (-50_i32..50).filter_map(move |iy| {
            let code = (ix * 73 + iy * 151).rem_euclid(997);
            if code % 11 == 0 { return None; }
            let jitter = f64::from(code % 7 - 3) * 0.003;
            let x = (f64::from(ix) + 0.5) * q + jitter;
            let y = (f64::from(iy) + 0.5) * q - jitter;
            let height = 0.035 * (1.2 * x).sin() + 0.025 * (1.7 * y + 0.4 * x).sin()
                + 0.010 * (3.1 * x + 2.3 * y).cos();
            let top = (height / q).round() as i32 - 1;
            let layers = 2 + code % 3;
            Some(([ix, iy], vec![[f64::from(top - layers) * q, f64::from(top + 1) * q]]))
        }));
        let map = Tomogram::from_columns_with_evidence(q, q, columns,
            0.46, 0.16, 0.15, 0.15, 18.0, 0.34, false).unwrap();
        for (start, goal) in [([0.0,0.0,0.01],[7.0,0.0,0.0287065174779703]),
            ([7.0,0.0,0.0287065174779703],[0.0,0.0,0.01])] {
            let route = map.plan(start, goal, 0.15, &[], None)
                .expect("细小缺测与厚层起伏不应让搜索已接纳的相邻边在输出时失配");
            assert!(route.points.windows(2).all(|edge|map.segment_clear(edge[0],edge[1],&[])));
            assert!(distance(*route.points.last().unwrap(),goal)<=0.15);
        }
    }
    #[test]
    fn noisy_same_level_ground_keeps_observed_progress_within_the_original_budget() {
        let columns = (-40..680).flat_map(|x| (-30..31).filter_map(move |y| {
            // 稀疏点云保留多种细小高程，不把缺测格补成地面。
            if x == 120 { return None; }
            let z = f64::from((x / 3_i32).rem_euclid(5)) * 0.0125;
            Some(([x,y], vec![[z-0.05,z]]))
        }));
        let map = Tomogram::from_columns_with_evidence(0.05,0.05,columns,
            0.46,0.16,0.15,0.15,18.0,0.34,true).unwrap();
        let start = [0.075,0.025,0.0]; let goal = [30.075,0.025,0.0];
        let route = map.plan(start,goal,0.15,&[],None)
            .expect("同层噪声地面的缺测外延不应提前耗尽已观测连接机会");
        assert!(distance(*route.points.last().unwrap(),goal)<=0.15);
        assert!(route.stats.expanded<=SearchLimits::default().max_states);
        assert!(route.points.windows(2).all(|p|map.segment_clear(p[0],p[1],&[])));
        eprintln!("NOISY_ROUTE expanded={} points={}",route.stats.expanded,route.points.len());
    }

    #[test]
    fn overlapping_floors_reach_the_observed_ramp_within_the_original_budget() {
        let mut columns = std::collections::BTreeMap::<[i32; 2], Vec<[f64; 2]>>::new();
        for x in -20..20 { for y in -20..20 {
            columns.insert([x,y], vec![[-0.05,0.0],[1.95,2.0]]);
        }}
        for x in 20..100 { for y in -9..9 {
            let z = ((f64::from(x)+0.5)*0.1-2.0)*0.25;
            columns.entry([x,y]).or_default().push([z-0.05,z]);
        }}
        for x in 20..120 { for y in 10..23 {
            columns.entry([x,y]).or_default().push([1.95,2.0]);
        }}
        for x in 100..120 { for y in -9..10 {
            columns.entry([x,y]).or_default().push([1.95,2.0]);
        }}
        let mut gap_columns = columns.clone();
        gap_columns.retain(|c,_| c[0] != 50 || c[1] >= 10);
        let gap = Tomogram::from_columns(0.1,gap_columns,0.46,0.16,0.15,0.15,18.0).unwrap();
        let map = Tomogram::from_columns(0.1,columns,0.46,0.16,0.15,0.15,18.0).unwrap();
        let start = [-1.0,0.0,0.0]; let goal = [-1.0,0.0,2.0];
        // 独立遍历已观测连接，先证明场景确实有路；不靠目标高程生成未知楼层。
        let mut seen = ahash::AHashSet::new();
        let mut queue = std::collections::VecDeque::from([map.reference_start(start).unwrap()]);
        let mut reached = false;
        while let Some(from) = queue.pop_front() {
            if !seen.insert(from) { continue; }
            reached |= distance(map.reference_position(from),goal) < 0.15;
            let c = map.reference_column(from);
            for x in -1..=1 { for y in -1..=1 {
                if let Some(to @ crate::reference::Reference::Known(_)) = map.reference_neighbor(from,[c[0]+x,c[1]+y]) {
                    if !seen.contains(&to) { queue.push_back(to); }
                }
            }}
        }
        assert!(reached, "独立观测连接必须能够跨层");
        eprintln!("RAMP_CONNECTED observed_states={}",seen.len());
        let route = map.plan(start,goal,0.15,&[],None).expect("有限已观测坡道不应被未知外延耗尽预算");
        assert!(route.stats.expanded <= SearchLimits::default().max_states);
        assert!(route.points.iter().any(|p|p[0]>9.0));
        assert!(distance(*route.points.last().unwrap(),goal)<=0.15);
        assert!(route.points.windows(2).all(|p|map.segment_clear(p[0],p[1],&[])));
        eprintln!("RAMP_ROUTE expanded={} points={} elapsed_ms={}",route.stats.expanded,route.points.len(),route.stats.elapsed_ms);
        let route = gap.plan(start,goal,0.15,&[],None).expect("坡道缺测缝仍可沿原高度参考接续");
        assert!(route.points.windows(2).all(|p|gap.segment_clear(p[0],p[1],&[])));
        assert!(distance(*route.points.last().unwrap(),goal)<=0.15);
        assert!(route.stats.expanded <= SearchLimits::default().max_states);
        eprintln!("RAMP_GAP expanded={} points={} elapsed_ms={}",route.stats.expanded,route.points.len(),route.stats.elapsed_ms);
    }

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

#[cfg(test)]
#[path = "reference_trace_tests.rs"]
pub(crate) mod trace_tests;

#[cfg(test)]
#[path = "search_budget_tests.rs"]
mod budget_tests;

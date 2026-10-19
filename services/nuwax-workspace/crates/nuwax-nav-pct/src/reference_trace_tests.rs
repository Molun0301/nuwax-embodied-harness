//! Fixed B1-07 old-search observation. This is planning state, never execution proof.
use super::*;
use crate::reference::Reference;
use std::cell::RefCell;

#[derive(Debug, Default)]
struct ParentObservations {
    updates: Vec<(usize, u64, u64, usize, usize, Reference)>,
    stale: Vec<(usize, u64, u64, usize)>,
}
thread_local! {
    static PARENTS: RefCell<Option<ParentObservations>> = const { RefCell::new(None) };
}
pub(super) fn record_parent_update(id: usize, old: f64, new: f64,
    old_parent: usize, new_parent: usize, reference: Reference) {
    event(Event::ParentUpdate(id, old.to_bits(), new.to_bits(), old_parent, new_parent));
    PARENTS.with(|value| {
        if let Some(value) = value.borrow_mut().as_mut() {
            value.updates.push((id, old.to_bits(), new.to_bits(), old_parent, new_parent, reference));
        }
    });
}
pub(super) fn record_stale_queue(id: usize, queued: f64, current: f64, parent: usize) {
    event(Event::Stale(id, queued.to_bits(), current.to_bits(), parent));
    PARENTS.with(|value| {
        if let Some(value) = value.borrow_mut().as_mut() {
            value.stale.push((id, queued.to_bits(), current.to_bits(), parent));
        }
    });
}
fn observe_parents<T>(call: impl FnOnce() -> T) -> (T, ParentObservations) {
    struct Reset(Option<ParentObservations>);
    impl Drop for Reset {
        fn drop(&mut self) { PARENTS.with(|value| *value.borrow_mut() = self.0.take()); }
    }
    let _reset = Reset(PARENTS.with(|value| value.replace(Some(ParentObservations::default()))));
    let result = call();
    let observations = PARENTS.with(|value| value.take().unwrap());
    (result, observations)
}

#[test]
fn b1_07_parent_improvement_and_stale_queue_keep_winning_trace() {
    let map = Tomogram::from_columns(0.1,
        (-2..=8).flat_map(|x| (-3..=5).map(move |y| ([x, y], vec![[-0.1, 0.0]]))),
        0.4, 0.0, 0.1, 0.1, 20.0).unwrap();
    assert_eq!(map.cells.len(), 99);
    let start = [0.05, 0.05, 0.0];
    let goal = [0.55, 0.05, 0.0];
    let boundary = [[-0.2, -0.3], [0.9, -0.3], [0.9, 0.6], [-0.2, 0.6]];
    let penalties = [crate::RoutePenalty { center: [0.15, 0.05, 0.0], radius: 0.02, weight: 0.01 }];
    let accepts = |a: [f64; 3], b: [f64; 3]| {
        if a[0].min(b[0]) <= 0.3 && a[0].max(b[0]) >= 0.3 {
            a[1] + (0.3 - a[0]) / (b[0] - a[0]) * (b[1] - a[1]) > 0.1
        } else { true }
    };
    let (result, observations) = observe_parents(|| map.plan_bounded_with_segment_filter_and_limits_typed(
        start, goal, 0.0, &boundary, None, None, &penalties, &accepts, SearchLimits::default()));
    eprintln!("B1_07_ORIGINAL result={result:?} updates={:?} stale={:?}", observations.updates, observations.stale);
    let route = result.expect("the fixed known-plane detour must be legal before observing parent updates");
    eprintln!("B1_07_ORIGINAL points_bits={:?} expanded={} cells={} updates={} stale={}",
        route.points.iter().map(|point| point.map(f64::to_bits)).collect::<Vec<_>>(),
        route.stats.expanded, route.stats.cells, observations.updates.len(), observations.stale.len());
    assert_eq!(route.points.first(), Some(&start));
    assert_eq!(route.points.last(), Some(&goal));
    assert!(route.points.windows(2).all(|edge| accepts(edge[0], edge[1])));
    assert!(!observations.updates.is_empty(), "fixed input did not cover an existing-state improvement; do not tune the fixture");
    assert!(observations.updates.iter().all(|update| f64::from_bits(update.2) < f64::from_bits(update.1)));
    assert!(observations.updates.iter().any(|update| update.3 != update.4), "no actual winning parent changed");
    assert!(observations.stale.iter().all(|entry| f64::from_bits(entry.1) > f64::from_bits(entry.2)));
    assert_eq!(observations.updates.len(), 3, "frozen original-algorithm observation");
    assert_eq!(observations.stale.len(), 2, "actual stale pops, not inferred from updates");
    let (traced, work) = paired(&map, start, goal, 0.0, &boundary, None, &penalties,
        &accepts, SearchLimits::default(), None);
    assert_eq!(bits(&route.points), bits(&traced.unwrap().route.points));
    assert_eq!(work.stale_parents, observations.stale.iter().map(|entry|entry.0).collect::<Vec<_>>());
    for update in &observations.updates {
        let last = work.parents.iter().rev().find(|entry| entry.0 == update.0).unwrap();
        assert_eq!(last.1, update.4, "stored trace belongs to current winning parent");
    }
}

#[derive(Debug, PartialEq)]
enum Event {
    Checkpoint,
    Evaluation([[u64; 3]; 2]),
    SegmentCost(u64),
    ReducedCost(u64),
    BestCost(u64),
    Accept([[u64; 3]; 2]),
    Neighbor([u64; 3]),
    ParentUpdate(usize, u64, u64, usize, usize),
    Stale(usize, u64, u64, usize),
}
#[derive(Debug, Default)]
struct Work {
    events: Vec<Event>,
    trace: [usize; 7], // scratch, visit, explicit trace storage growth, sidecar, arena, parent commits, Arc
    parents: Vec<(usize, usize, usize, Vec<TraceChange>)>,
    stale_parents: Vec<usize>,
    best: Vec<BestChoice>,
}
#[derive(Debug, PartialEq)]
struct BestChoice { precise: bool, from: [f64; 3], to: [f64; 3] }
thread_local! { static WORK: RefCell<Option<Work>> = const { RefCell::new(None) }; }
fn event(event: Event) { WORK.with(|work| { if let Some(work) = work.borrow_mut().as_mut() { work.events.push(event); } }); }
pub(crate) fn record_checkpoint() { event(Event::Checkpoint); }
pub(crate) fn record_evaluation(a: [f64; 3], b: [f64; 3]) { event(Event::Evaluation([a.map(f64::to_bits), b.map(f64::to_bits)])); }
pub(crate) fn record_segment_cost(cost: f64) { event(Event::SegmentCost(cost.to_bits())); }
pub(crate) fn record_reduced_cost(cost: f64) { event(Event::ReducedCost(cost.to_bits())); }
pub(crate) fn record_best_cost(cost: f64, precise: bool, from: [f64; 3], to: [f64; 3]) {
    event(Event::BestCost(cost.to_bits()));
    WORK.with(|work| { if let Some(work) = work.borrow_mut().as_mut() { work.best.push(BestChoice { precise, from, to }); } });
}
pub(crate) fn record_neighbor(point: [f64; 3]) { event(Event::Neighbor(point.map(f64::to_bits))); }
pub(crate) fn record_trace_work(kind: &str, count: usize) {
    let index = match kind { "scratch" => 0, "visit" => 1, "growth" => 2, "sidecar" => 3, "arena" => 4, "arc" => 6, _ => panic!("unknown trace counter") };
    WORK.with(|work| { if let Some(work) = work.borrow_mut().as_mut() { work.trace[index] += count; } });
}
fn decode_parent(evidence: &ParentEvidence) -> (usize, Vec<TraceChange>) {
    match evidence {
        ParentEvidence::Inline { references, samples, len, steps } => (*steps as usize,
            (0..*len as usize).map(|i|TraceChange { sample_index:samples[i] as usize, reference:references[i] }).collect()),
        ParentEvidence::Overflow { steps, changes } => (*steps, changes.clone()),
    }
}
pub(super) fn record_parent_trace(id: usize, parent: usize, evidence: &ParentEvidence, segment: &SegmentTrace) {
    WORK.with(|work| { if let Some(work) = work.borrow_mut().as_mut() {
        let (steps, changes) = decode_parent(evidence);
        assert_eq!(steps, segment.steps); assert_eq!(changes, segment.changes);
        work.trace[5] += 1;
        work.parents.push((id, parent, steps, changes));
    } });
}
pub(super) fn record_stale_parent(id: usize, parent: usize, evidence: &ParentEvidence) {
    WORK.with(|work| { if let Some(work) = work.borrow_mut().as_mut() {
        let (steps, changes) = decode_parent(evidence);
        let latest = work.parents.iter().rev().find(|entry|entry.0 == id).unwrap();
        assert_eq!((parent, steps, &changes), (latest.1, latest.2, &latest.3));
        work.stale_parents.push(id);
    } });
}

fn observe<T>(call: impl FnOnce() -> T) -> (T, Work) {
    struct Reset(Option<Work>);
    impl Drop for Reset { fn drop(&mut self) { WORK.with(|work| *work.borrow_mut() = self.0.take()); } }
    let _reset = Reset(WORK.with(|work| work.replace(Some(Work::default()))));
    let result = call();
    (result, WORK.with(|work| work.take().unwrap()))
}
fn map(columns: impl IntoIterator<Item = ([i32; 2], Vec<[f64; 2]>)>) -> Tomogram {
    Tomogram::from_columns(0.1, columns, 0.4, 0.0, 0.1, 0.1, 20.0).unwrap()
}
fn bits(points: &[[f64; 3]]) -> Vec<[u64; 3]> { points.iter().map(|p| p.map(f64::to_bits)).collect() }
const BOUNDARY: [[f64; 2]; 4] = [[0., 0.], [2., 0.], [2., 0.3], [0., 0.3]];
fn paired(map: &Tomogram, start: [f64; 3], goal: [f64; 3], tolerance: f64,
    boundary: &[[f64; 2]], radius: Option<f64>, penalties: &[crate::RoutePenalty],
    accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool, limits: SearchLimits,
    cancel_after: Option<usize>) -> (Result<TracedRoute, PlanningFailure>, Work) {
    let accept = |a: [f64; 3], b: [f64; 3]| { event(Event::Accept([a.map(f64::to_bits), b.map(f64::to_bits)])); accepts(a, b) };
    let old_call = || map.plan_bounded_with_segment_filter_and_limits_typed(start, goal, tolerance, boundary,
        None, radius, penalties, &accept, limits);
    let (old, old_work) = observe(|| match cancel_after {
        Some(n) => crate::cancellation::after_checkpoints(n, old_call), None => old_call(),
    });
    let new_call = || map.plan_bounded_traced(start, goal, tolerance, boundary, None, radius, penalties, &accept, limits);
    let (new, new_work) = observe(|| match cancel_after {
        Some(n) => crate::cancellation::after_checkpoints(n, new_call), None => new_call(),
    });
    assert_eq!(old_work.trace, [0; 7], "false must not record samples, grow trace storage, or admit trace sidecars");
    assert_eq!(old_work.events, new_work.events, "same accepts, checkpoints, full segment calls, costs and best costs in order");
    assert_eq!(old_work.best, new_work.best, "same actual center/precise winners");
    match (&old, &new) {
        (Ok(old), Ok(new)) => {
            assert_eq!(bits(&old.points), bits(&new.route.points));
            assert_eq!((old.stats.expanded, old.stats.cells), (new.route.stats.expanded, new.route.stats.cells));
            validate_trace(map, new);
        }
        (Err(old), Err(new)) => assert_eq!(old, new),
        _ => panic!("result mismatch old={old:?} new={new:?}"),
    }
    eprintln!("B1_PAIR old_trace={:?} true_trace={:?} common_events={} outcome={}",
        old_work.trace, new_work.trace, new_work.events.len(), if new.is_ok() { "OK" } else { "ERR" });
    (new, new_work)
}
fn validate_trace(map: &Tomogram, route: &TracedRoute) {
    assert!(!route.trace.connections.is_empty());
    let mut next = 0;
    for record in &route.trace.connections {
        assert!(record.point_span[0] <= record.point_span[1] && record.point_span[1] < route.route.points.len());
        assert_eq!(record.changes.start, next);
        let changes = &route.trace.changes[record.changes.clone()];
        assert_eq!(changes.first().unwrap().sample_index, 0);
        assert!(record.steps >= 1);
        assert!(changes.iter().all(|c| c.sample_index <= record.steps));
        assert!(changes.windows(2).all(|p| p[0].sample_index < p[1].sample_index && p[0].reference != p[1].reference));
        assert_eq!(changes[0].reference, map.reference_start(record.evaluated_from).unwrap());
        for change in changes { if let Reference::Known(id) = change.reference { assert!(id < map.cells.len()); } }
        next = record.changes.end;
    }
    assert_eq!(next, route.trace.changes.len());
}
fn segment(map: &Tomogram, a: [f64; 3], b: [f64; 3]) -> SegmentTrace {
    let mut scratch = Some(SegmentTrace::default());
    assert!(map.reference_segment_recorded::<true>(a, b, &[], None, &mut scratch).is_some());
    scratch.unwrap()
}
fn reduced(map: &Tomogram, points: Vec<[f64; 3]>, penalties: &[crate::RoutePenalty],
    accepts: &dyn Fn([f64; 3], [f64; 3]) -> bool,
    raw: Option<RouteReferenceTrace>, tail: usize) -> (Vec<[f64; 3]>, RouteReferenceTrace) {
    let accept = |a: [f64; 3], b: [f64; 3]| { event(Event::Accept([a.map(f64::to_bits), b.map(f64::to_bits)])); accepts(a, b) };
    let (old, old_work) = observe(|| map.reduce(points.clone(), &[], penalties, None, &accept));
    let mut scratch = Some(SegmentTrace::default());
    let (new, new_work) = observe(|| map.reduce_recorded::<true>(points, &[], penalties, None, &accept, raw, tail, &mut scratch));
    assert_eq!(old_work.trace, [0; 7]);
    assert_eq!(old_work.events, new_work.events);
    let (new, trace) = new.unwrap();
    assert_eq!(bits(&old.unwrap()), bits(&new));
    (new, trace.unwrap())
}

#[test]
fn b1_01_same_kernel_layout_and_false_zero_recording() {
    eprintln!("B1_LAYOUT Reference={} TraceChange={} SearchState={} Route={} ParentEvidence={} OptionParent={}",
        std::mem::size_of::<Reference>(), std::mem::size_of::<TraceChange>(), std::mem::size_of::<SearchState>(),
        std::mem::size_of::<Route>(), std::mem::size_of::<ParentEvidence>(), std::mem::size_of::<Option<ParentEvidence>>());
    assert!(std::mem::size_of::<Option<ParentEvidence>>() <= 96);
    let map = map([]);
    let (plain, work) = observe(|| map.plan_bounded_impl::<false>([0.05,0.05,0.], [0.55,0.05,0.],
        0., &[], None, None, &[], &|_,_|true, SearchLimits::default()));
    assert!(plain.unwrap().trace.is_none());
    assert_eq!(work.trace, [0; 7]);
    let (route, _) = paired(&map, [0.05,0.05,0.], [0.55,0.05,0.], 0., &[], None, &[], &|_,_|true, SearchLimits::default(), None);
    let route = route.unwrap(); let clone = route.clone();
    assert!(Arc::ptr_eq(&route.trace, &clone.trace));
    assert_eq!(bits(&route.route.points), bits(&clone.route.points));
}
#[test]
fn b1_02_direct_known_unknown_and_long_walk_sources() {
    for columns in [vec![], vec![([0,1],vec![[-0.1,0.]])],
        vec![([0,1],vec![[-0.1,0.]]),([19,1],vec![[-0.1,0.]])], vec![([19,1],vec![[-0.1,0.]])]] {
        let map = map(columns); let before = format!("{:?}", map.cells);
        let (route, work) = paired(&map, [0.05,0.15,0.], [1.95,0.15,0.], 0., &BOUNDARY,
            None, &[], &|_,_|true, SearchLimits { max_states: 0 }, None);
        let route = route.unwrap(); assert_eq!(route.route.stats.expanded, 0); assert_eq!(work.trace[3], 0);
        assert!(route.trace.changes.iter().any(|c| matches!(c.reference, Reference::Unknown { .. })));
        assert_eq!(before, format!("{:?}", map.cells));
        for c in &route.trace.changes { if let Reference::Known(id) = c.reference {
            assert_eq!(map.cells[id].ground_approximation, nuwax_types::ground_layer::GroundApproximation::LegacyContactInterval);
        } }
    }
    let map = map([]);
    for length in [1.,10.,100.] {
        let (route, work) = paired(&map, [0.05,0.05,0.125], [length,0.05,0.125], 0., &[],
            None, &[], &|_,_|true, SearchLimits { max_states: 0 }, None);
        let route = route.unwrap(); assert_eq!(work.trace[3],0); assert_eq!(route.route.stats.expanded,0);
        assert!(route.trace.changes.len() <= (length / 0.1) as usize + 2);
        assert!(route.trace.changes.iter().all(|c| matches!(c.reference, Reference::Unknown { height, .. } if height == 0.125f64.to_bits())));
    }
}
#[test]
fn b1_03_single_point_keeps_micro_and_zero_connections() {
    let map = map([]); let start = [0.05,0.15,0.];
    for goal in [[0.0500005,0.15,0.], start] {
        let (route, _) = paired(&map, start, goal, 0., &[], None, &[], &|_,_|true, SearchLimits { max_states:0 }, None);
        let route = route.unwrap(); assert_eq!(route.route.points, vec![start]);
        let connection = &route.trace.connections[0];
        assert_eq!(connection.point_span,[0,0]); assert_eq!(connection.steps,1);
        assert_eq!(connection.evaluated_from,start); assert_eq!(connection.evaluated_to,goal);
    }
}
#[test]
fn b1_04_search_short_route_uses_admitted_parent_without_requery() {
    let penalty = [crate::RoutePenalty { center:[10.,0.,0.], radius:0.1, weight:1. }];
    for known in [false,true] {
        let map = map(if known { (0..5).map(|x|([x,0],vec![[-0.1,0.]])).collect::<Vec<_>>() } else { vec![] });
        for (end,boundary,expanded) in [([0.15,0.05,0.],[[0.,0.],[0.2,0.],[0.2,0.1],[0.,0.1]],2),
            ([0.45,0.05,0.],[[0.,0.],[0.5,0.],[0.5,0.1],[0.,0.1]],5)] {
            let (route, work) = paired(&map,[0.05,0.05,0.],end,0.,&boundary,Some(1.),&penalty,&|_,_|true,SearchLimits::default(),None);
            let route = route.unwrap(); assert_eq!(route.route.stats.expanded,expanded);
            assert_eq!(work.events.iter().filter(|e| matches!(e,Event::Neighbor(_))).count(),expanded-1);
            assert!(!route.trace.connections.is_empty());
            if expanded == 2 {
                assert_eq!(route.route.points.len(), 2);
                assert_eq!(route.trace.connections.len(), 2, "parent plus actual deduped connector");
                let parent = &route.trace.connections[0]; let connector = &route.trace.connections[1];
                assert_eq!(parent.point_span, [0,1]); assert_eq!(connector.point_span, [1,1]);
                assert_eq!(parent.evaluated_from, [0.05,0.05,0.]);
                assert_eq!(parent.evaluated_to, connector.evaluated_from);
                assert_eq!(connector.evaluated_to, end);
                let best = work.best.last().unwrap();
                assert!(best.precise); assert_eq!(best.from, connector.evaluated_from); assert_eq!(best.to, connector.evaluated_to);
                eprintln!("B1_PRECISE_WINNER {best:?} span={:?}", connector.point_span);
                let stored = &work.parents.last().unwrap();
                assert_eq!(parent.steps, stored.2);
                assert_eq!(&route.trace.changes[parent.changes.clone()], &stored.3);
            }
        }
    }
}
#[test]
fn b1_05_reduction_uses_winning_shortcut_reference_walk() {
    let map = map([([0,2],vec![[-0.1,0.]])]);
    let a=[0.05,0.05,0.];let b=[0.05,0.25,0.];let c=[0.25,0.05,0.];
    let (points,trace)=reduced(&map,vec![a,b,c],&[],&|_,_|true,Some(RouteReferenceTrace::default()),0);
    assert_eq!(points,vec![a,c]); assert_eq!(trace.connections.len(),1);
    assert_eq!(trace.connections[0].point_span,[0,1]);
    assert_eq!(trace.changes.iter().map(|c|map.reference_column(c.reference)).collect::<Vec<_>>(),vec![[0,0],[1,0],[2,0]]);
    assert!(trace.changes.iter().all(|c|matches!(c.reference,Reference::Unknown{..})));
}
#[test]
fn b1_06_failed_or_costlier_shortcut_cannot_contaminate_prefix() {
    let map=map([([0,2],vec![[-0.1,0.]])]);
    let a=[0.05,0.05,0.];let b=[0.05,0.25,0.];let c=[0.25,0.05,0.];
    let penalty=[crate::RoutePenalty{center:[0.15,0.05,0.],radius:0.04,weight:1000.}];
    for rejected in [false,true] {
        let accepts=|from,to|!rejected||!(from==a&&to==c);
        let (points,trace)=reduced(&map,vec![a,b,c],if rejected{&[]}else{&penalty},&accepts,Some(RouteReferenceTrace::default()),0);
        assert_eq!(points,vec![a,b,c]); assert_eq!(trace.connections.len(),2);
        assert_eq!(trace.connections.iter().map(|c|c.point_span).collect::<Vec<_>>(),vec![[0,1],[1,2]]);
        for (record,edge) in trace.connections.iter().zip(points.windows(2)) {
            let expected=segment(&map,edge[0],edge[1]);
            assert_eq!(trace.changes[record.changes.clone()],expected.changes);
            assert_eq!(record.steps,expected.steps);
        }
    }
}

#[test]
fn b1_08_two_floors_gap_and_each_segment_reseed_are_preserved() {
    let mut columns=std::collections::BTreeMap::new();
    for x in -20..20 { for y in -20..20 { columns.insert([x,y],vec![[-0.05,0.],[1.95,2.]]); } }
    for x in 20..100 { for y in -9..9 { let z=((x as f64+0.5)*0.1-2.)*0.25;
        columns.entry([x,y]).or_insert_with(Vec::new).push([z-0.05,z]); } }
    for x in 20..120 { for y in 10..23 { columns.entry([x,y]).or_insert_with(Vec::new).push([1.95,2.]); } }
    for x in 100..120 { for y in -9..10 { columns.entry([x,y]).or_insert_with(Vec::new).push([1.95,2.]); } }
    for gap in [false,true] {
        let columns=columns.clone().into_iter().filter(|(c,_)|!gap||c[0]!=50||c[1]>=10);
        let map=Tomogram::from_columns(0.1,columns,0.46,0.16,0.15,0.15,18.).unwrap();
        let (route,_)=paired(&map,[-1.,0.,0.],[-1.,0.,2.],0.15,&[],None,&[],&|_,_|true,SearchLimits::default(),None);
        let route=route.unwrap(); assert!(route.route.points.iter().any(|p|p[0]>9.));
        let low=map.reference_start([-1.,0.,0.]).unwrap(); let high=map.reference_start([-1.,0.,2.]).unwrap();
        assert_ne!(low,high); assert_eq!(map.reference_column(low),map.reference_column(high));
        assert!(route.trace.changes.iter().any(|c|matches!(c.reference,Reference::Known(_))));
        if gap { assert!(route.trace.changes.iter().any(|c|matches!(c.reference,Reference::Unknown{..}))); }
        eprintln!("B1_TWO_FLOORS gap={gap} connections={} changes={}",route.trace.connections.len(),route.trace.changes.len());
    }
}
#[test]
fn b1_09_collapsed_tail_spans_and_legal_representation_overflow() {
    let map=map([]);
    let a=[0.05,0.05,0.];let b=[0.15,0.05,0.];let c=[0.25,0.05,0.];
    let tail=[c,[0.2500002,0.05,0.],[0.2500004,0.05,0.],[0.2500006,0.05,0.]];
    let mut raw=RouteReferenceTrace::default();
    let mut raw_points=vec![a]; let mut previous=a; let mut tail_start=0;
    for edge in [[a,b],[b,c]].into_iter().chain(tail.windows(2).map(|e|[e[0],e[1]])) {
        let segment=segment(&map,edge[0],edge[1]);
        let parent=ParentEvidence::from_segment(&segment);
        let (span, from)=push_reference_point::<true>(&mut raw_points,edge[1],&mut previous,raw.connections.len()+1,&mut tail_start);
        assert_eq!(from,edge[0]);
        parent.append(&mut raw,span,from,edge[1]);
    }
    assert_eq!(raw_points,vec![a,b,c]); assert_eq!(tail_start,2);
    assert_eq!(raw.connections.iter().map(|r|r.point_span).collect::<Vec<_>>(),vec![[0,1],[1,2],[2,2],[2,2],[2,2]]);
    let (points,trace)=reduced(&map,raw_points,&[],&|_,_|true,Some(raw),tail_start);
    assert_eq!(points,vec![a,c]); assert_eq!(trace.connections.len(),4);
    assert_eq!(trace.connections[0].point_span,[0,1]);
    for (record,edge) in trace.connections[1..].iter().zip(tail.windows(2)) {
        assert_eq!(record.point_span,[1,1]); assert_eq!(record.evaluated_from,edge[0]);assert_eq!(record.evaluated_to,edge[1]);
    }
    let (points,trace)=reduced(&map,vec![a,a,[0.0500002,0.05,0.],b],&[],&|_,_|true,Some(RouteReferenceTrace::default()),0);
    assert_eq!(points,vec![a,b]);assert_eq!(trace.connections[0].point_span,[0,1]);
    // Saturating original i32 columns permit a long legal walk with one RLE
    // reference but >255 original steps. This exercises representation, not admission.
    let long=segment(&map,[1e10,0.05,0.],[1e10+16.,0.05,0.]);
    assert!(long.steps>255);assert_eq!(long.changes.len(),1);
    let evidence=ParentEvidence::from_segment(&long);assert!(matches!(evidence,ParentEvidence::Overflow{..}));
    let mut expanded=RouteReferenceTrace::default();evidence.append(&mut expanded,[0,1],[1e10,0.05,0.],[1e10+16.,0.05,0.]);
    assert_eq!(expanded.connections[0].steps,long.steps);assert_eq!(expanded.changes,long.changes);
    let many=segment(&map,a,[0.55,0.05,0.]);assert!(many.changes.len()>3);assert!(many.steps<255);
    assert!(matches!(ParentEvidence::from_segment(&many),ParentEvidence::Overflow{..}));
}
#[test]
fn b1_10_geometry_budget_radius_filter_and_cancellation_match() {
    for bands in [vec![[-0.1,1.]],vec![[0.2,0.3]],vec![[-0.6,-0.5]]] {
        let map=map((0..3).map(|y|([10,y],bands.clone())));
        assert!(paired(&map,[0.05,0.15,0.],[1.95,0.15,0.],0.,&BOUNDARY,None,&[],&|_,_|true,SearchLimits::default(),None).0.is_err());
    }
    let corner=map([([1,0],vec![[-0.1,1.]]),([0,1],vec![[-0.1,1.]])]);
    assert!(paired(&corner,[0.05,0.05,0.],[0.15,0.15,0.],0.,&[[0.,0.],[0.2,0.],[0.2,0.2],[0.,0.2]],None,&[],&|_,_|true,SearchLimits::default(),None).0.is_err());
    let map=map([]);let penalty=[crate::RoutePenalty{center:[1.,0.15,0.],radius:0.2,weight:4.}];
    assert!(paired(&map,[0.05,0.15,0.],[1.95,0.15,0.],0.,&BOUNDARY,None,&penalty,&|_,_|true,SearchLimits{max_states:0},None).0.is_err());
    assert!(paired(&map,[0.05,0.15,0.],[1.95,0.15,0.],0.,&BOUNDARY,Some(0.2),&penalty,&|_,_|true,SearchLimits{max_states:4096},None).0.is_err());
    assert!(paired(&map,[0.05,0.15,0.],[1.95,0.15,0.],0.,&BOUNDARY,None,&[],&|_,_|false,SearchLimits::default(),None).0.is_err());
    for n in [1,20,100] { assert!(paired(&map,[0.05,0.05,0.],[100.,0.05,0.],0.,&[],None,&[],&|_,_|true,SearchLimits::default(),Some(n)).0.is_err()); }
    // Existing unknown_arrival_ball fixture, unchanged: exact goal XY is
    // outside, while the original arrival ball admits the center candidate.
    let boundary=[[0.,0.],[0.57,0.],[0.57,0.3],[0.,0.3]];
    let goal=[0.59,0.15,0.];
    let (route,work)=paired(&map,[0.05,0.15,0.],goal,0.05,&boundary,None,&[],&|_,_|true,SearchLimits::default(),None);
    let route=route.unwrap(); let best=work.best.last().unwrap(); let end=*route.route.points.last().unwrap();
    assert!(!best.precise,"the fixed scene must exercise an actual center winner");
    assert_ne!(best.to,goal);assert_eq!(best.to,end);assert!(end[0]<=0.57&&distance(end,goal)<=0.05);
    let connector=route.trace.connections.last().unwrap();
    assert_eq!(connector.evaluated_from,best.from);assert_eq!(connector.evaluated_to,best.to);
    assert_eq!(connector.point_span,[route.route.points.len()-1;2]);
    eprintln!("B1_CENTER_WINNER {best:?} span={:?} from_bits={:?} to_bits={:?}",connector.point_span,
        connector.evaluated_from.map(f64::to_bits),connector.evaluated_to.map(f64::to_bits));
    let concave=[[0.,0.],[0.6,0.],[0.6,0.6],[0.4,0.6],[0.4,0.2],[0.2,0.2],[0.2,0.6],[0.,0.6]];
    assert!(paired(&map,[0.1,0.5,0.],[0.5,0.5,0.],0.,&concave,None,&[],&|_,_|true,SearchLimits::default(),None).0.is_ok());
}

#[path="pure_dominance_tests.rs"]
mod pure_dominance_tests;

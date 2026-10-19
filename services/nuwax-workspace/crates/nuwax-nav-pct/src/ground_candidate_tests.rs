//! Include as a #[cfg(test)] child of ground_route, not as an integration test.
//! All original owners below come from real plan_ground_traced results. No
//! RouteReferenceTrace, ConnectionRecord, or TraceChange is constructed here.
use super::*;
use crate::reference::SegmentTrace;

fn c1b_columns(known: bool) -> Arc<Tomogram> {
    Arc::new(Tomogram::from_columns(
        0.1,
        if known {
            vec![([0, 1], vec![[-0.1, 0.0]]), ([19, 1], vec![[-0.1, 0.0]])]
        } else {
            vec![]
        },
        0.4, 0.0, 0.1, 0.1, 20.0,
    ).unwrap())
}

fn c1b_direct_owner(map: Arc<Tomogram>, unanchored: bool) -> GroundRouteReference {
    // Unchanged endpoints/columns/boundary from B1-02. The independent
    // Unanchored request supplies only a physical query Z, never a ground Z.
    let z = if unanchored { 0.72 } else { 0.0 };
    let traced = map.plan_ground_traced(
        [0.05, 0.15, z], [1.95, 0.15, z], 0.0, &C1B_BOUNDARY,
        None, None, &[], &|_, _| true, SearchLimits { max_states: 0 },
        unanchored.then_some(z), None,
    ).unwrap();
    assert_eq!(traced.route.stats.expanded, 0);
    GroundRouteReference { map, trace: traced.trace }
}

const C1B_BOUNDARY: [[f64; 2]; 4] = [[0.0, 0.0], [2.0, 0.0], [2.0, 0.3], [0.0, 0.3]];

fn c1b_assert_association(sample: PlannedGroundSample<'_>, reference: Reference) {
    match (sample.association, reference) {
        // from_columns is legacy occupancy-derived geometry and deliberately
        // has no native SourceGroundRecord. Do not invent source evidence.
        (PlannedGroundAssociation::Known { source: None }, Reference::Known(_)) => {}
        (PlannedGroundAssociation::Carried, Reference::Unknown { .. }) => {}
        (PlannedGroundAssociation::Unanchored, Reference::Unanchored { .. }) => {
            assert_eq!(sample.point.ground_z, None);
        }
        (association, reference) => panic!("association {association:?} differs from {reference:?}"),
    }
}

// The pre-existing 3D segment entry with the same admitted initial reference,
// wrapped with the same cancellation boundary as the new XY-only entry. On
// these flat fixtures it must yield exactly the same original discrete walk.
fn c1b_core_segment(
    state: GroundWalkState<'_>, to: [f64; 2], physical_z: f64,
    cancelled: Option<&AtomicBool>,
) -> Result<(Reference, SegmentTrace), PlanningFailure> {
    checkpoint(cancelled)?;
    let mut scratch = Some(SegmentTrace::default());
    let result = state.map.reference_segment_from_recorded::<true>(
        [state.xy[0], state.xy[1], physical_z], [to[0], to[1], physical_z],
        &C1B_BOUNDARY, cancelled, &mut scratch, Some(state.reference),
    );
    checkpoint(cancelled)?;
    let (_, _, reference) = result.ok_or("candidate XY reference is incompatible with its source")?;
    Ok((reference, scratch.unwrap()))
}

#[test]
fn c1b_01_original_trace_addresses() {
    // Exact B1-04 short-search input. Its real admitted parent is followed by
    // the real deduped terminal connector; do not tune this to force a family.
    for known in [false, true] {
        let columns = if known {
            (0..5).map(|x| ([x, 0], vec![[-0.1, 0.0]])).collect::<Vec<_>>()
        } else { vec![] };
        let map = Arc::new(Tomogram::from_columns(
            0.1, columns, 0.4, 0.0, 0.1, 0.1, 20.0,
        ).unwrap());
        let penalty = [RoutePenalty { center: [10.0, 0.0, 0.0], radius: 0.1, weight: 1.0 }];
        let boundary = [[0.0, 0.0], [0.2, 0.0], [0.2, 0.1], [0.0, 0.1]];
        let traced = map.plan_ground_traced(
            [0.05, 0.05, 0.0], [0.15, 0.05, 0.0], 0.0, &boundary,
            None, Some(1.0), &penalty, &|_, _| true, SearchLimits::default(), None, None,
        ).unwrap();
        assert_eq!(traced.route.stats.expanded, 2);
        assert_eq!(traced.route.points.len(), 2);
        assert_eq!(traced.trace.connections.len(), 2);
        assert_eq!(traced.trace.connections[0].point_span, [0, 1]);
        assert_eq!(traced.trace.connections[1].point_span, [1, 1]);
        let original_trace = Arc::clone(&traced.trace);
        let owner = GroundRouteReference { map, trace: traced.trace };
        let connections = owner.connections().collect::<Vec<_>>();
        for (connection, record) in original_trace.connections.iter().enumerate() {
            assert_eq!(connections[connection].point_span, record.point_span);
            assert_eq!(connections[connection].steps, record.steps);
            let changes = &original_trace.changes[record.changes.clone()];
            for index in 0..=record.steps {
                let expected_reference = changes.iter().rev()
                    .find(|change| change.sample_index <= index).unwrap().reference;
                for fraction in if index < record.steps { &[0.0, 0.5][..] } else { &[0.0][..] } {
                    let address = owner.position(connection, index, *fraction).unwrap();
                    assert_eq!(address.indices(), (connection, index, *fraction));
                    let t = (index as f64 + *fraction) / record.steps as f64;
                    let expected_xy = std::array::from_fn(|axis|
                        record.evaluated_from[axis]
                            + t * (record.evaluated_to[axis] - record.evaluated_from[axis]));
                    let state = address.walk_state();
                    assert!(std::ptr::eq(state.map, owner.map.as_ref()));
                    assert_eq!(state.reference, expected_reference);
                    assert_eq!(state.xy(), expected_xy);
                    assert_eq!(address.is_terminal(),
                        connection + 1 == connections.len() && index == record.steps && *fraction == 0.0);
                    if *fraction == 0.0 {
                        let sample = owner.sample(connection, index).unwrap();
                        assert_eq!(sample.point.xy, expected_xy);
                        assert_eq!(sample.point.ground_z,
                            owner.map.reference_height_at(expected_reference, expected_xy));
                        c1b_assert_association(sample, expected_reference);
                    }
                }
            }
            assert!(owner.position(connection, record.steps + 1, 0.0).is_none());
            assert!(owner.sample(connection, record.steps + 1).is_none());
            assert!(owner.position(connection, record.steps, 0.5).is_none());
        }
        for fraction in [-0.1, 1.0, f64::NAN, f64::INFINITY] {
            assert!(owner.position(0, 0, fraction).is_none());
        }
        assert!(owner.position(connections.len(), 0, 0.0).is_none());
        assert!(owner.sample(connections.len(), 0).is_none());
        let window = owner.full_window().unwrap();
        assert_eq!(window.from().indices(), (0, 0, 0.0));
        assert_eq!(window.to().indices(), (1, original_trace.connections[1].steps, 0.0));
        let frozen = window.freeze();
        assert_eq!(frozen.window().from().indices(), window.from().indices());
        assert_eq!(frozen.window().to().indices(), window.to().indices());
        assert!(Arc::ptr_eq(&frozen.window().owner().trace, &original_trace));
        assert!(Arc::ptr_eq(&frozen.window().owner().map, &owner.map));
        let clone = owner.clone();
        assert!(window.from().window_to(clone.full_window().unwrap().to()).is_some());
        assert!(Arc::ptr_eq(&owner.trace, &original_trace), "accessors retain the actual search trace");
    }

    // Exact B1-03 zero/micro-connection inputs. The output point is deduped,
    // while both original discrete addresses and their evaluated XY survive.
    let start = [0.05, 0.15, 0.0];
    for goal in [[0.0500005, 0.15, 0.0], start] {
        let map = c1b_columns(false);
        let traced = map.plan_ground_traced(
            start, goal, 0.0, &[], None, None, &[], &|_, _| true,
            SearchLimits { max_states: 0 }, None, None,
        ).unwrap();
        assert_eq!(traced.route.points, vec![start]);
        assert_eq!(traced.trace.connections.len(), 1);
        assert_eq!(traced.trace.connections[0].point_span, [0, 0]);
        assert_eq!(traced.trace.connections[0].steps, 1);
        let owner = GroundRouteReference { map, trace: traced.trace };
        let first = owner.position(0, 0, 0.0).unwrap();
        let last = owner.position(0, 1, 0.0).unwrap();
        assert_ne!(first.indices(), last.indices());
        assert!(!first.is_terminal());
        assert!(last.is_terminal());
        assert_eq!(first.walk_state().xy(), [start[0], start[1]]);
        assert_eq!(last.walk_state().xy(), [goal[0], goal[1]]);
        if goal == start {
            assert_eq!(first.walk_state().semantic_key(), last.walk_state().semantic_key());
        } else {
            assert_ne!(owner.sample(0, 1).unwrap().point.xy, [start[0], start[1]]);
        }
    }
    // B1-09's multiple-parent collapsed tail was assembled from recorded
    // segments, not obtained from a fixed plan_ground_traced search fixture.
    // This test intentionally claims no real-search multi-parent-tail coverage.
}

#[test]
fn c1b_02_candidate_own_walk_and_cancel() {
    for unanchored in [false, true] {
        let owner = c1b_direct_owner(c1b_columns(true), unanchored);
        let original = owner.full_window().unwrap();
        let start = original.from().walk_state();
        let start_key = start.semantic_key();
        let physical_z = if unanchored { 0.72 } else { 0.0 };
        // Deliberately different XY geometry: row 2 has no ground records,
        // then the last edge returns to the original known destination row.
        let waypoints = [[0.05, 0.25], [1.95, 0.25], [1.95, 0.15]];
        let mut state = start;
        let mut segments = Vec::new();
        let mut expected = Vec::new();
        for to in waypoints {
            let (end_reference, core) = c1b_core_segment(state, to, physical_z, None).unwrap();
            let segment = state.walk_xy(to, &C1B_BOUNDARY, None).unwrap();
            assert_eq!(segment.trace.steps, core.steps);
            assert_eq!(segment.trace.changes, core.changes);
            assert_eq!(segment.end().reference, end_reference);
            assert_eq!(segment.end().xy(), to);
            expected.push((state.xy(), to, core));
            state = segment.end();
            segments.push(segment);
        }
        assert_eq!(start.semantic_key(), start_key, "walking does not consume or mutate the input state");
        let candidate = owner.freeze_candidate(segments).unwrap();
        assert_eq!(candidate.connection_count(), waypoints.len());
        assert!(Arc::ptr_eq(&candidate.map, &owner.map));
        assert!(candidate.end_matches(original.to()));
        let mut known_count = 0;
        let mut carried_count = 0;
        let mut unanchored_count = 0;
        for (connection, (from, to, core)) in expected.iter().enumerate() {
            assert_eq!(candidate.connection(connection), Some((*from, *to, core.steps)));
            assert_eq!(candidate.constant_association(connection), core.changes.len() == 1);
            for index in 0..=core.steps {
                let reference = core.changes.iter().rev()
                    .find(|change| change.sample_index <= index).unwrap().reference;
                let t = index as f64 / core.steps as f64;
                let xy = std::array::from_fn(|axis| from[axis] + t * (to[axis] - from[axis]));
                let sample = candidate.sample(connection, index).unwrap();
                assert_eq!(sample.point.xy, xy);
                assert_eq!(sample.point.ground_z, owner.map.reference_height_at(reference, xy));
                c1b_assert_association(sample, reference);
                match sample.association {
                    PlannedGroundAssociation::Known { .. } => known_count += 1,
                    PlannedGroundAssociation::Carried => carried_count += 1,
                    PlannedGroundAssociation::Unanchored => unanchored_count += 1,
                }
            }
            assert!(candidate.sample(connection, core.steps + 1).is_none());
        }
        assert!(candidate.connection(waypoints.len()).is_none());
        assert!(candidate.sample(waypoints.len(), 0).is_none());
        assert_eq!(candidate.sample(1, 0).unwrap().point.xy, waypoints[0]);
        assert_ne!(candidate.sample(1, 0).unwrap().point.xy[1], owner.sample(0, 0).unwrap().point.xy[1]);
        if unanchored {
            assert_eq!((known_count, carried_count), (0, 0));
            assert!(unanchored_count > 0);
        } else {
            assert!(known_count > 0 && carried_count > 0);
            assert_eq!(unanchored_count, 0);
            assert!(matches!(candidate.sample(0, 0).unwrap().association, PlannedGroundAssociation::Known { .. }));
            assert!(matches!(candidate.sample(1, 0).unwrap().association, PlannedGroundAssociation::Carried));
            let steps = candidate.connection(2).unwrap().2;
            assert!(matches!(candidate.sample(2, steps).unwrap().association, PlannedGroundAssociation::Known { .. }));
        }

        // Byte-for-byte identical columns still belong to another map owner.
        // Never transplant either Known ids or carried/unanchored references.
        let foreign = c1b_direct_owner(c1b_columns(true), unanchored);
        let foreign_window = foreign.full_window().unwrap();
        assert_ne!(start.semantic_key(), foreign_window.from().walk_state().semantic_key());
        assert!(original.from().window_to(foreign_window.to()).is_none());
        assert!(!candidate.end_matches(foreign_window.to()));
        let foreign_segment = foreign_window.from().walk_state()
            .walk_xy(waypoints[0], &C1B_BOUNDARY, None).unwrap();
        assert_eq!(owner.freeze_candidate(vec![foreign_segment]).unwrap_err(),
            PlanningFailure::Other("candidate reference owner changed".into()));
        // A different original trace is also not the same original address
        // owner, even when its Tomogram Arc is shared.
        let other_trace = c1b_direct_owner(Arc::clone(&owner.map), unanchored);
        assert!(original.from().window_to(other_trace.full_window().unwrap().to()).is_none());
        let other_mode = c1b_direct_owner(Arc::clone(&owner.map), !unanchored);
        let other_state = other_mode.full_window().unwrap().from().walk_state();
        assert!(std::ptr::eq(start.map, other_state.map));
        assert_eq!(start.xy(), other_state.xy());
        assert_ne!(start.semantic_key(), other_state.semantic_key(),
            "Known and Unanchored at identical XY are different navigation states");

        let cancelled = AtomicBool::new(true);
        let expected_cancel = PlanningFailure::Other("层析规划已取消".into());
        assert_eq!(start.walk_xy(waypoints[1], &C1B_BOUNDARY, Some(&cancelled)).unwrap_err(), expected_cancel);
        assert_eq!(c1b_core_segment(start, waypoints[1], physical_z, Some(&cancelled)).unwrap_err(), expected_cancel);
        let steps = ((waypoints[1][0] - start.xy()[0]).hypot(waypoints[1][1] - start.xy()[1])
            / (owner.map.resolution * 0.4)).ceil().max(1.0) as usize;
        // Before the walk, during the walk, and after its last evaluator
        // sample: preserve cancellation instead of reporting incompatibility.
        for after in [0, 1, 4, steps + 1] {
            let xy_error = crate::cancellation::after_checkpoints(after, ||
                start.walk_xy(waypoints[1], &C1B_BOUNDARY, None)).unwrap_err();
            let core_error = crate::cancellation::after_checkpoints(after, ||
                c1b_core_segment(start, waypoints[1], physical_z, None)).unwrap_err();
            assert_eq!(xy_error, core_error);
            assert_eq!(xy_error, expected_cancel);
        }
        assert_eq!(start.semantic_key(), start_key);
        assert!(start.walk_xy(waypoints[0], &C1B_BOUNDARY, None).is_ok(),
            "cancelled scratch/scopes cannot poison a later independent walk");
    }
}

#[test]
fn c1c4_01_retained_slices_keep_original_grid_boundaries() {
    // The same 1.1 m / q=.1 straight evaluator grid as C3-02: this source-less
    // map isolates addressing, not native physical admission.
    let owner=c1b_direct_owner(c1b_columns(true),false);
    let start=owner.full_window().unwrap().from().walk_state();
    let segment=start.walk_xy([1.15,0.15],&C1B_BOUNDARY,None).unwrap();
    assert_eq!(segment.trace.steps,28);
    let original=owner.freeze_candidate_from(start,vec![segment]).unwrap();
    let leaf=original.leaf(0).unwrap().crop(0.2,0.9).unwrap();
    let cropped=GroundCandidateReference {map:Arc::clone(&original.map),
        recording:CandidateRecording::Slices(vec![leaf.owned()].into())};
    assert_eq!(cropped.original_range(0),Some((0,[0.2,0.9])));
    assert_eq!(cropped.connection(0),original.connection(0),"diagnostic grid is not resampled");
    let actual=[0.21,0.17];
    for index in 6..=25 {
        let at=index as f64/28.0;
        let u=(at-0.2)/(0.9-0.2);
        for (t,expected) in [(u.next_down(),index-1),(u,index),(u.next_up(),index)] {
            let decoded=cropped.original_sample(0,t).unwrap();
            assert_eq!(decoded.0,expected,"index={index} t={t:?}");
            assert_eq!(original.original_slice_sample(0,0.2,0.9,t),Some(decoded));
            let nested=leaf.crop(t,1.0).unwrap();
            assert_eq!(nested.decode(0.0).unwrap().0,expected,"nested crop preserves side at {index}/{t:?}");
            let singleton=leaf.crop(t,t).unwrap();
            assert_eq!(singleton.decode(0.0).unwrap().0,expected);
            assert_eq!(singleton.decode(1.0).unwrap().0,expected);
            let state=cropped.walk_state_at(0,t,actual).unwrap();
            assert_eq!(state.reference,original.leaf(0).unwrap().reference(expected));
            assert_eq!(state.reference_sample_xy(),original.sample(0,expected).unwrap().point.xy);
            assert_eq!(state.xy(),actual);
        }
        assert_eq!(cropped.original_sample(0,u),Some((index,0.0)));
    }
    let u:f64=(13.0/28.0-0.2)/0.7;
    eprintln!("C4 original steps=28 range=[.2,.9] index13 u={u:?} before={:?} exact={:?} after={:?}",
        cropped.original_sample(0,u.next_down()),cropped.original_sample(0,u),cropped.original_sample(0,u.next_up()));
    for t in [0.0,1.0] {
        let expected=original.original_sample(0,if t==0.0 {0.2}else {0.9});
        assert_eq!(cropped.original_sample(0,t),expected);
    }
    let zero=original.leaf(0).unwrap().crop(13.0/28.0,13.0/28.0).unwrap();
    for t in [0.0,0.5,1.0] {assert_eq!(zero.decode(t),Some((13,0.0)));}
    for t in [-0.1,1.1,f64::NAN] {assert!(cropped.original_sample(0,t).is_none());}
    assert!(cropped.with_body_navigation_slice(0,0.0,1.0,actual,actual,|_|()).is_err(),
        "source-less address fixture cannot fabricate physical evidence");
    let stationary=owner.freeze_candidate_from(start,vec![]).unwrap();
    assert_eq!(stationary.connection_count(),0);
    assert_eq!(stationary.walk_state_at(0,0.0,start.xy()).unwrap().semantic_key(),start.semantic_key());
}

#[test]
fn c1c4_02_flat_splices_keep_original_trace_owners() {
    let owner=c1b_direct_owner(c1b_columns(true),false);
    let initial=owner.full_window().unwrap().from().walk_state();
    let mut state=initial;let mut segments=Vec::new();
    for xy in [[0.05,0.25],[1.95,0.25],[1.95,0.15]] {
        let edge=state.walk_xy(xy,&C1B_BOUNDARY,None).unwrap();state=edge.end();segments.push(edge);
    }
    let original=owner.freeze_candidate_from(initial,segments).unwrap();
    let trace=match &original.recording {CandidateRecording::Recorded(trace)=>Arc::clone(trace),_=>unreachable!()};
    let old_changes=trace.changes.clone();
    let actual=original.walk_state_at(1,0.1,[0.20,0.23]).unwrap();
    let first_cut=[0.05+0.25*(1.95-0.05),0.25];
    let connector=actual.walk_xy(first_cut,&C1B_BOUNDARY,None).unwrap();
    let prefix=owner.freeze_candidate_from(actual,vec![connector]).unwrap();
    let prefix_trace=match &prefix.recording {CandidateRecording::Recorded(t)=>Arc::clone(t),_=>unreachable!()};
    let first=original.prepend_to_suffix(&prefix,1,0.25).unwrap();
    let CandidateRecording::Slices(leaves)=&first.recording else {panic!("flat slices required")};
    assert_eq!(leaves.len(),3);assert!(Arc::ptr_eq(&leaves[0].trace,&prefix_trace));
    assert!(leaves[1..].iter().all(|leaf|Arc::ptr_eq(&leaf.trace,&trace)));
    assert_eq!(first.original_range(1),Some((1,[0.25,1.0])));
    assert!(first.is_splice_boundary(1));assert!(!first.is_splice_boundary(2));
    // A second splice retains the original trace directly, never the first
    // composed object. Both connector recordings come from the same evaluator.
    let incoming=first.walk_state_at(1,0.1,[0.60,0.23]).unwrap();
    let new_cut=0.25+0.5*(1.0-0.25);
    let segment=incoming.walk_xy([0.05+new_cut*(1.95-0.05),0.25],&C1B_BOUNDARY,None).unwrap();
    let second_prefix=owner.freeze_candidate_from(incoming,vec![segment]).unwrap();
    let second=first.prepend_to_suffix(&second_prefix,1,0.5).unwrap();
    let CandidateRecording::Slices(leaves)=&second.recording else {panic!("flat slices required")};
    assert_eq!(leaves.len(),3);assert!(leaves[1..].iter().all(|leaf|Arc::ptr_eq(&leaf.trace,&trace)));
    assert_eq!(second.original_range(1),Some((1,[new_cut,1.0])));
    assert_eq!(trace.changes,old_changes);
    assert!(second.end_matches(owner.full_window().unwrap().to()));
    let wrong_initial=owner.position(0,1,0.0).unwrap().walk_state();
    assert!(owner.freeze_candidate_from(wrong_initial,vec![initial.walk_xy([0.05,0.25],&C1B_BOUNDARY,None).unwrap()]).is_err());
    let foreign=c1b_direct_owner(c1b_columns(true),false);
    let foreign_start=foreign.full_window().unwrap().from().walk_state();
    let foreign_prefix=foreign.freeze_candidate_from(foreign_start,
        vec![foreign_start.walk_xy([0.05,0.25],&C1B_BOUNDARY,None).unwrap()]).unwrap();
    assert!(original.prepend_to_suffix(&foreign_prefix,1,0.25).is_err());
    eprintln!("C4 two flat splices: original leaves={}, final leaves={}, original changes={} unchanged",
        original.connection_count(),second.connection_count(),old_changes.len());
}

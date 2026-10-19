// Real controller regressions: complete static deltas need a fresh stopping
// check, but do not require ranking every control candidate again.
fn geometry_recheck_map(previous: &SurfaceSnapshot, extra: &[[i32; 3]]) -> Arc<SurfaceSnapshot> {
    let base = crate::surface_follower::test_snapshot(0.0, false);
    let size = base.source_voxel_size_m();
    let points: Vec<_> = base
        .cell_keys()
        .map(|(x, y, z)| [x, y, z])
        .chain(extra.iter().copied())
        .map(|p| {
            (
                (p[0] as f32 + 0.5) * size,
                (p[1] as f32 + 0.5) * size,
                (p[2] as f32 + 0.5) * size,
            )
        })
        .collect();
    Arc::new(
        SurfaceSnapshot::from_points(&points, previous.config().clone(), size)
            .unwrap()
            .with_body_envelope(previous.body_envelope().unwrap().clone())
            .unwrap(),
    )
}

fn geometry_recheck_update(
    previous: &Arc<SurfaceSnapshot>,
    next: Arc<SurfaceSnapshot>,
    columns: Vec<[i32; 2]>,
) -> ExecutionMap {
    ExecutionMap::SurfaceUpdate(surface_changes::SurfaceMapUpdate {
        planar_reference: None,
        change_basis: Some(previous.clone()),
        changes: Arc::new(surface_changes::SurfaceGeometryChanges::Columns {
            fine_size: next.source_voxel_size_m(),
            fine: columns.clone(),
            planner_size: next.config().voxel_size,
            coarse: columns,
        }),
        snapshot: next,
    })
}

#[test]
fn complete_columns_recheck_inflight_and_published_commands_without_reranking() {
    for published in [false, true] {
        // Materializing maps is producer work and can exceed a command lease.
        // Prepare it before selecting the command; do not relax that lease.
        let map = recent_recheck_source();
        let next = geometry_recheck_map(&map, &[[100, 100, 5]]);
        let later = [110, 120].map(|cell| (cell, geometry_recheck_map(&map, &[[cell, 100, 5]])));
        let (mut worker, mut map, counts, computed, release) = recent_recheck_fixture_with_map(map);
        assert!(
            matches!(computed.recv_timeout(Duration::from_secs(2)).unwrap(),
            PlannerStep::Command(c) if c != Twist2::ZERO)
        );
        let mut original = None;
        if published {
            release.send(()).unwrap();
            wait_recent_rechecked(&worker);
            original = Some(worker.shared.0.lock().unwrap().result.as_ref().unwrap().0);
        }
        assert!(!nuwax_nav_pct::same_source_geometry(&map, &next));
        worker
            .update_execution_map(geometry_recheck_update(
                &map,
                next.clone(),
                vec![[100, 100]],
            ))
            .unwrap();
        map = next;
        if !published {
            release.send(()).unwrap();
        }
        wait_recent_rechecked(&worker);
        let started = worker.shared.0.lock().unwrap().result.as_ref().unwrap().0;
        let mut same_deadline = original.is_none_or(|original| started == original);
        // Complete far additions and deletions must retain the same selection.
        for (cell, next) in later {
            worker
                .update_execution_map(geometry_recheck_update(
                    &map,
                    next.clone(),
                    vec![[cell - 10, 100], [cell, 100]],
                ))
                .unwrap();
            map = next;
            wait_recent_rechecked(&worker);
            same_deadline &= worker.shared.0.lock().unwrap().result.as_ref().unwrap().0 == started;
        }
        let count = counts.lock().unwrap();
        eprintln!(
            "COMPLETE_COLUMNS published={published} full={} rechecks={} recheck_us={:?}",
            count.full, count.rechecks, count.recheck_us
        );
        assert_eq!(
            count.full, 1,
            "complete far deltas restarted full candidate ranking"
        );
        assert!(count.rechecks >= 3);
        assert!(
            same_deadline,
            "geometry recheck must not renew the original control deadline"
        );
        drop(count);
        assert!(matches!(worker.step(0.3), PlannerStep::Command(c) if c != Twist2::ZERO));
        worker.stop();
    }
}

#[test]
fn complete_columns_static_obstacle_rejects_inflight_and_published_commands() {
    for published in [false, true] {
        let map = recent_recheck_source();
        let next = geometry_recheck_map(&map, &[[5, 10, 5]]);
        let (mut worker, map, counts, computed, release) = recent_recheck_fixture_with_map(map);
        assert!(
            matches!(computed.recv_timeout(Duration::from_secs(2)).unwrap(),
            PlannerStep::Command(c) if c != Twist2::ZERO)
        );
        if published {
            release.send(()).unwrap();
            wait_recent_rechecked(&worker);
        }
        assert!(
            next.validate_body_pose([0.5, 1.0, 0.5], [0.0, 0.0, 0.0, 1.0])
                .is_err(),
            "fixture must put a real static obstacle in the current body"
        );
        worker
            .update_execution_map(geometry_recheck_update(&map, next, vec![[5, 10]]))
            .unwrap();
        if !published {
            release.send(()).unwrap();
        }
        wait_recent_rechecked(&worker);
        assert!(!matches!(worker.step(0.3), PlannerStep::Command(c) if c != Twist2::ZERO));
        assert_eq!(
            counts.lock().unwrap().full,
            1,
            "fresh stopping rejection must not rerank"
        );
        worker.stop();
    }
}

#[test]
fn complete_columns_coalescing_to_unknown_retires_selected_command() {
    let map = recent_recheck_source();
    let later = [100, 110].map(|cell| (cell, geometry_recheck_map(&map, &[[cell, 100, 5]])));
    let (mut worker, mut map, counts, computed, release) = recent_recheck_fixture_with_map(map);
    computed.recv_timeout(Duration::from_secs(2)).unwrap();
    let generation = worker.shared.0.lock().unwrap().terrain_generation;
    for (cell, next) in later {
        // Each batch fits; their coalesced proof exceeds the existing scope cap.
        let mut columns = vec![[cell, 100]; surface_changes::MAX_REFERENCE_SCOPE_WORK / 3];
        if cell > 100 {
            columns.push([cell - 10, 100]);
        }
        worker
            .update_execution_map(geometry_recheck_update(&map, next.clone(), columns))
            .unwrap();
        map = next;
    }
    {
        let input = worker.shared.0.lock().unwrap();
        assert!(
            matches!(input.map.as_ref(), Some(ExecutionMap::SurfaceUpdate(update))
            if matches!(update.changes.as_ref(), surface_changes::SurfaceGeometryChanges::Unknown))
        );
        assert!(input.terrain_generation > generation,
            "a coalesced Unknown must fence the selected trajectory even when each input was complete");
    }
    release.send(()).unwrap();
    wait_recent_rechecked(&worker);
    assert!(counts.lock().unwrap().full >= 2);
    worker.stop();
}

#[test]
fn complete_columns_recheck_requires_continuity_and_unchanged_query_semantics() {
    let map = Arc::new(
        crate::surface_follower::test_snapshot(0.0, false)
            .with_body_envelope(SurfaceBodyEnvelope {
                fixed_collision: None,
                footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                z_bounds_m: [-0.05, 0.2],
                body_to_support_z_m: 0.4,
                max_step_up_m: 0.2,
                max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap(),
    );
    let previous = ExecutionMap::Surface(map.clone());
    for invalid in 0..4 {
        let next = geometry_recheck_map(&map, &[[100, 100, 5]]);
        let ExecutionMap::SurfaceUpdate(mut update) =
            geometry_recheck_update(&map, next, vec![[100, 100]])
        else {
            unreachable!()
        };
        match invalid {
            0 => update.change_basis = None,
            1 => update.changes = Arc::new(surface_changes::SurfaceGeometryChanges::Unknown),
            2 => update.changes = Arc::new(surface_changes::SurfaceGeometryChanges::Empty),
            _ => {
                let mut model = update.snapshot.body_envelope().unwrap().clone();
                model.z_bounds_m[1] += 0.1;
                update.snapshot = Arc::new(update.snapshot.with_body_envelope(model).unwrap());
            }
        }
        assert!(
            !recheckable_surface_update(Some(&previous), &ExecutionMap::SurfaceUpdate(update)),
            "invalid map continuity or semantics {invalid} allowed selected-command replay"
        );
    }
}

// Paired against the same test compiled before the private-container change.
mod footprint_container_experiment {
    use super::*;
    use std::{collections::BTreeSet, hint::black_box, time::Instant};
    type Pose = ([f64; 3], [f64; 4]);
    const NOW: u64 = 10_100_000_000;
    const AGE: u64 = 1_000_000_000;

    #[test]
    #[ignore = "fixed-input complete footprint/public/checker paired container experiment"]
    fn benchmark_container_paths() {
        for size in [0.05, 0.01] {
            let source = polygon_footprint_fixture(size, false);
            let poses: Vec<Pose> = (0..16).map(|i| {
                let yaw = 0.20 + f64::from(i) * 0.01;
                ([-0.08 + f64::from(i) * 0.01, -0.05, 0.5],
                 [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()])
            }).collect();
            for kind in ["private_complete", "public_complete", "checker_cold", "checker_repeat", "checker_distinct"] {
                for exemption in ["empty", "occupied"] {
                    let map = source.fork_for_planning();
                    let occupied = if exemption == "occupied" { map.body_observation_footprint(poses[0]).unwrap() }
                                   else { BTreeSet::new() };
                    // Equality warm-up is outside both variants' timing and recorded.
                    for &pose in &poses {
                        assert_eq!(map.body_observation_footprint(pose), measured_footprint_block_probe::<false>(&map, pose));
                    }
                    let mut checker = map.observed_body_motion_checker_with_expiry(&occupied, AGE, &[]);
                    if kind == "checker_repeat" { checker(poses[0], poses[0], NOW).unwrap(); }
                    #[cfg(feature="body-query-profile")] query_profile::begin();
                    let started = Instant::now();
                    for i in 0..64 {
                        let pose = black_box(if kind == "checker_distinct" { poses[i % poses.len()] } else { poses[0] });
                        match kind {
                            "private_complete" => {
                                let mut cache = ObservationFootprintCache::new(black_box(&map));
                                black_box(cache.cells(pose).unwrap());
                                drop(cache);
                            }
                            "public_complete" => { drop(black_box(map.body_observation_footprint(pose).unwrap())); }
                            "checker_cold" => {
                                let mut cold = map.observed_body_motion_checker_with_expiry(&occupied, AGE, &[]);
                                black_box(cold(pose, pose, NOW).unwrap());
                            }
                            _ => { black_box(checker(pose, pose, NOW).unwrap()); }
                        }
                    }
                    let wall_ns = started.elapsed().as_nanos();
                    #[cfg(feature="body-query-profile")] {
                        let record = query_profile::end();
                        eprintln!("CONTAINER_PATH {}", serde_json::json!({
                            "size":size,"kind":kind,"exemption":exemption,"loops":64,"wall_ns":wall_ns,
                            "snapshot_geometry_warmed_by_parity":true,"time_ns":NOW,
                            "stage_ns":record.wall_nanos,"calls":record.calls,"counters":record.counters,
                            "public_conversion_and_result_retirement_included":true}));
                    }
                    #[cfg(not(feature="body-query-profile"))]
                    eprintln!("CONTAINER_PATH {}", serde_json::json!({"size":size,"kind":kind,"exemption":exemption,"loops":64,"wall_ns":wall_ns}));
                }
            }
        }
    }

    fn cursor<'a>(cells: &'a [[i32;3]], occupied: &'a BTreeSet<[i32;3]>, seek: bool)
        -> impl Iterator<Item=&'a [i32;3]> + 'a {
        let first = cells.first().copied().unwrap_or([i32::MAX;3]);
        let mut range = occupied.range(first..);
        let mut next = range.next();
        cells.iter().filter(move |&&cell| {
            if seek && next.is_some_and(|key| *key < cell) {
                range = occupied.range(cell..); next = range.next();
            } else {
                while next.is_some_and(|key| *key < cell) { next = range.next(); }
            }
            if next == Some(&cell) { next = range.next(); false } else { true }
        })
    }

    #[test]
    #[ignore = "contains versus ordered cursors including in-range K worst case"]
    fn benchmark_container_difference() {
        let cells: Vec<_> = (0..1000).map(|x|[x,0,0]).collect();
        for (name, occupied) in [
            ("empty", BTreeSet::new()),
            ("all", cells.iter().copied().collect()),
            ("half", cells.iter().copied().filter(|c| c[0]%2==0).collect()),
            ("wrong_z", cells.iter().map(|c|[c[0],0,1]).collect()),
            ("large_prefix", (-100_000..0).map(|x|[x,0,0]).collect()),
            ("in_range_K", (0..1000).flat_map(|x|(-100..100).map(move|z|[x,0,z])).collect()),
        ] {
            let expected:Vec<_> = cells.iter().filter(|c| !occupied.contains(*c)).copied().collect();
            for seek in [false,true] { assert_eq!(cursor(&cells,&occupied,seek).copied().collect::<Vec<_>>(),expected); }
            assert_eq!(unoccupied_observation_cells(&cells,&occupied).copied().collect::<Vec<_>>(),expected);
            for round in 0..6 {
                for method in if round%2==0 { [0,1,2,3] } else { [3,2,1,0] } {
                    let started=Instant::now(); let mut count=0;
                    for _ in 0..100 {
                        let (cells,occupied)=black_box((&cells,&occupied));
                        count+=match method {
                            0=>cells.iter().filter(|c|!occupied.contains(*c)).count(),
                            1=>cursor(cells,occupied,false).count(),
                            2=>cursor(cells,occupied,true).count(),
                            _=>unoccupied_observation_cells(cells,occupied).count(),
                        };
                    }
                    let ns=started.elapsed().as_nanos();assert_eq!(black_box(count),expected.len()*100);
                    eprintln!("CONTAINER_DIFFERENCE {}",serde_json::json!({"case":name,"n":cells.len(),"k":occupied.len(),"method":method,"round":round,"loops":100,"wall_ns":ns}));
                }
            }
        }
    }
    fn assert_private_parity(map: &SurfaceSnapshot, pose: Pose) {
        let expected = measured_footprint_block_probe::<false>(map, pose);
        let mut cache = ObservationFootprintCache::new(map);
        let actual = cache.cells(pose).map(|cells| cells.to_vec());
        assert_eq!(actual, expected.clone().map(|cells| cells.into_iter().collect()));
        assert_eq!(map.body_observation_footprint(pose), expected);
        if let Ok(cells) = actual {
            assert!(cells.windows(2).all(|pair| pair[0] < pair[1]), "strict XYZ order and unique");
            assert!(cells.windows(2).all(|pair| pair[0][..2] != pair[1][..2]), "one z per XY");
        }
    }

    #[test]
    fn vec_is_strict_xyz_across_negative_block_edges_heights_and_current_view() {
        for size in [0.05, 0.01] {
            let map = polygon_footprint_fixture(size, false);
            for x in [-0.321, -0.32, -0.319, -0.01, 0.0, 0.319, 0.32, 0.321] {
                for yaw in [0.0_f64, 0.37, -1.1] {
                    assert_private_parity(&map, ([x,-0.05,0.5],[0.0,0.0,(yaw/2.0).sin(),(yaw/2.0).cos()]));
                }
            }
        }
        for (z,layers) in [(0,vec![-1,0,9,20,31,32]),(-10,vec![-11,-10,-1,0]),(32,vec![0,31,32,33])] {
            let map=level_selection_fixture(0.01,z,&layers);
            assert_private_parity(&map,([0.05,0.05,0.4+(f64::from(z)+1.0)*0.01],[0.0,0.0,0.0,1.0]));
        }
        let map=scene(&[],true).with_observations(Some(observation_grid(|cells|{cells.remove(&[4,0,0]);})));
        let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        let view=map.with_current_body_support(pose.0,pose.1).unwrap();
        assert!(view.current_body_support.is_some());assert_private_parity(&view,pose);
        assert!(view.observations.as_ref().unwrap().get([4,0,0]).is_none());
    }

    #[test]
    fn vec_complete_geometry_failure_precedes_evidence_and_keeps_success_cache() {
        let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        let map=scene(&[],false).with_observations(Some(observation_grid(|cells|{
            for value in cells.values_mut(){value.observed_ns=1;}
        })));
        // Prime the unchanged pose check, then inject one absent late patch
        // column to isolate constructor completion from the earlier pose guard.
        map.validate_body_pose(pose.0,pose.1).unwrap();
        let cells=measured_footprint_block_probe::<false>(&map,pose).unwrap();
        let last=*cells.last().unwrap();let first=*cells.first().unwrap();assert!(first<last);
        let mut cache=ObservationFootprintCache::new(&map);let valid=cache.cells(pose).unwrap().to_vec();
        let model=map.body_envelope().unwrap();
        let centre=map.locate_support_from_body(pose.0,pose.1,model.body_to_support_z_m).unwrap();
        let radius=model.radius()+model.z_bounds_m[0].abs().max(model.z_bounds_m[1].abs())+model.body_to_support_z_m+f64::from(map.config().voxel_size)*2.0;
        let patch=map.ground_patch(centre,radius);let mut broken=(*patch).clone();
        let size=f64::from(map.observations.as_ref().unwrap().size_m);let coarse=f64::from(map.config().voxel_size);
        let column=(((f64::from(last[0])+0.5)*size/coarse).floor() as i32,((f64::from(last[1])+0.5)*size/coarse).floor() as i32);
        assert!(broken.remove(&column).is_some());
        map.data.body_cache.lock().unwrap().insert((centre.cell,radius.to_bits(),centre.surface_xyz[2].to_bits()),Arc::new(broken));
        assert_eq!(measured_footprint_block_probe::<false>(&map,pose),Err(SurfaceError::MissingSupport));
        let none=BTreeSet::new();let mut checker=map.observed_body_motion_checker_with_expiry(&none,AGE,&[]);
        #[cfg(feature="body-query-profile")] query_profile::begin();
        assert_eq!(checker(pose,pose,NOW),Err(SurfaceError::MissingSupport));
        #[cfg(feature="body-query-profile")] {
            let r=query_profile::end();assert_eq!(r.calls[QueryStage::ObservationFootprint as usize],1);
            assert_eq!(r.calls[QueryStage::ObservationEvidence as usize],0);
        }
        // Failed geometry never replaces the last success, even for an ULP key.
        let other=([f64::from_bits(pose.0[0].to_bits()+1),pose.0[1],pose.0[2]],pose.1);
        assert!(cache.cells(other).is_err());assert!(cache.cells(other).is_err());
        assert_eq!(cache.cells(pose).unwrap(),valid.as_slice());assert_eq!(cache.builds,3);
    }

    #[test]
    fn vec_difference_keeps_full_xyz_first_error_and_expiry_edges() {
        let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);let base=scene(&[],false);
        let good=base.with_observations(Some(observation_grid(|_|{})));
        let cells=measured_footprint_block_probe::<false>(&good,pose).unwrap();let first=*cells.first().unwrap();
        let wrong:BTreeSet<_>=cells.iter().map(|c|[c[0],c[1],c[2]+1]).collect();
        let mut checker=good.observed_body_motion_checker_with_expiry(&wrong,AGE,&[]);
        assert_eq!(checker(pose,pose,11_000_000_000),Ok(Some(11_000_000_000)));
        assert_eq!(checker(pose,pose,11_000_000_001),Err(SurfaceError::NearFieldObservation{cell:first,reason:"观察已过期",observed_ns:10_000_000_000,checked_ns:11_000_000_001}));
        let bad=base.with_observations(Some(observation_grid(|values|{let e=values.get_mut(&first).unwrap();e.hits=1;e.observed_ns=11_000_000_000;})));
        let mut checker=bad.observed_body_motion_checker_with_expiry(&wrong,AGE,&[]);
        assert_eq!(checker(pose,pose,NOW),Err(SurfaceError::NearFieldObservation{cell:first,reason:"独立采样命中不足",observed_ns:11_000_000_000,checked_ns:NOW}));
        let mut exempt=bad.observed_body_motion_checker_with_expiry(&cells,AGE,&[]);
        assert_eq!(exempt(pose,pose,12_000_000_000),Ok(None));
        for observed in [u64::MAX-100,u64::MAX-99] {
            let saturated=base.with_observations(Some(observation_grid(|values|{for e in values.values_mut(){e.observed_ns=observed;}})));
            let mut checker=saturated.observed_body_motion_checker_with_expiry(&wrong,100,&[]);
            assert_eq!(checker(pose,pose,u64::MAX),Ok(None));
        }
    }

    #[test]
    fn vec_cursor_matches_contains_for_empty_disjoint_and_dense_xyz_sets() {
        for n in [0,1,17,256] {
            let cells:Vec<_>=(0..n).map(|i|[i/17-8,i%17-8,i%3-1]).collect();
            for shift in [-9,0,1,9] {
                let occupied:BTreeSet<_>=cells.iter().flat_map(|c|(-2..3).map(move|z|[c[0],c[1],c[2]+z+shift])).collect();
                let expected:Vec<_>=cells.iter().filter(|c|!occupied.contains(*c)).copied().collect();
                assert_eq!(unoccupied_observation_cells(&cells,&occupied).copied().collect::<Vec<_>>(),expected);
            }
        }
    }

}

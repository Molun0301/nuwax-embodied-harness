// P0: paired current original kernel (const false) and the single guarded skip.
// Included beside the existing assigned-column fixtures and exact-bit helpers.
fn p0_pair(make:impl Fn()->(SurfaceSnapshot,Pose),mode:u8)->(Result<(),SurfaceError>,Option<BodyRejection>) {
    use crate::surface::body_query::GroundSample;
    let (old,pose)=make();let (new,new_pose)=make();assert_eq!(pose,new_pose);
    assert!(!Arc::ptr_eq(&old.body_query_cache,&new.body_query_cache));
    if let (Some(a),Some(b))=(&old.observations,&new.observations) {
        assert!(!Arc::ptr_eq(&a.legacy_reference_cache,&b.legacy_reference_cache));
    }
    let sample=GroundSample {mode:crate::surface::body_query::GroundSampleMode::Compatibility,reference_origin:[pose.0[0],pose.0[1],
        pose.0[2]-old.body_envelope().unwrap().body_to_support_z_m],
        reference_normal:None,query_state:GroundQueryState::Unknown,reference_sources:vec![]};
    let sample=match mode {0=>QuerySample::Unanchored,1=>QuerySample::LegacyNominal,
        2=>QuerySample::Ground(&sample),_=>unreachable!()};
    let mut last=None;
    for phase in ["independent-cold","same-owner-warm"] {
        let mut ow=Some(poison());let mut nw=Some(poison());
        let a=old.check_body_pose_in_context_impl::<false>(pose.0,pose.1,sample,&mut ow);
        let b=new.check_body_pose_in_context_impl::<true>(pose.0,pose.1,sample,&mut nw);
        assert_result_bits(&b,&a);assert_eq!(rejection_bits(nw),rejection_bits(ow));
        eprintln!("P0_PAIR mode={mode} phase={phase} pose_bits={:?}/{:?} result={a:?} witness={:?}",
            pose.0.map(f64::to_bits),pose.1.map(f64::to_bits),rejection_bits(ow));
        last=Some((a,ow));
    }
    let (expected,rejection)=last.unwrap();
    // These are real shipping entry calls, including its exact pose cache.
    let expected_witness=rejection.map(|rejection|BodyFailureWitness{pose,rejection});
    for phase in ["entry-cold","entry-hit"] {
        let mut witness=Some(BodyFailureWitness {pose:([999.0;3],[1.0;4]),rejection:poison()});
        let result=new.validate_body_pose_in_context(pose.0,pose.1,sample,&mut witness);
        assert_result_bits(&result,&expected);
        assert_eq!(witness_bits(witness),witness_bits(expected_witness),"{phase}");
    }
    (expected,rejection)
}

#[test]
fn p0_unanchored_disjoint_scalar_work_is_unused_and_source_caches_remain_sound() {
    for (fine,coarse) in [(0.01,0.1),(0.05,0.1),(0.05,0.05)] {
        for measured in [false,true] {for complete in [false,true] {
            let make=||disjoint_contact_fixture(fine,coarse,measured,complete);
            // A third owner proves branch eligibility without prewarming either pair.
            let (fixture,pose)=make();let a=admit_lazy_contact_pose(&fixture,pose);
            assert_eq!((a.assigned,a.disjoint,a.overlapping),(1,1,0));
            for mode in [0,1,2] {assert_eq!(p0_pair(make,mode),(Ok(()),None));}
        }}
        // Exact cache-site observation, not an allocation or timing counter.
        let (old,pose)=disjoint_contact_fixture(fine,coarse,false,true);
        let (new,_)=disjoint_contact_fixture(fine,coarse,false,true);
        assert!(old.body_query_cache.lock().unwrap().fine_bands.is_empty());
        assert!(new.body_query_cache.lock().unwrap().fine_bands.is_empty());
        old.check_body_pose_in_context_impl::<false>(pose.0,pose.1,QuerySample::Unanchored,&mut None).unwrap();
        new.check_body_pose_in_context_impl::<true>(pose.0,pose.1,QuerySample::Unanchored,&mut None).unwrap();
        let n=(coarse/fine).round()as usize;
        assert_eq!(old.body_query_cache.lock().unwrap().fine_bands.len(),n*n);
        assert!(new.body_query_cache.lock().unwrap().fine_bands.is_empty());
        // Skipping a source-cache fill may not change a later Legacy consumer.
        let mut a=None;let mut b=None;
        let ar=old.check_body_pose_in_context(pose.0,pose.1,QuerySample::LegacyNominal,&mut a);
        let br=new.check_body_pose_in_context(pose.0,pose.1,QuerySample::LegacyNominal,&mut b);
        assert_result_bits(&ar,&br);assert_eq!(rejection_bits(a),rejection_bits(b));
        eprintln!("P0_SITE fine={fine} coarse={coarse} old_height_band_entries={} skipped_entries=0",n*n);
    }
}

#[test]
fn p0_unanchored_empty_absent_and_legacy_cross_slope_keep_original_contracts() {
    for variant in [0,1] {
        let make=|| {let (base,pose)=disjoint_contact_fixture(0.05,0.1,false,true);
            let observations=if variant==0 {None}else{Some(Arc::new(SurfaceObservations::new(0.05,32).unwrap()))};
            (base.with_observations(observations),pose)};
        for mode in [0,1,2] {assert_eq!(p0_pair(make,mode),(Ok(()),None));}
    }
    // This real multi-column slope catches accidentally replacing Unanchored by
    // sample.ground().is_none(), which would also skip the Legacy fit consumer.
    for fine in [0.01,0.05] {
        let make=||disjoint_cross_slope_fixture(fine);
        let (map,pose)=make();let a=admit_lazy_contact_pose(&map,pose);
        assert!(a.assigned>2 && a.assigned==a.disjoint && a.overlapping==0);
        let (_,b,_)=fit_plane(&a.ground_points,pose.0).unwrap();
        assert!(b.abs().atan().to_degrees()>map.body_envelope().unwrap().max_cross_slope_deg+1e-5);
        assert_eq!(p0_pair(make,1),(Err(SurfaceError::UntraversableSegment),None));
        assert_eq!(p0_pair(make,0),(Ok(()),None));
    }
}

#[test]
fn p0_unanchored_overlap_fringe_recent_and_invalid_pose_preserve_exact_witness() {
    // Original mixed footprint: disjoint columns precede a true high echo.
    for observed in [0,2] {
        let make=|| (indexed_z_fixture(0.05,0.1,observed),([-0.05,0.05,0.4],[0.0,0.0,0.0,1.0]));
        let (result,witness)=p0_pair(make,0);
        assert_eq!(result,Err(SurfaceError::InsufficientClearance));
        let expected=if observed==0 {OccupancySource::Fine}else{OccupancySource::CoarseFallback};
        assert!(matches!(witness,Some(BodyRejection::Occupancy{source,..}) if source==expected));
    }
    let measured=||single_column_fixture(0.05,0.1,0,&[[0,0,1]],&[],true,true);
    let (result,witness)=p0_pair(measured,0);
    assert_eq!(result,Err(SurfaceError::InsufficientClearance));
    assert!(matches!(witness,Some(BodyRejection::Occupancy{cell:[0,0,1],source:OccupancySource::Fine,..})));
    let fringe=|| {
        let mut points:Vec<_>=(-30..30).flat_map(|x|(-30..30).map(move|y|
            ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,0.075))).collect();
        points.push((0.075,0.025,0.525));
        let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=0.1;cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let source=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(model()).unwrap()
            .with_observations(Some(witnessed_observations(&[(0.075,0.025,0.525)],0.05)));
        (source,([f64::from(0.1_f32)+0.55,0.05,0.5],[0.0,0.0,0.0,1.0]))
    };
    let (result,witness)=p0_pair(fringe,0);assert_eq!(result,Err(SurfaceError::InsufficientClearance));
    assert!(matches!(witness,Some(BodyRejection::Occupancy{source:OccupancySource::FineFringe,cell:[1,0,10],..})));
    let recent=|| {let (map,pose)=disjoint_contact_fixture(0.05,0.1,false,true);
        let point=pose.0;
        (map.with_recent_obstacle_points(1,1,0,&[],[(0,point)]).unwrap(),pose)};
    let (result,witness)=p0_pair(recent,0);assert_eq!(result,Err(SurfaceError::InsufficientClearance));
    assert!(matches!(witness,Some(BodyRejection::Occupancy{source:OccupancySource::Recent,..})));
    let invalid=|| {let (map,mut pose)=disjoint_contact_fixture(0.05,0.1,false,true);pose.1=[0.0;4];(map,pose)};
    assert_eq!(p0_pair(invalid,0),(Err(SurfaceError::InvalidPose),None));
}

// Finite paired diagnostic of the pose-miss kernel, not whole PCT/search latency.
// Validation, formatting, fixture construction and destruction are outside timers.
fn p0_bounded_pair(order:[bool;4]) {
    use std::time::Instant;
    let mut empty_max=0_u128;
    for _ in 0..32 {let start=Instant::now();empty_max=empty_max.max(start.elapsed().as_nanos());}
    let mut all=Vec::new();
    for measured in [false,true] {
        for (arm,skip) in order.into_iter().enumerate() {
            let (map,pose)=disjoint_contact_fixture(0.01,0.1,measured,true);
            let mut output=Vec::with_capacity(32);
            for i in 0..32 {
                let mut at=pose;at.0[0]+=(f64::from(i%16)-7.5)*0.0001;
                let mut rejection=None;
                let (result,elapsed)=if skip {
                    let start=Instant::now();
                    let result=map.check_body_pose_in_context_impl::<true>(at.0,at.1,QuerySample::Unanchored,&mut rejection);
                    (result,start.elapsed().as_nanos())
                } else {
                    let start=Instant::now();
                    let result=map.check_body_pose_in_context_impl::<false>(at.0,at.1,QuerySample::Unanchored,&mut rejection);
                    (result,start.elapsed().as_nanos())
                };
                output.push((at,result,rejection,elapsed));
            }
            all.push((measured,arm,skip,output));
        }
    }
    // Encode only after all arms, so printing cannot perturb the query sequence.
    for (measured,arm,skip,output) in &all {
        for (i,(pose,result,witness,nanos)) in output.iter().enumerate() {
            assert_eq!(*result,Ok(()));assert!(witness.is_none());
            eprintln!("P0_TIME measured={measured} arm={arm} skip={skip} call={i} pose_bits={:?}/{:?} nanos={nanos} result={result:?} witness={:?}",
                pose.0.map(f64::to_bits),pose.1.map(f64::to_bits),rejection_bits(*witness));
        }
    }
    eprintln!("P0_TIMER empty_max_ns={empty_max} calls={} order={order:?}",all.len()*32);
}

#[test]
#[ignore="bounded P0 timing diagnostic; explicitly run only after correctness"]
fn p0_bounded_pose_miss_pair_abba() {p0_bounded_pair([false,true,true,false]);}

#[test]
#[ignore="bounded P0 timing diagnostic; explicitly run only after correctness"]
fn p0_bounded_pose_miss_pair_baab() {p0_bounded_pair([true,false,false,true]);}

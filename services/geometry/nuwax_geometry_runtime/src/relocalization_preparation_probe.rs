// Included only inside correction_coordinator's cfg(test) tests module.
// Preparation diagnostic: no ICP, actor scheduling change, hashes or new limits.
fn preparation_probe_points(count: usize) -> Vec<Point3<f64>> {
    (0..count).map(|i| Point3::new(
        (i % 1024) as f64 * 0.125,
        ((i / 1024) % 1024) as f64 * 0.125,
        (i / (1024 * 1024)) as f64 * 0.125,
    )).collect()
}

fn preparation_probe_rss() -> (usize, usize) {
    let text = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    let read = |key: &str| text.lines().find(|line| line.starts_with(key))
        .and_then(|line| line.split_whitespace().nth(1))
        .and_then(|value| value.parse::<usize>().ok()).unwrap_or(0);
    (read("VmRSS:"), read("VmHWM:"))
}

// The transformation loop is copied verbatim from seal_relocalization_source.
// Whole-seal timing below always calls the actual production helper; this
// separate stage is not subtracted from it or represented as actor integration.
fn preparation_probe_transform(
    accepted: &[AccumulatedRelocalizationFrame],
    aggregate_input_points: usize,
) -> Result<Vec<Point3<f64>>, String> {
    let mut ordered = Vec::with_capacity(aggregate_input_points);
    for frame in accepted {
        for body_point in frame.input.body_points() {
            let odom_point = frame.input.correlated_odom().odom_from_body * body_point;
            if !odom_point.x.is_finite()
                || !odom_point.y.is_finite()
                || !odom_point.z.is_finite()
            {
                return Err("RELOCALIZATION_SOURCE_POINT_NONFINITE".to_owned());
            }
            ordered.push(odom_point);
        }
    }
    if ordered.len() != aggregate_input_points {
        return Err("RELOCALIZATION_AGGREGATE_POINT_COUNT_MISMATCH".to_owned());
    }
    Ok(ordered)
}

fn preparation_probe_accumulation(
    request: &crate::proto::RelocalizeRequest,
    snapshot: &SealedAssignedActivatedTargetSnapshot,
    frames: &[Arc<CorrectionFrameInput>],
    count: usize,
) -> RelocalizationAccumulation {
    let boundary = frames.last().unwrap();
    RelocalizationAccumulation {
        request: validate_relocalize_request(request.clone()).unwrap(),
        admission_ordinal: 1,
        mode_epoch: boundary.relocalization_mode_epoch.unwrap(),
        accumulation_deadline: Instant::now() + Duration::from_secs(2),
        snapshot: snapshot.clone(), arm_watermark: None, armed_map_watermarks: HashMap::new(),
        accepted: frames.iter().rev().map(|frame| AccumulatedRelocalizationFrame {input: Arc::clone(frame)}).collect(),
        aggregate_input_points: count,
        session_map_frame_generation: Some(boundary.map_frame_generation()),
        mapping_commit_boot_nonce: Some(boundary.mapping_commit_id.boot_nonce),
    }
}

fn preparation_probe_expected(frames: &[Arc<CorrectionFrameInput>], total: usize) -> Vec<Point3<f64>> {
    let mut sorted = frames.to_vec();
    sorted.sort_by_key(|f| relocalization_frame_key(f));
    let ordered: Vec<_> = sorted.iter().flat_map(|frame| frame.body_points().iter()
        .map(move |point| frame.correlated_odom().odom_from_body * point)).collect();
    assert_eq!(ordered.len(), total);
    let count = total.min(RELOCALIZATION_MAX_EMITTED_POINTS);
    (0..count).map(|index| ordered[index * total / count]).collect()
}

fn preparation_probe_equal_points(left: &[Point3<f64>], right: &[Point3<f64>]) {
    assert_eq!(left.len(), right.len());
    for (a,b) in left.iter().zip(right) {
        for (x,y) in a.iter().zip(b.iter()) { assert_eq!(x.to_bits(),y.to_bits()); }
    }
}

#[tokio::test]
#[ignore = "bounded offline preparation attribution, explicitly run without ICP"]
async fn relocalization_preparation_actual_helpers_profile() {
    const WARMUPS: usize = 2;
    const SAMPLES: usize = 5;
    let selected_target = std::env::var("NEH_PREPARATION_TARGET_POINTS").ok()
        .map(|value| value.parse::<usize>().expect("target cohort"));
    for target_count in [100_000_usize, 1_000_000, 4_000_000] {
        if selected_target.is_some_and(|n| n != target_count) {continue;}
        let phase_a = CorrectionPhaseAControl::for_tests();
        let mut request = relocalization_command_request("req-preparation-profile", 7);
        let snapshot = install_relocalization_target_points_for_request(
            &phase_a, &mut request, "robot_map", preparation_probe_points(target_count),
        );
        let fence = phase_a.relocalization_mode();
        let epoch = fence.activate_for_tests().epoch;
        let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
        let target_owner = snapshot.checkpoint_owner.cloud();
        assert_eq!(target_owner.point_count() as usize, target_count);
        let resident_before = preparation_probe_rss();
        for aggregate in [50_000_usize, 100_000, 250_000] {
            let sizes: Vec<usize> = if aggregate == 250_000 {vec![100_000,100_000,50_000]} else {vec![aggregate]};
            let frames: Vec<Arc<CorrectionFrameInput>> = sizes.iter().enumerate().map(|(index,&count)| {
                // Exact binary translation; distinct per-frame odometry, immutable
                // actual ingress admission, source ownership and proof validation.
                let sequence = index as u64 + 1;
                let pose = Isometry3::translation(sequence as f64, -(sequence as f64)*2.0, 0.5);
                let odom = preparation_probe_points(count).iter().map(|p| pose * p).collect();
                let mut frame = frame_from_admitted_source(sequence, admit_frame_source(
                    sequence, sequence*10, sequence*10, pose, odom, &adapter, Arc::clone(&fence),
                ));
                frame.relocalization_mode_epoch = Some(epoch);
                relocalization_frame_evidence(&owner_identity(),"robot_map","robot_odom","robot_body",&frame).unwrap();
                Arc::new(frame)
            }).collect();
            let expected = preparation_probe_expected(&frames, aggregate);
            let boundary = frames.last().unwrap().as_ref().clone();
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            for sample in 0..WARMUPS + SAMPLES {
                let accumulation = preparation_probe_accumulation(&request,&snapshot,&frames,aggregate);
                let lease = Arc::new(CorrectionRequestLease::new(CorrectionRequestCancellation::new(),
                    Some(Arc::clone(&lifecycle).lock_owned().await)));
                let owned_boundary = boundary.clone();
                let sealing_lease = Arc::clone(&lease);
                let began = Instant::now();
                let sealed = seal_relocalization_source(accumulation,owned_boundary,sealing_lease,
                    RelocalizationCutoffReasonV1::MinimumAggregateReached).unwrap();
                let seal_ns = began.elapsed().as_nanos();
                let receipt_start = Instant::now();
                let receipts = sealed.frames.iter().map(|frame| assigned_source_receipt(&frame.input,
                    "robot_map","robot_odom","robot_body")).collect::<Result<Vec<_>,_>>().unwrap();
                let source_arc = Arc::clone(&sealed.source_points);
                let target_arc = Arc::clone(&sealed.target_points);
                let _base_anchor = sealed.boundary_input.base_anchor_map_from_odom();
                let _registration = sealed.boundary_input.source_owner.binding().registration_map_from_odom;
                let metadata_ns = receipt_start.elapsed().as_nanos();
                let prep_ns = began.elapsed().as_nanos();
                assert_eq!(receipts.len(),frames.len());
                preparation_probe_equal_points(&sealed.source_points,&expected);
                assert_eq!(sealed.target_points.len(),target_count);
                for (point,bytes) in sealed.target_points.iter().zip(target_owner.xyz_f32_le().chunks_exact(12)) {
                    for axis in 0..3 {assert_eq!(point[axis].to_bits(),f64::from(f32::from_le_bytes(bytes[axis*4..axis*4+4].try_into().unwrap())).to_bits());}
                }
                assert!(sealed.snapshot.checkpoint_owner.shares_source_with(&snapshot.checkpoint_owner));
                assert!(sealed.boundary_lease.has_lifecycle_guard());
                assert!(lifecycle.try_lock().is_err());
                for (sealed_frame,original) in sealed.frames.iter().zip(&frames) { assert!(Arc::ptr_eq(&sealed_frame.input,original)); }
                assert!(Arc::ptr_eq(&source_arc,&sealed.source_points));assert!(Arc::ptr_eq(&target_arc,&sealed.target_points));
                let drop_start=Instant::now();drop(source_arc);drop(target_arc);drop(sealed);drop(receipts);lease.release();drop(lease);
                let teardown_ns=drop_start.elapsed().as_nanos();assert!(lifecycle.try_lock().is_ok());

                // Independent stage samples. Inputs prepared outside each timer.
                let stage_accumulation=preparation_probe_accumulation(&request,&snapshot,&frames,aggregate);
                let mut accepted=stage_accumulation.accepted;accepted.sort_by_key(|f|relocalization_frame_key(&f.input));
                let start=Instant::now();let transformed=preparation_probe_transform(&accepted,aggregate).unwrap();let transform_ns=start.elapsed().as_nanos();
                let transformed_capacity_bytes=transformed.capacity()*std::mem::size_of::<Point3<f64>>();
                let start=Instant::now();let reduced=stable_stratified_source(transformed).unwrap();let reduce_ns=start.elapsed().as_nanos();
                let reduced_capacity_bytes=reduced.capacity()*std::mem::size_of::<Point3<f64>>();
                let start=Instant::now();let handed:Arc<[Point3<f64>]>=Arc::from(reduced);let source_handoff_ns=start.elapsed().as_nanos();
                preparation_probe_equal_points(&handed,&expected);drop(handed);
                let start=Instant::now();let decoded=decode_relocalization_target(target_owner).unwrap();let decode_ns=start.elapsed().as_nanos();
                let decoded_capacity_bytes=decoded.capacity()*std::mem::size_of::<Point3<f64>>();
                let start=Instant::now();let handed:Arc<[Point3<f64>]>=Arc::from(decoded);let target_handoff_ns=start.elapsed().as_nanos();drop(handed);
                if sample>=WARMUPS {
                    let resident_after=preparation_probe_rss();
                    println!("PREPARATION_RESULT {}",serde_json::json!({"target_points":target_count,"source_points":aggregate,"frames":frames.len(),"sample":sample-WARMUPS,
                        "seal_ns":seal_ns,"post_seal_metadata_ns":metadata_ns,"prep_ns":prep_ns,"teardown_ns":teardown_ns,
                        "independent_transform_ns":transform_ns,"independent_reduce_ns":reduce_ns,"independent_source_arc_ns":source_handoff_ns,
                        "independent_decode_ns":decode_ns,"independent_target_arc_ns":target_handoff_ns,
                        "transformed_capacity_bytes":transformed_capacity_bytes,"reduced_capacity_bytes":reduced_capacity_bytes,"decoded_capacity_bytes":decoded_capacity_bytes,
                        "source_arc_payload_bytes":expected.len()*24,"target_arc_payload_bytes":target_count*24,"canonical_payload_bytes":target_owner.xyz_f32_le().len(),
                        "rss_before_cohort_kib":resident_before.0,"hwm_before_cohort_kib":resident_before.1,"rss_after_sample_kib":resident_after.0,"hwm_after_sample_kib":resident_after.1}));
                }
            }
            // Exact helper guard/owner/count errors; no source mutation admitted.
            let no_guard=Arc::new(CorrectionRequestLease::new(CorrectionRequestCancellation::new(),None));
            assert_eq!(seal_relocalization_source(preparation_probe_accumulation(&request,&snapshot,&frames,aggregate),boundary.clone(),no_guard,
                RelocalizationCutoffReasonV1::MinimumAggregateReached).err().unwrap(),"RELOCALIZATION_BOUNDARY_LIFECYCLE_GUARD_MISSING");
            for (count,error) in [(aggregate-1,"RELOCALIZATION_AGGREGATE_POINT_COUNT_MISMATCH"),(250_001,"RELOCALIZATION_AGGREGATE_BYTES_EXCEEDED")] {
                let lease=Arc::new(CorrectionRequestLease::new(CorrectionRequestCancellation::new(),Some(Arc::clone(&lifecycle).lock_owned().await)));
                assert_eq!(seal_relocalization_source(preparation_probe_accumulation(&request,&snapshot,&frames,count),boundary.clone(),Arc::clone(&lease),
                    RelocalizationCutoffReasonV1::MinimumAggregateReached).err().unwrap(),error);assert!(lifecycle.try_lock().is_err());lease.release();assert!(lifecycle.try_lock().is_ok());
            }
            let lease=Arc::new(CorrectionRequestLease::new(CorrectionRequestCancellation::new(),Some(Arc::clone(&lifecycle).lock_owned().await)));
            let mut mismatch=boundary.clone();mismatch.relocalization_mode_epoch=Some(epoch+1);
            assert_eq!(seal_relocalization_source(preparation_probe_accumulation(&request,&snapshot,&frames,aggregate),mismatch,Arc::clone(&lease),
                RelocalizationCutoffReasonV1::MinimumAggregateReached).err().unwrap(),"RELOCALIZATION_BOUNDARY_OWNER_MISMATCH");lease.release();assert!(lifecycle.try_lock().is_ok());
        }
    }
}

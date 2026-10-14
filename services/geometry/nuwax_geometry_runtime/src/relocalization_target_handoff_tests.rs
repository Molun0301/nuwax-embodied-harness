// cfg(test)-only target ownership regression. Source-array representation,
// canonical receipts, cutoffs, lifecycle guard and compute are unchanged.
#[tokio::test]
async fn relocalization_target_vec_handoff_retains_allocation_and_last_reader() {
    for target_count in [3_usize, 100_000] {
        let phase_a=CorrectionPhaseAControl::for_tests();
        let mut request=relocalization_command_request("req-target-vec-owner",7);
        let mut target=preparation_probe_points(target_count);
        target[0]=Point3::new(-0.0,0.0,1.0);
        let snapshot=install_relocalization_target_points_for_request(&phase_a,&mut request,"robot_map",target);
        let canonical=snapshot.checkpoint_owner.cloud();
        // Exactly the production decode + Arc::new construction: the point
        // allocation/capacity moves into the Arc metadata allocation unchanged.
        let decoded=decode_relocalization_target(canonical).unwrap();
        let pointer=decoded.as_ptr();let capacity=decoded.capacity();
        let retained=Arc::new(decoded);
        assert_eq!(retained.as_ref().as_ptr(),pointer);
        assert_eq!(retained.capacity(),capacity);
        assert_eq!(retained[0].x.to_bits(),(-0.0_f64).to_bits());
        let clone=Arc::clone(&retained);let weak=Arc::downgrade(&retained);
        assert!(Arc::ptr_eq(&retained,&clone));drop(retained);
        assert_eq!(clone.as_ref().as_ptr(),pointer);assert!(weak.upgrade().is_some());
        drop(clone);assert!(weak.upgrade().is_none());

        let fence=phase_a.relocalization_mode();let epoch=fence.activate_for_tests().epoch;
        let adapter=frame_source_adapter("mapping-session-owner",Isometry3::identity());
        let pose=Isometry3::translation(1.0,-2.0,0.5);
        let mut frame=frame_from_admitted_source(1,admit_frame_source(1,10,10,pose,
            preparation_probe_points(3).iter().map(|p|pose*p).collect(),&adapter,Arc::clone(&fence)));
        frame.relocalization_mode_epoch=Some(epoch);
        let frames=vec![Arc::new(frame)];let boundary=frames[0].as_ref().clone();
        let expected=preparation_probe_expected(&frames,3);
        let lifecycle=Arc::new(tokio::sync::Mutex::new(()));
        let lease=Arc::new(CorrectionRequestLease::new(CorrectionRequestCancellation::new(),
            Some(Arc::clone(&lifecycle).lock_owned().await)));
        let sealed=seal_relocalization_source(preparation_probe_accumulation(&request,&snapshot,&frames,3),boundary,Arc::clone(&lease),
            RelocalizationCutoffReasonV1::MinimumAggregateReached).unwrap();
        // Compile-time assertion of the actual private field's owner shape.
        let reader:&Arc<Vec<Point3<f64>>>=&sealed.target_points;
        let pointer=reader.as_ref().as_ptr();let last_reader=Arc::clone(reader);let weak=Arc::downgrade(reader);
        assert!(Arc::ptr_eq(reader,&last_reader));
        assert_eq!(reader.capacity(),target_count);
        preparation_probe_equal_points(&sealed.source_points,&expected);
        let decoded=decode_relocalization_target(canonical).unwrap();
        preparation_probe_equal_points(reader.as_slice(),&decoded);
        assert!(sealed.snapshot.checkpoint_owner.shares_source_with(&snapshot.checkpoint_owner));
        assert!(Arc::ptr_eq(&sealed.frames[0].input,&frames[0]));
        assert!(lifecycle.try_lock().is_err());
        drop(sealed);
        assert!(lifecycle.try_lock().is_err()); // Explicit original lease owner remains.
        lease.release();assert!(lifecycle.try_lock().is_ok());
        assert_eq!(last_reader.as_ref().as_ptr(),pointer);
        preparation_probe_equal_points(last_reader.as_slice(),&decoded);
        assert!(weak.upgrade().is_some());drop(last_reader);assert!(weak.upgrade().is_none());
        // No target or checkpoint data were mutated by readers or lease release.
        preparation_probe_equal_points(&decode_relocalization_target(canonical).unwrap(),&decoded);
    }
}

//! Assigned planar codec/lineage checks. Native lifecycle qualification is separate.
use crate::proto::*;
use crate::relocalization_identity::{AssignedCloudRelocalizeRequestIdentityV2 as Identity, CanonicalRelocalizationPolicyV2};
use crate::relocalization_terminal_v2::{AssignedRelocalizationTerminalV2 as Terminal, AssignedRelocalizationTerminalQueryV2 as Query};
use prost::Message;

struct NoHash;
impl NoHash { fn enter() -> Self { crate::map_checksum::FORBID_HASH.with(|value| value.set(true)); Self } }
impl Drop for NoHash { fn drop(&mut self) { crate::map_checksum::FORBID_HASH.with(|value| value.set(false)); } }

pub(crate) fn wire() -> RelocalizeRequest {
    let occupancy = |id: &str, generation, backend_generation| CanonicalOccupancyCommitReceipt {
        commit_id: Some(MappingCommitIdentity { boot_nonce: vec![7;16], sequence: 11 }),
        session_id: id.into(), session_generation: generation, occupancy_revision: 12,
        source_timestamp_ns: 13, backend_kind: "slam_toolbox".into(), backend_generation,
    };
    RelocalizeRequest { schema_version: 2, request_id: "planar:seed".into(),
        assigned_target: Some(relocalize_request::AssignedTarget::AssignedPlanarV2(AssignedPlanarRelocalizeRequestV2 {
            target: Some(InstalledTargetReceiptV2 {
                activation: Some(InstalledActivationIdentityV2 {
                    request: Some(ActivationRequestMetadataV2 { activation_request_id: "activate:planar".into(),
                        session_id: "destination".into(), map_name: "warehouse".into(),
                        asset_identity: Some(AssetIdentityV2 { asset_id: "a".repeat(32) }), checkpoint_id: "b".repeat(32),
                        expected_runtime_generation: 3 }), runtime_generation: 3, lifecycle_epoch: 2,
                    session_generation: 5, backend_kind: "slam_toolbox".into(), backend_generation: 7 }),
                source_graph_member: Some(CheckpointMemberIdentity { checkpoint_id: "b".repeat(32), role: CheckpointMemberRole::PlanarGraph as i32,
                    occupancy_receipt: Some(occupancy("source",1,2)), cloud_receipt: None,
                    frame_id: "map".into(), frame_generation: 0, producer_generation: 0 }),
                occupancy_receipt: Some(occupancy("destination",5,7)), cloud_receipt: None, source_cloud_member: None }),
            policy: Some(CanonicalRelocalizationPolicyV2::current_planar().to_wire()), logical_timeout_ms: 0,
            seed: Some(PlanarSeed { x: -0.0, y: 2.0, yaw: 0.1 }),
        })), ..Default::default() }
}
fn body(request: &mut RelocalizeRequest) -> &mut AssignedPlanarRelocalizeRequestV2 {
    let Some(relocalize_request::AssignedTarget::AssignedPlanarV2(body)) = request.assigned_target.as_mut() else { panic!("planar fixture") }; body
}
pub(crate) fn terminal(applied: bool) -> RelocalizeResult {
    let request = Identity::admit(wire()).unwrap().to_wire();
    let mut evidence = AssignedPlanarRelocalizationEvidenceV2 { schema_version: 2,
        phase: AssignedRelocalizationEvidencePhaseV2::PreAdmission as i32, ..Default::default() };
    if applied {
        evidence.phase = AssignedRelocalizationEvidencePhaseV2::Finalized as i32;
        evidence.admission_ordinal = Some(1);
        evidence.owner = Some(RelocalizationOwnerReceiptV2 { runtime_generation: 3, correction_backend_generation: 9,
            correction_lease_boot_nonce: vec![4;16], correction_lease_generation: 8,
            map_frame_id: "map".into(), odom_frame_id: "odom".into(), body_frame_id: "body".into(), relocalization_epoch: Some(1) });
        evidence.previous_process = "1".repeat(32); evidence.committed_process = "2".repeat(32); evidence.mode_epoch = 1;
        evidence.accepted_scans = (0..3).map(|sequence| PlanarRelocalizationScanV2 {
            process_owner: "2".repeat(32), session_generation: 5, mode_epoch: 1, source_sequence: sequence,
            source_timestamp_ns: sequence+100, source_unix_ns: sequence+100, matching_response_bits: 0.3_f64.to_bits(),
        }).collect();
        evidence.estimate = Some(PlanarRelocalizationEstimateV2 { source_timestamp_ns: 102, clock_domain: "unix_ns".into(), clock_id: String::new(),
            source_sequence: 2, graph_revision: 1, optimizations: 0, anchor_odom_from_base_bits: vec![0;3],
            odom_from_base_bits: vec![0;3], map_from_base_bits: vec![0;3], map_from_odom_bits: vec![0;3], covariance_bits: vec![0;9],
            matching_response_bits: 0.3_f64.to_bits(), local_response_bits: 0.3_f64.to_bits(), disposition: "accepted".into(), node_count: 4, edge_count: 3 });
        evidence.frame_generation = 7; evidence.source_unix_ns = 102; evidence.committed_unix_ns = 103;
    }
    RelocalizeResult { schema_version: 2, request_id: request.request_id.clone(), assigned_planar_v2: Some(AssignedPlanarRelocalizeResultV2 {
        request: Some(request), disposition: if applied { RelocalizeDisposition::Applied } else { RelocalizeDisposition::TargetUnavailable } as i32,
        correction_revision: u64::from(applied), detail: "native planar outcome".into(), evidence: Some(evidence),
    }), ..Default::default() }
}

#[test]
fn planar_c_request_preserves_seed_bits_source_and_logical_deadline_without_hash() {
    let _guard = NoHash::enter();
    let first = Identity::admit(wire()).unwrap();
    assert!(first.is_planar()); assert_eq!(first.logical_timeout_ms(),30_000);
    assert_eq!(first.planar_seed().unwrap().x.to_bits(),0_f64.to_bits());
    let encoded = first.to_wire().encode_to_vec();
    assert_eq!(Identity::admit(RelocalizeRequest::decode(encoded.as_slice()).unwrap()).unwrap(),first);
    let mut changed = wire(); body(&mut changed).seed.as_mut().unwrap().x = 1.0;
    assert_ne!(Identity::admit(changed).unwrap(),first);
    let mut changed = wire(); body(&mut changed).logical_timeout_ms = 250;
    assert_ne!(Identity::admit(changed).unwrap(),first);
    let mut changed = wire(); body(&mut changed).target.as_mut().unwrap().activation.as_mut().unwrap().lifecycle_epoch += 1;
    assert_ne!(Identity::admit(changed).unwrap(),first);
}

#[test]
fn planar_c_mixed_cloud_missing_seed_and_policy_substitution_fail_closed() {
    let _guard = NoHash::enter();
    let mutations: [fn(&mut RelocalizeRequest);6] = [
        |r| body(r).seed=None, |r| body(r).seed.as_mut().unwrap().yaw=f64::NAN,
        |r| body(r).policy=Some(CanonicalRelocalizationPolicyV2::current_cloud().to_wire()),
        |r| body(r).target.as_mut().unwrap().cloud_receipt=Some(CanonicalCloudCommitReceipt::default()),
        |r| body(r).target.as_mut().unwrap().source_cloud_member=Some(CheckpointMemberIdentity::default()),
        |r| body(r).target.as_mut().unwrap().occupancy_receipt.as_mut().unwrap().session_generation += 1,
    ];
    for mutation in mutations { let mut request=wire(); mutation(&mut request); assert!(Identity::admit(request).is_err()); }
}

#[test]
fn planar_c_applied_terminal_and_existing_durable_codec_replay_without_hash() {
    let _guard = NoHash::enter();
    let first=Terminal::from_encoded((terminal(true)).encode_to_vec()).unwrap();
    assert!(first.result().assigned_cloud_v2.is_none());
    let encoded=first.encode_durable().unwrap();
    assert_eq!(Terminal::decode_durable(&encoded).unwrap(),first);
    let query=Query::admit(GetRelocalizationTerminalRequest { schema_version:2,request_id:first.request().request_id().into(),
        assigned_request_v2:Some(first.request().to_wire()),..Default::default() }).unwrap();
    for status in [RelocalizationTerminalStatus::Pending,RelocalizationTerminalStatus::NotFound,RelocalizationTerminalStatus::IdentityConflict] {
        let response=query.response(status,None,None); assert!(response.terminal.is_none()); assert_eq!(response.status,status as i32);
    }
    let response=query.response(RelocalizationTerminalStatus::Terminal,Some(&first),None);
    assert_eq!(response.terminal,Some(first.to_wire()));
}



#[test]
fn planar_c_cancel_and_unknown_query_do_not_fabricate_scan_or_cloud_evidence() {
    let _guard = NoHash::enter();
    for disposition in [RelocalizeDisposition::Cancelled,RelocalizeDisposition::DeadlineExceeded,RelocalizeDisposition::OwnerFailed] {
        let mut value=terminal(false);value.assigned_planar_v2.as_mut().unwrap().disposition=disposition as i32;
        let value=Terminal::from_encoded((value).encode_to_vec()).unwrap();assert_eq!(Terminal::decode_durable(&value.encode_durable().unwrap()).unwrap(),value);
    }
    let mut malformed=terminal(true);malformed.assigned_cloud_v2=Some(AssignedCloudRelocalizeResultV2::default());
    assert!(Terminal::from_encoded((malformed).encode_to_vec()).is_err());
}

//! Actual Python schema-1 import outputs, admitted through the existing v2 store.
//! This synchronous slice intentionally does not claim activation/publication coverage.
use super::*;

struct NoHash {
    map: bool,
    archive: bool,
}
impl NoHash {
    fn enter() -> Self {
        Self {
            map: crate::map_checksum::FORBID_HASH.with(|flag| flag.replace(true)),
            archive: crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|flag| flag.replace(true)),
        }
    }
}
impl Drop for NoHash {
    fn drop(&mut self) {
        crate::map_checksum::FORBID_HASH.with(|flag| flag.set(self.map));
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|flag| flag.set(self.archive));
    }
}

struct Fixture {
    request: &'static [u8],
    fine: &'static [u8],
    dense: Option<&'static [u8]>,
    backend: MappingBackendKind,
}
fn fixtures() -> [Fixture; 2] {
    [
        Fixture {
            request: include_bytes!("../tests/fixtures/legacy_fine_import/ar2_voxel.request.pb"),
            fine: include_bytes!("../tests/fixtures/legacy_fine_import/ar2_voxel.fine.archive"),
            dense: None,
            backend: MappingBackendKind::Voxel,
        },
        Fixture {
            request: include_bytes!(
                "../tests/fixtures/legacy_fine_import/ar3_raytracing.request.pb"
            ),
            fine: include_bytes!(
                "../tests/fixtures/legacy_fine_import/ar3_raytracing.fine.archive"
            ),
            dense: Some(include_bytes!(
                "../tests/fixtures/legacy_fine_import/ar3_raytracing.dense.archive"
            )),
            backend: MappingBackendKind::Raytracing,
        },
    ]
}
fn transfer(
    store: &mut crate::planar_artifacts::Store,
    reference: &proto::GraphArtifactRef,
    action: u32,
    data: Vec<u8>,
) -> Result<proto::GraphArtifactTransferResponse, String> {
    store.transfer(proto::GraphArtifactTransferRequest {
        artifact: Some(reference.clone()),
        action,
        data,
        ..Default::default()
    })
}
fn upload(
    store: &mut crate::planar_artifacts::Store,
    reference: &proto::GraphArtifactRef,
    bytes: &[u8],
) -> Result<(), String> {
    transfer(store, reference, 1, Vec::new())?;
    transfer(store, reference, 2, bytes.to_vec())?;
    let sealed = transfer(store, reference, 3, Vec::new())?;
    if !sealed.sealed {
        return Err("fixture upload did not seal".into());
    }
    Ok(())
}

#[test]
fn python_legacy_fine_import_admits_exact_bytes_and_replays_after_release_without_hashes() {
    // All work stays on this thread, including Store sealing and source admission.
    let _no_hash = NoHash::enter();
    for fixture in fixtures() {
        let request = proto::ActivateMapRequest::decode(fixture.request).unwrap();
        assert_eq!(request.activation_contract_version, 2);
        assert!(request.asset_manifest_sha256.is_empty());
        assert!(request.asset_bundle_sha256.is_empty());
        let checkpoint = request.mapping_backend_checkpoint.as_ref().unwrap();
        let fine_reference = checkpoint.fine_map_artifact.clone().unwrap();
        let dense_reference = checkpoint.dense_cloud_artifact.clone();
        let member = fine_reference.member_identity.as_ref().unwrap();
        assert_eq!(fine_reference.identity_schema_version, 2);
        assert!(fine_reference.sha256.is_empty());
        assert_eq!(member.checkpoint_id, checkpoint.checkpoint_id);
        assert_eq!(member.role, proto::CheckpointMemberRole::FineMap as i32);
        assert_eq!(
            (member.frame_generation, member.producer_generation),
            (4, 8)
        );
        assert_eq!(member.occupancy_receipt, checkpoint.occupancy_receipt);
        assert_eq!(member.cloud_receipt, checkpoint.cloud_receipt);
        assert_eq!(member.cloud_receipt.as_ref().unwrap().cloud_revision, 7);
        let mut store = crate::planar_artifacts::Store::default();
        upload(&mut store, &fine_reference, fixture.fine).unwrap();
        match (&dense_reference, fixture.dense) {
            (Some(reference), Some(bytes)) => upload(&mut store, reference, bytes).unwrap(),
            (None, None) => {}
            _ => panic!("fixture dense member and bytes differ"),
        }
        let interner = CheckpointOwnerInternerV2::default();
        let admitted = ValidatedActivationRequestV2::admit(
            request.clone(),
            17,
            fixture.backend,
            "map",
            |reference| store.open(reference),
            &interner,
        )
        .unwrap();
        let fine_owner = admitted.checkpoint().fine_map().unwrap();
        assert_eq!(fine_owner.reference(), &fine_reference);
        let mut retained = vec![0; fixture.fine.len()];
        fine_owner.file().read_exact_at(&mut retained, 0).unwrap();
        assert_eq!(retained, fixture.fine);
        assert!(fine_owner.file().write_at(b"x", 0).is_err());
        transfer(&mut store, &fine_reference, 5, Vec::new()).unwrap();
        if let Some(reference) = &dense_reference {
            transfer(&mut store, reference, 5, Vec::new()).unwrap();
        }
        assert!(store.open(&fine_reference).is_err());
        fine_owner.file().read_exact_at(&mut retained, 0).unwrap();
        assert_eq!(retained, fixture.fine);
        let replay = ValidatedActivationRequestV2::admit(
            request.clone(),
            17,
            fixture.backend,
            "map",
            |_| panic!("retained replay must not reopen released transfers"),
            &interner,
        )
        .unwrap();
        admitted.validate_replay(&replay).unwrap();
        assert!(admitted
            .checkpoint()
            .shares_source_with(replay.checkpoint()));
        let mut changed = request;
        changed
            .mapping_backend_checkpoint
            .as_mut()
            .unwrap()
            .canonical_cloud_checkpoint
            .as_mut()
            .unwrap()
            .xyz_f32_le[0] ^= 1;
        assert!(ValidatedActivationRequestV2::admit(
            changed,
            17,
            fixture.backend,
            "map",
            |_| panic!("conflicting replay must use its retained source"),
            &interner,
        )
        .is_err());
    }
}

#[test]
fn python_legacy_fine_import_rejects_wrong_source_and_truncated_archives_without_hashes() {
    let _no_hash = NoHash::enter();
    for fixture in fixtures() {
        let request = proto::ActivateMapRequest::decode(fixture.request).unwrap();
        let reference = request
            .mapping_backend_checkpoint
            .unwrap()
            .fine_map_artifact
            .unwrap();
        let mut wrong = reference.clone();
        wrong.member_identity.as_mut().unwrap().producer_generation += 1;
        let mut store = crate::planar_artifacts::Store::default();
        assert!(upload(&mut store, &wrong, fixture.fine).is_err());
        assert!(store.open(&wrong).is_err());
        let mut truncated = reference;
        truncated.size -= 1;
        let mut store = crate::planar_artifacts::Store::default();
        assert!(upload(
            &mut store,
            &truncated,
            &fixture.fine[..fixture.fine.len() - 1]
        )
        .is_err());
        assert!(store.open(&truncated).is_err());
    }
}

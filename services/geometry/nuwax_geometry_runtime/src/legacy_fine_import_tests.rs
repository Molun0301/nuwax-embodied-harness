//! Actual Python schema-1 import outputs, admitted through the existing v2 store.
//! Includes AR1 cloud-writer installation and next commit; no end-to-end RPC activation claim.
use super::*;

struct NoHash {
    map: bool,
}
impl NoHash {
    fn enter() -> Self {
        Self {
            map: crate::map_checksum::FORBID_HASH.with(|flag| flag.replace(true)),
        }
    }
}
impl Drop for NoHash {
    fn drop(&mut self) {
        crate::map_checksum::FORBID_HASH.with(|flag| flag.set(self.map));
    }
}

struct Fixture {
    request: &'static [u8],
    fine: &'static [u8],
    dense: Option<&'static [u8]>,
    backend: MappingBackendKind,
}
fn fixtures() -> [Fixture; 3] {
    [
        Fixture {
            request: include_bytes!("../tests/fixtures/legacy_fine_import/ar1_voxel.request.pb"),
            fine: include_bytes!("../tests/fixtures/legacy_fine_import/ar1_voxel.fine.archive"),
            dense: None,
            backend: MappingBackendKind::Voxel,
        },
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
        let checkpoint = request.mapping_backend_checkpoint.as_ref().unwrap();
        let fine_reference = checkpoint.fine_map_artifact.clone().unwrap();
        let dense_reference = checkpoint.dense_cloud_artifact.clone();
        let member = fine_reference.member_identity.as_ref().unwrap();
        assert_eq!(fine_reference.identity_schema_version, 2);
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
        if fixture.fine.starts_with(b"NUWAXAR1") {
            install_ar1_and_commit(&admitted);
        }
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
        assert_eq!(admitted.metadata(), replay.metadata());
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
        let replay=ValidatedActivationRequestV2::admit(changed,17,fixture.backend,"map",
            |_|panic!("存活编号不重新打开制品"),&interner).unwrap();
        assert!(admitted.checkpoint().shares_source_with(replay.checkpoint()));
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

// Follow the production preparation/install APIs with the actual admitted owner.
// The occupancy input is synthetic; MappingCommitClock and cloud publication are real.
fn install_ar1_and_commit(admitted: &ValidatedActivationRequestV2) {
    use std::os::fd::AsRawFd;
    let owner = admitted.checkpoint();
    let fine = owner.fine_map().unwrap();
    let file = std::fs::File::open(format!("/proc/self/fd/{}", fine.file().as_raw_fd())).unwrap();
    let source = owner.occupancy_receipt();
    let old_cloud = owner.cloud_receipt();
    let prepared = crate::pointcloud::prepare_fine_cloud_archive(
        file,
        &source.session_id,
        source.session_generation,
        old_cloud.cloud_revision,
        old_cloud.source_timestamp_ns,
    )
    .unwrap();
    let mut writer = crate::pointcloud::CloudMapShmWriter::new_private_for_test().unwrap();
    let member = fine.reference().member_identity.as_ref().unwrap();
    writer
        .bind_fine_map_owner(member.frame_generation, member.producer_generation)
        .unwrap();
    let mut clock = crate::mapping_commit::MappingCommitClock::new();
    let mut occupancy = source.clone();
    occupancy.commit_id = clock.reserve().unwrap();
    occupancy.occupancy_revision += 1;
    let cloud = owner.cloud();
    let installed = writer
        .install_checkpoint_with_fine(
            &occupancy,
            cloud.voxel_size(),
            cloud.point_count(),
            cloud.xyz_f32_le(),
            Some(prepared),
        )
        .unwrap();
    let baseline = writer.committed_fine_cloud().unwrap();
    let root = baseline.manifest();
    assert_eq!(installed.receipt.commit_id, occupancy.commit_id);
    assert_eq!(
        installed.receipt.cloud_revision,
        old_cloud.cloud_revision + 1
    );
    assert_eq!(root.revision, installed.receipt.cloud_revision);
    assert_eq!(root.geometry_revision, root.revision);
    assert_eq!(root.domain.session, source.session_id);
    assert_eq!(root.domain.generation, source.session_generation);
    assert_eq!(root.domain.frame_generation, member.frame_generation);
    assert_eq!(root.domain.producer_generation, member.producer_generation);
    assert!(matches!(
        root.blocks[0].object,
        nuwax_map_chunks::ObjectIdentity::Legacy(_)
    ));
    let mut reader =
        nuwax_map_chunks::RegionReader::new(baseline.repository.clone(), 4 * 1024 * 1024);
    let old_evidence = reader.region(root, [0; 3], [1; 3], 10).unwrap();
    assert_eq!(
        (old_evidence[0].1.observed_ns, old_evidence[0].1.hits),
        (11, 1)
    );
    assert!(writer.accumulate_mapping_scan(
        source.session_generation,
        &[nalgebra::Point3::new(0.0625, 0.0625, 0.0625)],
        None,
        12
    ));
    let mut next = occupancy.clone();
    next.commit_id = clock.reserve().unwrap();
    next.occupancy_revision += 1;
    next.source_timestamp_ns = 12;
    let published = writer.commit_accumulated_snapshot(&next).unwrap().unwrap();
    let current = writer.committed_fine_cloud().unwrap();
    assert_eq!(published.receipt.commit_id, next.commit_id);
    assert_eq!(
        published.receipt.cloud_revision,
        installed.receipt.cloud_revision + 1
    );
    assert_eq!(
        current.manifest().revision,
        published.receipt.cloud_revision
    );
    assert_eq!(current.manifest().geometry_revision, root.geometry_revision);
    assert_eq!(
        current.manifest().blocks[0].block_revision,
        published.receipt.cloud_revision
    );
    assert!(matches!(
        current.manifest().blocks[0].object,
        nuwax_map_chunks::ObjectIdentity::Assigned(_)
    ));
    assert!(writer.commit_accumulated_snapshot(&occupancy).is_err());
    // Installation/publication must not mutate the retained imported source receipt.
    assert_eq!(owner.cloud_receipt(), old_cloud);
    assert_eq!(owner.cloud_receipt().cloud_revision, 7);
}

#[test]
fn python_legacy_fine_import_ar1_rejects_conflicting_alias_bodies_without_hashes() {
    let _no_hash = NoHash::enter();
    let fixture = fixtures().into_iter().next().unwrap();
    let request = proto::ActivateMapRequest::decode(fixture.request).unwrap();
    let reader = nuwax_map_chunks::ArchiveReader::open(std::io::Cursor::new(fixture.fine)).unwrap();
    let nuwax_map_chunks::ArchivedRoot::Legacy(legacy) = reader.root().clone() else {
        panic!("AR1 fixture required");
    };
    let mut root = legacy.manifest;
    let mut duplicate = root.blocks[0].clone();
    duplicate.key = [1, 0, 0];
    root.blocks.push(duplicate);
    let json = serde_json::to_vec(&root).unwrap();
    let mut bytes = b"NUWAXAR1".to_vec();
    bytes.extend_from_slice(&(json.len() as u64).to_le_bytes());
    bytes.extend_from_slice(&[b'X'; 32]);
    bytes.extend_from_slice(&json);
    bytes.extend_from_slice(&2_u64.to_le_bytes());
    for observed_ns in [11, 10] {
        let block = nuwax_map_chunks::encode_block(&nuwax_map_chunks::Block::from([(
            0,
            nuwax_map_chunks::Evidence {
                observed_ns,
                hits: 1,
            },
        )]))
        .unwrap();
        bytes.extend_from_slice(&(block.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&block);
    }
    let mut reference = request
        .mapping_backend_checkpoint
        .unwrap()
        .fine_map_artifact
        .unwrap();
    reference.size = bytes.len() as u64;
    let mut store = crate::planar_artifacts::Store::default();
    assert!(upload(&mut store, &reference, &bytes).is_err());
    assert!(store.open(&reference).is_err());
}

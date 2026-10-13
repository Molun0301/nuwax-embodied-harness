//! Lower-level AR1 restore qualification, not assigned AR1 admission permission.
//! The production v2 AR1 rejection gates intentionally remain in force.
use super::*;
use std::io::{Seek, SeekFrom, Write};

struct NoHash(bool, bool);
impl NoHash {
    fn enter() -> Self {
        Self(
            crate::map_checksum::FORBID_HASH.with(|flag| flag.replace(true)),
            crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|flag| flag.replace(true)),
        )
    }
}
impl Drop for NoHash {
    fn drop(&mut self) {
        crate::map_checksum::FORBID_HASH.with(|flag| flag.set(self.0));
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|flag| flag.set(self.1));
    }
}
fn source(known: bool) -> PreparedFineCloud {
    let mut bytes =
        include_bytes!("../tests/fixtures/legacy_fine_import/ar1_source.fine.archive").to_vec();
    if !known {
        // Retain real unknown evidence; import time must never turn it into an observation.
        let end = bytes.len();
        bytes[end - 10..].fill(0);
    }
    let path = std::env::temp_dir().join(format!("ar1-baseline-{}", uuid::Uuid::new_v4()));
    let mut file = std::fs::OpenOptions::new()
        .create_new(true)
        .read(true)
        .write(true)
        .open(&path)
        .unwrap();
    std::fs::remove_file(path).unwrap();
    file.write_all(&bytes).unwrap();
    file.seek(SeekFrom::Start(0)).unwrap();
    prepare_fine_cloud_archive(file, "source", 3, 7, 11).unwrap()
}
fn receipt(
    clock: &mut crate::mapping_commit::MappingCommitClock,
    session: &str,
    generation: u64,
    occupancy_revision: u64,
    timestamp: u64,
) -> OccupancyCommitReceipt {
    OccupancyCommitReceipt {
        commit_id: clock.reserve().unwrap(),
        session_id: session.into(),
        session_generation: generation,
        occupancy_revision,
        source_timestamp_ns: timestamp,
        backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
        backend_generation: 1,
    }
}
fn transfer(sealed: &nuwax_map_chunks::SealedMap) -> nuwax_map_chunks::Transfer {
    let root = sealed.manifest();
    nuwax_map_chunks::Transfer {
        root: root.encode().unwrap(),
        included: root
            .blocks
            .iter()
            .map(|block| {
                (
                    block.object,
                    sealed
                        .repository
                        .read_object(block.object, nuwax_map_chunks::MAX_BLOCK_BYTES)
                        .unwrap(),
                )
            })
            .collect(),
    }
}

#[test]
fn ar1_real_install_establishes_baseline_then_real_commits_preserve_evidence_and_revision_fences() {
    let _no_hash = NoHash::enter();
    for same_domain in [true, false] {
        for known in [true, false] {
            let prepared = source(known);
            let (session, generation, frame, producer) = if same_domain {
                ("source", 3, 4, 8)
            } else {
                ("installed-owner", 9, 6, 10)
            };
            let mut clock = crate::mapping_commit::MappingCommitClock::new();
            let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
            writer.bind_fine_map_owner(frame, producer).unwrap();
            let occupancy = receipt(&mut clock, session, generation, 8, 11);
            let xyz = [0.0625_f32; 3]
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect::<Vec<_>>();
            let installed = writer
                .install_checkpoint_with_fine(&occupancy, 0.125, 1, &xyz, Some(prepared))
                .unwrap();
            let baseline = writer.committed_fine_cloud().unwrap();
            let root = baseline.manifest();
            let initial_revision = if same_domain { 8 } else { 1 };
            assert_eq!(installed.receipt.cloud_revision, initial_revision);
            assert_eq!(root.revision, installed.receipt.cloud_revision);
            assert_eq!(root.geometry_revision, initial_revision);
            assert_eq!(root.domain.session, session);
            assert_eq!(
                (
                    root.domain.generation,
                    root.domain.frame_generation,
                    root.domain.producer_generation
                ),
                (generation, frame, producer)
            );
            assert_eq!(installed.receipt.commit_id, occupancy.commit_id);
            assert_eq!(
                installed.receipt.occupancy_revision,
                occupancy.occupancy_revision
            );
            assert_eq!(
                root.source_timestamp_ns,
                installed.receipt.source_timestamp_ns
            );
            assert_eq!(root.blocks[0].block_revision, initial_revision);
            assert_eq!(root.blocks[0].geometry_revision, initial_revision);
            assert!(matches!(
                root.blocks[0].object,
                nuwax_map_chunks::ObjectIdentity::Legacy(_)
            ));
            let mut reader =
                nuwax_map_chunks::RegionReader::new(baseline.repository.clone(), 4 * 1024 * 1024);
            let evidence = reader.region(root, [0; 3], [1; 3], 10).unwrap();
            assert_eq!(evidence.len(), 1);
            assert_eq!(
                (evidence[0].1.observed_ns, evidence[0].1.hits),
                if known { (11, 1) } else { (0, 0) }
            );
            let mut receiver =
                nuwax_map_chunks::Receiver::new(nuwax_map_chunks::Repository::temporary().unwrap());
            receiver
                .receive(&transfer(&baseline), &root.domain, |_, _| {
                    panic!("fixture has all objects")
                })
                .unwrap();

            // Observation-only update gets a new object/evidence revision, not geometry history.
            assert!(writer.accumulate_mapping_scan(
                generation,
                &[Point3::new(0.0625, 0.0625, 0.0625)],
                None,
                12
            ));
            let updated_receipt = receipt(&mut clock, session, generation, 9, 12);
            let updated = writer
                .commit_accumulated_snapshot(&updated_receipt)
                .unwrap()
                .unwrap();
            let current = writer.committed_fine_cloud().unwrap();
            assert_eq!(updated.receipt.cloud_revision, initial_revision + 1);
            assert_eq!(current.manifest().revision, updated.receipt.cloud_revision);
            assert_eq!(current.manifest().geometry_revision, initial_revision);
            assert_eq!(
                current.manifest().blocks[0].geometry_revision,
                initial_revision
            );
            assert_eq!(
                current.manifest().blocks[0].block_revision,
                initial_revision + 1
            );
            assert!(matches!(
                current.manifest().blocks[0].object,
                nuwax_map_chunks::ObjectIdentity::Assigned(_)
            ));
            let mut rollback = current.manifest().as_ref().clone();
            rollback.blocks[0].block_revision = initial_revision;
            let mut invalid = transfer(&current);
            invalid.root = rollback.encode().unwrap();
            assert!(receiver
                .receive(&invalid, &root.domain, |_, _| panic!(
                    "fixture has all objects"
                ))
                .is_err());
            receiver
                .receive(&transfer(&current), &root.domain, |_, _| {
                    panic!("fixture has all objects")
                })
                .unwrap();
            assert!(writer.commit_accumulated_snapshot(&occupancy).is_err());
            assert!(Arc::ptr_eq(
                &current,
                &writer.committed_fine_cloud().unwrap()
            ));

            // Membership change must advance both block and root geometry revisions.
            assert!(writer.accumulate_mapping_scan(
                generation,
                &[Point3::new(0.1875, 0.0625, 0.0625)],
                None,
                13
            ));
            let membership_receipt = receipt(&mut clock, session, generation, 10, 13);
            let membership = writer
                .commit_accumulated_snapshot(&membership_receipt)
                .unwrap()
                .unwrap();
            let final_root = writer.committed_fine_cloud().unwrap();
            assert_eq!(
                final_root.manifest().revision,
                membership.receipt.cloud_revision
            );
            assert_eq!(
                final_root.manifest().geometry_revision,
                initial_revision + 2
            );
            assert_eq!(
                final_root.manifest().blocks[0].geometry_revision,
                initial_revision + 2
            );
            receiver
                .receive(&transfer(&final_root), &root.domain, |_, _| {
                    panic!("fixture has all objects")
                })
                .unwrap();
            let retained = reader.region(root, [0; 3], [1; 3], 10).unwrap();
            assert_eq!(
                (retained[0].1.observed_ns, retained[0].1.hits),
                if known { (11, 1) } else { (0, 0) }
            );
        }
    }
}

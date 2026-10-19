//! 原点云拥有方维护的固定精度源；完整块提交和展示容量互不影响。
use super::{dynamic_cloud::DynamicCloudFilter, voxel_index::VoxelIndex, voxel_key, VOXEL_SIZE};
use ahash::AHashSet;
use nalgebra::Point3;
use nuwax_map_chunks::{Repository, SealedMap, Writer};
use std::collections::VecDeque;
use std::sync::{Arc, Weak};
#[cfg(test)]
#[path = "ground_observations.rs"]
mod ground_observations;
#[path = "heightfield_capture.rs"]
mod heightfield_capture;

/// The existing source identity, with either canonical admitted coordinates or
/// the current rebuild's corrected coordinates. No second source ledger.
#[derive(Clone, Copy)]
pub(super) enum FrameSource<'a> {
    Canonical(&'a crate::admitted_cloud::AcceptedMappingCloud),
    Corrected(&'a crate::admitted_cloud::AdmittedCloudSource),
}

pub(super) struct FineCloud {
    writer: Writer,
    dynamic: DynamicCloudFilter,
    failure: Option<String>,
    readers: FineCloudReaders,
    commits: u64,
    baseline: Option<nuwax_map_chunks::RootIdentity>,
    // One shared admitted payload, not another scan history or voxel classifier.
    // Original body coordinates and frozen binding preserve unresolved modes.
    latest_unclassified_scan: Option<crate::lio_ingress::AdmittedRegisteredScan>,
    latest_ground_source: Option<crate::lio_ingress::WeakAdmittedRegisteredScan>,
    latest_ground_indices: Arc<[u32]>,
}
impl FineCloud {
    pub(super) fn from_archive(file: std::fs::File) -> Result<(Self, nuwax_map_chunks::ArchivedRoot), String> {
        let mut archive = nuwax_map_chunks::ArchiveReader::open(file)?;
        let mut cloud = Self::new(archive.root().voxel_size_m())?;
        let root = archive.import(cloud.writer.repository())?;
        cloud.writer.restore_archived(&root)?;
        cloud.baseline = Some(root.identity());
        Ok((cloud, root))
    }
    pub(super) fn new(size: f32) -> Result<Self, String> {
        Ok(Self { writer: Writer::new(Repository::temporary()?, size, 32 * 1024 * 1024)?,
            dynamic: DynamicCloudFilter::default(), failure: None, readers: FineCloudReaders::default(), commits: 0, baseline: None, latest_unclassified_scan: None, latest_ground_source: None, latest_ground_indices: Arc::from([]) })
    }
    pub(super) fn fresh() -> Result<Self, String> { Self::new(VOXEL_SIZE) }
    #[cfg(test)]
    pub(super) fn use_legacy_filter(&mut self) { self.dynamic.legacy_only = true; }
    pub(super) fn removed_keys(&self) -> &[(i32, i32, i32)] { self.dynamic.removed_keys() }
    pub(super) fn voxel_size(&self) -> f32 { self.writer.voxel_size_m() }
    pub(super) fn is_dynamic(&self, point: &Point3<f64>) -> bool {
        self.dynamic.is_dynamic([point.x, point.y, point.z])
    }
    pub(super) fn retire_unclassified_scan(&mut self) { self.latest_unclassified_scan = None; self.latest_ground_source = None; self.latest_ground_indices = Arc::from([]); }
    pub(super) fn latest_ground_indices(&self, source: &crate::lio_ingress::AdmittedRegisteredScan) -> Option<Arc<[u32]>> {
        self.latest_ground_source.as_ref().filter(|old| old.matches(source)).map(|_| self.latest_ground_indices.clone())
    }
    pub(super) fn latest_unclassified_scan(&self) -> Option<&crate::lio_ingress::AdmittedRegisteredScan> {
        self.latest_unclassified_scan.as_ref()
    }
    pub(super) fn accumulate(&mut self, points: &[Point3<f64>], origin: Option<[f64; 3]>, timestamp_ns: u64,
        source: Option<&crate::admitted_cloud::AcceptedMappingCloud>) -> Result<bool, String> {
        self.accumulate_frame(points, origin, timestamp_ns, source.map(FrameSource::Canonical))
    }
    /// Return false only when the original persisted source watermark made the
    /// complete frame a successful no-op, before occupancy or classification.
    pub(super) fn accumulate_frame(&mut self, points: &[Point3<f64>], origin: Option<[f64; 3]>, timestamp_ns: u64,
        input: Option<FrameSource<'_>>) -> Result<bool, String> {
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let result: Result<bool, String> = (|| {
            let mut capture = None;
            let actual = if let Some(input) = input {
                let source = match input {
                    FrameSource::Canonical(value) => value.source(),
                    FrameSource::Corrected(value) => value.clone(),
                };
                if matches!(input, FrameSource::Corrected(_))
                    && (timestamp_ns == 0 || points.len() != source.body_points().len()) {
                    return Err("invalid archived ground source".into());
                }
                if timestamp_ns > 0 && source.timestamp_ns() == timestamp_ns {
                    let sequence = source.sequence()?;
                    let namespace = source.layer_source();
                    if self.writer.source_watermark(&namespace).is_some_and(|old| (timestamp_ns, sequence) <= old) {
                        self.dynamic.clear_frame();
                        return Ok(false);
                    }
                    match input {
                        FrameSource::Canonical(value) => {
                            if let Some(first) = value.map_points().len().checked_sub(points.len()) {
                                capture = Some((source, namespace, sequence, first));
                                &value.map_points()[first..]
                            } else { points }
                        },
                        FrameSource::Corrected(_) => {
                            capture = Some((source, namespace, sequence, 0));
                            points
                        },
                    }
                } else { points }
            } else { points };
            let keys = self.accumulate_inner(actual, origin, timestamp_ns)?;
            if let Some((source, namespace, sequence, first)) = capture {
                let returns: Vec<_> = actual.iter().zip(keys).enumerate().filter_map(|(index, (point, key))| {
                    let original_index = source.original_index(first + index)?;
                    let point = [point.x, point.y, point.z];
                    Some(heightfield_capture::Return::from_key(point, key, original_index,
                        self.dynamic.is_dynamic(point), self.writer.voxel_size_m()))
                }).collect();
                let removed: Vec<_> = self.dynamic.removed_keys().iter().map(|k| [k.0,k.1,k.2]).collect();
                let indices = heightfield_capture::capture(&mut self.writer, &returns, &removed,
                    sequence, timestamp_ns, namespace.clone())?;
                if matches!(input, Some(FrameSource::Canonical(_))) {
                    self.latest_ground_indices = indices.into();
                    self.latest_ground_source = source.lidar().map(|scan| scan.downgrade());
                }
                self.writer.record_source_watermark(namespace, timestamp_ns, sequence)?;
            }
            Ok(true)
        })();
        // A failed private batch must never be sealed as a complete publication.
        if let Err(error) = &result { self.failure = Some(error.clone()); }
        else if result == Ok(true) {
            self.latest_unclassified_scan = input.and_then(|input| match input {
                FrameSource::Canonical(value) => value.lidar().map(|value| value.scan().clone()),
                FrameSource::Corrected(_) => None,
            });
        }
        result
    }
    fn accumulate_inner(&mut self, points: &[Point3<f64>], origin: Option<[f64; 3]>, timestamp_ns: u64) -> Result<Vec<nuwax_map_chunks::Key>, String> {
        self.dynamic.clear_frame();
        let size = self.writer.voxel_size_m();
        let keys = points.iter().map(|p| voxel_key([p.x, p.y, p.z], size)
            .map(|k| [k.0,k.1,k.2]).ok_or("精细输入坐标无效"))
            .collect::<Result<Vec<_>, _>>()?;
        let hits: AHashSet<_> = keys.iter().map(|k| (k[0],k[1],k[2])).collect();
        if let Some(origin) = origin {
            let low = voxel_key(origin.map(|v| v - 30.0 - f64::from(size)), size).ok_or("精细动态窗口越界")?;
            let high = voxel_key(origin.map(|v| v + 30.0 + f64::from(size)), size).ok_or("精细动态窗口越界")?;
            let local = self.writer.region([low.0, low.1, low.2], [high.0, high.1, high.2], 2_000_000)?;
            let mut voxels: VoxelIndex = local.into_iter().map(|(key, _)| (key[0], key[1], key[2])).collect();
            self.dynamic.clear_observed_free_space_with_hits(&mut voxels, size, origin, timestamp_ns, points, &hits);
            self.writer.remove(self.dynamic.removed_keys().iter().map(|key| [key.0, key.1, key.2]))?;
        }
        if timestamp_ns == 0 {
            self.writer.seed_unknown(hits.into_iter().map(|key| [key.0, key.1, key.2]))?;
        } else {
            self.writer.observe(hits.into_iter().map(|key| [key.0, key.1, key.2]), timestamp_ns)?;
        }
        Ok(keys)
    }
    pub(super) fn publication_floor(&self, domain: &nuwax_map_chunks::Domain) -> u64 {
        self.baseline.as_ref().filter(|root| &root.domain == domain).map_or(0, |root| root.revision)
    }
    pub(super) fn readers(&self) -> FineCloudReaders { self.readers.clone() }
    pub(super) fn adopt_readers(&mut self, readers: FineCloudReaders) { self.readers = readers; }
    pub(super) fn seal(&mut self, domain: nuwax_map_chunks::Domain, revision: u64, timestamp_ns: u64) -> Result<Arc<SealedMap>, String> {
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let sealed = Arc::new(self.writer.seal(domain, revision, timestamp_ns)?);
        self.baseline = Some(sealed.manifest().identity());
        self.commits = self.commits.wrapping_add(1);
        if self.commits % 16 == 0 {
            let live = {
                let mut readers = self.readers.0.lock();
                let live = readers.readers.iter().filter_map(Weak::upgrade).collect::<Vec<_>>();
                readers.readers.retain(|r| r.strong_count() > 0);
                live
            };
            // GC stays in private preparation, outside the short installation
            // lock. The writer also pins this not-yet-published baseline.
            if let Err(error) = self.writer.collect_sealed(&live) {
                tracing::warn!(%error, "fine-map object collection deferred");
            }
        }
        Ok(sealed)
    }
    pub(super) fn rehome_committed(&mut self, expected: &nuwax_map_chunks::RootIdentity,
        new_frame: u64) -> Result<Arc<SealedMap>, String> {
        if self.baseline.as_ref() != Some(expected) {
            return Err("fine frame rehome does not name the committed baseline".into());
        }
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let root = Arc::new(self.writer.rehome_published_frame(expected, new_frame)?);
        self.baseline = Some(root.manifest().identity());
        Ok(root)
    }
    pub(super) fn retain_committed(&mut self, sealed: &Arc<SealedMap>) { self.readers.retain_committed(sealed); }

}

#[derive(Default)]
struct RetainedRoots {
    recent: VecDeque<Arc<SealedMap>>,
    readers: Vec<Weak<SealedMap>>,
}
/// Shared retention survives transfer of the mutable accumulator to a worker.
#[derive(Clone, Default)]
pub(super) struct FineCloudReaders(Arc<parking_lot::Mutex<RetainedRoots>>);
impl FineCloudReaders {
    pub(super) fn retain_committed(&self, sealed: &Arc<SealedMap>) {
        let mut readers = self.0.lock();
        readers.recent.push_back(Arc::clone(sealed));
        readers.readers.push(Arc::downgrade(sealed));
        while readers.recent.len() > 16 || (readers.recent.len() > 1 && readers.recent.iter().map(|r| r.manifest().blocks.len()).sum::<usize>() > 262144) {
            readers.recent.pop_front();
        }
    }
    /// Resolve the exact retained publication before starting asynchronous IO.
    pub(super) fn pin_root(&self, identity: &nuwax_map_chunks::RootIdentity,
        reference: &nuwax_map_chunks::BlockRef) -> Result<Arc<SealedMap>, String> {
        let retained = self.0.lock();
        let root = retained.recent.iter().find(|root| root.manifest().identity() == *identity)
            .cloned().or_else(|| retained.readers.iter().filter_map(Weak::upgrade)
                .find(|root| root.manifest().identity() == *identity))
            .ok_or("requested fine-map root expired; request the latest complete root")?;
        let manifest = root.manifest();
        let index = manifest.blocks.binary_search_by_key(&reference.key, |block| block.key)
            .map_err(|_| "requested block is not a member of the retained root")?;
        if &manifest.blocks[index] != reference {
            return Err("requested block identity/revisions differ from the retained root".into());
        }
        Ok(root)
    }
}

#[cfg(test)]
mod revision_tests {
    use super::*;
    fn domain() -> nuwax_map_chunks::Domain {
        nuwax_map_chunks::Domain { session: "owner".into(), generation: 3, frame_generation: 4, producer_generation: 5 }
    }
    #[test]
    fn retained_lookup_requires_exact_root_and_block_and_pins_old_reader() {
        let mut source = FineCloud::fresh().unwrap();
        source.accumulate(&[Point3::new(-0.025, 0.025, 0.175)], None, 1, None).unwrap();
        let first = source.seal(domain(), 1, 1).unwrap();
        let readers = source.readers();
        let identity = first.manifest().identity();
        let reference = first.manifest().blocks[0].clone();
        assert!(readers.pin_root(&identity, &reference).is_err());
        source.retain_committed(&first);
        let pinned = readers.pin_root(&identity, &reference).unwrap();
        let mut forged = reference.clone();
        forged.block_revision += 1;
        assert!(readers.pin_root(&identity, &forged).is_err());
        let mut wrong_domain = identity.clone();
        wrong_domain.domain.frame_generation += 1;
        assert!(readers.pin_root(&wrong_domain, &reference).is_err());
        for revision in 2..=20 {
            source.accumulate(&[Point3::new(-0.025, 0.025, 0.175)], None, revision, None).unwrap();
            let root = source.seal(domain(), revision, revision).unwrap();
            assert_eq!(root.manifest().geometry_revision, 1);
            source.retain_committed(&root);
        }
        assert!(pinned.repository.read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).is_ok());
        assert!(readers.pin_root(&identity, &reference).is_ok());
        drop(first);
        drop(pinned);
        assert!(readers.pin_root(&identity, &reference).is_err());
    }
}

#[cfg(test)]
mod owner_test_observations {
    use super::*;
    use nuwax_map_chunks::{ColumnGroundRecords, Evidence, Key, LayerSample};
    use std::collections::BTreeMap;

    pub(super) type Records = BTreeMap<[i32; 2], ColumnGroundRecords>;
    pub(super) fn snapshot(fine: &mut FineCloud) -> (Vec<(Key, Evidence)>, Records) {
        // These fixtures have at most a few hundred occupied voxels. This reads
        // every actual owner voxel, including any unintended extra occupancy.
        let occupancy = fine.writer.region([i32::MIN; 3], [i32::MAX; 3], 16_384).unwrap();
        let records = fine.writer.ground_columns(occupancy.iter().map(|(key, _)| [key[0], key[1]])).unwrap();
        (occupancy, records)
    }
    pub(super) fn records(fine: &mut FineCloud) -> Records { snapshot(fine).1 }
    pub(super) fn samples(records: &Records) -> impl Iterator<Item = &LayerSample> {
        records.values().flat_map(|column| &column.heightfields).flat_map(|field| &field.samples)
    }
}

#[cfg(test)]
mod unclassified_scan_tests {
    use super::*;
    use super::owner_test_observations::{records, samples, snapshot};
    use crate::admitted_cloud::{AcceptedMappingCloud, AdmittedCloudSource};
    use crate::lio_ingress::{AcceptedRegisteredCloud, CorrelatedOdometry, RegistrationBinding};
    use std::collections::BTreeSet;

    fn source(stamp: u64, points: &[Point3<f64>]) -> AcceptedRegisteredCloud {
        AcceptedRegisteredCloud::from_map_points_for_test(points.to_vec(), stamp, stamp, "map", "base_link", RegistrationBinding {
            runtime_generation: 5, cloud_source_sequence: stamp as i64,
            correlated_odometry: CorrelatedOdometry { source_sequence: stamp as i64, source_timestamp_ns: stamp as i64 },
            mapping_session_id: "owner".into(), mapping_session_generation: 3, map_frame_generation: 4,
            frozen_odom_from_body: nalgebra::Isometry3::identity(),
            base_anchor_map_from_odom: nalgebra::Isometry3::identity(),
            registration_map_from_odom: nalgebra::Isometry3::identity(), registration_correction_revision: 0,
            sensor_origin_map: None, frame_capture_relocalization_epoch: Some(0),
        })
    }
    fn floor() -> Vec<Point3<f64>> {
        (0..8).flat_map(|x| (0..8).map(move |y|
            Point3::new(0.008 + x as f64 * 0.022, 0.008 + y as f64 * 0.022, 0.01))).collect()
    }
    fn domain() -> nuwax_map_chunks::Domain {
        nuwax_map_chunks::Domain { session: "owner".into(), generation: 3,
            frame_generation: 4, producer_generation: 5 }
    }
    fn point_keys(points: &[Point3<f64>], q: f32) -> BTreeSet<nuwax_map_chunks::Key> {
        points.iter().map(|point| {
            let key = voxel_key([point.x, point.y, point.z], q).unwrap();
            [key.0, key.1, key.2]
        }).collect()
    }

    #[test]
    fn unclassified_stage_shares_modes_and_retires_with_owner_boundaries() {
        let points = [Point3::new(0.011, 0.011, 0.011), Point3::new(0.011, 0.011, 0.041)];
        let first = source(1, &points);
        assert!(!first.body_points_captured_for_test());
        let mut fine = FineCloud::fresh().unwrap();
        fine.accumulate(&points, None, 1, Some(&first.clone().into())).unwrap();
        let raw = fine.latest_unclassified_scan().unwrap();
        let retained = raw.retention_probe_for_test();
        assert!(first.body_points_captured_for_test());
        assert_eq!(raw.body_points().as_ptr(), first.scan().body_points().as_ptr());
        assert_eq!(&**raw.body_points(), &points);
        let voxels = fine.writer.region([0,0,0], [0,0,0], 1).unwrap();
        assert_eq!(voxels.len(), 1);
        assert_eq!(voxels[0].1.hits, 1, "two unresolved modes still produce one occupied voxel hit");
        drop(first);
        assert!(retained());
        let second = source(2, &points);
        fine.accumulate(&points, None, 2, Some(&second.clone().into())).unwrap();
        assert!(!retained(), "fine owner retains only the latest scan, not a second history");
        let sealed = fine.seal(domain(), 1, 2).unwrap();
        let (file, _) = nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
        let (mut restored, _) = FineCloud::from_archive(file).unwrap();
        assert!(restored.latest_unclassified_scan().is_none(), "archive retains occupancy, not fabricated raw scan provenance");
        let restored_voxels = restored.writer.region([0,0,0], [0,0,0], 1).unwrap();
        assert_eq!(restored_voxels[0].1.hits, 2);
        fine.accumulate(&points, None, 0, None).unwrap();
        assert!(fine.latest_unclassified_scan().is_none(), "legacy/replay without source does not inherit the prior stage");
    }

    #[test]
    fn original_indices_are_counted_once_and_resampling_is_not_new_evidence() {
        let points = floor();
        let admitted = source(10, &points);
        let mut fine = FineCloud::fresh().unwrap();
        assert!(fine.accumulate(&points, None, 10, Some(&admitted.clone().into())).unwrap());
        let before = records(&mut fine);
        assert!(samples(&before).next().is_some());
        assert!(!fine.accumulate(&points, None, 10, Some(&admitted.clone().into())).unwrap());
        assert_eq!(records(&mut fine), before);
        let next = source(11, &points);
        // Preserve the original resampling negative control byte-for-byte.
        let mut resampled = points.clone();
        resampled[0].x += 1e-9;
        assert!(fine.accumulate(&resampled, None, 11, Some(&next.clone().into())).unwrap());
        let observed = records(&mut fine);
        assert!(observed.values().all(|column| column.legacy.is_empty()));
        for sample in samples(&observed) {
            let expected: [f64; 3] = points[sample.point_index as usize].coords.into();
            assert_eq!(sample.point, expected, "statistics use the admitted allocation, never supplied reconstructed floats");
        }
        let sealed = fine.seal(domain(), 1, 11).unwrap();
        let (file, _) = nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
        let (mut restored, _) = FineCloud::from_archive(file).unwrap();
        assert!(!restored.accumulate(&points, None, 10, Some(&admitted.clone().into())).unwrap());
        assert_eq!(records(&mut restored), observed, "restored source watermark rejects old scan recounting");
    }

    #[test]
    fn corrected_replay_preserves_admitted_sample_identity_without_recounting() {
        let points = floor();
        let admitted = source(10, &points);
        let scan = admitted.scan();
        let transform = nalgebra::Isometry3::translation(0.2, 0.0, 0.1);
        let corrected = scan.body_points().iter().map(|point| transform * point).collect::<Vec<_>>();
        let original: AdmittedCloudSource = scan.clone().into();
        let mut fine = FineCloud::fresh().unwrap();
        assert!(fine.accumulate_frame(&corrected, None, 10, Some(FrameSource::Corrected(&original))).unwrap());
        let observed = records(&mut fine);
        assert!(samples(&observed).next().is_some());
        assert!(observed.values().all(|column| column.legacy.is_empty()));
        // Same original input at z=0.11, now H1's fixed q/8 representative.
        let expected_height = 17.5 * f64::from(fine.voxel_size()) / 8.0;
        for field in observed.values().flat_map(|column| &column.heightfields) {
            assert!((field.column.height() - expected_height).abs() < 1e-12);
            for sample in &field.samples {
                assert_eq!(sample.source, original.layer_source());
                assert_eq!(sample.source_sequence, 10);
                assert_eq!(sample.source_timestamp_ns, 10);
                let expected: [f64; 3] = corrected[sample.point_index as usize].coords.into();
                assert_eq!(sample.point, expected);
            }
        }
        assert!(!fine.accumulate_frame(&corrected, None, 10, Some(FrameSource::Corrected(&original))).unwrap());
        assert_eq!(records(&mut fine), observed);
    }

    #[test]
    fn canonical_duplicate_and_stale_frames_are_complete_owner_noops() {
        let points = floor();
        let first: AcceptedMappingCloud = source(10, &points).into();
        let latest: AcceptedMappingCloud = source(11, &points).into();
        let mut fine = FineCloud::fresh().unwrap();
        for admitted in [&first, &latest] {
            assert!(fine.accumulate(admitted.map_points(), Some([0.0; 3]),
                admitted.source().timestamp_ns(), Some(admitted)).unwrap());
        }
        let latest_scan = latest.lidar().unwrap().scan().clone();
        let indices = fine.latest_ground_indices(&latest_scan).unwrap();
        assert_eq!(indices.len(), points.len(), "a positive classification must precede the no-op controls");
        let before = snapshot(&mut fine);
        let sealed = fine.seal(domain(), 1, 11).unwrap();
        assert!(!fine.writer.has_unpublished_changes());
        for admitted in [&latest, &first] {
            assert!(!fine.accumulate(admitted.map_points(), Some([0.0; 3]),
                admitted.source().timestamp_ns(), Some(admitted)).unwrap());
            assert_eq!(snapshot(&mut fine), before, "no occupancy hit, observed timestamp, member, mass or diagnostic may change");
            assert_eq!(fine.writer.source_watermark(&admitted.source().layer_source()), Some((11, 11)));
            assert!(!fine.writer.has_unpublished_changes(), "a successful duplicate is not unpublished work");
            assert!(fine.removed_keys().is_empty(), "a duplicate cannot emit a prior frame's removal delta");
            assert_eq!(fine.latest_unclassified_scan().unwrap().body_points().as_ptr(), latest_scan.body_points().as_ptr());
            assert_eq!(fine.latest_ground_indices(&latest_scan).as_deref(), Some(indices.as_ref()));
        }
        let after = fine.seal(domain(), 2, 11).unwrap();
        assert_eq!(after.manifest().blocks, sealed.manifest().blocks, "duplicate cannot rewrite objects or block revisions");
        assert_eq!(after.manifest().geometry_revision, sealed.manifest().geometry_revision);
        assert_eq!(after.manifest().source_watermarks, sealed.manifest().source_watermarks);
    }

    #[test]
    fn restored_duplicate_frame_is_a_complete_owner_noop() {
        let points = floor();
        let admitted: AcceptedMappingCloud = source(10, &points).into();
        let mut fine = FineCloud::fresh().unwrap();
        assert!(fine.accumulate(admitted.map_points(), None, 10, Some(&admitted)).unwrap());
        let sealed = fine.seal(domain(), 1, 10).unwrap();
        let (file, _) = nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
        let (mut restored, _) = FineCloud::from_archive(file).unwrap();
        let before = snapshot(&mut restored);
        assert!(samples(&before.1).next().is_some());
        assert!(!restored.accumulate(admitted.map_points(), Some([0.0; 3]), 10, Some(&admitted)).unwrap());
        assert_eq!(snapshot(&mut restored), before);
        assert!(!restored.writer.has_unpublished_changes());
        assert!(restored.latest_unclassified_scan().is_none(), "rejected replay must not fabricate a live scan owner");
        assert!(restored.latest_ground_indices(admitted.lidar().unwrap().scan()).is_none());
        let after = restored.seal(domain(), 2, 10).unwrap();
        assert_eq!(after.manifest().blocks, sealed.manifest().blocks);
        assert_eq!(after.manifest().geometry_revision, sealed.manifest().geometry_revision);
        assert_eq!(after.manifest().source_watermarks, sealed.manifest().source_watermarks);
    }

    #[test]
    fn canonical_retained_suffix_uses_original_points_and_indices_for_both_outputs() {
        let points = floor();
        let admitted: AcceptedMappingCloud = source(10, &points).into();
        let first = 8;
        let retained_suffix = &admitted.map_points()[first..];
        let mut fine = FineCloud::fresh().unwrap();
        assert!(fine.accumulate(retained_suffix, None, 10, Some(&admitted)).unwrap());
        let (occupancy, observed) = snapshot(&mut fine);
        assert_eq!(occupancy.iter().map(|(key, _)| *key).collect::<BTreeSet<_>>(), point_keys(retained_suffix, fine.voxel_size()));
        assert!(occupancy.iter().all(|(_, evidence)| evidence.hits == 1 && evidence.observed_ns == 10));
        assert!(samples(&observed).next().is_some());
        for sample in samples(&observed) {
            assert!(sample.point_index >= first as u32);
            assert_eq!(sample.point, <[f64; 3]>::from(admitted.map_points()[sample.point_index as usize].coords));
            assert_eq!(sample.source, admitted.source().layer_source());
            assert_eq!((sample.source_timestamp_ns, sample.source_sequence), (10, 10));
        }
        let expected_indices: Vec<_> = (first as u32..points.len() as u32).collect();
        assert_eq!(fine.latest_ground_indices(admitted.lidar().unwrap().scan()).unwrap().as_ref(), expected_indices.as_slice());
    }

    #[test]
    fn corrected_fresh_writer_rebuilds_actual_points_and_duplicate_is_a_noop() {
        let points = floor();
        let admitted: AcceptedMappingCloud = source(10, &points).into();
        let original = admitted.source();
        let body_allocation = original.body_points().as_ptr();
        let original_body = original.body_points().to_vec();
        let mut old_owner = FineCloud::fresh().unwrap();
        assert!(old_owner.accumulate(admitted.map_points(), None, 10, Some(&admitted)).unwrap());
        let old = old_owner.seal(domain(), 1, 10).unwrap();
        assert_eq!(old.manifest().source_watermarks.len(), 1);

        let pose = nalgebra::Isometry3::translation(0.2, 0.0, 0.1);
        let corrected: Vec<_> = original.body_points().iter().map(|point| pose * point).collect();
        let mut rebuilt = FineCloud::fresh().unwrap();
        assert_eq!(rebuilt.writer.source_watermark(&original.layer_source()), None,
            "a fresh PGO Writer must not inherit the previous accumulator's watermark");
        assert!(rebuilt.accumulate_frame(&corrected, None, 10, Some(FrameSource::Corrected(&original))).unwrap());
        let before = snapshot(&mut rebuilt);
        let expected = point_keys(&corrected, rebuilt.voxel_size());
        assert!(expected.is_disjoint(&point_keys(admitted.map_points(), rebuilt.voxel_size())));
        assert_eq!(before.0.iter().map(|(key, _)| *key).collect::<BTreeSet<_>>(), expected,
            "corrected occupancy must use supplied corrected points, not canonical or body coordinates");
        assert!(before.0.iter().all(|(_, evidence)| evidence.hits == 1 && evidence.observed_ns == 10));
        assert!(samples(&before.1).next().is_some());
        for sample in samples(&before.1) {
            assert_eq!(sample.point, <[f64; 3]>::from(corrected[sample.point_index as usize].coords));
            assert_eq!(sample.source, original.layer_source());
            assert_eq!((sample.source_timestamp_ns, sample.source_sequence), (10, 10));
        }
        assert_eq!(original.body_points().as_ptr(), body_allocation);
        assert_eq!(&**original.body_points(), original_body.as_slice(), "a rebuild cannot rewrite admitted body points");
        assert!(rebuilt.latest_unclassified_scan().is_none(), "corrected rebuild must not publish a canonical recent raw scan");
        assert!(rebuilt.latest_ground_indices(original.lidar().unwrap()).is_none(), "corrected rebuild must not publish live recent indices");
        let sealed = rebuilt.seal(domain(), 1, 10).unwrap();
        assert!(!rebuilt.accumulate_frame(&corrected, None, 10, Some(FrameSource::Corrected(&original))).unwrap());
        assert_eq!(snapshot(&mut rebuilt), before);
        assert!(!rebuilt.writer.has_unpublished_changes());
        let after = rebuilt.seal(domain(), 2, 10).unwrap();
        assert_eq!(after.manifest().blocks, sealed.manifest().blocks);
        assert_eq!(after.manifest().geometry_revision, sealed.manifest().geometry_revision);
        assert_eq!(after.manifest().source_watermarks, sealed.manifest().source_watermarks);
    }

    #[test]
    fn source_none_uses_actual_points_without_inventing_ground_or_watermarks() {
        let points = floor();
        let pose = nalgebra::Isometry3::translation(0.2, 0.0, 0.1);
        let actual: Vec<_> = points.iter().map(|point| pose * point).collect();
        for stamp in [0, 10] {
            let mut fine = FineCloud::fresh().unwrap();
            assert!(fine.accumulate(&actual, None, stamp, None).unwrap());
            let (occupancy, observed) = snapshot(&mut fine);
            assert_eq!(occupancy.iter().map(|(key, _)| *key).collect::<BTreeSet<_>>(), point_keys(&actual, fine.voxel_size()));
            assert!(occupancy.iter().all(|(_, evidence)| evidence.observed_ns == stamp && evidence.hits == if stamp == 0 { 0 } else { 1 }));
            assert!(observed.is_empty(), "coordinates without admitted identity cannot manufacture measured ground");
            assert!(fine.latest_unclassified_scan().is_none());
            let sealed = fine.seal(domain(), 1, stamp.max(1)).unwrap();
            assert!(sealed.manifest().source_watermarks.is_empty());
        }
    }
}

#[cfg(test)]
mod auxiliary_source_tests {
    use super::*;
    use super::owner_test_observations::{records, samples, snapshot};
    use crate::admitted_cloud::{AcceptedMappingCloud, AdmittedCloudSource};
    use crate::lcm_runtime::mapping_observation::RegisteredObservation;
    use nalgebra::Isometry3;
    use nuwax_map_chunks::LayerSource;

    fn namespace(sensor: &str, clock: &str, runtime: u64, session: u64) -> LayerSource {
        LayerSource::Auxiliary { sensor_id: sensor.into(), clock_id: clock.into(),
            runtime_generation: runtime, session_generation: session }
    }
    fn floor() -> Vec<Point3<f64>> {
        (0..8).flat_map(|x| (0..8).map(move |y|
            Point3::new(0.008 + x as f64 * 0.004, 0.008 + y as f64 * 0.004, 0.01))).collect()
    }
    fn cloud(source: LayerSource, stamp: u64, sequence: u64, points: Vec<Point3<f64>>) -> AcceptedMappingCloud {
        let indices = (0..points.len()).map(|i| i as u32 * 3 + 4).collect();
        AcceptedMappingCloud::auxiliary(RegisteredObservation::for_test_source(stamp, sequence,
            source, points, indices, Isometry3::identity()))
    }
    fn ingest(fine: &mut FineCloud, source: &AcceptedMappingCloud) -> bool {
        fine.accumulate(source.map_points(), None, source.source().timestamp_ns(), Some(source)).unwrap()
    }
    fn domain() -> nuwax_map_chunks::Domain {
        nuwax_map_chunks::Domain { session: "aux-ground".into(), generation: 1,
            frame_generation: 1, producer_generation: 1 }
    }

    #[test]
    fn auxiliary_original_suffix_preserves_sources_and_short_vertical_returns() {
        let mut points = floor();
        // Historical 14 cm input is unchanged. H1 association of these short
        // vertical returns is not a traversability or body-clearance decision.
        points.extend((0..8).flat_map(|y| (0..8).map(move |z|
            Point3::new(0.28, 0.008 + y as f64 * 0.02, 0.21 + z as f64 * 0.02))));
        let source = cloud(namespace("depth-front", "device-clock", 1, 1), 10, 0, points);
        let mut fine = FineCloud::fresh().unwrap();
        let suffix = &source.map_points()[8..];
        assert!(fine.accumulate(suffix, None, 10, Some(&source)).unwrap());
        let measured = records(&mut fine);
        assert!(samples(&measured).next().is_some());
        for sample in samples(&measured) {
            assert_eq!(sample.source.as_ref(), source.source().layer_source().as_ref());
            assert_eq!(sample.source_sequence, 0, "actual zero-based auxiliary sequence is not synthesized");
            assert!(sample.point_index >= 8 * 3 + 4);
            let index = ((sample.point_index - 4) / 3) as usize;
            assert_eq!(sample.point, <[f64; 3]>::from(source.map_points()[index].coords));
            assert_eq!(sample.source_timestamp_ns, 10);
        }
        let admitted = source.source();
        let expected_indices: Vec<_> = (8..source.map_points().len())
            .map(|index| admitted.original_index(index).unwrap()).collect();
        assert_eq!(fine.latest_ground_indices.as_ref(), expected_indices.as_slice(),
            "the complete original admitted suffix is associated, independent of bounded diagnostic samples");
        let expected_occupancy = suffix.iter().map(|point| {
            let key = voxel_key([point.x, point.y, point.z], fine.voxel_size()).unwrap();
            [key.0, key.1, key.2]
        }).collect::<std::collections::BTreeSet<_>>();
        assert_eq!(snapshot(&mut fine).0.iter().map(|(key, _)| *key)
            .collect::<std::collections::BTreeSet<_>>(), expected_occupancy);
        // Execute the retired assertion against this same actual result and
        // retain its failure as evidence. This is NOT the old selector passing.
        let legacy_assertion = std::panic::catch_unwind(|| {
            for sample in samples(&measured) {
                assert_eq!(sample.point[2], 0.01, "vertical wall must remain occupied, not measured ground");
            }
        });
        assert!(legacy_assertion.is_err(), "the old plane-only assertion must demonstrably fail for the unchanged input");
        eprintln!("legacy assertion failed: unchanged 14 cm vertical returns are H1-associated; body/collision qualification remains required");
        assert!(fine.latest_unclassified_scan().is_none(), "auxiliary is not a LiDAR scan");
        let occupancy = fine.writer.region([5, 0, 4], [5, 4, 9], 128).unwrap();
        assert!(!occupancy.is_empty(), "wall occupancy is preserved");
    }

    #[test]
    fn same_time_sources_and_lower_clock_keep_bounded_distinct_evidence() {
        let front = cloud(namespace("depth-front", "clock-a", 1, 1), 1_000_000, 7, floor());
        let left = cloud(namespace("depth-left", "clock-a", 1, 1), 1_000_000, 7, floor());
        let reset = cloud(namespace("depth-front", "clock-b", 1, 1), 5, 7, floor());
        let mut fine = FineCloud::fresh().unwrap();
        assert!(ingest(&mut fine, &front));
        assert!(ingest(&mut fine, &left));
        let both = records(&mut fine);
        let retained = &both[&[0, 0]].heightfields[0].samples;
        assert_eq!(retained.len(), nuwax_map_chunks::MAX_LAYER_SAMPLES);
        assert!(retained.iter().any(|sample| sample.source == front.source().layer_source()));
        assert!(retained.iter().any(|sample| sample.source == left.source().layer_source()));
        let before_duplicate = snapshot(&mut fine);
        assert!(!ingest(&mut fine, &left));
        assert_eq!(snapshot(&mut fine), before_duplicate, "same source frame cannot vote or occupy twice");
        assert!(ingest(&mut fine, &reset));
        let observed = records(&mut fine);
        let retained = &observed[&[0, 0]].heightfields[0].samples;
        for source in [&front, &left, &reset] {
            assert!(retained.iter().any(|sample| sample.source == source.source().layer_source()),
                "a real producer must not be starved by another clock's numeric offset");
        }
        assert_eq!(retained.len(), nuwax_map_chunks::MAX_LAYER_SAMPLES);
    }

    #[test]
    fn persisted_source_watermarks_are_scoped_to_clock_and_admitted_owner_epoch() {
        let original = cloud(namespace("depth-front", "stable-label", 1, 1), 100, 10, floor());
        let mut fine = FineCloud::fresh().unwrap();
        assert!(ingest(&mut fine, &original));
        let before = snapshot(&mut fine);
        let sealed = fine.seal(domain(), 1, 100).unwrap();
        assert_eq!(sealed.manifest().version, 5, "heightfields require the native versioned root");
        let (file, _) = nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
        let (mut restored, _) = FineCloud::from_archive(file).unwrap();
        assert!(!ingest(&mut restored, &original));
        assert_eq!(snapshot(&mut restored), before);
        for (clock, runtime, session) in [("new-clock", 1, 1), ("stable-label", 2, 1), ("stable-label", 1, 2)] {
            let source = cloud(namespace("depth-front", clock, runtime, session), 5, 0, floor());
            assert!(ingest(&mut restored, &source));
            assert!(samples(&records(&mut restored)).any(|sample| sample.source == source.source().layer_source()));
        }
        let sealed = restored.seal(domain(), 2, 100).unwrap();
        assert_eq!(sealed.manifest().source_watermarks.len(), 4);
    }

    #[test]
    fn corrected_auxiliary_replay_retains_original_indices_and_is_idempotent() {
        let source = cloud(namespace("depth-front", "clock", 1, 1), 10, 4, floor());
        let admitted = source.source();
        assert!(matches!(admitted, AdmittedCloudSource::Auxiliary(_)));
        let body_allocation = admitted.body_points().as_ptr();
        let original_body = admitted.body_points().to_vec();
        let pose = Isometry3::translation(0.2, 0.0, 0.1);
        let corrected: Vec<_> = admitted.body_points().iter().map(|point| pose * point).collect();
        let mut fine = FineCloud::fresh().unwrap();
        assert!(fine.accumulate_frame(&corrected, None, 10, Some(FrameSource::Corrected(&admitted))).unwrap());
        let before = snapshot(&mut fine);
        assert!(samples(&before.1).next().is_some());
        for sample in samples(&before.1) {
            let index = ((sample.point_index - 4) / 3) as usize;
            assert_eq!(sample.point, <[f64; 3]>::from(corrected[index].coords));
            assert!((sample.point[2] - 0.11).abs() < 1e-12);
            assert_eq!(sample.source, admitted.layer_source());
            assert_eq!((sample.source_timestamp_ns, sample.source_sequence), (10, 4));
        }
        assert_eq!(before.0.len(), 1);
        assert_eq!(before.0[0].0, [4, 0, 2], "auxiliary corrected points, not the uncorrected body, own occupancy");
        assert_eq!((before.0[0].1.observed_ns, before.0[0].1.hits), (10, 1));
        assert_eq!(admitted.body_points().as_ptr(), body_allocation);
        assert_eq!(&**admitted.body_points(), original_body.as_slice());
        assert!(fine.latest_unclassified_scan().is_none());
        assert!(!fine.accumulate_frame(&corrected, None, 10, Some(FrameSource::Corrected(&admitted))).unwrap());
        assert_eq!(snapshot(&mut fine), before);
    }

    #[test]
    fn legacy_layer_archive_does_not_invent_a_typed_source_cutoff() {
        let points = floor();
        let mut fine = FineCloud::fresh().unwrap();
        // Actual Legacy plane construction. Do not run heightfield_capture and
        // relabel its root as AR3: the archived block must really be BL2.
        fine.accumulate(&points, None, 100, None).unwrap();
        ground_observations::capture(&mut fine.writer, &points, 100, 100, Arc::new(LayerSource::Legacy),
            |index| u32::try_from(index).ok(), |_| false).unwrap();
        let legacy_column = fine.writer.ground_column([0, 0]).unwrap();
        assert!(!legacy_column.legacy.is_empty());
        assert!(legacy_column.heightfields.is_empty());
        assert!(legacy_column.legacy.iter().flat_map(|(_, layer)| &layer.samples)
            .all(|sample| matches!(sample.source.as_ref(), LayerSource::Legacy)));
        let sealed = fine.seal(domain(), 1, 100).unwrap();
        for reference in &sealed.manifest().blocks {
            let bytes = sealed.repository.read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
            assert!(bytes.starts_with(b"NUWAXBL2"), "AR3 fixture must contain real Legacy-only plane bytes");
        }
        let mut legacy = (**sealed.manifest()).clone();
        legacy.version = 3;
        assert!(legacy.source_watermarks.is_empty());
        let (file, _) = nuwax_map_chunks::archive_file(&legacy, &sealed.repository).unwrap();
        let (mut restored, _) = FineCloud::from_archive(file).unwrap();
        let source = cloud(namespace("depth-front", "clock", 1, 1), 5, 1, points);
        assert_eq!(restored.writer.source_watermark(&source.source().layer_source()), None);
        assert!(ingest(&mut restored, &source));
        assert!(samples(&records(&mut restored)).any(|sample| sample.source == source.source().layer_source()));
    }

    #[test]
    fn auxiliary_actual_sequence_progression_and_many_sources_stay_bounded() {
        let mut fine = FineCloud::fresh().unwrap();
        let source_namespace = namespace("depth-front", "clock", 1, 1);
        for (stamp, sequence) in [(100, 0), (100, 1), (101, 0)] {
            let source = cloud(source_namespace.clone(), stamp, sequence, floor());
            assert!(ingest(&mut fine, &source));
            assert_eq!(fine.writer.source_watermark(&source_namespace), Some((stamp, sequence)));
            assert!(samples(&records(&mut fine)).any(|sample|
                (sample.source_timestamp_ns, sample.source_sequence) == (stamp, sequence)));
        }
        let mut many = FineCloud::fresh().unwrap();
        for index in 0..=nuwax_map_chunks::MAX_LAYER_SAMPLES {
            let source = cloud(namespace(&format!("depth-{index:03}"), "clock", 1, 1), 10, 0, floor());
            assert!(ingest(&mut many, &source));
            let retained = records(&mut many);
            assert!(retained.values().flat_map(|column| &column.heightfields)
                .all(|field| field.samples.len() <= nuwax_map_chunks::MAX_LAYER_SAMPLES));
            assert!(samples(&retained).any(|sample| sample.source == source.source().layer_source()),
                "the current admitted producer must retain evidence when producer count exceeds capacity");
        }
    }
}

#[cfg(test)]
#[path = "fine_ground_deletion_qualification.rs"]
mod ground_deletion_qualification;

#[cfg(test)]
#[path = "heightfield_consumer_controls.rs"]
mod heightfield_consumer_controls;

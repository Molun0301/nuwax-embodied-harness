//! 原点云拥有方维护的固定精度源；完整块提交和展示容量互不影响。
use super::{dynamic_cloud::DynamicCloudFilter, voxel_index::VoxelIndex, voxel_key, VOXEL_SIZE};
use ahash::AHashSet;
use nalgebra::Point3;
use nuwax_map_chunks::{Repository, SealedMap, Writer};
use std::collections::VecDeque;
use std::sync::{Arc, Weak};
#[path = "ground_observations.rs"]
mod ground_observations;

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
        source: Option<&crate::admitted_cloud::AcceptedMappingCloud>) -> Result<(), String> {
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let result = self.accumulate_inner(points, origin, timestamp_ns).and_then(|()| {
            let Some(accepted) = source else { return Ok(()); };
            let source = accepted.source();
            if timestamp_ns == 0 || source.timestamp_ns() != timestamp_ns { return Ok(()); }
            let sequence = source.sequence()?;
            let namespace = source.layer_source();
            if self.writer.source_watermark(&namespace).is_some_and(|old| (timestamp_ns, sequence) <= old) {
                return Ok(());
            }
            // 原准入分配是唯一统计来源；有界后缀沿原始索引投影，不能用重采样坐标替代。
            let Some(first) = accepted.map_points().len().checked_sub(points.len()) else { return Ok(()); };
            let original_points = &accepted.map_points()[first..];
            let dynamic = &self.dynamic;
            let indices = ground_observations::capture(&mut self.writer, original_points, sequence, timestamp_ns,
                Arc::clone(&namespace), |index| source.original_index(first + index),
                |p| dynamic.is_dynamic([p.x, p.y, p.z]))?;
            self.latest_ground_indices = indices.into();
            self.latest_ground_source = source.lidar().map(|scan| scan.downgrade());
            self.writer.record_source_watermark(namespace, timestamp_ns, sequence)?;
            Ok(())
        });
        // 存储故障后不能把部分更新公开成完整提交；由原拥有方的重建/新会话恢复。
        if let Err(error) = &result { self.failure = Some(error.clone()); }
        else {
            // Only successfully processed fine work demands the existing lazy
            // inverse-transform/allocation; sharing afterwards is cheap.
            self.latest_unclassified_scan = source.and_then(|accepted| accepted.lidar()).map(|accepted| accepted.scan().clone());
        }
        result
    }
    /// Rebuild one archived observation under a corrected pose. The original
    /// sequence/index identifies the sample; a transformed float is not a new hit.
    pub(super) fn capture_replayed_ground(&mut self, points: &[Point3<f64>],
        source: &crate::admitted_cloud::AdmittedCloudSource) -> Result<(), String> {
        if let Some(error) = &self.failure { return Err(error.clone()); }
        let timestamp = source.timestamp_ns();
        let sequence = source.sequence()?;
        if timestamp == 0 || points.len() != source.body_points().len() {
            return Err("invalid archived ground source".into());
        }
        let namespace = source.layer_source();
        if self.writer.source_watermark(&namespace).is_some_and(|old| (timestamp, sequence) <= old) {
            return Ok(());
        }
        let dynamic = &self.dynamic;
        let result = ground_observations::capture(&mut self.writer, points, sequence, timestamp,
            Arc::clone(&namespace), |index| source.original_index(index),
            |p| dynamic.is_dynamic([p.x, p.y, p.z]))
            .and_then(|_| self.writer.record_source_watermark(namespace, timestamp, sequence));
        if let Err(error) = &result { self.failure = Some(error.clone()); }
        result
    }
    fn accumulate_inner(&mut self, points: &[Point3<f64>], origin: Option<[f64; 3]>, timestamp_ns: u64) -> Result<(), String> {
        self.dynamic.clear_frame();
        let size = self.writer.voxel_size_m();
        let hits = points.iter().map(|p| voxel_key([p.x, p.y, p.z], size).ok_or("精细输入坐标无效"))
            .collect::<Result<AHashSet<_>, _>>()?;
        if let Some(origin) = origin {
            // 原动态过滤器的射线最多30米；只读这部分真实块，远场静态结构保留。
            let low = voxel_key(origin.map(|v| v - 30.0 - f64::from(size)), size).ok_or("精细动态窗口越界")?;
            let high = voxel_key(origin.map(|v| v + 30.0 + f64::from(size)), size).ok_or("精细动态窗口越界")?;
            let local = self.writer.region([low.0, low.1, low.2], [high.0, high.1, high.2], 2_000_000)?;
            let mut voxels: VoxelIndex = local.into_iter().map(|(key, _)| (key[0], key[1], key[2])).collect();
            self.dynamic.clear_observed_free_space_with_hits(&mut voxels, size, origin, timestamp_ns, points, &hits);
            self.writer.remove(self.dynamic.removed_keys().iter().map(|key| [key.0, key.1, key.2]))?;
        }
        if timestamp_ns == 0 {
            self.writer.seed_unknown(hits.into_iter().map(|key| [key.0, key.1, key.2]))
        } else {
            self.writer.observe(hits.into_iter().map(|key| [key.0, key.1, key.2]), timestamp_ns)
        }
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
mod unclassified_scan_tests {
    use super::*;
    use crate::lio_ingress::{AcceptedRegisteredCloud, CorrelatedOdometry, RegistrationBinding};

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
        let domain = nuwax_map_chunks::Domain { session: "owner".into(), generation: 3,
            frame_generation: 4, producer_generation: 5 };
        let sealed = fine.seal(domain, 1, 2).unwrap();
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
        let points=(0..8).flat_map(|x|(0..8).map(move|y|Point3::new(0.008+x as f64*0.022,0.008+y as f64*0.022,0.01))).collect::<Vec<_>>();
        let admitted=source(10,&points);let mut fine=FineCloud::fresh().unwrap();
        fine.accumulate(&points,None,10,Some(&admitted.clone().into())).unwrap();
        let layers=fine.writer.layer_observations([0,0,0]).unwrap();assert!(!layers.is_empty());
        fine.accumulate(&points,None,10,Some(&admitted.clone().into())).unwrap();
        assert_eq!(fine.writer.layer_observations([0,0,0]).unwrap(),layers);
        let next=source(11,&points);let mut resampled=points.clone();resampled[0].x+=1e-9;
        fine.accumulate(&resampled,None,11,Some(&next.clone().into())).unwrap();
        let layers=fine.writer.layer_observations([0,0,0]).unwrap();
        for layer in layers.values() { for sample in &layer.samples {
            let expected: [f64;3] = points[sample.point_index as usize].coords.into();
            assert_eq!(sample.point, expected, "statistics use the admitted allocation, never supplied reconstructed floats");
        }}
        let domain=nuwax_map_chunks::Domain{session:"owner".into(),generation:3,frame_generation:4,producer_generation:5};
        let sealed=fine.seal(domain,1,11).unwrap();let (file,_)=nuwax_map_chunks::archive_file(sealed.manifest(),&sealed.repository).unwrap();
        let (mut restored,_)=FineCloud::from_archive(file).unwrap();
        restored.accumulate(&points,None,10,Some(&admitted.clone().into())).unwrap();
        assert_eq!(restored.writer.layer_observations([0,0,0]).unwrap(),layers,"restored source watermark rejects old scan recounting");
    }

    #[test]
    fn corrected_replay_preserves_admitted_sample_identity_without_recounting() {
        let points=(0..8).flat_map(|x|(0..8).map(move|y|Point3::new(0.008+x as f64*0.022,0.008+y as f64*0.022,0.01))).collect::<Vec<_>>();
        let admitted=source(10,&points);let scan=admitted.scan();
        let transform=nalgebra::Isometry3::translation(0.2,0.0,0.1);
        let corrected=scan.body_points().iter().map(|point|transform*point).collect::<Vec<_>>();
        let mut fine=FineCloud::fresh().unwrap();
        fine.accumulate(&corrected,None,10,None).unwrap();
        fine.capture_replayed_ground(&corrected,&scan.clone().into()).unwrap();
        let layers=fine.writer.layer_observations([0,0,0]).unwrap();assert!(!layers.is_empty());
        for layer in layers.values(){
            assert!((layer.plane_origin[2]-0.11).abs()<1e-8);
            for sample in &layer.samples{assert_eq!(sample.source_sequence,10);assert_eq!(sample.source_timestamp_ns,10);let expected: [f64;3]=corrected[sample.point_index as usize].coords.into();assert_eq!(sample.point,expected);}
        }
        fine.capture_replayed_ground(&corrected,&scan.clone().into()).unwrap();
        assert_eq!(fine.writer.layer_observations([0,0,0]).unwrap(),layers);
        assert_eq!(fine.writer.layer_observations([0,0,0]).unwrap(),layers);
    }

}

#[cfg(test)]
mod auxiliary_source_tests {
    use super::*;
    use crate::admitted_cloud::{AcceptedMappingCloud, AdmittedCloudSource};
    use crate::lcm_runtime::mapping_observation::RegisteredObservation;
    use nalgebra::Isometry3;
    use nuwax_map_chunks::{LayerSource, LayerObservations};

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
    fn ingest(fine: &mut FineCloud, source: &AcceptedMappingCloud) {
        fine.accumulate(source.map_points(), None, source.source().timestamp_ns(), Some(source)).unwrap();
    }
    fn layers(fine: &mut FineCloud) -> LayerObservations {
        fine.writer.layer_observations([0, 0, 0]).unwrap()
    }
    fn domain() -> nuwax_map_chunks::Domain {
        nuwax_map_chunks::Domain { session: "aux-ground".into(), generation: 1,
            frame_generation: 1, producer_generation: 1 }
    }

    #[test]
    fn auxiliary_floor_and_wall_use_real_source_and_original_filtered_suffix_indices() {
        let mut points = floor();
        points.extend((0..8).flat_map(|y| (0..8).map(move |z|
            Point3::new(0.28, 0.008 + y as f64 * 0.02, 0.21 + z as f64 * 0.02))));
        let source = cloud(namespace("depth-front", "device-clock", 1, 1), 10, 0, points);
        let mut fine = FineCloud::fresh().unwrap();
        let suffix = &source.map_points()[8..];
        fine.accumulate(suffix, None, 10, Some(&source)).unwrap();
        let measured = layers(&mut fine);
        assert!(!measured.is_empty());
        for sample in measured.values().flat_map(|layer| &layer.samples) {
            assert_eq!(sample.source.as_ref(), source.source().layer_source().as_ref());
            assert_eq!(sample.source_sequence, 0, "actual zero-based auxiliary sequence is not synthesized");
            assert!(sample.point_index >= 8 * 3 + 4);
            let index = ((sample.point_index - 4) / 3) as usize;
            assert_eq!(sample.point, <[f64; 3]>::from(source.map_points()[index].coords));
            assert_eq!(sample.point[2], 0.01, "vertical wall must remain occupied, not measured ground");
        }
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
        ingest(&mut fine, &front);
        ingest(&mut fine, &left);
        let both = layers(&mut fine);
        let samples = &both.values().next().unwrap().samples;
        assert_eq!(samples.len(), nuwax_map_chunks::MAX_LAYER_SAMPLES);
        assert!(samples.iter().any(|sample| sample.source == front.source().layer_source()));
        assert!(samples.iter().any(|sample| sample.source == left.source().layer_source()));
        ingest(&mut fine, &left);
        assert_eq!(layers(&mut fine), both, "same source frame cannot vote twice");
        ingest(&mut fine, &reset);
        let retained = layers(&mut fine);
        let samples = &retained.values().next().unwrap().samples;
        for source in [&front, &left, &reset] {
            assert!(samples.iter().any(|sample| sample.source == source.source().layer_source()),
                "a real producer must not be starved by another clock's numeric offset");
        }
        assert_eq!(samples.len(), nuwax_map_chunks::MAX_LAYER_SAMPLES);
    }

    #[test]
    fn persisted_source_watermarks_are_scoped_to_clock_and_admitted_owner_epoch() {
        let original = cloud(namespace("depth-front", "stable-label", 1, 1), 100, 10, floor());
        let mut fine = FineCloud::fresh().unwrap();
        ingest(&mut fine, &original);
        let before = layers(&mut fine);
        let sealed = fine.seal(domain(), 1, 100).unwrap();
        assert_eq!(sealed.manifest().version, 4);
        let (file, _) = nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
        let (mut restored, _) = FineCloud::from_archive(file).unwrap();
        ingest(&mut restored, &original);
        assert_eq!(layers(&mut restored), before);
        for (clock, runtime, session) in [("new-clock", 1, 1), ("stable-label", 2, 1), ("stable-label", 1, 2)] {
            let source = cloud(namespace("depth-front", clock, runtime, session), 5, 0, floor());
            ingest(&mut restored, &source);
            assert!(layers(&mut restored).values().flat_map(|layer| &layer.samples)
                .any(|sample| sample.source == source.source().layer_source()));
        }
        let sealed = restored.seal(domain(), 2, 100).unwrap();
        assert_eq!(sealed.manifest().source_watermarks.len(), 4);
    }

    #[test]
    fn corrected_auxiliary_replay_retains_original_indices_and_is_idempotent() {
        let source = cloud(namespace("depth-front", "clock", 1, 1), 10, 4, floor());
        let admitted = source.source();
        assert!(matches!(admitted, AdmittedCloudSource::Auxiliary(_)));
        let pose = Isometry3::translation(0.2, 0.0, 0.1);
        let corrected: Vec<_> = admitted.body_points().iter().map(|point| pose * point).collect();
        let mut fine = FineCloud::fresh().unwrap();
        fine.accumulate(&corrected, None, 10, None).unwrap();
        fine.capture_replayed_ground(&corrected, &admitted).unwrap();
        let before = layers(&mut fine);
        assert!(!before.is_empty());
        for sample in before.values().flat_map(|layer| &layer.samples) {
            let index = ((sample.point_index - 4) / 3) as usize;
            assert_eq!(sample.point, <[f64; 3]>::from(corrected[index].coords));
            assert!((sample.point[2] - 0.11).abs() < 1e-12);
            assert_eq!(sample.source, admitted.layer_source());
            assert_eq!((sample.source_timestamp_ns, sample.source_sequence), (10, 4));
        }
        fine.capture_replayed_ground(&corrected, &admitted).unwrap();
        assert_eq!(layers(&mut fine), before);
    }

    #[test]
    fn legacy_layer_archive_does_not_invent_a_typed_source_cutoff() {
        let points = floor();
        let mut fine = FineCloud::fresh().unwrap();
        fine.accumulate(&points, None, 100, None).unwrap();
        ground_observations::capture(&mut fine.writer, &points, 100, 100, Arc::new(LayerSource::Legacy),
            |index| u32::try_from(index).ok(), |_| false).unwrap();
        let sealed = fine.seal(domain(), 1, 100).unwrap();
        let mut legacy = (**sealed.manifest()).clone();
        legacy.version = 3;
        assert!(legacy.source_watermarks.is_empty());
        let (file, _) = nuwax_map_chunks::archive_file(&legacy, &sealed.repository).unwrap();
        let (mut restored, _) = FineCloud::from_archive(file).unwrap();
        let source = cloud(namespace("depth-front", "clock", 1, 1), 5, 1, points);
        ingest(&mut restored, &source);
        assert!(layers(&mut restored).values().flat_map(|layer| &layer.samples)
            .any(|sample| sample.source == source.source().layer_source()));
    }
    #[test]
    fn auxiliary_actual_sequence_progression_and_many_sources_stay_bounded() {
        let mut fine = FineCloud::fresh().unwrap();
        let source_namespace = namespace("depth-front", "clock", 1, 1);
        for (stamp, sequence) in [(100, 0), (100, 1), (101, 0)] {
            let source = cloud(source_namespace.clone(), stamp, sequence, floor());
            ingest(&mut fine, &source);
            assert_eq!(fine.writer.source_watermark(&source_namespace), Some((stamp, sequence)));
            assert!(layers(&mut fine).values().flat_map(|layer| &layer.samples)
                .any(|sample| (sample.source_timestamp_ns, sample.source_sequence) == (stamp, sequence)));
        }
        let mut many = FineCloud::fresh().unwrap();
        for index in 0..=nuwax_map_chunks::MAX_LAYER_SAMPLES {
            let source = cloud(namespace(&format!("depth-{index:03}"), "clock", 1, 1), 10, 0, floor());
            ingest(&mut many, &source);
            let retained = layers(&mut many);
            assert!(retained.values().all(|layer| layer.samples.len() <= nuwax_map_chunks::MAX_LAYER_SAMPLES));
            assert!(retained.values().flat_map(|layer| &layer.samples)
                .any(|sample| sample.source == source.source().layer_source()),
                "the current admitted producer must retain evidence when producer count exceeds capacity");
        }
    }

}

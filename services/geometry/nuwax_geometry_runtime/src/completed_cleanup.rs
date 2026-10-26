//! Completed-map replay: original scans + committed final poses. This module
//! never changes the live map; its files belong to a private preview directory.
use crate::{
    dynamic_scene::{cell, Cell, DynamicScene},
    human_point_labels::{PointLabel, StaticReferences},
};
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use nalgebra::{Matrix4, Point3, Vector4};
use nuwax_cloud_cleanup::{
    offline_residual_cleanup::{asset_key, CleanupPreview, CleanupSettings, CompletedMap},
    scan_log::{FinalPoses, LoggedScan, ScanReader},
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fs::File,
    io::{BufReader, BufWriter, Read, Write},
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
pub(crate) struct Request {
    pub fine: PathBuf,
    pub dense: PathBuf,
    pub output: PathBuf,
    pub strength_percent: u8,
    pub scan_log: Option<PathBuf>,
    pub final_poses: Option<PathBuf>,
    pub source_timestamp_ns: u64,
    pub backend: Option<PathBuf>,
    pub occupancy: Option<PathBuf>,
    pub occupancy_descriptor: Option<serde_json::Value>,
    pub cancel_file: Option<PathBuf>,
}
#[derive(Serialize, Deserialize)]
pub(crate) struct Report {
    pub source_fine_sha256: String,
    pub source_dense_sha256: String,
    pub preview: CleanupPreview,
    pub offline_frames: usize,
    pub offline_points: usize,
    pub offline_complete: bool,
    pub offline_note: String,
    pub human_removed_voxels: usize,
    pub recovered_voxels: usize,
    pub recovered_points: usize,
    pub dense_before: usize,
    pub dense_after: usize,
    pub highest_before_m: Option<f64>,
    pub highest_after_m: Option<f64>,
}
fn err(e: impl std::fmt::Display) -> String {
    e.to_string()
}
fn digest(path: &Path) -> Result<String, String> {
    let mut f = File::open(path).map_err(err)?;
    let mut h = Sha256::new();
    let mut b = [0; 65536];
    loop {
        let n = f.read(&mut b).map_err(err)?;
        if n == 0 {
            break;
        }
        h.update(&b[..n]);
    }
    Ok(format!("{:x}", h.finalize()))
}
fn xyz_bits(p: [f64; 3]) -> [u32; 3] {
    p.map(|v| (v as f32).to_bits())
}
fn coarse(p: [f64; 3]) -> [i32; 3] {
    p.map(|v| (v / 0.3).floor() as i32)
}
fn transform(m: &Matrix4<f64>, p: [f64; 3]) -> Point3<f64> {
    let p = m * Vector4::new(p[0], p[1], p[2], 1.);
    Point3::new(p.x, p.y, p.z)
}
fn scan_points(
    scan: &LoggedScan,
    poses: &HashMap<u64, [f64; 16]>,
) -> (Vec<Point3<f64>>, Option<[f64; 3]>) {
    let m = poses
        .get(&scan.metadata.source_timestamp_ns)
        .map(|p| Matrix4::from_column_slice(p))
        .unwrap_or_else(|| {
            Matrix4::from_column_slice(&scan.metadata.map_from_odom)
                * Matrix4::from_column_slice(&scan.metadata.odom_from_body)
        });
    let points = scan.points.iter().map(|&p| transform(&m, p)).collect();
    let origin = scan
        .metadata
        .sensor_origin_body
        .map(|p| transform(&m, p).coords.into());
    (points, origin)
}
struct Ground {
    q: f32,
    points: HashMap<[i32; 2], Vec<[f64; 3]>>,
}
impl Ground {
    fn new(map: &CompletedMap) -> Self {
        let mut points: HashMap<_, Vec<_>> = HashMap::new();
        for &p in &map.ground_samples {
            let k = asset_key(p, map.voxel_size_m);
            points.entry([k[0], k[1]]).or_default().push(p);
        }
        Self {
            q: map.voxel_size_m,
            points,
        }
    }
    fn contains(&self, p: [f64; 3]) -> bool {
        let k = asset_key(p, self.q);
        self.points
            .get(&[k[0], k[1]])
            .is_some_and(|ps| ps.iter().any(|q| (p[2] - q[2]).abs() <= 0.03))
    }
    fn known_indices(&self, points: &[Point3<f64>]) -> Vec<u32> {
        points
            .iter()
            .enumerate()
            .filter_map(|(i, p)| {
                let k = asset_key([p.x, p.y, p.z], self.q);
                self.points.contains_key(&[k[0], k[1]]).then_some(i as u32)
            })
            .collect()
    }
    fn indices(&self, points: &[Point3<f64>]) -> Vec<u32> {
        points
            .iter()
            .enumerate()
            .filter_map(|(i, p)| self.contains([p.x, p.y, p.z]).then_some(i as u32))
            .collect()
    }
}
#[derive(Default)]
struct Support {
    stamps: HashSet<u64>,
    origins: Vec<[f64; 3]>,
    two_views: bool,
}
impl Support {
    fn observe(&mut self, stamp: u64, origin: Option<[f64; 3]>) {
        self.stamps.insert(stamp);
        if self.two_views {
            return;
        }
        if let Some(o) = origin {
            if let Some(&other) = self
                .origins
                .iter()
                .find(|p| (0..3).map(|i| (p[i] - o[i]).powi(2)).sum::<f64>() >= 0.25)
            {
                self.origins = vec![other, o];
                self.two_views = true;
            } else if !self.origins.contains(&o) {
                self.origins.push(o);
            }
        }
    }
    fn qualifies(&self) -> bool {
        self.stamps.len() >= 5 && self.two_views
    }
}
struct Offline {
    refs: StaticReferences,
    voids: HashMap<Cell, u64>,
    dynamic_points: HashSet<[u32; 3]>,
    recovery: HashMap<[i32; 3], Support>,
    void_coarse: HashSet<[i32; 3]>,
    frames: usize,
    points: usize,
    complete: bool,
    poses: HashMap<u64, [f64; 16]>,
}
enum Replay {
    Available(Offline),
    Unavailable(String),
}
fn replay(
    request: &Request,
    map: &CompletedMap,
    ground: &Ground,
    cancelled: &impl Fn() -> bool,
) -> Result<Replay, String> {
    let Some(path) = request.scan_log.as_ref() else {
        return Ok(Replay::Unavailable(
            "原始扫描日志缺失：仅完成地图浮空分析，未完成全量人体重判".into(),
        ));
    };
    let archived = nuwax_map_chunks::ArchiveReader::open(File::open(&request.fine).map_err(err)?)?;
    let root = archived.root();
    let mut scans = ScanReader::open(path)?;
    if scans.session.session_id != root.domain().session
        || scans.session.generation != root.domain().generation
    {
        return Err("原始扫描不属于此保存地图".into());
    }
    let mut poses = HashMap::new();
    if let Some(path) = &request.final_poses {
        let saved: FinalPoses =
            serde_json::from_reader(BufReader::new(File::open(path).map_err(err)?)).map_err(err)?;
        if saved.session != scans.session || saved.cloud_revision > root.revision() {
            return Err("最终位姿与保存地图不匹配".into());
        }
        let generation = root.domain().frame_generation;
        if !saved
            .frame_generation
            .map_or(generation == 0, |saved| saved == generation)
        {
            return Ok(Replay::Unavailable(
                "最终位姿的地图坐标代际与保存地图不一致；本次仅分析完成地图的浮空支撑".into(),
            ));
        }
        poses.extend(saved.poses);
    } else if root.domain().frame_generation > 0 {
        return Ok(Replay::Unavailable(
            "保存地图已做坐标校正，但缺少对应最终位姿；本次仅分析完成地图的浮空支撑".into(),
        ));
    }
    let pose_cut = poses.keys().copied().max().unwrap_or(0);
    let present: HashSet<_> = map.keys.iter().copied().collect();
    let mut scene = DynamicScene::default();
    scene.offline = true;
    let mut voids = HashMap::new();
    let mut frames = 0;
    let mut point_count = 0;
    let mut last = 0;
    while let Some(scan) = scans.next_scan()? {
        if cancelled() {
            return Err("清理已取消，原图未改变".into());
        }
        if scan.metadata.source_timestamp_ns > request.source_timestamp_ns {
            break;
        }
        if scan.metadata.source_timestamp_ns <= pose_cut
            && !poses.contains_key(&scan.metadata.source_timestamp_ns)
        {
            return Ok(Replay::Unavailable(
                "最终位姿未覆盖全部原始扫描；本次仅分析完成地图的浮空支撑".into(),
            ));
        }
        let (points, origin) = scan_points(&scan, &poses);
        if let Some(origin) = origin {
            if let Some(minimum) = scan.metadata.observation_minimum_range_m {
                if !minimum.is_finite() || minimum < 0.0 {
                    return Err("原始扫描近距标定无效".into());
                }
                scene.set_minimum_range_m(minimum);
            }
            scene.observe_with_known_ground(
                &points,
                origin,
                scan.metadata.source_timestamp_ns,
                &ground.indices(&points),
                &ground.known_indices(&points),
            );
            for k in scene.new_void_cells() {
                voids.insert(k, scan.metadata.source_timestamp_ns);
            }
        }
        frames += 1;
        point_count += points.len();
        last = scan.metadata.source_timestamp_ns;
    }
    if frames == 0 {
        return Err("原始扫描日志没有覆盖保存快照".into());
    }
    let mut refs = StaticReferences::default();
    let mut recovery: HashMap<[i32; 3], Support> = HashMap::new();
    let mut scans = ScanReader::open(path)?;
    while let Some(scan) = scans.next_scan()? {
        if cancelled() {
            return Err("清理已取消，原图未改变".into());
        }
        let stamp = scan.metadata.source_timestamp_ns;
        if stamp > request.source_timestamp_ns {
            break;
        }
        let (points, origin) = scan_points(&scan, &poses);
        let labels: Vec<_> = points
            .iter()
            .map(|p| {
                let p = [p.x, p.y, p.z];
                recovery
                    .entry(coarse(p))
                    .or_default()
                    .observe(stamp, origin);
                if ground.contains(p) {
                    PointLabel::Ground
                } else if present.contains(&asset_key(p, map.voxel_size_m))
                    && cell(p).is_some_and(|k| voids.get(&k).is_none_or(|&cleared| stamp > cleared))
                {
                    PointLabel::PendingNew
                } else {
                    PointLabel::Seed
                }
            })
            .collect();
        refs.observe(&points, &labels, stamp);
    }
    let mut void_coarse = HashSet::new();
    for k in voids.keys() {
        let lo = coarse([k.0 as f64 * 0.1, k.1 as f64 * 0.1, k.2 as f64 * 0.1]);
        let hi = coarse([
            (k.0 as f64 + 1.) * 0.1,
            (k.1 as f64 + 1.) * 0.1,
            (k.2 as f64 + 1.) * 0.1,
        ]);
        for x in lo[0]..=hi[0] {
            for y in lo[1]..=hi[1] {
                for z in lo[2]..=hi[2] {
                    void_coarse.insert([x, y, z]);
                }
            }
        }
    }
    let mut dynamic_points = HashSet::new();
    let mut scans = ScanReader::open(path)?;
    // Reasons are indexed by unchanged original source/index, not a shifted or
    // resampled point list. The original log remains immutable.
    let mut reasons = File::create(request.output.join("offline-labels.jsonl")).map_err(err)?;
    while let Some(scan) = scans.next_scan()? {
        if cancelled() {
            return Err("清理已取消，原图未改变".into());
        }
        if scan.metadata.source_timestamp_ns > request.source_timestamp_ns {
            break;
        }
        let (points, _) = scan_points(&scan, &poses);
        let mut labels: Vec<_> = points
            .iter()
            .map(|p| label([p.x, p.y, p.z], ground, &refs, &voids))
            .collect();
        crate::human_point_labels::grow(&points, &mut labels);
        for (p, l) in points.iter().zip(&labels) {
            if l.dynamic() {
                dynamic_points.insert(xyz_bits([p.x, p.y, p.z]));
            }
        }
        if last == request.source_timestamp_ns {
            for l in &mut labels {
                if *l == PointLabel::PendingNew {
                    *l = PointLabel::FinalStatic;
                }
            }
        }
        serde_json::to_writer(&mut reasons,&serde_json::json!({"source_timestamp_ns":scan.metadata.source_timestamp_ns,"source_sequence":scan.metadata.source_sequence,"labels":labels})).map_err(err)?;
        reasons.write_all(b"\n").map_err(err)?;
    }
    reasons.sync_all().map_err(err)?;
    Ok(Replay::Available(Offline {
        refs,
        voids,
        dynamic_points,
        recovery,
        void_coarse,
        frames,
        points: point_count,
        complete: last == request.source_timestamp_ns,
        poses,
    }))
}
fn label(
    p: [f64; 3],
    ground: &Ground,
    refs: &StaticReferences,
    voids: &HashMap<Cell, u64>,
) -> PointLabel {
    if ground.contains(p) {
        PointLabel::Ground
    } else if let Some(l) = refs.protect(p, None) {
        l
    } else if cell(p).is_some_and(|k| voids.contains_key(&k)) {
        PointLabel::Seed
    } else {
        PointLabel::PendingNew
    }
}
fn read_dense(path: &Path) -> Result<(serde_json::Value, Vec<[f64; 3]>), String> {
    let mut f = BufReader::new(File::open(path).map_err(err)?);
    let mut magic = [0; 8];
    f.read_exact(&mut magic).map_err(err)?;
    if &magic != b"NUWAXDC1" {
        return Err("完整扫描资产格式无效".into());
    }
    let mut n = [0; 4];
    f.read_exact(&mut n).map_err(err)?;
    let n = u32::from_le_bytes(n) as usize;
    if n > 65536 {
        return Err("完整扫描资产头无效".into());
    }
    let mut header = vec![0; n];
    f.read_exact(&mut header).map_err(err)?;
    let metadata: serde_json::Value = serde_json::from_slice(&header).map_err(err)?;
    let count = metadata["point_count"].as_u64().ok_or("完整扫描点数无效")? as usize;
    let expected = (12 + n) as u64 + (count as u64).checked_mul(12).ok_or("完整扫描大小溢出")?;
    if f.get_ref().metadata().map_err(err)?.len() != expected {
        return Err("完整扫描点数与实际文件大小不符".into());
    }
    let mut points = Vec::with_capacity(count);
    for _ in 0..count {
        let mut p = [0.; 3];
        for v in &mut p {
            let mut b = [0; 4];
            f.read_exact(&mut b).map_err(err)?;
            *v = f32::from_le_bytes(b) as f64;
        }
        if !p.iter().all(|v| v.is_finite()) {
            return Err("完整扫描含无效坐标".into());
        }
        points.push(p);
    }
    if f.read(&mut [0; 1]).map_err(err)? != 0 {
        return Err("完整扫描尾部多余数据".into());
    }
    Ok((metadata, points))
}
fn write_dense(
    path: &Path,
    mut metadata: serde_json::Value,
    points: &[[f64; 3]],
) -> Result<(), String> {
    if points.is_empty() {
        return Err("清理后地图为空，不能作为地图保存".into());
    }
    metadata["point_count"] = points.len().into();
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for p in points {
        for a in 0..3 {
            low[a] = low[a].min(p[a]);
            high[a] = high[a].max(p[a]);
        }
    }
    metadata["bounds_min"] = serde_json::json!(low);
    metadata["bounds_max"] = serde_json::json!(high);
    metadata["cloud_revision"] = (metadata["cloud_revision"]
        .as_u64()
        .ok_or("完整扫描版本缺失")?
        + 1)
    .into();
    let b = serde_json::to_vec(&metadata).map_err(err)?;
    let mut f = BufWriter::new(File::create(path).map_err(err)?);
    f.write_all(b"NUWAXDC1").map_err(err)?;
    f.write_all(&(b.len() as u32).to_le_bytes()).map_err(err)?;
    f.write_all(&b).map_err(err)?;
    for p in points {
        for v in p {
            f.write_all(&(*v as f32).to_le_bytes()).map_err(err)?;
        }
    }
    f.flush().map_err(err)?;
    f.get_ref().sync_all().map_err(err)
}
pub(crate) fn prepare(request: &Request) -> Result<Report, String> {
    std::fs::create_dir_all(&request.output).map_err(err)?;
    let cancelled = || request.cancel_file.as_ref().is_some_and(|p| p.exists());
    if cancelled() {
        return Err("清理已取消，原图未改变".into());
    }
    let archive = nuwax_map_chunks::ArchiveReader::open(File::open(&request.fine).map_err(err)?)?;
    let root = archive.root();
    let mut map = CompletedMap::read(&request.fine)?;
    let (metadata, dense) = read_dense(&request.dense)?;
    if metadata["session_id"].as_str() != Some(root.domain().session.as_str())
        || metadata["session_generation"].as_u64() != Some(root.domain().generation)
        || metadata["cloud_revision"].as_u64() != Some(root.revision())
        || metadata["source_timestamp_ns"].as_u64() != Some(root.source_timestamp_ns())
        || request.source_timestamp_ns != root.source_timestamp_ns()
    {
        return Err("累计扫描、细地图与指定保存快照来源不一致".into());
    }
    map.add_ground_fallback(&dense);
    let ground = Ground::new(&map);
    let mut source_xyz =
        BufWriter::new(File::create(request.output.join("source.xyzf32")).map_err(err)?);
    for k in &map.keys {
        for v in k {
            source_xyz
                .write_all(
                    &(((*v as f64 + 0.5) * f64::from(map.voxel_size_m)) as f32).to_le_bytes(),
                )
                .map_err(err)?;
        }
    }
    source_xyz.flush().map_err(err)?;
    source_xyz.get_ref().sync_all().map_err(err)?;
    let (offline, missing_note) = match replay(request, &map, &ground, &cancelled)? {
        Replay::Available(e) => (Some(e), String::new()),
        Replay::Unavailable(reason) => (None, reason),
    };
    let mut removed = HashSet::new();
    let mut retained = HashSet::new();
    if let Some(e) = &offline {
        for &p in &dense {
            let k = asset_key(p, map.voxel_size_m);
            let reason = label(p, &ground, &e.refs, &e.voids);
            let dynamic = reason.dynamic()
                || (reason == PointLabel::PendingNew && e.dynamic_points.contains(&xyz_bits(p)));
            if dynamic {
                removed.insert(k);
            } else {
                retained.insert(k);
            }
        }
        let present: HashSet<_> = map.keys.iter().copied().collect();
        removed.retain(|k| !retained.contains(k) && present.contains(k));
        for &k in &map.keys {
            if retained.contains(&k) || map.ground.contains(&k) {
                continue;
            }
            let p = k.map(|v| (v as f64 + 0.5) * f64::from(map.voxel_size_m));
            if label(p, &ground, &e.refs, &e.voids).dynamic() {
                removed.insert(k);
            }
        }
    }
    let human_candidates = removed.clone();
    // The final-support pass operates on the actual post-E keys, not a proxy
    // point cloud with a different floating-point quantum.
    let mut after = map.clone();
    let keep: Vec<_> = after
        .keys
        .iter()
        .enumerate()
        .filter_map(|(i, k)| (!removed.contains(k)).then_some(i))
        .collect();
    after.keys = keep.iter().map(|&i| map.keys[i]).collect();
    after.hits = keep.iter().map(|&i| map.hits[i]).collect();
    after.ground.retain(|k| !removed.contains(k));
    let mut preview = after.preview(
        CleanupSettings {
            strength_percent: request.strength_percent,
        },
        &cancelled,
    )?;
    removed.extend(preview.removed_keys.iter().copied());
    // DC1 stores f32 whereas the fine writer quantizes original f64 coordinates.
    // On an exact rounding boundary, preserve every possible present key if the
    // same archived point could belong to both a retained and a deleted key.
    let present: HashSet<_> = map.keys.iter().copied().collect();
    for &p in &dense {
        let keys = dense_keys(p, map.voxel_size_m, &present);
        if keys.iter().any(|k| !removed.contains(k)) {
            for k in keys {
                removed.remove(&k);
            }
        }
    }
    let mut recovered_keys = HashSet::new();
    let mut recovered_points = Vec::new();
    if let (Some(e), Some(log)) = (&offline, &request.scan_log) {
        let original: HashSet<_> = map.keys.iter().copied().collect();
        let mut scans = ScanReader::open(log)?;
        while let Some(scan) = scans.next_scan()? {
            if cancelled() {
                return Err("清理已取消，原图未改变".into());
            }
            if scan.metadata.source_timestamp_ns > request.source_timestamp_ns {
                break;
            }
            let (points, _) = scan_points(&scan, &e.poses);
            for p in points {
                let p = [p.x, p.y, p.z];
                let k = asset_key(p, map.voxel_size_m);
                // Any addition, including historically removed PendingNew, must
                // pass the original 0.3m / 5 sources / 0.5m / no-void rule.
                if recoverable(p, &original, map.voxel_size_m, e, &ground) {
                    recovered_keys.insert(k);
                    recovered_points.push(p);
                }
            }
        }
    }
    preview.source_points = map.keys.len();
    preview.removed_keys = removed.iter().copied().collect();
    preview.removed_keys.sort_unstable();
    preview.candidate_points = preview.removed_keys.len();
    if cancelled() {
        return Err("清理已取消，原图未改变".into());
    }
    // Exact preview membership is the only deletion input to the archive writer.
    let (cache_bytes, cache_policy) = crate::pointcloud::configured_writer_cache();
    nuwax_cloud_cleanup::offline_residual_cleanup::apply_fine_with_recovery_with_cache(
        &request.fine,
        &request.output.join("fine.bin"),
        &preview,
        &recovered_keys,
        cache_bytes,
        cache_policy,
    )?;
    let mut output_dense: Vec<_> = dense
        .iter()
        .copied()
        .filter(|p| {
            !dense_keys(*p, map.voxel_size_m, &present)
                .iter()
                .all(|k| removed.contains(k))
        })
        .collect();
    output_dense.extend(recovered_points.iter().copied());
    write_dense(&request.output.join("dense.bin"), metadata, &output_dense)?;
    let final_map = CompletedMap::read(&request.output.join("fine.bin"))?;
    write_other_members(request, &map, &final_map, &removed)?;
    let mut xyz =
        BufWriter::new(File::create(request.output.join("candidates.xyzf32")).map_err(err)?);
    for &k in &preview.removed_keys {
        for v in k {
            xyz.write_all(&(((v as f64 + 0.5) * f64::from(map.voxel_size_m)) as f32).to_le_bytes())
                .map_err(err)?;
        }
    }
    xyz.flush().map_err(err)?;
    xyz.get_ref().sync_all().map_err(err)?;
    let report = Report {
        source_fine_sha256: digest(&request.fine)?,
        source_dense_sha256: digest(&request.dense)?,
        preview,
        offline_frames: offline.as_ref().map_or(0, |e| e.frames),
        offline_points: offline.as_ref().map_or(0, |e| e.points),
        offline_complete: offline.as_ref().is_some_and(|e| e.complete),
        offline_note: match &offline {
            None => missing_note,
            Some(e) if !e.complete => "日志未覆盖快照尾帧，未把待定点统一转为静态".into(),
            Some(_) => "已使用完整原始扫描与已提交最终位姿重判；未使用语义相机".into(),
        },
        human_removed_voxels: human_candidates.intersection(&removed).count(),
        recovered_voxels: recovered_keys.len(),
        recovered_points: recovered_points.len(),
        dense_before: dense.len(),
        dense_after: output_dense.len(),
        highest_before_m: dense.iter().map(|p| p[2]).max_by(f64::total_cmp),
        highest_after_m: output_dense.iter().map(|p| p[2]).max_by(f64::total_cmp),
    };
    let mut f = File::create(request.output.join("preview.json")).map_err(err)?;
    serde_json::to_writer_pretty(&mut f, &report).map_err(err)?;
    f.sync_all().map_err(err)?;
    Ok(report)
}

fn recoverable(
    p: [f64; 3],
    original: &HashSet<[i32; 3]>,
    q: f32,
    e: &Offline,
    ground: &Ground,
) -> bool {
    let c = coarse(p);
    !original.contains(&asset_key(p, q))
        && !e.void_coarse.contains(&c)
        && e.recovery.get(&c).is_some_and(Support::qualifies)
        && !e.dynamic_points.contains(&xyz_bits(p))
        && !label(p, ground, &e.refs, &e.voids).dynamic()
}

fn dense_keys(p: [f64; 3], q: f32, present: &HashSet<[i32; 3]>) -> Vec<[i32; 3]> {
    let intervals = p.map(|x| {
        let v = x as f32;
        let down = v.next_down() as f64;
        let up = v.next_up() as f64;
        let inv = 1.0 / f64::from(q);
        [
            ((x + down) * 0.5 * inv).floor() as i32,
            ((x + up) * 0.5 * inv).floor() as i32,
        ]
    });
    let mut out = Vec::new();
    for x in intervals[0][0]..=intervals[0][1] {
        for y in intervals[1][0]..=intervals[1][1] {
            for z in intervals[2][0]..=intervals[2][1] {
                let k = [x, y, z];
                if present.contains(&k) {
                    out.push(k);
                }
            }
        }
    }
    if out.is_empty() {
        out.push(asset_key(p, q));
    }
    out
}
fn write_other_members(
    request: &Request,
    original: &CompletedMap,
    final_map: &CompletedMap,
    removed: &HashSet<[i32; 3]>,
) -> Result<(), String> {
    let q = f64::from(final_map.voxel_size_m);
    let centers: Vec<_> = final_map
        .keys
        .iter()
        .map(|k| k.map(|v| (v as f64 + 0.5) * q))
        .collect();
    let mut cloud = BufWriter::new(File::create(request.output.join("cloud.xyzf32")).map_err(err)?);
    for p in &centers {
        for &v in p {
            cloud.write_all(&(v as f32).to_le_bytes()).map_err(err)?;
        }
    }
    cloud.flush().map_err(err)?;
    cloud.get_ref().sync_all().map_err(err)?;
    let (Some(backend), Some(occupancy), Some(d)) = (
        &request.backend,
        &request.occupancy,
        &request.occupancy_descriptor,
    ) else {
        return Ok(());
    };
    let width = d["width"].as_u64().ok_or("占据图宽度缺失")? as u32;
    let height = d["height"].as_u64().ok_or("占据图高度缺失")? as u32;
    let resolution = d["resolution"].as_f64().ok_or("占据图分辨率缺失")?;
    let ox = d["origin_x"].as_f64().ok_or("占据图原点缺失")?;
    let oy = d["origin_y"].as_f64().ok_or("占据图原点缺失")?;
    let mut data: Vec<i8> = std::fs::read(occupancy)
        .map_err(err)?
        .into_iter()
        .map(|v| v as i8)
        .collect();
    if data.len() != width as usize * height as usize {
        return Err("占据图长度不符".into());
    }
    let index = |k: &[i32; 3]| -> Option<usize> {
        let x = (((k[0] as f64 + 0.5) * q - ox) / resolution).floor() as i64;
        let y = (((k[1] as f64 + 0.5) * q - oy) / resolution).floor() as i64;
        (x >= 0 && y >= 0 && x < i64::from(width) && y < i64::from(height))
            .then_some((y.max(0) as usize) * width as usize + x.max(0) as usize)
    };
    let keep: HashSet<_> = final_map
        .keys
        .iter()
        .filter(|k| !original.ground.contains(*k))
        .filter_map(index)
        .collect();
    for k in removed {
        if let Some(i) = index(k) {
            if !keep.contains(&i) && data[i] > 0 {
                data[i] = -1;
            }
        }
    }
    // Removing floating geometry does not manufacture a free-space observation.
    // Existing free and unknown semantics, including SCAN policy, stay intact.
    std::fs::write(
        request.output.join("occupancy.bin"),
        data.iter().map(|&v| v as u8).collect::<Vec<_>>(),
    )
    .map_err(err)?;
    let grid = nuwax_mapping::OccupancyGrid {
        data,
        width,
        height,
        resolution,
        origin: [ox, oy, 0.],
        origin_orientation: [0., 0., 0., 1.],
        frame_id: d["frame_id"].as_str().unwrap_or("map").into(),
        ts: d["timestamp_ns"]
            .as_u64()
            .unwrap_or(request.source_timestamp_ns) as f64
            / 1e9,
    };
    let payload = crate::mapping_backend::cleanup_completed_checkpoint(
        &std::fs::read(backend).map_err(err)?,
        final_map.voxel_size_m,
        removed,
        &final_map.keys,
        grid,
    )?;
    let mut f = File::create(request.output.join("backend.bin")).map_err(err)?;
    f.write_all(&payload).map_err(err)?;
    f.sync_all().map_err(err)
}

pub(crate) fn run_requested() -> Option<Result<(), String>> {
    let args: Vec<_> = std::env::args_os().collect();
    if args.get(1).is_none_or(|a| a != "--cleanup-completed-cloud") {
        return None;
    }
    Some((|| {
        let path = args.get(2).ok_or("缺少清理请求文件")?;
        let request: Request =
            serde_json::from_reader(File::open(path).map_err(err)?).map_err(err)?;
        prepare(&request)?;
        Ok(())
    })())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn final_poses_require_the_saved_map_frame_generation() {
        use nuwax_cloud_cleanup::scan_log::{ScanLog, ScanMetadata, Session};
        let folder = std::env::temp_dir().join(format!(
            "neh-cleanup-final-poses-{}",
            uuid::Uuid::new_v4().simple()
        ));
        std::fs::create_dir_all(&folder).unwrap();
        let session = Session {
            session_id: "位姿代际验证".into(),
            generation: 1,
        };
        let identity: [f64; 16] = nalgebra::Matrix4::identity().as_slice().try_into().unwrap();
        let mut log = ScanLog::open(&folder.join("scans"), session.clone()).unwrap();
        log.append(
            &ScanMetadata {
                source_timestamp_ns: 100,
                source_sequence: 1,
                sensor: "lidar".into(),
                source_generation: 1,
                odom_from_body: identity,
                map_from_odom: identity,
                sensor_origin_body: None,
                observation_minimum_range_m: None,
                point_count: 1,
            },
            [[0.025, 0.025, 0.025]].into_iter(),
            &[PointLabel::PendingNew],
        )
        .unwrap();
        log.sync().unwrap();
        for generation in [0, 2] {
            let mut writer = nuwax_map_chunks::Writer::new(
                nuwax_map_chunks::Repository::temporary().unwrap(),
                0.05,
                4 * 1024 * 1024,
            )
            .unwrap();
            writer.seed_unknown([[0, 0, 0]]).unwrap();
            let sealed = writer
                .seal(
                    nuwax_map_chunks::Domain {
                        session: session.session_id.clone(),
                        generation: 1,
                        frame_generation: generation,
                        producer_generation: 1,
                    },
                    1,
                    100,
                )
                .unwrap();
            let fine = folder.join(format!("fine-{generation}.bin"));
            let (mut archive, _) =
                nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
            std::io::copy(&mut archive, &mut File::create(&fine).unwrap()).unwrap();
            let map = CompletedMap::read(&fine).unwrap();
            let ground = Ground::new(&map);
            for supplied in [Some(generation), Some(1), None] {
                let poses = folder.join(format!("poses-{generation}-{supplied:?}.json"));
                let mut value = serde_json::json!({ "session": session, "cloud_revision": 1,
                    "poses": [[100, identity]] });
                if let Some(supplied) = supplied {
                    value["frame_generation"] = supplied.into();
                }
                std::fs::write(&poses, serde_json::to_vec(&value).unwrap()).unwrap();
                let output = folder.join(format!("output-{generation}-{supplied:?}"));
                std::fs::create_dir_all(&output).unwrap();
                let request = Request {
                    fine: fine.clone(),
                    dense: PathBuf::new(),
                    output,
                    strength_percent: 50,
                    scan_log: Some(log.path.clone()),
                    final_poses: Some(poses),
                    source_timestamp_ns: 100,
                    backend: None,
                    occupancy: None,
                    occupancy_descriptor: None,
                    cancel_file: None,
                };
                let outcome = replay(&request, &map, &ground, &|| false).unwrap();
                let expected =
                    supplied == Some(generation) || (supplied.is_none() && generation == 0);
                match outcome {
                    Replay::Available(result) => {
                        assert!(expected);
                        assert!(result.complete);
                        assert_eq!(result.frames, 1);
                    }
                    Replay::Unavailable(reason) => {
                        assert!(!expected);
                        assert!(reason.contains("代际"));
                    }
                }
            }
        }
        drop(log);
        std::fs::remove_dir_all(folder).unwrap();
    }

    #[test]
    fn historical_removed_point_must_still_pass_recovery_evidence() {
        let p = [1., 1., 1.];
        let mut e = Offline {
            refs: Default::default(),
            voids: Default::default(),
            dynamic_points: Default::default(),
            recovery: Default::default(),
            void_coarse: Default::default(),
            frames: 5,
            points: 5,
            complete: true,
            poses: Default::default(),
        };
        let ground = Ground {
            q: 0.05,
            points: Default::default(),
        };
        let original = HashSet::new();
        let support = e.recovery.entry(coarse(p)).or_default();
        for stamp in 1..=5 {
            support.observe(stamp, Some([0.; 3]));
        }
        assert_eq!(label(p, &ground, &e.refs, &e.voids), PointLabel::PendingNew);
        assert!(
            !recoverable(p, &original, 0.05, &e, &ground),
            "PendingNew/FinalStatic is not permission to revive a removed point"
        );
        e.recovery
            .get_mut(&coarse(p))
            .unwrap()
            .observe(6, Some([-0.3, 0., 0.]));
        e.recovery
            .get_mut(&coarse(p))
            .unwrap()
            .observe(7, Some([0.3, 0., 0.]));
        assert!(
            recoverable(p, &original, 0.05, &e, &ground),
            "the two qualifying views need not include the first viewpoint"
        );
        e.dynamic_points.insert(xyz_bits(p));
        assert!(
            !recoverable(p, &original, 0.05, &e, &ground),
            "grown points cannot bypass recovery through their own PendingNew cell"
        );
        e.dynamic_points.clear();
        e.void_coarse.insert(coarse(p));
        assert!(!recoverable(p, &original, 0.05, &e, &ground));
    }
}

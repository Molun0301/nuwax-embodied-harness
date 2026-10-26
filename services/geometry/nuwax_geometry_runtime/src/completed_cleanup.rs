//! 完成地图的DUFOMap离线判定：原始扫描与已提交最终位姿，仅写入私有预览目录。
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use nalgebra::{Matrix4, Vector4};
use nuwax_cloud_cleanup::{
    offline_dufomap,
    offline_residual_cleanup::{asset_key, CleanupPreview, CleanupSettings, CompletedMap},
    scan_log::{FinalPoses, LoggedScan, ScanReader},
};
use nuwax_dufomap_native::DufoMap;
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
fn transform(m: &Matrix4<f64>, p: [f32; 3]) -> [f32; 3] {
    let p = m * Vector4::new(p[0] as f64, p[1] as f64, p[2] as f64, 1.);
    [p.x as f32, p.y as f32, p.z as f32]
}
fn scan_points(
    scan: &LoggedScan,
    poses: &HashMap<u64, [f64; 16]>,
) -> Result<(Vec<[f32; 3]>, [f32; 3]), String> {
    let m = poses
        .get(&scan.metadata.source_timestamp_ns)
        .map(|p| Matrix4::from_column_slice(p))
        .unwrap_or_else(|| {
            let map_from_odom = Matrix4::from_column_slice(&scan.metadata.map_from_odom);
            let odom_from_body = Matrix4::from_column_slice(&scan.metadata.odom_from_body);
            // 文件位姿的合成结果明确初始化；发布优化与跨架构路径使用同一受检输出。
            let mut composed = Matrix4::zeros();
            composed.gemm(1.0, &map_from_odom, &odom_from_body, 0.0);
            composed
        });
    if !m.iter().all(|v| v.is_finite()) {
        return Err(format!(
            "原始扫描位姿无效：源时间{}，合成矩阵{:?}，原始矩阵{:?}/{:?}",
            scan.metadata.source_timestamp_ns,
            m,
            scan.metadata.map_from_odom,
            scan.metadata.odom_from_body
        ));
    }
    let origin = scan
        .metadata
        .sensor_origin_body
        .ok_or("原始扫描缺少雷达原点")?;
    Ok((
        scan.points.iter().map(|&p| transform(&m, p)).collect(),
        transform(&m, origin),
    ))
}
struct Offline {
    map: DufoMap,
    frames: usize,
    points: usize,
    complete: bool,
}
enum Replay {
    Available(Offline),
    Unavailable(String),
}
fn replay(request: &Request, cancelled: &impl Fn() -> bool) -> Result<Replay, String> {
    let unavailable = |reason: &str| {
        Ok(Replay::Unavailable(format!(
            "{reason}；仅执行半径离群兜底，离线判定不完整"
        )))
    };
    let Some(path) = &request.scan_log else {
        return unavailable("原始扫描日志缺失");
    };
    if path.with_extension("incomplete").exists() {
        return unavailable("原始扫描记录中断或丢帧");
    }
    let archive = nuwax_map_chunks::ArchiveReader::open(File::open(&request.fine).map_err(err)?)?;
    let root = archive.root();
    let mut scans = match ScanReader::open(path) {
        Ok(s) => s,
        Err(e) if e.contains("v1") => return unavailable(&e),
        Err(e) => return Err(e),
    };
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
            .map_or(generation == 0, |g| g == generation)
        {
            return unavailable("最终位姿的地图坐标代际与保存地图不一致");
        }
        poses.extend(saved.poses);
    } else if root.domain().frame_generation > 0 {
        return unavailable("保存地图已做坐标校正，但缺少对应最终位姿");
    }
    let pose_cut = poses.keys().copied().max().unwrap_or(0);
    let mut dufo = DufoMap::new(&offline_dufomap::params(request.strength_percent)?)?;
    let (mut frames, mut points, mut last) = (0, 0, 0);
    while let Some(scan) = scans.next_scan()? {
        if cancelled() {
            return Err("清理已取消，原图未改变".into());
        }
        let stamp = scan.metadata.source_timestamp_ns;
        if stamp > request.source_timestamp_ns {
            break;
        }
        if stamp <= last {
            return Err("原始扫描时间未严格前进".into());
        }
        if stamp <= pose_cut && !poses.contains_key(&stamp) {
            return unavailable("最终位姿未覆盖选中的原始扫描");
        }
        let (world, origin) = match scan_points(&scan, &poses) {
            Ok(v) => v,
            Err(e) => return unavailable(&e),
        };
        dufo.insert(&world, origin, false)?;
        frames += 1;
        points += world.len();
        last = stamp;
    }
    if frames == 0 {
        return unavailable("原始扫描日志没有覆盖保存快照");
    }
    dufo.propagate();
    Ok(Replay::Available(Offline {
        map: dufo,
        frames,
        points,
        complete: !scans.truncated
            && scans.dropped_frames == 0
            && scans.observed_until_ns >= request.source_timestamp_ns,
    }))
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
    let map = CompletedMap::read(&request.fine)?;
    let (metadata, dense) = read_dense(&request.dense)?;
    if metadata["session_id"].as_str() != Some(root.domain().session.as_str())
        || metadata["session_generation"].as_u64() != Some(root.domain().generation)
        || metadata["cloud_revision"].as_u64() != Some(root.revision())
        || metadata["source_timestamp_ns"].as_u64() != Some(root.source_timestamp_ns())
        || request.source_timestamp_ns != root.source_timestamp_ns()
    {
        return Err("累计扫描、细地图与指定保存快照来源不一致".into());
    }
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
    let (offline, missing_note) = match replay(request, &cancelled)? {
        Replay::Available(e) => (Some(e), String::new()),
        Replay::Unavailable(reason) => (None, reason),
    };
    let centers: Vec<[f32; 3]> = map
        .keys
        .iter()
        .map(|k| k.map(|v| ((v as f64 + 0.5) * f64::from(map.voxel_size_m)) as f32))
        .collect();
    let dense_f32: Vec<_> = dense.iter().map(|p| p.map(|v| v as f32)).collect();
    let present: HashSet<_> = map.keys.iter().copied().collect();
    let (fine_flags, dense_flags) = if let Some(e) = &offline {
        (e.map.seen_free(&centers)?, e.map.seen_free(&dense_f32)?)
    } else {
        let fine_flags =
            offline_dufomap::radius_outliers(&centers, request.strength_percent, &cancelled)?;
        let candidates: HashSet<_> = map
            .keys
            .iter()
            .zip(&fine_flags)
            .filter_map(|(&k, &v)| v.then_some(k))
            .collect();
        // 完整资产保留重复观测；邻居数量以规范细体素为准，重复的同一点不能伪造空间支持。
        let dense_flags = dense
            .iter()
            .map(|&p| {
                dense_keys(p, map.voxel_size_m, &present)
                    .iter()
                    .all(|k| candidates.contains(k))
            })
            .collect::<Vec<_>>();
        (fine_flags, dense_flags)
    };
    let mut removed: HashSet<_> = map
        .keys
        .iter()
        .zip(&fine_flags)
        .filter_map(|(&k, &dynamic)| dynamic.then_some(k))
        .collect();
    let mut retained = HashSet::new();
    for (&p, &dynamic) in dense.iter().zip(&dense_flags) {
        for k in dense_keys(p, map.voxel_size_m, &present) {
            if dynamic {
                if present.contains(&k) {
                    removed.insert(k);
                }
            } else {
                retained.insert(k);
            }
        }
    }
    // 单精度边界上的静态回波保护其可能属于的每个精细体素，避免量化误差造成误删。
    removed.retain(|k| !retained.contains(k));
    let mut removed_keys: Vec<_> = removed.iter().copied().collect();
    removed_keys.sort_unstable();
    let preview = CleanupPreview {
        settings: CleanupSettings {
            strength_percent: request.strength_percent,
        },
        gap_tolerance_m: 0.0,
        source_points: map.keys.len(),
        candidate_points: removed_keys.len(),
        measured_ground_points: map.ground.len(),
        components: Vec::new(),
        removed_keys,
    };
    if cancelled() {
        return Err("清理已取消，原图未改变".into());
    }
    // Exact preview membership is the only deletion input to the archive writer.
    let (cache_bytes, cache_policy) = crate::pointcloud::configured_writer_cache();
    nuwax_cloud_cleanup::offline_residual_cleanup::apply_fine_with_recovery_with_cache(
        &request.fine,
        &request.output.join("fine.bin"),
        &preview,
        &HashSet::new(),
        cache_bytes,
        cache_policy,
    )?;
    let output_dense: Vec<_> = dense
        .iter()
        .copied()
        .filter(|p| {
            !dense_keys(*p, map.voxel_size_m, &present)
                .iter()
                .all(|k| removed.contains(k))
        })
        .collect();

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
            Some(e) if !e.complete => {
                "日志进度未覆盖保存快照或存在截断/丢帧；已用可用扫描判定，离线输入不完整".into()
            }
            Some(_) => "已用v2选中扫描与最终位姿完成DUFOMap离线判定".into(),
        },
        human_removed_voxels: if offline.is_some() { removed.len() } else { 0 },
        recovered_voxels: 0,
        recovered_points: 0,
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
    fn offline_cleanup_removes_person_from_unfiltered_saved_assets_and_keeps_static() {
        use crate::dynamic_scene::tests::{corridor, ORIGIN};
        use nuwax_cloud_cleanup::scan_log::{ScanLog, ScanMetadata, Session};
        let folder =
            std::env::temp_dir().join(format!("neh-dufo-saved-{}", uuid::Uuid::new_v4().simple()));
        std::fs::create_dir_all(&folder).unwrap();
        let session = Session {
            session_id: "unfiltered".into(),
            generation: 1,
        };
        let mut log = ScanLog::open(&folder.join("scans"), session.clone()).unwrap();
        let identity = Matrix4::identity().as_slice().try_into().unwrap();
        let mut dense = Vec::new();
        let mut human = HashSet::new();
        let mut static_keys = HashSet::new();
        for frame in 0..60 {
            let (points, truth) = corridor(
                (frame < 30).then_some(-1.0 + frame as f64 * 2.0 / 29.0),
                (frame % 3) as f64,
            );
            let stamp = (frame + 1) * 100_000_000;
            let metadata = ScanMetadata {
                source_timestamp_ns: stamp,
                source_sequence: frame + 1,
                sensor: "lidar".into(),
                source_generation: 1,
                odom_from_body: identity,
                map_from_odom: identity,
                sensor_origin_body: Some(ORIGIN.map(|v| v as f32)),
                observation_minimum_range_m: None,
                point_count: points.len(),
            };
            log.append(
                &metadata,
                points.iter().map(|p| [p.x as f32, p.y as f32, p.z as f32]),
                None,
            )
            .unwrap();
            log.progress(stamp, 0).unwrap();
            for (p, on_person) in points.iter().zip(truth) {
                let p = [p.x, p.y, p.z];
                let key = asset_key(p, 0.05);
                if on_person {
                    if p[2] > 0.35 {
                        human.insert(key);
                    }
                } else {
                    static_keys.insert(key);
                }
                dense.push(p);
            }
        }
        log.sync().unwrap();
        let original: HashSet<_> = dense.iter().map(|&p| asset_key(p, 0.05)).collect();
        let mut writer = nuwax_map_chunks::Writer::new(
            nuwax_map_chunks::Repository::temporary().unwrap(),
            0.05,
            4 * 1024 * 1024,
        )
        .unwrap();
        writer.seed_unknown(original.iter().copied()).unwrap();
        let sealed = writer
            .seal(
                nuwax_map_chunks::Domain {
                    session: session.session_id.clone(),
                    generation: 1,
                    frame_generation: 0,
                    producer_generation: 1,
                },
                1,
                6_000_000_000,
            )
            .unwrap();
        let fine = folder.join("fine.bin");
        let (mut archive, _) =
            nuwax_map_chunks::archive_file(sealed.manifest(), &sealed.repository).unwrap();
        std::io::copy(&mut archive, &mut File::create(&fine).unwrap()).unwrap();
        let dense_path = folder.join("dense.bin");
        write_dense(&dense_path,serde_json::json!({"session_id":session.session_id,"session_generation":1,"cloud_revision":0,"source_timestamp_ns":6_000_000_000u64}),&dense).unwrap();
        let request = Request {
            fine,
            dense: dense_path,
            output: folder.join("cleaned"),
            strength_percent: 50,
            scan_log: Some(log.path.clone()),
            final_poses: None,
            source_timestamp_ns: 6_000_000_000,
            backend: None,
            occupancy: None,
            occupancy_descriptor: None,
            cancel_file: None,
        };
        let report = prepare(&request).unwrap();
        assert!(report.offline_complete);
        assert_eq!(report.offline_frames, 60);
        let after: HashSet<_> = CompletedMap::read(&request.output.join("fine.bin"))
            .unwrap()
            .keys
            .into_iter()
            .collect();
        let human: HashSet<_> = human.difference(&static_keys).copied().collect();
        assert!(!human.is_empty());
        let removed = human.difference(&after).count();
        assert!(
            removed as f64 / human.len() as f64 > 0.95,
            "未过滤保存地图的行人身体应被后续射线移除：{removed}/{}",
            human.len()
        );
        assert!(static_keys.is_subset(&after), "墙与地面应完整保留");
        assert!(after.is_subset(&original));
        assert_eq!(
            report.preview.candidate_points,
            original.difference(&after).count()
        );
        assert!(report.dense_after < report.dense_before);
        drop(log);
        std::fs::remove_dir_all(folder).unwrap();
    }
    #[test]
    fn final_pose_transforms_body_returns_and_sensor_origin_together() {
        use nuwax_cloud_cleanup::scan_log::ScanMetadata;
        let identity = Matrix4::identity().as_slice().try_into().unwrap();
        let metadata = ScanMetadata {
            source_timestamp_ns: 10,
            source_sequence: 1,
            sensor: "lidar".into(),
            source_generation: 1,
            odom_from_body: identity,
            map_from_odom: identity,
            sensor_origin_body: Some([0.28945, 0., -0.046825]),
            observation_minimum_range_m: None,
            point_count: 1,
        };
        let scan = LoggedScan {
            metadata,
            points: vec![[-2., 1., 3.]],
            intensities: None,
        };
        let corrected = nalgebra::Isometry3::new(
            nalgebra::Vector3::new(-5., 2., 1.),
            nalgebra::Vector3::new(0., 0., std::f64::consts::FRAC_PI_2),
        );
        let poses = HashMap::from([(
            10,
            corrected.to_homogeneous().as_slice().try_into().unwrap(),
        )]);
        let (points, origin) = scan_points(&scan, &poses).unwrap();
        for (&actual, expected) in points[0].iter().zip([-6., 0., 4.]) {
            assert!(
                (actual - expected).abs() < 1e-6,
                "单精度地图坐标应保持最终三维变换精度"
            );
        }
        for (actual, expected) in origin.into_iter().zip([-5., 2.28945, 0.953175]) {
            assert!((actual - expected).abs() < 1e-6);
        }
        let (uncorrected, _) = scan_points(&scan, &HashMap::new()).unwrap();
        assert_eq!(uncorrected, scan.points);
    }
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
                sensor_origin_body: Some([0.; 3]),
                observation_minimum_range_m: None,
                point_count: 1,
            },
            [[0.025, 0.025, 0.025]].into_iter(),
            None,
        )
        .unwrap();
        log.progress(100, 0).unwrap();
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
                let outcome = replay(&request, &|| false).unwrap();
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
}

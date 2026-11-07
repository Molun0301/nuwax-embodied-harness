//! v2 原始扫描日志：机体系单精度点、完整记录校验与源时间进度。
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fs::{File, OpenOptions},
    io::{BufReader, Read, Seek, Write},
    os::fd::AsRawFd,
    path::{Path, PathBuf},
};
mod save_policy;
mod pose_spool;
pub use save_policy::SavePolicy;
pub use pose_spool::PoseSpool;
const MAGIC: &[u8; 8] = b"NEHSCAN2";
const MAX_POINTS: usize = 2_000_000;
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ScanMetadata {
    pub source_timestamp_ns: u64,
    pub source_sequence: u64,
    pub sensor: String,
    pub source_generation: u64,
    pub odom_from_body: [f64; 16],
    pub map_from_odom: [f64; 16],
    pub sensor_origin_body: Option<[f32; 3]>,
    #[serde(default)]
    pub observation_minimum_range_m: Option<f64>,
    pub point_count: usize,
}
#[derive(Clone, Debug)]
pub struct LoggedScan {
    pub metadata: ScanMetadata,
    pub points: Vec<[f32; 3]>,
    pub intensities: Option<Vec<f32>>,
}
#[derive(Clone, Debug)]
pub enum ReplayRecord {
    Scan(LoggedScan),
    Progress,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct Session {
    pub session_id: String,
    pub generation: u64,
}
#[derive(Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Record {
    Scan {
        metadata: ScanMetadata,
        intensity: bool,
    },
    Progress {
        source_timestamp_ns: u64,
        dropped_frames: u64,
    },
}
pub struct ScanLog {
    file: File,
    pub path: PathBuf,
    session: Session,
    last_stamp: u64,
}
fn error(e: impl std::fmt::Display) -> String {
    e.to_string()
}
fn write_json(file: &mut impl Write, value: &impl Serialize) -> Result<(), String> {
    let b = serde_json::to_vec(value).map_err(error)?;
    file.write_all(&(b.len() as u32).to_le_bytes())
        .map_err(error)?;
    file.write_all(&b).map_err(error)
}
fn read_json<T: serde::de::DeserializeOwned>(file: &mut impl Read) -> Result<T, String> {
    let mut n = [0; 4];
    file.read_exact(&mut n).map_err(error)?;
    let n = u32::from_le_bytes(n) as usize;
    if n > 65536 {
        return Err("原始扫描元数据长度无效".into());
    }
    let mut b = vec![0; n];
    file.read_exact(&mut b).map_err(error)?;
    serde_json::from_slice(&b).map_err(error)
}
pub fn session_path(root: &Path, session: &Session) -> PathBuf {
    let id = Sha256::digest(format!("{}:{}", session.session_id, session.generation).as_bytes());
    root.join(format!("{:x}.scan", id))
}
pub fn scan_directory() -> PathBuf {
    let g = nuwax_config::geometry().expect("启动已捕获 geometry 配置");
    if g.raw_scan_directory.is_empty() {
        nuwax_config::resolve_path(&g.runtime_data_dir).join("scan_logs")
    } else {
        nuwax_config::resolve_path(&g.raw_scan_directory)
    }
}
pub fn storage_budget_bytes() -> Result<u64, String> {
    Ok(nuwax_config::geometry()?.raw_scan_max_mib as u64 * 1024 * 1024)
}
fn lock(file: &File, flags: i32) -> Result<(), String> {
    // 锁随文件描述符释放；异常退出不会遗留伪活动会话。
    if unsafe { libc::flock(file.as_raw_fd(), flags) } == 0 {
        Ok(())
    } else {
        Err(error(std::io::Error::last_os_error()))
    }
}

fn directory_lock(root: &Path) -> Result<File, String> {
    std::fs::create_dir_all(root).map_err(error)?;
    let file = OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(root.join(".scan-budget.lock"))
        .map_err(error)?;
    lock(&file, libc::LOCK_EX)?;
    Ok(file)
}

fn reserve(root: &Path, additional: u64) -> Result<(), String> {
    let used = std::fs::read_dir(root)
        .map_err(error)?
        .try_fold(0u64, |sum, item| {
            let item = item.map_err(error)?;
            let size = item.metadata().map_err(error)?.len();
            Ok::<_, String>(sum.saturating_add(size))
        })?;
    if used.saturating_add(additional) > storage_budget_bytes()? {
        return Err("原始扫描暂存已达目录预算，当前会话后续扫描不再记录；请先保存并结束建图会话或清理已有暂存".into());
    }
    Ok(())
}

fn reclaim_saved(root: &Path, protected: &Path) -> Result<(), String> {
    for item in std::fs::read_dir(root).map_err(error)? {
        let path = item.map_err(error)?.path();
        if path.extension().and_then(|v| v.to_str()) != Some("saved") {
            continue;
        }
        let scan = path.with_extension("scan");
        if scan == protected {
            continue;
        }
        let Some(stem) = scan.file_stem().and_then(|value| value.to_str()) else {
            continue;
        };
        let Ok(file) = OpenOptions::new().read(true).write(true).open(&scan) else {
            continue;
        };
        if lock(&file, libc::LOCK_EX | libc::LOCK_NB).is_err() {
            continue;
        }
        let saved_size = std::fs::read_to_string(&path)
            .ok()
            .and_then(|value| value.trim().parse::<u64>().ok());
        // 保存后仍有追加的会话不能回收；只依据已发布资产记录的完整字节范围。
        if saved_size != Some(file.metadata().map_err(error)?.len()) {
            continue;
        }
        for entry in std::fs::read_dir(root).map_err(error)? {
            let member = entry.map_err(error)?.path();
            let Some(name) = member.file_name().and_then(|v| v.to_str()) else {
                continue;
            };
            if member == scan
                || member == path
                || member == scan.with_extension("incomplete")
                || name.starts_with(&format!("{stem}-poses-"))
            {
                std::fs::remove_file(member).map_err(error)?;
            }
        }
    }
    Ok(())
}

impl ScanLog {
    pub fn open(root: &Path, session: Session) -> Result<Self, String> {
        let _guard = directory_lock(root)?;
        let path = session_path(root, &session);
        if path.with_extension("incomplete").exists() {
            return Err("此会话的原始扫描此前已经中断，不能接续为完整记录".into());
        }
        reclaim_saved(root, &path)?;
        let mut header = MAGIC.to_vec();
        write_json(&mut header, &session)?;
        if !path.exists() {
            reserve(root, header.len() as u64 + 64)?;
        }
        let mut file = OpenOptions::new()
            .read(true)
            .append(true)
            .create(true)
            .open(&path)
            .map_err(error)?;
        lock(&file, libc::LOCK_EX | libc::LOCK_NB)?;
        let mut last_stamp = 0;
        if file.metadata().map_err(error)?.len() == 0 {
            reserve(root, header.len() as u64 + 64)?;
            file.write_all(&header).map_err(error)?;
            file.sync_all().map_err(error)?;
        } else {
            let mut reader = ScanReader::open(&path)?;
            if reader.session != session {
                return Err("原始扫描日志会话不匹配".into());
            }
            while let Some(frame) = reader.next_scan()? {
                last_stamp = frame.metadata.source_timestamp_ns;
            }
            if reader.truncated {
                file.set_len(reader.valid_bytes).map_err(error)?;
                std::fs::write(
                    path.with_extension("incomplete"),
                    "断电截断，已恢复完整记录前缀",
                )
                .map_err(error)?;
            }
        }
        Ok(Self {
            file,
            path,
            session,
            last_stamp,
        })
    }
    fn record(&mut self, header: &Record, bytes: &[u8]) -> Result<(), String> {
        let mut payload = Vec::new();
        write_json(&mut payload, header)?;
        payload.extend_from_slice(bytes);
        let mut frame = Vec::with_capacity(payload.len() + 36);
        frame.extend_from_slice(&(payload.len() as u32).to_le_bytes());
        frame.extend_from_slice(&payload);
        frame.extend_from_slice(&Sha256::digest(&payload));
        let root = self.path.parent().ok_or("原始扫描目录缺失")?;
        let _guard = directory_lock(root)?;
        reserve(root, frame.len() as u64 + 64)?;
        let before = self.file.metadata().map_err(error)?.len();
        if let Err(e) = self.file.write_all(&frame) {
            self.file.set_len(before).map_err(error)?;
            return Err(error(e));
        }
        Ok(())
    }
    pub fn append(
        &mut self,
        metadata: &ScanMetadata,
        points: impl ExactSizeIterator<Item = [f32; 3]>,
        intensities: Option<&[f32]>,
    ) -> Result<(), String> {
        if metadata.point_count != points.len()
            || metadata.point_count > MAX_POINTS
            || intensities.is_some_and(|v| v.len() != points.len())
        {
            return Err("原始扫描点数与坐标/强度不一致".into());
        }
        if metadata.source_timestamp_ns <= self.last_stamp {
            return Err("原始扫描日志时间未前进".into());
        }
        let mut bytes =
            Vec::with_capacity(metadata.point_count * if intensities.is_some() { 16 } else { 12 });
        for p in points {
            for v in p {
                if !v.is_finite() {
                    return Err("原始扫描坐标无效".into());
                }
                bytes.extend_from_slice(&v.to_le_bytes());
            }
        }
        if let Some(values) = intensities {
            for v in values {
                if !v.is_finite() {
                    return Err("原始扫描强度无效".into());
                }
                bytes.extend_from_slice(&v.to_le_bytes());
            }
        }
        self.record(
            &Record::Scan {
                metadata: metadata.clone(),
                intensity: intensities.is_some(),
            },
            &bytes,
        )?;
        self.last_stamp = metadata.source_timestamp_ns;
        Ok(())
    }
    pub fn progress(
        &mut self,
        source_timestamp_ns: u64,
        dropped_frames: u64,
    ) -> Result<(), String> {
        self.record(
            &Record::Progress {
                source_timestamp_ns,
                dropped_frames,
            },
            &[],
        )
    }
    pub fn sync(&self) -> Result<(), String> {
        self.file.sync_all().map_err(error)
    }
    pub fn session(&self) -> &Session {
        &self.session
    }
}
pub struct ScanReader {
    file: std::io::Take<BufReader<File>>,
    pub session: Session,
    pub observed_until_ns: u64,
    pub dropped_frames: u64,
    pub truncated: bool,
    pub valid_bytes: u64,
}
impl ScanReader {
    pub fn open(path: &Path) -> Result<Self, String> {
        let mut file = File::open(path).map_err(error)?;
        let mut magic = [0; 8];
        file.read_exact(&mut magic).map_err(error)?;
        if &magic == b"NEHSCAN1" {
            return Err(
                "原始扫描日志 v1 已退役，不能用于 DUFOMap 离线判定；请重新建图生成 v2 日志".into(),
            );
        }
        if &magic != MAGIC {
            return Err("未知原始扫描日志格式".into());
        }
        let session = read_json(&mut file)?;
        let valid_bytes = file.stream_position().map_err(error)?;
        Ok(Self {
            file: BufReader::with_capacity(1024 * 1024, file).take(u64::MAX),
            session,
            observed_until_ns: 0,
            dropped_frames: 0,
            truncated: false,
            valid_bytes,
        })
    }
    pub fn next_scan(&mut self) -> Result<Option<LoggedScan>, String> {
        loop {
            match self.next_record()? {
                Some(ReplayRecord::Scan(scan)) => return Ok(Some(scan)),
                Some(ReplayRecord::Progress) => {}
                None => return Ok(None),
            }
        }
    }
    /// 每次只解析一条记录，按日志顺序读取扫描和源时间进度。
    pub fn next_record(&mut self) -> Result<Option<ReplayRecord>, String> {
        let mut size = [0; 4];
        let first = self.file.read(&mut size[..1]).map_err(error)?;
        if first == 0 {
            return Ok(None);
        }
        if let Err(e) = self.file.read_exact(&mut size[1..]) {
            return self.tail(e);
        }
        let n = u32::from_le_bytes(size) as usize;
        if !(4..=MAX_POINTS * 16 + 65540).contains(&n) {
            return Err("原始扫描记录长度无效".into());
        }
        let mut bytes = vec![0; n];
        if let Err(e) = self.file.read_exact(&mut bytes) {
            return self.tail(e);
        }
        let mut checksum = [0; 32];
        if let Err(e) = self.file.read_exact(&mut checksum) {
            return self.tail(e);
        }
        if Sha256::digest(&bytes)[..] != checksum {
            return Err("原始扫描记录校验失败".into());
        }
        let mut input = std::io::Cursor::new(&bytes);
        let header: Record = read_json(&mut input)?;
        let offset = input.position() as usize;
        self.valid_bytes = self.file.get_mut().stream_position().map_err(error)?;
        match header {
            Record::Progress {
                source_timestamp_ns,
                dropped_frames,
            } => {
                if offset != n || source_timestamp_ns < self.observed_until_ns {
                    return Err("原始扫描进度无效".into());
                }
                self.observed_until_ns = source_timestamp_ns;
                self.dropped_frames = self.dropped_frames.max(dropped_frames);
                Ok(Some(ReplayRecord::Progress))
            }
            Record::Scan {
                metadata,
                intensity,
            } => {
                if metadata.point_count > MAX_POINTS
                    || n - offset != metadata.point_count * if intensity { 16 } else { 12 }
                {
                    return Err("原始扫描点数与记录大小不一致".into());
                }
                let floats: Vec<f32> = bytes[offset..]
                    .chunks_exact(4)
                    .map(|b| f32::from_le_bytes(b.try_into().unwrap()))
                    .collect();
                if !floats.iter().all(|v| v.is_finite()) {
                    return Err("原始扫描含无效坐标或强度".into());
                }
                let points = floats[..metadata.point_count * 3]
                    .chunks_exact(3)
                    .map(|p| [p[0], p[1], p[2]])
                    .collect();
                let intensities = intensity.then(|| floats[metadata.point_count * 3..].to_vec());
                Ok(Some(ReplayRecord::Scan(LoggedScan {
                    metadata,
                    points,
                    intensities,
                })))
            }
        }
    }
    fn tail(&mut self, e: std::io::Error) -> Result<Option<ReplayRecord>, String> {
        if e.kind() != std::io::ErrorKind::UnexpectedEof {
            return Err(error(e));
        }
        self.truncated = true;
        Ok(None)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FinalPoses {
    pub session: Session,
    pub cloud_revision: u64,
    #[serde(default)]
    pub frame_generation: Option<u64>,
    pub poses: Vec<(u64, [f64; 16])>,
}
/// Written only after the corresponding PGO map has committed. A saved snapshot
/// selects the newest revision not newer than itself, never a future trajectory.
pub fn save_final_poses(root: &Path, poses: &FinalPoses) -> Result<PathBuf, String> {
    let _directory_lock = directory_lock(root)?;
    let stem = session_path(root, &poses.session);
    let stem = stem.file_stem().unwrap().to_string_lossy();
    let path = root.join(format!("{stem}-poses-{}.json", poses.cloud_revision));
    let temporary = path.with_extension("json.writing");
    let bytes = serde_json::to_vec(poses).map_err(error)?;
    reserve(root, bytes.len() as u64)?;
    let mut file = OpenOptions::new()
        .create(true)
        .truncate(true)
        .write(true)
        .open(&temporary)
        .map_err(error)?;
    file.write_all(&bytes).map_err(error)?;
    file.sync_all().map_err(error)?;
    std::fs::rename(&temporary, &path).map_err(error)?;
    File::open(root).map_err(error)?.sync_all().map_err(error)?;
    Ok(path)
}

#[cfg(test)]
mod storage_tests {
    use super::*;

    fn root(name: &str) -> PathBuf {
        let path =
            std::env::temp_dir().join(format!("neh-scan-storage-{}-{name}", std::process::id()));
        std::fs::create_dir_all(&path).unwrap();
        path
    }
    fn session(name: &str) -> Session {
        Session {
            session_id: name.into(),
            generation: 1,
        }
    }

    #[test]
    fn directory_budget_rejects_new_files_and_poses_before_writing() {
        let root = root("budget");
        let file = File::create(root.join("existing.scan")).unwrap();
        file.set_len(storage_budget_bytes().unwrap()).unwrap();
        let current = session("over-budget");
        assert!(ScanLog::open(&root, current.clone())
            .err()
            .unwrap()
            .contains("预算"));
        assert!(!session_path(&root, &current).exists());
        assert!(save_final_poses(
            &root,
            &FinalPoses {
                session: current,
                cloud_revision: 1,
                frame_generation: Some(0),
                poses: vec![]
            }
        )
        .is_err());
        assert_eq!(std::fs::read_dir(&root).unwrap().count(), 2);
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn reclamation_requires_saved_full_length_and_no_live_writer() {
        let root = root("reclaim");
        let saved = ScanLog::open(&root, session("saved")).unwrap();
        let path = saved.path.clone();
        std::fs::write(
            path.with_extension("saved"),
            path.metadata().unwrap().len().to_string(),
        )
        .unwrap();
        let next = ScanLog::open(&root, session("next")).unwrap();
        assert!(path.exists(), "活动会话不能回收");
        drop(next);
        drop(saved);
        let third = ScanLog::open(&root, session("third")).unwrap();
        assert!(!path.exists(), "完整副本已发布且写入器关闭后回收暂存");
        drop(third);
        let unsaved = ScanLog::open(&root, session("newer-than-save")).unwrap();
        let path = unsaved.path.clone();
        std::fs::write(
            path.with_extension("saved"),
            (path.metadata().unwrap().len() - 1).to_string(),
        )
        .unwrap();
        drop(unsaved);
        let fourth = ScanLog::open(&root, session("fourth")).unwrap();
        assert!(path.exists(), "保存范围未覆盖新数据，必须保留");
        drop(fourth);
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn interrupted_session_is_never_resumed_as_complete_history() {
        let root = root("incomplete");
        let current = session("failed");
        std::fs::write(
            session_path(&root, &current).with_extension("incomplete"),
            b"",
        )
        .unwrap();
        assert!(ScanLog::open(&root, current).is_err());
        std::fs::remove_dir_all(root).unwrap();
    }
}

#[cfg(test)]
mod v2_tests {
    use super::*;
    fn metadata(stamp: u64) -> ScanMetadata {
        let identity = nalgebra::Matrix4::identity().as_slice().try_into().unwrap();
        ScanMetadata {
            source_timestamp_ns: stamp,
            source_sequence: 3,
            sensor: "lidar".into(),
            source_generation: 1,
            odom_from_body: identity,
            map_from_odom: identity,
            sensor_origin_body: Some([0.28945, 0., -0.046825]),
            observation_minimum_range_m: Some(0.2),
            point_count: 2,
        }
    }
    fn folder(name: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("neh-v2-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).unwrap();
        p
    }
    #[test]
    fn v2_preserves_order_pose_origin_optional_intensity_and_progress() {
        let root = folder("roundtrip");
        let session = Session {
            session_id: "test".into(),
            generation: 1,
        };
        let mut log = ScanLog::open(&root, session.clone()).unwrap();
        let m = metadata(10);
        let points = [[1., 2., 3.], [1., 2., 3.]];
        log.append(&m, points.into_iter(), Some(&[4., 5.])).unwrap();
        log.append(&metadata(20), points.into_iter(), None).unwrap();
        log.progress(30, 0).unwrap();
        log.sync().unwrap();
        let mut reader = ScanReader::open(&log.path).unwrap();
        let scan = reader.next_scan().unwrap().unwrap();
        assert_eq!(scan.metadata, m);
        assert_eq!(scan.points, points);
        assert_eq!(scan.intensities, Some(vec![4., 5.]));
        assert!(reader.next_scan().unwrap().unwrap().intensities.is_none());
        assert!(reader.next_scan().unwrap().is_none());
        assert_eq!(reader.observed_until_ns, 30);
        assert!(!reader.truncated);
        drop(log);
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn every_torn_record_boundary_recovers_only_complete_prefix() {
        let root = folder("torn");
        let session = Session {
            session_id: "test".into(),
            generation: 1,
        };
        let mut log = ScanLog::open(&root, session.clone()).unwrap();
        log.append(&metadata(10), [[1., 2., 3.]; 2].into_iter(), None)
            .unwrap();
        log.sync().unwrap();
        let prefix = std::fs::read(&log.path).unwrap();
        log.append(&metadata(20), [[4., 5., 6.]; 2].into_iter(), None)
            .unwrap();
        log.sync().unwrap();
        let whole = std::fs::read(&log.path).unwrap();
        drop(log);
        let path = session_path(&root, &session);
        // 覆盖长度、元数据、坐标和校验尾部的每一个截断字节。
        for cut in prefix.len() + 1..whole.len() {
            std::fs::write(&path, &whole[..cut]).unwrap();
            let mut reader = ScanReader::open(&path).unwrap();
            let mut stamps = Vec::new();
            while let Some(scan) = reader.next_scan().unwrap() {
                stamps.push(scan.metadata.source_timestamp_ns);
            }
            assert_eq!(stamps[0], 10);
            assert!(stamps == [10] || stamps == [10, 20]);
            assert!(reader.valid_bytes <= cut as u64);
            if reader.truncated {
                assert!(reader.valid_bytes < cut as u64);
            }
        }
        std::fs::write(&path, &whole[..whole.len() - 1]).unwrap();
        let recovered = ScanLog::open(&root, session).unwrap();
        assert!(recovered.path.with_extension("incomplete").exists());
        assert!(std::fs::metadata(&path).unwrap().len() < (whole.len() - 1) as u64);
        drop(recovered);
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn legacy_format_and_checksum_corruption_are_explicitly_rejected() {
        let root = folder("reject");
        let path = root.join("old.scan");
        std::fs::write(&path, b"NEHSCAN1").unwrap();
        assert!(ScanReader::open(&path).err().unwrap().contains("v1"));
        let session = Session {
            session_id: "test".into(),
            generation: 1,
        };
        let mut log = ScanLog::open(&root, session).unwrap();
        log.append(&metadata(10), [[1., 2., 3.]; 2].into_iter(), None)
            .unwrap();
        log.sync().unwrap();
        let mut bytes = std::fs::read(&log.path).unwrap();
        let last = bytes.len() - 1;
        bytes[last] ^= 1;
        std::fs::write(&log.path, bytes).unwrap();
        let mut reader = ScanReader::open(&log.path).unwrap();
        assert!(reader.next_scan().unwrap_err().contains("校验"));
        drop(log);
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn one_record_reader_yields_at_every_progress_before_the_next_scan() {
        let root = folder("single-record");
        let session = Session {
            session_id: "single-record".into(),
            generation: 1,
        };
        let mut log = ScanLog::open(&root, session.clone()).unwrap();
        log.append(&metadata(10), [[1., 2., 3.]; 2].into_iter(), None)
            .unwrap();
        for stamp in 10..=1010 {
            log.progress(stamp, 0).unwrap();
        }
        log.append(&metadata(2000), [[4., 5., 6.]; 2].into_iter(), None)
            .unwrap();
        log.progress(2000, 0).unwrap();
        let mut reader = ScanReader::open(&log.path).unwrap();
        assert!(
            matches!(reader.next_record().unwrap(), Some(ReplayRecord::Scan(scan))
            if scan.metadata.source_timestamp_ns == 10)
        );
        assert_eq!(reader.observed_until_ns, 0);
        for stamp in 10..=1010 {
            assert!(matches!(
                reader.next_record().unwrap(),
                Some(ReplayRecord::Progress)
            ));
            assert_eq!(
                reader.observed_until_ns, stamp,
                "单步读取不能跨过下一条进度"
            );
        }
        assert!(
            matches!(reader.next_record().unwrap(), Some(ReplayRecord::Scan(scan))
            if scan.metadata.source_timestamp_ns == 2000)
        );
        assert_eq!(reader.observed_until_ns, 1010);
        assert!(matches!(
            reader.next_record().unwrap(),
            Some(ReplayRecord::Progress)
        ));
        assert_eq!(reader.observed_until_ns, 2000);
        assert!(reader.next_record().unwrap().is_none());
        drop(log);
        std::fs::remove_dir_all(root).unwrap();
    }
}

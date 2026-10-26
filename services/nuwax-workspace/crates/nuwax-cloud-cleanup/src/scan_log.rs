//! Complete append-only source scans. Labels never replace original returns.
//! Frames retain source time, original index (record order), body pose and origin.
use crate::human_point_labels::PointLabel;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fs::{File, OpenOptions},
    io::{BufReader, Read, Write},
    os::fd::AsRawFd,
    path::{Path, PathBuf},
};
const MAGIC: &[u8; 8] = b"NEHSCAN1";
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ScanMetadata {
    pub source_timestamp_ns: u64,
    pub source_sequence: u64,
    pub sensor: String,
    pub source_generation: u64,
    pub odom_from_body: [f64; 16],
    pub map_from_odom: [f64; 16],
    pub sensor_origin_body: Option<[f64; 3]>,
    #[serde(default)]
    pub observation_minimum_range_m: Option<f64>,
    pub point_count: usize,
}
#[derive(Clone, Debug)]
pub struct LoggedScan {
    pub metadata: ScanMetadata,
    pub points: Vec<[f64; 3]>,
    pub labels: Vec<PointLabel>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Session {
    pub session_id: String,
    pub generation: u64,
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
/// 原始扫描目录与全部暂存文件的共同预算；空值遵循默认运行数据根。
pub fn scan_directory() -> PathBuf {
    std::env::var_os("NUWAX_RAW_SCAN_DIRECTORY")
        .filter(|value| !value.to_string_lossy().trim().is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(std::env::var_os("NUWAX_RUNTIME_DATA_DIR")
            .filter(|value| !value.is_empty()).unwrap_or_else(|| "runtime_data".into())).join("scan_logs"))
}

pub fn storage_budget_bytes() -> Result<u64, String> {
    let value = match std::env::var("NUWAX_RAW_SCAN_MAX_MIB") {
        Ok(value) => value,
        Err(std::env::VarError::NotPresent) => "8192".into(),
        Err(_) => return Err("NUWAX_RAW_SCAN_MAX_MIB 必须是有效文本".into()),
    };
    let value = if value.trim().is_empty() { "8192" } else { value.trim() };
    let mib = value.bytes().all(|byte| byte.is_ascii_digit()).then(|| value.parse::<u64>().ok()).flatten()
        .filter(|mib| (64..=1_048_576).contains(mib))
        .ok_or("NUWAX_RAW_SCAN_MAX_MIB 必须是64～1048576的整数（MiB）")?;
    Ok(mib * 1024 * 1024)
}

fn lock(file: &File, flags: i32) -> Result<(), String> {
    // 锁随文件描述符释放；异常退出不会遗留伪活动会话。
    if unsafe { libc::flock(file.as_raw_fd(), flags) } == 0 { Ok(()) }
    else { Err(error(std::io::Error::last_os_error())) }
}

fn directory_lock(root: &Path) -> Result<File, String> {
    std::fs::create_dir_all(root).map_err(error)?;
    let file = OpenOptions::new().create(true).truncate(false).write(true)
        .open(root.join(".scan-budget.lock")).map_err(error)?;
    lock(&file, libc::LOCK_EX)?;
    Ok(file)
}

fn reserve(root: &Path, additional: u64) -> Result<(), String> {
    let used = std::fs::read_dir(root).map_err(error)?.try_fold(0u64, |sum, item| {
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
        if path.extension().and_then(|v| v.to_str()) != Some("saved") { continue; }
        let scan = path.with_extension("scan");
        if scan == protected { continue; }
        let Some(stem) = scan.file_stem().and_then(|value| value.to_str()) else { continue; };
        let Ok(file) = OpenOptions::new().read(true).write(true).open(&scan) else { continue; };
        if lock(&file, libc::LOCK_EX | libc::LOCK_NB).is_err() { continue; }
        let saved_size = std::fs::read_to_string(&path).ok().and_then(|value| value.trim().parse::<u64>().ok());
        // 保存后仍有追加的会话不能回收；只依据已发布资产记录的完整字节范围。
        if saved_size != Some(file.metadata().map_err(error)?.len()) { continue; }
        for entry in std::fs::read_dir(root).map_err(error)? {
            let member = entry.map_err(error)?.path();
            let Some(name) = member.file_name().and_then(|v| v.to_str()) else { continue; };
            if member == scan || member == path || member == scan.with_extension("incomplete") || name.starts_with(&format!("{stem}-poses-")) {
                std::fs::remove_file(member).map_err(error)?;
            }
        }
    }
    Ok(())
}

impl ScanLog {
    pub fn open(root: &Path, session: Session) -> Result<Self, String> {
        let _directory_lock = directory_lock(root)?;
        let path = session_path(root, &session);
        if path.with_extension("incomplete").exists() {
            return Err("此会话的原始扫描此前已经中断，不能接续为完整记录".into());
        }
        reclaim_saved(root, &path)?;
        let mut header = MAGIC.to_vec();
        write_json(&mut header, &session)?;
        if !path.exists() { reserve(root, header.len() as u64 + 64)?; }
        let mut file = OpenOptions::new().read(true).append(true).create(true)
            .open(&path).map_err(error)?;
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
                last_stamp = last_stamp.max(frame.metadata.source_timestamp_ns);
            }
        }
        Ok(Self {
            file,
            path,
            session,
            last_stamp,
        })
    }
    pub fn append(
        &mut self,
        metadata: &ScanMetadata,
        points: impl ExactSizeIterator<Item = [f64; 3]>,
        labels: &[PointLabel],
    ) -> Result<(), String> {
        if metadata.point_count != points.len() || metadata.point_count != labels.len() {
            return Err("原始扫描点数与标签不一致".into());
        }
        if metadata.source_timestamp_ns <= self.last_stamp {
            return Err("原始扫描日志时间未前进".into());
        }
        let root = self.path.parent().ok_or("原始扫描目录缺失")?;
        let _directory_lock = directory_lock(root)?;
        let mut frame = Vec::with_capacity(metadata.point_count.saturating_mul(25) + 1024);
        write_json(&mut frame, metadata)?;
        for p in points {
            for value in p {
                frame.extend_from_slice(&value.to_le_bytes());
            }
        }
        for label in labels {
            frame.push(code(*label));
        }
        reserve(root, frame.len() as u64 + 64)?;
        let original_size = self.file.metadata().map_err(error)?.len();
        if let Err(failure) = self.file.write_all(&frame) {
            // 写入中断时回退至上一完整帧，复制保存不读取半帧。
            self.file.set_len(original_size).map_err(error)?;
            return Err(error(failure));
        }
        self.last_stamp = metadata.source_timestamp_ns;
        Ok(())
    }
    pub fn sync(&self) -> Result<(), String> {
        self.file.sync_all().map_err(error)
    }
    pub fn session(&self) -> &Session {
        &self.session
    }
}
pub struct ScanReader {
    file: BufReader<File>,
    pub session: Session,
}
impl ScanReader {
    pub fn open(path: &Path) -> Result<Self, String> {
        let mut file = File::open(path).map_err(error)?;
        let mut magic = [0; 8];
        file.read_exact(&mut magic).map_err(error)?;
        if &magic != MAGIC {
            return Err("未知原始扫描日志格式".into());
        }
        let session = read_json(&mut file)?;
        Ok(Self {
            file: BufReader::with_capacity(1024 * 1024, file),
            session,
        })
    }
    pub fn next_scan(&mut self) -> Result<Option<LoggedScan>, String> {
        use std::io::{Seek, SeekFrom};
        let position = self.file.stream_position().map_err(error)?;
        let mut first = [0];
        if self.file.read(&mut first).map_err(error)? == 0 {
            return Ok(None);
        }
        self.file.seek(SeekFrom::Start(position)).map_err(error)?;
        let metadata: ScanMetadata = read_json(&mut self.file)?;
        // Wire-format bound only. It never changes a geometric decision.
        if metadata.point_count > 2_000_000 {
            return Err("原始扫描帧点数超出源格式上限".into());
        }
        let mut points = Vec::with_capacity(metadata.point_count);
        for _ in 0..metadata.point_count {
            let mut p = [0.; 3];
            for v in &mut p {
                let mut b = [0; 8];
                self.file.read_exact(&mut b).map_err(error)?;
                *v = f64::from_le_bytes(b);
            }
            if !p.iter().all(|v| v.is_finite()) {
                return Err("原始扫描坐标无效".into());
            }
            points.push(p);
        }
        let mut labels = Vec::with_capacity(metadata.point_count);
        for _ in 0..metadata.point_count {
            let mut b = [0];
            self.file.read_exact(&mut b).map_err(error)?;
            labels.push(decode(b[0])?);
        }
        Ok(Some(LoggedScan {
            metadata,
            points,
            labels,
        }))
    }
}
fn code(v: PointLabel) -> u8 {
    match v {
        PointLabel::Ground => 0,
        PointLabel::SurfacePlane => 1,
        PointLabel::SurfaceLine => 2,
        PointLabel::SurfacePoint => 3,
        PointLabel::Seed => 4,
        PointLabel::Grown => 5,
        PointLabel::PendingNew => 6,
        PointLabel::FinalStatic => 7,
        PointLabel::PendingTrack => 8,
    }
}
fn decode(v: u8) -> Result<PointLabel, String> {
    Ok(match v {
        0 => PointLabel::Ground,
        1 => PointLabel::SurfacePlane,
        2 => PointLabel::SurfaceLine,
        3 => PointLabel::SurfacePoint,
        4 => PointLabel::Seed,
        5 => PointLabel::Grown,
        6 => PointLabel::PendingNew,
        7 => PointLabel::FinalStatic,
        8 => PointLabel::PendingTrack,
        _ => return Err("未知原始扫描标签".into()),
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn raw_source_order_and_pending_labels_survive_reload() {
        let root = std::env::temp_dir().join(format!("neh-scan-test-{}", std::process::id()));
        let session = Session {
            session_id: "test".into(),
            generation: 1,
        };
        let path = session_path(&root, &session);
        let _ = std::fs::remove_file(&path);
        let mut log = ScanLog::open(&root, session.clone()).unwrap();
        let m = ScanMetadata {
            source_timestamp_ns: 1,
            source_sequence: 1,
            sensor: "lidar".into(),
            source_generation: 1,
            odom_from_body: [0.; 16],
            map_from_odom: [0.; 16],
            sensor_origin_body: Some([0.; 3]),
            observation_minimum_range_m: Some(0.5),
            point_count: 2,
        };
        let raw = [[1., 2., 3.], [1., 2., 3.]];
        log.append(
            &m,
            raw.into_iter(),
            &[PointLabel::Seed, PointLabel::PendingNew],
        )
        .unwrap();
        log.sync().unwrap();
        let mut reader = ScanReader::open(&path).unwrap();
        assert_eq!(reader.session, session);
        let frame = reader.next_scan().unwrap().unwrap();
        assert_eq!(frame.points, raw);
        assert_eq!(frame.metadata, m);
        assert_eq!(frame.labels, [PointLabel::Seed, PointLabel::PendingNew]);
        assert!(reader.next_scan().unwrap().is_none());
        std::fs::remove_file(path).unwrap();
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
        let path = std::env::temp_dir().join(format!("neh-scan-storage-{}-{name}", std::process::id()));
        std::fs::create_dir_all(&path).unwrap();
        path
    }
    fn session(name: &str) -> Session {
        Session { session_id: name.into(), generation: 1 }
    }

    #[test]
    fn directory_budget_rejects_new_files_and_poses_before_writing() {
        let root = root("budget");
        let file = File::create(root.join("existing.scan")).unwrap();
        file.set_len(storage_budget_bytes().unwrap()).unwrap();
        let current = session("over-budget");
        assert!(ScanLog::open(&root, current.clone()).err().unwrap().contains("预算"));
        assert!(!session_path(&root, &current).exists());
        assert!(save_final_poses(&root, &FinalPoses { session: current, cloud_revision: 1, frame_generation: Some(0), poses: vec![] }).is_err());
        assert_eq!(std::fs::read_dir(&root).unwrap().count(), 2);
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn reclamation_requires_saved_full_length_and_no_live_writer() {
        let root = root("reclaim");
        let saved = ScanLog::open(&root, session("saved")).unwrap();
        let path = saved.path.clone();
        std::fs::write(path.with_extension("saved"), path.metadata().unwrap().len().to_string()).unwrap();
        let next = ScanLog::open(&root, session("next")).unwrap();
        assert!(path.exists(), "活动会话不能回收");
        drop(next);
        drop(saved);
        let third = ScanLog::open(&root, session("third")).unwrap();
        assert!(!path.exists(), "完整副本已发布且写入器关闭后回收暂存");
        drop(third);
        let unsaved = ScanLog::open(&root, session("newer-than-save")).unwrap();
        let path = unsaved.path.clone();
        std::fs::write(path.with_extension("saved"), (path.metadata().unwrap().len() - 1).to_string()).unwrap();
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
        std::fs::write(session_path(&root, &current).with_extension("incomplete"), b"").unwrap();
        assert!(ScanLog::open(&root, current).is_err());
        std::fs::remove_dir_all(root).unwrap();
    }
}

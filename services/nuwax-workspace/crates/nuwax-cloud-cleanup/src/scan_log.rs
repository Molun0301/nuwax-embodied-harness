//! Complete append-only source scans. Labels never replace original returns.
//! Frames retain source time, original index (record order), body pose and origin.
use crate::human_point_labels::PointLabel;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fs::{File, OpenOptions},
    io::{BufReader, BufWriter, Read, Write},
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
    file: BufWriter<File>,
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
impl ScanLog {
    pub fn open(root: &Path, session: Session) -> Result<Self, String> {
        std::fs::create_dir_all(root).map_err(error)?;
        let path = session_path(root, &session);
        let mut file = OpenOptions::new()
            .read(true)
            .append(true)
            .create(true)
            .open(&path)
            .map_err(error)?;
        let mut last_stamp = 0;
        if file.metadata().map_err(error)?.len() == 0 {
            file.write_all(MAGIC).map_err(error)?;
            write_json(&mut file, &session)?;
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
            file: BufWriter::with_capacity(1024 * 1024, file),
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
        write_json(&mut self.file, metadata)?;
        for p in points {
            for value in p {
                self.file.write_all(&value.to_le_bytes()).map_err(error)?;
            }
        }
        for label in labels {
            self.file.write_all(&[code(*label)]).map_err(error)?;
        }
        self.file.flush().map_err(error)?;
        self.last_stamp = metadata.source_timestamp_ns;
        Ok(())
    }
    pub fn sync(&self) -> Result<(), String> {
        self.file.get_ref().sync_all().map_err(error)
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
    pub poses: Vec<(u64, [f64; 16])>,
}
/// Written only after the corresponding PGO map has committed. A saved snapshot
/// selects the newest revision not newer than itself, never a future trajectory.
pub fn save_final_poses(root: &Path, poses: &FinalPoses) -> Result<PathBuf, String> {
    std::fs::create_dir_all(root).map_err(error)?;
    let stem = session_path(root, &poses.session);
    let stem = stem.file_stem().unwrap().to_string_lossy();
    let path = root.join(format!("{stem}-poses-{}.json", poses.cloud_revision));
    let temporary = path.with_extension("json.writing");
    let bytes = serde_json::to_vec(poses).map_err(error)?;
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

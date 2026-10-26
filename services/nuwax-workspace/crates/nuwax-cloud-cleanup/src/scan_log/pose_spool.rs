//! 最终位姿的私有流式暂存，取消及交接失败时随拥有方删除。
use super::*;
use std::sync::atomic::{AtomicU64, Ordering};

const POSE_BYTES: usize = 8 + 16 * 8;
static NEXT_SPOOL: AtomicU64 = AtomicU64::new(0);

#[derive(Debug)]
pub struct PoseSpool {
    file: Option<File>,
    path: PathBuf,
    len: u64,
}
impl PoseSpool {
    pub fn new(root: PathBuf) -> Result<Self, String> {
        let _guard = directory_lock(&root)?;
        reserve(&root, 0)?;
        loop {
            let sequence = NEXT_SPOOL.fetch_add(1, Ordering::Relaxed);
            let path = root.join(format!(".pose-spool-{}-{sequence}.tmp", std::process::id()));
            match OpenOptions::new()
                .read(true)
                .write(true)
                .create_new(true)
                .open(&path)
            {
                Ok(file) => {
                    return Ok(Self {
                        file: Some(file),
                        path,
                        len: 0,
                    })
                }
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
                Err(e) => return Err(error(e)),
            }
        }
    }
    pub fn append(&mut self, stamp: u64, pose: [f64; 16]) -> Result<(), String> {
        if !pose.iter().all(|value| value.is_finite()) {
            return Err("最终位姿暂存含非有限矩阵".into());
        }
        let root = self.path.parent().ok_or("最终位姿暂存目录缺失")?;
        let _guard = directory_lock(root)?;
        reserve(root, POSE_BYTES as u64)?;
        let mut bytes = [0; POSE_BYTES];
        bytes[..8].copy_from_slice(&stamp.to_le_bytes());
        for (chunk, value) in bytes[8..].chunks_exact_mut(8).zip(pose) {
            chunk.copy_from_slice(&value.to_le_bytes());
        }
        let file = self.file.as_mut().ok_or("最终位姿暂存已关闭")?;
        let before = self.len * POSE_BYTES as u64;
        if let Err(e) = file.write_all(&bytes) {
            file.set_len(before).map_err(error)?;
            file.seek(std::io::SeekFrom::Start(before)).map_err(error)?;
            return Err(error(e));
        }
        self.len += 1;
        Ok(())
    }
    pub fn len(&self) -> u64 {
        self.len
    }
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
    pub fn bytes(&self) -> u64 {
        self.len * POSE_BYTES as u64
    }
    pub(super) fn path(&self) -> &Path {
        &self.path
    }
    pub(super) fn belongs_to(&self, root: &Path) -> bool {
        self.path().parent() == Some(root)
    }
    fn rewind(&mut self) -> Result<(), String> {
        let file = self.file.as_mut().ok_or("最终位姿暂存已关闭")?;
        if file.metadata().map_err(error)?.len() != self.len * POSE_BYTES as u64 {
            return Err("最终位姿暂存长度无效".into());
        }
        file.rewind().map_err(error)
    }
    fn read_pose(&mut self) -> Result<(u64, [f64; 16]), String> {
        let mut bytes = [0; POSE_BYTES];
        self.file
            .as_mut()
            .ok_or("最终位姿暂存已关闭")?
            .read_exact(&mut bytes)
            .map_err(error)?;
        let stamp = u64::from_le_bytes(bytes[..8].try_into().unwrap());
        let mut pose = [0.; 16];
        for (value, chunk) in pose.iter_mut().zip(bytes[8..].chunks_exact(8)) {
            *value = f64::from_le_bytes(chunk.try_into().unwrap());
        }
        if !pose.iter().all(|value| value.is_finite()) {
            return Err("最终位姿暂存含非有限矩阵".into());
        }
        Ok((stamp, pose))
    }
}
impl Drop for PoseSpool {
    fn drop(&mut self) {
        drop(self.file.take());
        let _ = std::fs::remove_file(&self.path);
    }
}

fn write_budgeted(file: &mut File, root: &Path, bytes: &[u8]) -> Result<(), String> {
    let _guard = directory_lock(root)?;
    reserve(root, bytes.len() as u64)?;
    file.write_all(bytes).map_err(error)
}

pub(super) fn save_spooled_final_poses(
    root: &Path,
    metadata: &FinalPoses,
    mut spool: PoseSpool,
) -> Result<PathBuf, String> {
    if !metadata.poses.is_empty() || !spool.belongs_to(root) {
        return Err("流式最终位姿必须使用同目录暂存及空位姿元数据".into());
    }
    spool.rewind()?;
    let stem = session_path(root, &metadata.session);
    let stem = stem.file_stem().unwrap().to_string_lossy();
    let path = root.join(format!("{stem}-poses-{}.json", metadata.cloud_revision));
    let temporary = path.with_extension("json.writing");
    let result = (|| {
        let mut file = {
            let _guard = directory_lock(root)?;
            reserve(root, 0)?;
            OpenOptions::new()
                .create(true)
                .truncate(true)
                .write(true)
                .open(&temporary)
                .map_err(error)?
        };
        // 空列表尾部固定为 []}；只改写列表内容，元数据仍由原结构序列化。
        let mut header = serde_json::to_vec(metadata).map_err(error)?;
        if !header.ends_with(b"[]}") {
            return Err("最终位姿元数据序列化结构无效".into());
        }
        header.truncate(header.len() - 2);
        write_budgeted(&mut file, root, &header)?;
        for index in 0..spool.len() {
            let pose = spool.read_pose()?;
            let mut bytes = serde_json::to_vec(&pose).map_err(error)?;
            if index != 0 {
                bytes.insert(0, b',');
            }
            write_budgeted(&mut file, root, &bytes)?;
        }
        write_budgeted(&mut file, root, b"]}")?;
        file.sync_all().map_err(error)?;
        std::fs::rename(&temporary, &path).map_err(error)?;
        File::open(root).map_err(error)?.sync_all().map_err(error)?;
        Ok(path.clone())
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(temporary);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    fn folder(name: &str) -> PathBuf {
        let root =
            std::env::temp_dir().join(format!("neh-pose-spool-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        root
    }
    fn metadata() -> FinalPoses {
        FinalPoses {
            session: Session {
                session_id: "流式\"位姿".into(),
                generation: u64::MAX,
            },
            cloud_revision: u64::MAX,
            frame_generation: Some(u64::MAX),
            poses: vec![],
        }
    }
    #[test]
    fn spooled_json_matches_existing_format_for_many_poses_and_empty_lists() {
        let root = folder("format");
        let mut expected = metadata();
        let mut spool = PoseSpool::new(root.clone()).unwrap();
        let temporary = spool.path().to_path_buf();
        for index in 0..1000 {
            let mut pose: [f64; 16] = nalgebra::Matrix4::<f64>::identity()
                .as_slice()
                .try_into()
                .unwrap();
            pose[12] = index as f64 * 0.125;
            pose[13] = -0.0;
            pose[14] = if index == 999 { f64::MAX } else { 1e-300 };
            let stamp = if index == 999 { u64::MAX } else { index };
            spool.append(stamp, pose).unwrap();
            expected.poses.push((stamp, pose));
        }
        assert_eq!(spool.len(), 1000);
        assert_eq!(spool.bytes(), 136_000);
        assert_eq!(temporary.metadata().unwrap().len(), spool.bytes());
        let path = save_spooled_final_poses(&root, &metadata(), spool).unwrap();
        assert!(!temporary.exists());
        let bytes = std::fs::read(path).unwrap();
        assert_eq!(bytes, serde_json::to_vec(&expected).unwrap());
        let actual: FinalPoses = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(actual.session, expected.session);
        assert_eq!(actual.cloud_revision, expected.cloud_revision);
        assert_eq!(actual.frame_generation, expected.frame_generation);
        assert_eq!(actual.poses, expected.poses);

        let mut empty_metadata = metadata();
        empty_metadata.cloud_revision = 0;
        empty_metadata.frame_generation = None;
        let empty = PoseSpool::new(root.clone()).unwrap();
        assert!(empty.is_empty());
        assert_eq!(empty.bytes(), 0);
        let temporary = empty.path().to_path_buf();
        let path = save_spooled_final_poses(&root, &empty_metadata, empty).unwrap();
        assert_eq!(
            std::fs::read(path).unwrap(),
            serde_json::to_vec(&empty_metadata).unwrap()
        );
        assert!(!temporary.exists());
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn cancellation_invalid_input_and_broken_spool_remove_owned_temporary_files() {
        let root = folder("cleanup");
        let mut spool = PoseSpool::new(root.clone()).unwrap();
        let temporary = spool.path().to_path_buf();
        assert!(spool.append(0, [f64::NAN; 16]).is_err());
        assert_eq!(spool.len(), 0);
        assert_eq!(temporary.metadata().unwrap().len(), 0);
        drop(spool);
        assert!(!temporary.exists());

        let mut spool = PoseSpool::new(root.clone()).unwrap();
        spool.append(10, [0.; 16]).unwrap();
        let temporary = spool.path().to_path_buf();
        spool.file.as_mut().unwrap().set_len(135).unwrap();
        assert!(save_spooled_final_poses(&root, &metadata(), spool)
            .unwrap_err()
            .contains("长度无效"));
        assert!(!temporary.exists());

        let spool = PoseSpool::new(root.clone()).unwrap();
        let temporary = spool.path().to_path_buf();
        let mut invalid_metadata = metadata();
        invalid_metadata.poses.push((0, [0.; 16]));
        assert!(save_spooled_final_poses(&root, &invalid_metadata, spool).is_err());
        assert!(!temporary.exists());
        std::fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn spool_and_streamed_json_both_obey_the_existing_directory_budget() {
        let root = folder("budget");
        let mut spool = PoseSpool::new(root.clone()).unwrap();
        spool.append(10, [0.; 16]).unwrap();
        let temporary = spool.path().to_path_buf();
        File::create(root.join("budget-blocker"))
            .unwrap()
            .set_len(storage_budget_bytes().unwrap() - spool.bytes())
            .unwrap();
        assert!(spool.append(20, [0.; 16]).unwrap_err().contains("预算"));
        assert_eq!(spool.len(), 1);
        assert_eq!(temporary.metadata().unwrap().len(), 136);
        assert!(save_spooled_final_poses(&root, &metadata(), spool)
            .unwrap_err()
            .contains("预算"));
        assert!(!temporary.exists());
        assert!(root.read_dir().unwrap().all(|entry| {
            let path = entry.unwrap().path();
            !path.to_string_lossy().ends_with(".json.writing")
                && path.extension().is_none_or(|extension| extension != "json")
        }));
        std::fs::remove_dir_all(root).unwrap();
    }
}

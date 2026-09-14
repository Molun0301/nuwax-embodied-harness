//! 完整图与精细三维资产共用有界分块传输，按真实格式校验后封存。
//! 未命名私有文件由描述符拥有，关闭即释放。
use crate::proto::{GraphArtifactRef, GraphArtifactTransferRequest, GraphArtifactTransferResponse};
use nuwax_slam_2d_runtime::archive;
use std::{
    collections::HashMap,
    fs::File,
    io::{Read, Seek, SeekFrom, Write},
    os::unix::fs::OpenOptionsExt,
    time::{Duration, Instant},
};

struct Entry {
    file: File,
    size: u64,
    sha: [u8; 32],
    written: u64,
    sealed: bool,
    touched: Instant,
}
#[derive(Default)]
pub(crate) struct Store {
    entries: HashMap<String, Entry>,
}
impl Store {
    fn collect(&mut self) {
        self.entries
            .retain(|_, e| e.touched.elapsed() < Duration::from_secs(600));
    }
    fn identity(reference: &GraphArtifactRef) -> Result<[u8; 32], String> {
        let sha: [u8; 32] = reference
            .sha256
            .as_slice()
            .try_into()
            .map_err(|_| "二维制品摘要尺寸错误")?;
        if reference.id.is_empty()
            || reference.id.len() > 128
            || reference.size == 0
            || reference.size > archive::MAX_ARCHIVE
            || sha == [0; 32]
        {
            return Err("二维制品身份或大小非法".into());
        }
        Ok(sha)
    }
    pub fn insert(
        &mut self,
        mut file: File,
        size: u64,
        sha: [u8; 32],
    ) -> Result<GraphArtifactRef, String> {
        self.collect();
        if self.entries.len() >= 4 {
            return Err("二维传输制品数量达到上限".into());
        }
        let reference = GraphArtifactRef {
            id: uuid::Uuid::new_v4().simple().to_string(),
            size,
            sha256: sha.to_vec(),
        };
        Self::identity(&reference)?;
        file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        if archive::hash_reader(&mut file)? != (size, sha) {
            return Err("二维制品导入摘要变化".into());
        }
        self.entries.insert(
            reference.id.clone(),
            Entry {
                file,
                size,
                sha,
                written: size,
                sealed: true,
                touched: Instant::now(),
            },
        );
        Ok(reference)
    }
    pub fn open(&mut self, reference: &GraphArtifactRef) -> Result<File, String> {
        self.collect();
        let e = self.entry(reference)?;
        if !e.sealed {
            return Err("二维上传制品未封存".into());
        }
        // 打开同一不可变inode的独立描述符，避免并行读取共享游标。
        File::open(format!(
            "/proc/self/fd/{}",
            std::os::fd::AsRawFd::as_raw_fd(&e.file)
        ))
        .map_err(|e| e.to_string())
    }
    fn entry(&mut self, r: &GraphArtifactRef) -> Result<&mut Entry, String> {
        let sha = Self::identity(r)?;
        let e = self
            .entries
            .get_mut(&r.id)
            .ok_or("二维制品不存在或已过期")?;
        if e.size != r.size || e.sha != sha {
            return Err("二维传输制品身份冲突".into());
        }
        e.touched = Instant::now();
        Ok(e)
    }
    pub fn transfer(
        &mut self,
        request: GraphArtifactTransferRequest,
    ) -> Result<GraphArtifactTransferResponse, String> {
        self.collect();
        let r = request.artifact.ok_or("缺少二维制品身份")?;
        Self::identity(&r)?;
        if request.data.len() > archive::CHUNK_BYTES {
            return Err("二维制品分块超过预算".into());
        }
        if request.action == 1 {
            if !request.data.is_empty() || request.offset != 0 {
                return Err("初始化不能携带制品数据".into());
            }
            // 上传方提供随机请求身份，使丢失初始化回执能够原请求重放。
            if !self.entries.contains_key(&r.id) {
                if self.entries.len() >= 4 {
                    return Err("二维传输制品数量达到上限".into());
                }
                let path = std::env::temp_dir().join(format!(
                    "nuwax-planar-transfer-{}",
                    uuid::Uuid::new_v4().simple()
                ));
                let file = std::fs::OpenOptions::new()
                    .read(true)
                    .write(true)
                    .create_new(true)
                    .mode(0o600)
                    .open(&path)
                    .map_err(|e| e.to_string())?;
                std::fs::remove_file(path).map_err(|e| e.to_string())?;
                self.entries.insert(
                    r.id.clone(),
                    Entry {
                        file,
                        size: r.size,
                        sha: Self::identity(&r)?,
                        written: 0,
                        sealed: false,
                        touched: Instant::now(),
                    },
                );
            }
        }
        if request.action == 5 {
            if self.entries.contains_key(&r.id) {
                self.entry(&r)?;
                self.entries.remove(&r.id);
            }
            return Ok(GraphArtifactTransferResponse {
                artifact: Some(r),
                ..Default::default()
            });
        }
        let e = self.entry(&r)?;
        let mut data = Vec::new();
        match request.action {
            1 => {}
            2 => {
                let end = request
                    .offset
                    .checked_add(request.data.len() as u64)
                    .ok_or("分块位置溢出")?;
                if request.data.is_empty() || end > e.size || request.offset > e.written {
                    return Err("二维上传分块位置非法".into());
                }
                e.file
                    .seek(SeekFrom::Start(request.offset))
                    .map_err(|e| e.to_string())?;
                if request.offset < e.written || e.sealed {
                    if end > e.written {
                        return Err("重复分块跨越已写入边界".into());
                    }
                    let mut previous = vec![0; request.data.len()];
                    e.file
                        .read_exact(&mut previous)
                        .map_err(|e| e.to_string())?;
                    if previous != request.data {
                        return Err("相同位置分块内容冲突".into());
                    }
                } else {
                    e.file.write_all(&request.data).map_err(|e| e.to_string())?;
                    e.written = end;
                }
            }
            3 => {
                if e.written != e.size {
                    return Err("二维上传尚未完整，不能封存".into());
                }
                if !e.sealed {
                    e.file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
                    if archive::hash_reader(&mut e.file)? != (e.size, e.sha) {
                        return Err("二维上传完整摘要不匹配".into());
                    }
                    e.file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
                    let mut magic = [0; 8];
                    e.file.read_exact(&mut magic).map_err(|e| e.to_string())?;
                    if &magic == b"NUWAXAR1" {
                        let mut fine = nuwax_map_chunks::ArchiveReader::open(&mut e.file)?;
                        for reference in fine.manifest().blocks.clone() { fine.block(&reference)?; }
                    } else {
                        archive::verify(&mut e.file)?;
                    }
                    e.file.sync_all().map_err(|e| e.to_string())?;
                    e.sealed = true;
                }
            }
            4 => {
                if !e.sealed || request.offset >= e.size || !request.data.is_empty() {
                    return Err("二维下载状态或位置非法".into());
                }
                data.resize(
                    (e.size - request.offset).min(archive::CHUNK_BYTES as u64) as usize,
                    0,
                );
                e.file
                    .seek(SeekFrom::Start(request.offset))
                    .map_err(|e| e.to_string())?;
                e.file.read_exact(&mut data).map_err(|e| e.to_string())?;
            }
            _ => return Err("未知二维制品操作".into()),
        }
        Ok(GraphArtifactTransferResponse {
            artifact: Some(r),
            offset: e.written,
            sealed: e.sealed,
            data,
        })
    }
}

#[derive(Debug)]
pub(crate) struct Target {
    pub file: std::sync::Arc<File>,
    pub size: u64,
    pub sha: [u8; 32],
    pub metadata: archive::Metadata,
}
impl PartialEq for Target {
    fn eq(&self, other: &Self) -> bool {
        self.size == other.size && self.sha == other.sha && self.metadata == other.metadata
    }
}
impl Target {
    pub fn open(&self) -> Result<File, String> {
        File::open(format!(
            "/proc/self/fd/{}",
            std::os::fd::AsRawFd::as_raw_fd(&*self.file)
        ))
        .map_err(|e| e.to_string())
    }
    pub fn receipt(
        &self,
        activation: [u8; 32],
        receipt: &crate::mapping_commit::OccupancyCommitReceipt,
    ) -> [u8; 32] {
        let mut data = b"nuwax.geometry.planar-target.v1\0".to_vec();
        data.extend_from_slice(&activation);
        data.extend_from_slice(&self.sha);
        data.extend_from_slice(&receipt.commit_id.boot_nonce.to_be_bytes());
        for n in [
            self.size,
            receipt.commit_id.sequence,
            receipt.session_generation,
            receipt.occupancy_revision,
            receipt.source_timestamp_ns,
            receipt.backend_generation,
        ] {
            data.extend_from_slice(&n.to_be_bytes());
        }
        nuwax_codec::hash::sha256_bytes(&data)
    }
}

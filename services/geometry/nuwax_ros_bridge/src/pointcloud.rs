use std::collections::HashMap;

use memmap2::MmapMut;
use tracing::{info, error};

pub const VOXEL_SIZE: f32 = 0.05;

// ─── 累积地图双缓冲共享内存写入器 ───

const CM_SHM_MAGIC: u64 = 0x4E55_5741_584D_4150; // "NUWAXMAP"
const CM_SHM_VERSION: u32 = 1;
const CM_HEADER_SIZE: usize = 64;
const CM_SINGLE_BUFFER_SIZE: usize = 16 * 1024 * 1024; // 16MB per buffer
const CM_TOTAL_SIZE: usize = CM_HEADER_SIZE + CM_SINGLE_BUFFER_SIZE * 2;

pub struct CloudMapShmWriter {
    mmap: MmapMut,
    revision: u64,
    voxel_size: f32,
}

impl CloudMapShmWriter {
    pub fn new(shm_path: &std::path::Path) -> std::io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .read(true).write(true).create(true).truncate(true)
            .open(shm_path)?;
        file.set_len(CM_TOTAL_SIZE as u64)?;
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        mmap[0..8].copy_from_slice(&CM_SHM_MAGIC.to_le_bytes());
        mmap[8..12].copy_from_slice(&CM_SHM_VERSION.to_le_bytes());
        mmap[12..CM_HEADER_SIZE].fill(0);
        info!("累积地图双缓冲 mmap 已创建: {:?} ({} bytes)", shm_path, CM_TOTAL_SIZE);
        Ok(Self { mmap, revision: 0, voxel_size: VOXEL_SIZE })
    }

    pub fn write_cloud_map(&mut self, data: &[u8], point_step: u32) -> Option<(u64, u32, f32)> {
        let ps = point_step as usize;
        if ps < 12 || data.is_empty() { return None; }
        let raw_count = data.len() / ps;
        let mut points: Vec<[f32; 3]> = Vec::with_capacity(raw_count);
        for i in 0..raw_count {
            let off = i * ps;
            if off + 12 > data.len() { break; }
            let x = f32::from_le_bytes(data[off..off+4].try_into().unwrap());
            let y = f32::from_le_bytes(data[off+4..off+8].try_into().unwrap());
            let z = f32::from_le_bytes(data[off+8..off+12].try_into().unwrap());
            if x.is_nan() || y.is_nan() || z.is_nan() { continue; }
            points.push([x, y, z]);
        }
        if points.is_empty() { return None; }
        let mut vs = self.voxel_size;
        let downsampled = loop {
            let ds = voxel_downsample(&points, vs);
            let byte_size = ds.len() * 12;
            if byte_size <= CM_SINGLE_BUFFER_SIZE || vs > 2.0 { break ds; }
            vs *= 2.0;
        };
        self.voxel_size = vs;
        let point_count = downsampled.len() as u32;
        let data_size = downsampled.len() * 12;
        self.revision += 1;
        let rev = self.revision;
        let active_buf = ((rev - 1) % 2) as usize;
        let buf_offset = CM_HEADER_SIZE + active_buf * CM_SINGLE_BUFFER_SIZE;
        for (i, p) in downsampled.iter().enumerate() {
            let off = buf_offset + i * 12;
            self.mmap[off..off+4].copy_from_slice(&p[0].to_le_bytes());
            self.mmap[off+4..off+8].copy_from_slice(&p[1].to_le_bytes());
            self.mmap[off+8..off+12].copy_from_slice(&p[2].to_le_bytes());
        }
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_nanos() as u64;
        self.mmap[12..16].copy_from_slice(&(active_buf as u32).to_le_bytes());
        self.mmap[16..20].copy_from_slice(&point_count.to_le_bytes());
        self.mmap[20..28].copy_from_slice(&now_ns.to_le_bytes());
        self.mmap[36..40].copy_from_slice(&vs.to_le_bytes());
        self.mmap[28..36].copy_from_slice(&rev.to_le_bytes());
        if let Err(e) = self.mmap.flush_range(0, buf_offset + data_size) {
            error!("累积地图 mmap flush 失败: {}", e);
        }
        Some((rev, point_count, vs))
    }
}

pub fn voxel_downsample(points: &[[f32; 3]], voxel_size: f32) -> Vec<[f32; 3]> {
    if points.is_empty() || voxel_size <= 0.0 {
        return points.to_vec();
    }
    let inv = 1.0 / voxel_size;
    let mut grid: HashMap<(i32, i32, i32), (f64, f64, f64, u32)> = HashMap::new();

    for p in points {
        let key = (
            (p[0] * inv).floor() as i32,
            (p[1] * inv).floor() as i32,
            (p[2] * inv).floor() as i32,
        );
        let entry = grid.entry(key).or_insert((0.0, 0.0, 0.0, 0));
        entry.0 += p[0] as f64;
        entry.1 += p[1] as f64;
        entry.2 += p[2] as f64;
        entry.3 += 1;
    }

    grid.values()
        .map(|(sx, sy, sz, n)| {
            let n = *n as f64;
            [(*sx / n) as f32, (*sy / n) as f32, (*sz / n) as f32]
        })
        .collect()
}

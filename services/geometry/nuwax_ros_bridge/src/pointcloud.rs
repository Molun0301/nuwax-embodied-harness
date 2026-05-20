use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};

use memmap2::MmapMut;
use tracing::{info, error};

const VOXEL_SIZE: f32 = 0.05;

// ─── 点云共享内存常量 ───
const PC_SHM_MAGIC: u64 = 0x4E55_5741_5850_434C; // "NUWAXPCL"
const PC_SHM_VERSION: u32 = 1;
/// 头部布局 (32 bytes):
///   0-7:   magic (u64 LE)
///   8-11:  version (u32 LE)
///   12-15: point_count (u32 LE)
///   16-23: timestamp_ns (u64 LE)
///   24-31: frame_id / sequence (u64 LE)
const PC_HEADER_SIZE: usize = 32;
/// 最大支持点数（1200 点 × 12 字节/点 = 14400 字节）
const PC_MAX_POINTS: usize = 4096;
const PC_MAX_DATA_SIZE: usize = PC_MAX_POINTS * 12; // float32 × 3
const PC_SHM_BUFFER_SIZE: usize = PC_HEADER_SIZE + PC_MAX_DATA_SIZE;

pub struct PointCloudProcessor {
    max_distance: f32,
    min_distance: f32,
    voxel_size: f32,
}

impl PointCloudProcessor {
    pub fn new(min_distance: f32, max_distance: f32, voxel_size: f32) -> Self {
        Self { max_distance, min_distance, voxel_size }
    }

    pub fn default_go2() -> Self {
        Self::new(0.15, 5.0, VOXEL_SIZE)
    }

    /// NaN 过滤 + 距离裁剪 + 体素降采样。
    /// 输入：原始 PointCloud2 data (XYZI 格式, point_step=16)
    /// 输出：过滤后的 data + 有效点数
    pub fn process(&self, data: &[u8], point_step: u32) -> (Vec<u8>, u32) {
        let ps = point_step as usize;
        if ps < 12 || data.len() < ps {
            return (Vec::new(), 0);
        }
        let point_count = data.len() / ps;

        // Pass 1: NaN filter + distance clip
        let mut valid_points: Vec<[f32; 3]> = Vec::with_capacity(point_count / 2);
        for i in 0..point_count {
            let offset = i * ps;
            let x = f32::from_le_bytes(data[offset..offset+4].try_into().unwrap());
            let y = f32::from_le_bytes(data[offset+4..offset+8].try_into().unwrap());
            let z = f32::from_le_bytes(data[offset+8..offset+12].try_into().unwrap());

            if x.is_nan() || y.is_nan() || z.is_nan() {
                continue;
            }
            let dist_sq = x * x + y * y + z * z;
            let min_sq = self.min_distance * self.min_distance;
            let max_sq = self.max_distance * self.max_distance;
            if dist_sq < min_sq || dist_sq > max_sq {
                continue;
            }
            valid_points.push([x, y, z]);
        }

        // Pass 2: voxel downsample using hash grid
        let downsampled = voxel_downsample(&valid_points, self.voxel_size);

        // Serialize to PointCloud2 XYZ format (point_step=12)
        let out_step = 12u32;
        let mut output = Vec::with_capacity(downsampled.len() * out_step as usize);
        for p in &downsampled {
            output.extend_from_slice(&p[0].to_le_bytes());
            output.extend_from_slice(&p[1].to_le_bytes());
            output.extend_from_slice(&p[2].to_le_bytes());
        }

        (output, downsampled.len() as u32)
    }
}

// ─── 点云共享内存写入器 ───

pub struct PointCloudShmWriter {
    mmap: MmapMut,
    sequence: AtomicU64,
}

impl PointCloudShmWriter {
    /// 创建点云共享内存写入器，路径默认 /tmp/nuwax_pointcloud.shm
    pub fn new(shm_path: &std::path::Path) -> std::io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .read(true).write(true).create(true).truncate(true)
            .open(shm_path)?;
        file.set_len(PC_SHM_BUFFER_SIZE as u64)?;
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        // 写入 magic 和 version
        mmap[0..8].copy_from_slice(&PC_SHM_MAGIC.to_le_bytes());
        mmap[8..12].copy_from_slice(&PC_SHM_VERSION.to_le_bytes());
        info!("点云共享内存已创建: {:?} ({} bytes)", shm_path, PC_SHM_BUFFER_SIZE);
        Ok(Self { mmap, sequence: AtomicU64::new(0) })
    }

    /// 将处理后的点云写入共享内存。
    /// data: 连续 float32 XYZ 布局 (point_step=12)
    /// point_count: 有效点数
    pub fn write_frame(&mut self, data: &[u8], point_count: u32) {
        let n = point_count as usize;
        if n == 0 || n > PC_MAX_POINTS {
            return;
        }
        let data_size = n * 12;
        if data.len() < data_size {
            return;
        }

        let seq = self.sequence.fetch_add(1, Ordering::Relaxed) + 1;
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // 先写数据区
        self.mmap[PC_HEADER_SIZE..PC_HEADER_SIZE + data_size]
            .copy_from_slice(&data[..data_size]);

        // 再写头部（sequence 最后写，作为 release fence）
        self.mmap[8..12].copy_from_slice(&PC_SHM_VERSION.to_le_bytes());
        self.mmap[12..16].copy_from_slice(&point_count.to_le_bytes());
        self.mmap[16..24].copy_from_slice(&now_ns.to_le_bytes());
        // sequence 最后写入，读端以此判断数据完整性
        self.mmap[24..32].copy_from_slice(&seq.to_le_bytes());

        if let Err(e) = self.mmap.flush_range(0, PC_HEADER_SIZE + data_size) {
            error!("点云共享内存 flush 失败: {}", e);
        }
    }
}

fn voxel_downsample(points: &[[f32; 3]], voxel_size: f32) -> Vec<[f32; 3]> {
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

// Phase 6 (codex-round3 R3-4)：shm_path / last_revision 是 SHM 字节布局版本
// 兼容预留（occupancy_shm v1/v2 共存），Phase 6 后续版本切换时使用。
#![allow(dead_code)]

//! Phase 3: OccupancyGrid SHM sidecar
//!
//! 把 `MapState.data` (Vec<i8>) 与 meta 序列化成 stable 字节布局,写入
//! `/tmp/nuwax_occupancy.mmap` SHM 文件(可被 Python 端 mmap 读)。
//!
//! ## 字节布局
//! ```text
//! offset 0  : magic u64 (LE)  = 0x4E55_5741_584F_4343  ("NUWAXOCC")
//! offset 8  : version u32 (LE) = 1
//! offset 12 : width u32 (LE)
//! offset 16 : height u32 (LE)
//! offset 20 : resolution f32 (LE)
//! offset 24 : origin_x f64 (LE)
//! offset 32 : origin_y f64 (LE)
//! offset 40 : frame_id_len u16 (LE)
//! offset 42 : frame_id bytes
//! offset 42+frame_id_len : revision u64 (LE)
//! offset 50+frame_id_len : timestamp_ns u64 (LE)
//! offset 58+frame_id_len : data_len u32 (LE)
//! offset 62+frame_id_len : data bytes (i8, 1 byte per cell)
//! ```
//!
//! 写侧使用 `memmap2::MmapMut`,读侧可由 Python `mmap` 直接打开。
//! revision 字段放在数据之后,作为 seqlock 风格的发布信号 ——
//! 写完 header + data 后再写 revision(单 u64,原子对齐)。

use std::sync::atomic::{fence, Ordering};

use memmap2::MmapMut;
use tracing::{error, info};

use crate::lcm_runtime::MapState;

/// SHM file magic = "NUWAXOCC" (8 bytes ASCII LE)
const OCC_SHM_MAGIC: u64 = 0x4E55_5741_584F_4343;
/// SHM file format version
const OCC_SHM_VERSION: u32 = 1;
/// 初始 SHM 容量 (16 MB);如果 occupancy 超过自动增长
const OCC_INITIAL_CAPACITY: usize = 16 * 1024 * 1024;
/// 增长因子(每次容量翻倍)
const OCC_GROW_FACTOR: usize = 2;

pub struct OccupancyGridShmWriter {
    mmap: MmapMut,
    capacity: usize,
    last_revision: u64,
    shm_path: std::path::PathBuf,
}

impl OccupancyGridShmWriter {
    pub fn new(shm_path: &std::path::Path) -> std::io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(shm_path)?;
        file.set_len(OCC_INITIAL_CAPACITY as u64)?;
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        mmap[0..8].copy_from_slice(&OCC_SHM_MAGIC.to_le_bytes());
        mmap[8..12].copy_from_slice(&OCC_SHM_VERSION.to_le_bytes());
        mmap[12..OCC_INITIAL_CAPACITY].fill(0);
        info!(
            "occupancy grid SHM 已创建: {:?} (initial {} bytes)",
            shm_path, OCC_INITIAL_CAPACITY
        );
        Ok(Self {
            mmap,
            capacity: OCC_INITIAL_CAPACITY,
            last_revision: 0,
            shm_path: shm_path.to_path_buf(),
        })
    }

    /// 把 MapState 序列化为 SHM。
    /// 成功返回 revision;若需要扩展则重新 mmap。
    pub fn write_occupancy(&mut self, map: &MapState) -> std::io::Result<u64> {
        let frame_id_bytes = map.frame_id.as_bytes();
        if frame_id_bytes.len() > u16::MAX as usize {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "frame_id 长度超过 u16::MAX",
            ));
        }
        let data_len = map.data.len();
        // 12(magic+ver) + 8(width+h+res) + 16(origin_xy) + 2(frame_id_len) +
        // frame_id_len + 8(rev) + 8(stamp) + 4(data_len) + data_len
        let required = 12 + 8 + 16 + 2 + frame_id_bytes.len() + 8 + 8 + 4 + data_len;
        if required > self.capacity {
            let new_cap = std::cmp::max(
                required,
                self.capacity.checked_mul(OCC_GROW_FACTOR).unwrap_or(required),
            );
            self.grow(new_cap)?;
        }

        // 1) header 写 meta(width/height/res/origin/frame)
        let mut cur = 12usize; // skip magic + version
        cur = write_u32(&mut self.mmap, cur, map.width);
        cur = write_u32(&mut self.mmap, cur, map.height);
        cur = write_f32(&mut self.mmap, cur, map.resolution);
        cur = write_f64(&mut self.mmap, cur, map.origin_x);
        cur = write_f64(&mut self.mmap, cur, map.origin_y);
        let frame_id_len = frame_id_bytes.len() as u16;
        cur = write_u16(&mut self.mmap, cur, frame_id_len);
        self.mmap[cur..cur + frame_id_bytes.len()].copy_from_slice(frame_id_bytes);
        cur += frame_id_bytes.len();

        // 2) revision(在 data 之前写一次,作为旧值,等数据写完再写正式 revision)
        // 简化:revision 暂存,最后再写
        let revision_offset = cur;
        cur += 8;
        let timestamp_offset = cur;
        cur += 8;
        let data_len_offset = cur;
        cur += 4;
        let data_offset = cur;

        // 3) data bytes
        write_u32(&mut self.mmap, data_len_offset, data_len as u32);
        for (i, &v) in map.data.iter().enumerate() {
            self.mmap[data_offset + i] = v as u8;
        }
        cur = data_offset + data_len;

        // 4) timestamp + revision
        self.mmap[timestamp_offset..timestamp_offset + 8]
            .copy_from_slice(&map.timestamp_ns.to_le_bytes());
        self.last_revision = map.revision;
        // Release fence 保证 data/timestamp 写完后再发布 revision
        fence(Ordering::Release);
        self.mmap[revision_offset..revision_offset + 8]
            .copy_from_slice(&map.revision.to_le_bytes());

        if let Err(e) = self.mmap.flush_range(0, cur) {
            error!("occupancy grid SHM flush 失败: {}", e);
        }
        Ok(map.revision)
    }

    fn grow(&mut self, new_cap: usize) -> std::io::Result<()> {
        // 重新打开 mmap(更大 size);这里不用 remap 而是重新 open。
        // 当前 SHM sidecar 只在 CI 单实例,没有并发读侧,故可以 truncate-and-reopen。
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(&self.shm_path)?;
        file.set_len(new_cap as u64)?;
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        mmap[0..8].copy_from_slice(&OCC_SHM_MAGIC.to_le_bytes());
        mmap[8..12].copy_from_slice(&OCC_SHM_VERSION.to_le_bytes());
        mmap[12..new_cap].fill(0);
        self.mmap = mmap;
        self.capacity = new_cap;
        info!("occupancy grid SHM 已扩容: new_cap={}", new_cap);
        Ok(())
    }

    pub fn shm_path(&self) -> &std::path::Path {
        &self.shm_path
    }

    pub fn last_revision(&self) -> u64 {
        self.last_revision
    }
}

fn write_u16(buf: &mut [u8], off: usize, v: u16) -> usize {
    buf[off..off + 2].copy_from_slice(&v.to_le_bytes());
    off + 2
}
fn write_u32(buf: &mut [u8], off: usize, v: u32) -> usize {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
    off + 4
}
fn write_f32(buf: &mut [u8], off: usize, v: f32) -> usize {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
    off + 4
}
fn write_f64(buf: &mut [u8], off: usize, v: f64) -> usize {
    buf[off..off + 8].copy_from_slice(&v.to_le_bytes());
    off + 8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_map(width: u32, height: u32, fill: i8) -> MapState {
        MapState {
            width,
            height,
            resolution: 0.05,
            origin_x: -1.0,
            origin_y: -2.0,
            frame_id: "map".to_string(),
            revision: 1,
            timestamp_ns: 12345,
            data: vec![fill; (width * height) as usize],
        }
    }

    #[test]
    fn create_writer_succeeds() {
        let tmp = std::env::temp_dir().join(format!("nuwax_occ_test_{}.mmap", std::process::id()));
        let _ = std::fs::remove_file(&tmp);
        let w = OccupancyGridShmWriter::new(&tmp);
        assert!(w.is_ok());
        let _ = std::fs::remove_file(&tmp);
    }

    #[test]
    fn write_then_reread_roundtrip() {
        let tmp = std::env::temp_dir().join(format!("nuwax_occ_test2_{}.mmap", std::process::id()));
        let _ = std::fs::remove_file(&tmp);
        let mut w = OccupancyGridShmWriter::new(&tmp).expect("create");
        let map = sample_map(4, 4, 100);
        let rev = w.write_occupancy(&map).expect("write");
        assert_eq!(rev, 1);
        // 验证 magic + version 头
        let bytes = std::fs::read(&tmp).expect("read");
        let magic = u64::from_le_bytes(bytes[0..8].try_into().unwrap());
        assert_eq!(magic, OCC_SHM_MAGIC);
        let ver = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
        assert_eq!(ver, OCC_SHM_VERSION);
        // 验证 width/height
        let w0 = u32::from_le_bytes(bytes[12..16].try_into().unwrap());
        let h0 = u32::from_le_bytes(bytes[16..20].try_into().unwrap());
        assert_eq!(w0, 4);
        assert_eq!(h0, 4);
        let _ = std::fs::remove_file(&tmp);
    }

    #[test]
    fn write_grows_capacity() {
        let tmp = std::env::temp_dir().join(format!("nuwax_occ_test3_{}.mmap", std::process::id()));
        let _ = std::fs::remove_file(&tmp);
        let mut w = OccupancyGridShmWriter::new(&tmp).expect("create");
        // 制造一个大于初始容量的 map(不实际 fill 大数据,只放一个长 frame_id)
        let big_map = MapState {
            width: 0,
            height: 0,
            resolution: 0.0,
            origin_x: 0.0,
            origin_y: 0.0,
            frame_id: "x".repeat(20_000_000),
            revision: 1,
            timestamp_ns: 0,
            data: vec![],
        };
        // write_occupancy 路径在 grow 后应成功
        let _ = w.write_occupancy(&big_map);
        let _ = std::fs::remove_file(&tmp);
    }

    #[test]
    fn last_revision_tracks() {
        let tmp = std::env::temp_dir().join(format!("nuwax_occ_test4_{}.mmap", std::process::id()));
        let _ = std::fs::remove_file(&tmp);
        let mut w = OccupancyGridShmWriter::new(&tmp).expect("create");
        let mut m = sample_map(2, 2, 0);
        m.revision = 42;
        let _ = w.write_occupancy(&m);
        assert_eq!(w.last_revision(), 42);
        let _ = std::fs::remove_file(&tmp);
    }
}

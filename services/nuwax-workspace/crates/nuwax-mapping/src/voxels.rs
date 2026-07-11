//! 体素地图
//!
//! ARCH-DEVIATION: see `docs/开发文档/建图导航探索重构/补丁文档/2026-07-09_移植问题修复/
//! 20_rework20_全仓审计_对dimos原始.md` §2.1 H-7。
//!
//! Python SSOT：`/home/ml/Deeprob/dimos/dimos/mapping/voxels.py`
//!   - 用 Open3D `o3d.t.geometry.VoxelBlockGrid`（CUDA 加速）
//!   - `block_count=2_000_000` + `block_resolution=1`
//!   - `_carve_and_insert()` 列雕刻（删除同 (X,Y) 已存在 voxel 再 insert）
//!   - 每帧结束 `o3c.cuda.release_cache()`（防止 VRAM 累积 ~0.8 MB/call）
//!
//! Rust 当前实现：
//!   - 自研 `AHashMap<VoxelKey, Vec<u8>>` + `VOXEL_BLOCK_SIZE=8`（**完全偏离** Python）
//!   - 缺 `_carve_and_insert()` 与 `release_cache()`
//!
//! 本期（修复 H-7）**只做骨架兼容**：补 `_carve_and_insert()` API stub + 调用点
//! 骨架（默认 noop，便于上层接入点对齐）；**不重写** 自研 hash map（那是 P2 工作）。
//! 设计目标：）。

use ahash::AHashMap;
use nalgebra::{Isometry3, Point3, Vector3};
use serde::{Deserialize, Serialize};

/// 单个体素块的边长（cells per dimension）
pub const VOXEL_BLOCK_SIZE: usize = 8;

/// 体素块（8x8x8 cells，存 occupancy 值）
///
/// 注意：原计划用 `arrayvec::ArrayVec<u8, 512>` 固定大小，但 `ArrayVec::from([0u8; 512])`
/// 在 arrayvec 0.7 上会触发容量边界（已满），且 `block[idx] = v` 要求 `idx < len()`。
/// 这里改用 `Vec<u8>`，实现等价且编译更稳定；如需零分配可后续换 `Box<[u8; 512]>`。
pub type VoxelBlock = Vec<u8>;

/// 体素 key（基于 block 索引）
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VoxelKey {
    /// 块 X 索引
    pub ix: i64,
    /// 块 Y 索引
    pub iy: i64,
    /// 块 Z 索引
    pub iz: i64,
}

impl VoxelKey {
    /// 从 world 坐标构造 voxel key（已知 voxel size）
    #[must_use]
    pub fn from_world(p: Point3<f64>, voxel_size: f64) -> Self {
        Self {
            ix: (p.x / voxel_size / VOXEL_BLOCK_SIZE as f64).floor() as i64,
            iy: (p.y / voxel_size / VOXEL_BLOCK_SIZE as f64).floor() as i64,
            iz: (p.z / voxel_size / VOXEL_BLOCK_SIZE as f64).floor() as i64,
        }
    }

    /// 块在 world 坐标系下的中心点
    #[must_use]
    pub fn center(&self, voxel_size: f64) -> Point3<f64> {
        let block_world_size = voxel_size * VOXEL_BLOCK_SIZE as f64;
        Point3::new(
            (self.ix as f64 + 0.5) * block_world_size,
            (self.iy as f64 + 0.5) * block_world_size,
            (self.iz as f64 + 0.5) * block_world_size,
        )
    }
}

/// 体素 hash map
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VoxelHashMap {
    /// key → 体素块
    blocks: AHashMap<VoxelKey, VoxelBlock>,
    /// voxel 大小（m）
    pub voxel_size: f64,
}

impl VoxelHashMap {
    /// 构造新 voxel hash map
    #[must_use]
    pub fn new(voxel_size: f64) -> Self {
        Self {
            blocks: AHashMap::new(),
            voxel_size,
        }
    }

    /// 插入一个 point + 占据值
    pub fn insert_point(&mut self, p: Point3<f64>, occupancy: u8) {
        let key = VoxelKey::from_world(p, self.voxel_size);
        let local = self.world_to_local(p);
        let idx =
            local.0 * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE + local.1 * VOXEL_BLOCK_SIZE + local.2;
        let block = self
            .blocks
            .entry(key)
            .or_insert_with(|| vec![0u8; VOXEL_BLOCK_SIZE.pow(3)]);
        if idx < block.len() {
            block[idx] = occupancy;
        }
    }

    /// 插入多个点（并行，rayon）
    pub fn insert_points_par<I: IntoIterator<Item = (Point3<f64>, u8)>>(&mut self, points: I) {
        // 简单串行版本（避免借用冲突；真实并行用 chunk-and-merge）
        for (p, occ) in points {
            self.insert_point(p, occ);
        }
    }

    /// 插入一批 voxel keys（带列雕刻）
    ///
    /// 对应 Python `VoxelGrid._carve_and_insert(new_keys)`（dimos/mapping/voxels.py:120-153）：
    ///   1. 删除同 `(ix, iy)` 已存在 block（不论 iz）—— "列雕刻"
    ///   2. 真正 insert 新键对应的 occupancy（每点写回原 occupancy 值）
    ///
    /// 本期（rework22 P0-09.2）**真激活**新 voxel keys：原骨架版只删不 insert，
    /// 调用方需自己重传 points 再调 `insert_points_par`，实际使用中容易漏。
    /// 现在 `carve_and_insert` 自己消费 points 并完整执行两步。
    ///
    /// 实现开销：O(N) extra hashmap 扫描 + O(N) insert；GPU 版可降到 O(N)（P2）。
    pub fn carve_and_insert<I: IntoIterator<Item = (Point3<f64>, u8)>>(&mut self, points: I) {
        // Step 1：物化 points + 收集新 key（points 是 IntoIterator，可能是一次性的）
        let points: Vec<(Point3<f64>, u8)> = points.into_iter().collect();
        let new_keys: Vec<VoxelKey> = points
            .iter()
            .map(|(p, _)| VoxelKey::from_world(*p, self.voxel_size))
            .collect();

        // Step 2：列雕刻 —— 删除同 (ix, iy) 已存在 block
        let carve_xy: ahash::AHashSet<(i64, i64)> =
            new_keys.iter().map(|k| (k.ix, k.iy)).collect();
        self.blocks.retain(|k, _| !carve_xy.contains(&(k.ix, k.iy)));

        // Step 3：真正 insert 新键（直接走 insert_point 路径，保证 (ix,iy,iz) 落点正确）
        for (p, occ) in points {
            self.insert_point(p, occ);
        }
    }

    /// 释放 CUDA 缓存（对应 Python `o3c.cuda.release_cache()`）
    ///
    /// ARCH-DEVIATION：Rust 当前 CPU hash map 不持有 CUDA 缓存；本期仅保留调用点，
    /// 便于上层接入与审计文档对齐。P2 替换为 Open3D / cust 后再实际执行 CUDA free。
    pub fn release_cache(&mut self) {
        // 当前实现 noop：CPU 端无 CUDA 缓存可释放。
        // P2 follow-up：替换为 cust / Open3D 后，调用 cudarc::driver::result::free。
        let _ = &mut self.blocks;
    }

    /// 查询某点最近 block 的占据值
    #[must_use]
    pub fn query(&self, p: Point3<f64>) -> Option<u8> {
        let key = VoxelKey::from_world(p, self.voxel_size);
        let block = self.blocks.get(&key)?;
        let local = self.world_to_local(p);
        let idx =
            local.0 * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE + local.1 * VOXEL_BLOCK_SIZE + local.2;
        block.get(idx).copied()
    }

    /// ray cast：从 origin 沿 direction 投射直到命中第一个占据 voxel 或到达 max_distance
    #[must_use]
    pub fn raycast(
        &self,
        origin: Point3<f64>,
        direction: Vector3<f64>,
        max_distance: f64,
        occupied_threshold: u8,
    ) -> Option<(Point3<f64>, f64)> {
        let dir = direction.normalize();
        let mut t = 0.0;
        let step = self.voxel_size * 0.5;
        while t < max_distance {
            let p = origin + dir * t;
            if let Some(occ) = self.query(p) {
                if occ >= occupied_threshold {
                    return Some((p, t));
                }
            }
            t += step;
        }
        None
    }

    /// block 计数
    #[must_use]
    pub fn block_count(&self) -> usize {
        self.blocks.len()
    }

    /// 起源位姿（可选，用于 raycast 时变换）
    pub fn with_origin(self, _origin: Isometry3<f64>) -> Self {
        // 当前实现假设 origin = identity；扩展点
        self
    }

    /// world → block 内 local 索引（返回 (cx, cy, cz) ∈ [0, 8)^3）
    fn world_to_local(&self, p: Point3<f64>) -> (usize, usize, usize) {
        let block_world_size = self.voxel_size * VOXEL_BLOCK_SIZE as f64;
        let fx = (p.x / block_world_size).fract();
        let fy = (p.y / block_world_size).fract();
        let fz = (p.z / block_world_size).fract();
        // fract() ∈ [-1, 1)，clamp 到 [0, 1)
        let clamp = |v: f64| -> usize {
            let v = if v < 0.0 { v + 1.0 } else { v };
            ((v * VOXEL_BLOCK_SIZE as f64).floor() as usize).min(VOXEL_BLOCK_SIZE - 1)
        };
        (clamp(fx), clamp(fy), clamp(fz))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn voxel_key_from_world() {
        let p = nalgebra::point![0.1, 0.1, 0.1];
        let k = VoxelKey::from_world(p, 0.05);
        // (0.1 / 0.05 / 8).floor() = 0
        assert_eq!(k.ix, 0);
        assert_eq!(k.iy, 0);
        assert_eq!(k.iz, 0);
    }

    #[test]
    fn insert_and_query() {
        let mut m = VoxelHashMap::new(0.1);
        let p = nalgebra::point![0.05, 0.05, 0.05];
        m.insert_point(p, 100);
        assert_eq!(m.query(p), Some(100));
    }

    #[test]
    fn raycast_hit() {
        let mut m = VoxelHashMap::new(0.1);
        // 在 (1, 0, 0) 放置占据
        m.insert_point(nalgebra::point![1.0, 0.0, 0.0], 100);

        let origin = nalgebra::point![0.0, 0.0, 0.0];
        let dir = Vector3::new(1.0, 0.0, 0.0);
        let hit = m.raycast(origin, dir, 5.0, 50);
        assert!(hit.is_some());
        let (_, dist) = hit.unwrap();
        assert!(dist > 0.0 && dist < 5.0);
    }

    #[test]
    fn raycast_miss() {
        let m = VoxelHashMap::new(0.1);
        let origin = nalgebra::point![0.0, 0.0, 0.0];
        let dir = Vector3::new(1.0, 0.0, 0.0);
        let hit = m.raycast(origin, dir, 5.0, 50);
        assert!(hit.is_none());
    }

    #[test]
    fn block_count_grows() {
        let mut m = VoxelHashMap::new(0.1);
        m.insert_point(nalgebra::point![0.05, 0.05, 0.05], 100);
        m.insert_point(nalgebra::point![100.0, 0.0, 0.0], 100); // 不同块
        assert!(m.block_count() >= 2);
    }
}

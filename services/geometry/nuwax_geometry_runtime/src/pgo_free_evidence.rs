//! 供私有流式重放使用的有限自由增量与已保留原点相关性证据。
use nalgebra::Point3;
use nuwax_dufomap_native::FreeCell;
use std::{collections::VecDeque, sync::Arc};

pub(crate) const FREE_CHANGE_PAYLOAD_BYTES: usize = 2 * 1024 * 1024;
pub(crate) const FREE_CHANGE_MAX_REVISIONS: usize = 64;
pub(crate) const PROCESSED_EVIDENCE_BYTES: usize = 8 * 1024 * 1024;
const CELL_BYTES: usize = std::mem::size_of::<FreeCell>();
const BLOOM_HASHES: u64 = 4;
const BLOOM_BIT_MASK: u64 = (PROCESSED_EVIDENCE_BYTES * 8 - 1) as u64;

/// 空间证据变化日志；克隆只复制有限目录并共享不可变单次增量载荷。
#[derive(Clone, Debug)]
pub(crate) struct FreeChangeJournal {
    entries: VecDeque<(u64, Arc<[FreeCell]>)>,
    payload_bytes: usize,
    latest: u64,
    missing_through: u64,
}
impl Default for FreeChangeJournal {
    fn default() -> Self {
        Self {
            entries: VecDeque::with_capacity(FREE_CHANGE_MAX_REVISIONS),
            payload_bytes: 0,
            latest: 0,
            missing_through: 0,
        }
    }
}
impl FreeChangeJournal {
    #[cfg(test)]
    pub(crate) fn payload_stats(&self) -> (usize, usize) {
        (self.payload_bytes, self.entries.len())
    }

    pub(crate) fn record(&mut self, revision: u64, cells: &[FreeCell]) {
        if cells.is_empty() {
            return;
        }
        if self.latest.checked_add(1) != Some(revision) {
            // 不认识的旧代码/跳跃修订只保证这一条及后续连续变化。
            self.entries.clear();
            self.payload_bytes = 0;
            self.missing_through = revision.saturating_sub(1);
        }
        self.latest = revision;
        if cells.len() > FREE_CHANGE_PAYLOAD_BYTES / CELL_BYTES {
            self.entries.clear();
            self.payload_bytes = 0;
            self.missing_through = revision;
            return;
        }
        let incoming_bytes = cells.len() * CELL_BYTES;
        while self.entries.len() >= FREE_CHANGE_MAX_REVISIONS
            || self.payload_bytes + incoming_bytes > FREE_CHANGE_PAYLOAD_BYTES
        {
            let (revision, cells) = self.entries.pop_front().expect("超载前必有可淘汰载荷");
            self.payload_bytes -= cells.len() * CELL_BYTES;
            self.missing_through = self.missing_through.max(revision);
        }
        self.entries.push_back((revision, Arc::from(cells)));
        self.payload_bytes += incoming_bytes;
    }

    /// 返回严格位于 after 之后的完整增量；未知旧修订、缺口和未来修订均保守失败。
    pub(crate) fn since(&self, after: u64, current: u64) -> Option<Vec<FreeCell>> {
        if after > current {
            return None;
        }
        if after == current {
            return Some(Vec::new());
        }
        if self.latest != current || after < self.missing_through {
            return None;
        }
        let cells = self
            .entries
            .iter()
            .filter(|(revision, _)| *revision > after);
        let count: usize = cells.clone().map(|(_, cells)| cells.len()).sum();
        let mut out = Vec::with_capacity(count);
        for (_, cells) in cells {
            out.extend_from_slice(cells);
        }
        Some(out)
    }
}

/// 固定大小布隆过滤器；只收录已重放并保留的冻结里程计原点叶子键。
/// 散列碰撞只会要求额外重放，不会漏掉同一键的真实自由变化。
pub(crate) struct ProcessedEvidence {
    words: Box<[u64]>,
    resolution: f64,
    unknown_point: bool,
}
impl Default for ProcessedEvidence {
    fn default() -> Self {
        Self::new()
    }
}
impl ProcessedEvidence {
    pub(crate) fn new() -> Self {
        Self {
            words: vec![0; PROCESSED_EVIDENCE_BYTES / std::mem::size_of::<u64>()]
                .into_boxed_slice(),
            resolution: super::online_params().resolution,
            unknown_point: false,
        }
    }

    pub(crate) fn memory_bytes(&self) -> usize {
        self.words.len() * std::mem::size_of::<u64>() + std::mem::size_of::<Self>()
    }

    fn leaf_key(&self, point: Point3<f64>) -> Option<[i32; 3]> {
        let mut key = [0; 3];
        for (axis, value) in point.coords.iter().enumerate() {
            // 原生 coord_t 为 float、node_size_t 为 double；必须保留这次降精度。
            let value = (f64::from(*value as f32) / self.resolution).floor();
            if !value.is_finite() || value < f64::from(i32::MIN) || value > f64::from(i32::MAX) {
                return None;
            }
            key[axis] = value as i32;
        }
        Some(key)
    }

    pub(crate) fn insert_point(&mut self, point: Point3<f64>) {
        let Some(key) = self.leaf_key(point) else {
            self.unknown_point = true;
            return;
        };
        for bit in bloom_bits(key) {
            self.words[(bit / 64) as usize] |= 1 << (bit % 64);
        }
    }

    fn may_contain_key(&self, key: [i32; 3]) -> bool {
        bloom_bits(key).all(|bit| self.words[(bit / 64) as usize] & (1 << (bit % 64)) != 0)
    }

    pub(crate) fn may_intersect(&self, cells: &[FreeCell]) -> bool {
        if cells.is_empty() {
            return false;
        }
        if self.unknown_point {
            return true;
        }
        for cell in cells {
            if cell.depth > 1 {
                return true;
            }
            let side = 1i32 << cell.depth;
            for x in 0..side {
                for y in 0..side {
                    for z in 0..side {
                        let offset = [x, y, z];
                        let mut key = [0; 3];
                        for axis in 0..3 {
                            let Some(value) = cell.key[axis].checked_add(offset[axis]) else {
                                return true;
                            };
                            key[axis] = value;
                        }
                        if self.may_contain_key(key) {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }
}

fn mix64(mut value: u64) -> u64 {
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn bloom_bits(key: [i32; 3]) -> impl Iterator<Item = u64> {
    let [x, y, z] = key.map(|v| u64::from(v as u32));
    let packed = mix64(x ^ 0x517c_c1b7_2722_0a95)
        ^ mix64(y ^ 0x6eed_0e9d_a4d9_4a4f).rotate_left(21)
        ^ mix64(z ^ 0x9e37_79b9_7f4a_7c15).rotate_left(42);
    let a = mix64(packed);
    let b = mix64(packed ^ 0xd6e8_feb8_6659_fd93) | 1;
    (0..BLOOM_HASHES).map(move |index| a.wrapping_add(index.wrapping_mul(b)) & BLOOM_BIT_MASK)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamic_backend::OnlineDufo;

    fn cell(key: [i32; 3], depth: u32) -> FreeCell {
        FreeCell { key, depth }
    }

    #[test]
    fn bloom_is_fixed_eight_mib_and_never_misses_signed_f32_leaf_or_parent() {
        let mut bloom = ProcessedEvidence::new();
        let size = bloom.memory_bytes();
        assert_eq!(
            size,
            PROCESSED_EVIDENCE_BYTES + std::mem::size_of::<ProcessedEvidence>()
        );
        assert!(!bloom.may_intersect(&[cell([0; 3], 0)]));
        let r = super::super::online_params().resolution;
        // 0.15 的 f32 值位于双精度边界另一侧；-0.15 的符号方向也必须正确。
        for value in [
            0.,
            -0.,
            0.15,
            -0.15,
            0.3,
            -0.3,
            1.5,
            -1.5,
            f64::from(f32::from_bits(0.15f32.to_bits() - 1)),
            f64::from(f32::from_bits(0.15f32.to_bits() + 1)),
            f64::from(f32::from_bits((-0.15f32).to_bits() - 1)),
            f64::from(f32::from_bits((-0.15f32).to_bits() + 1)),
        ] {
            let point = Point3::new(value, -value, value / 2.);
            let key = [point.x, point.y, point.z].map(|v| (f64::from(v as f32) / r).floor() as i32);
            bloom.insert_point(point);
            assert_eq!(bloom.leaf_key(point), Some(key));
            assert!(bloom.may_intersect(&[cell(key, 0)]));
            assert!(bloom.may_intersect(&[cell(key.map(|v| v.div_euclid(2) * 2), 1)]));
        }
        assert_eq!(
            bloom.leaf_key(Point3::new(-0.15, 0.15, 0.)),
            Some([-2, 1, 0])
        );
        for x in -32..=32 {
            for y in -8..=8 {
                for z in -2..=2 {
                    let key = [x, y, z];
                    let point = Point3::from(key.map(|v| (f64::from(v) + 0.5) * r));
                    bloom.insert_point(point);
                    assert!(bloom.may_intersect(&[cell(key, 0)]), "已插入叶子不得漏检");
                    assert!(bloom.may_intersect(&[cell(key.map(|v| v.div_euclid(2) * 2), 1)]));
                }
            }
        }
        assert!(bloom.may_intersect(&[cell([i32::MAX; 3], 1)]));
        assert!(bloom.may_intersect(&[cell([0; 3], 2)]));
        assert_eq!(bloom.memory_bytes(), size);
        bloom.insert_point(Point3::new(f64::INFINITY, 0., 0.));
        assert!(bloom.may_intersect(&[cell([12345; 3], 0)]));
        assert!(!bloom.may_intersect(&[]));
    }

    #[test]
    fn journal_bounds_gaps_eviction_oversize_and_shared_clone_continuation() {
        let mut journal = FreeChangeJournal::default();
        assert_eq!(journal.since(0, 0), Some(vec![]));
        assert!(journal.since(0, 1).is_none(), "旧代码未知修订必须保守拒绝");
        for revision in 1..=65 {
            journal.record(revision, &[cell([revision as i32, 0, 0], 0)]);
        }
        assert_eq!(journal.entries.len(), FREE_CHANGE_MAX_REVISIONS);
        assert!(journal.since(0, 65).is_none());
        assert_eq!(journal.since(1, 65).unwrap().len(), 64);
        let mut fork = journal.clone();
        assert!(Arc::ptr_eq(&journal.entries[0].1, &fork.entries[0].1));
        fork.record(66, &[cell([66, 0, 0], 0)]);
        assert_eq!(journal.latest, 65);
        assert_eq!(fork.latest, 66);
        assert_eq!(journal.entries.len(), 64);
        let half = vec![cell([7; 3], 0); FREE_CHANGE_PAYLOAD_BYTES / CELL_BYTES / 2];
        journal.record(66, &half);
        journal.record(67, &half);
        assert_eq!(journal.payload_bytes, FREE_CHANGE_PAYLOAD_BYTES);
        assert!(journal.since(64, 67).is_none());
        assert_eq!(journal.since(65, 67).unwrap().len(), half.len() * 2);
        let oversized = vec![cell([8; 3], 0); FREE_CHANGE_PAYLOAD_BYTES / CELL_BYTES + 1];
        journal.record(68, &oversized);
        assert!(journal.entries.is_empty());
        assert_eq!(journal.payload_bytes, 0);
        assert!(journal.since(67, 68).is_none());
        journal.record(69, &[]);
        assert_eq!(journal.latest, 68);
        journal.record(69, &[cell([9; 3], 0)]);
        assert_eq!(journal.since(68, 69), Some(vec![cell([9; 3], 0)]));
        journal.record(72, &[cell([10; 3], 0)]);
        assert!(journal.since(69, 72).is_none());
        assert_eq!(journal.since(71, 72).unwrap().len(), 1);
        assert!(journal.since(73, 72).is_none());
    }

    #[test]
    fn real_online_free_changes_distinguish_far_space_and_previously_retained_person() {
        use crate::dynamic_scene::tests::{corridor, ORIGIN};
        let (points, truth) = corridor(Some(0.), 0.);
        let mut online = OnlineDufo::default();
        let (_, kept) = online
            .observe(&points, ORIGIN, 1, nalgebra::Isometry3::identity())
            .unwrap();
        let people: Vec<_> = kept
            .into_iter()
            .filter(|&i| truth[i as usize])
            .map(|i| points[i as usize])
            .collect();
        assert!(!people.is_empty());
        let mut bloom = ProcessedEvidence::new();
        for point in &people {
            bloom.insert_point(*point);
        }
        let before = online.free_revision();
        let (empty, _) = corridor(None, 0.);
        for index in 0..32 {
            let far: Vec<_> = empty
                .iter()
                .map(|p| p + nalgebra::Vector3::new(100., 0., 0.))
                .collect();
            online
                .observe(
                    &far,
                    [ORIGIN[0] + 100., ORIGIN[1], ORIGIN[2]],
                    index + 2,
                    nalgebra::Isometry3::identity(),
                )
                .unwrap();
        }
        let (frozen, changes) = online.snapshot_since(before);
        let changes = changes.expect("短场自由变化必须有完整日志");
        assert!(!changes.is_empty());
        assert!(
            !bloom.may_intersect(&changes),
            "远处变化不能要求重放近处保留点"
        );
        let before = frozen.free_revision();
        for index in 0..33 {
            online
                .observe(&empty, ORIGIN, index + 34, nalgebra::Isometry3::identity())
                .unwrap();
        }
        let (newer, changes) = online.snapshot_since(before);
        let changes = changes.expect("相邻修订必须有完整日志");
        let frame = newer.terminal_frame(&people, &people).unwrap();
        assert!(
            people.iter().any(|p| frame.is_dynamic([p.x, p.y, p.z])),
            "人体离开后必须出现真实相关空闲证据"
        );
        assert!(
            bloom.may_intersect(&changes),
            "真实相关变化不得被布隆过滤器遗漏"
        );
        assert_eq!(
            frozen.free_revision(),
            before,
            "冻结树的独立后续修订不能被在线修改"
        );
        assert_eq!(frozen.snapshot_since(before).1, Some(vec![]));
        assert_eq!(OnlineDufo::default().snapshot_since(0).1, Some(vec![]));
    }
}

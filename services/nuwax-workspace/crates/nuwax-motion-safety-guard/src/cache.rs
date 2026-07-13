//! nuwax-motion-safety-guard dedup cache + emergency reserve(§P0B #23/#24)
//!
//! 对齐 W32 Python 端 `tools/motion_safety_guard/dedup_cache.py`:
//! - 75% 普通 + 25% emergency reserve 物理隔离
//! - 普通 `try_add`:满 → RESOURCE_EXHAUSTED(不消耗 reserve)
//! - `try_emergency`:仅安全路径(LatchSafetyStop / RevokeAndZero / ADMIN)可用
//! - `add_tombstone`:Acquire tombstone 整个 boot epoch 保留(上限 1M)
//! - 80% 告警 / 90% 阻止(`CacheHealth` advisory 信号,实际写入仍可成功)
//!
//! L-N0.1 简化:不持久化,内存 HashMap;真持久化由后续 P1-P5 lane 接通。

use std::collections::{HashMap, HashSet};

/// §P0B #24 emergency reserve 固定占比 25%。
pub const EMERGENCY_RESERVE_PCT: u64 = 25;

/// §P0B #23 Acquire tombstone 上限(整个 boot epoch 保留)。
pub const TOMBSTONE_LIMIT: u64 = 1_000_000;

/// §P0B #24 ≥ 80% ordinary 触发告警。
pub const PLANNED_RESTART_PCT: u64 = 80;

/// §P0B #24 ≥ 90% ordinary 触发阻止位(advisory)。
pub const BLOCK_NEW_PCT: u64 = 90;

/// §P0B #24 dedup capacity 默认值。
pub const DEDUP_CAPACITY_DEFAULT: u64 = 1_000_000;

/// dedup cache 结果类型:`(ok, code)` 风格,与 Python 端 1:1。
///
/// L-N0.1 用 `&'static str` 而非 owned `String` 避免 alloc。
pub type DedupResult = (bool, &'static str);

/// §P0B #24 cache 健康度快照(advisory 信号,供 LatchSafetyStop 决策使用)。
#[derive(Debug, Clone, PartialEq)]
pub struct CacheHealth {
    /// ordinary_used 占用数
    pub ordinary_used: u64,
    /// ordinary_capacity 容量
    pub ordinary_capacity: u64,
    /// ordinary_pct = ordinary_used / ordinary_capacity × 100
    pub ordinary_pct: f64,
    /// emergency_used 占用数
    pub emergency_used: u64,
    /// emergency_reserve 容量
    pub emergency_reserve: u64,
    /// emergency_pct
    pub emergency_pct: f64,
    /// 当前 tombstone 数
    pub tombstones: u64,
    /// tombstone_pct
    pub tombstone_pct: f64,
    /// ordinary_pct >= 80% → true
    pub alert_planned_restart: bool,
    /// ordinary_pct >= 90% → true
    pub block_new_acquire: bool,
}

/// §P0B #23 + #24 dedup cache + emergency reserve。
///
/// capacity 按 75% 普通 + 25% reserve 分配;**reserve 物理隔离**,普通入边不可达。
#[derive(Debug)]
pub struct DedupCache {
    /// 普通 cache 容量 = capacity * 75%
    ordinary_capacity: u64,
    /// emergency reserve 容量 = capacity - ordinary_capacity
    emergency_reserve: u64,
    /// 普通已占用
    ordinary_used: u64,
    /// emergency 已占用
    emergency_used: u64,
    /// key → payload_hash
    entries: HashMap<String, String>,
    /// Acquire tombstone 整个 boot epoch 保留
    tombstones: HashSet<String>,
}

impl DedupCache {
    /// §P0B #23 + #24:默认 capacity = 1M(75% + 25%)。
    pub fn new() -> Self {
        Self::with_capacity(DEDUP_CAPACITY_DEFAULT)
    }

    /// 指定 capacity 构造。
    ///
    /// capacity 必须为正;非法值 panic(L-N0.1 简化:用 assert 与 Python `ValueError` 对齐)。
    pub fn with_capacity(capacity: u64) -> Self {
        assert!(capacity > 0, "capacity 必须为正,got {capacity}");
        let ordinary_capacity = capacity * (100 - EMERGENCY_RESERVE_PCT) / 100;
        let emergency_reserve = capacity - ordinary_capacity;
        Self {
            ordinary_capacity,
            emergency_reserve,
            ordinary_used: 0,
            emergency_used: 0,
            entries: HashMap::new(),
            tombstones: HashSet::new(),
        }
    }

    fn ordinary_full(&self) -> bool {
        self.ordinary_used >= self.ordinary_capacity
    }

    fn emergency_full(&self) -> bool {
        self.emergency_used >= self.emergency_reserve
    }

    /// §P0B #23 普通 cache 添加。
    ///
    /// - 同 key 同 hash:`(true, "CACHE_HIT")`(幂等)
    /// - 同 key 异 hash:`(false, "PAYLOAD_CONFLICT")`
    /// - 异 key 且满:`(false, "RESOURCE_EXHAUSTED")`(不消耗 reserve)
    pub fn try_add(&mut self, key: &str, payload_hash: &str) -> DedupResult {
        if let Some(existing) = self.entries.get(key) {
            if existing == payload_hash {
                return (true, "CACHE_HIT");
            }
            return (false, "PAYLOAD_CONFLICT");
        }
        if self.ordinary_full() {
            return (false, "RESOURCE_EXHAUSTED");
        }
        self.entries
            .insert(key.to_string(), payload_hash.to_string());
        self.ordinary_used += 1;
        (true, "OK")
    }

    /// §P0B #24 emergency reserve 添加,仅安全路径可用。
    ///
    /// - 同 key 已在(不论哪个 partition):`(true, "CACHE_HIT")`
    /// - 异 key 且 reserve 满:`(false, "RESOURCE_EXHAUSTED")`
    pub fn try_emergency(&mut self, key: &str, payload_hash: &str) -> DedupResult {
        if self.entries.contains_key(key) {
            return (true, "CACHE_HIT");
        }
        if self.emergency_full() {
            return (false, "RESOURCE_EXHAUSTED");
        }
        self.entries
            .insert(key.to_string(), payload_hash.to_string());
        self.emergency_used += 1;
        (true, "OK")
    }

    /// §P0B #23 Acquire tombstone 整个 boot epoch 保留。
    ///
    /// 满 1M → `TOMBSTONE_EXHAUSTED`(此时 LatchSafetyStop,避免永久停服)。
    pub fn add_tombstone(&mut self, acquire_key: &str) -> DedupResult {
        if self.tombstones.contains(acquire_key) {
            return (true, "TOMBSTONE_HIT");
        }
        if self.tombstones.len() as u64 >= TOMBSTONE_LIMIT {
            return (false, "TOMBSTONE_EXHAUSTED");
        }
        self.tombstones.insert(acquire_key.to_string());
        (true, "OK")
    }

    /// §P0B #24 健康度快照:ordinary/emergency 占比与告警位。
    pub fn get_health(&self) -> CacheHealth {
        let ordinary_pct = if self.ordinary_capacity > 0 {
            self.ordinary_used as f64 / self.ordinary_capacity as f64 * 100.0
        } else {
            0.0
        };
        let emergency_pct = if self.emergency_reserve > 0 {
            self.emergency_used as f64 / self.emergency_reserve as f64 * 100.0
        } else {
            0.0
        };
        let tombstone_pct = self.tombstones.len() as f64 / TOMBSTONE_LIMIT as f64 * 100.0;
        CacheHealth {
            ordinary_used: self.ordinary_used,
            ordinary_capacity: self.ordinary_capacity,
            ordinary_pct: round2(ordinary_pct),
            emergency_used: self.emergency_used,
            emergency_reserve: self.emergency_reserve,
            emergency_pct: round2(emergency_pct),
            tombstones: self.tombstones.len() as u64,
            tombstone_pct: round2(tombstone_pct),
            alert_planned_restart: ordinary_pct >= PLANNED_RESTART_PCT as f64,
            block_new_acquire: ordinary_pct >= BLOCK_NEW_PCT as f64,
        }
    }

    /// ordinary 容量(诊断用)
    pub fn ordinary_capacity(&self) -> u64 {
        self.ordinary_capacity
    }

    /// emergency reserve 容量(诊断用)
    pub fn emergency_reserve(&self) -> u64 {
        self.emergency_reserve
    }

    /// ordinary 已占用
    pub fn ordinary_used(&self) -> u64 {
        self.ordinary_used
    }

    /// emergency 已占用
    pub fn emergency_used(&self) -> u64 {
        self.emergency_used
    }
}

impl Default for DedupCache {
    fn default() -> Self {
        Self::new()
    }
}

/// L-N0.1 helper:四舍五入到 2 位小数(与 Python `round(..., 2)` 对齐)。
fn round2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §P0B #24 默认 capacity 分配:ordinary 75% + reserve 25%。
    #[test]
    fn default_capacity_partition() {
        let cache = DedupCache::new();
        assert_eq!(cache.ordinary_capacity(), 750_000);
        assert_eq!(cache.emergency_reserve(), 250_000);
    }

    /// §P0B #24 小 capacity 测试分配。
    #[test]
    fn small_capacity_partition() {
        let cache = DedupCache::with_capacity(100);
        assert_eq!(cache.ordinary_capacity(), 75);
        assert_eq!(cache.emergency_reserve(), 25);
    }

    /// §P0B #24 capacity=0 panic。
    #[test]
    #[should_panic(expected = "capacity 必须为正")]
    fn zero_capacity_panics() {
        let _ = DedupCache::with_capacity(0);
    }

    /// §P0B #23 ordinary 入边基本流 + 幂等 + CONFLICT。
    #[test]
    fn ordinary_basic_flow() {
        let mut cache = DedupCache::new();
        // 1) 写入 OK
        let (ok, code) = cache.try_add("k1", "h1");
        assert!(ok);
        assert_eq!(code, "OK");
        assert_eq!(cache.ordinary_used(), 1);
        assert_eq!(cache.emergency_used(), 0);

        // 2) 幂等 CACHE_HIT
        let (ok, code) = cache.try_add("k1", "h1");
        assert!(ok);
        assert_eq!(code, "CACHE_HIT");
        assert_eq!(cache.ordinary_used(), 1, "CACHE_HIT 不应递增计数");

        // 3) PAYLOAD_CONFLICT
        let (ok, code) = cache.try_add("k1", "h1-diff");
        assert!(!ok);
        assert_eq!(code, "PAYLOAD_CONFLICT");
        assert_eq!(cache.ordinary_used(), 1, "CONFLICT 不应递增计数");
    }

    /// §P0B #24 填满 ordinary → RESOURCE_EXHAUSTED 且不动 reserve。
    #[test]
    fn ordinary_full_blocks_without_consuming_reserve() {
        let mut cache = DedupCache::with_capacity(100);
        for i in 0..75 {
            let (ok, code) = cache.try_add(&format!("k{i}"), &format!("h{i}"));
            assert!(ok, "i={i} 写入失败: {code}");
            assert_eq!(code, "OK");
        }
        assert_eq!(cache.ordinary_used(), 75);

        // 普通 cache 已满:异 key 必须 RESOURCE_EXHAUSTED(且不动 reserve)
        let (ok, code) = cache.try_add("overflow", "x");
        assert!(!ok);
        assert_eq!(code, "RESOURCE_EXHAUSTED");
        assert_eq!(cache.ordinary_used(), 75);
        assert_eq!(
            cache.emergency_used(),
            0,
            "RESOURCE_EXHAUSTED 不应消耗 reserve"
        );
    }

    /// §P0B #24 emergency reserve 物理隔离 + 满后 RESOURCE_EXHAUSTED。
    #[test]
    fn emergency_reserve_isolation() {
        let mut cache = DedupCache::with_capacity(100);
        // 先填满 ordinary
        for i in 0..75 {
            cache.try_add(&format!("k{i}"), &format!("h{i}"));
        }

        // emergency reserve 仍 OK
        let (ok, code) = cache.try_emergency("emergency-1", "h-em-1");
        assert!(ok);
        assert_eq!(code, "OK");
        assert_eq!(cache.emergency_used(), 1);
        assert_eq!(cache.ordinary_used(), 75, "emergency 不动 ordinary");

        // CACHE_HIT
        let (ok, code) = cache.try_emergency("emergency-1", "h-em-1");
        assert!(ok);
        assert_eq!(code, "CACHE_HIT");

        // 满后 RESOURCE_EXHAUSTED
        let mut cache2 = DedupCache::with_capacity(100);
        for i in 0..25 {
            let (ok, code) = cache2.try_emergency(&format!("e{i}"), &format!("eh{i}"));
            assert!(ok, "emergency i={i} 写入失败: {code}");
        }
        assert_eq!(cache2.emergency_used(), 25);
        let (ok, code) = cache2.try_emergency("e-overflow", "x");
        assert!(!ok);
        assert_eq!(code, "RESOURCE_EXHAUSTED");
    }

    /// §P0B #23 Acquire tombstone 整个 boot epoch 保留。
    #[test]
    fn tombstone_boot_epoch_kept() {
        let mut cache = DedupCache::with_capacity(100);
        let (ok, code) = cache.add_tombstone("acquire-key-1");
        assert!(ok);
        assert_eq!(code, "OK");
        let (ok, code) = cache.add_tombstone("acquire-key-1");
        assert!(ok);
        assert_eq!(code, "TOMBSTONE_HIT");
    }

    /// §P0B #24 健康度阈值:80% 告警 / 90% 阻止。
    #[test]
    fn health_thresholds() {
        let cache_low = DedupCache::with_capacity(100);
        let h = cache_low.get_health();
        assert_eq!(h.ordinary_pct, 0.0);
        assert!(!h.alert_planned_restart);
        assert!(!h.block_new_acquire);

        // 80% 告警
        let mut warn = DedupCache::with_capacity(100);
        for i in 0..60 {
            warn.try_add(&format!("k{i}"), "h");
        }
        let h = warn.get_health();
        assert_eq!(h.ordinary_pct, 80.0);
        assert!(h.alert_planned_restart);
        assert!(!h.block_new_acquire, "80% 仅告警,未阻止");

        // 90% 阻止
        let mut block = DedupCache::with_capacity(100);
        for i in 0..68 {
            block.try_add(&format!("k{i}"), "h");
        }
        let h = block.get_health();
        assert!(h.ordinary_pct >= 90.0);
        assert!(h.alert_planned_restart);
        assert!(h.block_new_acquire);

        // block_new_acquire 是 advisory,实际写入仍可成功(物理未满)
        let (ok, code) = block.try_add("k-new", "h");
        assert!(ok, "90% 仅告警,物理未满仍写成功: code={code}");
        assert_eq!(code, "OK");

        // emergency reserve 独立于 ordinary 状态
        let (ok, _code) = block.try_emergency("emergency-final", "h-em");
        assert!(ok, "emergency reserve 必须独立于 ordinary 状态");
    }

    /// §P0B #24 物理满后必须拒绝。
    #[test]
    fn physical_full_rejects() {
        let mut cache = DedupCache::with_capacity(100);
        for i in 0..75 {
            cache.try_add(&format!("k{i}"), "h");
        }
        assert_eq!(cache.ordinary_used(), 75);
        let (ok, code) = cache.try_add("k-overflow", "h");
        assert!(!ok);
        assert_eq!(code, "RESOURCE_EXHAUSTED");
    }
}

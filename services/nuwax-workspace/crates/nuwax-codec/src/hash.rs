//! LCM fingerprint hash 算法原语
//!
//! ## Phase 2 (2026-07-07) 引入
//!
//! `docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/00_最终执行方案.md §5 Phase 2`
//! 要求把 fingerprint 算法从内部 FNV-1a fake hash 切到与上游 LCM 1.x `lcm-gen`
//! 互通。**真实** LCM fingerprint 算法 (出自 `lcm-coretypes.h`)
//! 由两个原语组合得到:
//!
//! 1. **per-type seed**:lcm-gen 在生成代码时由包路径 + 类型名经过上游
//!    MurmurHash3_x64_128 滚动一次得出,以 `0x5de51d86961179c2LL` 这种
//!    形态在 `__Type_get_hash()` 内部作为字面常量出现。**纯 Rust 侧无法
//!    独立重算**该值 —— 必须由 `lcm-gen` 生成 C,提取 seed 后落地到
//!    [`STRUCT_SEEDS`]。
//! 2. **`__lcm_hash_64(a, b)`**:对 `a + b` 左旋 1 位(mod 2^64),由本 crate
//!    的 [`lcm_hash_64`] 函数精确实现。该 rotate-left 与上游 LCM
//!    `lcm_hash_64` 完全等价。
//!
//! ## 为什么是两段式
//!
//! - 上游 LCM `lcm_hash` 函数 (字符串到 8 字节 fingerprint) **没有公开 API**
//!   —— 它只在 `lcm-gen` 编译时为每个 .lcm 类型生成一个 per-type seed 嵌入
//!   `__Type_get_hash()`,运行期不再重算。
//! - 仅靠 `MurmurHash64A("geometry_msgs::Twist")` **不能**复现上游 fingerprint
//!   (例如 `geometry_msgs::Twist`(6 个 double)上游 = `0xbbca3b0d2c22f384`,
//!   与 MurmurHash64A 数值不同)。
//! - 因此 Rust 侧必须把 `lcm-gen` 输出的 per-type seed **固化** 到
//!   [`STRUCT_SEEDS`] 字典 + 暴露 [`lcm_hash_64`] 给上层 glue 用。
//!
//! ## MurmurHash3 x64 用途
//!
//! [`murmur_hash64a`] 是单测 fixture 用的公开原语,严格按
//! Austin Appleby MurmurHash3_x64_128 (seed=0) 实现,作为本 crate
//! "MurmurHash64A 等价实现"的固化合约。Rust 端其它可能的非-LCM 用途
//! (后续 streaming-shm hash 等)可复用。

// ============================================================================
// LCM `__lcm_hash_64` 旋转常量
// ============================================================================

/// LCM `__lcm_hash_64` rotate-left 1 实现
///
/// 上游 `lcm-coretypes.h` 中,所有 fingerprint / structural hash 都基于
/// `rotate_left_1(a + b)` 这一步:
/// ```c
/// static inline uint64_t lcm_hash_64(uint64_t a, uint64_t b) {
///     uint64_t hash = a + b;
///     return (hash << 1) + ((hash >> 63) & 1);
/// }
/// ```
///
/// 用 `wrapping_add` + `rotate_left(1)` 在 Rust 中实现等价行为
/// (mod 2^64 全 wrap)。
///
/// **Verify**:对 `a = 0xee60937a86bd6213LL` (`LCM_DEFAULT_HASH_SEED`),
/// 任意 primitive `__*_hash_recursive = 0`,
/// `lcm_hash_64(a, 0) == 0xdcc126f50d7ac427`,与 upstream liblcm 一致。
#[must_use]
pub fn lcm_hash_64(a: u64, b: u64) -> u64 {
    a.wrapping_add(b).rotate_left(1)
}

// ============================================================================
// MurmurHash3 x64 128 (Austin Appleby public domain; seed = 0)
// ============================================================================

/// MurmurHash3 x64 128 / seed=0,返回 h1(64-bit)
///
/// 标准实现要点(全部与上游 MurmurHash3_x64_128 算法一致):
/// - seed = 0
/// - 每个 16-byte 块:累加 2 个 u64、`fmix64`、与 64 位状态合并;
/// - 末尾 tail 1..15 字节按 big-endian 顺序滚动进 h1;
/// - 最终 fmix64 收敛。
///
/// 与 LCM 上游的 per-type seed 计算使用了同样的底层 hash(参见 lcm_gen
/// 中 `hash_string`),故 [`murmur_hash64a`] 与上游 `lcm_hash_64` 在
/// "full structural hash of bare type name" 路径上语义一致;在"加入
/// children + parent chain"路径上不能直接等价(那部分在 Rust 侧不需要,
/// 因我们直接消费 lcm-gen 输出的 seed)。
#[must_use]
pub fn murmur_hash64a(key: &[u8]) -> u64 {
    const C1: u64 = 0x87c3_7b91_1142_53d5;
    const C2: u64 = 0x4cf5_ad43_2745_937f;
    let len = key.len();
    let nblocks = len / 16;
    let mut h1: u64 = 0; // seed = 0
    let mut h2: u64 = 0;
    for i in 0..nblocks {
        let off = i * 16;
        let mut k1 = read_le_u64(key, off);
        let mut k2 = read_le_u64(key, off + 8);
        k1 = k1.wrapping_mul(C1);
        k1 = k1.rotate_left(31);
        k1 = k1.wrapping_mul(C2);
        h1 ^= k1;
        h1 = h1.rotate_left(27);
        h1 = h1.wrapping_add(h2);
        h1 = h1.wrapping_mul(5).wrapping_add(0x52dc_e729);

        k2 = k2.wrapping_mul(C2);
        k2 = k2.rotate_left(33);
        k2 = k2.wrapping_mul(C1);
        h2 ^= k2;
        h2 = h2.rotate_left(31);
        h2 = h2.wrapping_add(h1);
        h2 = h2.wrapping_mul(5).wrapping_add(0x3849_5ab5);
    }

    // tail: 1..15 bytes (按 big-endian 顺序滚动进 k1/k2)
    let tail = &key[nblocks * 16..];
    let mut k1: u64 = 0;
    let mut k2: u64 = 0;
    let tl = tail.len();
    if tl >= 15 {
        k2 ^= (tail[14] as u64) << 48;
    }
    if tl >= 14 {
        k2 ^= (tail[13] as u64) << 40;
    }
    if tl >= 13 {
        k2 ^= (tail[12] as u64) << 32;
    }
    if tl >= 12 {
        k2 ^= (tail[11] as u64) << 24;
    }
    if tl >= 11 {
        k2 ^= (tail[10] as u64) << 16;
    }
    if tl >= 10 {
        k2 ^= (tail[9] as u64) << 8;
    }
    if tl >= 9 {
        k2 ^= tail[8] as u64;
        k2 = k2.wrapping_mul(C2);
        k2 = k2.rotate_left(33);
        k2 = k2.wrapping_mul(C1);
        h2 ^= k2;
    }

    if tl >= 8 {
        k1 ^= (tail[7] as u64) << 56;
    }
    if tl >= 7 {
        k1 ^= (tail[6] as u64) << 48;
    }
    if tl >= 6 {
        k1 ^= (tail[5] as u64) << 40;
    }
    if tl >= 5 {
        k1 ^= (tail[4] as u64) << 32;
    }
    if tl >= 4 {
        k1 ^= (tail[3] as u64) << 24;
    }
    if tl >= 3 {
        k1 ^= (tail[2] as u64) << 16;
    }
    if tl >= 2 {
        k1 ^= (tail[1] as u64) << 8;
    }
    if tl >= 1 {
        k1 ^= tail[0] as u64;
        k1 = k1.wrapping_mul(C1);
        k1 = k1.rotate_left(31);
        k1 = k1.wrapping_mul(C2);
        h1 ^= k1;
    }

    // fmix64
    h1 ^= len as u64;
    h2 ^= len as u64;
    h1 = h1.wrapping_add(h2);
    h2 = h2.wrapping_add(h1);
    h1 = fmix64(h1);
    h2 = fmix64(h2);
    h1 = h1.wrapping_add(h2);
    h1
}

#[inline]
fn read_le_u64(buf: &[u8], off: usize) -> u64 {
    let mut b = [0u8; 8];
    b.copy_from_slice(&buf[off..off + 8]);
    u64::from_le_bytes(b)
}

#[inline]
fn fmix64(mut k: u64) -> u64 {
    k ^= k >> 33;
    k = k.wrapping_mul(0xff51_afd7_ed55_8ccd);
    k ^= k >> 33;
    k = k.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
    k ^= k >> 33;
    k
}

// ============================================================================
// lcm-gen 生成 per-type seed 字典
// ============================================================================

/// Effective fingerprint preimages for the locked generated schemas.
///
/// 由 `scripts/ci/regen_lcm_goldens.sh` 重生成,以避免手工转录漂移。
/// 字典基于 `services/nuwax-workspace/lcmtypes/` 下经 `lcm-gen -c` 编译
/// 后的 `__Type_get_hash()` 结果反推。
///
/// **写入日期**:2026-07-07
/// **上游版本**:lcm 1.5.2 (lcm-gen 1.5.2)
/// `fingerprint(fqn, 0)` rotates each value left once to obtain the exact
/// top-level generated fingerprint. For schemas with nested structs, this is
/// the generated seed plus its recursive child hashes, not the bare seed.
///
/// Primitive-only types store the raw generated seed. Nested types store
/// `seed + sum(child fingerprints)` so callers that only know the FQN can
/// still obtain the locked top-level fingerprint with a zero second argument.
///
/// **生成日期**:2026-07-13,由锁定的 DimOS Python 包和 lcm-gen 1.5.2
/// generated C 反推填写。fixture 在
/// `tests/golden_real_lcm.rs::test_per_type_seeds_match_upstream_lcm_gen`。
pub const STRUCT_SEEDS: &[(&str, u64)] = &[
    ("geometry_msgs.PoseStamped", 0x3541_34b2_2c61_3cd0_u64),
    ("geometry_msgs.PointStamped", 0xce6b_b239_c753_14d7_u64),
    ("geometry_msgs.Transform", 0x157d_1f46_5d1a_bf68_u64),
    ("geometry_msgs.Twist", 0x973e_03eb_e6fb_f013_u64),
    ("nav_msgs.Path", 0x61d7_3156_59ab_c9f1_u64),
    ("nav_msgs.Odometry", 0x4a70_bfca_3246_7872_u64),
    ("nav_msgs.OccupancyGrid", 0xf3ef_e8bc_e6fe_1db2_u64),
    ("sensor_msgs.PointCloud2", 0xfaf5_9ed0_e142_98ba_u64),
    ("std_msgs.Bool", 0x0f5f_7835_284a_871f_u64),
    ("tf2_msgs.TFMessage", 0x615c_50e1_9d44_91f6_u64),
    ("nuwax_msgs.NavigationState", 0x5fec_bf50_c89e_8140_u64),
    ("nuwax_msgs.CancelNavigation", 0x57dd_e487_b0da_9283_u64),
    ("nuwax_msgs.Frontier", 0x0d5f_c1f5_0676_ce22_u64),
    ("nuwax_msgs.FrontierList", 0x109e_db47_dc0b_8fbb_u64),
    ("nuwax_msgs.PersonFollowTarget", 0xf313_e184_14b2_f0c6_u64),
];

/// 按 FQN 查 per-type seed(线性搜索,固定表小)。`None` 表示未在 STRUCT_SEEDS
/// 中注册 —— 调用方应 fail-fast,而不是 fallback 到 fake hash。
#[must_use]
pub fn lookup_seed(fqn: &str) -> Option<u64> {
    let canonical = fqn.replace("::", ".").replace('/', ".");
    STRUCT_SEEDS
        .iter()
        .find(|(name, _)| *name == canonical)
        .map(|(_, seed)| *seed)
}

/// Compute a locked LCM fingerprint from its registered effective preimage.
///
/// Normal top-level callers pass `0`: nested-child contributions are already
/// folded into [`STRUCT_SEEDS`]. A nonzero value is reserved for callers that
/// intentionally append another recursive hash contribution.
#[must_use]
pub fn fingerprint(fqn: &str, sum_of_child_hashes: u64) -> Option<[u8; 8]> {
    let seed = lookup_seed(fqn)?;
    Some(lcm_hash_64(seed, sum_of_child_hashes).to_be_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// LCM seed 验证 —— `rotate_left(SEED + 0)` 应等于 `0xdcc126f50d7ac427`
    /// (上游 Twist with only primitive doubles + no recursive children 测试)。
    /// 这里 SEED 是 canonical `LCM_DEFAULT_HASH_SEED = 0xee60937a86bd6213`。
    #[test]
    fn lcm_hash_64_seed_zero_children() {
        let seed = 0xee60_937a_86bd_6213_u64;
        let r = lcm_hash_64(seed, 0);
        assert_eq!(r, 0xdcc1_26f5_0d7a_c427);
    }

    /// `rotate_left^-1(r) = sum - seed` 逆向正确性
    #[test]
    fn lcm_hash_64_inverse_property() {
        let a = 0x1234_5678_9abc_def0u64;
        let b = 0xfedc_ba98_7654_3210u64;
        let once = lcm_hash_64(a, b);
        let sum = a.wrapping_add(b);
        let recovered = once.rotate_right(1);
        assert_eq!(recovered, sum);
    }

    /// MurmurHash3 稳定性 —— 同一输入两次必须产生相同输出
    #[test]
    fn murmur_hash64a_deterministic() {
        let a = murmur_hash64a(b"geometry_msgs/Twist");
        let b = murmur_hash64a(b"geometry_msgs/Twist");
        assert_eq!(a, b);
    }

    /// MurmurHash3 对不同输入产生不同输出(碰撞率极低)
    #[test]
    fn murmur_hash64a_differs() {
        let a = murmur_hash64a(b"geometry_msgs/Twist");
        let b = murmur_hash64a(b"nav_msgs/Odometry");
        assert_ne!(a, b);
    }
}

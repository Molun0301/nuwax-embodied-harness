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

/// Compute a FIPS 180-4 SHA-256 digest without adding a crypto dependency.
///
/// Navigation envelopes use this digest as an immutable payload identity, not
/// as an authentication primitive. Authenticity remains the transport and
/// process-identity layer's responsibility.
#[must_use]
pub fn sha256_bytes(data: &[u8]) -> [u8; 32] {
    const H_INIT: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];

    let bit_len = (data.len() as u64).wrapping_mul(8);
    let mut padded = data.to_vec();
    padded.push(0x80);
    while padded.len() % 64 != 56 {
        padded.push(0);
    }
    padded.extend_from_slice(&bit_len.to_be_bytes());

    let mut h = H_INIT;
    for chunk in padded.chunks_exact(64) {
        let mut w = [0_u32; 64];
        for (index, word) in w.iter_mut().take(16).enumerate() {
            let offset = index * 4;
            *word = u32::from_be_bytes(chunk[offset..offset + 4].try_into().expect("four bytes"));
        }
        for index in 16..64 {
            let s0 = w[index - 15].rotate_right(7)
                ^ w[index - 15].rotate_right(18)
                ^ (w[index - 15] >> 3);
            let s1 = w[index - 2].rotate_right(17)
                ^ w[index - 2].rotate_right(19)
                ^ (w[index - 2] >> 10);
            w[index] = w[index - 16]
                .wrapping_add(s0)
                .wrapping_add(w[index - 7])
                .wrapping_add(s1);
        }

        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh] = h;
        for (word, constant) in w.iter().zip(K) {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let choose = (e & f) ^ ((!e) & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(choose)
                .wrapping_add(constant)
                .wrapping_add(*word);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let majority = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(majority);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        for (target, value) in h.iter_mut().zip([a, b, c, d, e, f, g, hh]) {
            *target = target.wrapping_add(value);
        }
    }

    let mut digest = [0_u8; 32];
    for (index, value) in h.iter().enumerate() {
        digest[index * 4..index * 4 + 4].copy_from_slice(&value.to_be_bytes());
    }
    digest
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
    ("geometry_msgs.Pose", 0x122b_1a57_1558_bdbb_u64),
    ("geometry_msgs.PoseStamped", 0x3541_34b2_2c61_3cd0_u64),
    ("geometry_msgs.PointStamped", 0xce6b_b239_c753_14d7_u64),
    ("geometry_msgs.Transform", 0xf72f_fa13_23df_9fc8_u64),
    ("geometry_msgs.Twist", 0x973e_03eb_e6fb_f013_u64),
    ("nav_msgs.Path", 0x61d7_3156_59ab_c9f1_u64),
    ("nav_msgs.Odometry", 0x4a70_bfca_3246_7872_u64),
    ("nav_msgs.OccupancyGrid", 0xf3ef_e8bc_e6fe_1db2_u64),
    ("sensor_msgs.PointCloud2", 0xfaf5_9ed0_e142_98ba_u64),
    ("sensor_msgs.Imu", 0x18d5_902e_46ea_bd54_u64),
    ("std_msgs.Bool", 0x0f5f_7835_284a_871f_u64),
    ("tf2_msgs.TFMessage", 0x615c_50e1_9d44_91f6_u64),
    ("nuwax_msgs.GoalEnvelope", u64::from_be_bytes(crate::lcm::schema::GOAL_ENVELOPE_FINGERPRINT).rotate_right(1)),
    ("nuwax_msgs.PathEnvelope", u64::from_be_bytes(crate::lcm::schema::PATH_ENVELOPE_FINGERPRINT).rotate_right(1)),
    ("nuwax_msgs.NavigationState", 0x4f51_f9fa_fead_7ded_u64),
    // 复用生成器确认的真实指纹，按既有注册表约定存放逆旋转值。
    (
        "nuwax_msgs.NavigationMap3D",
        u64::from_be_bytes(crate::lcm::schema::NAVIGATION_MAP_3D_FINGERPRINT).rotate_right(1),
    ),
    ("nuwax_msgs.NavigationMapBlock", u64::from_be_bytes(crate::lcm::schema::NAVIGATION_MAP_BLOCK_FINGERPRINT).rotate_right(1)),
    ("nuwax_msgs.CancelNavigation", 0x4b82_2594_74e2_ee91_u64),
    ("nuwax_msgs.Frontier", 0x0d5f_c1f5_0676_ce22_u64),
    ("nuwax_msgs.FrontierList", 0x109e_db47_dc0b_8fbb_u64),
    ("nuwax_msgs.PlanarNavigationMap", u64::from_be_bytes(crate::lcm::planar_navigation::FINGERPRINT).rotate_right(1)),
    ("nuwax_msgs.PlanarScan", 0x14c1_fbf5_9745_c1cb_u64),
    ("nuwax_msgs.PersonFollowTarget", 0xa4d8_55f7_31c5_8fed_u64),
    ("nuwax_msgs.PersonFollowCommand", 0x3b56_bc67_ec81_a3f4_u64),
    ("nuwax_msgs.LioOdometryEnvelope", 0xba16_cd83_6063_d111_u64),
    ("nuwax_msgs.LioCloudEnvelope", 0xee6a_5707_c1d4_d40d_u64),
    ("nuwax_msgs.LioStatus", 0x36bd_731b_6c5d_20d9_u64),
    ("nuwax_msgs.LioInputAck", 0xbb64_9f22_aa1e_d58c_u64),
    ("nuwax_msgs.LioInputFrame", 0x6191_9fad_0a2d_4535_u64),
    ("nuwax_msgs.CorrectionCandidate", 0xe57c31a15c8dfa3eu64),
    ("nuwax_msgs.NativePgoRequest", 0x8efd141e6f5310aeu64),
    ("nuwax_msgs.NativePgoResult", 0x3c8c7caae4477c6bu64),
    ("nuwax_msgs.NativePgoStatus", 0xfb0f5e91107d2ef1u64),
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

    #[test]
    fn sha256_matches_nist_vectors() {
        assert_eq!(
            sha256_bytes(b""),
            [
                0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f,
                0xb9, 0x24, 0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b,
                0x78, 0x52, 0xb8, 0x55,
            ]
        );
        assert_eq!(
            sha256_bytes(b"abc"),
            [
                0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
                0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
                0xf2, 0x00, 0x15, 0xad,
            ]
        );
    }
}

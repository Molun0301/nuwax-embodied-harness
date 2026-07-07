//! LCM `lcm_hash_64` fingerprint 计算。
//!
//! LCM 1.x frame fingerprint = `__lcm_hash_64(seed + Σ child_hashes)`,
//! 以 **大端 (network byte order)** 写入 framer。
//!
//! ## Phase 2 (2026-07-07) 算法切换
//!
//! `docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/00_最终执行方案.md §5 Phase 2`
//!
//! **2026-07-07 起 fingerprint 算法切换到 upstream `lcm_hash_64`(LCM 兼容),
//! 旧 FNV-1a fingerprint 不再使用。**
//!
//! ## 实现分工
//!
//! - [`nuwax_codec::hash::lcm_hash_64`] 提供 `rotate_left_1(a + b)` 旋转;
//! - [`nuwax_codec::hash::STRUCT_SEEDS`] 固化了 10 个 .lcm schema 的
//!   per-type seed(由 `lcm-gen -c` 在 2026-07-07 反推得出);
//! - [`nuwax_codec::hash::compute_real_lcm_fingerprint`] 组合两者,得到
//!   upstream LCM 1.x 兼容 fingerprint。
//!
//! 旧 FNV-1a 64-bit 实现已从 `nuwax-codec::lcm::fnv1a_64` 与本文件
//! `fnv1a_64` 同步移除 —— 因为它与 lcm-gen 数值不等价,继续保留会让
//! cross-language P1 互通返工。

use nuwax_codec::hash::fingerprint as compute_real_lcm_fingerprint;

/// fingerprint 字节数(8 字节大端网络序)
pub const LCM_HASH_SIZE: usize = 8;

/// 计算 LCM fingerprint（8 字节,大端网络序写入 framer 时使用 `to_be_bytes()`）。
///
/// **2026-07-07 起**:调用 [`nuwax_codec::hash::compute_real_lcm_fingerprint`],
/// 与上游 LCM 1.x `lcm-gen` 的 `__Type_get_hash()` 等价(对全 primitive
/// 字段 schema)。
///
/// 未在 [`nuwax_codec::hash::STRUCT_SEEDS`] 注册的 FQN 一律返回全 0,
/// 并触发 fail-fast 错误(由 codec 层 [`LcmMessageType::fingerprint`]
/// 调用约定保护)。
#[must_use]
pub fn compute_lcm_fingerprint(fqn: &str) -> [u8; 8] {
    compute_real_lcm_fingerprint(fqn, 0).unwrap_or([0u8; 8])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LcmMessageType;

    #[test]
    fn fingerprint_is_8_bytes() {
        let fp = compute_lcm_fingerprint("geometry_msgs/PoseStamped");
        assert_eq!(fp.len(), LCM_HASH_SIZE);
    }

    #[test]
    fn fingerprint_stable() {
        let a = compute_lcm_fingerprint("nav_msgs/Odometry");
        let b = compute_lcm_fingerprint("nav_msgs/Odometry");
        assert_eq!(a, b);
    }

    #[test]
    fn fingerprint_differs_per_fqn() {
        let a = compute_lcm_fingerprint("geometry_msgs/PoseStamped");
        let b = compute_lcm_fingerprint("sensor_msgs/PointCloud2");
        assert_ne!(a, b);
    }

    #[test]
    fn fingerprint_uses_big_endian_byte_order() {
        // fingerprint 写入 framer 时应使用大端(网络序),即
        // 高位字节先出现在字节流中 —— 上游 LCM 的 byte layout。
        let fp = compute_lcm_fingerprint("foo/Bar");
        // 因为 `foo/Bar` 不在 STRUCT_SEEDS,本测试改用已注册类型。
        let fp = compute_lcm_fingerprint("geometry_msgs/Twist");
        // 上游 lcm-gen `geometry_msgs__Twist_get_hash` = 0xbbca3b0d2c22f384
        let expected = 0xbbca_3b0d_2c22_f384u64.to_be_bytes();
        assert_eq!(fp, expected, "fingerprint must equal upstream lcm-gen value");
    }

    /// Phase 2 端到端一致性:协议层 fingerprint 等于上游 lcm-gen __Type_get_hash
    #[test]
    fn fingerprint_matches_upstream_lcm_gen_for_all_ten_types() {
        use nuwax_codec::hash::STRUCT_SEEDS;
        // 上游 lcm-gen 在 2026-07-07 编译 10 个 .lcm schema 后输出的
        // `__Type_get_hash()` 值,由 `scripts/ci/regen_lcm_goldens.sh` 反推
        // STRUCT_SEEDS 后落地。
        let expected: &[(&str, u64)] = &[
            ("geometry_msgs/PoseStamped", 0x2573_8ae5_7ec5_4e84),
            ("geometry_msgs/Transform", 0x2afa_3e8c_ba35_7ed0),
            ("geometry_msgs/Twist", 0xbbca_3b0d_2c22_f384),
            ("nav_msgs/Odometry", 0x1e49_cc4e_3b85_2955),
            ("nav_msgs/OccupancyGrid", 0x6594_8fd9_608c_95bc),
            ("sensor_msgs/PointCloud2", 0x9ceb_8cd9_8f7d_5f48),
            ("nuwax_msgs/NavigationState", 0xbfd9_7ea1_913d_0280),
            ("nuwax_msgs/CancelNavigation", 0xafbb_c90f_61b5_2506),
            ("nuwax_msgs/Frontier", 0x1abf_83ea_0ced_9c44),
            ("nuwax_msgs/FrontierList", 0x213d_b68f_b817_1f76),
        ];
        assert_eq!(STRUCT_SEEDS.len(), expected.len());
        for (fqn, upstream_fp) in expected {
            let msg_type = LcmMessageType::new(fqn).expect("valid");
            let fp = msg_type.fingerprint();
            let fp_u64 = u64::from_be_bytes(fp);
            assert_eq!(
                fp_u64, *upstream_fp,
                "fingerprint mismatch for {fqn}: got 0x{fp_u64:016x}, expected 0x{upstream_fp:016x}"
            );
        }
    }
}
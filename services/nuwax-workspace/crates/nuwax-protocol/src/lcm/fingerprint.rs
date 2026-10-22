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
//! - [`nuwax_codec::hash::STRUCT_SEEDS`] 固化了锁定 schema 的
//!   effective fingerprint preimage(由锁定 Python/C generated artifacts 反推);
//! - [`nuwax_codec::hash::fingerprint`] 组合两者,得到
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
/// **2026-07-07 起**:调用 [`nuwax_codec::hash::fingerprint`],
/// 与上游 LCM 1.x `lcm-gen` 的 `__Type_get_hash()` 等价(对全 primitive
/// 字段 schema)。
///
/// 未在 [`nuwax_codec::hash::STRUCT_SEEDS`] 注册的 FQN 一律返回全 0,
/// 并触发 fail-fast 错误(由 codec 层 [`crate::LcmMessageType::fingerprint`]
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
        let fp = compute_lcm_fingerprint("geometry_msgs.Twist");
        let expected = 0x2e7c_07d7_cdf7_e027_u64.to_be_bytes();
        assert_eq!(
            fp, expected,
            "fingerprint must equal upstream lcm-gen value"
        );
    }

    /// Phase 2 端到端一致性:协议层 fingerprint 等于上游 lcm-gen __Type_get_hash
    #[test]
    fn fingerprint_matches_locked_generated_schemas() {
        use nuwax_codec::hash::STRUCT_SEEDS;
        // Values come from the locked generated DimOS packages.
        let expected: &[(&str, u64)] = &[
            ("geometry_msgs.Pose", 0x2456_34ae_2ab1_7b76),
            ("geometry_msgs.PoseStamped", 0x6a82_6964_58c2_79a0),
            ("geometry_msgs.PointStamped", 0x9cd7_6473_8ea6_29af),
            ("geometry_msgs.Transform", 0xee5f_f426_47bf_3f91),
            ("geometry_msgs.Twist", 0x2e7c_07d7_cdf7_e027),
            ("nav_msgs.Path", 0xc3ae_62ac_b357_93e2),
            ("nav_msgs.Odometry", 0x94e1_7f94_648c_f0e4),
            ("nav_msgs.OccupancyGrid", 0xe7df_d179_cdfc_3b65),
            ("sensor_msgs.PointCloud2", 0xf5eb_3da1_c285_3175),
            ("sensor_msgs.Imu", 0x31ab_205c_8dd5_7aa8),
            ("std_msgs.Bool", 0x1ebe_f06a_5095_0e3e),
            ("tf2_msgs.TFMessage", 0xc2b8_a1c3_3a89_23ec),
            ("nuwax_msgs.GoalEnvelope", 0x8949_7302_1b29_5f7e),
            ("nuwax_msgs.PathEnvelope", 0x8056_3fb3_d7e6_6c57),
            ("nuwax_msgs.NavigationState", 0x9f14_6a1b_54a1_b25d),
            ("nuwax_msgs.NavigationMap3D", 0x131a_5884_d2d6_8d28),
            ("nuwax_msgs.NavigationMapBlock", 0xe544_975d_2a8d_dc13),
            ("nuwax_msgs.PlanarNavigationMap", 0x2a33_0e41_787c_4b45),
            ("nuwax_msgs.PlanarScan", 0x2983f7eb2e8b8396),
            ("nuwax_msgs.CancelNavigation", 0x9704_4b28_e9c5_dd22),
            ("nuwax_msgs.Frontier", 0x1abf_83ea_0ced_9c44),
            ("nuwax_msgs.FrontierList", 0x213d_b68f_b817_1f76),
            ("nuwax_msgs.PersonFollowTarget", 0x49b0_abee_638b_1fdb),
            ("nuwax_msgs.PersonFollowCommand", 0x76ad_78cf_d903_47e8),
            ("nuwax_msgs.LioOdometryEnvelope", 0x742d_9b06_c0c7_a223),
            ("nuwax_msgs.LioCloudEnvelope", 0xdcd4_ae0f_83a9_a81b),
            ("nuwax_msgs.LioStatus", 0x6d7a_e636_d8ba_41b2),
            ("nuwax_msgs.LioInputAck", 0x76c9_3e45_543d_ab19),
            ("nuwax_msgs.LioInputFrame", 0xc323_3f5a_145a_8a6a),
            ("nuwax_msgs.CorrectionCandidate", 0xcaf86342b91bf47d),
            ("nuwax_msgs.NativePgoRequest", 0x1dfa283cdea6215d),
            ("nuwax_msgs.NativePgoResult", 0x7918f955c88ef8d6),
            ("nuwax_msgs.NativePgoStatus", 0xf61ebd2220fa5de3),
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

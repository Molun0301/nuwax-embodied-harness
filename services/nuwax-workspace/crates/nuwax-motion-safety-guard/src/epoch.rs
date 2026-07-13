//! nuwax-motion-safety-guard epoch.json / provisioned.json schema + UID 期望(§P0B #15/#19)
//!
//! 对齐 W32 Python 端:
//! - `tools/motion_safety_guard/w31_guard_epoch_record.py`(`GuardEpochRecord` v1 + provisioned)
//! - `tools/motion_safety_guard/uids_provisioning.py`(4 UID + 1 group)
//!
//! L-N0.1 简化:仅 schema struct + UID 列表常量;真持久化(atomic write + fsync +
//! flock + checksum 校验)由后续 P1-P5 lane 接通。

use serde::{Deserialize, Serialize};

/// §P0B #15 GuardEpochRecord v1 schema version。
pub const SCHEMA_VERSION_EPOCH: u32 = 1;

/// §P0B #15 provisioned.json schema version。
pub const SCHEMA_VERSION_PROVISIONED: u32 = 1;

/// §P0B #15 provisioned.json 默认路径(0400 mode)。
///
/// L-N0.1:仅常量,不实装文件 IO。
pub const PROVISIONED_PATH: &str = "/var/lib/nuwax/motion-guard/provisioned.json";

/// §P0B #19 严格 4 UID,任何变更触发 breaking-change CI BLOCKING。
///
/// 与 Python `EXPECTED_USERS` 1:1 对齐。
pub const EXPECTED_USERS: [&str; 4] = [
    "nuwax-motion-guard",
    "nuwax-motion-admin",
    "nuwax-geometry",
    "nuwax-gateway",
];

/// §P0B #19 1 group,允许 nuwax-geometry / nuwax-gateway 调用 guard UDS。
///
/// 与 Python `EXPECTED_GROUPS` 1:1 对齐。
pub const EXPECTED_GROUPS: [&str; 1] = ["nuwax-motion-clients"];

/// §P0B #15 GuardEpochRecord v1 schema。
///
/// 字段对齐 Python `GuardEpochRecord`:
/// - `schema_version: u32`(默认 1)
/// - `generation: u64`
/// - `boot_nonce_hex: String`(32 chars = 128-bit)
/// - `safety_gate: SafetyGate`
/// - `lease_state: LeaseState`
/// - `reset_generation: u64`
/// - `last_zero_ack_id: String`(u64 或 UUID 字符串)
/// - `peer_uid_allowlist: Vec<String>`
/// - `actuator_symbols_sha256: String`(以 "sha256:" 开头)
/// - `fenced: bool`(initial = true)
/// - `created_at: String`(ISO8601 UTC)
/// - `checksum: String`(sha256:hex)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GuardEpochRecord {
    /// schema 版本(必须为 `SCHEMA_VERSION_EPOCH`)
    pub schema_version: u32,
    /// 单调递增 generation
    pub generation: u64,
    /// boot nonce hex(32 chars = 128-bit);初始为空,首次 boot 后生成
    pub boot_nonce_hex: String,
    /// 当前 SafetyGate
    pub safety_gate: crate::state::SafetyGate,
    /// 当前 LeaseState
    pub lease_state: crate::state::LeaseState,
    /// admin_reset 计数
    pub reset_generation: u64,
    /// 最近 zero ack id(字符串以兼容 UUID 或 u64)
    pub last_zero_ack_id: String,
    /// peer UID allowlist(必须为 `EXPECTED_USERS` 子集)
    pub peer_uid_allowlist: Vec<String>,
    /// actuator symbols sha256(以 "sha256:" 开头;W31 placeholder 形如
    /// `"sha256:placeholder"`)
    pub actuator_symbols_sha256: String,
    /// fenced 标志(initial = true;boot 启动后 fenced = false 等 ADMIN_RESET)
    pub fenced: bool,
    /// 创建时间(ISO8601 UTC)
    pub created_at: String,
    /// sha256(canonical body) checksum
    pub checksum: String,
}

impl Default for GuardEpochRecord {
    fn default() -> Self {
        Self {
            schema_version: SCHEMA_VERSION_EPOCH,
            generation: 0,
            boot_nonce_hex: String::new(),
            safety_gate: crate::state::SafetyGate::ResetRequired,
            lease_state: crate::state::LeaseState::Idle,
            reset_generation: 0,
            last_zero_ack_id: String::new(),
            peer_uid_allowlist: EXPECTED_USERS.iter().map(|s| s.to_string()).collect(),
            actuator_symbols_sha256: "sha256:placeholder".to_string(),
            fenced: true,
            created_at: String::new(),
            checksum: String::new(),
        }
    }
}

impl GuardEpochRecord {
    /// §P0B #15:校验 schema 字段约束;返错误列表(空=PASS)。
    ///
    /// L-N0.1 简化:仅做基本字段检查,真 checksum 校验由 P1-P5 lane 接通。
    pub fn verify(&self) -> Vec<String> {
        let mut errs = Vec::new();
        if self.schema_version != SCHEMA_VERSION_EPOCH {
            errs.push(format!("schema_version != {SCHEMA_VERSION_EPOCH}"));
        }
        if !self.boot_nonce_hex.is_empty() && self.boot_nonce_hex.len() != 32 {
            errs.push(format!(
                "boot_nonce 长度 != 32 chars (128-bit): got {}",
                self.boot_nonce_hex.len()
            ));
        }
        if !self.actuator_symbols_sha256.starts_with("sha256:") {
            errs.push(format!(
                "actuator_symbols_sha256 必须以 'sha256:' 开头,got {:?}",
                self.actuator_symbols_sha256
            ));
        }
        for uid in &self.peer_uid_allowlist {
            if !EXPECTED_USERS.contains(&uid.as_str()) {
                errs.push(format!("peer_uid_allowlist 含未授权 UID:{uid}"));
            }
        }
        errs
    }

    /// §P0B #15:boot advance,generation + 1 + 新 nonce;不动 `safety_gate`,
    /// `lease_state = Idle`,`fenced = false`(等 ADMIN_RESET)。
    ///
    /// L-N0.1 简化:nonce 用确定性 counter(用 generation 衍生),
    /// 避免引入 `rand` 依赖;真 crypto-random nonce 由 P1-P5 lane 接通。
    pub fn boot_advance(&self) -> Self {
        let next_gen = if self.fenced && self.generation == 0 {
            1
        } else {
            self.generation + 1
        };
        let mut nonce = String::with_capacity(32);
        let v = next_gen.wrapping_mul(0x9E3779B97F4A7C15);
        // 用 u64 → 16 hex + 高位零填充到 16 hex,共 32 hex
        use std::fmt::Write;
        let _ = write!(&mut nonce, "{v:016x}");
        let _ = write!(&mut nonce, "{:016x}", v.wrapping_add(0x12345678));
        // 长度恰好 32 chars
        debug_assert_eq!(nonce.len(), 32);
        Self {
            schema_version: SCHEMA_VERSION_EPOCH,
            generation: next_gen,
            boot_nonce_hex: nonce,
            safety_gate: self.safety_gate,
            lease_state: crate::state::LeaseState::Idle,
            reset_generation: self.reset_generation,
            last_zero_ack_id: self.last_zero_ack_id.clone(),
            peer_uid_allowlist: self.peer_uid_allowlist.clone(),
            actuator_symbols_sha256: self.actuator_symbols_sha256.clone(),
            fenced: false,
            created_at: self.created_at.clone(),
            checksum: String::new(),
        }
    }
}

/// §P0B #15 provisioned.json schema。
///
/// instance_id + schema_version + checksum;mode 0400。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProvisionedRecord {
    /// crypto-random instance_id
    pub instance_id: String,
    /// schema 版本(必须为 `SCHEMA_VERSION_PROVISIONED`)
    pub schema_version: u32,
    /// sha256(canonical body, no checksum) checksum
    pub checksum: String,
}

impl Default for ProvisionedRecord {
    fn default() -> Self {
        Self {
            instance_id: String::new(),
            schema_version: SCHEMA_VERSION_PROVISIONED,
            checksum: String::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §P0B #19 4 UID + 1 group 锁定。
    #[test]
    fn expected_users_groups_frozen() {
        assert_eq!(EXPECTED_USERS.len(), 4, "service UID 必须 4 项");
        for u in &[
            "nuwax-motion-guard",
            "nuwax-motion-admin",
            "nuwax-geometry",
            "nuwax-gateway",
        ] {
            assert!(EXPECTED_USERS.contains(u), "service UID 缺: {u}");
        }
        assert_eq!(EXPECTED_GROUPS.len(), 1);
        assert_eq!(EXPECTED_GROUPS[0], "nuwax-motion-clients");
    }

    /// §P0B #15 default GuardEpochRecord:generation=0, fenced=true。
    #[test]
    fn default_epoch_initial_state() {
        let r = GuardEpochRecord::default();
        assert_eq!(r.generation, 0);
        assert!(r.fenced);
        assert!(r.boot_nonce_hex.is_empty());
        assert_eq!(r.safety_gate, crate::state::SafetyGate::ResetRequired);
        assert_eq!(r.lease_state, crate::state::LeaseState::Idle);
    }

    /// §P0B #15 boot advance:generation+1, fenced=false, 新 32-char nonce。
    #[test]
    fn boot_advance_updates_generation_and_nonce() {
        let r = GuardEpochRecord::default();
        let advanced = r.boot_advance();
        assert_eq!(advanced.generation, 1);
        assert!(!advanced.fenced, "boot advance 后必须 fenced=false");
        assert_eq!(advanced.boot_nonce_hex.len(), 32);
        assert!(!advanced.boot_nonce_hex.is_empty());
        assert_eq!(advanced.lease_state, crate::state::LeaseState::Idle);
    }

    /// §P0B #15 verify:错误 schema_version 应被拒。
    #[test]
    fn verify_rejects_wrong_schema_version() {
        let r = GuardEpochRecord {
            schema_version: 999,
            ..GuardEpochRecord::default()
        };
        let errs = r.verify();
        assert!(
            errs.iter().any(|e| e.contains("schema_version")),
            "wrong schema_version 必须报: {errs:?}"
        );
    }

    /// §P0B #15 verify:错误 boot_nonce 长度应被拒。
    #[test]
    fn verify_rejects_wrong_nonce_length() {
        let r = GuardEpochRecord {
            boot_nonce_hex: "tooshort".to_string(),
            ..GuardEpochRecord::default()
        };
        let errs = r.verify();
        assert!(
            errs.iter().any(|e| e.contains("boot_nonce")),
            "wrong nonce 长度必须报: {errs:?}"
        );
    }

    /// §P0B #15 verify:错误 peer_uid_allowlist 应被拒。
    #[test]
    fn verify_rejects_unauthorized_peer_uid() {
        let mut r = GuardEpochRecord::default();
        r.peer_uid_allowlist.push("nuwax-malicious".to_string());
        let errs = r.verify();
        assert!(
            errs.iter().any(|e| e.contains("peer_uid_allowlist")),
            "unauthorized UID 必须报: {errs:?}"
        );
    }

    /// §P0B #15 verify:缺 sha256: 前缀应被拒。
    #[test]
    fn verify_rejects_missing_sha256_prefix() {
        let r = GuardEpochRecord {
            actuator_symbols_sha256: "plaintext".to_string(),
            ..GuardEpochRecord::default()
        };
        let errs = r.verify();
        assert!(
            errs.iter().any(|e| e.contains("actuator_symbols_sha256")),
            "缺 'sha256:' 前缀必须报: {errs:?}"
        );
    }
}

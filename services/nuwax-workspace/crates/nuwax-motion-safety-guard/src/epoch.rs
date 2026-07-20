//! Motion-guard epoch/provisioning schemas shared with the Python authority.

use serde::{Deserialize, Serialize};

/// Current guard epoch schema version.
pub const SCHEMA_VERSION_EPOCH: u32 = 2;

/// §P0B #15 provisioned.json schema version。
pub const SCHEMA_VERSION_PROVISIONED: u32 = 1;

/// §P0B #15 provisioned.json 默认路径(0400 mode)。
///
/// L-N0.1:仅常量,不实装文件 IO。
pub const PROVISIONED_PATH: &str = "/var/lib/nuwax/motion-guard/provisioned.json";

/// Runtime identities authorized by the v2 deployment.
pub const EXPECTED_USERS: [&str; 3] = ["nuwax-motion-guard", "nuwax-navigation", "nuwax-gateway"];

pub(crate) const V1_EXPECTED_USER_SETS: [[&str; 4]; 2] = [
    [
        "nuwax-motion-guard",
        "nuwax-motion-admin",
        "nuwax-navigation",
        "nuwax-gateway",
    ],
    [
        "nuwax-motion-guard",
        "nuwax-motion-admin",
        "nuwax-geometry",
        "nuwax-gateway",
    ],
];

/// §P0B #19 1 group,允许 nuwax-geometry / nuwax-gateway 调用 guard UDS。
///
/// 与 Python `EXPECTED_GROUPS` 1:1 对齐。
pub const EXPECTED_GROUPS: [&str; 1] = ["nuwax-motion-clients"];

/// Durable guard epoch schema.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GuardEpochRecord {
    /// schema 版本(必须为 `SCHEMA_VERSION_EPOCH`)
    pub schema_version: u32,
    /// 单调递增 generation
    pub generation: u64,
    /// boot nonce hex(32 chars = 128-bit);初始为空,首次 boot 后生成
    pub boot_nonce_hex: String,
    /// 当前 LeaseState
    pub lease_state: crate::state::LeaseState,
    /// 最近 zero ack id(字符串以兼容 UUID 或 u64)
    pub last_zero_ack_id: String,
    /// peer UID allowlist(必须为 `EXPECTED_USERS` 子集)
    pub peer_uid_allowlist: Vec<String>,
    /// actuator symbols sha256(以 "sha256:" 开头;W31 placeholder 形如
    /// `"sha256:placeholder"`)
    pub actuator_symbols_sha256: String,
    /// Whether prior actuator ownership is fenced.
    pub fenced: bool,
    /// Whether this process boot completed its automatic zero handshake.
    pub startup_zero_confirmed: bool,
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
            lease_state: crate::state::LeaseState::Idle,
            last_zero_ack_id: String::new(),
            peer_uid_allowlist: EXPECTED_USERS.iter().map(|s| s.to_string()).collect(),
            actuator_symbols_sha256: format!("sha256:{}", "0".repeat(64)),
            fenced: true,
            startup_zero_confirmed: false,
            created_at: "1970-01-01T00:00:00.000Z".to_string(),
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
        let digest = self
            .actuator_symbols_sha256
            .strip_prefix("sha256:")
            .unwrap_or_default();
        if digest.len() != 64
            || !digest
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
        {
            errs.push(format!(
                "actuator_symbols_sha256 必须是 lowercase SHA-256,got {:?}",
                self.actuator_symbols_sha256
            ));
        }
        let actual_users: Vec<&str> = self.peer_uid_allowlist.iter().map(String::as_str).collect();
        if actual_users.as_slice() != EXPECTED_USERS {
            errs.push("peer_uid_allowlist 必须精确匹配 current service identity tuple".to_string());
        }
        if self.created_at.trim().is_empty() {
            errs.push("created_at is required".to_string());
        }
        errs
    }

    /// Advance the process boot identity before automatic zero initialization.
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
            lease_state: crate::state::LeaseState::Idle,
            last_zero_ack_id: self.last_zero_ack_id.clone(),
            peer_uid_allowlist: EXPECTED_USERS
                .iter()
                .map(|user| (*user).to_string())
                .collect(),
            actuator_symbols_sha256: self.actuator_symbols_sha256.clone(),
            fenced: true,
            startup_zero_confirmed: false,
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

    /// Runtime UID tuple excludes the retired administrator identity.
    #[test]
    fn expected_users_groups_frozen() {
        assert_eq!(EXPECTED_USERS.len(), 3, "service UID 必须 3 项");
        for u in &["nuwax-motion-guard", "nuwax-navigation", "nuwax-gateway"] {
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
        assert!(!r.startup_zero_confirmed);
        assert!(r.boot_nonce_hex.is_empty());
        assert_eq!(r.lease_state, crate::state::LeaseState::Idle);
    }

    /// Boot advance rotates the nonce and remains fenced pending startup zero.
    #[test]
    fn boot_advance_updates_generation_and_nonce() {
        let r = GuardEpochRecord::default();
        let advanced = r.boot_advance();
        assert_eq!(advanced.generation, 1);
        assert!(advanced.fenced, "boot advance 后必须保持 fenced=true");
        assert!(!advanced.startup_zero_confirmed);
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

    #[test]
    fn verify_rejects_incomplete_peer_tuple() {
        let mut incomplete = GuardEpochRecord::default();
        incomplete.peer_uid_allowlist.pop();
        assert!(incomplete
            .verify()
            .iter()
            .any(|error| error.contains("精确匹配")));
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

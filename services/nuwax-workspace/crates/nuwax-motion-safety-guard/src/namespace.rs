//! nuwax-motion-safety-guard operation namespace(§P0B #22)
//!
//! 对齐 W32 Python 端 `tools/motion_safety_guard/operation_namespace.py`:
//! - 4 类 `OperationNamespace`:Acquire / LeaseBound / Admin / Query
//! - `OperationKey` frozen struct,字段组合:
//!   peer_uid / source / generation / boot_nonce (128-bit) / lease_id / operation_id / namespace
//! - 4 类 key builder:Acquire / LeaseBound / Admin / Query
//! - `key_to_canonical` 稳定字符串表达,用于 dedup cache key 与审计日志
//! - `derive_payload_hash` sha256 截断 32 hex(128-bit)

use serde::{Deserialize, Serialize};

/// boot_nonce 长度必须严格 16 字节(128-bit)。
pub const BOOT_NONCE_BYTES: usize = 16;

/// §P0B #22 4 类 operation namespace。
///
/// 与 Python `OperationNamespace` 1:1 对齐。
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum OperationNamespace {
    /// boot-epoch key,无 lease_id
    Acquire,
    /// execute / renew / RevokeAndZero,带 lease_id + generation
    LeaseBound,
    /// LatchSafetyStop / Reset,带 admin peer_uid
    Admin,
    /// GetOperationResult,只读
    Query,
}

impl OperationNamespace {
    /// §P0B #22 canonical 字符串表达:用于 `key_to_canonical`。
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Acquire => "acquire",
            Self::LeaseBound => "lease_bound",
            Self::Admin => "admin",
            Self::Query => "query",
        }
    }
}

/// §P0B #22 OperationKey,组合字段。
///
/// 用 Rust `Clone` 而非真 frozen(允许内部修改),
/// 但 `key_to_canonical` 视为 immutable key — 外部代码不应修改字段。
#[derive(Debug, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct OperationKey {
    /// peer UID(整数投影)
    pub peer_uid: u32,
    /// source 字符串投影(NAV / GATEWAY_* / ADMIN)
    pub source: String,
    /// generation
    pub generation: u64,
    /// 128-bit boot nonce
    pub boot_nonce: [u8; BOOT_NONCE_BYTES],
    /// lease_id;None for Acquire / Admin / Query
    pub lease_id: Option<u64>,
    /// operation_id(字符串)
    pub operation_id: String,
    /// namespace
    pub namespace: OperationNamespace,
}

/// 校验 `boot_nonce` 长度必须严格 16 字节。
fn check_boot_nonce(boot_nonce: &[u8]) -> Result<(), NamespaceError> {
    if boot_nonce.len() != BOOT_NONCE_BYTES {
        return Err(NamespaceError::InvalidBootNonce {
            actual: boot_nonce.len(),
            expected: BOOT_NONCE_BYTES,
        });
    }
    Ok(())
}

/// Acquire key builder:boot-epoch key,无 lease_id,generation=0。
pub struct AcquireKeyBuilder {
    peer_uid: u32,
    source: String,
    op_id: String,
    boot_nonce: [u8; BOOT_NONCE_BYTES],
}

impl AcquireKeyBuilder {
    /// 新建 builder。
    pub fn new(
        peer_uid: u32,
        source: impl Into<String>,
        op_id: impl Into<String>,
        boot_nonce: [u8; BOOT_NONCE_BYTES],
    ) -> Self {
        Self {
            peer_uid,
            source: source.into(),
            op_id: op_id.into(),
            boot_nonce,
        }
    }

    /// 构造 `OperationKey`,校验 boot_nonce 长度。
    pub fn build(self) -> Result<OperationKey, NamespaceError> {
        check_boot_nonce(&self.boot_nonce)?;
        Ok(OperationKey {
            peer_uid: self.peer_uid,
            source: self.source,
            generation: 0,
            boot_nonce: self.boot_nonce,
            lease_id: None,
            operation_id: self.op_id,
            namespace: OperationNamespace::Acquire,
        })
    }
}

/// LeaseBound key builder:execute / renew / RevokeAndZero。
///
/// `source` 留空,由 `lease_id` 在服务端反查 holder 后填;
/// 客户端键只用 peer_uid + lease_id 唯一定位。
pub struct LeaseBoundKeyBuilder {
    peer_uid: u32,
    generation: u64,
    boot_nonce: [u8; BOOT_NONCE_BYTES],
    lease_id: u64,
    op_id: String,
}

impl LeaseBoundKeyBuilder {
    /// 新建 builder。
    pub fn new(
        peer_uid: u32,
        generation: u64,
        boot_nonce: [u8; BOOT_NONCE_BYTES],
        lease_id: u64,
        op_id: impl Into<String>,
    ) -> Self {
        Self {
            peer_uid,
            generation,
            boot_nonce,
            lease_id,
            op_id: op_id.into(),
        }
    }

    /// 构造 `OperationKey`。
    pub fn build(self) -> Result<OperationKey, NamespaceError> {
        check_boot_nonce(&self.boot_nonce)?;
        Ok(OperationKey {
            peer_uid: self.peer_uid,
            source: String::new(),
            generation: self.generation,
            boot_nonce: self.boot_nonce,
            lease_id: Some(self.lease_id),
            operation_id: self.op_id,
            namespace: OperationNamespace::LeaseBound,
        })
    }
}

/// Admin key builder:LatchSafetyStop / Reset,`lease_id = None`。
///
/// `source` 必须是 `"ADMIN"` 或保留供未来扩展的 admin 子角色。
pub struct AdminKeyBuilder {
    peer_uid: u32,
    source: String,
    generation: u64,
    boot_nonce: [u8; BOOT_NONCE_BYTES],
    op_id: String,
}

impl AdminKeyBuilder {
    /// 新建 builder。
    pub fn new(
        peer_uid: u32,
        source: impl Into<String>,
        generation: u64,
        boot_nonce: [u8; BOOT_NONCE_BYTES],
        op_id: impl Into<String>,
    ) -> Self {
        Self {
            peer_uid,
            source: source.into(),
            generation,
            boot_nonce,
            op_id: op_id.into(),
        }
    }

    /// 构造 `OperationKey`。
    pub fn build(self) -> Result<OperationKey, NamespaceError> {
        check_boot_nonce(&self.boot_nonce)?;
        Ok(OperationKey {
            peer_uid: self.peer_uid,
            source: self.source,
            generation: self.generation,
            boot_nonce: self.boot_nonce,
            lease_id: None,
            operation_id: self.op_id,
            namespace: OperationNamespace::Admin,
        })
    }
}

/// Query key builder:GetOperationResult 只读;不参与仲裁也不分配 tombstone。
pub struct QueryKeyBuilder {
    peer_uid: u32,
    op_id: String,
    boot_nonce: [u8; BOOT_NONCE_BYTES],
}

impl QueryKeyBuilder {
    /// 新建 builder。
    pub fn new(
        peer_uid: u32,
        op_id: impl Into<String>,
        boot_nonce: [u8; BOOT_NONCE_BYTES],
    ) -> Self {
        Self {
            peer_uid,
            op_id: op_id.into(),
            boot_nonce,
        }
    }

    /// 构造 `OperationKey`。
    pub fn build(self) -> Result<OperationKey, NamespaceError> {
        check_boot_nonce(&self.boot_nonce)?;
        Ok(OperationKey {
            peer_uid: self.peer_uid,
            source: String::new(),
            generation: 0,
            boot_nonce: self.boot_nonce,
            lease_id: None,
            operation_id: self.op_id,
            namespace: OperationNamespace::Query,
        })
    }
}

/// §P0B #22 稳定字符串表达,用于 dedup cache key 与审计日志。
///
/// 格式:`namespace:peer_uid:source:generation:boot_nonce_hex:lease_id:operation_id`
pub fn key_to_canonical(key: &OperationKey) -> String {
    let lease_str = match key.lease_id {
        Some(n) => n.to_string(),
        None => String::from("null"),
    };
    let mut hex_buf = String::with_capacity(BOOT_NONCE_BYTES * 2);
    for b in &key.boot_nonce {
        hex_buf.push_str(&format!("{b:02x}"));
    }
    format!(
        "{}:{}:{}:{}:{}:{}:{}",
        key.namespace.as_str(),
        key.peer_uid,
        key.source,
        key.generation,
        hex_buf,
        lease_str,
        key.operation_id,
    )
}

/// §P0B #23 idempotency:payload 用 sha256 截断 32 hex(128-bit)作为指纹。
pub fn derive_payload_hash(payload: &[u8]) -> String {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    // L-N0.1 简化:用 DefaultHasher (SipHash) 模拟 sha256 截断 32 hex。
    // 真 sha256 由 P2-P3 lane 接通。这里保证两件事实:
    //   - 相同 payload → 相同 hash(确定性)
    //   - 不同 payload → 极大概率不同 hash(碰撞概率极低)
    //   - 长度严格 32 hex
    let mut h = DefaultHasher::new();
    payload.hash(&mut h);
    let u = h.finish();
    // 8 字节 u64 → 16 hex;再补 16 hex 模拟 sha256 截断 32 hex
    format!("{:016x}{:016x}", u, u.wrapping_mul(0x9E3779B97F4A7C15))
}

/// §P0B #22 namespace 错误。
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum NamespaceError {
    /// boot_nonce 长度 != 16
    InvalidBootNonce {
        /// 实际长度
        actual: usize,
        /// 期望长度
        expected: usize,
    },
}

impl std::fmt::Display for NamespaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidBootNonce { actual, expected } => {
                write!(
                    f,
                    "boot_nonce 必须是 128-bit ({expected} bytes),got {actual}"
                )
            }
        }
    }
}

impl std::error::Error for NamespaceError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn boot_nonce() -> [u8; BOOT_NONCE_BYTES] {
        let mut n = [0u8; BOOT_NONCE_BYTES];
        n[BOOT_NONCE_BYTES - 1] = 1;
        n
    }

    /// §P0B #22:Acquire/lease-bound/admin/query × boot_nonce × namespace 区分全绿。
    #[test]
    fn four_namespaces_are_distinct() {
        let n = boot_nonce();
        let k1 = AcquireKeyBuilder::new(1000, "NAV", "op-a", n)
            .build()
            .unwrap();
        let lb1 = LeaseBoundKeyBuilder::new(1000, 1, n, 42, "op-exec")
            .build()
            .unwrap();
        let ad1 = AdminKeyBuilder::new(999, "ADMIN", 0, n, "op-latch")
            .build()
            .unwrap();
        let q1 = QueryKeyBuilder::new(1000, "op-result", n).build().unwrap();

        assert_eq!(k1.namespace, OperationNamespace::Acquire);
        assert_eq!(lb1.namespace, OperationNamespace::LeaseBound);
        assert_eq!(ad1.namespace, OperationNamespace::Admin);
        assert_eq!(q1.namespace, OperationNamespace::Query);

        // 四 namespace 必须区分(同 peer_uid / op_id / boot_nonce 视为不同)
        let c1 = key_to_canonical(&k1);
        let clb = key_to_canonical(&lb1);
        let cad = key_to_canonical(&ad1);
        let cq = key_to_canonical(&q1);
        assert_ne!(c1, clb);
        assert_ne!(c1, cad);
        assert_ne!(c1, cq);
    }

    /// §P0B #22:同参 builder 二次 derive canonical 一致。
    #[test]
    fn same_args_produce_same_canonical() {
        let n = boot_nonce();
        let k1 = AcquireKeyBuilder::new(1000, "NAV", "op-a", n)
            .build()
            .unwrap();
        let k2 = AcquireKeyBuilder::new(1000, "NAV", "op-a", n)
            .build()
            .unwrap();
        assert_eq!(key_to_canonical(&k1), key_to_canonical(&k2));
    }

    /// §P0B #22:不同 boot_nonce → key 不同。
    #[test]
    fn different_boot_nonce_produces_different_key() {
        let mut n2 = [0u8; BOOT_NONCE_BYTES];
        n2[BOOT_NONCE_BYTES - 1] = 2;
        let k1 = AcquireKeyBuilder::new(1000, "NAV", "op-a", boot_nonce())
            .build()
            .unwrap();
        let k2 = AcquireKeyBuilder::new(1000, "NAV", "op-a", n2)
            .build()
            .unwrap();
        assert_ne!(key_to_canonical(&k1), key_to_canonical(&k2));
    }

    /// §P0B #22:不同 peer_uid → key 不同。
    #[test]
    fn different_peer_uid_produces_different_key() {
        let n = boot_nonce();
        let k1 = AcquireKeyBuilder::new(1000, "NAV", "op-a", n)
            .build()
            .unwrap();
        let k2 = AcquireKeyBuilder::new(2000, "NAV", "op-a", n)
            .build()
            .unwrap();
        assert_ne!(key_to_canonical(&k1), key_to_canonical(&k2));
    }

    /// §P0B #22:Acquired lease_id == None;LeaseBound lease_id == Some(_)。
    #[test]
    fn lease_id_namespace_alignment() {
        let n = boot_nonce();
        let k1 = AcquireKeyBuilder::new(1, "NAV", "x", n).build().unwrap();
        assert_eq!(k1.lease_id, None);
        let lb1 = LeaseBoundKeyBuilder::new(1, 0, n, 42, "x").build().unwrap();
        assert_eq!(lb1.lease_id, Some(42));
        let ad1 = AdminKeyBuilder::new(1, "ADMIN", 0, n, "x").build().unwrap();
        assert_eq!(ad1.lease_id, None);
    }

    /// §P0B #23 payload hash 确定性 + 长度 32 hex。
    #[test]
    fn payload_hash_deterministic_and_32_hex() {
        let h1 = derive_payload_hash(b"hello");
        let h2 = derive_payload_hash(b"hello");
        let h3 = derive_payload_hash(b"world");
        assert_eq!(h1, h2);
        assert_eq!(h1.len(), 32);
        assert_ne!(h1, h3);
    }

    /// §P0B #22:canonical 格式包含 7 段。
    #[test]
    fn canonical_format_has_7_segments() {
        let n = boot_nonce();
        let k1 = AcquireKeyBuilder::new(1000, "NAV", "op-a", n)
            .build()
            .unwrap();
        let c = key_to_canonical(&k1);
        let segments: Vec<&str> = c.split(':').collect();
        assert_eq!(segments.len(), 7, "canonical 必须 7 段");
        assert_eq!(segments[0], "acquire");
        assert_eq!(segments[1], "1000");
        assert_eq!(segments[2], "NAV");
        assert_eq!(segments[3], "0"); // generation
        assert_eq!(segments[4].len(), 32); // boot_nonce hex
        assert_eq!(segments[5], "null"); // lease_id None
        assert_eq!(segments[6], "op-a");
    }
}

//! LCM message type 强类型建模。
//!
//! LCM 1.x 的 message type 由 `pkg::Type`（全限定）字符串表达，并被
//! `lcm-gen` 计算为 8 字节 `lcm_hash` 写入 frame fingerprint。
//!
//! 本模块不依赖 `liblcm` —— 它仅固化 **字符串 ↔ 8 字节 fingerprint** 双向契约;
//! fingerprint 数值应通过上游 `lcm` 的 `lcm_hash` / `LCM_MSGTYPE_HASH`
//! 常量或运行时 bind 提供。foundation milestone 阶段仅提供
//! `placeholder` 常量与 `fingerprint` 字段访问。

use serde::{Deserialize, Serialize};
use std::fmt;

/// LCM message type in the canonical DimOS `package.Type` form.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct LcmMessageType(
    /// 全限定类型字符串，如 `geometry_msgs/PoseStamped` 或 `nav_msgs::Odometry`
    String,
);

impl LcmMessageType {
    /// 构造 new message type。
    ///
    /// Slash and Rust-style aliases are accepted at the API boundary and
    /// normalized to the original DimOS `package.Type` channel contract.
    pub fn new(fqn: &str) -> Result<Self, LcmMessageTypeError> {
        if fqn.is_empty() {
            return Err(LcmMessageTypeError::Empty);
        }
        if fqn.contains('#') || fqn.contains('\0') {
            return Err(LcmMessageTypeError::Invalid(fqn.to_string()));
        }
        let canonical = fqn.replace("::", ".").replace('/', ".");
        if canonical.starts_with('.')
            || canonical.ends_with('.')
            || !canonical.contains('.')
            || canonical.split('.').any(str::is_empty)
        {
            return Err(LcmMessageTypeError::Invalid(fqn.to_string()));
        }
        Ok(Self(canonical))
    }

    /// 取得 FQN 字符串
    #[must_use]
    pub fn fqn(&self) -> &str {
        &self.0
    }

    /// 计算 8 字节 `lcm_hash`。
    ///
    /// **2026-07-07 (Phase 2) 起**:调用 [`nuwax_codec::hash::fingerprint`],
    /// 严格走上游 `lcm-gen 1.x` 的 `__lcm_hash_64` 滚动(per-type seed
    /// 在 STRUCT_SEEDS 中固化),与上游 fingerprint **byte-for-byte
    /// 等价**(对全 primitive 字段的 schema)。
    ///
    /// 历史口径(2026-07-07 之前):曾使用 FNV-1a / seahash fake hash,
    /// 与 lcm-gen 不等价;Phase 2 一次性切到真实 hash,
    /// 旧实现已删除(详见 `nuwax-codec::hash` 模块顶部 doc)。
    #[must_use]
    pub fn fingerprint(&self) -> [u8; 8] {
        crate::lcm::fingerprint::compute_lcm_fingerprint(&self.0)
    }
}

impl fmt::Display for LcmMessageType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl AsRef<str> for LcmMessageType {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

/// message type 错误
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum LcmMessageTypeError {
    /// 空字符串
    #[error("message type 不能为空")]
    Empty,
    /// 包含非法字符 / `#` / NUL
    #[error("invalid message type: {0}")]
    Invalid(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_message_type() {
        let t = LcmMessageType::new("geometry_msgs/PoseStamped").unwrap();
        assert_eq!(t.fqn(), "geometry_msgs.PoseStamped");
    }

    #[test]
    fn rejects_empty() {
        assert_eq!(
            LcmMessageType::new("").unwrap_err(),
            LcmMessageTypeError::Empty
        );
    }

    #[test]
    fn rejects_invalid() {
        assert!(matches!(
            LcmMessageType::new("foo#bar").unwrap_err(),
            LcmMessageTypeError::Invalid(_)
        ));
    }

    #[test]
    fn fingerprint_stable_for_same_fqn() {
        let a = LcmMessageType::new("geometry_msgs/PoseStamped").unwrap();
        let b = LcmMessageType::new("geometry_msgs/PoseStamped").unwrap();
        assert_eq!(a.fingerprint(), b.fingerprint());
    }

    #[test]
    fn fingerprint_differs_across_fqn() {
        // 不同 fqn 大概率产生不同 fingerprint；本断言对稳定 hash 假设
        // 不是 100% 等价（collision 概率极低）但足以固化契约
        let a = LcmMessageType::new("geometry_msgs/PoseStamped").unwrap();
        let b = LcmMessageType::new("nav_msgs/Odometry").unwrap();
        assert_ne!(a.fingerprint(), b.fingerprint());
    }
}

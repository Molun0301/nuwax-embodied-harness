//! LCM topic 强类型建模（含 typed suffix 解析）。
//!
//! - [`LcmTopic`] = 纯 channel 字符串（如 `/odom`、`/frontier_goal`）。
//! - typed suffix 形式 `/topic#pkg.Msg` 由 [`LcmTopic::as_subscription_pattern`]
//!   在编译期显式拆分；类型标签不在 channel 字符串中冗余保存。

use serde::{Deserialize, Serialize};
use std::fmt;

/// typed-suffix 分隔符：与上游 LCM `lcm.subscribe("foo#pkg.Msg")` 一致
pub const LCM_TOPIC_TYPED_SUFFIX_SEP: char = '#';

/// LCM topic 字符串。
///
/// 仅承载 channel 字符串；typed suffix `#pkg.Msg` 通过
/// [`LcmTopic::as_subscription_pattern`] 拆分并以独立结构返回。
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct LcmTopic(
    /// 不含 `#pkg.Msg` 后缀的 channel 字符串
    String,
);

impl LcmTopic {
    /// 构造新 topic。
    ///
    /// LCM 1.x 允许任意非空 channel；本 crate 强约束以提升契约清晰度:
    /// - 必须以 `/` 开头；
    /// - 不允许内嵌 `#` 或 `\0`；
    /// - 长度不得为 0。
    pub fn new(channel: &str) -> Result<Self, LcmTopicParseError> {
        if channel.is_empty() {
            return Err(LcmTopicParseError::Empty);
        }
        if !channel.starts_with('/') {
            return Err(LcmTopicParseError::MissingLeadingSlash(channel.to_string()));
        }
        if channel.contains('#') {
            return Err(LcmTopicParseError::EmbeddedSuffixSeparator(
                channel.to_string(),
            ));
        }
        if channel.contains('\0') {
            return Err(LcmTopicParseError::NulByte(channel.to_string()));
        }
        Ok(Self(channel.to_string()))
    }

    /// 取得 channel 字符串（不含 typed suffix）
    #[must_use]
    pub fn channel(&self) -> &str {
        &self.0
    }

    /// 构造带 typed suffix 的 subscription pattern。
    ///
    /// 上游 LCM 习惯 `lcm.subscribe("/topic#pkg.Msg")` 一并接收 type 约束。
    /// 这里把 channel 与 type suffix 显式合并为字符串以方便 stack 调用。
    #[must_use]
    pub fn as_subscription_pattern(&self, msg_type: &str) -> String {
        format!("{}{}{}", self.0, LCM_TOPIC_TYPED_SUFFIX_SEP, msg_type)
    }

    /// 解析 typed-suffix subscription pattern。
    ///
    /// - `/scan` -> 仅 channel；
    /// - `/scan#sensor_msgs/PointCloud2` -> channel + type。
    pub fn parse_subscription_pattern(
        pattern: &str,
    ) -> Result<(Self, Option<String>), LcmTopicParseError> {
        if let Some(idx) = pattern.find(LCM_TOPIC_TYPED_SUFFIX_SEP) {
            let channel = &pattern[..idx];
            let msg_type = pattern[idx + LCM_TOPIC_TYPED_SUFFIX_SEP.len_utf8()..].to_string();
            let topic = Self::new(channel)?;
            Ok((topic, Some(msg_type)))
        } else {
            Ok((Self::new(pattern)?, None))
        }
    }

    /// 拼装 typed channel 全限定字符串 `/<channel>#<msg_type>`。
    ///
    /// 对齐 Python 侧 `lcmpubsub.py:36-72` 的 publish/subscribe channel 形式:
    /// 上游 liblcm 1.x 允许 `lcm.subscribe("/topic#pkg.Msg")` 一并指定 channel
    /// 与类型标签。本函数只负责字符串拼装,**不**做 STRUCT_SEEDS 校验
    /// (校验由 [`crate::LcmMessageType::new`] 在 msg_type 上承担)。
    ///
    /// # Arguments
    ///
    /// * `msg_type` — message FQN(如 `nav_msgs/Odometry`)。
    #[must_use]
    pub fn fqn(&self, msg_type: &str) -> String {
        self.as_subscription_pattern(msg_type)
    }

    /// 判定 `actual_channel` 是否匹配本 topic 期望的 typed-channel pattern。
    ///
    /// 匹配语义(对齐 Python `lcmpubsub.py:36-72` subscribe 端):
    /// - `actual == "/<channel>#<msg_type>"` -> 完全匹配(channel + suffix 都需命中);
    /// - `actual == "/<channel>"` (无 typed suffix) -> pattern 匹配
    ///   (即 `(#.*)?` 后缀可省略);
    /// - 其它情况 -> 不匹配。
    ///
    /// 该函数**不**做 msg_type 等价校验 —— typed suffix 部分仅作为
    /// "存在性" 过滤;`msg_type` 由 [`crate::LcmMessageType`] 在更上层校验。
    #[must_use]
    pub fn matches(&self, actual: &str) -> bool {
        // 拆出 channel 部分 + 可选 typed suffix
        let (actual_channel, _actual_suffix) = match actual.find(LCM_TOPIC_TYPED_SUFFIX_SEP) {
            Some(idx) => (&actual[..idx], Some(&actual[idx + 1..])),
            None => (actual, None),
        };
        // 永远只比较 channel 部分 —— typed suffix 仅作可选项存在,
        // 匹配语义与 Python lcmpubsub.subscribe("(/topic)(#.*)?") 等价。
        actual_channel == self.0
    }
}

impl fmt::Display for LcmTopic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl AsRef<str> for LcmTopic {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

/// topic parse 错误
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum LcmTopicParseError {
    /// 空字符串
    #[error("topic 不能为空")]
    Empty,
    /// 缺少前导 `/`
    #[error("topic 必须以 `/` 开头: {0}")]
    MissingLeadingSlash(String),
    /// 内嵌了 typed suffix 分隔符 `#`
    #[error("topic 不应包含 typed suffix 分隔符 `#`: {0}")]
    EmbeddedSuffixSeparator(String),
    /// NUL 字节
    #[error("topic 包含 NUL 字节: {0}")]
    NulByte(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_topic() {
        let t = LcmTopic::new("/scan").unwrap();
        assert_eq!(t.channel(), "/scan");
    }

    #[test]
    fn rejects_empty() {
        assert_eq!(LcmTopic::new("").unwrap_err(), LcmTopicParseError::Empty);
    }

    #[test]
    fn rejects_missing_slash() {
        assert_eq!(
            LcmTopic::new("scan").unwrap_err(),
            LcmTopicParseError::MissingLeadingSlash("scan".into())
        );
    }

    #[test]
    fn rejects_embedded_hash() {
        assert!(matches!(
            LcmTopic::new("/scan#foo").unwrap_err(),
            LcmTopicParseError::EmbeddedSuffixSeparator(_)
        ));
    }

    #[test]
    fn rejects_nul_byte() {
        assert!(matches!(
            LcmTopic::new("/scan\0").unwrap_err(),
            LcmTopicParseError::NulByte(_)
        ));
    }

    #[test]
    fn subscription_pattern_roundtrip() {
        let t = LcmTopic::new("/scan").unwrap();
        assert_eq!(
            t.as_subscription_pattern("sensor_msgs/PointCloud2"),
            "/scan#sensor_msgs/PointCloud2"
        );
        let (parsed, suffix) =
            LcmTopic::parse_subscription_pattern("/scan#sensor_msgs/PointCloud2").unwrap();
        assert_eq!(parsed, t);
        assert_eq!(suffix.as_deref(), Some("sensor_msgs/PointCloud2"));
    }

    #[test]
    fn subscription_pattern_no_suffix() {
        let (parsed, suffix) = LcmTopic::parse_subscription_pattern("/scan").unwrap();
        assert_eq!(parsed.channel(), "/scan");
        assert!(suffix.is_none());
    }
}

//! nuwax 共享类型与序列化基础
use serde::{Deserialize, Serialize};

pub type NuwaxUuid = uuid::Uuid;

/// Unix timestamp（秒）。
pub type UnixTs = i64;

/// nuwax 时间戳(UTC),LCM `std_msgs/Time` wire format 对齐。
///
/// 字段名/类型严格对齐 `dimos_lcm.std_msgs.Time` / lcm-gen 生成的 C/Rust 类型:
/// - `sec: i64` —— LCM `int64_t`
/// - `nsec: i32` —— LCM `int32_t`
///
/// 跨进程 LCM 互通(Python/Rust)字段顺序与字节布局必须保持一致。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Timestamp {
    /// Unix epoch 秒
    pub sec: i64,
    /// 纳秒分量
    pub nsec: i32,
}

impl Timestamp {
    /// 当前时刻
    #[must_use]
    pub fn now() -> Self {
        let now = chrono::Utc::now();
        Self {
            sec: now.timestamp(),
            nsec: now.timestamp_subsec_nanos() as i32,
        }
    }
}

/// 空 payload 占位。
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Empty;

/// 标准 Header,LCM `std_msgs/Header` wire format 对齐。
///
/// 字段顺序与类型严格对齐 `dimos_lcm.std_msgs.Header` / lcm-gen 生成的 C/Rust 类型:
/// - `seq: u32` —— LCM `int32_t` wire 占位(即使 Rust 端不主动递增,wire format 必须保留)
/// - `stamp: Timestamp` —— `Time(sec, nsec)`
/// - `frame_id: String`
///
/// 跨进程 LCM 互通(Python/Rust)字段顺序与字节布局必须保持一致。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    /// 序列号(LCM `int32_t` 占位,Rust 端默认 0;Python 端递增)
    pub seq: u32,
    /// 时间戳
    pub stamp: Timestamp,
    /// frame_id(坐标系名)
    pub frame_id: String,
}

impl Header {
    /// 构造默认值,`seq = 0`
    #[must_use]
    pub fn new(frame_id: impl Into<String>) -> Self {
        Self {
            seq: 0,
            stamp: Timestamp::now(),
            frame_id: frame_id.into(),
        }
    }

    /// 显式指定 seq 的构造(LCM 互通用)
    #[must_use]
    pub fn with_seq(seq: u32, frame_id: impl Into<String>) -> Self {
        Self {
            seq,
            stamp: Timestamp::now(),
            frame_id: frame_id.into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_serialize() {
        let e = Empty;
        let s = serde_json::to_string(&e).unwrap();
        assert_eq!(s, "null");
    }

    #[test]
    fn timestamp_now_recent() {
        let t = Timestamp::now();
        assert!(t.sec > 1_700_000_000); // 2023-11-14 之后
    }

    #[test]
    fn timestamp_field_names_match_lcm() {
        // 字段名与 lcm-gen 生成的 std_msgs/Time 一致:sec, nsec
        let t = Timestamp {
            sec: 1_700_000_000,
            nsec: 42,
        };
        let s = serde_json::to_string(&t).unwrap();
        assert!(s.contains("\"sec\":1700000000"));
        assert!(s.contains("\"nsec\":42"));
    }

    #[test]
    fn header_has_seq_field() {
        // LCM std_msgs/Header wire format 必须有 seq 字段
        let h = Header::new("base_link");
        assert_eq!(h.seq, 0);
        let h2 = Header::with_seq(42, "map");
        assert_eq!(h2.seq, 42);
        assert_eq!(h2.frame_id, "map");
    }

    #[test]
    fn header_field_order_lcm_compat() {
        // 字段顺序与 lcm-gen 生成的对齐:seq, stamp, frame_id
        let h = Header::new("base_link");
        let s = serde_json::to_string(&h).unwrap();
        let seq_idx = s.find("\"seq\"").unwrap();
        let stamp_idx = s.find("\"stamp\"").unwrap();
        let frame_idx = s.find("\"frame_id\"").unwrap();
        assert!(seq_idx < stamp_idx);
        assert!(stamp_idx < frame_idx);
    }
}

pub mod navigation;

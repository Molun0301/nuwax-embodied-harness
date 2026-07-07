//! nuwax 共享类型与序列化基础
use serde::{Deserialize, Serialize};

pub type NuwaxUuid = uuid::Uuid;

/// Unix timestamp（秒）。
pub type UnixTs = i64;

/// nuwax 时间戳（UTC）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Timestamp {
    /// Unix epoch 秒
    pub secs: UnixTs,
    /// 纳秒分量
    pub nanos: u32,
}

impl Timestamp {
    /// 当前时刻
    #[must_use]
    pub fn now() -> Self {
        let now = chrono::Utc::now();
        Self {
            secs: now.timestamp(),
            nanos: now.timestamp_subsec_nanos(),
        }
    }
}

/// 空 payload 占位。
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Empty;

/// 标准 Header。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    /// 时间戳
    pub stamp: Timestamp,
    /// frame_id（坐标系名）
    pub frame_id: String,
}

impl Header {
    /// 构造默认值
    #[must_use]
    pub fn new(frame_id: impl Into<String>) -> Self {
        Self {
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
        assert!(t.secs > 1_700_000_000); // 2023-11-14 之后
    }

    #[test]
    fn header_frame_id() {
        let h = Header::new("base_link");
        assert_eq!(h.frame_id, "base_link");
    }
}

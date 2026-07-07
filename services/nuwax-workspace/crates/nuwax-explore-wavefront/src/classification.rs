//! PointClassification:8-bit bitflags 复刻历史 Python `IntFlag` 语义(0/1/2/4/8)。

use bitflags::bitflags;
use serde::{Deserialize, Serialize};

bitflags! {
    /// 栅格点的分类标志(必须与历史 `IntFlag` 位值一致:0/1/2/4/8)。
    ///
    /// 重要:**NO_INFORMATION = 0**。检测 no-information 时必须用 `flags.is_empty()`
    /// 或 `flags == PointClassification::NO_INFORMATION`,**不能**用 `contains(NO_INFORMATION)`。
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct PointClassification: u8 {
        /// 无信息(位值 0,bitflags 不持有,需用 `is_empty()` 判定)
        const NO_INFORMATION  = 0;
        /// 地图开放
        const MAP_OPEN        = 1 << 0;
        /// 地图关闭
        const MAP_CLOSED      = 1 << 1;
        /// frontier 开放
        const FRONTIER_OPEN   = 1 << 2;
        /// frontier 关闭
        const FRONTIER_CLOSED = 1 << 3;
    }
}

impl Default for PointClassification {
    fn default() -> Self {
        Self::NO_INFORMATION
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_information_is_empty() {
        let f = PointClassification::NO_INFORMATION;
        assert!(f.is_empty());
        assert_eq!(f.bits(), 0);
    }

    #[test]
    fn bit_values_match_python_intflag() {
        // 历史 Python IntFlag 数值:NO_INFORMATION=0, MAP_OPEN=1, MAP_CLOSED=2, FRONTIER_OPEN=4, FRONTIER_CLOSED=8
        assert_eq!(PointClassification::MAP_OPEN.bits(), 1);
        assert_eq!(PointClassification::MAP_CLOSED.bits(), 2);
        assert_eq!(PointClassification::FRONTIER_OPEN.bits(), 4);
        assert_eq!(PointClassification::FRONTIER_CLOSED.bits(), 8);
    }

    #[test]
    fn bit_or_combines_flags() {
        let f = PointClassification::MAP_OPEN | PointClassification::FRONTIER_OPEN;
        assert!(f.contains(PointClassification::MAP_OPEN));
        assert!(f.contains(PointClassification::FRONTIER_OPEN));
        assert!(!f.contains(PointClassification::MAP_CLOSED));
    }

    #[test]
    fn serde_roundtrip_preserves_bits() {
        let f = PointClassification::MAP_CLOSED | PointClassification::FRONTIER_OPEN;
        let s = serde_json::to_string(&f).unwrap();
        let back: PointClassification = serde_json::from_str(&s).unwrap();
        assert_eq!(back, f);
    }

    #[test]
    fn serde_roundtrip_empty_serializes_empty_string() {
        // bitflags 2.x 的 default impl 把 empty flag 序列化为 `""` 字符串
        // (无 flag name);反序列化时回 `NO_INFORMATION` (is_empty = true)。
        let f = PointClassification::NO_INFORMATION;
        let s = serde_json::to_string(&f).unwrap();
        assert_eq!(s, "\"\"");
        let back: PointClassification = serde_json::from_str(&s).unwrap();
        assert!(back.is_empty());
    }
}

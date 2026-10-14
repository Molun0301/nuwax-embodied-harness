//! 坐标系标识
//!
//! `NavFrameId` 是导航板块统一的 frame 标识,与 `nuwax-tf` 配套使用。
//! 序列化使用 `snake_case`(例如 `"map"`、`"base_link"`),自定义 frame
//! 通过 `Custom(String)` 承载(避免给每个历史 frame 都开枚举变体)。

use serde::{Deserialize, Serialize};

/// 导航坐标系标识
///
/// - 常用 frame 直接是枚举变体,序列化输出字符串小写
/// - 其它/实验性 frame 通过 `Custom(String)` 透传
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum NavFrameId {
    /// 全局地图坐标系(`map`)
    #[default]
    Map,
    /// 里程计坐标系(`odom`)
    Odom,
    /// 机器人底盘坐标系(`base_link`)
    Base,
    /// 机器人本体坐标系(`base_footprint`)
    Body,
    /// 相机坐标系(`camera_*` / `camera_optical_frame`)
    Camera,
    /// 自定义 frame,字符串透传
    Custom(String),
}

impl NavFrameId {
    /// 返回 frame 字符串表示
    #[must_use]
    pub fn as_str(&self) -> &str {
        match self {
            Self::Map => "map",
            Self::Odom => "odom",
            Self::Base => "base_link",
            Self::Body => "base_footprint",
            Self::Camera => "camera_optical_frame",
            Self::Custom(s) => s.as_str(),
        }
    }

    /// 由字符串解析,空字符串返回 `Custom(String::new())`
    #[must_use]
    pub fn from_static(s: &str) -> Self {
        match s {
            "map" => Self::Map,
            "odom" => Self::Odom,
            "base_link" => Self::Base,
            "base_footprint" => Self::Body,
            "camera_optical_frame" => Self::Camera,
            other => Self::Custom(other.to_string()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn static_frame_str_roundtrip() {
        for f in [
            NavFrameId::Map,
            NavFrameId::Odom,
            NavFrameId::Base,
            NavFrameId::Body,
            NavFrameId::Camera,
        ] {
            assert_eq!(NavFrameId::from_static(f.as_str()), f);
        }
    }

    #[test]
    fn custom_frame_passthrough() {
        let f = NavFrameId::Custom("foo_frame".to_string());
        assert_eq!(f.as_str(), "foo_frame");
        assert_eq!(NavFrameId::from_static("foo_frame"), f);
    }

    #[test]
    fn serde_snake_case_roundtrip() {
        let f = NavFrameId::Base;
        let s = serde_json::to_string(&f).unwrap();
        assert!(s.contains("\"base\""));
        let back: NavFrameId = serde_json::from_str(&s).unwrap();
        assert_eq!(back, f);
    }

    #[test]
    fn serde_custom_string_shape() {
        let f = NavFrameId::Custom("wheel_odom".to_string());
        let s = serde_json::to_string(&f).unwrap();
        assert!(s.contains("wheel_odom"));
        let back: NavFrameId = serde_json::from_str(&s).unwrap();
        assert_eq!(back, f);
    }
}

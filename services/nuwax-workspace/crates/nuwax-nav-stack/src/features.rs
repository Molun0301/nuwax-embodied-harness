// Copyright 2026 Nuwax.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! 可选行为开关。
//!
//! 与 Cargo features 形成双重门控:
//! - **Cargo feature**:控制能否 *编译进* crate(防止编译期引入不需要的依赖)。
//! - **`NavFeatureFlags`**:控制运行时是否 *启用* 某行为(可视化、replay、record 等)。
//!
//! 这种分层的好处:
//! - 默认 `cargo build` 不会拉入任何 visualizer / rerun / foxglove / record / explore 依赖。
//! - 即使所有 Cargo feature 都关闭,`NavStackBuilder` 仍可 dry-run,只是实际启动能力受限。

use serde::{Deserialize, Serialize};

bitflags::bitflags! {
    /// 运行时可选行为位。
    ///
    /// 注意:本类型为 *运行时* 开关,即便全部位都置 1,
    /// 实际生效仍需要对应的 Cargo feature 也已启用(编译期门控)。
    #[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct NavFeatureFlags: u32 {
        /// 启用 replay/smoke 入口(rosbag fixture 驱动 stack)。
        const REPLAY          = 1 << 0;
        /// 启用可视化模块(rerun / foxglove 等)。
        const VIEWER          = 1 << 1;
        /// 启用导航 topic 录制 profile(必须配合 `record` Cargo feature)。
        const RECORD          = 1 << 2;
        /// 启用 MLS 3D pipeline(必须配合 `mls` Cargo feature)。
        const MLS             = 1 << 3;
        /// 启用探索目标源(必须配合 `explore` Cargo feature)。
        const EXPLORE         = 1 << 4;
        /// 启用 click-goal 路由(交互式 UI 点击)。
        const CLICK_GOAL      = 1 << 5;
        /// 启用 patrolling 行为层。
        const PATROLLING      = 1 << 6;
        /// 启用 bbox tracker / visual servoing 链路。
        const VISUAL          = 1 << 7;
        /// 启用 movement manager(teleop mux + click relay)。
        const MOVEMENT        = 1 << 8;
        /// 启用 basic-path-follower(Rust fallback controller)。
        const BASIC_FOLLOWER  = 1 << 9;
    }
}

impl NavFeatureFlags {
    /// 全部禁用(用于 dry-run / 单元测试)。
    pub const fn none() -> Self {
        Self::empty()
    }

    /// 全部启用(用于 dev/集成测试,但仍受 Cargo feature gate 约束)。
    ///
    /// 命名上避免与 `bitflags!` 自动生成的 `all()`(返回 "all bits set" 即 u32::MAX)
    /// 冲突。`bitflags!` 已经在 Self 上生成了 `all()` 方法,直接同名覆盖会导致歧义。
    pub fn all_enabled() -> Self {
        Self::REPLAY
            | Self::VIEWER
            | Self::RECORD
            | Self::MLS
            | Self::EXPLORE
            | Self::CLICK_GOAL
            | Self::PATROLLING
            | Self::VISUAL
            | Self::MOVEMENT
            | Self::BASIC_FOLLOWER
    }
}

impl Serialize for NavFeatureFlags {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        // 位标志序列化为 kebab-case 字符串列表,便于 dry-run / JSON 输出。
        // bitflags 宏生成的 `iter_names()` 返回常量定义时写的大写名,
        // 这里手动映射为 kebab-case。
        use serde::ser::SerializeSeq;
        let names: Vec<&'static str> = self
            .iter_names()
            .map(|(name, _)| flag_to_kebab(name))
            .collect();
        let mut seq = s.serialize_seq(Some(names.len()))?;
        for n in names {
            seq.serialize_element(n)?;
        }
        seq.end()
    }
}

/// 把 bitflags 常量名(全大写下划线)转换为 kebab-case 字符串。
///
/// 用于序列化、dry-run 输出与 CLI preset 字符串对齐。
pub fn flag_to_kebab(name: &str) -> &'static str {
    match name {
        "REPLAY" => "replay",
        "VIEWER" => "viewer",
        "RECORD" => "record",
        "MLS" => "mls",
        "EXPLORE" => "explore",
        "CLICK_GOAL" => "click-goal",
        "PATROLLING" => "patrolling",
        "VISUAL" => "visual",
        "MOVEMENT" => "movement",
        "BASIC_FOLLOWER" => "basic-follower",
        _ => "unknown",
    }
}

impl<'de> Deserialize<'de> for NavFeatureFlags {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        use serde::de::Error;
        let names: Vec<String> = Vec::deserialize(d)?;
        let mut out = NavFeatureFlags::empty();
        for name in names {
            let flag = match name.as_str() {
                "replay" => NavFeatureFlags::REPLAY,
                "viewer" => NavFeatureFlags::VIEWER,
                "record" => NavFeatureFlags::RECORD,
                "mls" => NavFeatureFlags::MLS,
                "explore" => NavFeatureFlags::EXPLORE,
                "click-goal" => NavFeatureFlags::CLICK_GOAL,
                "patrolling" => NavFeatureFlags::PATROLLING,
                "visual" => NavFeatureFlags::VISUAL,
                "movement" => NavFeatureFlags::MOVEMENT,
                "basic-follower" => NavFeatureFlags::BASIC_FOLLOWER,
                other => {
                    return Err(D::Error::custom(format!(
                        "unknown nav feature flag: `{other}`"
                    )));
                }
            };
            out |= flag;
        }
        Ok(out)
    }
}

/// 常用 feature 预设。
///
/// 这些常量对应 `--preset=<name>` CLI 参数。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum NavStackPreset {
    /// 全部关闭。
    None,
    /// Simple2D 默认(feature-gated)。
    Simple,
    /// CMU native 默认。
    Cmu,
    /// 探索模式(Cmu + explore)。
    Explore,
    /// MLS 3D 模式。
    Mls,
    /// 全部启用(开发期 / 集成测试,仍受 Cargo feature gate 约束)。
    All,
}

impl NavStackPreset {
    /// 转换为对应 `NavFeatureFlags` 位。
    pub fn to_flags(self) -> NavFeatureFlags {
        match self {
            NavStackPreset::None => NavFeatureFlags::none(),
            NavStackPreset::Simple => NavFeatureFlags::BASIC_FOLLOWER,
            NavStackPreset::Cmu => NavFeatureFlags::BASIC_FOLLOWER | NavFeatureFlags::MOVEMENT,
            NavStackPreset::Explore => {
                NavFeatureFlags::BASIC_FOLLOWER
                    | NavFeatureFlags::MOVEMENT
                    | NavFeatureFlags::EXPLORE
            }
            NavStackPreset::Mls => NavFeatureFlags::BASIC_FOLLOWER | NavFeatureFlags::MLS,
            NavStackPreset::All => NavFeatureFlags::all_enabled(),
        }
    }

    /// CLI 字符串解析。
    pub fn from_cli_str(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "none" => Some(NavStackPreset::None),
            "simple" => Some(NavStackPreset::Simple),
            "cmu" => Some(NavStackPreset::Cmu),
            "explore" | "exploration" => Some(NavStackPreset::Explore),
            "mls" => Some(NavStackPreset::Mls),
            "all" => Some(NavStackPreset::All),
            _ => None,
        }
    }

    /// 规范名称。
    pub fn as_str(&self) -> &'static str {
        match self {
            NavStackPreset::None => "none",
            NavStackPreset::Simple => "simple",
            NavStackPreset::Cmu => "cmu",
            NavStackPreset::Explore => "explore",
            NavStackPreset::Mls => "mls",
            NavStackPreset::All => "all",
        }
    }
}

impl std::fmt::Display for NavStackPreset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn none_is_empty() {
        assert!(NavFeatureFlags::none().is_empty());
    }

    #[test]
    fn preset_simple_enables_basic_follower() {
        let f = NavStackPreset::Simple.to_flags();
        assert!(f.contains(NavFeatureFlags::BASIC_FOLLOWER));
        assert!(!f.contains(NavFeatureFlags::MLS));
    }

    #[test]
    fn preset_all_enables_everything() {
        let f = NavStackPreset::All.to_flags();
        assert_eq!(f, NavFeatureFlags::all_enabled());
    }

    #[test]
    fn flags_serialize_to_kebab_case() {
        let f = NavFeatureFlags::REPLAY | NavFeatureFlags::MLS;
        let s = serde_json::to_string(&f).unwrap();
        // 位标志序列化为数组
        assert!(s.contains("\"replay\""));
        assert!(s.contains("\"mls\""));
    }
}

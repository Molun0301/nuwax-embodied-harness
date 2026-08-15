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

//! Stack profile 枚举。
//!
//! 方案 §6.8 规定 5 个 profile:
//!
//! - `Simple2D`:基于 `nuwax-nav-simple-planner`，经 `guarded_nav_scheduler`
//!   内部 `LocalPlanner` 接入 Motion Guard 的轻量纯 Rust pipeline。
//! - `CmuNative`:基于 FAR + Local + PathFollower + PGO + `nuwax-nav-terrain-map-ext`
//!   的 CMU stack,使用 native C++ bridge。
//! - `ReplanningAStar`:基于 `nuwax-nav-astar` 的纯 Rust replanning pipeline。
//! - `ExplorationNative`:在 CmuNative 之上叠加 TARE / Wavefront 探索目标源。
//! - `Mls3D`:基于 `nuwax-nav-mls-planner` 的 3D MLS pipeline。

use serde::{Deserialize, Serialize};

use crate::error::{NavStackError, NavStackResult};

/// Stack profile 标识。
///
/// 字符串表示与 `--profile=<NAME>` CLI 参数对齐。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum NavStackProfile {
    /// 纯 Rust 轻量 2D pipeline。
    Simple2D,
    /// CMU 原生 native pipeline(FAR + Local + PathFollower + PGO)。
    CmuNative,
    /// 纯 Rust replanning A* pipeline。
    ReplanningAStar,
    /// CMU 原生 pipeline + 探索目标源(TARE / wavefront)。
    ExplorationNative,
    /// 3D MLS planner pipeline。
    Mls3D,
}

impl NavStackProfile {
    /// 全部 profile(用于 `--list-features` 与单元测试)。
    pub fn all() -> &'static [NavStackProfile] {
        &[
            NavStackProfile::Simple2D,
            NavStackProfile::CmuNative,
            NavStackProfile::ReplanningAStar,
            NavStackProfile::ExplorationNative,
            NavStackProfile::Mls3D,
        ]
    }

    /// 从 CLI 字符串解析。
    ///
    /// 接受 `"simple-2d"` / `"cmu-native"` / `"replanning-astar"`
    /// / `"exploration-native"` / `"mls-3d"`,也接受兼容大小写。
    pub fn from_cli_str(s: &str) -> NavStackResult<Self> {
        let normalized = s.trim().to_ascii_lowercase().replace('_', "-");
        match normalized.as_str() {
            "simple-2d" | "simple2d" | "simple" => Ok(NavStackProfile::Simple2D),
            "cmu-native" | "cmunative" | "cmu" => Ok(NavStackProfile::CmuNative),
            "replanning-astar" | "replanningastar" | "astar" => {
                Ok(NavStackProfile::ReplanningAStar)
            }
            "exploration-native" | "explorationnative" | "exploration" => {
                Ok(NavStackProfile::ExplorationNative)
            }
            "mls-3d" | "mls3d" | "mls" => Ok(NavStackProfile::Mls3D),
            other => Err(NavStackError::invalid(format!(
                "unknown profile: `{other}` (allowed: simple-2d, cmu-native, \
                 replanning-astar, exploration-native, mls-3d)"
            ))),
        }
    }

    /// 规范的 kebab-case 名称。
    pub fn as_str(&self) -> &'static str {
        match self {
            NavStackProfile::Simple2D => "simple-2d",
            NavStackProfile::CmuNative => "cmu-native",
            NavStackProfile::ReplanningAStar => "replanning-astar",
            NavStackProfile::ExplorationNative => "exploration-native",
            NavStackProfile::Mls3D => "mls-3d",
        }
    }

    /// profile 简短描述,供 dry-run 与文档输出。
    pub fn description(&self) -> &'static str {
        match self {
            NavStackProfile::Simple2D => {
                "纯 Rust 轻量 2D pipeline(simple_planner -> guarded_nav_scheduler（内部 LocalPlanner）-> Motion Guard)"
            }
            NavStackProfile::CmuNative => {
                "CMU 原生 native pipeline(FAR + Local + PathFollower + PGO + terrain-map-ext)"
            }
            NavStackProfile::ReplanningAStar => {
                "纯 Rust replanning A* pipeline(astar + local + basic-path-follower)"
            }
            NavStackProfile::ExplorationNative => {
                "探索目标源增强的 CMU pipeline(在 cmu-native 基础上叠加 TARE / wavefront)"
            }
            NavStackProfile::Mls3D => "3D MLS planner pipeline(mls-planner + binding)",
        }
    }
}

impl std::fmt::Display for NavStackProfile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl std::str::FromStr for NavStackProfile {
    type Err = NavStackError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::from_cli_str(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_all_profiles() {
        for p in NavStackProfile::all() {
            let s = p.as_str();
            let parsed = NavStackProfile::from_cli_str(s).expect("parse");
            assert_eq!(*p, parsed);
        }
    }

    #[test]
    fn simple2d_description_names_canonical_guarded_chain() {
        let description = NavStackProfile::Simple2D.description();
        assert!(description.contains("simple_planner"));
        assert!(description.contains("guarded_nav_scheduler"));
        assert!(description.contains("LocalPlanner"));
        assert!(description.contains("Motion Guard"));
        assert!(!description.contains("basic-path-follower"));
    }

    #[test]
    fn accepts_aliases() {
        assert_eq!(
            NavStackProfile::from_cli_str("cmu").unwrap(),
            NavStackProfile::CmuNative
        );
        assert_eq!(
            NavStackProfile::from_cli_str("ASTAR").unwrap(),
            NavStackProfile::ReplanningAStar
        );
        assert_eq!(
            NavStackProfile::from_cli_str("Simple2D").unwrap(),
            NavStackProfile::Simple2D
        );
    }

    #[test]
    fn rejects_unknown() {
        let err = NavStackProfile::from_cli_str("not-a-profile").unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("unknown profile"));
    }
}

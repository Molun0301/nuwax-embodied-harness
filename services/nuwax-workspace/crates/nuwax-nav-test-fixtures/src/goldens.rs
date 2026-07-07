//! Synthetic golden fixtures for bridge integration tests
//!
//! 设计:每个 bridge 有一份 fixture JSON,描述
//! - `topics`:topic 名映射(bridge 输入/输出)
//! - `expected_argv_contains`:argv 必须包含的关键 token(按出现位置检查)
//! - `expected_argv_pairs`:argv 中必须以 `--key value` 形式出现的相邻对
//! - `expected_topics_count`:至少期望的 topic 数量
//!
//! 不依赖真实 rosbag / C++ binary;bridge 测试只把 fixture 灌进 bridge 配置,
//! 然后断言 argv 渲染结果与 fixture 期望一致 — 确定性、可重放、零外部依赖。

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::PathBuf;

/// 单个 bridge 的 golden fixture。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct BridgeGoldenFixture {
    /// fixture 名称(用于测试 case 名)
    pub name: String,
    /// 桥接类型(far_planner / local_planner / path_follower / pgo)
    pub bridge: String,
    /// 输入 topic 映射(物理 topic 名 → bridge port 名)
    #[serde(default)]
    pub topics: BTreeMap<String, String>,
    /// 配置覆盖(可空;空表示用 bridge 默认值)
    #[serde(default)]
    pub config_overrides: BTreeMap<String, serde_json::Value>,
    /// argv 中必须出现的字符串 token(任意顺序)
    #[serde(default)]
    pub expected_argv_contains: Vec<String>,
    /// argv 中必须以 `--key value` 相邻形式出现的 pair
    #[serde(default)]
    pub expected_argv_pairs: Vec<(String, String)>,
    /// 至少期望的 topic 数量(BTreeMap size;若 topics 为空则跳过)
    #[serde(default)]
    pub expected_topics_count: usize,
}

/// 加载指定名称的 golden fixture(从本 crate 的 `goldens/<name>.json`)
///
/// 路径解析:`CARGO_MANIFEST_DIR` 是调用方 crate 的 manifest 目录(4 个 bridge
/// crate 的位置各不相同),而 golden 文件只存在于本 crate 目录下的 `goldens/`。
/// 因此从 caller 的 manifest_dir 上溯两级到 workspace 根 `crates/`,再定位到
/// `nuwax-nav-test-fixtures/goldens/<name>.json`。
pub fn load(name: &str) -> BridgeGoldenFixture {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_else(|_| ".".to_string());
    // caller 形如 `<workspace>/crates/<bridge-crate>`,上溯两级得到 workspace 根
    let path = PathBuf::from(manifest_dir)
        .parent() // <workspace>/crates
        .and_then(|p| p.parent()) // <workspace>
        .unwrap_or_else(|| {
            panic!("golden fixture: cannot resolve workspace root from CARGO_MANIFEST_DIR")
        })
        .join("crates")
        .join("nuwax-nav-test-fixtures")
        .join("goldens")
        .join(format!("{name}.json"));
    let raw = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("read golden fixture {}: {e}", path.display()));
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse golden fixture {}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_far_planner_golden() {
        let g = load("far_planner_smoke");
        assert_eq!(g.bridge, "far_planner");
        assert!(g.expected_argv_contains.iter().any(|s| s == "/odom"));
        assert!(g
            .expected_argv_pairs
            .iter()
            .any(|(k, v)| k == "--robot_dim" && v == "0.5"));
    }

    #[test]
    fn load_local_planner_golden() {
        let g = load("local_planner_smoke");
        assert_eq!(g.bridge, "local_planner");
        assert!(g
            .expected_argv_pairs
            .iter()
            .any(|(k, v)| k == "--maxSpeed" && v == "0.75"));
    }

    #[test]
    fn load_pgo_golden() {
        let g = load("pgo_smoke");
        assert_eq!(g.bridge, "pgo");
        assert!(g
            .expected_argv_pairs
            .iter()
            .any(|(k, v)| k == "--world_frame" && v == "map"));
    }
}

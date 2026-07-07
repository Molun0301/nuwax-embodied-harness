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

//! Replay / smoke test 入口(占位实现)。
//!
//! 真实 replay 会在后续 phase 中由各 upstream crate 的 rosbag fixture 提供。
//! 本模块定义最小可执行的接口与错误类型,保证 CLI 在没有 fixture 时也能给出
//! 有意义的诊断信息。
//!
//! 设计要点:
//! - 不在 build.rs / runtime 中下载 fixture。
//! - fixture 必须由用户在 `--replay <path>` 时显式指定。
//! - 仅检查路径存在 + JSON 可解析,不假设具体 rosbag 格式(后续可由 foundation crate
//!   提供解析能力)。

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::error::{NavStackError, NavStackResult};

/// Fixture manifest(最小可用字段)。
///
/// 真实 fixture 由 foundation lane 的 `nuwax-nav-test-fixtures` 提供;这里定义
/// 通用 envelope,以便 dry-run 输出说明本 fixture 期望的 topic / 时间窗口。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReplayFixture {
    /// fixture 名称。
    pub name: String,
    /// fixture 来源(例如 `"rosbag:2024-05-cmu-walk"`)。
    pub source: String,
    /// 期望的 topic 列表。
    pub topics: Vec<String>,
    /// 时间窗口纳秒。
    pub time_window_ns: (i64, i64),
    /// 可选 topic manifest(每个 topic 的 schema/type,用于 replay 阶段做字段级校验)。
    /// 缺失时仅做 topic 名 presence check。
    #[serde(default)]
    pub topic_manifest: Vec<TopicManifestEntry>,
}

/// 单个 topic 在 fixture manifest 中的元数据。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TopicManifestEntry {
    /// topic 名(与 `ReplayFixture::topics` 一致)。
    pub topic: String,
    /// 消息 schema 名(例如 `"geometry_msgs/PoseStamped"`、`"nuwax/Costmap"`)。
    pub schema: String,
    /// 可选频率提示(Hz);缺失则为 0。
    #[serde(default)]
    pub rate_hz: f32,
}

/// 解析 fixture manifest。
///
/// 期望 JSON 格式,字段对应 `ReplayFixture`。
pub fn parse_fixture(path: &Path) -> NavStackResult<ReplayFixture> {
    if !path.exists() {
        return Err(NavStackError::PathNotFound(path.to_path_buf()));
    }
    let bytes = std::fs::read(path)?;
    let fx: ReplayFixture = serde_json::from_slice(&bytes)?;
    Ok(fx)
}

/// 校验 `topic_manifest` 与 `topics` 一致:
///
/// - `topic_manifest` 的每个条目都必须存在于 `topics` 中
/// - `topic_manifest` 至少覆盖一个 topic 才算有效
/// - `topics` 中的每个条目都应当至少有一条 manifest 描述(可选但推荐)
pub fn validate_topic_manifest(fixture: &ReplayFixture) -> NavStackResult<()> {
    if fixture.topic_manifest.is_empty() {
        // 没有 manifest 时不做严格校验,但要求 topics 非空(已在调用方保证)
        return Ok(());
    }
    // manifest 中的 topic 必须存在于 topics
    for entry in &fixture.topic_manifest {
        if !fixture.topics.iter().any(|t| t == &entry.topic) {
            return Err(NavStackError::Replay(format!(
                "topic_manifest entry `{}` is not declared in fixture.topics",
                entry.topic
            )));
        }
    }
    // topics 中的每个条目都应至少有一条 manifest 描述
    for t in &fixture.topics {
        if !fixture.topic_manifest.iter().any(|m| &m.topic == t) {
            return Err(NavStackError::Replay(format!(
                "fixture topic `{t}` has no entry in topic_manifest"
            )));
        }
    }
    Ok(())
}

/// Smoke test run 占位。
///
/// 后续会接入真实 replay 引擎;目前只做 sanity check + topic_manifest 校验。
pub fn run_smoke(fixture: PathBuf) -> NavStackResult<()> {
    let fx = parse_fixture(&fixture)?;
    if fx.topics.is_empty() {
        return Err(NavStackError::Replay(
            "fixture has no topics declared".to_string(),
        ));
    }
    validate_topic_manifest(&fx)?;
    // 真实 replay 由各 upstream crate 在后续 phase 提供。
    // 这里仅做日志记录,保证 CLI 不假报"成功"。
    eprintln!(
        "[nav-stack smoke] fixture `{}` parsed: {} topics, {} manifest entries, time window {:?}",
        fx.name,
        fx.topics.len(),
        fx.topic_manifest.len(),
        fx.time_window_ns
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_fixture_returns_path_error() {
        let err = parse_fixture(Path::new("/non/existent/path.json")).unwrap_err();
        assert!(matches!(err, NavStackError::PathNotFound(_)));
    }

    #[test]
    fn empty_topics_returns_replay_error() {
        // write temp fixture
        let dir = std::env::temp_dir().join("nuwax-nav-stack-test");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("empty.json");
        std::fs::write(
            &path,
            r#"{
                "name": "empty",
                "source": "unit-test",
                "topics": [],
                "time_window_ns": [0, 0]
            }"#,
        )
        .unwrap();
        let err = run_smoke(path).unwrap_err();
        assert!(matches!(err, NavStackError::Replay(_)));
    }
}

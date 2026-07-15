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

//! Topic / frame 配置。
//!
//! 桥接 bridge 与 native C++ 时,topic 名会作为 `--<port> <topic>` 传入命令行。
//! 本结构集中维护所有可配置字段,避免散落字符串。
//!
//! 命名约定:
//! - 字段使用 `snake_case`(serde 默认)。
//! - 缺省值与原 Python `create_cmu_nav` 命名保持一致(`corrected_odometry` 等),
//!   但允许外部覆盖。
//! - 不在模块内部硬编码 `/stop` 之类的全局路径前缀。

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// Topic 名字符串(语义上非空,但允许运行时再决定)。
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct TopicName(pub String);

impl TopicName {
    /// 构造一个 topic 名称。
    pub fn new(name: impl Into<String>) -> Self {
        Self(name.into())
    }

    /// 取出字符串引用。
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for TopicName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl From<&str> for TopicName {
    fn from(s: &str) -> Self {
        Self(s.to_owned())
    }
}

impl From<String> for TopicName {
    fn from(s: String) -> Self {
        Self(s)
    }
}

/// 集中维护所有 topic / frame 名称。
///
/// 字段以 BTreeMap 形式存储,以保证序列化确定性(dry-run 输出稳定)。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NavTopicConfig {
    /// 内部以 `BTreeMap<"port_name" -> TopicName>` 存储。
    pub topics: BTreeMap<String, TopicName>,
    /// world frame 名,默认 `"map"`。
    #[serde(default = "default_world_frame")]
    pub world_frame: String,
    /// robot base frame 名,默认 `"base_link"`。
    #[serde(default = "default_base_frame")]
    pub base_frame: String,
    /// sensor frame 名,默认 `"sensor"`。
    #[serde(default = "default_sensor_frame")]
    pub sensor_frame: String,
}

fn default_world_frame() -> String {
    "map".to_owned()
}
fn default_base_frame() -> String {
    "base_link".to_owned()
}
fn default_sensor_frame() -> String {
    "sensor".to_owned()
}

impl NavTopicConfig {
    /// 使用 CMU stack 默认值构造(尽量贴近 `create_cmu_nav` 命名约定)。
    pub fn cmu_defaults() -> Self {
        let mut topics = BTreeMap::new();
        // 输入
        topics.insert("registered_scan".into(), TopicName::new("registered_scan"));
        topics.insert("occupancy".into(), TopicName::new("occupancy"));
        topics.insert("terrain_map".into(), TopicName::new("terrain_map"));
        topics.insert("terrain_map_ext".into(), TopicName::new("terrain_map_ext"));
        topics.insert("odometry".into(), TopicName::new("odometry"));
        topics.insert("joy_cmd".into(), TopicName::new("joy_cmd"));
        topics.insert("clicked_point".into(), TopicName::new("clicked_point"));
        topics.insert("speed".into(), TopicName::new("speed"));
        topics.insert("safety_stop".into(), TopicName::new("safety_stop"));
        topics.insert("slow_down".into(), TopicName::new("slow_down"));
        topics.insert("added_obstacles".into(), TopicName::new("added_obstacles"));
        topics.insert("check_obstacle".into(), TopicName::new("check_obstacle"));
        topics.insert("cancel_goal".into(), TopicName::new("cancel_goal"));
        topics.insert("goal".into(), TopicName::new("goal"));
        topics.insert("patrol_enabled".into(), TopicName::new("patrol_enabled"));
        topics.insert("goal_failed".into(), TopicName::new("goal_failed"));
        topics.insert(
            "patrol_planner_control".into(),
            TopicName::new("patrol_planner_control"),
        );
        topics.insert(
            "patrol_planner_control_ack".into(),
            TopicName::new("patrol_planner_control_ack"),
        );
        topics.insert("stop_movement".into(), TopicName::new("stop_movement"));
        topics.insert("path".into(), TopicName::new("path"));
        // 输出
        topics.insert("nav_cmd_vel".into(), TopicName::new("/nav/cmd_vel"));
        topics.insert("cmd_vel".into(), TopicName::new("cmd_vel"));
        topics.insert("way_point".into(), TopicName::new("way_point"));
        topics.insert("goal_path".into(), TopicName::new("goal_path"));
        topics.insert("graph_nodes".into(), TopicName::new("graph_nodes"));
        topics.insert("graph_edges".into(), TopicName::new("graph_edges"));
        topics.insert(
            "contour_polygons".into(),
            TopicName::new("contour_polygons"),
        );
        topics.insert("nav_boundary".into(), TopicName::new("nav_boundary"));
        topics.insert("path_out".into(), TopicName::new("path"));
        topics.insert(
            "effective_cmd_vel".into(),
            TopicName::new("effective_cmd_vel"),
        );
        topics.insert("free_paths".into(), TopicName::new("free_paths"));
        topics.insert("goal_reached".into(), TopicName::new("goal_reached"));
        topics.insert(
            "corrected_odometry".into(),
            TopicName::new("corrected_odometry"),
        );
        topics.insert("global_map".into(), TopicName::new("global_map"));
        topics.insert("global_map_pgo".into(), TopicName::new("global_map_pgo"));
        topics.insert("pgo_tf".into(), TopicName::new("pgo_tf"));
        Self {
            topics,
            world_frame: default_world_frame(),
            base_frame: default_base_frame(),
            sensor_frame: default_sensor_frame(),
        }
    }

    /// 缺省 / 自实现的简单 fallback(空 topic 表 + map frame)。
    pub fn empty() -> Self {
        Self {
            topics: BTreeMap::new(),
            world_frame: default_world_frame(),
            base_frame: default_base_frame(),
            sensor_frame: default_sensor_frame(),
        }
    }

    /// Platform channel defaults shared with `nuwax-geometry-runtime`.
    /// Typed `#message.Type` suffixes are stripped because the stack binds the
    /// generated message type separately at subscription time.
    pub fn geometry_runtime_defaults() -> Self {
        let mut config = Self::cmu_defaults();
        for (port, keys, fallback) in [
            (
                "goal_pose",
                &[
                    "NUWAX_GEOMETRY_RUNTIME_NAV_GOAL_CHANNEL",
                    "NUWAX_LCM_CHANNEL_GOAL_POSE",
                ][..],
                "/nav/goal",
            ),
            (
                "nav_cancel",
                &[
                    "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
                    "NUWAX_LCM_CHANNEL_NAV_CANCEL",
                ][..],
                "/nav/cancel",
            ),
            (
                "nav_state",
                &[
                    "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL",
                    "NUWAX_LCM_CHANNEL_NAV_STATE",
                ][..],
                "/nav/state",
            ),
            (
                "odometry",
                &[
                    "NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL",
                    "NUWAX_LCM_CHANNEL_ODOM",
                ][..],
                "/odom",
            ),
            (
                "tf",
                &["NUWAX_GEOMETRY_RUNTIME_TF_CHANNEL", "NUWAX_LCM_CHANNEL_TF"][..],
                "/tf",
            ),
            (
                "occupancy",
                &[
                    "NUWAX_GEOMETRY_RUNTIME_OCCUPANCY_CHANNEL",
                    "NUWAX_LCM_CHANNEL_OCCUPANCY",
                ][..],
                "/map/occupancy",
            ),
            (
                "terrain_map",
                &[
                    "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL",
                    "NUWAX_LCM_CHANNEL_CLOUD_MAP",
                ][..],
                "/cloud_map",
            ),
            (
                "terrain_map_ext",
                &[
                    "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL",
                    "NUWAX_LCM_CHANNEL_CLOUD_MAP",
                ][..],
                "/cloud_map",
            ),
            (
                "cmd_vel",
                &[
                    "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",
                    "NUWAX_LCM_CHANNEL_CMD_VEL",
                ][..],
                "/cmd_vel",
            ),
        ] {
            config.set(port, configured_channel(keys, fallback));
        }
        for (port, channel) in [
            ("map_pose", "/nav/map_pose"),
            ("clicked_point", "/clicked_point"),
            ("joy_cmd", "/teleop/cmd_vel"),
            ("nav_cmd_vel", "/nav/cmd_vel"),
            ("goal_path", "/nav/path"),
            ("goal_reached", "/nav/goal_reached"),
            ("goal_failed", "/nav/goal_failed"),
            ("patrol_enabled", "/nav/patrol_enabled"),
            ("patrol_planner_control", "/nav/patrol_planner_control"),
            (
                "patrol_planner_control_ack",
                "/nav/patrol_planner_control_ack",
            ),
            ("stop_movement", "/nav/stop_movement"),
        ] {
            config.set(port, channel);
        }
        config
    }

    /// 取出某个 topic 名,缺失时回落到 `fallback`。
    pub fn get_or<'a>(&'a self, port: &str, fallback: &'a str) -> &'a str {
        self.topics
            .get(port)
            .map(|t| t.as_str())
            .unwrap_or(fallback)
    }

    /// 设置某个 topic。
    pub fn set(&mut self, port: impl Into<String>, topic: impl Into<String>) {
        self.topics.insert(port.into(), TopicName::new(topic));
    }
}

fn configured_channel(keys: &[&str], fallback: &str) -> String {
    let configured = keys
        .iter()
        .filter_map(|key| std::env::var(key).ok())
        .find(|value| !value.trim().is_empty())
        .unwrap_or_else(|| fallback.to_string());
    configured
        .split_once('#')
        .map_or(configured.clone(), |(channel, _)| channel.to_string())
}

impl Default for NavTopicConfig {
    fn default() -> Self {
        Self::cmu_defaults()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_include_way_point() {
        let cfg = NavTopicConfig::cmu_defaults();
        assert_eq!(cfg.get_or("way_point", "fallback"), "way_point");
        assert_eq!(cfg.world_frame, "map");
        assert_eq!(cfg.base_frame, "base_link");
    }

    #[test]
    fn fallback_is_used_when_missing() {
        let cfg = NavTopicConfig::empty();
        assert_eq!(cfg.get_or("missing", "/default/topic"), "/default/topic");
    }

    #[test]
    fn set_overrides_existing() {
        let mut cfg = NavTopicConfig::empty();
        cfg.set("cmd_vel", "robot/cmd_vel");
        assert_eq!(cfg.get_or("cmd_vel", "fallback"), "robot/cmd_vel");
    }

    #[test]
    fn serde_roundtrip_preserves_keys() {
        let cfg = NavTopicConfig::cmu_defaults();
        let s = serde_json::to_string(&cfg).expect("ser");
        let back: NavTopicConfig = serde_json::from_str(&s).expect("de");
        assert_eq!(cfg, back);
    }

    #[test]
    fn geometry_runtime_channel_strips_typed_suffix() {
        assert_eq!(
            configured_channel(&[], "/nav/goal#nuwax_msgs.GoalEnvelope"),
            "/nav/goal"
        );
    }

    #[test]
    fn occupancy_and_registered_scan_remain_type_distinct() {
        let config = NavTopicConfig::geometry_runtime_defaults();
        assert_eq!(config.get_or("occupancy", "missing"), "/map/occupancy");
        assert_eq!(
            config.get_or("registered_scan", "missing"),
            "registered_scan"
        );
        assert_ne!(
            config.get_or("occupancy", "missing"),
            config.get_or("registered_scan", "missing")
        );
        assert_eq!(config.get_or("nav_cmd_vel", "missing"), "/nav/cmd_vel");
        assert!(!config.topics.contains_key("start_pose"));
    }
}

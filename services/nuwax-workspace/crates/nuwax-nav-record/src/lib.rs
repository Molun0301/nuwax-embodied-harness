//! NavRecordProfile：导航 topic 录制配置。
//!
//! **职责**：仅拥有 `NavRecordProfile` 和导航 topic/type 清单。
//! 真实录制执行委托给 workspace 通用 recorder 能力（shared recorder）。
//! 若通用 recorder 尚未迁移，本 crate 只提供 profile，**不**复制 memory recorder 实现。
//!
//! 行为对账源：`dimos/navigation/cmu_nav/modules/nav_record/nav_record.py`。
//!
//! 默认录制 topic（与 Python 一致）：
//! cmd_vel / corrected_odometry / path / goal_path / way_point / goal / stop_movement /
//! effective_cmd_vel / slow_down / goal_reached / terrain_map / global_map /
//! odometry / registered_scan

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use thiserror::Error;

// Foundation 协议面 re-export — 共享 NavError / PoseStamped / Twist2 协议语义
// (Foundation 当前未提供 PointStamped / Twist; 仅 re-export 现有类型)
pub use nuwax_nav_base::NavError as FoundationNavError;
pub use nuwax_nav_types_messages::{
    PoseStamped as FoundationPoseStamped, Twist2 as FoundationTwist2,
};

/// 单条 topic 录制配置
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NavRecordTopic {
    /// topic 名
    pub topic: String,
    /// 消息类型
    pub msg_type: String,
}

/// Profile：导航 topic + 类型的清单
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavRecordProfile {
    /// 配置名（用于日志）
    pub name: String,
    /// db_path（兼容字段；具体落盘由共享 recorder 决定）
    pub db_path: String,
    /// topic 列表
    pub topics: Vec<NavRecordTopic>,
}

impl NavRecordProfile {
    /// 默认 profile（与 Python `NavRecordConfig` 默认一致）
    #[must_use]
    pub fn default_topics() -> Vec<NavRecordTopic> {
        vec![
            NavRecordTopic {
                topic: "cmd_vel".into(),
                msg_type: "geometry_msgs/Twist".into(),
            },
            NavRecordTopic {
                topic: "corrected_odometry".into(),
                msg_type: "nav_msgs/Odometry".into(),
            },
            NavRecordTopic {
                topic: "path".into(),
                msg_type: "nav_msgs/Path".into(),
            },
            NavRecordTopic {
                topic: "goal_path".into(),
                msg_type: "nav_msgs/Path".into(),
            },
            NavRecordTopic {
                topic: "way_point".into(),
                msg_type: "geometry_msgs/PointStamped".into(),
            },
            NavRecordTopic {
                topic: "goal".into(),
                msg_type: "geometry_msgs/PointStamped".into(),
            },
            NavRecordTopic {
                topic: "stop_movement".into(),
                msg_type: "std_msgs/Bool".into(),
            },
            NavRecordTopic {
                topic: "effective_cmd_vel".into(),
                msg_type: "geometry_msgs/Twist".into(),
            },
            NavRecordTopic {
                topic: "slow_down".into(),
                msg_type: "std_msgs/Int8".into(),
            },
            NavRecordTopic {
                topic: "goal_reached".into(),
                msg_type: "std_msgs/Bool".into(),
            },
            NavRecordTopic {
                topic: "terrain_map".into(),
                msg_type: "sensor_msgs/PointCloud2".into(),
            },
            NavRecordTopic {
                topic: "global_map".into(),
                msg_type: "sensor_msgs/PointCloud2".into(),
            },
            NavRecordTopic {
                topic: "odometry".into(),
                msg_type: "nav_msgs/Odometry".into(),
            },
            NavRecordTopic {
                topic: "registered_scan".into(),
                msg_type: "sensor_msgs/PointCloud2".into(),
            },
        ]
    }

    /// 构造默认 profile
    #[must_use]
    pub fn new(name: impl Into<String>, db_path: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            db_path: db_path.into(),
            topics: Self::default_topics(),
        }
    }

    /// 校验 profile：topic 不重复，msg_type 不为空
    pub fn validate(&self) -> Result<(), NavRecordError> {
        if self.topics.is_empty() {
            return Err(NavRecordError::Empty);
        }
        let mut seen = BTreeSet::new();
        for t in &self.topics {
            if t.topic.is_empty() {
                return Err(NavRecordError::EmptyTopic);
            }
            if t.msg_type.is_empty() {
                return Err(NavRecordError::EmptyMsgType(t.topic.clone()));
            }
            if !seen.insert(t.topic.clone()) {
                return Err(NavRecordError::DuplicateTopic(t.topic.clone()));
            }
        }
        Ok(())
    }

    /// 所有 topic 名（用于订阅注册）
    #[must_use]
    pub fn topic_names(&self) -> Vec<&str> {
        self.topics.iter().map(|t| t.topic.as_str()).collect()
    }

    /// 是否包含某 topic
    #[must_use]
    pub fn contains(&self, topic: &str) -> bool {
        self.topics.iter().any(|t| t.topic == topic)
    }

    /// 数量
    #[must_use]
    pub fn len(&self) -> usize {
        self.topics.len()
    }

    /// 是否为空
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.topics.is_empty()
    }
}

/// NavRecord 错误
#[derive(Debug, Error)]
pub enum NavRecordError {
    /// topic 列表为空
    #[error("NavRecord profile has no topics")]
    Empty,
    /// topic 名为空
    #[error("topic name cannot be empty")]
    EmptyTopic,
    /// msg_type 为空
    #[error("msg_type cannot be empty for topic '{0}'")]
    EmptyMsgType(String),
    /// topic 重复
    #[error("duplicate topic '{0}' in NavRecord profile")]
    DuplicateTopic(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    #[test]
    fn default_topics_match_python_parity() {
        // 与 `nav_record.py` 订阅列表一致
        let topics = NavRecordProfile::default_topics();
        let names: Vec<&str> = topics.iter().map(|t| t.topic.as_str()).collect();
        for required in [
            "cmd_vel",
            "corrected_odometry",
            "path",
            "goal_path",
            "way_point",
            "goal",
            "stop_movement",
            "effective_cmd_vel",
            "slow_down",
            "goal_reached",
            "terrain_map",
            "global_map",
            "odometry",
            "registered_scan",
        ] {
            assert!(names.contains(&required), "missing topic {required}");
        }
    }

    #[test]
    fn no_duplicate_topics() {
        let p = NavRecordProfile::new("default", "nav.db");
        // 构造一个带重复的 profile
        let mut bad = NavRecordProfile::default_topics();
        bad.push(bad[0].clone());
        let prof = NavRecordProfile {
            name: p.name,
            db_path: p.db_path,
            topics: bad,
        };
        match prof.validate() {
            Err(NavRecordError::DuplicateTopic(_)) => {}
            _ => panic!("expected duplicate"),
        }
    }

    #[test]
    fn validate_ok_for_default() {
        let p = NavRecordProfile::new("default", "nav.db");
        assert!(p.validate().is_ok());
    }

    #[test]
    fn validate_empty_profile_fails() {
        let p = NavRecordProfile {
            name: "x".into(),
            db_path: "x.db".into(),
            topics: vec![],
        };
        assert!(p.validate().is_err());
    }

    #[test]
    fn contains_and_len() {
        let p = NavRecordProfile::new("default", "nav.db");
        assert!(p.contains("cmd_vel"));
        assert!(!p.contains("nope"));
        assert_eq!(p.len(), NavRecordProfile::default_topics().len());
    }
}

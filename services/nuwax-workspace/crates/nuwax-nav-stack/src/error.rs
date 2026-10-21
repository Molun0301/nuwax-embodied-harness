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

//! Stack 编排层的错误类型。
//!
//! 本 crate 不持有算法,所以错误只反映编排问题(profile / feature / topic 配置不正确),
//! 不混入 `NavError`(那是 `nuwax-nav-base` 的领域)。

use thiserror::Error;

/// Stack 操作的统一错误类型。
#[derive(Debug, Error)]
pub enum NavStackError {
    /// 用户传入的 profile / topic / feature 名称无法识别。
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),

    /// 某个 profile 需要的 feature 没有启用。
    #[error("feature `{feature}` required for profile `{profile}` but it is not enabled in Cargo features")]
    MissingFeature {
        /// profile 名称。
        profile: String,
        /// 缺失的 feature。
        feature: String,
    },

    /// serde / JSON / 序列化错误。
    #[error("serialization error: {0}")]
    Serde(#[from] serde_json::Error),

    /// IO 错误(读 fixture、写 stdout 重定向等)。
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),

    /// P0-07 StackRunner:启动 native bridge 子进程失败。
    #[error("failed to spawn native bridge `{node}`: {source}")]
    SpawnFailed {
        /// 节点 id(`ModuleNode.id`)。
        node: String,
        /// 底层 std::io::Error。
        #[source]
        source: std::io::Error,
    },

    /// One or more graph nodes have no concrete runtime owner.
    #[error("runtime owners are missing for graph nodes: {nodes:?}")]
    MissingRuntimeOwners {
        /// Node ids and kinds collected during the all-or-nothing preflight.
        nodes: Vec<String>,
    },

    /// A concrete in-process owner could not be created or started.
    #[error("failed to start in-process node `{node}`: {reason}")]
    NodeStartFailed {
        /// Graph node id.
        node: String,
        /// Factory or lifecycle error.
        reason: String,
    },

    /// Startup was requested for a runner that already owns live nodes.
    #[error("navigation stack runner is already running")]
    AlreadyRunning,

    /// A child process could not be inspected.
    #[error("failed to inspect runtime node `{node}`: {source}")]
    ProcessInspectionFailed {
        /// Graph node id.
        node: String,
        /// Underlying process error.
        #[source]
        source: std::io::Error,
    },

    /// A runtime node exited before the requested stack shutdown.
    #[error("runtime node `{node}` exited before stack shutdown")]
    RuntimeNodeExited {
        /// Graph node id.
        node: String,
    },

    /// A node could not be stopped or reaped during shutdown.
    #[error("failed to shut down runtime node `{node}`: {reason}")]
    ShutdownFailed {
        /// Graph node id.
        node: String,
        /// Shutdown error details.
        reason: String,
    },

    /// Producer-typed PCT outcome, preserving inconclusive resource/domain status.
    #[cfg(feature = "mls")]
    #[error("pct global planning: {0}")]
    Planning(String),

    /// 任意内部错误(不应当发生,如果发生请报告)。
    #[error("internal nav-stack error: {0}")]
    Internal(String),
}

impl NavStackError {
    /// 快速构造一个 `MissingFeature`。
    pub fn missing_feature(profile: impl Into<String>, feature: impl Into<String>) -> Self {
        Self::MissingFeature {
            profile: profile.into(),
            feature: feature.into(),
        }
    }

    /// 快速构造一个 `InvalidConfig`。
    pub fn invalid(msg: impl Into<String>) -> Self {
        Self::InvalidConfig(msg.into())
    }
}

/// Stack 操作的统一 Result 类型别名。
pub type NavStackResult<T> = std::result::Result<T, NavStackError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_feature_message_names_both() {
        let err = NavStackError::missing_feature("CmuNative", "native-far");
        let msg = err.to_string();
        assert!(msg.contains("native-far"));
        assert!(msg.contains("CmuNative"));
    }
}

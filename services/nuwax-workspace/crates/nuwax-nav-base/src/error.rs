//! `NavError` 错误类型(分层,thiserror)
//!
//! 禁止把 failure reason 退化为字符串;调用方按变体处理。

use thiserror::Error;

use crate::native::NativeProcessError;

/// `NavError` 处理结果
pub type NavResult<T> = Result<T, NavError>;

/// 导航模块统一错误,分层避免字符串解析
#[derive(Debug, Error)]
pub enum NavError {
    /// 输入无效(字段非法 / 缺失)
    #[error("invalid input on field `{field}`: {reason}")]
    InvalidInput {
        /// 字段名
        field: &'static str,
        /// 原因(短描述)
        reason: String,
    },
    /// 地图缺失 / 不可用
    #[error("map unavailable")]
    MapUnavailable,
    /// 终点越界
    #[error("goal out of bounds")]
    GoalOutOfBounds,
    /// 终点处于 occupied cell
    #[error("goal occupied")]
    GoalOccupied,
    /// 无可达路径
    #[error("no path found (explored {explored_nodes} nodes)")]
    NoPath {
        /// 已扩展节点数
        explored_nodes: usize,
    },
    /// native 进程错误,透传
    #[error("native process error: {0}")]
    NativeProcess(#[from] NativeProcessError),
    /// 传输 / 序列化失败
    #[error("transport error: {0}")]
    Transport(String),
    /// 操作超时
    #[error("timeout in {operation} after {millis} ms")]
    Timeout {
        /// 操作名
        operation: &'static str,
        /// 超时阈值(毫秒)
        millis: u64,
    },
    /// 内部错误(实现 bug / 不可恢复异常)
    #[error("internal error: {0}")]
    Internal(String),
}

impl NavError {
    /// 是否可重试(瞬时错误)
    #[must_use]
    pub fn is_retryable(&self) -> bool {
        matches!(
            self,
            Self::Timeout { .. } | Self::Transport(_) | Self::Internal(_)
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_display_contains_field() {
        let e = NavError::InvalidInput {
            field: "costmap",
            reason: "empty".to_string(),
        };
        assert!(e.to_string().contains("costmap"));
        assert!(e.to_string().contains("empty"));
    }

    #[test]
    fn error_from_native_process_error() {
        let inner = NativeProcessError::ExecutableMissing {
            path: std::path::PathBuf::from("/opt/x"),
        };
        let e: NavError = inner.into();
        assert!(matches!(
            e,
            NavError::NativeProcess(NativeProcessError::ExecutableMissing { .. })
        ));
    }

    #[test]
    fn error_retryable_partition() {
        assert!(NavError::Timeout {
            operation: "plan",
            millis: 10
        }
        .is_retryable());
        assert!(!NavError::GoalOutOfBounds.is_retryable());
    }
}

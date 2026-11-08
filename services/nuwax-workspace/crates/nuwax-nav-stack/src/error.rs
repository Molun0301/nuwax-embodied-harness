//! 导航配置、实际任务启动与执行错误。
use thiserror::Error;

/// 导航栈操作错误。
#[derive(Debug, Error)]
pub enum NavStackError {
    /// 配置或命令行无法解析。
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
    /// 消息或数据序列化失败。
    #[error("serialization error: {0}")]
    Serde(#[from] serde_json::Error),
    /// 文件或线程创建失败。
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    /// 任务在请求关闭前退出。
    #[error("runtime task `{node}` exited before stack shutdown")]
    RuntimeNodeExited {
        /// 任务名称。
        node: String,
    },
    /// 全局规划器返回的结果。
    #[cfg(feature = "mls")]
    #[error("pct global planning: {0}")]
    Planning(String),
    /// 任务报告的具体错误。
    #[error("internal nav-stack error: {0}")]
    Internal(String),
}
impl NavStackError {
    /// 构造配置错误。
    pub fn invalid(message: impl Into<String>) -> Self { Self::InvalidConfig(message.into()) }
}
/// 导航栈结果。
pub type NavStackResult<T> = Result<T, NavStackError>;

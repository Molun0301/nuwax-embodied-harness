//! native runner 相关状态、错误与诊断
//!
//! 与 `nuwax-nav-native-runner` 共享;不允许 bridge crate 把这些错误降级为字符串。

use std::path::PathBuf;

use serde::{Deserialize, Serialize};

/// Native 进程状态
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum NativeRunnerState {
    /// 未启动
    NotStarted,
    /// 正在构建(执行 build_command)
    Building,
    /// 已运行
    Running {
        /// 进程 pid
        pid: u32,
    },
    /// 正在停止(graceful → timeout)
    Stopping {
        /// 进程 pid
        pid: u32,
    },
    /// 已退出
    Exited {
        /// 退出码
        code: Option<i32>,
    },
    /// 启动 / 运行失败
    Failed {
        /// 失败原因
        error: NativeProcessError,
    },
}

impl NativeRunnerState {
    /// 是否正在运行(包含正在 graceful stop)
    #[must_use]
    pub fn is_running(&self) -> bool {
        matches!(self, Self::Running { .. } | Self::Stopping { .. })
    }

    /// 是否已结束(包括退出 / 失败 / 未启动)
    #[must_use]
    pub fn is_terminated(&self) -> bool {
        matches!(
            self,
            Self::NotStarted | Self::Exited { .. } | Self::Failed { .. }
        )
    }
}

/// Native 进程错误(透传到 `NavError::NativeProcess`)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum NativeProcessError {
    /// Spec 字段非法
    InvalidSpec {
        /// 字段名(由调用方字面量 `.into()` 为 `String`,避免 `tag/content` derive 的 `'static` 借用约束)
        field: String,
        /// 原因
        reason: String,
    },
    /// executable 不存在
    ExecutableMissing {
        /// 缺失路径
        path: PathBuf,
    },
    /// 构建失败
    BuildFailed {
        /// 命令字符串
        command: String,
        /// 退出码
        code: Option<i32>,
    },
    /// 构建后 executable 仍不存在
    ExecutableMissingAfterBuild {
        /// 期望路径
        path: PathBuf,
    },
    /// spawn 失败
    SpawnFailed {
        /// 来源描述
        source: String,
    },
    /// 非零退出
    ExitedNonZero {
        /// 退出码
        code: Option<i32>,
    },
    /// 停止超时
    StopTimeout {
        /// 超时阈值(毫秒)
        millis: u64,
    },
}

impl std::fmt::Display for NativeProcessError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidSpec { field, reason } => write!(f, "invalid spec `{field}`: {reason}"),
            Self::ExecutableMissing { path } => write!(f, "executable missing: {}", path.display()),
            Self::BuildFailed { command, code } => {
                write!(f, "build failed (code={code:?}) cmd=`{command}`")
            }
            Self::ExecutableMissingAfterBuild { path } => {
                write!(
                    f,
                    "executable still missing after build: {}",
                    path.display()
                )
            }
            Self::SpawnFailed { source } => write!(f, "spawn failed: {source}"),
            Self::ExitedNonZero { code } => write!(f, "exited non-zero: code={code:?}"),
            Self::StopTimeout { millis } => write!(f, "stop timeout after {millis} ms"),
        }
    }
}

impl NativeProcessError {
    /// 由 `&'static str` 字面量构造 `InvalidSpec`,免调用方手写 `.into()`
    #[must_use]
    pub fn invalid_spec(field: &'static str, reason: impl Into<String>) -> Self {
        Self::InvalidSpec {
            field: field.to_string(),
            reason: reason.into(),
        }
    }
}

impl std::error::Error for NativeProcessError {}

/// 重启策略
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum RestartPolicy {
    /// 不重启
    #[default]
    Never,
    /// 失败时重启,带最大次数与 backoff
    OnFailure {
        /// 最大重启次数
        max_restarts: u32,
        /// backoff(毫秒)
        backoff_millis: u64,
    },
}

/// 图像视图(借用,不持有像素数据)
///
/// `ImageEncoding::Other(String)` 内含堆字符串,故 `ImageView` 仅派生 `Clone`,
/// 不派生 `Copy`;调用方如需复用请显式 `.clone()`。
#[derive(Debug, Clone)]
pub struct ImageView<'a> {
    /// 宽度
    pub width: u32,
    /// 高度
    pub height: u32,
    /// 编码
    pub encoding: nuwax_nav_types_messages::visual::ImageEncoding,
    /// 数据
    pub data: &'a [u8],
}

impl<'a> ImageView<'a> {
    /// 由 RGB 数据构造(每像素 3 字节)
    #[must_use]
    pub fn from_rgb(width: u32, height: u32, data: &'a [u8]) -> Self {
        debug_assert_eq!(data.len(), (width as usize) * (height as usize) * 3);
        Self {
            width,
            height,
            encoding: nuwax_nav_types_messages::visual::ImageEncoding::Rgb8,
            data,
        }
    }

    /// 是否与给定 width/height 一致(用于校验)
    #[must_use]
    pub fn matches(&self, width: u32, height: u32) -> bool {
        self.width == width && self.height == height
    }
}

/// native 进程诊断
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct NativeProcessDiagnostics {
    /// 组件名
    pub component: String,
    /// 当前状态(快照)
    pub state: Option<NativeRunnerState>,
    /// 最近一次启动时间戳
    pub last_started_at: Option<nuwax_nav_types_messages::Timestamp>,
    /// 最近一次退出时间戳
    pub last_exited_at: Option<nuwax_nav_types_messages::Timestamp>,
    /// 累计重启次数
    pub restart_count: u32,
    /// 最近一次退出码
    pub last_exit_code: Option<i32>,
    /// stdout 末尾截断(最多 4 KiB)
    pub stdout_tail: String,
    /// stderr 末尾截断(最多 4 KiB)
    pub stderr_tail: String,
    /// 启动耗时(纳秒;None 表示未启动过)
    pub last_startup_latency_ns: Option<i64>,
}

impl NativeProcessDiagnostics {
    /// 新建诊断(给定 component)
    #[must_use]
    pub fn new(component: impl Into<String>) -> Self {
        Self {
            component: component.into(),
            ..Self::default()
        }
    }

    /// 追加到 stdout tail(保留最近 4 KiB)
    pub fn push_stdout(&mut self, s: &str) {
        push_tail(&mut self.stdout_tail, s);
    }

    /// 追加到 stderr tail(保留最近 4 KiB)
    pub fn push_stderr(&mut self, s: &str) {
        push_tail(&mut self.stderr_tail, s);
    }
}

const TAIL_MAX: usize = 4096;

fn push_tail(buf: &mut String, s: &str) {
    buf.push_str(s);
    if buf.len() > TAIL_MAX {
        let cut = buf.len() - TAIL_MAX;
        // 找到下一个 char boundary(避免在多字节字符中间切)
        let mut start = cut;
        while start < buf.len() && !buf.is_char_boundary(start) {
            start += 1;
        }
        buf.drain(..start);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::visual::ImageEncoding;

    #[test]
    fn runner_state_running_predicate() {
        assert!(NativeRunnerState::Running { pid: 1 }.is_running());
        assert!(NativeRunnerState::Stopping { pid: 1 }.is_running());
        assert!(!NativeRunnerState::NotStarted.is_running());
        assert!(NativeRunnerState::Exited { code: Some(0) }.is_terminated());
    }

    #[test]
    fn process_error_display_includes_path() {
        let e = NativeProcessError::ExecutableMissing {
            path: PathBuf::from("/opt/foo"),
        };
        assert!(e.to_string().contains("/opt/foo"));
    }

    #[test]
    fn restart_policy_default_never() {
        assert_eq!(RestartPolicy::default(), RestartPolicy::Never);
    }

    #[test]
    fn image_view_rgb_basic() {
        let data = vec![0u8; 3];
        let v = ImageView::from_rgb(1, 1, &data);
        assert_eq!(v.width, 1);
        assert_eq!(v.height, 1);
        assert_eq!(v.encoding, ImageEncoding::Rgb8);
        assert!(v.matches(1, 1));
    }

    #[test]
    fn diagnostics_stdout_tail_caps_at_4kib() {
        let mut d = NativeProcessDiagnostics::new("test");
        for _ in 0..10 {
            d.push_stdout(&"x".repeat(1024));
        }
        assert!(d.stdout_tail.len() <= TAIL_MAX + 64);
    }

    #[test]
    fn process_error_serde_tagged() {
        let e = NativeProcessError::SpawnFailed {
            source: "no such file".to_string(),
        };
        let s = serde_json::to_string(&e).unwrap();
        assert!(s.contains("spawn_failed"));
        let back: NativeProcessError = serde_json::from_str(&s).unwrap();
        assert_eq!(back, e);
    }
}

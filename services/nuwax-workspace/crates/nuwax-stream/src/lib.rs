//! nuwax 流式数据原语
//!
//! 提供 Subject / Observer 抽象,与 `nuwax-core::In<T>/Out<T>` 互转。
//!
//! SSOT:dimos Python 端
//! - `Subject` 来自 RxPy(`reactivex.subject.Subject`),提供 `on_completed()` / `on_error()`
//!   标准钩子(见 `/home/ml/Deeprob/dimos/dimos/stream/audio/node_key_recorder.py:23` 等)
//! - 视频帧类型为 `SerializableVideoFrame`(见
//!   `/home/ml/Deeprob/dimos/dimos/robot/unitree/connection.py:70-93`),
//!   Rust 端口命名为 `VideoFrame`,字段集与 Python 等价

#![deny(unsafe_code)]
#![warn(missing_docs)]

use futures::{Stream, StreamExt};
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll};
use tokio::sync::{broadcast, watch};
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;
use tokio_stream::wrappers::BroadcastStream;

/// 可序列化的视频帧 — 对齐 dimos `SerializableVideoFrame`
///
/// 见 `/home/ml/Deeprob/dimos/dimos/robot/unitree/connection.py:70-93`
///
/// Python 端的 `data` 是 `np.ndarray`,因此除字节外还必须保留
/// shape / dtype / strides。`data` 保存 ndarray 底层 buffer,
/// `byte_offset` 指向第一个逻辑元素,可表达带 padding 或负 stride 的 view。
#[derive(Debug, Clone, PartialEq)]
pub struct VideoFrame {
    /// ndarray 底层字节 buffer
    pub data: Vec<u8>,
    /// ndarray shape,视频通常为 `[height, width, channels]`
    pub shape: Vec<usize>,
    /// NumPy dtype string,优先使用 `dtype.str` 形式(如 `"|u1"` / `"<f4"`)
    pub dtype: String,
    /// 每个维度的字节 stride,与 NumPy `ndarray.strides` 一致
    pub strides: Vec<isize>,
    /// 第一个逻辑元素在 `data` 中的字节偏移
    pub byte_offset: usize,
    /// 显示时间戳(PTS,可选)
    pub pts: Option<i64>,
    /// 显示时间(秒,可选)
    pub time: Option<f64>,
    /// 解码时间戳(DTS,可选)
    pub dts: Option<i64>,
    /// 帧宽(像素),对应 ndarray shape 的第二维
    pub width: Option<u32>,
    /// 帧高(像素),对应 ndarray shape 的第一维
    pub height: Option<u32>,
    /// 像素格式名(如 `"rgb24"` / `"bgr24"`)
    pub format: Option<String>,
}

impl VideoFrame {
    /// 构造一帧 RGB24 数据
    #[must_use]
    pub fn rgb24(width: u32, height: u32, data: Vec<u8>) -> Self {
        let width_usize = usize::try_from(width).expect("u32 width must fit usize");
        let height_usize = usize::try_from(height).expect("u32 height must fit usize");
        let row_stride = width_usize.saturating_mul(3);
        Self {
            data,
            shape: vec![height_usize, width_usize, 3],
            dtype: "|u1".to_string(),
            strides: vec![isize::try_from(row_stride).unwrap_or(isize::MAX), 3, 1],
            byte_offset: 0,
            pts: None,
            time: None,
            dts: None,
            width: Some(width),
            height: Some(height),
            format: Some("rgb24".to_string()),
        }
    }

    /// 从 ndarray 布局构造帧,并从 `[height, width, ...]` 推导像素维度。
    ///
    /// # Errors
    /// 当 shape / dtype / strides / buffer 边界不一致时返回 `VideoFrameError`。
    pub fn from_ndarray_parts(
        data: Vec<u8>,
        shape: Vec<usize>,
        dtype: impl Into<String>,
        strides: Vec<isize>,
        byte_offset: usize,
    ) -> Result<Self, VideoFrameError> {
        let width = shape
            .get(1)
            .copied()
            .map(u32::try_from)
            .transpose()
            .map_err(|_| VideoFrameError::DimensionOverflow)?;
        let height = shape
            .first()
            .copied()
            .map(u32::try_from)
            .transpose()
            .map_err(|_| VideoFrameError::DimensionOverflow)?;
        let frame = Self {
            data,
            shape,
            dtype: dtype.into(),
            strides,
            byte_offset,
            pts: None,
            time: None,
            dts: None,
            width,
            height,
            format: None,
        };
        frame.validate_layout()?;
        Ok(frame)
    }

    /// 验证 ndarray 布局及 width / height 的语义一致性。
    ///
    /// # Errors
    /// 当元数据无法对应 `data` 中的有效字节区间时返回错误。
    pub fn validate_layout(&self) -> Result<(), VideoFrameError> {
        if self.shape.len() != self.strides.len() {
            return Err(VideoFrameError::RankMismatch {
                shape: self.shape.len(),
                strides: self.strides.len(),
            });
        }
        if let Some(width) = self.width {
            let width = usize::try_from(width).map_err(|_| VideoFrameError::DimensionOverflow)?;
            if Some(width) != self.shape.get(1).copied() {
                return Err(VideoFrameError::WidthMismatch);
            }
        }
        if let Some(height) = self.height {
            let height = usize::try_from(height).map_err(|_| VideoFrameError::DimensionOverflow)?;
            if Some(height) != self.shape.first().copied() {
                return Err(VideoFrameError::HeightMismatch);
            }
        }

        let item_size = dtype_item_size(&self.dtype)?;
        if self.shape.contains(&0) {
            return Ok(());
        }
        let mut first =
            i128::try_from(self.byte_offset).map_err(|_| VideoFrameError::LayoutOverflow)?;
        let mut last = first;
        for (&dimension, &stride) in self.shape.iter().zip(&self.strides) {
            let extent = i128::try_from(dimension.saturating_sub(1))
                .map_err(|_| VideoFrameError::LayoutOverflow)?
                .checked_mul(i128::try_from(stride).map_err(|_| VideoFrameError::LayoutOverflow)?)
                .ok_or(VideoFrameError::LayoutOverflow)?;
            if extent < 0 {
                first = first
                    .checked_add(extent)
                    .ok_or(VideoFrameError::LayoutOverflow)?;
            } else {
                last = last
                    .checked_add(extent)
                    .ok_or(VideoFrameError::LayoutOverflow)?;
            }
        }
        let end = last
            .checked_add(i128::try_from(item_size).map_err(|_| VideoFrameError::LayoutOverflow)?)
            .ok_or(VideoFrameError::LayoutOverflow)?;
        let data_len =
            i128::try_from(self.data.len()).map_err(|_| VideoFrameError::LayoutOverflow)?;
        if first < 0 || end > data_len {
            return Err(VideoFrameError::BufferTooSmall);
        }
        Ok(())
    }
}

/// `VideoFrame` ndarray 元数据错误。
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum VideoFrameError {
    /// shape 与 strides 维数不同。
    #[error("shape rank {shape} does not match strides rank {strides}")]
    RankMismatch {
        /// shape 维数
        shape: usize,
        /// strides 维数
        strides: usize,
    },
    /// width 与 `shape[1]` 不一致。
    #[error("width does not match ndarray shape[1]")]
    WidthMismatch,
    /// height 与 `shape[0]` 不一致。
    #[error("height does not match ndarray shape[0]")]
    HeightMismatch,
    /// 像素维度超出 u32。
    #[error("video dimension does not fit u32")]
    DimensionOverflow,
    /// dtype 不是可解析的 NumPy dtype string。
    #[error("unsupported numpy dtype: {0}")]
    UnsupportedDtype(String),
    /// stride 计算溢出。
    #[error("ndarray layout arithmetic overflow")]
    LayoutOverflow,
    /// 布局超出 data buffer。
    #[error("ndarray layout exceeds data buffer")]
    BufferTooSmall,
}

fn dtype_item_size(dtype: &str) -> Result<usize, VideoFrameError> {
    let named = match dtype {
        "bool" | "uint8" | "int8" => Some(1),
        "uint16" | "int16" | "float16" => Some(2),
        "uint32" | "int32" | "float32" => Some(4),
        "uint64" | "int64" | "float64" => Some(8),
        _ => None,
    };
    if let Some(size) = named {
        return Ok(size);
    }
    let normalized = dtype.trim_start_matches(['|', '<', '>', '=']);
    let mut chars = normalized.chars();
    let kind = chars.next();
    let width = chars.as_str().parse::<usize>().ok();
    match (kind, width) {
        (Some('u' | 'i' | 'f' | 'c' | 'b'), Some(size)) if size > 0 => Ok(size),
        _ => Err(VideoFrameError::UnsupportedDtype(dtype.to_string())),
    }
}

/// `Subject` 事件 — 对齐 RxPy `on_next` / `on_completed` / `on_error`
///
/// 推荐用法:`let subject = Subject::<SubjectEvent<MyMsg>>::new(64)`,
/// 这样同一个 `Subject` 可同时表达数据 / 完成 / 错误三种信号,
/// 类似 RxPy 的 `Subject.on_next(v)` / `.on_completed()` / `.on_error(e)`。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SubjectEvent<T> {
    /// 正常数据(对应 `on_next`)
    Next(T),
    /// 流结束(对应 `on_completed`)
    Completed,
    /// 流错误(对应 `on_error`)
    Error(SubjectError),
}

/// `SubjectEvent::Error` 携带的错误类型
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SubjectError {
    /// 来自上游提供方的错误(例如视频帧获取失败)
    #[error("upstream error: {0}")]
    Upstream(String),
    /// 自定义错误消息
    #[error("stream error: {0}")]
    Custom(String),
}

/// `Subject` —— 多订阅者广播
///
/// 对齐 RxPy `reactivex.subject.Subject` 的三组钩子:
/// - `push` ⇔ `on_next`
/// - `complete` ⇔ `on_completed` (要求 `T = SubjectEvent<U>`)
/// - `error` ⇔ `on_error` (要求 `T = SubjectEvent<U>`)
pub struct Subject<T: Clone + Send + 'static> {
    inner: Arc<SubjectInner<T>>,
}

struct SubjectInner<T: Clone + Send + 'static> {
    tx: broadcast::Sender<SubjectSignal<T>>,
    terminal: Mutex<Option<T>>,
}

#[derive(Clone)]
enum SubjectSignal<T> {
    Next(T),
    Terminal(T),
}

impl<T: Clone + Send + 'static> Subject<T> {
    /// 构造新 subject
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        Self {
            inner: Arc::new(SubjectInner {
                tx,
                terminal: Mutex::new(None),
            }),
        }
    }

    /// 向所有订阅者推送一条消息(对应 RxPy `on_next`)
    ///
    /// # Errors
    /// - 当 subject 已 Completed/Error 时返回 `StreamError::Terminated`
    pub fn push(&self, value: T) -> Result<(), StreamError> {
        let terminal = lock_unpoisoned(&self.inner.terminal);
        if terminal.is_some() {
            return Err(StreamError::Terminated);
        }
        // Rx Subject treats on_next without observers as a successful no-op.
        let _ = self.inner.tx.send(SubjectSignal::Next(value));
        Ok(())
    }

    /// 订阅 —— 返回 `Stream<Item = T>`
    #[must_use]
    pub fn subscribe(&self) -> SubscriberStream<T> {
        let terminal = lock_unpoisoned(&self.inner.terminal);
        if let Some(value) = terminal.clone() {
            SubscriberStream {
                inner: None,
                pending_terminal: Some(value),
                done: false,
            }
        } else {
            let rx = self.inner.tx.subscribe();
            SubscriberStream {
                inner: Some(BroadcastStream::new(rx)),
                pending_terminal: None,
                done: false,
            }
        }
    }

    /// 内部订阅。
    ///
    /// 返回与 `subscribe` 相同的 sticky-terminal stream,保留方法名以兼容
    /// 早期 `connect` 调用方。
    #[must_use]
    pub fn subscribe_internal(&self) -> SubscriberStream<T> {
        self.subscribe()
    }

    /// `Subject::connect` 内部使用 —— 直接向底 `broadcast::Sender` 推一条
    ///
    /// # Errors
    /// 当 subject 已终止时返回 `StreamError::Terminated`。
    pub(crate) fn push_internal(&self, value: T) -> Result<(), StreamError> {
        self.push(value)
    }

    fn terminate(&self, value: T) -> Result<(), StreamError> {
        let mut terminal = lock_unpoisoned(&self.inner.terminal);
        if terminal.is_some() {
            return Err(StreamError::Terminated);
        }
        *terminal = Some(value.clone());
        // Rx Subject 允许在没有当前 observer 时终止,终态仍会传给 late subscriber。
        let _ = self.inner.tx.send(SubjectSignal::Terminal(value));
        Ok(())
    }

    /// 把多个可终止 `Subject` 合并成一个由可停止句柄拥有的连接。
    ///
    /// 与 Rx `merge` 一致:普通值继续 fan-in,任一 source Error 立即终止合流,
    /// 所有 source Completed 后才 Completed。返回的 `ConnectedSubject` 在 drop/stop 时
    /// 取消全部转发任务,不留 detached task。
    ///
    /// # Arguments
    /// * `sources` - 要合并的 source subjects
    /// * `merged_capacity` - 合并目标 subject 的 broadcast 容量
    #[must_use]
    pub fn connect(sources: Vec<Subject<T>>, merged_capacity: usize) -> ConnectedSubject<T>
    where
        T: TerminalEvent + Sync,
    {
        let merged = Subject::<T>::new(merged_capacity);
        let remaining = Arc::new(AtomicUsize::new(sources.len()));
        let stopped = Arc::new(AtomicBool::new(false));
        let (cancel_tx, _) = watch::channel(false);
        let mut aborts = Vec::with_capacity(sources.len());
        for src in sources {
            let mut rx = src.subscribe();
            let merged_inner = merged.clone();
            let remaining_inner = Arc::clone(&remaining);
            let stopped_inner = Arc::clone(&stopped);
            let mut cancel_rx = cancel_tx.subscribe();
            let cancel_on_terminal = cancel_tx.clone();
            let task = tokio::spawn(async move {
                // 任务拥有 source clone,连接存活期内不会因 caller 丢弃额外 clone 而假关闭。
                let _source_owner = src;
                loop {
                    tokio::select! {
                        changed = cancel_rx.changed() => {
                            if changed.is_err() || *cancel_rx.borrow() {
                                break;
                            }
                        }
                        item = rx.next() => match item {
                            Some(Ok(value)) => match value.terminal_kind() {
                                None => {
                                    if matches!(
                                        merged_inner.push_internal(value),
                                        Err(StreamError::Terminated)
                                    ) {
                                        break;
                                    }
                                }
                                Some(TerminalKind::Error) => {
                                    let _ = merged_inner.terminate(value);
                                    stopped_inner.store(true, Ordering::Release);
                                    let _ = cancel_on_terminal.send(true);
                                    break;
                                }
                                Some(TerminalKind::Completed) => {
                                    if remaining_inner.fetch_sub(1, Ordering::AcqRel) == 1 {
                                        let _ = merged_inner.terminate(value);
                                        stopped_inner.store(true, Ordering::Release);
                                        let _ = cancel_on_terminal.send(true);
                                    }
                                    break;
                                }
                            },
                            Some(Err(error)) => {
                                let _ = merged_inner.terminate(T::upstream_error(error.to_string()));
                                stopped_inner.store(true, Ordering::Release);
                                let _ = cancel_on_terminal.send(true);
                                break;
                            }
                            None => {
                                let _ = merged_inner.terminate(T::upstream_error(
                                    "source closed without terminal event".to_string(),
                                ));
                                stopped_inner.store(true, Ordering::Release);
                                let _ = cancel_on_terminal.send(true);
                                break;
                            }
                        }
                    }
                }
            });
            aborts.push(task.abort_handle());
        }
        ConnectedSubject {
            subject: merged,
            connection: ConnectionHandle {
                cancel: cancel_tx,
                aborts,
                stopped,
            },
        }
    }
}

impl<U: Clone + Send + 'static> Subject<SubjectEvent<U>> {
    /// 推送一个 `Next` 事件
    ///
    /// # Errors
    /// 当 subject 已终止时返回 `StreamError::Terminated`。
    pub fn push_next(&self, value: U) -> Result<(), StreamError> {
        self.push(SubjectEvent::Next(value))
    }

    /// 推送一个 `Completed` 事件(对应 RxPy `on_completed`)
    ///
    /// # Errors
    /// 当 subject 已终止时返回 `StreamError::Terminated`。
    pub fn complete(&self) -> Result<(), StreamError> {
        self.terminate(SubjectEvent::<U>::Completed)
    }

    /// 推送一个 `Error` 事件(对应 RxPy `on_error`)
    ///
    /// # Errors
    /// 当 subject 已终止时返回 `StreamError::Terminated`。
    pub fn error(&self, err: SubjectError) -> Result<(), StreamError> {
        self.terminate(SubjectEvent::<U>::Error(err))
    }
}

/// 可由 `Subject::connect` 识别的终态事件。
pub trait TerminalEvent: Clone + Send + 'static {
    /// 普通值返回 `None`,终态返回其类型。
    fn terminal_kind(&self) -> Option<TerminalKind>;

    /// 将转发层错误转换为该事件类型的 Error 终态。
    fn upstream_error(message: String) -> Self;
}

impl<U: Clone + Send + 'static> TerminalEvent for SubjectEvent<U> {
    fn terminal_kind(&self) -> Option<TerminalKind> {
        match self {
            Self::Next(_) => None,
            Self::Completed => Some(TerminalKind::Completed),
            Self::Error(_) => Some(TerminalKind::Error),
        }
    }

    fn upstream_error(message: String) -> Self {
        Self::Error(SubjectError::Upstream(message))
    }
}

/// Subject 终态类型。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminalKind {
    /// 正常完成。
    Completed,
    /// 错误终止。
    Error,
}

/// 拥有 `Subject::connect` 转发任务的连接句柄。
pub struct ConnectionHandle {
    cancel: watch::Sender<bool>,
    aborts: Vec<tokio::task::AbortHandle>,
    stopped: Arc<AtomicBool>,
}

impl ConnectionHandle {
    /// 停止所有转发任务。多次调用是幂等的。
    pub fn stop(&self) {
        if !self.stopped.swap(true, Ordering::AcqRel) {
            let _ = self.cancel.send(true);
        }
        for abort in &self.aborts {
            abort.abort();
        }
    }

    /// 连接是否已停止或已因终态自动结束。
    #[must_use]
    pub fn is_stopped(&self) -> bool {
        self.stopped.load(Ordering::Acquire)
    }
}

impl Drop for ConnectionHandle {
    fn drop(&mut self) {
        self.stop();
    }
}

/// `Subject::connect` 的合流 subject 与其所有权句柄。
pub struct ConnectedSubject<T: Clone + Send + 'static> {
    subject: Subject<T>,
    connection: ConnectionHandle,
}

impl<T: Clone + Send + 'static> ConnectedSubject<T> {
    /// 访问合流 subject。
    #[must_use]
    pub fn subject(&self) -> &Subject<T> {
        &self.subject
    }

    /// 访问连接所有权句柄。
    #[must_use]
    pub fn connection(&self) -> &ConnectionHandle {
        &self.connection
    }

    /// 显式停止转发任务。
    pub fn stop(&self) {
        self.connection.stop();
    }
}

impl<T: Clone + Send + 'static> std::ops::Deref for ConnectedSubject<T> {
    type Target = Subject<T>;

    fn deref(&self) -> &Self::Target {
        &self.subject
    }
}

/// 流错误
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum StreamError {
    /// 上游抛错(对应 RxPy `on_error`)
    #[error("upstream error: {0}")]
    Upstream(String),
    /// Lagged(消费者跟不上)
    #[error("subscriber lagged: skipped {0} items")]
    Lagged(u64),
    /// 流已结束(对应 RxPy `on_completed`)
    #[error("stream completed")]
    Completed,
    /// Subject 已被 Completed/Error 终止。
    #[error("subject already terminated")]
    Terminated,
}

/// `SubscriberStream` —— Subject 订阅的流式包装
pub struct SubscriberStream<T> {
    inner: Option<BroadcastStream<SubjectSignal<T>>>,
    pending_terminal: Option<T>,
    done: bool,
}

// SubscriberStream 不保存指向自身字段的引用,移动 pending value 是安全的。
impl<T> Unpin for SubscriberStream<T> {}

// BroadcastStream 要求 T: Clone + Send + 'static 才能 impl Stream,
// 因此 SubscriberStream 也必须相同 trait bounds。
impl<T: Clone + Send + 'static> Stream for SubscriberStream<T> {
    type Item = Result<T, StreamError>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();
        if this.done {
            return Poll::Ready(None);
        }
        if let Some(terminal) = this.pending_terminal.take() {
            this.done = true;
            return Poll::Ready(Some(Ok(terminal)));
        }
        let Some(inner) = this.inner.as_mut() else {
            this.done = true;
            return Poll::Ready(None);
        };
        match Pin::new(inner).poll_next(cx) {
            Poll::Ready(Some(Ok(SubjectSignal::Next(v)))) => Poll::Ready(Some(Ok(v))),
            Poll::Ready(Some(Ok(SubjectSignal::Terminal(v)))) => {
                this.done = true;
                this.inner = None;
                Poll::Ready(Some(Ok(v)))
            }
            // BroadcastStreamRecvError::Lagged(u64) 表示跳过的消息数;
            // 我们把它映射为 StreamError::Lagged(u64)。
            Poll::Ready(Some(Err(BroadcastStreamRecvError::Lagged(lag)))) => {
                Poll::Ready(Some(Err(StreamError::Lagged(lag))))
            }
            Poll::Ready(None) => Poll::Ready(None),
            Poll::Pending => Poll::Pending,
        }
    }
}

/// 共享 Subject 的便捷 alias
pub type SharedSubject<T> = std::sync::Arc<Subject<T>>;

/// Subject 持有 `broadcast::Sender<T>`,本身可 Clone:多个 owner 推同一组订阅者
impl<T: Clone + Send + 'static> Clone for Subject<T> {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
        }
    }
}

fn lock_unpoisoned<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures::StreamExt;

    #[tokio::test]
    async fn subject_broadcast() {
        let subject = Subject::<u32>::new(16);
        let mut sub_a = subject.subscribe();
        let mut sub_b = subject.subscribe();

        subject.push(1).unwrap();
        subject.push(2).unwrap();

        assert_eq!(sub_a.next().await.unwrap().unwrap(), 1);
        assert_eq!(sub_a.next().await.unwrap().unwrap(), 2);
        assert_eq!(sub_b.next().await.unwrap().unwrap(), 1);
        assert_eq!(sub_b.next().await.unwrap().unwrap(), 2);
    }

    #[tokio::test]
    async fn subject_without_subscribers_is_a_successful_noop() {
        let subject = Subject::<u32>::new(16);
        assert_eq!(subject.push(1), Ok(()));
    }

    #[test]
    fn video_frame_rgb24_roundtrip() {
        let frame = VideoFrame::rgb24(4, 4, vec![0u8; 4 * 4 * 3]);
        assert_eq!(frame.width, Some(4));
        assert_eq!(frame.height, Some(4));
        assert_eq!(frame.shape, vec![4, 4, 3]);
        assert_eq!(frame.dtype, "|u1");
        assert_eq!(frame.strides, vec![12, 3, 1]);
        assert_eq!(frame.data.len(), 48);
        assert_eq!(frame.format.as_deref(), Some("rgb24"));
        frame.validate_layout().unwrap();
    }

    #[test]
    fn video_frame_serializable_matches_python() {
        // 字段集合与 dimos `SerializableVideoFrame` 完全一致
        let frame = VideoFrame {
            data: vec![1, 2, 3],
            shape: vec![1, 1, 3],
            dtype: "|u1".into(),
            strides: vec![3, 3, 1],
            byte_offset: 0,
            pts: Some(100),
            time: Some(0.5),
            dts: Some(0),
            width: Some(1),
            height: Some(1),
            format: Some("rgb24".into()),
        };
        assert!(frame.pts.is_some());
        assert!(frame.time.is_some());
        assert_eq!(frame.width, Some(1));
        frame.validate_layout().unwrap();
    }

    #[tokio::test]
    async fn subject_event_supports_next_completed_error() {
        let subject = Subject::<SubjectEvent<u32>>::new(16);
        let mut sub = subject.subscribe();

        subject.push_next(7).unwrap();
        subject
            .error(SubjectError::Upstream("boom".into()))
            .unwrap();
        assert_eq!(subject.complete(), Err(StreamError::Terminated));
        assert_eq!(subject.push_next(9), Err(StreamError::Terminated));

        let first = sub.next().await.unwrap().unwrap();
        match first {
            SubjectEvent::Next(v) => assert_eq!(v, 7),
            _ => panic!("expected Next(7)"),
        }
        let second = sub.next().await.unwrap().unwrap();
        match second {
            SubjectEvent::Error(SubjectError::Upstream(msg)) => assert_eq!(msg, "boom"),
            _ => panic!("expected Error(Upstream)"),
        }
        assert!(sub.next().await.is_none());
    }
}

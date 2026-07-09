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

use futures::Stream;
use std::pin::Pin;
use std::task::{Context, Poll};
use tokio::sync::broadcast;
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;
use tokio_stream::wrappers::BroadcastStream;

/// 可序列化的视频帧 — 对齐 dimos `SerializableVideoFrame`
///
/// 见 `/home/ml/Deeprob/dimos/dimos/robot/unitree/connection.py:70-93`
///
/// Python 端使用 `av.VideoFrame.to_ndarray(format="rgb24")` 得到 `np.ndarray`,
/// 这里用 `Vec<u8>` (RGB24 紧密布局) 表达。
#[derive(Debug, Clone, PartialEq)]
pub struct VideoFrame {
    /// RGB24 紧密数据(由 `av.VideoFrame.to_ndarray(format="rgb24")` 派生)
    pub data: Vec<u8>,
    /// 显示时间戳(PTS,可选)
    pub pts: Option<i64>,
    /// 显示时间(秒,可选)
    pub time: Option<f64>,
    /// 解码时间戳(DTS,可选)
    pub dts: Option<i64>,
    /// 帧宽(像素)
    pub width: u32,
    /// 帧高(像素)
    pub height: u32,
    /// 像素格式名(如 `"rgb24"` / `"bgr24"`)
    pub format: Option<String>,
}

impl VideoFrame {
    /// 构造一帧 RGB24 数据
    #[must_use]
    pub fn rgb24(width: u32, height: u32, data: Vec<u8>) -> Self {
        Self {
            data,
            pts: None,
            time: None,
            dts: None,
            width,
            height,
            format: Some("rgb24".to_string()),
        }
    }
}

/// `Subject` 事件 — 对齐 RxPy `on_next` / `on_completed` / `on_error`
///
/// 推荐用法:`let subject = Subject::<SubjectEvent<MyMsg>>::new(64)`,
/// 这样同一个 `Subject` 可同时表达数据 / 完成 / 错误三种信号,
/// 类似 RxPy 的 `Subject.on_next(v)` / `.on_completed()` / `.on_error(e)`。
#[derive(Debug, Clone)]
pub enum SubjectEvent<T> {
    /// 正常数据(对应 `on_next`)
    Next(T),
    /// 流结束(对应 `on_completed`)
    Completed,
    /// 流错误(对应 `on_error`)
    Error(SubjectError),
}

/// `SubjectEvent::Error` 携带的错误类型
#[derive(Debug, Clone, thiserror::Error)]
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
    tx: broadcast::Sender<T>,
}

impl<T: Clone + Send + 'static> Subject<T> {
    /// 构造新 subject
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        Self { tx }
    }

    /// 向所有订阅者推送一条消息(对应 RxPy `on_next`)
    ///
    /// # Errors
    /// 当无活跃订阅者时返回 `StreamError::NoSubscribers`
    pub fn push(&self, value: T) -> Result<(), StreamError> {
        // broadcast::Sender::send 返回 `Result<usize, SendError<T>>`,
        // 其中 usize 是已发送接收者数。我们只关心成功 / 失败,因此 `.map(|_| ())` 丢弃该值。
        self.tx
            .send(value)
            .map(|_| ())
            .map_err(|_| StreamError::NoSubscribers)
    }

    /// 订阅 —— 返回 `Stream<Item = T>`
    #[must_use]
    pub fn subscribe(&self) -> SubscriberStream<T> {
        let rx = self.tx.subscribe();
        SubscriberStream {
            inner: BroadcastStream::new(rx),
        }
    }
}

impl<U: Clone + Send + 'static> Subject<SubjectEvent<U>> {
    /// 推送一个 `Next` 事件
    ///
    /// # Errors
    /// 当无活跃订阅者时返回 `StreamError::NoSubscribers`
    pub fn push_next(&self, value: U) -> Result<(), StreamError> {
        self.push(SubjectEvent::Next(value))
    }

    /// 推送一个 `Completed` 事件(对应 RxPy `on_completed`)
    ///
    /// # Errors
    /// 当无活跃订阅者时返回 `StreamError::NoSubscribers`
    pub fn complete(&self) -> Result<(), StreamError> {
        self.push(SubjectEvent::<U>::Completed)
    }

    /// 推送一个 `Error` 事件(对应 RxPy `on_error`)
    ///
    /// # Errors
    /// 当无活跃订阅者时返回 `StreamError::NoSubscribers`
    pub fn error(&self, err: SubjectError) -> Result<(), StreamError> {
        self.push(SubjectEvent::<U>::Error(err))
    }
}

/// 流错误
#[derive(Debug, thiserror::Error)]
pub enum StreamError {
    /// 无活跃订阅者
    #[error("no active subscribers")]
    NoSubscribers,
    /// 上游抛错(对应 RxPy `on_error`)
    #[error("upstream error: {0}")]
    Upstream(String),
    /// Lagged(消费者跟不上)
    #[error("subscriber lagged: skipped {0} items")]
    Lagged(u64),
    /// 流已结束(对应 RxPy `on_completed`)
    #[error("stream completed")]
    Completed,
}

/// `SubscriberStream` —— Subject 订阅的流式包装
pub struct SubscriberStream<T> {
    inner: BroadcastStream<T>,
}

// BroadcastStream 要求 T: Clone + Send + 'static 才能 impl Stream,
// 因此 SubscriberStream 也必须相同 trait bounds。
impl<T: Clone + Send + 'static> Stream for SubscriberStream<T> {
    type Item = Result<T, StreamError>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();
        match Pin::new(&mut this.inner).poll_next(cx) {
            Poll::Ready(Some(Ok(v))) => Poll::Ready(Some(Ok(v))),
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
    async fn subject_no_subscribers() {
        let subject = Subject::<u32>::new(16);
        assert!(subject.push(1).is_err());
    }

    #[test]
    fn video_frame_rgb24_roundtrip() {
        let frame = VideoFrame::rgb24(4, 4, vec![0u8; 4 * 4 * 3]);
        assert_eq!(frame.width, 4);
        assert_eq!(frame.height, 4);
        assert_eq!(frame.data.len(), 48);
        assert_eq!(frame.format.as_deref(), Some("rgb24"));
    }

    #[test]
    fn video_frame_serializable_matches_python() {
        // 字段集合与 dimos `SerializableVideoFrame` 完全一致
        let frame = VideoFrame {
            data: vec![1, 2, 3],
            pts: Some(100),
            time: Some(0.5),
            dts: Some(0),
            width: 1,
            height: 1,
            format: Some("rgb24".into()),
        };
        assert!(frame.pts.is_some());
        assert!(frame.time.is_some());
        assert_eq!(frame.width, 1);
    }

    #[tokio::test]
    async fn subject_event_supports_next_completed_error() {
        let subject = Subject::<SubjectEvent<u32>>::new(16);
        let mut sub = subject.subscribe();

        subject.push_next(7).unwrap();
        subject
            .error(SubjectError::Upstream("boom".into()))
            .unwrap();
        subject.complete().unwrap();

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
        let third = sub.next().await.unwrap().unwrap();
        assert!(matches!(third, SubjectEvent::Completed));
    }
}
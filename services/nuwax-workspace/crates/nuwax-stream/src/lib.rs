//! nuwax 流式数据原语
//! 提供 Subject / Observer 抽象，与 `nuwax-core::In<T>/Out<T>` 互转。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use futures::Stream;
use std::pin::Pin;
use std::task::{Context, Poll};
use tokio::sync::broadcast;
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;
use tokio_stream::wrappers::BroadcastStream;

/// Subject——多订阅者广播
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

    /// 向所有订阅者推送一条消息
    ///
    /// # Errors
    /// 当无活跃订阅者时返回 `StreamError::NoSubscribers`
    pub fn push(&self, value: T) -> Result<(), StreamError> {
        // broadcast::Sender::send 返回 `Result<usize, SendError<T>>`，
        // 其中 usize 是已发送接收者数。我们只关心成功 / 失败，因此 `.map(|_| ())` 丢弃该值。
        self.tx
            .send(value)
            .map(|_| ())
            .map_err(|_| StreamError::NoSubscribers)
    }

    /// 订阅——返回 Stream<Item = T>
    #[must_use]
    pub fn subscribe(&self) -> SubscriberStream<T> {
        let rx = self.tx.subscribe();
        SubscriberStream {
            inner: BroadcastStream::new(rx),
        }
    }
}

/// 流错误
#[derive(Debug, thiserror::Error)]
pub enum StreamError {
    /// 无活跃订阅者
    #[error("no active subscribers")]
    NoSubscribers,
    /// Lagged（消费者跟不上）
    #[error("subscriber lagged: skipped {0} items")]
    Lagged(u64),
}

/// SubscriberStream——Subject 订阅的流式包装
pub struct SubscriberStream<T> {
    inner: BroadcastStream<T>,
}

// BroadcastStream 要求 T: Clone + Send + 'static 才能 impl Stream，
// 因此 SubscriberStream 也必须相同 trait bounds。
impl<T: Clone + Send + 'static> Stream for SubscriberStream<T> {
    type Item = Result<T, StreamError>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();
        match Pin::new(&mut this.inner).poll_next(cx) {
            Poll::Ready(Some(Ok(v))) => Poll::Ready(Some(Ok(v))),
            // BroadcastStreamRecvError::Lagged(u64) 表示跳过的消息数；
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
}

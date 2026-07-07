//! nuwax Module 基类与 In[T]/Out[T] 类型化流
//! - `In[T]` / `Out[T]` 类型化流

#![deny(unsafe_code)]
#![warn(missing_docs)]

use async_trait::async_trait;
use nuwax_types::Header;
use std::sync::Arc;
use thiserror::Error;
use tokio::sync::mpsc;

/// Module 生命周期错误
#[derive(Debug, Error)]
pub enum ModuleError {
    /// 已停止
    #[error("module already stopped")]
    AlreadyStopped,
    /// 内部通道关闭
    #[error("internal channel closed")]
    ChannelClosed,
    /// 其他错误
    #[error("module error: {0}")]
    Other(String),
}

/// Module 结果
pub type ModuleResult<T> = Result<T, ModuleError>;

/// nuwax Module trait
#[async_trait]
pub trait Module: Send + Sync + 'static {
    /// 启动模块
    async fn start(&self) -> ModuleResult<()>;
    /// 停止模块
    async fn stop(&self) -> ModuleResult<()>;
}

/// In[T]——输入流（消费者）
pub struct In<T> {
    rx: mpsc::Receiver<T>,
}

impl<T> In<T> {
    /// 接受下一条消息
    pub async fn recv(&mut self) -> Option<T> {
        self.rx.recv().await
    }
}

/// Out[T]——输出流（生产者）
pub struct Out<T> {
    tx: mpsc::Sender<T>,
}

impl<T> Out<T> {
    /// 发布一条消息
    ///
    /// # Errors
    /// 返回 `ModuleError::ChannelClosed` 若下游 In 已 drop
    pub async fn send(&self, msg: T) -> ModuleResult<()> {
        self.tx
            .send(msg)
            .await
            .map_err(|_| ModuleError::ChannelClosed)
    }
}

/// 创建连接的 In/Out 对（单向流水线）
#[must_use]
pub fn channel<T>(capacity: usize) -> (Out<T>, In<T>) {
    let (tx, rx) = mpsc::channel(capacity);
    (Out { tx }, In { rx })
}

/// 标准 Module——header + payload 点对点通信
#[derive(Debug, Clone)]
pub struct HeaderedMessage<T> {
    /// Header
    pub header: Header,
    /// payload
    pub payload: T,
}

/// 模块包装器——基于 `Arc` 的共享所有权
pub struct ModuleHandle<M: Module + ?Sized> {
    inner: Arc<M>,
}

impl<M: Module> ModuleHandle<M> {
    /// 构造共享 handle
    pub fn new(inner: M) -> Self {
        Self {
            inner: Arc::new(inner),
        }
    }

    /// 启动所有副本
    pub async fn start(&self) -> ModuleResult<()> {
        self.inner.start().await
    }

    /// 停止
    pub async fn stop(&self) -> ModuleResult<()> {
        self.inner.stop().await
    }

    /// 共享引用
    #[must_use]
    pub fn inner(&self) -> &M {
        &self.inner
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn channel_roundtrip() {
        let (out, mut inp) = channel::<u32>(8);
        out.send(42).await.unwrap();
        out.send(100).await.unwrap();
        drop(out);
        assert_eq!(inp.recv().await, Some(42));
        assert_eq!(inp.recv().await, Some(100));
        assert_eq!(inp.recv().await, None);
    }

    #[tokio::test]
    async fn channel_send_after_drop_fails() {
        let (out, inp) = channel::<u32>(1);
        drop(inp);
        assert!(out.send(1).await.is_err());
    }

    #[test]
    fn header_construct() {
        let h = Header::new("base_link");
        assert_eq!(h.frame_id, "base_link");
    }
}

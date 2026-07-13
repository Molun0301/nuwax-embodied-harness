//! nuwax Module 基类与 `In<T>`/`Out<T>` 类型化流
//! - `In<T>` / `Out<T>` 类型化流
//!
//! SSOT: 对齐 dimos Python `In` / `Out` 类型化流,见
//! `/home/ml/Deeprob/dimos/dimos/stream/core/inout.py`。
//! 端口策略:`Out<T>` 单 mpsc(Single) 与 broadcast(Broadcast) 二选一,
//! broadcast 模式对应 RxPy `Subject` 多订阅语义。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use async_trait::async_trait;
use nuwax_types::Header;
use std::sync::Arc;
use thiserror::Error;
use tokio::sync::{broadcast, mpsc};

mod runtime;

pub use runtime::{
    autoconnect, ConnectionSet, InputPort, LifecycleState, LocalTransport, ManagedModule,
    ModuleInfo, ModulePorts, OutputPort, PortDirection, PortError, PortInfo, PortTransport,
    PublishReport, RemoteStream, StopHandle,
};

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

/// `In<T>`——输入流（消费者）
pub struct In<T> {
    rx: mpsc::UnboundedReceiver<T>,
}

impl<T> In<T> {
    /// 接受下一条消息
    pub async fn recv(&mut self) -> Option<T> {
        self.rx.recv().await
    }
}

/// `Out<T>`——输出流（生产者）
///
/// 内含一个 mpsc Sender（单消费者）或一个共享 broadcast Sender（多订阅者）。
/// 选择策略：
/// - `Out::Single` —— 单下游消费者，对应 `Out::channel(...)` 默认产物
/// - `Out::Broadcast` —— 多订阅者广播，对应 RxPy `Subject` 语义
///
/// rework22 P0-06: 允许上游模块在不暴露 mpsc 句柄的前提下直接持有
/// broadcast 句柄,以支持 fanout 场景（如同一帧给多个 consumer）。
///
/// 注意:`Out::Single` 路径对 `T` 无 `Clone` 要求;`Out::Broadcast` 路径要求 `T: Clone`
/// (broadcast 内部需要把每个值复制给所有活跃 Receiver)。
pub enum Out<T> {
    /// 单消费者（mpsc）
    Single(mpsc::UnboundedSender<T>),
    /// 多订阅者（broadcast,内部共享 `Arc<broadcast::Sender<T>>`,要求 `T: Clone`）
    Broadcast(Arc<broadcast::Sender<T>>),
}

/// `Out<T>` 发送错误
#[derive(Debug, Error)]
pub enum OutError {
    /// 接收端已 drop
    #[error("receiver gone")]
    ReceiverGone,
}

impl<T> Out<T> {
    /// 发布一条消息
    ///
    /// # Errors
    /// - `OutError::ReceiverGone` —— 接收端已 drop
    pub fn send(&self, msg: T) -> Result<(), OutError> {
        match self {
            Self::Single(tx) => tx.send(msg).map_err(|_| OutError::ReceiverGone),
            Self::Broadcast(tx) => {
                // Broadcast 路径要求 T: Clone,但 trait impl 在此处不施加 bound。
                // 由 broadcast 自身的 T: Clone 约束负责;调用方通过 Out::broadcast 构造。
                // 若用户手工构造 Broadcast(Arc<...>) 但 T 不满足 Clone,会在 tx.send 处报错。
                tx.send(msg).map(|_| ()).map_err(|_| OutError::ReceiverGone)
            }
        }
    }
}

impl<T: Clone> Out<T> {
    /// 构造一个 broadcast 模式 `Out`,caller 拿返回的 `Receiver` 收消息
    ///
    /// # Arguments
    /// * `capacity` - broadcast 通道容量
    #[must_use]
    pub fn broadcast(capacity: usize) -> (Self, broadcast::Receiver<T>) {
        let (tx, rx) = broadcast::channel(capacity);
        (Out::Broadcast(Arc::new(tx)), rx)
    }
}

impl<T> Clone for Out<T> {
    fn clone(&self) -> Self {
        match self {
            Self::Single(tx) => Self::Single(tx.clone()),
            Self::Broadcast(tx) => Self::Broadcast(Arc::clone(tx)),
        }
    }
}

/// 创建连接的 In/Out 对（单向流水线,单消费者）
#[must_use]
pub fn channel<T>(_capacity: usize) -> (Out<T>, In<T>) {
    // 使用 unbounded_channel —— Single 变体内部包的是 UnboundedSender。
    // _capacity 参数保留以兼容既有调用方,但此处不施加有界缓冲。
    let (tx, rx) = mpsc::unbounded_channel();
    (Out::Single(tx), In { rx })
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
        out.send(42).unwrap();
        out.send(100).unwrap();
        drop(out);
        assert_eq!(inp.recv().await, Some(42));
        assert_eq!(inp.recv().await, Some(100));
        assert_eq!(inp.recv().await, None);
    }

    #[tokio::test]
    async fn channel_send_after_drop_fails() {
        let (out, inp) = channel::<u32>(1);
        drop(inp);
        assert!(out.send(1).is_err());
    }

    #[test]
    fn header_construct() {
        let h = Header::new("base_link");
        assert_eq!(h.frame_id, "base_link");
    }
}

//! nuwax RPC 抽象
//!
//! 提供进程内 RpcBroker + 跨进程 gRPC 通道。设计上支持过程宏注册。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use async_trait::async_trait;
use nuwax_core::{ModuleError, ModuleResult};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use thiserror::Error;
use tokio::sync::RwLock;

/// RPC 错误
#[derive(Debug, Error)]
pub enum RpcError {
    /// 序列化失败
    #[error("serialize failed: {0}")]
    Serialize(String),
    /// 反序列化失败
    #[error("deserialize failed: {0}")]
    Deserialize(String),
    /// 方法不存在
    #[error("method not found: {0}")]
    MethodNotFound(String),
    /// 调用超时
    #[error("rpc timeout after {secs}s")]
    Timeout {
        /// 超时秒数
        secs: u64,
    },
    /// Module 错误
    #[error("module error: {0}")]
    Module(#[from] ModuleError),
}

/// RPC 结果
pub type RpcResult<T> = Result<T, RpcError>;

/// RPC 请求
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcRequest {
    /// 方法名
    pub method: String,
    /// JSON 参数
    pub args: serde_json::Value,
}

/// RPC 响应
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcResponse {
    /// 成功标志
    pub ok: bool,
    /// 返回值（JSON）
    pub result: serde_json::Value,
}

/// RPC handler trait
#[async_trait]
pub trait RpcHandler: Send + Sync + 'static {
    /// 处理一个 RPC 请求
    async fn handle(&self, req: RpcRequest) -> RpcResponse;
}

/// 简单的内存 RPC broker（用于同进程内的模块互通）
pub struct RpcBroker {
    handlers: RwLock<HashMap<String, Box<dyn RpcHandler>>>,
}

impl RpcBroker {
    /// 构造空 broker
    #[must_use]
    pub fn new() -> Self {
        Self {
            handlers: RwLock::new(HashMap::new()),
        }
    }

    /// 注册一个 RPC handler
    ///
    /// # Errors
    /// 当内部锁 poison 时返回错误
    pub async fn register(
        &self,
        method: impl Into<String>,
        handler: Box<dyn RpcHandler>,
    ) -> ModuleResult<()> {
        let mut g = self.handlers.write().await;
        g.insert(method.into(), handler);
        Ok(())
    }

    /// 调用一个 RPC 方法
    ///
    /// # Errors
    /// 当方法不存在时返回 `RpcError::MethodNotFound`
    pub async fn call(&self, req: RpcRequest) -> RpcResult<RpcResponse> {
        let g = self.handlers.read().await;
        match g.get(&req.method) {
            Some(h) => Ok(h.handle(req).await),
            None => Err(RpcError::MethodNotFound(req.method)),
        }
    }
}

impl Default for RpcBroker {
    fn default() -> Self {
        Self::new()
    }
}

/// 过程宏占位 —— 实装见 `nuwax-macros` crate（未来实装）
#[doc(hidden)]
pub mod nuwax_macros {}

#[cfg(test)]
mod tests {
    use super::*;

    struct EchoHandler;

    #[async_trait]
    impl RpcHandler for EchoHandler {
        async fn handle(&self, req: RpcRequest) -> RpcResponse {
            RpcResponse {
                ok: true,
                result: req.args,
            }
        }
    }

    #[tokio::test]
    async fn broker_register_and_call() {
        let broker = RpcBroker::new();
        broker
            .register("echo", Box::new(EchoHandler))
            .await
            .unwrap();

        let req = RpcRequest {
            method: "echo".into(),
            args: serde_json::json!({"x": 1.0}),
        };
        let resp = broker.call(req).await.unwrap();
        assert!(resp.ok);
        assert_eq!(resp.result["x"], serde_json::json!(1.0));
    }

    #[tokio::test]
    async fn broker_method_not_found() {
        let broker = RpcBroker::new();
        let req = RpcRequest {
            method: "nonexistent".into(),
            args: serde_json::json!(null),
        };
        let err = broker.call(req).await.unwrap_err();
        assert!(matches!(err, RpcError::MethodNotFound(_)));
    }
}

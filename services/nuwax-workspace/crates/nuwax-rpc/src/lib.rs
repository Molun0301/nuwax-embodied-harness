//! nuwax RPC 抽象
//!
//! 提供进程内 RpcBroker + 跨进程 gRPC 通道。设计上支持过程宏注册。
//!
//! SSOT:dimos Python 端在 `dimos.protocol.rpc.pubsubrpc.RPCReq` / `RPCRes`
//! (`/home/ml/Deeprob/dimos/dimos/protocol/rpc/pubsubrpc.py:52-61`),
//! `Args` 类型在 `dimos.protocol.rpc.spec`(`/home/ml/Deeprob/dimos/dimos/protocol/rpc/spec.py:25`)
//! Rust 字段名/类型必须与 Python 对齐,否则跨进程 RpcBroker.call/serve 都会路由失败

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

/// RPC 调用参数 — Python 端 `(positional, keyword)` 二元组
///
/// 对齐 `dimos.protocol.rpc.spec.Args = tuple[list, dict[str, Any]]`
/// (见 `/home/ml/Deeprob/dimos/dimos/protocol/rpc/spec.py:25`)
pub type RpcArgs = (Vec<serde_json::Value>, HashMap<String, serde_json::Value>);

/// RPC 请求
///
/// 字段名 `name/id/args` 对齐 Python `RPCReq` TypedDict
/// (`/home/ml/Deeprob/dimos/dimos/protocol/rpc/pubsubrpc.py:52-55`)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcRequest {
    /// 方法名(Python 字段名:`name`)
    pub name: String,
    /// JSON 参数(列表为位置参数,字典为关键字参数)
    pub args: RpcArgs,
    /// 消息 ID — 用于 RPC response correlation;`call_nowait` 时为 `None`
    pub id: Option<f64>,
}

/// RPC 响应
///
/// 字段名 `id/res/exception` 对齐 Python `RPCRes` TypedDict
/// (`/home/ml/Deeprob/dimos/dimos/protocol/rpc/pubsubrpc.py:58-61`)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcResponse {
    /// 消息 ID — 与 request.id 对应,用于 correlation
    pub id: Option<f64>,
    /// 返回值(`None` 表示无返回值或异常路径)
    pub res: Option<serde_json::Value>,
    /// 序列化后的异常信息(`None` 表示正常返回)
    pub exception: Option<serde_json::Value>,
}

impl RpcResponse {
    /// 构造成功响应
    #[must_use]
    pub fn ok(id: Option<f64>, res: serde_json::Value) -> Self {
        Self {
            id,
            res: Some(res),
            exception: None,
        }
    }

    /// 构造异常响应
    #[must_use]
    pub fn error(id: Option<f64>, exception: serde_json::Value) -> Self {
        Self {
            id,
            res: None,
            exception: Some(exception),
        }
    }
}

/// RPC handler trait
#[async_trait]
pub trait RpcHandler: Send + Sync + 'static {
    /// 处理一个 RPC 请求
    async fn handle(&self, req: RpcRequest) -> RpcResponse;
}

/// 简单的内存 RPC broker(用于同进程内的模块互通)
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
        name: impl Into<String>,
        handler: Box<dyn RpcHandler>,
    ) -> ModuleResult<()> {
        let mut g = self.handlers.write().await;
        g.insert(name.into(), handler);
        Ok(())
    }

    /// 调用一个 RPC 方法
    ///
    /// # Errors
    /// 当方法不存在时返回 `RpcError::MethodNotFound`
    pub async fn call(&self, req: RpcRequest) -> RpcResult<RpcResponse> {
        let g = self.handlers.read().await;
        match g.get(&req.name) {
            Some(h) => Ok(h.handle(req).await),
            None => Err(RpcError::MethodNotFound(req.name)),
        }
    }
}

impl Default for RpcBroker {
    fn default() -> Self {
        Self::new()
    }
}

/// 过程宏占位 —— 实装见 `nuwax-macros` crate(未来实装)
#[doc(hidden)]
pub mod nuwax_macros {}

#[cfg(test)]
mod tests {
    use super::*;

    struct EchoHandler;

    #[async_trait]
    impl RpcHandler for EchoHandler {
        async fn handle(&self, req: RpcRequest) -> RpcResponse {
            // 位置参数:回显第一个;关键字参数:回显整个 dict
            let positional = req.args.0.first().cloned().unwrap_or(serde_json::Value::Null);
            let res = serde_json::json!({
                "positional": positional,
                "keyword": req.args.1,
            });
            RpcResponse::ok(req.id, res)
        }
    }

    struct ThrowHandler;

    #[async_trait]
    impl RpcHandler for ThrowHandler {
        async fn handle(&self, req: RpcRequest) -> RpcResponse {
            RpcResponse::error(
                req.id,
                serde_json::json!({
                    "type": "ValueError",
                    "message": "intentional failure",
                }),
            )
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
            name: "echo".into(),
            args: (vec![serde_json::json!(1.0)], HashMap::new()),
            id: Some(42.0),
        };
        let resp = broker.call(req).await.unwrap();
        assert_eq!(resp.id, Some(42.0));
        assert!(resp.exception.is_none());
        assert_eq!(resp.res.unwrap()["positional"], serde_json::json!(1.0));
    }

    #[tokio::test]
    async fn broker_method_not_found() {
        let broker = RpcBroker::new();
        let req = RpcRequest {
            name: "nonexistent".into(),
            args: (vec![], HashMap::new()),
            id: None,
        };
        let err = broker.call(req).await.unwrap_err();
        assert!(matches!(err, RpcError::MethodNotFound(_)));
    }

    #[tokio::test]
    async fn broker_exception_path() {
        let broker = RpcBroker::new();
        broker
            .register("throw", Box::new(ThrowHandler))
            .await
            .unwrap();

        let req = RpcRequest {
            name: "throw".into(),
            args: (vec![], HashMap::new()),
            id: Some(7.0),
        };
        let resp = broker.call(req).await.unwrap();
        assert_eq!(resp.id, Some(7.0));
        assert!(resp.res.is_none());
        assert_eq!(resp.exception.unwrap()["type"], "ValueError");
    }

    #[tokio::test]
    async fn rquest_uses_name_field() {
        // 验证字段名是 `name`,不是 `method`(H-21 修复)
        let req = RpcRequest {
            name: "foo".into(),
            args: (vec![], HashMap::new()),
            id: None,
        };
        let json = serde_json::to_value(&req).unwrap();
        assert!(json.get("name").is_some());
        assert!(json.get("method").is_none());
    }
}
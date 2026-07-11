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
    /// 后端不可用(例如 `serve()` 调用时 `link-liblcm` feature 未启用)
    #[error("rpc backend unavailable: {0}")]
    BackendUnavailable(String),
    /// handler 异常
    #[error("rpc handler error: {0}")]
    Handler(String),
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

/// `serve()` 返回的 server handle —— 用于 stop / inspect
///
/// 当前是占位(handle 内部仅有 task handle 与名称),后续 link-liblcm 实现后扩展。
#[derive(Debug)]
pub struct RpcServerHandle {
    /// 服务名(LCM channel 前缀)
    pub service_name: String,
    /// 后端实现 tag("in-memory" / "lcm")
    pub backend: &'static str,
}

impl RpcServerHandle {
    /// 构造 in-memory 占位 handle(只在 in-memory broker 场景使用)
    #[must_use]
    pub fn in_memory(service_name: impl Into<String>) -> Self {
        Self {
            service_name: service_name.into(),
            backend: "in-memory",
        }
    }

    /// 构造 LCM backend 占位 handle
    #[must_use]
    pub fn lcm(service_name: impl Into<String>) -> Self {
        Self {
            service_name: service_name.into(),
            backend: "lcm",
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

    /// 进程内直接调用(不走 LCM transport)
    ///
    /// 便捷重载,内部用 `call(RpcRequest)` 实现;若 method 不存在返回 `RpcError::MethodNotFound`,
    /// 若 handler 返回异常响应,返回 `RpcError::Handler(异常 message)`。
    ///
    /// # Errors
    /// - `RpcError::MethodNotFound` —— 方法未注册
    /// - `RpcError::Handler` —— handler 返回 `RpcResponse.exception`
    pub async fn call_method(
        &self,
        method: &str,
        args: Vec<serde_json::Value>,
    ) -> Result<serde_json::Value, RpcError> {
        let req = RpcRequest {
            name: method.into(),
            args: (args, HashMap::new()),
            id: None,
        };
        let resp = self.call(req).await?;
        if let Some(exception) = resp.exception {
            let msg = exception
                .get("message")
                .and_then(|v| v.as_str())
                .unwrap_or("handler returned exception")
                .to_string();
            return Err(RpcError::Handler(msg));
        }
        Ok(resp.res.unwrap_or(serde_json::Value::Null))
    }

    /// 真实 LCM Req/Res 服务端
    ///
    /// 默认 feature off(无 `link-liblcm`)时,**返 `RpcError::BackendUnavailable` 而非编译失败**,
    /// 以便 dev / 静态验证场景不依赖 liblcm 也能跑通。
    ///
    /// 启用 `link-liblcm` feature 后,`serve()` 才真正注册到 LCM transport;
    /// 当前实现为占位(`unimplemented!`),后续 follow-up 接 nuwax-lcm LCM Req/Res 通道。
    ///
    /// # Errors
    /// - 默认 feature off 时永远返 `RpcError::BackendUnavailable`
    /// - 启用 `link-liblcm` 时当前 `unimplemented!`
    #[cfg(feature = "link-liblcm")]
    pub async fn serve(
        &self,
        _transport: std::sync::Arc<dyn std::any::Any + Send + Sync>,
        _service_name: &str,
    ) -> Result<RpcServerHandle, RpcError> {
        // rework22 P0-06:LCM transport 集成留待 follow-up。
        // 当前 dev 不链 liblcm,默认 feature off 路径已覆盖。
        unimplemented!("rework22 P0-06:LCM transport integration 待 follow-up")
    }

    /// 默认 feature off 路径 —— `serve()` 立即返错,确保 cargo check 在无 liblcm 时也能通过
    #[cfg(not(feature = "link-liblcm"))]
    pub async fn serve(
        &self,
        _transport: std::sync::Arc<dyn std::any::Any + Send + Sync>,
        _service_name: &str,
    ) -> Result<RpcServerHandle, RpcError> {
        Err(RpcError::BackendUnavailable(
            "RpcBroker::serve() 需要启用 link-liblcm feature".into(),
        ))
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
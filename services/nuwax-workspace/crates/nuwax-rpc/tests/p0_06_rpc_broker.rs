//! P0-06 RpcBroker 测试
//!
//! rework22 验证：
//! - `call_method()` 走进程内 handler 路径,成功时返 handler 结果
//! - `call_method()` 对未知方法返 `RpcError::MethodNotFound`
//! - `serve()` 在默认 feature off 时返 `RpcError::BackendUnavailable`,不触发 liblcm 链接
//! - `call_method()` handler 抛异常时返回 `RpcError::Handler`

use nuwax_rpc::{RpcError, RpcHandler, RpcRequest, RpcResponse, RpcServerHandle};
use serde_json::Value;
use std::sync::Arc;

struct EchoHandler;

#[async_trait::async_trait]
impl RpcHandler for EchoHandler {
    async fn handle(&self, req: RpcRequest) -> RpcResponse {
        // 回显第一个位置参数
        let positional = req.args.0.first().cloned().unwrap_or(Value::Null);
        RpcResponse::ok(req.id, positional)
    }
}

struct ThrowHandler;

#[async_trait::async_trait]
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
async fn p0_06_rpc_broker_call_returns_handler_result() {
    let broker = nuwax_rpc::RpcBroker::new();
    broker
        .register("echo", Box::new(EchoHandler))
        .await
        .unwrap();

    let result = broker
        .call_method("echo", vec![Value::from(42i64)])
        .await
        .unwrap();
    assert_eq!(result, Value::from(42i64));
}

#[tokio::test]
async fn p0_06_rpc_broker_call_unknown_method_errors() {
    let broker = nuwax_rpc::RpcBroker::new();
    let err = broker
        .call_method("nonexistent", vec![])
        .await
        .unwrap_err();
    assert!(matches!(err, RpcError::MethodNotFound(_)));
}

#[tokio::test]
async fn p0_06_rpc_broker_serve_default_feature_returns_backend_unavailable() {
    // 默认 feature off(无 link-liblcm)路径应返 BackendUnavailable,而不是 panic 或触发链 liblcm。
    let broker = nuwax_rpc::RpcBroker::new();
    let transport: Arc<dyn std::any::Any + Send + Sync> = Arc::new(());
    let err = broker
        .serve(transport, "test_service")
        .await
        .unwrap_err();
    assert!(matches!(err, RpcError::BackendUnavailable(_)));
}

#[tokio::test]
async fn p0_06_rpc_broker_call_handler_exception_surfaces_as_handler_error() {
    let broker = nuwax_rpc::RpcBroker::new();
    broker
        .register("throw", Box::new(ThrowHandler))
        .await
        .unwrap();
    let err = broker.call_method("throw", vec![]).await.unwrap_err();
    assert!(matches!(err, RpcError::Handler(_)));
}

#[test]
fn p0_06_rpc_server_handle_in_memory_construct() {
    // 占位 handle 构造 + 字段读取(cargo check 不依赖 link-liblcm)
    let h = RpcServerHandle::in_memory("svc");
    assert_eq!(h.service_name, "svc");
    assert_eq!(h.backend, "in-memory");
    let h2 = RpcServerHandle::lcm("svc2");
    assert_eq!(h2.backend, "lcm");
}

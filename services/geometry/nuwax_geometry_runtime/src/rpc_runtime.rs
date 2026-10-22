//! DimOS-compatible LCM RPC facade for the production geometry runtime.

use async_trait::async_trait;
use nuwax_rpc::{
    RpcArgs, RpcBroker, RpcHandler, RpcRequest, RpcResponse, RpcResult, RpcServerGroup,
    RpcTransport,
};
use serde::de::DeserializeOwned;
use serde_json::{Map, Value};
use std::sync::Arc;
use tonic::Request;

use crate::proto::geometry_runtime_server::GeometryRuntime;
use crate::proto::*;
use crate::server::GeometryRuntimeService;

const RPC_NAME: &str = "GeometryRuntimeService/get_navigation_map_block";

#[derive(Debug)]
struct InvocationError {
    type_name: &'static str,
    message: String,
}

impl InvocationError {
    fn invalid_arguments(message: impl Into<String>) -> Self {
        Self {
            type_name: "InvalidArguments",
            message: message.into(),
        }
    }

    fn serialization(message: impl Into<String>) -> Self {
        Self {
            type_name: "SerializationError",
            message: message.into(),
        }
    }

    fn status(status: tonic::Status) -> Self {
        Self {
            type_name: "GeometryRuntimeStatus",
            message: format!("{}: {}", status.code(), status.message()),
        }
    }

    fn into_value(self) -> Value {
        serde_json::json!({
            "type_name": self.type_name,
            "type_module": "nuwax.geometry",
            "args": [self.message],
            "traceback": "",
        })
    }
}

fn decode_request<T: DeserializeOwned>(args: RpcArgs) -> Result<T, InvocationError> {
    let (mut positional, keyword) = args;
    let value = match (positional.len(), keyword.is_empty()) {
        (0, _) => Value::Object(keyword.into_iter().collect::<Map<String, Value>>()),
        (1, true) => positional.pop().expect("length checked above"),
        (1, false) => {
            return Err(InvocationError::invalid_arguments(
                "pass either one request object or keyword fields, not both",
            ));
        }
        (count, _) => {
            return Err(InvocationError::invalid_arguments(format!(
                "expected at most one positional request object, got {count}"
            )));
        }
    };
    serde_json::from_value(value)
        .map_err(|error| InvocationError::invalid_arguments(error.to_string()))
}

fn encode_navigation_map_block_response(response: NavigationMapBlockResponse) -> Result<Value, InvocationError> {
    use base64::{engine::general_purpose::STANDARD, Engine as _};
    match response.schema_version {
        2 => Ok(serde_json::json!({"schema_version":2,"root":response.root,
            "block":response.block,"data":response.data})),
        3 => {
            if response.data.is_empty()
                || response.data.len() > nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES
                || response.total_bytes as usize > nuwax_map_chunks::MAX_BLOCK_BYTES
                || response.offset as u64 + response.data.len() as u64 > response.total_bytes as u64 {
                return Err(InvocationError::serialization("精细块响应长度错误"));
            }
            // 只改变显式新版RPC的字节表示，避免数百万整数值及容器克隆。
            // 原不可变块、完整身份、拥有方和期限继续由原服务检查。
            Ok(serde_json::json!({"schema_version":3,"root":response.root,
                "block":response.block,"offset":response.offset,"total_bytes":response.total_bytes,
                "data_base64":STANDARD.encode(&response.data)}))
        }
        _ => Err(InvocationError::serialization("不支持的精细块响应版本")),
    }
}

struct GeometryRpcHandler {
    service: Arc<GeometryRuntimeService>,
}

impl GeometryRpcHandler {
    async fn invoke(&self, args: RpcArgs) -> Result<Value, InvocationError> {
        let request = decode_request::<NavigationMapBlockRequest>(args)?;
        let response = GeometryRuntime::get_navigation_map_block(&*self.service, Request::new(request))
            .await.map_err(InvocationError::status)?;
        encode_navigation_map_block_response(response.into_inner())
    }
}

#[async_trait]
impl RpcHandler for GeometryRpcHandler {
    async fn handle(&self, request: RpcRequest) -> RpcResponse {
        let request_id = request.id;
        match self.invoke(request.args).await {
            Ok(value) => RpcResponse::ok(request_id, value),
            Err(error) => RpcResponse::error(request_id, error.into_value()),
        }
    }
}

/// 保留导航栈实际消费的地图块接口，沿原RPC服务组拥有方启动和关闭。
pub async fn serve_geometry_rpc(
    service: GeometryRuntimeService,
    transport: Arc<dyn RpcTransport>,
) -> RpcResult<RpcServerGroup> {
    let broker = RpcBroker::new();
    broker.register(RPC_NAME, Box::new(GeometryRpcHandler { service: Arc::new(service) })).await?;
    broker.serve_many(transport, [RPC_NAME]).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn navigation_block_encoding_preserves_legacy_and_compact_bytes() {
        let original = vec![0, 1, 254, 255];
        for schema_version in [2, 3] {
            let value = encode_navigation_map_block_response(NavigationMapBlockResponse {
                schema_version, root: None, block: None, data: original.clone(), offset:0,total_bytes:4,
            }).unwrap();
            assert_eq!(value["schema_version"], schema_version);
            if schema_version == 2 {
                assert_eq!(value["data"], serde_json::json!([0, 1, 254, 255]));
                assert!(value.get("data_base64").is_none());
            } else {
                assert_eq!(value["data_base64"], "AAH+/w==", "标准编码的固定独立向量");
                assert!(value.get("data").is_none());
            }
        }
        for (schema_version, length) in [(3, nuwax_map_chunks::MAX_BLOCK_BYTES + 1), (4, 1)] {
            assert!(encode_navigation_map_block_response(NavigationMapBlockResponse {
                schema_version, root: None, block: None, data: vec![0; length],offset:0,total_bytes:length as u32,
            }).is_err());
        }
    }

    #[test]
    fn map_block_request_decoder_accepts_positional_or_keyword_fields() {
        let body = serde_json::to_value(NavigationMapBlockRequest {
            schema_version: 3, max_bytes: 7, ..Default::default()
        }).unwrap();
        let request = decode_request::<NavigationMapBlockRequest>((
            Vec::new(), body.as_object().unwrap().iter().map(|(key, value)| (key.clone(), value.clone())).collect(),
        )).unwrap();
        assert_eq!(request.schema_version, 3);
        assert_eq!(request.max_bytes, 7);
        assert!(decode_request::<NavigationMapBlockRequest>((vec![body.clone()], HashMap::new())).is_ok());
        let error = decode_request::<NavigationMapBlockRequest>((
            vec![body], HashMap::from([("schema_version".to_string(), Value::from(3))]),
        )).unwrap_err();
        assert_eq!(error.type_name, "InvalidArguments");
    }

    #[tokio::test]
    async fn map_block_rpc_uses_real_service_errors_and_closes_its_transport() {
        use nuwax_rpc::{LcmRpcTransport, RpcClient, RpcError};
        use std::time::Duration;
        let handle = crate::lcm_runtime::build_handle_for_tests().await.expect("真实服务句柄");
        let listener = std::net::TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener);
        let url = format!("udpm://239.255.76.67:{port}?ttl=0");
        let transport: Arc<dyn RpcTransport> = Arc::new(LcmRpcTransport::from_provider(&url).unwrap());
        let group = serve_geometry_rpc(GeometryRuntimeService::new(handle), transport.clone()).await.unwrap();
        assert_eq!(group.service_names(), &[RPC_NAME.to_owned()]);
        let client = RpcClient::new(transport, RPC_NAME).unwrap();
        let body = serde_json::to_value(NavigationMapBlockRequest {
            schema_version: 3, max_bytes: 7, ..Default::default()
        }).unwrap();
        let error = client.call((vec![body.clone()], HashMap::new()), Duration::from_secs(2)).await.unwrap_err();
        let RpcError::RemoteException(error) = error else { panic!("真实服务错误应经RPC返回") };
        assert_eq!(error["type_name"], "GeometryRuntimeStatus");
        assert!(error["args"][0].as_str().unwrap().contains("missing retained root"));
        let error = client.call((vec![Value::String("invalid".into())], HashMap::new()), Duration::from_secs(2)).await.unwrap_err();
        assert!(matches!(error, RpcError::RemoteException(error) if error["type_name"] == "InvalidArguments"));
        group.stop().await.expect("原拥有方等待实际退订完成");
        assert!(matches!(client.call((vec![body], HashMap::new()), Duration::from_millis(50)).await,
            Err(RpcError::Timeout { .. } | RpcError::Transport(_))));
    }
}

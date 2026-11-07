//! 导航栈保留的地图块 LCM（轻量通信与封送）接口；不属于远程会话协议。
use crate::session::SessionHandle;
use async_trait::async_trait;
use base64::{engine::general_purpose::STANDARD, Engine as _};
use nuwax_map_chunks::{ObjectId, ObjectIdentity, ObjectRangeRead, MAX_BLOCK_BYTES, MAX_BLOCK_TRANSFER_PART_BYTES};
use nuwax_rpc::{
    RpcArgs, RpcBroker, RpcHandler, RpcRequest, RpcResponse, RpcResult, RpcServerGroup,
    RpcTransport,
};
use serde::Deserialize;
use serde_json::Value;
use std::sync::Arc;
use tonic::Status;

const RPC_NAME: &str = "GeometryRuntimeService/get_navigation_map_block";

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BlockRequest {
    object_id: [u8; 16],
    offset: usize,
}

struct Handler(SessionHandle);
impl Handler {
    async fn invoke(&self, args: RpcArgs) -> Result<Value, Status> {
        let (mut positional, keywords) = args;
        let value = match (positional.len(), keywords.is_empty()) {
            (0, _) => serde_json::to_value(keywords)
                .map_err(|e| Status::invalid_argument(e.to_string()))?,
            (1, true) => positional.pop().unwrap(),
            _ => {
                return Err(Status::invalid_argument(
                    "地图块请求只接受一个对象或关键字参数",
                ))
            }
        };
        let request: BlockRequest =
            serde_json::from_value(value).map_err(|e| Status::invalid_argument(e.to_string()))?;
        let missing = || Status::not_found("地图块已被新修订替换或地图已切换");
        let root = self.0.mapper().map_err(|_| missing())?.fine_root().ok_or_else(missing)?;
        let read = tokio::task::spawn_blocking(move || root.repository.read_object_range(
            ObjectIdentity::Assigned(ObjectId(request.object_id)), MAX_BLOCK_BYTES,
            request.offset, MAX_BLOCK_TRANSFER_PART_BYTES,
        )).await.map_err(|_| missing())?.map_err(|error|
            Status::not_found(format!("地图块已被新修订替换或地图已切换：{error}")))?;
        let ObjectRangeRead::Data { bytes, total_bytes } = read else {
            return Err(Status::invalid_argument("地图块偏移超出范围"));
        };
        let mut response = serde_json::json!({"total_bytes":total_bytes});
        if let Some(compressed) = nuwax_codec::lcm::map_block_part::compress(&bytes) {
            response["data_lz4_base64"] = Value::String(STANDARD.encode(compressed));
        } else {
            response["data_base64"] = Value::String(STANDARD.encode(bytes));
        }
        Ok(response)
    }
}
#[async_trait]
impl RpcHandler for Handler {
    async fn handle(&self, request: RpcRequest) -> RpcResponse {
        match self.invoke(request.args).await {
            Ok(value) => RpcResponse::ok(request.id, value),
            Err(error) => RpcResponse::error(
                request.id,
                serde_json::json!({"type_name":"GeometryRuntimeStatus","type_module":"nuwax.geometry","args":[format!("{}: {}",error.code(),error.message())],"traceback":""}),
            ),
        }
    }
}
pub async fn start(session: SessionHandle) -> RpcResult<RpcServerGroup> {
    let url = nuwax_config::transport()
        .map_err(|e| nuwax_rpc::RpcError::InvalidServerConfig(e))?
        .lcm_url
        .clone();
    let transport: Arc<dyn RpcTransport> =
        Arc::new(nuwax_rpc::LcmRpcTransport::from_provider(&url)?);
    let broker = RpcBroker::new();
    broker
        .register(RPC_NAME, Box::new(Handler(session)))
        .await?;
    broker.serve_many(transport, [RPC_NAME]).await
}

#[cfg(test)]
pub(crate) async fn test_block(mapper: Arc<crate::mapper::Mapper>, object: ObjectIdentity) -> Result<Value, Status> {
    let ObjectIdentity::Assigned(id) = object;
    Handler(crate::session::test_block_session(mapper)).invoke((
        vec![serde_json::json!({"object_id":id.0,"offset":0})], Default::default(),
    )).await
}

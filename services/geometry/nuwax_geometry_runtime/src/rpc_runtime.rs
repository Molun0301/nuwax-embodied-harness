//! 导航栈保留的地图块 LCM（轻量通信与封送）接口；不属于远程会话协议。
use crate::session::SessionHandle;
use async_trait::async_trait;
use base64::{engine::general_purpose::STANDARD, Engine as _};
use nuwax_map_chunks::{BlockRef, ObjectId, ObjectIdentity, ObjectRangeRead, RootIdentity};
use nuwax_rpc::{
    RpcArgs, RpcBroker, RpcHandler, RpcRequest, RpcResponse, RpcResult, RpcServerGroup,
    RpcTransport,
};
use serde::Deserialize;
use serde_json::Value;
use std::sync::Arc;
use tonic::Status;

const RPC_NAME: &str = "GeometryRuntimeService/get_navigation_map_block";

// 以下仅保持导航栈现有地图块线上格式，冻结会话协议里没有这些消息。
#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
struct BlockRequest {
    schema_version: u32,
    root: Option<RootIdentity>,
    block: Option<WireBlock>,
    offset: u32,
    max_bytes: u32,
    part_encoding: Option<String>,
}
impl Default for BlockRequest {
    fn default() -> Self {
        Self {
            schema_version: 0,
            root: None,
            block: None,
            offset: 0,
            max_bytes: 0,
            part_encoding: None,
        }
    }
}
#[derive(Clone, serde::Serialize, Deserialize)]
struct WireObject {
    kind: u32,
    object_id: Vec<u8>,
}
#[derive(Clone, serde::Serialize, Deserialize)]
struct WireBlock {
    key: Vec<i32>,
    object: WireObject,
    block_revision: u64,
    geometry_revision: u64,
    points: u32,
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
        if !matches!(request.schema_version, 2 | 3)
            || (request.schema_version == 2 && (request.offset != 0 || request.max_bytes != 0))
            || (request.schema_version == 3
                && (request.max_bytes == 0
                    || request.max_bytes as usize
                        > nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES))
        {
            return Err(Status::invalid_argument("地图块协议版本或范围无效"));
        }
        let root = request
            .root
            .ok_or_else(|| Status::invalid_argument("缺少地图根"))?;
        let block = request
            .block
            .ok_or_else(|| Status::invalid_argument("缺少地图块"))?;
        let reference = BlockRef {
            key: block
                .key
                .as_slice()
                .try_into()
                .map_err(|_| Status::invalid_argument("地图块坐标须为三维"))?,
            object: match block.object.kind {
                1 => ObjectIdentity::Assigned(ObjectId(
                    block
                        .object
                        .object_id
                        .as_slice()
                        .try_into()
                        .map_err(|_| Status::invalid_argument("地图块对象须为16字节"))?,
                )),
                _ => return Err(Status::invalid_argument("地图块对象格式无效")),
            },
            block_revision: block.block_revision,
            geometry_revision: block.geometry_revision,
            points: block.points,
        };
        let range = (request.schema_version == 3)
            .then_some((request.offset as usize, request.max_bytes as usize));
        let read = self
            .0
            .mapper()?
            .navigation_map_block(root.clone(), reference, range)
            .await
            .map_err(Status::not_found)?;
        let ObjectRangeRead::Data { bytes, total_bytes } = read else {
            return Err(Status::invalid_argument("地图块偏移超出范围"));
        };
        if request.schema_version == 2 {
            if bytes.len() > nuwax_map_chunks::MAX_V1_TO_V5_BLOCK_BYTES {
                return Err(Status::invalid_argument("整块超出传输范围"));
            }
            return Ok(
                serde_json::json!({"schema_version":2,"root":root,"block":block,"data":bytes,"offset":0,"total_bytes":0}),
            );
        }
        let mut response = serde_json::json!({"schema_version":3,"root":root,"block":block,"offset":request.offset,"total_bytes":total_bytes});
        match request.part_encoding.as_deref().and_then(|encoding| {
            (encoding == "lz4")
                .then(|| nuwax_codec::lcm::map_block_part::compress(&bytes))
                .flatten()
        }) {
            Some(compressed) => {
                response["data_lz4_base64"] = Value::String(STANDARD.encode(compressed));
            }
            None => {
                response["data_base64"] = Value::String(STANDARD.encode(bytes));
            }
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
    let url =
        std::env::var("NUWAX_LCM_URL").unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=0".into());
    let transport: Arc<dyn RpcTransport> =
        Arc::new(nuwax_rpc::LcmRpcTransport::from_provider(&url)?);
    let broker = RpcBroker::new();
    broker
        .register(RPC_NAME, Box::new(Handler(session)))
        .await?;
    broker.serve_many(transport, [RPC_NAME]).await
}

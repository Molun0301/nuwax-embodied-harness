//! DimOS-compatible LCM RPC facade for the production geometry runtime.

use async_trait::async_trait;
use nuwax_rpc::{
    RpcArgs, RpcBroker, RpcHandler, RpcRequest, RpcResponse, RpcResult, RpcServerGroup,
    RpcTransport,
};
use serde::{de::DeserializeOwned, Serialize};
use serde_json::{Map, Value};
use std::sync::Arc;
use tonic::Request;

use crate::proto::geometry_runtime_server::GeometryRuntime;
use crate::proto::*;
use crate::server::GeometryRuntimeService;

const RPC_MODULE_NAME: &str = "GeometryRuntimeService";

// Streaming gRPC methods stay on their native stream/typed-LCM paths. Exposing
// them as one-shot pickle RPC would discard cancellation and backpressure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GeometryRpcMethod {
    Start,
    Stop,
    GetStatus,
    HealthCheck,
    GetCurrentPose,
    GetFrameTree,
    GetOccupancyGrid,
    GetOccupancyGridData,
    GetOccupancyGridShm,
    StartMapping,
    StopMapping,
    GetMappingState,
    ActivateMap,
    SetNavigationGoal,
    CancelNavigation,
    GetNavigationReadiness,
    GetMappingReadiness,
    GetExplorationReadiness,
    SetCmdVelSinkReady,
    GetFrontiers,
    StartExploration,
    StopExploration,
    GetExplorationState,
    GetExplorationDiagnostics,
    PublishLaserScan,
    PublishOdom,
    PublishPointCloud,
}

const RPC_METHODS: &[GeometryRpcMethod] = &[
    GeometryRpcMethod::Start,
    GeometryRpcMethod::Stop,
    GeometryRpcMethod::GetStatus,
    GeometryRpcMethod::HealthCheck,
    GeometryRpcMethod::GetCurrentPose,
    GeometryRpcMethod::GetFrameTree,
    GeometryRpcMethod::GetOccupancyGrid,
    GeometryRpcMethod::GetOccupancyGridData,
    GeometryRpcMethod::GetOccupancyGridShm,
    GeometryRpcMethod::StartMapping,
    GeometryRpcMethod::StopMapping,
    GeometryRpcMethod::GetMappingState,
    GeometryRpcMethod::ActivateMap,
    GeometryRpcMethod::SetNavigationGoal,
    GeometryRpcMethod::CancelNavigation,
    GeometryRpcMethod::GetNavigationReadiness,
    GeometryRpcMethod::GetMappingReadiness,
    GeometryRpcMethod::GetExplorationReadiness,
    GeometryRpcMethod::SetCmdVelSinkReady,
    GeometryRpcMethod::GetFrontiers,
    GeometryRpcMethod::StartExploration,
    GeometryRpcMethod::StopExploration,
    GeometryRpcMethod::GetExplorationState,
    GeometryRpcMethod::GetExplorationDiagnostics,
    GeometryRpcMethod::PublishLaserScan,
    GeometryRpcMethod::PublishOdom,
    GeometryRpcMethod::PublishPointCloud,
];

impl GeometryRpcMethod {
    const fn method_name(self) -> &'static str {
        match self {
            Self::Start => "start",
            Self::Stop => "stop",
            Self::GetStatus => "get_status",
            Self::HealthCheck => "health_check",
            Self::GetCurrentPose => "get_current_pose",
            Self::GetFrameTree => "get_frame_tree",
            Self::GetOccupancyGrid => "get_occupancy_grid",
            Self::GetOccupancyGridData => "get_occupancy_grid_data",
            Self::GetOccupancyGridShm => "get_occupancy_grid_shm",
            Self::StartMapping => "start_mapping",
            Self::StopMapping => "stop_mapping",
            Self::GetMappingState => "get_mapping_state",
            Self::ActivateMap => "activate_map",
            Self::SetNavigationGoal => "set_navigation_goal",
            Self::CancelNavigation => "cancel_navigation",
            Self::GetNavigationReadiness => "get_navigation_readiness",
            Self::GetMappingReadiness => "get_mapping_readiness",
            Self::GetExplorationReadiness => "get_exploration_readiness",
            Self::SetCmdVelSinkReady => "set_cmd_vel_sink_ready",
            Self::GetFrontiers => "get_frontiers",
            Self::StartExploration => "start_exploration",
            Self::StopExploration => "stop_exploration",
            Self::GetExplorationState => "get_exploration_state",
            Self::GetExplorationDiagnostics => "get_exploration_diagnostics",
            Self::PublishLaserScan => "publish_laser_scan",
            Self::PublishOdom => "publish_odom",
            Self::PublishPointCloud => "publish_point_cloud",
        }
    }

    fn rpc_name(self) -> String {
        format!("{RPC_MODULE_NAME}/{}", self.method_name())
    }
}

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

fn encode_response<T: Serialize>(response: T) -> Result<Value, InvocationError> {
    serde_json::to_value(response)
        .map_err(|error| InvocationError::serialization(error.to_string()))
}

struct GeometryRpcHandler {
    service: Arc<GeometryRuntimeService>,
    method: GeometryRpcMethod,
}

impl GeometryRpcHandler {
    async fn invoke(&self, args: RpcArgs) -> Result<Value, InvocationError> {
        macro_rules! call_unary {
            ($request_type:ty, $method:ident) => {{
                let request = decode_request::<$request_type>(args)?;
                let response = GeometryRuntime::$method(&*self.service, Request::new(request))
                    .await
                    .map_err(InvocationError::status)?;
                encode_response(response.into_inner())
            }};
        }

        match self.method {
            GeometryRpcMethod::Start => call_unary!(StartRequest, start),
            GeometryRpcMethod::Stop => call_unary!(StopRequest, stop),
            GeometryRpcMethod::GetStatus => call_unary!(Empty, get_status),
            GeometryRpcMethod::HealthCheck => call_unary!(HealthCheckRequest, health_check),
            GeometryRpcMethod::GetCurrentPose => call_unary!(PoseRequest, get_current_pose),
            GeometryRpcMethod::GetFrameTree => call_unary!(Empty, get_frame_tree),
            GeometryRpcMethod::GetOccupancyGrid => call_unary!(Empty, get_occupancy_grid),
            GeometryRpcMethod::GetOccupancyGridData => {
                call_unary!(Empty, get_occupancy_grid_data)
            }
            GeometryRpcMethod::GetOccupancyGridShm => {
                call_unary!(Empty, get_occupancy_grid_shm)
            }
            GeometryRpcMethod::StartMapping => {
                call_unary!(StartMappingRequest, start_mapping)
            }
            GeometryRpcMethod::StopMapping => {
                call_unary!(StopMappingRequest, stop_mapping)
            }
            GeometryRpcMethod::GetMappingState => {
                call_unary!(Empty, get_mapping_state)
            }
            GeometryRpcMethod::ActivateMap => {
                call_unary!(ActivateMapRequest, activate_map)
            }
            GeometryRpcMethod::SetNavigationGoal => {
                call_unary!(NavigationGoal, set_navigation_goal)
            }
            GeometryRpcMethod::CancelNavigation => {
                call_unary!(CancelNavigationRequest, cancel_navigation)
            }
            GeometryRpcMethod::GetNavigationReadiness => {
                call_unary!(Empty, get_navigation_readiness)
            }
            GeometryRpcMethod::GetMappingReadiness => {
                call_unary!(Empty, get_mapping_readiness)
            }
            GeometryRpcMethod::GetExplorationReadiness => {
                call_unary!(Empty, get_exploration_readiness)
            }
            GeometryRpcMethod::SetCmdVelSinkReady => {
                call_unary!(CmdVelSinkReadyRequest, set_cmd_vel_sink_ready)
            }
            GeometryRpcMethod::GetFrontiers => call_unary!(Empty, get_frontiers),
            GeometryRpcMethod::StartExploration => {
                call_unary!(ExploreRequest, start_exploration)
            }
            GeometryRpcMethod::StopExploration => {
                call_unary!(StopExplorationRequest, stop_exploration)
            }
            GeometryRpcMethod::GetExplorationState => {
                call_unary!(Empty, get_exploration_state)
            }
            GeometryRpcMethod::GetExplorationDiagnostics => {
                call_unary!(Empty, get_exploration_diagnostics)
            }
            GeometryRpcMethod::PublishLaserScan => call_unary!(PublishLaserScanRequest, publish_laser_scan),
            GeometryRpcMethod::PublishOdom => call_unary!(PublishOdomRequest, publish_odom),
            GeometryRpcMethod::PublishPointCloud => {
                call_unary!(PublishPointCloudRequest, publish_point_cloud)
            }
        }
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

/// Start every unary geometry method over the production LCM RPC transport.
pub async fn serve_geometry_rpc(
    service: GeometryRuntimeService,
    transport: Arc<dyn RpcTransport>,
) -> RpcResult<RpcServerGroup> {
    let service = Arc::new(service);
    let broker = RpcBroker::new();
    let mut names = Vec::with_capacity(RPC_METHODS.len());
    for method in RPC_METHODS.iter().copied() {
        let name = method.rpc_name();
        broker
            .register(
                name.clone(),
                Box::new(GeometryRpcHandler {
                    service: Arc::clone(&service),
                    method,
                }),
            )
            .await?;
        names.push(name);
    }
    broker.serve_many(transport, names).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn names_follow_original_module_class_method_contract() {
        let names = RPC_METHODS
            .iter()
            .copied()
            .map(GeometryRpcMethod::rpc_name)
            .collect::<Vec<_>>();
        assert_eq!(names.len(), 26);
        assert!(names.contains(&"GeometryRuntimeService/health_check".to_string()));
        assert!(names.contains(&"GeometryRuntimeService/stop".to_string()));
        assert!(names
            .iter()
            .all(|name| name.starts_with("GeometryRuntimeService/")));
    }

    #[test]
    fn request_decoder_accepts_keyword_fields_and_rejects_mixed_arguments() {
        let request = decode_request::<PoseRequest>((
            Vec::new(),
            HashMap::from([
                ("target_frame".to_string(), Value::from("map")),
                ("source_frame".to_string(), Value::from("base_link")),
            ]),
        ))
        .unwrap();
        assert_eq!(request.target_frame, "map");
        assert_eq!(request.source_frame, "base_link");

        let error = decode_request::<PoseRequest>((
            vec![serde_json::json!({"target_frame": "map"})],
            HashMap::from([("source_frame".to_string(), Value::from("base_link"))]),
        ))
        .unwrap_err();
        assert_eq!(error.type_name, "InvalidArguments");
    }

    #[tokio::test]
    async fn handler_dispatches_to_real_geometry_service_and_serializes_errors() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real LCM test handle");
        let service = Arc::new(GeometryRuntimeService::new(handle));
        let handler = GeometryRpcHandler {
            service,
            method: GeometryRpcMethod::HealthCheck,
        };

        let response = handler
            .handle(RpcRequest {
                name: GeometryRpcMethod::HealthCheck.rpc_name(),
                args: (Vec::new(), HashMap::new()),
                id: Some(7.0),
            })
            .await;
        assert_eq!(response.id, Some(7.0));
        assert_eq!(response.res.as_ref().unwrap()["healthy"], Value::Bool(true));
        assert_eq!(response.res.as_ref().unwrap()["phase"], "running");

        let error = handler
            .handle(RpcRequest {
                name: GeometryRpcMethod::HealthCheck.rpc_name(),
                args: (vec![Value::String("not-a-request".into())], HashMap::new()),
                id: Some(8.0),
            })
            .await;
        assert_eq!(
            error.exception.as_ref().unwrap()["type_name"],
            "InvalidArguments"
        );
    }
}

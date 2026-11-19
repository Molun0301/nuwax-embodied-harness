//! 固定导航任务的真实通信适配。

#[cfg(any(feature = "cmu-native", feature = "explore"))]
use std::collections::{HashMap, HashSet};
use std::future::Future;
use std::pin::Pin;
#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{mpsc as std_mpsc, Arc, Mutex};
use std::time::Duration;
#[cfg(feature = "guard-runtime")]
use std::time::Instant;

use futures::{FutureExt, Stream, StreamExt};
use nuwax_codec::lcm::schema;
#[cfg(feature = "patrolling")]
use nuwax_nav_types_messages::GridSnapshot;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "cmu-native",
    feature = "explore",
    feature = "patrolling"
))]
use nuwax_nav_types_messages::Path2D;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "cmu-native",
    feature = "explore"
))]
use nuwax_nav_types_messages::Point2;
#[cfg(any(
    feature = "simple-2d",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
use nuwax_nav_types_messages::PointCloud;
#[cfg(feature = "simple-2d")]
use nuwax_nav_types_messages::PointStamped;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "patrolling",
    feature = "click-goal",
    feature = "explore"
))]
use nuwax_nav_types_messages::PoseStamped;
#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "patrolling"
))]
use nuwax_nav_types_messages::Twist2;
use nuwax_nav_types_messages::{GoalKey, NavFrameId, Point3, Pose3, Quaternion, Timestamp};
#[cfg(feature = "explore")]
use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
use nuwax_transport::{LcmFrameStream, LcmTransport, TransportResult};
use tokio::sync::oneshot;

#[cfg(feature = "mls")]
#[path = "surface_dataflow.rs"]
mod surface_dataflow;
#[cfg(feature = "mls")]
pub(crate) use surface_dataflow::start_surface_tasks;

use crate::error::{NavStackError, NavStackResult};
use crate::topics::TaskChannels;
use crate::task::{spawn_task, ReadySender, TaskThread};


fn lcm_transport() -> NavStackResult<LcmTransport> {
    let provider = nuwax_config::transport()
        .map_err(NavStackError::InvalidConfig)?
        .lcm_url
        .clone();
    LcmTransport::new_real(&provider)
        .map_err(|error| NavStackError::Internal(format!("real LCM startup failed: {error}")))
}

async fn typed_stream(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
) -> NavStackResult<LcmFrameStream> {
    let topic =
        protocol_topic(channel).map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    let message_type = LcmMessageType::new(type_name)
        .map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    if type_name == "nav_msgs.OccupancyGrid" && planar_navigation_enabled() {
        return planar_navigation_stream(transport,topic).await;
    }
    Ok(transport.subscribe(topic, Some(message_type)).await)
}

fn planar_navigation_enabled() -> bool {
    nuwax_config::geometry()
        .expect("启动已捕获 geometry 配置")
        .pipeline
        == "slam_2d"
}

/// 每个消费者独立核对运行、会话和观测顺序，清空与切域先向执行者交付撤销。
async fn planar_navigation_stream(transport:&LcmTransport,topic:LcmTopic)->NavStackResult<LcmFrameStream> {
    use nuwax_codec::lcm::planar_navigation::{self,Admission};
    let message_type=LcmMessageType::new(planar_navigation::TYPE_NAME)
        .map_err(|e|NavStackError::InvalidConfig(e.to_string()))?;
    let stream=transport.subscribe(topic,Some(message_type)).await;
    let mapped=stream.scan((Admission::default(),std::time::Instant::now()),|(admission,last_diagnostic),item| {
        let mut output=Vec::new();
        match item {
            Err(error)=>output.push(Err(error)),
            Ok(frame)=>{
                match planar_navigation::decode_body(&frame.payload)
                    .map_err(|e|e.to_string())
                    .and_then(|map|admission.admit(&map).map(|changed|(map,changed))) {
                    Ok((map,changed))=>{
                        let convert=|grid:&schema::OccupancyGrid| -> TransportResult<RawLcmFrame> {
                            let mut translated=frame.clone();
                            translated.payload=schema::encode_occupancy_grid_body(grid)?;
                            translated.msg_type=LcmMessageType::new("nav_msgs.OccupancyGrid")
                                .map_err(|e|nuwax_transport::TransportError::MessageType(e.to_string()))?;
                            Ok(translated)
                        };
                        if changed || map.cleared {
                            let cleared=schema::OccupancyGrid {header:map.grid.header.clone(),..Default::default()};
                            output.push(convert(&cleared));
                        }
                        if !map.cleared {output.push(convert(&map.grid));}
                    },
                    Err(error)=>{
                        if last_diagnostic.elapsed()>=Duration::from_secs(1) {
                            eprintln!("二维导航观测拒收：{error}");
                            *last_diagnostic=std::time::Instant::now();
                        }
                    },
                }
            },
        }
        futures::future::ready(Some(futures::stream::iter(output)))
    }).flatten();
    Ok(Box::pin(mapped))
}

fn build_body_frame(channel: &str, type_name: &str, body: Vec<u8>) -> NavStackResult<RawLcmFrame> {
    let topic =
        protocol_topic(channel).map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    let message_type = LcmMessageType::new(type_name)
        .map_err(|error| NavStackError::InvalidConfig(error.to_string()))?;
    Ok(RawLcmFrameBuilder::new(topic, message_type)
        .payload(body)
        .build())
}

async fn publish_body(
    transport: &LcmTransport,
    channel: &str,
    type_name: &str,
    body: Vec<u8>,
) -> NavStackResult<()> {
    transport
        .publish(build_body_frame(channel, type_name, body)?)
        .await
        .map_err(|error| NavStackError::Internal(format!("publish {channel}: {error}")))
}

fn protocol_topic(channel: &str) -> Result<LcmTopic, nuwax_protocol::LcmTopicParseError> {
    if channel.starts_with('/') {
        LcmTopic::new(channel)
    } else {
        LcmTopic::new(&format!("/{channel}"))
    }
}

fn required_frame(
    item: Option<TransportResult<RawLcmFrame>>,
    component: &str,
) -> NavStackResult<RawLcmFrame> {
    item.ok_or_else(|| NavStackError::Internal(format!("{component} LCM stream closed")))?
        .map_err(|error| NavStackError::Internal(format!("{component} LCM stream: {error}")))
}

#[cfg(feature = "guard-runtime")]
use nuwax_core::logging::DiagnosticLimiter as InputDiagnosticLimiter;
#[cfg(all(test, feature = "guard-runtime"))]
use nuwax_core::logging::INPUT_DIAGNOSTIC_INTERVAL;

#[cfg(feature = "guard-runtime")]
#[derive(Debug, PartialEq)]
enum OwnerInput<T> {
    Accepted(T),
    Rejected(String),
}

#[cfg(feature = "guard-runtime")]
type OwnerInputResult<T> = NavStackResult<OwnerInput<T>>;

#[cfg(feature = "guard-runtime")]
fn runtime_input_error<T>(
    component: &str,
    error: nuwax_nav_runtime::RuntimeError,
) -> OwnerInputResult<T> {
    use nuwax_nav_runtime::RuntimeError;

    let fatal = matches!(
        &error,
        RuntimeError::SchedulerStopped
    );
    if fatal {
        Err(module_error(component, error))
    } else {
        Ok(OwnerInput::Rejected(error.to_string()))
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn module_input_error<T>(component: &str, error: nuwax_nav_base::NavError) -> OwnerInputResult<T> {
    use nuwax_nav_base::NavError;

    let external = matches!(
        &error,
        NavError::InvalidInput { .. }
            | NavError::MapUnavailable
            | NavError::GoalOutOfBounds
            | NavError::GoalOccupied
            | NavError::NoPath { .. }
    );
    if external {
        Ok(OwnerInput::Rejected(error.to_string()))
    } else {
        Err(module_error(component, error))
    }
}

#[cfg(any(feature = "simple-2d", feature = "replanning-astar"))]
fn module_tick_error(component: &str, error: nuwax_nav_base::NavError) -> NavStackError {
    module_error(component, error)
}

fn signal_ready(ready: ReadySender) -> NavStackResult<()> {
    ready
        .send(Ok(()))
        .map_err(|_| NavStackError::Internal("startup handshake receiver dropped".into()))
}




















fn module_error(component: &str, error: impl std::fmt::Display) -> NavStackError {
    NavStackError::Internal(format!("{component}: {error}"))
}

// 按任务拆分的子模块：函数体与拆分前相同，模块内私有项改为 pub(super) 供同级模块使用。
// 模块条件取其中各项原有条件的并集；只被测试用到的项按测试条件汇总。
#[cfg(feature = "guard-runtime")]
mod pose_adapter;
#[cfg(any(feature = "guard-runtime", feature = "click-goal"))]
mod guarded_loop;
#[cfg(feature = "click-goal")]
mod click_goal;
#[cfg(feature = "explore")]
mod wavefront;
#[cfg(any(feature = "cmu-native", feature = "explore"))]
mod terrain_map;
#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
mod boundary;
#[cfg(any(feature = "cmu-native", feature = "explore"))]
mod far_adapter;
#[cfg(any(feature = "simple-2d", feature = "replanning-astar", feature = "basic-path-follower"))]
mod planar_planning;
#[cfg(feature = "patrolling")]
mod patrolling;
mod codec;

#[cfg(any(feature = "guard-runtime", feature = "click-goal"))]
use guarded_loop::*;
#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
use boundary::*;
use codec::*;
#[cfg(all(test, feature = "guard-runtime"))]
use pose_adapter::*;
#[cfg(all(test, feature = "click-goal"))]
use click_goal::*;
#[cfg(all(test, any(feature = "cmu-native", feature = "explore")))]
use far_adapter::*;
#[cfg(all(test, any(feature = "simple-2d", feature = "replanning-astar", feature = "basic-path-follower")))]
use planar_planning::*;

#[cfg(all(feature = "mls", feature = "simple-2d"))]
pub(crate) use guarded_loop::guarded_nav_scheduler_owner;
#[cfg(all(feature = "mls", feature = "simple-2d"))]
pub(crate) use pose_adapter::map_pose_owner;
#[cfg(all(feature = "mls", feature = "simple-2d"))]
pub(crate) use planar_planning::simple_planner_owner;

#[cfg(all(test, any(feature = "simple-2d", feature = "replanning-astar")))]
mod planner_input_tests {
    include!("lcm_dataflow/planner_input_tests.rs");
}

#[cfg(all(test, feature = "simple-2d"))]
mod simple_tests {
    include!("lcm_dataflow/simple_tests.rs");
}

#[cfg(all(test, feature = "guard-runtime"))]
mod guarded_tests {
    include!("lcm_dataflow/guarded_tests.rs");
}

#[cfg(all(test, feature = "replanning-astar"))]
mod tests {
    include!("lcm_dataflow/tests.rs");
}

#[cfg(all(test, feature = "simple-2d", feature = "replanning-astar"))]
mod boundary_tests {
    include!("lcm_dataflow/boundary_tests.rs");
}

#[cfg(all(
    test,
    any(feature = "cmu-native", feature = "explore")
))]
mod native_adapter_tests {
    include!("lcm_dataflow/native_adapter_tests.rs");
}

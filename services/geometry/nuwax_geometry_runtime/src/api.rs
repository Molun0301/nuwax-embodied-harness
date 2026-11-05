//! 冻结协议的远程边界：解码、投递命令或读取快照、编码。

use crate::proto::geometry_runtime_server::GeometryRuntime;
use crate::proto::*;
use crate::session::SessionHandle;
use futures::{Stream, StreamExt};
use std::pin::Pin;
use tokio::sync::watch;
use tonic::{Request, Response, Status};

pub type ResponseStream<T> = Pin<Box<dyn Stream<Item = Result<T, Status>> + Send>>;

#[derive(Clone)]
pub struct GeometryApi {
    session: SessionHandle,
}

impl GeometryApi {
    pub fn new(session: SessionHandle) -> Self {
        Self { session }
    }
    fn until_closed<T: Send + 'static>(&self, stream: ResponseStream<T>) -> ResponseStream<T> {
        let session = self.session.clone();
        Box::pin(stream.take_until(async move { session.closed().await }))
    }
}

/// 每次只读取最新快照；慢读者不会排队保留中间值。
fn latest<T: Clone + Send + Sync + 'static>(rx: watch::Receiver<T>) -> ResponseStream<T> {
    Box::pin(futures::stream::unfold(
        (rx, true),
        |(mut rx, first)| async move {
            if !first && rx.changed().await.is_err() {
                return None;
            }
            let value = rx.borrow_and_update().clone();
            Some((Ok(value), (rx, false)))
        },
    ))
}

fn present<T: Clone + Send + Sync + 'static>(rx: watch::Receiver<Option<T>>) -> ResponseStream<T> {
    Box::pin(latest(rx).filter_map(|item| async {
        match item {
            Ok(Some(value)) => Some(Ok(value)),
            Ok(None) => None,
            Err(error) => Some(Err(error)),
        }
    }))
}

#[tonic::async_trait]
impl GeometryRuntime for GeometryApi {
    async fn start_mapping(
        &self,
        request: Request<StartMappingRequest>,
    ) -> Result<Response<GeometryState>, Status> {
        let request = request.into_inner();
        map_name(&request.map_name)?;
        Ok(Response::new(self.session.start_mapping(request).await?))
    }
    async fn stop_session(&self, _: Request<Empty>) -> Result<Response<GeometryState>, Status> {
        Ok(Response::new(self.session.stop_session().await?))
    }
    async fn save_map(
        &self,
        request: Request<SaveMapRequest>,
    ) -> Result<Response<SaveMapResponse>, Status> {
        let request = request.into_inner();
        if !request.map_name.is_empty() {
            map_name(&request.map_name)?;
        }
        Ok(Response::new(self.session.save_map(request).await?))
    }
    async fn load_map(
        &self,
        request: Request<LoadMapRequest>,
    ) -> Result<Response<GeometryState>, Status> {
        let request = request.into_inner();
        map_name(&request.map_name)?;
        Ok(Response::new(self.session.load_map(request).await?))
    }
    async fn relocalize(
        &self,
        request: Request<RelocalizeRequest>,
    ) -> Result<Response<RelocalizationResult>, Status> {
        let request = request.into_inner();
        if request.seed.is_some()
            && self.session.state().info.as_ref().unwrap().pipeline != "slam_2d"
        {
            return Err(Status::invalid_argument("三维重定位不接受初值"));
        }
        if let Some(seed) = &request.seed {
            if ![seed.x, seed.y, seed.yaw].iter().all(|v| v.is_finite()) {
                return Err(Status::invalid_argument("重定位初值必须为有限数值"));
            }
        }
        Ok(Response::new(self.session.relocalize(request).await?))
    }
    async fn set_goal(
        &self,
        request: Request<NavigationGoal>,
    ) -> Result<Response<SetGoalResponse>, Status> {
        let mut request = request.into_inner();
        let pose = request
            .pose
            .as_ref()
            .ok_or_else(|| Status::invalid_argument("缺少导航目标位姿"))?;
        validate_pose(pose)?;
        let map_frame = self.session.state().info.unwrap().map_frame_id;
        if !pose.frame_id.is_empty() && pose.frame_id != map_frame {
            return Err(Status::invalid_argument("导航目标必须使用地图坐标系"));
        }
        if request
            .position_tolerance_m
            .is_some_and(|v| !v.is_finite() || v <= 0.0)
            || request
                .yaw_tolerance_rad
                .is_some_and(|v| !v.is_finite() || v <= 0.0)
        {
            return Err(Status::invalid_argument("导航容差必须为有限正数"));
        }
        if request.pose.as_ref().unwrap().frame_id.is_empty() {
            request.pose.as_mut().unwrap().frame_id = map_frame;
        }
        Ok(Response::new(self.session.set_goal(request).await?))
    }
    async fn cancel_goal(
        &self,
        request: Request<CancelGoalRequest>,
    ) -> Result<Response<Empty>, Status> {
        self.session.cancel_goal(request.into_inner()).await?;
        Ok(Response::new(Empty {}))
    }
    async fn start_exploration(
        &self,
        request: Request<ExplorationRequest>,
    ) -> Result<Response<StartExplorationResponse>, Status> {
        Ok(Response::new(
            self.session.start_exploration(request.into_inner()).await?,
        ))
    }
    async fn stop_exploration(
        &self,
        request: Request<StopExplorationRequest>,
    ) -> Result<Response<Empty>, Status> {
        self.session.stop_exploration(request.into_inner()).await?;
        Ok(Response::new(Empty {}))
    }
    async fn get_exploration_progress(
        &self,
        request: Request<ExplorationProgressRequest>,
    ) -> Result<Response<ExplorationProgress>, Status> {
        Ok(Response::new(
            self.session
                .exploration_progress(request.into_inner())
                .await?,
        ))
    }
    async fn get_state(&self, _: Request<Empty>) -> Result<Response<GeometryState>, Status> {
        Ok(Response::new(self.session.state()))
    }
    type SubscribeStateStream = ResponseStream<GeometryState>;
    async fn subscribe_state(
        &self,
        _: Request<Empty>,
    ) -> Result<Response<Self::SubscribeStateStream>, Status> {
        let session = self.session.clone();
        let stream = Box::pin(latest(self.session.state_rx()).map(move |state| {
            state.map(|mut state| {
                state.pose = session.pose_rx().borrow().clone();
                state
            })
        }));
        Ok(Response::new(self.until_closed(stream)))
    }
    type SubscribePoseStream = ResponseStream<Pose>;
    async fn subscribe_pose(
        &self,
        _: Request<Empty>,
    ) -> Result<Response<Self::SubscribePoseStream>, Status> {
        Ok(Response::new(
            self.until_closed(present(self.session.pose_rx())),
        ))
    }
    type SubscribeMapStream = ResponseStream<MapUpdate>;
    async fn subscribe_map(
        &self,
        _: Request<Empty>,
    ) -> Result<Response<Self::SubscribeMapStream>, Status> {
        Ok(Response::new(
            self.until_closed(present(self.session.map_rx())),
        ))
    }
    async fn get_occupancy_grid(
        &self,
        request: Request<GetOccupancyGridRequest>,
    ) -> Result<Response<OccupancyGrid>, Status> {
        Ok(Response::new(
            self.session
                .occupancy(request.into_inner().include_forbidden)?,
        ))
    }
    async fn get_dense_cloud_chunk(
        &self,
        request: Request<DenseCloudChunkRequest>,
    ) -> Result<Response<DenseCloudChunk>, Status> {
        Ok(Response::new(
            self.session.dense_chunk(request.into_inner()).await?,
        ))
    }
    async fn get_frame_tree(&self, _: Request<Empty>) -> Result<Response<FrameTree>, Status> {
        Ok(Response::new(self.session.frame_tree()))
    }
    async fn publish_point_cloud(
        &self,
        request: Request<PublishPointCloudRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        self.session
            .lio()?
            .publish_point_cloud(request.into_inner())
            .await?;
        Ok(Response::new(PublishResult {
            success: true,
            message: String::new(),
        }))
    }
    async fn publish_odom(
        &self,
        request: Request<PublishOdomRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        self.session
            .lio()?
            .publish_odom(request.into_inner())
            .await?;
        Ok(Response::new(PublishResult {
            success: true,
            message: String::new(),
        }))
    }
    async fn publish_laser_scan(
        &self,
        request: Request<PublishLaserScanRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        self.session
            .lio()?
            .publish_laser_scan(request.into_inner())
            .await?;
        Ok(Response::new(PublishResult {
            success: true,
            message: String::new(),
        }))
    }
}

pub(crate) fn map_name(name: &str) -> Result<(), Status> {
    if name.is_empty()
        || name.chars().count() > 64
        || name.starts_with('.')
        || name.chars().any(|c| c == '/' || c.is_control())
    {
        return Err(Status::invalid_argument(
            "地图名须为1至64个字符，不能以点开头或包含斜线、控制字符",
        ));
    }
    Ok(())
}

fn validate_pose(pose: &Pose) -> Result<(), Status> {
    let p = pose
        .position
        .as_ref()
        .ok_or_else(|| Status::invalid_argument("缺少位置"))?;
    let q = pose
        .orientation
        .as_ref()
        .ok_or_else(|| Status::invalid_argument("缺少姿态"))?;
    if ![p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        .iter()
        .all(|v| v.is_finite())
        || q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w == 0.0
    {
        return Err(Status::invalid_argument("位姿含无效数值或零四元数"));
    }
    Ok(())
}

#[cfg(test)]
#[path = "api_behavior_tests.rs"]
mod behavior_tests;

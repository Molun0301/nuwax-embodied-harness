//! 新守卫协议的有序 Unix 套接字客户端；导航调度器独占连接与租约。

use crate::constants::{motion_guard_socket_path, MAX_FRAME_BYTES};
use crate::wire::{
    guard_request, guard_response, AcquireRequest, ErrorCode, GuardRequest, GuardResponse,
    ReleaseRequest, Source, StatusRequest, StatusResponse, VelocityRequest, VelocityResponse,
};
use prost::Message;
use std::path::{Path, PathBuf};
use thiserror::Error;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UnixStream;

/// 守卫连接位置。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClientConfig {
    /// 私有套接字路径。
    pub socket_path: PathBuf,
}
impl Default for ClientConfig {
    fn default() -> Self {
        Self {
            socket_path: motion_guard_socket_path(),
        }
    }
}

/// 传输、解码或守卫拒绝原因。
#[derive(Debug, Error)]
pub enum MotionGuardClientError {
    /// 套接字读写失败。
    #[error("运动守卫传输失败：{0}")]
    Transport(String),
    /// 帧或应答类型不符。
    #[error("运动守卫协议错误：{0}")]
    Protocol(String),
    /// 守卫返回的协议错误。
    #[error("运动守卫拒绝：{code:?}：{message}")]
    Remote {
        /// 原协议错误码。
        code: ErrorCode,
        /// 原协议原因。
        message: String,
    },
}

/// 单连接上一问一答；租约由同一个导航调度器拥有。
pub struct MotionGuardClient {
    socket_path: PathBuf,
    stream: Option<UnixStream>,
    lease_id: Option<u64>,
    measured_motion: crate::measured_motion::MeasurementCache,
}
impl MotionGuardClient {
    /// 连接指定守卫。
    pub async fn connect(path: impl AsRef<Path>) -> Result<Self, MotionGuardClientError> {
        let socket_path = path.as_ref().to_path_buf();
        let stream = UnixStream::connect(&socket_path).await.map_err(transport)?;
        Ok(Self {
            socket_path,
            stream: Some(stream),
            lease_id: None,
            measured_motion: Default::default(),
        })
    }
    /// 按配置连接守卫。
    pub async fn connect_with_config(config: ClientConfig) -> Result<Self, MotionGuardClientError> {
        Self::connect(config.socket_path).await
    }
    /// 当前客户端持有的租约。
    pub fn active_lease(&self) -> Option<u64> {
        self.lease_id
    }
    /// 当前来源期限内的真实运动反馈。
    pub fn measured_motion(&self) -> Option<crate::MeasuredMotion> {
        self.measured_motion.current()
    }
    /// 新目标用受理时的运动代次接管租约；不先发送零速。
    pub async fn acquire(
        &mut self,
        source: Source,
        generation: u64,
    ) -> Result<u64, MotionGuardClientError> {
        if self.stream.is_none() {
            self.lease_id = None;
            self.measured_motion = Default::default();
            self.stream = Some(UnixStream::connect(&self.socket_path).await.map_err(transport)?);
        }
        match self
            .exchange(guard_request::Op::Acquire(AcquireRequest {
                source: source as i32,
                generation,
            }))
            .await?
        {
            guard_response::Result::Acquire(response) => {
                self.lease_id = Some(response.lease_id);
                Ok(response.lease_id)
            }
            _ => Err(protocol("应答不是 Acquire")),
        }
    }
    /// 发送机体速度并续租。
    pub async fn velocity(
        &mut self,
        vx_mps: f64,
        vy_mps: f64,
        wz_rps: f64,
    ) -> Result<VelocityResponse, MotionGuardClientError> {
        let lease_id = self.lease_id.ok_or_else(|| protocol("没有当前租约"))?;
        match self
            .exchange(guard_request::Op::Velocity(VelocityRequest {
                lease_id,
                vx_mps,
                vy_mps,
                wz_rps,
            }))
            .await?
        {
            guard_response::Result::Velocity(response) => Ok(response),
            _ => Err(protocol("应答不是 Velocity")),
        }
    }
    /// 目标结束时释放；已作废的租约无需再次停车。
    pub async fn release(&mut self) -> Result<(), MotionGuardClientError> {
        let Some(lease_id) = self.lease_id.take() else {
            return Ok(());
        };
        match self
            .exchange(guard_request::Op::Release(ReleaseRequest { lease_id }))
            .await
        {
            Ok(guard_response::Result::Release(_)) => Ok(()),
            Err(MotionGuardClientError::Remote {
                code: ErrorCode::LeaseRevoked,
                ..
            }) => Ok(()),
            Ok(_) => Err(protocol("应答不是 Release")),
            Err(error) => Err(error),
        }
    }
    /// 查询状态与实际设备限幅。
    pub async fn status(&mut self) -> Result<StatusResponse, MotionGuardClientError> {
        match self
            .exchange(guard_request::Op::Status(StatusRequest {}))
            .await?
        {
            guard_response::Result::Status(response) => Ok(response),
            _ => Err(protocol("应答不是 Status")),
        }
    }
    async fn exchange(
        &mut self,
        op: guard_request::Op,
    ) -> Result<guard_response::Result, MotionGuardClientError> {
        // 问答中途失败或被调用方取消时关闭连接，避免下个目标读到半条旧应答。
        let mut stream = self
            .stream
            .take()
            .ok_or_else(|| MotionGuardClientError::Transport("连接已关闭".into()))?;
        let body = GuardRequest { op: Some(op) }.encode_to_vec();
        stream
            .write_all(&(body.len() as u32).to_be_bytes())
            .await
            .map_err(transport)?;
        stream.write_all(&body).await.map_err(transport)?;
        let size = stream.read_u32().await.map_err(transport)? as usize;
        if size == 0 || size > MAX_FRAME_BYTES {
            return Err(protocol("帧长度超出协议范围"));
        }
        let mut body = vec![0; size];
        stream.read_exact(&mut body).await.map_err(transport)?;
        let response =
            GuardResponse::decode(body.as_slice()).map_err(|error| protocol(error.to_string()))?;
        self.stream = Some(stream);
        self.measured_motion.update(
            response.measured_motion,
            crate::measured_motion::monotonic_ns(),
        );
        if let Some(error) = response.error {
            let code = ErrorCode::try_from(error.code).map_err(|_| protocol("未知错误码"))?;
            if code == ErrorCode::LeaseRevoked {
                self.lease_id = None;
            }
            return Err(MotionGuardClientError::Remote {
                code,
                message: error.message,
            });
        }
        response.result.ok_or_else(|| protocol("应答缺少结果"))
    }
}
fn transport(error: std::io::Error) -> MotionGuardClientError {
    MotionGuardClientError::Transport(error.to_string())
}
fn protocol(message: impl Into<String>) -> MotionGuardClientError {
    MotionGuardClientError::Protocol(message.into())
}

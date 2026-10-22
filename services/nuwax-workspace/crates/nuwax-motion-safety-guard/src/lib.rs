//! 运动守卫私有协议、导航客户端与实测运动缓存。
//!
//! 生产套接字和厂商执行器由Python守卫进程唯一拥有。本包通过同一协议
//! 为Rust导航与几何调用方提供客户端，不实现另一套守卫状态机或持久账本。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod client;
pub mod constants;
mod crypto;
pub mod measured_motion;
pub mod wire;

pub use client::{
    ClientConfig, CommandConfirmation, GlobalStopConfirmation, HeldRecoveryFenceProof,
    LeaseMetadata, MotionAdmissionProof, MotionGuardClient, MotionGuardClientError,
    PriorityFenceRequest, ReadinessStatus, RenewalMetadata, ZeroConfirmation,
};
pub use constants::{
    COMMAND_DEADLINE_MS, DEFAULT_REQUEST_TIMEOUT_MS, DEFAULT_UDS_PATH, MAX_FRAME_BYTES,
    RENEW_INTERVAL_MS,
};
pub use measured_motion::MeasuredMotion;

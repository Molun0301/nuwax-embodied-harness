//! 运动守卫私有协议、导航客户端与实测运动缓存。
//!
//! 生产套接字和厂商执行器由Python守卫进程唯一拥有。本包通过同一协议
//! 为Rust导航与几何调用方提供客户端，不实现另一套守卫状态机或持久账本。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod client;
pub mod command_limits;
pub mod constants;
pub mod measured_motion;
pub mod wire;

pub use client::{ClientConfig, MotionGuardClient, MotionGuardClientError};
pub use constants::{motion_guard_socket_path, MAX_FRAME_BYTES};
pub use measured_motion::MeasuredMotion;

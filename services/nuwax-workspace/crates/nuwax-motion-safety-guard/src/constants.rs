//! 守卫协议帧与连接位置。
/// 协议规定的最大帧体长度。
pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;
/// 默认私有套接字路径。
pub const DEFAULT_UDS_PATH: &str = "/run/nuwax/motion-guard.sock";

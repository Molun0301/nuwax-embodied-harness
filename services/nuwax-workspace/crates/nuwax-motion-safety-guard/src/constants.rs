//! 守卫协议帧与连接位置。
/// 协议规定的最大帧体长度。
pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;
const DEFAULT_UDS_RELATIVE_PATH: &str =
    include_str!("../../../../navigation/proto/motion_guard_socket_relative.txt");

/// 读取显式守卫地址，缺省时使用安装根下的套接字。
pub fn motion_guard_socket_path() -> std::path::PathBuf {
    nuwax_config::resolve_path(DEFAULT_UDS_RELATIVE_PATH.trim())
}

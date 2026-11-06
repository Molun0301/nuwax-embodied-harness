//! 守卫协议帧与连接位置。
/// 协议规定的最大帧体长度。
pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;
const DEFAULT_UDS_RELATIVE_PATH: &str =
    include_str!("../../../../navigation/proto/motion_guard_socket_relative.txt");

/// 读取显式守卫地址，缺省时使用安装根下的套接字。
pub fn motion_guard_socket_path() -> std::path::PathBuf {
    use std::{env, path::PathBuf};

    env::var_os("NUWAX_MOTION_GUARD_SOCKET")
        .filter(|path| !path.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            env::var_os("NUWAX_INSTALL_HOME")
                .map_or_else(|| PathBuf::from("."), PathBuf::from)
                .join(DEFAULT_UDS_RELATIVE_PATH.trim())
        })
}

//! 运动守卫客户端使用的请求时限、帧大小和默认连接参数。

/// 客户端续租调度的最长期限；实际时刻还会扣除服务端剩余时间与通信余量。
pub const RENEW_INTERVAL_MS: u64 = 250;

/// 单个运动请求的服务端截止上限。
pub const COMMAND_DEADLINE_MS: u32 = 300;

/// 私有Unix套接字中Protobuf（协议结构编码）帧体的大小上限，不含长度前缀。
pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;

/// 普通关联请求的默认传输等待时间。
pub const DEFAULT_REQUEST_TIMEOUT_MS: u64 = 1_000;

/// 生产守卫的默认私有Unix套接字路径。
pub const DEFAULT_UDS_PATH: &str = "/run/nuwax/motion-guard.sock";

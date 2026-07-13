//! nuwax-motion-safety-guard 时序常量 + UDS 路径常量(§P0B #10/#14/#23)
//!
//! 对齐 W32 Python 端 `tools/motion_safety_guard/w29_uds_constants.py` +
//! `tools/motion_safety_guard/w6_skeleton.MotionGuard.*`。
//!
//! FROZEN — 任何字段变更都会破坏上下游一致性,需要走 breaking-change CI。

/// §P0B #23 lease 绝对时长:600 秒。
///
/// 对齐 Python `LEASE_ABSOLUTE_MS = 600_000`。
pub const LEASE_ABSOLUTE_MS: u64 = 600_000;

/// §P0B #23 NAV owner 每 250ms Renew 一次。
///
/// 对齐 Python `RENEW_INTERVAL_MS = 250`。
pub const RENEW_INTERVAL_MS: u64 = 250;

/// §P0B #23 guard owner_timeout 阈值:3 倍 renew interval 冗余。
///
/// 对齐 Python `OWNER_TIMEOUT_MS = 750`。
pub const OWNER_TIMEOUT_MS: u64 = 750;

/// §P0B #23 MotionCommand deadline 上限 300ms。
///
/// 对齐 Python `COMMAND_DEADLINE_MS = 300`。
pub const COMMAND_DEADLINE_MS: u32 = 300;

/// Private UDS protobuf frame upper bound (16 MiB).
///
/// This matches Python `MAX_FRAME_BYTES` and includes only the protobuf body,
/// excluding the four-byte big-endian prefix.
pub const MAX_FRAME_BYTES: usize = 16 * 1024 * 1024;

/// Default timeout for one correlated UDS request/response exchange.
pub const DEFAULT_REQUEST_TIMEOUT_MS: u64 = 1_000;

/// owner_timeout / renew_interval 推导(3 倍冗余)。
///
/// 对齐 Python `RENEW_TO_OWNER_TIMEOUT_RATIO = 3`。
pub const RENEW_TO_OWNER_TIMEOUT_RATIO: u64 = 3;

/// §P0B #14 / #10 UDS 默认路径。
///
/// 对齐 Python `UDS_PATH = "/run/nuwax/motion-guard.sock"`。
pub const DEFAULT_UDS_PATH: &str = "/run/nuwax/motion-guard.sock";

/// §P0B #15 / #15 epoch.json 默认路径。
///
/// 对齐 Python `EPOCH_JSON_PATH = "/var/lib/nuwax/motion-guard/epoch.json"`。
pub const DEFAULT_EPOCH_PATH: &str = "/var/lib/nuwax/motion-guard/epoch.json";

#[cfg(test)]
mod tests {
    use super::*;

    /// §P0B #23 时序常量 FROZEN 锁定。
    #[test]
    fn timing_constants_frozen() {
        assert_eq!(LEASE_ABSOLUTE_MS, 600_000);
        assert_eq!(RENEW_INTERVAL_MS, 250);
        assert_eq!(OWNER_TIMEOUT_MS, 750);
        assert_eq!(COMMAND_DEADLINE_MS, 300);
        assert_eq!(MAX_FRAME_BYTES, 16 * 1024 * 1024);
        assert_eq!(DEFAULT_REQUEST_TIMEOUT_MS, 1_000);
        assert_eq!(
            RENEW_TO_OWNER_TIMEOUT_RATIO,
            OWNER_TIMEOUT_MS / RENEW_INTERVAL_MS
        );
    }

    /// §P0B #10/#15 默认路径 FROZEN 锁定。
    #[test]
    fn default_paths_frozen() {
        assert_eq!(DEFAULT_UDS_PATH, "/run/nuwax/motion-guard.sock");
        assert_eq!(DEFAULT_EPOCH_PATH, "/var/lib/nuwax/motion-guard/epoch.json");
    }
}

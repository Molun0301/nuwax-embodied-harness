// Phase 6 (codex-round3 R3-4)：TYPED_CHANNELS / resolve_all / strip_env_prefix
// 是 platform API 预留入口（Phase 3 mapping / Phase 5 explore 调用面未全部接通）。
// 在不破坏 contract 前提下用 file-level allow 标注；P2 收口期 Phase 6 后期再清理。
#![allow(dead_code)]

//! LCM typed channel registry —— 统一解析 env。
//!
//! 该模块把 `lcm_runtime.rs` 与 `main.rs` 中分散的 `std::env::var(...)` 读
//! 取收敛到单一入口,并把 `NUWAX_LCM_CHANNEL_*` (Python contracts 平台级)
//! 与 `NUWAX_GEOMETRY_RUNTIME_*_CHANNEL` (本进程内部 alias) 按优先级统一:
//!
//! 1. `NUWAX_GEOMETRY_RUNTIME_<NAME>_CHANNEL` —— 本进程内直读(Phase 1 channel
//!    registry 补齐的 4 类 typed channel:pose/odom/cmd_vel/nav_cancel)
//! 2. `NUWAX_LCM_CHANNEL_<NAME>` —— 平台级 Python contract 默认
//! 3. 内置 default(本文件固化,仅在 CI/contract test 改)
//!
//! ## 命名约定
//! - channel 字符串必须是 LCM typed format:`/topic#pkg.Msg`(`LcmTopic::new`
//!   校验该后缀)
//! - 解析失败的 env 值会被丢弃并打 warn,回退到下一档
//! - 解析失败时返回 `None`,调用方按需决定是否 fail-fast
//!
//! ## 不读取
//! - `NUWAX_LCM_BACKEND` / `NUWAX_LCM_URL` —— 这两个是 backend 选择,不属于
//!   channel registry
//! - 任何 `NUWAX_GEOMETRY_RUNTIME_*` 中非 `_CHANNEL` 后缀的 env(端口/ready
//!   文件/二进制路径)

use tracing::warn;

/// 标准 typed channel 集合(Phase 1 + Phase 2 收敛版)。
///
/// `(&str, &str, &str)` = (env_alias_runtime, env_alias_platform, default)
/// 三元组按顺序探测。
pub const TYPED_CHANNELS: &[(&str, &str, &str)] = &[
    // occupancy grid —— Phase 0
    (
        "NUWAX_GEOMETRY_RUNTIME_OCCUPANCY_CHANNEL",
        "NUWAX_LCM_CHANNEL_OCCUPANCY",
        "/map/occupancy#nav_msgs.OccupancyGrid",
    ),
    // tf —— Phase 0
    (
        "NUWAX_GEOMETRY_RUNTIME_TF_CHANNEL",
        "NUWAX_LCM_CHANNEL_TF",
        "/tf#geometry_msgs.Transform",
    ),
    // cloud map (cumulative point cloud)
    (
        "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL",
        "NUWAX_LCM_CHANNEL_CLOUD_MAP",
        "/go2/lidar/points#sensor_msgs.PointCloud2",
    ),
    // navigation state
    (
        "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL",
        "NUWAX_LCM_CHANNEL_GOAL_POSE",
        "/nav/state#nuwax_msgs.NavigationState",
    ),
    // exploration frontiers
    (
        "NUWAX_GEOMETRY_RUNTIME_FRONTIERS_CHANNEL",
        "NUWAX_LCM_CHANNEL_FRONTIER",
        "/explore/frontiers#nuwax_msgs.FrontierList",
    ),
    // pose stamped (Phase 1)
    (
        "NUWAX_GEOMETRY_RUNTIME_POSE_CHANNEL",
        "NUWAX_LCM_CHANNEL_POSE_STAMPED",
        "/pose_stamped#geometry_msgs.PoseStamped",
    ),
    // odometry (Phase 1)
    (
        "NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL",
        "NUWAX_LCM_CHANNEL_ODOM",
        "/odom#nav_msgs.Odometry",
    ),
    // cmd vel (Phase 1)
    (
        "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_CMD_VEL",
        "/cmd_vel#geometry_msgs.Twist",
    ),
    // nav cancel (Phase 1)
    (
        "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_NAV_CANCEL",
        "/nav/cancel#nuwax_msgs.CancelNavigation",
    ),
];

/// 探测 `runtime-alias → platform-alias → default`,返回第一个非空字符串。
///
/// 不校验 channel 是否为合法 LCM typed format;校验由调用方在拿到
/// `LcmTopic::new` 时完成,以保留 warn 信息(fallback 路径不会因为
/// `LcmTopic::new` 报错而 panic)。
pub fn resolve_channel(runtime_alias: &str, platform_alias: &str, default: &str) -> String {
    if let Ok(value) = std::env::var(runtime_alias) {
        let trimmed = value.trim();
        if !trimmed.is_empty() {
            return trimmed.to_string();
        }
        warn!(
            "{} 设置为空字符串；回退到 {} / default",
            runtime_alias, platform_alias
        );
    }
    if let Ok(value) = std::env::var(platform_alias) {
        let trimmed = value.trim();
        if !trimmed.is_empty() {
            return trimmed.to_string();
        }
        warn!(
            "{} 设置为空字符串；回退到 default",
            platform_alias
        );
    }
    default.to_string()
}

/// 解析所有 typed channel,返回 `(name, channel_string)` 列表。
///
/// 调用方负责把 channel 字符串喂给 `LcmTopic::new`;本函数只做 env 探测
/// + 默认值兜底。
pub fn resolve_all() -> Vec<(&'static str, String)> {
    TYPED_CHANNELS
        .iter()
        .map(|(runtime, platform, default)| {
            // TYPED_CHANNELS 的 runtime alias 已经包含规范名(用作输出标识)
            let canonical = strip_env_prefix(runtime);
            let value = resolve_channel(runtime, platform, default);
            (canonical, value)
        })
        .collect()
}

/// 把 `NUWAX_GEOMETRY_RUNTIME_FOO_CHANNEL` 简化成 `FOO`(用于日志/registry
/// 标识)。不是通用 env 工具函数,只服务本模块 TYPED_CHANNELS。
///
/// 不使用 `unsafe transmute`:运行时 alias env key 全是字面量,但要用
/// `Box::leak` 重建字符串;考虑到本函数只在 `resolve_all` 被调用一次,
/// 实际只对 9 类 typed channel 调用 9 次,可接受微小分配。
fn strip_env_prefix(name: &str) -> &'static str {
    let stripped = name
        .strip_prefix("NUWAX_GEOMETRY_RUNTIME_")
        .unwrap_or(name)
        .strip_suffix("_CHANNEL")
        .unwrap_or(name);
    Box::leak(stripped.to_string().into_boxed_str())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_channel_uses_default_when_no_env() {
        // 不污染环境:用不可能设置的 env 名
        let got = resolve_channel(
            "TEST_NUWAX_RUNTIME_NOT_SET",
            "TEST_NUWAX_PLATFORM_NOT_SET",
            "/foo#bar.Baz",
        );
        assert_eq!(got, "/foo#bar.Baz");
    }

    #[test]
    fn resolve_channel_runtime_wins_over_platform() {
        // 用临时 env 验证优先级
        std::env::set_var("TEST_RUNTIME_X", "/rt#x.Rt");
        std::env::set_var("TEST_PLATFORM_X", "/pf#x.Pf");
        let got = resolve_channel(
            "TEST_RUNTIME_X",
            "TEST_PLATFORM_X",
            "/df#x.Df",
        );
        assert_eq!(got, "/rt#x.Rt");
        std::env::remove_var("TEST_RUNTIME_X");
        std::env::remove_var("TEST_PLATFORM_X");
    }

    #[test]
    fn resolve_channel_platform_used_when_runtime_empty() {
        std::env::remove_var("TEST_RUNTIME_Y");
        std::env::set_var("TEST_PLATFORM_Y", "/pf#y.Pf");
        let got = resolve_channel(
            "TEST_RUNTIME_Y",
            "TEST_PLATFORM_Y",
            "/df#y.Df",
        );
        assert_eq!(got, "/pf#y.Pf");
        std::env::remove_var("TEST_PLATFORM_Y");
    }
}

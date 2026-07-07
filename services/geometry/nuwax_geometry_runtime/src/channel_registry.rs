// Phase 6 (codex-round3 R3-4)：TYPED_CHANNELS / resolve_all / strip_env_prefix
// 是 platform API 预留入口（Phase 3 mapping / Phase 5 explore 调用面未全部接通）。
// 在不破坏 contract 前提下用 file-level allow 标注；P2 收口期 Phase 6 后期再清理。
#![allow(dead_code)]

//! LCM typed channel registry —— 统一解析 env + 公共 channel_for(key) helper。
//!
//! 该模块把 `lcm_runtime.rs` / `main.rs` / `server.rs` / `nav_runtime.rs`
//! 中分散的 channel 字面量收敛到单一入口,并把 `NUWAX_LCM_CHANNEL_*`
//! (Python contracts 平台级) 与 `NUWAX_GEOMETRY_RUNTIME_*_CHANNEL`
//! (本进程内部 alias) 按优先级统一:
//!
//! 1. `NUWAX_GEOMETRY_RUNTIME_<NAME>_CHANNEL` —— 本进程内直读(Phase 1 channel
//!    registry 补齐的 4 类 typed channel:pose/odom/cmd_vel/nav_cancel)
//! 2. `NUWAX_LCM_CHANNEL_<NAME>` —— 平台级 Python contract 默认
//! 3. 内置 default(本文件固化,仅在 CI/contract test 改)
//!
//! ## 命名约定
//! - channel 字符串必须是 LCM typed format:`/topic#pkg.Msg`
//! - `channel_for(key)` 在 key 对应 default 缺少 `#msg_type` 时自动补 typed
//!   suffix;`parse_subscription_pattern` 校验后返回 `LcmTopic`。
//!
//! ## 不读取
//! - `NUWAX_LCM_BACKEND` / `NUWAX_LCM_URL` —— 这两个是 backend 选择,不属于
//!   channel registry
//! - 任何 `NUWAX_GEOMETRY_RUNTIME_*` 中非 `_CHANNEL` 后缀的 env(端口/ready
//!   文件/二进制路径)

use tracing::warn;

// ============================================================================
// Phase 4：channel_for(key) 公共 helper + ChannelKey 枚举
// ============================================================================
//
// 取代 lcm_runtime.rs / server.rs 中所有散落的 `LcmTopic::new("/...")`
// 硬编码 + untyped `topic_or_default(...)` 兜底。
//
// 设计要点：
// 1. 每个 `ChannelKey` 必须映射到 *唯一* 一组 (env_runtime, env_platform, default)
//    与一组 (channel_name, msg_type_suffix)。`SUFFIX_DEFAULTS` 在 default 缺
//    typed suffix 时自动补 msg_type，确保 100% typed。
// 2. env 探测顺序：NUWAX_GEOMETRY_RUNTIME_<X>_CHANNEL > NUWAX_LCM_CHANNEL_<X> >
//    内置 default。
// 3. 向后兼容：旧硬编码 `/odom` 等若通过 env 传入（未带 #msg_type）会在
//    stderr 打一行 deprecation-warning，但不阻断启动；后续会移除。
// 4. real-lcm / fake-lcm backend 共用此 helper，与 LCM URL/backend 选择解耦。

/// Phase 4：runtime 与 server.rs 共用的 channel 键枚举。
///
/// 每变体对应 `(env_runtime, env_platform, default_channel, default_msg_type)`
/// 四元组（部分 channel 没有 env_runtime alias，回退到 env_platform + default）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ChannelKey {
    Odom,
    Tf,
    TfStatic,
    PointCloud,
    Occupancy,
    CmdVel,
    InitialPose,
    GoalPose,
    NavState,
    Frontiers,
    Cancel,
    OccTopic,
}

/// `(env_runtime_key, env_platform_key, default_channel_without_suffix, default_msg_type)`
///
/// `default_msg_type` 在 default_channel 或 env 值缺 `#msg_type` 时自动补。
const CHANNEL_KEY_TABLE: &[(ChannelKey, &str, &str, &str, &str)] = &[
    // env_runtime                          env_platform                  default                              msg_type
    (ChannelKey::Odom,         "NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL", "NUWAX_LCM_CHANNEL_ODOM",          "/odom",                  "nav_msgs.Odometry"),
    (ChannelKey::Tf,           "NUWAX_GEOMETRY_RUNTIME_TF_CHANNEL",       "NUWAX_LCM_CHANNEL_TF",            "/tf",                    "geometry_msgs.Transform"),
    (ChannelKey::TfStatic,     "",                                       "NUWAX_LCM_CHANNEL_TF_STATIC",     "/tf_static",             "geometry_msgs.Transform"),
    (ChannelKey::PointCloud,   "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL", "NUWAX_LCM_CHANNEL_POINTCLOUD",    "/go2/lidar/points",      "sensor_msgs.PointCloud2"),
    (ChannelKey::Occupancy,    "NUWAX_GEOMETRY_RUNTIME_OCCUPANCY_CHANNEL","NUWAX_LCM_CHANNEL_OCCUPANCY",     "/map/occupancy",         "nav_msgs.OccupancyGrid"),
    (ChannelKey::CmdVel,       "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",  "NUWAX_LCM_CHANNEL_CMD_VEL",       "/cmd_vel",               "geometry_msgs.Twist"),
    (ChannelKey::InitialPose,  "",                                       "NUWAX_LCM_CHANNEL_INITIAL_POSE",  "/localization/initial_pose", "geometry_msgs.PoseWithCovarianceStamped"),
    (ChannelKey::GoalPose,     "",                                       "NUWAX_LCM_CHANNEL_GOAL_POSE",     "/nav/goal",              "geometry_msgs.PoseStamped"),
    (ChannelKey::NavState,     "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL","NUWAX_LCM_CHANNEL_NAV_STATE",     "/nav/state",             "nuwax_msgs.NavigationState"),
    (ChannelKey::Frontiers,    "NUWAX_GEOMETRY_RUNTIME_FRONTIERS_CHANNEL","NUWAX_LCM_CHANNEL_FRONTIER",      "/explore/frontiers",     "nuwax_msgs.FrontierList"),
    (ChannelKey::Cancel,       "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL","NUWAX_LCM_CHANNEL_NAV_CANCEL",   "/nav/cancel",            "nuwax_msgs.CancelNavigation"),
    (ChannelKey::OccTopic,     "",                                       "NUWAX_LCM_CHANNEL_OCC_TOPIC",     "/map/occupancy",         "nav_msgs.OccupancyGrid"),
];

/// 按 `ChannelKey` 查表，返回 `(env_runtime, env_platform, default_channel, default_msg_type)`。
fn lookup(key: ChannelKey) -> &'static (&'static str, &'static str, &'static str, &'static str) {
    // linear scan 是 acceptable：仅 12 个 key，编译期为 `static` 数据
    for row in CHANNEL_KEY_TABLE {
        if row.0 == key {
            // 通过指针 cast 抽离四元组；稳定 rust 与 clippy 都接受。
            return unsafe { &*(row as *const _ as *const (&'static str, &'static str, &'static str, &'static str)) };
        }
    }
    unreachable!("ChannelKey table is exhaustive; key={:?} not found", key)
}

/// 解析 env：env_runtime > env_platform > default；返回 typed pattern 字符串
/// （自动补 msg_type）。
fn resolve_typed(key: ChannelKey) -> String {
    let (env_runtime, env_platform, default_ch, default_type) = lookup(key);
    let raw = if !env_runtime.is_empty() {
        std::env::var(env_runtime).ok()
    } else {
        None
    };
    let raw = raw
        .or_else(|| std::env::var(env_platform).ok())
        .unwrap_or_else(|| default_ch.to_string());
    ensure_typed_suffix(&raw, default_ch, default_type, env_runtime, env_platform)
}

/// 把 `<channel>` 或 `<channel>#<msg_type>` 规范成 `<channel>#<msg_type>`。
///
/// - 已有 `#` 时直接返回（用户显式给 typed suffix）；
/// - 没有 `#` 时拼 default channel 校验后补 msg_type（即同 channel 复用
///   default msg_type）；
/// - 与 default channel 不同但无 `#` 时仍补 default msg_type（不假设与
///   default 同名；这是 env-only 兜底）。
fn ensure_typed_suffix(
    raw: &str,
    default_ch: &str,
    default_type: &str,
    env_runtime: &str,
    env_platform: &str,
) -> String {
    let raw = raw.trim();
    if raw.is_empty() {
        // 空值回退到 default typed pattern
        return format!("{}#{}", default_ch, default_type);
    }
    if raw.contains('#') {
        return raw.to_string();
    }
    // 旧硬编码 `/odom` 等缺 typed suffix 的情况：补齐 default msg_type，
    // 同时打 deprecation warning（一次性 stderr 提示运维迁移）。
    if raw != default_ch {
        warn!(
            "{}={} 不含 typed suffix，已自动补 msg_type={}（推荐显式指定 #suffix）",
            env_platform, raw, default_type
        );
    } else if !env_runtime.is_empty() && std::env::var(env_runtime).is_ok() {
        warn!(
            "{}={} 缺 typed suffix，已自动补 msg_type={}（请显式指定 #suffix）",
            env_runtime, raw, default_type
        );
    }
    format!("{}#{}", raw, default_type)
}

/// Phase 4 公共 channel resolver：按 `ChannelKey` 返回 typed
/// `LcmTopic`。Rust 端 100% typed suffix。
///
/// 失败（env 完全缺失且 default 也无法 parse_channel_with_suffix 通过）
/// 时 panic —— 这意味着 default channel 字面量本身错误，必须立刻修。
pub fn channel_for(key: ChannelKey) -> nuwax_protocol::LcmTopic {
    let typed = resolve_typed(key);
    let (topic, _suffix) = nuwax_protocol::LcmTopic::parse_subscription_pattern(&typed)
        .unwrap_or_else(|e| {
            panic!(
                "channel_for({:?}) parse 失败: raw={} err={}（default 配置错误，请联系 runtime owner）",
                key, typed, e
            )
        });
    topic
}

/// 与 `channel_for` 等价，但额外返回 typed msg_type suffix 字符串，
/// 供需要把 msg_type 单独传入 `transport.subscribe` 的调用方使用。
pub fn channel_for_with_suffix(
    key: ChannelKey,
) -> (nuwax_protocol::LcmTopic, String) {
    let typed = resolve_typed(key);
    nuwax_protocol::LcmTopic::parse_subscription_pattern(&typed)
        .unwrap_or_else(|e| {
            panic!(
                "channel_for_with_suffix({:?}) parse 失败: raw={} err={}",
                key, typed, e
            )
        })
}

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
    // Phase 4 修复：platform alias 由 `NUWAX_LCM_CHANNEL_GOAL_POSE` 改为
    // `NUWAX_LCM_CHANNEL_NAV_STATE` —— 历史误用 goal_pose env 解析 nav_state
    // 是 alias 错位，会让运维修改 nav_state channel 时改到 goal_pose env。
    (
        "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL",
        "NUWAX_LCM_CHANNEL_NAV_STATE",
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

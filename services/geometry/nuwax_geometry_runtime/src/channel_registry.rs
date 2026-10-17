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
//!    registry 补齐的 typed channel:odom/cmd_vel/nav_cancel 等)
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

use std::{env::VarError, fmt};

use nuwax_protocol::{LcmMessageType, LcmTopic};
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
// 4. channel 解析与 LCM URL/backend 选择解耦。

/// Phase 4：runtime 与 server.rs 共用的 channel 键枚举。
///
/// 每变体对应 `(env_runtime, env_platform, default_channel, default_msg_type)`
/// 四元组（部分 channel 没有 env_runtime alias，回退到 env_platform + default）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ChannelKey {
    Odom,
    Tf,
    PointCloud,
    Imu,
    CloudMap,
    Occupancy,
    PlanarNavigation,
    RecentCloud3D,
    CmdVel,
    GoalPose,
    NavState,
    Frontiers,
    Cancel,
}

impl ChannelKey {
    pub(crate) const ALL: [Self; 13] = [
        Self::Odom,
        Self::Tf,
        Self::PointCloud,
        Self::Imu,
        Self::CloudMap,
        Self::Occupancy,
        Self::PlanarNavigation,
        Self::RecentCloud3D,
        Self::CmdVel,
        Self::GoalPose,
        Self::NavState,
        Self::Frontiers,
        Self::Cancel,
    ];

    pub(crate) const fn stable_name(self) -> &'static str {
        match self {
            Self::Odom => "odom",
            Self::Tf => "tf",
            Self::PointCloud => "pointcloud",
            Self::Imu => "imu",
            Self::CloudMap => "cloud_map",
            Self::Occupancy => "occupancy",
            Self::PlanarNavigation => "planar_navigation",
            Self::RecentCloud3D => "recent_cloud_3d",
            Self::CmdVel => "cmd_vel",
            Self::GoalPose => "goal_pose",
            Self::NavState => "nav_state",
            Self::Frontiers => "frontiers",
            Self::Cancel => "cancel",
        }
    }
}

/// `(env_runtime_key, env_platform_key, default_channel_without_suffix, default_msg_type)`
///
/// `default_msg_type` 在 default_channel 或 env 值缺 `#msg_type` 时自动补。
const CHANNEL_KEY_TABLE: &[(ChannelKey, &str, &str, &str, &str)] = &[
    // env_runtime                          env_platform                  default                              msg_type
    (
        ChannelKey::Odom,
        "NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL",
        "NUWAX_LCM_CHANNEL_ODOM",
        "/odom",
        "nav_msgs.Odometry",
    ),
    (
        ChannelKey::Tf,
        "NUWAX_GEOMETRY_RUNTIME_TF_CHANNEL",
        "NUWAX_LCM_CHANNEL_TF",
        "/tf",
        "tf2_msgs.TFMessage",
    ),
    (
        ChannelKey::PointCloud,
        "NUWAX_GEOMETRY_RUNTIME_POINTCLOUD_CHANNEL",
        "NUWAX_LCM_CHANNEL_POINTCLOUD",
        "/lidar/points",
        "sensor_msgs.PointCloud2",
    ),
    (
        ChannelKey::Imu,
        "NUWAX_GEOMETRY_RUNTIME_IMU_CHANNEL",
        "NUWAX_LCM_CHANNEL_IMU",
        "/imu/data",
        "sensor_msgs.Imu",
    ),
    (
        ChannelKey::CloudMap,
        "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL",
        "NUWAX_LCM_CHANNEL_CLOUD_MAP",
        "/cloud_map",
        "sensor_msgs.PointCloud2",
    ),
    (
        ChannelKey::PlanarNavigation,
        "NUWAX_GEOMETRY_RUNTIME_PLANAR_NAVIGATION_CHANNEL",
        "NUWAX_LCM_CHANNEL_PLANAR_NAVIGATION",
        nuwax_codec::lcm::planar_navigation::DEFAULT_TOPIC,
        nuwax_codec::lcm::planar_navigation::TYPE_NAME,
    ),
    (
        ChannelKey::RecentCloud3D,
        "", "",
        nuwax_codec::lcm::recent_cloud::DEFAULT_TOPIC,
        nuwax_codec::lcm::recent_cloud::TYPE_NAME,
    ),
    (
        ChannelKey::Occupancy,
        "NUWAX_GEOMETRY_RUNTIME_OCCUPANCY_CHANNEL",
        "NUWAX_LCM_CHANNEL_OCCUPANCY",
        "/map/occupancy",
        "nav_msgs.OccupancyGrid",
    ),
    (
        ChannelKey::CmdVel,
        "NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_CMD_VEL",
        "/cmd_vel",
        "geometry_msgs.Twist",
    ),
    (
        ChannelKey::GoalPose,
        crate::nav_runtime::GOAL_CHANNEL_RUNTIME,
        crate::nav_runtime::GOAL_CHANNEL_PLATFORM,
        "/nav/goal",
        "nuwax_msgs.GoalEnvelope",
    ),
    (
        ChannelKey::NavState,
        "NUWAX_GEOMETRY_RUNTIME_NAV_STATE_CHANNEL",
        "NUWAX_LCM_CHANNEL_NAV_STATE",
        "/nav/state",
        "nuwax_msgs.NavigationState",
    ),
    (
        ChannelKey::Frontiers,
        "NUWAX_GEOMETRY_RUNTIME_FRONTIERS_CHANNEL",
        "NUWAX_LCM_CHANNEL_FRONTIER",
        "/explore/frontiers",
        "nuwax_msgs.FrontierList",
    ),
    (
        ChannelKey::Cancel,
        "NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL",
        "NUWAX_LCM_CHANNEL_NAV_CANCEL",
        "/nav/cancel",
        "nuwax_msgs.CancelNavigation",
    ),
];

/// 按 `ChannelKey` 查表,返回 `(env_runtime, env_platform, default_channel, default_msg_type)`。
///
/// rework19 fix(2026-07-09): 旧的 unsafe `&*(row as *const _ as *const (4-tuple))` 错误
/// — row 是 5 元组 `(ChannelKey, &str, &str, &str, &str)`,cast 写成了 4 元组,
/// 导致重新解释内存布局时偏移错位 + default_type 落到 ChannelKey/padding 字节,
/// `*const u8 = 0x0` + `len = 0xaaaa_...` → `format!("/odom#{}")` 内部 alloc 一个
/// 长度 ~187GB 的 String → `__rust_alloc` 触发 OOM abort。
///
/// 修复:直接解构 row,免去 unsafe cast 的偏移错位风险。
fn lookup(
    key: ChannelKey,
) -> (
    ChannelKey,
    &'static str,
    &'static str,
    &'static str,
    &'static str,
) {
    for row in CHANNEL_KEY_TABLE {
        if row.0 == key {
            return *row;
        }
    }
    unreachable!("ChannelKey table is exhaustive; key={:?} not found", key)
}

/// 解析 env：env_runtime > env_platform > default；返回 typed pattern 字符串
/// （自动补 msg_type）。
fn resolve_typed(key: ChannelKey) -> String {
    let (_key, env_runtime, env_platform, default_ch, default_type) = lookup(key);
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
///
/// 返回 tuple 的第二个元素 `Option<String>`：当 raw pattern 含 `#msg_type`
/// 后缀时为 `Some`，否则为 `None`（与 `parse_subscription_pattern` 行为一致）。
pub fn channel_for_with_suffix(key: ChannelKey) -> (nuwax_protocol::LcmTopic, Option<String>) {
    let typed = resolve_typed(key);
    nuwax_protocol::LcmTopic::parse_subscription_pattern(&typed).unwrap_or_else(|e| {
        panic!(
            "channel_for_with_suffix({:?}) parse 失败: raw={} err={}",
            key, typed, e
        )
    })
}

#[derive(Clone, Debug)]
struct TypedChannelBinding {
    key: ChannelKey,
    topic: LcmTopic,
    message_type: LcmMessageType,
}

impl TypedChannelBinding {
    #[cfg(test)]
    fn pattern(&self) -> String {
        self.topic.as_subscription_pattern(self.message_type.fqn())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct TypedChannelBindingError {
    pub(crate) key: ChannelKey,
    pub(crate) reason: String,
}

impl fmt::Display for TypedChannelBindingError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "LCM_CHANNEL_PREFLIGHT_FAILED channel={}: {}",
            self.key.stable_name(),
            self.reason
        )
    }
}

impl std::error::Error for TypedChannelBindingError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum StrictChannelEnvError {
    Empty { env_key: String },
    NotUnicode { env_key: String },
}

impl fmt::Display for StrictChannelEnvError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty { env_key } => write!(formatter, "env={env_key} value_is_empty"),
            Self::NotUnicode { env_key } => {
                write!(formatter, "env={env_key} value_is_not_unicode")
            }
        }
    }
}

impl std::error::Error for StrictChannelEnvError {}

/// Resolve startup configuration without repairing an explicitly invalid
/// higher-priority value. Only an absent variable may fall through.
pub(crate) fn resolve_strict_channel(
    runtime_alias: &str,
    platform_alias: &str,
    default: &str,
) -> Result<String, StrictChannelEnvError> {
    if let Some(value) = strict_env_value(runtime_alias)? {
        return Ok(value);
    }
    if let Some(value) = strict_env_value(platform_alias)? {
        return Ok(value);
    }
    Ok(default.to_string())
}

fn strict_env_value(env_key: &str) -> Result<Option<String>, StrictChannelEnvError> {
    match std::env::var(env_key) {
        Ok(value) => {
            let trimmed = value.trim();
            if trimmed.is_empty() {
                Err(StrictChannelEnvError::Empty {
                    env_key: env_key.to_string(),
                })
            } else {
                Ok(Some(trimmed.to_string()))
            }
        }
        Err(VarError::NotPresent) => Ok(None),
        Err(VarError::NotUnicode(_)) => Err(StrictChannelEnvError::NotUnicode {
            env_key: env_key.to_string(),
        }),
    }
}

/// Immutable strict bindings for every production registry channel. Unlike
/// the compatibility helpers above, explicitly configured values must carry
/// the correct typed suffix; startup never repairs a malformed binding.
#[derive(Clone, Debug)]
pub(crate) struct ValidatedChannelBindings {
    bindings: Vec<TypedChannelBinding>,
}

impl ValidatedChannelBindings {
    pub(crate) fn from_env() -> Result<Self, TypedChannelBindingError> {
        let bindings = ChannelKey::ALL
            .into_iter()
            .map(strict_binding_for_key)
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self { bindings })
    }

    pub(crate) fn binding(&self, key: ChannelKey) -> (LcmTopic, LcmMessageType) {
        let binding = self
            .bindings
            .iter()
            .find(|binding| binding.key == key)
            .expect("validated channel registry must contain every ChannelKey");
        (binding.topic.clone(), binding.message_type.clone())
    }
}

fn strict_binding_for_key(
    key: ChannelKey,
) -> Result<TypedChannelBinding, TypedChannelBindingError> {
    let (_key, runtime_env, platform_env, default_channel, expected_type) = lookup(key);
    let default_pattern = format!("{default_channel}#{expected_type}");
    let raw =
        resolve_strict_channel(runtime_env, platform_env, &default_pattern).map_err(|error| {
            TypedChannelBindingError {
                key,
                reason: error.to_string(),
            }
        })?;
    strict_binding_from_pattern(key, &raw, expected_type)
}

fn strict_binding_from_pattern(
    key: ChannelKey,
    raw: &str,
    expected_type: &str,
) -> Result<TypedChannelBinding, TypedChannelBindingError> {
    let (topic, suffix) =
        LcmTopic::parse_subscription_pattern(raw).map_err(|error| TypedChannelBindingError {
            key,
            reason: format!("raw={raw} topic_error={error}"),
        })?;
    let suffix = suffix.ok_or_else(|| TypedChannelBindingError {
        key,
        reason: format!("raw={raw} missing typed message suffix"),
    })?;
    let message_type = LcmMessageType::new(&suffix).map_err(|error| TypedChannelBindingError {
        key,
        reason: format!("raw={raw} message_type_error={error}"),
    })?;
    let expected =
        LcmMessageType::new(expected_type).expect("registry message type literals must be valid");
    if message_type != expected {
        return Err(TypedChannelBindingError {
            key,
            reason: format!(
                "raw={raw} message_type={} expected={}",
                message_type.fqn(),
                expected.fqn()
            ),
        });
    }
    Ok(TypedChannelBinding {
        key,
        topic,
        message_type,
    })
}

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
        warn!("{} 设置为空字符串；回退到 default", platform_alias);
    }
    default.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(unix)]
    use std::{ffi::OsString, os::unix::ffi::OsStringExt};

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
        let got = resolve_channel("TEST_RUNTIME_X", "TEST_PLATFORM_X", "/df#x.Df");
        assert_eq!(got, "/rt#x.Rt");
        std::env::remove_var("TEST_RUNTIME_X");
        std::env::remove_var("TEST_PLATFORM_X");
    }

    #[test]
    fn resolve_channel_platform_used_when_runtime_missing() {
        std::env::remove_var("TEST_RUNTIME_Y");
        std::env::set_var("TEST_PLATFORM_Y", "/pf#y.Pf");
        let got = resolve_channel("TEST_RUNTIME_Y", "TEST_PLATFORM_Y", "/df#y.Df");
        assert_eq!(got, "/pf#y.Pf");
        std::env::remove_var("TEST_PLATFORM_Y");
    }

    #[test]
    fn strict_channel_resolution_preserves_valid_precedence_and_default() {
        const RUNTIME: &str = "TEST_STRICT_CHANNEL_VALID_RUNTIME";
        const PLATFORM: &str = "TEST_STRICT_CHANNEL_VALID_PLATFORM";
        std::env::remove_var(RUNTIME);
        std::env::remove_var(PLATFORM);

        assert_eq!(
            resolve_strict_channel(RUNTIME, PLATFORM, "/default#pkg.Default")
                .expect("missing aliases use the default"),
            "/default#pkg.Default"
        );

        std::env::set_var(PLATFORM, "/platform#pkg.Platform");
        assert_eq!(
            resolve_strict_channel(RUNTIME, PLATFORM, "/default#pkg.Default")
                .expect("a valid platform alias is accepted"),
            "/platform#pkg.Platform"
        );

        std::env::set_var(RUNTIME, "/runtime#pkg.Runtime");
        std::env::set_var(PLATFORM, " ");
        assert_eq!(
            resolve_strict_channel(RUNTIME, PLATFORM, "/default#pkg.Default")
                .expect("a valid runtime alias wins without reading platform"),
            "/runtime#pkg.Runtime"
        );

        std::env::remove_var(RUNTIME);
        std::env::remove_var(PLATFORM);
    }

    #[test]
    fn strict_channel_resolution_rejects_whitespace_runtime_before_platform() {
        const RUNTIME: &str = "TEST_STRICT_CHANNEL_EMPTY_RUNTIME";
        const PLATFORM: &str = "TEST_STRICT_CHANNEL_EMPTY_RUNTIME_PLATFORM";
        std::env::set_var(RUNTIME, " \t\n");
        std::env::set_var(PLATFORM, "/platform#pkg.Platform");

        let error = resolve_strict_channel(RUNTIME, PLATFORM, "/default#pkg.Default")
            .expect_err("an explicit empty runtime alias must not fall through");
        assert_eq!(error.to_string(), format!("env={RUNTIME} value_is_empty"));

        std::env::remove_var(RUNTIME);
        std::env::remove_var(PLATFORM);
    }

    #[test]
    fn strict_channel_resolution_rejects_whitespace_platform() {
        const RUNTIME: &str = "TEST_STRICT_CHANNEL_EMPTY_PLATFORM_RUNTIME";
        const PLATFORM: &str = "TEST_STRICT_CHANNEL_EMPTY_PLATFORM";
        std::env::remove_var(RUNTIME);
        std::env::set_var(PLATFORM, " \t\n");

        let error = resolve_strict_channel(RUNTIME, PLATFORM, "/default#pkg.Default")
            .expect_err("an explicit empty platform alias must not fall through");
        assert_eq!(error.to_string(), format!("env={PLATFORM} value_is_empty"));

        std::env::remove_var(PLATFORM);
    }

    #[cfg(unix)]
    #[test]
    fn strict_channel_resolution_rejects_non_unicode_runtime_before_platform() {
        const RUNTIME: &str = "TEST_STRICT_CHANNEL_NON_UNICODE_RUNTIME";
        const PLATFORM: &str = "TEST_STRICT_CHANNEL_NON_UNICODE_RUNTIME_PLATFORM";
        std::env::set_var(RUNTIME, OsString::from_vec(vec![0xff]));
        std::env::set_var(PLATFORM, "/platform#pkg.Platform");

        let error = resolve_strict_channel(RUNTIME, PLATFORM, "/default#pkg.Default")
            .expect_err("a non-Unicode runtime alias must not fall through");
        assert_eq!(
            error.to_string(),
            format!("env={RUNTIME} value_is_not_unicode")
        );

        std::env::remove_var(RUNTIME);
        std::env::remove_var(PLATFORM);
    }

    #[cfg(unix)]
    #[test]
    fn strict_channel_resolution_rejects_non_unicode_platform() {
        const RUNTIME: &str = "TEST_STRICT_CHANNEL_NON_UNICODE_PLATFORM_RUNTIME";
        const PLATFORM: &str = "TEST_STRICT_CHANNEL_NON_UNICODE_PLATFORM";
        std::env::remove_var(RUNTIME);
        std::env::set_var(PLATFORM, OsString::from_vec(vec![0xff]));

        let error = resolve_strict_channel(RUNTIME, PLATFORM, "/default#pkg.Default")
            .expect_err("a non-Unicode platform alias must not fall through");
        assert_eq!(
            error.to_string(),
            format!("env={PLATFORM} value_is_not_unicode")
        );

        std::env::remove_var(PLATFORM);
    }

    #[test]
    fn raw_pointcloud_and_cloud_map_have_distinct_contracts() {
        let (_, raw_runtime, raw_platform, raw_default, raw_type) = lookup(ChannelKey::PointCloud);
        let (_, map_runtime, map_platform, map_default, map_type) = lookup(ChannelKey::CloudMap);

        assert_eq!(raw_runtime, "NUWAX_GEOMETRY_RUNTIME_POINTCLOUD_CHANNEL");
        assert_eq!(raw_platform, "NUWAX_LCM_CHANNEL_POINTCLOUD");
        assert_eq!(raw_default, "/lidar/points");
        assert_eq!(map_runtime, "NUWAX_GEOMETRY_RUNTIME_CLOUDMAP_CHANNEL");
        assert_eq!(map_platform, "NUWAX_LCM_CHANNEL_CLOUD_MAP");
        assert_eq!(map_default, "/cloud_map");
        assert_eq!(raw_type, "sensor_msgs.PointCloud2");
        assert_eq!(map_type, "sensor_msgs.PointCloud2");
    }

    #[test]
    fn every_registry_default_is_a_strict_typed_binding() {
        for key in ChannelKey::ALL {
            let (_, _, _, default_channel, expected_type) = lookup(key);
            let pattern = format!("{default_channel}#{expected_type}");
            let binding = strict_binding_from_pattern(key, &pattern, expected_type)
                .expect("registry default must validate");
            assert_eq!(binding.pattern(), pattern, "key={key:?}");
        }
    }

    #[test]
    fn every_registry_key_rejects_missing_or_wrong_type_suffixes() {
        for key in ChannelKey::ALL {
            let (_, _, _, default_channel, expected_type) = lookup(key);
            let missing = strict_binding_from_pattern(key, default_channel, expected_type)
                .expect_err("explicit untyped channel must fail preflight");
            assert!(missing.reason.contains("missing typed message suffix"));

            let wrong_pattern = format!("{default_channel}#wrong_msgs.Wrong");
            let wrong = strict_binding_from_pattern(key, &wrong_pattern, expected_type)
                .expect_err("wrong message type must fail preflight");
            assert!(wrong.reason.contains("expected="));
        }
    }

    #[test]
    fn malformed_topic_is_rejected_without_panicking() {
        let error = strict_binding_from_pattern(
            ChannelKey::Odom,
            "not-an-lcm-topic#nav_msgs.Odometry",
            "nav_msgs.Odometry",
        )
        .expect_err("topic without leading slash must fail");
        assert!(error.reason.contains("topic_error="));
    }

    #[test]
    fn goal_pose_registry_is_the_versioned_goal_envelope_contract() {
        let (_, runtime, platform, default_channel, expected_type) = lookup(ChannelKey::GoalPose);
        assert_eq!(runtime, crate::nav_runtime::GOAL_CHANNEL_RUNTIME);
        assert_eq!(platform, crate::nav_runtime::GOAL_CHANNEL_PLATFORM);
        assert_eq!(
            format!("{default_channel}#{expected_type}"),
            crate::nav_runtime::GOAL_CHANNEL_DEFAULT
        );
        assert_eq!(expected_type, "nuwax_msgs.GoalEnvelope");
        assert_eq!(CHANNEL_KEY_TABLE.len(), ChannelKey::ALL.len());
        let keys: std::collections::HashSet<_> = CHANNEL_KEY_TABLE.iter().map(|row| row.0).collect();
        assert_eq!(keys.len(), ChannelKey::ALL.len());
        for key in ChannelKey::ALL {
            assert!(keys.contains(&key), "每个生产通道必须出现在唯一表中：{key:?}");
            let (_, _, _, channel, message_type) = lookup(key);
            strict_binding_from_pattern(key, &format!("{channel}#{message_type}"), message_type)
                .expect("生产默认通道必须具备正确消息类型");
        }
    }
}

// 是 platform API 预留入口（Phase 3 mapping / Phase 5 explore 调用面未全部接通）。
// 在不破坏 contract 前提下用 file-level allow 标注；P2 收口期 Phase 6 后期再清理。

//! LCM（轻量通信与编组）通道注册表，只读取平台规范配置名。

use std::{env::VarError, fmt};

use nuwax_protocol::{LcmMessageType, LcmTopic};
use tracing::warn;

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
const CHANNEL_KEY_TABLE: &[(ChannelKey, &str, &str, &str)] = &[
    // 配置名、默认话题、消息类型。
    (
        ChannelKey::Odom,
        "NUWAX_LCM_CHANNEL_ODOM",
        "/odom",
        "nav_msgs.Odometry",
    ),
    (
        ChannelKey::Tf,
        "NUWAX_LCM_CHANNEL_TF",
        "/tf",
        "tf2_msgs.TFMessage",
    ),
    (
        ChannelKey::PointCloud,
        "NUWAX_LCM_CHANNEL_POINTCLOUD",
        "/lidar/points",
        "sensor_msgs.PointCloud2",
    ),
    (
        ChannelKey::Imu,
        "NUWAX_LCM_CHANNEL_IMU",
        "/imu/data",
        "sensor_msgs.Imu",
    ),
    (
        ChannelKey::CloudMap,
        "NUWAX_LCM_CHANNEL_CLOUD_MAP",
        "/cloud_map",
        "sensor_msgs.PointCloud2",
    ),
    (
        ChannelKey::PlanarNavigation,
        "NUWAX_LCM_CHANNEL_PLANAR_NAVIGATION",
        nuwax_codec::lcm::planar_navigation::DEFAULT_TOPIC,
        nuwax_codec::lcm::planar_navigation::TYPE_NAME,
    ),
    (
        ChannelKey::RecentCloud3D,
        "",
        nuwax_codec::lcm::recent_cloud::DEFAULT_TOPIC,
        nuwax_codec::lcm::recent_cloud::TYPE_NAME,
    ),
    (
        ChannelKey::Occupancy,
        "NUWAX_LCM_CHANNEL_OCCUPANCY",
        "/map/occupancy",
        "nav_msgs.OccupancyGrid",
    ),
    (
        ChannelKey::CmdVel,
        "NUWAX_LCM_CHANNEL_CMD_VEL",
        "/cmd_vel",
        "geometry_msgs.Twist",
    ),
    (
        ChannelKey::GoalPose,
        "NUWAX_LCM_CHANNEL_GOAL_POSE",
        "/nav/goal",
        "nuwax_msgs.GoalEnvelope",
    ),
    (
        ChannelKey::NavState,
        "NUWAX_LCM_CHANNEL_NAV_STATE",
        "/nav/state",
        "nuwax_msgs.NavigationState",
    ),
    (
        ChannelKey::Frontiers,
        "NUWAX_LCM_CHANNEL_FRONTIER",
        "/explore/frontiers",
        "nuwax_msgs.FrontierList",
    ),
    (
        ChannelKey::Cancel,
        "NUWAX_LCM_CHANNEL_NAV_CANCEL",
        "/nav/cancel",
        "nuwax_msgs.CancelNavigation",
    ),
];

/// 按通道键查表，返回规范配置名、默认话题与消息类型。
fn lookup(key: ChannelKey) -> (ChannelKey, &'static str, &'static str, &'static str) {
    for row in CHANNEL_KEY_TABLE {
        if row.0 == key {
            return *row;
        }
    }
    unreachable!("ChannelKey table is exhaustive; key={:?} not found", key)
}

/// 读取规范配置名并补齐消息类型。
fn resolve_typed(key: ChannelKey) -> String {
    let (_key, env_key, default_ch, default_type) = lookup(key);
    let raw = std::env::var(env_key).unwrap_or_else(|_| default_ch.to_string());
    ensure_typed_suffix(&raw, default_ch, default_type, env_key)
}

/// 把 `<channel>` 或 `<channel>#<msg_type>` 规范成 `<channel>#<msg_type>`。
///
/// - 已有 `#` 时直接返回（用户显式给 typed suffix）；
/// - 没有 `#` 时拼 default channel 校验后补 msg_type（即同 channel 复用
///   default msg_type）；
/// - 与 default channel 不同但无 `#` 时仍补 default msg_type（不假设与
///   default 同名；这是 env-only 兜底）。
fn ensure_typed_suffix(raw: &str, default_ch: &str, default_type: &str, env_key: &str) -> String {
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
            env_key, raw, default_type
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
    env_key: &str,
    default: &str,
) -> Result<String, StrictChannelEnvError> {
    if let Some(value) = strict_env_value(env_key)? {
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
    let (_key, env_key, default_channel, expected_type) = lookup(key);
    let default_pattern = format!("{default_channel}#{expected_type}");
    let raw = resolve_strict_channel(env_key, &default_pattern).map_err(|error| {
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

/// 读取规范通道配置，缺省使用现有默认值。
pub fn resolve_channel(env_key: &str, default: &str) -> String {
    if let Ok(value) = std::env::var(env_key) {
        let trimmed = value.trim();
        if !trimmed.is_empty() {
            return trimmed.to_string();
        }
        warn!("{} 设置为空字符串；回退到 default", env_key);
    }
    default.to_string()
}

#[cfg(test)]
#[path = "channel_registry_tests.rs"]
mod tests;

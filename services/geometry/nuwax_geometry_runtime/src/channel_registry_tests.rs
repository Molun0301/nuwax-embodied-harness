use super::*;

#[cfg(unix)]
use std::{ffi::OsString, os::unix::ffi::OsStringExt};

#[test]
fn resolve_channel_uses_default_when_no_env() {
    // 不污染环境:用不可能设置的 env 名
    let got = resolve_channel("TEST_NUWAX_PLATFORM_NOT_SET", "/foo#bar.Baz");
    assert_eq!(got, "/foo#bar.Baz");
}

#[test]
fn resolve_channel_platform_used_when_runtime_missing() {
    std::env::remove_var("TEST_RUNTIME_Y");
    std::env::set_var("TEST_PLATFORM_Y", "/pf#y.Pf");
    let got = resolve_channel("TEST_PLATFORM_Y", "/df#y.Df");
    assert_eq!(got, "/pf#y.Pf");
    std::env::remove_var("TEST_PLATFORM_Y");
}

#[test]
fn strict_channel_resolution_rejects_whitespace_platform() {
    const RUNTIME: &str = "TEST_STRICT_CHANNEL_EMPTY_PLATFORM_RUNTIME";
    const PLATFORM: &str = "TEST_STRICT_CHANNEL_EMPTY_PLATFORM";
    std::env::remove_var(RUNTIME);
    std::env::set_var(PLATFORM, " \t\n");

    let error = resolve_strict_channel(PLATFORM, "/default#pkg.Default")
        .expect_err("an explicit empty platform alias must not fall through");
    assert_eq!(error.to_string(), format!("env={PLATFORM} value_is_empty"));

    std::env::remove_var(PLATFORM);
}

#[cfg(unix)]
#[test]
fn strict_channel_resolution_rejects_non_unicode_platform() {
    const RUNTIME: &str = "TEST_STRICT_CHANNEL_NON_UNICODE_PLATFORM_RUNTIME";
    const PLATFORM: &str = "TEST_STRICT_CHANNEL_NON_UNICODE_PLATFORM";
    std::env::remove_var(RUNTIME);
    std::env::set_var(PLATFORM, OsString::from_vec(vec![0xff]));

    let error = resolve_strict_channel(PLATFORM, "/default#pkg.Default")
        .expect_err("a non-Unicode platform alias must not fall through");
    assert_eq!(
        error.to_string(),
        format!("env={PLATFORM} value_is_not_unicode")
    );

    std::env::remove_var(PLATFORM);
}

#[test]
fn raw_pointcloud_and_cloud_map_have_distinct_contracts() {
    let (_, raw_platform, raw_default, raw_type) = lookup(ChannelKey::PointCloud);
    let (_, map_platform, map_default, map_type) = lookup(ChannelKey::CloudMap);

    assert_eq!(raw_platform, "NUWAX_LCM_CHANNEL_POINTCLOUD");
    assert_eq!(raw_default, "/lidar/points");
    assert_eq!(map_platform, "NUWAX_LCM_CHANNEL_CLOUD_MAP");
    assert_eq!(map_default, "/cloud_map");
    assert_eq!(raw_type, "sensor_msgs.PointCloud2");
    assert_eq!(map_type, "sensor_msgs.PointCloud2");
}

#[test]
fn every_registry_default_is_a_strict_typed_binding() {
    for key in ChannelKey::ALL {
        let (_, _, default_channel, expected_type) = lookup(key);
        let pattern = format!("{default_channel}#{expected_type}");
        let binding = strict_binding_from_pattern(key, &pattern, expected_type)
            .expect("registry default must validate");
        assert_eq!(binding.pattern(), pattern, "key={key:?}");
    }
}

#[test]
fn every_registry_key_rejects_missing_or_wrong_type_suffixes() {
    for key in ChannelKey::ALL {
        let (_, _, default_channel, expected_type) = lookup(key);
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
    let (_, platform, default_channel, expected_type) = lookup(ChannelKey::GoalPose);
    assert_eq!(platform, "NUWAX_LCM_CHANNEL_GOAL_POSE");
    assert_eq!(
        format!("{default_channel}#{expected_type}"),
        "/nav/goal#nuwax_msgs.GoalEnvelope"
    );
    assert_eq!(expected_type, "nuwax_msgs.GoalEnvelope");
    assert_eq!(CHANNEL_KEY_TABLE.len(), ChannelKey::ALL.len());
    let keys: std::collections::HashSet<_> = CHANNEL_KEY_TABLE.iter().map(|row| row.0).collect();
    assert_eq!(keys.len(), ChannelKey::ALL.len());
    for key in ChannelKey::ALL {
        assert!(
            keys.contains(&key),
            "每个生产通道必须出现在唯一表中：{key:?}"
        );
        let (_, _, channel, message_type) = lookup(key);
        strict_binding_from_pattern(key, &format!("{channel}#{message_type}"), message_type)
            .expect("生产默认通道必须具备正确消息类型");
    }
}

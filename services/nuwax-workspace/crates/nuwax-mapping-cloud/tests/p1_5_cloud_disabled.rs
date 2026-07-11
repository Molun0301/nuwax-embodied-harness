//! P1 #5 回归测试:默认 feature off 时 cloud map 调用返 `Disabled` 错误。
//!
//! 覆盖：
//! - `google_maps_search` 在默认 build 返 `Err(CloudMapError::Disabled)`
//! - `osm_fetch_tile` 在默认 build 返 `Err(CloudMapError::Disabled)`
//! - `CloudMapError` 错误码可比较、可打印
//!
//! 当 feature `cloud-enabled` 打开时,此测试应被 `cfg(feature = "...")` 跳过
//! （这里不做双 feature 切换,dev 默认 off 已覆盖主路径）。

use nuwax_mapping_cloud::{CloudMapError, google_maps_search, osm_fetch_tile};

#[test]
fn google_maps_disabled_by_default() {
    let r = google_maps_search("any query");
    assert!(matches!(r, Err(CloudMapError::Disabled)));
}

#[test]
fn osm_tile_disabled_by_default() {
    let r = osm_fetch_tile(10, 100, 200);
    assert!(matches!(r, Err(CloudMapError::Disabled)));
}

#[test]
fn cloud_map_error_display_includes_feature_hint() {
    let msg = format!("{}", CloudMapError::Disabled);
    assert!(
        msg.contains("cloud-enabled"),
        "error message should hint the feature flag: {}",
        msg
    );
}

#[test]
fn cloud_map_error_not_implemented_display() {
    let msg = format!("{}", CloudMapError::NotImplemented("text_search"));
    assert!(
        msg.contains("not implemented"),
        "NotImplemented error display: {}",
        msg
    );
}
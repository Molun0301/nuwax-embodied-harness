//! P1 #5 (rework22 Lane D) 实装回归测试。
//!
//! 覆盖：
//! - geocode Mock/Live 分支
//! - places_nearby Mock/Live 分支
//! - nominatim Mock/Live 分支
//! - download_tile Mock 返回 PNG magic
//! - stitch_tiles bytes 拼接
//! - WGS84 ↔ Web Mercator ↔ tile 坐标转换（含 roundtrip 与边界）
//!
//! 所有测试都跑在 Mock 分支（不需要 API key / 外网）。
//!
//! 注意：本测试依赖 `cloud-enabled` feature（默认开启）。当 workspace
//! 中其他 crate 强制不带 feature 编译时，整个 integration test 会被跳过
//! （避免编译错误）。

#[cfg(feature = "cloud-enabled")]
mod cloud_enabled_tests {
    use nuwax_mapping_cloud::{
        CloudError, CloudMode, LatLng, download_tile, geocode, nominatim_search, places_nearby,
        stitch_tiles, web_mercator_to_wgs84, wgs84_to_tile, wgs84_to_web_mercator,
    };

// === google_maps: geocode ============================================

#[test]
fn p1_5_geocode_mock_returns_valid_latlng() {
    let r = geocode("123 Main St, Springfield, USA", CloudMode::Mock).expect("mock ok");
    assert_eq!(r.address, "123 Main St, Springfield, USA");
    assert!((r.location.lat - 37.7749).abs() < 1e-9);
    assert!((r.location.lng - -122.4194).abs() < 1e-9);
    assert_eq!(r.components.street, "123 Main St");
    assert_eq!(r.components.city, "Springfield");
    assert_eq!(r.components.country, "USA");
}

#[test]
fn p1_5_geocode_live_returns_api_key_missing() {
    let r = geocode("anywhere", CloudMode::Live);
    assert!(
        matches!(r, Err(CloudError::ApiKeyMissing)),
        "Live 模式应返 ApiKeyMissing，实际：{:?}",
        r
    );
}

#[test]
fn p1_5_geocode_mock_handles_single_segment() {
    // 仅一个 segment（无逗号）应整体作为 street/country
    let r = geocode("Tokyo Tower", CloudMode::Mock).expect("mock ok");
    assert_eq!(r.components.street, "Tokyo Tower");
    assert_eq!(r.components.city, "");
    assert_eq!(r.components.country, "Tokyo Tower");
}

// === google_maps: places_nearby =====================================

#[test]
fn p1_5_places_nearby_mock_returns_3_places() {
    let p = places_nearby(
        LatLng { lat: 37.7749, lng: -122.4194 },
        1500,
        CloudMode::Mock,
    )
    .expect("mock ok");
    assert_eq!(p.len(), 3);
    // 距离按 radius_m 均匀递增
    assert_eq!(p[0].distance_m, 500);
    assert_eq!(p[1].distance_m, 1000);
    assert_eq!(p[2].distance_m, 1500);
    // categories 不同
    assert_eq!(p[0].category, "cafe");
    assert_eq!(p[1].category, "restaurant");
    assert_eq!(p[2].category, "store");
}

#[test]
fn p1_5_places_nearby_live_returns_api_key_missing() {
    let r = places_nearby(LatLng { lat: 0.0, lng: 0.0 }, 1000, CloudMode::Live);
    assert!(matches!(r, Err(CloudError::ApiKeyMissing)));
}

// === osm: nominatim ==================================================

#[test]
fn p1_5_nominatim_mock_returns_address() {
    let r = nominatim_search("Springfield", CloudMode::Mock).expect("mock ok");
    assert_eq!(r.len(), 1);
    assert_eq!(r[0].display_name, "Mock: Springfield");
    assert_eq!(r[0].osm_type, "way");
    assert!((r[0].lat - 37.7749).abs() < 1e-9);
    assert!((r[0].lng - -122.4194).abs() < 1e-9);
}

#[test]
fn p1_5_nominatim_live_returns_api_key_missing() {
    let r = nominatim_search("Springfield", CloudMode::Live);
    assert!(matches!(r, Err(CloudError::ApiKeyMissing)));
}

// === osm: tile download =============================================

#[test]
fn p1_5_tile_download_mock_returns_png_bytes() {
    let bytes = download_tile(5, 10, 20, CloudMode::Mock).expect("mock ok");
    // PNG file signature (8 bytes)
    assert_eq!(
        &bytes[..8],
        &[0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]
    );
    // IHDR chunk marker
    assert_eq!(&bytes[12..16], b"IHDR");
    // width=256, height=256 (big-endian u32 at offset 16, 20)
    let width = u32::from_be_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
    let height = u32::from_be_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]);
    assert_eq!(width, 256);
    assert_eq!(height, 256);
}

#[test]
fn p1_5_tile_download_live_returns_api_key_missing() {
    let r = download_tile(5, 10, 20, CloudMode::Live);
    assert!(matches!(r, Err(CloudError::ApiKeyMissing)));
}

// === osm: stitch =====================================================

#[test]
fn p1_5_stitch_tiles_concatenates_bytes() {
    let a = vec![1u8; 16];
    let b = vec![2u8; 16];
    let out = stitch_tiles(&[(0, 0, a.clone()), (1, 0, b.clone())], 5).expect("ok");
    // 头部 2 字节（z + count） + 16 + 16 = 34
    assert_eq!(out.len(), 34);
    assert_eq!(out[0], 5);
    assert_eq!(out[1], 2);
    assert_eq!(&out[2..18], &a[..]);
    assert_eq!(&out[18..34], &b[..]);
}

#[test]
fn p1_5_stitch_tiles_empty() {
    let out = stitch_tiles(&[], 5).expect("ok");
    // 空输入：仅头部 2 字节
    assert_eq!(out.len(), 2);
    assert_eq!(out[0], 5);
    assert_eq!(out[1], 0);
}

// === coords: WGS84 <-> Web Mercator ==================================

#[test]
fn p1_5_wgs84_to_web_mercator_known_value() {
    // (lat=0, lng=0) -> (0, 0)
    // FP 噪声：tan(π/4) 在 f64 中不是精确 1，R 倍放大后约 1e-8 量级。
    let (x, y) = wgs84_to_web_mercator(0.0, 0.0);
    assert!(x.abs() < 1e-7, "x drift: {x}");
    assert!(y.abs() < 1e-7, "y drift: {y}");
}

#[test]
fn p1_5_wgs84_to_web_mercator_roundtrip() {
    // 多个点：roundtrip 误差 < 1e-6 度
    let points = [
        (0.0, 0.0),
        (37.7749, -122.4194),    // San Francisco
        (51.5074, -0.1278),       // London
        (-33.8688, 151.2093),     // Sydney
        (35.6762, 139.6503),      // Tokyo
    ];
    for (lat, lng) in points {
        let (x, y) = wgs84_to_web_mercator(lat, lng);
        let (lat2, lng2) = web_mercator_to_wgs84(x, y);
        assert!(
            (lat2 - lat).abs() < 1e-6,
            "lat drift for ({}, {}): {} -> {}",
            lat,
            lng,
            lat,
            lat2
        );
        assert!(
            (lng2 - lng).abs() < 1e-6,
            "lng drift for ({}, {}): {} -> {}",
            lat,
            lng,
            lng,
            lng2
        );
    }
}

// === coords: WGS84 -> tile ===========================================

#[test]
fn p1_5_wgs84_to_tile_at_zoom_0() {
    // z=0：整世界是单瓦片 (0, 0)
    assert_eq!(wgs84_to_tile(0.0, 0.0, 0), (0, 0));
    assert_eq!(wgs84_to_tile(45.0, 90.0, 0), (0, 0));
    assert_eq!(wgs84_to_tile(-45.0, -90.0, 0), (0, 0));
}

#[test]
fn p1_5_wgs84_to_tile_growth() {
    // 同一经纬度在更高 zoom 下应返回更大的 (x, y)
    let (x0, y0) = wgs84_to_tile(37.7749, -122.4194, 5);
    let (x1, y1) = wgs84_to_tile(37.7749, -122.4194, 10);
    let (x2, y2) = wgs84_to_tile(37.7749, -122.4194, 15);
    assert!(x1 > x0, "x should grow with zoom: {x0} -> {x1}");
    assert!(y1 > y0, "y should grow with zoom: {y0} -> {y1}");
    assert!(x2 > x1);
    assert!(y2 > y1);
}

#[test]
fn p1_5_wgs84_to_tile_overflow_safe() {
    // z>=32 不应 panic（实际返 (0, 0)）
    let _ = wgs84_to_tile(0.0, 0.0, 32);
    let _ = wgs84_to_tile(0.0, 0.0, 40);
}

// === 编译期 sanity：所有导出项可见 ===================================

#[test]
fn p1_5_cloud_mode_eq_and_copy() {
    // CloudMode 派生 Copy + Eq，可在 match / assert 内自由使用
    let m = CloudMode::Mock;
    let m2 = m; // Copy
    assert_eq!(m, m2);
    assert_ne!(m, CloudMode::Live);
}
}
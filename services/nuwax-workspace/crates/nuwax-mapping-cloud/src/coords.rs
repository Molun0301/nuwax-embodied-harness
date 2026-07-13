//! 坐标转换工具
//!
//! P1 #5 (rework22 Lane D) 实装：WGS84 ↔ Web Mercator（EPSG:3857）+ tile 坐标。
//!
//! 公式参考：
//! - Web Mercator: `https://en.wikipedia.org/wiki/Web_Mercator_projection`
//! - Slippy map tile: `https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames`

use std::f64::consts::PI;

/// Web Mercator 半周长（米），等价于地球赤道周长一半
const WEB_MERCATOR_HALF_CIRCUMFERENCE: f64 = 20_037_508.342_789_244;

/// WGS84 (lat/lng degrees) → Web Mercator (meters, EPSG:3857)。
///
/// 返回 `(x, y)`，单位米。`x ∈ [-20_037_508.34, +20_037_508.34]`
/// 对应经度 `[-180°, +180°]`；`y` 范围同上对应纬度 `≈[-85.05°, +85.05°]`
/// （Web Mercator 在极区收敛到无穷大，工程上裁剪到 ±85.05112878）。
///
/// 公式：`x = lng * R`、`y = R * ln(tan(π/4 + lat_rad/2))`。
///
/// # Panics
/// 不 panic。`|lat| > 85.05112878` 时 `y` 会趋向 ±∞，调用方需自行裁剪。
#[must_use]
pub fn wgs84_to_web_mercator(lat: f64, lng: f64) -> (f64, f64) {
    let x = lng * WEB_MERCATOR_HALF_CIRCUMFERENCE / 180.0;
    let lat_rad = lat * PI / 180.0;
    let y = WEB_MERCATOR_HALF_CIRCUMFERENCE * ((PI / 4.0 + lat_rad / 2.0).tan().ln());
    (x, y)
}

/// Web Mercator (meters) → WGS84 (lat/lng degrees)。
///
/// `wgs84_to_web_mercator` 的精确逆运算。
///
/// 公式：`lng = x / R`、`lat_rad = 2 * atan(exp(y / R)) - π/2`。
///
/// # Panics
/// 不 panic。
#[must_use]
pub fn web_mercator_to_wgs84(x: f64, y: f64) -> (f64, f64) {
    let lng = x * 180.0 / WEB_MERCATOR_HALF_CIRCUMFERENCE;
    let lat_rad = 2.0 * (y / WEB_MERCATOR_HALF_CIRCUMFERENCE).exp().atan() - PI / 2.0;
    let lat = lat_rad * 180.0 / PI;
    (lat, lng)
}

/// WGS84 (lat/lng) → tile 坐标 `(x, y)` at zoom level `z`。
///
/// 采用 slippy map tile 命名约定（OSM / Google Maps 一致）：
/// `z=0` 时整世界为单瓦片 `(0, 0)`；`z` 每 +1，瓦片网格边长 ×2。
///
/// 边界行为：`(lat, lng) = (边界值)` 时返回对应瓦片下边界索引。
///
/// # Panics
/// 不 panic。`z > 31` 时 `2^z` 可能溢出 u32，函数会返回 `(0, 0)`。
#[must_use]
pub fn wgs84_to_tile(lat: f64, lng: f64, z: u8) -> (u32, u32) {
    // 防止 z 过大导致 2^z 溢出 u32（>31 即溢出）
    let n = if z >= 32 {
        0.0
    } else {
        f64::from(2_u32.pow(u32::from(z)))
    };
    let x_norm = (lng + 180.0) / 360.0;
    let x = (x_norm * n).floor().max(0.0) as u32;
    let lat_rad = lat * PI / 180.0;
    let y_norm = (1.0 - ((lat_rad.tan() + 1.0 / lat_rad.cos()).ln() / PI)) / 2.0;
    let y = (y_norm * n).floor().max(0.0) as u32;
    (x, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mercator_roundtrip_zero() {
        let (x, y) = wgs84_to_web_mercator(0.0, 0.0);
        let (lat, lng) = web_mercator_to_wgs84(x, y);
        assert!(lat.abs() < 1e-9, "lat drift: {lat}");
        assert!(lng.abs() < 1e-9, "lng drift: {lng}");
    }

    #[test]
    fn mercator_roundtrip_sf() {
        // San Francisco: lat=37.7749, lng=-122.4194
        let (x, y) = wgs84_to_web_mercator(37.7749, -122.4194);
        let (lat, lng) = web_mercator_to_wgs84(x, y);
        assert!((lat - 37.7749).abs() < 1e-6, "lat drift: {lat}");
        assert!((lng - -122.4194).abs() < 1e-6, "lng drift: {lng}");
    }

    #[test]
    fn mercator_known_values() {
        // 已知参考点：(lat=0, lng=0) → (0, 0)
        // FP 噪声：tan(π/4) 在 f64 中 ≈ 0.9999...，经 R 倍放大后约 1e-9 量级。
        let (x0, y0) = wgs84_to_web_mercator(0.0, 0.0);
        assert!(x0.abs() < 1e-7, "x drift: {x0}");
        assert!(y0.abs() < 1e-7, "y drift: {y0}");
        // (lat=0, lng=180) → x = 20_037_508.34, y = 0
        let (x, y) = wgs84_to_web_mercator(0.0, 180.0);
        assert!(
            (x - WEB_MERCATOR_HALF_CIRCUMFERENCE).abs() < 1e-3,
            "x drift: {x}"
        );
        // FP 噪声：tan(π/4) 在 f64 中不是精确 1，R 倍放大后约 1e-8 量级。
        assert!(y.abs() < 1e-7, "y drift at lng=180: {y}");
    }

    #[test]
    fn tile_zero_at_zoom_zero() {
        assert_eq!(wgs84_to_tile(0.0, 0.0, 0), (0, 0));
    }

    #[test]
    fn tile_sf_at_zoom_10() {
        // SF (37.7749, -122.4194) at zoom 10 → (163, 395)（OSM 标准）
        // 容许 1 瓦片误差（不同实现取整边界可能差 1）
        let (x, y) = wgs84_to_tile(37.7749, -122.4194, 10);
        assert!(x == 163 || x == 162, "x={x}, expected 162-163");
        assert!(y == 395 || y == 396, "y={y}, expected 395-396");
    }

    #[test]
    fn tile_north_pole_clamps() {
        // 北极 (90, 0) 在 z=0 应映射到 (0, 0)（y=0 是顶部）
        let (_x, y) = wgs84_to_tile(90.0, 0.0, 0);
        assert_eq!(y, 0);
    }

    #[test]
    fn tile_overflow_z_does_not_panic() {
        // z=32 防止溢出，应返 (0, 0)
        let _ = wgs84_to_tile(0.0, 0.0, 32);
        let _ = wgs84_to_tile(0.0, 0.0, 40);
    }
}
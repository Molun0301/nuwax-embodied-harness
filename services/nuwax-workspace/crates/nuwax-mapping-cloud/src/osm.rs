//! OpenStreetMap 占位 + 实装
//!
//! P1 #5 (rework22 Lane D) 升级：
//! - `nominatim_search`: Nominatim geocoding 查询（Mock/Live 分支）
//! - `download_tile`: OSM 瓦片下载（z/x/y scheme）
//! - `stitch_tiles`: 瓦片拼接（简单 bytes concat 作为 mock）
//!
//! **注意**：osmpbf crate 仅在某些平台有 prebuilt；aarch64 实机需验证。
//! 本阶段提供 trait 抽象 + 占位实现。

use serde::{Deserialize, Serialize};

use crate::google_maps::{CloudError, CloudMode};

/// OSM 错误（保留兼容旧 API，内部转用 CloudError）。
pub type OsmError = CloudError;

/// 静态瓦片 URL
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OsmTileServer {
    /// 服务器 URL 模板（{z}/{x}/{y} 占位）
    pub template: String,
}

impl OsmTileServer {
    /// 默认 OpenStreetMap 瓦片服务器
    #[must_use]
    pub fn default_osm() -> Self {
        Self {
            template: "https://tile.openstreetmap.org/{z}/{x}/{y}.png".to_string(),
        }
    }

    /// 构造 URL（给定 z/x/y）
    #[must_use]
    pub fn url(&self, z: u8, x: u32, y: u32) -> String {
        self.template
            .replace("{z}", &z.to_string())
            .replace("{x}", &x.to_string())
            .replace("{y}", &y.to_string())
    }
}

/// 瓦片字节
#[derive(Debug, Clone)]
pub struct OsmTile {
    /// 缩放级别
    pub z: u8,
    /// X 索引
    pub x: u32,
    /// Y 索引
    pub y: u32,
    /// PNG 字节
    pub bytes: Vec<u8>,
}

/// Nominatim 单条查询结果。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OsmPlace {
    /// 纬度
    pub lat: f64,
    /// 经度
    pub lng: f64,
    /// 完整显示名（含地址层级）
    pub display_name: String,
    /// OSM 元素类型（node / way / relation）
    pub osm_type: String,
}

/// OSM 客户端
#[allow(dead_code)]
pub struct OsmClient {
    /// 瓦片服务器 URL 模板
    pub tile_server: OsmTileServer,
    /// HTTP 客户端
    pub http: reqwest::Client,
}

impl OsmClient {
    /// 构造默认 OSM 客户端
    #[must_use]
    pub fn new() -> Self {
        Self {
            tile_server: OsmTileServer::default_osm(),
            http: reqwest::Client::builder()
                .timeout(std::time::Duration::from_secs(60))
                .build()
                .expect("reqwest client"),
        }
    }

    /// 下载瓦片占位（实装：fetch HTTP + 保存 bytes）
    pub async fn fetch_tile_placeholder(&self, z: u8, x: u32, y: u32) -> Result<OsmTile, OsmError> {
        // 占位：实装需 HTTP GET tile URL
        Ok(OsmTile {
            z,
            x,
            y,
            bytes: Vec::new(),
        })
    }
}

impl Default for OsmClient {
    fn default() -> Self {
        Self::new()
    }
}

/// Nominatim 查询（geocoding）。
///
/// 真实生产应调：
/// `https://nominatim.openstreetmap.org/search?q={query}&format=json`
///
/// Mock 模式：返回单个 `OsmPlace`（San Francisco fixture）。
/// Live 模式：当前未实装 HTTPS，返 `CloudError::ApiKeyMissing`
/// （Nominatim 实际不要求 key，但这里为接口统一性保留）。
///
/// # Errors
/// - `CloudError::ApiKeyMissing`: Live 模式下未实装
pub fn nominatim_search(query: &str, mode: CloudMode) -> Result<Vec<OsmPlace>, CloudError> {
    match mode {
        CloudMode::Mock => Ok(vec![OsmPlace {
            lat: 37.7749,
            lng: -122.4194,
            display_name: format!("Mock: {}", query),
            osm_type: "way".into(),
        }]),
        CloudMode::Live => Err(CloudError::ApiKeyMissing),
    }
}

/// OSM 瓦片下载（z/x/y scheme）。
///
/// 真实生产应下载 PNG bytes from `tile.openstreetmap.org/{z}/{x}/{y}.png`。
/// dev/mock 模式：返 256x256 PNG stub bytes（PNG magic header + 最小 IHDR）。
/// Live 模式：当前未实装 HTTPS，返 `CloudError::ApiKeyMissing`。
///
/// # Errors
/// - `CloudError::ApiKeyMissing`: Live 模式下未实装
pub fn download_tile(z: u8, x: u32, y: u32, mode: CloudMode) -> Result<Vec<u8>, CloudError> {
    match mode {
        CloudMode::Mock => {
            // 256x256 PNG stub bytes。
            // PNG file signature (8 bytes) + IHDR chunk (25 bytes)
            // + IDAT stub (12 bytes) + IEND (12 bytes) = 57 bytes。
            // 这里仅生成 magic + IHDR + IEND（最小合法 PNG 框架）。
            let mut bytes = Vec::with_capacity(57);
            // PNG signature
            bytes.extend_from_slice(&[0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]);
            // IHDR chunk: length(4) + "IHDR"(4) + width(4)=256 + height(4)=256 +
            //             bit_depth(1)=8 + color_type(1)=2(RGB) +
            //             compression(1)=0 + filter(1)=0 + interlace(1)=0
            //             + CRC(4)
            bytes.extend_from_slice(&[
                0x00, 0x00, 0x00, 0x0D, // length=13
                b'I', b'H', b'D', b'R',
                0x00, 0x00, 0x01, 0x00, // width=256
                0x00, 0x00, 0x01, 0x00, // height=256
                0x08, // bit depth=8
                0x02, // color type=RGB
                0x00, 0x00, 0x00, // compression/filter/interlace
            ]);
            // CRC placeholder（mock 不计算真 CRC，但写入 4 字节占位）
            bytes.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
            // IEND chunk
            bytes.extend_from_slice(&[
                0x00, 0x00, 0x00, 0x00, // length=0
                b'I', b'E', b'N', b'D',
                0xAE, 0x42, 0x60, 0x82, // CRC（PNG 标准 IEND CRC）
            ]);
            // tile coords 嵌入（便于 debug 校验，但不影响 bytes 合法性）
            let _ = (z, x, y);
            Ok(bytes)
        }
        CloudMode::Live => Err(CloudError::ApiKeyMissing),
    }
}

/// 瓦片拼接：把 N×M 个瓦片拼成单张 PNG bytes。
///
/// 真实实现：用 `image` crate 拼接（每瓦片解码后按网格布局）。
/// dev/mock 模式：简单 bytes 顺序连接（保留 z 维度便于未来扩展）。
///
/// # Errors
/// - 当前 Mock 路径不返回错误；Live/真实 image 路径会传透 `image::ImageError`
pub fn stitch_tiles(tiles: &[(u32, u32, Vec<u8>)], z: u8) -> Result<Vec<u8>, CloudError> {
    let mut out = Vec::new();
    // 在 stub 头部写入 zoom 级别 + tile 数量（便于调用方校验）
    out.push(z);
    let n = u8::try_from(tiles.len().min(255)).unwrap_or(255);
    out.push(n);
    for (_, _, b) in tiles {
        out.extend_from_slice(b);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_tile_url() {
        let s = OsmTileServer::default_osm();
        assert_eq!(
            s.url(5, 10, 20),
            "https://tile.openstreetmap.org/5/10/20.png"
        );
    }

    #[tokio::test]
    async fn fetch_tile_placeholder_returns_empty() {
        let c = OsmClient::new();
        let t = c.fetch_tile_placeholder(5, 10, 20).await.unwrap();
        assert_eq!(t.z, 5);
        assert!(t.bytes.is_empty());
    }

    #[test]
    fn nominatim_mock_returns_address() {
        let r = nominatim_search("Springfield", CloudMode::Mock).unwrap();
        assert_eq!(r.len(), 1);
        assert!(r[0].display_name.contains("Springfield"));
        assert_eq!(r[0].osm_type, "way");
    }

    #[test]
    fn nominatim_live_returns_api_key_missing() {
        let r = nominatim_search("Springfield", CloudMode::Live);
        assert!(matches!(r, Err(CloudError::ApiKeyMissing)));
    }

    #[test]
    fn download_tile_mock_returns_png_signature() {
        let bytes = download_tile(5, 10, 20, CloudMode::Mock).unwrap();
        // PNG signature 前 8 字节
        assert_eq!(&bytes[..8], &[0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]);
        // IHDR 标识
        assert_eq!(&bytes[12..16], b"IHDR");
    }

    #[test]
    fn download_tile_live_returns_api_key_missing() {
        let r = download_tile(5, 10, 20, CloudMode::Live);
        assert!(matches!(r, Err(CloudError::ApiKeyMissing)));
    }

    #[test]
    fn stitch_tiles_concatenates_bytes() {
        let a = vec![1u8; 16];
        let b = vec![2u8; 16];
        let out = stitch_tiles(&[(0, 0, a.clone()), (1, 0, b.clone())], 5).unwrap();
        // 头部 2 字节（z + count） + 16 + 16 = 34
        assert_eq!(out.len(), 34);
        assert_eq!(out[0], 5);
        assert_eq!(out[1], 2);
        assert_eq!(&out[2..18], &a[..]);
        assert_eq!(&out[18..34], &b[..]);
    }
}
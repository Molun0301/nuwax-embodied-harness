//! OpenStreetMap 占位
//! **注意**：osmpbf crate 仅在某些平台有 prebuilt；aarch64 实机需验证。
//! 本阶段提供 trait 抽象 + 占位实现。

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// OSM 错误
#[derive(Debug, Error)]
pub enum OsmError {
    /// 解析错误
    #[error("parse error: {0}")]
    Parse(String),
    /// 下载错误
    #[error("download error: {0}")]
    Download(String),
}

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

/// OSM 客户端
#[allow(dead_code)] // http 字段为占位骨架预留，实装 Overpass API 时启用
pub struct OsmClient {
    /// 瓦片服务器 URL 模板
    pub tile_server: OsmTileServer,
    /// HTTP 客户端（实装后用于 Overpass API 调用）
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
}

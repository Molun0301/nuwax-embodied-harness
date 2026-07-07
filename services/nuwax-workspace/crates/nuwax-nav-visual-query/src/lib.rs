//! 视觉查询（图像 + 自然语言描述 -> bbox 或 None）。
//!
//! 兼容基线是 VLM JSON bbox query，不是 embedding/排序式检索。
//!
//! 模型 runtime ownership 硬边界：
//! - 本 crate 只定义 `VlBBoxModel` trait、JSON bbox parser、prompt 模板、mock/test adapter。
//! - 任何模型加载、权重路径、tokenizer、GPU provider、下载缓存属于视觉/感知板块 crate。
//! - 导航 runtime 只能通过 trait object / 已注册 vision service 调用模型。
//!
//! 单元测试必须使用 mock model，不得访问网络。

#![deny(unsafe_code)]
#![warn(missing_docs)]

// Foundation 协议面：BoundingBox2D / ImageEncoding 等强类型
pub use nuwax_nav_types_messages::BoundingBox2D;
pub use nuwax_nav_types_messages::ImageEncoding;

/// Foundation `ImageView` 别名（带 encoding 字段）。
///
/// 外部调用方可以选用 `nuwax_nav_visual_query::FoundationImageView` 表达完整协议面
/// （width + height + encoding + data）；本 crate 内部继续使用本地轻量 `ImageView`。
pub use nuwax_nav_base::ImageView as FoundationImageView;

/// 轻量 ImageView（自包含，width/height/data 三件套；与 Foundation ImageView 同语义）。
///
/// 历史 `vl_bbox` trait 接收 3 字段 struct literal；保留本地类型以不破坏测试与
/// 现有调用方。Foundation `ImageView` 多一个 `encoding: ImageEncoding` 字段，
/// 通过 `FoundationImageView` 别名独立暴露。
#[derive(Debug, Clone, Copy)]
pub struct ImageView<'a> {
    /// width
    pub width: u32,
    /// height
    pub height: u32,
    /// raw bytes
    pub data: &'a [u8],
}

/// VLM bbox model trait（由视觉/感知板块实现；本 crate 仅消费）
pub trait VlBBoxModel: Send + Sync {
    /// 查询图像，返回模型原始输出（字符串）。模型可能返回多余文本，parser 会抽取 JSON。
    fn query(&self, image: ImageView<'_>, prompt: &str) -> Result<String, VisualQueryError>;
}

/// Visual query 错误（不含模型选择相关枚举）
#[derive(Debug, thiserror::Error)]
pub enum VisualQueryError {
    /// 模型调用失败
    #[error("VLM model query failed: {0}")]
    ModelFailed(String),
    /// 模型返回空
    #[error("VLM model returned empty response")]
    Empty,
}

/// prompt 模板
#[must_use]
pub fn build_prompt(object_description: &str) -> String {
    format!(
        "Look at this image and find the '{object_description}'. \
         Return ONLY a JSON object with format: {{\"name\": \"object_name\", \
         \"bbox\": [x1, y1, x2, y2]}} where x1,y1 is the top-left and \
         x2,y2 is the bottom-right corner of the bounding box. \
         If not found, return None."
    )
}

/// 从模型响应中提取 JSON bbox；不 eval 任意字符串。
///
/// 处理：
/// - 找到第一个 `{...}` JSON object 段；
/// - 解析失败或不含 `bbox` 字段返回 `Ok(None)`；
/// - bbox 长度必须 = 4；坐标无效或 `x1>x2 || y1>y2` 返回 `Ok(None)`；
/// - 不抛模型响应异常给上层。
#[must_use]
pub fn extract_bbox_from_response(response: &str) -> Option<BoundingBox2D> {
    let json_segment = extract_first_json_object(response)?;
    let parsed: serde_json::Value = match serde_json::from_str(&json_segment) {
        Ok(v) => v,
        Err(_) => return None,
    };
    let bbox = parsed.get("bbox")?;
    let arr = bbox.as_array()?;
    if arr.len() != 4 {
        return None;
    }
    let mut coords = [0.0_f32; 4];
    for (i, v) in arr.iter().enumerate() {
        let n = v.as_f64()?;
        if !n.is_finite() {
            return None;
        }
        coords[i] = n as f32;
    }
    // 严格 reject 语义（与历史本地 BoundingBox2D 一致）：x1>x2 或 y1>y2 视为无效；
    // Foundation `from_xyxy` 会自动 normalize，故先按 strictly ordered 校验一遍。
    if !(coords[0] <= coords[2] && coords[1] <= coords[3]) {
        return None;
    }
    let name = parsed
        .get("name")
        .and_then(|v| v.as_str())
        .map(String::from);
    let mut bbox = BoundingBox2D::from_xyxy(coords[0], coords[1], coords[2], coords[3])?;
    bbox.class_name = name;
    Some(bbox)
}

fn extract_first_json_object(s: &str) -> Option<String> {
    let bytes = s.as_bytes();
    let mut start = None;
    let mut depth: i32 = 0;
    let mut in_string = false;
    let mut escape = false;
    for (i, &b) in bytes.iter().enumerate() {
        let c = b as char;
        if in_string {
            if escape {
                escape = false;
            } else if c == '\\' {
                escape = true;
            } else if c == '"' {
                in_string = false;
            }
            continue;
        }
        match c {
            '"' => in_string = true,
            '{' => {
                if depth == 0 {
                    start = Some(i);
                }
                depth += 1;
            }
            '}' if depth > 0 => {
                depth -= 1;
                if depth == 0 {
                    let start_idx = start?;
                    // 必须按 UTF-8 char boundary 切（i 是字节偏移）
                    if !s.is_char_boundary(i + 1) {
                        return None;
                    }
                    return Some(s[start_idx..=i].to_string());
                }
            }
            _ => {}
        }
    }
    None
}

/// 主入口：通过模型拿 bbox
pub fn get_object_bbox_from_image(
    model: &dyn VlBBoxModel,
    image: ImageView<'_>,
    object_description: &str,
) -> Result<Option<BoundingBox2D>, VisualQueryError> {
    let prompt = build_prompt(object_description);
    let response = model.query(image, &prompt)?;
    if response.trim().is_empty() {
        return Ok(None);
    }
    Ok(extract_bbox_from_response(&response))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;

    /// Mock model，直接返回固定字符串
    struct MockModel(&'static str);

    impl VlBBoxModel for MockModel {
        fn query(&self, _image: ImageView<'_>, _prompt: &str) -> Result<String, VisualQueryError> {
            Ok(self.0.to_string())
        }
    }

    #[test]
    fn prompt_references_description() {
        let p = build_prompt("red chair");
        assert!(p.contains("red chair"));
        assert!(p.contains("bbox"));
        assert!(p.contains("None"));
    }

    #[test]
    fn pure_json_parses() {
        let m = MockModel(r#"{"name": "chair", "bbox": [100, 100, 200, 200]}"#);
        let bytes: &[u8] = &[];
        let r = get_object_bbox_from_image(
            &m,
            ImageView {
                width: 640,
                height: 480,
                data: bytes,
            },
            "chair",
        )
        .unwrap();
        let bbox = r.expect("should parse");
        assert_eq!(bbox.x1, 100.0);
        assert_eq!(bbox.x2, 200.0);
        assert_eq!(bbox.class_name.as_deref(), Some("chair"));
    }

    #[test]
    fn extra_text_around_json_parses() {
        let m = MockModel("Sure! Here it is: {\"bbox\": [10, 20, 30, 40], \"name\": \"obj\"} ok.");
        let bytes: &[u8] = &[];
        let r = get_object_bbox_from_image(
            &m,
            ImageView {
                width: 640,
                height: 480,
                data: bytes,
            },
            "obj",
        )
        .unwrap();
        let bbox = r.expect("should parse");
        assert_eq!(bbox.x1, 10.0);
        assert_eq!(bbox.y2, 40.0);
    }

    #[test]
    fn model_says_none_returns_none() {
        let m = MockModel("None");
        let bytes: &[u8] = &[];
        let r = get_object_bbox_from_image(
            &m,
            ImageView {
                width: 640,
                height: 480,
                data: bytes,
            },
            "obj",
        )
        .unwrap();
        assert!(r.is_none());
    }

    #[test]
    fn no_bbox_field_returns_none() {
        let m = MockModel(r#"{"name": "obj"}"#);
        let bytes: &[u8] = &[];
        let r = get_object_bbox_from_image(
            &m,
            ImageView {
                width: 640,
                height: 480,
                data: bytes,
            },
            "obj",
        )
        .unwrap();
        assert!(r.is_none());
    }

    #[test]
    fn bbox_wrong_length_returns_none() {
        let m = MockModel(r#"{"bbox": [1, 2, 3]}"#);
        let bytes: &[u8] = &[];
        let r = get_object_bbox_from_image(
            &m,
            ImageView {
                width: 640,
                height: 480,
                data: bytes,
            },
            "obj",
        )
        .unwrap();
        assert!(r.is_none());
    }

    #[test]
    fn bbox_non_numeric_returns_none() {
        let m = MockModel(r#"{"bbox": ["a", "b", "c", "d"]}"#);
        let bytes: &[u8] = &[];
        let r = get_object_bbox_from_image(
            &m,
            ImageView {
                width: 640,
                height: 480,
                data: bytes,
            },
            "obj",
        )
        .unwrap();
        assert!(r.is_none());
    }

    #[test]
    fn bbox_inverted_coords_returns_none() {
        let m = MockModel(r#"{"bbox": [200, 200, 100, 100]}"#); // x2 < x1
        let bytes: &[u8] = &[];
        let r = get_object_bbox_from_image(
            &m,
            ImageView {
                width: 640,
                height: 480,
                data: bytes,
            },
            "obj",
        )
        .unwrap();
        assert!(r.is_none());
    }

    #[test]
    fn empty_response_no_error() {
        struct EmptyModel;
        impl VlBBoxModel for EmptyModel {
            fn query(&self, _: ImageView<'_>, _: &str) -> Result<String, VisualQueryError> {
                Ok(String::new())
            }
        }
        let bytes: &[u8] = &[];
        let r = get_object_bbox_from_image(
            &EmptyModel,
            ImageView {
                width: 640,
                height: 480,
                data: bytes,
            },
            "obj",
        )
        .unwrap();
        assert!(r.is_none());
    }

    #[test]
    fn arbitrary_string_does_not_panic() {
        // 不应崩
        let _ = extract_bbox_from_response("random words without json");
        let _ = extract_bbox_from_response("{ malformed json");
        let _ = extract_bbox_from_response("");
    }
}

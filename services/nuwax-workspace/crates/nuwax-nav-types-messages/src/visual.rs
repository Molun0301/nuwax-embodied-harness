//! 视觉相关消息:图像编码、bbox、visual target、frontier goal
//!
//! ## 硬约束(全仓)
//!
//! - `BoundingBox2D` 一律使用 **xyxy 像素坐标**(原点左上,`x1 <= x2`, `y1 <= y2`)
//! - 任何模型 / 外部来源返回的 xywh / 归一化坐标必须先转换
//! - `FrontierGoal` 不暴露 wavefront 算法内部结构

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::geometry::{PoseStamped, Quaternion, Timestamp};

/// 图像编码
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum ImageEncoding {
    /// 8-bit RGB(每像素 3 字节,R G B)
    #[default]
    Rgb8,
    /// 8-bit BGR
    Bgr8,
    /// 8-bit RGBA
    Rgba8,
    /// 8-bit Gray
    Gray8,
    /// JPEG 压缩
    Jpeg,
    /// PNG 压缩
    Png,
    /// 其它编码(字符串透传)
    Other(String),
}

/// 2D 包围盒(像素坐标,xyxy,原点左上)
///
/// 全仓统一表达。`x1 <= x2`, `y1 <= y2`。`score/class_name/source` 为可选信息。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox2D {
    /// 左上角 x
    pub x1: f32,
    /// 左上角 y
    pub y1: f32,
    /// 右下角 x
    pub x2: f32,
    /// 右下角 y
    pub y2: f32,
    /// 置信度
    pub score: Option<f32>,
    /// 类别名
    pub class_name: Option<String>,
    /// 来源(模型 / tracker / 标注)
    pub source: Option<String>,
}

impl BoundingBox2D {
    /// 由 xywh 像素坐标构造(自动转为 xyxy)
    ///
    /// 任何传入 NaN / 负宽高都会返回 `None`,不 panic。
    #[must_use]
    pub fn from_xywh(x: f32, y: f32, w: f32, h: f32) -> Option<Self> {
        if !x.is_finite() || !y.is_finite() || !w.is_finite() || !h.is_finite() {
            return None;
        }
        if w < 0.0 || h < 0.0 {
            return None;
        }
        Some(Self {
            x1: x,
            y1: y,
            x2: x + w,
            y2: y + h,
            score: None,
            class_name: None,
            source: None,
        })
    }

    /// 由 xyxy 像素坐标构造,自动归一化(若 `x1 > x2` 则交换)
    ///
    /// 任何 NaN 返回 `None`。
    #[must_use]
    pub fn from_xyxy(x1: f32, y1: f32, x2: f32, y2: f32) -> Option<Self> {
        if !x1.is_finite() || !y1.is_finite() || !x2.is_finite() || !y2.is_finite() {
            return None;
        }
        let (lo_x, hi_x) = if x1 <= x2 { (x1, x2) } else { (x2, x1) };
        let (lo_y, hi_y) = if y1 <= y2 { (y1, y2) } else { (y2, y1) };
        Some(Self {
            x1: lo_x,
            y1: lo_y,
            x2: hi_x,
            y2: hi_y,
            score: None,
            class_name: None,
            source: None,
        })
    }

    /// 宽度
    #[must_use]
    pub fn width(&self) -> f32 {
        self.x2 - self.x1
    }

    /// 高度
    #[must_use]
    pub fn height(&self) -> f32 {
        self.y2 - self.y1
    }

    /// 中心点(像素)
    #[must_use]
    pub fn center(&self) -> (f32, f32) {
        ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)
    }

    /// 面积
    #[must_use]
    pub fn area(&self) -> f32 {
        self.width().max(0.0) * self.height().max(0.0)
    }

    /// 是否合理(非 NaN / 非退化 / xyxy 有序)
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.x1.is_finite()
            && self.y1.is_finite()
            && self.x2.is_finite()
            && self.y2.is_finite()
            && self.x1 <= self.x2
            && self.y1 <= self.y2
    }
}

/// 视觉目标(visual servoing / bbox tracker 输入)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VisualTarget {
    /// bbox(像素 xyxy)
    pub bbox: BoundingBox2D,
    /// 自然语言描述 / 类别短语
    pub phrase: String,
    /// 图像时间戳
    pub image_stamp: Timestamp,
    /// 图像坐标系
    pub camera_frame: crate::frames::NavFrameId,
    /// 可选:目标在 camera frame 下的 3D 估计(由上游检测器提供)
    pub camera_position: Option<crate::geometry::Point3>,
}

impl VisualTarget {
    /// 由 bbox 与 phrase 构造
    #[must_use]
    pub fn new(
        bbox: BoundingBox2D,
        phrase: impl Into<String>,
        camera_frame: crate::frames::NavFrameId,
    ) -> Self {
        Self {
            bbox,
            phrase: phrase.into(),
            image_stamp: Timestamp::default(),
            camera_frame,
            camera_position: None,
        }
    }
}

/// frontier goal(探索板块到导航板块的**唯一** frontier 协议)
///
/// 导航 crate 可以排序、排队、取消、执行这些目标,但**不持有 wavefront 算法内部结构**
/// (cell cluster / mask / open set / 图搜索队列等)。如探索板块需要调试这些内容,
/// 应放在探索 diagnostics topic,不进入导航 API。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrontierGoal {
    /// goal id
    pub id: String,
    /// 目标位姿(pose + frame + stamp)
    pub pose: PoseStamped,
    /// frontier score(由探索板块打分,无单位约定)
    pub score: f32,
    /// 信息增益(平方米)
    pub information_gain: Option<f32>,
    /// 距离代价(米)
    pub distance_cost: Option<f32>,
    /// frontier 覆盖面积(平方米)
    pub frontier_area_m2: Option<f32>,
    /// producer 字符串(标识 frontier 来源 crate / 算法版本)
    pub producer: String,
    /// frontier 产生时间戳
    pub stamp: Timestamp,
    /// 任意元数据(键值对;探索侧调试用)
    pub metadata: BTreeMap<String, String>,
}

impl FrontierGoal {
    /// 由 pose + score 构造最简 frontier goal
    #[must_use]
    pub fn new(id: impl Into<String>, pose: PoseStamped, score: f32) -> Self {
        Self {
            id: id.into(),
            pose,
            score,
            information_gain: None,
            distance_cost: None,
            frontier_area_m2: None,
            producer: String::from("unknown"),
            stamp: Timestamp::default(),
            metadata: BTreeMap::new(),
        }
    }

    /// 校验 frontier 字段合理性(NaN 检查 + 基本非空)
    ///
    /// 仅做基础 sanity;不替代探索板块的内部一致性检查。
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self.score.is_finite() {
            return Err("frontier score is not finite");
        }
        if self.id.is_empty() {
            return Err("frontier id is empty");
        }
        if let Some(v) = self.information_gain {
            if !v.is_finite() {
                return Err("information_gain is not finite");
            }
        }
        if let Some(v) = self.distance_cost {
            if !v.is_finite() {
                return Err("distance_cost is not finite");
            }
        }
        if let Some(v) = self.frontier_area_m2 {
            if !v.is_finite() {
                return Err("frontier_area_m2 is not finite");
            }
        }
        Ok(())
    }

    /// 构造带 identity 朝向的 PoseStamped(常用 helper)
    #[must_use]
    pub fn pose_with_identity(frame: crate::frames::NavFrameId, x: f64, y: f64) -> PoseStamped {
        PoseStamped {
            frame_id: frame,
            stamp: Timestamp::default(),
            pose: crate::geometry::Pose3 {
                position: crate::geometry::Point3 { x, y, z: 0.0 },
                orientation: Quaternion::IDENTITY,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::NavFrameId;

    #[test]
    fn bbox_from_xywh_converts_to_xyxy() {
        let b = BoundingBox2D::from_xywh(10.0, 20.0, 30.0, 40.0).unwrap();
        assert_eq!(b.x1, 10.0);
        assert_eq!(b.y1, 20.0);
        assert_eq!(b.x2, 40.0);
        assert_eq!(b.y2, 60.0);
        assert!((b.area() - 30.0 * 40.0).abs() < 1e-6);
    }

    #[test]
    fn bbox_from_xyxy_swaps_if_inverted() {
        let b = BoundingBox2D::from_xyxy(40.0, 60.0, 10.0, 20.0).unwrap();
        assert_eq!(b.x1, 10.0);
        assert_eq!(b.x2, 40.0);
    }

    #[test]
    fn bbox_rejects_nan_and_negative_size() {
        assert!(BoundingBox2D::from_xywh(0.0, 0.0, -1.0, 1.0).is_none());
        assert!(BoundingBox2D::from_xywh(0.0, 0.0, f32::NAN, 1.0).is_none());
    }

    #[test]
    fn bbox_center_is_average() {
        let b = BoundingBox2D::from_xyxy(0.0, 0.0, 100.0, 80.0).unwrap();
        let (cx, cy) = b.center();
        assert!((cx - 50.0).abs() < 1e-6);
        assert!((cy - 40.0).abs() < 1e-6);
    }

    #[test]
    fn visual_target_new_has_default_stamp() {
        let b = BoundingBox2D::from_xyxy(0.0, 0.0, 10.0, 10.0).unwrap();
        let v = VisualTarget::new(b, "cup", NavFrameId::Camera);
        assert_eq!(v.phrase, "cup");
        assert!(v.camera_position.is_none());
    }

    #[test]
    fn frontier_goal_validate_rejects_bad_score() {
        let pose = FrontierGoal::pose_with_identity(NavFrameId::Map, 0.0, 0.0);
        let mut g = FrontierGoal::new("f-1", pose, f32::NAN);
        assert!(g.validate().is_err());
        g.score = 0.5;
        g.id = String::new();
        assert!(g.validate().is_err());
        g.id = "f-1".to_string();
        assert!(g.validate().is_ok());
    }

    #[test]
    fn frontier_goal_metadata_serde_roundtrip() {
        let pose = FrontierGoal::pose_with_identity(NavFrameId::Map, 0.0, 0.0);
        let mut g = FrontierGoal::new("f-1", pose, 0.7);
        g.metadata.insert("source".into(), "wavefront_v1".into());
        g.metadata.insert("cell_count".into(), "42".into());
        let s = serde_json::to_string(&g).unwrap();
        let back: FrontierGoal = serde_json::from_str(&s).unwrap();
        assert_eq!(
            back.metadata.get("source").map(String::as_str),
            Some("wavefront_v1")
        );
    }

    #[test]
    fn bbox_serde_roundtrip() {
        let b = BoundingBox2D::from_xyxy(1.0, 2.0, 3.0, 4.0).unwrap();
        let s = serde_json::to_string(&b).unwrap();
        let back: BoundingBox2D = serde_json::from_str(&s).unwrap();
        assert_eq!(back, b);
    }
}

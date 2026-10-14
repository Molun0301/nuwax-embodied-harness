//! TARE waypoint typed schema
//!
//! TARE 原生进程通过 `way_point` topic 发布下一可达 waypoint;本 crate **不订阅/不解码**
//! native payload,仅定义/导出 typed schema,供 stack/protocol adapter ingestion 之后构造
//! `FrontierGoal` (转换唯一归属 `nuwax-exploration::TareWaypointGoalSourceAdapter`)。
//!
//! 完成边界:serde roundtrip、clone/debug、`validate()` 成功/失败、手工构造字段语义。

use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::Timestamp;
use serde::{Deserialize, Serialize};

/// TARE bridge typed waypoint schema(对应 native `way_point` topic payload 的 typed 抽象)
///
/// `TareWaypoint -> FrontierGoal` 唯一归属 `nuwax-exploration::TareWaypointGoalSourceAdapter`,
/// 本 crate 仅定义 schema。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TareWaypoint {
    /// 坐标系(frame_id;默认 `Map`)
    pub frame_id: NavFrameId,
    /// 时间戳
    pub stamp: Timestamp,
    /// X (meters, world frame)
    pub x: f64,
    /// Y (meters, world frame)
    pub y: f64,
    /// Z (meters, world frame;通常为 0,保留以支持 3D TARE)
    pub z: f64,
}

impl TareWaypoint {
    /// 构造 waypoint
    #[must_use]
    pub fn new(frame_id: NavFrameId, stamp: Timestamp, x: f64, y: f64, z: f64) -> Self {
        Self {
            frame_id,
            stamp,
            x,
            y,
            z,
        }
    }

    /// 字段合法性校验
    ///
    /// - 坐标必须 finite(非 NaN / 非 Inf)
    /// - 坐标范围按现实世界米级尺度粗略限制:±1e6 m(防止 typed adapter ingestion 后构造
    ///   `FrontierGoal` 时被异常值污染)
    ///
    /// # Errors
    /// 字段非法时返回 `TareWaypointError`。
    pub fn validate(&self) -> Result<(), TareWaypointError> {
        for (name, value) in [("x", self.x), ("y", self.y), ("z", self.z)] {
            if !value.is_finite() {
                return Err(TareWaypointError::NonFiniteCoordinate { field: name, value });
            }
            if value.abs() > MAX_COORDINATE_M {
                return Err(TareWaypointError::CoordinateOutOfRange {
                    field: name,
                    value,
                    limit_m: MAX_COORDINATE_M,
                });
            }
        }
        Ok(())
    }
}

/// `TareWaypoint` 字段校验错误
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum TareWaypointError {
    /// 坐标非 finite(NaN / Inf)
    #[error("TareWaypoint coordinate `{field}` is not finite (got {value})")]
    NonFiniteCoordinate {
        /// 字段名(`x` / `y` / `z`)
        field: &'static str,
        /// 实际值
        value: f64,
    },
    /// 坐标超过米级尺度上限
    #[error("TareWaypoint coordinate `{field}` = {value} exceeds ±{limit_m} m limit")]
    CoordinateOutOfRange {
        /// 字段名
        field: &'static str,
        /// 实际值
        value: f64,
        /// 限制(米)
        limit_m: f64,
    },
}

/// 现实世界米级尺度上限,防止 typed adapter ingestion 后构造 `FrontierGoal` 时被异常值污染
const MAX_COORDINATE_M: f64 = 1.0e6;

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> TareWaypoint {
        TareWaypoint::new(
            NavFrameId::Map,
            Timestamp::from_unix(1_700_000_000, 0),
            1.0,
            2.0,
            0.0,
        )
    }

    #[test]
    fn sample_validates_ok() {
        let wp = sample();
        assert!(wp.validate().is_ok());
    }

    #[test]
    fn reject_nan_x() {
        let mut wp = sample();
        wp.x = f64::NAN;
        let err = wp.validate().unwrap_err();
        assert!(matches!(
            err,
            TareWaypointError::NonFiniteCoordinate { field: "x", .. }
        ));
    }

    #[test]
    fn reject_inf_y() {
        let mut wp = sample();
        wp.y = f64::INFINITY;
        let err = wp.validate().unwrap_err();
        assert!(matches!(
            err,
            TareWaypointError::NonFiniteCoordinate { field: "y", .. }
        ));
    }

    #[test]
    fn reject_neg_inf_z() {
        let mut wp = sample();
        wp.z = f64::NEG_INFINITY;
        let err = wp.validate().unwrap_err();
        assert!(matches!(
            err,
            TareWaypointError::NonFiniteCoordinate { field: "z", .. }
        ));
    }

    #[test]
    fn reject_out_of_range() {
        let mut wp = sample();
        wp.x = MAX_COORDINATE_M * 2.0;
        let err = wp.validate().unwrap_err();
        assert!(matches!(
            err,
            TareWaypointError::CoordinateOutOfRange { field: "x", .. }
        ));
    }

    #[test]
    fn serde_roundtrip() {
        let wp = sample();
        let s = serde_json::to_string(&wp).expect("serialize");
        let back: TareWaypoint = serde_json::from_str(&s).expect("deserialize");
        assert_eq!(back, wp);
    }

    #[test]
    fn clone_preserves_fields() {
        let wp = sample();
        let cp = wp.clone();
        assert_eq!(cp, wp);
        assert_eq!(cp.frame_id, NavFrameId::Map);
        assert_eq!(cp.x, 1.0);
        assert_eq!(cp.y, 2.0);
        assert_eq!(cp.z, 0.0);
    }

    #[test]
    fn debug_includes_field_values() {
        let wp = sample();
        let dbg = format!("{wp:?}");
        assert!(dbg.contains("TareWaypoint"));
        assert!(dbg.contains("1.0") || dbg.contains("1"));
    }
}

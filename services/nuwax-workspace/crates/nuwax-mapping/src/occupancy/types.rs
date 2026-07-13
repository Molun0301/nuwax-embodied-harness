//! 占据栅格基础类型
//!
//! SSOT：`/home/ml/Deeprob/dimos/dimos/msgs/nav_msgs/OccupancyGrid.py`
//!   + lcm `/home/ml/Deeprob/dimos/rust重写与移植/lcmtypes/nav_msgs/OccupancyGrid.lcm`
//!     （若存在）。本模块为 Rust 端结构化表达，必须保持 wire format 兼容。
//!
//! 修复要点（见 docs/.../20_rework20_全仓审计_对dimos原始.md §2.1）：
//! - H-1：`OccupancyValue::from_i8` 对齐 Python `IntEnum {UNKNOWN=-1, FREE=0, OCCUPIED=100}`，
//!   删掉 `v >= 65` 阈值（Python 中不存在）。
//! - H-1：删掉 `CostValues` bitflags（dimos wire format 仅三态）。
//! - H-2：补齐 `MapMetaData`（resolution/width/height/origin）+ `frame_id` + `ts` + `Pose` origin。

use nuwax_types::Timestamp;
use serde::{Deserialize, Serialize};

/// 3D 位姿（map → world），对应 Python `dimos.msgs.geometry_msgs.Pose.Pose`
///
/// 简化版：只承载 position(x,y,z) + orientation(x,y,z,w) 四元数。
/// 不引入 nuwax-tf 依赖以避免 cyclic。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    /// 位置
    pub position: [f64; 3],
    /// 四元数 (qx, qy, qz, qw)，XYZW 顺序与 Python Quaternion 一致
    pub orientation: [f64; 4],
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

/// 单元值语义（Python `IntEnum UNKNOWN=-1, FREE=0, OCCUPIED=100`）
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[repr(i8)]
pub enum OccupancyValue {
    /// 未知（ROS 原生用 -1）
    #[default]
    Unknown = -1,
    /// 空闲
    Free = 0,
    /// 占据
    Occupied = 100,
}

/// 单元（保留原始 int8 + 可选概率，rework22 P1 #1）
///
/// 用途：当 OccupancyGrid.data 中存在 1..99 概率值（dimos Python `_preprocess`
/// `ProbabilityCost` / `ProbabilisticOccupancy` 算法产物），`OccupancyValue::from_i8`
/// 只能识别三态（0/100/-1），中间概率全部映射为 Unknown，丢失信息。
///
/// `OccupancyCell` 同时保留：
/// - `value: i8` —— 原始 int8（与 wire format 兼容，1..99 是概率，100=致死）
/// - `prob: Option<f32>` —— 可选显式概率（0.0~1.0），上层有需要时设置
///
/// 兼容策略：
/// - `OccupancyValue` enum 不变（API 兼容）
/// - `OccupancyCell::to_value()` 按">=100 → Occupied, ==0 → Free, 其他 → Unknown"
///   转 enum（保留 from_i8 语义）
/// - `OccupancyCell::from_value()` 反向
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OccupancyCell {
    /// 原始 int8（-1=Unknown, 0=Free, 1..99=概率, 100=致死）
    pub value: i8,
    /// 可选概率（0.0~1.0）。None 表示未设置概率值
    pub prob: Option<f32>,
}

impl OccupancyCell {
    /// 从 i8 构造（无显式概率）
    #[must_use]
    pub fn from_i8(v: i8) -> Self {
        Self {
            value: v,
            prob: None,
        }
    }

    /// 从 i8 + 概率构造
    #[must_use]
    pub fn from_i8_with_prob(v: i8, prob: f32) -> Self {
        Self {
            value: v,
            prob: Some(prob.clamp(0.0, 1.0)),
        }
    }

    /// 转 OccupancyValue enum（三态语义）
    #[must_use]
    pub fn to_value(&self) -> OccupancyValue {
        match self.value {
            0 => OccupancyValue::Free,
            100 => OccupancyValue::Occupied,
            _ => OccupancyValue::Unknown,
        }
    }

    /// 从 OccupancyValue 构造（概率 = None）
    #[must_use]
    pub fn from_value(v: OccupancyValue) -> Self {
        Self {
            value: v as i8,
            prob: None,
        }
    }

    /// 是否为致死占据
    #[must_use]
    pub fn is_lethal(&self) -> bool {
        self.value >= 100
    }

    /// 是否存在概率值（1..99 或 prob 显式设置）
    #[must_use]
    pub fn has_probability(&self) -> bool {
        self.prob.is_some() || (1..100).contains(&self.value)
    }

    /// 取概率值：若 prob 显式设置则用之，否则从 value 推断（100 → 1.0, 0 → 0.0）
    #[must_use]
    pub fn probability(&self) -> f32 {
        if let Some(p) = self.prob {
            return p;
        }
        match self.value {
            v if v >= 100 => 1.0,
            0 => 0.0,
            v if v > 0 => f32::from(v) / 100.0,
            _ => -1.0, // Unknown: 特殊标记（caller 应解读为"未观测"）
        }
    }
}

impl Default for OccupancyCell {
    fn default() -> Self {
        Self {
            value: -1,
            prob: None,
        }
    }
}

impl OccupancyValue {
    /// 从原始 i8 转换（Python OccupancyGrid.lcm_encode 直接 `flatten().tolist()`）
    ///
    /// Python 实际编码：值 ∈ {-1, 0, 1..100}。其中 0=Free，100=Occupied=致死，
    /// 1..99=占据概率值，-1=Unknown。其他值在协议中**不存在**。
    /// 这里按 Python IntEnum 三态处理：精确命中 100 → Occupied，精确命中 0 → Free，
    /// 其余（含 -1、1..99）按 Unknown 处理（Python 端 OccupancyGrid 也只识别三态）。
    #[must_use]
    pub fn from_i8(v: i8) -> Self {
        match v {
            0 => Self::Free,
            100 => Self::Occupied,
            _ => Self::Unknown,
        }
    }
}

/// 地图元数据（Python `MapMetaData` 字段集）
///
/// 对应 Python `dimos_lcm.nav_msgs.MapMetaData`：
/// `map_load_time, resolution, width, height, origin`
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MapMetaData {
    /// 加载时间
    pub map_load_time: Timestamp,
    /// 分辨率（m/cell），Python 默认 0.05
    pub resolution: f64,
    /// 宽度（cells）
    pub width: u32,
    /// 高度（cells）
    pub height: u32,
    /// 原点位姿（map → world）
    pub origin: Pose,
}

impl MapMetaData {
    /// 构造（默认加载时间 = now）
    #[must_use]
    pub fn new(resolution: f64, width: u32, height: u32, origin: Pose) -> Self {
        Self {
            map_load_time: Timestamp::now(),
            resolution,
            width,
            height,
            origin,
        }
    }
}

/// 占据栅格（Python `OccupancyGrid`）
///
/// wire format 字段集（Python `lcm_encode` 顺序）：
/// `header.stamp.sec/nsec, header.frame_id, info, data_length, data`
///
/// 字段命名沿用 nuwax-mapping 旧约定（`resolution` / `origin` 直接 public 字段），
/// 以便 `operations.rs` / `extrude_occupancy.rs` / `path_mask.rs` 等既有消费者无
/// 需同步改方法调用。H-2 修复增字段（`info` / `frame_id` / `ts`）以对齐 Python
/// wire format。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OccupancyGrid {
    /// 单元值（按行优先，对应 Python `grid: NDArray[np.int8]` flatten）
    pub data: Vec<i8>,
    /// 宽度（列数）
    pub width: u32,
    /// 高度（行数）
    pub height: u32,
    /// 分辨率（m/cell）—— 历史 public 字段，与 `info.resolution` 等价
    pub resolution: f64,
    /// 原点平移分量（map → world，XYZ）—— 历史 public 字段
    pub origin: [f64; 3],
    /// 原点朝向（XYZW 四元数）—— H-2 增量
    pub origin_orientation: [f64; 4],
    /// 坐标系名（Python `frame_id`，默认 "world"）—— H-2 增量
    pub frame_id: String,
    /// 时间戳（秒，Python `ts`）—— H-2 增量
    pub ts: f64,
}

impl OccupancyGrid {
    /// 构造新栅格（Python 默认 resolution=0.05, frame_id="world"）
    #[must_use]
    pub fn new(width: u32, height: u32, resolution: f64) -> Self {
        Self {
            data: vec![OccupancyValue::Unknown as i8; (width * height) as usize],
            width,
            height,
            resolution,
            origin: [0.0, 0.0, 0.0],
            origin_orientation: [0.0, 0.0, 0.0, 1.0],
            frame_id: "world".to_string(),
            ts: 0.0,
        }
    }

    /// 完整构造（含 frame_id）
    #[must_use]
    pub fn with_frame(
        width: u32,
        height: u32,
        resolution: f64,
        frame_id: impl Into<String>,
    ) -> Self {
        let mut g = Self::new(width, height, resolution);
        g.frame_id = frame_id.into();
        g
    }

    /// 从 Python `MapMetaData` 派生
    #[must_use]
    pub fn from_meta(info: &MapMetaData, frame_id: impl Into<String>, ts: f64) -> Self {
        Self {
            data: vec![OccupancyValue::Unknown as i8; (info.width * info.height) as usize],
            width: info.width,
            height: info.height,
            resolution: info.resolution,
            origin: info.origin.position,
            origin_orientation: info.origin.orientation,
            frame_id: frame_id.into(),
            ts,
        }
    }

    /// 索引：(row, col) → data index
    #[must_use]
    pub fn index(&self, row: u32, col: u32) -> usize {
        (row * self.width + col) as usize
    }

    /// 读取某格
    /// Returns the cell value, or `None` if out of bounds.
    #[must_use]
    pub fn get(&self, row: u32, col: u32) -> Option<OccupancyValue> {
        if row >= self.height || col >= self.width {
            return None;
        }
        Some(OccupancyValue::from_i8(self.data[self.index(row, col)]))
    }

    /// 写入某格
    /// Returns `true` on success.
    pub fn set(&mut self, row: u32, col: u32, value: OccupancyValue) -> bool {
        if row >= self.height || col >= self.width {
            return false;
        }
        let idx = self.index(row, col);
        self.data[idx] = value as i8;
        true
    }

    /// 提取 MapMetaData（用于 cost_mapper / wire encode）
    #[must_use]
    pub fn to_meta(&self) -> MapMetaData {
        MapMetaData {
            map_load_time: nuwax_types::Timestamp {
                sec: self.ts as i64,
                nsec: 0,
            },
            resolution: self.resolution,
            width: self.width,
            height: self.height,
            origin: Pose {
                position: self.origin,
                orientation: self.origin_orientation,
            },
        }
    }

    /// world → grid（Python `world_to_grid`，仅 XY，简化无旋转）
    #[must_use]
    pub fn world_to_grid(&self, x: f64, y: f64) -> (f64, f64) {
        let ox = self.origin[0];
        let oy = self.origin[1];
        let r = self.resolution;
        ((x - ox) / r, (y - oy) / r)
    }

    /// grid → world（Python `grid_to_world`，仅 XY，简化无旋转）
    #[must_use]
    pub fn grid_to_world(&self, gx: f64, gy: f64) -> (f64, f64) {
        let ox = self.origin[0];
        let oy = self.origin[1];
        let r = self.resolution;
        (ox + gx * r, oy + gy * r)
    }

    /// 是否所有 cell 都为 Free（unknown 不计）
    #[must_use]
    pub fn is_all_free(&self) -> bool {
        self.data.iter().all(|&v| v <= 0)
    }

    /// 占据率（Python `occupied_percent / 100`，0.0 ~ 1.0）
    #[must_use]
    pub fn occupied_ratio(&self) -> f64 {
        if self.data.is_empty() {
            return 0.0;
        }
        // Python `occupied_cells` = grid[grid >= 1]（包含 1..99 概率值与 100 致死值）
        let occ = self.data.iter().filter(|&&v| v >= 1).count();
        occ as f64 / self.data.len() as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn occupancy_value_from_i8_python_contract() {
        // Python IntEnum 严格三态
        assert_eq!(OccupancyValue::from_i8(-1), OccupancyValue::Unknown);
        assert_eq!(OccupancyValue::from_i8(0), OccupancyValue::Free);
        assert_eq!(OccupancyValue::from_i8(50), OccupancyValue::Unknown); // 1..99 在 IntEnum 中无对应
        assert_eq!(OccupancyValue::from_i8(100), OccupancyValue::Occupied);
        // 越界 / 负值 = Unknown（不再有 v>=65 阈值）
        assert_eq!(OccupancyValue::from_i8(65), OccupancyValue::Unknown);
        assert_eq!(OccupancyValue::from_i8(127), OccupancyValue::Unknown);
        assert_eq!(OccupancyValue::from_i8(-128), OccupancyValue::Unknown);
    }

    #[test]
    fn grid_new_all_unknown() {
        let g = OccupancyGrid::new(10, 20, 0.05);
        assert_eq!(g.width, 10);
        assert_eq!(g.height, 20);
        assert!((g.resolution - 0.05).abs() < f64::EPSILON);
        assert_eq!(g.frame_id, "world");
        assert!((g.origin[2]).abs() < f64::EPSILON);
        for r in 0..20 {
            for c in 0..10 {
                assert_eq!(g.get(r, c), Some(OccupancyValue::Unknown));
            }
        }
    }

    #[test]
    fn grid_set_get_roundtrip() {
        let mut g = OccupancyGrid::new(4, 4, 0.1);
        assert!(g.set(1, 2, OccupancyValue::Occupied));
        assert_eq!(g.get(1, 2), Some(OccupancyValue::Occupied));
        assert_eq!(g.get(0, 0), Some(OccupancyValue::Unknown));
    }

    #[test]
    fn grid_out_of_bounds() {
        let g = OccupancyGrid::new(4, 4, 0.1);
        assert_eq!(g.get(10, 0), None);
        assert_eq!(g.get(0, 10), None);
    }

    #[test]
    fn grid_occupied_ratio_python_semantics() {
        let mut g = OccupancyGrid::new(2, 2, 0.1);
        g.set(0, 0, OccupancyValue::Occupied);
        g.set(0, 1, OccupancyValue::Occupied);
        // 2/4 = 0.5（按 Python `grid >= 1` 语义）
        assert!((g.occupied_ratio() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn grid_world_to_grid_roundtrip() {
        let g = OccupancyGrid::new(10, 10, 0.05);
        let (gx, gy) = g.world_to_grid(1.0, 2.0);
        let (wx, wy) = g.grid_to_world(gx, gy);
        assert!((wx - 1.0).abs() < 1e-9);
        assert!((wy - 2.0).abs() < 1e-9);
    }

    #[test]
    fn map_metadata_default_resolution_matches_python() {
        let m = MapMetaData::new(0.05, 100, 100, Pose::default());
        assert_eq!(m.resolution, 0.05);
        assert_eq!(m.width, 100);
        assert_eq!(m.height, 100);
    }

    #[test]
    fn pose_default_identity() {
        let p = Pose::default();
        assert_eq!(p.position, [0.0, 0.0, 0.0]);
        assert_eq!(p.orientation, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn frame_id_override() {
        let g = OccupancyGrid::with_frame(4, 4, 0.05, "map");
        assert_eq!(g.frame_id, "map");
    }
}

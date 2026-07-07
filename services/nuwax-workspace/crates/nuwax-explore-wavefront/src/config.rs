//! `WavefrontConfig`:所有可调参数(默认值与方案 §4.5 严格一致)。

use serde::{Deserialize, Serialize};

/// Wavefront 探索默认配置(方案 §4.5 默认值,首版必须保留以保证行为兼容)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WavefrontConfig {
    /// frontier 最小周长(米)
    pub min_frontier_perimeter_m: f64,
    /// 邻居占用阈值(超过该值视为 occupied,strict `>` 用于 frontier 排除)
    pub occupancy_threshold: i16,
    /// 安全距离(米)
    pub safe_distance_m: f64,
    /// obstacle inflation 半径(米,runtime loop 使用,不在 scorer 中)
    pub obstacle_inflation_radius_m: f64,
    /// 期望 frontier 距离(米)
    pub lookahead_distance_m: f64,
    /// 最大 explored goal 距离(米)
    pub max_explored_distance_m: f64,
    /// 信息增益判定比例阈值
    pub info_gain_threshold_ratio: f64,
    /// 触发 no-gain stop 的连续低增益次数
    pub num_no_gain_attempts: u32,
    /// 单 goal 超时(毫秒)
    pub goal_timeout_ms: u64,
    /// 最大连续失败次数
    pub max_consecutive_failures: u32,
    /// 启用 no-gain 判定前需发布的最小 goal 数(方案 §4.10,替代历史硬编码 5)
    pub min_goals_before_no_gain_check: u32,
}

impl Default for WavefrontConfig {
    fn default() -> Self {
        Self {
            min_frontier_perimeter_m: 0.5,
            occupancy_threshold: 99,
            safe_distance_m: 3.0,
            obstacle_inflation_radius_m: 0.25,
            lookahead_distance_m: 5.0,
            max_explored_distance_m: 10.0,
            info_gain_threshold_ratio: 0.03,
            num_no_gain_attempts: 2,
            goal_timeout_ms: 15_000,
            max_consecutive_failures: 10,
            min_goals_before_no_gain_check: 5,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_scheme_4_5() {
        let c = WavefrontConfig::default();
        assert!((c.min_frontier_perimeter_m - 0.5).abs() < 1e-9);
        assert_eq!(c.occupancy_threshold, 99);
        assert!((c.safe_distance_m - 3.0).abs() < 1e-9);
        assert!((c.obstacle_inflation_radius_m - 0.25).abs() < 1e-9);
        assert!((c.lookahead_distance_m - 5.0).abs() < 1e-9);
        assert!((c.max_explored_distance_m - 10.0).abs() < 1e-9);
        assert!((c.info_gain_threshold_ratio - 0.03).abs() < 1e-9);
        assert_eq!(c.num_no_gain_attempts, 2);
        assert_eq!(c.goal_timeout_ms, 15_000);
        assert_eq!(c.max_consecutive_failures, 10);
        assert_eq!(c.min_goals_before_no_gain_check, 5);
    }

    #[test]
    fn serde_roundtrip_preserves_fields() {
        let c = WavefrontConfig::default();
        let s = serde_json::to_string(&c).unwrap();
        let back: WavefrontConfig = serde_json::from_str(&s).unwrap();
        assert_eq!(back, c);
    }
}

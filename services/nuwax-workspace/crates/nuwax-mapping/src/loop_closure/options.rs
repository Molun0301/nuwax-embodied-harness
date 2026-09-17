//! 库内回环的启动参数覆盖；保存端与这里共享字段及范围，不改变运行中的图。

use super::coordinator::{LoopClosureCoordinator, LoopClosurePipelineConfig};
use serde::de::{MapAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::Value;
use std::{collections::BTreeMap, fmt, sync::OnceLock};

/// 由共同配置文件加载器导出的库内PGO参数。
pub const PGO_OPTIONS_ENV: &str = "NUWAX_GEOMETRY_PGO_OPTIONS";
const SCHEMA: &str = include_str!("../../../../../../contracts/pgo_options.json");

/// 仅包含显式覆盖项；缺少的字段保留调用方选择的前端默认配置。
#[derive(Debug, Default)]
pub struct PgoOptions(BTreeMap<String, Value>);

impl<'de> Deserialize<'de> for PgoOptions {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct OptionsVisitor;
        impl<'de> Visitor<'de> for OptionsVisitor {
            type Value = PgoOptions;
            fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
                f.write_str("无重复键的PGO参数对象")
            }
            fn visit_map<M: MapAccess<'de>>(self, mut map: M) -> Result<Self::Value, M::Error> {
                let mut values = BTreeMap::new();
                while let Some((name, value)) = map.next_entry::<String, Value>()? {
                    if values.insert(name.clone(), value).is_some() {
                        return Err(serde::de::Error::custom(format!("PGO参数重复：{name}")));
                    }
                }
                Ok(PgoOptions(values))
            }
        }
        deserializer.deserialize_map(OptionsVisitor)
    }
}

impl PgoOptions {
    /// 解析启动时的参数对象；未知键、重复键、非有限值与类型退化均拒绝。
    pub fn from_json(raw: &str) -> Result<Self, String> {
        if raw.trim().is_empty() {
            return Ok(Self::default());
        }
        let options: Self = serde_json::from_str(raw)
            .map_err(|error| format!("PGO参数无效：{error}"))?;
        static PARSED: OnceLock<Value> = OnceLock::new();
        let schema = PARSED.get_or_init(|| serde_json::from_str(SCHEMA).expect("内置PGO定义合法"));
        for (name, value) in &options.0 {
            let spec = schema["parameters"].get(name)
                .ok_or_else(|| format!("未知PGO参数：{name}"))?;
            let valid = value.as_f64().is_some_and(|n| {
                n.is_finite()
                    && (spec["type"] != "int" || value.as_u64().is_some())
                    && spec["minimum"].as_f64().is_none_or(|min| n >= min)
                    && spec["exclusive_minimum"].as_f64().is_none_or(|min| n > min)
                    && spec["maximum"].as_f64().is_none_or(|max| n <= max)
            });
            if !valid {
                return Err(format!("PGO参数{name}的类型或范围无效：{}", spec["description"]));
            }
        }
        Ok(options)
    }

    /// 在已选好的Point或通用默认值上应用覆盖，并校验完整生产配置。
    pub fn apply_to(&self, mut config: LoopClosurePipelineConfig) -> Result<LoopClosurePipelineConfig, String> {
        for (name, value) in &self.0 {
            macro_rules! number { ($field:ident) => { config.$field = value.as_f64().ok_or("参数必须为数值")? }; }
            macro_rules! count { ($field:ident) => { config.$field = value.as_u64().ok_or("参数必须为整数")? as usize }; }
            match name.as_str() {
                "max_correspondence_distance" => config.icp.max_correspondence_distance = value.as_f64().ok_or("对应距离必须为数值")?,
                "icp_max_iterations" => config.icp.max_iterations = value.as_u64().ok_or("配准迭代数必须为整数")? as usize,
                "optimizer_max_iterations" => config.optimizer.max_iterations = value.as_u64().ok_or("优化迭代数必须为整数")? as usize,
                "max_loop_mse" => number!(max_loop_mse),
                "min_loop_overlap_ratio" => number!(min_loop_overlap_ratio),
                "loop_confirmation_frames" => count!(loop_confirmation_frames),
                "loop_confirmation_interval_secs" => number!(loop_confirmation_interval_secs),
                "min_loop_separation_secs" => number!(min_loop_separation_secs),
                "min_loop_interval_secs" => number!(min_loop_interval_secs),
                "stationary_probe_interval_secs" => number!(stationary_probe_interval_secs),
                "keyframe_translation" => number!(keyframe_translation),
                "keyframe_rotation" => number!(keyframe_rotation),
                "loop_search_radius" => number!(loop_search_radius),
                "min_keyframe_points" => count!(min_keyframe_points),
                "submap_resolution_m" => number!(submap_resolution_m),
                "loop_submap_half_range" => count!(loop_submap_half_range),
                "min_keyframes_for_loop_search" => count!(min_keyframes_for_loop_search),
                _ => return Err(format!("未接通PGO参数：{name}")),
            }
        }
        LoopClosureCoordinator::new(config).map_err(|error| error.to_string())?;
        Ok(config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partial_override_preserves_selected_frontend_defaults_and_empty_resets() {
        let options = PgoOptions::from_json(r#"{"max_correspondence_distance":0.25,"icp_max_iterations":20,"loop_confirmation_frames":4}"#).unwrap();
        for base in [LoopClosurePipelineConfig::default(), LoopClosurePipelineConfig::for_point_lio()] {
            let changed = options.apply_to(base).unwrap();
            assert_eq!(changed.icp.max_correspondence_distance, 0.25);
            assert_eq!(changed.icp.max_iterations, 20);
            assert_eq!(changed.loop_confirmation_frames, 4);
            assert_eq!(changed.max_loop_mse, base.max_loop_mse);
            for empty in ["", " ", "{}"] {
                let reset = PgoOptions::from_json(empty).unwrap().apply_to(base).unwrap();
                assert_eq!(reset.icp.max_correspondence_distance, base.icp.max_correspondence_distance);
            }
        }
    }

    #[test]
    fn invalid_override_never_silently_becomes_a_default() {
        for invalid in ["[]", "null", r#"{"unknown":1}"#, r#"{"max_loop_mse":null}"#,
            r#"{"max_loop_mse":true}"#, r#"{"max_loop_mse":"0.01"}"#, r#"{"max_loop_mse":NaN}"#,
            r#"{"max_loop_mse":1e999}"#, r#"{"max_loop_mse":0.1,"max_loop_mse":0.2}"#,
            r#"{"loop_confirmation_frames":2.5}"#, r#"{"loop_confirmation_frames":9}"#,
            r#"{"loop_confirmation_interval_secs":11}"#, r#"{"max_correspondence_distance":0}"#,
            r#"{"min_loop_overlap_ratio":1.1}"#, r#"{"icp_max_iterations":301}"#] {
            assert!(PgoOptions::from_json(invalid).is_err(), "{invalid}");
        }
    }

    #[test]
    fn every_advertised_parameter_has_a_production_assignment() {
        let schema: Value = serde_json::from_str(SCHEMA).unwrap();
        for (name, spec) in schema["parameters"].as_object().unwrap() {
            let value = spec.get("maximum").or_else(|| spec.get("minimum")).cloned()
                .unwrap_or_else(|| Value::from(0.1));
            let raw = serde_json::json!({name: value}).to_string();
            PgoOptions::from_json(&raw).unwrap().apply_to(LoopClosurePipelineConfig::for_point_lio()).unwrap();
        }
    }
}

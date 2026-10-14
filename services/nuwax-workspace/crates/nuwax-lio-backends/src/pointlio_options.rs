//! Point参数覆盖：与设置保存共用定义，禁止未知键及类型退化。

use serde::de::{MapAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::fmt;
use std::sync::OnceLock;

pub const POINTLIO_OPTIONS_ENV: &str = "NUWAX_GEOMETRY_POINTLIO_OPTIONS";
const SCHEMA: &str = include_str!("../../../../../contracts/pointlio_options.json");

#[derive(Debug, Clone, Default, PartialEq, Serialize)]
#[serde(transparent)]
pub struct PointLioOptions(pub BTreeMap<String, Value>);

impl<'de> Deserialize<'de> for PointLioOptions {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct OptionsVisitor;
        impl<'de> Visitor<'de> for OptionsVisitor {
            type Value = PointLioOptions;
            fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
                f.write_str("无重复键的Point参数对象")
            }
            fn visit_map<M: MapAccess<'de>>(self, mut map: M) -> Result<Self::Value, M::Error> {
                let mut values = BTreeMap::new();
                while let Some((key, value)) = map.next_entry::<String, Value>()? {
                    if values.insert(key.clone(), value).is_some() {
                        return Err(serde::de::Error::custom(format!("Point参数重复：{key}")));
                    }
                }
                let result = PointLioOptions(values);
                result.validate().map_err(serde::de::Error::custom)?;
                Ok(result)
            }
        }
        deserializer.deserialize_map(OptionsVisitor)
    }
}

impl PointLioOptions {
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn from_json(raw: &str) -> Result<Self, String> {
        if raw.trim().is_empty() {
            return Ok(Self::default());
        }
        serde_json::from_str(raw).map_err(|error| format!("Point参数无效：{error}"))
    }

    pub fn validate(&self) -> Result<(), String> {
        static PARSED: OnceLock<Value> = OnceLock::new();
        let schema =
            PARSED.get_or_init(|| serde_json::from_str(SCHEMA).expect("内置Point定义合法"));
        for (name, value) in &self.0 {
            let spec = schema["parameters"].get(name).ok_or_else(|| {
                let reason = schema["unsupported"][name].as_str().unwrap_or("未知参数");
                format!("Point参数{name}不受支持：{reason}")
            })?;
            let valid = match spec["type"].as_str().unwrap_or("") {
                "bool" => value.is_boolean(),
                "enum" => {
                    value.is_string()
                        && spec["choices"]
                            .as_array()
                            .is_some_and(|a| a.contains(value))
                }
                "vector3" => value.as_array().is_some_and(|a| {
                    a.len() == 3
                        && a.iter().all(|v| {
                            v.as_f64()
                                .is_some_and(|n| n.is_finite() && n.abs() <= f64::from(f32::MAX))
                        })
                        && a.iter()
                            .filter_map(Value::as_f64)
                            .map(|n| n * n)
                            .sum::<f64>()
                            > 1e-12
                }),
                kind @ ("int" | "number") => value.as_f64().is_some_and(|n| {
                    n.is_finite()
                        && (kind != "int" || value.as_i64().is_some())
                        && spec
                            .get("minimum")
                            .and_then(Value::as_f64)
                            .is_none_or(|min| n >= min)
                        && spec
                            .get("exclusive_minimum")
                            .and_then(Value::as_f64)
                            .is_none_or(|min| n > min)
                        && spec
                            .get("maximum")
                            .and_then(Value::as_f64)
                            .is_none_or(|max| n <= max)
                        && (kind != "number"
                            || (n.abs() <= f64::from(f32::MAX)
                                && (n == 0. || n.abs() >= f64::from(f32::MIN_POSITIVE))))
                }),
                _ => false,
            };
            if !valid {
                return Err(format!("Point参数{name}的类型或范围无效"));
            }
        }
        if self
            .0
            .get("use_imu_as_input")
            .and_then(Value::as_bool)
            .unwrap_or(false)
            && !self
                .0
                .get("imu_en")
                .and_then(Value::as_bool)
                .unwrap_or(true)
        {
            return Err("惯性输入模式必须启用惯性".into());
        }
        let number = |name, default| self.0.get(name).and_then(Value::as_f64).unwrap_or(default);
        if number("blind", 0.5) >= number("det_range", 100.) {
            return Err("Point盲区必须小于最大距离".into());
        }
        Ok(())
    }

    pub fn render_args(&self) -> Result<Vec<String>, String> {
        self.validate()?;
        let mut args = Vec::with_capacity(self.0.len() * 2);
        for (name, value) in &self.0 {
            let text = match value {
                Value::String(s) => s.clone(),
                Value::Array(a) => a
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(","),
                _ => value.to_string(),
            };
            args.extend([format!("--{name}"), text]);
        }
        Ok(args)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn point_options_reject_silent_fallback_and_render_exact_values() {
        for invalid in [
            r#"{"det_range":0}"#,
            r#"{"point_filter_num":1.5}"#,
            r#"{"use_imu_as_input":"true"}"#,
            r#"{"ivox_nearby_type":26}"#,
            r#"{"det_range":12,"det_range":30}"#,
            r#"{"timestamp_unit":"second"}"#,
            r#"{"use_imu_as_input":true,"imu_en":false}"#,
            r#"{"unknown":1}"#,
        ] {
            assert!(PointLioOptions::from_json(invalid).is_err(), "{invalid}");
        }
        let options = PointLioOptions::from_json(r#"{"point_filter_num":1,"lidar_meas_cov":0.001,"use_imu_as_input":true,"gravity":[0,0,-9.81]}"#).unwrap();
        let args = options.render_args().unwrap();
        assert!(args.windows(2).any(|x| x == ["--use_imu_as_input", "true"]));
        assert!(args.windows(2).any(|x| x == ["--gravity", "0,0,-9.81"]));
    }
}

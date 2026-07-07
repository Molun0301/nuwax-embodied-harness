//! Spec 类型(`NativeProgramSpec` / `NativeBuildSpec` / `NativeBuildPolicy` / `NativeCliMapping`)

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use nuwax_nav_base::native::RestartPolicy;
use serde::{Deserialize, Serialize};

/// 原生程序 spec
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NativeProgramSpec {
    /// 程序名(用于诊断 / 日志;**不**作为二进制名)
    pub name: String,
    /// 可执行文件路径(由调用方指定,不得硬编码)
    pub executable: PathBuf,
    /// topic args:port 名 → topic 名(渲染为 `--{port} {topic}`)
    pub topics: BTreeMap<String, String>,
    /// 额外 args(原样追加;空字符串禁止)
    pub args: Vec<String>,
    /// 环境变量(继承 + 覆盖)
    pub env: BTreeMap<String, String>,
    /// 工作目录
    pub cwd: Option<PathBuf>,
    /// 重启策略
    pub restart: RestartPolicy,
}

impl NativeProgramSpec {
    /// 校验 spec(空 topic name / 空 arg / executable 不存在 → `InvalidInput`)
    pub fn validate(&self) -> Result<(), String> {
        if self.executable.as_os_str().is_empty() {
            return Err("executable is empty".to_string());
        }
        for (port, topic) in &self.topics {
            if port.is_empty() {
                return Err("topic port name is empty".to_string());
            }
            if port.chars().any(|c| c.is_whitespace()) {
                return Err(format!("topic port name `{port}` contains whitespace"));
            }
            if topic.is_empty() {
                return Err(format!("topic value for port `{port}` is empty"));
            }
        }
        for (i, arg) in self.args.iter().enumerate() {
            if arg.is_empty() {
                return Err(format!("args[{i}] is empty"));
            }
        }
        Ok(())
    }
}

/// 构建策略
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NativeBuildPolicy {
    /// executable 不存在即报错(不构建)
    #[default]
    Never,
    /// 缺失时运行 build_command
    IfMissing,
    /// 每次 start 前运行 build_command
    Always,
}

/// 构建 spec
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct NativeBuildSpec {
    /// 构建命令(shell 字符串)
    pub build_command: Option<String>,
    /// 构建策略
    pub policy: NativeBuildPolicy,
    /// 工作目录(覆盖 spec.cwd)
    pub cwd: Option<PathBuf>,
    /// 额外环境
    pub env: BTreeMap<String, String>,
}

/// CLI 映射规则(用于 bridge 把 typed config 字段映射到 native argv)
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct NativeCliMapping {
    /// 排除的字段名
    pub exclude: BTreeSet<String>,
    /// 字段名 → C++ CLI 参数名覆盖
    pub name_override: BTreeMap<String, String>,
    /// config 是否走 stdin(JSON)
    pub stdin_config: bool,
}

impl NativeCliMapping {
    /// 把单个 `(name, value)` 配置项映射为 CLI 片段(可能空 = 不渲染)
    ///
    /// - `name` 在 `exclude` 中 → 返回空
    /// - `name` 在 `name_override` 中 → 使用 override
    /// - 否则按原 `name`
    /// - `None` value → 空(调用方跳过)
    pub fn render_arg(&self, name: &str, value: Option<ConfigValue>) -> Vec<String> {
        if self.exclude.contains(name) {
            return Vec::new();
        }
        let value = match value {
            Some(v) => v,
            None => return Vec::new(),
        };
        let mapped = self
            .name_override
            .get(name)
            .map(String::as_str)
            .unwrap_or(name);
        let mut out = vec![format!("--{mapped}")];
        match value {
            ConfigValue::Bool(b) => out.push(if b {
                "true".to_string()
            } else {
                "false".to_string()
            }),
            ConfigValue::F64(f) => out.push(format!("{f}")),
            ConfigValue::I64(i) => out.push(format!("{i}")),
            ConfigValue::String(s) => {
                if s.is_empty() {
                    out.pop();
                } else {
                    out.push(s);
                }
            }
            ConfigValue::List(items) => {
                if items.is_empty() {
                    out.pop();
                } else {
                    out.push(items.join(","));
                }
            }
        }
        out
    }
}

/// 配置值类型(供 `NativeCliMapping::render_arg` 使用)
#[derive(Debug, Clone, PartialEq)]
pub enum ConfigValue {
    /// 布尔
    Bool(bool),
    /// 浮点
    F64(f64),
    /// 整数
    I64(i64),
    /// 字符串(空字符串视同 None,不渲染)
    String(String),
    /// 列表(逗号拼接;空列表不渲染)
    List(Vec<String>),
}

impl From<bool> for ConfigValue {
    fn from(v: bool) -> Self {
        Self::Bool(v)
    }
}

impl From<f64> for ConfigValue {
    fn from(v: f64) -> Self {
        Self::F64(v)
    }
}

impl From<i64> for ConfigValue {
    fn from(v: i64) -> Self {
        Self::I64(v)
    }
}

impl From<String> for ConfigValue {
    fn from(v: String) -> Self {
        Self::String(v)
    }
}

impl From<&str> for ConfigValue {
    fn from(v: &str) -> Self {
        Self::String(v.to_string())
    }
}

impl From<Vec<String>> for ConfigValue {
    fn from(v: Vec<String>) -> Self {
        Self::List(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spec_validate_rejects_empty_executable() {
        let s = NativeProgramSpec {
            name: "x".into(),
            executable: PathBuf::new(),
            topics: BTreeMap::new(),
            args: vec![],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        };
        assert!(s.validate().is_err());
    }

    #[test]
    fn spec_validate_rejects_empty_topic() {
        let mut topics = BTreeMap::new();
        topics.insert("scan".into(), String::new());
        let s = NativeProgramSpec {
            name: "x".into(),
            executable: PathBuf::from("/bin/echo"),
            topics,
            args: vec![],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        };
        assert!(s.validate().is_err());
    }

    #[test]
    fn spec_validate_rejects_port_whitespace() {
        let mut topics = BTreeMap::new();
        topics.insert("bad port".into(), "topic".into());
        let s = NativeProgramSpec {
            name: "x".into(),
            executable: PathBuf::from("/bin/echo"),
            topics,
            args: vec![],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        };
        assert!(s.validate().is_err());
    }

    #[test]
    fn spec_validate_rejects_empty_arg() {
        let s = NativeProgramSpec {
            name: "x".into(),
            executable: PathBuf::from("/bin/echo"),
            topics: BTreeMap::new(),
            args: vec![String::new()],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        };
        assert!(s.validate().is_err());
    }

    #[test]
    fn spec_validate_accepts_well_formed() {
        let mut topics = BTreeMap::new();
        topics.insert("scan".into(), "/scan".into());
        let s = NativeProgramSpec {
            name: "x".into(),
            executable: PathBuf::from("/bin/echo"),
            topics,
            args: vec!["--rate".into(), "10".into()],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        };
        assert!(s.validate().is_ok());
    }

    #[test]
    fn mapping_render_arg_basic_types() {
        let m = NativeCliMapping::default();
        assert_eq!(
            m.render_arg("rate", Some(10.5_f64.into())),
            vec!["--rate".to_string(), "10.5".to_string()]
        );
        assert_eq!(
            m.render_arg("flag", Some(true.into())),
            vec!["--flag".to_string(), "true".to_string()]
        );
        assert_eq!(
            m.render_arg("paths", Some(vec!["a".to_string(), "b".to_string()].into())),
            vec!["--paths".to_string(), "a,b".to_string()]
        );
    }

    #[test]
    fn mapping_render_arg_excludes_and_overrides() {
        let mut m = NativeCliMapping::default();
        m.exclude.insert("internal".into());
        m.name_override
            .insert("vehicle_width".into(), "vehicleWidth".into());
        assert!(m.render_arg("internal", Some(1.into())).is_empty());
        assert_eq!(
            m.render_arg("vehicle_width", Some(0.5_f64.into())),
            vec!["--vehicleWidth".to_string(), "0.5".to_string()]
        );
    }

    #[test]
    fn mapping_render_arg_none_value_skips() {
        let m = NativeCliMapping::default();
        assert!(m.render_arg("x", None).is_empty());
    }

    #[test]
    fn mapping_render_arg_empty_string_skips() {
        let m = NativeCliMapping::default();
        assert!(m.render_arg("x", Some(String::new().into())).is_empty());
    }
}

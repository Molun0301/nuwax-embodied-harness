//! 启动时读取由设备管理程序合成的配置；默认值仅由模式文件提供。
use serde::{de::DeserializeOwned, Deserialize, Deserializer};
use std::{
    path::{Path, PathBuf},
    sync::OnceLock,
};

pub mod sections;

#[derive(Clone, Debug)]
pub struct Config(toml::Table);
impl Config {
    pub fn parse(text: &str) -> Result<Self, String> {
        toml::from_str(text)
            .map(Self)
            .map_err(|e| format!("配置解析失败：{e}"))
    }
    pub fn read(path: &Path) -> Result<Self, String> {
        Self::parse(
            &std::fs::read_to_string(path)
                .map_err(|e| format!("读取配置 {}：{e}", path.display()))?,
        )
    }
    pub fn section<T: DeserializeOwned>(&self, name: &str) -> Result<T, String> {
        self.0
            .get(name)
            .ok_or_else(|| format!("配置缺少 [{name}] 段"))?
            .clone()
            .try_into()
            .map_err(|e| format!("配置 [{name}]：{e}"))
    }
}

pub fn install_home() -> PathBuf {
    std::env::var_os("NUWAX_INSTALL_HOME")
        .map(PathBuf::from)
        .expect("启动缺少 NUWAX_INSTALL_HOME 安装根")
}

pub fn resolve_path(path: impl AsRef<Path>) -> PathBuf {
    install_home().join(path)
}

/// 每个进程只读取一次文件；后续段解析使用同一份不可变快照。
pub fn section<T: DeserializeOwned>(name: &str) -> Result<T, String> {
    static CONFIG: OnceLock<Result<Config, String>> = OnceLock::new();
    CONFIG
        .get_or_init(|| Config::read(&resolve_path("run/config.toml")))
        .as_ref()
        .map_err(Clone::clone)?
        .section(name)
}

/// 数字键的空字符串表示未声明；其它类型交给反序列化边界报错。
pub fn optional_number<'de, D, T>(d: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Value<T> {
        Number(T),
        Empty(String),
    }
    match Value::<T>::deserialize(d)? {
        Value::Number(n) => Ok(Some(n)),
        Value::Empty(s) if s.is_empty() => Ok(None),
        Value::Empty(_) => Err(serde::de::Error::custom("数字键只允许数字或空字符串")),
    }
}

#[cfg(any(test, feature = "test-support"))]
pub mod test_support;

/// 进程启动捕获的 [robot] 段。
pub fn robot() -> Result<&'static sections::Robot, String> {
    static VALUE: OnceLock<Result<sections::Robot, String>> = OnceLock::new();
    VALUE
        .get_or_init(|| section("robot"))
        .as_ref()
        .map_err(Clone::clone)
}

/// 进程启动捕获的 [transport] 段。
pub fn transport() -> Result<&'static sections::Transport, String> {
    static VALUE: OnceLock<Result<sections::Transport, String>> = OnceLock::new();
    VALUE
        .get_or_init(|| section("transport"))
        .as_ref()
        .map_err(Clone::clone)
}

/// 进程启动捕获的 [geometry] 段。
pub fn geometry() -> Result<&'static sections::Geometry, String> {
    static VALUE: OnceLock<Result<sections::Geometry, String>> = OnceLock::new();
    VALUE
        .get_or_init(|| section("geometry"))
        .as_ref()
        .map_err(Clone::clone)
}

/// 进程启动捕获的 [navigation] 段。
pub fn navigation() -> Result<&'static sections::Navigation, String> {
    static VALUE: OnceLock<Result<sections::Navigation, String>> = OnceLock::new();
    VALUE
        .get_or_init(|| section("navigation"))
        .as_ref()
        .map_err(Clone::clone)
}

/// 进程启动捕获的 [person_follow] 段。
pub fn person_follow() -> Result<&'static sections::PersonFollow, String> {
    static VALUE: OnceLock<Result<sections::PersonFollow, String>> = OnceLock::new();
    VALUE
        .get_or_init(|| section("person_follow"))
        .as_ref()
        .map_err(Clone::clone)
}

/// 进程启动捕获的 [logging] 段。
pub fn logging() -> Result<&'static sections::Logging, String> {
    static VALUE: OnceLock<Result<sections::Logging, String>> = OnceLock::new();
    VALUE
        .get_or_init(|| section("logging"))
        .as_ref()
        .map_err(Clone::clone)
}

/// 进程启动捕获的 [derived] 段。
pub fn derived() -> Result<&'static sections::Derived, String> {
    static VALUE: OnceLock<Result<sections::Derived, String>> = OnceLock::new();
    VALUE
        .get_or_init(|| section("derived"))
        .as_ref()
        .map_err(Clone::clone)
}

/// 与发布包同目录的程序位置，由父进程的真实可执行文件位置确定。
pub fn sibling_binary(name: &str) -> Result<PathBuf, String> {
    Ok(std::env::current_exe()
        .map_err(|e| e.to_string())?
        .parent()
        .expect("可执行文件目录")
        .join(name))
}
pub const GEOMETRY_READY: &str = "runtime_data/geometry/nuwax_geometry_runtime.ready.runtime";
pub const NAV_READY: &str = "runtime_data/navigation/nuwax_nav_stack.ready.runtime";
pub const FOLLOW_READY: &str = "runtime_data/person-follow/nuwax_person_follow.ready.runtime";

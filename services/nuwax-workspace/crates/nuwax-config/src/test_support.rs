//! 测试从唯一模式提取默认值；推导段由夹具显式提供。
use super::*;
pub fn defaults() -> Config {
    let schema: toml::Table =
        toml::from_str(include_str!("../../../../../core/config_schema.toml")).unwrap();
    let mut result = toml::Table::new();
    for (name, section) in schema {
        let fields = section
            .as_table()
            .unwrap()
            .iter()
            .map(|(name, spec)| (name.clone(), spec.get("default").unwrap().clone()))
            .collect();
        result.insert(name, toml::Value::Table(fields));
    }
    Config(result)
}
pub fn section<T: DeserializeOwned>(name: &str) -> T {
    defaults().section(name).unwrap()
}
impl Config {
    pub fn set(&mut self, section: &str, key: &str, value: impl Into<toml::Value>) {
        self.0
            .entry(section.to_owned())
            .or_insert_with(|| toml::Value::Table(toml::Table::new()))
            .as_table_mut()
            .unwrap()
            .insert(key.to_owned(), value.into());
    }
    pub fn write_install(&self, root: &Path) -> PathBuf {
        let path = root.join("run/config.toml");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, toml::to_string(&self.0).unwrap()).unwrap();
        path
    }
}

//! 构建 DUFOMap/UFOMap 原生封装。UFOMap 需要 C++20（`<concepts>`、`<ranges>`），
//! 而 Go2 的 Ubuntu 20.04 默认 gcc 9 不支持；交叉构建与设备本机构建都会优先选用 g++-10，
//! 只影响本 crate，其它原生 crate 仍用原编译器。
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=native");
    let target = std::env::var("TARGET").expect("cargo 提供 TARGET");
    let target_key = target.replace('-', "_");
    for name in [format!("NUWAX_CXX20_{target_key}"), "NUWAX_CXX20".into(), format!("CXX_{target_key}"), "CXX".into()] {
        println!("cargo:rerun-if-env-changed={name}");
    }
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++20")
        .opt_level(3)
        .include("native/upstream/ufomap/include")
        .include("native/upstream/liblzf")
        .file("native/bridge.cc")
        // 上游头文件告警很多，不影响本封装的正确性。
        .warnings(false);
    if let Some(compiler) = cxx20_compiler(&target_key, &build) {
        build.compiler(compiler);
    }
    build.compile("nuwax_dufomap_native");
}

/// 选择支持 C++20 的编译器：显式变量优先；否则当默认编译器主版本低于 10 时，
/// 改用同目录或命令搜索路径中的 g++-10。都没有时保留默认编译器，由编译错误如实暴露。
fn cxx20_compiler(target_key: &str, build: &cc::Build) -> Option<PathBuf> {
    for name in [format!("NUWAX_CXX20_{target_key}"), "NUWAX_CXX20".into()] {
        if let Some(value) = std::env::var_os(&name).filter(|value| !value.is_empty()) {
            return Some(PathBuf::from(value));
        }
    }
    let default = build.get_compiler().path().to_path_buf();
    if gcc_major(&default).is_none_or(|major| major >= 10) {
        return None;
    }
    let sibling = default.file_name().and_then(|name| name.to_str()).map(|name| default.with_file_name(format!("{name}-10")));
    sibling
        .filter(|path| path.is_file())
        .or_else(|| which("g++-10"))
}

fn gcc_major(compiler: &Path) -> Option<u32> {
    let output = Command::new(compiler).arg("-dumpversion").output().ok()?;
    let text = String::from_utf8(output.stdout).ok()?;
    text.trim().split('.').next()?.parse().ok()
}

fn which(name: &str) -> Option<PathBuf> {
    std::env::var_os("PATH").and_then(|paths| std::env::split_paths(&paths).map(|dir| dir.join(name)).find(|path| path.is_file()))
}

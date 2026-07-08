//! `nuwax-lcm-sys` 的 build.rs。
//!
//! ## 行为
//!
//! - 探测系统 `liblcm` (>=1.4) —— 通过 `pkg-config --modversion lcm`
//!   (用 std::process::Command 跑,不引入 pkg-config crate)
//! - 成功:把 cflags / libs 通过 `cargo:rustc-link-*` 输出,让 `nuwax-lcm-sys`
//!   的 extern "C" 函数能正常链接到 `liblcm.so`
//! - 失败:fail-fast —— 把安装指令写到 stderr,然后 `std::process::exit(2)`。
//!
//! ## 为什么不用 `pkg-config` crate?
//!
//! 1. 减少 build dep;dev/CI 镜像若不装 `pkg-config` crate,本 crate 仍能
//!    (间接)被编译链接
//! 2. 直接控制输出格式 + 失败信息
//! 3. 探测逻辑只有 30 行,不必为它引 crate

use std::process::{Command, ExitCode, Stdio};

/// 期望的 liblcm 最低版本(major.minor)。
///
/// 2026-07-08 修订:从 "1.4" 降到 "1.3"。
/// 原因:实机(Go2 Jetson / Ubuntu 20.04 focal)apt 仓库最高只提供 liblcm 1.3.1,
/// 按"实机配置是 SSOT"原则不能动实机;liblcm 1.x 系列协议与 7 个 FFI 函数
/// (lcm_create/destroy/publish/subscribe/unsubscribe/handle_timeout/get_fileno)
/// 自 1.0 起签名未变,1.3 与 1.4 ABI 实际兼容;原先的"1.4 之前 ABI 不稳定"
/// 描述是误判——focal 仓库的 1.3.1 是上游 vanilla,不是私有 fork。
const REQUIRED_LCM_VERSION: &str = "1.3";

fn main() -> ExitCode {
    // Phase 3 设计:
    // - 默认情况下(skip-liblcm-feature)build.rs **不探测** liblcm,
    //   让 dev / CI 镜像无 liblcm-dev 时 `cargo check` 仍可走 workspace default。
    // - 用户启用 `link-liblcm` feature 时(经 `nuwax-transport/real-lcm`),
    //   才探测 + 链 liblcm;失败 → fail-fast。
    //
    // 探测入口由 Cargo feature flag 控制 —— 在 cargo 里:
    //   cargo check -p nuwax-lcm-sys
    //   cargo build -p nuwax-transport --features real-lcm
    let want_link = std::env::var("CARGO_FEATURE_LINK_LIBLCM").is_ok()
        || std::env::var("NUWAX_FORCE_LIBLCM_LINK").is_ok();
    if !want_link {
        // 不打 cargo:rerun-if-changed-env,build.rs 不依赖环境;
        // 默认编译链路仅占位 Rust 源码里的 extern "C",**不**链 .so
        // —— CI 通过 fake backend 跑通,real backend 走可选 feature。
        println!(
            "cargo:warning=nuwax-lcm-sys: 默认编译 — 跳过 liblcm 探测;\
             real-lcm 用户请加 `--features link-liblcm`(由 nuwax-transport/real-lcm 自动启用)"
        );
        return ExitCode::SUCCESS;
    }

    // 1. 跑 pkg-config --modversion lcm
    let ver_out = Command::new("pkg-config")
        .args(["--modversion", "lcm"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output();

    let version = match ver_out {
        Ok(o) if o.status.success() => {
            String::from_utf8_lossy(&o.stdout).trim().to_owned()
        }
        Ok(o) => {
            let stderr = String::from_utf8_lossy(&o.stderr);
            return fail_fast(&stderr);
        }
        Err(e) => {
            return fail_fast(&format!(
                "无法执行 pkg-config 命令(系统是否安装 pkg-config?): {}",
                e
            ));
        }
    };

    // 2. 校验版本 >= REQUIRED_LCM_VERSION
    if !version_at_least(&version, REQUIRED_LCM_VERSION) {
        return fail_fast(&format!(
            "liblcm version = {}, 本 crate 最低要求 {}",
            version, REQUIRED_LCM_VERSION
        ));
    }

    // 3. 取 cflags
    let cflags_out = Command::new("pkg-config")
        .args(["--cflags", "lcm"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output();
    let cflags = match cflags_out {
        Ok(o) if o.status.success() => String::from_utf8_lossy(&o.stdout).trim().to_owned(),
        Ok(o) => {
            return fail_fast(&String::from_utf8_lossy(&o.stderr));
        }
        Err(e) => {
            return fail_fast(&format!("pkg-config --cflags 失败: {}", e));
        }
    };

    // 4. 取 libs
    let libs_out = Command::new("pkg-config")
        .args(["--libs", "lcm"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output();
    let libs_str = match libs_out {
        Ok(o) if o.status.success() => String::from_utf8_lossy(&o.stdout).trim().to_owned(),
        Ok(o) => {
            return fail_fast(&String::from_utf8_lossy(&o.stderr));
        }
        Err(e) => {
            return fail_fast(&format!("pkg-config --libs 失败: {}", e));
        }
    };

    // 5. 解析 cflags: -I/usr/include/lcm   -I...
    for token in cflags.split_whitespace() {
        if let Some(path) = token.strip_prefix("-I") {
            println!("cargo:include={}", path);
        }
    }

    // 6. 解析 libs: -L/usr/lib/x86_64-linux-gnu -llcm ...
    for token in libs_str.split_whitespace() {
        if let Some(path) = token.strip_prefix("-L") {
            println!("cargo:rustc-link-search=native={}", path);
        } else if let Some(lib) = token.strip_prefix("-l") {
            println!("cargo:rustc-link-lib={}", lib);
        }
    }

    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_SYSROOT_DIR");
    println!("cargo:rerun-if-env-changed=LIBLCM_PKG_CONFIG_PATH");
    println!(
        "cargo:warning=nuwax-lcm-sys: liblcm {} 链接 OK (cflags={:?}, libs={:?})",
        version, cflags, libs_str
    );
    ExitCode::SUCCESS
}

/// 简易 semver 比较:`>=`(只比 major.minor,够 liblcm 用)。
fn version_at_least(have: &str, want: &str) -> bool {
    let parse = |s: &str| -> Vec<u32> {
        s.split('.')
            .filter_map(|p| p.split('-').next().and_then(|x| x.parse::<u32>().ok()))
            .collect()
    };
    let h = parse(have);
    let w = parse(want);
    if h.is_empty() || w.is_empty() {
        return false;
    }
    for (a, b) in h.iter().zip(w.iter()) {
        if a > b {
            return true;
        }
        if a < b {
            return false;
        }
    }
    h.len() >= w.len()
}

fn fail_fast(detail: &str) -> ExitCode {
    eprintln!(
        "nuwax-lcm-sys build.rs: 探测系统 liblcm 失败 —— real-lcm feature 必须依赖 \
         liblcm (>=1.4)。\n\
         \n\
         错误细节: {}\n\
         \n\
         修复:\n\
         - Ubuntu/Debian:  sudo apt-get install liblcm-dev pkg-config\n\
         - Fedora/RHEL:    sudo dnf install lcm-devel pkgconfig\n\
         - macOS (brew):   brew install lcm pkg-config\n\
         \n\
         本 crate **不会** mock 或 fallback 到 fake backend;fake backend 走 \
         `nuwax-transport` 默认(不启用 real-lcm feature)。\n\
         \n\
         退出码 2 供 start_gateway.sh / CI 区分 \"缺系统库\" 与 \"代码错误\"。",
        detail
    );
    ExitCode::from(2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_compare_basic() {
        assert!(version_at_least("1.4.0", "1.4"));
        assert!(version_at_least("1.4.1", "1.4"));
        assert!(version_at_least("1.5.0", "1.4"));
        assert!(version_at_least("2.0.0", "1.4"));
        assert!(!version_at_least("1.3.9", "1.4"));
        assert!(!version_at_least("0.5.0", "1.4"));
    }
}
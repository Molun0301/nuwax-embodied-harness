//! 导航板块测试夹具
//!
//! - [`fixtures`]:JSON 序列化友好的构造器(pose / path / occupancy grid / bbox / goal / diagnostics)
//! - 配套 mock native binary:`src/bin/mock_native.rs`(打印 argv / 根据 env 控制 sleep 与 exit code)
//! - [`mock_native_path`]:跨 bridge crate 共用的 mock-native binary 定位函数
//!   (优先 `CARGO_BIN_EXE_mock-native`,否则向上查找 workspace `target/debug/mock-native`)
//! - [`goldens`]:synthetic golden fixture(JSON:bridge 输入 + 期望 CLI 输出),用于
//!   各 bridge 的确定性冒烟测试,不依赖真实 rosbag。

#![deny(unsafe_code)]

pub mod fixtures;
pub mod goldens;
pub mod recorded_events;

pub use fixtures::*;

use std::path::PathBuf;

/// 定位 `mock-native` binary 路径(供各 bridge crate 的集成测试共享)。
///
/// 优先级:
/// 1. cargo test 在编译期注入的 `CARGO_BIN_EXE_mock-native`(推荐路径);
/// 2. 回退到 workspace 根的 `target/debug/mock-native`;
/// 3. 再回退到 `CARGO_TARGET_DIR` 下的 `mock-native`。
/// 4. **占位返回** —— 如果以上都未找到,返回 `PathBuf::from("mock-native")`
///    (相对路径,适合"只看 argv 渲染"的纯 Rust 单元 / 集成测试使用,不启动
///    binary,不会触发文件不存在错误)。Bridge lifecycle 集成测试**已经迁移**
///    到 `nuwax-nav-test-fixtures/tests/bridge_lifecycle.rs`,该处使用
///    `env!("CARGO_BIN_EXE_mock-native")` 编译期取得精确路径 — 走步骤 1,本
///    fallback 仅服务于"argv 渲染 / golden fixture 对比"等不需要 binary 启动的
///    测试场景,确保 `cargo test -p <bridge> --tests` 在任意 `CARGO_TARGET_DIR`
///    (包括全新临时目录)下都能 PASS。
///
/// 该函数独立于 `nuwax-nav-test-fixtures` 的 [[bin]] 是否在当前 package 内被
/// 显式声明 — 只要 workspace target 目录里存在 mock-native 产物,函数即可定位。
#[must_use]
pub fn mock_native_path() -> PathBuf {
    // 1. cargo 注入的标准路径(直接测试 test-fixtures 时一定可用)
    if let Ok(p) = std::env::var("CARGO_BIN_EXE_mock-native") {
        return PathBuf::from(p);
    }

    // 2. 沿 CARGO_MANIFEST_DIR 向上找 workspace 根,取 target/debug/mock-native
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR")
        .or_else(|_| std::env::var("CARGO_MANIFEST"))
        .unwrap_or_default();
    let mut dir = PathBuf::from(&manifest_dir);
    for _ in 0..6 {
        let candidate = dir.join("target").join("debug").join("mock-native");
        if candidate.exists() {
            return candidate;
        }
        if !dir.pop() {
            break;
        }
    }

    // 3. CARGO_TARGET_DIR 环境变量指定的目标目录
    if let Ok(target_dir) = std::env::var("CARGO_TARGET_DIR") {
        let candidate = PathBuf::from(&target_dir).join("debug").join("mock-native");
        if candidate.exists() {
            return candidate;
        }
        // 也可能是 target 直接写 mock-native
        let candidate2 = PathBuf::from(&target_dir).join("mock-native");
        if candidate2.exists() {
            return candidate2;
        }
    }

    // 4. 占位 fallback — 不再 panic
    PathBuf::from("mock-native")
}

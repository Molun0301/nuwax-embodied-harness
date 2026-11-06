// Copyright 2026 Nuwax.
// SPDX-License-Identifier: GPL-2.0-or-later

//! `nuwax-nav-stack` CLI 入口。
//!
//! 设计原则:
//! - **不引入** clap / structopt 等重 CLI 库,直接手工解析以避免 MSRV / edition 风险。
//! - **不引入** DDS / ROS2 runtime / CUDA 默认依赖。
//! - **不**在 build.rs 下载模型 / 二进制 / 访问网络。
//! - 解析失败时打印 usage 并 exit(2),与 Unix 工具惯例一致。
//!
//! 用法:
//! ```text
//! nuwax-nav-stack --profile=cmu-native --dry-run
//! nuwax-nav-stack --list-features
//! nuwax-nav-stack --profile=simple-2d --preset=all --json
//! ```

use nuwax_nav_stack::bin_support::{self, CliOptions};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if bin_support::launcher_contract_requested(&args) {
        println!("{}", bin_support::LAUNCHER_CONTRACT);
        return;
    }
    nuwax_core::logging::init_component_logging("nuwax-nav-stack", "info");
    let opts = match CliOptions::parse(&args) {
        Ok(o) => o,
        Err(e) => {
            nuwax_core::logging::process_error("nuwax-nav-stack", &e);
            eprintln!();
            eprintln!("Try `--help` for usage.");
            std::process::exit(2);
        }
    };
    if let Err(e) = bin_support::run(opts) {
        nuwax_core::logging::process_error("nuwax-nav-stack", &e);
        std::process::exit(1);
    }
}

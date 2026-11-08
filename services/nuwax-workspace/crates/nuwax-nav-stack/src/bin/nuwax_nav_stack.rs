// Copyright 2026 Nuwax.
// SPDX-License-Identifier: GPL-2.0-or-later

//! 固定二维与三维导航任务的命令入口。

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

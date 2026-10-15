//! Keep native builds inside Cargo's shared parallelism budget.
use std::{env, fs, path::Path, process::Command};

pub fn command(build: &Path) -> Command {
    let mut command = Command::new("cmake");
    command.arg("--build").arg(build);
    // This environment variable makes CMake add -j even without --parallel,
    // which would replace the inherited GNU Make jobserver with another pool.
    command.env_remove("CMAKE_BUILD_PARALLEL_LEVEL");
    if let Some(flags) = env::var_os("CARGO_MAKEFLAGS") {
        let cache = fs::read_to_string(build.join("CMakeCache.txt")).expect("CMake cache");
        if cache
            .lines()
            .any(|line| line == "CMAKE_GENERATOR:INTERNAL=Unix Makefiles")
        {
            command.env("MAKEFLAGS", flags);
        } else {
            // Older Ninja versions cannot consume Cargo's pipe jobserver. Keep
            // the existing cache/generator and the build script's single slot.
            command.args(["--parallel", "1"]);
            println!("cargo:warning=Native CMake generator does not share Cargo's jobserver; using one slot. Unix Makefiles enables shared parallel builds in a new build directory.");
        }
    } else {
        let jobs = env::var("CMAKE_BUILD_PARALLEL_LEVEL")
            .or_else(|_| env::var("NUM_JOBS"))
            .unwrap_or_else(|_| "1".into());
        command.arg("--parallel").arg(jobs);
    }
    command
}

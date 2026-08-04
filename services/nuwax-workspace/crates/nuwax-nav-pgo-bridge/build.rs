use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    let build = out.join("native-pgo-build");
    let source = PathBuf::from("native/pgo");
    std::fs::create_dir_all(&build).expect("create native build directory");

    let configure = Command::new("cmake")
        .args([
            "-S",
            source.to_str().unwrap(),
            "-B",
            build.to_str().unwrap(),
        ])
        .status()
        .expect("run cmake configure for native PGO");
    assert!(configure.success(), "native PGO cmake configure failed");
    let compile = Command::new("cmake")
        .args(["--build", build.to_str().unwrap(), "--config", "Release"])
        .status()
        .expect("build native PGO");
    assert!(compile.success(), "native PGO build failed");

    let binary = build.join("pgo");
    assert!(binary.is_file(), "native PGO artifact missing");
    println!("cargo:rustc-env=NUWAX_PGO_NATIVE_BIN={}", binary.display());
    println!("cargo:rerun-if-changed=native/pgo");
    println!("cargo:rerun-if-changed=../../lcmtypes/nuwax_msgs/NativePgoRequest.lcm");
    println!("cargo:rerun-if-changed=../../lcmtypes/nuwax_msgs/NativePgoResult.lcm");
    println!("cargo:rerun-if-changed=../../lcmtypes/nuwax_msgs/NativePgoStatus.lcm");
}

use std::env;
use std::path::PathBuf;
use std::process::Command;

fn compile_guardian(output: &PathBuf, test_hooks: bool) {
    let compiler = env::var_os("CC").unwrap_or_else(|| "cc".into());
    let mut build = Command::new(compiler);
    if test_hooks {
        build.arg("-DNUWAX_GUARDIAN_TEST_HOOKS=1");
    }
    let status = build
        .args([
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wconversion",
            "-Wshadow",
            "-Wformat=2",
            "-fno-strict-aliasing",
            "native/group_guardian.c",
            "-o",
        ])
        .arg(output)
        .status()
        .expect("failed to run the C compiler for the native group guardian");
    assert!(status.success(), "native group guardian compilation failed");
}

fn main() {
    println!("cargo:rerun-if-changed=native/group_guardian.c");
    println!("cargo:rerun-if-env-changed=CC");

    if env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("linux") {
        println!("cargo:rustc-env=NUWAX_GROUP_GUARDIAN_PATH=");
        println!("cargo:rustc-env=NUWAX_GROUP_GUARDIAN_TEST_PATH=");
        return;
    }

    let output_directory =
        PathBuf::from(env::var_os("OUT_DIR").expect("Cargo must provide OUT_DIR"));
    let output = output_directory.join("nuwax-group-guardian");
    compile_guardian(&output, false);

    println!(
        "cargo:rustc-env=NUWAX_GROUP_GUARDIAN_PATH={}",
        output.display()
    );
    let test_output = output_directory.join("nuwax-group-guardian-test");
    compile_guardian(&test_output, true);
    println!(
        "cargo:rustc-env=NUWAX_GROUP_GUARDIAN_TEST_PATH={}",
        test_output.display()
    );
}

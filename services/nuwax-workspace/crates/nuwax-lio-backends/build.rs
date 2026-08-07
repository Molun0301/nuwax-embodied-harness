use std::env;
use std::process::Command;

fn main() {
    let rustc = env::var_os("RUSTC").expect("Cargo did not provide RUSTC");
    let target = env::var("TARGET").expect("Cargo did not provide TARGET");
    let output = Command::new(rustc)
        .arg("-vV")
        .output()
        .expect("failed to execute build compiler with -vV");
    assert!(output.status.success(), "build compiler rustc -vV failed");
    let stdout = String::from_utf8(output.stdout).expect("rustc -vV output is not UTF-8");
    let release = stdout
        .lines()
        .find_map(|line| line.strip_prefix("release: "))
        .expect("rustc -vV has no release line");
    let host = stdout
        .lines()
        .find_map(|line| line.strip_prefix("host: "))
        .expect("rustc -vV has no host line");
    fn valid(value: &str) -> bool {
        !value.is_empty()
            && value
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || b"._+-".contains(&b))
    }
    assert!(
        valid(release),
        "rustc release contains unsupported characters"
    );
    assert!(valid(host), "rustc host contains unsupported characters");
    assert!(valid(&target), "TARGET contains unsupported characters");
    println!("cargo:rustc-env=NUWAX_BUILD_RUSTC_RELEASE={release}");
    println!("cargo:rustc-env=NUWAX_BUILD_TARGET={target}");
    println!("cargo:rerun-if-env-changed=RUSTC");
    println!("cargo:rerun-if-env-changed=TARGET");
    println!("cargo:rerun-if-changed=build.rs");
}

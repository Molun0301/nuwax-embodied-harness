fn main() {
    println!("cargo:rerun-if-changed=native");
    println!("cargo:rerun-if-env-changed=EIGEN3_INCLUDE_DIR");
    let eigen =
        std::env::var("EIGEN3_INCLUDE_DIR").unwrap_or_else(|_| "/usr/include/eigen3".into());
    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .opt_level(2)
        .include(eigen)
        .include("native/upstream")
        .file("native/upstream/a_star/a_star_search.cc")
        .file("native/bridge.cc")
        .warnings(true)
        .compile("nuwax_pct_native");
}

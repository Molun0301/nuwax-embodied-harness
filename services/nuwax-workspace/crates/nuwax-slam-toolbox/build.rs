fn main() {
    if let Ok(path) = std::env::var("DEP_NUWAX_SLAM_TOOLBOX_LIBDIR") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{path}");
    }
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../lib");
}

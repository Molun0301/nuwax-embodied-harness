fn main() -> Result<(), Box<dyn std::error::Error>> {
    let proto = "../../../navigation/proto/motion_guard.proto";
    println!("cargo:rerun-if-changed={proto}");
    tonic_build::configure()
        .build_client(false)
        .build_server(false)
        .compile_protos(&[proto], &["../../../navigation/proto"])?;
    Ok(())
}

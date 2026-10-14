fn main() {
    let proto = "../../../geometry/nuwax_geometry_runtime/proto/geometry_runtime.proto";
    println!("cargo:rerun-if-changed={proto}");
    let mut prost = prost_build::Config::new();
    prost.protoc_arg("--experimental_allow_proto3_optional");
    tonic_build::configure()
        .build_server(false)
        .compile_protos_with_config(
            prost,
            &[proto],
            &["../../../geometry/nuwax_geometry_runtime/proto"],
        )
        .expect("compile authoritative Geometry Runtime proto");
}

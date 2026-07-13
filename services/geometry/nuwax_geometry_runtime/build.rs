fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_build::configure()
        .type_attribute(".", "#[derive(serde::Serialize, serde::Deserialize)]")
        .type_attribute(".", "#[serde(default, deny_unknown_fields)]")
        .compile_protos(&["proto/geometry_runtime.proto"], &["proto"])?;
    Ok(())
}

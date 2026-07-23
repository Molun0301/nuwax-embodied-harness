fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_build::configure()
        .protoc_arg("--experimental_allow_proto3_optional")
        .type_attribute(".", "#[derive(serde::Serialize, serde::Deserialize)]")
        .message_attribute(".", "#[serde(default, deny_unknown_fields)]")
        .skip_debug("nuwax.geometry.ExplorationRecoveryGuardProof")
        .skip_debug("nuwax.geometry.RecoverExplorationAfterNavigationRolloverRequest")
        .skip_debug("nuwax.geometry.ReconcileExplorationAfterGlobalStopRequest")
        .compile_protos(&["proto/geometry_runtime.proto"], &["proto"])?;
    Ok(())
}

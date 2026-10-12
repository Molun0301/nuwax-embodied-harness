use super::*;

#[test]
fn assigned_artifact_roundtrip_never_serializes_a_digest() {
    let value = FileArtifact {
        file_name: "input-42.nws2d".into(), payload_size: 1024,
        identity: ArtifactIdentity::Assigned { checkpoint_id: "1".repeat(32) },
    };
    value.validate().unwrap();
    let bytes = serde_json::to_vec(&value).unwrap();
    assert!(!String::from_utf8_lossy(&bytes).contains("sha256"));
    assert_eq!(serde_json::from_slice::<FileArtifact>(&bytes).unwrap(), value);
    assert_eq!(VERSION, 7);
    assert_eq!(WORKER_CONTRACT, "nuwax-slam-2d-worker-v7");
}
#[test]
fn legacy_artifact_is_only_a_format_marker() {
    let value = FileArtifact { file_name: "legacy.nws2d".into(), payload_size: 19, identity: ArtifactIdentity::Legacy };
    value.validate().unwrap();
    assert_eq!(serde_json::to_value(&value).unwrap(), serde_json::json!({
        "file_name":"legacy.nws2d", "payload_size":19, "identity":{"scheme":"legacy"}
    }));
    assert_eq!(serde_json::from_value::<FileArtifact>(serde_json::to_value(value.clone()).unwrap()).unwrap(), value);
}
#[test]
fn missing_identity_old_wire_and_invalid_ids_are_rejected() {
    for raw in [
        serde_json::json!({"file_name":"a.nws2d","payload_size":1,"sha256":[1,2,3]}),
        serde_json::json!({"file_name":"a.nws2d","payload_size":1,"sha256":[],"identity":{"scheme":"legacy"}}),
        serde_json::json!({"file_name":"a.nws2d","payload_size":1,"identity":{"scheme":"unknown"}}),
    ] { assert!(serde_json::from_value::<FileArtifact>(raw.clone()).is_err(), "{raw}"); }
    for id in ["".into(), "0".repeat(32), "A".repeat(32), "1".repeat(31)] {
        assert!(FileArtifact { file_name:"a.nws2d".into(),payload_size:1,
            identity:ArtifactIdentity::Assigned {checkpoint_id:id} }.validate().is_err());
    }
}
#[test]
fn artifact_locators_stay_local_and_bounded() {
    for name in ["../a", "/a", ".hidden", "a/b"] {
        assert!(FileArtifact {file_name:name.into(),payload_size:1,identity:ArtifactIdentity::Legacy}.validate().is_err());
    }
    for size in [0,crate::archive::MAX_ARCHIVE+1] {
        assert!(FileArtifact {file_name:"a.nws2d".into(),payload_size:size,identity:ArtifactIdentity::Legacy}.validate().is_err());
    }
}

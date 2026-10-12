//! Durable publication for mapping CLI file artifacts.
//!
//! Every artifact is built in a uniquely owned sibling directory, validated, and
//! synced before an atomic no-clobber link exposes the final path. Keeping the
//! staging directory beside the destination makes the publication boundary stay on
//! one filesystem.

use std::collections::BTreeSet;
use std::ffi::{OsStr, OsString};
use std::fmt;
use std::fs::{self, File, Metadata};
use std::io::{self, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use anyhow::{bail, Context};
use serde_json::{json, Map, Value};

const STAGE_DOMAIN: &str = ".nuwax-mapping-cli-stage-v2-";
const MAX_STAGE_ATTEMPTS: usize = 32;
const MAX_STAGES_PER_ARTIFACT: usize = 64;
const SQLITE_SIDECAR_SUFFIXES: [&str; 3] = ["-wal", "-shm", "-journal"];

static NEXT_STAGE: AtomicU64 = AtomicU64::new(0);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ArtifactFormat {
    DatasetJsonLines,
    #[allow(dead_code)]
    MapPgm,
    SummaryJson,
    Memory2Sqlite,
}

impl ArtifactFormat {
    fn stage_label(self) -> &'static str {
        match self {
            Self::DatasetJsonLines => "dataset-jsonl",
            Self::MapPgm => "map-pgm",
            Self::SummaryJson => "summary-json",
            Self::Memory2Sqlite => "memory2-sqlite",
        }
    }

    fn accepts_extension(self, extension: Option<&str>) -> bool {
        match self {
            Self::DatasetJsonLines => matches!(extension, Some("jsonl" | "ndjson")),
            Self::MapPgm => extension == Some("pgm"),
            Self::SummaryJson => extension == Some("json"),
            Self::Memory2Sqlite => extension == Some("db"),
        }
    }

    fn expected_extension(self) -> &'static str {
        match self {
            Self::DatasetJsonLines => ".jsonl or .ndjson",
            Self::MapPgm => ".pgm",
            Self::SummaryJson => ".json",
            Self::Memory2Sqlite => ".db",
        }
    }

    fn sqlite_sidecars(self) -> &'static [&'static str] {
        match self {
            Self::Memory2Sqlite => &SQLITE_SIDECAR_SUFFIXES,
            Self::DatasetJsonLines | Self::MapPgm | Self::SummaryJson => &[],
        }
    }
}

/// Structured failure while removing or syncing a staging entry.
#[derive(Debug)]
pub struct ArtifactCleanupFailure {
    /// Staging path used for diagnostics and later recovery.
    pub staging: PathBuf,
    /// Cleanup I/O failure.
    pub source: io::Error,
}

impl fmt::Display for ArtifactCleanupFailure {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "cleanup staging {} failed: {}",
            self.staging.display(),
            self.source
        )
    }
}

impl std::error::Error for ArtifactCleanupFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

/// Typed terminal failures after the destination path participates in a
/// publication attempt.
#[derive(Debug)]
pub enum ArtifactPublicationError {
    /// Another writer already published the destination.
    Conflict {
        /// Destination that already exists.
        path: PathBuf,
        /// Optional cleanup failure. The primary disposition remains conflict.
        cleanup: Option<ArtifactCleanupFailure>,
    },
    /// The final name is visible, but syncing its directory failed.
    DurabilityUnknown {
        /// Destination whose directory sync failed.
        path: PathBuf,
        /// Staging entry deliberately retained as recovery evidence.
        staging: PathBuf,
        /// Directory sync failure.
        source: io::Error,
    },
    /// The final name is durable, but removal of the sibling staging entry
    /// could not be made durable.
    CleanupFailure {
        /// Durable destination.
        path: PathBuf,
        /// Structured cleanup failure.
        cleanup: ArtifactCleanupFailure,
    },
}

impl fmt::Display for ArtifactPublicationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Conflict { path, cleanup } => {
                write!(
                    formatter,
                    "refusing to overwrite existing artifact: {}",
                    path.display()
                )?;
                if let Some(cleanup) = cleanup {
                    write!(formatter, "; {cleanup}")?;
                }
                Ok(())
            }
            Self::DurabilityUnknown {
                path,
                staging,
                source,
            } => write!(
                formatter,
                "artifact {} is visible and validated, but publication durability is unknown; staging {} was retained: {}",
                path.display(),
                staging.display(),
                source
            ),
            Self::CleanupFailure { path, cleanup } => write!(
                formatter,
                "artifact {} is durably published, but {cleanup}",
                path.display()
            ),
        }
    }
}

impl std::error::Error for ArtifactPublicationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Conflict {
                cleanup: Some(cleanup),
                ..
            }
            | Self::CleanupFailure { cleanup, .. } => Some(cleanup),
            Self::DurabilityUnknown { source, .. } => Some(source),
            Self::Conflict { cleanup: None, .. } => None,
        }
    }
}

/// Map bundle directory publication outcomes that callers must distinguish.
#[derive(Debug)]
pub(crate) enum MapBundlePublicationError {
    PublishedCleanupFailure {
        path: PathBuf,
        cleanup: ArtifactCleanupFailure,
    },
    Conflict {
        path: PathBuf,
        cleanup: Option<ArtifactCleanupFailure>,
    },
    CleanupFailure {
        path: PathBuf,
        primary: anyhow::Error,
        cleanup: ArtifactCleanupFailure,
    },
    RollbackFailure {
        path: PathBuf,
        primary: anyhow::Error,
        rollback: ArtifactCleanupFailure,
    },
    DurabilityUnknown {
        path: PathBuf,
        source: io::Error,
    },
}

impl fmt::Display for MapBundlePublicationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PublishedCleanupFailure { path, cleanup } => write!(formatter, "map bundle {} is durably published, but {cleanup}", path.display()),
            Self::Conflict { path, cleanup } => {
                write!(
                    formatter,
                    "refusing to overwrite existing map bundle: {}",
                    path.display()
                )?;
                if let Some(cleanup) = cleanup {
                    write!(formatter, "; {cleanup}")?;
                }
                Ok(())
            }
            Self::CleanupFailure {
                path,
                primary,
                cleanup,
            } => write!(
                formatter,
                "map bundle {} failed: {}; staging cleanup also failed: {}",
                path.display(),
                primary,
                cleanup
            ),
            Self::RollbackFailure {
                path,
                primary,
                rollback,
            } => write!(
                formatter,
                "map bundle {} publication verification failed: {}; rollback failed and final state is unknown: {}",
                path.display(),
                primary,
                rollback
            ),
            Self::DurabilityUnknown { path, source } => write!(
                formatter,
                "map bundle {} is visible and validated, but publication durability is unknown: {}",
                path.display(),
                source
            ),
        }
    }
}

impl std::error::Error for MapBundlePublicationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::PublishedCleanupFailure { cleanup, .. } => Some(cleanup),
            Self::Conflict {
                cleanup: Some(cleanup),
                ..
            } => Some(cleanup),
            Self::CleanupFailure { cleanup, .. } => Some(cleanup),
            Self::RollbackFailure { rollback, .. } => Some(rollback),
            Self::DurabilityUnknown { source, .. } => Some(source),
            Self::Conflict { cleanup: None, .. } => None,
        }
    }
}

/// Stage3 权威候选资产的 CLI 边界镜像。复杂字段以 canonical JSON bytes
/// 传递，避免 CLI 依赖 Geometry crate，保留离线候选原始身份与来源，不分配 Stage5 资产。
#[derive(Clone, Debug)]
pub struct OfflineCandidateAssetsV2Dto {
    pub schema_version: u32,
    pub snapshot_schema_version: u32,
    pub identity_json: Vec<u8>,
    pub source_provenance_json: Vec<u8>,
    pub backend_kind: String,
    pub last_source_timestamp_ns: u64,
    pub georeference_json: Vec<u8>,
    pub processing_receipt_json: Vec<u8>,
    pub map_snapshot_seed_json: Vec<u8>,
    pub deferred_json: Vec<u8>,
    pub members: Vec<OfflineCandidateAssetMemberV2Dto>,
}

#[derive(Clone, Debug)]
pub struct OfflineCandidateAssetMemberV2Dto {
    pub relative_path_intent: String,
    pub media_type: String,
    pub encoding: String,
    pub bytes: Vec<u8>,
    pub receipt_relative_path_intent: String,
    pub receipt_media_type: String,
    pub receipt_encoding: String,
    pub size_bytes: u64,
    pub source_commit_id_json: Vec<u8>,
    pub source_binding: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OfflineCandidateMemberV2 {
    pub relative_path: String,
    pub media_type: String,
    pub encoding: String,
    pub bytes: Vec<u8>,
    pub size_bytes: u64,
    pub source_commit_id_json: Vec<u8>,
    pub source_binding: String,
}

#[derive(Clone, Debug)]
pub struct OfflineCheckpointBindingV2 {
    pub object_path: String,
    pub object_size_bytes: u64,
    pub manifest_path: String,
    pub selector_path: String,
}

impl OfflineCandidateAssetsV2Dto {
    pub fn into_publish_bundle(self) -> anyhow::Result<OfflineCandidateBundleV2> {
        if self.schema_version != 2 || self.snapshot_schema_version != 3 {
            bail!("unsupported OfflineCandidateAssetsV2 schema");
        }
        let identity = canonical_boundary_json("identity", &self.identity_json)?;
        let _georeference = canonical_boundary_json("georeference", &self.georeference_json)?;
        let _processing_receipt =
            canonical_boundary_json("processing_receipt", &self.processing_receipt_json)?;
        let _map_snapshot_seed =
            canonical_boundary_json("map_snapshot_seed", &self.map_snapshot_seed_json)?;
        let _deferred = canonical_boundary_json("deferred", &self.deferred_json)?;
        let map_name = identity
            .get("map_name")
            .and_then(Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("identity.map_name is missing"))?
            .to_owned();
        let frame_id = identity
            .get("map_frame_id")
            .and_then(Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("identity.map_frame_id is missing"))?
            .to_owned();
        let checkpoint_member = self
            .members
            .iter()
            .find(|member| member.encoding == "checkpoint_object_v1")
            .ok_or_else(|| anyhow::anyhow!("candidate has no checkpoint object"))?;
        let checkpoint = OfflineCheckpointBindingV2 {
            object_path: checkpoint_member.relative_path_intent.clone(),
            object_size_bytes: checkpoint_member.size_bytes,
            manifest_path: "mapping_backend/checkpoint-manifest.json".to_owned(),
            selector_path: "mapping_backend/checkpoint-selector.json".to_owned(),
        };
        let members = self
            .members
            .into_iter()
            .map(|member| {
                if member.relative_path_intent != member.receipt_relative_path_intent
                    || member.media_type != member.receipt_media_type
                    || member.encoding != member.receipt_encoding
                {
                    bail!(
                        "member and receipt metadata mismatch: {}",
                        member.relative_path_intent
                    );
                }
                Ok(OfflineCandidateMemberV2 {
                    relative_path: member.relative_path_intent,
                    media_type: member.media_type,
                    encoding: member.encoding,
                    bytes: member.bytes,
                    size_bytes: member.size_bytes,
                    source_commit_id_json: member.source_commit_id_json,
                    source_binding: member.source_binding,
                })
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        Ok(OfflineCandidateBundleV2 {
            schema_version: self.schema_version,
            snapshot_schema_version: self.snapshot_schema_version,
            map_name,
            frame_id,
            backend_kind: self.backend_kind,
            last_source_timestamp_ns: self.last_source_timestamp_ns,
            identity_json: self.identity_json,
            source_provenance_json: self.source_provenance_json,
            georeference_json: self.georeference_json,
            processing_receipt_json: self.processing_receipt_json,
            map_snapshot_seed_json: self.map_snapshot_seed_json,
            deferred_json: self.deferred_json,
            members,
            checkpoint,
        })
    }
}

fn canonical_boundary_json(label: &str, bytes: &[u8]) -> anyhow::Result<Value> {
    let value: Value =
        serde_json::from_slice(bytes).with_context(|| format!("{label} is not JSON"))?;
    if canonical_json_value(&value)? != bytes {
        bail!("{label} must be canonical JSON");
    }
    Ok(value)
}

#[derive(Clone, Debug)]
pub struct OfflineCandidateBundleV2 {
    pub schema_version: u32,
    pub snapshot_schema_version: u32,
    pub map_name: String,
    pub frame_id: String,
    pub backend_kind: String,
    pub last_source_timestamp_ns: u64,
    pub identity_json: Vec<u8>,
    pub source_provenance_json: Vec<u8>,
    pub georeference_json: Vec<u8>,
    pub processing_receipt_json: Vec<u8>,
    pub map_snapshot_seed_json: Vec<u8>,
    pub deferred_json: Vec<u8>,
    pub members: Vec<OfflineCandidateMemberV2>,
    pub checkpoint: OfflineCheckpointBindingV2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OfflinePublishedBundleV2 {
    pub final_directory: PathBuf,
    pub source_commit_id_json: Vec<u8>,
}

/// 物化并发布 Stage4 离线候选；不接入 map 命令，也不生成 Stage5 资产。
pub fn publish_offline_candidate_bundle(
    final_directory: &Path,
    candidate: &OfflineCandidateBundleV2,
) -> anyhow::Result<OfflinePublishedBundleV2> {
    let provenance = validate_offline_candidate(candidate)?;
    publish_map_bundle_stable(
        final_directory,
        |staging| materialize_offline_candidate(staging, candidate, &provenance),
        |staging| {
            validate_offline_candidate_directory(staging, candidate, &provenance)?;
            Ok(())
        },
    )?;
    Ok(OfflinePublishedBundleV2 {
        final_directory: final_directory.to_path_buf(),
        source_commit_id_json: canonical_json_value(&provenance["commit_id"])?,
    })
}

fn validate_offline_candidate(candidate: &OfflineCandidateBundleV2) -> anyhow::Result<Value> {
    if candidate.schema_version != 2
        || candidate.snapshot_schema_version != 3
        || !matches!(candidate.backend_kind.as_str(), "voxel" | "raytracing")
    {
        bail!("invalid Stage4 candidate schema or backend kind");
    }
    for (label, bytes) in [
        ("identity", &candidate.identity_json),
        ("georeference", &candidate.georeference_json),
        ("processing_receipt", &candidate.processing_receipt_json),
        ("map_snapshot_seed", &candidate.map_snapshot_seed_json),
        ("deferred", &candidate.deferred_json),
    ] {
        canonical_boundary_json(label, bytes)?;
    }
    let provenance = canonical_boundary_json("source_provenance", &candidate.source_provenance_json)?;
    let commit = source_commit(&provenance)?;
    let mut paths = Vec::new();
    for member in &candidate.members {
        validate_relative_path(&member.relative_path)?;
        let expected_media = match member.encoding.as_str() {
            "nav2_occupancy_yaml_v1" => ("application/x-yaml", "canonical_raw_occupancy"),
            "nav2_occupancy_pgm_p5_v1" => ("image/x-portable-graymap", "canonical_raw_occupancy"),
            "pcd_xyz_binary_v1" => (
                "application/vnd.pointclouds.pcd",
                "committed_transformed_accepted_cloud",
            ),
            "checkpoint_object_v1" => ("application/octet-stream", "mapping_backend_checkpoint"),
            _ => bail!("unsupported candidate encoding: {}", member.encoding),
        };
        if member.media_type != expected_media.0 || member.source_binding != expected_media.1 {
            bail!(
                "candidate media/source binding mismatch: {}",
                member.relative_path
            );
        }
        if member.size_bytes != member.bytes.len() as u64
            || canonical_boundary_json("source_commit_id", &member.source_commit_id_json)? != *commit
        {
            bail!(
                "candidate receipt does not match bytes: {}",
                member.relative_path
            );
        }
        paths.push(member.relative_path.as_str());
    }
    let mut checkpoint_iter = candidate
        .members
        .iter()
        .filter(|m| m.encoding == "checkpoint_object_v1");
    let checkpoint = checkpoint_iter
        .next()
        .ok_or_else(|| anyhow::anyhow!("candidate has no checkpoint object"))?;
    if checkpoint_iter.next().is_some() {
        bail!("candidate has more than one checkpoint object");
    }
    if checkpoint.relative_path != candidate.checkpoint.object_path
        || checkpoint.size_bytes != candidate.checkpoint.object_size_bytes
        || checkpoint.relative_path != format!("mapping_backend/{}/objects/checkpoint.bin", candidate.backend_kind)
    {
        bail!("checkpoint binding mismatch");
    }
    validate_relative_path(&candidate.checkpoint.manifest_path)?;
    validate_relative_path(&candidate.checkpoint.selector_path)?;
    paths.extend([
        candidate.checkpoint.manifest_path.as_str(),
        candidate.checkpoint.selector_path.as_str(),
        "manifest.json",
    ]);
    for (index, left) in paths.iter().enumerate() {
        for right in paths.iter().skip(index + 1) {
            if path_conflicts(left, right) {
                bail!("candidate paths conflict: {left} and {right}");
            }
        }
    }
    Ok(provenance)
}

fn source_commit(provenance: &Value) -> anyhow::Result<&Value> {
    let commit = provenance.get("commit_id")
        .ok_or_else(|| anyhow::anyhow!("source provenance commit_id is missing"))?;
    let nonce = commit.get("boot_nonce").and_then(Value::as_str)
        .ok_or_else(|| anyhow::anyhow!("source commit boot_nonce must be decimal text"))?;
    let parsed = nonce.parse::<u128>().context("invalid source commit boot_nonce")?;
    if parsed.to_string() != nonce || parsed == 0
        || commit.get("sequence").and_then(Value::as_u64).filter(|v| *v > 0).is_none()
        || provenance.pointer("/occupancy_receipt/commit_id") != Some(commit)
        || provenance.pointer("/cloud_receipt/commit_id") != Some(commit)
    {
        bail!("source provenance commit binding mismatch");
    }
    Ok(commit)
}

fn path_conflicts(left: &str, right: &str) -> bool {
    left == right
        || left
            .strip_prefix(right)
            .is_some_and(|tail| tail.starts_with('/'))
        || right
            .strip_prefix(left)
            .is_some_and(|tail| tail.starts_with('/'))
}

fn validate_relative_path(path: &str) -> anyhow::Result<()> {
    let p = Path::new(path);
    if path.is_empty()
        || p.is_absolute()
        || p.components()
            .any(|c| !matches!(c, std::path::Component::Normal(_)))
    {
        bail!("invalid relative candidate path: {path}");
    }
    Ok(())
}

fn materialize_offline_candidate(
    staging: &StableDirectory,
    candidate: &OfflineCandidateBundleV2,
    provenance: &Value,
) -> anyhow::Result<()> {
    for member in &candidate.members {
        staging.write_synced_beneath(Path::new(&member.relative_path), &member.bytes)?;
    }
    let object = candidate
        .members
        .iter()
        .find(|m| m.encoding == "checkpoint_object_v1")
        .unwrap();
    let commit = &provenance["commit_id"];
    let manifest = canonical_json_value(&json!({
        "schema_version": 2, "backend_kind": candidate.backend_kind,
        "processing_receipt": serde_json::from_slice::<Value>(&candidate.processing_receipt_json)?,
        "source_provenance": provenance,
        "object": {"path": object.relative_path, "media_type": object.media_type, "encoding": object.encoding,
            "source_binding": object.source_binding, "size": object.size_bytes, "source_commit_id": commit}
    }))?;
    staging.write_synced_beneath(Path::new(&candidate.checkpoint.manifest_path), &manifest)?;
    let selector = canonical_json_value(&json!({
        "schema_version": 2, "backend_kind": candidate.backend_kind,
        "source_commit_id": commit,
        "manifest": {"path": candidate.checkpoint.manifest_path}
    }))?;
    staging.write_synced_beneath(Path::new(&candidate.checkpoint.selector_path), &selector)?;
    let mut files = candidate
        .members
        .iter()
        .map(|member| {
            json!({
                "path": member.relative_path,
                "media_type": member.media_type,
                "encoding": member.encoding,
                "source_binding": member.source_binding,
                "size": member.size_bytes,
                "source_commit_id": commit,
            })
        })
        .collect::<Vec<_>>();
    files.push(json!({
        "path": candidate.checkpoint.manifest_path,
        "media_type": "application/json",
        "encoding": "canonical_json_v1",
        "source_binding": "stage4_checkpoint_manifest",
        "size": manifest.len(),
        "source_commit_id": commit,
    }));
    files.push(json!({
        "path": candidate.checkpoint.selector_path,
        "media_type": "application/json",
        "encoding": "canonical_json_v1",
        "source_binding": "stage4_checkpoint_selector",
        "size": selector.len(),
        "source_commit_id": commit,
    }));
    files.sort_by(|left, right| left["path"].as_str().cmp(&right["path"].as_str()));
    let top = canonical_json_value(&json!({
        "schema_version": 2, "map_name": candidate.map_name, "frame_id": candidate.frame_id,
        "backend_kind": candidate.backend_kind,
        "snapshot_schema_version": candidate.snapshot_schema_version,
        "last_source_timestamp_ns": candidate.last_source_timestamp_ns,
        "identity": serde_json::from_slice::<Value>(&candidate.identity_json)?,
        "georeference": serde_json::from_slice::<Value>(&candidate.georeference_json)?,
        "processing_receipt": serde_json::from_slice::<Value>(&candidate.processing_receipt_json)?,
        "source_provenance": provenance,
        "map_snapshot_seed": serde_json::from_slice::<Value>(&candidate.map_snapshot_seed_json)?,
        "deferred": serde_json::from_slice::<Value>(&candidate.deferred_json)?,
        "files": files,
        "checkpoint": {"selector_path": candidate.checkpoint.selector_path, "manifest_path": candidate.checkpoint.manifest_path,
            "object_path": object.relative_path, "source_commit_id": commit}
    }))?;
    staging.write_synced_beneath(Path::new("manifest.json"), &top)?;
    Ok(())
}

fn validate_offline_candidate_directory(
    root: &StableDirectory,
    candidate: &OfflineCandidateBundleV2,
    provenance: &Value,
) -> anyhow::Result<()> {
    // 原文件以 fresh no-follow FD 核尺寸；小型 JSON 仍读取并核对规范化来源绑定。
    let commit = &provenance["commit_id"];
    let mut expected = BTreeSet::new();
    for member in &candidate.members {
        expected.insert(member.relative_path.clone());
        validate_open_file(
            root,
            &member.relative_path,
            member.size_bytes,
        )?;
    }
    let object = candidate
        .members
        .iter()
        .find(|m| m.encoding == "checkpoint_object_v1")
        .ok_or_else(|| anyhow::anyhow!("candidate has no checkpoint object"))?;
    expected.insert(candidate.checkpoint.selector_path.clone());
    expected.insert(candidate.checkpoint.manifest_path.clone());

    let manifest_bytes = read_fresh_file(root, &candidate.checkpoint.manifest_path)?;
    let manifest: Value =
        serde_json::from_slice(&manifest_bytes).context("checkpoint manifest is not JSON")?;
    if canonical_json_value(&manifest)? != manifest_bytes {
        bail!("checkpoint manifest is not canonical JSON");
    }
    if manifest.get("schema_version") != Some(&json!(2))
        || manifest.get("backend_kind") != Some(&json!(candidate.backend_kind))
        || manifest.get("processing_receipt")
            != Some(&canonical_boundary_json(
                "processing_receipt",
                &candidate.processing_receipt_json,
            )?)
        || manifest.pointer("/object/path").and_then(Value::as_str) != Some(&object.relative_path)
        || manifest
            .pointer("/object/media_type")
            .and_then(Value::as_str)
            != Some(object.media_type.as_str())
        || manifest.pointer("/object/encoding").and_then(Value::as_str)
            != Some(object.encoding.as_str())
        || manifest
            .pointer("/object/source_binding")
            .and_then(Value::as_str)
            != Some(object.source_binding.as_str())
        || manifest.pointer("/object/size").and_then(Value::as_u64) != Some(object.size_bytes)
        || manifest.get("source_provenance") != Some(provenance)
        || manifest.pointer("/object/source_commit_id") != Some(commit)
    {
        bail!("checkpoint manifest binding mismatch");
    }

    let selector_bytes = read_fresh_file(root, &candidate.checkpoint.selector_path)?;
    let selector: Value =
        serde_json::from_slice(&selector_bytes).context("checkpoint selector is not JSON")?;
    if canonical_json_value(&selector)? != selector_bytes
        || selector.get("schema_version") != Some(&json!(2))
        || selector.get("backend_kind") != Some(&json!(candidate.backend_kind))
        || selector.pointer("/manifest/path").and_then(Value::as_str)
            != Some(&candidate.checkpoint.manifest_path)
        || selector.get("source_commit_id") != Some(commit)
    {
        bail!("checkpoint selector binding mismatch");
    }

    let top_bytes = read_fresh_file(root, "manifest.json")?;
    let top: Value =
        serde_json::from_slice(&top_bytes).context("top-level manifest is not JSON")?;
    if canonical_json_value(&top)? != top_bytes
        || top.get("schema_version") != Some(&json!(2))
        || top.get("map_name") != Some(&json!(candidate.map_name))
        || top.get("frame_id") != Some(&json!(candidate.frame_id))
        || top.get("backend_kind") != Some(&json!(candidate.backend_kind))
        || top.get("snapshot_schema_version") != Some(&json!(candidate.snapshot_schema_version))
        || top.get("last_source_timestamp_ns") != Some(&json!(candidate.last_source_timestamp_ns))
        || top.get("identity")
            != Some(&canonical_boundary_json(
                "identity",
                &candidate.identity_json,
            )?)
        || top.get("georeference")
            != Some(&canonical_boundary_json(
                "georeference",
                &candidate.georeference_json,
            )?)
        || top.get("processing_receipt")
            != Some(&canonical_boundary_json(
                "processing_receipt",
                &candidate.processing_receipt_json,
            )?)
        || top.get("map_snapshot_seed")
            != Some(&canonical_boundary_json(
                "map_snapshot_seed",
                &candidate.map_snapshot_seed_json,
            )?)
        || top.get("deferred")
            != Some(&canonical_boundary_json(
                "deferred",
                &candidate.deferred_json,
            )?)
        || top
            .pointer("/checkpoint/selector_path")
            .and_then(Value::as_str)
            != Some(&candidate.checkpoint.selector_path)
        || top
            .pointer("/checkpoint/manifest_path")
            .and_then(Value::as_str)
            != Some(&candidate.checkpoint.manifest_path)
        || top
            .pointer("/checkpoint/object_path")
            .and_then(Value::as_str)
            != Some(&object.relative_path)
        || top.get("source_provenance") != Some(provenance)
        || top.pointer("/checkpoint/source_commit_id") != Some(commit)
    {
        bail!("top-level manifest binding mismatch");
    }

    let listed = top
        .get("files")
        .and_then(Value::as_array)
        .ok_or_else(|| anyhow::anyhow!("top-level files is not an array"))?;
    let mut listed_paths = BTreeSet::new();
    for entry in listed {
        let path = entry
            .get("path")
            .and_then(Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("invalid top-level file entry"))?;
        let size = entry
            .get("size")
            .and_then(Value::as_u64)
            .ok_or_else(|| anyhow::anyhow!("invalid top-level file size"))?;
        if entry.get("source_commit_id") != Some(commit) {
            bail!("top-level file source commit mismatch");
        }
        let media_type = entry
            .get("media_type")
            .and_then(Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("invalid top-level file media_type"))?;
        let encoding = entry
            .get("encoding")
            .and_then(Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("invalid top-level file encoding"))?;
        let source_binding = entry
            .get("source_binding")
            .and_then(Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("invalid top-level file source_binding"))?;
        if !listed_paths.insert(path.to_owned())
            || !expected.contains(path)
            || path == "manifest.json"
        {
            bail!("invalid top-level file set");
        }
        if let Some(member) = candidate
            .members
            .iter()
            .find(|member| member.relative_path == path)
        {
            if media_type != member.media_type
                || encoding != member.encoding
                || source_binding != member.source_binding
                || size != member.size_bytes
            {
                bail!("top-level member receipt mismatch: {path}");
            }
        } else {
            let expected_generated = if path == candidate.checkpoint.manifest_path {
                ("stage4_checkpoint_manifest", manifest_bytes.as_slice())
            } else if path == candidate.checkpoint.selector_path {
                ("stage4_checkpoint_selector", selector_bytes.as_slice())
            } else {
                bail!("unexpected generated file entry: {path}");
            };
            if media_type != "application/json"
                || encoding != "canonical_json_v1"
                || source_binding != expected_generated.0
                || size != expected_generated.1.len() as u64
            {
                bail!("top-level generated receipt mismatch: {path}");
            }
        }
    }
    if listed_paths != expected {
        bail!("top-level file set does not match candidate");
    }
    Ok(())
}

fn read_fresh_file(root: &StableDirectory, relative: &str) -> anyhow::Result<Vec<u8>> {
    validate_relative_path(relative)?;
    let mut file = root.open_beneath(Path::new(relative), false)?;
    let metadata = file.metadata()?;
    if !metadata.is_file() {
        bail!("candidate member is not a regular file: {relative}");
    }
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    if file.metadata()?.len() != bytes.len() as u64 {
        bail!("candidate member changed while being read: {relative}");
    }
    Ok(bytes)
}

fn validate_open_file(
    root: &StableDirectory,
    relative: &str,
    expected_size: u64,
) -> anyhow::Result<()> {
    validate_relative_path(relative)?;
    let file = root.open_beneath(Path::new(relative), false)?;
    let metadata = file.metadata()?;
    if !metadata.is_file() || metadata.len() != expected_size {
        bail!("fresh validation failed: {relative}");
    }
    Ok(())
}

fn canonical_json_value(value: &Value) -> anyhow::Result<Vec<u8>> {
    fn sort(value: &Value) -> Value {
        match value {
            Value::Object(m) => {
                let mut keys: Vec<_> = m.keys().collect();
                keys.sort();
                let mut out = Map::new();
                for k in keys {
                    out.insert(k.clone(), sort(&m[k]));
                }
                Value::Object(out)
            }
            Value::Array(a) => Value::Array(a.iter().map(sort).collect()),
            _ => value.clone(),
        }
    }
    let mut bytes = serde_json::to_vec(&sort(value))?;
    bytes.push(b'\n');
    Ok(bytes)
}

pub(crate) fn publish_map_bundle<B, V>(
    final_directory: &Path,
    build: B,
    validate: V,
) -> anyhow::Result<()>
where
    B: FnOnce(&Path) -> anyhow::Result<()>,
    V: FnOnce(&Path) -> anyhow::Result<()>,
{
    publish_map_bundle_stable(
        final_directory,
        |staging| build(&staging.descriptor_path()),
        |staging| {
            validate(&staging.descriptor_path())?;
            Ok(())
        },
    )
}

fn publish_map_bundle_stable<B, V, R>(
    final_directory: &Path,
    build: B,
    validate: V,
) -> anyhow::Result<R>
where
    B: FnOnce(&StableDirectory) -> anyhow::Result<()>,
    V: FnOnce(&StableDirectory) -> anyhow::Result<R>,
{
    let final_name = final_directory
        .file_name()
        .ok_or_else(|| anyhow::anyhow!("map bundle output has no directory name"))?;
    let parent = parent_directory(final_directory);
    let recovery_policy = RecoveryPolicy::acquire()?;
    fs::create_dir_all(parent)?;
    let directory = StableDirectory::open(parent)?;
    recover_stale_map_bundle_stages(&directory, final_name, final_directory, &recovery_policy)?;
    if directory.entry_exists(final_name)? {
        return Err(MapBundlePublicationError::Conflict {
            path: final_directory.to_path_buf(),
            cleanup: None,
        }
        .into());
    }
    let area = StagingArea::reserve(&directory, final_name, "bundle", &recovery_policy)?;
    let staging_name = payload_name(final_name, None);
    area.contents.create_directory(&staging_name)?;
    let staging_directory = area.contents.open_stable_directory(&staging_name)?;
    let staging = staging_directory.descriptor_path();

    let operation = build(&staging_directory)
        .and_then(|()| validate(&staging_directory))
        .and_then(|result| {
            staging_directory.sync_tree()?;
            Ok(result)
        });
    let result = match operation {
        Ok(result) => result,
        Err(primary) => {
            return match area.cleanup(&directory, &staging_name, None) {
                Ok(()) => Err(primary),
                Err(source) => Err(MapBundlePublicationError::CleanupFailure {
                    path: final_directory.to_path_buf(),
                    primary,
                    cleanup: ArtifactCleanupFailure {
                        staging: staging.clone(),
                        source,
                    },
                }
                .into()),
            };
        }
    };

    area.require_bound(&directory)?;
    let named = area.contents.open_stable_directory(&staging_name)?;
    if !same_file_identity(&staging_directory.metadata()?, &named.metadata()?) {
        let primary = anyhow::anyhow!("staging directory inode changed before publication");
        return match area.cleanup(&directory, &staging_name, None) {
            Ok(()) => Err(primary),
            Err(source) => Err(MapBundlePublicationError::CleanupFailure {
                path: final_directory.to_path_buf(),
                primary,
                cleanup: ArtifactCleanupFailure {
                    staging: staging.clone(),
                    source,
                },
            }
            .into()),
        };
    }
    match rename_directory_no_replace_at(&area.contents, &staging_name, &directory, final_name) {
        Ok(()) => {}
        Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {
            let cleanup = area.cleanup(&directory, &staging_name, None)
                .err()
                .map(|source| ArtifactCleanupFailure {
                    staging: staging.clone(),
                    source,
                });
            return Err(MapBundlePublicationError::Conflict {
                path: final_directory.to_path_buf(),
                cleanup,
            }
            .into());
        }
        Err(primary) => {
            return match area.cleanup(&directory, &staging_name, None) {
                Ok(()) => Err(primary.into()),
                Err(source) => Err(MapBundlePublicationError::CleanupFailure {
                    path: final_directory.to_path_buf(),
                    primary: primary.into(),
                    cleanup: ArtifactCleanupFailure {
                        staging: staging.clone(),
                        source,
                    },
                }
                .into()),
            };
        }
    }
    let published = directory.open_stable_directory(final_name)?;
    if !same_file_identity(&staging_directory.metadata()?, &published.metadata()?) {
        let primary = anyhow::anyhow!("published directory is not the validated staging inode");
        let rollback = directory
            .remove_tree_bound(final_name, &published)
            .and_then(|()| directory.sync());
        return match rollback {
            Ok(()) => Err(primary),
            Err(source) => Err(MapBundlePublicationError::RollbackFailure {
                path: final_directory.to_path_buf(),
                primary,
                rollback: ArtifactCleanupFailure {
                    staging: directory.child_path(final_name),
                    source,
                },
            }
            .into()),
        };
    }
    area.contents.sync().and_then(|()| area.wrapper.sync()).and_then(|()| directory.sync())
        .map_err(|source| MapBundlePublicationError::DurabilityUnknown {
            path: final_directory.to_path_buf(),
            source,
        })?;
    area.cleanup_empty(&directory).map_err(|source|
        MapBundlePublicationError::PublishedCleanupFailure {
            path: final_directory.to_path_buf(),
            cleanup: ArtifactCleanupFailure { staging, source },
        })?;
    Ok(result)
}

// A short, per-attempt owner wrapper carries the original name as an OsStr
// directory entry. Payload basenames stay short even for SQLite sidecars.
struct StagingArea {
    wrapper_name: OsString,
    final_name: OsString,
    wrapper: StableDirectory,
    contents: StableDirectory,
}

impl StagingArea {
    fn reserve(parent: &StableDirectory, final_name: &OsStr, kind: &str,
               policy: &RecoveryPolicy) -> anyhow::Result<Self> {
        for _ in 0..MAX_STAGE_ATTEMPTS {
            let sequence = NEXT_STAGE.fetch_add(1, Ordering::Relaxed);
            let name = OsString::from(format!("{STAGE_DOMAIN}{kind}-{}-{:016x}-{sequence:016x}",
                std::process::id(), policy.current_process_start()));
            if name == final_name { continue; }
            match parent.create_directory_mode(&name, 0o700) {
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
                Err(error) => return Err(error.into()),
                Ok(()) => {}
            }
            let wrapper = parent.open_stable_directory(&name)?;
            wrapper.create_directory_mode(final_name, 0o700)?;
            let contents = wrapper.open_stable_directory(final_name)?;
            contents.sync()?;
            wrapper.sync()?;
            parent.sync()?;
            return Ok(Self { wrapper_name: name, final_name: final_name.to_os_string(), wrapper, contents });
        }
        bail!("could not reserve a unique staging directory after {} attempts", MAX_STAGE_ATTEMPTS)
    }

    fn require_bound(&self, parent: &StableDirectory) -> io::Result<()> {
        parent.require_named_identity(&self.wrapper_name, &self.wrapper)?;
        self.wrapper.require_named_identity(&self.final_name, &self.contents)
    }

    fn has_expected_shape(&self, payload: &OsStr, format: Option<ArtifactFormat>) -> io::Result<bool> {
        let mut entries = self.wrapper.read_dir()?;
        if entries.next().transpose()?.map(|entry| entry.file_name()).as_deref() != Some(self.final_name.as_os_str())
            || entries.next().transpose()?.is_some() { return Ok(false); }
        for entry in self.contents.read_dir()? {
            let entry = entry?;
            let name = entry.file_name();
            let kind = entry.file_type()?;
            if name == payload {
                if format.is_some() && !kind.is_file() || format.is_none() && !kind.is_dir() { return Ok(false); }
            } else if !format.is_some_and(|format| format.sqlite_sidecars().iter()
                .any(|suffix| name == append_name_suffix(payload, suffix))) || !kind.is_file() {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn cleanup(&self, parent: &StableDirectory, payload: &OsStr, format: Option<ArtifactFormat>) -> io::Result<()> {
        self.require_bound(parent)?;
        if !self.has_expected_shape(payload, format)? {
            return Err(io::Error::new(io::ErrorKind::Other, "staging wrapper contains unknown entries; retained"));
        }
        if let Some(format) = format {
            cleanup_stage(&self.contents, payload, format)?;
        } else if self.contents.entry_exists(payload)? {
            let child = self.contents.open_stable_directory(payload)?;
            self.contents.remove_tree_bound(payload, &child)?;
            self.contents.sync()?;
        }
        self.cleanup_empty(parent)
    }

    fn cleanup_empty(&self, parent: &StableDirectory) -> io::Result<()> {
        self.require_bound(parent)?;
        self.wrapper.remove_directory(&self.final_name)?;
        self.wrapper.sync()?;
        parent.require_named_identity(&self.wrapper_name, &self.wrapper)?;
        parent.remove_directory(&self.wrapper_name)?;
        parent.sync()
    }
}

fn parse_wrapper_name(name: &str, kind: &str) -> Option<StageOwner> {
    let prefix = format!("{STAGE_DOMAIN}{kind}-");
    let mut parts = name.strip_prefix(&prefix)?.split('-');
    let pid_text = parts.next()?;
    let process_id: u32 = pid_text.parse().ok()?;
    let process_start = parse_exact_lower_hex_u64(parts.next()?)?;
    parse_exact_lower_hex_u64(parts.next()?)?;
    if parts.next().is_some() || process_id == 0 || process_id > i32::MAX as u32
        || process_id.to_string() != pid_text || process_start == 0 { return None; }
    Some(StageOwner { process_id, process_start })
}

fn recover_stale_wrappers(
    parent: &StableDirectory, final_name: &OsStr, final_path: &Path,
    format: Option<ArtifactFormat>, owner_is_proven_dead: impl Fn(StageOwner) -> bool,
) -> anyhow::Result<()> {
    let kind = format.map_or("bundle", ArtifactFormat::stage_label);
    let payload = payload_name(final_name, format);
    let mut stages = Vec::new();
    for entry in parent.read_dir()? {
        let entry = entry?;
        let name = entry.file_name();
        let Some(owner) = name.to_str().and_then(|name| parse_wrapper_name(name, kind)) else { continue; };
        // Empty, foreign, legacy, non-directory and unbound wrappers are retained.
        let Ok(wrapper) = parent.open_stable_directory(&name) else { continue; };
        let Ok(contents) = wrapper.open_stable_directory(final_name) else { continue; };
        let area = StagingArea { wrapper_name: name, final_name: final_name.to_os_string(), wrapper, contents };
        if !area.has_expected_shape(&payload, format)? { continue; }
        stages.push((area, owner));
        if stages.len() > MAX_STAGES_PER_ARTIFACT {
            bail!("more than {} staging artifacts exist for {}; refusing unbounded cleanup", MAX_STAGES_PER_ARTIFACT, final_path.display());
        }
    }
    for (area, owner) in stages {
        if owner_is_proven_dead(owner) { area.cleanup(parent, &payload, format)?; }
    }
    Ok(())
}

fn payload_name(final_name: &OsStr, format: Option<ArtifactFormat>) -> OsString {
    if format.is_some() { OsString::from(format!("payload.tmp{}", staging_extension_from_name(final_name))) }
    else { OsString::from("payload") }
}

fn recover_stale_map_bundle_stages(parent: &StableDirectory, final_name: &OsStr,
    final_directory: &Path, policy: &RecoveryPolicy) -> anyhow::Result<()> {
    recover_stale_wrappers(parent, final_name, final_directory, None, |owner| policy.owner_is_proven_dead(owner))
}

#[cfg(target_os = "linux")]
fn rename_directory_no_replace_at(
    directory: &StableDirectory,
    source: &OsStr,
    destination_directory: &StableDirectory,
    destination: &OsStr,
) -> io::Result<()> {
    use std::ffi::CString;
    use std::os::fd::AsRawFd;
    use std::os::raw::{c_char, c_int};
    use std::os::unix::ffi::OsStrExt;
    extern "C" {
        fn renameat2(
            olddirfd: c_int,
            oldpath: *const c_char,
            newdirfd: c_int,
            newpath: *const c_char,
            flags: u32,
        ) -> c_int;
    }
    let source = CString::new(source.as_bytes())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "source contains NUL"))?;
    let destination = CString::new(destination.as_bytes())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "destination contains NUL"))?;
    let result = unsafe {
        renameat2(
            directory.file.as_raw_fd(),
            source.as_ptr(),
            destination_directory.file.as_raw_fd(),
            destination.as_ptr(),
            1,
        )
    };
    if result == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}

#[cfg(not(target_os = "linux"))]
fn rename_directory_no_replace_at(
    _directory: &StableDirectory,
    _source: &OsStr,
    _destination_directory: &StableDirectory,
    _destination: &OsStr,
) -> io::Result<()> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "map bundle publication requires Linux renameat2",
    ))
}

/// Build and publish a new regular file through the shared durable boundary.
///
/// The writer receives a buffered sibling staging file. The validator runs
/// only after the buffer is flushed and the staging inode is synced.
pub(crate) fn publish_new_file<W, V>(
    final_path: &Path,
    format: ArtifactFormat,
    write: W,
    validate: V,
) -> anyhow::Result<()>
where
    W: FnOnce(&mut BufWriter<File>) -> anyhow::Result<()>,
    V: FnOnce(&Path) -> anyhow::Result<()>,
{
    publish_artifact(
        final_path,
        format,
        |staging, reservation| {
            let mut writer = BufWriter::new(reservation);
            write(&mut writer)?;
            writer
                .flush()
                .with_context(|| format!("flush staging artifact {}", staging.display()))?;
            writer
                .into_inner()
                .map_err(std::io::IntoInnerError::into_error)
                .with_context(|| format!("finish staging artifact {}", staging.display()))?;
            Ok(())
        },
        validate,
    )
}

/// Build and publish a new path-based artifact through the shared durable
/// boundary.
///
/// Path-based builders are used for SQLite, which must close its connection
/// before the file can be reopened, validated, and synced. The initially
/// reserved file belongs to the builder and may be dropped before reopening
/// `staging`.
pub(crate) fn publish_new_artifact<B, V>(
    final_path: &Path,
    build: B,
    validate: V,
) -> anyhow::Result<()>
where
    B: FnOnce(&Path, File) -> anyhow::Result<()>,
    V: FnOnce(&Path) -> anyhow::Result<()>,
{
    publish_artifact(final_path, ArtifactFormat::Memory2Sqlite, build, validate)
}

fn publish_artifact<B, V>(
    final_path: &Path,
    format: ArtifactFormat,
    build: B,
    validate: V,
) -> anyhow::Result<()>
where
    B: FnOnce(&Path, File) -> anyhow::Result<()>,
    V: FnOnce(&Path) -> anyhow::Result<()>,
{
    let parent = parent_directory(final_path);
    let final_name = final_path
        .file_name()
        .ok_or_else(|| {
            anyhow::anyhow!("artifact output has no file name: {}", final_path.display())
        })?
        .to_os_string();
    let extension = final_path.extension().and_then(|value| value.to_str());
    if !format.accepts_extension(extension) {
        bail!(
            "{} output must use {} extension: {}",
            format.stage_label(),
            format.expected_extension(),
            final_path.display()
        );
    }
    let recovery_policy = RecoveryPolicy::acquire()?;
    let directory = StableDirectory::open(parent)
        .with_context(|| format!("open stable output directory {}", parent.display()))?;

    recover_stale_stages(
        &directory,
        &final_name,
        final_path,
        format,
        &recovery_policy,
    )?;
    require_absent(&directory, &final_name, final_path)?;

    let area = StagingArea::reserve(&directory, &final_name, format.stage_label(), &recovery_policy)?;
    let staging_name = payload_name(&final_name, Some(format));
    let staging_path = area.contents.child_path(&staging_name);
    let reservation = area.contents.create_new(&staging_name)?;
    let reserved_metadata = reservation.metadata()?;
    if let Err(error) = build(&staging_path, reservation) {
        return Err(cleanup_failed_build(
            &directory,
            &area,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = verify_staging_file(&area.contents, &staging_name, &reserved_metadata) {
        return Err(cleanup_failed_build(
            &directory,
            &area,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = reject_sqlite_sidecars(&area.contents, &staging_name, format) {
        return Err(cleanup_failed_build(
            &directory,
            &area,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = sync_staging_file(&area.contents, &staging_name, &staging_path) {
        return Err(cleanup_failed_build(
            &directory,
            &area,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = validate(&staging_path)
        .with_context(|| format!("validate staging artifact {}", staging_path.display()))
    {
        return Err(cleanup_failed_build(
            &directory,
            &area,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = verify_staging_file(&area.contents, &staging_name, &reserved_metadata) {
        return Err(cleanup_failed_build(
            &directory,
            &area,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = reject_sqlite_sidecars(&area.contents, &staging_name, format) {
        return Err(cleanup_failed_build(
            &directory,
            &area,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }

    area.require_bound(&directory)?;
    match area.contents.link_no_replace_to(&staging_name, &directory, &final_name) {
        Ok(()) => {}
        Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {
            return Err(conflict_after_cleanup(
                final_path,
                &staging_path,
                area.cleanup(&directory, &staging_name, Some(format)),
            ));
        }
        Err(error) => {
            if directory.entry_exists(&final_name)? {
                return Err(conflict_after_cleanup(
                    final_path,
                    &staging_path,
                    area.cleanup(&directory, &staging_name, Some(format)),
                ));
            }
            return Err(cleanup_failed_build(
                &directory,
                &area,
                &staging_name,
                &staging_path,
                format,
                anyhow::Error::new(error).context(format!(
                    "atomically publish {} as {}",
                    staging_path.display(),
                    final_path.display()
                )),
            ));
        }
    }

    if let Err(source) = area.contents.sync().and_then(|()| area.wrapper.sync()).and_then(|()| directory.sync()) {
        return Err(ArtifactPublicationError::DurabilityUnknown {
            path: final_path.to_path_buf(),
            staging: staging_path,
            source,
        }
        .into());
    }

    if let Err(source) = area.cleanup(&directory, &staging_name, Some(format)) {
        return Err(ArtifactPublicationError::CleanupFailure {
            path: final_path.to_path_buf(),
            cleanup: ArtifactCleanupFailure {
                staging: staging_path,
                source,
            },
        }
        .into());
    }
    Ok(())
}

fn conflict_after_cleanup(
    final_path: &Path,
    staging_path: &Path,
    cleanup_result: io::Result<()>,
) -> anyhow::Error {
    ArtifactPublicationError::Conflict {
        path: final_path.to_path_buf(),
        cleanup: cleanup_result.err().map(|source| ArtifactCleanupFailure {
            staging: staging_path.to_path_buf(),
            source,
        }),
    }
    .into()
}

fn require_absent(
    directory: &StableDirectory,
    final_name: &OsStr,
    final_path: &Path,
) -> anyhow::Result<()> {
    if directory.entry_exists(final_name)? {
        return Err(ArtifactPublicationError::Conflict {
            path: final_path.to_path_buf(),
            cleanup: None,
        }
        .into());
    }
    Ok(())
}

fn verify_staging_file(
    directory: &StableDirectory,
    staging_name: &OsStr,
    reserved: &Metadata,
) -> anyhow::Result<()> {
    let path = directory.child_path(staging_name);
    let metadata = directory
        .open_read(staging_name)
        .with_context(|| format!("open completed staging artifact {}", path.display()))?
        .metadata()
        .with_context(|| format!("inspect completed staging artifact {}", path.display()))?;
    if !metadata.file_type().is_file() {
        bail!("staging artifact is not a regular file: {}", path.display());
    }
    if metadata.len() == 0 {
        bail!("staging artifact is empty: {}", path.display());
    }
    if !same_file_identity(reserved, &metadata) {
        bail!(
            "staging artifact identity changed while building: {}",
            path.display()
        );
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        if metadata.nlink() != 1 {
            bail!(
                "staging artifact unexpectedly has {} links: {}",
                metadata.nlink(),
                path.display()
            );
        }
    }
    Ok(())
}

#[cfg(unix)]
fn same_file_identity(left: &Metadata, right: &Metadata) -> bool {
    use std::os::unix::fs::MetadataExt;
    left.dev() == right.dev() && left.ino() == right.ino()
}

#[cfg(not(unix))]
fn same_file_identity(_left: &Metadata, _right: &Metadata) -> bool {
    true
}

fn sync_staging_file(
    directory: &StableDirectory,
    staging_name: &OsStr,
    staging_path: &Path,
) -> anyhow::Result<()> {
    directory
        .open_read(staging_name)
        .with_context(|| format!("reopen staging artifact {}", staging_path.display()))?
        .sync_all()
        .with_context(|| format!("sync staging artifact {}", staging_path.display()))
}

fn reject_sqlite_sidecars(
    directory: &StableDirectory,
    staging_name: &OsStr,
    format: ArtifactFormat,
) -> anyhow::Result<()> {
    for suffix in format.sqlite_sidecars() {
        let sidecar_name = append_name_suffix(staging_name, suffix);
        if directory.entry_exists(&sidecar_name)? {
            bail!(
                "staging artifact left an unpublished SQLite sidecar: {}",
                directory.child_path(&sidecar_name).display()
            );
        }
    }
    Ok(())
}

fn cleanup_failed_build(
    directory: &StableDirectory,
    area: &StagingArea,
    staging_name: &OsStr,
    staging_path: &Path,
    format: ArtifactFormat,
    error: anyhow::Error,
) -> anyhow::Error {
    match area.cleanup(directory, staging_name, Some(format)) {
        Ok(()) => error,
        Err(cleanup_error) => error.context(format!(
            "also failed to clean staging artifact {}: {cleanup_error}",
            staging_path.display()
        )),
    }
}

fn cleanup_stage(
    directory: &StableDirectory,
    staging_name: &OsStr,
    format: ArtifactFormat,
) -> io::Result<()> {
    let mut removed = directory.remove_if_present(staging_name)?;
    for suffix in format.sqlite_sidecars() {
        removed |= directory.remove_if_present(&append_name_suffix(staging_name, suffix))?;
    }
    if removed {
        directory.sync()?;
    }
    Ok(())
}

fn recover_stale_stages(
    directory: &StableDirectory,
    final_name: &OsStr,
    final_path: &Path,
    format: ArtifactFormat,
    recovery_policy: &RecoveryPolicy,
) -> anyhow::Result<()> {
    recover_stale_stages_with(directory, final_name, final_path, format, |owner| {
        recovery_policy.owner_is_proven_dead(owner)
    })
}

fn recover_stale_stages_with(
    directory: &StableDirectory,
    final_name: &OsStr,
    final_path: &Path,
    format: ArtifactFormat,
    owner_is_proven_dead: impl Fn(StageOwner) -> bool,
) -> anyhow::Result<()> {
    recover_stale_wrappers(directory, final_name, final_path, Some(format), owner_is_proven_dead)
}

#[derive(Clone, Copy)]
struct StageOwner {
    process_id: u32,
    process_start: u64,
}

fn parse_exact_lower_hex_u64(text: &str) -> Option<u64> {
    if text.len() != 16
        || !text
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return None;
    }
    u64::from_str_radix(text, 16).ok()
}

#[cfg(target_os = "linux")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProcessProbe {
    Running(u64),
    Absent,
    Unverifiable,
}

#[derive(Debug, Clone, Copy)]
enum RecoveryPolicy {
    #[cfg(target_os = "linux")]
    Linux { current_process_start: u64 },
    #[cfg(not(target_os = "linux"))]
    NoAutomaticReaping,
}

impl RecoveryPolicy {
    fn acquire() -> anyhow::Result<Self> {
        #[cfg(target_os = "linux")]
        {
            Self::from_current_process_probe(linux_process_probe(std::process::id()))
        }
        #[cfg(not(target_os = "linux"))]
        {
            Ok(Self::NoAutomaticReaping)
        }
    }

    #[cfg(target_os = "linux")]
    fn from_current_process_probe(probe: ProcessProbe) -> anyhow::Result<Self> {
        match probe {
            ProcessProbe::Running(current_process_start) if current_process_start != 0 => {
                Ok(Self::Linux {
                    current_process_start,
                })
            }
            probe => bail!(
                "cannot prove current Linux process start identity ({probe:?}); refusing artifact publication before file side effects"
            ),
        }
    }

    fn current_process_start(self) -> u64 {
        match self {
            #[cfg(target_os = "linux")]
            Self::Linux {
                current_process_start,
            } => current_process_start,
            #[cfg(not(target_os = "linux"))]
            Self::NoAutomaticReaping => 0,
        }
    }

    fn owner_is_proven_dead(self, owner: StageOwner) -> bool {
        match self {
            #[cfg(target_os = "linux")]
            Self::Linux { .. } => {
                owner_is_proven_dead_from_probe(owner, linux_process_probe(owner.process_id))
            }
            #[cfg(not(target_os = "linux"))]
            Self::NoAutomaticReaping => false,
        }
    }
}

#[cfg(target_os = "linux")]
fn owner_is_proven_dead_from_probe(owner: StageOwner, probe: ProcessProbe) -> bool {
    match probe {
        ProcessProbe::Running(start) => start != owner.process_start,
        ProcessProbe::Absent => true,
        ProcessProbe::Unverifiable => false,
    }
}

#[cfg(target_os = "linux")]
fn linux_process_probe(process_id: u32) -> ProcessProbe {
    let process_directory = Path::new("/proc").join(process_id.to_string());
    match fs::read_to_string(process_directory.join("stat")) {
        Ok(text) => parse_linux_process_start(&text)
            .map_or(ProcessProbe::Unverifiable, ProcessProbe::Running),
        Err(error) if error.kind() == io::ErrorKind::NotFound => {
            match fs::symlink_metadata(&process_directory) {
                Err(directory_error) if directory_error.kind() == io::ErrorKind::NotFound => {
                    ProcessProbe::Absent
                }
                Ok(_) | Err(_) => ProcessProbe::Unverifiable,
            }
        }
        Err(_) => ProcessProbe::Unverifiable,
    }
}

#[cfg(target_os = "linux")]
fn parse_linux_process_start(text: &str) -> Option<u64> {
    let close = text.rfind(')')?;
    let start = text
        .get(close + 1..)?
        .split_whitespace()
        .nth(19)?
        .parse()
        .ok()?;
    (start != 0).then_some(start)
}

#[cfg(target_os = "linux")]
fn directory_or_regular_file<T>(opened: io::Result<T>) -> io::Result<Option<T>> {
    match opened {
        Ok(directory) => Ok(Some(directory)),
        Err(error) if matches!(error.raw_os_error(), Some(20)) => Ok(None),
        Err(error) => Err(error),
    }
}

#[cfg(target_os = "linux")]
pub(crate) struct StableDirectory {
    file: File,
}

#[cfg(target_os = "linux")]
impl StableDirectory {
    fn open(path: &Path) -> anyhow::Result<Self> {
        use std::ffi::CString;
        use std::os::raw::{c_char, c_int, c_long};
        use std::os::unix::ffi::OsStrExt;
        use std::os::unix::io::FromRawFd;

        const AT_FDCWD: c_int = -100;
        const O_CLOEXEC: u64 = 0o2_000_000;
        const O_DIRECTORY: u64 = 0o200_000;
        const RESOLVE_NO_MAGICLINKS: u64 = 0x02;
        const RESOLVE_NO_SYMLINKS: u64 = 0x04;
        const SYS_OPENAT2: c_long = 437;

        #[repr(C)]
        struct OpenHow {
            flags: u64,
            mode: u64,
            resolve: u64,
        }

        extern "C" {
            fn syscall(number: c_long, ...) -> c_long;
        }

        let path = CString::new(path.as_os_str().as_bytes())
            .context("output directory path contains NUL")?;
        let how = OpenHow {
            flags: O_CLOEXEC | O_DIRECTORY,
            mode: 0,
            resolve: RESOLVE_NO_MAGICLINKS | RESOLVE_NO_SYMLINKS,
        };
        // SAFETY: `path` and `how` remain live for the syscall, and a
        // non-negative result is an exclusively owned directory descriptor.
        let descriptor = unsafe {
            syscall(
                SYS_OPENAT2,
                c_long::from(AT_FDCWD),
                path.as_ptr().cast::<c_char>(),
                &how as *const OpenHow,
                std::mem::size_of::<OpenHow>(),
            )
        };
        if descriptor < 0 {
            return Err(io::Error::last_os_error())
                .context("openat2 output parent with RESOLVE_NO_MAGICLINKS|RESOLVE_NO_SYMLINKS");
        }
        let descriptor = c_int::try_from(descriptor).context("openat2 descriptor exceeds c_int")?;
        // SAFETY: successful openat2 returned one owned descriptor.
        let file = unsafe { File::from_raw_fd(descriptor) };
        let directory = Self { file };
        let descriptor_path = directory.descriptor_path();
        let via_descriptor = File::open(&descriptor_path).with_context(|| {
            format!(
                "open stable descriptor path {}; /proc/self/fd is required",
                descriptor_path.display()
            )
        })?;
        if !same_file_identity(&directory.file.metadata()?, &via_descriptor.metadata()?)
            || !via_descriptor.metadata()?.is_dir()
        {
            bail!(
                "stable output directory descriptor identity check failed: {}",
                descriptor_path.display()
            );
        }
        Ok(directory)
    }

    pub(crate) fn descriptor_path(&self) -> PathBuf {
        use std::os::fd::AsRawFd;
        Path::new("/proc/self/fd").join(self.file.as_raw_fd().to_string())
    }

    fn child_path(&self, name: &OsStr) -> PathBuf {
        self.descriptor_path().join(name)
    }

    fn create_new(&self, name: &OsStr) -> io::Result<File> {
        const O_CLOEXEC: i32 = 0o2_000_000;
        const O_CREAT: i32 = 0o100;
        const O_EXCL: i32 = 0o200;
        const O_NOFOLLOW: i32 = 0o400_000;
        const O_RDWR: i32 = 0o2;
        self.open_at(
            name,
            O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
            Some(0o666),
        )
    }

    fn open_read(&self, name: &OsStr) -> io::Result<File> {
        const O_CLOEXEC: i32 = 0o2_000_000;
        const O_NOFOLLOW: i32 = 0o400_000;
        self.open_at(name, O_CLOEXEC | O_NOFOLLOW, None)
    }

    fn open_directory(&self, name: &OsStr) -> io::Result<File> {
        const O_CLOEXEC: i32 = 0o2_000_000;
        const O_DIRECTORY: i32 = 0o200_000;
        const O_NOFOLLOW: i32 = 0o400_000;
        self.open_at(name, O_CLOEXEC | O_DIRECTORY | O_NOFOLLOW, None)
    }

    fn metadata(&self) -> io::Result<Metadata> {
        self.file.metadata()
    }

    fn open_stable_directory(&self, name: &OsStr) -> io::Result<Self> {
        Ok(Self {
            file: self.open_directory(name)?,
        })
    }

    fn create_directory(&self, name: &OsStr) -> io::Result<()> {
        self.create_directory_mode(name, 0o777)
    }

    fn create_directory_mode(&self, name: &OsStr, mode: u32) -> io::Result<()> {
        use std::ffi::CString;
        use std::os::fd::AsRawFd;
        use std::os::raw::{c_char, c_int};
        use std::os::unix::ffi::OsStrExt;
        extern "C" {
            fn mkdirat(directory: c_int, path: *const c_char, mode: u32) -> c_int;
        }
        let name = CString::new(name.as_bytes()).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidInput, "directory name contains NUL")
        })?;
        let result = unsafe { mkdirat(self.file.as_raw_fd(), name.as_ptr(), mode) };
        if result == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    fn open_beneath(&self, path: &Path, directory: bool) -> io::Result<File> {
        use std::ffi::CString;
        use std::os::fd::{AsRawFd, FromRawFd};
        use std::os::raw::{c_char, c_int, c_long};
        use std::os::unix::ffi::OsStrExt;
        const O_CLOEXEC: u64 = 0o2_000_000;
        const O_DIRECTORY: u64 = 0o200_000;
        const RESOLVE_NO_MAGICLINKS: u64 = 0x02;
        const RESOLVE_NO_SYMLINKS: u64 = 0x04;
        const RESOLVE_BENEATH: u64 = 0x08;
        const SYS_OPENAT2: c_long = 437;
        #[repr(C)]
        struct OpenHow {
            flags: u64,
            mode: u64,
            resolve: u64,
        }
        extern "C" {
            fn syscall(number: c_long, ...) -> c_long;
        }
        let path = CString::new(path.as_os_str().as_bytes()).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidInput, "relative path contains NUL")
        })?;
        let how = OpenHow {
            flags: O_CLOEXEC | if directory { O_DIRECTORY } else { 0 },
            mode: 0,
            resolve: RESOLVE_BENEATH | RESOLVE_NO_MAGICLINKS | RESOLVE_NO_SYMLINKS,
        };
        let descriptor = unsafe {
            syscall(
                SYS_OPENAT2,
                c_long::from(self.file.as_raw_fd()),
                path.as_ptr().cast::<c_char>(),
                &how,
                std::mem::size_of::<OpenHow>(),
            )
        };
        if descriptor < 0 {
            return Err(io::Error::last_os_error());
        }
        let descriptor = c_int::try_from(descriptor)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "descriptor exceeds c_int"))?;
        Ok(unsafe { File::from_raw_fd(descriptor) })
    }

    fn write_synced_beneath(&self, path: &Path, bytes: &[u8]) -> io::Result<()> {
        let mut components = path.components().peekable();
        let mut directory = Self {
            file: self.file.try_clone()?,
        };
        while let Some(component) = components.next() {
            let std::path::Component::Normal(name) = component else {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "bundle member path must be relative and normalized",
                ));
            };
            if components.peek().is_none() {
                let mut file = directory.create_new(name)?;
                file.write_all(bytes)?;
                file.sync_all()?;
                return Ok(());
            }
            match directory.create_directory(name) {
                Ok(()) => {}
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {}
                Err(error) => return Err(error),
            }
            directory = directory.open_stable_directory(name)?;
        }
        Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "bundle member path is empty",
        ))
    }

    fn remove_tree(&self, name: &OsStr) -> io::Result<()> {
        let child = self.open_stable_directory(name)?;
        self.remove_tree_bound(name, &child)
    }

    fn remove_tree_bound(&self, name: &OsStr, child: &StableDirectory) -> io::Result<()> {
        self.require_named_identity(name, child)?;
        for entry in child.read_dir()? {
            let entry = entry?;
            let entry_name = entry.file_name();
            match child.open_stable_directory(&entry_name) {
                Ok(grandchild) => child.remove_tree_bound(&entry_name, &grandchild)?,
                Err(error) if matches!(error.raw_os_error(), Some(20)) => {
                    let file = child.open_read(&entry_name)?;
                    child.require_named_file_identity(&entry_name, &file)?;
                    child.remove_if_present(&entry_name)?;
                }
                Err(error) => return Err(error),
            }
        }
        self.require_named_identity(name, child)?;
        self.remove_directory(name)
    }

    fn require_named_identity(&self, name: &OsStr, held: &StableDirectory) -> io::Result<()> {
        let named = self.open_stable_directory(name)?;
        if same_file_identity(&named.metadata()?, &held.metadata()?) {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::Other,
                "directory name no longer identifies held inode",
            ))
        }
    }

    fn require_named_file_identity(&self, name: &OsStr, held: &File) -> io::Result<()> {
        let named = self.open_read(name)?;
        if same_file_identity(&named.metadata()?, &held.metadata()?) {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::Other,
                "file name no longer identifies held inode",
            ))
        }
    }

    fn remove_directory(&self, name: &OsStr) -> io::Result<()> {
        use std::ffi::CString;
        use std::os::fd::AsRawFd;
        use std::os::raw::{c_char, c_int};
        use std::os::unix::ffi::OsStrExt;
        extern "C" {
            fn unlinkat(directory: c_int, path: *const c_char, flags: c_int) -> c_int;
        }
        const AT_REMOVEDIR: c_int = 0x200;
        let name = CString::new(name.as_bytes()).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidInput, "directory name contains NUL")
        })?;
        let result = unsafe { unlinkat(self.file.as_raw_fd(), name.as_ptr(), AT_REMOVEDIR) };
        if result == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    fn sync_tree(&self) -> io::Result<()> {
        for entry in self.read_dir()? {
            let entry = entry?;
            if let Some(child) =
                directory_or_regular_file(self.open_stable_directory(&entry.file_name()))?
            {
                child.sync_tree()?;
            }
        }
        self.sync()
    }

    fn open_at(&self, name: &OsStr, flags: i32, mode: Option<u32>) -> io::Result<File> {
        use std::ffi::CString;
        use std::os::raw::{c_char, c_int};
        use std::os::unix::ffi::OsStrExt;
        use std::os::unix::io::{AsRawFd, FromRawFd};

        extern "C" {
            fn openat(directory: c_int, path: *const c_char, flags: c_int, ...) -> c_int;
        }

        let name = CString::new(name.as_bytes())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "file name contains NUL"))?;
        // SAFETY: `name` is NUL-terminated, the directory descriptor is live,
        // and a mode argument is supplied exactly when O_CREAT is present.
        let descriptor = unsafe {
            match mode {
                Some(mode) => openat(self.file.as_raw_fd(), name.as_ptr(), flags, mode),
                None => openat(self.file.as_raw_fd(), name.as_ptr(), flags),
            }
        };
        if descriptor < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: successful openat returned one owned descriptor.
        Ok(unsafe { File::from_raw_fd(descriptor) })
    }

    fn entry_exists(&self, name: &OsStr) -> io::Result<bool> {
        match fs::symlink_metadata(self.child_path(name)) {
            Ok(_) => Ok(true),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(false),
            Err(error) => Err(error),
        }
    }

    fn link_no_replace_to(&self, staging_name: &OsStr, destination: &StableDirectory, final_name: &OsStr) -> io::Result<()> {
        use std::ffi::CString;
        use std::os::raw::{c_char, c_int};
        use std::os::unix::ffi::OsStrExt;
        use std::os::unix::io::AsRawFd;

        extern "C" {
            fn linkat(
                old_directory: c_int,
                old_path: *const c_char,
                new_directory: c_int,
                new_path: *const c_char,
                flags: c_int,
            ) -> c_int;
        }

        let staging = CString::new(staging_name.as_bytes()).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidInput, "staging name contains NUL")
        })?;
        let final_name = CString::new(final_name.as_bytes())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "final name contains NUL"))?;
        // SAFETY: both names are NUL-terminated and both operations are bound
        // to live source and destination descriptors. flags=0 never follows a
        // symbolic-link staging entry.
        let result = unsafe {
            linkat(
                self.file.as_raw_fd(),
                staging.as_ptr(),
                destination.file.as_raw_fd(),
                final_name.as_ptr(),
                0,
            )
        };
        if result == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    fn remove_if_present(&self, name: &OsStr) -> io::Result<bool> {
        use std::ffi::CString;
        use std::os::raw::{c_char, c_int};
        use std::os::unix::ffi::OsStrExt;
        use std::os::unix::io::AsRawFd;

        extern "C" {
            fn unlinkat(directory: c_int, path: *const c_char, flags: c_int) -> c_int;
        }

        let name = CString::new(name.as_bytes())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "file name contains NUL"))?;
        // SAFETY: name is NUL-terminated and the directory descriptor is live.
        let result = unsafe { unlinkat(self.file.as_raw_fd(), name.as_ptr(), 0) };
        if result == 0 {
            Ok(true)
        } else {
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::NotFound {
                Ok(false)
            } else {
                Err(error)
            }
        }
    }

    fn read_dir(&self) -> io::Result<fs::ReadDir> {
        fs::read_dir(self.descriptor_path())
    }

    fn sync(&self) -> io::Result<()> {
        self.file.sync_all()
    }
}

#[cfg(not(target_os = "linux"))]
struct StableDirectory;

#[cfg(not(target_os = "linux"))]
impl StableDirectory {
    fn unsupported() -> io::Error {
        io::Error::new(
            io::ErrorKind::Unsupported,
            "durable artifact publication requires Linux openat2 stable-dirfd semantics",
        )
    }

    fn open(_path: &Path) -> anyhow::Result<Self> {
        Err(Self::unsupported().into())
    }

    fn child_path(&self, _name: &OsStr) -> PathBuf {
        PathBuf::new()
    }

    fn create_new(&self, _name: &OsStr) -> io::Result<File> {
        Err(Self::unsupported())
    }

    fn open_read(&self, _name: &OsStr) -> io::Result<File> {
        Err(Self::unsupported())
    }

    fn metadata(&self) -> io::Result<Metadata> {
        Err(Self::unsupported())
    }
    fn open_stable_directory(&self, _name: &OsStr) -> io::Result<Self> {
        Err(Self::unsupported())
    }
    fn create_directory_mode(&self, _name: &OsStr, _mode: u32) -> io::Result<()> {
        Err(Self::unsupported())
    }
    fn create_directory(&self, _name: &OsStr) -> io::Result<()> {
        Err(Self::unsupported())
    }
    fn open_beneath(&self, _path: &Path, _directory: bool) -> io::Result<File> {
        Err(Self::unsupported())
    }
    fn write_synced_beneath(&self, _path: &Path, _bytes: &[u8]) -> io::Result<()> {
        Err(Self::unsupported())
    }
    fn remove_tree(&self, _name: &OsStr) -> io::Result<()> {
        Err(Self::unsupported())
    }
    fn remove_tree_bound(&self, _name: &OsStr, _child: &StableDirectory) -> io::Result<()> {
        Err(Self::unsupported())
    }
    fn require_named_identity(&self, _name: &OsStr, _held: &StableDirectory) -> io::Result<()> {
        Err(Self::unsupported())
    }
    fn remove_directory(&self, _name: &OsStr) -> io::Result<()> {
        Err(Self::unsupported())
    }
    fn sync_tree(&self) -> io::Result<()> {
        Err(Self::unsupported())
    }

    fn open_directory(&self, _name: &OsStr) -> io::Result<File> {
        Err(Self::unsupported())
    }

    fn entry_exists(&self, _name: &OsStr) -> io::Result<bool> {
        Err(Self::unsupported())
    }

    fn link_no_replace_to(&self, _staging_name: &OsStr, _destination: &StableDirectory, _final_name: &OsStr) -> io::Result<()> {
        Err(Self::unsupported())
    }

    fn remove_if_present(&self, _name: &OsStr) -> io::Result<bool> {
        Err(Self::unsupported())
    }

    fn read_dir(&self) -> io::Result<fs::ReadDir> {
        Err(Self::unsupported())
    }

    fn sync(&self) -> io::Result<()> {
        Err(Self::unsupported())
    }
}

#[cfg(all(test, target_os = "linux"))]
fn stage_prefix(final_path: &Path, format: ArtifactFormat) -> String {
    final_path.file_name().map_or_else(
        || stage_prefix_from_name(OsStr::new(""), format),
        |name| stage_prefix_from_name(name, format),
    )
}

#[cfg(all(test, target_os = "linux"))]
fn stage_prefix_from_name(_final_name: &OsStr, format: ArtifactFormat) -> String {
    format!("{STAGE_DOMAIN}{}-", format.stage_label())
}

#[cfg(all(test, target_os = "linux"))]
fn staging_extension(final_path: &Path) -> String {
    final_path
        .file_name()
        .map_or_else(String::new, staging_extension_from_name)
}

fn staging_extension_from_name(final_name: &OsStr) -> String {
    Path::new(final_name)
        .extension()
        .and_then(|extension| extension.to_str())
        .filter(|extension| {
            !extension.is_empty()
                && extension.len() <= 16
                && extension.bytes().all(|byte| byte.is_ascii_alphanumeric())
        })
        .map_or_else(String::new, |extension| format!(".{extension}"))
}

fn append_name_suffix(name: &OsStr, suffix: &str) -> OsString {
    let mut value = name.to_os_string();
    value.push(suffix);
    value
}

#[cfg(all(test, target_os = "linux"))]
fn append_suffix(path: &Path, suffix: &str) -> PathBuf {
    let mut value = path.as_os_str().to_os_string();
    value.push(suffix);
    PathBuf::from(value)
}

fn parent_directory(path: &Path) -> &Path {
    path.parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."))
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use std::cell::{Cell, RefCell};
    use std::os::unix::ffi::OsStringExt;
    use std::os::unix::fs::PermissionsExt;
    use std::sync::{Arc, Barrier};
    use std::thread;

    use super::*;

    static NEXT_TEST_DIR: AtomicU64 = AtomicU64::new(0);

    struct TestDir(PathBuf);

    impl TestDir {
        fn new(name: &str) -> Self {
            for _ in 0..32 {
                let sequence = NEXT_TEST_DIR.fetch_add(1, Ordering::Relaxed);
                let path = std::env::temp_dir().join(format!(
                    "nuwax-mapping-artifact-{name}-{}-{sequence}",
                    std::process::id()
                ));
                match fs::create_dir(&path) {
                    Ok(()) => return Self(path),
                    Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
                    Err(error) => panic!("create test directory {}: {error}", path.display()),
                }
            }
            panic!("could not allocate artifact test directory");
        }
    }

    impl Drop for TestDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    fn publish_text(path: &Path, text: &str) -> anyhow::Result<()> {
        publish_new_file(
            path,
            ArtifactFormat::SummaryJson,
            |writer| {
                writer.write_all(text.as_bytes())?;
                Ok(())
            },
            |staging| {
                if fs::read_to_string(staging)? != text {
                    bail!("staging content mismatch");
                }
                Ok(())
            },
        )
    }

    fn publish_sqlite_text(path: &Path, text: &str) -> anyhow::Result<()> {
        publish_new_artifact(
            path,
            |_, mut reservation| {
                reservation.write_all(text.as_bytes())?;
                reservation.flush()?;
                Ok(())
            },
            |staging| {
                if fs::read_to_string(staging)? != text {
                    bail!("staging content mismatch");
                }
                Ok(())
            },
        )
    }

    fn stage_entries(path: &Path) -> Vec<PathBuf> {
        fs::read_dir(parent_directory(path))
            .unwrap()
            .filter_map(Result::ok)
            .filter(|entry| {
                entry
                    .file_name()
                    .to_string_lossy()
                    .starts_with(STAGE_DOMAIN)
            })
            .map(|entry| entry.path())
            .collect()
    }

    fn stage_fixture(
        path: &Path,
        format: Option<ArtifactFormat>,
        process_id: u32,
        process_start: u64,
        sequence: u64,
    ) -> (PathBuf, PathBuf) {
        let prefix = format.map_or_else(
            || ".nuwax-mapping-cli-stage-v2-bundle-".to_string(),
            |format| stage_prefix(path, format),
        );
        let wrapper = parent_directory(path).join(format!(
            "{prefix}{process_id}-{process_start:016x}-{sequence:016x}"
        ));
        let contents = wrapper.join(path.file_name().unwrap());
        fs::create_dir_all(&contents).unwrap();
        let payload = contents.join(if format.is_some() {
            format!("payload.tmp{}", staging_extension(path))
        } else {
            "payload".to_string()
        });
        if format.is_none() {
            fs::create_dir(&payload).unwrap();
        }
        (wrapper, payload)
    }

    fn dead_stage(path: &Path, format: ArtifactFormat) -> (PathBuf, PathBuf) {
        stage_fixture(path, Some(format), absent_process_id(), 1, 2)
    }

    fn publish_bundle_marker(path: &Path, bytes: &[u8]) -> anyhow::Result<()> {
        publish_map_bundle_stable(
            path,
            |staging| staging.write_synced_beneath(Path::new("marker"), bytes).map_err(Into::into),
            |staging| {
                if read_fresh_file(staging, "marker")? != bytes {
                    bail!("bundle marker mismatch");
                }
                Ok(())
            },
        )
    }

    fn absent_process_id() -> u32 {
        (2_000_000..=2_001_000)
            .find(|process_id| linux_process_probe(*process_id) == ProcessProbe::Absent)
            .expect("find a process id proven absent through /proc")
    }

    #[test]
    fn artifact_formats_emit_exact_distinct_labels_and_reject_wrong_extensions() {
        let directory = TestDir::new("format-labels");
        let cases = [
            (
                ArtifactFormat::DatasetJsonLines,
                "dataset-jsonl",
                "dataset.jsonl",
            ),
            (ArtifactFormat::MapPgm, "map-pgm", "map.pgm"),
            (ArtifactFormat::SummaryJson, "summary-json", "summary.json"),
            (
                ArtifactFormat::Memory2Sqlite,
                "memory2-sqlite",
                "dataset.db",
            ),
        ];

        let mut prefixes = std::collections::BTreeSet::new();
        for (format, label, file_name) in cases {
            let output = directory.0.join(file_name);
            let observed_name = RefCell::new(None);
            publish_artifact(
                &output,
                format,
                |staging, mut reservation| {
                    observed_name.replace(staging.file_name().map(OsStr::to_os_string));
                    let wrappers = stage_entries(&output);
                    assert_eq!(wrappers.len(), 1);
                    let wrapper = &wrappers[0];
                    let name = wrapper.file_name().unwrap().to_str().unwrap();
                    let expected_prefix = format!(".nuwax-mapping-cli-stage-v2-{label}-");
                    let fields: Vec<_> = name.strip_prefix(&expected_prefix).unwrap().split('-').collect();
                    assert_eq!(fields.len(), 3);
                    assert_eq!(fields[0], std::process::id().to_string());
                    let policy = RecoveryPolicy::acquire().unwrap();
                    assert_eq!(fields[1], format!("{:016x}", policy.current_process_start()));
                    assert_eq!(fields[2].len(), 16);
                    assert!(fields[2].bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)));
                    let scope_entries: Vec<_> = fs::read_dir(wrapper).unwrap()
                        .map(|entry| entry.unwrap().file_name()).collect();
                    assert_eq!(scope_entries, vec![output.file_name().unwrap().to_os_string()]);
                    let contents = wrapper.join(output.file_name().unwrap());
                    assert_eq!(fs::metadata(wrapper)?.permissions().mode() & 0o077, 0);
                    assert_eq!(fs::metadata(&contents)?.permissions().mode() & 0o077, 0);
                    assert!(contents.join(format!("payload.tmp{}", staging_extension(&output))).is_file());
                    reservation.write_all(b"validated")?;
                    reservation.flush()?;
                    Ok(())
                },
                |staging| {
                    if fs::read(staging)? != b"validated" {
                        bail!("staging content mismatch");
                    }
                    Ok(())
                },
            )
            .unwrap();

            let expected_prefix = stage_prefix(&output, format);
            assert!(expected_prefix.ends_with(&format!("-{label}-")));
            assert!(prefixes.insert(expected_prefix.clone()));
            assert_eq!(expected_prefix, format!(".nuwax-mapping-cli-stage-v2-{label}-"));
            assert_eq!(
                observed_name.into_inner().unwrap(),
                OsString::from(format!("payload.tmp{}", staging_extension(&output)))
            );

            let wrong_output = directory.0.join(format!("{label}.bin"));
            let builder_ran = Cell::new(false);
            let error = publish_new_file(
                &wrong_output,
                format,
                |_| {
                    builder_ran.set(true);
                    Ok(())
                },
                |_| Ok(()),
            )
            .unwrap_err();
            assert!(!builder_ran.get());
            assert!(error.to_string().contains(format.expected_extension()));
            assert!(!wrong_output.exists());
        }
        assert!(stage_entries(&directory.0.join("unused.json")).is_empty());
    }

    #[test]
    fn final_is_invisible_until_validation_finishes() {
        let directory = TestDir::new("visibility");
        let output = directory.0.join("artifact.json");
        publish_new_file(
            &output,
            ArtifactFormat::SummaryJson,
            |writer| {
                assert!(!output.exists());
                writer.write_all(br#"{"ok":true}"#)?;
                assert!(!output.exists());
                Ok(())
            },
            |staging| {
                assert!(!output.exists());
                let value: serde_json::Value = serde_json::from_reader(File::open(staging)?)?;
                assert_eq!(value, serde_json::json!({"ok": true}));
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(fs::read_to_string(output).unwrap(), r#"{"ok":true}"#);
    }

    #[test]
    fn concurrent_writers_publish_exactly_one_valid_winner() {
        let directory = TestDir::new("race");
        let output = Arc::new(directory.0.join("winner.json"));
        let barrier = Arc::new(Barrier::new(16));
        let mut threads = Vec::new();
        for writer_id in 0..16 {
            let output = Arc::clone(&output);
            let barrier = Arc::clone(&barrier);
            threads.push(thread::spawn(move || {
                barrier.wait();
                publish_text(&output, &format!("writer-{writer_id}"))
                    .map(|()| writer_id)
                    .map_err(|error| {
                        assert!(
                            error
                                .downcast_ref::<ArtifactPublicationError>()
                                .is_some_and(|error| matches!(
                                    error,
                                    ArtifactPublicationError::Conflict { .. }
                                )),
                            "unexpected publication error: {error:#}"
                        );
                    })
            }));
        }
        let winners: Vec<_> = threads
            .into_iter()
            .filter_map(|handle| handle.join().unwrap().ok())
            .collect();
        assert_eq!(winners.len(), 1);
        assert_eq!(
            fs::read_to_string(output.as_ref()).unwrap(),
            format!("writer-{}", winners[0])
        );
        assert!(stage_entries(&output).is_empty());
    }

    #[test]
    fn builder_and_validator_failures_leave_no_visible_or_staged_file() {
        let directory = TestDir::new("failure");
        let build_output = directory.0.join("build.json");
        let build_error = publish_new_file(
            &build_output,
            ArtifactFormat::SummaryJson,
            |writer| {
                writer.write_all(b"partial")?;
                bail!("injected builder failure")
            },
            |_| Ok(()),
        )
        .unwrap_err();
        assert!(format!("{build_error:#}").contains("injected builder failure"));
        assert!(!build_output.exists());
        assert!(stage_entries(&build_output).is_empty());

        let validate_output = directory.0.join("validate.json");
        let validate_error = publish_new_file(
            &validate_output,
            ArtifactFormat::SummaryJson,
            |writer| {
                writer.write_all(b"corrupt")?;
                Ok(())
            },
            |_| bail!("injected validation failure"),
        )
        .unwrap_err();
        assert!(format!("{validate_error:#}").contains("injected validation failure"));
        assert!(!validate_output.exists());
        assert!(stage_entries(&validate_output).is_empty());
    }

    #[test]
    fn dead_staging_and_sqlite_sidecars_are_recovered_before_publish() {
        let directory = TestDir::new("stale");
        let output = directory.0.join("artifact.db");
        let (wrapper, stale) = dead_stage(&output, ArtifactFormat::Memory2Sqlite);
        fs::write(&stale, b"partial database").unwrap();
        for suffix in ["-wal", "-shm", "-journal"] {
            fs::write(append_suffix(&stale, suffix), b"partial sidecar").unwrap();
        }

        publish_sqlite_text(&output, "validated winner").unwrap();
        assert_eq!(fs::read_to_string(&output).unwrap(), "validated winner");
        assert!(!wrapper.exists());
        assert!(!stale.exists());
        for suffix in ["-wal", "-shm", "-journal"] {
            assert!(!append_suffix(&stale, suffix).exists());
        }
        assert!(stage_entries(&output).is_empty());
    }

    #[test]
    fn recovery_never_removes_an_existing_winner() {
        let directory = TestDir::new("preserve-winner");
        let output = directory.0.join("artifact.json");
        fs::write(&output, b"winner").unwrap();
        let (wrapper, stale) = dead_stage(&output, ArtifactFormat::SummaryJson);
        fs::write(&stale, b"loser").unwrap();

        let error = publish_text(&output, "replacement").unwrap_err();
        assert!(matches!(
            error.downcast_ref::<ArtifactPublicationError>(),
            Some(ArtifactPublicationError::Conflict { .. })
        ));
        assert_eq!(fs::read(&output).unwrap(), b"winner");
        assert!(!wrapper.exists());
        assert!(stage_entries(&output).is_empty());
    }

    #[test]
    fn active_staging_is_not_reaped_without_a_winner() {
        let directory = TestDir::new("active");
        let output = directory.0.join("artifact.json");
        let process_id = std::process::id();
        let policy = RecoveryPolicy::acquire().unwrap();
        let process_start = policy.current_process_start();
        let (wrapper, active) = stage_fixture(
            &output, Some(ArtifactFormat::SummaryJson), process_id, process_start, 7,
        );
        fs::write(&active, b"active").unwrap();

        let stable = StableDirectory::open(&directory.0).unwrap();
        recover_stale_stages(
            &stable,
            output.file_name().unwrap(),
            &output,
            ArtifactFormat::SummaryJson,
            &policy,
        )
        .unwrap();
        assert!(wrapper.is_dir());
        assert_eq!(fs::read(&active).unwrap(), b"active");
        fs::remove_dir_all(wrapper).unwrap();
    }

    #[test]
    fn recovery_accepts_only_exact_emitted_names() {
        let directory = TestDir::new("strict-parser");
        let output = directory.0.join("artifact.json");
        let (exact_wrapper, exact) = dead_stage(&output, ArtifactFormat::SummaryJson);
        let exact_name = exact_wrapper.file_name().unwrap().to_str().unwrap();
        let absent_process = absent_process_id();
        fs::write(&exact, b"exact stale staging").unwrap();
        let hostile_names = [
            format!("{exact_name}.user-tail"),
            format!("{exact_name}-wal"),
            format!("{exact_name}-wal-extra"),
            exact_name.replacen("-0000000000000001-", "-0000000000000001A-", 1),
            exact_name.replacen("-0000000000000001-", "-000000000000000A-", 1),
            exact_name.replacen("-0000000000000001-", "-0000000000000000-", 1),
            exact_name.replacen("-0000000000000002", "-000000000000000A", 1),
            exact_name.replacen(&format!("{absent_process}-"), &format!("0{absent_process}-"), 1),
            exact_name.replacen("-summary-json-", "-replay-json-", 1),
        ];
        for name in &hostile_names {
            let contents = directory.0.join(name).join(output.file_name().unwrap());
            fs::create_dir_all(&contents).unwrap();
            fs::write(contents.join("payload.tmp.json"), b"user file").unwrap();
        }

        publish_text(&output, "winner").unwrap();

        assert!(!exact_wrapper.exists());
        assert!(!exact.exists());
        assert_eq!(fs::read_to_string(&output).unwrap(), "winner");
        for name in hostile_names {
            let wrapper = directory.0.join(name);
            assert!(wrapper.is_dir());
            assert_eq!(
                fs::read(wrapper.join(output.file_name().unwrap()).join("payload.tmp.json")).unwrap(),
                b"user file"
            );
        }
    }

    #[test]
    fn unverifiable_process_identity_never_reaps_staging() {
        let directory = TestDir::new("unverifiable-owner");
        let output = directory.0.join("artifact.json");
        let (wrapper, staging) = dead_stage(&output, ArtifactFormat::SummaryJson);
        fs::write(&staging, b"must remain").unwrap();
        let stable = StableDirectory::open(&directory.0).unwrap();

        recover_stale_stages_with(
            &stable,
            output.file_name().unwrap(),
            &output,
            ArtifactFormat::SummaryJson,
            |owner| owner_is_proven_dead_from_probe(owner, ProcessProbe::Unverifiable),
        )
        .unwrap();

        assert!(wrapper.is_dir());
        assert_eq!(fs::read(staging).unwrap(), b"must remain");
        assert!(!owner_is_proven_dead_from_probe(
            StageOwner {
                process_id: absent_process_id(),
                process_start: 1,
            },
            ProcessProbe::Unverifiable
        ));
        assert!(parse_linux_process_start("malformed proc stat").is_none());
    }

    #[test]
    fn unverifiable_current_process_identity_is_rejected() {
        for probe in [ProcessProbe::Unverifiable, ProcessProbe::Absent] {
            let error = RecoveryPolicy::from_current_process_probe(probe).unwrap_err();
            assert!(error
                .to_string()
                .contains("refusing artifact publication before file side effects"));
        }
        assert!(RecoveryPolicy::from_current_process_probe(ProcessProbe::Running(0)).is_err());
    }

    #[test]
    fn conflict_keeps_primary_type_when_cleanup_fails() {
        let error = conflict_after_cleanup(
            Path::new("winner.json"),
            Path::new("stage.json"),
            Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "injected cleanup",
            )),
        );
        let typed = error.downcast_ref::<ArtifactPublicationError>().unwrap();

        assert!(matches!(
            typed,
            ArtifactPublicationError::Conflict {
                cleanup: Some(_),
                ..
            }
        ));
        assert!(error.to_string().contains("refusing to overwrite"));
        assert!(error.to_string().contains("injected cleanup"));
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn sync_tree_directory_open_classification_propagates_non_enotdir_errors() {
        assert_eq!(
            directory_or_regular_file::<()>(Err(io::Error::from_raw_os_error(20))).unwrap(),
            None
        );

        for raw_os_error in [13, 5, 24, 40, 116] {
            let error =
                directory_or_regular_file::<()>(Err(io::Error::from_raw_os_error(raw_os_error)))
                    .unwrap_err();
            assert_eq!(error.raw_os_error(), Some(raw_os_error));
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn symbolic_link_output_parent_is_rejected_before_writes() {
        use std::os::unix::fs::symlink;

        let directory = TestDir::new("symlink-parent");
        let real_parent = directory.0.join("real");
        let linked_parent = directory.0.join("linked");
        fs::create_dir(&real_parent).unwrap();
        symlink(&real_parent, &linked_parent).unwrap();

        let error = publish_text(&linked_parent.join("artifact.json"), "blocked").unwrap_err();

        assert!(format!("{error:#}").contains("openat2"));
        assert!(fs::read_dir(real_parent).unwrap().next().is_none());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn stable_bundle_transaction_returns_validated_manifest_size() {
        let directory = TestDir::new("bundle-size");
        let output = directory.0.join("bundle");
        let expected = br#"{"schema_version":1}"#.len();

        let observed = publish_map_bundle_stable(
            &output,
            |staging| {
                staging
                    .write_synced_beneath(Path::new("manifest.json"), br#"{"schema_version":1}"#)?;
                Ok(())
            },
            |staging| Ok(read_fresh_file(staging, "manifest.json")?.len()),
        )
        .unwrap();

        assert_eq!(observed, expected);
        assert_eq!(
            fs::read(output.join("manifest.json")).unwrap().len(),
            expected
        );
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn stable_directory_writes_nested_files_without_following_symlinks() {
        use std::os::unix::fs::symlink;

        let directory = TestDir::new("bundle-write-symlink");
        let outside = directory.0.join("outside");
        let root = directory.0.join("root");
        fs::create_dir(&outside).unwrap();
        fs::create_dir(&root).unwrap();
        symlink(&outside, root.join("nested")).unwrap();
        let stable = StableDirectory::open(&root).unwrap();

        assert!(stable
            .write_synced_beneath(Path::new("nested/member.bin"), b"blocked")
            .is_err());
        assert!(!outside.join("member.bin").exists());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn remove_tree_rejects_name_rebound_to_another_inode() {
        let directory = TestDir::new("bundle-remove-binding");
        let original = directory.0.join("original");
        let moved = directory.0.join("moved");
        fs::create_dir(&original).unwrap();
        let parent = StableDirectory::open(&directory.0).unwrap();
        let held = parent
            .open_stable_directory(OsStr::new("original"))
            .unwrap();
        fs::rename(&original, &moved).unwrap();
        fs::create_dir(&original).unwrap();

        assert!(parent
            .remove_tree_bound(OsStr::new("original"), &held)
            .is_err());
        assert!(original.exists());
        assert!(moved.exists());
    }


    #[test]
    fn recovery_preserves_entire_wrappers_with_unknown_entries() {
        for format in [Some(ArtifactFormat::Memory2Sqlite), None] {
            for extra_in_wrapper in [true, false] {
                let directory = TestDir::new("unknown-wrapper-entry");
                let output = directory.0.join(if format.is_some() { "artifact.db" } else { "bundle" });
                let (wrapper, payload) = stage_fixture(&output, format, absent_process_id(), 1, 20);
                let original = if format.is_some() { payload.clone() } else { payload.join("marker") };
                fs::write(&original, b"original payload").unwrap();
                if format.is_some() {
                    fs::write(append_suffix(&payload, "-wal"), b"original wal").unwrap();
                }
                let extra = if extra_in_wrapper {
                    wrapper.join("user-file")
                } else if format.is_some() {
                    append_suffix(&payload, "-wal-extra")
                } else {
                    payload.parent().unwrap().join("payload-extra")
                };
                fs::write(&extra, b"user file").unwrap();

                if format.is_some() {
                    publish_sqlite_text(&output, "validated winner").unwrap();
                    assert_eq!(fs::read(&output).unwrap(), b"validated winner");
                    assert_eq!(fs::read(append_suffix(&payload, "-wal")).unwrap(), b"original wal");
                } else {
                    publish_bundle_marker(&output, b"validated winner").unwrap();
                    assert_eq!(fs::read(output.join("marker")).unwrap(), b"validated winner");
                }
                assert!(wrapper.is_dir());
                assert_eq!(fs::read(&original).unwrap(), b"original payload");
                assert_eq!(fs::read(&extra).unwrap(), b"user file");
                assert_eq!(stage_entries(&output), vec![wrapper]);
            }
        }
    }

    #[test]
    fn staging_preserves_raw_names_for_long_and_non_utf8_outputs() {
        let file_names = [
            OsString::from(format!("{}.db", "x".repeat(252))),
            OsString::from_vec(b"artifact-\xff.db".to_vec()),
        ];
        for name in file_names {
            let directory = TestDir::new("raw-file-name");
            let output = directory.0.join(&name);
            let (stale_wrapper, stale) = dead_stage(&output, ArtifactFormat::Memory2Sqlite);
            fs::write(&stale, b"stale").unwrap();
            for suffix in ["-wal", "-shm", "-journal"] {
                fs::write(append_suffix(&stale, suffix), b"stale sidecar").unwrap();
            }
            publish_new_artifact(
                &output,
                |staging, mut reservation| {
                    assert!(!stale_wrapper.exists());
                    assert_eq!(staging.file_name(), Some(OsStr::new("payload.tmp.db")));
                    let wrappers = stage_entries(&output);
                    assert_eq!(wrappers.len(), 1);
                    let names: Vec<_> = fs::read_dir(&wrappers[0])?
                        .map(|entry| entry.unwrap().file_name()).collect();
                    assert_eq!(names, vec![name.clone()]);
                    reservation.write_all(b"validated winner")?;
                    Ok(())
                },
                |staging| {
                    assert_eq!(fs::read(staging)?, b"validated winner");
                    Ok(())
                },
            ).unwrap();
            assert_eq!(fs::read(&output).unwrap(), b"validated winner");
            assert!(stage_entries(&output).is_empty());
        }

        let bundle_names = [OsString::from("b".repeat(255)), OsString::from_vec(b"bundle-\xff".to_vec())];
        for name in bundle_names {
            let directory = TestDir::new("raw-bundle-name");
            let output = directory.0.join(&name);
            let (wrapper, payload) = stage_fixture(&output, None, absent_process_id(), 1, 30);
            fs::write(payload.join("marker"), b"stale").unwrap();
            publish_bundle_marker(&output, b"validated winner").unwrap();
            assert!(!wrapper.exists());
            assert_eq!(fs::read(output.join("marker")).unwrap(), b"validated winner");
            assert!(stage_entries(&output).is_empty());
        }
    }

    #[test]
    fn recovery_preserves_different_raw_final_names() {
        for format in [Some(ArtifactFormat::Memory2Sqlite), None] {
            let directory = TestDir::new("different-raw-name");
            let (name, foreign_name) = if format.is_some() {
                (b"artifact-\xff.db".as_slice(), b"artifact-\xfe.db".as_slice())
            } else {
                (b"bundle-\xff".as_slice(), b"bundle-\xfe".as_slice())
            };
            let output = directory.0.join(OsString::from_vec(name.to_vec()));
            let foreign_output = directory.0.join(OsString::from_vec(foreign_name.to_vec()));
            let (foreign_wrapper, foreign_payload) = stage_fixture(&foreign_output, format, absent_process_id(), 1, 40);
            let foreign_file = if format.is_some() { foreign_payload } else { foreign_payload.join("marker") };
            fs::write(&foreign_file, b"foreign payload").unwrap();
            let (matching_wrapper, matching_payload) = stage_fixture(&output, format, absent_process_id(), 1, 41);
            let matching_file = if format.is_some() { matching_payload } else { matching_payload.join("marker") };
            fs::write(&matching_file, b"matching stale payload").unwrap();

            if format.is_some() {
                publish_sqlite_text(&output, "winner").unwrap();
                assert_eq!(fs::read(&output).unwrap(), b"winner");
            } else {
                publish_bundle_marker(&output, b"winner").unwrap();
                assert_eq!(fs::read(output.join("marker")).unwrap(), b"winner");
            }
            assert!(!matching_wrapper.exists());
            assert!(foreign_wrapper.is_dir());
            assert_eq!(fs::read(foreign_file).unwrap(), b"foreign payload");
            assert_eq!(stage_entries(&output), vec![foreign_wrapper]);
        }
    }

    #[test]
    fn recovery_leaves_legacy_v1_file_and_bundle_staging_untouched() {
        let directory = TestDir::new("legacy-v1");
        let process_id = absent_process_id();
        // Literal legacy-v1 shapes. Recovery must ignore every v1 name without hashing.
        let legacy_file = directory.0.join(format!(
            ".nuwax-mapping-cli-stage-v1-0123456789abcdef0123456789abcdef-memory2-sqlite-{process_id}-0000000000000001-0000000000000002.tmp.db"
        ));
        fs::write(&legacy_file, b"legacy database").unwrap();
        let legacy_wal = append_suffix(&legacy_file, "-wal");
        fs::write(&legacy_wal, b"legacy wal").unwrap();
        let legacy_bundle = directory.0.join(format!(
            ".nuwax-mapping-cli-map-bundle-v1-{process_id}-0000000000000001-0000000000000002-fedcba9876543210fedcba9876543210"
        ));
        fs::create_dir(&legacy_bundle).unwrap();
        fs::write(legacy_bundle.join("marker"), b"legacy bundle").unwrap();
        let output = directory.0.join("artifact.db");
        let bundle = directory.0.join("bundle");

        publish_sqlite_text(&output, "winner").unwrap();
        publish_bundle_marker(&bundle, b"bundle winner").unwrap();

        assert_eq!(fs::read(&output).unwrap(), b"winner");
        assert_eq!(fs::read(bundle.join("marker")).unwrap(), b"bundle winner");
        assert_eq!(fs::read(legacy_file).unwrap(), b"legacy database");
        assert_eq!(fs::read(legacy_wal).unwrap(), b"legacy wal");
        assert_eq!(fs::read(legacy_bundle.join("marker")).unwrap(), b"legacy bundle");
        assert!(stage_entries(&output).is_empty());
    }

    #[test]
    fn bundle_wrapper_lifecycle_preserves_active_and_unverifiable_owners() {
        let directory = TestDir::new("bundle-wrapper-lifecycle");
        let output = directory.0.join("bundle");
        let policy = RecoveryPolicy::acquire().unwrap();
        let (active_wrapper, active_payload) = stage_fixture(
            &output, None, std::process::id(), policy.current_process_start(), 50,
        );
        fs::write(active_payload.join("marker"), b"active").unwrap();
        let (dead_wrapper, dead_payload) = stage_fixture(&output, None, absent_process_id(), 1, 51);
        fs::write(dead_payload.join("marker"), b"dead").unwrap();
        let (published_wrapper, moved_payload) = stage_fixture(&output, None, absent_process_id(), 1, 52);
        fs::remove_dir(moved_payload).unwrap(); // Crash after payload rename: raw name remains.
        let (empty_wrapper, unused_payload) = stage_fixture(&output, None, absent_process_id(), 1, 53);
        fs::remove_dir(&unused_payload).unwrap();
        fs::remove_dir(unused_payload.parent().unwrap()).unwrap(); // No destination binding remains.
        let parent = StableDirectory::open(&directory.0).unwrap();

        recover_stale_wrappers(&parent, output.file_name().unwrap(), &output, None,
            |owner| owner_is_proven_dead_from_probe(owner, ProcessProbe::Unverifiable)).unwrap();
        assert_eq!(fs::read(dead_payload.join("marker")).unwrap(), b"dead");
        assert_eq!(fs::read(active_payload.join("marker")).unwrap(), b"active");

        publish_bundle_marker(&output, b"validated winner").unwrap();

        assert_eq!(fs::read(output.join("marker")).unwrap(), b"validated winner");
        assert!(!dead_wrapper.exists());
        assert!(active_wrapper.is_dir());
        assert_eq!(fs::read(active_payload.join("marker")).unwrap(), b"active");
        assert!(!published_wrapper.exists());
        assert!(empty_wrapper.is_dir());
        let mut remaining = stage_entries(&output);
        remaining.sort();
        let mut expected = vec![active_wrapper, empty_wrapper];
        expected.sort();
        assert_eq!(remaining, expected);
    }

    #[test]
    fn bundle_postpublish_cleanup_preserves_recreated_payload() {
        let directory = TestDir::new("bundle-postpublish-cleanup");
        let parent = StableDirectory::open(&directory.0).unwrap();
        let final_name = OsStr::new("bundle");
        let policy = RecoveryPolicy::acquire().unwrap();
        let area = StagingArea::reserve(&parent, final_name, "bundle", &policy).unwrap();
        let payload = OsStr::new("payload");
        area.contents.create_directory(payload).unwrap();
        let staging = area.contents.open_stable_directory(payload).unwrap();
        staging.write_synced_beneath(Path::new("marker"), b"winner").unwrap();
        staging.sync_tree().unwrap();
        area.require_bound(&parent).unwrap();
        rename_directory_no_replace_at(&area.contents, payload, &parent, final_name).unwrap();
        area.contents.sync().unwrap();
        parent.sync().unwrap();
        area.contents.create_directory(payload).unwrap();
        let replacement = area.contents.open_stable_directory(payload).unwrap();
        replacement.write_synced_beneath(Path::new("marker"), b"new payload").unwrap();

        assert!(area.cleanup_empty(&parent).is_err());
        assert_eq!(fs::read(directory.0.join("bundle/marker")).unwrap(), b"winner");
        assert_eq!(read_fresh_file(&replacement, "marker").unwrap(), b"new payload");
        area.contents.require_named_identity(payload, &replacement).unwrap();
        area.require_bound(&parent).unwrap();
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn stable_directory_operations_ignore_parent_path_retarget() {
        let directory = TestDir::new("parent-retarget");
        let parent = directory.0.join("output");
        let moved = directory.0.join("opened-output");
        fs::create_dir(&parent).unwrap();
        let stable = StableDirectory::open(&parent).unwrap();
        let staging_name = OsStr::new("payload.tmp.json");
        let final_name = OsStr::new("winner.json");
        let policy = RecoveryPolicy::acquire().unwrap();
        let area = StagingArea::reserve(&stable, final_name, "summary-json", &policy).unwrap();
        let mut staging = area.contents.create_new(staging_name).unwrap();
        staging.write_all(b"winner").unwrap();
        staging.sync_all().unwrap();
        drop(staging);

        fs::rename(&parent, &moved).unwrap();
        fs::create_dir(&parent).unwrap();
        area.require_bound(&stable).unwrap();
        area.contents.link_no_replace_to(staging_name, &stable, final_name).unwrap();
        stable.sync().unwrap();
        area.cleanup(&stable, staging_name, Some(ArtifactFormat::SummaryJson)).unwrap();

        assert_eq!(fs::read(moved.join(final_name)).unwrap(), b"winner");
        assert!(stage_entries(&moved.join(final_name)).is_empty());
        assert!(fs::read_dir(parent).unwrap().next().is_none());
    }
    // Test-only DTO adapter: production has no Geometry -> CLI adapter.
    fn offline_v2_fixture() -> OfflineCandidateBundleV2 {
        let commit = json!({"boot_nonce": "340282366920938463463374607431768211455", "sequence": 7});
        let provenance = json!({
            "commit_id": commit,
            "occupancy_receipt": {"commit_id": commit, "session_id":"fixture-session", "session_generation":2,
                "occupancy_revision":7, "source_timestamp_ns":99, "backend_kind":"voxel", "backend_generation":1},
            "cloud_receipt": {"commit_id":commit, "session_generation":2, "occupancy_revision":7,
                "cloud_revision":7, "source_timestamp_ns":99, "point_count":1}
        });
        let members = [
            ("navigation/map.yaml", "application/x-yaml", "nav2_occupancy_yaml_v1", "canonical_raw_occupancy", b"image: map.pgm\n".to_vec()),
            ("navigation/map.pgm", "image/x-portable-graymap", "nav2_occupancy_pgm_p5_v1", "canonical_raw_occupancy", b"P5\n1 1\n255\n\0".to_vec()),
            ("cloud/map.pcd", "application/vnd.pointclouds.pcd", "pcd_xyz_binary_v1", "committed_transformed_accepted_cloud", b"test-pcd-body".to_vec()),
            ("mapping_backend/voxel/objects/checkpoint.bin", "application/octet-stream", "checkpoint_object_v1", "mapping_backend_checkpoint", b"test-checkpoint".to_vec()),
        ].into_iter().map(|(path,media,encoding,source,bytes)| OfflineCandidateAssetMemberV2Dto {
            relative_path_intent:path.into(), receipt_relative_path_intent:path.into(),
            media_type:media.into(),receipt_media_type:media.into(),encoding:encoding.into(),receipt_encoding:encoding.into(),
            size_bytes:bytes.len() as u64,bytes,source_binding:source.into(),source_commit_id_json:canonical_json_value(&commit).unwrap(),
        }).collect();
        OfflineCandidateAssetsV2Dto {
            schema_version:2,snapshot_schema_version:3,
            identity_json:canonical_json_value(&json!({"map_name":"fixture-map","map_frame_id":"map"})).unwrap(),
            source_provenance_json:canonical_json_value(&provenance).unwrap(),backend_kind:"voxel".into(),last_source_timestamp_ns:99,
            georeference_json:canonical_json_value(&json!({"frame_id":"map"})).unwrap(),
            processing_receipt_json:canonical_json_value(&json!({"committed_point_count":2,"flush_committed":true})).unwrap(),
            map_snapshot_seed_json:canonical_json_value(&json!({"fixture":true})).unwrap(),
            deferred_json:canonical_json_value(&json!([])).unwrap(),members,
        }.into_publish_bundle().unwrap()
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn offline_v2_publishes_original_commit_and_source_roles() {
        let directory=TestDir::new("offline-v2-publish");
        let candidate=offline_v2_fixture();
        let output=directory.0.join("bundle");
        let result=publish_offline_candidate_bundle(&output,&candidate).unwrap();
        assert_eq!(result.final_directory,output);
        assert_eq!(result.source_commit_id_json,canonical_json_value(&validate_offline_candidate(&candidate).unwrap()["commit_id"]).unwrap());
        let top:Value=serde_json::from_slice(&fs::read(output.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(top["schema_version"],2);
        assert_eq!(top["source_provenance"]["cloud_receipt"]["point_count"],1);
        assert_eq!(top["processing_receipt"]["committed_point_count"],2);
        assert_eq!(top["source_provenance"]["commit_id"]["boot_nonce"],"340282366920938463463374607431768211455");
        assert!(top["files"].as_array().unwrap().iter().all(|entry|entry.get("sha256").is_none()));
        let pcd=top["files"].as_array().unwrap().iter().find(|entry|entry["encoding"]=="pcd_xyz_binary_v1").unwrap();
        assert_eq!(pcd["source_binding"],"committed_transformed_accepted_cloud");
        assert!(publish_offline_candidate_bundle(&output,&candidate).is_err());
    }

    #[test]
    fn offline_v2_rejects_wrong_schema_commit_role_size_and_path() {
        for mutation in 0..6 {
            let mut candidate=offline_v2_fixture();
            match mutation {
                0=>candidate.schema_version=1,
                1=>candidate.snapshot_schema_version=2,
                2=>candidate.members[0].source_commit_id_json=canonical_json_value(&json!({"boot_nonce":"1","sequence":7})).unwrap(),
                3=>candidate.members[2].source_binding="canonical_cloud".into(),
                4=>candidate.members[0].size_bytes+=1,
                _=>{candidate.members[3].relative_path="mapping_backend/voxel/objects/old-name.bin".into();candidate.checkpoint.object_path=candidate.members[3].relative_path.clone();}
            }
            assert!(validate_offline_candidate(&candidate).is_err(),"mutation {mutation}");
        }
    }

    #[test]
    fn offline_v2_rejects_noncanonical_or_mismatched_provenance() {
        for mutation in 0..4 {
            let mut candidate=offline_v2_fixture();
            let mut provenance:Value=serde_json::from_slice(&candidate.source_provenance_json).unwrap();
            match mutation {
                0=>provenance["commit_id"]["boot_nonce"]=json!(7),
                1=>provenance["commit_id"]["boot_nonce"]=json!("01"),
                2=>provenance["cloud_receipt"]["commit_id"]["sequence"]=json!(8),
                _=>provenance["commit_id"]["boot_nonce"]=json!("340282366920938463463374607431768211456"),
            }
            candidate.source_provenance_json=canonical_json_value(&provenance).unwrap();
            assert!(validate_offline_candidate(&candidate).is_err(),"mutation {mutation}");
        }
        let mut candidate=offline_v2_fixture();
        candidate.source_provenance_json.push(b' ');
        assert!(validate_offline_candidate(&candidate).is_err());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn offline_v2_directory_checks_sizes_but_not_static_binary_content() {
        let directory=TestDir::new("offline-v2-static");
        let stable=StableDirectory::open(&directory.0).unwrap();
        let candidate=offline_v2_fixture();
        materialize_offline_candidate(&stable,&candidate,&validate_offline_candidate(&candidate).unwrap()).unwrap();
        let target=directory.0.join(&candidate.members[3].relative_path);
        fs::write(&target,vec![b'X';candidate.members[3].size_bytes as usize]).unwrap();
        validate_offline_candidate_directory(&stable,&candidate,&validate_offline_candidate(&candidate).unwrap()).unwrap();
        fs::write(&target,b"short").unwrap();
        assert!(validate_offline_candidate_directory(&stable,&candidate,&validate_offline_candidate(&candidate).unwrap()).is_err());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn offline_v2_directory_rejects_changed_json_commit_and_file_set() {
        for mutation in 0..4 {
            let directory=TestDir::new("offline-v2-bindings");
            let stable=StableDirectory::open(&directory.0).unwrap();
            let candidate=offline_v2_fixture();
            materialize_offline_candidate(&stable,&candidate,&validate_offline_candidate(&candidate).unwrap()).unwrap();
            let path=if mutation==0 {&candidate.checkpoint.selector_path} else {"manifest.json"};
            let mut value:Value=serde_json::from_slice(&fs::read(directory.0.join(path)).unwrap()).unwrap();
            match mutation {
                0=>value["source_commit_id"]["sequence"]=json!(8),
                1=>value["files"][0]["source_binding"]=json!("wrong"),
                2=>value["files"].as_array_mut().unwrap().pop().map(|_|()).unwrap(),
                _=>value["source_provenance"]["cloud_receipt"]["point_count"]=json!(2),
            }
            fs::write(directory.0.join(path),canonical_json_value(&value).unwrap()).unwrap();
            assert!(validate_offline_candidate_directory(&stable,&candidate,&validate_offline_candidate(&candidate).unwrap()).is_err(),"mutation {mutation}");
        }
    }


}

#[cfg(all(test, not(target_os = "linux")))]
mod unsupported_platform_tests {
    use super::*;

    #[test]
    fn publication_fails_before_file_side_effects() {
        let directory = std::env::temp_dir().join(format!(
            "nuwax-mapping-unsupported-platform-{}",
            std::process::id()
        ));
        fs::create_dir(&directory).unwrap();
        let output = directory.join("artifact.json");

        let error = publish_new_file(
            &output,
            ArtifactFormat::SummaryJson,
            |writer| {
                writer.write_all(b"must not run")?;
                Ok(())
            },
            |_| Ok(()),
        )
        .unwrap_err();

        assert!(format!("{error:#}").contains("requires Linux openat2"));
        assert!(!output.exists());
        fs::remove_dir(directory).unwrap();
    }
}

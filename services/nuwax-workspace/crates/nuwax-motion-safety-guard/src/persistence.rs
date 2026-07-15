//! Durable boot-epoch state for the motion guard.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};

use chrono::{SecondsFormat, Utc};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uuid::Uuid;

use crate::crypto::{constant_time_eq, sha256_hex};
use crate::epoch::{GuardEpochRecord, ProvisionedRecord, SCHEMA_VERSION_PROVISIONED};
use crate::state::{LeaseState, SafetyGate};

const STATE_FILE_LIMIT: u64 = 64 * 1024;
const CHECKSUM_PREFIX: &str = "sha256:";

#[derive(Debug, Serialize)]
struct ProvisionedBody<'a> {
    instance_id: &'a str,
    schema_version: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StoredProvisionedRecord {
    instance_id: String,
    schema_version: u32,
    checksum: String,
}

#[derive(Debug, Serialize)]
struct EpochBody<'a> {
    schema_version: u32,
    generation: u64,
    boot_nonce_hex: &'a str,
    safety_gate: SafetyGate,
    lease_state: LeaseState,
    reset_generation: u64,
    last_zero_ack_id: &'a str,
    peer_uid_allowlist: &'a [String],
    actuator_symbols_sha256: &'a str,
    fenced: bool,
    created_at: &'a str,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StoredEpochRecord {
    schema_version: u32,
    generation: u64,
    boot_nonce_hex: String,
    safety_gate: SafetyGate,
    lease_state: LeaseState,
    reset_generation: u64,
    last_zero_ack_id: String,
    peer_uid_allowlist: Vec<String>,
    actuator_symbols_sha256: String,
    fenced: bool,
    created_at: String,
    checksum: String,
}

impl From<StoredProvisionedRecord> for ProvisionedRecord {
    fn from(value: StoredProvisionedRecord) -> Self {
        Self {
            instance_id: value.instance_id,
            schema_version: value.schema_version,
            checksum: value.checksum,
        }
    }
}

impl From<StoredEpochRecord> for GuardEpochRecord {
    fn from(value: StoredEpochRecord) -> Self {
        Self {
            schema_version: value.schema_version,
            generation: value.generation,
            boot_nonce_hex: value.boot_nonce_hex,
            safety_gate: value.safety_gate,
            lease_state: value.lease_state,
            reset_generation: value.reset_generation,
            last_zero_ack_id: value.last_zero_ack_id,
            peer_uid_allowlist: value.peer_uid_allowlist,
            actuator_symbols_sha256: value.actuator_symbols_sha256,
            fenced: value.fenced,
            created_at: value.created_at,
            checksum: value.checksum,
        }
    }
}

/// Persistent-state failures always make the guard fail closed.
#[derive(Debug, Error)]
pub enum PersistenceError {
    /// A required state file is missing.
    #[error("required state file is missing: {0}")]
    Missing(PathBuf),
    /// A state file is not a regular, owner-private file.
    #[error("state file permissions/type are unsafe: {path}: {reason}")]
    UnsafeFile {
        /// File path.
        path: PathBuf,
        /// Validation failure.
        reason: String,
    },
    /// A state file exceeded its bounded size.
    #[error("state file exceeds {STATE_FILE_LIMIT} bytes: {0}")]
    TooLarge(PathBuf),
    /// JSON could not be decoded.
    #[error("invalid state JSON in {path}: {source}")]
    InvalidJson {
        /// File path.
        path: PathBuf,
        /// Decoder error.
        source: serde_json::Error,
    },
    /// A schema or integrity constraint failed.
    #[error("invalid state record in {path}: {reason}")]
    InvalidRecord {
        /// File path.
        path: PathBuf,
        /// Validation failure.
        reason: String,
    },
    /// Generation cannot advance safely.
    #[error("epoch generation overflow")]
    GenerationOverflow,
    /// Durable file I/O failed.
    #[error("durable state I/O failed for {path}: {source}")]
    Io {
        /// File path.
        path: PathBuf,
        /// Operating-system error.
        source: io::Error,
    },
}

/// Result of loading and durably advancing the boot epoch.
#[derive(Debug)]
pub struct BootEpoch {
    /// The in-memory fail-closed record for this process.
    pub record: GuardEpochRecord,
    /// Store used for later state transitions.
    pub store: EpochStore,
}

/// Atomic epoch writer. `durable == false` permanently disables mutation.
#[derive(Debug)]
pub struct EpochStore {
    epoch_path: PathBuf,
    durable: bool,
    status: String,
}

impl EpochStore {
    /// Loads provisioning/epoch records and commits generation+1 before use.
    /// Any failure returns a volatile RESET_REQUIRED record and a disabled store.
    #[must_use]
    pub fn boot(epoch_path: impl Into<PathBuf>, provisioned_path: impl Into<PathBuf>) -> BootEpoch {
        let epoch_path = epoch_path.into();
        let provisioned_path = provisioned_path.into();
        match boot_durable(&epoch_path, &provisioned_path) {
            Ok(record) => BootEpoch {
                record,
                store: Self {
                    epoch_path,
                    durable: true,
                    status: "OK".to_string(),
                },
            },
            Err(error) => {
                let record = GuardEpochRecord {
                    boot_nonce_hex: nonce_hex(),
                    created_at: now_rfc3339(),
                    fenced: true,
                    ..GuardEpochRecord::default()
                };
                BootEpoch {
                    record,
                    store: Self {
                        epoch_path,
                        durable: false,
                        status: error.to_string(),
                    },
                }
            }
        }
    }

    /// Whether the epoch was loaded and committed durably for this boot.
    #[must_use]
    pub fn is_durable(&self) -> bool {
        self.durable
    }

    /// Stable diagnostic for readiness/error reporting.
    #[must_use]
    pub fn status(&self) -> &str {
        &self.status
    }

    /// Atomically commits a state transition to the current epoch record.
    pub fn commit(&mut self, record: &mut GuardEpochRecord) -> Result<(), PersistenceError> {
        if !self.durable {
            return Err(PersistenceError::InvalidRecord {
                path: self.epoch_path.clone(),
                reason: self.status.clone(),
            });
        }
        record.checksum = epoch_checksum(record)?;
        if let Err(error) = atomic_write_json(&self.epoch_path, record, 0o600) {
            self.durable = false;
            self.status = error.to_string();
            return Err(error);
        }
        Ok(())
    }
}

fn boot_durable(
    epoch_path: &Path,
    provisioned_path: &Path,
) -> Result<GuardEpochRecord, PersistenceError> {
    let provisioned = load_provisioned(provisioned_path)?;
    if provisioned.instance_id.is_empty() || Uuid::parse_str(&provisioned.instance_id).is_err() {
        return Err(invalid_record(
            provisioned_path,
            "instance_id is not a UUID",
        ));
    }

    let mut record = load_epoch(epoch_path)?;
    record.generation = record
        .generation
        .checked_add(1)
        .ok_or(PersistenceError::GenerationOverflow)?;
    record.boot_nonce_hex = nonce_hex();
    record.safety_gate = SafetyGate::ResetRequired;
    record.lease_state = LeaseState::Idle;
    record.fenced = true;
    record.created_at = now_rfc3339();
    record.checksum = epoch_checksum(&record)?;
    atomic_write_json(epoch_path, &record, 0o600)?;
    Ok(record)
}

fn load_provisioned(path: &Path) -> Result<ProvisionedRecord, PersistenceError> {
    let raw = read_private_file(path)?;
    let stored: StoredProvisionedRecord =
        serde_json::from_slice(&raw).map_err(|source| PersistenceError::InvalidJson {
            path: path.to_path_buf(),
            source,
        })?;
    let record = ProvisionedRecord::from(stored);
    if record.schema_version != SCHEMA_VERSION_PROVISIONED {
        return Err(invalid_record(
            path,
            "unsupported provisioned schema_version",
        ));
    }
    let expected = provisioned_checksum(&record)?;
    verify_checksum(path, &record.checksum, &expected)?;
    Ok(record)
}

fn load_epoch(path: &Path) -> Result<GuardEpochRecord, PersistenceError> {
    let raw = read_private_file(path)?;
    let stored: StoredEpochRecord =
        serde_json::from_slice(&raw).map_err(|source| PersistenceError::InvalidJson {
            path: path.to_path_buf(),
            source,
        })?;
    let record = GuardEpochRecord::from(stored);
    let errors = record.verify();
    if !errors.is_empty() {
        return Err(invalid_record(path, errors.join("; ")));
    }
    if record.boot_nonce_hex.len() != 32 && record.generation != 0 {
        return Err(invalid_record(
            path,
            "non-initial epoch requires a 16-byte nonce",
        ));
    }
    if !record
        .boot_nonce_hex
        .bytes()
        .all(|byte| byte.is_ascii_hexdigit())
    {
        return Err(invalid_record(path, "boot_nonce_hex is not hexadecimal"));
    }
    validate_sha256(
        path,
        &record.actuator_symbols_sha256,
        "actuator_symbols_sha256",
    )?;
    let expected = epoch_checksum(&record)?;
    verify_checksum(path, &record.checksum, &expected)?;
    Ok(record)
}

fn provisioned_checksum(record: &ProvisionedRecord) -> Result<String, PersistenceError> {
    let body = ProvisionedBody {
        instance_id: &record.instance_id,
        schema_version: record.schema_version,
    };
    checksum_body(&body, Path::new("provisioned.json"))
}

fn epoch_checksum(record: &GuardEpochRecord) -> Result<String, PersistenceError> {
    let body = EpochBody {
        schema_version: record.schema_version,
        generation: record.generation,
        boot_nonce_hex: &record.boot_nonce_hex,
        safety_gate: record.safety_gate,
        lease_state: record.lease_state,
        reset_generation: record.reset_generation,
        last_zero_ack_id: &record.last_zero_ack_id,
        peer_uid_allowlist: &record.peer_uid_allowlist,
        actuator_symbols_sha256: &record.actuator_symbols_sha256,
        fenced: record.fenced,
        created_at: &record.created_at,
    };
    checksum_body(&body, Path::new("epoch.json"))
}

fn checksum_body<T: Serialize>(body: &T, path: &Path) -> Result<String, PersistenceError> {
    let canonical = serde_json::to_vec(body).map_err(|source| PersistenceError::InvalidJson {
        path: path.to_path_buf(),
        source,
    })?;
    Ok(format!("{CHECKSUM_PREFIX}{}", sha256_hex(&canonical)))
}

fn verify_checksum(path: &Path, actual: &str, expected: &str) -> Result<(), PersistenceError> {
    validate_sha256(path, actual, "checksum")?;
    if !constant_time_eq(actual.as_bytes(), expected.as_bytes()) {
        return Err(invalid_record(path, "checksum mismatch"));
    }
    Ok(())
}

fn validate_sha256(path: &Path, value: &str, field: &str) -> Result<(), PersistenceError> {
    let Some(hex) = value.strip_prefix(CHECKSUM_PREFIX) else {
        return Err(invalid_record(path, format!("{field} lacks sha256 prefix")));
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(invalid_record(
            path,
            format!("{field} is not lowercase SHA-256"),
        ));
    }
    Ok(())
}

fn read_private_file(path: &Path) -> Result<Vec<u8>, PersistenceError> {
    let metadata = fs::symlink_metadata(path).map_err(|source| {
        if source.kind() == io::ErrorKind::NotFound {
            PersistenceError::Missing(path.to_path_buf())
        } else {
            PersistenceError::Io {
                path: path.to_path_buf(),
                source,
            }
        }
    })?;
    if !metadata.file_type().is_file() || metadata.file_type().is_symlink() {
        return Err(PersistenceError::UnsafeFile {
            path: path.to_path_buf(),
            reason: "not a regular file".to_string(),
        });
    }
    if metadata.permissions().mode() & 0o077 != 0 {
        return Err(PersistenceError::UnsafeFile {
            path: path.to_path_buf(),
            reason: "group/world permission bits must be zero".to_string(),
        });
    }
    if metadata.len() > STATE_FILE_LIMIT {
        return Err(PersistenceError::TooLarge(path.to_path_buf()));
    }
    fs::read(path).map_err(|source| PersistenceError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn atomic_write_json<T: Serialize>(
    path: &Path,
    value: &T,
    mode: u32,
) -> Result<(), PersistenceError> {
    let parent = path
        .parent()
        .ok_or_else(|| invalid_record(path, "path has no parent"))?;
    let bytes =
        serde_json::to_vec_pretty(value).map_err(|source| PersistenceError::InvalidJson {
            path: path.to_path_buf(),
            source,
        })?;
    let temp_path = parent.join(format!(
        ".{}.tmp.{}",
        path.file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("state"),
        Uuid::new_v4()
    ));
    let result = (|| {
        let mut file = OpenOptions::new()
            .create_new(true)
            .write(true)
            .mode(mode)
            .open(&temp_path)
            .map_err(|source| PersistenceError::Io {
                path: temp_path.clone(),
                source,
            })?;
        file.write_all(&bytes)
            .and_then(|()| file.write_all(b"\n"))
            .and_then(|()| file.sync_all())
            .map_err(|source| PersistenceError::Io {
                path: temp_path.clone(),
                source,
            })?;
        fs::rename(&temp_path, path).map_err(|source| PersistenceError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        File::open(parent)
            .and_then(|directory| directory.sync_all())
            .map_err(|source| PersistenceError::Io {
                path: parent.to_path_buf(),
                source,
            })
    })();
    if result.is_err() {
        let _ = fs::remove_file(&temp_path);
    }
    result
}

fn nonce_hex() -> String {
    let mut output = String::with_capacity(32);
    for byte in Uuid::new_v4().as_bytes() {
        use std::fmt::Write as _;
        let _ = write!(output, "{byte:02x}");
    }
    output
}

fn now_rfc3339() -> String {
    Utc::now().to_rfc3339_opts(SecondsFormat::Millis, true)
}

fn invalid_record(path: &Path, reason: impl Into<String>) -> PersistenceError {
    PersistenceError::InvalidRecord {
        path: path.to_path_buf(),
        reason: reason.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_private(path: &Path, bytes: &[u8], mode: u32) {
        let mut file = OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .mode(mode)
            .open(path)
            .unwrap();
        file.write_all(bytes).unwrap();
        file.sync_all().unwrap();
        fs::set_permissions(path, fs::Permissions::from_mode(mode)).unwrap();
    }

    fn write_valid_initial_state(directory: &Path) -> (PathBuf, PathBuf) {
        let provisioned_path = directory.join("provisioned.json");
        let epoch_path = directory.join("epoch.json");
        let mut provisioned = ProvisionedRecord {
            instance_id: Uuid::new_v4().to_string(),
            schema_version: SCHEMA_VERSION_PROVISIONED,
            checksum: String::new(),
        };
        provisioned.checksum = provisioned_checksum(&provisioned).unwrap();
        let mut epoch = GuardEpochRecord::default();
        epoch.actuator_symbols_sha256 = format!("sha256:{}", "a".repeat(64));
        epoch.created_at = now_rfc3339();
        epoch.checksum = epoch_checksum(&epoch).unwrap();
        write_private(
            &provisioned_path,
            &serde_json::to_vec_pretty(&provisioned).unwrap(),
            0o400,
        );
        write_private(
            &epoch_path,
            &serde_json::to_vec_pretty(&epoch).unwrap(),
            0o600,
        );
        (epoch_path, provisioned_path)
    }

    #[test]
    fn valid_boot_advances_and_commits_epoch() {
        let directory = tempfile::tempdir().unwrap();
        let (epoch_path, provisioned_path) = write_valid_initial_state(directory.path());
        let boot = EpochStore::boot(&epoch_path, &provisioned_path);
        assert!(boot.store.is_durable(), "{}", boot.store.status());
        assert_eq!(boot.record.generation, 1);
        assert_eq!(boot.record.boot_nonce_hex.len(), 32);
        assert_eq!(boot.record.safety_gate, SafetyGate::ResetRequired);
        assert_eq!(boot.record.lease_state, LeaseState::Idle);
        assert!(boot.record.fenced);

        let persisted = load_epoch(&epoch_path).unwrap();
        assert_eq!(persisted.generation, 1);
        assert_eq!(persisted.boot_nonce_hex, boot.record.boot_nonce_hex);
    }

    #[test]
    fn corrupt_epoch_is_fail_closed_and_not_overwritten() {
        let directory = tempfile::tempdir().unwrap();
        let (epoch_path, provisioned_path) = write_valid_initial_state(directory.path());
        let corrupt = b"{\"schema_version\":1,\"checksum\":\"sha256:bad\"}";
        write_private(&epoch_path, corrupt, 0o600);
        let before = fs::read(&epoch_path).unwrap();

        let boot = EpochStore::boot(&epoch_path, &provisioned_path);
        assert!(!boot.store.is_durable());
        assert_eq!(boot.record.safety_gate, SafetyGate::ResetRequired);
        assert_eq!(boot.record.lease_state, LeaseState::Idle);
        assert!(boot.record.fenced);
        assert_eq!(fs::read(&epoch_path).unwrap(), before);
    }

    #[test]
    fn missing_provisioning_is_fail_closed() {
        let directory = tempfile::tempdir().unwrap();
        let boot = EpochStore::boot(
            directory.path().join("epoch.json"),
            directory.path().join("provisioned.json"),
        );
        assert!(!boot.store.is_durable());
        assert_eq!(boot.record.safety_gate, SafetyGate::ResetRequired);
        assert!(boot.store.status().contains("missing"));
    }

    #[test]
    fn checksum_canonical_body_golden() {
        let record = ProvisionedRecord {
            instance_id: "00000000-0000-4000-8000-000000000000".to_string(),
            schema_version: 1,
            checksum: String::new(),
        };
        assert_eq!(
            provisioned_checksum(&record).unwrap(),
            "sha256:08cecca5d50812c87b4d38973fe3931bbf27aefad53b4e1ed57186664ab12b14"
        );

        let epoch = GuardEpochRecord {
            actuator_symbols_sha256: format!("sha256:{}", "a".repeat(64)),
            created_at: "1970-01-01T00:00:00.000Z".to_string(),
            ..GuardEpochRecord::default()
        };
        assert_eq!(
            epoch_checksum(&epoch).unwrap(),
            "sha256:45fc4d3040276e09d8b9176009ccae8c499e7a6cd44cc06c52074f14142f683c"
        );
    }
}

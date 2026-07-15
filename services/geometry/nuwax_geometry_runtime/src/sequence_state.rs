//! Crash-durable monotonic sequence allocation for the geometry producer.
//!
//! One process holds an exclusive flock for the lifetime of this store. Each
//! reservation is written through a temporary file, fsynced, atomically
//! renamed, and followed by a parent-directory fsync before it becomes visible
//! to callers.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use nix::fcntl::{Flock, FlockArg};
use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use thiserror::Error;

const SEQUENCE_STATE_SCHEMA_VERSION: u32 = 1;
const PRODUCTION_SEQUENCE_STATE_PATH: &str =
    "/var/lib/nuwax_robot_bridge/geometry/sequence_state.json";

static NEXT_TEMP_FILE_ID: AtomicU64 = AtomicU64::new(0);
const TEMP_FILE_CREATE_ATTEMPTS: usize = 128;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
struct SequenceState {
    schema_version: u32,
    map_revision: u64,
    producer_runtime_generation: u64,
}

impl Default for SequenceState {
    fn default() -> Self {
        Self {
            schema_version: SEQUENCE_STATE_SCHEMA_VERSION,
            map_revision: 0,
            producer_runtime_generation: 0,
        }
    }
}

#[derive(Debug)]
struct SequenceStateInner {
    state: SequenceState,
    poisoned: Option<String>,
}

/// Persistent sequence allocation failure. Callers must fail closed and must
/// not project a revision/generation that was not returned successfully.
#[derive(Debug, Error)]
pub(crate) enum SequenceStateError {
    #[error("sequence state path has no usable parent: {0}")]
    InvalidPath(PathBuf),
    #[error("sequence state I/O failed at {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("sequence state is already locked by another geometry process: {0}")]
    AlreadyLocked(PathBuf),
    #[error("sequence state JSON is invalid at {path}: {reason}")]
    InvalidState { path: PathBuf, reason: String },
    #[error("sequence allocator is poisoned after an uncertain persistence result: {0}")]
    Poisoned(String),
    #[error("positive LCM i32 map revision domain is exhausted (floor={floor})")]
    MapRevisionExhausted { floor: u64 },
    #[error("GoalEnvelope producer generation domain is exhausted (floor={floor})")]
    ProducerGenerationExhausted { floor: u64 },
}

/// Process-exclusive, crash-durable sequence store shared by mapping and nav.
pub(crate) struct SequenceStateStore {
    path: PathBuf,
    parent: PathBuf,
    _process_lock: Flock<File>,
    inner: Mutex<SequenceStateInner>,
}

impl SequenceStateStore {
    /// Open and exclusively lock a store. A missing file is initialized
    /// durably, so startup also proves that the configured directory is
    /// writable before the runtime advertises readiness.
    pub(crate) fn open(path: impl Into<PathBuf>) -> Result<Self, SequenceStateError> {
        let path = path.into();
        let parent = usable_parent(&path)?;
        fs::create_dir_all(&parent).map_err(|source| SequenceStateError::Io {
            path: parent.clone(),
            source,
        })?;

        let lock_path = lock_path_for(&path)?;
        let lock_file = OpenOptions::new()
            .create(true)
            .truncate(false)
            .read(true)
            .write(true)
            .open(&lock_path)
            .map_err(|source| SequenceStateError::Io {
                path: lock_path.clone(),
                source,
            })?;
        let process_lock = Flock::lock(lock_file, FlockArg::LockExclusiveNonblock)
            .map_err(|_| SequenceStateError::AlreadyLocked(lock_path))?;

        let state = match fs::read(&path) {
            Ok(bytes) => decode_state(&path, &bytes)?,
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                let initial = SequenceState::default();
                persist_state(&path, &parent, &initial)?;
                initial
            }
            Err(source) => {
                return Err(SequenceStateError::Io {
                    path: path.clone(),
                    source,
                });
            }
        };

        Ok(Self {
            path,
            parent,
            _process_lock: process_lock,
            inner: Mutex::new(SequenceStateInner {
                state,
                poisoned: None,
            }),
        })
    }

    /// Persist and reserve the next globally monotonic positive LCM i32 map
    /// revision above both the durable state and the supplied local floor.
    pub(crate) fn reserve_map_revision(&self, floor: u64) -> Result<u64, SequenceStateError> {
        let mut inner = self.inner.lock();
        ensure_usable(&inner)?;
        let effective_floor = inner.state.map_revision.max(floor);
        let next =
            effective_floor
                .checked_add(1)
                .ok_or(SequenceStateError::MapRevisionExhausted {
                    floor: effective_floor,
                })?;
        let wire_revision =
            i32::try_from(next).map_err(|_| SequenceStateError::MapRevisionExhausted {
                floor: effective_floor,
            })?;
        if wire_revision <= 0 {
            return Err(SequenceStateError::MapRevisionExhausted {
                floor: effective_floor,
            });
        }

        let mut candidate = inner.state.clone();
        candidate.map_revision = next;
        self.persist_or_poison(&mut inner, &candidate)?;
        inner.state = candidate;
        Ok(next)
    }

    /// Persist and reserve the next positive producer epoch representable by
    /// GoalEnvelope's signed i64 wire field.
    pub(crate) fn reserve_producer_runtime_generation(&self) -> Result<u64, SequenceStateError> {
        let mut inner = self.inner.lock();
        ensure_usable(&inner)?;
        let floor = inner.state.producer_runtime_generation;
        let next = floor
            .checked_add(1)
            .ok_or(SequenceStateError::ProducerGenerationExhausted { floor })?;
        let wire_generation = i64::try_from(next)
            .map_err(|_| SequenceStateError::ProducerGenerationExhausted { floor })?;
        if wire_generation <= 0 {
            return Err(SequenceStateError::ProducerGenerationExhausted { floor });
        }

        let mut candidate = inner.state.clone();
        candidate.producer_runtime_generation = next;
        self.persist_or_poison(&mut inner, &candidate)?;
        inner.state = candidate;
        Ok(next)
    }

    pub(crate) fn current_map_revision(&self) -> u64 {
        self.inner.lock().state.map_revision
    }

    #[cfg(test)]
    pub(crate) fn current_producer_runtime_generation(&self) -> u64 {
        self.inner.lock().state.producer_runtime_generation
    }

    fn persist_or_poison(
        &self,
        inner: &mut SequenceStateInner,
        candidate: &SequenceState,
    ) -> Result<(), SequenceStateError> {
        if let Err(error) = persist_state(&self.path, &self.parent, candidate) {
            inner.poisoned = Some(error.to_string());
            return Err(error);
        }
        Ok(())
    }
}

pub(crate) fn sequence_state_path_from_env() -> PathBuf {
    std::env::var_os("NUWAX_GEOMETRY_SEQUENCE_STATE_PATH")
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(PRODUCTION_SEQUENCE_STATE_PATH))
}

fn ensure_usable(inner: &SequenceStateInner) -> Result<(), SequenceStateError> {
    match inner.poisoned.as_ref() {
        Some(reason) => Err(SequenceStateError::Poisoned(reason.clone())),
        None => Ok(()),
    }
}

fn usable_parent(path: &Path) -> Result<PathBuf, SequenceStateError> {
    let parent = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .ok_or_else(|| SequenceStateError::InvalidPath(path.to_path_buf()))?;
    Ok(parent.to_path_buf())
}

fn lock_path_for(path: &Path) -> Result<PathBuf, SequenceStateError> {
    let parent = usable_parent(path)?;
    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .filter(|name| !name.is_empty())
        .ok_or_else(|| SequenceStateError::InvalidPath(path.to_path_buf()))?;
    Ok(parent.join(format!("{file_name}.lock")))
}

fn decode_state(path: &Path, bytes: &[u8]) -> Result<SequenceState, SequenceStateError> {
    let state: SequenceState =
        serde_json::from_slice(bytes).map_err(|error| SequenceStateError::InvalidState {
            path: path.to_path_buf(),
            reason: error.to_string(),
        })?;
    if state.schema_version != SEQUENCE_STATE_SCHEMA_VERSION {
        return Err(SequenceStateError::InvalidState {
            path: path.to_path_buf(),
            reason: format!("unsupported schema_version {}", state.schema_version),
        });
    }
    if state.map_revision > i32::MAX as u64 {
        return Err(SequenceStateError::InvalidState {
            path: path.to_path_buf(),
            reason: format!(
                "map_revision {} exceeds positive LCM i32 domain",
                state.map_revision
            ),
        });
    }
    if state.producer_runtime_generation > i64::MAX as u64 {
        return Err(SequenceStateError::InvalidState {
            path: path.to_path_buf(),
            reason: format!(
                "producer_runtime_generation {} exceeds GoalEnvelope i64 domain",
                state.producer_runtime_generation
            ),
        });
    }
    Ok(state)
}

fn persist_state(
    path: &Path,
    parent: &Path,
    state: &SequenceState,
) -> Result<(), SequenceStateError> {
    let bytes = serde_json::to_vec(state).map_err(|error| SequenceStateError::InvalidState {
        path: path.to_path_buf(),
        reason: error.to_string(),
    })?;
    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| SequenceStateError::InvalidPath(path.to_path_buf()))?;
    let mut selected = None;
    let mut last_collision = None;
    for _ in 0..TEMP_FILE_CREATE_ATTEMPTS {
        let candidate = parent.join(format!(
            ".{file_name}.tmp.{}.{}",
            std::process::id(),
            NEXT_TEMP_FILE_ID.fetch_add(1, Ordering::Relaxed),
        ));
        match OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&candidate)
        {
            Ok(file) => {
                selected = Some((candidate, file));
                break;
            }
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {
                last_collision = Some((candidate, error));
            }
            Err(source) => {
                return Err(SequenceStateError::Io {
                    path: candidate,
                    source,
                });
            }
        }
    }
    let (temp_path, mut file) = selected.ok_or_else(|| {
        let (path, source) = last_collision.expect("at least one temp create attempt");
        SequenceStateError::Io { path, source }
    })?;

    let write_result = (|| {
        file.write_all(&bytes)
            .and_then(|_| file.write_all(b"\n"))
            .and_then(|_| file.sync_all())
            .map_err(|source| SequenceStateError::Io {
                path: temp_path.clone(),
                source,
            })?;
        fs::rename(&temp_path, path).map_err(|source| SequenceStateError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        File::open(parent)
            .and_then(|directory| directory.sync_all())
            .map_err(|source| SequenceStateError::Io {
                path: parent.to_path_buf(),
                source,
            })
    })();

    if write_result.is_err() {
        let _ = fs::remove_file(&temp_path);
    }
    write_result
}

#[cfg(test)]
mod tests {
    use super::*;

    static NEXT_TEST_DIR: AtomicU64 = AtomicU64::new(0);

    fn test_path(label: &str) -> PathBuf {
        std::env::temp_dir()
            .join(format!(
                "nuwax-sequence-state-{}-{}-{}",
                std::process::id(),
                NEXT_TEST_DIR.fetch_add(1, Ordering::Relaxed),
                label
            ))
            .join("sequence_state.json")
    }

    #[test]
    fn production_default_uses_systemd_writable_state_root() {
        assert_eq!(
            Path::new(PRODUCTION_SEQUENCE_STATE_PATH),
            Path::new("/var/lib/nuwax_robot_bridge/geometry/sequence_state.json")
        );
    }

    #[test]
    fn reservations_survive_store_restart_without_regression() {
        let path = test_path("restart");
        let first = SequenceStateStore::open(&path).expect("first store");
        assert_eq!(first.reserve_producer_runtime_generation().unwrap(), 1);
        assert_eq!(first.reserve_map_revision(0).unwrap(), 1);
        drop(first);

        let second = SequenceStateStore::open(&path).expect("restarted store");
        assert_eq!(second.current_producer_runtime_generation(), 1);
        assert_eq!(second.current_map_revision(), 1);
        assert_eq!(second.reserve_producer_runtime_generation().unwrap(), 2);
        assert_eq!(second.reserve_map_revision(0).unwrap(), 2);
    }

    #[test]
    fn orphaned_next_temp_file_does_not_poison_persistence_or_regress_watermarks() {
        let path = test_path("orphan-temp");
        let parent = path.parent().expect("test state parent");
        fs::create_dir_all(parent).expect("create state parent");
        let next_id = NEXT_TEMP_FILE_ID.load(Ordering::Relaxed);
        let file_name = path.file_name().unwrap().to_string_lossy();
        let orphan = parent.join(format!(
            ".{file_name}.tmp.{}.{}",
            std::process::id(),
            next_id
        ));
        fs::write(&orphan, b"crash residue must never become authoritative")
            .expect("precreate colliding orphan");

        let first = SequenceStateStore::open(&path)
            .expect("a colliding orphan must not poison initial persistence");
        assert_eq!(first.reserve_map_revision(0).unwrap(), 1);
        assert_eq!(first.reserve_producer_runtime_generation().unwrap(), 1);
        drop(first);

        let reopened = SequenceStateStore::open(&path).expect("reopen durable state");
        assert_eq!(reopened.current_map_revision(), 1);
        assert_eq!(reopened.current_producer_runtime_generation(), 1);
        assert_eq!(reopened.reserve_map_revision(0).unwrap(), 2);
        assert_eq!(reopened.reserve_producer_runtime_generation().unwrap(), 2);
    }

    #[test]
    fn concurrent_process_owner_is_rejected_until_lock_is_released() {
        let path = test_path("lock");
        let first = SequenceStateStore::open(&path).expect("first owner");
        assert!(matches!(
            SequenceStateStore::open(&path),
            Err(SequenceStateError::AlreadyLocked(_))
        ));
        drop(first);
        SequenceStateStore::open(&path).expect("lock released after owner drop");
    }

    #[test]
    fn map_revision_boundaries_fail_without_advancing_state() {
        let path = test_path("map-boundary");
        let store = SequenceStateStore::open(&path).expect("store");
        assert!(matches!(
            store.reserve_map_revision(u64::MAX),
            Err(SequenceStateError::MapRevisionExhausted { floor: u64::MAX })
        ));
        assert_eq!(store.current_map_revision(), 0);
        assert_eq!(
            store.reserve_map_revision(i32::MAX as u64 - 1).unwrap(),
            i32::MAX as u64
        );
        assert!(matches!(
            store.reserve_map_revision(0),
            Err(SequenceStateError::MapRevisionExhausted { .. })
        ));
        assert_eq!(store.current_map_revision(), i32::MAX as u64);
    }

    #[test]
    fn producer_boundary_fails_without_advancing_state() {
        let path = test_path("producer-boundary");
        let parent = path.parent().unwrap();
        fs::create_dir_all(parent).unwrap();
        fs::write(
            &path,
            serde_json::to_vec(&SequenceState {
                schema_version: SEQUENCE_STATE_SCHEMA_VERSION,
                map_revision: 0,
                producer_runtime_generation: i64::MAX as u64,
            })
            .unwrap(),
        )
        .unwrap();
        let store = SequenceStateStore::open(&path).expect("max producer store");
        assert!(matches!(
            store.reserve_producer_runtime_generation(),
            Err(SequenceStateError::ProducerGenerationExhausted { .. })
        ));
        assert_eq!(store.current_producer_runtime_generation(), i64::MAX as u64);
    }

    #[test]
    fn persistence_failure_poisoning_prevents_retry_or_memory_advance() {
        let path = test_path("poison");
        let parent = path.parent().unwrap().to_path_buf();
        let lock_path = lock_path_for(&path).unwrap();
        let store = SequenceStateStore::open(&path).expect("store");
        fs::remove_file(&path).unwrap();
        fs::remove_file(lock_path).unwrap();
        fs::remove_dir(&parent).unwrap();
        fs::write(&parent, b"not a directory").unwrap();

        assert!(matches!(
            store.reserve_map_revision(0),
            Err(SequenceStateError::Io { .. })
        ));
        assert_eq!(store.current_map_revision(), 0);
        assert!(matches!(
            store.reserve_map_revision(0),
            Err(SequenceStateError::Poisoned(_))
        ));
    }
}

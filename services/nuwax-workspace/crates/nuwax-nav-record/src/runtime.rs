//! Lifecycle owner for recording typed payloads into the framed log.

use crate::{
    NavRecord, NavRecordError, NavRecordPose, NavRecordProfile, NavRecordWriter, SidecarLock,
};
use nuwax_core::{OutputPort, StopHandle};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};
use thiserror::Error;

/// Existing-file behavior applied when the recorder starts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OnExisting {
    /// Truncate and replace the current file.
    Overwrite,
    /// Reject startup when the path already exists.
    Error,
    /// Move the current file to a retained backup before creating a new file.
    Backup,
    /// Validate the existing file and append new records.
    Append,
}

/// Runtime-only recorder policy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NavRecorderConfig {
    /// Existing-file policy.
    pub on_existing: OnExisting,
    /// Number of newest backups retained by `Backup`; zero removes the old file.
    pub backup_keep_last: usize,
    /// Frame used when an input has no frame metadata.
    pub default_frame_id: String,
    /// Incoming stream/topic name to persisted profile topic name.
    pub stream_remapping: BTreeMap<String, String>,
    /// Flush every accepted record instead of flushing only during stop.
    pub flush_on_write: bool,
}

impl Default for NavRecorderConfig {
    fn default() -> Self {
        Self {
            on_existing: OnExisting::Backup,
            backup_keep_last: 10,
            default_frame_id: "base_link".to_string(),
            stream_remapping: BTreeMap::new(),
            flush_on_write: false,
        }
    }
}

/// Typed transport envelope accepted by the recorder owner.
#[derive(Debug, Clone, PartialEq)]
pub struct NavRecordInput {
    /// Incoming stream or topic name before recorder-local remapping.
    pub stream: String,
    /// Declared wire payload type.
    pub msg_type: String,
    /// Source timestamp in nanoseconds, or current system time when omitted.
    pub timestamp_ns: Option<u64>,
    /// Optional source frame; the runtime default is used when omitted.
    pub frame_id: Option<String>,
    /// Optional reference-frame pose metadata.
    pub pose: Option<NavRecordPose>,
    /// Opaque typed wire payload.
    pub payload: Vec<u8>,
}

/// Recorder lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecorderState {
    /// Constructed and available for source binding.
    Created,
    /// Existing-file policy and writer creation are in progress.
    Starting,
    /// Accepting records.
    Running,
    /// Writer flush and subscription teardown are in progress.
    Stopping,
    /// Cleanly stopped; restart is intentionally rejected.
    Stopped,
    /// Start, write, or stop failed.
    Failed,
}

/// Snapshot of recorder health and counters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NavRecorderDiagnostics {
    /// Current lifecycle state.
    pub state: RecorderState,
    /// Active output subscriptions owned by the runtime.
    pub active_bindings: usize,
    /// Accepted and persisted records.
    pub records_written: u64,
    /// Writes rejected by lifecycle or persistence validation.
    pub rejected_writes: u64,
    /// Last accepted timestamp.
    pub last_timestamp_ns: Option<u64>,
    /// Last persisted topic after remapping.
    pub last_topic: Option<String>,
    /// Most recent failure detail.
    pub last_error: Option<String>,
    /// Number of backups created by this owner instance.
    pub backups_created: u64,
    /// Active output path.
    pub path: PathBuf,
}

/// Runtime lifecycle or persistence failure.
#[derive(Debug, Error)]
pub enum RecorderRuntimeError {
    /// Operation is invalid for the current lifecycle state.
    #[error("recorder operation requires {expected}, current state is {actual:?}")]
    InvalidState {
        /// Required state description.
        expected: &'static str,
        /// Observed state.
        actual: RecorderState,
    },
    /// Existing-file policy rejected startup.
    #[error("recording already exists: {0}")]
    AlreadyExists(PathBuf),
    /// System time could not be represented as Unix nanoseconds.
    #[error("system clock is before the Unix epoch or timestamp overflowed")]
    InvalidSystemTime,
    /// Core port binding failed.
    #[error("recorder source binding failed: {0}")]
    Binding(String),
    /// Framed log operation failed.
    #[error(transparent)]
    Persistence(#[from] NavRecordError),
    /// File policy operation failed.
    #[error("recorder file operation failed: {0}")]
    Io(#[from] std::io::Error),
}

struct RuntimeInner {
    state: RecorderState,
    writer: Option<NavRecordWriter>,
    bindings: Vec<StopHandle>,
    records_written: u64,
    rejected_writes: u64,
    last_timestamp_ns: Option<u64>,
    last_topic: Option<String>,
    last_error: Option<String>,
    backups_created: u64,
}

/// Start/stop owner for navigation recording subscriptions and persistence.
pub struct NavRecorderRuntime {
    profile: NavRecordProfile,
    config: NavRecorderConfig,
    path: PathBuf,
    inner: Mutex<RuntimeInner>,
}

impl NavRecorderRuntime {
    /// Creates an inactive owner using `profile.db_path` as its output path.
    #[must_use]
    pub fn new(profile: NavRecordProfile, config: NavRecorderConfig) -> Self {
        let path = PathBuf::from(&profile.db_path);
        Self {
            profile,
            config,
            path,
            inner: Mutex::new(RuntimeInner {
                state: RecorderState::Created,
                writer: None,
                bindings: Vec::new(),
                records_written: 0,
                rejected_writes: 0,
                last_timestamp_ns: None,
                last_topic: None,
                last_error: None,
                backups_created: 0,
            }),
        }
    }

    /// Binds a typed core output to this recorder and owns the unsubscribe handle.
    ///
    /// Binding is allowed before or after `start`; messages received outside the running
    /// state are rejected and counted. Stop drops every owned binding.
    pub fn bind_source(
        self: &Arc<Self>,
        source: &OutputPort<NavRecordInput>,
    ) -> Result<(), RecorderRuntimeError> {
        let mut inner = lock_unpoisoned(&self.inner);
        if matches!(
            inner.state,
            RecorderState::Stopping | RecorderState::Stopped | RecorderState::Failed
        ) {
            return Err(RecorderRuntimeError::InvalidState {
                expected: "created or running",
                actual: inner.state,
            });
        }
        let owner = Arc::clone(self);
        let binding = source
            .subscribe(move |input| {
                let _ = owner.record(input);
            })
            .map_err(|error| RecorderRuntimeError::Binding(error.to_string()))?;
        inner.bindings.push(binding);
        Ok(())
    }

    /// Applies the existing-file policy and starts accepting records.
    pub fn start(&self) -> Result<(), RecorderRuntimeError> {
        let mut inner = lock_unpoisoned(&self.inner);
        if inner.state != RecorderState::Created {
            return Err(RecorderRuntimeError::InvalidState {
                expected: "created",
                actual: inner.state,
            });
        }
        inner.state = RecorderState::Starting;
        let writer = self.prepare_writer(&mut inner);
        match writer {
            Ok(writer) => {
                inner.writer = Some(writer);
                inner.state = RecorderState::Running;
                inner.last_error = None;
                Ok(())
            }
            Err(error) => {
                inner.state = RecorderState::Failed;
                inner.last_error = Some(error.to_string());
                inner.bindings.clear();
                Err(error)
            }
        }
    }

    /// Persists one typed input after remapping and metadata resolution.
    pub fn record(&self, input: NavRecordInput) -> Result<(), RecorderRuntimeError> {
        let mut inner = lock_unpoisoned(&self.inner);
        if inner.state != RecorderState::Running {
            inner.rejected_writes = inner.rejected_writes.saturating_add(1);
            return Err(RecorderRuntimeError::InvalidState {
                expected: "running",
                actual: inner.state,
            });
        }
        let timestamp_ns = match input.timestamp_ns {
            Some(timestamp_ns) => timestamp_ns,
            None => unix_timestamp_ns()?,
        };
        let topic = self
            .config
            .stream_remapping
            .get(&input.stream)
            .cloned()
            .unwrap_or(input.stream);
        let mut record = NavRecord::new(timestamp_ns, topic.clone(), input.msg_type, input.payload);
        record.frame_id = Some(
            input
                .frame_id
                .unwrap_or_else(|| self.config.default_frame_id.clone()),
        );
        record.pose = input.pose;

        let result = inner
            .writer
            .as_mut()
            .expect("running recorder must own a writer")
            .append(&record)
            .and_then(|()| {
                if self.config.flush_on_write {
                    inner
                        .writer
                        .as_mut()
                        .expect("running recorder must own a writer")
                        .flush()
                } else {
                    Ok(())
                }
            });
        if let Err(error) = result {
            inner.rejected_writes = inner.rejected_writes.saturating_add(1);
            inner.last_error = Some(error.to_string());
            inner.state = RecorderState::Failed;
            inner.bindings.clear();
            return Err(error.into());
        }
        inner.records_written = inner.records_written.saturating_add(1);
        inner.last_timestamp_ns = Some(timestamp_ns);
        inner.last_topic = Some(topic);
        Ok(())
    }

    /// Flushes the writer, removes every source subscription, and rejects future writes.
    pub fn stop(&self) -> Result<(), RecorderRuntimeError> {
        let mut inner = lock_unpoisoned(&self.inner);
        if inner.state != RecorderState::Running {
            return Err(RecorderRuntimeError::InvalidState {
                expected: "running",
                actual: inner.state,
            });
        }
        inner.state = RecorderState::Stopping;
        inner.bindings.clear();
        let result = inner
            .writer
            .as_mut()
            .expect("stopping recorder must own a writer")
            .flush();
        inner.writer = None;
        match result {
            Ok(()) => {
                inner.state = RecorderState::Stopped;
                Ok(())
            }
            Err(error) => {
                inner.state = RecorderState::Failed;
                inner.last_error = Some(error.to_string());
                Err(error.into())
            }
        }
    }

    /// Returns a consistent lifecycle and health snapshot.
    #[must_use]
    pub fn diagnostics(&self) -> NavRecorderDiagnostics {
        let inner = lock_unpoisoned(&self.inner);
        NavRecorderDiagnostics {
            state: inner.state,
            active_bindings: inner.bindings.len(),
            records_written: inner.records_written,
            rejected_writes: inner.rejected_writes,
            last_timestamp_ns: inner.last_timestamp_ns,
            last_topic: inner.last_topic.clone(),
            last_error: inner.last_error.clone(),
            backups_created: inner.backups_created,
            path: self.path.clone(),
        }
    }

    fn prepare_writer(
        &self,
        inner: &mut RuntimeInner,
    ) -> Result<NavRecordWriter, RecorderRuntimeError> {
        self.profile.validate()?;
        let path_lock = SidecarLock::acquire(&self.path)?;
        if self.path.exists() {
            match self.config.on_existing {
                OnExisting::Overwrite => {
                    return Ok(NavRecordWriter::create_locked(
                        &self.path,
                        &self.profile,
                        path_lock,
                    )?)
                }
                OnExisting::Error => {
                    return Err(RecorderRuntimeError::AlreadyExists(self.path.clone()))
                }
                OnExisting::Backup => {
                    if backup_existing(&self.path, self.config.backup_keep_last)? {
                        inner.backups_created = inner.backups_created.saturating_add(1);
                    }
                }
                OnExisting::Append => {
                    return Ok(NavRecordWriter::open_append_locked(
                        &self.path,
                        &self.profile,
                        path_lock,
                    )?)
                }
            }
        }
        Ok(NavRecordWriter::create_locked(
            &self.path,
            &self.profile,
            path_lock,
        )?)
    }
}

fn backup_existing(path: &Path, keep_last: usize) -> Result<bool, std::io::Error> {
    if keep_last == 0 {
        fs::remove_file(path)?;
        return Ok(false);
    }
    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("recording");
    let prefix = format!("{file_name}.bak.");
    let mut suffix = unix_timestamp_ns().unwrap_or(0);
    let backup = loop {
        let candidate = parent.join(format!("{prefix}{suffix:020}"));
        if !candidate.exists() {
            break candidate;
        }
        suffix = suffix.saturating_add(1);
    };
    fs::rename(path, &backup)?;

    let mut backups = collect_backup_paths(
        fs::read_dir(parent)?.map(|entry| entry.map(|entry| entry.path())),
        &prefix,
    )?;
    backups.sort();
    let remove_count = backups.len().saturating_sub(keep_last);
    for old in backups.into_iter().take(remove_count) {
        fs::remove_file(old)?;
    }
    Ok(true)
}

fn collect_backup_paths(
    entries: impl IntoIterator<Item = Result<PathBuf, std::io::Error>>,
    prefix: &str,
) -> Result<Vec<PathBuf>, std::io::Error> {
    entries
        .into_iter()
        .collect::<Result<Vec<_>, _>>()
        .map(|paths| {
            paths
                .into_iter()
                .filter(|candidate| {
                    candidate
                        .file_name()
                        .and_then(|name| name.to_str())
                        .is_some_and(|name| name.starts_with(prefix))
                })
                .collect()
        })
}

fn unix_timestamp_ns() -> Result<u64, RecorderRuntimeError> {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| RecorderRuntimeError::InvalidSystemTime)?
        .as_nanos();
    u64::try_from(nanos).map_err(|_| RecorderRuntimeError::InvalidSystemTime)
}

fn lock_unpoisoned<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{NavRecordReader, NavRecordTopic};
    use nuwax_codec::lcm::schema::{decode_twist, encode_twist, Twist};
    use std::sync::atomic::{AtomicU64, Ordering};

    static NEXT_PATH: AtomicU64 = AtomicU64::new(0);

    struct TempRecording(PathBuf);

    impl TempRecording {
        fn new() -> Self {
            let id = NEXT_PATH.fetch_add(1, Ordering::Relaxed);
            Self(std::env::temp_dir().join(format!(
                "nuwax-nav-recorder-runtime-{}-{id}.bin",
                std::process::id()
            )))
        }

        fn backups(&self) -> Vec<PathBuf> {
            let parent = self.0.parent().unwrap();
            let prefix = format!("{}.bak.", self.0.file_name().unwrap().to_string_lossy());
            fs::read_dir(parent)
                .unwrap()
                .filter_map(Result::ok)
                .map(|entry| entry.path())
                .filter(|path| {
                    path.file_name()
                        .and_then(|name| name.to_str())
                        .is_some_and(|name| name.starts_with(&prefix))
                })
                .collect()
        }
    }

    impl Drop for TempRecording {
        fn drop(&mut self) {
            let _ = fs::remove_file(&self.0);
            let _ = fs::remove_file(format!("{}.lock", self.0.display()));
            for backup in self.backups() {
                let _ = fs::remove_file(backup);
            }
        }
    }

    fn profile(path: &Path) -> NavRecordProfile {
        NavRecordProfile {
            name: "runtime-test".into(),
            db_path: path.to_string_lossy().into_owned(),
            topics: vec![NavRecordTopic {
                topic: "cmd_vel".into(),
                msg_type: "geometry_msgs/Twist".into(),
            }],
        }
    }

    fn input(payload: Vec<u8>, timestamp_ns: u64) -> NavRecordInput {
        NavRecordInput {
            stream: "raw_cmd".into(),
            msg_type: "geometry_msgs/Twist".into(),
            timestamp_ns: Some(timestamp_ns),
            frame_id: Some("base_link".into()),
            pose: Some(NavRecordPose {
                reference_frame: "map".into(),
                position_xyz: [1.0, 2.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            }),
            payload,
        }
    }

    fn config(on_existing: OnExisting) -> NavRecorderConfig {
        NavRecorderConfig {
            on_existing,
            stream_remapping: BTreeMap::from([("raw_cmd".into(), "cmd_vel".into())]),
            ..NavRecorderConfig::default()
        }
    }

    #[test]
    fn backup_scan_propagates_directory_entry_errors() {
        let injected = std::io::Error::new(std::io::ErrorKind::PermissionDenied, "injected");
        let result = collect_backup_paths(
            [Ok(PathBuf::from("recording.bak.1")), Err(injected)],
            "recording.bak.",
        );
        assert_eq!(
            result.unwrap_err().kind(),
            std::io::ErrorKind::PermissionDenied
        );
    }

    #[test]
    fn typed_core_boundary_records_reopens_and_replays_codec_payload() {
        let temp = TempRecording::new();
        let profile = profile(&temp.0);
        let runtime = Arc::new(NavRecorderRuntime::new(
            profile.clone(),
            config(OnExisting::Overwrite),
        ));
        let source = OutputPort::<NavRecordInput>::new("cmd_vel_source");
        runtime.bind_source(&source).unwrap();
        runtime.start().unwrap();

        let twist = Twist {
            linear: [0.4, -0.2, 0.0],
            angular: [0.0, 0.0, 0.7],
        };
        assert_eq!(
            source
                .publish(input(encode_twist(&twist), 123))
                .unwrap()
                .local_deliveries,
            1
        );
        runtime.stop().unwrap();
        assert_eq!(
            source.publish(input(vec![], 124)).unwrap().local_deliveries,
            0
        );
        assert!(runtime.record(input(vec![], 125)).is_err());

        let reader = NavRecordReader::open(&temp.0, &profile).unwrap();
        assert_eq!(reader.records().len(), 1);
        let record = &reader.records()[0];
        assert_eq!(record.topic, "cmd_vel");
        assert_eq!(record.timestamp_ns, 123);
        assert_eq!(record.frame_id.as_deref(), Some("base_link"));
        assert_eq!(record.pose.as_ref().unwrap().reference_frame, "map");
        assert_eq!(decode_twist(&record.payload).unwrap(), twist);

        let diagnostics = runtime.diagnostics();
        assert_eq!(diagnostics.state, RecorderState::Stopped);
        assert_eq!(diagnostics.records_written, 1);
        assert_eq!(diagnostics.rejected_writes, 1);
        assert_eq!(diagnostics.active_bindings, 0);
        assert_eq!(diagnostics.last_topic.as_deref(), Some("cmd_vel"));
    }

    #[test]
    fn existing_file_error_overwrite_backup_and_append_policies() {
        let temp = TempRecording::new();
        let profile = profile(&temp.0);
        fs::write(&temp.0, b"old-content").unwrap();

        let error_owner = NavRecorderRuntime::new(profile.clone(), config(OnExisting::Error));
        assert!(matches!(
            error_owner.start(),
            Err(RecorderRuntimeError::AlreadyExists(_))
        ));
        assert_eq!(fs::read(&temp.0).unwrap(), b"old-content");
        assert_eq!(error_owner.diagnostics().state, RecorderState::Failed);

        let backup_owner = NavRecorderRuntime::new(profile.clone(), config(OnExisting::Backup));
        backup_owner.start().unwrap();
        backup_owner.stop().unwrap();
        assert_eq!(backup_owner.diagnostics().backups_created, 1);
        assert_eq!(temp.backups().len(), 1);

        let overwrite_owner =
            NavRecorderRuntime::new(profile.clone(), config(OnExisting::Overwrite));
        overwrite_owner.start().unwrap();
        overwrite_owner
            .record(input(encode_twist(&Twist::default()), 1))
            .unwrap();
        overwrite_owner.stop().unwrap();

        let append_owner = NavRecorderRuntime::new(profile.clone(), config(OnExisting::Append));
        append_owner.start().unwrap();
        append_owner
            .record(input(encode_twist(&Twist::default()), 2))
            .unwrap();
        append_owner.stop().unwrap();
        assert_eq!(
            NavRecordReader::open(&temp.0, &profile)
                .unwrap()
                .records()
                .len(),
            2
        );
    }

    #[test]
    fn backup_waits_for_direct_writer_and_preserves_locked_inode() {
        use std::sync::mpsc::{self, RecvTimeoutError};
        use std::time::Duration;

        let temp = TempRecording::new();
        let profile = profile(&temp.0);
        let mut direct_writer = NavRecordWriter::create(&temp.0, &profile).unwrap();
        direct_writer
            .append(&NavRecord::new(
                1,
                "cmd_vel",
                "geometry_msgs/Twist",
                encode_twist(&Twist::default()),
            ))
            .unwrap();
        direct_writer.flush().unwrap();
        let locked_bytes = fs::read(&temp.0).unwrap();

        let competitor_profile = profile.clone();
        let (sender, receiver) = mpsc::sync_channel(1);
        let competitor = std::thread::spawn(move || {
            let runtime = NavRecorderRuntime::new(competitor_profile, config(OnExisting::Backup));
            let result = runtime.start();
            sender.send((runtime, result)).unwrap();
        });

        assert!(matches!(
            receiver.recv_timeout(Duration::from_millis(100)),
            Err(RecvTimeoutError::Timeout)
        ));
        assert_eq!(fs::read(&temp.0).unwrap(), locked_bytes);
        assert!(temp.backups().is_empty());

        drop(direct_writer);
        let (runtime, result) = receiver
            .recv_timeout(Duration::from_secs(2))
            .expect("backup runtime should acquire the released sidecar lock");
        result.unwrap();
        competitor.join().unwrap();

        let backups = temp.backups();
        assert_eq!(backups.len(), 1);
        assert_eq!(fs::read(&backups[0]).unwrap(), locked_bytes);
        assert_eq!(
            NavRecordReader::open(&backups[0], &profile)
                .unwrap()
                .records()
                .len(),
            1
        );
        runtime
            .record(input(encode_twist(&Twist::default()), 2))
            .unwrap();
        runtime.stop().unwrap();
        assert_eq!(
            NavRecordReader::open(&temp.0, &profile)
                .unwrap()
                .records()
                .len(),
            1
        );
    }

    #[test]
    fn omitted_timestamp_and_frame_are_resolved_while_running() {
        let temp = TempRecording::new();
        let profile = profile(&temp.0);
        let owner = NavRecorderRuntime::new(profile.clone(), config(OnExisting::Overwrite));
        owner.start().unwrap();
        let mut value = input(encode_twist(&Twist::default()), 0);
        value.timestamp_ns = None;
        value.frame_id = None;
        owner.record(value).unwrap();
        owner.stop().unwrap();

        let reader = NavRecordReader::open(&temp.0, &profile).unwrap();
        assert!(reader.records()[0].timestamp_ns > 0);
        assert_eq!(reader.records()[0].frame_id.as_deref(), Some("base_link"));
    }
}

//! Durable publication for mapping CLI file artifacts.
//!
//! Every artifact is built in a uniquely named sibling file, validated, and
//! synced before an atomic no-clobber link exposes the final path. Keeping the
//! staging file beside the destination makes the publication boundary stay on
//! one filesystem.

use std::collections::BTreeMap;
use std::ffi::{OsStr, OsString};
use std::fmt;
use std::fs::{self, File, Metadata};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context};

const STAGE_DOMAIN: &str = ".nuwax-mapping-cli-stage-v1-";
const BUNDLE_STAGE_DOMAIN: &str = ".nuwax-mapping-cli-map-bundle-v1-";
const MAX_STAGE_ATTEMPTS: usize = 32;
const MAX_STAGES_PER_ARTIFACT: usize = 64;
const SQLITE_SIDECAR_SUFFIXES: [&str; 3] = ["-wal", "-shm", "-journal"];

static NEXT_STAGE: AtomicU64 = AtomicU64::new(0);

#[cfg(target_os = "linux")]
struct MapBundleStageReaper {
    child_pid: i32,
    control_fd: i32,
    armed: bool,
}

#[cfg(target_os = "linux")]
impl MapBundleStageReaper {
    fn arm(parent: &Path, staging: &Path) -> io::Result<Self> {
        use std::ffi::CString;
        use std::os::fd::AsRawFd;
        use std::os::raw::{c_char, c_int, c_void};
        use std::os::unix::ffi::OsStrExt;

        const O_CLOEXEC: c_int = 0o2000000;
        const AT_REMOVEDIR: c_int = 0x200;
        extern "C" {
            fn pipe2(pipefd: *mut c_int, flags: c_int) -> c_int;
            fn fork() -> c_int;
            fn setsid() -> c_int;
            fn read(fd: c_int, buffer: *mut c_void, count: usize) -> isize;
            fn write(fd: c_int, buffer: *const c_void, count: usize) -> isize;
            fn close(fd: c_int) -> c_int;
            fn unlinkat(directory: c_int, path: *const c_char, flags: c_int) -> c_int;
            fn fsync(fd: c_int) -> c_int;
            fn _exit(status: c_int) -> !;
        }

        let parent_directory = File::open(parent)?;
        let staging_name_bytes = staging
            .file_name()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "staging has no name"))?
            .as_bytes();
        let staging_name = CString::new(staging_name_bytes)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "staging name contains NUL"))?;
        let member = |name: &[u8]| -> io::Result<CString> {
            let mut relative = staging_name_bytes.to_vec();
            relative.push(b'/');
            relative.extend_from_slice(name);
            CString::new(relative).map_err(|_| {
                io::Error::new(io::ErrorKind::InvalidInput, "staging member contains NUL")
            })
        };
        let members = [
            member(b"manifest.json")?,
            member(b"asset.json")?,
            member(b"map.pgm")?,
        ];
        let mut pipe_fds = [-1, -1];
        let mut ready_fds = [-1, -1];
        if unsafe { pipe2(pipe_fds.as_mut_ptr(), O_CLOEXEC) } != 0 {
            return Err(io::Error::last_os_error());
        }
        if unsafe { pipe2(ready_fds.as_mut_ptr(), O_CLOEXEC) } != 0 {
            let error = io::Error::last_os_error();
            unsafe {
                close(pipe_fds[0]);
                close(pipe_fds[1]);
            }
            return Err(error);
        }
        let child_pid = unsafe { fork() };
        if child_pid < 0 {
            let error = io::Error::last_os_error();
            unsafe {
                close(pipe_fds[0]);
                close(pipe_fds[1]);
                close(ready_fds[0]);
                close(ready_fds[1]);
            }
            return Err(error);
        }
        if child_pid == 0 {
            unsafe {
                close(pipe_fds[1]);
                close(ready_fds[0]);
                setsid();
                let ready = 1_u8;
                write(
                    ready_fds[1],
                    (&ready as *const u8).cast::<c_void>(),
                    1,
                );
                close(ready_fds[1]);
                let mut disarm = 0_u8;
                let received = loop {
                    let result = read(
                        pipe_fds[0],
                        (&mut disarm as *mut u8).cast::<c_void>(),
                        1,
                    );
                    if result >= 0
                        || io::Error::last_os_error().kind() != io::ErrorKind::Interrupted
                    {
                        break result;
                    }
                };
                close(pipe_fds[0]);
                if received == 0 {
                    for member in &members {
                        unlinkat(parent_directory.as_raw_fd(), member.as_ptr(), 0);
                    }
                    unlinkat(
                        parent_directory.as_raw_fd(),
                        staging_name.as_ptr(),
                        AT_REMOVEDIR,
                    );
                    fsync(parent_directory.as_raw_fd());
                }
                _exit(0);
            }
        }
        unsafe {
            close(pipe_fds[0]);
            close(ready_fds[1]);
            let mut ready = 0_u8;
            let received = read(
                ready_fds[0],
                (&mut ready as *mut u8).cast::<c_void>(),
                1,
            );
            close(ready_fds[0]);
            if received != 1 || ready != 1 {
                close(pipe_fds[1]);
                wait_for_reaper(child_pid);
                return Err(io::Error::new(
                    io::ErrorKind::Other,
                    "map bundle staging reaper did not become ready",
                ));
            }
        }
        Ok(Self {
            child_pid,
            control_fd: pipe_fds[1],
            armed: true,
        })
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

#[cfg(target_os = "linux")]
impl Drop for MapBundleStageReaper {
    fn drop(&mut self) {
        use std::os::raw::{c_int, c_void};
        extern "C" {
            fn write(fd: c_int, buffer: *const c_void, count: usize) -> isize;
            fn close(fd: c_int) -> c_int;
        }
        unsafe {
            if !self.armed {
                let disarm = 1_u8;
                write(
                    self.control_fd,
                    (&disarm as *const u8).cast::<c_void>(),
                    1,
                );
            }
            close(self.control_fd);
        }
        wait_for_reaper(self.child_pid);
    }
}

#[cfg(target_os = "linux")]
fn wait_for_reaper(child_pid: i32) {
    use std::os::raw::c_int;
    extern "C" {
        fn waitpid(pid: c_int, status: *mut c_int, options: c_int) -> c_int;
    }
    loop {
        let waited = unsafe { waitpid(child_pid, std::ptr::null_mut(), 0) };
        if waited == child_pid
            || waited < 0 && io::Error::last_os_error().kind() != io::ErrorKind::Interrupted
        {
            break;
        }
    }
}

#[cfg(not(target_os = "linux"))]
struct MapBundleStageReaper;

#[cfg(not(target_os = "linux"))]
impl MapBundleStageReaper {
    fn arm(_parent: &Path, _staging: &Path) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "map bundle forced-termination cleanup requires Linux",
        ))
    }

    fn disarm(&mut self) {}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ArtifactFormat {
    DatasetJsonLines,
    MapPgm,
    ReplayJson,
    MarkerReplayJson,
    SummaryJson,
    Memory2Sqlite,
}

impl ArtifactFormat {
    fn stage_label(self) -> &'static str {
        match self {
            Self::DatasetJsonLines => "dataset-jsonl",
            Self::MapPgm => "map-pgm",
            Self::ReplayJson => "replay-json",
            Self::MarkerReplayJson => "marker-replay-json",
            Self::SummaryJson => "summary-json",
            Self::Memory2Sqlite => "memory2-sqlite",
        }
    }

    fn accepts_extension(self, extension: Option<&str>) -> bool {
        match self {
            Self::DatasetJsonLines => matches!(extension, Some("jsonl" | "ndjson")),
            Self::MapPgm => extension == Some("pgm"),
            Self::ReplayJson | Self::MarkerReplayJson | Self::SummaryJson => {
                extension == Some("json")
            }
            Self::Memory2Sqlite => extension == Some("db"),
        }
    }

    fn expected_extension(self) -> &'static str {
        match self {
            Self::DatasetJsonLines => ".jsonl or .ndjson",
            Self::MapPgm => ".pgm",
            Self::ReplayJson | Self::MarkerReplayJson | Self::SummaryJson => ".json",
            Self::Memory2Sqlite => ".db",
        }
    }

    fn sqlite_sidecars(self) -> &'static [&'static str] {
        match self {
            Self::Memory2Sqlite => &SQLITE_SIDECAR_SUFFIXES,
            Self::DatasetJsonLines
            | Self::MapPgm
            | Self::ReplayJson
            | Self::MarkerReplayJson
            | Self::SummaryJson => &[],
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
    Conflict {
        path: PathBuf,
        cleanup: Option<ArtifactCleanupFailure>,
    },
    DurabilityUnknown {
        path: PathBuf,
        source: io::Error,
    },
    CleanupFailure {
        path: PathBuf,
        cleanup: ArtifactCleanupFailure,
    },
}

impl fmt::Display for MapBundlePublicationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
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
            Self::DurabilityUnknown { path, source } => write!(
                formatter,
                "map bundle {} is visible and validated, but publication durability is unknown: {}",
                path.display(),
                source
            ),
            Self::CleanupFailure { path, cleanup } => write!(
                formatter,
                "map bundle {} was not published, but {cleanup}",
                path.display()
            ),
        }
    }
}

impl std::error::Error for MapBundlePublicationError {
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

pub(crate) fn publish_map_bundle<B, V>(
    final_directory: &Path,
    build: B,
    validate: V,
) -> anyhow::Result<()>
where
    B: FnOnce(&Path) -> anyhow::Result<()>,
    V: FnOnce(&Path) -> anyhow::Result<()>,
{
    let parent = parent_directory(final_directory);
    final_directory.file_name().ok_or_else(|| {
        anyhow::anyhow!(
            "map bundle output has no directory name: {}",
            final_directory.display()
        )
    })?;
    if final_directory.exists() {
        return Err(MapBundlePublicationError::Conflict {
            path: final_directory.to_path_buf(),
            cleanup: None,
        }
        .into());
    }
    fs::create_dir_all(parent)
        .with_context(|| format!("create map bundle parent {}", parent.display()))?;
    let recovery_policy = RecoveryPolicy::acquire()?;
    let final_name = final_directory
        .file_name()
        .ok_or_else(|| anyhow::anyhow!("map bundle output has no directory name"))?;
    recover_stale_map_bundle_stages(parent, final_name, final_directory, &recovery_policy)?;
    let sequence = NEXT_STAGE.fetch_add(1, Ordering::Relaxed);
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;
    let nonce = mix64(sequence ^ timestamp ^ u64::from(std::process::id()));
    let staging = parent.join(format!(
        "{BUNDLE_STAGE_DOMAIN}{}-{:016x}-{nonce:016x}-{}",
        std::process::id(),
        recovery_policy.current_process_start(),
        bundle_name_hash(final_name)
    ));
    fs::create_dir(&staging)
        .with_context(|| format!("reserve map bundle staging {}", staging.display()))?;
    let mut stage_reaper = match MapBundleStageReaper::arm(parent, &staging) {
        Ok(reaper) => reaper,
        Err(error) => {
            let cleanup = match fs::remove_dir(&staging) {
                Ok(()) => sync_directory_io(parent),
                Err(source) if source.kind() == io::ErrorKind::NotFound => {
                    sync_directory_io(parent)
                }
                Err(source) => Err(source),
            };
            if let Err(source) = cleanup {
                return Err(MapBundlePublicationError::CleanupFailure {
                    path: final_directory.to_path_buf(),
                    cleanup: ArtifactCleanupFailure {
                        staging: staging.clone(),
                        source,
                    },
                }
                .into());
            }
            return Err(anyhow::Error::new(error).context(format!(
                "arm map bundle staging reaper {}",
                staging.display()
            )));
        }
    };

    if let Err(error) = build(&staging).and_then(|()| {
        validate(&staging)
            .with_context(|| format!("validate map bundle staging {}", staging.display()))
    }) {
        match cleanup_map_bundle_staging(parent, &staging, final_directory) {
            Ok(()) => stage_reaper.disarm(),
            Err(cleanup_error) => return Err(cleanup_error),
        }
        return Err(error);
    }
    if let Err(error) = sync_directory(&staging) {
        match cleanup_map_bundle_staging(parent, &staging, final_directory) {
            Ok(()) => stage_reaper.disarm(),
            Err(cleanup_error) => return Err(cleanup_error),
        }
        return Err(error);
    }

    match rename_directory_no_replace(&staging, final_directory) {
        Ok(()) => {
            stage_reaper.disarm();
        }
        Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {
            let cleanup = cleanup_map_bundle_staging_io(parent, &staging)
                .err()
                .map(|source| ArtifactCleanupFailure {
                    staging: staging.clone(),
                    source,
                });
            if cleanup.is_none() {
                stage_reaper.disarm();
            }
            return Err(MapBundlePublicationError::Conflict {
                path: final_directory.to_path_buf(),
                cleanup,
            }
            .into());
        }
        Err(error) => {
            match cleanup_map_bundle_staging(parent, &staging, final_directory) {
                Ok(()) => stage_reaper.disarm(),
                Err(cleanup_error) => return Err(cleanup_error),
            }
            return Err(anyhow::Error::new(error).context(format!(
                "atomically publish map bundle {} as {}",
                staging.display(),
                final_directory.display()
            )));
        }
    }

    if let Err(source) = sync_directory_io(parent) {
        return Err(MapBundlePublicationError::DurabilityUnknown {
            path: final_directory.to_path_buf(),
            source,
        }
        .into());
    }
    Ok(())
}

fn bundle_name_hash(final_name: &OsStr) -> String {
    let (high, low) = stable_hash128(&os_str_bytes(final_name));
    format!("{high:016x}{low:016x}")
}

fn parse_map_bundle_stage_name(name: &str, expected_hash: &str) -> Option<StageOwner> {
    let remainder = name.strip_prefix(BUNDLE_STAGE_DOMAIN)?;
    let mut parts = remainder.split('-');
    let process_id_text = parts.next()?;
    let process_start_text = parts.next()?;
    let nonce_text = parts.next()?;
    let final_hash = parts.next()?;
    if parts.next().is_some()
        || final_hash != expected_hash
        || final_hash.len() != 32
        || !final_hash
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return None;
    }
    let process_id: u32 = process_id_text.parse().ok()?;
    let process_start = parse_exact_lower_hex_u64(process_start_text)?;
    parse_exact_lower_hex_u64(nonce_text)?;
    if process_id == 0
        || process_id > i32::MAX as u32
        || process_id.to_string() != process_id_text
        || process_start == 0
    {
        return None;
    }
    Some(StageOwner {
        process_id,
        process_start,
    })
}

fn recover_stale_map_bundle_stages(
    parent: &Path,
    final_name: &OsStr,
    final_directory: &Path,
    recovery_policy: &RecoveryPolicy,
) -> anyhow::Result<()> {
    let expected_hash = bundle_name_hash(final_name);
    let mut stale = Vec::new();
    for entry in fs::read_dir(parent)
        .with_context(|| format!("scan map bundle staging for {}", final_directory.display()))?
    {
        let entry = entry
            .with_context(|| format!("scan map bundle staging for {}", final_directory.display()))?;
        let Some(name) = entry.file_name().to_str().map(str::to_owned) else {
            continue;
        };
        let Some(owner) = parse_map_bundle_stage_name(&name, &expected_hash) else {
            continue;
        };
        stale.push((entry.path(), owner));
        if stale.len() > MAX_STAGES_PER_ARTIFACT {
            bail!(
                "more than {} map bundle staging directories exist for {}; refusing unbounded cleanup",
                MAX_STAGES_PER_ARTIFACT,
                final_directory.display()
            );
        }
    }
    for (staging, owner) in stale {
        if recovery_policy.owner_is_proven_dead(owner) {
            let metadata = fs::symlink_metadata(&staging)?;
            if !metadata.file_type().is_dir() || metadata.file_type().is_symlink() {
                bail!("map bundle staging is not a real directory: {}", staging.display());
            }
            cleanup_map_bundle_staging_io(parent, &staging).with_context(|| {
                format!("recover stale map bundle staging {}", staging.display())
            })?;
        }
    }
    Ok(())
}

fn cleanup_map_bundle_staging(
    parent: &Path,
    staging: &Path,
    final_directory: &Path,
) -> anyhow::Result<()> {
    cleanup_map_bundle_staging_io(parent, staging).map_err(|source| {
        MapBundlePublicationError::CleanupFailure {
            path: final_directory.to_path_buf(),
            cleanup: ArtifactCleanupFailure {
                staging: staging.to_path_buf(),
                source,
            },
        }
        .into()
    })
}

fn cleanup_map_bundle_staging_io(parent: &Path, staging: &Path) -> io::Result<()> {
    fs::remove_dir_all(staging)?;
    sync_directory_io(parent)
}

fn sync_directory(path: &Path) -> anyhow::Result<()> {
    sync_directory_io(path).with_context(|| format!("sync directory {}", path.display()))
}

fn sync_directory_io(path: &Path) -> io::Result<()> {
    File::open(path)?.sync_all()
}

#[cfg(target_os = "linux")]
fn rename_directory_no_replace(source: &Path, destination: &Path) -> io::Result<()> {
    use std::ffi::CString;
    use std::os::raw::{c_char, c_int};
    use std::os::unix::ffi::OsStrExt;

    const AT_FDCWD: c_int = -100;
    const RENAME_NOREPLACE: u32 = 1;
    extern "C" {
        fn renameat2(
            olddirfd: c_int,
            oldpath: *const c_char,
            newdirfd: c_int,
            newpath: *const c_char,
            flags: u32,
        ) -> c_int;
    }
    let source = CString::new(source.as_os_str().as_bytes())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "source path contains NUL"))?;
    let destination = CString::new(destination.as_os_str().as_bytes()).map_err(|_| {
        io::Error::new(io::ErrorKind::InvalidInput, "destination path contains NUL")
    })?;
    // SAFETY: 两个路径均为 NUL 结尾的有效字符串，flags 请求内核禁止覆盖目标。
    let result = unsafe {
        renameat2(
            AT_FDCWD,
            source.as_ptr(),
            AT_FDCWD,
            destination.as_ptr(),
            RENAME_NOREPLACE,
        )
    };
    if result == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}

#[cfg(not(target_os = "linux"))]
fn rename_directory_no_replace(_source: &Path, _destination: &Path) -> io::Result<()> {
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

    let (staging_name, staging_path, reservation, reserved_metadata) = reserve_staging_file(
        &directory,
        &final_name,
        final_path,
        format,
        &recovery_policy,
    )?;
    if let Err(error) = build(&staging_path, reservation) {
        return Err(cleanup_failed_build(
            &directory,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = verify_staging_file(&directory, &staging_name, &reserved_metadata) {
        return Err(cleanup_failed_build(
            &directory,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = reject_sqlite_sidecars(&directory, &staging_name, format) {
        return Err(cleanup_failed_build(
            &directory,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = sync_staging_file(&directory, &staging_name, &staging_path) {
        return Err(cleanup_failed_build(
            &directory,
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
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = verify_staging_file(&directory, &staging_name, &reserved_metadata) {
        return Err(cleanup_failed_build(
            &directory,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }
    if let Err(error) = reject_sqlite_sidecars(&directory, &staging_name, format) {
        return Err(cleanup_failed_build(
            &directory,
            &staging_name,
            &staging_path,
            format,
            error,
        ));
    }

    match directory.link_no_replace(&staging_name, &final_name) {
        Ok(()) => {}
        Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {
            return Err(conflict_after_cleanup(
                final_path,
                &staging_path,
                cleanup_stage(&directory, &staging_name, format),
            ));
        }
        Err(error) => {
            if directory.entry_exists(&final_name)? {
                return Err(conflict_after_cleanup(
                    final_path,
                    &staging_path,
                    cleanup_stage(&directory, &staging_name, format),
                ));
            }
            return Err(cleanup_failed_build(
                &directory,
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

    if let Err(source) = directory.sync() {
        return Err(ArtifactPublicationError::DurabilityUnknown {
            path: final_path.to_path_buf(),
            staging: staging_path,
            source,
        }
        .into());
    }

    if let Err(source) = cleanup_stage(&directory, &staging_name, format) {
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

fn reserve_staging_file(
    directory: &StableDirectory,
    final_name: &OsStr,
    final_path: &Path,
    format: ArtifactFormat,
    recovery_policy: &RecoveryPolicy,
) -> anyhow::Result<(OsString, PathBuf, File, Metadata)> {
    let process_id = std::process::id();
    let process_start = recovery_policy.current_process_start();
    let extension = staging_extension_from_name(final_name);
    let prefix = stage_prefix_from_name(final_name, format);

    for _ in 0..MAX_STAGE_ATTEMPTS {
        let sequence = NEXT_STAGE.fetch_add(1, Ordering::Relaxed);
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let nonce = mix64(sequence ^ timestamp ^ process_start.rotate_left(17));
        let name = format!("{prefix}{process_id}-{process_start:016x}-{nonce:016x}.tmp{extension}");
        let staging_name = OsString::from(name);
        let staging_path = directory.child_path(&staging_name);
        match directory.create_new(&staging_name) {
            Ok(file) => {
                let metadata = file.metadata().with_context(|| {
                    format!(
                        "inspect reserved staging artifact {}",
                        staging_path.display()
                    )
                })?;
                return Ok((staging_name, staging_path, file, metadata));
            }
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(error) => {
                return Err(error).with_context(|| {
                    format!("reserve staging artifact {}", staging_path.display())
                });
            }
        }
    }
    bail!(
        "could not reserve a unique staging file for {} after {} attempts",
        final_path.display(),
        MAX_STAGE_ATTEMPTS
    )
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
    staging_name: &OsStr,
    staging_path: &Path,
    format: ArtifactFormat,
    error: anyhow::Error,
) -> anyhow::Error {
    match cleanup_stage(directory, staging_name, format) {
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
    let prefix = stage_prefix_from_name(final_name, format);
    let extension = staging_extension_from_name(final_name);
    let mut stages = BTreeMap::<OsString, StageOwner>::new();
    for entry in directory
        .read_dir()
        .with_context(|| format!("scan stable output directory for {}", final_path.display()))?
    {
        let entry = entry.with_context(|| {
            format!("scan stable output directory for {}", final_path.display())
        })?;
        let Some(name) = entry.file_name().to_str().map(str::to_owned) else {
            continue;
        };
        let Some((base_name, owner)) = parse_stage_name(&name, &prefix, &extension, format) else {
            continue;
        };
        stages.entry(OsString::from(base_name)).or_insert(owner);
        if stages.len() > MAX_STAGES_PER_ARTIFACT {
            bail!(
                "more than {} staging artifacts exist for {}; refusing unbounded cleanup",
                MAX_STAGES_PER_ARTIFACT,
                final_path.display()
            );
        }
    }

    for (staging_name, owner) in stages {
        if owner_is_proven_dead(owner) {
            let staging_path = directory.child_path(&staging_name);
            cleanup_stage(directory, &staging_name, format).with_context(|| {
                format!("recover stale staging artifact {}", staging_path.display())
            })?;
        }
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct StageOwner {
    process_id: u32,
    process_start: u64,
}

fn parse_stage_name(
    name: &str,
    prefix: &str,
    expected_extension: &str,
    format: ArtifactFormat,
) -> Option<(String, StageOwner)> {
    let (base_name, sidecar_suffix) = format
        .sqlite_sidecars()
        .iter()
        .find_map(|suffix| name.strip_suffix(suffix).map(|base| (base, Some(*suffix))))
        .unwrap_or((name, None));
    let remainder = base_name.strip_prefix(prefix)?;
    let expected_tail = format!(".tmp{expected_extension}");
    let owner_text = remainder.strip_suffix(&expected_tail)?;
    let mut parts = owner_text.split('-');
    let process_id_text = parts.next()?;
    let process_start_text = parts.next()?;
    let nonce_text = parts.next()?;
    if parts.next().is_some() {
        return None;
    }
    let process_id: u32 = process_id_text.parse().ok()?;
    let process_start = parse_exact_lower_hex_u64(process_start_text)?;
    let nonce = parse_exact_lower_hex_u64(nonce_text)?;
    if process_id == 0
        || process_id > i32::MAX as u32
        || process_id.to_string() != process_id_text
        || process_start == 0
    {
        return None;
    }
    let reconstructed =
        format!("{prefix}{process_id}-{process_start:016x}-{nonce:016x}{expected_tail}");
    if reconstructed != base_name {
        return None;
    }
    if let Some(suffix) = sidecar_suffix {
        if format != ArtifactFormat::Memory2Sqlite || name != format!("{base_name}{suffix}") {
            return None;
        }
    }
    Some((
        base_name.to_string(),
        StageOwner {
            process_id,
            process_start,
        },
    ))
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
struct StableDirectory {
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

    fn descriptor_path(&self) -> PathBuf {
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

    fn link_no_replace(&self, staging_name: &OsStr, final_name: &OsStr) -> io::Result<()> {
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
        // to the same live directory descriptor. flags=0 never follows a
        // symbolic-link staging entry.
        let result = unsafe {
            linkat(
                self.file.as_raw_fd(),
                staging.as_ptr(),
                self.file.as_raw_fd(),
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

    fn entry_exists(&self, _name: &OsStr) -> io::Result<bool> {
        Err(Self::unsupported())
    }

    fn link_no_replace(&self, _staging_name: &OsStr, _final_name: &OsStr) -> io::Result<()> {
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

fn stage_prefix_from_name(final_name: &OsStr, format: ArtifactFormat) -> String {
    let bytes = os_str_bytes(final_name);
    let (first, second) = stable_hash128(&bytes);
    format!(
        "{STAGE_DOMAIN}{first:016x}{second:016x}-{}-",
        format.stage_label()
    )
}

#[cfg(unix)]
fn os_str_bytes(value: &OsStr) -> Vec<u8> {
    use std::os::unix::ffi::OsStrExt;
    value.as_bytes().to_vec()
}

#[cfg(not(unix))]
fn os_str_bytes(value: &OsStr) -> Vec<u8> {
    value.to_string_lossy().as_bytes().to_vec()
}

fn stable_hash128(bytes: &[u8]) -> (u64, u64) {
    let mut first = 0xcbf2_9ce4_8422_2325_u64;
    let mut second = 0x8422_2325_cbf2_9ce4_u64;
    for (index, byte) in bytes.iter().copied().enumerate() {
        first ^= u64::from(byte);
        first = first.wrapping_mul(0x0000_0100_0000_01b3);
        second ^= u64::from(byte).wrapping_add((index as u64).rotate_left(23));
        second = second.rotate_left(9).wrapping_mul(0x9e37_79b1_85eb_ca87);
    }
    (first, second)
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

fn mix64(mut value: u64) -> u64 {
    value ^= value >> 30;
    value = value.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value ^= value >> 27;
    value = value.wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use std::cell::{Cell, RefCell};
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

    fn dead_stage(path: &Path, format: ArtifactFormat) -> PathBuf {
        let extension = staging_extension(path);
        parent_directory(path).join(format!(
            "{}{}-{:016x}-{:016x}.tmp{extension}",
            stage_prefix(path, format),
            absent_process_id(),
            1_u64,
            2_u64
        ))
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
            (ArtifactFormat::ReplayJson, "replay-json", "replay.json"),
            (
                ArtifactFormat::MarkerReplayJson,
                "marker-replay-json",
                "markers.json",
            ),
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
            assert!(observed_name
                .into_inner()
                .unwrap()
                .to_string_lossy()
                .starts_with(&expected_prefix));

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
        let stale = dead_stage(&output, ArtifactFormat::Memory2Sqlite);
        let hostile_extra = PathBuf::from(format!("{}-wal-extra", stale.display()));
        fs::write(&stale, b"partial database").unwrap();
        fs::write(append_suffix(&stale, "-wal"), b"partial wal").unwrap();
        fs::write(&hostile_extra, b"user file").unwrap();

        publish_sqlite_text(&output, "validated winner").unwrap();
        assert_eq!(fs::read_to_string(&output).unwrap(), "validated winner");
        assert!(!stale.exists());
        assert!(!append_suffix(&stale, "-wal").exists());
        assert_eq!(fs::read(hostile_extra).unwrap(), b"user file");
    }

    #[test]
    fn recovery_never_removes_an_existing_winner() {
        let directory = TestDir::new("preserve-winner");
        let output = directory.0.join("artifact.json");
        fs::write(&output, b"winner").unwrap();
        let stale = dead_stage(&output, ArtifactFormat::SummaryJson);
        fs::write(&stale, b"loser").unwrap();

        let error = publish_text(&output, "replacement").unwrap_err();
        assert!(matches!(
            error.downcast_ref::<ArtifactPublicationError>(),
            Some(ArtifactPublicationError::Conflict { .. })
        ));
        assert_eq!(fs::read(&output).unwrap(), b"winner");
        assert!(stage_entries(&output).is_empty());
    }

    #[test]
    fn active_staging_is_not_reaped_without_a_winner() {
        let directory = TestDir::new("active");
        let output = directory.0.join("artifact.json");
        let process_id = std::process::id();
        let policy = RecoveryPolicy::acquire().unwrap();
        let process_start = policy.current_process_start();
        let active = directory.0.join(format!(
            "{}{process_id}-{process_start:016x}-{:016x}.tmp.json",
            stage_prefix(&output, ArtifactFormat::SummaryJson),
            7_u64
        ));
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
        assert!(active.exists());
        fs::remove_file(active).unwrap();
    }

    #[test]
    fn recovery_accepts_only_exact_emitted_names() {
        let directory = TestDir::new("strict-parser");
        let output = directory.0.join("artifact.json");
        let exact = dead_stage(&output, ArtifactFormat::SummaryJson);
        let exact_name = exact.file_name().unwrap().to_string_lossy();
        let other_label_name = dead_stage(&output, ArtifactFormat::ReplayJson)
            .file_name()
            .unwrap()
            .to_string_lossy()
            .into_owned();
        let absent_process = absent_process_id();
        fs::write(&exact, b"exact stale staging").unwrap();
        let hostile_names = [
            format!("{exact_name}.user-tail"),
            format!("{exact_name}-wal"),
            format!("{exact_name}-wal-extra"),
            exact_name.replacen("-0000000000000001-", "-0000000000000001A-", 1),
            format!(
                "{}{absent_process}-000000000000000A-0000000000000002.tmp.json",
                stage_prefix(&output, ArtifactFormat::SummaryJson)
            ),
            exact_name.replacen(
                &format!("{absent_process}-"),
                &format!("0{absent_process}-"),
                1,
            ),
            other_label_name,
        ];
        for name in &hostile_names {
            fs::write(directory.0.join(name), b"user file").unwrap();
        }

        publish_text(&output, "winner").unwrap();

        assert!(!exact.exists());
        assert_eq!(fs::read_to_string(&output).unwrap(), "winner");
        for name in hostile_names {
            assert_eq!(fs::read(directory.0.join(name)).unwrap(), b"user file");
        }
    }

    #[test]
    fn unverifiable_process_identity_never_reaps_staging() {
        let directory = TestDir::new("unverifiable-owner");
        let output = directory.0.join("artifact.json");
        let staging = dead_stage(&output, ArtifactFormat::SummaryJson);
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
    fn stable_directory_operations_ignore_parent_path_retarget() {
        let directory = TestDir::new("parent-retarget");
        let parent = directory.0.join("output");
        let moved = directory.0.join("opened-output");
        fs::create_dir(&parent).unwrap();
        let stable = StableDirectory::open(&parent).unwrap();
        let staging_name = OsStr::new("staging.tmp.json");
        let final_name = OsStr::new("winner.json");
        let mut staging = stable.create_new(staging_name).unwrap();
        staging.write_all(b"winner").unwrap();
        staging.sync_all().unwrap();
        drop(staging);

        fs::rename(&parent, &moved).unwrap();
        fs::create_dir(&parent).unwrap();
        stable.link_no_replace(staging_name, final_name).unwrap();
        stable.sync().unwrap();
        cleanup_stage(&stable, staging_name, ArtifactFormat::SummaryJson).unwrap();

        assert_eq!(fs::read(moved.join(final_name)).unwrap(), b"winner");
        assert!(fs::read_dir(parent).unwrap().next().is_none());
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

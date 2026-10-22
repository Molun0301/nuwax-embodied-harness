#![allow(clippy::uninlined_format_args)]

use memmap2::MmapMut;
use nix::fcntl::{Flock, FlockArg};
use std::ffi::{CString, OsStr, OsString};
use std::fs::File;
use std::io::{self, Read};
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::ffi::OsStrExt;
use std::os::unix::fs::{FileTypeExt, MetadataExt, PermissionsExt};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{fence, AtomicBool, AtomicU64, Ordering};
#[cfg(test)]
use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tokio::sync::Notify;
use tracing::error;

const SHM_MAGIC: u64 = 0x4E55_5741_5856_4944;
const SHM_VERSION: u32 = 1;
const HEADER_SIZE: usize = 64;
const MAX_FRAME_SIZE: usize = 512 * 1024;
const BUFFER_SIZE: usize = HEADER_SIZE + MAX_FRAME_SIZE * 2;
const ACCEPT_POLL_INTERVAL: Duration = Duration::from_millis(20);
const CONNECTION_READ_TIMEOUT: Duration = Duration::from_millis(50);
const CONTROL_PAGE_SIZE: usize = 4096;
const CONTROL_MAGIC: u64 = 0x4e55_5741_5856_4354;
const CONTROL_VERSION: u32 = 1;
const IPC_MAGIC: u64 = 0x4e55_5741_5856_4950;
const IPC_VERSION: u32 = 1;
const IPC_RECORD_SIZE: usize = 9216;
const IPC_NAME_CAPACITY: usize = 256;
const IPC_PATH_CAPACITY: usize = 4096;
const IPC_SHM_NAME_OFFSET: usize = 128;
const IPC_UDS_NAME_OFFSET: usize = IPC_SHM_NAME_OFFSET + IPC_NAME_CAPACITY;
const IPC_SHM_PARENT_OFFSET: usize = IPC_UDS_NAME_OFFSET + IPC_NAME_CAPACITY;
const IPC_UDS_PARENT_OFFSET: usize = IPC_SHM_PARENT_OFFSET + IPC_PATH_CAPACITY;
const HELPER_REQUEST_ENV: &str = "NUWAX_VIDEO_RELAY_HELPER_REQUEST_V1";
const HELPER_IPC_FD_ENV: &str = "NUWAX_VIDEO_RELAY_HELPER_IPC_FD_V1";
const HELPER_CONTROL_FD_ENV: &str = "NUWAX_VIDEO_RELAY_HELPER_CONTROL_FD_V1";
const HELPER_SHM_DIR_FD_ENV: &str = "NUWAX_VIDEO_RELAY_HELPER_SHM_DIR_FD_V1";
const HELPER_UDS_DIR_FD_ENV: &str = "NUWAX_VIDEO_RELAY_HELPER_UDS_DIR_FD_V1";
const HELPER_SHM_LEASE_FD_ENV: &str = "NUWAX_VIDEO_RELAY_HELPER_SHM_LEASE_FD_V1";
const HELPER_UDS_LEASE_FD_ENV: &str = "NUWAX_VIDEO_RELAY_HELPER_UDS_LEASE_FD_V1";
const HELPER_PARENT_PID_ENV: &str = "NUWAX_VIDEO_RELAY_HELPER_PARENT_PID_V1";
#[cfg(not(test))]
const STARTUP_READY_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const STARTUP_READY_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(not(test))]
const DEFAULT_STOP_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const DEFAULT_STOP_TIMEOUT: Duration = Duration::from_secs(1);
#[cfg(not(test))]
const DROP_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(test)]
const DROP_TIMEOUT: Duration = Duration::from_millis(500);
const SUPERVISOR_POLL_INTERVAL: Duration = Duration::from_millis(5);
const TERM_GRACE_LIMIT: Duration = Duration::from_millis(100);
const CLEANUP_RENAME_ATTEMPTS: usize = 32;

static CLEANUP_NAME_SEQUENCE: AtomicU64 = AtomicU64::new(0);


#[derive(Clone)]
#[cfg(test)]
pub struct VideoRelayHandle {
    pub sequence: Arc<AtomicU64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VideoRelayFailure {
    pub operation: &'static str,
    pub kind: io::ErrorKind,
    pub detail: String,
}

impl VideoRelayFailure {
    fn from_error(operation: &'static str, error: &io::Error) -> Self {
        Self {
            operation,
            kind: error.kind(),
            detail: error.to_string(),
        }
    }

    fn new(operation: &'static str, kind: io::ErrorKind, detail: impl Into<String>) -> Self {
        Self {
            operation,
            kind,
            detail: detail.into(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VideoRelayTerminalReason {
    StopRequested,
    Failed(VideoRelayFailure),
    ThreadPanicked(String),
    KillUnreaped(VideoRelayFailure),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VideoRelayMmapCleanup {
    NotCreated,
    FlushedAndReleased,
    ReleasedWithoutFlushAfterFailure,
    FlushFailed(String),
    KillUnreaped,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VideoRelayPathCleanup {
    NotCreated,
    Removed,
    AlreadyAbsent,
    ReplacementPreserved,
    RemovedButDirectorySyncFailed(String),
    Residual { path: PathBuf, detail: String },
}

impl VideoRelayPathCleanup {
    fn is_complete(&self) -> bool {
        matches!(
            self,
            Self::NotCreated | Self::Removed | Self::AlreadyAbsent | Self::ReplacementPreserved
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VideoRelayCleanup {
    pub mmap: VideoRelayMmapCleanup,
    pub uds: VideoRelayPathCleanup,
    pub shm: VideoRelayPathCleanup,
    pub uds_lease: VideoRelayPathCleanup,
    pub shm_lease: VideoRelayPathCleanup,
}

impl VideoRelayCleanup {
    fn empty() -> Self {
        Self {
            mmap: VideoRelayMmapCleanup::NotCreated,
            uds: VideoRelayPathCleanup::NotCreated,
            shm: VideoRelayPathCleanup::NotCreated,
            uds_lease: VideoRelayPathCleanup::NotCreated,
            shm_lease: VideoRelayPathCleanup::NotCreated,
        }
    }

    pub fn is_complete(&self) -> bool {
        matches!(
            self.mmap,
            VideoRelayMmapCleanup::NotCreated
                | VideoRelayMmapCleanup::FlushedAndReleased
                | VideoRelayMmapCleanup::ReleasedWithoutFlushAfterFailure
        ) && self.uds.is_complete()
            && self.shm.is_complete()
            && self.uds_lease.is_complete()
            && self.shm_lease.is_complete()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VideoRelayTerminal {
    pub reason: VideoRelayTerminalReason,
    pub cleanup: VideoRelayCleanup,
}

impl VideoRelayTerminal {
    pub fn is_expected_shutdown(&self) -> bool {
        matches!(self.reason, VideoRelayTerminalReason::StopRequested) && self.cleanup.is_complete()
    }

    pub fn is_clean(&self) -> bool {
        self.cleanup.is_complete()
    }
}

#[derive(Debug)]
pub struct VideoRelayStartError {
    pub primary: VideoRelayFailure,
    pub terminal: Box<VideoRelayTerminal>,
    pub thread_started: bool,
    pub thread_joined: bool,
}

impl std::fmt::Display for VideoRelayStartError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            formatter,
            "{}: {}; thread_started={}; thread_joined={}; cleanup_complete={}; cleanup={:?}",
            self.primary.operation,
            self.primary.detail,
            self.thread_started,
            self.thread_joined,
            self.terminal.is_clean(),
            self.terminal.cleanup
        )
    }
}

impl std::error::Error for VideoRelayStartError {}

fn start_error_without_thread(
    primary: VideoRelayFailure,
    cleanup: VideoRelayCleanup,
) -> VideoRelayStartError {
    VideoRelayStartError {
        terminal: Box::new(VideoRelayTerminal {
            reason: VideoRelayTerminalReason::Failed(primary.clone()),
            cleanup,
        }),
        primary,
        thread_started: false,
        thread_joined: true,
    }
}

fn lock_unpoisoned<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

#[derive(Clone)]
struct StableDirectory {
    file: Arc<File>,
    display_path: PathBuf,
    device: u64,
    inode: u64,
}

impl StableDirectory {
    fn open(path: &Path) -> io::Result<Self> {
        let file = open_stable_directory(path)?;
        let metadata = file.metadata()?;
        if !metadata.is_dir() {
            return Err(io::Error::new(
                io::ErrorKind::NotADirectory,
                format!("视频路径父级不是目录: {}", path.display()),
            ));
        }
        Ok(Self {
            file: Arc::new(file),
            display_path: path.to_path_buf(),
            device: metadata.dev(),
            inode: metadata.ino(),
        })
    }

    fn from_inherited(
        file: File,
        display_path: PathBuf,
        expected_device: u64,
        expected_inode: u64,
    ) -> io::Result<Self> {
        let metadata = file.metadata()?;
        if !metadata.is_dir()
            || metadata.dev() != expected_device
            || metadata.ino() != expected_inode
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("继承的视频目录身份不匹配: {}", display_path.display()),
            ));
        }
        Ok(Self {
            file: Arc::new(file),
            display_path,
            device: expected_device,
            inode: expected_inode,
        })
    }

    fn revalidate_public_path(&self) -> io::Result<()> {
        let current = Self::open(&self.display_path)?;
        if current.device != self.device || current.inode != self.inode {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!(
                    "视频公共父路径已不再指向固定目录: {}",
                    self.display_path.display()
                ),
            ));
        }
        Ok(())
    }

    fn descriptor_path(&self) -> PathBuf {
        Path::new("/proc/self/fd").join(self.file.as_raw_fd().to_string())
    }

    fn child_path(&self, name: &OsStr) -> PathBuf {
        self.descriptor_path().join(name)
    }

    fn display_child(&self, name: &OsStr) -> PathBuf {
        self.display_path.join(name)
    }

    fn create_new(&self, name: &OsStr) -> io::Result<File> {
        self.open_at(
            name,
            nix::libc::O_RDWR
                | nix::libc::O_CREAT
                | nix::libc::O_EXCL
                | nix::libc::O_CLOEXEC
                | nix::libc::O_NOFOLLOW,
            Some(0o600),
        )
    }

    fn open_lock(&self, name: &OsStr) -> io::Result<File> {
        self.open_at(
            name,
            nix::libc::O_RDWR | nix::libc::O_CREAT | nix::libc::O_CLOEXEC | nix::libc::O_NOFOLLOW,
            Some(0o600),
        )
    }

    fn open_at(&self, name: &OsStr, flags: i32, mode: Option<u32>) -> io::Result<File> {
        let name = c_string(name)?;
        let descriptor = unsafe {
            match mode {
                Some(mode) => nix::libc::openat(
                    self.file.as_raw_fd(),
                    name.as_ptr(),
                    flags,
                    mode as nix::libc::mode_t,
                ),
                None => nix::libc::openat(self.file.as_raw_fd(), name.as_ptr(), flags),
            }
        };
        if descriptor < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(unsafe { File::from_raw_fd(descriptor) })
        }
    }

    fn metadata(&self, name: &OsStr) -> io::Result<Option<std::fs::Metadata>> {
        match self.open_at(
            name,
            nix::libc::O_PATH | nix::libc::O_NOFOLLOW | nix::libc::O_CLOEXEC,
            None,
        ) {
            Ok(file) => Ok(Some(file.metadata()?)),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(None),
            Err(error) => Err(error),
        }
    }

    fn rename_noreplace(&self, source: &OsStr, target: &OsStr) -> io::Result<()> {
        let source = c_string(source)?;
        let target = c_string(target)?;
        let result = unsafe {
            nix::libc::renameat2(
                self.file.as_raw_fd(),
                source.as_ptr(),
                self.file.as_raw_fd(),
                target.as_ptr(),
                nix::libc::RENAME_NOREPLACE,
            )
        };
        if result == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    fn link_identity_noreplace(&self, identity: &File, target: &OsStr) -> io::Result<()> {
        let empty = CString::new("").unwrap();
        let target = c_string(target)?;
        let result = unsafe {
            nix::libc::linkat(
                identity.as_raw_fd(),
                empty.as_ptr(),
                self.file.as_raw_fd(),
                target.as_ptr(),
                nix::libc::AT_EMPTY_PATH,
            )
        };
        if result == 0 {
            return Ok(());
        }

        let error = io::Error::last_os_error();
        if !matches!(
            error.raw_os_error(),
            Some(nix::libc::ENOENT | nix::libc::EPERM)
        ) {
            return Err(error);
        }

        // Jetson 5.10 的无特权进程不能用 AT_EMPTY_PATH 链接已打开的
        // socket inode；procfd 仍指向同一身份，且目标端保持不覆盖语义。
        let procfd = CString::new(format!("/proc/self/fd/{}", identity.as_raw_fd())).unwrap();
        let fallback = unsafe {
            nix::libc::linkat(
                nix::libc::AT_FDCWD,
                procfd.as_ptr(),
                self.file.as_raw_fd(),
                target.as_ptr(),
                nix::libc::AT_SYMLINK_FOLLOW,
            )
        };
        if fallback == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    fn unlink(&self, name: &OsStr) -> io::Result<()> {
        let name = c_string(name)?;
        let result = unsafe { nix::libc::unlinkat(self.file.as_raw_fd(), name.as_ptr(), 0) };
        if result == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    fn sync(&self) -> io::Result<()> {
        self.file.sync_all()
    }
}

#[cfg(target_os = "linux")]
fn open_stable_directory(path: &Path) -> io::Result<File> {
    const AT_FDCWD: i32 = -100;
    const RESOLVE_NO_MAGICLINKS: u64 = 0x02;
    const RESOLVE_NO_SYMLINKS: u64 = 0x04;
    const SYS_OPENAT2: nix::libc::c_long = 437;

    #[repr(C)]
    struct OpenHow {
        flags: u64,
        mode: u64,
        resolve: u64,
    }

    let path = CString::new(path.as_os_str().as_bytes())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "视频父目录路径包含 NUL"))?;
    let how = OpenHow {
        flags: (nix::libc::O_CLOEXEC | nix::libc::O_DIRECTORY) as u64,
        mode: 0,
        resolve: RESOLVE_NO_MAGICLINKS | RESOLVE_NO_SYMLINKS,
    };
    let descriptor = unsafe {
        nix::libc::syscall(
            SYS_OPENAT2,
            nix::libc::c_long::from(AT_FDCWD),
            path.as_ptr(),
            &how as *const OpenHow,
            std::mem::size_of::<OpenHow>(),
        )
    };
    if descriptor < 0 {
        Err(io::Error::last_os_error())
    } else {
        let descriptor = i32::try_from(descriptor)
            .map_err(|_| io::Error::other("稳定目录描述符超出 i32 范围"))?;
        Ok(unsafe { File::from_raw_fd(descriptor) })
    }
}

#[cfg(not(target_os = "linux"))]
fn open_stable_directory(_path: &Path) -> io::Result<File> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "视频中转需要 Linux openat2 稳定目录语义",
    ))
}

fn c_string(name: &OsStr) -> io::Result<CString> {
    CString::new(name.as_bytes())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "视频路径名称包含 NUL"))
}

#[derive(Clone)]
struct PathAnchor {
    directory: StableDirectory,
    name: OsString,
    public_path: PathBuf,
}

impl PathAnchor {
    fn open(path: &Path) -> io::Result<Self> {
        let name = path.file_name().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("视频路径缺少文件名: {}", path.display()),
            )
        })?;
        if name == OsStr::new(".") || name == OsStr::new("..") {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "视频路径文件名不能是 . 或 ..",
            ));
        }
        let parent = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
            .unwrap_or_else(|| Path::new("."));
        Ok(Self {
            directory: StableDirectory::open(parent)?,
            name: name.to_os_string(),
            public_path: path.to_path_buf(),
        })
    }

    fn stable_key(&self) -> (u64, u64, Vec<u8>) {
        (
            self.directory.device,
            self.directory.inode,
            self.name.as_bytes().to_vec(),
        )
    }

    fn same_location(&self, other: &Self) -> bool {
        self.stable_key() == other.stable_key()
    }

    fn metadata(&self) -> io::Result<Option<std::fs::Metadata>> {
        self.directory.metadata(&self.name)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EntryRole {
    Shm,
    Uds,
    Lease,
}

struct UnpublishedEntry {
    directory: StableDirectory,
    name: OsString,
    public_path: PathBuf,
    identity_file: Option<File>,
    armed: bool,
}

impl UnpublishedEntry {
    fn new(
        directory: StableDirectory,
        name: OsString,
        public_path: PathBuf,
        identity_file: File,
    ) -> Self {
        Self {
            directory,
            name,
            public_path,
            identity_file: Some(identity_file),
            armed: true,
        }
    }

    fn file(&self) -> &File {
        self.identity_file.as_ref().unwrap()
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for UnpublishedEntry {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }
        let Some(identity_file) = self.identity_file.take() else {
            return;
        };
        let Ok(identity) = identity_file.metadata() else {
            return;
        };
        let candidate_anchor = PathAnchor {
            directory: self.directory.clone(),
            name: self.name.clone(),
            public_path: self.public_path.clone(),
        };
        let cleanup = OwnedEntry {
            anchor: candidate_anchor,
            name: self.name.clone(),
            identity_file,
            device: identity.dev(),
            inode: identity.ino(),
            role: EntryRole::Shm,
        }
        .cleanup();
        if !cleanup.is_complete() {
            error!(?cleanup, "清理未发布视频节点失败");
        }
    }
}

struct OwnedEntry {
    anchor: PathAnchor,
    name: OsString,
    identity_file: File,
    device: u64,
    inode: u64,
    role: EntryRole,
}

impl OwnedEntry {
    fn from_metadata(
        anchor: PathAnchor,
        name: OsString,
        metadata: &std::fs::Metadata,
        identity_file: File,
        role: EntryRole,
    ) -> io::Result<Self> {
        let identity = identity_file.metadata()?;
        if identity.dev() != metadata.dev() || identity.ino() != metadata.ino() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                "视频节点在取得身份描述符前已被替换",
            ));
        }
        Ok(Self {
            anchor,
            name,
            identity_file,
            device: metadata.dev(),
            inode: metadata.ino(),
            role,
        })
    }

    fn identity_matches(&self, metadata: &std::fs::Metadata) -> bool {
        self.identity_file.metadata().is_ok_and(|identity| {
            identity.dev() == self.device
                && identity.ino() == self.inode
                && metadata.dev() == identity.dev()
                && metadata.ino() == identity.ino()
        })
    }

    fn reopen_identity(&self) -> io::Result<File> {
        let reopened = self.anchor.directory.open_at(
            &self.name,
            nix::libc::O_PATH | nix::libc::O_NOFOLLOW | nix::libc::O_CLOEXEC,
            None,
        )?;
        let metadata = reopened.metadata()?;
        if !self.identity_matches(&metadata) {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!(
                    "视频节点身份已变化: {}",
                    self.anchor.directory.display_child(&self.name).display()
                ),
            ));
        }
        Ok(reopened)
    }

    fn open_expected(
        anchor: PathAnchor,
        device: u64,
        inode: u64,
        role: EntryRole,
    ) -> io::Result<Self> {
        let identity_file = anchor.directory.open_at(
            &anchor.name,
            nix::libc::O_PATH | nix::libc::O_NOFOLLOW | nix::libc::O_CLOEXEC,
            None,
        )?;
        let metadata = identity_file.metadata()?;
        if metadata.dev() != device || metadata.ino() != inode {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("视频公共节点身份不匹配: {}", anchor.public_path.display()),
            ));
        }
        Self::from_metadata(
            anchor.clone(),
            anchor.name.clone(),
            &metadata,
            identity_file,
            role,
        )
    }

    fn identity(&self) -> (u64, u64) {
        (self.device, self.inode)
    }

    fn try_clone_exact(&self) -> io::Result<Self> {
        Ok(Self {
            anchor: self.anchor.clone(),
            name: self.name.clone(),
            identity_file: self.identity_file.try_clone()?,
            device: self.device,
            inode: self.inode,
            role: self.role,
        })
    }

    fn retarget(self, anchor: PathAnchor, name: OsString) -> Self {
        Self {
            anchor,
            name,
            identity_file: self.identity_file,
            device: self.device,
            inode: self.inode,
            role: self.role,
        }
    }

    fn cleanup(self) -> VideoRelayPathCleanup {
        let initial = match self.anchor.directory.metadata(&self.name) {
            Ok(Some(metadata)) => metadata,
            Ok(None) => return VideoRelayPathCleanup::AlreadyAbsent,
            Err(error) => {
                return VideoRelayPathCleanup::Residual {
                    path: self.anchor.directory.display_child(&self.name),
                    detail: format!("读取待清理节点失败: {error}"),
                };
            }
        };
        if !self.identity_matches(&initial) {
            return VideoRelayPathCleanup::ReplacementPreserved;
        }

        let quarantine = match reserve_cleanup_name(&self.anchor.directory, self.role) {
            Ok(name) => name,
            Err(error) => {
                return VideoRelayPathCleanup::Residual {
                    path: self.anchor.directory.display_child(&self.name),
                    detail: format!("分配隔离名称失败: {error}"),
                };
            }
        };
        match self
            .anchor
            .directory
            .rename_noreplace(&self.name, &quarantine)
        {
            Ok(()) => {}
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                return VideoRelayPathCleanup::AlreadyAbsent;
            }
            Err(error) => {
                return VideoRelayPathCleanup::Residual {
                    path: self.anchor.directory.display_child(&self.name),
                    detail: format!("隔离待清理节点失败: {error}"),
                };
            }
        }

        let quarantined_metadata = match self.anchor.directory.metadata(&quarantine) {
            Ok(Some(metadata)) => metadata,
            Ok(None) => return VideoRelayPathCleanup::AlreadyAbsent,
            Err(error) => {
                return VideoRelayPathCleanup::Residual {
                    path: self.anchor.directory.display_child(&quarantine),
                    detail: format!("读取隔离节点失败: {error}"),
                };
            }
        };
        if !self.identity_matches(&quarantined_metadata) {
            return match self
                .anchor
                .directory
                .rename_noreplace(&quarantine, &self.name)
            {
                Ok(()) => VideoRelayPathCleanup::ReplacementPreserved,
                Err(error) => VideoRelayPathCleanup::Residual {
                    path: self.anchor.directory.display_child(&quarantine),
                    detail: format!("隔离节点身份变化且恢复原名失败: {error}"),
                },
            };
        }

        #[cfg(test)]
        if self.role == EntryRole::Shm
            && take_test_fault(&self.anchor.public_path, TestFaultPoint::ShmCleanup)
        {
            return VideoRelayPathCleanup::Residual {
                path: self.anchor.directory.display_child(&quarantine),
                detail: "注入共享内存清理失败".to_string(),
            };
        }

        match self.anchor.directory.unlink(&quarantine) {
            Ok(()) => match self.anchor.directory.sync() {
                Ok(()) => VideoRelayPathCleanup::Removed,
                Err(error) => {
                    VideoRelayPathCleanup::RemovedButDirectorySyncFailed(error.to_string())
                }
            },
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                VideoRelayPathCleanup::AlreadyAbsent
            }
            Err(error) => VideoRelayPathCleanup::Residual {
                path: self.anchor.directory.display_child(&quarantine),
                detail: format!("删除隔离节点失败: {error}"),
            },
        }
    }
}

fn reserve_cleanup_name(directory: &StableDirectory, role: EntryRole) -> io::Result<OsString> {
    for _ in 0..CLEANUP_RENAME_ATTEMPTS {
        let sequence = CLEANUP_NAME_SEQUENCE.fetch_add(1, Ordering::Relaxed);
        let name = OsString::from(format!(
            ".nuwax-video-cleanup-{}-{}-{sequence}",
            std::process::id(),
            match role {
                EntryRole::Shm => "shm",
                EntryRole::Uds => "uds",
                EntryRole::Lease => "lease",
            }
        ));
        if directory.metadata(&name)?.is_none() {
            return Ok(name);
        }
    }
    Err(io::Error::new(
        io::ErrorKind::AlreadyExists,
        "无法分配唯一视频清理隔离名称",
    ))
}

struct ResourceLease {
    lock: Option<Flock<File>>,
    entry: Option<OwnedEntry>,
}

impl ResourceLease {
    fn acquire(anchor: &PathAnchor) -> io::Result<Self> {
        let name = lease_name(&anchor.name);
        let file = anchor.directory.open_lock(&name)?;
        let metadata = file.metadata()?;
        if !metadata.is_file() || metadata.nlink() != 1 || metadata.mode() & 0o777 != 0o600 {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                format!(
                    "视频路径租约必须是 0600 单链接普通文件: {}",
                    anchor.directory.display_child(&name).display()
                ),
            ));
        }
        let entry = OwnedEntry::from_metadata(
            anchor.clone(),
            name,
            &metadata,
            file.try_clone()?,
            EntryRole::Lease,
        )?;
        let lock = Flock::lock(file, FlockArg::LockExclusiveNonblock).map_err(|(_, error)| {
            io::Error::new(
                io::ErrorKind::AddrInUse,
                format!("视频路径租约已被持有: {error}"),
            )
        })?;
        Ok(Self {
            lock: Some(lock),
            entry: Some(entry),
        })
    }

    fn reopen_identity(&self) -> io::Result<File> {
        self.entry
            .as_ref()
            .ok_or_else(|| io::Error::other("视频路径租约身份已被释放"))?
            .reopen_identity()
    }

    fn cleanup(mut self) -> VideoRelayPathCleanup {
        // 正常 owner 仍持有 flock，先按 inode 身份删除租约路径，再释放锁不会与接管者并发。
        let result = self
            .entry
            .take()
            .map_or(VideoRelayPathCleanup::NotCreated, OwnedEntry::cleanup);
        self.lock.take();
        result
    }
}

impl Drop for ResourceLease {
    fn drop(&mut self) {
        if let Some(entry) = self.entry.take() {
            let _ = entry.cleanup();
        }
        self.lock.take();
    }
}

fn lease_name(name: &OsStr) -> OsString {
    let mut first = 0xcbf2_9ce4_8422_2325_u64;
    let mut second = 0x8422_2325_cbf2_9ce4_u64;
    for byte in name.as_bytes() {
        first ^= u64::from(*byte);
        first = first.wrapping_mul(0x0000_0100_0000_01b3);
        second ^= u64::from(*byte).rotate_left(1);
        second = second.wrapping_mul(0x9e37_79b1_85eb_ca87);
    }
    OsString::from(format!(
        ".nuwax-video-relay-{first:016x}{second:016x}.lease"
    ))
}

struct PairLeases {
    shm_anchor: PathAnchor,
    uds_anchor: PathAnchor,
    shm_lease: Option<ResourceLease>,
    uds_lease: Option<ResourceLease>,
}

impl PairLeases {
    fn acquire(shm_path: &Path, uds_path: &Path) -> Result<Self, VideoRelayStartError> {
        let shm_anchor = PathAnchor::open(shm_path).map_err(|error| {
            start_error_without_thread(
                VideoRelayFailure::from_error("打开共享内存稳定父目录", &error),
                VideoRelayCleanup::empty(),
            )
        })?;
        let uds_anchor = PathAnchor::open(uds_path).map_err(|error| {
            start_error_without_thread(
                VideoRelayFailure::from_error("打开 UDS 稳定父目录", &error),
                VideoRelayCleanup::empty(),
            )
        })?;
        if shm_anchor.same_location(&uds_anchor) {
            return Err(start_error_without_thread(
                VideoRelayFailure::new(
                    "取得视频路径对租约",
                    io::ErrorKind::InvalidInput,
                    "视频共享内存与 UDS 指向同一稳定目录节点",
                ),
                VideoRelayCleanup::empty(),
            ));
        }

        let shm_first = shm_anchor.stable_key() < uds_anchor.stable_key();
        let (first_anchor, second_anchor) = if shm_first {
            (&shm_anchor, &uds_anchor)
        } else {
            (&uds_anchor, &shm_anchor)
        };
        let first = ResourceLease::acquire(first_anchor).map_err(|error| {
            start_error_without_thread(
                VideoRelayFailure::from_error("取得首个视频路径租约", &error),
                VideoRelayCleanup::empty(),
            )
        })?;
        let second = match ResourceLease::acquire(second_anchor) {
            Ok(lease) => lease,
            Err(error) => {
                let first_cleanup = first.cleanup();
                let mut cleanup = VideoRelayCleanup::empty();
                if shm_first {
                    cleanup.shm_lease = first_cleanup;
                } else {
                    cleanup.uds_lease = first_cleanup;
                }
                return Err(start_error_without_thread(
                    VideoRelayFailure::from_error("原子取得视频路径对租约", &error),
                    cleanup,
                ));
            }
        };
        let (shm_lease, uds_lease) = if shm_first {
            (first, second)
        } else {
            (second, first)
        };
        Ok(Self {
            shm_anchor,
            uds_anchor,
            shm_lease: Some(shm_lease),
            uds_lease: Some(uds_lease),
        })
    }

    fn cleanup(&mut self) -> (VideoRelayPathCleanup, VideoRelayPathCleanup) {
        let uds = self
            .uds_lease
            .take()
            .map_or(VideoRelayPathCleanup::NotCreated, ResourceLease::cleanup);
        let shm = self
            .shm_lease
            .take()
            .map_or(VideoRelayPathCleanup::NotCreated, ResourceLease::cleanup);
        (uds, shm)
    }
}

impl Drop for PairLeases {
    fn drop(&mut self) {
        let _ = self.cleanup();
    }
}

mod process_helper {
    use super::*;
    use std::io::Write;
    use std::os::fd::RawFd;
    use std::os::unix::ffi::OsStringExt;
    use std::os::unix::process::CommandExt;
    use std::process::{Child, Command, ExitStatus, Stdio};
    use std::sync::atomic::AtomicU32;

    const CONTROL_GENERATION_HIGH_OFFSET: usize = 16;
    const CONTROL_GENERATION_LOW_OFFSET: usize = 24;
    const CONTROL_STOP_OFFSET: usize = 32;
    const CONTROL_PUBLISH_STATE_OFFSET: usize = 36;
    const CONTROL_PUBLISHED_SEQUENCE_OFFSET: usize = 40;
    const CONTROL_HELPER_PHASE_OFFSET: usize = 48;
    const CONTROL_TEST_FAULTS_OFFSET: usize = 56;
    const CONTROL_TEST_RESUME_OFFSET: usize = 64;
    #[cfg(test)]
    const CONTROL_TEST_PAYLOAD_POINTER_OFFSET: usize = 72;
    #[cfg(test)]
    const CONTROL_TEST_PAYLOAD_CAPACITY_OFFSET: usize = 80;
    #[cfg(test)]
    const CONTROL_TEST_PAYLOAD_OBSERVATIONS_OFFSET: usize = 88;

    const IPC_KIND_OFFSET: usize = 12;
    const IPC_GENERATION_HIGH_OFFSET: usize = 16;
    const IPC_GENERATION_LOW_OFFSET: usize = 24;
    const IPC_CODE_OFFSET: usize = 32;
    const IPC_ERRNO_OFFSET: usize = 36;
    const IPC_SHM_DEVICE_OFFSET: usize = 40;
    const IPC_SHM_INODE_OFFSET: usize = 48;
    const IPC_UDS_DEVICE_OFFSET: usize = 56;
    const IPC_UDS_INODE_OFFSET: usize = 64;
    const IPC_FLAGS_OFFSET: usize = 72;
    const IPC_SHM_NAME_LENGTH_OFFSET: usize = 80;
    const IPC_UDS_NAME_LENGTH_OFFSET: usize = 82;
    const IPC_SHM_PARENT_LENGTH_OFFSET: usize = 84;
    const IPC_UDS_PARENT_LENGTH_OFFSET: usize = 86;

    pub(super) const PUBLISH_NOT_AUTHORIZED: u32 = 0;
    pub(super) const PUBLISH_AUTHORIZED: u32 = 1;
    pub(super) const PUBLISH_COMPLETE: u32 = 2;
    pub(super) const PUBLISH_CANCELLED: u32 = 3;
    pub(super) const PHASE_IDLE: u32 = 0;
    pub(super) const PHASE_BEFORE_INVALIDATE: u32 = 2;
    pub(super) const PHASE_AFTER_INVALIDATE: u32 = 3;
    pub(super) const PHASE_AFTER_FLUSH: u32 = 4;
    pub(super) const PHASE_MSYNC_HANG: u32 = 5;
    pub(super) const PHASE_BEFORE_SHM_CLEANUP: u32 = 6;
    pub(super) const PHASE_BEFORE_UDS_BIND: u32 = 7;
    pub(super) const PHASE_AFTER_UDS_BIND: u32 = 8;
    pub(super) const PHASE_BEFORE_PARENT_TERMINAL: u32 = 9;
    pub(super) const PHASE_AFTER_PARENT_TERMINAL: u32 = 10;
    pub(super) const PHASE_AFTER_UDS_BIND_BEFORE_IDENTITY: u32 = 11;
    pub(super) const PHASE_AFTER_UDS_IDENTITY_BEFORE_PUBLISH: u32 = 12;

    pub(super) const FAULT_BIND_UDS: u64 = 1 << 0;
    pub(super) const FAULT_STARTUP_HEADER_FLUSH: u64 = 1 << 1;
    pub(super) const FAULT_STARTUP_TIMEOUT: u64 = 1 << 2;
    pub(super) const FAULT_HELPER_PANIC: u64 = 1 << 3;
    pub(super) const FAULT_COMMIT_FLUSH: u64 = 1 << 4;
    pub(super) const FAULT_CLEANUP_FLUSH: u64 = 1 << 5;
    pub(super) const FAULT_SHM_CLEANUP: u64 = 1 << 6;
    pub(super) const FAULT_MSYNC_HANG: u64 = 1 << 7;
    pub(super) const FAULT_IPC_TRUNCATED: u64 = 1 << 8;
    pub(super) const FAULT_WRONG_GENERATION: u64 = 1 << 9;
    pub(super) const FAULT_PAUSE_BEFORE_INVALIDATE: u64 = 1 << 11;
    pub(super) const FAULT_PAUSE_AFTER_INVALIDATE: u64 = 1 << 12;
    pub(super) const FAULT_PAUSE_AFTER_FLUSH: u64 = 1 << 13;
    pub(super) const FAULT_PAUSE_SHM_CLEANUP: u64 = 1 << 14;
    pub(super) const FAULT_SUPERVISOR_SPAWN: u64 = 1 << 15;
    pub(super) const FAULT_PARENT_TRY_WAIT_STARTUP: u64 = 1 << 16;
    pub(super) const FAULT_PARENT_TRY_WAIT_SUPERVISOR: u64 = 1 << 17;
    pub(super) const FAULT_PAUSE_BEFORE_UDS_BIND: u64 = 1 << 18;
    pub(super) const FAULT_PAUSE_AFTER_UDS_BIND: u64 = 1 << 19;
    pub(super) const FAULT_PAUSE_BEFORE_PARENT_TERMINAL: u64 = 1 << 20;
    pub(super) const FAULT_PAUSE_AFTER_PARENT_TERMINAL: u64 = 1 << 21;
    pub(super) const FAULT_PAUSE_AFTER_UDS_BIND_BEFORE_IDENTITY: u64 = 1 << 22;
    pub(super) const FAULT_PAUSE_AFTER_UDS_IDENTITY_BEFORE_PUBLISH: u64 = 1 << 23;
    pub(super) const FAULT_SHM_PERMISSIONS: u64 = 1 << 24;
    pub(super) const FAULT_SHM_METADATA: u64 = 1 << 25;
    pub(super) const FAULT_SHM_TRY_CLONE: u64 = 1 << 26;
    pub(super) const FAULT_SHM_IDENTITY: u64 = 1 << 27;
    pub(super) const FAULT_SHM_PUBLISH_REPLACEMENT: u64 = 1 << 28;

    const HELPER_CODE_STOP_REQUESTED: u32 = 1;
    const HELPER_CODE_INIT_SHM_FAILED: u32 = 2;
    const HELPER_CODE_BIND_UDS_FAILED: u32 = 3;
    const HELPER_CODE_COMMIT_FAILED: u32 = 4;
    const HELPER_CODE_CLEANUP_FAILED: u32 = 5;
    const HELPER_CODE_PARENT_GONE: u32 = 6;
    const HELPER_FLAG_MMAP_CREATED: u64 = 1 << 0;
    const HELPER_FLAG_MMAP_FLUSHED: u64 = 1 << 1;
    const HELPER_FLAG_MMAP_FLUSH_FAILED: u64 = 1 << 2;
    const HELPER_FLAG_UDS_CLEAN: u64 = 1 << 3;
    const HELPER_FLAG_SHM_CLEAN: u64 = 1 << 4;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(super) struct Generation {
        pub high: u64,
        pub low: u64,
    }

    impl Generation {
        pub(super) fn fresh() -> Self {
            let mut value = uuid::Uuid::new_v4().as_u128();
            if value == 0 {
                value = 1;
            }
            Self {
                high: (value >> 64) as u64,
                low: value as u64,
            }
        }

        pub(super) fn raw_bytes(self) -> [u8; 16] {
            let mut out = [0_u8; 16];
            out[..8].copy_from_slice(&self.high.to_be_bytes());
            out[8..].copy_from_slice(&self.low.to_be_bytes());
            out
        }
    }

    pub(super) struct ControlPage {
        file: File,
        mmap: MmapMut,
    }

    impl ControlPage {
        pub(super) fn create(generation: Generation, test_faults: u64) -> io::Result<Arc<Self>> {
            let name = CString::new("nuwax-video-relay-control-v1").unwrap();
            let descriptor =
                unsafe { nix::libc::memfd_create(name.as_ptr(), nix::libc::MFD_CLOEXEC) };
            if descriptor < 0 {
                return Err(io::Error::last_os_error());
            }
            let file = unsafe { File::from_raw_fd(descriptor) };
            file.set_len(CONTROL_PAGE_SIZE as u64)?;
            let mut mmap = unsafe { MmapMut::map_mut(&file)? };
            mmap.fill(0);
            write_u64(&mut mmap, 0, CONTROL_MAGIC);
            write_u32(&mut mmap, 8, CONTROL_VERSION);
            write_u32(&mut mmap, 12, CONTROL_PAGE_SIZE as u32);
            write_u64(&mut mmap, CONTROL_GENERATION_HIGH_OFFSET, generation.high);
            write_u64(&mut mmap, CONTROL_GENERATION_LOW_OFFSET, generation.low);
            write_u64(&mut mmap, CONTROL_TEST_FAULTS_OFFSET, test_faults);
            fence(Ordering::Release);
            Ok(Arc::new(Self { file, mmap }))
        }

        pub(super) fn from_inherited(file: File) -> io::Result<Arc<Self>> {
            let mmap = unsafe { MmapMut::map_mut(&file)? };
            if mmap.len() != CONTROL_PAGE_SIZE
                || read_u64(&mmap, 0) != CONTROL_MAGIC
                || read_u32(&mmap, 8) != CONTROL_VERSION
                || read_u32(&mmap, 12) != CONTROL_PAGE_SIZE as u32
            {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "视频 helper control page 版本或长度错误",
                ));
            }
            Ok(Arc::new(Self { file, mmap }))
        }

        pub(super) fn file_descriptor(&self) -> RawFd {
            self.file.as_raw_fd()
        }

        pub(super) fn generation(&self) -> Generation {
            Generation {
                high: self
                    .atomic_u64(CONTROL_GENERATION_HIGH_OFFSET)
                    .load(Ordering::Acquire),
                low: self
                    .atomic_u64(CONTROL_GENERATION_LOW_OFFSET)
                    .load(Ordering::Acquire),
            }
        }

        pub(super) fn generation_matches(&self, generation: Generation) -> bool {
            self.generation() == generation && generation != Generation { high: 0, low: 0 }
        }

        #[cfg(test)]
        pub(super) fn invalidate_generation_for_test(&self) {
            self.atomic_u64(CONTROL_GENERATION_HIGH_OFFSET)
                .fetch_xor(1, Ordering::AcqRel);
        }

        pub(super) fn request_stop(&self) -> bool {
            let publish = self.atomic_u32(CONTROL_PUBLISH_STATE_OFFSET);
            let mut state = publish.load(Ordering::Acquire);
            while matches!(state, PUBLISH_NOT_AUTHORIZED | PUBLISH_AUTHORIZED) {
                match publish.compare_exchange_weak(
                    state,
                    PUBLISH_CANCELLED,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => break,
                    Err(actual) => state = actual,
                }
            }
            self.atomic_u32(CONTROL_STOP_OFFSET)
                .swap(1, Ordering::AcqRel)
                == 0
        }

        pub(super) fn stop_requested(&self) -> bool {
            self.atomic_u32(CONTROL_STOP_OFFSET).load(Ordering::Acquire) != 0
                || HELPER_SIGNAL_STOP.load(Ordering::Acquire)
        }

        pub(super) fn authorize_publish(&self) -> bool {
            self.atomic_u32(CONTROL_PUBLISH_STATE_OFFSET)
                .compare_exchange(
                    PUBLISH_NOT_AUTHORIZED,
                    PUBLISH_AUTHORIZED,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
        }

        pub(super) fn publish_state(&self) -> u32 {
            self.atomic_u32(CONTROL_PUBLISH_STATE_OFFSET)
                .load(Ordering::Acquire)
        }

        pub(super) fn complete_publish(&self, generation: Generation) -> bool {
            self.generation_matches(generation)
                && self
                    .atomic_u32(CONTROL_PUBLISH_STATE_OFFSET)
                    .compare_exchange(
                        PUBLISH_AUTHORIZED,
                        PUBLISH_COMPLETE,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    )
                    .is_ok()
        }

        pub(super) fn published_sequence(&self) -> u64 {
            self.atomic_u64(CONTROL_PUBLISHED_SEQUENCE_OFFSET)
                .load(Ordering::Acquire)
        }

        pub(super) fn set_published_sequence(&self, sequence: u64) {
            self.atomic_u64(CONTROL_PUBLISHED_SEQUENCE_OFFSET)
                .store(sequence, Ordering::Release);
        }

        #[cfg(test)]
        pub(super) fn helper_phase(&self) -> u32 {
            self.atomic_u32(CONTROL_HELPER_PHASE_OFFSET)
                .load(Ordering::Acquire)
        }

        pub(super) fn set_helper_phase(&self, phase: u32) {
            self.atomic_u32(CONTROL_HELPER_PHASE_OFFSET)
                .store(phase, Ordering::Release);
        }

        pub(super) fn take_test_fault(&self, mask: u64) -> bool {
            let faults = self.atomic_u64(CONTROL_TEST_FAULTS_OFFSET);
            let mut current = faults.load(Ordering::Acquire);
            loop {
                if current & mask == 0 {
                    return false;
                }
                match faults.compare_exchange_weak(
                    current,
                    current & !mask,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => return true,
                    Err(actual) => current = actual,
                }
            }
        }

        #[cfg(test)]
        pub(super) fn add_test_fault(&self, mask: u64) {
            self.atomic_u64(CONTROL_TEST_FAULTS_OFFSET)
                .fetch_or(mask, Ordering::AcqRel);
        }

        #[cfg(test)]
        pub(super) fn resume_test_gate(&self) {
            self.atomic_u32(CONTROL_TEST_RESUME_OFFSET)
                .store(1, Ordering::Release);
        }

        pub(super) fn take_test_resume(&self) -> bool {
            self.atomic_u32(CONTROL_TEST_RESUME_OFFSET)
                .swap(0, Ordering::AcqRel)
                != 0
        }

        fn atomic_u32(&self, offset: usize) -> &AtomicU32 {
            debug_assert_eq!(offset % std::mem::align_of::<AtomicU32>(), 0);
            unsafe { &*self.mmap.as_ptr().add(offset).cast::<AtomicU32>() }
        }

        fn atomic_u64(&self, offset: usize) -> &AtomicU64 {
            debug_assert_eq!(offset % std::mem::align_of::<AtomicU64>(), 0);
            unsafe { &*self.mmap.as_ptr().add(offset).cast::<AtomicU64>() }
        }
    }

    #[cfg(test)]
    fn record_payload_buffer_for_test(control: &ControlPage, pointer: *const u8, capacity: usize) {
        control
            .atomic_u64(CONTROL_TEST_PAYLOAD_POINTER_OFFSET)
            .store(pointer as usize as u64, Ordering::Relaxed);
        control
            .atomic_u64(CONTROL_TEST_PAYLOAD_CAPACITY_OFFSET)
            .store(capacity as u64, Ordering::Relaxed);
        control
            .atomic_u64(CONTROL_TEST_PAYLOAD_OBSERVATIONS_OFFSET)
            .fetch_add(1, Ordering::Release);
    }

    #[cfg(test)]
    pub(super) fn payload_buffer_observation_for_test(control: &ControlPage) -> (u64, u64, u64) {
        let observations = control
            .atomic_u64(CONTROL_TEST_PAYLOAD_OBSERVATIONS_OFFSET)
            .load(Ordering::Acquire);
        let pointer = control
            .atomic_u64(CONTROL_TEST_PAYLOAD_POINTER_OFFSET)
            .load(Ordering::Acquire);
        let capacity = control
            .atomic_u64(CONTROL_TEST_PAYLOAD_CAPACITY_OFFSET)
            .load(Ordering::Acquire);
        (pointer, capacity, observations)
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[repr(u32)]
    pub(super) enum IpcKind {
        Start = 1,
        Prepared = 2,
        Ready = 3,
        Terminal = 4,
    }

    pub(super) struct IpcRecord {
        bytes: Box<[u8; IPC_RECORD_SIZE]>,
    }

    impl IpcRecord {
        pub(super) fn new(kind: IpcKind, generation: Generation) -> Self {
            let mut bytes = Box::new([0_u8; IPC_RECORD_SIZE]);
            write_u64(&mut bytes[..], 0, IPC_MAGIC);
            write_u32(&mut bytes[..], 8, IPC_VERSION);
            write_u32(&mut bytes[..], IPC_KIND_OFFSET, kind as u32);
            write_u64(&mut bytes[..], IPC_GENERATION_HIGH_OFFSET, generation.high);
            write_u64(&mut bytes[..], IPC_GENERATION_LOW_OFFSET, generation.low);
            Self { bytes }
        }

        pub(super) fn read(stream: &mut UnixStream) -> io::Result<Self> {
            let mut record = Self {
                bytes: Box::new([0_u8; IPC_RECORD_SIZE]),
            };
            stream.read_exact(&mut record.bytes[..])?;
            record.validate()?;
            Ok(record)
        }

        pub(super) fn write(&self, stream: &mut UnixStream) -> io::Result<()> {
            stream.write_all(&self.bytes[..])
        }

        pub(super) fn write_prefix(
            &self,
            stream: &mut UnixStream,
            length: usize,
        ) -> io::Result<()> {
            stream.write_all(&self.bytes[..length.min(IPC_RECORD_SIZE)])
        }

        pub(super) fn validate(&self) -> io::Result<()> {
            if read_u64(&self.bytes[..], 0) != IPC_MAGIC
                || read_u32(&self.bytes[..], 8) != IPC_VERSION
            {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "视频 helper IPC magic/version 错误",
                ));
            }
            self.kind()?;
            if self.generation() == (Generation { high: 0, low: 0 }) {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "视频 helper IPC generation 不能为零",
                ));
            }
            Ok(())
        }

        pub(super) fn kind(&self) -> io::Result<IpcKind> {
            match read_u32(&self.bytes[..], IPC_KIND_OFFSET) {
                1 => Ok(IpcKind::Start),
                2 => Ok(IpcKind::Prepared),
                3 => Ok(IpcKind::Ready),
                4 => Ok(IpcKind::Terminal),
                other => Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("未知视频 helper IPC 类型: {other}"),
                )),
            }
        }

        pub(super) fn generation(&self) -> Generation {
            Generation {
                high: read_u64(&self.bytes[..], IPC_GENERATION_HIGH_OFFSET),
                low: read_u64(&self.bytes[..], IPC_GENERATION_LOW_OFFSET),
            }
        }

        pub(super) fn set_generation(&mut self, generation: Generation) {
            write_u64(
                &mut self.bytes[..],
                IPC_GENERATION_HIGH_OFFSET,
                generation.high,
            );
            write_u64(
                &mut self.bytes[..],
                IPC_GENERATION_LOW_OFFSET,
                generation.low,
            );
        }

        pub(super) fn set_code(&mut self, code: u32, errno: i32) {
            write_u32(&mut self.bytes[..], IPC_CODE_OFFSET, code);
            write_i32(&mut self.bytes[..], IPC_ERRNO_OFFSET, errno);
        }

        pub(super) fn code(&self) -> (u32, i32) {
            (
                read_u32(&self.bytes[..], IPC_CODE_OFFSET),
                read_i32(&self.bytes[..], IPC_ERRNO_OFFSET),
            )
        }

        pub(super) fn set_identities(&mut self, shm: (u64, u64), uds: (u64, u64)) {
            write_u64(&mut self.bytes[..], IPC_SHM_DEVICE_OFFSET, shm.0);
            write_u64(&mut self.bytes[..], IPC_SHM_INODE_OFFSET, shm.1);
            write_u64(&mut self.bytes[..], IPC_UDS_DEVICE_OFFSET, uds.0);
            write_u64(&mut self.bytes[..], IPC_UDS_INODE_OFFSET, uds.1);
        }

        pub(super) fn identities(&self) -> ((u64, u64), (u64, u64)) {
            (
                (
                    read_u64(&self.bytes[..], IPC_SHM_DEVICE_OFFSET),
                    read_u64(&self.bytes[..], IPC_SHM_INODE_OFFSET),
                ),
                (
                    read_u64(&self.bytes[..], IPC_UDS_DEVICE_OFFSET),
                    read_u64(&self.bytes[..], IPC_UDS_INODE_OFFSET),
                ),
            )
        }

        pub(super) fn set_flags(&mut self, flags: u64) {
            write_u64(&mut self.bytes[..], IPC_FLAGS_OFFSET, flags);
        }

        pub(super) fn flags(&self) -> u64 {
            read_u64(&self.bytes[..], IPC_FLAGS_OFFSET)
        }

        pub(super) fn set_config(
            &mut self,
            shm_anchor: &PathAnchor,
            uds_anchor: &PathAnchor,
        ) -> io::Result<()> {
            let shm_name = shm_anchor.name.as_bytes();
            let uds_name = uds_anchor.name.as_bytes();
            let shm_parent = shm_anchor.directory.display_path.as_os_str().as_bytes();
            let uds_parent = uds_anchor.directory.display_path.as_os_str().as_bytes();
            validate_component_length(shm_name, IPC_NAME_CAPACITY, "SHM 文件名")?;
            validate_component_length(uds_name, IPC_NAME_CAPACITY, "UDS 文件名")?;
            validate_component_length(shm_parent, IPC_PATH_CAPACITY, "SHM 父路径")?;
            validate_component_length(uds_parent, IPC_PATH_CAPACITY, "UDS 父路径")?;
            write_u16(
                &mut self.bytes[..],
                IPC_SHM_NAME_LENGTH_OFFSET,
                shm_name.len() as u16,
            );
            write_u16(
                &mut self.bytes[..],
                IPC_UDS_NAME_LENGTH_OFFSET,
                uds_name.len() as u16,
            );
            write_u16(
                &mut self.bytes[..],
                IPC_SHM_PARENT_LENGTH_OFFSET,
                shm_parent.len() as u16,
            );
            write_u16(
                &mut self.bytes[..],
                IPC_UDS_PARENT_LENGTH_OFFSET,
                uds_parent.len() as u16,
            );
            self.bytes[IPC_SHM_NAME_OFFSET..IPC_SHM_NAME_OFFSET + shm_name.len()]
                .copy_from_slice(shm_name);
            self.bytes[IPC_UDS_NAME_OFFSET..IPC_UDS_NAME_OFFSET + uds_name.len()]
                .copy_from_slice(uds_name);
            self.bytes[IPC_SHM_PARENT_OFFSET..IPC_SHM_PARENT_OFFSET + shm_parent.len()]
                .copy_from_slice(shm_parent);
            self.bytes[IPC_UDS_PARENT_OFFSET..IPC_UDS_PARENT_OFFSET + uds_parent.len()]
                .copy_from_slice(uds_parent);
            self.set_identities(
                (shm_anchor.directory.device, shm_anchor.directory.inode),
                (uds_anchor.directory.device, uds_anchor.directory.inode),
            );
            Ok(())
        }

        pub(super) fn config(&self) -> io::Result<HelperConfig> {
            let shm_name_length = read_u16(&self.bytes[..], IPC_SHM_NAME_LENGTH_OFFSET) as usize;
            let uds_name_length = read_u16(&self.bytes[..], IPC_UDS_NAME_LENGTH_OFFSET) as usize;
            let shm_parent_length =
                read_u16(&self.bytes[..], IPC_SHM_PARENT_LENGTH_OFFSET) as usize;
            let uds_parent_length =
                read_u16(&self.bytes[..], IPC_UDS_PARENT_LENGTH_OFFSET) as usize;
            validate_component_length(&vec![0; shm_name_length], IPC_NAME_CAPACITY, "SHM 文件名")?;
            validate_component_length(&vec![0; uds_name_length], IPC_NAME_CAPACITY, "UDS 文件名")?;
            validate_component_length(
                &vec![0; shm_parent_length],
                IPC_PATH_CAPACITY,
                "SHM 父路径",
            )?;
            validate_component_length(
                &vec![0; uds_parent_length],
                IPC_PATH_CAPACITY,
                "UDS 父路径",
            )?;
            let (shm_directory, uds_directory) = self.identities();
            Ok(HelperConfig {
                generation: self.generation(),
                shm_name: OsString::from_vec(
                    self.bytes[IPC_SHM_NAME_OFFSET..IPC_SHM_NAME_OFFSET + shm_name_length].to_vec(),
                ),
                uds_name: OsString::from_vec(
                    self.bytes[IPC_UDS_NAME_OFFSET..IPC_UDS_NAME_OFFSET + uds_name_length].to_vec(),
                ),
                shm_parent: PathBuf::from(OsString::from_vec(
                    self.bytes[IPC_SHM_PARENT_OFFSET..IPC_SHM_PARENT_OFFSET + shm_parent_length]
                        .to_vec(),
                )),
                uds_parent: PathBuf::from(OsString::from_vec(
                    self.bytes[IPC_UDS_PARENT_OFFSET..IPC_UDS_PARENT_OFFSET + uds_parent_length]
                        .to_vec(),
                )),
                shm_directory,
                uds_directory,
            })
        }
    }

    pub(super) struct HelperConfig {
        pub generation: Generation,
        pub shm_name: OsString,
        pub uds_name: OsString,
        pub shm_parent: PathBuf,
        pub uds_parent: PathBuf,
        pub shm_directory: (u64, u64),
        pub uds_directory: (u64, u64),
    }

    pub(super) fn request_present() -> bool {
        std::env::var_os(HELPER_REQUEST_ENV).is_some()
    }

    pub(super) fn inherited_descriptor(name: &str) -> io::Result<RawFd> {
        let value = std::env::var(name).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("缺少视频 helper 继承 fd: {name}"),
            )
        })?;
        let descriptor = value.parse::<RawFd>().map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("视频 helper 继承 fd 非法: {name}"),
            )
        })?;
        if descriptor < 0 || unsafe { nix::libc::fcntl(descriptor, nix::libc::F_GETFD) } < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(descriptor)
    }

    pub(super) fn expected_parent_pid() -> io::Result<nix::libc::pid_t> {
        std::env::var(HELPER_PARENT_PID_ENV)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "缺少 helper parent pid"))?
            .parse::<nix::libc::pid_t>()
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "helper parent pid 非法"))
    }

    pub(super) fn establish_parent_death_contract(
        expected_parent: nix::libc::pid_t,
    ) -> io::Result<File> {
        HELPER_SIGNAL_STOP.store(false, Ordering::Release);
        let parent_pid = u32::try_from(expected_parent).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidInput, "视频 helper parent pid 非法")
        })?;
        let parent_pidfd = open_pidfd(parent_pid)?;
        let mut action: nix::libc::sigaction = unsafe { std::mem::zeroed() };
        action.sa_sigaction = helper_stop_signal as usize;
        action.sa_flags = 0;
        unsafe {
            nix::libc::sigemptyset(&mut action.sa_mask);
            if nix::libc::sigaction(nix::libc::SIGTERM, &action, std::ptr::null_mut()) != 0 {
                return Err(io::Error::last_os_error());
            }
            if nix::libc::prctl(nix::libc::PR_SET_PDEATHSIG, nix::libc::SIGTERM) != 0 {
                return Err(io::Error::last_os_error());
            }
            if nix::libc::getppid() != expected_parent {
                return Err(io::Error::new(
                    io::ErrorKind::BrokenPipe,
                    "视频 helper 父进程在 PDEATHSIG 建立前已退出",
                ));
            }
        }
        Ok(parent_pidfd)
    }

    struct WatchdogCleanupState {
        parent_gone: bool,
        shm: Option<OwnedEntry>,
        uds_candidate: Option<OwnedEntry>,
        uds_public: Option<OwnedEntry>,
        shm_lease: Option<OwnedEntry>,
        uds_lease: Option<OwnedEntry>,
    }

    struct WatchdogOwnedEntries {
        shm: Option<OwnedEntry>,
        uds_candidate: Option<OwnedEntry>,
        uds_public: Option<OwnedEntry>,
        shm_lease: Option<OwnedEntry>,
        uds_lease: Option<OwnedEntry>,
    }

    impl WatchdogCleanupState {
        fn with_leases(shm_lease: OwnedEntry, uds_lease: OwnedEntry) -> Self {
            Self {
                parent_gone: false,
                shm: None,
                uds_candidate: None,
                uds_public: None,
                shm_lease: Some(shm_lease),
                uds_lease: Some(uds_lease),
            }
        }

        fn take_after_parent_death(&mut self) -> WatchdogOwnedEntries {
            self.parent_gone = true;
            WatchdogOwnedEntries {
                shm: self.shm.take(),
                uds_candidate: self.uds_candidate.take(),
                uds_public: self.uds_public.take(),
                shm_lease: self.shm_lease.take(),
                uds_lease: self.uds_lease.take(),
            }
        }
    }

    impl WatchdogOwnedEntries {
        fn cleanup_after_parent_death(self) -> VideoRelayCleanup {
            let shm = Self::cleanup_exact(self.shm, "SHM");
            let uds_public = Self::cleanup_exact(self.uds_public, "公共 UDS");
            let uds_candidate = Self::cleanup_exact(self.uds_candidate, "UDS candidate");
            let uds = merge_path_cleanup(uds_public, uds_candidate);
            // lease 是跨 owner 代际复用的协调 inode。旧 helper 只能关闭身份句柄，
            // 不能在新 owner 已接管同一 inode 后 unlink 路径。
            drop(self.shm_lease);
            drop(self.uds_lease);
            let shm_lease = VideoRelayPathCleanup::AlreadyAbsent;
            let uds_lease = VideoRelayPathCleanup::AlreadyAbsent;
            VideoRelayCleanup {
                mmap: VideoRelayMmapCleanup::NotCreated,
                uds,
                shm,
                uds_lease,
                shm_lease,
            }
        }

        fn cleanup_exact(entry: Option<OwnedEntry>, label: &str) -> VideoRelayPathCleanup {
            let Some(entry) = entry else {
                return VideoRelayPathCleanup::NotCreated;
            };
            let path = entry.anchor.directory.display_child(&entry.name);
            match entry.cleanup() {
                VideoRelayPathCleanup::ReplacementPreserved => VideoRelayPathCleanup::Residual {
                    path,
                    detail: format!("父进程退出时 {label} 已被外来身份替换，已保留 replacement"),
                },
                cleanup => cleanup,
            }
        }
    }

    fn merge_path_cleanup(
        first: VideoRelayPathCleanup,
        second: VideoRelayPathCleanup,
    ) -> VideoRelayPathCleanup {
        if !first.is_complete() {
            return first;
        }
        if !second.is_complete() {
            return second;
        }
        if matches!(first, VideoRelayPathCleanup::ReplacementPreserved) {
            return first;
        }
        if matches!(second, VideoRelayPathCleanup::ReplacementPreserved) {
            return second;
        }
        if matches!(first, VideoRelayPathCleanup::Removed) {
            return first;
        }
        if matches!(second, VideoRelayPathCleanup::Removed) {
            return second;
        }
        if matches!(first, VideoRelayPathCleanup::AlreadyAbsent) {
            return first;
        }
        second
    }

    fn start_parent_death_watchdog(
        parent_pidfd: File,
        control: Arc<ControlPage>,
        cleanup: Arc<Mutex<WatchdogCleanupState>>,
    ) -> io::Result<()> {
        thread::Builder::new()
            .name("nuwax-video-parent-watchdog".to_string())
            .spawn(move || {
                let mut poll_descriptor = nix::libc::pollfd {
                    fd: parent_pidfd.as_raw_fd(),
                    events: nix::libc::POLLIN,
                    revents: 0,
                };
                loop {
                    let result = unsafe { nix::libc::poll(&mut poll_descriptor, 1, -1) };
                    if result > 0 {
                        break;
                    }
                    if result < 0 && io::Error::last_os_error().kind() == io::ErrorKind::Interrupted
                    {
                        continue;
                    }
                    if result < 0 {
                        break;
                    }
                }
                HELPER_SIGNAL_STOP.store(true, Ordering::Release);
                control.request_stop();
                let cleanup = {
                    let mut state = lock_unpoisoned(&cleanup);
                    state.take_after_parent_death().cleanup_after_parent_death()
                };
                if !cleanup.is_complete() {
                    error!(?cleanup, "视频 helper 父进程退出后的精确清理不完整");
                }
                let kill_at = Instant::now() + TERM_GRACE_LIMIT;
                while Instant::now() < kill_at {
                    thread::sleep(Duration::from_millis(2));
                }
                unsafe {
                    nix::libc::kill(nix::libc::getpid(), nix::libc::SIGKILL);
                    nix::libc::_exit(128 + nix::libc::SIGKILL);
                }
            })
            .map(drop)
    }

    fn pidfd_is_ready(pidfd: &File) -> io::Result<bool> {
        let mut poll_descriptor = nix::libc::pollfd {
            fd: pidfd.as_raw_fd(),
            events: nix::libc::POLLIN,
            revents: 0,
        };
        loop {
            let result = unsafe { nix::libc::poll(&mut poll_descriptor, 1, 0) };
            if result >= 0 {
                return Ok(result > 0);
            }
            let error = io::Error::last_os_error();
            if error.kind() != io::ErrorKind::Interrupted {
                return Err(error);
            }
        }
    }

    fn inherited_lease_entry(identity_file: File, anchor: &PathAnchor) -> io::Result<OwnedEntry> {
        let flags = unsafe { nix::libc::fcntl(identity_file.as_raw_fd(), nix::libc::F_GETFL) };
        if flags < 0 {
            return Err(io::Error::last_os_error());
        }
        if flags & nix::libc::O_PATH != nix::libc::O_PATH {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "视频 helper lease 身份 fd 必须是 O_PATH",
            ));
        }
        let name = lease_name(&anchor.name);
        let metadata = anchor.directory.metadata(&name)?.ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::NotFound,
                format!(
                    "视频 helper lease 已不存在: {}",
                    anchor.directory.display_child(&name).display()
                ),
            )
        })?;
        if !metadata.is_file() || metadata.nlink() != 1 || metadata.mode() & 0o777 != 0o600 {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "视频 helper lease 身份不是 0600 单链接普通文件",
            ));
        }
        OwnedEntry::from_metadata(
            anchor.clone(),
            name,
            &metadata,
            identity_file,
            EntryRole::Lease,
        )
    }

    pub(super) fn helper_candidate_name(generation: Generation) -> OsString {
        OsString::from(format!(
            ".nuwax-video-candidate-{:016x}{:016x}",
            generation.high, generation.low
        ))
    }

    fn helper_candidate_anchor(public_anchor: &PathAnchor, generation: Generation) -> PathAnchor {
        let name = helper_candidate_name(generation);
        PathAnchor {
            public_path: public_anchor.directory.display_child(&name),
            directory: public_anchor.directory.clone(),
            name,
        }
    }

    pub(super) fn duplicate_inheritable(descriptor: RawFd) -> io::Result<File> {
        let duplicate = unsafe { nix::libc::fcntl(descriptor, nix::libc::F_DUPFD, 3) };
        if duplicate < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(unsafe { File::from_raw_fd(duplicate) })
        }
    }

    pub(super) fn read_record_until(
        stream: &mut UnixStream,
        deadline: Instant,
    ) -> io::Result<IpcRecord> {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "等待视频 helper IPC 超时",
            ));
        }
        stream.set_read_timeout(Some(remaining))?;
        IpcRecord::read(stream).map_err(|error| {
            if matches!(
                error.kind(),
                io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
            ) {
                io::Error::new(io::ErrorKind::TimedOut, "等待视频 helper IPC 超时")
            } else {
                error
            }
        })
    }

    struct HelperSession {
        generation: Generation,
        control: Arc<ControlPage>,
        watchdog_cleanup: Arc<Mutex<WatchdogCleanupState>>,
        prepublish_uds_cleanup_complete: Arc<AtomicBool>,
        mmap: Option<MmapMut>,
        shm: Option<OwnedEntry>,
        uds: Option<OwnedEntry>,
        listener: Option<UnixListener>,
    }

    impl HelperSession {
        fn new(
            generation: Generation,
            control: Arc<ControlPage>,
            watchdog_cleanup: Arc<Mutex<WatchdogCleanupState>>,
        ) -> Self {
            Self {
                generation,
                control,
                watchdog_cleanup,
                prepublish_uds_cleanup_complete: Arc::new(AtomicBool::new(true)),
                mmap: None,
                shm: None,
                uds: None,
                listener: None,
            }
        }

        fn finish(mut self, mut code: u32, mut errno: i32) -> IpcRecord {
            self.control.set_published_sequence(0);
            let shm_identity = self
                .shm
                .as_ref()
                .map(OwnedEntry::identity)
                .unwrap_or((0, 0));
            let uds_identity = self
                .uds
                .as_ref()
                .map(OwnedEntry::identity)
                .unwrap_or((0, 0));
            let mut flags = 0_u64;
            if let Some(mmap) = self.mmap.take() {
                flags |= HELPER_FLAG_MMAP_CREATED;
                store_header_sequence(&mmap, 0);
                let flush = if self.control.take_test_fault(FAULT_CLEANUP_FLUSH) {
                    Err(io::Error::other("注入 helper 终态共享内存刷新失败"))
                } else {
                    mmap.flush()
                };
                if let Err(error) = flush {
                    flags |= HELPER_FLAG_MMAP_FLUSH_FAILED;
                    if code == HELPER_CODE_STOP_REQUESTED {
                        code = HELPER_CODE_CLEANUP_FAILED;
                    }
                    errno = helper_errno(&error, nix::libc::EIO);
                } else {
                    flags |= HELPER_FLAG_MMAP_FLUSHED;
                }
                drop(mmap);
            }
            self.listener.take();
            let uds_cleanup = self
                .uds
                .take()
                .map_or(VideoRelayPathCleanup::NotCreated, OwnedEntry::cleanup);
            if uds_cleanup.is_complete()
                && self.prepublish_uds_cleanup_complete.load(Ordering::Acquire)
            {
                flags |= HELPER_FLAG_UDS_CLEAN;
            }
            let shm_cleanup = if self.control.take_test_fault(FAULT_SHM_CLEANUP) {
                self.shm.take();
                if code == HELPER_CODE_STOP_REQUESTED {
                    code = HELPER_CODE_CLEANUP_FAILED;
                }
                errno = nix::libc::EIO;
                VideoRelayPathCleanup::Residual {
                    path: PathBuf::from("helper-owned-shm"),
                    detail: "注入 helper SHM 清理失败".to_string(),
                }
            } else if !helper_cleanup_pause(
                &self.control,
                FAULT_PAUSE_SHM_CLEANUP,
                PHASE_BEFORE_SHM_CLEANUP,
            ) {
                self.shm.take();
                VideoRelayPathCleanup::Residual {
                    path: PathBuf::from("helper-owned-shm"),
                    detail: "helper 在 SHM 清理隔离前被停止".to_string(),
                }
            } else {
                self.shm
                    .take()
                    .map_or(VideoRelayPathCleanup::NotCreated, OwnedEntry::cleanup)
            };
            if shm_cleanup.is_complete() {
                flags |= HELPER_FLAG_SHM_CLEAN;
            }
            let mut terminal = IpcRecord::new(IpcKind::Terminal, self.generation);
            terminal.set_code(code, errno);
            terminal.set_flags(flags);
            terminal.set_identities(shm_identity, uds_identity);
            terminal
        }
    }

    impl Drop for HelperSession {
        fn drop(&mut self) {
            self.control.set_published_sequence(0);
            if let Some(mmap) = self.mmap.take() {
                store_header_sequence(&mmap, 0);
                drop(mmap);
            }
            self.listener.take();
            if let Some(entry) = self.uds.take() {
                let _ = entry.cleanup();
            }
            if let Some(entry) = self.shm.take() {
                if self.control.take_test_fault(FAULT_SHM_CLEANUP) {
                    drop(entry);
                } else {
                    let _ = entry.cleanup();
                }
            }
        }
    }

    struct ObservedEntry(OwnedEntry);

    impl ObservedEntry {
        fn from_metadata(
            anchor: PathAnchor,
            name: OsString,
            metadata: &std::fs::Metadata,
            identity_file: File,
            role: EntryRole,
        ) -> io::Result<Self> {
            OwnedEntry::from_metadata(anchor, name, metadata, identity_file, role).map(Self)
        }

        fn reopen_identity(&self) -> io::Result<File> {
            self.0.reopen_identity()
        }

        fn into_owned(self) -> OwnedEntry {
            self.0
        }
    }

    struct PreparedUds {
        listener: Option<UnixListener>,
        observed_candidate: Option<ObservedEntry>,
        candidate: Option<OwnedEntry>,
        public: Option<OwnedEntry>,
        public_anchor: PathAnchor,
        watchdog_cleanup: Arc<Mutex<WatchdogCleanupState>>,
        cleanup_complete: Arc<AtomicBool>,
    }

    impl Drop for PreparedUds {
        fn drop(&mut self) {
            self.listener.take();
            if self.observed_candidate.take().is_some() {
                self.cleanup_complete.store(false, Ordering::Release);
            }
            let public = self
                .public
                .take()
                .map_or(VideoRelayPathCleanup::NotCreated, OwnedEntry::cleanup);
            let candidate = self
                .candidate
                .take()
                .map_or(VideoRelayPathCleanup::NotCreated, OwnedEntry::cleanup);
            let exact_cleanup = |cleanup: &VideoRelayPathCleanup| {
                cleanup.is_complete()
                    && !matches!(cleanup, VideoRelayPathCleanup::ReplacementPreserved)
            };
            if !exact_cleanup(&public) || !exact_cleanup(&candidate) {
                self.cleanup_complete.store(false, Ordering::Release);
            }
        }
    }

    pub(super) fn run_requested() -> io::Result<()> {
        let expected_parent = expected_parent_pid()?;
        let parent_pidfd = establish_parent_death_contract(expected_parent)?;
        let ipc_descriptor = inherited_descriptor(HELPER_IPC_FD_ENV)?;
        let control_descriptor = inherited_descriptor(HELPER_CONTROL_FD_ENV)?;
        let shm_directory_descriptor = inherited_descriptor(HELPER_SHM_DIR_FD_ENV)?;
        let uds_directory_descriptor = inherited_descriptor(HELPER_UDS_DIR_FD_ENV)?;
        let shm_lease_descriptor = inherited_descriptor(HELPER_SHM_LEASE_FD_ENV)?;
        let uds_lease_descriptor = inherited_descriptor(HELPER_UDS_LEASE_FD_ENV)?;
        let mut ipc = unsafe { UnixStream::from_raw_fd(ipc_descriptor) };
        let control_file = unsafe { File::from_raw_fd(control_descriptor) };
        let shm_directory_file = unsafe { File::from_raw_fd(shm_directory_descriptor) };
        let uds_directory_file = unsafe { File::from_raw_fd(uds_directory_descriptor) };
        let shm_lease_file = unsafe { File::from_raw_fd(shm_lease_descriptor) };
        let uds_lease_file = unsafe { File::from_raw_fd(uds_lease_descriptor) };
        let control = ControlPage::from_inherited(control_file)?;
        let start = IpcRecord::read(&mut ipc)?;
        if start.kind()? != IpcKind::Start {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "视频 helper 首条 IPC 不是 Start",
            ));
        }
        let config = start.config()?;
        if !control.generation_matches(config.generation) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "视频 helper Start 与 control generation 不一致",
            ));
        }
        let shm_directory = StableDirectory::from_inherited(
            shm_directory_file,
            config.shm_parent.clone(),
            config.shm_directory.0,
            config.shm_directory.1,
        )?;
        let uds_directory = StableDirectory::from_inherited(
            uds_directory_file,
            config.uds_parent.clone(),
            config.uds_directory.0,
            config.uds_directory.1,
        )?;
        let shm_anchor = PathAnchor {
            public_path: config.shm_parent.join(&config.shm_name),
            directory: shm_directory,
            name: config.shm_name,
        };
        let uds_anchor = PathAnchor {
            public_path: config.uds_parent.join(&config.uds_name),
            directory: uds_directory,
            name: config.uds_name,
        };
        let shm_lease = inherited_lease_entry(shm_lease_file, &shm_anchor)?;
        let uds_lease = inherited_lease_entry(uds_lease_file, &uds_anchor)?;
        let watchdog_cleanup = Arc::new(Mutex::new(WatchdogCleanupState::with_leases(
            shm_lease, uds_lease,
        )));
        let completion_pidfd = parent_pidfd.try_clone()?;
        start_parent_death_watchdog(parent_pidfd, control.clone(), watchdog_cleanup.clone())?;
        let result = run_helper(
            &mut ipc,
            control,
            watchdog_cleanup.clone(),
            config.generation,
            shm_anchor,
            uds_anchor,
        );
        if pidfd_is_ready(&completion_pidfd)? {
            let cleanup = {
                let mut state = lock_unpoisoned(&watchdog_cleanup);
                state.take_after_parent_death().cleanup_after_parent_death()
            };
            if !cleanup.is_complete() {
                error!(?cleanup, "视频 helper 返回前的父死亡精确清理不完整");
            }
        }
        result
    }

    fn run_helper(
        ipc: &mut UnixStream,
        control: Arc<ControlPage>,
        watchdog_cleanup: Arc<Mutex<WatchdogCleanupState>>,
        generation: Generation,
        shm_anchor: PathAnchor,
        uds_anchor: PathAnchor,
    ) -> io::Result<()> {
        let mut session = HelperSession::new(generation, control.clone(), watchdog_cleanup);
        if let Err(error) = helper_init_shm(&mut session, shm_anchor) {
            let terminal = session.finish(
                HELPER_CODE_INIT_SHM_FAILED,
                helper_errno(&error, nix::libc::EIO),
            );
            terminal.write(ipc)?;
            return Ok(());
        }
        if control.take_test_fault(FAULT_HELPER_PANIC) {
            panic!("注入视频 helper panic");
        }
        if control.take_test_fault(FAULT_STARTUP_TIMEOUT) {
            while !control.stop_requested() {
                thread::sleep(SUPERVISOR_POLL_INTERVAL);
            }
            let terminal = session.finish(HELPER_CODE_STOP_REQUESTED, 0);
            terminal.write(ipc)?;
            return Ok(());
        }
        let prepared_uds = match prepare_helper_uds(
            &control,
            generation,
            uds_anchor,
            session.watchdog_cleanup.clone(),
            session.prepublish_uds_cleanup_complete.clone(),
        ) {
            Ok(prepared) => prepared,
            Err(error) => {
                let terminal = session.finish(
                    HELPER_CODE_BIND_UDS_FAILED,
                    helper_errno(&error, nix::libc::EADDRNOTAVAIL),
                );
                terminal.write(ipc)?;
                return Ok(());
            }
        };
        let shm_identity = session.shm.as_ref().unwrap().identity();
        let mut prepared = IpcRecord::new(IpcKind::Prepared, generation);
        prepared.set_identities(shm_identity, (0, 0));
        if control.take_test_fault(FAULT_WRONG_GENERATION) {
            prepared.set_generation(Generation {
                high: generation.high ^ 1,
                low: generation.low,
            });
        }
        if control.take_test_fault(FAULT_IPC_TRUNCATED) {
            prepared.write_prefix(ipc, IPC_RECORD_SIZE / 2)?;
            drop(prepared_uds);
            drop(session);
            ipc.shutdown(std::net::Shutdown::Both)?;
            return Ok(());
        }
        prepared.write(ipc)?;
        while control.publish_state() == PUBLISH_NOT_AUTHORIZED && !control.stop_requested() {
            thread::sleep(SUPERVISOR_POLL_INTERVAL);
        }
        if control.stop_requested()
            || !control.generation_matches(generation)
            || control.publish_state() != PUBLISH_AUTHORIZED
        {
            drop(prepared_uds);
            let terminal = session.finish(HELPER_CODE_STOP_REQUESTED, 0);
            terminal.write(ipc)?;
            return Ok(());
        }
        let (listener, uds_entry) = match publish_helper_uds(&control, generation, prepared_uds) {
            Ok(published) => published,
            Err(error) => {
                let terminal = session.finish(
                    HELPER_CODE_BIND_UDS_FAILED,
                    helper_errno(&error, nix::libc::EADDRINUSE),
                );
                terminal.write(ipc)?;
                return Ok(());
            }
        };
        session.listener = Some(listener);
        session.uds = Some(uds_entry);
        let mut ready = IpcRecord::new(IpcKind::Ready, generation);
        ready.set_identities(
            shm_identity,
            session
                .uds
                .as_ref()
                .expect("Ready 前必须持有公共 UDS 身份")
                .identity(),
        );
        ready.write(ipc)?;
        let (code, errno) = helper_relay_loop(&mut session);
        let terminal = session.finish(code, errno);
        terminal.write(ipc)
    }

    fn helper_init_shm(session: &mut HelperSession, anchor: PathAnchor) -> io::Result<()> {
        anchor.directory.revalidate_public_path()?;
        if let Some(metadata) = anchor.metadata()? {
            if !metadata.is_file() {
                return Err(io::Error::new(
                    io::ErrorKind::AlreadyExists,
                    format!(
                        "拒绝替换非普通文件视频共享内存: {}",
                        anchor.public_path.display()
                    ),
                ));
            }
            // 父进程已经取得路径对的排他租约；能进入 helper 说明没有活动 owner。
            // 按 inode 身份隔离删除上次异常退出留下的普通 SHM，禁止直接覆盖。
            let identity_file = anchor.directory.open_at(
                &anchor.name,
                nix::libc::O_PATH | nix::libc::O_NOFOLLOW | nix::libc::O_CLOEXEC,
                None,
            )?;
            let stale = OwnedEntry::from_metadata(
                anchor.clone(),
                anchor.name.clone(),
                &metadata,
                identity_file,
                EntryRole::Shm,
            )?;
            let cleanup = stale.cleanup();
            if !matches!(
                cleanup,
                VideoRelayPathCleanup::Removed | VideoRelayPathCleanup::AlreadyAbsent
            ) {
                return Err(io::Error::other(format!(
                    "清理失效视频共享内存未完成: {cleanup:?}"
                )));
            }
        }
        let watchdog_cleanup = session.watchdog_cleanup.clone();
        let mut watchdog = lock_unpoisoned(&watchdog_cleanup);
        if watchdog.parent_gone {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 父进程已退出，拒绝创建共享内存",
            ));
        }
        let candidate_name = reserve_cleanup_name(&anchor.directory, EntryRole::Shm)?;
        let file = anchor.directory.create_new(&candidate_name)?;
        let mut unpublished = UnpublishedEntry::new(
            anchor.directory.clone(),
            candidate_name.clone(),
            anchor.public_path.clone(),
            file,
        );
        if session.control.take_test_fault(FAULT_SHM_METADATA) {
            return Err(io::Error::other("注入 helper 共享内存 metadata 失败"));
        }
        let metadata = unpublished.file().metadata()?;
        if session.control.take_test_fault(FAULT_SHM_TRY_CLONE) {
            return Err(io::Error::other("注入 helper 共享内存 try_clone 失败"));
        }
        let identity_file = unpublished.file().try_clone()?;
        if session.control.take_test_fault(FAULT_SHM_IDENTITY) {
            return Err(io::Error::other("注入 helper 共享内存身份建立失败"));
        }
        let candidate_anchor = PathAnchor {
            directory: anchor.directory.clone(),
            name: candidate_name.clone(),
            public_path: anchor.public_path.clone(),
        };
        let owned = OwnedEntry::from_metadata(
            candidate_anchor,
            candidate_name.clone(),
            &metadata,
            identity_file,
            EntryRole::Shm,
        )?;
        if session.control.take_test_fault(FAULT_SHM_PERMISSIONS) {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "注入 helper 共享内存权限设置失败",
            ));
        }
        unpublished
            .file()
            .set_permissions(std::fs::Permissions::from_mode(0o660))?;
        unpublished.file().set_len(BUFFER_SIZE as u64)?;
        let mut mmap = unsafe { MmapMut::map_mut(unpublished.file())? };
        mmap[..HEADER_SIZE].fill(0);
        mmap[0..8].copy_from_slice(&SHM_MAGIC.to_le_bytes());
        mmap[8..12].copy_from_slice(&SHM_VERSION.to_le_bytes());
        mmap[44..60].copy_from_slice(&session.generation.raw_bytes());
        mmap[60..64].copy_from_slice(&0_u32.to_le_bytes());
        if session.control.take_test_fault(FAULT_STARTUP_HEADER_FLUSH) {
            return Err(io::Error::other("注入 helper 共享内存初始化刷新失败"));
        }
        mmap.flush_range(0, HEADER_SIZE)?;
        anchor.directory.revalidate_public_path()?;
        if session
            .control
            .take_test_fault(FAULT_SHM_PUBLISH_REPLACEMENT)
        {
            std::fs::write(&anchor.public_path, b"replacement-before-shm-publish")?;
        }
        anchor
            .directory
            .rename_noreplace(&candidate_name, &anchor.name)?;
        session.shm = Some(owned.retarget(anchor.clone(), anchor.name.clone()));
        let watchdog_entry = match session.shm.as_ref().unwrap().try_clone_exact() {
            Ok(entry) => entry,
            Err(error) => {
                let _ = session.shm.take().unwrap().cleanup();
                return Err(error);
            }
        };
        watchdog.shm = Some(watchdog_entry);
        unpublished.disarm();
        drop(watchdog);
        anchor.directory.sync()?;
        session.mmap = Some(mmap);
        Ok(())
    }

    fn prepare_helper_uds(
        control: &ControlPage,
        generation: Generation,
        public_anchor: PathAnchor,
        watchdog_cleanup: Arc<Mutex<WatchdogCleanupState>>,
        cleanup_complete: Arc<AtomicBool>,
    ) -> io::Result<PreparedUds> {
        if control.take_test_fault(FAULT_BIND_UDS) {
            return Err(io::Error::new(
                io::ErrorKind::AddrNotAvailable,
                "注入 helper UDS 绑定失败",
            ));
        }
        public_anchor.directory.revalidate_public_path()?;
        if public_anchor.metadata()?.is_some() {
            return Err(io::Error::new(
                io::ErrorKind::AlreadyExists,
                format!(
                    "拒绝删除或替换既有视频 UDS: {}",
                    public_anchor.public_path.display()
                ),
            ));
        }
        let candidate_anchor = helper_candidate_anchor(&public_anchor, generation);
        if candidate_anchor.metadata()?.is_some() {
            return Err(io::Error::new(
                io::ErrorKind::AlreadyExists,
                format!(
                    "拒绝删除或替换既有视频 UDS candidate: {}",
                    candidate_anchor.public_path.display()
                ),
            ));
        }

        if lock_unpoisoned(&watchdog_cleanup).parent_gone {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 父进程已退出，拒绝绑定 UDS candidate",
            ));
        }
        let listener = UnixListener::bind(
            candidate_anchor
                .directory
                .child_path(&candidate_anchor.name),
        )?;
        cleanup_complete.store(false, Ordering::Release);
        let mut prepared = PreparedUds {
            listener: Some(listener),
            observed_candidate: None,
            candidate: None,
            public: None,
            public_anchor,
            watchdog_cleanup: watchdog_cleanup.clone(),
            cleanup_complete,
        };
        let identity_file = candidate_anchor.directory.open_at(
            &candidate_anchor.name,
            nix::libc::O_PATH | nix::libc::O_NOFOLLOW | nix::libc::O_CLOEXEC,
            None,
        )?;
        let metadata = identity_file.metadata()?;
        if !metadata.file_type().is_socket() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "视频 UDS candidate 不是套接字",
            ));
        }
        let observation = ObservedEntry::from_metadata(
            candidate_anchor,
            helper_candidate_name(generation),
            &metadata,
            identity_file,
            EntryRole::Uds,
        )?;
        prepared.observed_candidate = Some(observation);
        prepared
            .listener
            .as_ref()
            .expect("candidate bind 后必须持有 listener")
            .set_nonblocking(true)?;
        let probe = UnixStream::connect(
            prepared
                .public_anchor
                .directory
                .child_path(&helper_candidate_name(generation)),
        )?;
        let accepted = prepared
            .listener
            .as_ref()
            .expect("candidate 自连接时必须持有 listener")
            .accept()
            .map_err(|error| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("视频 UDS candidate 未连接到本 helper listener: {error}"),
                )
            })?;
        ensure_self_peer(&accepted.0)?;
        drop(accepted);
        drop(probe);
        prepared
            .observed_candidate
            .as_ref()
            .expect("candidate 自连接后必须持有观察身份")
            .reopen_identity()?;
        let candidate = prepared
            .observed_candidate
            .take()
            .expect("candidate 身份证明后必须持有观察身份")
            .into_owned();
        prepared.candidate = Some(candidate);
        prepared.cleanup_complete.store(true, Ordering::Release);
        let mut watchdog = lock_unpoisoned(&watchdog_cleanup);
        if watchdog.parent_gone {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 父进程已退出，拒绝登记 UDS candidate",
            ));
        }
        let watchdog_candidate = prepared
            .candidate
            .as_ref()
            .expect("candidate 身份证明后必须持有清理权")
            .try_clone_exact()?;
        watchdog.uds_candidate = Some(watchdog_candidate);
        drop(watchdog);
        Ok(prepared)
    }

    fn publish_helper_uds(
        control: &ControlPage,
        expected_generation: Generation,
        mut prepared: PreparedUds,
    ) -> io::Result<(UnixListener, OwnedEntry)> {
        ensure_uds_publish_allowed(control, expected_generation)?;
        prepared.public_anchor.directory.revalidate_public_path()?;
        prepared
            .candidate
            .as_ref()
            .expect("PreparedUds 必须持有 candidate 身份")
            .reopen_identity()?;
        if !helper_test_pause(control, FAULT_PAUSE_BEFORE_UDS_BIND, PHASE_BEFORE_UDS_BIND) {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 在 UDS link publish 前被停止",
            ));
        }
        ensure_uds_publish_allowed(control, expected_generation)?;
        prepared.public_anchor.directory.revalidate_public_path()?;
        let public_entry = prepared
            .candidate
            .as_ref()
            .expect("link publish 前必须持有 candidate 身份")
            .try_clone_exact()?
            .retarget(
                prepared.public_anchor.clone(),
                prepared.public_anchor.name.clone(),
            );
        let watchdog_public = public_entry.try_clone_exact()?;
        prepared.public = Some(public_entry);
        prepared
            .candidate
            .as_ref()
            .expect("link publish 前必须持有 candidate 身份")
            .reopen_identity()?;
        if !helper_test_pause(
            control,
            FAULT_PAUSE_AFTER_UDS_BIND_BEFORE_IDENTITY,
            PHASE_AFTER_UDS_BIND_BEFORE_IDENTITY,
        ) {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 在 UDS 固定身份 link publish 前被停止",
            ));
        }
        ensure_uds_publish_allowed(control, expected_generation)?;
        prepared.public_anchor.directory.revalidate_public_path()?;
        let watchdog_cleanup = prepared.watchdog_cleanup.clone();
        let mut watchdog = lock_unpoisoned(&watchdog_cleanup);
        if watchdog.parent_gone {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 父进程已退出，拒绝 link publish UDS",
            ));
        }
        prepared.public_anchor.directory.link_identity_noreplace(
            &prepared
                .candidate
                .as_ref()
                .expect("link publish 时必须持有 candidate 身份")
                .identity_file,
            &prepared.public_anchor.name,
        )?;
        watchdog.uds_public = Some(watchdog_public);
        drop(watchdog);
        prepared
            .public
            .as_ref()
            .expect("link publish 后必须持有 public 身份")
            .reopen_identity()?;

        let candidate_cleanup = prepared
            .candidate
            .take()
            .expect("link publish 后必须持有 candidate 身份")
            .cleanup();
        if !matches!(
            candidate_cleanup,
            VideoRelayPathCleanup::Removed | VideoRelayPathCleanup::AlreadyAbsent
        ) {
            prepared.cleanup_complete.store(false, Ordering::Release);
            return Err(io::Error::other(format!(
                "视频 UDS candidate 清理未完成: {candidate_cleanup:?}"
            )));
        }

        let probe = UnixStream::connect(
            prepared
                .public_anchor
                .directory
                .child_path(&prepared.public_anchor.name),
        )?;
        let accepted = prepared
            .listener
            .as_ref()
            .expect("public 自连接时必须持有 listener")
            .accept()
            .map_err(|error| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("视频 UDS public 未连接到本 helper listener: {error}"),
                )
            })?;
        ensure_self_peer(&accepted.0)?;
        drop(accepted);
        drop(probe);
        prepared
            .public
            .as_ref()
            .expect("public 自连接后必须持有身份")
            .reopen_identity()?;
        if !helper_test_pause(control, FAULT_PAUSE_AFTER_UDS_BIND, PHASE_AFTER_UDS_BIND) {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 在 UDS link publish 后被停止",
            ));
        }
        ensure_uds_publish_allowed(control, expected_generation)?;
        prepared
            .public
            .as_ref()
            .expect("PreparedUds 必须持有公开身份")
            .reopen_identity()?;
        if !helper_test_pause(
            control,
            FAULT_PAUSE_AFTER_UDS_IDENTITY_BEFORE_PUBLISH,
            PHASE_AFTER_UDS_IDENTITY_BEFORE_PUBLISH,
        ) {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 在 UDS 最终身份确认后、公开前被停止",
            ));
        }
        prepared.public_anchor.directory.sync()?;
        ensure_uds_publish_allowed(control, expected_generation)?;
        prepared
            .public
            .as_ref()
            .expect("PreparedUds 最终公开前必须持有身份")
            .reopen_identity()?;
        if !control.complete_publish(expected_generation) {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 UDS 公开在线性化点被停止或错代取消",
            ));
        }
        let listener = prepared
            .listener
            .take()
            .expect("PreparedUds 必须持有 listener");
        let public_entry = prepared
            .public
            .take()
            .expect("PreparedUds 必须持有公开身份");
        Ok((listener, public_entry))
    }

    fn ensure_self_peer(stream: &UnixStream) -> io::Result<()> {
        let mut credentials: nix::libc::ucred = unsafe { std::mem::zeroed() };
        let mut length = std::mem::size_of::<nix::libc::ucred>() as nix::libc::socklen_t;
        let result = unsafe {
            nix::libc::getsockopt(
                stream.as_raw_fd(),
                nix::libc::SOL_SOCKET,
                nix::libc::SO_PEERCRED,
                std::ptr::addr_of_mut!(credentials).cast(),
                &mut length,
            )
        };
        if result != 0 {
            return Err(io::Error::last_os_error());
        }
        if length as usize != std::mem::size_of::<nix::libc::ucred>()
            || credentials.pid != unsafe { nix::libc::getpid() }
        {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "视频 UDS 自连接不是本 helper 进程",
            ));
        }
        Ok(())
    }

    fn ensure_uds_publish_allowed(
        control: &ControlPage,
        expected_generation: Generation,
    ) -> io::Result<()> {
        if control.stop_requested()
            || !control.generation_matches(expected_generation)
            || control.publish_state() != PUBLISH_AUTHORIZED
        {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 UDS 公开已被停止、错代或取消授权",
            ));
        }
        Ok(())
    }

    fn helper_relay_loop(session: &mut HelperSession) -> (u32, i32) {
        let mut committed_sequence = 0_u64;
        while !session.control.stop_requested() {
            let accepted = session.listener.as_ref().unwrap().accept();
            match accepted {
                Ok((mut stream, _)) => {
                    if let Err(error) = stream.set_read_timeout(Some(CONNECTION_READ_TIMEOUT)) {
                        return (
                            HELPER_CODE_COMMIT_FAILED,
                            helper_errno(&error, nix::libc::EIO),
                        );
                    }
                    match helper_handle_connection(&mut stream, session, &mut committed_sequence) {
                        Ok(()) => {}
                        Err(error) if error.kind() == io::ErrorKind::InvalidData => {
                            error!("视频 helper 客户端协议错误: {}", error);
                        }
                        Err(error)
                            if matches!(
                                error.kind(),
                                io::ErrorKind::UnexpectedEof
                                    | io::ErrorKind::ConnectionReset
                                    | io::ErrorKind::BrokenPipe
                            ) => {}
                        Err(error) if error.kind() == io::ErrorKind::Interrupted => break,
                        Err(error) => {
                            return (
                                HELPER_CODE_COMMIT_FAILED,
                                helper_errno(&error, nix::libc::EIO),
                            );
                        }
                    }
                }
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                    thread::sleep(ACCEPT_POLL_INTERVAL);
                }
                Err(error) => {
                    return (
                        HELPER_CODE_COMMIT_FAILED,
                        helper_errno(&error, nix::libc::EIO),
                    );
                }
            }
        }
        let code = if unsafe { nix::libc::getppid() } == 1 {
            HELPER_CODE_PARENT_GONE
        } else {
            HELPER_CODE_STOP_REQUESTED
        };
        (code, 0)
    }

    fn helper_handle_connection(
        stream: &mut UnixStream,
        session: &mut HelperSession,
        committed_sequence: &mut u64,
    ) -> io::Result<()> {
        let mut length_bytes = [0_u8; 4];
        // 单个连接串行提交帧，复用 payload 容量，避免每帧重新分配和释放。
        let mut payload = Vec::new();
        loop {
            helper_read_exact(stream, &mut length_bytes, &session.control)?;
            let frame_length = u32::from_le_bytes(length_bytes) as usize;
            if frame_length == 0 || frame_length > MAX_FRAME_SIZE {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("视频帧长度无效: {frame_length}"),
                ));
            }
            payload.resize(frame_length, 0);
            #[cfg(test)]
            record_payload_buffer_for_test(&session.control, payload.as_ptr(), payload.capacity());
            helper_read_exact(stream, &mut payload, &session.control)?;
            helper_commit_frame(session, &payload, committed_sequence)?;
        }
    }

    fn helper_read_exact(
        stream: &mut UnixStream,
        mut buffer: &mut [u8],
        control: &ControlPage,
    ) -> io::Result<()> {
        while !buffer.is_empty() {
            if control.stop_requested() {
                return Err(io::Error::new(
                    io::ErrorKind::Interrupted,
                    "视频 helper 读取被停止",
                ));
            }
            match stream.read(buffer) {
                Ok(0) => {
                    return Err(io::Error::new(
                        io::ErrorKind::UnexpectedEof,
                        "视频客户端提前关闭",
                    ));
                }
                Ok(read) => buffer = &mut buffer[read..],
                Err(error)
                    if matches!(
                        error.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) => {}
                Err(error) => return Err(error),
            }
        }
        Ok(())
    }

    fn helper_commit_frame(
        session: &mut HelperSession,
        payload: &[u8],
        committed_sequence: &mut u64,
    ) -> io::Result<()> {
        if !helper_test_pause(
            &session.control,
            FAULT_PAUSE_BEFORE_INVALIDATE,
            PHASE_BEFORE_INVALIDATE,
        ) || session.control.stop_requested()
        {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 在帧失效前停止",
            ));
        }
        let mmap = session.mmap.as_mut().unwrap();
        store_header_sequence(mmap, 0);
        session.control.set_published_sequence(0);
        fence(Ordering::Release);
        if !helper_test_pause(
            &session.control,
            FAULT_PAUSE_AFTER_INVALIDATE,
            PHASE_AFTER_INVALIDATE,
        ) || session.control.stop_requested()
        {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 在帧失效后停止",
            ));
        }
        let next_sequence = committed_sequence.wrapping_add(1);
        let active_buffer = (next_sequence % 2) as u32;
        let buffer_offset = HEADER_SIZE + active_buffer as usize * MAX_FRAME_SIZE;
        mmap[buffer_offset..buffer_offset + payload.len()].copy_from_slice(payload);
        let timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        mmap[24..28].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        mmap[32..40].copy_from_slice(&timestamp_ns.to_le_bytes());
        mmap[40..44].copy_from_slice(&active_buffer.to_le_bytes());
        if session.control.take_test_fault(FAULT_MSYNC_HANG) {
            session.control.set_helper_phase(PHASE_MSYNC_HANG);
            unsafe {
                nix::libc::signal(nix::libc::SIGTERM, nix::libc::SIG_IGN);
            }
            loop {
                thread::sleep(Duration::from_secs(60));
            }
        }
        if session.control.take_test_fault(FAULT_COMMIT_FLUSH) {
            return Err(io::Error::other("注入 helper 帧 MS_SYNC 失败"));
        }
        mmap.flush_range(0, buffer_offset + payload.len())?;
        if !helper_test_pause(&session.control, FAULT_PAUSE_AFTER_FLUSH, PHASE_AFTER_FLUSH)
            || session.control.stop_requested()
            || !session.control.generation_matches(session.generation)
        {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "视频 helper 刷新后发布前停止或错代",
            ));
        }
        fence(Ordering::Release);
        store_header_sequence(mmap, next_sequence);
        *committed_sequence = next_sequence;
        session.control.set_published_sequence(next_sequence);
        Ok(())
    }

    fn helper_test_pause(control: &ControlPage, fault: u64, phase: u32) -> bool {
        if !control.take_test_fault(fault) {
            return true;
        }
        control.set_helper_phase(phase);
        while !control.stop_requested() {
            if control.take_test_resume() {
                control.set_helper_phase(PHASE_IDLE);
                return true;
            }
            thread::sleep(Duration::from_millis(1));
        }
        false
    }

    fn helper_cleanup_pause(control: &ControlPage, fault: u64, phase: u32) -> bool {
        if !control.take_test_fault(fault) {
            return true;
        }
        control.set_helper_phase(phase);
        while !control.take_test_resume() {
            thread::sleep(Duration::from_millis(1));
        }
        control.set_helper_phase(PHASE_IDLE);
        true
    }

    fn store_header_sequence(mmap: &MmapMut, value: u64) {
        debug_assert_eq!(16 % std::mem::align_of::<AtomicU64>(), 0);
        let sequence = unsafe { &*mmap.as_ptr().add(16).cast::<AtomicU64>() };
        sequence.store(value, Ordering::Release);
    }

    struct ParentLedger {
        shm: Option<OwnedEntry>,
        uds: Option<OwnedEntry>,
        initial_shm: Option<ObservedIdentity>,
        initial_uds: Option<ObservedIdentity>,
        initial_uds_candidate: Option<ObservedIdentity>,
    }

    struct ObservedIdentity {
        file: File,
        device: u64,
        inode: u64,
    }

    impl ObservedIdentity {
        fn matches(&self, metadata: &std::fs::Metadata) -> bool {
            self.file.metadata().is_ok_and(|identity| {
                identity.dev() == self.device
                    && identity.ino() == self.inode
                    && metadata.dev() == identity.dev()
                    && metadata.ino() == identity.ino()
            })
        }
    }

    struct ParentResources {
        generation: Generation,
        child: Child,
        pidfd: Option<File>,
        ipc: UnixStream,
        leases: PairLeases,
        ledger: ParentLedger,
    }

    struct SupervisorShared {
        control: Arc<ControlPage>,
        sequence: Arc<AtomicU64>,
        notify: Arc<Notify>,
        terminal_signal: Arc<TerminalSignal>,
        requested_deadline: Mutex<Option<Instant>>,
        wake: Condvar,
        helper_exited: AtomicBool,
    }

    impl SupervisorShared {
        fn request_stop(&self, deadline: Instant) -> bool {
            let first = self.control.request_stop();
            let mut requested = lock_unpoisoned(&self.requested_deadline);
            *requested = Some(requested.map_or(deadline, |current| current.min(deadline)));
            drop(requested);
            self.wake.notify_all();
            first
        }

        fn deadline(&self) -> Option<Instant> {
            *lock_unpoisoned(&self.requested_deadline)
        }
    }

    pub(super) struct ProcessOwner {
        shared: Arc<SupervisorShared>,
        supervisor: Option<JoinHandle<()>>,
        helper_pid: u32,
        helper_pidfd: File,
        helper_ipc: UnixStream,
    }

    impl ProcessOwner {
        pub(super) fn launch(
            shm_path: &Path,
            uds_path: &Path,
            sequence: Arc<AtomicU64>,
            notify: Arc<Notify>,
            terminal_signal: Arc<TerminalSignal>,
            test_faults: u64,
        ) -> Result<Self, VideoRelayStartError> {
            let mut leases = PairLeases::acquire(shm_path, uds_path)?;
            let generation = Generation::fresh();
            let control = ControlPage::create(generation, test_faults).map_err(|error| {
                start_error_with_leases(
                    &mut leases,
                    VideoRelayFailure::from_error("创建视频 helper control page", &error),
                )
            })?;
            #[cfg(test)]
            register_helper_control(shm_path, &control);
            let initial_shm = snapshot_path_identity(&leases.shm_anchor).map_err(|error| {
                start_error_with_leases(
                    &mut leases,
                    VideoRelayFailure::from_error("记录视频 SHM 初始身份", &error),
                )
            })?;
            let initial_uds = snapshot_path_identity(&leases.uds_anchor).map_err(|error| {
                start_error_with_leases(
                    &mut leases,
                    VideoRelayFailure::from_error("记录视频 UDS 初始身份", &error),
                )
            })?;
            let uds_candidate_anchor = helper_candidate_anchor(&leases.uds_anchor, generation);
            let initial_uds_candidate =
                snapshot_path_identity(&uds_candidate_anchor).map_err(|error| {
                    start_error_with_leases(
                        &mut leases,
                        VideoRelayFailure::from_error("记录视频 UDS candidate 初始身份", &error),
                    )
                })?;
            let (parent_ipc, child_ipc_source) = UnixStream::pair().map_err(|error| {
                start_error_with_leases(
                    &mut leases,
                    VideoRelayFailure::from_error("创建视频 helper IPC", &error),
                )
            })?;
            let child_ipc =
                duplicate_inheritable(child_ipc_source.as_raw_fd()).map_err(|error| {
                    start_error_with_leases(
                        &mut leases,
                        VideoRelayFailure::from_error("继承视频 helper IPC", &error),
                    )
                })?;
            drop(child_ipc_source);
            let child_control =
                duplicate_inheritable(control.file_descriptor()).map_err(|error| {
                    start_error_with_leases(
                        &mut leases,
                        VideoRelayFailure::from_error("继承视频 helper control page", &error),
                    )
                })?;
            let child_shm_directory = duplicate_inheritable(
                leases.shm_anchor.directory.file.as_raw_fd(),
            )
            .map_err(|error| {
                start_error_with_leases(
                    &mut leases,
                    VideoRelayFailure::from_error("继承视频 SHM 稳定目录", &error),
                )
            })?;
            let child_uds_directory = duplicate_inheritable(
                leases.uds_anchor.directory.file.as_raw_fd(),
            )
            .map_err(|error| {
                start_error_with_leases(
                    &mut leases,
                    VideoRelayFailure::from_error("继承视频 UDS 稳定目录", &error),
                )
            })?;
            let child_shm_lease = leases
                .shm_lease
                .as_ref()
                .expect("已取得的 SHM lease 必须保有身份")
                .reopen_identity()
                .map_err(|error| {
                    start_error_with_leases(
                        &mut leases,
                        VideoRelayFailure::from_error("继承视频 SHM lease 身份", &error),
                    )
                })?;
            let child_uds_lease = leases
                .uds_lease
                .as_ref()
                .expect("已取得的 UDS lease 必须保有身份")
                .reopen_identity()
                .map_err(|error| {
                    start_error_with_leases(
                        &mut leases,
                        VideoRelayFailure::from_error("继承视频 UDS lease 身份", &error),
                    )
                })?;
            let executable = std::env::current_exe().map_err(|error| {
                start_error_with_leases(
                    &mut leases,
                    VideoRelayFailure::from_error("解析视频 helper 可执行文件", &error),
                )
            })?;
            let mut command = Command::new(executable);
            configure_helper_command(&mut command);
            configure_helper_fd_allowlist(
                &mut command,
                [
                    child_ipc.as_raw_fd(),
                    child_control.as_raw_fd(),
                    child_shm_directory.as_raw_fd(),
                    child_uds_directory.as_raw_fd(),
                    child_shm_lease.as_raw_fd(),
                    child_uds_lease.as_raw_fd(),
                ],
            );
            command
                .env(HELPER_REQUEST_ENV, "1")
                .env(HELPER_IPC_FD_ENV, child_ipc.as_raw_fd().to_string())
                .env(HELPER_CONTROL_FD_ENV, child_control.as_raw_fd().to_string())
                .env(
                    HELPER_SHM_DIR_FD_ENV,
                    child_shm_directory.as_raw_fd().to_string(),
                )
                .env(
                    HELPER_UDS_DIR_FD_ENV,
                    child_uds_directory.as_raw_fd().to_string(),
                )
                .env(
                    HELPER_SHM_LEASE_FD_ENV,
                    child_shm_lease.as_raw_fd().to_string(),
                )
                .env(
                    HELPER_UDS_LEASE_FD_ENV,
                    child_uds_lease.as_raw_fd().to_string(),
                )
                .env(HELPER_PARENT_PID_ENV, std::process::id().to_string())
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null());
            let child = command.spawn().map_err(|error| {
                start_error_with_leases(
                    &mut leases,
                    VideoRelayFailure::from_error("创建视频 helper 进程", &error),
                )
            })?;
            drop(child_ipc);
            drop(child_control);
            drop(child_shm_directory);
            drop(child_uds_directory);
            drop(child_shm_lease);
            drop(child_uds_lease);
            let pidfd = match open_pidfd(child.id()) {
                Ok(pidfd) => pidfd,
                Err(error) => {
                    let mut resources = ParentResources {
                        generation,
                        child,
                        pidfd: None,
                        ipc: parent_ipc,
                        leases,
                        ledger: ParentLedger {
                            shm: None,
                            uds: None,
                            initial_shm,
                            initial_uds,
                            initial_uds_candidate,
                        },
                    };
                    return Err(abort_start(
                        &control,
                        &mut resources,
                        VideoRelayFailure::from_error("打开视频 helper pidfd", &error),
                        Instant::now() + STARTUP_READY_TIMEOUT,
                    ));
                }
            };
            let mut resources = ParentResources {
                generation,
                child,
                pidfd: Some(pidfd),
                ipc: parent_ipc,
                leases,
                ledger: ParentLedger {
                    shm: None,
                    uds: None,
                    initial_shm,
                    initial_uds,
                    initial_uds_candidate,
                },
            };
            let startup_deadline = Instant::now() + STARTUP_READY_TIMEOUT;
            let mut start = IpcRecord::new(IpcKind::Start, generation);
            if let Err(error) =
                start.set_config(&resources.leases.shm_anchor, &resources.leases.uds_anchor)
            {
                return Err(abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::from_error("编码视频 helper Start", &error),
                    startup_deadline,
                ));
            }
            if let Err(error) = start.write(&mut resources.ipc) {
                return Err(abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::from_error("发送视频 helper Start", &error),
                    startup_deadline,
                ));
            }
            let prepared = match read_record_until(&mut resources.ipc, startup_deadline) {
                Ok(record) => record,
                Err(error) => {
                    return Err(abort_start(
                        &control,
                        &mut resources,
                        VideoRelayFailure::from_error("等待视频 helper Prepared", &error),
                        startup_deadline,
                    ));
                }
            };
            if prepared.kind().ok() == Some(IpcKind::Terminal) {
                adopt_terminal_identities(&mut resources, &prepared);
                return Err(abort_start(
                    &control,
                    &mut resources,
                    failure_from_helper_record("视频 helper 准备失败", &prepared),
                    startup_deadline,
                ));
            }
            if prepared.kind().ok() != Some(IpcKind::Prepared)
                || prepared.generation() != generation
            {
                return Err(abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::new(
                        "验证视频 helper Prepared",
                        io::ErrorKind::InvalidData,
                        "Prepared 类型或 generation 错误",
                    ),
                    startup_deadline,
                ));
            }
            let (shm_identity, prepared_uds_identity) = prepared.identities();
            if prepared_uds_identity != (0, 0) {
                return Err(abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::new(
                        "验证视频 helper Prepared",
                        io::ErrorKind::InvalidData,
                        "Prepared 不得在授权前声称公共 UDS 身份",
                    ),
                    startup_deadline,
                ));
            }
            resources.ledger.shm = match OwnedEntry::open_expected(
                resources.leases.shm_anchor.clone(),
                shm_identity.0,
                shm_identity.1,
                EntryRole::Shm,
            ) {
                Ok(entry) => Some(entry),
                Err(error) => {
                    return Err(abort_start(
                        &control,
                        &mut resources,
                        VideoRelayFailure::from_error("采纳视频 helper SHM 身份", &error),
                        startup_deadline,
                    ));
                }
            };
            if let Err(error) = resources
                .leases
                .shm_anchor
                .directory
                .revalidate_public_path()
                .and_then(|_| {
                    resources
                        .leases
                        .uds_anchor
                        .directory
                        .revalidate_public_path()
                })
            {
                return Err(abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::from_error("复验视频 helper 公共父路径", &error),
                    startup_deadline,
                ));
            }
            if !control.authorize_publish() {
                return Err(abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::new(
                        "授权视频 helper 公开",
                        io::ErrorKind::Interrupted,
                        "视频 helper 公开授权已被停止取消",
                    ),
                    startup_deadline,
                ));
            }
            let ready = match read_record_until(&mut resources.ipc, startup_deadline) {
                Ok(record) => record,
                Err(error) => {
                    return Err(abort_start(
                        &control,
                        &mut resources,
                        VideoRelayFailure::from_error("等待视频 helper Ready", &error),
                        startup_deadline,
                    ));
                }
            };
            if ready.kind().ok() == Some(IpcKind::Terminal) {
                adopt_terminal_identities(&mut resources, &ready);
                return Err(abort_start(
                    &control,
                    &mut resources,
                    failure_from_helper_record("视频 helper 公开失败", &ready),
                    startup_deadline,
                ));
            }
            if ready.kind().ok() != Some(IpcKind::Ready)
                || ready.generation() != generation
                || control.publish_state() != PUBLISH_COMPLETE
            {
                return Err(abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::new(
                        "验证视频 helper Ready",
                        io::ErrorKind::InvalidData,
                        "Ready 类型、generation、身份或 Published 状态错误",
                    ),
                    startup_deadline,
                ));
            }
            let (ready_shm_identity, uds_identity) = ready.identities();
            if ready_shm_identity != shm_identity || uds_identity == (0, 0) {
                return Err(abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::new(
                        "验证视频 helper Ready",
                        io::ErrorKind::InvalidData,
                        "Ready SHM 身份不一致或 UDS 身份为空",
                    ),
                    startup_deadline,
                ));
            }
            resources.ledger.uds = match OwnedEntry::open_expected(
                resources.leases.uds_anchor.clone(),
                uds_identity.0,
                uds_identity.1,
                EntryRole::Uds,
            ) {
                Ok(entry) => Some(entry),
                Err(error) => {
                    return Err(abort_start(
                        &control,
                        &mut resources,
                        VideoRelayFailure::from_error("采纳视频 helper UDS 身份", &error),
                        startup_deadline,
                    ));
                }
            };
            let helper_pid = resources.child.id();
            let helper_pidfd = resources
                .pidfd
                .as_ref()
                .expect("成功启动的 helper 必须持 pidfd")
                .try_clone()
                .map_err(|error| {
                    abort_start(
                        &control,
                        &mut resources,
                        VideoRelayFailure::from_error("复制视频 helper pidfd", &error),
                        startup_deadline,
                    )
                })?;
            let helper_ipc = resources.ipc.try_clone().map_err(|error| {
                abort_start(
                    &control,
                    &mut resources,
                    VideoRelayFailure::from_error("复制视频 helper IPC", &error),
                    startup_deadline,
                )
            })?;
            let shared = Arc::new(SupervisorShared {
                control,
                sequence,
                notify,
                terminal_signal,
                requested_deadline: Mutex::new(None),
                wake: Condvar::new(),
                helper_exited: AtomicBool::new(false),
            });
            let resources_slot = Arc::new(Mutex::new(Some(resources)));
            let spawn_result = if shared.control.take_test_fault(FAULT_SUPERVISOR_SPAWN) {
                Err(io::Error::other("注入视频 helper supervisor 创建失败"))
            } else {
                let shared_for_thread = shared.clone();
                let resources_for_thread = resources_slot.clone();
                thread::Builder::new()
                    .name("nuwax-video-supervisor".to_string())
                    .spawn(move || {
                        let resources = lock_unpoisoned(&resources_for_thread)
                            .take()
                            .expect("视频 helper 资源必须仅交接给 supervisor 一次");
                        supervise_helper(shared_for_thread, resources);
                    })
            };
            let supervisor = match spawn_result {
                Ok(supervisor) => supervisor,
                Err(error) => {
                    let primary =
                        VideoRelayFailure::from_error("创建视频 helper supervisor", &error);
                    let mut resources = lock_unpoisoned(&resources_slot)
                        .take()
                        .expect("supervisor 创建失败时父侧必须保有 helper 资源");
                    let mut failure = abort_start(
                        &shared.control,
                        &mut resources,
                        primary,
                        Instant::now() + DEFAULT_STOP_TIMEOUT,
                    );
                    failure.thread_started = false;
                    return Err(failure);
                }
            };
            Ok(Self {
                shared,
                supervisor: Some(supervisor),
                helper_pid,
                helper_pidfd,
                helper_ipc,
            })
        }

        pub(super) fn request_stop(&self, deadline: Instant) -> bool {
            self.shared.request_stop(deadline)
        }

        #[cfg(test)]
        pub(super) fn try_terminal(&self) -> Option<VideoRelayTerminal> {
            self.shared.terminal_signal.try_get()
        }

        pub(super) fn is_finished(&self) -> bool {
            self.shared.helper_exited.load(Ordering::Acquire)
                || self.shared.terminal_signal.try_get().is_some()
        }

        pub(super) async fn wait_terminal(&self) -> VideoRelayTerminal {
            self.shared.terminal_signal.wait().await
        }

        pub(super) fn join_until(&mut self, deadline: Instant) -> io::Result<VideoRelayTerminal> {
            let remaining = deadline.saturating_duration_since(Instant::now());
            let terminal = self
                .shared
                .terminal_signal
                .wait_timeout(remaining)
                .ok_or_else(|| {
                    io::Error::new(io::ErrorKind::TimedOut, "等待视频 helper 终态超时")
                })?;
            if let Some(supervisor) = self.supervisor.take() {
                supervisor
                    .join()
                    .map_err(|_| io::Error::other("视频 helper supervisor 在线程边界外恐慌"))?;
            }
            Ok(terminal)
        }

        pub(super) fn shutdown(&mut self, timeout: Duration) -> io::Result<VideoRelayTerminal> {
            let deadline = Instant::now() + timeout;
            self.request_stop(deadline);
            self.join_until(deadline)
        }

        #[cfg(test)]
        pub(super) fn helper_pid(&self) -> u32 {
            self.helper_pid
        }

        #[cfg(test)]
        pub(super) fn inherited_fds_are_live(&self) -> bool {
            unsafe {
                nix::libc::fcntl(self.helper_pidfd.as_raw_fd(), nix::libc::F_GETFD) >= 0
                    && nix::libc::fcntl(self.helper_ipc.as_raw_fd(), nix::libc::F_GETFD) >= 0
            }
        }

        #[cfg(test)]
        pub(super) fn inherited_fd_numbers(&self) -> (RawFd, RawFd) {
            (self.helper_pidfd.as_raw_fd(), self.helper_ipc.as_raw_fd())
        }

        #[cfg(test)]
        pub(super) fn control(&self) -> Arc<ControlPage> {
            self.shared.control.clone()
        }

        #[cfg(test)]
        pub(super) fn terminal_signal(&self) -> Arc<TerminalSignal> {
            self.shared.terminal_signal.clone()
        }
    }

    impl Drop for ProcessOwner {
        fn drop(&mut self) {
            let Some(supervisor) = self.supervisor.take() else {
                return;
            };
            self.shared.request_stop(Instant::now() + DROP_TIMEOUT);
            let _ = self.shared.terminal_signal.wait_timeout(DROP_TIMEOUT);
            let _ = supervisor.join();
        }
    }

    fn configure_helper_command(command: &mut Command) {
        #[cfg(test)]
        {
            command
                .arg("--exact")
                .arg("video_relay::tests::requested_helper_child_entry")
                .arg("--nocapture")
                .arg("--test-threads=1");
        }
        #[cfg(not(test))]
        {
            command.arg("--nuwax-video-relay-helper-v1");
        }
    }

    fn configure_helper_fd_allowlist(command: &mut Command, inherited: [RawFd; 6]) {
        const CLOSE_RANGE_CLOEXEC: nix::libc::c_uint = 1 << 2;
        unsafe {
            command.pre_exec(move || {
                let result = nix::libc::syscall(
                    nix::libc::SYS_close_range,
                    3_u32,
                    u32::MAX,
                    CLOSE_RANGE_CLOEXEC,
                );
                if result != 0 {
                    let error = io::Error::last_os_error();
                    match error.raw_os_error() {
                        Some(nix::libc::EINVAL | nix::libc::ENOSYS) => {
                            // Jetson 旧内核可能没有 close_range，或不支持
                            // CLOSE_RANGE_CLOEXEC；逐个标记以保持相同的 exec 边界。
                            let mut limit: nix::libc::rlimit = std::mem::zeroed();
                            if nix::libc::getrlimit(nix::libc::RLIMIT_NOFILE, &mut limit) != 0 {
                                return Err(io::Error::last_os_error());
                            }
                            let upper = limit.rlim_cur.min(nix::libc::c_int::MAX as _);
                            let mut descriptor = 3;
                            while (descriptor as nix::libc::rlim_t) < upper {
                                let flags = nix::libc::fcntl(descriptor, nix::libc::F_GETFD);
                                if flags >= 0
                                    && nix::libc::fcntl(
                                        descriptor,
                                        nix::libc::F_SETFD,
                                        flags | nix::libc::FD_CLOEXEC,
                                    ) < 0
                                {
                                    return Err(io::Error::last_os_error());
                                }
                                descriptor += 1;
                            }
                        }
                        _ => return Err(error),
                    }
                }
                for descriptor in inherited {
                    if nix::libc::fcntl(descriptor, nix::libc::F_SETFD, 0) < 0 {
                        return Err(io::Error::last_os_error());
                    }
                }
                Ok(())
            });
        }
    }

    fn start_error_with_leases(
        leases: &mut PairLeases,
        primary: VideoRelayFailure,
    ) -> VideoRelayStartError {
        let (uds_lease, shm_lease) = leases.cleanup();
        VideoRelayStartError {
            primary: primary.clone(),
            terminal: Box::new(VideoRelayTerminal {
                reason: VideoRelayTerminalReason::Failed(primary),
                cleanup: VideoRelayCleanup {
                    mmap: VideoRelayMmapCleanup::NotCreated,
                    uds: VideoRelayPathCleanup::NotCreated,
                    shm: VideoRelayPathCleanup::NotCreated,
                    uds_lease,
                    shm_lease,
                },
            }),
            thread_started: false,
            thread_joined: true,
        }
    }

    fn snapshot_path_identity(anchor: &PathAnchor) -> io::Result<Option<ObservedIdentity>> {
        let file = match anchor.directory.open_at(
            &anchor.name,
            nix::libc::O_PATH | nix::libc::O_NOFOLLOW | nix::libc::O_CLOEXEC,
            None,
        ) {
            Ok(file) => file,
            Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(error),
        };
        let metadata = file.metadata()?;
        Ok(Some(ObservedIdentity {
            file,
            device: metadata.dev(),
            inode: metadata.ino(),
        }))
    }

    #[derive(Clone, Copy)]
    enum ParentWaitSite {
        Startup,
        Supervisor,
    }

    fn try_wait_helper(
        control: &ControlPage,
        resources: &mut ParentResources,
        site: ParentWaitSite,
    ) -> io::Result<Option<ExitStatus>> {
        let fault = match site {
            ParentWaitSite::Startup => FAULT_PARENT_TRY_WAIT_STARTUP,
            ParentWaitSite::Supervisor => FAULT_PARENT_TRY_WAIT_SUPERVISOR,
        };
        if control.take_test_fault(fault) {
            return Err(io::Error::other(match site {
                ParentWaitSite::Startup => "注入启动期视频 helper try_wait 失败",
                ParentWaitSite::Supervisor => "注入运行期视频 helper try_wait 失败",
            }));
        }
        resources.child.try_wait()
    }

    fn kill_after_unknown_wait(
        resources: &mut ParentResources,
        operation: &'static str,
        wait_error: &io::Error,
    ) -> VideoRelayTerminal {
        let signal_detail = signal_helper(resources, nix::libc::SIGKILL)
            .err()
            .map(|error| format!("; pidfd SIGKILL 也失败: {error}"))
            .unwrap_or_default();
        kill_unreaped_terminal(
            resources,
            VideoRelayFailure::new(
                operation,
                wait_error.kind(),
                format!("helper 退出状态未知: {wait_error}{signal_detail}"),
            ),
        )
    }

    fn abort_start(
        control: &ControlPage,
        resources: &mut ParentResources,
        primary: VideoRelayFailure,
        readiness_deadline: Instant,
    ) -> VideoRelayStartError {
        let deadline = readiness_deadline.max(Instant::now() + DEFAULT_STOP_TIMEOUT);
        control.request_stop();
        let _ = signal_helper(resources, nix::libc::SIGTERM);
        let kill_at = (Instant::now() + TERM_GRACE_LIMIT).min(deadline);
        let mut kill_sent = false;
        loop {
            match try_wait_helper(control, resources, ParentWaitSite::Startup) {
                Ok(Some(status)) => {
                    let cleanup = cleanup_parent_resources(resources, None);
                    let reason = if status.success() {
                        VideoRelayTerminalReason::Failed(primary.clone())
                    } else {
                        VideoRelayTerminalReason::ThreadPanicked(format!(
                            "视频 helper 在启动边界异常退出: {status}"
                        ))
                    };
                    return VideoRelayStartError {
                        primary: primary.clone(),
                        terminal: Box::new(VideoRelayTerminal { reason, cleanup }),
                        thread_started: true,
                        thread_joined: true,
                    };
                }
                Ok(None) => {}
                Err(error) => {
                    let terminal =
                        kill_after_unknown_wait(resources, "回收失败启动的视频 helper", &error);
                    return VideoRelayStartError {
                        primary,
                        terminal: Box::new(terminal),
                        thread_started: true,
                        thread_joined: false,
                    };
                }
            }
            let now = Instant::now();
            if !kill_sent && now >= kill_at {
                let _ = signal_helper(resources, nix::libc::SIGKILL);
                kill_sent = true;
            }
            if now >= deadline {
                let failure = VideoRelayFailure::new(
                    "强制回收失败启动的视频 helper",
                    io::ErrorKind::TimedOut,
                    format!(
                        "SIGKILL 后 helper pid={} 未在总期限内 reap",
                        resources.child.id()
                    ),
                );
                return VideoRelayStartError {
                    primary,
                    terminal: Box::new(kill_unreaped_terminal(resources, failure)),
                    thread_started: true,
                    thread_joined: false,
                };
            }
            thread::sleep(SUPERVISOR_POLL_INTERVAL);
        }
    }

    fn supervise_helper(shared: Arc<SupervisorShared>, mut resources: ParentResources) {
        let mut last_sequence = 0_u64;
        let mut term_sent_at = None;
        let mut kill_sent = false;
        loop {
            let published = shared.control.published_sequence();
            if published != last_sequence {
                last_sequence = published;
                shared.sequence.store(published, Ordering::Release);
                shared.notify.notify_waiters();
            }
            match try_wait_helper(&shared.control, &mut resources, ParentWaitSite::Supervisor) {
                Ok(Some(status)) => {
                    shared.helper_exited.store(true, Ordering::Release);
                    let terminal = finalize_reaped_helper(&mut resources, status);
                    publish_parent_terminal(&shared, terminal);
                    return;
                }
                Ok(None) => {}
                Err(error) => {
                    let terminal =
                        kill_after_unknown_wait(&mut resources, "轮询视频 helper 退出状态", &error);
                    publish_parent_terminal(&shared, terminal);
                    return;
                }
            }
            if shared.control.stop_requested() {
                if term_sent_at.is_none() {
                    let _ = signal_helper(&resources, nix::libc::SIGTERM);
                    term_sent_at = Some(Instant::now());
                }
                let deadline = shared
                    .deadline()
                    .unwrap_or_else(|| Instant::now() + DEFAULT_STOP_TIMEOUT);
                let kill_at = (term_sent_at.unwrap() + TERM_GRACE_LIMIT).min(deadline);
                let now = Instant::now();
                if !kill_sent && now >= kill_at {
                    let _ = signal_helper(&resources, nix::libc::SIGKILL);
                    kill_sent = true;
                }
                if now >= deadline {
                    match try_wait_helper(
                        &shared.control,
                        &mut resources,
                        ParentWaitSite::Supervisor,
                    ) {
                        Ok(Some(status)) => {
                            shared.helper_exited.store(true, Ordering::Release);
                            let terminal = finalize_reaped_helper(&mut resources, status);
                            publish_parent_terminal(&shared, terminal);
                        }
                        Ok(None) => {
                            let failure = VideoRelayFailure::new(
                                "强制回收视频 helper",
                                io::ErrorKind::TimedOut,
                                format!(
                                    "SIGKILL 后 helper pid={} 未在总期限内 reap",
                                    resources.child.id()
                                ),
                            );
                            let terminal = kill_unreaped_terminal(&mut resources, failure);
                            publish_parent_terminal(&shared, terminal);
                        }
                        Err(error) => {
                            let terminal = kill_after_unknown_wait(
                                &mut resources,
                                "期限末轮询视频 helper 退出状态",
                                &error,
                            );
                            publish_parent_terminal(&shared, terminal);
                        }
                    }
                    return;
                }
            }
            let state = lock_unpoisoned(&shared.requested_deadline);
            let _ = shared
                .wake
                .wait_timeout(state, SUPERVISOR_POLL_INTERVAL)
                .unwrap_or_else(|poisoned| poisoned.into_inner());
        }
    }

    fn publish_parent_terminal(shared: &SupervisorShared, terminal: VideoRelayTerminal) {
        shared.control.set_published_sequence(0);
        shared.sequence.store(0, Ordering::Release);
        shared.notify.notify_waiters();
        if shared
            .control
            .take_test_fault(FAULT_PAUSE_BEFORE_PARENT_TERMINAL)
        {
            shared
                .control
                .set_helper_phase(PHASE_BEFORE_PARENT_TERMINAL);
            while !shared.control.take_test_resume() {
                thread::sleep(Duration::from_millis(1));
            }
            shared.control.set_helper_phase(PHASE_IDLE);
        }
        shared.terminal_signal.publish(terminal);
        if shared
            .control
            .take_test_fault(FAULT_PAUSE_AFTER_PARENT_TERMINAL)
        {
            shared.control.set_helper_phase(PHASE_AFTER_PARENT_TERMINAL);
            while !shared.control.take_test_resume() {
                thread::sleep(Duration::from_millis(1));
            }
            shared.control.set_helper_phase(PHASE_IDLE);
        }
    }

    fn finalize_reaped_helper(
        resources: &mut ParentResources,
        status: ExitStatus,
    ) -> VideoRelayTerminal {
        resources
            .ipc
            .set_read_timeout(Some(Duration::from_millis(50)))
            .ok();
        let report = IpcRecord::read(&mut resources.ipc).ok();
        let reason = match report.as_ref() {
            Some(record)
                if record.kind().ok() == Some(IpcKind::Terminal)
                    && record.generation() == resources.generation =>
            {
                let (code, errno) = record.code();
                match code {
                    HELPER_CODE_STOP_REQUESTED if status.success() => {
                        VideoRelayTerminalReason::StopRequested
                    }
                    HELPER_CODE_PARENT_GONE => {
                        VideoRelayTerminalReason::Failed(VideoRelayFailure::new(
                            "视频 helper 父进程复验",
                            io::ErrorKind::BrokenPipe,
                            "helper 检测到父进程退出",
                        ))
                    }
                    HELPER_CODE_INIT_SHM_FAILED => VideoRelayTerminalReason::Failed(
                        helper_failure("初始化视频共享内存", errno),
                    ),
                    HELPER_CODE_BIND_UDS_FAILED => {
                        VideoRelayTerminalReason::Failed(helper_failure("绑定视频 UDS", errno))
                    }
                    HELPER_CODE_CLEANUP_FAILED => {
                        VideoRelayTerminalReason::Failed(helper_failure("回收视频中转资源", errno))
                    }
                    HELPER_CODE_COMMIT_FAILED => {
                        VideoRelayTerminalReason::Failed(helper_failure("提交视频共享内存", errno))
                    }
                    _ => VideoRelayTerminalReason::Failed(helper_failure("视频 helper", errno)),
                }
            }
            _ if !status.success() => VideoRelayTerminalReason::Failed(VideoRelayFailure::new(
                "视频 helper 异常退出",
                io::ErrorKind::Other,
                format!("helper exit status={status}"),
            )),
            _ => VideoRelayTerminalReason::Failed(VideoRelayFailure::new(
                "读取视频 helper Terminal",
                io::ErrorKind::UnexpectedEof,
                "helper 已退出但 Terminal IPC 缺失、截断或错代",
            )),
        };
        let cleanup = cleanup_parent_resources(resources, report.as_ref());
        VideoRelayTerminal { reason, cleanup }
    }

    fn cleanup_parent_resources(
        resources: &mut ParentResources,
        report: Option<&IpcRecord>,
    ) -> VideoRelayCleanup {
        let uds_cleanup = resources
            .ledger
            .uds
            .take()
            .map_or(VideoRelayPathCleanup::NotCreated, OwnedEntry::cleanup);
        let uds = probe_public_path(
            &resources.leases.uds_anchor,
            uds_cleanup,
            resources.ledger.initial_uds.as_ref(),
            "未采纳的公共 UDS",
        );
        let candidate_anchor =
            helper_candidate_anchor(&resources.leases.uds_anchor, resources.generation);
        let candidate = probe_public_path(
            &candidate_anchor,
            VideoRelayPathCleanup::NotCreated,
            resources.ledger.initial_uds_candidate.as_ref(),
            "未采纳的私有 UDS candidate",
        );
        let mut uds = merge_path_cleanup(uds, candidate);
        if report.is_some_and(|record| record.flags() & HELPER_FLAG_UDS_CLEAN == 0)
            && uds.is_complete()
        {
            uds = VideoRelayPathCleanup::Residual {
                path: candidate_anchor.public_path,
                detail: "helper 报告 UDS 清理未完成，父侧无法证明目录同步已完成".to_string(),
            };
        }
        let shm_cleanup = resources
            .ledger
            .shm
            .take()
            .map_or(VideoRelayPathCleanup::NotCreated, OwnedEntry::cleanup);
        let shm = probe_public_path(
            &resources.leases.shm_anchor,
            shm_cleanup,
            resources.ledger.initial_shm.as_ref(),
            "未采纳的公共 SHM",
        );
        let (uds_lease, shm_lease) = resources.leases.cleanup();
        let mmap = match report.map(IpcRecord::flags) {
            Some(flags) if flags & HELPER_FLAG_MMAP_FLUSH_FAILED != 0 => {
                VideoRelayMmapCleanup::FlushFailed("helper MS_SYNC 失败".to_string())
            }
            Some(flags)
                if flags & HELPER_FLAG_MMAP_CREATED != 0
                    && flags & HELPER_FLAG_MMAP_FLUSHED != 0 =>
            {
                VideoRelayMmapCleanup::FlushedAndReleased
            }
            Some(flags) if flags & HELPER_FLAG_MMAP_CREATED != 0 => {
                VideoRelayMmapCleanup::ReleasedWithoutFlushAfterFailure
            }
            None if !matches!(shm, VideoRelayPathCleanup::NotCreated) => {
                VideoRelayMmapCleanup::ReleasedWithoutFlushAfterFailure
            }
            _ => VideoRelayMmapCleanup::NotCreated,
        };
        VideoRelayCleanup {
            mmap,
            uds,
            shm,
            uds_lease,
            shm_lease,
        }
    }

    fn probe_public_path(
        anchor: &PathAnchor,
        cleanup: VideoRelayPathCleanup,
        initial_identity: Option<&ObservedIdentity>,
        label: &str,
    ) -> VideoRelayPathCleanup {
        match anchor.metadata() {
            Ok(None) if matches!(cleanup, VideoRelayPathCleanup::NotCreated) => {
                VideoRelayPathCleanup::AlreadyAbsent
            }
            Ok(None) => cleanup,
            Ok(Some(metadata))
                if initial_identity.is_some_and(|initial| initial.matches(&metadata)) =>
            {
                VideoRelayPathCleanup::ReplacementPreserved
            }
            Ok(Some(metadata)) => VideoRelayPathCleanup::Residual {
                path: anchor.public_path.clone(),
                detail: format!(
                    "helper 已 reap，但{label}仍存在且身份未知 (dev={}, ino={})",
                    metadata.dev(),
                    metadata.ino()
                ),
            },
            Err(error) => VideoRelayPathCleanup::Residual {
                path: anchor.public_path.clone(),
                detail: format!("无法证明{label}已清零: {error}"),
            },
        }
    }

    fn adopt_terminal_identities(resources: &mut ParentResources, report: &IpcRecord) {
        let (shm_identity, uds_identity) = report.identities();
        if resources.ledger.shm.is_none() && shm_identity != (0, 0) {
            resources.ledger.shm = OwnedEntry::open_expected(
                resources.leases.shm_anchor.clone(),
                shm_identity.0,
                shm_identity.1,
                EntryRole::Shm,
            )
            .ok();
        }
        if resources.ledger.uds.is_none() && uds_identity != (0, 0) {
            resources.ledger.uds = OwnedEntry::open_expected(
                resources.leases.uds_anchor.clone(),
                uds_identity.0,
                uds_identity.1,
                EntryRole::Uds,
            )
            .ok();
        }
    }

    fn kill_unreaped_terminal(
        resources: &mut ParentResources,
        failure: VideoRelayFailure,
    ) -> VideoRelayTerminal {
        let cleanup = VideoRelayCleanup {
            mmap: VideoRelayMmapCleanup::KillUnreaped,
            uds: residual_path(
                &resources.leases.uds_anchor.public_path,
                "helper 未 reap，拒绝并发清理 UDS",
            ),
            shm: residual_path(
                &resources.leases.shm_anchor.public_path,
                "helper 未 reap，拒绝并发清理 SHM",
            ),
            uds_lease: residual_path(
                &resources.leases.uds_anchor.public_path,
                "helper 未 reap，UDS lease 被故意保留",
            ),
            shm_lease: residual_path(
                &resources.leases.shm_anchor.public_path,
                "helper 未 reap，SHM lease 被故意保留",
            ),
        };
        if let Some(entry) = resources.ledger.uds.take() {
            std::mem::forget(entry);
        }
        if let Some(entry) = resources.ledger.shm.take() {
            std::mem::forget(entry);
        }
        if let Some(lease) = resources.leases.uds_lease.take() {
            std::mem::forget(lease);
        }
        if let Some(lease) = resources.leases.shm_lease.take() {
            std::mem::forget(lease);
        }
        VideoRelayTerminal {
            reason: VideoRelayTerminalReason::KillUnreaped(failure),
            cleanup,
        }
    }

    fn residual_path(path: &Path, detail: &str) -> VideoRelayPathCleanup {
        VideoRelayPathCleanup::Residual {
            path: path.to_path_buf(),
            detail: detail.to_string(),
        }
    }

    fn failure_from_helper_record(fallback: &'static str, record: &IpcRecord) -> VideoRelayFailure {
        let (code, errno) = record.code();
        let operation = match code {
            HELPER_CODE_INIT_SHM_FAILED => "初始化视频共享内存",
            HELPER_CODE_BIND_UDS_FAILED => "绑定视频 UDS",
            HELPER_CODE_COMMIT_FAILED => "提交视频共享内存",
            HELPER_CODE_CLEANUP_FAILED => "回收视频中转资源",
            _ => fallback,
        };
        helper_failure(operation, errno)
    }

    fn helper_failure(operation: &'static str, errno: i32) -> VideoRelayFailure {
        let error = if errno > 0 {
            io::Error::from_raw_os_error(errno)
        } else {
            io::Error::other("视频 helper 返回未分类失败")
        };
        VideoRelayFailure::from_error(operation, &error)
    }

    fn helper_errno(error: &io::Error, fallback: i32) -> i32 {
        error.raw_os_error().unwrap_or_else(|| match error.kind() {
            io::ErrorKind::AlreadyExists => nix::libc::EEXIST,
            io::ErrorKind::AddrInUse => nix::libc::EADDRINUSE,
            io::ErrorKind::AddrNotAvailable => nix::libc::EADDRNOTAVAIL,
            io::ErrorKind::InvalidInput | io::ErrorKind::InvalidData => nix::libc::EINVAL,
            io::ErrorKind::NotFound => nix::libc::ENOENT,
            io::ErrorKind::PermissionDenied => nix::libc::EACCES,
            io::ErrorKind::TimedOut => nix::libc::ETIMEDOUT,
            io::ErrorKind::Interrupted => nix::libc::EINTR,
            io::ErrorKind::BrokenPipe => nix::libc::EPIPE,
            _ => fallback,
        })
    }

    fn open_pidfd(pid: u32) -> io::Result<File> {
        let descriptor = unsafe {
            nix::libc::syscall(
                nix::libc::SYS_pidfd_open,
                nix::libc::pid_t::try_from(pid).map_err(|_| {
                    io::Error::new(io::ErrorKind::InvalidInput, "helper pid 超出 pid_t")
                })?,
                0_u32,
            )
        };
        if descriptor < 0 {
            return Err(io::Error::last_os_error());
        }
        let descriptor =
            RawFd::try_from(descriptor).map_err(|_| io::Error::other("pidfd 超出 RawFd"))?;
        Ok(unsafe { File::from_raw_fd(descriptor) })
    }

    fn send_pidfd_signal(pidfd: &File, signal: i32) -> io::Result<()> {
        let result = unsafe {
            nix::libc::syscall(
                nix::libc::SYS_pidfd_send_signal,
                pidfd.as_raw_fd(),
                signal,
                std::ptr::null::<nix::libc::siginfo_t>(),
                0_u32,
            )
        };
        if result == 0 {
            Ok(())
        } else {
            let error = io::Error::last_os_error();
            if error.raw_os_error() == Some(nix::libc::ESRCH) {
                Ok(())
            } else {
                Err(error)
            }
        }
    }

    fn signal_helper(resources: &ParentResources, signal: i32) -> io::Result<()> {
        if let Some(pidfd) = &resources.pidfd {
            return send_pidfd_signal(pidfd, signal);
        }
        let pid = nix::libc::pid_t::try_from(resources.child.id())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "helper pid 超出 pid_t"))?;
        let result = unsafe { nix::libc::kill(pid, signal) };
        if result == 0 {
            Ok(())
        } else {
            let error = io::Error::last_os_error();
            if error.raw_os_error() == Some(nix::libc::ESRCH) {
                Ok(())
            } else {
                Err(error)
            }
        }
    }

    static HELPER_SIGNAL_STOP: AtomicBool = AtomicBool::new(false);

    extern "C" fn helper_stop_signal(_signal: nix::libc::c_int) {
        HELPER_SIGNAL_STOP.store(true, Ordering::Release);
    }

    fn validate_component_length(bytes: &[u8], capacity: usize, field: &str) -> io::Result<()> {
        if bytes.is_empty() || bytes.len() > capacity {
            Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("{field} 长度超出固定 IPC 合同"),
            ))
        } else {
            Ok(())
        }
    }

    fn write_u16(bytes: &mut [u8], offset: usize, value: u16) {
        bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
    }

    fn write_u32(bytes: &mut [u8], offset: usize, value: u32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn write_i32(bytes: &mut [u8], offset: usize, value: i32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn write_u64(bytes: &mut [u8], offset: usize, value: u64) {
        bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }

    fn read_u16(bytes: &[u8], offset: usize) -> u16 {
        u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap())
    }

    fn read_u32(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }

    fn read_i32(bytes: &[u8], offset: usize) -> i32 {
        i32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }

    fn read_u64(bytes: &[u8], offset: usize) -> u64 {
        u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
    }
}

/// 在进程入口最早阶段显式调用；仅当固定 helper 请求标记存在时接管进程。
pub fn run_requested_helper() -> io::Result<bool> {
    if !process_helper::request_present() {
        return Ok(false);
    }
    process_helper::run_requested()?;
    Ok(true)
}








struct TerminalSignal {
    terminal: Mutex<Option<VideoRelayTerminal>>,
    wake_blocking: Condvar,
    wake_async: Notify,
}

impl TerminalSignal {
    fn new() -> Self {
        Self {
            terminal: Mutex::new(None),
            wake_blocking: Condvar::new(),
            wake_async: Notify::new(),
        }
    }

    fn publish(&self, terminal: VideoRelayTerminal) {
        let mut slot = lock_unpoisoned(&self.terminal);
        if slot.is_none() {
            *slot = Some(terminal);
        }
        drop(slot);
        self.wake_blocking.notify_all();
        self.wake_async.notify_waiters();
    }

    fn try_get(&self) -> Option<VideoRelayTerminal> {
        lock_unpoisoned(&self.terminal).clone()
    }

    fn wait_timeout(&self, timeout: Duration) -> Option<VideoRelayTerminal> {
        let deadline = Instant::now().checked_add(timeout)?;
        let mut slot = lock_unpoisoned(&self.terminal);
        loop {
            if let Some(terminal) = slot.clone() {
                return Some(terminal);
            }
            let remaining = deadline.checked_duration_since(Instant::now())?;
            let (next, result) = self
                .wake_blocking
                .wait_timeout(slot, remaining)
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            slot = next;
            if result.timed_out() && slot.is_none() {
                return None;
            }
        }
    }

    async fn wait(&self) -> VideoRelayTerminal {
        loop {
            let notified = self.wake_async.notified();
            if let Some(terminal) = self.try_get() {
                return terminal;
            }
            notified.await;
        }
    }
}




/// helper 进程、pidfd、终态和全部路径租约的唯一父侧所有者。
pub struct VideoRelayOwner {
    #[cfg(test)] handle: VideoRelayHandle,
    process: process_helper::ProcessOwner,
}

impl VideoRelayOwner {
    pub fn start(shm_path: &Path, uds_path: &Path) -> Result<Self, VideoRelayStartError> {
        let sequence = Arc::new(AtomicU64::new(0));
        let notify = Arc::new(Notify::new());
        let terminal_signal = Arc::new(TerminalSignal::new());
        let process = process_helper::ProcessOwner::launch(
            shm_path,
            uds_path,
            sequence.clone(),
            notify.clone(),
            terminal_signal,
            helper_start_faults(shm_path),
        )?;
        Ok(Self {
            #[cfg(test)] handle: VideoRelayHandle {
                sequence,
            },
            process,
        })
    }

    #[cfg(test)]
    pub fn handle(&self) -> VideoRelayHandle {
        self.handle.clone()
    }

    #[cfg(test)]
    pub fn stop(&self) -> bool {
        self.process
            .request_stop(Instant::now() + DEFAULT_STOP_TIMEOUT)
    }

    #[cfg(test)]
    pub fn try_terminal(&self) -> Option<VideoRelayTerminal> {
        self.process.try_terminal()
    }

    pub fn is_finished(&self) -> bool {
        self.process.is_finished()
    }

    pub async fn wait_terminal(&self) -> VideoRelayTerminal {
        self.process.wait_terminal().await
    }

    #[cfg(test)]
    pub fn join(&mut self, timeout: Duration) -> io::Result<VideoRelayTerminal> {
        self.process.join_until(Instant::now() + timeout)
    }

    pub fn shutdown(&mut self, timeout: Duration) -> io::Result<VideoRelayTerminal> {
        self.process.shutdown(timeout)
    }

    #[cfg(test)]
    fn helper_pid(&self) -> u32 {
        self.process.helper_pid()
    }

    #[cfg(test)]
    fn helper_control(&self) -> Arc<process_helper::ControlPage> {
        self.process.control()
    }

    #[cfg(test)]
    fn helper_fds_are_live(&self) -> bool {
        self.process.inherited_fds_are_live()
    }

    #[cfg(test)]
    fn helper_fd_numbers(&self) -> (i32, i32) {
        self.process.inherited_fd_numbers()
    }

    #[cfg(test)]
    fn terminal_signal(&self) -> Arc<TerminalSignal> {
        self.process.terminal_signal()
    }
}

impl Drop for VideoRelayOwner {
    fn drop(&mut self) {
        let _ = self.process.shutdown(DROP_TIMEOUT);
    }
}

#[cfg(not(test))]
fn helper_start_faults(_shm_path: &Path) -> u64 {
    0
}

#[cfg(test)]
fn helper_start_faults(shm_path: &Path) -> u64 {
    let mut pending = lock_unpoisoned(test_faults());
    let points = pending
        .keys()
        .filter_map(|(path, point)| (path == shm_path).then_some(*point))
        .collect::<Vec<_>>();
    let mut mask = 0_u64;
    for point in points {
        pending.remove(&(shm_path.to_path_buf(), point));
        mask |= helper_fault_mask(point);
    }
    mask
}

pub fn spawn_video_relay_owner(
    shm_path: &Path,
    uds_path: &Path,
) -> Result<VideoRelayOwner, VideoRelayStartError> {
    VideoRelayOwner::start(shm_path, uds_path)
}

/// 生产兼容名称也返回唯一所有者；调用者必须保留返回值。
pub fn spawn_video_relay(
    shm_path: &Path,
    uds_path: &Path,
) -> Result<VideoRelayOwner, VideoRelayStartError> {
    spawn_video_relay_owner(shm_path, uds_path)
}















#[cfg(test)]
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
enum TestFaultPoint {
    BindUds,
    StartupHeaderFlush,
    StartupTimeout,
    PanicAfterShm,
    CommitFlush,
    CleanupFlush,
    ShmCleanup,
    MsyncHang,
    IpcTruncated,
    WrongGeneration,
    PauseBeforeInvalidate,
    PauseAfterInvalidate,
    PauseAfterFlush,
    PauseShmCleanup,
    SupervisorSpawn,
    ParentTryWaitStartup,
    ParentTryWaitSupervisor,
    PauseBeforeUdsBind,
    PauseAfterUdsBind,
    PauseBeforeParentTerminal,
    PauseAfterParentTerminal,
    PauseAfterUdsBindBeforeIdentity,
    PauseAfterUdsIdentityBeforePublish,
    ShmPermissions,
    ShmMetadata,
    ShmTryClone,
    ShmIdentity,
    ShmPublishReplacement,
}

#[cfg(test)]
fn test_faults() -> &'static Mutex<std::collections::HashMap<(PathBuf, TestFaultPoint), usize>> {
    static FAULTS: std::sync::OnceLock<
        Mutex<std::collections::HashMap<(PathBuf, TestFaultPoint), usize>>,
    > = std::sync::OnceLock::new();
    FAULTS.get_or_init(|| Mutex::new(std::collections::HashMap::new()))
}

#[cfg(test)]
fn inject_test_fault(path: &Path, point: TestFaultPoint) {
    if let Some(control) = helper_control_for_path(path) {
        control.add_test_fault(helper_fault_mask(point));
        return;
    }
    *lock_unpoisoned(test_faults())
        .entry((path.to_path_buf(), point))
        .or_insert(0) += 1;
}

#[cfg(test)]
fn helper_fault_mask(point: TestFaultPoint) -> u64 {
    match point {
        TestFaultPoint::BindUds => process_helper::FAULT_BIND_UDS,
        TestFaultPoint::StartupHeaderFlush => process_helper::FAULT_STARTUP_HEADER_FLUSH,
        TestFaultPoint::StartupTimeout => process_helper::FAULT_STARTUP_TIMEOUT,
        TestFaultPoint::PanicAfterShm => process_helper::FAULT_HELPER_PANIC,
        TestFaultPoint::CommitFlush => process_helper::FAULT_COMMIT_FLUSH,
        TestFaultPoint::CleanupFlush => process_helper::FAULT_CLEANUP_FLUSH,
        TestFaultPoint::ShmCleanup => process_helper::FAULT_SHM_CLEANUP,
        TestFaultPoint::MsyncHang => process_helper::FAULT_MSYNC_HANG,
        TestFaultPoint::IpcTruncated => process_helper::FAULT_IPC_TRUNCATED,
        TestFaultPoint::WrongGeneration => process_helper::FAULT_WRONG_GENERATION,
        TestFaultPoint::PauseBeforeInvalidate => process_helper::FAULT_PAUSE_BEFORE_INVALIDATE,
        TestFaultPoint::PauseAfterInvalidate => process_helper::FAULT_PAUSE_AFTER_INVALIDATE,
        TestFaultPoint::PauseAfterFlush => process_helper::FAULT_PAUSE_AFTER_FLUSH,
        TestFaultPoint::PauseShmCleanup => process_helper::FAULT_PAUSE_SHM_CLEANUP,
        TestFaultPoint::SupervisorSpawn => process_helper::FAULT_SUPERVISOR_SPAWN,
        TestFaultPoint::ParentTryWaitStartup => process_helper::FAULT_PARENT_TRY_WAIT_STARTUP,
        TestFaultPoint::ParentTryWaitSupervisor => process_helper::FAULT_PARENT_TRY_WAIT_SUPERVISOR,
        TestFaultPoint::PauseBeforeUdsBind => process_helper::FAULT_PAUSE_BEFORE_UDS_BIND,
        TestFaultPoint::PauseAfterUdsBind => process_helper::FAULT_PAUSE_AFTER_UDS_BIND,
        TestFaultPoint::PauseBeforeParentTerminal => {
            process_helper::FAULT_PAUSE_BEFORE_PARENT_TERMINAL
        }
        TestFaultPoint::PauseAfterParentTerminal => {
            process_helper::FAULT_PAUSE_AFTER_PARENT_TERMINAL
        }
        TestFaultPoint::PauseAfterUdsBindBeforeIdentity => {
            process_helper::FAULT_PAUSE_AFTER_UDS_BIND_BEFORE_IDENTITY
        }
        TestFaultPoint::PauseAfterUdsIdentityBeforePublish => {
            process_helper::FAULT_PAUSE_AFTER_UDS_IDENTITY_BEFORE_PUBLISH
        }
        TestFaultPoint::ShmPermissions => process_helper::FAULT_SHM_PERMISSIONS,
        TestFaultPoint::ShmMetadata => process_helper::FAULT_SHM_METADATA,
        TestFaultPoint::ShmTryClone => process_helper::FAULT_SHM_TRY_CLONE,
        TestFaultPoint::ShmIdentity => process_helper::FAULT_SHM_IDENTITY,
        TestFaultPoint::ShmPublishReplacement => process_helper::FAULT_SHM_PUBLISH_REPLACEMENT,
    }
}

#[cfg(test)]
fn helper_control_registry(
) -> &'static Mutex<std::collections::HashMap<PathBuf, std::sync::Weak<process_helper::ControlPage>>>
{
    static CONTROLS: std::sync::OnceLock<
        Mutex<std::collections::HashMap<PathBuf, std::sync::Weak<process_helper::ControlPage>>>,
    > = std::sync::OnceLock::new();
    CONTROLS.get_or_init(|| Mutex::new(std::collections::HashMap::new()))
}

#[cfg(test)]
fn register_helper_control(path: &Path, control: &Arc<process_helper::ControlPage>) {
    lock_unpoisoned(helper_control_registry()).insert(path.to_path_buf(), Arc::downgrade(control));
}

#[cfg(test)]
fn helper_control_for_path(path: &Path) -> Option<Arc<process_helper::ControlPage>> {
    lock_unpoisoned(helper_control_registry())
        .get(path)
        .and_then(std::sync::Weak::upgrade)
}

#[cfg(test)]
fn take_test_fault(path: &Path, point: TestFaultPoint) -> bool {
    let mut faults = lock_unpoisoned(test_faults());
    let key = (path.to_path_buf(), point);
    let Some(remaining) = faults.get_mut(&key) else {
        return false;
    };
    *remaining -= 1;
    if *remaining == 0 {
        faults.remove(&key);
    }
    true
}

#[cfg(test)]
struct TestCommitGate {
    control: Arc<process_helper::ControlPage>,
    phase: u32,
}

#[cfg(test)]
fn install_commit_gate(path: &Path) -> Arc<TestCommitGate> {
    let control = helper_control_for_path(path).expect("helper control 已注册");
    control.add_test_fault(process_helper::FAULT_PAUSE_BEFORE_INVALIDATE);
    Arc::new(TestCommitGate {
        control,
        phase: process_helper::PHASE_BEFORE_INVALIDATE,
    })
}


#[cfg(test)]
fn install_cleanup_gate(path: &Path) -> Arc<TestCommitGate> {
    let control = helper_control_for_path(path).expect("helper control 已注册");
    control.add_test_fault(process_helper::FAULT_PAUSE_SHM_CLEANUP);
    Arc::new(TestCommitGate {
        control,
        phase: process_helper::PHASE_BEFORE_SHM_CLEANUP,
    })
}






#[cfg(test)]
impl TestCommitGate {
    fn wait_entered(&self) {
        let deadline = Instant::now() + Duration::from_secs(2);
        while self.control.helper_phase() != self.phase {
            assert!(
                Instant::now() < deadline,
                "helper gate 未进入 phase={}",
                self.phase
            );
            thread::sleep(Duration::from_millis(2));
        }
    }

    fn release(&self) {
        self.control.resume_test_gate();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::process::{Command, Stdio};

    static TEST_SEQUENCE: AtomicU64 = AtomicU64::new(0);

    #[test]
    fn requested_helper_child_entry() {
        if process_helper::request_present() {
            assert!(run_requested_helper().expect("运行显式视频 helper 请求"));
        }
    }

    #[test]
    fn parent_death_owner_child_entry() {
        let Ok(root) = std::env::var("NUWAX_VIDEO_PARENT_DEATH_ROOT_V1") else {
            return;
        };
        let root = PathBuf::from(root);
        let shm = root.join("video.shm");
        let uds = root.join("video.sock");
        let msync_hang = std::env::var_os("NUWAX_VIDEO_PARENT_DEATH_MSYNC_V1").is_some();
        if msync_hang {
            inject_test_fault(&shm, TestFaultPoint::MsyncHang);
        }
        let owner = spawn_video_relay_owner(&shm, &uds).expect("parent death child 启动 owner");
        if msync_hang {
            send_frame(&uds, b"parent-death-msync-hang");
            assert!(wait_until(
                || owner.helper_control().helper_phase() == process_helper::PHASE_MSYNC_HANG,
                Duration::from_secs(1)
            ));
        }
        std::fs::write(root.join("helper.pid"), owner.helper_pid().to_string())
            .expect("记录 parent death helper pid");
        std::thread::sleep(Duration::from_millis(20));
        unsafe { nix::libc::_exit(0) }
    }

    #[test]
    fn unknown_try_wait_owner_child_entry() {
        let Ok(root) = std::env::var("NUWAX_VIDEO_UNKNOWN_WAIT_ROOT_V1") else {
            return;
        };
        let mode =
            std::env::var("NUWAX_VIDEO_UNKNOWN_WAIT_MODE_V1").expect("unknown wait child mode");
        let root = PathBuf::from(root);
        let shm = root.join("video.shm");
        let uds = root.join("video.sock");
        let terminal = match mode.as_str() {
            "startup" => {
                inject_test_fault(&shm, TestFaultPoint::BindUds);
                inject_test_fault(&shm, TestFaultPoint::ParentTryWaitStartup);
                *expect_start_error(spawn_video_relay_owner(&shm, &uds)).terminal
            }
            "supervisor" => {
                inject_test_fault(&shm, TestFaultPoint::ParentTryWaitSupervisor);
                let mut owner = spawn_video_relay_owner(&shm, &uds)
                    .expect("运行期 try_wait fault 前 owner 应完成启动");
                owner
                    .join(Duration::from_secs(1))
                    .expect("运行期 try_wait fault 应发布有界终态")
            }
            other => panic!("未知 unknown wait child mode: {other}"),
        };
        assert!(matches!(
            terminal.reason,
            VideoRelayTerminalReason::KillUnreaped(_)
        ));
        assert!(!terminal.is_clean());
        std::fs::write(root.join("verified"), mode).expect("记录 unknown wait 子进程验证结果");
        unsafe { nix::libc::_exit(0) }
    }

    struct TestPaths {
        root: PathBuf,
        shm: PathBuf,
        uds: PathBuf,
    }

    impl TestPaths {
        fn new(label: &str) -> Self {
            let id = TEST_SEQUENCE.fetch_add(1, Ordering::Relaxed);
            let root = std::env::temp_dir().join(format!(
                "nuwax-video-relay-{label}-{}-{id}",
                std::process::id()
            ));
            std::fs::create_dir(&root).expect("创建视频中转测试目录");
            Self {
                shm: root.join("video.shm"),
                uds: root.join("video.sock"),
                root,
            }
        }
    }

    impl Drop for TestPaths {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.root);
        }
    }

    fn send_frame(path: &Path, frame: &[u8]) {
        let mut sender = UnixStream::connect(path).expect("连接视频 UDS");
        sender
            .write_all(&(frame.len() as u32).to_le_bytes())
            .expect("发送帧长度");
        sender.write_all(frame).expect("发送视频帧");
    }

    fn expect_start_error(
        result: Result<VideoRelayOwner, VideoRelayStartError>,
    ) -> VideoRelayStartError {
        match result {
            Ok(_) => panic!("视频中转启动应失败"),
            Err(error) => error,
        }
    }

    fn wait_until(mut condition: impl FnMut() -> bool, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if condition() {
                return true;
            }
            thread::sleep(Duration::from_millis(5));
        }
        condition()
    }

    fn mapped_sequence(reader: &memmap2::Mmap) -> u64 {
        debug_assert_eq!(16 % std::mem::align_of::<AtomicU64>(), 0);
        let sequence = unsafe { &*reader.as_ptr().add(16).cast::<AtomicU64>() };
        sequence.load(Ordering::Acquire)
    }

    fn mapped_generation(reader: &memmap2::Mmap) -> [u8; 16] {
        reader[44..60].try_into().unwrap()
    }

    fn mapped_snapshot(reader: &memmap2::Mmap) -> Option<(u64, Vec<u8>, u64)> {
        for _ in 0..3 {
            let sequence = mapped_sequence(reader);
            if sequence == 0 {
                return None;
            }
            let frame_size = u32::from_le_bytes(reader[24..28].try_into().unwrap()) as usize;
            let timestamp = u64::from_le_bytes(reader[32..40].try_into().unwrap());
            let active_buffer = u32::from_le_bytes(reader[40..44].try_into().unwrap()) as usize;
            if frame_size == 0 || frame_size > MAX_FRAME_SIZE || active_buffer > 1 {
                return None;
            }
            let offset = HEADER_SIZE + active_buffer * MAX_FRAME_SIZE;
            let payload = reader[offset..offset + frame_size].to_vec();
            if mapped_sequence(reader) == sequence {
                return Some((sequence, payload, timestamp));
            }
        }
        None
    }

    fn install_process_gate(
        owner: &VideoRelayOwner,
        fault: u64,
        phase: u32,
    ) -> Arc<TestCommitGate> {
        let control = owner.helper_control();
        control.add_test_fault(fault);
        Arc::new(TestCommitGate { control, phase })
    }

    fn process_exists(pid: u32) -> bool {
        Path::new("/proc").join(pid.to_string()).exists()
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    struct ProcessFdIdentity {
        descriptor: i32,
        target: PathBuf,
        device: u64,
        inode: u64,
        fdinfo: String,
    }

    impl ProcessFdIdentity {
        fn same_kernel_object(&self, other: &Self) -> bool {
            self.target == other.target && self.device == other.device && self.inode == other.inode
        }

        fn still_at_original_slot(&self) -> bool {
            process_fd_identity(self.descriptor).is_some_and(|current| {
                self.same_kernel_object(&current) && self.fdinfo == current.fdinfo
            })
        }
    }

    fn process_fd_identity(descriptor: i32) -> Option<ProcessFdIdentity> {
        let descriptor_path = Path::new("/proc/self/fd").join(descriptor.to_string());
        let target = std::fs::read_link(&descriptor_path).ok()?;
        let metadata = std::fs::metadata(&descriptor_path).ok()?;
        let fdinfo =
            std::fs::read_to_string(Path::new("/proc/self/fdinfo").join(descriptor.to_string()))
                .ok()?;
        Some(ProcessFdIdentity {
            descriptor,
            target,
            device: metadata.dev(),
            inode: metadata.ino(),
            fdinfo,
        })
    }

    fn process_fd_target_count(identity: &ProcessFdIdentity) -> usize {
        std::fs::read_dir("/proc/self/fd")
            .unwrap()
            .filter_map(Result::ok)
            .filter_map(|entry| entry.file_name().to_string_lossy().parse::<i32>().ok())
            .filter_map(process_fd_identity)
            .filter(|candidate| identity.same_kernel_object(candidate))
            .count()
    }

    fn duplicate_fd_cloexec(descriptor: i32) -> File {
        let duplicate = unsafe { nix::libc::fcntl(descriptor, nix::libc::F_DUPFD_CLOEXEC, 3) };
        assert!(
            duplicate >= 0,
            "复制测试 fd 失败: {}",
            io::Error::last_os_error()
        );
        unsafe { File::from_raw_fd(duplicate) }
    }

    fn same_open_file_description(first: i32, second: i32) -> bool {
        const KCMP_FILE: nix::libc::c_int = 0;
        let pid = unsafe { nix::libc::getpid() };
        let result =
            unsafe { nix::libc::syscall(nix::libc::SYS_kcmp, pid, pid, KCMP_FILE, first, second) };
        assert!(
            result >= 0,
            "kcmp(KCMP_FILE) 失败: {}",
            io::Error::last_os_error()
        );
        result == 0
    }

    fn same_open_file_description_across_processes(
        local_descriptor: i32,
        remote_pid: u32,
        remote_descriptor: i32,
    ) -> bool {
        const KCMP_FILE: nix::libc::c_int = 0;
        let local_pid = unsafe { nix::libc::getpid() };
        let remote_pid = nix::libc::pid_t::try_from(remote_pid).unwrap();
        let result = unsafe {
            nix::libc::syscall(
                nix::libc::SYS_kcmp,
                local_pid,
                remote_pid,
                KCMP_FILE,
                local_descriptor,
                remote_descriptor,
            )
        };
        assert!(
            result >= 0,
            "跨进程 kcmp(KCMP_FILE) 失败: {}",
            io::Error::last_os_error()
        );
        result == 0
    }

    fn create_non_cloexec_memfd(label: &str) -> File {
        let name = CString::new(format!("nuwax-video-fd-sentinel-{label}")).unwrap();
        let descriptor = unsafe { nix::libc::memfd_create(name.as_ptr(), 0) };
        assert!(
            descriptor >= 0,
            "创建非 CLOEXEC sentinel 失败: {}",
            io::Error::last_os_error()
        );
        assert_eq!(
            unsafe { nix::libc::fcntl(descriptor, nix::libc::F_GETFD) } & nix::libc::FD_CLOEXEC,
            0
        );
        unsafe { File::from_raw_fd(descriptor) }
    }

    fn duplicate_fd_non_cloexec(descriptor: i32) -> File {
        let duplicate = unsafe { nix::libc::fcntl(descriptor, nix::libc::F_DUPFD, 3) };
        assert!(
            duplicate >= 0,
            "复制非 CLOEXEC sentinel 失败: {}",
            io::Error::last_os_error()
        );
        unsafe { File::from_raw_fd(duplicate) }
    }

    fn process_descriptors(pid: u32) -> Vec<i32> {
        let mut descriptors =
            std::fs::read_dir(Path::new("/proc").join(pid.to_string()).join("fd"))
                .unwrap()
                .filter_map(Result::ok)
                .filter_map(|entry| entry.file_name().to_string_lossy().parse::<i32>().ok())
                .collect::<Vec<_>>();
        descriptors.sort_unstable();
        descriptors
    }

    fn open_test_pidfd(pid: u32) -> File {
        let descriptor = unsafe {
            nix::libc::syscall(
                nix::libc::SYS_pidfd_open,
                nix::libc::pid_t::try_from(pid).unwrap(),
                0_u32,
            )
        };
        assert!(
            descriptor >= 0,
            "打开测试 pidfd 失败: {}",
            io::Error::last_os_error()
        );
        unsafe { File::from_raw_fd(i32::try_from(descriptor).unwrap()) }
    }

    fn reuse_fd_slot_with_pidfd(descriptor: i32, pid: u32) -> File {
        let source = open_test_pidfd(pid);
        if source.as_raw_fd() == descriptor {
            return source;
        }
        let replacement =
            unsafe { nix::libc::dup3(source.as_raw_fd(), descriptor, nix::libc::O_CLOEXEC) };
        assert_eq!(
            replacement,
            descriptor,
            "复用 pidfd slot 失败: {}",
            io::Error::last_os_error()
        );
        unsafe { File::from_raw_fd(replacement) }
    }

    fn wait_for_helper_control(path: &Path) -> Arc<process_helper::ControlPage> {
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if let Some(control) = helper_control_for_path(path) {
                return control;
            }
            assert!(Instant::now() < deadline, "helper control 未注册");
            thread::sleep(Duration::from_millis(2));
        }
    }

    fn assert_uds_publish_cancel_window(
        fault: TestFaultPoint,
        phase: u32,
        invalidate_generation: bool,
        public_after_bind: bool,
        label: &str,
    ) {
        let paths = TestPaths::new(label);
        inject_test_fault(&paths.shm, fault);
        let shm = paths.shm.clone();
        let uds = paths.uds.clone();
        let starter = thread::spawn(move || spawn_video_relay_owner(&shm, &uds));
        let control = wait_for_helper_control(&paths.shm);
        let gate = TestCommitGate { control, phase };
        gate.wait_entered();
        assert_eq!(paths.uds.exists(), public_after_bind, "{label}");
        if invalidate_generation {
            gate.control.invalidate_generation_for_test();
        } else {
            assert!(gate.control.request_stop());
        }
        gate.release();
        let error = expect_start_error(starter.join().unwrap());
        assert!(error.thread_joined, "{label}: {error}");
        assert!(error.terminal.is_clean(), "{label}: {:?}", error.terminal);
        assert!(!paths.shm.exists() && !paths.uds.exists(), "{label}");
        assert_eq!(
            std::fs::read_dir(&paths.root).unwrap().count(),
            0,
            "{label}"
        );
    }

    fn run_unknown_wait_child(mode: &str) {
        let paths = TestPaths::new(&format!("unknown-wait-{mode}"));
        let status = Command::new(std::env::current_exe().unwrap())
            .arg("--exact")
            .arg("video_relay::tests::unknown_try_wait_owner_child_entry")
            .arg("--nocapture")
            .arg("--test-threads=1")
            .env("NUWAX_VIDEO_UNKNOWN_WAIT_ROOT_V1", &paths.root)
            .env("NUWAX_VIDEO_UNKNOWN_WAIT_MODE_V1", mode)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("启动 unknown wait 独立测试子进程");
        assert!(status.success(), "unknown wait child mode={mode}: {status}");
        assert_eq!(
            std::fs::read_to_string(paths.root.join("verified")).unwrap(),
            mode
        );
    }

    fn assert_stop_window(fault: u64, phase: u32, sequence_at_gate: u64, label: &str) {
        let paths = TestPaths::new(label);
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let handle = owner.handle();
        let reader = unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        send_frame(&paths.uds, b"first-frame");
        assert!(wait_until(
            || handle.sequence.load(Ordering::Acquire) == 1,
            Duration::from_secs(1)
        ));
        let generation = mapped_generation(&reader);
        let gate = install_process_gate(&owner, fault, phase);
        let uds = paths.uds.clone();
        let sender = thread::spawn(move || send_frame(&uds, b"blocked-frame"));
        gate.wait_entered();
        assert_eq!(mapped_sequence(&reader), sequence_at_gate);
        assert!(owner.stop());
        gate.release();
        sender.join().unwrap();
        let terminal = owner.join(Duration::from_secs(1)).unwrap();
        assert!(terminal.is_expected_shutdown(), "{label}: {terminal:?}");
        assert_eq!(handle.sequence.load(Ordering::Acquire), 0);
        assert_eq!(mapped_sequence(&reader), 0);
        assert_eq!(mapped_generation(&reader), generation);
        assert!(mapped_snapshot(&reader).is_none());
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn owner_ready_repeated_shutdown_and_cleanup_are_complete() {
        let paths = TestPaths::new("clean");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).expect("启动视频中转");
        assert!(paths.shm.exists() && paths.uds.exists());
        assert!(owner.stop());
        assert!(!owner.stop());
        let first = owner
            .shutdown(Duration::from_secs(1))
            .expect("关闭视频中转");
        let second = owner
            .shutdown(Duration::from_millis(1))
            .expect("重复关闭视频中转");
        assert_eq!(first, second);
        assert!(first.is_expected_shutdown() && first.is_clean());
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn stale_regular_shm_is_reclaimed_under_exclusive_pair_lease() {
        let paths = TestPaths::new("stale-shm");
        std::fs::write(&paths.shm, b"stale-shm").unwrap();
        let other_uds = paths.root.join("other.sock");
        let mut owner = spawn_video_relay_owner(&paths.shm, &other_uds)
            .expect("排他租约持有者应回收异常退出遗留 SHM");
        assert_ne!(std::fs::read(&paths.shm).unwrap(), b"stale-shm");
        assert!(other_uds.exists());
        assert!(owner.shutdown(Duration::from_secs(1)).unwrap().is_clean());
        assert!(!paths.shm.exists() && !other_uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn simultaneous_same_pair_has_exactly_one_owner() {
        for round in 0..50 {
            let paths = TestPaths::new(&format!("same-pair-{round}"));
            let barrier = Arc::new(std::sync::Barrier::new(3));
            let spawn_candidate = |barrier: Arc<std::sync::Barrier>| {
                let shm = paths.shm.clone();
                let uds = paths.uds.clone();
                thread::spawn(move || {
                    barrier.wait();
                    spawn_video_relay_owner(&shm, &uds)
                })
            };
            let first = spawn_candidate(barrier.clone());
            let second = spawn_candidate(barrier.clone());
            barrier.wait();
            let first = first.join().unwrap();
            let second = second.join().unwrap();
            let (mut owner, loser) = match (first, second) {
                (Ok(owner), Err(error)) | (Err(error), Ok(owner)) => (owner, error),
                (Ok(_), Ok(_)) => panic!("同一路径对不能产生两个所有者"),
                (Err(first), Err(second)) => {
                    panic!("同一路径对必须产生一个所有者: {first}; {second}")
                }
            };
            assert_eq!(loser.primary.kind, io::ErrorKind::AddrInUse);
            assert!(owner.shutdown(Duration::from_secs(1)).unwrap().is_clean());
            assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
        }
    }

    #[test]
    fn same_shm_with_different_uds_is_rejected_before_touching_second_uds() {
        let paths = TestPaths::new("shared-shm");
        let mut first = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let other_uds = paths.root.join("other.sock");
        let shm_inode = std::fs::metadata(&paths.shm).unwrap().ino();
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &other_uds));
        assert_eq!(error.primary.kind, io::ErrorKind::AddrInUse);
        assert_eq!(std::fs::metadata(&paths.shm).unwrap().ino(), shm_inode);
        assert!(!other_uds.exists());
        assert!(first.shutdown(Duration::from_secs(1)).unwrap().is_clean());
    }

    #[test]
    fn startup_bind_failure_joins_and_cleans_partial_creation() {
        let paths = TestPaths::new("bind-failure");
        inject_test_fault(&paths.shm, TestFaultPoint::BindUds);
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert_eq!(error.primary.operation, "绑定视频 UDS");
        assert!(error.thread_started && error.thread_joined);
        assert!(error.terminal.is_clean());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn startup_header_flush_failure_joins_and_cleans_partial_creation() {
        let paths = TestPaths::new("header-flush-failure");
        inject_test_fault(&paths.shm, TestFaultPoint::StartupHeaderFlush);
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert_eq!(error.primary.operation, "初始化视频共享内存");
        assert!(error.thread_started && error.thread_joined);
        assert!(error.terminal.is_clean());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn shm_prepublication_failures_leave_no_public_path_and_allow_recreate() {
        for (label, point) in [
            ("metadata", TestFaultPoint::ShmMetadata),
            ("try-clone", TestFaultPoint::ShmTryClone),
            ("identity", TestFaultPoint::ShmIdentity),
            ("permissions", TestFaultPoint::ShmPermissions),
        ] {
            let paths = TestPaths::new(&format!("shm-prepublish-{label}"));
            inject_test_fault(&paths.shm, point);
            let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
            assert_eq!(error.primary.operation, "初始化视频共享内存");
            assert!(!paths.shm.exists());
            assert!(error.thread_started && error.thread_joined);
            assert!(error.terminal.is_clean(), "{label}: {error:?}");
            assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);

            let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
            assert!(owner
                .shutdown(Duration::from_secs(1))
                .unwrap()
                .is_expected_shutdown());
            assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
        }
    }

    #[test]
    fn shm_publish_replacement_is_preserved_and_candidate_is_cleaned() {
        let paths = TestPaths::new("shm-publish-replacement");
        inject_test_fault(&paths.shm, TestFaultPoint::ShmPublishReplacement);
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert!(matches!(
            error.primary.kind,
            io::ErrorKind::AlreadyExists | io::ErrorKind::AddrInUse
        ));
        assert_eq!(
            std::fs::read(&paths.shm).unwrap(),
            b"replacement-before-shm-publish"
        );
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 1);
        std::fs::remove_file(&paths.shm).unwrap();

        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        assert!(owner
            .shutdown(Duration::from_secs(1))
            .unwrap()
            .is_expected_shutdown());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }



    #[test]
    fn startup_timeout_stops_joins_and_returns_clean_typed_terminal() {
        let paths = TestPaths::new("startup-timeout");
        inject_test_fault(&paths.shm, TestFaultPoint::StartupTimeout);
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert_eq!(error.primary.kind, io::ErrorKind::TimedOut);
        assert!(error.thread_started && error.thread_joined);
        assert!(error.terminal.is_clean());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn startup_panic_is_joined_and_resources_are_cleaned() {
        let paths = TestPaths::new("startup-panic");
        inject_test_fault(&paths.shm, TestFaultPoint::PanicAfterShm);
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert!(matches!(
            error.terminal.reason,
            VideoRelayTerminalReason::ThreadPanicked(_)
        ));
        assert!(error.thread_started && error.thread_joined);
        assert!(error.terminal.is_clean());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn supervisor_spawn_failure_reaps_helper_and_cleans_all_owned_resources() {
        let paths = TestPaths::new("supervisor-spawn-failure");
        inject_test_fault(&paths.shm, TestFaultPoint::SupervisorSpawn);
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert_eq!(error.primary.operation, "创建视频 helper supervisor");
        assert!(!error.thread_started);
        assert!(error.thread_joined);
        assert!(error.terminal.is_clean(), "{error:?}");
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn parent_try_wait_errors_are_kill_unreaped_in_isolated_processes() {
        run_unknown_wait_child("startup");
        run_unknown_wait_child("supervisor");
    }

    #[test]
    fn helper_cleanup_failure_is_recovered_and_primary_is_retained() {
        let paths = TestPaths::new("cleanup-failure");
        inject_test_fault(&paths.shm, TestFaultPoint::BindUds);
        inject_test_fault(&paths.shm, TestFaultPoint::ShmCleanup);
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert_eq!(error.primary.operation, "绑定视频 UDS");
        assert!(error.thread_started && error.thread_joined);
        assert!(error.terminal.is_clean());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn lstat_to_unlink_replacement_is_restored_and_never_deleted() {
        let paths = TestPaths::new("cleanup-race");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let gate = install_cleanup_gate(&paths.shm);
        let closer = thread::spawn(move || owner.shutdown(Duration::from_secs(1)));
        gate.wait_entered();
        std::fs::remove_file(&paths.shm).unwrap();
        std::fs::write(&paths.shm, b"replacement-during-cleanup").unwrap();
        gate.release();
        let terminal = closer.join().unwrap().unwrap();
        let VideoRelayPathCleanup::Residual { path, .. } = &terminal.cleanup.shm else {
            panic!("未知 replacement 必须报告 Residual: {:?}", terminal.cleanup);
        };
        assert_eq!(path, &paths.shm);
        assert!(!terminal.is_clean());
        assert_eq!(
            std::fs::read(&paths.shm).unwrap(),
            b"replacement-during-cleanup"
        );
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 1);
        std::fs::remove_file(&paths.shm).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn client_protocol_error_is_nonfatal_and_next_client_commits() {
        let paths = TestPaths::new("client-error");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let handle = owner.handle();
        let mut invalid = UnixStream::connect(&paths.uds).unwrap();
        invalid.write_all(&0_u32.to_le_bytes()).unwrap();
        drop(invalid);
        send_frame(&paths.uds, b"valid-frame");
        assert!(wait_until(
            || handle.sequence.load(Ordering::Acquire) == 1,
            Duration::from_secs(1)
        ));
        assert!(owner.shutdown(Duration::from_secs(1)).unwrap().is_clean());
    }

    #[test]
    fn same_connection_reuses_helper_payload_buffer_for_varying_frames_and_stops_cleanly() {
        let paths = TestPaths::new("same-connection-payload-reuse");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let helper_pid = owner.helper_pid();
        let handle = owner.handle();
        let control = owner.helper_control();
        let reader = unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        let generation = mapped_generation(&reader);
        let mut sender = UnixStream::connect(&paths.uds).unwrap();
        let frames = [
            vec![0x11; 192],
            (0_u8..17).collect::<Vec<_>>(),
            vec![0x33; 128],
        ];
        assert!(frames[1].len() < frames[0].len());
        assert!(frames[2].len() > frames[1].len() && frames[2].len() < frames[0].len());

        let mut observations = Vec::new();
        for (index, frame) in frames.iter().enumerate() {
            sender
                .write_all(&(frame.len() as u32).to_le_bytes())
                .unwrap();
            sender.write_all(frame).unwrap();
            let expected_sequence = index as u64 + 1;
            assert!(wait_until(
                || handle.sequence.load(Ordering::Acquire) == expected_sequence,
                Duration::from_secs(1)
            ));
            let (sequence, payload, timestamp_ns) =
                mapped_snapshot(&reader).expect("连续帧必须发布到 SHM");
            assert_eq!(sequence, expected_sequence);
            assert_eq!(payload, *frame);
            assert!(timestamp_ns > 0);
            let observation = process_helper::payload_buffer_observation_for_test(&control);
            assert_eq!(observation.2, expected_sequence);
            observations.push(observation);
        }

        let first = observations[0];
        assert_ne!(first.0, 0);
        assert!(first.1 >= frames[0].len() as u64);
        assert!(
            observations
                .iter()
                .all(|observation| observation.0 == first.0 && observation.1 == first.1),
            "同一 helper 连接的 payload 指针和容量必须跨缩短、再次增长保持复用: {observations:?}"
        );

        assert!(owner.stop());
        let terminal = owner.join(Duration::from_secs(1)).unwrap();
        drop(sender);
        assert!(terminal.is_expected_shutdown(), "{terminal:?}");
        assert!(!process_exists(helper_pid));
        assert_eq!(handle.sequence.load(Ordering::Acquire), 0);
        assert_eq!(mapped_sequence(&reader), 0);
        assert_eq!(mapped_generation(&reader), generation);
        assert!(mapped_snapshot(&reader).is_none());
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn shared_memory_flush_failure_is_fatal_and_remains_the_primary_reason() {
        let paths = TestPaths::new("flush-failure");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let reader = unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        inject_test_fault(&paths.shm, TestFaultPoint::CommitFlush);
        send_frame(&paths.uds, b"frame");
        assert!(wait_until(|| owner.is_finished(), Duration::from_secs(1)));
        let terminal = owner.join(Duration::from_secs(1)).unwrap();
        let VideoRelayTerminalReason::Failed(failure) = &terminal.reason else {
            panic!("刷新失败必须是资源终态");
        };
        assert_eq!(failure.operation, "提交视频共享内存");
        assert_eq!(owner.handle().sequence.load(Ordering::Acquire), 0);
        assert_eq!(u64::from_le_bytes(reader[16..24].try_into().unwrap()), 0);
    }

    #[test]
    fn mmap_reader_rejects_old_sequence_during_failed_commit() {
        let paths = TestPaths::new("mmap-oracle");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let handle = owner.handle();
        let reader = unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        send_frame(&paths.uds, b"old-frame");
        assert!(wait_until(
            || handle.sequence.load(Ordering::Acquire) == 1,
            Duration::from_secs(1)
        ));
        let old = mapped_snapshot(&reader).expect("旧帧已发布");
        let generation = mapped_generation(&reader);

        let oracle_reader =
            unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        let done = Arc::new(AtomicBool::new(false));
        let saw_zero = Arc::new(AtomicBool::new(false));
        let saw_old = Arc::new(AtomicBool::new(false));
        let violation = Arc::new(AtomicBool::new(false));
        let done_for_thread = done.clone();
        let saw_zero_for_thread = saw_zero.clone();
        let saw_old_for_thread = saw_old.clone();
        let violation_for_thread = violation.clone();
        let old_payload = old.1.clone();
        let observer = thread::spawn(move || {
            while !done_for_thread.load(Ordering::Acquire) {
                match mapped_snapshot(&oracle_reader) {
                    Some((sequence, payload, timestamp)) => {
                        saw_old_for_thread.store(true, Ordering::Release);
                        if sequence != old.0 || payload != old_payload || timestamp != old.2 {
                            violation_for_thread.store(true, Ordering::Release);
                        }
                    }
                    None => saw_zero_for_thread.store(true, Ordering::Release),
                }
                thread::yield_now();
            }
        });
        assert!(wait_until(
            || saw_old.load(Ordering::Acquire),
            Duration::from_secs(1)
        ));

        let gate = install_process_gate(
            &owner,
            process_helper::FAULT_PAUSE_AFTER_INVALIDATE,
            process_helper::PHASE_AFTER_INVALIDATE,
        );
        owner
            .helper_control()
            .add_test_fault(process_helper::FAULT_COMMIT_FLUSH);
        let uds = paths.uds.clone();
        let sender = thread::spawn(move || send_frame(&uds, b"new-frame"));
        gate.wait_entered();
        assert_eq!(mapped_sequence(&reader), 0);
        assert!(mapped_snapshot(&reader).is_none());
        gate.release();
        sender.join().unwrap();
        let terminal = owner.join(Duration::from_secs(1)).unwrap();
        done.store(true, Ordering::Release);
        observer.join().unwrap();

        let VideoRelayTerminalReason::Failed(failure) = terminal.reason else {
            panic!("刷新失败必须成为失败终态: {terminal:?}");
        };
        assert_eq!(failure.operation, "提交视频共享内存");
        assert!(!violation.load(Ordering::Acquire));
        assert!(saw_zero.load(Ordering::Acquire));
        assert_eq!(handle.sequence.load(Ordering::Acquire), 0);
        assert_eq!(mapped_sequence(&reader), 0);
        assert_eq!(mapped_generation(&reader), generation);
        assert!(mapped_snapshot(&reader).is_none());
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn parent_terminal_is_hidden_until_handle_sequence_is_zero() {
        let paths = TestPaths::new("terminal-mirror-order");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let handle = owner.handle();
        let reader = unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        send_frame(&paths.uds, b"published-before-terminal");
        assert!(wait_until(
            || handle.sequence.load(Ordering::Acquire) == 1,
            Duration::from_secs(1)
        ));
        let gate = install_process_gate(
            &owner,
            process_helper::FAULT_PAUSE_BEFORE_PARENT_TERMINAL,
            process_helper::PHASE_BEFORE_PARENT_TERMINAL,
        );
        owner
            .helper_control()
            .add_test_fault(process_helper::FAULT_COMMIT_FLUSH);
        send_frame(&paths.uds, b"terminal-trigger");
        gate.wait_entered();
        assert_eq!(handle.sequence.load(Ordering::Acquire), 0);
        assert_eq!(mapped_sequence(&reader), 0);
        assert!(owner.try_terminal().is_none());
        gate.release();
        let terminal = owner.join(Duration::from_secs(1)).unwrap();
        assert!(matches!(
            terminal.reason,
            VideoRelayTerminalReason::Failed(_)
        ));
        assert!(terminal.is_clean());
        assert_eq!(handle.sequence.load(Ordering::Acquire), 0);
        assert!(!paths.shm.exists() && !paths.uds.exists());
    }

    #[test]
    fn final_cleanup_flush_failure_cannot_be_classified_as_expected_shutdown() {
        let paths = TestPaths::new("cleanup-flush-failure");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        inject_test_fault(&paths.shm, TestFaultPoint::CleanupFlush);
        let terminal = owner.shutdown(Duration::from_secs(1)).unwrap();
        let VideoRelayTerminalReason::Failed(failure) = &terminal.reason else {
            panic!("最终刷新失败必须成为失败终态");
        };
        assert_eq!(failure.operation, "回收视频中转资源");
        assert!(!terminal.is_expected_shutdown());
        assert!(!terminal.is_clean());
        assert!(!paths.shm.exists() && !paths.uds.exists());
    }

    #[test]
    fn stop_linearizes_before_blocked_commit_and_prevents_late_publication() {
        let paths = TestPaths::new("stop-commit");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let handle = owner.handle();
        let reader = unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        let gate = install_commit_gate(&paths.shm);
        let uds = paths.uds.clone();
        let sender = thread::spawn(move || send_frame(&uds, b"blocked-frame"));
        gate.wait_entered();
        assert!(owner.stop());
        gate.release();
        sender.join().unwrap();
        let terminal = owner.join(Duration::from_secs(1)).unwrap();
        assert!(terminal.is_expected_shutdown());
        assert_eq!(handle.sequence.load(Ordering::Acquire), 0);
        assert_eq!(u64::from_le_bytes(reader[16..24].try_into().unwrap()), 0);
    }

    #[test]
    fn stop_before_invalidate_clears_prior_sequence() {
        assert_stop_window(
            process_helper::FAULT_PAUSE_BEFORE_INVALIDATE,
            process_helper::PHASE_BEFORE_INVALIDATE,
            1,
            "stop-before-invalidate",
        );
    }

    #[test]
    fn stop_after_invalidate_keeps_sequence_zero() {
        assert_stop_window(
            process_helper::FAULT_PAUSE_AFTER_INVALIDATE,
            process_helper::PHASE_AFTER_INVALIDATE,
            0,
            "stop-after-invalidate",
        );
    }

    #[test]
    fn stop_after_flush_keeps_unpublished_frame_inactive() {
        assert_stop_window(
            process_helper::FAULT_PAUSE_AFTER_FLUSH,
            process_helper::PHASE_AFTER_FLUSH,
            0,
            "stop-after-flush",
        );
    }

    #[test]
    fn msync_hang_is_killed_reaped_and_leaves_zero_sequence() {
        let paths = TestPaths::new("msync-hang");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let handle = owner.handle();
        let reader = unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        let helper_pid = owner.helper_pid();
        inject_test_fault(&paths.shm, TestFaultPoint::MsyncHang);
        send_frame(&paths.uds, b"hang-frame");
        assert!(wait_until(
            || owner.helper_control().helper_phase() == process_helper::PHASE_MSYNC_HANG,
            Duration::from_secs(1)
        ));
        assert_eq!(mapped_sequence(&reader), 0);
        let started = Instant::now();
        let terminal = owner
            .shutdown(Duration::from_millis(800))
            .expect("SIGKILL 后应在期限内 reap helper");
        assert!(started.elapsed() < Duration::from_secs(2));
        assert!(!process_exists(helper_pid));
        assert!(matches!(
            terminal.reason,
            VideoRelayTerminalReason::Failed(_) | VideoRelayTerminalReason::KillUnreaped(_)
        ));
        assert!(terminal.is_clean());
        assert_eq!(handle.sequence.load(Ordering::Acquire), 0);
        assert_eq!(mapped_sequence(&reader), 0);
        assert!(mapped_snapshot(&reader).is_none());
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn malformed_helper_ipc_is_bounded_and_cleans_paths() {
        for (fault, operation, kind, label) in [
            (
                TestFaultPoint::IpcTruncated,
                "等待视频 helper Prepared",
                io::ErrorKind::UnexpectedEof,
                "ipc-truncated",
            ),
            (
                TestFaultPoint::WrongGeneration,
                "验证视频 helper Prepared",
                io::ErrorKind::InvalidData,
                "ipc-wrong-generation",
            ),
        ] {
            let paths = TestPaths::new(label);
            inject_test_fault(&paths.shm, fault);
            let started = Instant::now();
            let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
            assert!(started.elapsed() < Duration::from_secs(3));
            assert_eq!(error.primary.operation, operation);
            assert_eq!(error.primary.kind, kind);
            assert!(error.thread_started && error.thread_joined);
            assert!(error.terminal.is_clean(), "{label}: {:?}", error.terminal);
            assert!(!paths.shm.exists() && !paths.uds.exists());
            let residual = std::fs::read_dir(&paths.root)
                .unwrap()
                .map(|entry| entry.unwrap().file_name())
                .collect::<Vec<_>>();
            assert!(residual.is_empty(), "{label}: residual={residual:?}");
        }
    }

    #[test]
    fn truncated_ipc_with_unreported_shm_cleanup_failure_is_residual_and_preserved() {
        let paths = TestPaths::new("ipc-truncated-shm-residual");
        inject_test_fault(&paths.shm, TestFaultPoint::IpcTruncated);
        inject_test_fault(&paths.shm, TestFaultPoint::ShmCleanup);
        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert_eq!(error.primary.operation, "等待视频 helper Prepared");
        assert!(error.thread_joined);
        let VideoRelayPathCleanup::Residual { path, .. } = &error.terminal.cleanup.shm else {
            panic!("未采纳 SHM 必须报告 Residual: {:?}", error.terminal.cleanup);
        };
        assert_eq!(path, &paths.shm);
        assert!(!error.terminal.is_clean());
        assert!(paths.shm.exists());
        assert!(!paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 1);
        std::fs::remove_file(&paths.shm).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn uds_publish_stop_and_generation_are_linearized_before_and_after_direct_bind() {
        assert_uds_publish_cancel_window(
            TestFaultPoint::PauseBeforeUdsBind,
            process_helper::PHASE_BEFORE_UDS_BIND,
            false,
            false,
            "uds-stop-before-direct-bind",
        );
        assert_uds_publish_cancel_window(
            TestFaultPoint::PauseAfterUdsBind,
            process_helper::PHASE_AFTER_UDS_BIND,
            false,
            true,
            "uds-stop-after-direct-bind",
        );
        assert_uds_publish_cancel_window(
            TestFaultPoint::PauseBeforeUdsBind,
            process_helper::PHASE_BEFORE_UDS_BIND,
            true,
            false,
            "uds-generation-before-direct-bind",
        );
        assert_uds_publish_cancel_window(
            TestFaultPoint::PauseAfterUdsBind,
            process_helper::PHASE_AFTER_UDS_BIND,
            true,
            true,
            "uds-generation-after-direct-bind",
        );
    }

    #[test]
    fn foreign_public_socket_inserted_before_direct_bind_is_preserved_and_residual() {
        let paths = TestPaths::new("foreign-before-direct-bind");
        inject_test_fault(&paths.shm, TestFaultPoint::PauseBeforeUdsBind);
        let shm = paths.shm.clone();
        let uds = paths.uds.clone();
        let starter = thread::spawn(move || spawn_video_relay_owner(&shm, &uds));
        let control = wait_for_helper_control(&paths.shm);
        let gate = TestCommitGate {
            control: control.clone(),
            phase: process_helper::PHASE_BEFORE_UDS_BIND,
        };
        gate.wait_entered();
        assert!(!paths.uds.exists());
        let foreign = UnixListener::bind(&paths.uds).expect("绑定授权后的外来公共 UDS");
        let metadata = std::fs::metadata(&paths.uds).unwrap();
        let foreign_identity = (metadata.dev(), metadata.ino());
        gate.release();
        let error = expect_start_error(starter.join().unwrap());
        assert_eq!(error.primary.operation, "绑定视频 UDS");
        assert_ne!(control.publish_state(), process_helper::PUBLISH_COMPLETE);
        let VideoRelayPathCleanup::Residual { path, .. } = &error.terminal.cleanup.uds else {
            panic!(
                "外来公共 UDS 必须报告 Residual: {:?}",
                error.terminal.cleanup
            );
        };
        assert_eq!(path, &paths.uds);
        assert!(!error.terminal.is_clean());
        let metadata = std::fs::metadata(&paths.uds).unwrap();
        assert_eq!((metadata.dev(), metadata.ino()), foreign_identity);
        assert!(!paths.shm.exists());
        drop(foreign);
        std::fs::remove_file(&paths.uds).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn fixed_candidate_identity_links_to_public_without_leaving_candidate() {
        let paths = TestPaths::new("candidate-link-no-attacker");
        inject_test_fault(&paths.shm, TestFaultPoint::PauseAfterUdsBindBeforeIdentity);
        let shm = paths.shm.clone();
        let uds = paths.uds.clone();
        let (ready_tx, ready_rx) = mpsc::channel();
        let (stop_tx, stop_rx) = mpsc::channel();
        let starter = thread::spawn(move || {
            let mut owner =
                spawn_video_relay_owner(&shm, &uds).expect("固定 candidate 身份应发布成功");
            ready_tx.send(owner.handle()).unwrap();
            stop_rx.recv().unwrap();
            owner.shutdown(Duration::from_secs(1)).unwrap()
        });
        let control = wait_for_helper_control(&paths.shm);
        let candidate = paths
            .root
            .join(process_helper::helper_candidate_name(control.generation()));
        let gate = TestCommitGate {
            control: control.clone(),
            phase: process_helper::PHASE_AFTER_UDS_BIND_BEFORE_IDENTITY,
        };
        gate.wait_entered();
        assert!(candidate.exists());
        assert!(!paths.uds.exists());
        gate.release();

        let handle = ready_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("link publish 后 owner 必须 Ready");
        assert_eq!(control.publish_state(), process_helper::PUBLISH_COMPLETE);
        assert!(!control.stop_requested());
        assert!(!candidate.exists());
        assert!(paths.uds.exists());
        send_frame(&paths.uds, b"candidate-linked-frame");
        assert!(wait_until(
            || handle.sequence.load(Ordering::Acquire) == 1,
            Duration::from_secs(1)
        ));
        stop_tx.send(()).unwrap();
        assert!(starter.join().unwrap().is_clean());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn foreign_candidate_replacement_at_link_gate_is_never_published_or_deleted() {
        let paths = TestPaths::new("foreign-candidate-at-link-gate");
        inject_test_fault(&paths.shm, TestFaultPoint::PauseAfterUdsBindBeforeIdentity);
        let shm = paths.shm.clone();
        let uds = paths.uds.clone();
        let starter = thread::spawn(move || spawn_video_relay_owner(&shm, &uds));
        let control = wait_for_helper_control(&paths.shm);
        let candidate = paths
            .root
            .join(process_helper::helper_candidate_name(control.generation()));
        let gate = TestCommitGate {
            control: control.clone(),
            phase: process_helper::PHASE_AFTER_UDS_BIND_BEFORE_IDENTITY,
        };
        gate.wait_entered();
        assert!(candidate.exists());
        assert!(!paths.uds.exists());
        std::fs::remove_file(&candidate).unwrap();
        let candidate_directory = StableDirectory::open(&paths.root).unwrap();
        let foreign =
            UnixListener::bind(candidate_directory.child_path(candidate.file_name().unwrap()))
                .expect("替换 link gate 的私有 candidate");
        let metadata = std::fs::metadata(&candidate).unwrap();
        let foreign_identity = (metadata.dev(), metadata.ino());
        gate.release();

        let error = expect_start_error(starter.join().unwrap());
        assert_eq!(error.primary.operation, "绑定视频 UDS");
        assert_ne!(control.publish_state(), process_helper::PUBLISH_COMPLETE);
        let VideoRelayPathCleanup::Residual { path, .. } = &error.terminal.cleanup.uds else {
            panic!(
                "candidate replacement 必须报告 Residual: {:?}",
                error.terminal.cleanup
            );
        };
        assert_eq!(path, &candidate);
        assert!(!error.terminal.is_clean());
        assert!(!paths.uds.exists());
        let metadata = std::fs::metadata(&candidate).unwrap();
        assert_eq!((metadata.dev(), metadata.ino()), foreign_identity);
        assert!(!paths.shm.exists());
        drop(foreign);
        std::fs::remove_file(&candidate).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn foreign_replacement_after_final_identity_before_publish_is_preserved() {
        let paths = TestPaths::new("foreign-after-final-identity");
        inject_test_fault(
            &paths.shm,
            TestFaultPoint::PauseAfterUdsIdentityBeforePublish,
        );
        let shm = paths.shm.clone();
        let uds = paths.uds.clone();
        let starter = thread::spawn(move || spawn_video_relay_owner(&shm, &uds));
        let control = wait_for_helper_control(&paths.shm);
        let gate = TestCommitGate {
            control: control.clone(),
            phase: process_helper::PHASE_AFTER_UDS_IDENTITY_BEFORE_PUBLISH,
        };
        gate.wait_entered();
        std::fs::remove_file(&paths.uds).unwrap();
        let foreign = UnixListener::bind(&paths.uds).expect("替换最终身份确认后的公共 UDS");
        let metadata = std::fs::metadata(&paths.uds).unwrap();
        let foreign_identity = (metadata.dev(), metadata.ino());
        gate.release();

        let error = expect_start_error(starter.join().unwrap());
        assert_eq!(error.primary.operation, "绑定视频 UDS");
        assert_ne!(control.publish_state(), process_helper::PUBLISH_COMPLETE);
        let VideoRelayPathCleanup::Residual { path, .. } = &error.terminal.cleanup.uds else {
            panic!(
                "最终身份确认后的 replacement 必须报告 Residual: {:?}",
                error.terminal.cleanup
            );
        };
        assert_eq!(path, &paths.uds);
        assert!(!error.terminal.is_clean());
        let metadata = std::fs::metadata(&paths.uds).unwrap();
        assert_eq!((metadata.dev(), metadata.ino()), foreign_identity);
        drop(foreign);
        std::fs::remove_file(&paths.uds).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn foreign_replacement_after_direct_bind_is_preserved_and_residual() {
        let paths = TestPaths::new("foreign-after-direct-bind");
        inject_test_fault(&paths.shm, TestFaultPoint::PauseAfterUdsBind);
        let shm = paths.shm.clone();
        let uds = paths.uds.clone();
        let starter = thread::spawn(move || spawn_video_relay_owner(&shm, &uds));
        let control = wait_for_helper_control(&paths.shm);
        let gate = TestCommitGate {
            control: control.clone(),
            phase: process_helper::PHASE_AFTER_UDS_BIND,
        };
        gate.wait_entered();
        std::fs::remove_file(&paths.uds).unwrap();
        let foreign = UnixListener::bind(&paths.uds).expect("替换 direct-bind 公共 UDS");
        let metadata = std::fs::metadata(&paths.uds).unwrap();
        let foreign_identity = (metadata.dev(), metadata.ino());
        gate.release();
        let error = expect_start_error(starter.join().unwrap());
        assert_eq!(error.primary.operation, "绑定视频 UDS");
        assert_ne!(control.publish_state(), process_helper::PUBLISH_COMPLETE);
        let VideoRelayPathCleanup::Residual { path, .. } = &error.terminal.cleanup.uds else {
            panic!(
                "外来 replacement 必须报告 Residual: {:?}",
                error.terminal.cleanup
            );
        };
        assert_eq!(path, &paths.uds);
        assert!(!error.terminal.is_clean());
        let metadata = std::fs::metadata(&paths.uds).unwrap();
        assert_eq!((metadata.dev(), metadata.ino()), foreign_identity);
        assert!(!paths.shm.exists());
        drop(foreign);
        std::fs::remove_file(&paths.uds).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn refused_foreign_uds_present_before_start_is_never_deleted() {
        let paths = TestPaths::new("existing-refused-uds");
        let foreign = UnixListener::bind(&paths.uds).expect("创建启动前 foreign UDS");
        let metadata = std::fs::metadata(&paths.uds).unwrap();
        let foreign_identity = (metadata.dev(), metadata.ino());
        drop(foreign);
        assert!(wait_until(
            || match UnixStream::connect(&paths.uds) {
                Ok(stream) => {
                    drop(stream);
                    false
                }
                Err(error) if error.kind() == io::ErrorKind::ConnectionRefused => true,
                Err(error) => panic!("等待 foreign UDS 拒绝连接时失败: {error}"),
            },
            Duration::from_secs(1)
        ));

        let error = expect_start_error(spawn_video_relay_owner(&paths.shm, &paths.uds));
        assert_eq!(error.primary.operation, "绑定视频 UDS");
        assert!(error.thread_started && error.thread_joined);
        assert_eq!(
            error.terminal.cleanup.uds,
            VideoRelayPathCleanup::ReplacementPreserved
        );
        assert!(error.terminal.is_clean());
        let metadata = std::fs::metadata(&paths.uds).unwrap();
        assert_eq!((metadata.dev(), metadata.ino()), foreign_identity);
        assert!(!paths.shm.exists());
        std::fs::remove_file(&paths.uds).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn parent_death_pdeathsig_cleans_helper_and_next_owner_recovers_leases() {
        let paths = TestPaths::new("parent-death");
        let shm_lease = paths.root.join(lease_name(paths.shm.file_name().unwrap()));
        let uds_lease = paths.root.join(lease_name(paths.uds.file_name().unwrap()));
        let executable = std::env::current_exe().unwrap();
        let mut child = Command::new(executable)
            .arg("--exact")
            .arg("video_relay::tests::parent_death_owner_child_entry")
            .arg("--nocapture")
            .arg("--test-threads=1")
            .env("NUWAX_VIDEO_PARENT_DEATH_ROOT_V1", &paths.root)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("启动 parent death owner child");
        assert!(child.wait().unwrap().success());
        let helper_pid = std::fs::read_to_string(paths.root.join("helper.pid"))
            .expect("parent death child 已记录 helper pid")
            .trim()
            .parse::<u32>()
            .unwrap();
        assert!(wait_until(
            || {
                !process_exists(helper_pid)
                    && !paths.shm.exists()
                    && !paths.uds.exists()
                    && shm_lease.exists()
                    && uds_lease.exists()
            },
            Duration::from_secs(3)
        ));
        std::fs::remove_file(paths.root.join("helper.pid")).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 2);

        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds)
            .expect("新 owner 应能在 watchdog 零残留后启动");
        let handle = owner.handle();
        let reader = unsafe { memmap2::Mmap::map(&File::open(&paths.shm).unwrap()).unwrap() };
        assert_ne!(mapped_generation(&reader), [0_u8; 16]);
        send_frame(&paths.uds, b"recovered-frame");
        assert!(wait_until(
            || handle.sequence.load(Ordering::Acquire) == 1,
            Duration::from_secs(1)
        ));
        assert!(owner.shutdown(Duration::from_secs(1)).unwrap().is_clean());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn parent_death_watchdog_sigkills_helper_stuck_in_msync_hang() {
        let paths = TestPaths::new("parent-death-msync-hang");
        let shm_lease = paths.root.join(lease_name(paths.shm.file_name().unwrap()));
        let uds_lease = paths.root.join(lease_name(paths.uds.file_name().unwrap()));
        let started = Instant::now();
        let status = Command::new(std::env::current_exe().unwrap())
            .arg("--exact")
            .arg("video_relay::tests::parent_death_owner_child_entry")
            .arg("--nocapture")
            .arg("--test-threads=1")
            .env("NUWAX_VIDEO_PARENT_DEATH_ROOT_V1", &paths.root)
            .env("NUWAX_VIDEO_PARENT_DEATH_MSYNC_V1", "1")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("启动 parent death + msync owner child");
        assert!(status.success());
        let helper_pid = std::fs::read_to_string(paths.root.join("helper.pid"))
            .expect("parent death + msync child 已记录 helper pid")
            .trim()
            .parse::<u32>()
            .unwrap();
        assert!(wait_until(
            || {
                !process_exists(helper_pid)
                    && !paths.shm.exists()
                    && !paths.uds.exists()
                    && shm_lease.exists()
                    && uds_lease.exists()
            },
            Duration::from_secs(3)
        ));
        assert!(started.elapsed() < Duration::from_secs(4));
        std::fs::remove_file(paths.root.join("helper.pid")).unwrap();
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 2);

        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds)
            .expect("MsyncHang 父死亡后下一 owner 应能复用同路径");
        let helper_pid = owner.helper_pid();
        let handle = owner.handle();
        send_frame(&paths.uds, b"recovered-after-parent-death-msync");
        assert!(wait_until(
            || handle.sequence.load(Ordering::Acquire) == 1,
            Duration::from_secs(1)
        ));
        let started = Instant::now();
        assert!(owner
            .shutdown(Duration::from_secs(1))
            .expect("下一 owner 应有界 shutdown")
            .is_expected_shutdown());
        assert!(started.elapsed() < Duration::from_secs(2));
        assert!(!process_exists(helper_pid));
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn helper_exec_does_not_inherit_non_allowlisted_open_file_descriptions() {
        let paths = TestPaths::new("helper-fd-allowlist");
        let first = create_non_cloexec_memfd("first");
        let first_duplicate = duplicate_fd_non_cloexec(first.as_raw_fd());
        let second = create_non_cloexec_memfd("second");
        let sentinels = [first, first_duplicate, second];
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let helper_pid = owner.helper_pid();
        let helper_descriptors = process_descriptors(helper_pid);
        assert!(!helper_descriptors.is_empty());

        for sentinel in &sentinels {
            assert_eq!(
                unsafe { nix::libc::fcntl(sentinel.as_raw_fd(), nix::libc::F_GETFD) }
                    & nix::libc::FD_CLOEXEC,
                0
            );
            for helper_descriptor in &helper_descriptors {
                assert!(
                    !same_open_file_description_across_processes(
                        sentinel.as_raw_fd(),
                        helper_pid,
                        *helper_descriptor,
                    ),
                    "helper fd {helper_descriptor} 继承了非白名单 parent fd {}",
                    sentinel.as_raw_fd()
                );
            }
        }

        assert!(owner
            .shutdown(Duration::from_secs(1))
            .unwrap()
            .is_expected_shutdown());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn owner_holds_pidfd_ipc_and_path_leases_until_drop() {
        let paths = TestPaths::new("resource-oracle");
        let mut owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let helper_pid = owner.helper_pid();
        let (pidfd, ipc_fd) = owner.helper_fd_numbers();
        assert_ne!(pidfd, ipc_fd);
        let pidfd_reference = duplicate_fd_cloexec(pidfd);
        assert!(same_open_file_description(
            pidfd,
            pidfd_reference.as_raw_fd()
        ));
        let running_pidfd = process_fd_identity(pidfd).expect("owner pidfd 身份可读");
        let running_ipc = process_fd_identity(ipc_fd).expect("owner IPC fd 身份可读");
        assert!(owner.helper_fds_are_live());
        assert!(process_exists(helper_pid));
        assert_eq!(running_pidfd.target, PathBuf::from("anon_inode:[pidfd]"));
        assert!(running_pidfd
            .fdinfo
            .contains(&format!("Pid:\t{helper_pid}")));
        assert!(running_ipc.target.to_string_lossy().starts_with("socket:["));
        assert!(running_pidfd.still_at_original_slot());
        assert!(running_ipc.still_at_original_slot());
        assert!(process_fd_target_count(&running_ipc) >= 2);
        assert!(paths.shm.exists() && paths.uds.exists());
        assert!(paths
            .root
            .join(lease_name(paths.shm.file_name().unwrap()))
            .exists());
        assert!(paths
            .root
            .join(lease_name(paths.uds.file_name().unwrap()))
            .exists());

        assert!(owner
            .shutdown(Duration::from_secs(1))
            .unwrap()
            .is_expected_shutdown());
        assert!(!process_exists(helper_pid));
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
        assert!(owner.helper_fds_are_live());
        assert_eq!(owner.helper_fd_numbers(), (pidfd, ipc_fd));
        let stopped_pidfd = process_fd_identity(pidfd).expect("终态 owner 仍持 pidfd");
        let stopped_ipc = process_fd_identity(ipc_fd).expect("终态 owner 仍持 IPC fd");
        assert!(same_open_file_description(
            pidfd,
            pidfd_reference.as_raw_fd()
        ));
        assert!(running_pidfd.same_kernel_object(&stopped_pidfd));
        assert!(running_ipc.same_kernel_object(&stopped_ipc));
        assert_eq!(process_fd_target_count(&stopped_ipc), 1);

        drop(owner);
        assert!(wait_until(
            || !stopped_pidfd.still_at_original_slot()
                && !stopped_ipc.still_at_original_slot()
                && process_fd_target_count(&stopped_ipc) == 0,
            Duration::from_secs(2)
        ));
        let replacement = reuse_fd_slot_with_pidfd(pidfd, std::process::id());
        assert_eq!(replacement.as_raw_fd(), pidfd);
        assert!(!same_open_file_description(
            pidfd_reference.as_raw_fd(),
            replacement.as_raw_fd()
        ));
        let replacement_identity = process_fd_identity(pidfd).expect("复用 slot 的 pidfd 可读");
        assert!(replacement_identity
            .fdinfo
            .contains(&format!("Pid:\t{}", std::process::id())));
        assert!(!replacement_identity
            .fdinfo
            .contains(&format!("Pid:\t{helper_pid}")));
    }

    #[test]
    fn kill_unreaped_terminal_is_typed_and_not_clean() {
        let residual = || VideoRelayPathCleanup::Residual {
            path: PathBuf::from("/tmp/unreaped-video-node"),
            detail: "helper 未 reap".to_string(),
        };
        let terminal = VideoRelayTerminal {
            reason: VideoRelayTerminalReason::KillUnreaped(VideoRelayFailure::new(
                "强制回收视频 helper",
                io::ErrorKind::TimedOut,
                "SIGKILL 后 helper 未 reap",
            )),
            cleanup: VideoRelayCleanup {
                mmap: VideoRelayMmapCleanup::KillUnreaped,
                uds: residual(),
                shm: residual(),
                uds_lease: residual(),
                shm_lease: residual(),
            },
        };
        assert!(matches!(
            terminal.reason,
            VideoRelayTerminalReason::KillUnreaped(_)
        ));
        assert!(!terminal.cleanup.is_complete());
        assert!(!terminal.is_clean());
        assert!(!terminal.is_expected_shutdown());
    }

    #[test]
    fn partial_read_is_interruptible_and_drop_never_detaches_thread() {
        let paths = TestPaths::new("partial-read");
        let owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let mut sender = UnixStream::connect(&paths.uds).unwrap();
        sender.write_all(&128_u32.to_le_bytes()).unwrap();
        sender.write_all(b"partial").unwrap();
        drop(owner);
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }

    #[test]
    fn drop_waits_for_supervisor_return_after_terminal_is_visible() {
        let paths = TestPaths::new("drop-after-terminal-visible");
        let owner = spawn_video_relay_owner(&paths.shm, &paths.uds).unwrap();
        let helper_pid = owner.helper_pid();
        let terminal_signal = owner.terminal_signal();
        let gate = install_process_gate(
            &owner,
            process_helper::FAULT_PAUSE_AFTER_PARENT_TERMINAL,
            process_helper::PHASE_AFTER_PARENT_TERMINAL,
        );
        let (finished_tx, finished_rx) = mpsc::channel();
        let dropper = thread::spawn(move || {
            drop(owner);
            finished_tx.send(()).unwrap();
        });

        gate.wait_entered();
        let terminal = terminal_signal
            .try_get()
            .expect("after-terminal gate 必须位于终态可见之后");
        assert!(terminal.is_expected_shutdown(), "{terminal:?}");
        assert!(!process_exists(helper_pid));
        assert!(matches!(
            finished_rx.recv_timeout(Duration::from_millis(100)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ));

        gate.release();
        finished_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("释放 gate 后 Drop 必须等到 supervisor 返回");
        dropper.join().unwrap();
        assert!(!paths.shm.exists() && !paths.uds.exists());
        assert_eq!(std::fs::read_dir(&paths.root).unwrap().count(), 0);
    }
}

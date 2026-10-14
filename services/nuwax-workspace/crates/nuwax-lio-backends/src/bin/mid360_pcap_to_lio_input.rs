//! Mid-360 PCAP 到 LIO 输入的受控离线发布入口。

#![deny(unsafe_code)]

use nuwax_lio_backends::mid360_pcap::{
    convert_pcap, ConversionManifest, ConversionRules, WINDOW_NS,
};
use nuwax_lio_backends::{BackendKind, LioBackendSlot, LioInputFrame};
use rustix::fs::{self as rfs, AtFlags, FlockOperation, Mode, OFlags, RenameFlags};
use serde::de::{DeserializeSeed, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use std::cell::Cell;
use std::collections::BTreeMap;
use std::env;
use std::ffi::OsString;
use std::fmt;
use std::fs::{File, Metadata, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::os::fd::OwnedFd;
use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
use std::path::{Path, PathBuf};
use std::rc::Rc;

const O_NOFOLLOW: i32 = 0o400000;
const HASH_BUFFER_BYTES: usize = 1024 * 1024;
const BLIND_RADIUS_MM: u32 = 500;

type AnyResult<T> = Result<T, CliError>;

#[derive(Debug, Clone, Copy)]
enum ErrorKind {
    Argument,
    Limit,
    Identity,
    Conversion,
    Validation,
    Io,
    Conflict,
    Durability,
}
#[derive(Debug)]
struct CliError {
    kind: ErrorKind,
    message: String,
}
impl CliError {
    fn new(kind: ErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
        }
    }
    fn code(&self) -> i32 {
        match self.kind {
            ErrorKind::Argument | ErrorKind::Limit => 2,
            ErrorKind::Identity => 3,
            ErrorKind::Conversion => 4,
            ErrorKind::Validation => 5,
            ErrorKind::Io => 6,
            ErrorKind::Conflict => 7,
            ErrorKind::Durability => 8,
        }
    }
}
impl fmt::Display for CliError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}: {}", self.kind, self.message)
    }
}
impl std::error::Error for CliError {}
impl From<io::Error> for CliError {
    fn from(e: io::Error) -> Self {
        Self::new(ErrorKind::Io, e.to_string())
    }
}
impl From<serde_json::Error> for CliError {
    fn from(e: serde_json::Error) -> Self {
        Self::new(ErrorKind::Validation, e.to_string())
    }
}
impl From<std::num::ParseIntError> for CliError {
    fn from(e: std::num::ParseIntError) -> Self {
        Self::new(ErrorKind::Argument, e.to_string())
    }
}
impl From<std::net::AddrParseError> for CliError {
    fn from(e: std::net::AddrParseError) -> Self {
        Self::new(ErrorKind::Argument, e.to_string())
    }
}
impl From<std::string::FromUtf8Error> for CliError {
    fn from(e: std::string::FromUtf8Error) -> Self {
        Self::new(ErrorKind::Io, e.to_string())
    }
}
impl From<nuwax_lio_backends::mid360_pcap::ConversionError> for CliError {
    fn from(e: nuwax_lio_backends::mid360_pcap::ConversionError) -> Self {
        match e {
            nuwax_lio_backends::mid360_pcap::ConversionError::Io(x) => {
                Self::new(ErrorKind::Conversion, x.to_string())
            }
            nuwax_lio_backends::mid360_pcap::ConversionError::Invalid(x) => {
                Self::new(ErrorKind::Conversion, x)
            }
            nuwax_lio_backends::mid360_pcap::ConversionError::Limit(x) => {
                Self::new(ErrorKind::Limit, x)
            }
        }
    }
}

#[derive(Debug)]
struct Args {
    input: PathBuf,
    input_size: u64,
    input_sha256: String,
    output_parent: PathBuf,
    output_name: String,
    expected_mac: [u8; 6],
    expected_ip: [u8; 4],
    expected_serial: String,
    frame_id: String,
    window_ns: u64,
    blind_radius_mm: u32,
    max_output_bytes: u64,
}

fn main() {
    let code = match run() {
        Ok(()) => 0,
        Err(error) => {
            eprintln!("nuwax-mid360-pcap-to-lio-input: {error}");
            error.code()
        }
    };
    std::process::exit(code);
}

fn run() -> AnyResult<()> {
    let args = parse_args(env::args_os().skip(1))?;
    validate_args(&args)?;
    let executable = OpenOptions::new()
        .read(true)
        .open("/proc/self/exe")
        .map_err(|e| CliError::new(ErrorKind::Identity, e.to_string()))?;
    if !executable
        .metadata()
        .map_err(CliError::from)?
        .file_type()
        .is_file()
    {
        return Err(CliError::new(
            ErrorKind::Identity,
            "execution image is not regular",
        ));
    }
    let binary_sha256 = hash_file(executable)?;

    let parent_fd = rfs::open(
        &args.output_parent,
        OFlags::RDONLY | OFlags::DIRECTORY | OFlags::NOFOLLOW,
        Mode::empty(),
    )
    .map_err(parent_open_error)?;
    let parent_stat = rfs::fstat(&parent_fd).map_err(io_error)?;
    validate_parent(&parent_stat)?;
    rfs::flock(&parent_fd, FlockOperation::NonBlockingLockExclusive).map_err(|error| {
        CliError::new(
            ErrorKind::Conflict,
            format!("output parent is locked: {error}"),
        )
    })?;
    revalidate_parent_path(&args.output_parent, &parent_stat)?;
    let stage_name = random_stage_name()?;
    rfs::mkdirat(
        &parent_fd,
        stage_name.as_str(),
        Mode::RUSR | Mode::WUSR | Mode::XUSR,
    )
    .map_err(io_error)?;
    let stage_fd = rfs::openat(
        &parent_fd,
        stage_name.as_str(),
        OFlags::RDONLY | OFlags::DIRECTORY | OFlags::NOFOLLOW,
        Mode::empty(),
    )
    .map_err(io_error)?;
    let stage_stat = rfs::fstat(&stage_fd).map_err(io_error)?;
    let mut transaction = BundleTransaction::new(
        rustix::io::dup(&parent_fd).map_err(io_error)?,
        stage_name.clone(),
        rustix::io::dup(&stage_fd).map_err(io_error)?,
        stage_stat.st_dev,
        stage_stat.st_ino,
    );

    let mut input = open_read_nofollow(&args.input)?;
    let identity_before = input.metadata().map_err(CliError::from)?;
    validate_regular_identity(&identity_before, args.input_size, "input")?;
    if identity_before.nlink() != 1 {
        return Err(CliError::new(
            ErrorKind::Identity,
            "input must have exactly one hard link",
        ));
    }
    if hash_reader(&mut input).map_err(CliError::from)? != args.input_sha256 {
        return Err(CliError::new(ErrorKind::Identity, "input SHA-256 mismatch"));
    }

    let (output_fd, output_identity) = create_owned_member(&stage_fd, "lio-input.json")?;
    transaction.members[0] = Some(output_identity);
    let output_file = File::from(output_fd);
    let writer_failure = Rc::new(Cell::new(None));
    let mut writer = HashingWriter::new(output_file, args.max_output_bytes, writer_failure.clone());
    writer
        .write_all(b"[")
        .map_err(|e| writer_error(&writer_failure, e))?;
    let mut first = true;
    input.seek(SeekFrom::Start(0)).map_err(CliError::from)?;
    let rules = ConversionRules {
        expected_mac: args.expected_mac,
        expected_ip: args.expected_ip,
        expected_serial: args.expected_serial.clone(),
        frame_id: args.frame_id.clone(),
    };
    let converted = convert_pcap(&mut input, &rules, |frame| {
        if !first {
            writer.write_all(b",")?;
        }
        first = false;
        if let Err(error) = serde_json::to_writer(&mut writer, &frame) {
            if writer_failure.get().is_none() {
                writer_failure.set(Some(ErrorKind::Validation));
            }
            return Err(nuwax_lio_backends::mid360_pcap::ConversionError::Invalid(
                error.to_string(),
            ));
        }
        Ok(())
    });
    let mut manifest = match converted {
        Ok(value) => value,
        Err(error) => {
            return Err(if let Some(kind) = writer_failure.get() {
                CliError::new(kind, error.to_string())
            } else {
                CliError::from(error)
            })
        }
    };
    writer
        .write_all(b"]\n")
        .map_err(|error| writer_error(&writer_failure, error))?;
    let (output_file, output_size, output_hash) = writer.finish().map_err(CliError::from)?;
    output_file.sync_all().map_err(durability_error)?;
    drop(output_file);

    let output_check = File::from(open_member(&stage_fd, "lio-input.json")?);
    let output_metadata = output_check.metadata().map_err(CliError::from)?;
    validate_regular_identity(&output_metadata, output_size, "staged output")?;
    if output_metadata.nlink() != 1 {
        return Err(CliError::new(
            ErrorKind::Identity,
            "staged output link count changed",
        ));
    }
    let validation = validate_output(output_check, &manifest)?;
    input.seek(SeekFrom::Start(0)).map_err(CliError::from)?;
    if hash_reader(&mut input).map_err(CliError::from)? != args.input_sha256
        || !same_identity(&identity_before, &input.metadata().map_err(CliError::from)?)
    {
        return Err(CliError::new(ErrorKind::Identity, "input identity changed"));
    }

    let external = &mut manifest.external_provenance;
    external.input.basename = args
        .input
        .file_name()
        .map(|x| x.to_string_lossy().into_owned());
    external.input.size_bytes = Some(args.input_size);
    external.input.sha256 = Some(args.input_sha256.clone());
    external.input.fd_device = Some(identity_before.dev());
    external.input.fd_inode = Some(identity_before.ino());
    external.input.fd_link_count = Some(identity_before.nlink());
    external.output.size_bytes = Some(output_size);
    external.output.sha256 = Some(output_hash.clone());
    external.output.max_output_bytes = Some(args.max_output_bytes);
    external.output.deserialized_frames = Some(validation.frames);
    external.validation.fastlio2_frames_processed = Some(validation.frames);
    external.validation.fastlio2_contract_passed = Some(true);
    external.validation.fastlio2_healthy_frames = Some(validation.fastlio2_healthy);
    external.validation.fastlio2_unhealthy_frames =
        Some(validation.frames - validation.fastlio2_healthy);
    external.validation.pointlio_frames_processed = Some(validation.frames);
    external.validation.pointlio_contract_passed = Some(true);
    external.validation.pointlio_healthy_frames = Some(validation.pointlio_healthy);
    external.validation.pointlio_unhealthy_frames =
        Some(validation.frames - validation.pointlio_healthy);
    external.tool.package_version = Some(env!("CARGO_PKG_VERSION").into());
    external.tool.binary_sha256 = Some(binary_sha256);
    external.tool.rust_version = Some(env!("NUWAX_BUILD_RUSTC_RELEASE").into());
    external.tool.target_triple = Some(env!("NUWAX_BUILD_TARGET").into());
    let (provenance_fd, provenance_identity) = create_owned_member(&stage_fd, "provenance.json")?;
    transaction.members[1] = Some(provenance_identity);
    let mut provenance_file = File::from(provenance_fd);
    write_canonical_manifest(&mut provenance_file, &manifest)?;
    revalidate_member(
        &stage_fd,
        "lio-input.json",
        output_identity,
        output_size,
        &output_hash,
    )?;
    let manifest_hash = hash_member(&stage_fd, "provenance.json")?;
    let manifest_size = rfs::fstat(&open_member(&stage_fd, "provenance.json")?)
        .map_err(io_error)?
        .st_size as u64;
    revalidate_member(
        &stage_fd,
        "provenance.json",
        provenance_identity,
        manifest_size,
        &manifest_hash,
    )?;
    rfs::fsync(&stage_fd).map_err(durability_error)?;
    verify_stage_entry(&parent_fd, &stage_name, &stage_stat)?;
    revalidate_parent_path(&args.output_parent, &parent_stat)?;
    rfs::renameat_with(
        &parent_fd,
        stage_name.as_str(),
        &parent_fd,
        args.output_name.as_str(),
        RenameFlags::NOREPLACE,
    )
    .map_err(|e| {
        if e == rustix::io::Errno::EXIST {
            CliError::new(ErrorKind::Conflict, e.to_string())
        } else {
            io_error(e)
        }
    })?;
    transaction.set_final(args.output_name.clone());
    if let Err(error) = rfs::fsync(&parent_fd) {
        let state = transaction.rollback();
        eprintln!("warning: durability rollback state: {state:?}");
        return Err(durability_error(error));
    }
    transaction.committed = true;
    if let Err(error) = revalidate_parent_path(&args.output_parent, &parent_stat) {
        eprintln!(
            "warning: durable bundle published but output parent path drifted: {}",
            error.kind as u8
        );
    }
    Ok(())
}
fn usage() -> &'static str {
    "required: --input PATH --input-size U64 --input-sha256 HEX --output-parent DIR --output-name NAME --expected-mac MAC --expected-ip IPV4 --expected-serial SERIAL --frame-id FRAME --window-ns U64 --blind-radius-mm U32 --max-output-bytes U64"
}

fn parse_args<I>(args: I) -> AnyResult<Args>
where
    I: IntoIterator<Item = OsString>,
{
    let mut values = BTreeMap::<String, OsString>::new();
    let mut iter = args.into_iter();
    while let Some(flag) = iter.next() {
        let flag = flag
            .into_string()
            .map_err(|_| CliError::new(ErrorKind::Argument, "argument name is not UTF-8"))?;
        if !flag.starts_with("--") || values.contains_key(&flag) {
            return Err(CliError::new(ErrorKind::Argument, usage()));
        }
        let value = iter
            .next()
            .ok_or_else(|| CliError::new(ErrorKind::Argument, usage()))?;
        values.insert(flag, value);
    }
    fn take(values: &mut BTreeMap<String, OsString>, name: &str) -> AnyResult<OsString> {
        values
            .remove(name)
            .ok_or_else(|| CliError::new(ErrorKind::Argument, format!("missing {name}")))
    }
    fn text(value: OsString, name: &str) -> AnyResult<String> {
        value
            .into_string()
            .map_err(|_| CliError::new(ErrorKind::Argument, format!("{name} is not UTF-8")))
    }
    let input = PathBuf::from(take(&mut values, "--input")?);
    let input_size = text(take(&mut values, "--input-size")?, "--input-size")?.parse()?;
    let input_sha256 = text(take(&mut values, "--input-sha256")?, "--input-sha256")?;
    let output_parent = PathBuf::from(take(&mut values, "--output-parent")?);
    let output_name = text(take(&mut values, "--output-name")?, "--output-name")?;
    let expected_mac = parse_mac(&text(
        take(&mut values, "--expected-mac")?,
        "--expected-mac",
    )?)?;
    let expected_ip = text(take(&mut values, "--expected-ip")?, "--expected-ip")?
        .parse::<std::net::Ipv4Addr>()?
        .octets();
    let expected_serial = text(take(&mut values, "--expected-serial")?, "--expected-serial")?;
    let frame_id = text(take(&mut values, "--frame-id")?, "--frame-id")?;
    let window_ns = text(take(&mut values, "--window-ns")?, "--window-ns")?.parse()?;
    let blind_radius_mm =
        text(take(&mut values, "--blind-radius-mm")?, "--blind-radius-mm")?.parse()?;
    let max_output_bytes = text(
        take(&mut values, "--max-output-bytes")?,
        "--max-output-bytes",
    )?
    .parse()?;
    if !values.is_empty() {
        return Err(CliError::new(ErrorKind::Argument, usage()));
    }
    Ok(Args {
        input,
        input_size,
        input_sha256,
        output_parent,
        output_name,
        expected_mac,
        expected_ip,
        expected_serial,
        frame_id,
        window_ns,
        blind_radius_mm,
        max_output_bytes,
    })
}

fn validate_args(args: &Args) -> AnyResult<()> {
    if args.input_size == 0 || args.max_output_bytes == 0 {
        return Err(CliError::new(
            ErrorKind::Argument,
            "size limits must be non-zero",
        ));
    }
    if args.input_sha256.len() != 64
        || !args
            .input_sha256
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(CliError::new(
            ErrorKind::Argument,
            "--input-sha256 must be 64 lowercase hexadecimal characters",
        ));
    }
    if args.output_name.is_empty()
        || args.output_name == "."
        || args.output_name == ".."
        || args.output_name.contains('/')
        || args.output_name.as_bytes().contains(&0)
    {
        return Err(CliError::new(
            ErrorKind::Argument,
            "--output-name must be one safe path component",
        ));
    }
    if args.window_ns != WINDOW_NS {
        return Err(CliError::new(
            ErrorKind::Argument,
            format!("--window-ns must equal frozen value {WINDOW_NS}"),
        ));
    }
    if args.blind_radius_mm != BLIND_RADIUS_MM {
        return Err(CliError::new(
            ErrorKind::Argument,
            format!("--blind-radius-mm must equal frozen value {BLIND_RADIUS_MM}"),
        ));
    }
    ConversionRules {
        expected_mac: args.expected_mac,
        expected_ip: args.expected_ip,
        expected_serial: args.expected_serial.clone(),
        frame_id: args.frame_id.clone(),
    }
    .validate()
    .map_err(|error| CliError::new(ErrorKind::Argument, error.to_string()))?;
    Ok(())
}

fn parse_mac(value: &str) -> AnyResult<[u8; 6]> {
    let parts = value.split(':').collect::<Vec<_>>();
    if parts.len() != 6 || parts.iter().any(|part| part.len() != 2) {
        return Err(CliError::new(
            ErrorKind::Argument,
            "--expected-mac must use xx:xx:xx:xx:xx:xx",
        ));
    }
    let mut mac = [0_u8; 6];
    for (slot, part) in mac.iter_mut().zip(parts) {
        *slot = u8::from_str_radix(part, 16)?;
    }
    Ok(mac)
}

fn open_read_nofollow(path: &Path) -> io::Result<File> {
    OpenOptions::new()
        .read(true)
        .custom_flags(O_NOFOLLOW)
        .open(path)
}

fn validate_regular_identity(metadata: &Metadata, size: u64, label: &str) -> AnyResult<()> {
    if !metadata.file_type().is_file() || metadata.len() != size {
        return Err(CliError::new(
            ErrorKind::Identity,
            format!("{label} is not the pinned regular file"),
        ));
    }
    Ok(())
}

fn same_identity(left: &Metadata, right: &Metadata) -> bool {
    left.file_type().is_file()
        && right.file_type().is_file()
        && left.dev() == right.dev()
        && left.ino() == right.ino()
        && left.len() == right.len()
        && left.nlink() == right.nlink()
}

fn parent_open_error(error: rustix::io::Errno) -> CliError {
    match error {
        rustix::io::Errno::LOOP | rustix::io::Errno::NOTDIR => {
            CliError::new(ErrorKind::Identity, error.to_string())
        }
        _ => io_error(error),
    }
}
fn io_error(error: rustix::io::Errno) -> CliError {
    CliError::new(ErrorKind::Io, error.to_string())
}
fn durability_error(error: impl fmt::Display) -> CliError {
    CliError::new(ErrorKind::Durability, error.to_string())
}
fn create_owned_member(stage: &OwnedFd, name: &str) -> AnyResult<(OwnedFd, MemberIdentity)> {
    if member_index(name).is_none() {
        return Err(CliError::new(
            ErrorKind::Identity,
            "member outside creation allowlist",
        ));
    }
    let fd = rfs::openat(
        stage,
        name,
        OFlags::WRONLY | OFlags::CREATE | OFlags::EXCL | OFlags::NOFOLLOW,
        Mode::RUSR | Mode::WUSR,
    )
    .map_err(io_error)?;
    let identity = match rfs::fstat(&fd) {
        Ok(stat) if rfs::FileType::from_raw_mode(stat.st_mode).is_file() && stat.st_nlink == 1 => {
            MemberIdentity {
                file_type: rfs::FileType::from_raw_mode(stat.st_mode),
                dev: stat.st_dev,
                ino: stat.st_ino,
                nlink: u64::from(stat.st_nlink),
            }
        }
        result => {
            let original = match result {
                Err(error) => CliError::new(
                    ErrorKind::Identity,
                    format!("created member identity acquisition failed: {error}"),
                ),
                Ok(_) => CliError::new(ErrorKind::Identity, "created member identity invalid"),
            };
            let cleanup = rfs::unlinkat(stage, name, AtFlags::empty());
            let sync = rfs::fsync(stage);
            return Err(match (cleanup, sync) {
                (Ok(()), Ok(())) => original,
                (cleanup, sync) => CliError::new(
                    original.kind,
                    format!(
                        "{}; best-effort cleanup={cleanup:?}; stage fsync={sync:?}",
                        original.message
                    ),
                ),
            });
        }
    };
    Ok((fd, identity))
}
fn open_member(stage: &OwnedFd, name: &str) -> AnyResult<OwnedFd> {
    rfs::openat(
        stage,
        name,
        OFlags::RDONLY | OFlags::NOFOLLOW,
        Mode::empty(),
    )
    .map_err(io_error)
}
fn random_stage_name() -> AnyResult<String> {
    let mut file = OpenOptions::new()
        .read(true)
        .open("/dev/urandom")
        .map_err(CliError::from)?;
    let mut bytes = [0u8; 16];
    file.read_exact(&mut bytes).map_err(CliError::from)?;
    Ok(format!(".mid360-lio-{}.staging", hex(&bytes)))
}
fn revalidate_parent_path(path: &Path, expected: &rfs::Stat) -> AnyResult<()> {
    let fd = rfs::open(
        path,
        OFlags::RDONLY | OFlags::DIRECTORY | OFlags::NOFOLLOW,
        Mode::empty(),
    )
    .map_err(io_error)?;
    let actual = rfs::fstat(fd).map_err(io_error)?;
    if actual.st_dev != expected.st_dev || actual.st_ino != expected.st_ino {
        return Err(CliError::new(
            ErrorKind::Identity,
            "output parent path drifted",
        ));
    }
    Ok(())
}
fn verify_stage_entry(parent: &OwnedFd, name: &str, expected: &rfs::Stat) -> AnyResult<()> {
    let actual = rfs::statat(parent, name, AtFlags::SYMLINK_NOFOLLOW).map_err(io_error)?;
    if actual.st_dev != expected.st_dev || actual.st_ino != expected.st_ino {
        return Err(CliError::new(
            ErrorKind::Identity,
            "staging entry identity changed",
        ));
    }
    Ok(())
}
#[derive(Clone, Copy, Debug)]
enum EntryState {
    Stage,
    Final,
    Foreign,
}
#[derive(Clone, Copy)]
struct MemberIdentity {
    file_type: rfs::FileType,
    dev: u64,
    ino: u64,
    nlink: u64,
}
struct BundleTransaction {
    parent: OwnedFd,
    stage_fd: OwnedFd,
    stage: String,
    final_name: String,
    dev: u64,
    ino: u64,
    members: [Option<MemberIdentity>; 2],
    committed: bool,
}
impl BundleTransaction {
    fn new(parent: OwnedFd, stage: String, stage_fd: OwnedFd, dev: u64, ino: u64) -> Self {
        Self {
            parent,
            stage_fd,
            stage,
            final_name: String::new(),
            dev,
            ino,
            members: [None, None],
            committed: false,
        }
    }
    fn identity_matches(&self, name: &str) -> bool {
        entry_matches(&self.parent, name, self.dev, self.ino)
    }
    fn set_final(&mut self, name: String) {
        self.final_name = name;
    }
    fn rollback(&mut self) -> EntryState {
        if self.final_name.is_empty() {
            return if self.identity_matches(&self.stage) {
                EntryState::Stage
            } else {
                EntryState::Foreign
            };
        }
        if !self.identity_matches(&self.final_name) {
            return EntryState::Foreign;
        }
        if rfs::renameat_with(
            &self.parent,
            self.final_name.as_str(),
            &self.parent,
            self.stage.as_str(),
            RenameFlags::NOREPLACE,
        )
        .is_ok()
        {
            self.final_name.clear();
            let _ = rfs::fsync(&self.parent);
            EntryState::Stage
        } else if self.identity_matches(&self.final_name) {
            EntryState::Final
        } else {
            EntryState::Foreign
        }
    }
    fn clean_members(&mut self) {
        for (name, index) in [("lio-input.json", 0usize), ("provenance.json", 1usize)] {
            let Some(expected) = self.members[index] else {
                continue;
            };
            match rfs::statat(&self.stage_fd, name, AtFlags::SYMLINK_NOFOLLOW) {
                Err(rustix::io::Errno::NOENT) => {}
                Ok(actual)
                    if rfs::FileType::from_raw_mode(actual.st_mode) == expected.file_type
                        && actual.st_dev == expected.dev
                        && actual.st_ino == expected.ino
                        && u64::from(actual.st_nlink) == expected.nlink =>
                {
                    let _ = rfs::unlinkat(&self.stage_fd, name, AtFlags::empty());
                }
                _ => eprintln!("warning: retained identity-mismatched transaction member"),
            }
        }
        let _ = rfs::fsync(&self.stage_fd);
    }
}
impl Drop for BundleTransaction {
    fn drop(&mut self) {
        if self.committed {
            return;
        }
        self.clean_members();
        let name = if self.final_name.is_empty() {
            &self.stage
        } else {
            &self.final_name
        };
        if self.identity_matches(name) {
            if let Err(error) = rfs::unlinkat(&self.parent, name.as_str(), AtFlags::REMOVEDIR) {
                eprintln!("warning: retained transaction directory: {error}");
            }
        } else {
            eprintln!("warning: retained identity-mismatched transaction directory");
        }
        let _ = rfs::fsync(&self.parent);
    }
}
fn member_index(name: &str) -> Option<usize> {
    match name {
        "lio-input.json" => Some(0),
        "provenance.json" => Some(1),
        _ => None,
    }
}
fn entry_matches(parent: &OwnedFd, name: &str, dev: u64, ino: u64) -> bool {
    rfs::statat(parent, name, AtFlags::SYMLINK_NOFOLLOW)
        .map(|stat| stat.st_dev == dev && stat.st_ino == ino)
        .unwrap_or(false)
}
fn effective_uid() -> AnyResult<u32> {
    let mut text = String::new();
    OpenOptions::new()
        .read(true)
        .open("/proc/self/status")
        .map_err(CliError::from)?
        .read_to_string(&mut text)
        .map_err(CliError::from)?;
    let line = text
        .lines()
        .find(|line| line.starts_with("Uid:"))
        .ok_or_else(|| CliError::new(ErrorKind::Identity, "effective uid unavailable"))?;
    line.split_whitespace()
        .nth(2)
        .ok_or_else(|| CliError::new(ErrorKind::Identity, "effective uid unavailable"))?
        .parse::<u32>()
        .map_err(|e| CliError::new(ErrorKind::Identity, e.to_string()))
}
fn validate_parent(stat: &rfs::Stat) -> AnyResult<()> {
    let mode = stat.st_mode & 0o7777;
    if !rfs::FileType::from_raw_mode(stat.st_mode).is_dir() {
        return Err(CliError::new(
            ErrorKind::Identity,
            "output parent is not a directory",
        ));
    }
    let euid = effective_uid()?;
    if stat.st_uid != euid {
        return Err(CliError::new(
            ErrorKind::Identity,
            "output parent owner does not match effective uid",
        ));
    }
    if mode != 0o700 {
        return Err(CliError::new(
            ErrorKind::Identity,
            "output parent mode must be exactly 0700",
        ));
    }
    Ok(())
}
fn write_canonical_manifest(file: &mut File, manifest: &ConversionManifest) -> AnyResult<()> {
    let value = canonicalize_value(serde_json::to_value(manifest)?);
    serde_json::to_writer(&mut *file, &value)?;
    file.write_all(b"\n").map_err(CliError::from)?;
    file.sync_all().map_err(durability_error)
}
fn canonicalize_value(value: serde_json::Value) -> serde_json::Value {
    match value {
        serde_json::Value::Array(v) => {
            serde_json::Value::Array(v.into_iter().map(canonicalize_value).collect())
        }
        serde_json::Value::Object(v) => {
            let mut e = v.into_iter().collect::<Vec<_>>();
            e.sort_by(|a, b| a.0.as_bytes().cmp(b.0.as_bytes()));
            let mut m = serde_json::Map::new();
            for (k, v) in e {
                m.insert(k, canonicalize_value(v));
            }
            serde_json::Value::Object(m)
        }
        x => x,
    }
}
fn open_member_for_revalidation(stage: &OwnedFd, name: &str) -> AnyResult<File> {
    let entry = match rfs::statat(stage, name, AtFlags::SYMLINK_NOFOLLOW) {
        Ok(value) => value,
        Err(error) if error == rustix::io::Errno::NOENT || error == rustix::io::Errno::NOTDIR => {
            return member_identity_error(name, "type mismatch");
        }
        Err(error) => return Err(io_error(error)),
    };
    if !rfs::FileType::from_raw_mode(entry.st_mode).is_file() {
        return member_identity_error(name, "type mismatch");
    }
    let fd = match rfs::openat(
        stage,
        name,
        OFlags::RDONLY | OFlags::NOFOLLOW | OFlags::NONBLOCK,
        Mode::empty(),
    ) {
        Ok(value) => value,
        Err(error)
            if error == rustix::io::Errno::LOOP
                || error == rustix::io::Errno::NOENT
                || error == rustix::io::Errno::NOTDIR =>
        {
            return member_identity_error(name, "type mismatch");
        }
        Err(error) => return Err(io_error(error)),
    };
    Ok(File::from(fd))
}

fn revalidate_member(
    stage: &OwnedFd,
    name: &str,
    expected: MemberIdentity,
    size: u64,
    hash: &str,
) -> AnyResult<()> {
    let mut file = open_member_for_revalidation(stage, name)?;
    let metadata = file.metadata().map_err(CliError::from)?;
    if !metadata.file_type().is_file() || !expected.file_type.is_file() {
        return member_identity_error(name, "type mismatch");
    }
    if metadata.dev() != expected.dev || metadata.ino() != expected.ino {
        return member_identity_error(name, "device/inode mismatch");
    }
    if metadata.nlink() != expected.nlink || metadata.nlink() != 1 {
        return member_identity_error(name, "link-count mismatch");
    }
    if metadata.len() != size {
        return member_identity_error(name, "size mismatch");
    }
    if hash_reader(&mut file).map_err(CliError::from)? != hash {
        return member_identity_error(name, "SHA-256 mismatch");
    }
    Ok(())
}
fn member_identity_error<T>(name: &str, category: &str) -> AnyResult<T> {
    Err(CliError::new(
        ErrorKind::Identity,
        format!("{name}: {category}"),
    ))
}
fn hash_member(stage: &OwnedFd, name: &str) -> AnyResult<String> {
    let mut f = File::from(open_member(stage, name)?);
    hash_reader(&mut f).map_err(CliError::from)
}
fn hash_file(mut file: File) -> AnyResult<String> {
    hash_reader(&mut file).map_err(CliError::from)
}

fn hash_reader<R: Read>(reader: &mut R) -> io::Result<String> {
    let mut hash = Sha256::new();
    let mut buffer = vec![0_u8; HASH_BUFFER_BYTES];
    loop {
        let count = reader.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        hash.update(&buffer[..count]);
    }
    Ok(hex(&hash.finish()))
}

struct HashingWriter<W> {
    inner: W,
    hash: Sha256,
    bytes: u64,
    limit: u64,
    failure: Rc<Cell<Option<ErrorKind>>>,
}
impl<W: Write> HashingWriter<W> {
    fn new(inner: W, limit: u64, failure: Rc<Cell<Option<ErrorKind>>>) -> Self {
        Self {
            inner,
            hash: Sha256::new(),
            bytes: 0,
            limit,
            failure,
        }
    }
    fn finish(self) -> io::Result<(W, u64, String)> {
        Ok((self.inner, self.bytes, hex(&self.hash.finish())))
    }
}
impl<W: Write> Write for HashingWriter<W> {
    fn write(&mut self, buffer: &[u8]) -> io::Result<usize> {
        let requested = match u64::try_from(buffer.len()) {
            Ok(value) => value,
            Err(_) => {
                self.failure.set(Some(ErrorKind::Limit));
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "write length overflow",
                ));
            }
        };
        let next = match self.bytes.checked_add(requested) {
            Some(value) => value,
            None => {
                self.failure.set(Some(ErrorKind::Limit));
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "output length overflow",
                ));
            }
        };
        if next > self.limit {
            self.failure.set(Some(ErrorKind::Limit));
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "output byte limit exceeded",
            ));
        }
        if let Err(error) = self.inner.write_all(buffer) {
            self.failure.set(Some(ErrorKind::Io));
            return Err(error);
        }
        self.hash.update(buffer);
        self.bytes = next;
        Ok(buffer.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

fn writer_error(state: &Rc<Cell<Option<ErrorKind>>>, error: io::Error) -> CliError {
    CliError::new(state.get().unwrap_or(ErrorKind::Io), error.to_string())
}

#[derive(Default)]
struct ValidationResult {
    frames: u64,
    points: u64,
    imu: u64,
    first_timestamp_ns: Option<i64>,
    last_timestamp_ns: Option<i64>,
    fastlio2_healthy: u64,
    pointlio_healthy: u64,
}
struct FrameSeed<'a> {
    result: &'a mut ValidationResult,
    fast: &'a mut LioBackendSlot,
    point: &'a mut LioBackendSlot,
}
impl<'de> DeserializeSeed<'de> for FrameSeed<'_> {
    type Value = ();
    fn deserialize<D>(self, deserializer: D) -> Result<(), D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let frame = LioInputFrame::deserialize(deserializer)?;
        let fast = self
            .fast
            .process(&frame)
            .map_err(serde::de::Error::custom)?;
        let point = self
            .point
            .process(&frame)
            .map_err(serde::de::Error::custom)?;
        self.result.frames = self
            .result
            .frames
            .checked_add(1)
            .ok_or_else(|| serde::de::Error::custom("frame count overflow"))?;
        let points = u64::try_from(frame.cloud.points.len()).map_err(serde::de::Error::custom)?;
        let imu = u64::try_from(frame.imu.len()).map_err(serde::de::Error::custom)?;
        self.result.points = self
            .result
            .points
            .checked_add(points)
            .ok_or_else(|| serde::de::Error::custom("point count overflow"))?;
        self.result.imu = self
            .result
            .imu
            .checked_add(imu)
            .ok_or_else(|| serde::de::Error::custom("IMU count overflow"))?;
        if self.result.first_timestamp_ns.is_none() {
            self.result.first_timestamp_ns = Some(frame.cloud.timestamp_ns);
        }
        self.result.last_timestamp_ns = Some(frame.cloud.timestamp_ns);
        if fast.health.healthy {
            self.result.fastlio2_healthy += 1;
        }
        if point.health.healthy {
            self.result.pointlio_healthy += 1;
        }
        Ok(())
    }
}
struct FramesVisitor<'a> {
    result: &'a mut ValidationResult,
    fast: &'a mut LioBackendSlot,
    point: &'a mut LioBackendSlot,
}
impl<'de> Visitor<'de> for FramesVisitor<'_> {
    type Value = ();
    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("a LIO frame array")
    }
    fn visit_seq<A>(self, mut sequence: A) -> Result<(), A::Error>
    where
        A: SeqAccess<'de>,
    {
        while sequence
            .next_element_seed(FrameSeed {
                result: self.result,
                fast: self.fast,
                point: self.point,
            })?
            .is_some()
        {}
        Ok(())
    }
}
fn validate_output(file: File, manifest: &ConversionManifest) -> AnyResult<ValidationResult> {
    let mut fast = LioBackendSlot::new();
    let mut point = LioBackendSlot::new();
    fast.start(BackendKind::FastLio2)
        .map_err(|e| CliError::new(ErrorKind::Validation, e))?;
    point
        .start(BackendKind::PointLio)
        .map_err(|e| CliError::new(ErrorKind::Validation, e))?;
    let mut result = ValidationResult::default();
    let mut deserializer = serde_json::Deserializer::from_reader(file);
    deserializer.deserialize_seq(FramesVisitor {
        result: &mut result,
        fast: &mut fast,
        point: &mut point,
    })?;
    deserializer.end()?;
    let expected = &manifest.statistics.windows;
    if result.frames != expected.emitted
        || result.points != expected.total_points
        || result.imu != expected.total_imu
        || result.first_timestamp_ns != expected.first_timestamp_ns
        || result.last_timestamp_ns != expected.last_timestamp_ns
    {
        return Err(CliError::new(
            ErrorKind::Validation,
            "deserialized output statistics do not match converter statistics",
        ));
    }
    Ok(result)
}

fn hex(bytes: &[u8]) -> String {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(DIGITS[(byte >> 4) as usize] as char);
        output.push(DIGITS[(byte & 15) as usize] as char);
    }
    output
}

#[derive(Clone)]
struct Sha256 {
    state: [u32; 8],
    buffer: [u8; 64],
    buffered: usize,
    length: u64,
}
impl Sha256 {
    fn new() -> Self {
        Self {
            state: [
                0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
                0x5be0cd19,
            ],
            buffer: [0; 64],
            buffered: 0,
            length: 0,
        }
    }
    fn update(&mut self, mut input: &[u8]) {
        self.length = self
            .length
            .checked_add(input.len() as u64)
            .expect("SHA-256 input length overflow");
        if self.buffered != 0 {
            let count = (64 - self.buffered).min(input.len());
            self.buffer[self.buffered..self.buffered + count].copy_from_slice(&input[..count]);
            self.buffered += count;
            input = &input[count..];
            if self.buffered != 64 {
                return;
            }
            let block = self.buffer;
            self.compress(&block);
            self.buffered = 0;
        }
        while input.len() >= 64 {
            let mut block = [0; 64];
            block.copy_from_slice(&input[..64]);
            self.compress(&block);
            input = &input[64..];
        }
        self.buffer[..input.len()].copy_from_slice(input);
        self.buffered = input.len();
    }
    fn finish(mut self) -> [u8; 32] {
        let bit_length = self
            .length
            .checked_mul(8)
            .expect("SHA-256 bit length overflow");
        self.buffer[self.buffered] = 0x80;
        self.buffered += 1;
        if self.buffered > 56 {
            self.buffer[self.buffered..].fill(0);
            let block = self.buffer;
            self.compress(&block);
            self.buffered = 0;
        }
        self.buffer[self.buffered..56].fill(0);
        self.buffer[56..64].copy_from_slice(&bit_length.to_be_bytes());
        let block = self.buffer;
        self.compress(&block);
        let mut output = [0; 32];
        for (chunk, value) in output.chunks_exact_mut(4).zip(self.state) {
            chunk.copy_from_slice(&value.to_be_bytes());
        }
        output
    }
    fn compress(&mut self, block: &[u8; 64]) {
        const K: [u32; 64] = [
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
            0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
            0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
            0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
            0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
            0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
            0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
            0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
            0xc67178f2,
        ];
        let mut w = [0u32; 64];
        for (index, chunk) in block.chunks_exact(4).enumerate() {
            w[index] = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        }
        for index in 16..64 {
            let s0 = w[index - 15].rotate_right(7)
                ^ w[index - 15].rotate_right(18)
                ^ (w[index - 15] >> 3);
            let s1 = w[index - 2].rotate_right(17)
                ^ w[index - 2].rotate_right(19)
                ^ (w[index - 2] >> 10);
            w[index] = w[index - 16]
                .wrapping_add(s0)
                .wrapping_add(w[index - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = self.state;
        for index in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ (!e & g);
            let t1 = h
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[index])
                .wrapping_add(w[index]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        for (slot, value) in self.state.iter_mut().zip([a, b, c, d, e, f, g, h]) {
            *slot = slot.wrapping_add(value);
        }
    }
}

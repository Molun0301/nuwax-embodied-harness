//! Supervised process fallback for raytracing compute.

use super::*;

use std::ffi::OsStr;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::ops::Range;
#[cfg(unix)]
use std::os::unix::process::CommandExt;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::{Condvar, Mutex};

const SIDECAR_ARG: &str = "--nuwax-mapping-raytracing-sidecar-v1";
const SIDECAR_PROTOCOL_MAGIC: [u8; 8] = *b"NWRAYS01";
const SIDECAR_MAX_STRING_BYTES: usize = 4 * 1024;
const SIDECAR_MAX_POINT_COUNT: usize = 2_000_000;
const SIDECAR_MAX_OCCUPANCY_BYTES: usize = 64 * 1024 * 1024;
const SIDECAR_MAX_OCCUPANCY_DIMENSION: u32 = 8_192;
// The runtime-facing compute wait is five seconds. Keep enough margin for the
// parent worker to report the failure and reap the process group before then.
const SIDECAR_RESPONSE_TIMEOUT: Duration = Duration::from_secs(3);

const REQUEST_START: u8 = 1;
const REQUEST_COMPUTE: u8 = 2;
const REQUEST_FLUSH: u8 = 3;
const REQUEST_EXPORT: u8 = 4;
const REQUEST_RESTORE: u8 = 5;
const REQUEST_RESOLVE: u8 = 6;
const REQUEST_STOP: u8 = 7;

const RESPONSE_STARTED: u8 = 101;
const RESPONSE_COMPUTE: u8 = 102;
const RESPONSE_EXPORT: u8 = 103;
const RESPONSE_RESTORE: u8 = 104;
const RESPONSE_RESOLVED: u8 = 105;
const RESPONSE_STOPPED: u8 = 106;
const RESPONSE_ERROR: u8 = 199;

const RESULT_ERROR: u8 = 0;
const RESULT_OK: u8 = 1;
const RESULT_CANDIDATE: u8 = 2;

const WIRE_PREFIX_BYTES: usize = SIDECAR_PROTOCOL_MAGIC.len() + size_of::<u8>();
const WIRE_STRING_MAX_BYTES: usize = size_of::<u32>() + SIDECAR_MAX_STRING_BYTES;
const WIRE_TOKEN_MAX_BYTES: usize =
    size_of::<u128>() + WIRE_STRING_MAX_BYTES + 6 * size_of::<u64>();
const WIRE_POINTS_MAX_BYTES: usize =
    size_of::<u64>() + SIDECAR_MAX_POINT_COUNT * 3 * size_of::<f64>();
const WIRE_INPUT_MAX_BYTES: usize = WIRE_STRING_MAX_BYTES
    + 5 * size_of::<u64>()
    + WIRE_STRING_MAX_BYTES
    + 3 * size_of::<f64>()
    + WIRE_POINTS_MAX_BYTES;
const WIRE_OCCUPANCY_MAX_BYTES: usize = 2 * size_of::<u32>()
    + size_of::<f64>()
    + 3 * size_of::<f64>()
    + 4 * size_of::<f64>()
    + WIRE_STRING_MAX_BYTES
    + size_of::<f64>()
    + size_of::<u64>()
    + SIDECAR_MAX_OCCUPANCY_BYTES;
const WIRE_OCCUPANCY_MIN_BYTES: usize = 2 * size_of::<u32>()
    + size_of::<f64>()
    + 3 * size_of::<f64>()
    + 4 * size_of::<f64>()
    + size_of::<u32>()
    + size_of::<f64>()
    + size_of::<u64>();

const REQUEST_START_FRAME_BYTES: usize = WIRE_PREFIX_BYTES + 9 * size_of::<u32>() + 21;
const REQUEST_COMPUTE_MIN_FRAME_BYTES: usize = WIRE_PREFIX_BYTES
    + size_of::<u128>()
    + size_of::<u128>()
    + size_of::<u32>()
    + 6 * size_of::<u64>()
    + size_of::<u32>()
    + 5 * size_of::<u64>()
    + size_of::<u32>()
    + 3 * size_of::<f64>()
    + size_of::<u64>();
const REQUEST_COMPUTE_MAX_FRAME_BYTES: usize =
    WIRE_PREFIX_BYTES + size_of::<u128>() + WIRE_TOKEN_MAX_BYTES + WIRE_INPUT_MAX_BYTES;
const REQUEST_FLUSH_MIN_FRAME_BYTES: usize = WIRE_PREFIX_BYTES
    + size_of::<u128>()
    + size_of::<u128>()
    + size_of::<u32>()
    + 6 * size_of::<u64>();
const REQUEST_FLUSH_MAX_FRAME_BYTES: usize =
    WIRE_PREFIX_BYTES + size_of::<u128>() + WIRE_TOKEN_MAX_BYTES;
const REQUEST_EXPORT_FRAME_BYTES: usize = WIRE_PREFIX_BYTES;
const REQUEST_RESTORE_MIN_FRAME_BYTES: usize =
    WIRE_PREFIX_BYTES + size_of::<u128>() + size_of::<u64>();
const REQUEST_RESTORE_MAX_FRAME_BYTES: usize =
    REQUEST_RESTORE_MIN_FRAME_BYTES + MAX_MAPPING_BACKEND_CHECKPOINT_BYTES;
const REQUEST_RESOLVE_FRAME_BYTES: usize = WIRE_PREFIX_BYTES + size_of::<u128>() + size_of::<u8>();
const REQUEST_STOP_FRAME_BYTES: usize = WIRE_PREFIX_BYTES;

const RESPONSE_STARTED_FRAME_BYTES: usize = WIRE_PREFIX_BYTES;
const RESPONSE_COMPUTE_MIN_FRAME_BYTES: usize =
    WIRE_PREFIX_BYTES + size_of::<u128>() + size_of::<u8>();
const WIRE_CANDIDATE_MAX_BYTES: usize = WIRE_STRING_MAX_BYTES
    + 5 * size_of::<u64>()
    + size_of::<u8>()
    + 2 * WIRE_OCCUPANCY_MAX_BYTES
    + size_of::<u64>();
const WIRE_CANDIDATE_MIN_BYTES: usize = size_of::<u32>()
    + 5 * size_of::<u64>()
    + size_of::<u8>()
    + 2 * WIRE_OCCUPANCY_MIN_BYTES
    + size_of::<u64>();
const RESPONSE_COMPUTE_MAX_FRAME_BYTES: usize =
    RESPONSE_COMPUTE_MIN_FRAME_BYTES + WIRE_CANDIDATE_MAX_BYTES;
const RESPONSE_COMPUTE_OK_FRAME_BYTES: usize = RESPONSE_COMPUTE_MIN_FRAME_BYTES;
const RESPONSE_COMPUTE_ERROR_MIN_FRAME_BYTES: usize =
    RESPONSE_COMPUTE_MIN_FRAME_BYTES + size_of::<u32>();
const RESPONSE_COMPUTE_ERROR_MAX_FRAME_BYTES: usize =
    RESPONSE_COMPUTE_MIN_FRAME_BYTES + WIRE_STRING_MAX_BYTES;
const RESPONSE_COMPUTE_CANDIDATE_MIN_FRAME_BYTES: usize =
    RESPONSE_COMPUTE_MIN_FRAME_BYTES + WIRE_CANDIDATE_MIN_BYTES;
const RESPONSE_EXPORT_MIN_FRAME_BYTES: usize = WIRE_PREFIX_BYTES + size_of::<u8>();
const RESPONSE_EXPORT_MAX_FRAME_BYTES: usize =
    WIRE_PREFIX_BYTES + size_of::<u8>() + size_of::<u64>() + MAX_MAPPING_BACKEND_CHECKPOINT_BYTES;
const RESPONSE_EXPORT_OK_MIN_FRAME_BYTES: usize =
    RESPONSE_EXPORT_MIN_FRAME_BYTES + size_of::<u64>();
const RESPONSE_EXPORT_ERROR_MIN_FRAME_BYTES: usize =
    RESPONSE_EXPORT_MIN_FRAME_BYTES + size_of::<u32>();
const RESPONSE_EXPORT_ERROR_MAX_FRAME_BYTES: usize =
    RESPONSE_EXPORT_MIN_FRAME_BYTES + WIRE_STRING_MAX_BYTES;
const RESPONSE_RESTORE_MIN_FRAME_BYTES: usize = RESPONSE_COMPUTE_MIN_FRAME_BYTES;
const RESPONSE_RESTORE_MAX_FRAME_BYTES: usize =
    RESPONSE_COMPUTE_MIN_FRAME_BYTES + WIRE_STRING_MAX_BYTES;
const RESPONSE_RESTORE_OK_FRAME_BYTES: usize = RESPONSE_RESTORE_MIN_FRAME_BYTES;
const RESPONSE_RESTORE_ERROR_MIN_FRAME_BYTES: usize =
    RESPONSE_RESTORE_MIN_FRAME_BYTES + size_of::<u32>();
const RESPONSE_RESTORE_ERROR_MAX_FRAME_BYTES: usize = RESPONSE_RESTORE_MAX_FRAME_BYTES;
const RESPONSE_RESOLVED_FRAME_BYTES: usize = WIRE_PREFIX_BYTES + size_of::<u128>();
const RESPONSE_STOPPED_FRAME_BYTES: usize = WIRE_PREFIX_BYTES;
const RESPONSE_ERROR_MIN_FRAME_BYTES: usize = WIRE_PREFIX_BYTES + size_of::<u32>();
const RESPONSE_ERROR_MAX_FRAME_BYTES: usize =
    RESPONSE_ERROR_MIN_FRAME_BYTES + SIDECAR_MAX_STRING_BYTES;
const MAX_STAGED_FRAME_PREFIX_BYTES: usize = RESPONSE_COMPUTE_MIN_FRAME_BYTES;

const REQUEST_DIRECTION_MAX_FRAME_BYTES: usize = REQUEST_RESTORE_MAX_FRAME_BYTES;
const RESPONSE_DIRECTION_MAX_FRAME_BYTES: usize = RESPONSE_COMPUTE_MAX_FRAME_BYTES;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RequestPhase {
    Start,
    Active,
    PendingResolution,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FrameExpectation {
    Request(RequestPhase),
    Response(u8),
}

impl FrameExpectation {
    const fn direction_max(self) -> usize {
        match self {
            Self::Request(_) => REQUEST_DIRECTION_MAX_FRAME_BYTES,
            Self::Response(_) => RESPONSE_DIRECTION_MAX_FRAME_BYTES,
        }
    }

    const fn permits(self, tag: u8) -> bool {
        match self {
            Self::Request(RequestPhase::Start) => tag == REQUEST_START,
            Self::Request(RequestPhase::Active) => matches!(
                tag,
                REQUEST_COMPUTE | REQUEST_FLUSH | REQUEST_EXPORT | REQUEST_RESTORE | REQUEST_STOP
            ),
            Self::Request(RequestPhase::PendingResolution) => {
                matches!(tag, REQUEST_RESOLVE | REQUEST_STOP)
            }
            Self::Response(expected) => tag == expected || tag == RESPONSE_ERROR,
        }
    }
}

fn wire_frame_bounds(tag: u8) -> Option<(usize, usize)> {
    match tag {
        REQUEST_START => Some((REQUEST_START_FRAME_BYTES, REQUEST_START_FRAME_BYTES)),
        REQUEST_COMPUTE => Some((
            REQUEST_COMPUTE_MIN_FRAME_BYTES,
            REQUEST_COMPUTE_MAX_FRAME_BYTES,
        )),
        REQUEST_FLUSH => Some((REQUEST_FLUSH_MIN_FRAME_BYTES, REQUEST_FLUSH_MAX_FRAME_BYTES)),
        REQUEST_EXPORT => Some((REQUEST_EXPORT_FRAME_BYTES, REQUEST_EXPORT_FRAME_BYTES)),
        REQUEST_RESTORE => Some((
            REQUEST_RESTORE_MIN_FRAME_BYTES,
            REQUEST_RESTORE_MAX_FRAME_BYTES,
        )),
        REQUEST_RESOLVE => Some((REQUEST_RESOLVE_FRAME_BYTES, REQUEST_RESOLVE_FRAME_BYTES)),
        REQUEST_STOP => Some((REQUEST_STOP_FRAME_BYTES, REQUEST_STOP_FRAME_BYTES)),
        RESPONSE_STARTED => Some((RESPONSE_STARTED_FRAME_BYTES, RESPONSE_STARTED_FRAME_BYTES)),
        RESPONSE_COMPUTE => Some((
            RESPONSE_COMPUTE_MIN_FRAME_BYTES,
            RESPONSE_COMPUTE_MAX_FRAME_BYTES,
        )),
        RESPONSE_EXPORT => Some((
            RESPONSE_EXPORT_MIN_FRAME_BYTES,
            RESPONSE_EXPORT_MAX_FRAME_BYTES,
        )),
        RESPONSE_RESTORE => Some((
            RESPONSE_RESTORE_MIN_FRAME_BYTES,
            RESPONSE_RESTORE_MAX_FRAME_BYTES,
        )),
        RESPONSE_RESOLVED => Some((RESPONSE_RESOLVED_FRAME_BYTES, RESPONSE_RESOLVED_FRAME_BYTES)),
        RESPONSE_STOPPED => Some((RESPONSE_STOPPED_FRAME_BYTES, RESPONSE_STOPPED_FRAME_BYTES)),
        RESPONSE_ERROR => Some((
            RESPONSE_ERROR_MIN_FRAME_BYTES,
            RESPONSE_ERROR_MAX_FRAME_BYTES,
        )),
        _ => None,
    }
}

const fn staged_frame_prefix_len(tag: u8) -> usize {
    match tag {
        RESPONSE_COMPUTE | RESPONSE_RESTORE => RESPONSE_COMPUTE_MIN_FRAME_BYTES,
        RESPONSE_EXPORT => RESPONSE_EXPORT_MIN_FRAME_BYTES,
        _ => WIRE_PREFIX_BYTES,
    }
}

fn result_variant_bounds(tag: u8, result: u8) -> Result<(usize, usize), String> {
    match (tag, result) {
        (RESPONSE_COMPUTE, RESULT_OK) => Ok((
            RESPONSE_COMPUTE_OK_FRAME_BYTES,
            RESPONSE_COMPUTE_OK_FRAME_BYTES,
        )),
        (RESPONSE_COMPUTE, RESULT_ERROR) => Ok((
            RESPONSE_COMPUTE_ERROR_MIN_FRAME_BYTES,
            RESPONSE_COMPUTE_ERROR_MAX_FRAME_BYTES,
        )),
        (RESPONSE_COMPUTE, RESULT_CANDIDATE) => Ok((
            RESPONSE_COMPUTE_CANDIDATE_MIN_FRAME_BYTES,
            RESPONSE_COMPUTE_MAX_FRAME_BYTES,
        )),
        (RESPONSE_EXPORT, RESULT_OK) => Ok((
            RESPONSE_EXPORT_OK_MIN_FRAME_BYTES,
            RESPONSE_EXPORT_MAX_FRAME_BYTES,
        )),
        (RESPONSE_EXPORT, RESULT_ERROR) => Ok((
            RESPONSE_EXPORT_ERROR_MIN_FRAME_BYTES,
            RESPONSE_EXPORT_ERROR_MAX_FRAME_BYTES,
        )),
        (RESPONSE_RESTORE, RESULT_OK) => Ok((
            RESPONSE_RESTORE_OK_FRAME_BYTES,
            RESPONSE_RESTORE_OK_FRAME_BYTES,
        )),
        (RESPONSE_RESTORE, RESULT_ERROR) => Ok((
            RESPONSE_RESTORE_ERROR_MIN_FRAME_BYTES,
            RESPONSE_RESTORE_ERROR_MAX_FRAME_BYTES,
        )),
        (RESPONSE_COMPUTE | RESPONSE_EXPORT | RESPONSE_RESTORE, _) => Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown result tag {result} for frame tag {tag}"
        )),
        _ => wire_frame_bounds(tag)
            .ok_or_else(|| format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown frame tag {tag}")),
    }
}

fn validate_frame_variant(prefix: &[u8], frame_len: usize, tag: u8) -> Result<(), String> {
    let staged_len = staged_frame_prefix_len(tag);
    let (minimum, maximum) = if staged_len == WIRE_PREFIX_BYTES {
        wire_frame_bounds(tag)
            .ok_or_else(|| format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown frame tag {tag}"))?
    } else {
        if prefix.len() < staged_len {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: frame tag {tag} is truncated before its result discriminator"
            ));
        }
        result_variant_bounds(tag, prefix[staged_len - 1])?
    };
    if frame_len < minimum || frame_len > maximum {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: declared frame size {frame_len} is outside {minimum}..={maximum} for tag {tag} result variant"
        ));
    }
    Ok(())
}

pub(super) const fn requires_supervision(spec: MappingBackendSpec) -> bool {
    if cfg!(test) {
        false
    } else {
        production_requires_supervision(spec)
    }
}

const fn production_requires_supervision(spec: MappingBackendSpec) -> bool {
    matches!(spec, MappingBackendSpec::Raytracing(_))
}

pub(super) fn run_requested_sidecar() -> Option<Result<(), String>> {
    let mut args = std::env::args_os();
    let _executable = args.next();
    if args.next().as_deref() != Some(OsStr::new(SIDECAR_ARG)) {
        return None;
    }
    if args.next().is_some() {
        return Some(Err(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: unexpected sidecar argument".to_owned(),
        ));
    }
    Some(run_stdio_sidecar())
}

#[derive(Clone, Debug, PartialEq)]
struct ParentCandidateAuthority {
    received_at_ns: u64,
    registered_frame_id: String,
    occupancy_resolution: f64,
}

impl ParentCandidateAuthority {
    fn from_input(input: &MappingBackendInput, occupancy_resolution: f64) -> Self {
        Self {
            received_at_ns: input.received_at_ns,
            registered_frame_id: input.registered_frame_id.clone(),
            occupancy_resolution,
        }
    }
}

pub(super) struct SupervisedRaySidecar {
    child: Child,
    request: BufWriter<ChildStdin>,
    response: BufReader<ChildStdout>,
    compute_config: MappingBackendComputeConfig,
    reaped: bool,
}

impl SupervisedRaySidecar {
    pub(super) fn start(
        spec: RaytracingBackendSpec,
        compute_config: MappingBackendComputeConfig,
    ) -> Result<Self, String> {
        #[cfg(target_os = "linux")]
        let executable = std::path::PathBuf::from("/proc/self/exe");
        #[cfg(not(target_os = "linux"))]
        let executable = std::env::current_exe().map_err(|error| {
            format!("MAPPING_BACKEND_SIDECAR_START_FAILED: resolve current executable: {error}")
        })?;
        let mut command = Command::new(&executable);
        command
            .arg(SIDECAR_ARG)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit());
        #[cfg(unix)]
        command.process_group(0);
        #[cfg(target_os = "linux")]
        // SAFETY: prctl/getppid are async-signal-safe syscalls, and the closure
        // does not access shared process state before exec.
        unsafe {
            command.pre_exec(|| {
                nix::sys::prctl::set_pdeathsig(nix::sys::signal::Signal::SIGKILL)
                    .map_err(|error| io::Error::from_raw_os_error(error as i32))?;
                if nix::unistd::getppid().as_raw() == 1 {
                    return Err(io::Error::from_raw_os_error(nix::libc::EPIPE));
                }
                Ok(())
            });
        }
        let mut child = command.spawn().map_err(|error| {
            format!(
                "MAPPING_BACKEND_SIDECAR_START_FAILED: spawn {}: {error}",
                executable.display()
            )
        })?;
        let request = match child.stdin.take() {
            Some(request) => request,
            None => {
                kill_and_reap_child(&mut child);
                return Err(
                    "MAPPING_BACKEND_SIDECAR_START_FAILED: child stdin was not piped".to_owned(),
                );
            }
        };
        let response = match child.stdout.take() {
            Some(response) => response,
            None => {
                kill_and_reap_child(&mut child);
                return Err(
                    "MAPPING_BACKEND_SIDECAR_START_FAILED: child stdout was not piped".to_owned(),
                );
            }
        };
        let mut sidecar = Self {
            child,
            request: BufWriter::new(request),
            response: BufReader::new(response),
            compute_config,
            reaped: false,
        };
        let frame = encode_start_request(spec, compute_config)?;
        let response = sidecar.exchange(&frame, RESPONSE_STARTED)?;
        let mut decoder = WireDecoder::new(&response);
        expect_response(&mut decoder, RESPONSE_STARTED)?;
        decoder.finish()?;
        Ok(sidecar)
    }

    fn compute(
        &mut self,
        token: &MappingComputeToken,
        input: &MappingBackendInput,
    ) -> Result<Option<MappingBackendCandidate>, String> {
        let frame = encode_compute_request(token, input)?;
        let authority = ParentCandidateAuthority::from_input(input, self.compute_config.resolution);
        self.candidate_response(token, Some(&authority), &frame)
    }

    fn flush(
        &mut self,
        token: &MappingComputeToken,
        authority: Option<&ParentCandidateAuthority>,
    ) -> Result<Option<MappingBackendCandidate>, String> {
        let frame = encode_flush_request(token)?;
        self.candidate_response(token, authority, &frame)
    }

    fn candidate_response(
        &mut self,
        token: &MappingComputeToken,
        authority: Option<&ParentCandidateAuthority>,
        frame: &[u8],
    ) -> Result<Option<MappingBackendCandidate>, String> {
        let response = self.exchange(frame, RESPONSE_COMPUTE)?;
        decode_candidate_response_frame(&response, token, authority)
    }

    fn export_checkpoint(&mut self) -> Result<Vec<u8>, String> {
        let mut encoder = WireEncoder::request(REQUEST_EXPORT);
        let response = self.exchange(&encoder.finish()?, RESPONSE_EXPORT)?;
        let mut decoder = WireDecoder::new(&response);
        expect_response(&mut decoder, RESPONSE_EXPORT)?;
        let payload_range = match decoder.u8()? {
            RESULT_ERROR => {
                let error = decoder.string()?;
                decoder.finish()?;
                return Err(error);
            }
            RESULT_OK => decoder.bytes_range()?,
            tag => Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown export result tag {tag}"
            ))?,
        };
        decoder.finish()?;
        extract_payload_in_place(response, payload_range)
    }

    fn restore_checkpoint(&mut self, transaction_id: u128, payload: &[u8]) -> Result<(), String> {
        let frame = encode_restore_request(transaction_id, payload)?;
        let response = self.exchange(&frame, RESPONSE_RESTORE)?;
        let mut decoder = WireDecoder::new(&response);
        expect_response(&mut decoder, RESPONSE_RESTORE)?;
        let response_transaction_id = decoder.u128()?;
        if response_transaction_id != transaction_id {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: restore transaction mismatch: expected={transaction_id} actual={response_transaction_id}"
            ));
        }
        let result = match decoder.u8()? {
            RESULT_ERROR => Err(decoder.string()?),
            RESULT_OK => Ok(()),
            tag => Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown restore result tag {tag}"
            )),
        };
        decoder.finish()?;
        result
    }

    fn resolve(&mut self, transaction_id: u128, commit: bool) -> Result<(), String> {
        let frame = encode_resolve_request(transaction_id, commit)?;
        let response = self.exchange(&frame, RESPONSE_RESOLVED)?;
        let mut decoder = WireDecoder::new(&response);
        expect_response(&mut decoder, RESPONSE_RESOLVED)?;
        let response_transaction_id = decoder.u128()?;
        if response_transaction_id != transaction_id {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: resolution transaction mismatch: expected={transaction_id} actual={response_transaction_id}"
            ));
        }
        decoder.finish()
    }

    fn exchange(&mut self, frame: &[u8], expected_response: u8) -> Result<Vec<u8>, String> {
        let finished = Arc::new((Mutex::new(false), Condvar::new()));
        let timed_out = Arc::new(AtomicBool::new(false));
        let watchdog_finished = Arc::clone(&finished);
        let watchdog_timed_out = Arc::clone(&timed_out);
        let child_pid = self.child.id();
        let watchdog = thread::spawn(move || {
            let (lock, wake) = &*watchdog_finished;
            let done = lock.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
            let (done, timeout) = wake
                .wait_timeout_while(done, SIDECAR_RESPONSE_TIMEOUT, |done| !*done)
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            if !*done && timeout.timed_out() {
                watchdog_timed_out.store(true, Ordering::Release);
                kill_sidecar_process_group(child_pid);
            }
        });
        let response = (|| {
            write_frame(&mut self.request, frame)?;
            self.request.flush().map_err(|error| {
                format!("MAPPING_BACKEND_SIDECAR_IO: flush child request: {error}")
            })?;
            read_frame(
                &mut self.response,
                FrameExpectation::Response(expected_response),
            )
        })();
        {
            let (lock, wake) = &*finished;
            *lock.lock().unwrap_or_else(|poisoned| poisoned.into_inner()) = true;
            wake.notify_one();
        }
        let _ = watchdog.join();
        if timed_out.load(Ordering::Acquire) {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_TIMEOUT: no response within {} ms",
                SIDECAR_RESPONSE_TIMEOUT.as_millis()
            ));
        }
        response?.ok_or_else(|| {
            "MAPPING_BACKEND_SIDECAR_IO: child closed its response stream".to_owned()
        })
    }

    fn stop(&mut self) {
        if self.reaped {
            return;
        }
        let mut encoder = WireEncoder::request(REQUEST_STOP);
        if let Ok(frame) = encoder.finish() {
            if let Ok(response) = self.exchange(&frame, RESPONSE_STOPPED) {
                let mut decoder = WireDecoder::new(&response);
                let _ =
                    expect_response(&mut decoder, RESPONSE_STOPPED).and_then(|()| decoder.finish());
            }
        }
        self.kill_and_reap();
    }

    fn kill_and_reap(&mut self) {
        if self.reaped {
            return;
        }
        match self.child.try_wait() {
            Ok(Some(_)) => {}
            Ok(None) | Err(_) => kill_and_reap_child(&mut self.child),
        }
        self.reaped = true;
    }
}

fn extract_payload_in_place(
    mut frame: Vec<u8>,
    payload_range: Range<usize>,
) -> Result<Vec<u8>, String> {
    if payload_range.start > payload_range.end || payload_range.end > frame.len() {
        return Err(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: checkpoint payload range is outside frame"
                .to_owned(),
        );
    }
    let payload_len = payload_range.len();
    frame.copy_within(payload_range, 0);
    frame.truncate(payload_len);
    Ok(frame)
}

fn decode_candidate_response_frame(
    response: &[u8],
    token: &MappingComputeToken,
    authority: Option<&ParentCandidateAuthority>,
) -> Result<Option<MappingBackendCandidate>, String> {
    let mut decoder = WireDecoder::new(response);
    expect_response(&mut decoder, RESPONSE_COMPUTE)?;
    let response_transaction_id = decoder.u128()?;
    if response_transaction_id != token.token_id {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: transaction mismatch: expected={} actual={response_transaction_id}",
            token.token_id
        ));
    }
    let result = match decoder.u8()? {
        RESULT_ERROR => Err(decoder.string()?),
        RESULT_OK => Ok(None),
        RESULT_CANDIDATE => {
            let authority = authority.ok_or_else(|| {
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: candidate has no parent authority".to_owned()
            })?;
            Ok(Some(decode_candidate(&mut decoder, token, authority)?))
        }
        tag => Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown compute result tag {tag}"
        )),
    };
    decoder.finish()?;
    result
}

fn kill_sidecar_process_group(child_pid: u32) {
    let Ok(pid) = i32::try_from(child_pid) else {
        return;
    };
    #[cfg(unix)]
    let _ = nix::sys::signal::kill(
        nix::unistd::Pid::from_raw(-pid),
        nix::sys::signal::Signal::SIGKILL,
    );
    let _ = nix::sys::signal::kill(
        nix::unistd::Pid::from_raw(pid),
        nix::sys::signal::Signal::SIGKILL,
    );
}

fn kill_and_reap_child(child: &mut Child) {
    kill_sidecar_process_group(child.id());
    let _ = child.kill();
    let _ = child.wait();
}

impl Drop for SupervisedRaySidecar {
    fn drop(&mut self) {
        self.kill_and_reap();
    }
}

pub(super) fn run_worker_loop(
    mut sidecar: SupervisedRaySidecar,
    receiver: mpsc::Receiver<MappingBackendCommand>,
    worker_stopped: Arc<AtomicBool>,
) {
    let mut last_parent_authority: Option<ParentCandidateAuthority> = None;
    while let Ok(command) = receiver.recv() {
        if worker_stopped.load(Ordering::Acquire) {
            break;
        }
        match command {
            MappingBackendCommand::Compute(job) => {
                let accepted_points_map = job.input.registered_points_map.clone();
                let parent_authority = ParentCandidateAuthority::from_input(
                    &job.input,
                    sidecar.compute_config.resolution,
                );
                let mut result = sidecar.compute(&job.token, &job.input);
                if let Ok(Some(candidate)) = &mut result {
                    apply_parent_candidate_fields(
                        candidate,
                        &parent_authority,
                        &accepted_points_map,
                    );
                }
                if result.is_ok() && !accepted_points_map.is_empty() {
                    last_parent_authority = Some(parent_authority);
                }
                let can_commit = result.is_ok();
                let transaction_id = job.token.token_id;
                let (resolution, resolution_rx) =
                    mapping_backend_resolution_channel(transaction_id);
                let delivered = job
                    .completion
                    .send(MappingBackendCompletion {
                        token: job.token,
                        accepted_points_map,
                        result,
                        resolution,
                    })
                    .is_ok();
                let committed = delivered
                    && can_commit
                    && mapping_backend_resolution_commits(transaction_id, &resolution_rx);
                if can_commit && sidecar.resolve(transaction_id, committed).is_err() {
                    break;
                }
                if !committed || worker_stopped.load(Ordering::Acquire) {
                    break;
                }
            }
            MappingBackendCommand::Flush(job) => {
                let mut result = sidecar.flush(&job.token, last_parent_authority.as_ref());
                if let (Ok(Some(candidate)), Some(authority)) =
                    (&mut result, last_parent_authority.as_ref())
                {
                    apply_parent_candidate_fields(candidate, authority, &[]);
                }
                let can_commit = result.is_ok();
                let transaction_id = job.token.token_id;
                let (resolution, resolution_rx) =
                    mapping_backend_resolution_channel(transaction_id);
                let delivered = job
                    .completion
                    .send(MappingBackendCompletion {
                        token: job.token,
                        accepted_points_map: Vec::new(),
                        result,
                        resolution,
                    })
                    .is_ok();
                let committed = delivered
                    && can_commit
                    && mapping_backend_resolution_commits(transaction_id, &resolution_rx);
                if can_commit && sidecar.resolve(transaction_id, committed).is_err() {
                    break;
                }
                if !committed || worker_stopped.load(Ordering::Acquire) {
                    break;
                }
            }
            MappingBackendCommand::ExportCheckpoint(job) => {
                let _ = job.completion.send(sidecar.export_checkpoint());
            }
            MappingBackendCommand::RestoreCheckpoint(job) => {
                // 独立进程保留既有跨进程编码合同；进程内后端直接接收预解码内核。
                let payload = match job.input {
                    MappingBackendRestoreInput::Encoded(payload) => payload,
                    MappingBackendRestoreInput::Prepared(prepared) => prepared.encoded,
                };
                let result = sidecar.restore_checkpoint(job.transaction_id, &payload);
                match result {
                    Ok(()) => {
                        let (resolution, resolution_rx) =
                            mapping_backend_resolution_channel(job.transaction_id);
                        let delivered = job.completion.send(Ok(resolution)).is_ok();
                        let committed = delivered
                            && mapping_backend_resolution_commits(
                                job.transaction_id,
                                &resolution_rx,
                            );
                        let keep_running = finish_restore_resolution(
                            &mut last_parent_authority,
                            committed,
                            sidecar.resolve(job.transaction_id, committed),
                        );
                        if keep_running != Ok(true) {
                            break;
                        }
                    }
                    Err(error) => {
                        let _ = job.completion.send(Err(error));
                    }
                }
            }
            MappingBackendCommand::Stop => break,
        }
    }
    sidecar.stop();
    worker_stopped.store(true, Ordering::Release);
}

fn apply_parent_candidate_fields(
    candidate: &mut MappingBackendCandidate,
    authority: &ParentCandidateAuthority,
    accepted_points_map: &[Point3<f64>],
) {
    candidate.received_at_ns = authority.received_at_ns;
    candidate.accepted_points_map.clear();
    candidate
        .accepted_points_map
        .extend_from_slice(accepted_points_map);
}

fn finish_restore_resolution(
    last_parent_authority: &mut Option<ParentCandidateAuthority>,
    committed: bool,
    resolution: Result<(), String>,
) -> Result<bool, String> {
    resolution?;
    if committed {
        *last_parent_authority = None;
    }
    Ok(committed)
}

fn run_stdio_sidecar() -> Result<(), String> {
    let stdin = io::stdin();
    let stdout = io::stdout();
    let mut request = BufReader::new(stdin.lock());
    let mut response = BufWriter::new(stdout.lock());
    run_sidecar_session(&mut request, &mut response)
}

fn run_sidecar_session(request: &mut impl Read, response: &mut impl Write) -> Result<(), String> {
    let start = read_frame(request, FrameExpectation::Request(RequestPhase::Start))?
        .ok_or_else(|| "MAPPING_BACKEND_SIDECAR_PROTOCOL: missing start request".to_owned())?;
    let (spec, compute_config) = match decode_start_request(&start) {
        Ok(start) => start,
        Err(error) => {
            let _ = write_error_response(response, &error);
            return Err(error);
        }
    };
    let mut kernel =
        MappingBackendSpec::Raytracing(spec).into_kernel(compute_config.window_capacity);
    let mut last_input: Option<MappingBackendInput> = None;
    let mut pending_transaction: Option<u128> = None;
    let mut started = WireEncoder::response(RESPONSE_STARTED);
    write_frame(response, &started.finish()?)?;
    response
        .flush()
        .map_err(|error| format!("MAPPING_BACKEND_SIDECAR_IO: flush start response: {error}"))?;

    loop {
        let phase = if pending_transaction.is_some() {
            RequestPhase::PendingResolution
        } else {
            RequestPhase::Active
        };
        let frame = match read_frame(request, FrameExpectation::Request(phase)) {
            Ok(Some(frame)) => frame,
            Ok(None) => return Ok(()),
            Err(error) => {
                let _ = write_error_response(response, &error);
                return Err(error);
            }
        };
        let outcome = handle_sidecar_request(
            &frame,
            &mut kernel,
            compute_config,
            &mut last_input,
            &mut pending_transaction,
        );
        let (response_frame, keep_running) = match outcome {
            Ok(outcome) => outcome,
            Err(error) => {
                write_error_response(response, &error)?;
                return Err(error);
            }
        };
        write_frame(response, &response_frame)?;
        response.flush().map_err(|error| {
            format!("MAPPING_BACKEND_SIDECAR_IO: flush child response: {error}")
        })?;
        if !keep_running {
            return Ok(());
        }
    }
}

fn handle_sidecar_request(
    frame: &[u8],
    kernel: &mut MappingKernel,
    compute_config: MappingBackendComputeConfig,
    last_input: &mut Option<MappingBackendInput>,
    pending_transaction: &mut Option<u128>,
) -> Result<(Vec<u8>, bool), String> {
    let mut decoder = WireDecoder::new(frame);
    decoder.magic()?;
    let request_tag = decoder.u8()?;
    if pending_transaction.is_some() && !matches!(request_tag, REQUEST_RESOLVE | REQUEST_STOP) {
        return Err(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: unresolved transaction blocks new work".to_owned(),
        );
    }

    match request_tag {
        REQUEST_COMPUTE => {
            let transaction_id = decoder.u128()?;
            let token = decode_token(&mut decoder)?;
            let input = decode_input(&mut decoder)?;
            decoder.finish()?;
            validate_transaction_identity(transaction_id, &token, Some(&input))?;
            let result = validate_backend_input(kernel, &input)
                .and_then(|()| compute_candidate_in_place(kernel, &input, compute_config));
            if result.is_ok() {
                if !input.registered_points_map.is_empty() {
                    *last_input = Some(input);
                }
                *pending_transaction = Some(transaction_id);
            }
            Ok((encode_candidate_response(transaction_id, result)?, true))
        }
        REQUEST_FLUSH => {
            let transaction_id = decoder.u128()?;
            let token = decode_token(&mut decoder)?;
            decoder.finish()?;
            validate_transaction_identity(transaction_id, &token, None)?;
            let result = match last_input.as_ref() {
                Some(input) => flush_candidate(kernel, input, &token, compute_config),
                None => Ok(None),
            };
            if result.is_ok() {
                *pending_transaction = Some(transaction_id);
            }
            Ok((encode_candidate_response(transaction_id, result)?, true))
        }
        REQUEST_EXPORT => {
            decoder.finish()?;
            let result = export_mapping_kernel_checkpoint(kernel);
            Ok((encode_export_response(result)?, true))
        }
        REQUEST_RESTORE => {
            let transaction_id = decoder.u128()?;
            let payload_range = decoder.bytes_range()?;
            decoder.finish()?;
            let result = restore_mapping_kernel_checkpoint(
                MappingBackendSpec::Raytracing(match kernel {
                    MappingKernel::Raytracing(ray) => RaytracingBackendSpec {
                        voxel_size: ray.config.voxel_size,
                        max_range: ray.config.max_range,
                        ray_subsample: ray.config.ray_subsample,
                        shadow_depth: ray.config.shadow_depth,
                        grace_depth: ray.config.grace_depth,
                        min_health: ray.config.min_health,
                        max_health: ray.config.max_health,
                        graze_cos: ray.config.graze_cos,
                        recency_window: ray.config.recency_window,
                    },
                    MappingKernel::Voxel(_) => {
                        return Err(
                            "MAPPING_BACKEND_SIDECAR_PROTOCOL: ray sidecar owns voxel state"
                                .to_owned(),
                        )
                    }
                }),
                compute_config,
                &frame[payload_range],
            );
            let response = match result {
                Ok(restored) => {
                    *kernel = restored;
                    *last_input = None;
                    *pending_transaction = Some(transaction_id);
                    encode_restore_response(transaction_id, Ok(()))?
                }
                Err(error) => encode_restore_response(transaction_id, Err(error))?,
            };
            Ok((response, true))
        }
        REQUEST_RESOLVE => {
            let transaction_id = decoder.u128()?;
            let commit = decoder.boolean()?;
            decoder.finish()?;
            if *pending_transaction != Some(transaction_id) {
                return Err(format!(
                    "MAPPING_BACKEND_SIDECAR_PROTOCOL: resolution does not match pending transaction: pending={pending_transaction:?} actual={transaction_id}"
                ));
            }
            *pending_transaction = None;
            let mut encoder = WireEncoder::response(RESPONSE_RESOLVED);
            encoder.u128(transaction_id);
            Ok((encoder.finish()?, commit))
        }
        REQUEST_STOP => {
            decoder.finish()?;
            let mut encoder = WireEncoder::response(RESPONSE_STOPPED);
            Ok((encoder.finish()?, false))
        }
        tag => Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown request tag {tag}"
        )),
    }
}

fn validate_transaction_identity(
    transaction_id: u128,
    token: &MappingComputeToken,
    input: Option<&MappingBackendInput>,
) -> Result<(), String> {
    if transaction_id != token.token_id {
        return Err(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: transaction and token ids differ".to_owned(),
        );
    }
    if let Some(input) = input {
        if input.session_id != token.session_id
            || input.session_generation != token.session_generation
            || input.backend_generation != token.backend_generation
            || input.frame_sequence != token.source_frame_sequence
            || input.source_timestamp_ns != token.source_timestamp_ns
        {
            return Err(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: token/input identity mismatch".to_owned(),
            );
        }
    }
    Ok(())
}

fn encode_start_request(
    spec: RaytracingBackendSpec,
    compute_config: MappingBackendComputeConfig,
) -> Result<Vec<u8>, String> {
    let mut encoder = WireEncoder::request(REQUEST_START);
    encode_ray_spec(&mut encoder, spec);
    encode_compute_config(&mut encoder, compute_config)?;
    encoder.finish()
}

fn decode_start_request(
    frame: &[u8],
) -> Result<(RaytracingBackendSpec, MappingBackendComputeConfig), String> {
    let mut decoder = WireDecoder::new(frame);
    decoder.magic()?;
    if decoder.u8()? != REQUEST_START {
        return Err("MAPPING_BACKEND_SIDECAR_PROTOCOL: first request is not start".to_owned());
    }
    let spec = decode_ray_spec(&mut decoder)?;
    spec.validate()?;
    let compute_config = decode_compute_config(&mut decoder)?;
    decoder.finish()?;
    Ok((spec, compute_config))
}

fn encode_compute_request(
    token: &MappingComputeToken,
    input: &MappingBackendInput,
) -> Result<Vec<u8>, String> {
    let mut encoder = WireEncoder::request(REQUEST_COMPUTE);
    encoder.u128(token.token_id);
    encode_token(&mut encoder, token)?;
    encode_input(&mut encoder, input)?;
    encoder.finish()
}

fn encode_flush_request(token: &MappingComputeToken) -> Result<Vec<u8>, String> {
    let mut encoder = WireEncoder::request(REQUEST_FLUSH);
    encoder.u128(token.token_id);
    encode_token(&mut encoder, token)?;
    encoder.finish()
}

fn encode_restore_request(transaction_id: u128, payload: &[u8]) -> Result<Vec<u8>, String> {
    let mut encoder = WireEncoder::request(REQUEST_RESTORE);
    encoder.u128(transaction_id);
    encoder.bytes(payload)?;
    encoder.finish()
}

fn encode_resolve_request(transaction_id: u128, commit: bool) -> Result<Vec<u8>, String> {
    let mut encoder = WireEncoder::request(REQUEST_RESOLVE);
    encoder.u128(transaction_id);
    encoder.boolean(commit);
    encoder.finish()
}

fn encode_candidate_response(
    transaction_id: u128,
    result: Result<Option<MappingBackendCandidate>, String>,
) -> Result<Vec<u8>, String> {
    let mut encoder = WireEncoder::response(RESPONSE_COMPUTE);
    encoder.u128(transaction_id);
    match result {
        Err(error) => {
            encoder.u8(RESULT_ERROR);
            encoder.string(&error)?;
        }
        Ok(None) => encoder.u8(RESULT_OK),
        Ok(Some(candidate)) => {
            encoder.u8(RESULT_CANDIDATE);
            encode_candidate(&mut encoder, &candidate)?;
        }
    }
    encoder.finish()
}

fn encode_export_response(result: Result<Vec<u8>, String>) -> Result<Vec<u8>, String> {
    let mut encoder = WireEncoder::response(RESPONSE_EXPORT);
    match result {
        Ok(payload) => {
            encoder.u8(RESULT_OK);
            encoder.bytes(&payload)?;
        }
        Err(error) => {
            encoder.u8(RESULT_ERROR);
            encoder.string(&error)?;
        }
    }
    encoder.finish()
}

fn encode_restore_response(
    transaction_id: u128,
    result: Result<(), String>,
) -> Result<Vec<u8>, String> {
    let mut encoder = WireEncoder::response(RESPONSE_RESTORE);
    encoder.u128(transaction_id);
    match result {
        Ok(()) => encoder.u8(RESULT_OK),
        Err(error) => {
            encoder.u8(RESULT_ERROR);
            encoder.string(&error)?;
        }
    }
    encoder.finish()
}

fn write_error_response(response: &mut impl Write, error: &str) -> Result<(), String> {
    let mut encoder = WireEncoder::response(RESPONSE_ERROR);
    encoder.string(error)?;
    write_frame(response, &encoder.finish()?)?;
    response
        .flush()
        .map_err(|flush_error| format!("MAPPING_BACKEND_SIDECAR_IO: flush error: {flush_error}"))
}

fn expect_response(decoder: &mut WireDecoder<'_>, expected: u8) -> Result<(), String> {
    decoder.magic()?;
    let tag = decoder.u8()?;
    if tag == RESPONSE_ERROR {
        let error = decoder.string()?;
        decoder.ensure_finished()?;
        return Err(error);
    }
    if tag != expected {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: expected response {expected}, got {tag}"
        ));
    }
    Ok(())
}

fn encode_ray_spec(encoder: &mut WireEncoder, spec: RaytracingBackendSpec) {
    encoder.f32(spec.voxel_size);
    encoder.f32(spec.max_range);
    encoder.u32(spec.ray_subsample);
    encoder.f32(spec.shadow_depth);
    encoder.f32(spec.grace_depth);
    encoder.i32(spec.min_health);
    encoder.i32(spec.max_health);
    encoder.f32(spec.graze_cos);
    encoder.u32(spec.recency_window);
}

fn decode_ray_spec(decoder: &mut WireDecoder<'_>) -> Result<RaytracingBackendSpec, String> {
    Ok(RaytracingBackendSpec {
        voxel_size: decoder.f32()?,
        max_range: decoder.f32()?,
        ray_subsample: decoder.u32()?,
        shadow_depth: decoder.f32()?,
        grace_depth: decoder.f32()?,
        min_health: decoder.i32()?,
        max_health: decoder.i32()?,
        graze_cos: decoder.f32()?,
        recency_window: decoder.u32()?,
    })
}

fn encode_compute_config(
    encoder: &mut WireEncoder,
    config: MappingBackendComputeConfig,
) -> Result<(), String> {
    encoder.f64(config.resolution);
    // 固定水平扫描标签；旧进程不能将此配置误解为已移除的投影。
    encoder.u8(5);
    encoder.f32(config.initial_safe_radius_meters);
    encoder.u64(
        u64::try_from(config.window_capacity).map_err(|_| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: window capacity exceeds u64".to_owned()
        })?,
    );
    Ok(())
}

fn decode_compute_config(
    decoder: &mut WireDecoder<'_>,
) -> Result<MappingBackendComputeConfig, String> {
    let resolution = decoder.f64()?;
    let projection = decoder.u8()?;
    if projection != 5 {
        return Err("MAPPING_BACKEND_SIDECAR_PROTOCOL: obsolete occupancy projection".into());
    }
    let initial_safe_radius_meters = decoder.f32()?;
    let window_capacity = usize::try_from(decoder.u64()?).map_err(|_| {
        "MAPPING_BACKEND_SIDECAR_PROTOCOL: window capacity exceeds usize".to_owned()
    })?;
    if !resolution.is_finite()
        || resolution <= 0.0
        || !initial_safe_radius_meters.is_finite()
        || initial_safe_radius_meters < 0.0
        || window_capacity == 0
    {
        return Err("MAPPING_BACKEND_SIDECAR_PROTOCOL: invalid compute config".to_owned());
    }
    Ok(MappingBackendComputeConfig {
        resolution,
        initial_safe_radius_meters,
        window_capacity,
    })
}

fn encode_token(encoder: &mut WireEncoder, token: &MappingComputeToken) -> Result<(), String> {
    encoder.u128(token.token_id);
    encoder.string(&token.session_id)?;
    encoder.u64(token.session_generation);
    encoder.u64(token.backend_generation);
    encoder.u64(token.source_frame_sequence);
    encoder.u64(token.source_timestamp_ns);
    encoder.u64(token.prepared_at_ns);
    encoder.u64(token.base_occupancy_revision);
    Ok(())
}

fn decode_token(decoder: &mut WireDecoder<'_>) -> Result<MappingComputeToken, String> {
    Ok(MappingComputeToken {
        token_id: decoder.u128()?,
        session_id: decoder.string()?,
        session_generation: decoder.u64()?,
        backend_generation: decoder.u64()?,
        source_frame_sequence: decoder.u64()?,
        source_timestamp_ns: decoder.u64()?,
        prepared_at_ns: decoder.u64()?,
        base_occupancy_revision: decoder.u64()?,
    })
}

fn encode_input(encoder: &mut WireEncoder, input: &MappingBackendInput) -> Result<(), String> {
    encoder.string(&input.session_id)?;
    encoder.u64(input.session_generation);
    encoder.u64(input.backend_generation);
    encoder.u64(input.frame_sequence);
    encoder.u64(input.source_timestamp_ns);
    encoder.u64(input.received_at_ns);
    encoder.string(&input.registered_frame_id)?;
    for value in input.sensor_origin_map {
        encoder.f64(value);
    }
    encoder.points(&input.registered_points_map)
}

fn decode_input(decoder: &mut WireDecoder<'_>) -> Result<MappingBackendInput, String> {
    let session_id = decoder.string()?;
    let session_generation = decoder.u64()?;
    let backend_generation = decoder.u64()?;
    let frame_sequence = decoder.u64()?;
    let source_timestamp_ns = decoder.u64()?;
    let received_at_ns = decoder.u64()?;
    let registered_frame_id = decoder.string()?;
    let sensor_origin_map = [decoder.f64()?, decoder.f64()?, decoder.f64()?];
    let registered_points_map = decoder.points()?;
    Ok(MappingBackendInput {
        session_id,
        session_generation,
        backend_generation,
        frame_sequence,
        source_timestamp_ns,
        received_at_ns,
        registered_frame_id,
        sensor_origin_map,
        registered_points_map,
    })
}

fn encode_candidate(
    encoder: &mut WireEncoder,
    candidate: &MappingBackendCandidate,
) -> Result<(), String> {
    encoder.string(&candidate.session_id)?;
    encoder.u64(candidate.session_generation);
    encoder.u64(candidate.backend_generation);
    encoder.u64(candidate.source_frame_sequence);
    encoder.u64(candidate.source_timestamp_ns);
    encoder.u64(candidate.received_at_ns);
    encoder.u8(match candidate.backend_kind {
        MappingBackendKind::SlamToolbox => return Err("二维算法不得进入点云工作进程".into()),
        MappingBackendKind::Voxel => 0,
        MappingBackendKind::Raytracing => 1,
    });
    encoder.occupancy(&candidate.live_window_occupancy)?;
    encoder.occupancy(&candidate.latest_frame_occupancy)?;
    encoder.u64(candidate.compute_elapsed_us);
    Ok(())
}

#[derive(Clone, Copy)]
struct OccupancyExpectation<'a> {
    frame_id: &'a str,
    resolution: f64,
    timestamp: f64,
}

fn decode_candidate(
    decoder: &mut WireDecoder<'_>,
    token: &MappingComputeToken,
    authority: &ParentCandidateAuthority,
) -> Result<MappingBackendCandidate, String> {
    let session_id = decoder.string_ref()?;
    let session_generation = decoder.u64()?;
    let backend_generation = decoder.u64()?;
    let source_frame_sequence = decoder.u64()?;
    let source_timestamp_ns = decoder.u64()?;
    let received_at_ns = decoder.u64()?;
    let backend_kind = match decoder.u8()? {
        0 => MappingBackendKind::Voxel,
        1 => MappingBackendKind::Raytracing,
        tag => {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown backend kind {tag}"
            ))
        }
    };
    if session_id != token.session_id
        || session_generation != token.session_generation
        || backend_generation != token.backend_generation
        || source_frame_sequence != token.source_frame_sequence
        || source_timestamp_ns != token.source_timestamp_ns
        || received_at_ns != authority.received_at_ns
        || backend_kind != MappingBackendKind::Raytracing
    {
        return Err("MAPPING_BACKEND_SIDECAR_PROTOCOL: candidate identity mismatch".to_owned());
    }
    let occupancy_expectation = OccupancyExpectation {
        frame_id: &authority.registered_frame_id,
        resolution: authority.occupancy_resolution,
        timestamp: token.source_timestamp_ns as f64 / 1_000_000_000.0,
    };
    let live_window_occupancy = decoder.occupancy(occupancy_expectation)?;
    let latest_frame_occupancy = decoder.occupancy(occupancy_expectation)?;
    let compute_elapsed_us = decoder.u64()?;
    Ok(MappingBackendCandidate {
        session_id: token.session_id.clone(),
        session_generation: token.session_generation,
        backend_generation: token.backend_generation,
        source_frame_sequence: token.source_frame_sequence,
        source_timestamp_ns: token.source_timestamp_ns,
        received_at_ns: authority.received_at_ns,
        backend_kind: MappingBackendKind::Raytracing,
        live_window_occupancy,
        latest_frame_occupancy,
        accepted_points_map: Vec::new(),
        compute_elapsed_us,
    })
}

fn validate_frame_prefix(
    prefix: &[u8],
    frame_len: usize,
    expectation: Option<FrameExpectation>,
) -> Result<u8, String> {
    if prefix.len() < WIRE_PREFIX_BYTES {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: frame size {frame_len} is smaller than the {WIRE_PREFIX_BYTES}-byte prefix"
        ));
    }
    if prefix[..SIDECAR_PROTOCOL_MAGIC.len()] != SIDECAR_PROTOCOL_MAGIC {
        return Err("MAPPING_BACKEND_SIDECAR_PROTOCOL: invalid protocol magic".to_owned());
    }
    let tag = prefix[SIDECAR_PROTOCOL_MAGIC.len()];
    let (minimum, maximum) = wire_frame_bounds(tag)
        .ok_or_else(|| format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: unknown frame tag {tag}"))?;
    if expectation.is_some_and(|expectation| !expectation.permits(tag)) {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: frame tag {tag} is not allowed for {expectation:?}"
        ));
    }
    if frame_len < minimum || frame_len > maximum {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: declared frame size {frame_len} is outside {minimum}..={maximum} for tag {tag}"
        ));
    }
    Ok(tag)
}

fn write_frame(writer: &mut impl Write, frame: &[u8]) -> Result<(), String> {
    let tag = validate_frame_prefix(frame, frame.len(), None)?;
    validate_frame_variant(frame, frame.len(), tag)?;
    let length = u64::try_from(frame.len())
        .map_err(|_| "MAPPING_BACKEND_SIDECAR_PROTOCOL: frame length exceeds u64".to_owned())?;
    writer
        .write_all(&length.to_be_bytes())
        .and_then(|()| writer.write_all(frame))
        .map_err(|error| format!("MAPPING_BACKEND_SIDECAR_IO: write frame: {error}"))
}

fn read_frame(
    reader: &mut impl Read,
    expectation: FrameExpectation,
) -> Result<Option<Vec<u8>>, String> {
    let mut header = [0_u8; size_of::<u64>()];
    match reader.read(&mut header[..1]) {
        Ok(0) => return Ok(None),
        Ok(1) => {}
        Ok(_) => unreachable!("one-byte read returned more than one byte"),
        Err(error) => {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_IO: read frame header: {error}"
            ))
        }
    }
    reader
        .read_exact(&mut header[1..])
        .map_err(|error| format!("MAPPING_BACKEND_SIDECAR_IO: read frame header: {error}"))?;
    let length = usize::try_from(u64::from_be_bytes(header))
        .map_err(|_| "MAPPING_BACKEND_SIDECAR_PROTOCOL: frame length exceeds usize".to_owned())?;
    if length < WIRE_PREFIX_BYTES || length > expectation.direction_max() {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: declared frame size {length} is outside {WIRE_PREFIX_BYTES}..={} for {expectation:?}",
            expectation.direction_max()
        ));
    }
    let mut prefix = [0_u8; WIRE_PREFIX_BYTES];
    reader
        .read_exact(&mut prefix)
        .map_err(|error| format!("MAPPING_BACKEND_SIDECAR_IO: read frame prefix: {error}"))?;
    let tag = validate_frame_prefix(&prefix, length, Some(expectation))?;
    let staged_len = staged_frame_prefix_len(tag);
    let mut staged_prefix = [0_u8; MAX_STAGED_FRAME_PREFIX_BYTES];
    staged_prefix[..WIRE_PREFIX_BYTES].copy_from_slice(&prefix);
    reader
        .read_exact(&mut staged_prefix[WIRE_PREFIX_BYTES..staged_len])
        .map_err(|error| {
            format!("MAPPING_BACKEND_SIDECAR_IO: read frame variant prefix: {error}")
        })?;
    validate_frame_variant(&staged_prefix[..staged_len], length, tag)?;
    let mut frame = Vec::new();
    frame.try_reserve_exact(length).map_err(|error| {
        format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: frame allocation failed: {error}")
    })?;
    frame.extend_from_slice(&staged_prefix[..staged_len]);
    frame.resize(length, 0);
    reader
        .read_exact(&mut frame[staged_len..])
        .map_err(|error| format!("MAPPING_BACKEND_SIDECAR_IO: read frame body: {error}"))?;
    Ok(Some(frame))
}

#[allow(clippy::too_many_arguments)]
fn validate_occupancy_metadata(
    width: u32,
    height: u32,
    resolution: f64,
    origin: &[f64; 3],
    origin_orientation: &[f64; 4],
    frame_id: &str,
    timestamp: f64,
    data_length: usize,
    expected: Option<OccupancyExpectation<'_>>,
) -> Result<(), String> {
    if width == 0
        || height == 0
        || width > SIDECAR_MAX_OCCUPANCY_DIMENSION
        || height > SIDECAR_MAX_OCCUPANCY_DIMENSION
    {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy dimensions {width}x{height} are outside 1..={SIDECAR_MAX_OCCUPANCY_DIMENSION} per axis"
        ));
    }
    let expected_length = usize::try_from(width)
        .ok()
        .and_then(|width| {
            usize::try_from(height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or_else(|| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy dimensions overflow".to_owned()
        })?;
    if data_length != expected_length || data_length > SIDECAR_MAX_OCCUPANCY_BYTES {
        return Err(format!(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy length {data_length} is invalid for {width}x{height}"
        ));
    }
    if !resolution.is_finite()
        || resolution <= 0.0
        || origin.iter().any(|value| !value.is_finite())
        || origin_orientation.iter().any(|value| !value.is_finite())
        || frame_id.trim().is_empty()
        || !timestamp.is_finite()
    {
        return Err("MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy metadata is invalid".to_owned());
    }
    if let Some(expected) = expected {
        if frame_id != expected.frame_id
            || resolution.to_bits() != expected.resolution.to_bits()
            || timestamp.to_bits() != expected.timestamp.to_bits()
        {
            return Err(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy metadata does not match parent authority"
                    .to_owned(),
            );
        }
    }
    Ok(())
}

fn validate_encoded_occupancy_values(encoded: &[u8]) -> Result<(), String> {
    if encoded
        .iter()
        .any(|value| *value > 100 && *value != u8::MAX)
    {
        return Err(
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy value is outside [-1, 100]".to_owned(),
        );
    }
    Ok(())
}

struct WireEncoder {
    frame: Vec<u8>,
}

impl WireEncoder {
    fn request(tag: u8) -> Self {
        Self::new(tag)
    }

    fn response(tag: u8) -> Self {
        Self::new(tag)
    }

    fn new(tag: u8) -> Self {
        let mut frame = Vec::with_capacity(128);
        frame.extend_from_slice(&SIDECAR_PROTOCOL_MAGIC);
        frame.push(tag);
        Self { frame }
    }

    fn finish(&mut self) -> Result<Vec<u8>, String> {
        let tag = validate_frame_prefix(&self.frame, self.frame.len(), None)?;
        validate_frame_variant(&self.frame, self.frame.len(), tag)?;
        Ok(std::mem::take(&mut self.frame))
    }

    fn u8(&mut self, value: u8) {
        self.frame.push(value);
    }

    fn boolean(&mut self, value: bool) {
        self.u8(u8::from(value));
    }

    fn u32(&mut self, value: u32) {
        self.frame.extend_from_slice(&value.to_le_bytes());
    }

    fn i32(&mut self, value: i32) {
        self.frame.extend_from_slice(&value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.frame.extend_from_slice(&value.to_le_bytes());
    }

    fn u128(&mut self, value: u128) {
        self.frame.extend_from_slice(&value.to_le_bytes());
    }

    fn f32(&mut self, value: f32) {
        self.u32(value.to_bits());
    }

    fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
    }

    fn string(&mut self, value: &str) -> Result<(), String> {
        if value.len() > SIDECAR_MAX_STRING_BYTES {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: string exceeds {SIDECAR_MAX_STRING_BYTES} bytes"
            ));
        }
        let length = u32::try_from(value.len())
            .map_err(|_| "MAPPING_BACKEND_SIDECAR_PROTOCOL: string exceeds u32 bytes".to_owned())?;
        self.u32(length);
        self.frame.extend_from_slice(value.as_bytes());
        Ok(())
    }

    fn bytes(&mut self, value: &[u8]) -> Result<(), String> {
        if value.len() > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: checkpoint exceeds {MAX_MAPPING_BACKEND_CHECKPOINT_BYTES} bytes"
            ));
        }
        self.u64(u64::try_from(value.len()).map_err(|_| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: byte payload exceeds u64".to_owned()
        })?);
        self.frame.extend_from_slice(value);
        Ok(())
    }

    fn points(&mut self, points: &[Point3<f64>]) -> Result<(), String> {
        if points.len() > SIDECAR_MAX_POINT_COUNT {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: point count exceeds {SIDECAR_MAX_POINT_COUNT}"
            ));
        }
        self.u64(
            u64::try_from(points.len()).map_err(|_| {
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: point count exceeds u64".to_owned()
            })?,
        );
        for point in points {
            self.f64(point.x);
            self.f64(point.y);
            self.f64(point.z);
        }
        Ok(())
    }

    fn occupancy(&mut self, occupancy: &OccupancyGrid) -> Result<(), String> {
        validate_occupancy_metadata(
            occupancy.width,
            occupancy.height,
            occupancy.resolution,
            &occupancy.origin,
            &occupancy.origin_orientation,
            &occupancy.frame_id,
            occupancy.ts,
            occupancy.data.len(),
            None,
        )?;
        if occupancy
            .data
            .iter()
            .any(|value| !(-1..=100).contains(value))
        {
            return Err(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy value is outside [-1, 100]".to_owned(),
            );
        }
        self.u32(occupancy.width);
        self.u32(occupancy.height);
        self.f64(occupancy.resolution);
        for value in occupancy.origin {
            self.f64(value);
        }
        for value in occupancy.origin_orientation {
            self.f64(value);
        }
        self.string(&occupancy.frame_id)?;
        self.f64(occupancy.ts);
        self.u64(u64::try_from(occupancy.data.len()).map_err(|_| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy data exceeds u64".to_owned()
        })?);
        self.frame
            .extend(occupancy.data.iter().map(|value| value.to_le_bytes()[0]));
        Ok(())
    }
}

struct WireDecoder<'a> {
    frame: &'a [u8],
    offset: usize,
}

impl<'a> WireDecoder<'a> {
    fn new(frame: &'a [u8]) -> Self {
        Self { frame, offset: 0 }
    }

    fn magic(&mut self) -> Result<(), String> {
        if self.take(SIDECAR_PROTOCOL_MAGIC.len())? != SIDECAR_PROTOCOL_MAGIC {
            return Err("MAPPING_BACKEND_SIDECAR_PROTOCOL: invalid protocol magic".to_owned());
        }
        Ok(())
    }

    fn finish(self) -> Result<(), String> {
        self.ensure_finished()
    }

    fn ensure_finished(&self) -> Result<(), String> {
        if self.offset == self.frame.len() {
            Ok(())
        } else {
            Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: {} trailing bytes",
                self.frame.len() - self.offset
            ))
        }
    }

    fn remaining(&self) -> usize {
        self.frame.len() - self.offset
    }

    fn take(&mut self, length: usize) -> Result<&'a [u8], String> {
        let end = self
            .offset
            .checked_add(length)
            .ok_or_else(|| "MAPPING_BACKEND_SIDECAR_PROTOCOL: decode offset overflow".to_owned())?;
        let value = self
            .frame
            .get(self.offset..end)
            .ok_or_else(|| "MAPPING_BACKEND_SIDECAR_PROTOCOL: truncated frame".to_owned())?;
        self.offset = end;
        Ok(value)
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N], String> {
        self.take(N)?
            .try_into()
            .map_err(|_| "MAPPING_BACKEND_SIDECAR_PROTOCOL: truncated frame".to_owned())
    }

    fn u8(&mut self) -> Result<u8, String> {
        Ok(self.array::<1>()?[0])
    }

    fn boolean(&mut self) -> Result<bool, String> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            value => Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: invalid boolean {value}"
            )),
        }
    }

    fn u32(&mut self) -> Result<u32, String> {
        Ok(u32::from_le_bytes(self.array()?))
    }

    fn i32(&mut self) -> Result<i32, String> {
        Ok(i32::from_le_bytes(self.array()?))
    }

    fn u64(&mut self) -> Result<u64, String> {
        Ok(u64::from_le_bytes(self.array()?))
    }

    fn u128(&mut self) -> Result<u128, String> {
        Ok(u128::from_le_bytes(self.array()?))
    }

    fn f32(&mut self) -> Result<f32, String> {
        Ok(f32::from_bits(self.u32()?))
    }

    fn f64(&mut self) -> Result<f64, String> {
        Ok(f64::from_bits(self.u64()?))
    }

    fn string(&mut self) -> Result<String, String> {
        let value = self.string_ref()?;
        let mut owned = String::new();
        owned.try_reserve_exact(value.len()).map_err(|error| {
            format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: string allocation failed: {error}")
        })?;
        owned.push_str(value);
        Ok(owned)
    }

    fn string_ref(&mut self) -> Result<&'a str, String> {
        let length = usize::try_from(self.u32()?).map_err(|_| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: string length exceeds usize".to_owned()
        })?;
        if length > SIDECAR_MAX_STRING_BYTES {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: string exceeds {SIDECAR_MAX_STRING_BYTES} bytes"
            ));
        }
        let value = std::str::from_utf8(self.take(length)?).map_err(|error| {
            format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: string is not UTF-8: {error}")
        })?;
        Ok(value)
    }

    fn bytes_range(&mut self) -> Result<Range<usize>, String> {
        let length = usize::try_from(self.u64()?).map_err(|_| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: byte length exceeds usize".to_owned()
        })?;
        if length > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: checkpoint exceeds {MAX_MAPPING_BACKEND_CHECKPOINT_BYTES} bytes"
            ));
        }
        let start = self.offset;
        self.take(length)?;
        Ok(start..self.offset)
    }

    fn points(&mut self) -> Result<Vec<Point3<f64>>, String> {
        let count = usize::try_from(self.u64()?).map_err(|_| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: point count exceeds usize".to_owned()
        })?;
        if count > SIDECAR_MAX_POINT_COUNT {
            return Err(format!(
                "MAPPING_BACKEND_SIDECAR_PROTOCOL: point count exceeds {SIDECAR_MAX_POINT_COUNT}"
            ));
        }
        let bytes = count.checked_mul(3 * size_of::<f64>()).ok_or_else(|| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: point byte count overflow".to_owned()
        })?;
        if bytes > self.remaining() {
            return Err("MAPPING_BACKEND_SIDECAR_PROTOCOL: truncated point array".to_owned());
        }
        let mut points = Vec::new();
        points.try_reserve_exact(count).map_err(|error| {
            format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: point allocation failed: {error}")
        })?;
        for _ in 0..count {
            points.push(Point3::new(self.f64()?, self.f64()?, self.f64()?));
        }
        Ok(points)
    }

    fn occupancy(&mut self, expected: OccupancyExpectation<'_>) -> Result<OccupancyGrid, String> {
        let width = self.u32()?;
        let height = self.u32()?;
        let resolution = self.f64()?;
        let origin = [self.f64()?, self.f64()?, self.f64()?];
        let origin_orientation = [self.f64()?, self.f64()?, self.f64()?, self.f64()?];
        let frame_id = self.string_ref()?;
        let ts = self.f64()?;
        let data_length = usize::try_from(self.u64()?).map_err(|_| {
            "MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy length exceeds usize".to_owned()
        })?;
        validate_occupancy_metadata(
            width,
            height,
            resolution,
            &origin,
            &origin_orientation,
            frame_id,
            ts,
            data_length,
            Some(expected),
        )?;
        let encoded = self.take(data_length)?;
        validate_encoded_occupancy_values(encoded)?;
        let mut data = Vec::new();
        data.try_reserve_exact(data_length).map_err(|error| {
            format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: occupancy allocation failed: {error}")
        })?;
        data.extend(encoded.iter().map(|value| i8::from_le_bytes([*value])));
        let mut owned_frame_id = String::new();
        owned_frame_id
            .try_reserve_exact(frame_id.len())
            .map_err(|error| {
                format!("MAPPING_BACKEND_SIDECAR_PROTOCOL: string allocation failed: {error}")
            })?;
        owned_frame_id.push_str(frame_id);
        let occupancy = OccupancyGrid {
            data,
            width,
            height,
            resolution,
            origin,
            origin_orientation,
            frame_id: owned_frame_id,
            ts,
        };
        Ok(occupancy)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_compute_config() -> MappingBackendComputeConfig {
        MappingBackendComputeConfig {
            resolution: 0.1,
            initial_safe_radius_meters: 0.0,
            window_capacity: 1_024,
        }
    }

    fn sample_token() -> MappingComputeToken {
        MappingComputeToken {
            token_id: 9,
            session_id: "session".to_owned(),
            session_generation: 3,
            backend_generation: 4,
            source_frame_sequence: 5,
            source_timestamp_ns: 2_000_000_000,
            prepared_at_ns: 7,
            base_occupancy_revision: 8,
        }
    }

    fn sample_input(token: &MappingComputeToken) -> MappingBackendInput {
        MappingBackendInput {
            session_id: token.session_id.clone(),
            session_generation: token.session_generation,
            backend_generation: token.backend_generation,
            frame_sequence: token.source_frame_sequence,
            source_timestamp_ns: token.source_timestamp_ns,
            received_at_ns: 10,
            registered_frame_id: "map".to_owned(),
            sensor_origin_map: [1.0, 2.0, 3.0],
            registered_points_map: vec![Point3::new(4.0, 5.0, 6.0)],
        }
    }

    fn sample_occupancy(token: &MappingComputeToken) -> OccupancyGrid {
        let mut occupancy = OccupancyGrid::with_frame(2, 2, 0.1, "map");
        occupancy.ts = token.source_timestamp_ns as f64 / 1_000_000_000.0;
        occupancy.data = vec![-1, 0, 65, 100];
        occupancy
    }

    fn sample_candidate(
        token: &MappingComputeToken,
        input: &MappingBackendInput,
    ) -> MappingBackendCandidate {
        let occupancy = sample_occupancy(token);
        MappingBackendCandidate {
            session_id: token.session_id.clone(),
            session_generation: token.session_generation,
            backend_generation: token.backend_generation,
            source_frame_sequence: token.source_frame_sequence,
            source_timestamp_ns: token.source_timestamp_ns,
            received_at_ns: input.received_at_ns,
            backend_kind: MappingBackendKind::Raytracing,
            live_window_occupancy: occupancy.clone(),
            latest_frame_occupancy: occupancy,
            accepted_points_map: vec![Point3::new(99.0, 99.0, 99.0)],
            compute_elapsed_us: 12,
        }
    }

    fn sample_authority(input: &MappingBackendInput) -> ParentCandidateAuthority {
        ParentCandidateAuthority::from_input(input, sample_compute_config().resolution)
    }

    fn declared_prefix(length: usize, magic: [u8; 8], tag: u8) -> Vec<u8> {
        let mut framed = Vec::with_capacity(size_of::<u64>() + WIRE_PREFIX_BYTES);
        framed.extend_from_slice(
            &u64::try_from(length)
                .expect("declared test length fits u64")
                .to_be_bytes(),
        );
        framed.extend_from_slice(&magic);
        framed.push(tag);
        framed
    }

    fn declared_result_prefix(length: usize, tag: u8, result: u8) -> Vec<u8> {
        let staged_len = staged_frame_prefix_len(tag);
        assert!(staged_len > WIRE_PREFIX_BYTES);
        let mut framed = Vec::with_capacity(size_of::<u64>() + staged_len);
        framed.extend_from_slice(
            &u64::try_from(length)
                .expect("declared test length fits u64")
                .to_be_bytes(),
        );
        framed.resize(size_of::<u64>() + staged_len, 0);
        let frame = &mut framed[size_of::<u64>()..];
        frame[..SIDECAR_PROTOCOL_MAGIC.len()].copy_from_slice(&SIDECAR_PROTOCOL_MAGIC);
        frame[SIDECAR_PROTOCOL_MAGIC.len()] = tag;
        frame[staged_len - 1] = result;
        framed
    }

    #[derive(Clone)]
    struct CandidateIdentityWire {
        session_id: String,
        session_generation: u64,
        backend_generation: u64,
        source_frame_sequence: u64,
        source_timestamp_ns: u64,
        received_at_ns: u64,
        backend_kind: u8,
    }

    fn candidate_identity(
        token: &MappingComputeToken,
        authority: &ParentCandidateAuthority,
    ) -> CandidateIdentityWire {
        CandidateIdentityWire {
            session_id: token.session_id.clone(),
            session_generation: token.session_generation,
            backend_generation: token.backend_generation,
            source_frame_sequence: token.source_frame_sequence,
            source_timestamp_ns: token.source_timestamp_ns,
            received_at_ns: authority.received_at_ns,
            backend_kind: 1,
        }
    }

    fn encode_candidate_identity_only(
        transaction_id: u128,
        identity: &CandidateIdentityWire,
    ) -> Vec<u8> {
        let mut encoder = WireEncoder::response(RESPONSE_COMPUTE);
        encoder.u128(transaction_id);
        encoder.u8(RESULT_CANDIDATE);
        encoder
            .string(&identity.session_id)
            .expect("encode test session");
        encoder.u64(identity.session_generation);
        encoder.u64(identity.backend_generation);
        encoder.u64(identity.source_frame_sequence);
        encoder.u64(identity.source_timestamp_ns);
        encoder.u64(identity.received_at_ns);
        encoder.u8(identity.backend_kind);
        encoder.frame
    }

    #[allow(clippy::too_many_arguments)]
    fn raw_occupancy(
        width: u32,
        height: u32,
        resolution: f64,
        origin: [f64; 3],
        orientation: [f64; 4],
        frame_id: &str,
        timestamp: f64,
        declared_data_length: u64,
        encoded_data: &[u8],
    ) -> Vec<u8> {
        let mut encoded = Vec::new();
        encoded.extend_from_slice(&width.to_le_bytes());
        encoded.extend_from_slice(&height.to_le_bytes());
        encoded.extend_from_slice(&resolution.to_bits().to_le_bytes());
        for value in origin {
            encoded.extend_from_slice(&value.to_bits().to_le_bytes());
        }
        for value in orientation {
            encoded.extend_from_slice(&value.to_bits().to_le_bytes());
        }
        encoded.extend_from_slice(
            &u32::try_from(frame_id.len())
                .expect("test frame id fits u32")
                .to_le_bytes(),
        );
        encoded.extend_from_slice(frame_id.as_bytes());
        encoded.extend_from_slice(&timestamp.to_bits().to_le_bytes());
        encoded.extend_from_slice(&declared_data_length.to_le_bytes());
        encoded.extend_from_slice(encoded_data);
        encoded
    }

    fn occupancy_expectation<'a>(
        token: &MappingComputeToken,
        authority: &'a ParentCandidateAuthority,
    ) -> OccupancyExpectation<'a> {
        OccupancyExpectation {
            frame_id: &authority.registered_frame_id,
            resolution: authority.occupancy_resolution,
            timestamp: token.source_timestamp_ns as f64 / 1_000_000_000.0,
        }
    }

    #[test]
    fn production_policy_never_admits_ray_in_process() {
        assert!(!production_requires_supervision(MappingBackendSpec::Voxel));
        assert!(production_requires_supervision(
            MappingBackendSpec::raytracing_defaults()
        ));
        assert!(!requires_supervision(
            MappingBackendSpec::raytracing_defaults()
        ));
    }

    #[test]
    fn planar_scan_selection_survives_sidecar_wire_format() {
        let mut encoder = WireEncoder { frame: Vec::new() };
        encode_compute_config(&mut encoder, sample_compute_config()).unwrap();
        let mut encoded = encoder.frame;
        assert_eq!(encoded[8], 5);
        for obsolete in 0..5 {
            encoded[8] = obsolete;
            assert!(decode_compute_config(&mut WireDecoder::new(&encoded)).is_err());
        }
    }

    #[test]
    fn binary_protocol_round_trips_identity_and_payload_without_json() {
        let token = sample_token();
        let input = sample_input(&token);
        let frame = encode_compute_request(&token, &input).expect("encode compute");
        let mut decoder = WireDecoder::new(&frame);
        decoder.magic().expect("magic");
        assert_eq!(decoder.u8().expect("tag"), REQUEST_COMPUTE);
        assert_eq!(decoder.u128().expect("transaction"), token.token_id);
        assert_eq!(decode_token(&mut decoder).expect("token"), token);
        assert_eq!(decode_input(&mut decoder).expect("input"), input);
        decoder.finish().expect("exact frame");

        let payload = vec![0, 1, 2, 254, 255];
        let frame = encode_restore_request(11, &payload).expect("encode restore");
        let mut decoder = WireDecoder::new(&frame);
        decoder.magic().expect("magic");
        assert_eq!(decoder.u8().expect("tag"), REQUEST_RESTORE);
        assert_eq!(decoder.u128().expect("transaction"), 11);
        let payload_range = decoder.bytes_range().expect("payload range");
        decoder.finish().expect("exact frame");
        assert_eq!(&frame[payload_range.clone()], payload);
    }

    #[test]
    fn checkpoint_export_extraction_preserves_backing_allocation() {
        let payload = vec![0, 1, 2, 254, 255];
        let response = encode_export_response(Ok(payload.clone())).expect("encode export");
        let mut decoder = WireDecoder::new(&response);
        expect_response(&mut decoder, RESPONSE_EXPORT).expect("export response");
        assert_eq!(decoder.u8().expect("result"), RESULT_OK);
        let payload_range = decoder.bytes_range().expect("payload range");
        decoder.finish().expect("exact frame");

        let backing = response.as_ptr();
        let extracted =
            extract_payload_in_place(response, payload_range).expect("extract checkpoint payload");
        assert_eq!(extracted.as_ptr(), backing);
        assert_eq!(extracted, payload);
    }

    #[test]
    fn message_specific_caps_are_exact_and_checked_before_body_allocation() {
        let cases = [
            (
                FrameExpectation::Request(RequestPhase::Start),
                REQUEST_START,
                66,
                66,
            ),
            (
                FrameExpectation::Request(RequestPhase::Active),
                REQUEST_COMPUTE,
                173,
                48_012_461,
            ),
            (
                FrameExpectation::Request(RequestPhase::Active),
                REQUEST_FLUSH,
                93,
                4_189,
            ),
            (
                FrameExpectation::Request(RequestPhase::Active),
                REQUEST_EXPORT,
                9,
                9,
            ),
            (
                FrameExpectation::Request(RequestPhase::Active),
                REQUEST_RESTORE,
                33,
                67_108_897,
            ),
            (
                FrameExpectation::Request(RequestPhase::PendingResolution),
                REQUEST_RESOLVE,
                26,
                26,
            ),
            (
                FrameExpectation::Request(RequestPhase::Active),
                REQUEST_STOP,
                9,
                9,
            ),
            (
                FrameExpectation::Response(RESPONSE_STARTED),
                RESPONSE_STARTED,
                9,
                9,
            ),
            (
                FrameExpectation::Response(RESPONSE_COMPUTE),
                RESPONSE_COMPUTE,
                26,
                134_230_279,
            ),
            (
                FrameExpectation::Response(RESPONSE_EXPORT),
                RESPONSE_EXPORT,
                10,
                67_108_882,
            ),
            (
                FrameExpectation::Response(RESPONSE_RESTORE),
                RESPONSE_RESTORE,
                26,
                4_126,
            ),
            (
                FrameExpectation::Response(RESPONSE_RESOLVED),
                RESPONSE_RESOLVED,
                25,
                25,
            ),
            (
                FrameExpectation::Response(RESPONSE_STOPPED),
                RESPONSE_STOPPED,
                9,
                9,
            ),
            (
                FrameExpectation::Response(RESPONSE_ERROR),
                RESPONSE_ERROR,
                13,
                4_109,
            ),
        ];
        assert_eq!(cases.len(), 14);
        for (expectation, tag, minimum, maximum) in cases {
            assert_eq!(wire_frame_bounds(tag), Some((minimum, maximum)));
            let framed = declared_prefix(minimum, SIDECAR_PROTOCOL_MAGIC, tag);
            let prefix = &framed[size_of::<u64>()..];
            assert_eq!(
                validate_frame_prefix(prefix, minimum, Some(expectation)),
                Ok(tag)
            );
            assert_eq!(
                validate_frame_prefix(prefix, maximum, Some(expectation)),
                Ok(tag)
            );
            let error = validate_frame_prefix(prefix, minimum - 1, Some(expectation))
                .expect_err("tag-specific minimum-1 must fail");
            assert!(error.contains("declared frame size"), "tag={tag} {error}");

            let bytes = declared_prefix(maximum + 1, SIDECAR_PROTOCOL_MAGIC, tag);
            let error = read_frame(&mut bytes.as_slice(), expectation)
                .expect_err("tag-specific oversize must fail from prefix only");
            assert!(
                error.contains("declared frame size"),
                "tag={tag} error={error}"
            );
            assert!(
                !error.contains("read frame body"),
                "tag={tag} attempted to read body: {error}"
            );
            assert!(
                !error.contains("frame allocation"),
                "tag={tag} attempted allocation: {error}"
            );
        }
    }

    #[test]
    fn result_variant_caps_are_exact_and_staged_before_body_allocation() {
        let cases = [
            (RESPONSE_COMPUTE, RESULT_OK, 26, 26),
            (RESPONSE_COMPUTE, RESULT_ERROR, 30, 4_126),
            (RESPONSE_COMPUTE, RESULT_CANDIDATE, 263, 134_230_279),
            (RESPONSE_EXPORT, RESULT_OK, 18, 67_108_882),
            (RESPONSE_EXPORT, RESULT_ERROR, 14, 4_110),
            (RESPONSE_RESTORE, RESULT_OK, 26, 26),
            (RESPONSE_RESTORE, RESULT_ERROR, 30, 4_126),
        ];
        for (tag, result, minimum, maximum) in cases {
            assert_eq!(result_variant_bounds(tag, result), Ok((minimum, maximum)));
            let framed = declared_result_prefix(minimum, tag, result);
            let prefix = &framed[size_of::<u64>()..];
            validate_frame_variant(prefix, minimum, tag).expect("variant minimum");
            validate_frame_variant(prefix, maximum, tag).expect("variant maximum");
            let error =
                validate_frame_variant(prefix, minimum - 1, tag).expect_err("variant minimum-1");
            assert!(error.contains("declared frame size"), "tag={tag} {error}");
            let error =
                validate_frame_variant(prefix, maximum + 1, tag).expect_err("variant maximum+1");
            assert!(error.contains("declared frame size"), "tag={tag} {error}");
        }

        let staged_rejections = [
            (RESPONSE_COMPUTE, RESULT_OK, 27),
            (RESPONSE_COMPUTE, RESULT_ERROR, 4_127),
            (RESPONSE_EXPORT, RESULT_ERROR, 4_111),
            (RESPONSE_RESTORE, RESULT_OK, 27),
        ];
        for (tag, result, declared) in staged_rejections {
            let bytes = declared_result_prefix(declared, tag, result);
            let error = read_frame(&mut bytes.as_slice(), FrameExpectation::Response(tag))
                .expect_err("variant cap must reject from staged prefix");
            assert!(error.contains("declared frame size"), "tag={tag} {error}");
            assert!(!error.contains("read frame body"), "tag={tag} {error}");
            assert!(!error.contains("frame allocation"), "tag={tag} {error}");
        }

        for tag in [RESPONSE_COMPUTE, RESPONSE_EXPORT, RESPONSE_RESTORE] {
            let declared = staged_frame_prefix_len(tag);
            let bytes = declared_result_prefix(declared, tag, u8::MAX);
            let error = read_frame(&mut bytes.as_slice(), FrameExpectation::Response(tag))
                .expect_err("unknown variant must reject from staged prefix");
            assert!(error.contains("unknown result tag"), "tag={tag} {error}");
            assert!(!error.contains("read frame body"), "tag={tag} {error}");
            assert!(!error.contains("frame allocation"), "tag={tag} {error}");
        }
    }

    #[test]
    fn frame_prefix_magic_tag_phase_and_truncation_fail_closed() {
        let short = 8_u64.to_be_bytes();
        let error = read_frame(
            &mut short.as_slice(),
            FrameExpectation::Request(RequestPhase::Start),
        )
        .expect_err("short declared frame");
        assert!(error.contains("declared frame size"));

        let bad_magic = declared_prefix(REQUEST_START_FRAME_BYTES, *b"BADMAGIC", REQUEST_START);
        let error = read_frame(
            &mut bad_magic.as_slice(),
            FrameExpectation::Request(RequestPhase::Start),
        )
        .expect_err("bad magic");
        assert!(error.contains("invalid protocol magic"));

        let unknown = declared_prefix(WIRE_PREFIX_BYTES, SIDECAR_PROTOCOL_MAGIC, 77);
        let error = read_frame(
            &mut unknown.as_slice(),
            FrameExpectation::Request(RequestPhase::Active),
        )
        .expect_err("unknown tag");
        assert!(error.contains("unknown frame tag"));

        let wrong_phase = declared_prefix(
            REQUEST_COMPUTE_MIN_FRAME_BYTES,
            SIDECAR_PROTOCOL_MAGIC,
            REQUEST_COMPUTE,
        );
        let error = read_frame(
            &mut wrong_phase.as_slice(),
            FrameExpectation::Request(RequestPhase::PendingResolution),
        )
        .expect_err("pending phase rejects compute before body");
        assert!(error.contains("not allowed"));
        assert!(!error.contains("read frame body"));

        let wrong_response = declared_prefix(
            RESPONSE_COMPUTE_MIN_FRAME_BYTES,
            SIDECAR_PROTOCOL_MAGIC,
            RESPONSE_COMPUTE,
        );
        let error = read_frame(
            &mut wrong_response.as_slice(),
            FrameExpectation::Response(RESPONSE_STARTED),
        )
        .expect_err("unexpected response tag");
        assert!(error.contains("not allowed"));

        let partial_header = [0_u8; 4];
        let mut partial_header = partial_header.as_slice();
        let error = read_frame(
            &mut partial_header,
            FrameExpectation::Request(RequestPhase::Start),
        )
        .expect_err("truncated length header");
        assert!(error.contains("read frame header"));
    }

    #[test]
    fn nested_string_point_and_checkpoint_lengths_reject_before_bulk_reads() {
        let mut string_wire = (u32::try_from(SIDECAR_MAX_STRING_BYTES + 1).unwrap())
            .to_le_bytes()
            .to_vec();
        let error = WireDecoder::new(&string_wire)
            .string_ref()
            .expect_err("oversized string");
        assert!(error.contains("string exceeds"));
        string_wire.clear();

        let point_wire = u64::try_from(SIDECAR_MAX_POINT_COUNT + 1)
            .unwrap()
            .to_le_bytes()
            .to_vec();
        let error = WireDecoder::new(&point_wire)
            .points()
            .expect_err("oversized point count");
        assert!(error.contains("point count exceeds"));

        let checkpoint_wire = u64::try_from(MAX_MAPPING_BACKEND_CHECKPOINT_BYTES + 1)
            .unwrap()
            .to_le_bytes()
            .to_vec();
        let error = WireDecoder::new(&checkpoint_wire)
            .bytes_range()
            .expect_err("oversized checkpoint");
        assert!(error.contains("checkpoint exceeds"));
    }

    #[test]
    fn candidate_transaction_and_identity_are_rejected_before_occupancy_decode() {
        let token = sample_token();
        let input = sample_input(&token);
        let authority = sample_authority(&input);

        let mut transaction = WireEncoder::response(RESPONSE_COMPUTE);
        transaction.u128(token.token_id + 1);
        transaction.u8(RESULT_CANDIDATE);
        let transaction = transaction.frame;
        let error = decode_candidate_response_frame(&transaction, &token, Some(&authority))
            .expect_err("forged transaction");
        assert!(error.contains("transaction mismatch"));
        assert!(!error.contains("truncated frame"));

        let base = candidate_identity(&token, &authority);
        let mut forgeries = Vec::new();
        let mut forged = base.clone();
        forged.session_id = "forged".to_owned();
        forgeries.push(forged);
        let mut forged = base.clone();
        forged.session_generation += 1;
        forgeries.push(forged);
        let mut forged = base.clone();
        forged.backend_generation += 1;
        forgeries.push(forged);
        let mut forged = base.clone();
        forged.source_frame_sequence += 1;
        forgeries.push(forged);
        let mut forged = base.clone();
        forged.source_timestamp_ns += 1;
        forgeries.push(forged);
        let mut forged = base.clone();
        forged.received_at_ns += 1;
        forgeries.push(forged);
        let mut forged = base;
        forged.backend_kind = 0;
        forgeries.push(forged);

        for forged in forgeries {
            let frame = encode_candidate_identity_only(token.token_id, &forged);
            let error = decode_candidate_response_frame(&frame, &token, Some(&authority))
                .expect_err("forged candidate identity");
            assert!(error.contains("candidate identity mismatch"), "{error}");
            assert!(
                !error.contains("truncated frame"),
                "identity validation ran after occupancy decode: {error}"
            );
        }

        let frame =
            encode_candidate_identity_only(token.token_id, &candidate_identity(&token, &authority));
        let error = decode_candidate_response_frame(&frame, &token, Some(&authority))
            .expect_err("valid identity reaches missing occupancy");
        assert!(error.contains("truncated frame"));
    }

    #[test]
    fn candidate_uses_parent_received_time_and_accepted_points() {
        let token = sample_token();
        let input = sample_input(&token);
        let authority = sample_authority(&input);
        let candidate = sample_candidate(&token, &input);
        let response = encode_candidate_response(token.token_id, Ok(Some(candidate)))
            .expect("encode candidate response");
        let mut decoded = decode_candidate_response_frame(&response, &token, Some(&authority))
            .expect("decode candidate response")
            .expect("candidate");
        assert_eq!(decoded.received_at_ns, input.received_at_ns);
        assert!(decoded.accepted_points_map.is_empty());

        decoded.received_at_ns = u64::MAX;
        decoded.accepted_points_map = vec![Point3::new(-1.0, -1.0, -1.0)];
        apply_parent_candidate_fields(&mut decoded, &authority, &input.registered_points_map);
        assert_eq!(decoded.received_at_ns, input.received_at_ns);
        assert_eq!(decoded.accepted_points_map, input.registered_points_map);

        apply_parent_candidate_fields(&mut decoded, &authority, &[]);
        assert!(decoded.accepted_points_map.is_empty());
    }

    #[test]
    fn committed_restore_clears_parent_authority_before_forged_flush_candidate_decode() {
        let token = sample_token();
        let input = sample_input(&token);
        let parent_authority = sample_authority(&input);
        let mut last_parent_authority = Some(parent_authority.clone());

        assert_eq!(
            finish_restore_resolution(&mut last_parent_authority, true, Ok(())),
            Ok(true)
        );
        assert!(last_parent_authority.is_none());

        let forged_candidate =
            encode_candidate_response(token.token_id, Ok(Some(sample_candidate(&token, &input))))
                .expect("encode forged post-restore flush candidate");
        let error = decode_candidate_response_frame(
            &forged_candidate,
            &token,
            last_parent_authority.as_ref(),
        )
        .expect_err("post-restore flush candidate has no parent authority");
        assert!(
            error.contains("candidate has no parent authority"),
            "{error}"
        );

        let mut empty_flush = WireEncoder::response(RESPONSE_COMPUTE);
        empty_flush.u128(token.token_id);
        empty_flush.u8(RESULT_OK);
        let empty_flush = empty_flush.finish().expect("encode empty flush result");
        assert!(decode_candidate_response_frame(
            &empty_flush,
            &token,
            last_parent_authority.as_ref(),
        )
        .expect("post-restore empty flush is valid")
        .is_none());

        let mut aborted_authority = Some(parent_authority.clone());
        assert_eq!(
            finish_restore_resolution(&mut aborted_authority, false, Ok(())),
            Ok(false)
        );
        assert_eq!(aborted_authority, Some(parent_authority.clone()));

        let mut failed_authority = Some(parent_authority.clone());
        let error = finish_restore_resolution(
            &mut failed_authority,
            true,
            Err("resolution failed".to_owned()),
        )
        .expect_err("failed resolution");
        assert_eq!(error, "resolution failed");
        assert_eq!(failed_authority, Some(parent_authority));
    }

    #[test]
    fn occupancy_axes_metadata_values_truncation_and_trailing_bytes_fail_closed() {
        let token = sample_token();
        let input = sample_input(&token);
        let authority = sample_authority(&input);
        let expected = occupancy_expectation(&token, &authority);
        let identity = [0.0, 0.0, 0.0, 1.0];
        let timestamp = expected.timestamp;

        for (width, height) in [(0, 1), (1, 0), (8_193, 1), (1, 8_193)] {
            let wire = raw_occupancy(
                width,
                height,
                0.1,
                [0.0; 3],
                identity,
                "map",
                timestamp,
                u64::from(width) * u64::from(height),
                &[],
            );
            let error = WireDecoder::new(&wire)
                .occupancy(expected)
                .expect_err("invalid occupancy axis");
            assert!(error.contains("dimensions"), "{error}");
            assert!(!error.contains("truncated frame"));
        }

        let mismatched = raw_occupancy(2, 2, 0.1, [0.0; 3], identity, "map", timestamp, 3, &[]);
        let error = WireDecoder::new(&mismatched)
            .occupancy(expected)
            .expect_err("mismatched occupancy length");
        assert!(error.contains("occupancy length"));
        assert!(!error.contains("truncated frame"));

        let metadata_cases = [
            (f64::NAN, [0.0; 3], identity, "map", timestamp),
            (0.0, [0.0; 3], identity, "map", timestamp),
            (0.1, [f64::INFINITY, 0.0, 0.0], identity, "map", timestamp),
            (0.1, [0.0; 3], [0.0, f64::NAN, 0.0, 1.0], "map", timestamp),
            (0.1, [0.0; 3], identity, "", timestamp),
            (0.1, [0.0; 3], identity, "odom", timestamp),
            (0.1, [0.0; 3], identity, "map", f64::NAN),
            (0.1, [0.0; 3], identity, "map", timestamp + 1.0),
            (0.2, [0.0; 3], identity, "map", timestamp),
        ];
        for (resolution, origin, orientation, frame_id, timestamp) in metadata_cases {
            let wire = raw_occupancy(
                1,
                1,
                resolution,
                origin,
                orientation,
                frame_id,
                timestamp,
                1,
                &[],
            );
            let error = WireDecoder::new(&wire)
                .occupancy(expected)
                .expect_err("invalid occupancy metadata");
            assert!(error.contains("metadata"), "{error}");
            assert!(!error.contains("truncated frame"));
        }

        for invalid in [101_u8, 254_u8] {
            let wire = raw_occupancy(
                1,
                1,
                0.1,
                [0.0; 3],
                identity,
                "map",
                timestamp,
                1,
                &[invalid],
            );
            let error = WireDecoder::new(&wire)
                .occupancy(expected)
                .expect_err("invalid occupancy value");
            assert!(error.contains("occupancy value"));
        }

        let truncated = raw_occupancy(1, 1, 0.1, [0.0; 3], identity, "map", timestamp, 1, &[]);
        let error = WireDecoder::new(&truncated)
            .occupancy(expected)
            .expect_err("truncated occupancy data");
        assert!(error.contains("truncated frame"));

        let mut trailing = raw_occupancy(
            1,
            1,
            0.1,
            [0.0; 3],
            identity,
            "map",
            timestamp,
            1,
            &[u8::MAX],
        );
        trailing.push(0);
        let mut decoder = WireDecoder::new(&trailing);
        let occupancy = decoder.occupancy(expected).expect("valid occupancy body");
        assert_eq!(occupancy.data, vec![-1]);
        let error = decoder.finish().expect_err("occupancy trailing byte");
        assert!(error.contains("trailing bytes"));
    }

    #[test]
    fn response_error_requires_exact_framing() {
        let mut encoder = WireEncoder::response(RESPONSE_ERROR);
        encoder.string("child failed").expect("encode error");
        let exact = encoder.finish().expect("exact error frame");
        let mut decoder = WireDecoder::new(&exact);
        let error = expect_response(&mut decoder, RESPONSE_COMPUTE).expect_err("child error");
        assert_eq!(error, "child failed");

        let mut trailing = exact;
        trailing.push(0);
        let mut decoder = WireDecoder::new(&trailing);
        let error = expect_response(&mut decoder, RESPONSE_COMPUTE)
            .expect_err("trailing error response must be protocol failure");
        assert!(error.contains("trailing bytes"), "{error}");
        assert_ne!(error, "child failed");
    }

    #[test]
    fn malformed_inner_checkpoint_is_transactional_through_restore_request() {
        let spec = MappingBackendSpec::raytracing_defaults();
        let config = sample_compute_config();
        let mut kernel = spec.into_kernel(config.window_capacity);
        let before = export_mapping_kernel_checkpoint(&kernel).expect("empty ray checkpoint");
        let mut corrupt = before.clone();
        corrupt[0] ^= 0xff;
        let request = encode_restore_request(91, &corrupt).expect("encode corrupt restore");
        let mut last_input = None;
        let mut pending = None;
        let (response, keep_running) =
            handle_sidecar_request(&request, &mut kernel, config, &mut last_input, &mut pending)
                .expect("restore failure is encoded as a response");
        assert!(keep_running);
        assert!(pending.is_none());
        assert_eq!(
            export_mapping_kernel_checkpoint(&kernel).expect("kernel remains exportable"),
            before
        );
        let mut decoder = WireDecoder::new(&response);
        expect_response(&mut decoder, RESPONSE_RESTORE).expect("restore response tag");
        assert_eq!(decoder.u128().expect("transaction"), 91);
        assert_eq!(decoder.u8().expect("result"), RESULT_ERROR);
        assert!(decoder.string().expect("restore error").contains("magic"));
        decoder.finish().expect("exact restore response");
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn stopped_child_write_block_times_out_below_parent_budget_and_is_reaped() {
        use std::os::fd::AsRawFd;

        let mut command = Command::new("/bin/sh");
        command
            .arg("-c")
            .arg("sleep 60 & wait")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null());
        command.process_group(0);
        let mut child = command.spawn().expect("spawn stopped sidecar stand-in");
        let child_pid = child.id();
        let request = child.stdin.take().expect("stand-in stdin");
        // SAFETY: F_GETPIPE_SZ only queries the valid pipe descriptor owned by request.
        let pipe_capacity =
            unsafe { nix::libc::fcntl(request.as_raw_fd(), nix::libc::F_GETPIPE_SZ) };
        assert!(pipe_capacity > 0, "query child stdin pipe capacity");
        let pipe_capacity = usize::try_from(pipe_capacity).expect("positive pipe capacity");
        assert!(pipe_capacity < MAX_MAPPING_BACKEND_CHECKPOINT_BYTES);
        let response = child.stdout.take().expect("stand-in stdout");
        let mut sidecar = SupervisedRaySidecar {
            child,
            request: BufWriter::new(request),
            response: BufReader::new(response),
            compute_config: sample_compute_config(),
            reaped: false,
        };
        let raw_pid = i32::try_from(child_pid).expect("child pid fits i32");
        nix::sys::signal::kill(
            nix::unistd::Pid::from_raw(-raw_pid),
            nix::sys::signal::Signal::SIGSTOP,
        )
        .expect("stop sidecar process group");

        let payload = vec![0_u8; pipe_capacity + 1];
        let frame = encode_restore_request(17, &payload).expect("large restore request");
        assert!(payload.len() > pipe_capacity);
        assert!(frame.len() > pipe_capacity);
        let started = Instant::now();
        let error = sidecar
            .exchange(&frame, RESPONSE_RESTORE)
            .expect_err("write-blocked stopped child must time out");
        assert!(error.contains("SIDECAR_TIMEOUT"), "{error}");
        sidecar.kill_and_reap();
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "sidecar timeout and reap exceeded parent budget: {:?}",
            started.elapsed()
        );
        assert_eq!(
            nix::sys::signal::kill(nix::unistd::Pid::from_raw(raw_pid), None),
            Err(nix::errno::Errno::ESRCH)
        );
    }
}

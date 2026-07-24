"""Lightweight Python client for the ``raytrace_json`` process boundary."""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Mapping, Sequence


class RayTraceProcessError(RuntimeError):
    """The native boundary rejected or could not execute a request."""

    def __init__(self, returncode: int, message: str) -> None:
        super().__init__(message)
        self.returncode = returncode


DEFAULT_TIMEOUT_SECONDS = 30.0
MAX_REQUEST_BYTES = 32 * 1024 * 1024
MAX_RESPONSE_BYTES = 64 * 1024 * 1024
MAX_DIAGNOSTIC_BYTES = 1024 * 1024
TERMINATION_GRACE_SECONDS = 0.2
THREAD_JOIN_GRACE_SECONDS = 0.2


def _communicate_bounded(command: Sequence[str], request: bytes, timeout: float) -> tuple[int, bytes, bytes]:
    use_process_group = os.name == "posix"
    process = subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
        start_new_session=use_process_group,
    )
    assert process.stdin is not None
    assert process.stdout is not None
    assert process.stderr is not None

    outputs = {"stdout": bytearray(), "stderr": bytearray()}
    primary_error: list[str] = []
    cleanup_errors: list[str] = []
    closed_streams: set[str] = set()
    state_lock = threading.Lock()
    kill_signal = getattr(signal, "SIGKILL", signal.SIGTERM)

    def record_primary_error(message: str) -> None:
        with state_lock:
            if not primary_error:
                primary_error.append(message)

    def record_cleanup_error(message: str) -> None:
        with state_lock:
            cleanup_errors.append(message)

    def close_stream(name: str, stream: Any) -> Exception | None:
        with state_lock:
            if name in closed_streams:
                return None
            closed_streams.add(name)
        try:
            stream.close()
        except BrokenPipeError:
            return None
        except (OSError, ValueError) as error:
            return error
        return None

    def signal_direct_child(action_name: str, *, terminate: bool) -> None:
        action = process.terminate if terminate else process.kill
        try:
            action()
        except ProcessLookupError:
            pass
        except OSError as error:
            record_cleanup_error(f"direct child {action_name} failed: {error}")

    def signal_process_tree(group_signal: int, action_name: str, *, terminate: bool) -> None:
        if not use_process_group:
            signal_direct_child(action_name, terminate=terminate)
            return
        try:
            os.killpg(process.pid, group_signal)
        except ProcessLookupError:
            signal_direct_child(action_name, terminate=terminate)
        except OSError as error:
            record_cleanup_error(f"process group {action_name} failed: {error}")
            signal_direct_child(action_name, terminate=terminate)

    def wait_for_child(stage: str, *, record_timeout: bool) -> bool:
        try:
            process.wait(timeout=TERMINATION_GRACE_SECONDS)
            return True
        except subprocess.TimeoutExpired:
            if process.poll() is not None:
                return True
            if record_timeout:
                record_cleanup_error(
                    f"direct child reap after {stage} timed out after {TERMINATION_GRACE_SECONDS} seconds"
                )
            return False
        except (OSError, subprocess.SubprocessError) as error:
            record_cleanup_error(f"direct child wait after {stage} failed: {error}")
            return False

    def read_stream(name: str, limit: int) -> None:
        stream = process.stdout if name == "stdout" else process.stderr
        try:
            while chunk := os.read(stream.fileno(), 64 * 1024):
                destination = outputs[name]
                remaining = limit + 1 - len(destination)
                if remaining > 0:
                    destination.extend(chunk[:remaining])
                if len(destination) > limit:
                    record_primary_error(f"raytrace_json {name} exceeded the {limit}-byte limit")
                    signal_process_tree(kill_signal, "SIGKILL", terminate=False)
                    break
        except (OSError, ValueError) as error:
            record_primary_error(f"failed to read raytrace_json {name}: {error}")
            signal_process_tree(kill_signal, "SIGKILL", terminate=False)
        finally:
            if error := close_stream(name, stream):
                record_primary_error(f"failed to close raytrace_json {name}: {error}")
                signal_process_tree(kill_signal, "SIGKILL", terminate=False)

    def write_request() -> None:
        try:
            request_view = memoryview(request)
            while request_view:
                written = process.stdin.write(request_view)
                if written is None or written <= 0:
                    raise OSError("raytrace_json stdin accepted zero bytes")
                request_view = request_view[written:]
        except BrokenPipeError:
            pass
        except (OSError, ValueError) as error:
            record_primary_error(f"failed to write raytrace_json stdin: {error}")
            signal_process_tree(kill_signal, "SIGKILL", terminate=False)
        finally:
            if error := close_stream("stdin", process.stdin):
                record_primary_error(f"failed to close raytrace_json stdin: {error}")
                signal_process_tree(kill_signal, "SIGKILL", terminate=False)

    threads = [
        threading.Thread(
            target=read_stream,
            args=("stdout", MAX_RESPONSE_BYTES),
            name="raytrace-stdout-reader",
            daemon=True,
        ),
        threading.Thread(
            target=read_stream,
            args=("stderr", MAX_DIAGNOSTIC_BYTES),
            name="raytrace-stderr-reader",
            daemon=True,
        ),
        threading.Thread(target=write_request, name="raytrace-stdin-writer", daemon=True),
    ]
    for thread in threads:
        thread.start()

    deadline = time.monotonic() + timeout
    timed_out = False
    try:
        returncode = process.wait(timeout=max(0.0, deadline - time.monotonic()))
    except subprocess.TimeoutExpired:
        timed_out = True
        returncode = -1

    if not timed_out:
        for thread in threads:
            thread.join(max(0.0, deadline - time.monotonic()))
        timed_out = any(thread.is_alive() for thread in threads)

    if timed_out:
        record_primary_error(f"raytrace_json timed out after {timeout} seconds")
        signal_process_tree(signal.SIGTERM, "SIGTERM", terminate=True)
        wait_for_child("SIGTERM", record_timeout=False)
        signal_process_tree(kill_signal, "SIGKILL", terminate=False)
        if not wait_for_child("SIGKILL", record_timeout=False):
            signal_direct_child("SIGKILL", terminate=False)
            wait_for_child("direct SIGKILL", record_timeout=True)
        for name, stream in (
            ("stdin", process.stdin),
            ("stdout", process.stdout),
            ("stderr", process.stderr),
        ):
            if error := close_stream(name, stream):
                record_cleanup_error(f"failed to close {name}: {error}")
        join_deadline = time.monotonic() + THREAD_JOIN_GRACE_SECONDS
        for thread in threads:
            thread.join(max(0.0, join_deadline - time.monotonic()))
        alive_threads = [thread.name for thread in threads if thread.is_alive()]
        if alive_threads:
            record_cleanup_error(f"I/O threads did not stop: {', '.join(alive_threads)}")
        try:
            process.poll()
        except OSError as error:
            record_cleanup_error(f"direct child final poll failed: {error}")
        if process.returncode is None:
            record_cleanup_error("direct child cleanup incomplete: process remains alive or unreaped")

    def with_cleanup_errors(message: str) -> str:
        with state_lock:
            errors = tuple(cleanup_errors)
        if not errors:
            return message
        return f"{message}; cleanup errors: {'; '.join(errors)}"

    if primary_error:
        raise RayTraceProcessError(-1, with_cleanup_errors(primary_error[0]))
    return returncode, bytes(outputs["stdout"]), bytes(outputs["stderr"])


def run_raytrace(
    request: Mapping[str, Any],
    *,
    binary: str | os.PathLike[str] | None = None,
    pretty: bool = False,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
    extra_args: Sequence[str] = (),
) -> dict[str, Any]:
    """Run one request and return the decoded native result.

    ``binary`` defaults to ``NUWAX_RAYTRACE_JSON`` and then ``raytrace_json``
    on ``PATH``. Nonzero native exits and malformed output fail closed.
    """

    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise ValueError("timeout must be a positive finite number")
    timeout = float(timeout)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("timeout must be a positive finite number")

    request_bytes = json.dumps(request, allow_nan=False).encode("utf-8")
    if len(request_bytes) > MAX_REQUEST_BYTES:
        raise ValueError(f"request exceeds the {MAX_REQUEST_BYTES}-byte limit")

    executable = Path(binary) if binary is not None else os.environ.get("NUWAX_RAYTRACE_JSON", "raytrace_json")
    command = [os.fspath(executable)]
    if pretty:
        command.append("--pretty")
    command.extend(extra_args)
    try:
        returncode, stdout, stderr = _communicate_bounded(command, request_bytes, timeout)
    except (OSError, subprocess.SubprocessError) as error:
        raise RayTraceProcessError(-1, f"failed to run raytrace_json: {error}") from error
    if returncode != 0:
        detail = stderr.decode("utf-8", errors="replace").strip() or "raytrace_json failed without diagnostics"
        raise RayTraceProcessError(returncode, detail)
    try:
        response = json.loads(stdout)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise RayTraceProcessError(4, f"invalid JSON from raytrace_json: {error}") from error
    if not isinstance(response, dict):
        raise RayTraceProcessError(4, "raytrace_json response must be a JSON object")
    return response

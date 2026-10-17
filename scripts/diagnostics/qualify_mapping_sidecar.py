#!/usr/bin/env python3
"""Wire-only production sidecar qualification; Python 3.8-compatible, stdlib only.

No ray/occupancy/checkpoint implementation. Expected candidates and opaque checkpoint
bytes come from the previously recorded real-kernel lifecycle fixtures. This program
never invokes Cargo. --self-check never spawns a process.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import time

MAGIC = b"NWRAYS01"
SIDECAR_ARG = "--nuwax-mapping-raytracing-sidecar-v1"
LIMIT = 134_250_000  # Above the two 64 MiB occupancy response bound; hard read cap.
TIMEOUT = 3.0  # Same exchange bound as SIDECAR_RESPONSE_TIMEOUT; not a speed metric.


def require(value, message):
    if not value:
        raise AssertionError(message)


def integer(value, size):
    return int(value).to_bytes(size, "little", signed=False)


def string(value):
    encoded = value.encode("utf-8")
    require(len(encoded) <= 4096, "wire string too long")
    return integer(len(encoded), 4) + encoded


def blob(value):
    return integer(len(value), 8) + value


def message(tag, payload=b""):
    return MAGIC + integer(tag, 1) + payload


def framed(body):
    return struct.pack(">Q", len(body)) + body


def f64_bits(value):
    return struct.unpack("<Q", struct.pack("<d", value))[0]


def occupancy(grid):
    result = integer(grid["width"], 4) + integer(grid["height"], 4)
    result += integer(grid["resolution_bits"], 8)
    result += b"".join(integer(v, 8) for v in grid["origin_bits"])
    result += b"".join(integer(v, 8) for v in grid["orientation_bits"])
    result += string(grid["frame_id"]) + integer(grid["timestamp_bits"], 8)
    cells = grid["cells"]
    require(len(cells) == grid["width"] * grid["height"], "invalid fixture grid")
    require(all(-1 <= v <= 100 for v in cells), "invalid fixture cell")
    return result + blob(bytes(v & 255 for v in cells))


def candidate_bytes(candidate):
    require(candidate["backend_kind"] == "raytracing", "fixture is not raytracing")
    result = string(candidate["session_id"])
    for name in ("session_generation", "backend_generation", "source_frame_sequence",
                 "source_timestamp_ns", "received_at_ns"):
        result += integer(candidate[name], 8)
    # accepted_xyz_bits is deliberately absent from the real sidecar wire protocol.
    # Its exact bits are reused below as request input, never asserted as wire output.
    return result + b"\x01" + occupancy(candidate["live"]) + occupancy(candidate["latest"])


def input_from_candidate(candidate, fallback_points=None):
    points = candidate["accepted_xyz_bits"] or fallback_points
    require(points is not None and len(points) > 0, "missing authentic fixture points")
    return {
        "session_id": candidate["session_id"],
        "session_generation": candidate["session_generation"],
        "backend_generation": candidate["backend_generation"],
        "frame_sequence": candidate["source_frame_sequence"],
        "source_timestamp_ns": candidate["source_timestamp_ns"],
        "received_at_ns": candidate["received_at_ns"],
        "registered_frame_id": candidate["live"]["frame_id"],
        # frame() in mapping_backend_ray_count_equivalence_tests.rs, not robot data.
        "sensor_origin_bits": [f64_bits(0.025), f64_bits(0.025), f64_bits(0.0)],
        "points_bits": copy.deepcopy(points),
    }


def token(input_value, transaction):
    result = integer(transaction, 16) + string(input_value["session_id"])
    for name in ("session_generation", "backend_generation", "frame_sequence",
                 "source_timestamp_ns", "received_at_ns"):
        result += integer(input_value[name], 8)
    return result + integer(0, 8)  # base_occupancy_revision, as in token_for().


def input_bytes(input_value):
    result = string(input_value["session_id"])
    for name in ("session_generation", "backend_generation", "frame_sequence",
                 "source_timestamp_ns", "received_at_ns"):
        result += integer(input_value[name], 8)
    result += string(input_value["registered_frame_id"])
    result += b"".join(integer(v, 8) for v in input_value["sensor_origin_bits"])
    points = input_value["points_bits"]
    result += integer(len(points), 8)
    return result + b"".join(integer(v, 8) for point in points for v in point)


def start_bytes():
    # five_cm_spec(): defaults from RaytracingBackendSpec, voxel_size only = .05.
    spec = struct.pack("<ffIffiifI", 0.05, 50.0, 1, 0.2, 0.2, -2, 1, 0.7, 15)
    # five_cm_config(): .05m occupancy, projection tag5, zero safe radius, 16384.
    config = struct.pack("<dBfQ", 0.05, 5, 0.0, 16384)
    return message(1, spec + config)


def load_fixtures(directory):
    lifecycle = json.loads((directory / "ray-count-kernel-lifecycle.json").read_text())
    historical = json.loads((directory / "ray-count-restore-original-counter.json").read_text())
    observations = lifecycle["observations"]
    require([v["kind"] for v in observations] ==
            ["candidate", "candidate", "candidate", "skipped", "flush", "continued"],
            "fixture observation sequence changed; review driver")
    candidates = [v["candidate"] for v in observations if "candidate" in v]
    inputs = [input_from_candidate(v, candidates[0]["accepted_xyz_bits"]) for v in candidates]
    require([v["frame_sequence"] for v in inputs] == [1, 2, 3, 4, 5], "fixture sources changed")
    require(inputs[3]["received_at_ns"] - inputs[2]["received_at_ns"] == 1_000_000,
            "missing actual 1ms skip fixture")
    require(all(v["points_bits"] == inputs[0]["points_bits"] for v in inputs),
            "fixture repeated points changed; do not silently invent skip input")
    require(len(inputs[0]["points_bits"]) == 36, "expected real 36-point fixture")
    for candidate in candidates + [historical["continued_candidate"]]:
        candidate_bytes(candidate)  # Validate all saved fields/cells offline as well.
        require(candidate["live"]["resolution_bits"] == f64_bits(0.05),
                "fixture occupancy config changed")
    require(len(start_bytes()) == 66, "START layout mismatch")
    for name in ("checkpoint", "continued_checkpoint"):
        require(bytes(lifecycle[name]).startswith(b"NWMBCKP\0"), "missing authentic checkpoint")
    require(bytes(lifecycle["checkpoint"])[16:52] == start_bytes()[9:45],
            "START ray spec differs from saved checkpoint config")
    return lifecycle, historical, candidates, inputs


class Session:
    def __init__(self, binary, directory, name):
        self.path = directory / name
        self.path.mkdir()
        self.trace = []
        self.index = 0
        self.closed = False
        self.stderr = (self.path / "stderr.log").open("wb")
        env = dict(os.environ)
        # main checks video-helper environment before the sidecar argument.
        for key in list(env):
            if key.startswith("NUWAX_VIDEO_RELAY_HELPER_"):
                del env[key]
        env["NUWAX_LOG_DIR"] = str(self.path / "logs")
        env["TMPDIR"] = str(directory / "scratch")
        self.process = subprocess.Popen(
            [str(binary), SIDECAR_ARG], cwd=str(self.path), env=env,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr,
            bufsize=0, close_fds=True, start_new_session=True)
        os.set_blocking(self.process.stdin.fileno(), False)
        os.set_blocking(self.process.stdout.fileno(), False)
        self.record({"event": "spawn", "pid": self.process.pid,
                     "argv": [str(binary), SIDECAR_ARG]})

    def record(self, value):
        self.trace.append(value)
        (self.path / "trace.json").write_text(json.dumps(self.trace, indent=2) + "\n")

    def read_exact(self, count, deadline):
        result = bytearray()
        while len(result) < count:
            remaining = deadline - time.monotonic()
            require(remaining > 0, "sidecar response exceeded 3 seconds")
            ready, _, _ = select.select([self.process.stdout], [], [], remaining)
            require(ready, "sidecar response exceeded 3 seconds")
            part = os.read(self.process.stdout.fileno(), count - len(result))
            require(part, "unexpected response EOF")
            result.extend(part)
        return bytes(result)

    def exchange(self, request, expected=None, candidate=None, error_contains=None):
        self.index += 1
        stem = "{:03d}-tag{}".format(self.index, request[8])
        wire = framed(request)
        (self.path / (stem + ".request.bin")).write_bytes(wire)
        deadline = time.monotonic() + TIMEOUT
        offset = 0
        while offset < len(wire):
            remaining = deadline - time.monotonic()
            require(remaining > 0, "sidecar request exceeded 3 seconds")
            _, ready, _ = select.select([], [self.process.stdin], [], remaining)
            require(ready, "sidecar request exceeded 3 seconds")
            offset += os.write(self.process.stdin.fileno(), wire[offset:])
        header = self.read_exact(8, deadline)
        size = struct.unpack(">Q", header)[0]
        require(9 <= size <= LIMIT, "invalid response frame size {}".format(size))
        response = self.read_exact(size, deadline)
        (self.path / (stem + ".response.bin")).write_bytes(header + response)
        detail = {"event": "exchange", "request": stem + ".request.bin",
                  "response": stem + ".response.bin", "response_tag": response[8]}
        # Record wire evidence before assertions, so failures remain inspectable.
        self.record(detail)
        require(response[:8] == MAGIC, "unexpected protocol magic / stdout contamination")
        if candidate is not None:
            require(response[:-8] == expected + candidate_bytes(candidate),
                    "candidate business bytes differ: " + str(self.path / stem))
            detail["compute_elapsed_us"] = int.from_bytes(response[-8:], "little")
            self.record({"event": "candidate_equal", "exchange": self.index,
                         "compute_elapsed_us": detail["compute_elapsed_us"]})
        elif error_contains is not None:
            require(response[8] == 199 and len(response) >= 13, "expected fatal protocol error")
            length = int.from_bytes(response[9:13], "little")
            require(len(response) == 13 + length, "invalid protocol error framing")
            error = response[13:].decode("utf-8")
            require(error_contains in error, "unexpected protocol error: " + error)
            self.record({"event": "expected_protocol_error", "error": error})
        else:
            require(response == expected, "response bytes differ: " + str(self.path / stem))
        return response

    def start(self):
        self.exchange(start_bytes(), message(101))

    def compute(self, value, transaction, candidate=None, error=None):
        request = message(2, integer(transaction, 16) + token(value, transaction) + input_bytes(value))
        prefix = message(102, integer(transaction, 16))
        if error is not None:
            return self.exchange(request, prefix + b"\0" + string(error))
        return self.exchange(request, prefix + (b"\x02" if candidate else b"\x01"), candidate)

    def flush(self, value, transaction, candidate=None, error=None):
        request = message(3, integer(transaction, 16) + token(value, transaction))
        prefix = message(102, integer(transaction, 16))
        if error is not None:
            return self.exchange(request, prefix + b"\0" + string(error))
        return self.exchange(request, prefix + (b"\x02" if candidate else b"\x01"), candidate)

    def resolve(self, transaction, commit=True):
        self.exchange(message(6, integer(transaction, 16) + bytes([int(commit)])),
                      message(105, integer(transaction, 16)))

    def export(self, checkpoint=None, error=None):
        expected = message(103, b"\0" + string(error)) if error else message(103, b"\x01" + blob(checkpoint))
        self.exchange(message(4), expected)

    def restore(self, transaction, checkpoint):
        self.exchange(message(5, integer(transaction, 16) + blob(checkpoint)),
                      message(104, integer(transaction, 16) + b"\x01"))

    def finish(self, expected_exit=0, stop=False):
        if stop:
            self.exchange(message(7), message(106))
        self.process.stdin.close()
        exit_code = self.process.wait(timeout=TIMEOUT)
        require(exit_code == expected_exit,
                "exit code {} != {}".format(exit_code, expected_exit))
        require(os.read(self.process.stdout.fileno(), 1) == b"", "unexpected output after terminal response")
        self.process.stdout.close()
        self.stderr.close()
        self.closed = True
        self.record({"event": "reaped", "exit_code": exit_code, "stdout_eof": True})

    def cleanup(self):
        if not self.closed:
            # Failure cleanup is not a qualified normal shutdown. Only this local PID.
            if self.process.poll() is None:
                self.process.kill()
            self.process.wait(timeout=TIMEOUT)
            self.process.stdin.close()
            self.process.stdout.close()
            self.stderr.close()
            self.closed = True
            self.record({"event": "failure_cleanup", "exit_code": self.process.returncode})


def qualify(binary, output, fixtures):
    lifecycle, historical, candidates, inputs = fixtures
    checkpoint = bytes(lifecycle["checkpoint"])
    continued = bytes(lifecycle["continued_checkpoint"])
    output.mkdir(parents=True, exist_ok=False)
    (output / "scratch").mkdir()
    (output / "input-manifest.json").write_text(json.dumps(inputs, indent=2) + "\n")
    (output / "baseline-checkpoint.bin").write_bytes(checkpoint)
    (output / "baseline-continued-checkpoint.bin").write_bytes(continued)
    passed = []

    def run(name, action):
        session = Session(binary, output, name)
        try:
            session.start()
            action(session)
            require(session.closed, "case did not assert terminal child exit")
            passed.append(name)
            (output / "passed-cases.json").write_text(json.dumps(passed, indent=2) + "\n")
        finally:
            session.cleanup()

    def warm_dirty(session):
        for i in range(3):
            session.compute(inputs[i], i + 1, candidates[i])
            session.resolve(i + 1)
        session.compute(inputs[3], 4)
        session.resolve(4)

    def lifecycle_case(session):
        warm_dirty(session)
        session.export(error=lifecycle["observations"][3]["export_error"])
        wrong = copy.deepcopy(inputs[3])
        wrong["frame_sequence"] += 1
        session.flush(wrong, 40, error=lifecycle["observations"][3]["wrong_flush_error"])
        session.flush(inputs[3], 41, candidates[3])
        session.resolve(41)
        session.export(checkpoint)
        session.restore(100, checkpoint)
        session.resolve(100)
        session.export(checkpoint)
        # RESTORE clears sidecar last_input; this is the actual protocol clean path.
        session.flush(inputs[3], 101)
        session.resolve(101)
        session.export(checkpoint)
        session.compute(inputs[4], 5, candidates[4])
        session.resolve(5)
        session.export(continued)
        empty = copy.deepcopy(inputs[4])
        empty.update(frame_sequence=6, source_timestamp_ns=1_251_000_000,
                     received_at_ns=1_251_000_000, points_bits=[])
        session.compute(empty, 6)
        session.resolve(6)
        session.export(continued)
        invalid = copy.deepcopy(empty)
        invalid["points_bits"] = copy.deepcopy(inputs[4]["points_bits"])
        invalid["points_bits"][0][0] = 0x7ff8000000000000
        session.compute(invalid, 7, error=lifecycle["invalid_input_error"])
        session.export(continued)  # Error created no pending transaction and no mutation.
        session.finish(stop=True)

    run("01-lifecycle", lifecycle_case)

    def restore_continue(session):
        session.restore(200, checkpoint)
        session.resolve(200)
        session.export(checkpoint)
        session.compute(inputs[4], 5, candidates[4])
        session.resolve(5)
        session.export(continued)
        session.finish(stop=True)

    run("02-fresh-process-restore-continue", restore_continue)

    def historical_counter(session):
        saved = bytes(historical["restored_checkpoint"])
        session.restore(300, saved)
        session.resolve(300)
        session.export(saved)
        session.flush(inputs[2], 301)
        session.resolve(301)
        session.export(saved)
        expected = historical["continued_candidate"]
        next_input = input_from_candidate(expected)
        session.compute(next_input, 302, expected)
        session.resolve(302)
        session.export(bytes(historical["continued_checkpoint"]))
        session.finish(stop=True)

    run("03-preserve-historical-counter", historical_counter)

    def cancel_compute(session):
        session.compute(inputs[0], 1, candidates[0])
        session.resolve(1, False)
        session.finish()

    run("04-cancel-compute-exits", cancel_compute)

    def cancel_flush(session):
        warm_dirty(session)
        session.flush(inputs[3], 41, candidates[3])
        session.resolve(41, False)
        session.finish()

    run("05-cancel-flush-exits", cancel_flush)

    def cancel_restore(session):
        session.restore(400, checkpoint)
        session.resolve(400, False)
        session.finish()

    run("06-cancel-restore-exits", cancel_restore)

    def stop_pending(session):
        session.compute(inputs[0], 1, candidates[0])
        session.finish(stop=True)

    run("07-stop-pending", stop_pending)

    def eof_pending(session):
        session.compute(inputs[0], 1, candidates[0])
        session.finish()

    run("08-eof-pending", eof_pending)

    def wrong_resolution(session):
        session.compute(inputs[0], 1, candidates[0])
        session.exchange(message(6, integer(999, 16) + b"\x01"),
                         error_contains="resolution does not match pending transaction")
        session.finish(expected_exit=1)

    run("09-wrong-resolution-fails-closed", wrong_resolution)

    def pending_export(session):
        session.compute(inputs[0], 1, candidates[0])
        session.exchange(message(4), error_contains="not allowed for Some(Request(PendingResolution))")
        session.finish(expected_exit=1)

    run("10-unresolved-export-fails-closed", pending_export)

    def mismatched_source(session):
        wrong = copy.deepcopy(inputs[0])
        wrong["source_timestamp_ns"] += 1
        request = message(2, integer(1, 16) + token(inputs[0], 1) + input_bytes(wrong))
        session.exchange(request, error_contains="token/input identity mismatch")
        session.finish(expected_exit=1)

    run("11-source-identity-fails-closed", mismatched_source)
    return passed


def production_artifact(build_json, binary):
    rows = []
    for line in build_json.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue
    require(any(v.get("reason") == "build-finished" and v.get("success") is True for v in rows),
            "build JSON does not confirm success")
    artifacts = [v for v in rows if v.get("reason") == "compiler-artifact"
                 and v.get("target", {}).get("name") == "nuwax-geometry-runtime"
                 and v.get("target", {}).get("kind") == ["bin"]
                 and v.get("profile", {}).get("test") is False
                 and v.get("executable")
                 and Path(v["executable"]).resolve() == binary]
    require(len(artifacts) == 1, "need one actual non-test binary artifact, not test --no-run")
    require(binary.name == "nuwax-geometry-runtime" and binary.parent.name == "release",
            "only ordinary existing release binary is accepted")
    require(binary.is_file() and os.access(str(binary), os.X_OK), "production binary missing")
    return artifacts[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--build-json", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    fixtures = load_fixtures(args.baseline.resolve())
    if args.self_check:
        print("PASS offline fixture/codec-shape checks only; no binary launched; Python "
              + sys.version.split()[0])
        return
    require(args.binary and args.build_json and args.output,
            "--binary, --build-json and --output required for process qualification")
    binary = args.binary.resolve()
    artifact = production_artifact(args.build_json.resolve(), binary)
    before = binary.stat()
    output = args.output.resolve()
    passed = qualify(binary, output, fixtures)
    after = binary.stat()
    require((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) ==
            (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            "binary was replaced/changed during qualification")
    (output / "production-artifact.json").write_text(json.dumps(artifact, indent=2) + "\n")
    (output / "result.json").write_text(json.dumps({
        "status": "passed", "cases": passed, "binary": str(binary),
        "python_version": sys.version.split()[0],
        "python_implementation": sys.implementation.name,
        "build_json": str(args.build_json.resolve()), "baseline": str(args.baseline.resolve()),
        "binary_size": after.st_size, "binary_mtime_ns": after.st_mtime_ns,
        "scope": "real non-test binary child stdio; exact candidate wire business bytes and opaque checkpoint bytes",
        "excluded": ["compute_elapsed_us", "parent supervisor/commit-owner execution",
                     "real robot input or LCM transport", "ARM", "performance qualification"],
    }, indent=2) + "\n")
    print("PASS {} actual production sidecar process cases; {}".format(len(passed), output))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Serial opt-in synthetic-worker qualification; Python 3.8 compatible.

Build the Runtime test binary first, then supply its exact path. This runner never
builds production code, changes deadlines, retries a failed arm, or talks to a robot.
The existing six real-boundary memq qualifications remain separate and unchanged.
"""
import argparse
import datetime
import json
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--stage", choices=("short", "full", "all"), default="all")
    args = parser.parse_args()
    binary = args.binary.resolve(strict=True)
    args.output.mkdir(parents=True, exist_ok=True)
    report = {
        "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "binary": str(binary), "binary_size": binary.stat().st_size,
        "binary_mtime_ns": binary.stat().st_mtime_ns,
        "seed": 20261013, "runs": [],
        "scope": "Persistent real RouteFollower/PlannerWorker, ideal synthetic plant; not NavScheduler/memq/physical motion-port qualification",
        "limits": {"goal_seconds": 60, "step_map_recheck_ms": 50,
                   "consumer_gap_ms": 75, "source_health_seconds": 1,
                   "position_tolerance_m": 0.15, "yaw_tolerance_rad": 0.15},
        "memory": "RSS is diagnostic only; no leak pass/fail threshold",
    }
    stages = ("short", "full") if args.stage == "all" else (args.stage,)
    for stage in stages:
        failed = False
        for mode in ("off", "on"):
            suffix = "short" if stage == "short" else "5min"
            test = "route_moving_worker_tests::sustained_worker::sustained_worker_{}_{}".format(suffix, mode)
            command = [str(binary), test, "--exact", "--ignored", "--nocapture", "--test-threads=1"]
            log = args.output / ("{}-{}.log".format(stage, mode))
            if log.exists():
                raise SystemExit("Refusing to overwrite prior evidence: {}".format(log))
            print("RUN {} -> {}".format(test, log), flush=True)
            started = time.monotonic()
            with log.open("w") as stream:
                try:
                    result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=1200)
                    returncode = result.returncode
                except subprocess.TimeoutExpired:
                    returncode = "runner_timeout_1200s"
            lines = log.read_text().splitlines()
            markers = [line for line in lines if line.startswith("SUSTAINED_") or line.startswith("PROOF_RECEIPTS")]
            passed = returncode == 0 and any(line.startswith("SUSTAINED_DONE") for line in markers)
            report["runs"].append({"stage": stage, "mode": mode, "command": command,
                                  "returncode": returncode, "passed": passed,
                                  "wall_seconds": time.monotonic() - started,
                                  "log": str(log.resolve()), "markers": markers})
            (args.output / "results.json").write_text(json.dumps(report, indent=2) + "\n")
            print("RESULT {} passed={} returncode={}".format(test, passed, returncode), flush=True)
            failed = failed or not passed
        if failed:
            raise SystemExit("Failures preserved; later stages not run. Diagnose without relaxing criteria.")


if __name__ == "__main__":
    main()

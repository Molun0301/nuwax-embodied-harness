#!/usr/bin/env python3
"""独立离线建图 CLI 生产进程根。"""
from __future__ import annotations

import argparse
import json
import signal
import sys
from pathlib import Path
from typing import Optional, Sequence

_SERVICES_ROOT = Path(__file__).resolve().parents[1]
for module_root in (_SERVICES_ROOT / "geometry", _SERVICES_ROOT / "mapping"):
    if str(module_root) not in sys.path:
        sys.path.insert(0, str(module_root))

from release_manifest import ReleaseProof, verify_release_manifest
from offline_cli_owner import (
    MapCommand,
    OfflineCliReleaseProof,
    OfflineCliRequest,
    OfflineMappingCliOwner,
)


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="运行独立离线地图资产发布请求")
    parser.add_argument("--release-manifest", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--state-root", type=Path, required=True)
    parser.add_argument("--state-auth-key-file", type=Path, required=True)
    parser.add_argument("--request-id", required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--frame-id", required=True)
    parser.add_argument("--timeout-seconds", type=float, required=True)
    parser.add_argument("--theme", default="default")
    parser.add_argument("--resolution", type=float, default=0.1)
    return parser.parse_args(argv)


def _owner_release_proof(proof: ReleaseProof) -> OfflineCliReleaseProof:
    artifact = proof.artifact_for_role("offline_mapping_cli")
    if not artifact.executable or artifact.fd is None:
        raise RuntimeError("offline_mapping_cli release artifact lacks retained executable FD")
    return OfflineCliReleaseProof(
        generation=proof.generation,
        manifest_sha256=proof.manifest_sha256,
        artifact_sha256=artifact.sha256,
        artifact_size=artifact.size,
        artifact_path=str(artifact.path),
        artifact_fd=artifact.fd,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    proof = verify_release_manifest(
        args.release_manifest,
        retain_artifact_fds=True,
    )
    owner: Optional[OfflineMappingCliOwner] = None
    active_request_id: Optional[str] = None
    status = None
    received_signal: Optional[int] = None
    cancel_requested = False
    shutdown_error: Optional[BaseException] = None
    cleanup_error: Optional[BaseException] = None
    primary_error: Optional[BaseException] = None

    def request_stop(signum: int, _frame: object) -> None:
        nonlocal received_signal
        if received_signal is None:
            received_signal = signum

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, request_stop)
    try:
        owner_proof = _owner_release_proof(proof)
        owner = OfflineMappingCliOwner(
            owner_proof.artifact_path,
            release_proof=owner_proof,
            input_root=args.input_root,
            output_root=args.output_root,
            state_root=args.state_root,
            state_auth_key=args.state_auth_key_file.read_bytes(),
        )
        request = OfflineCliRequest(
            request_id=args.request_id,
            command=MapCommand(
                input_path=args.dataset,
                output_path=args.output,
                map_name=args.map_name,
                frame_id=args.frame_id,
                theme=args.theme,
                resolution=args.resolution,
            ),
            timeout_seconds=args.timeout_seconds,
        )
        status = owner.start(request)
        active_request_id = status.request_id
        while not status.terminal:
            if received_signal is not None and not cancel_requested:
                status = owner.cancel(active_request_id)
                cancel_requested = True
                continue
            try:
                status = owner.wait(active_request_id, timeout=0.2)
            except TimeoutError:
                continue
    except BaseException as error:
        primary_error = error
    finally:
        try:
            if owner is not None:
                try:
                    if (
                        active_request_id is not None
                        and received_signal is not None
                        and status is not None
                        and not status.terminal
                    ):
                        try:
                            status = owner.cancel(active_request_id)
                        except BaseException as error:
                            shutdown_error = error
                finally:
                    try:
                        owner.shutdown()
                    except BaseException as error:
                        shutdown_error = error
        finally:
            try:
                proof.close()
            except BaseException as error:
                cleanup_error = error

    if status is not None and status.terminal:
        output = dict(status.__dict__)
        output["runtime_signal"] = received_signal
        print(json.dumps(output, default=str, ensure_ascii=False, sort_keys=True))
    if shutdown_error is not None:
        raise shutdown_error
    if cleanup_error is not None:
        raise cleanup_error
    if primary_error is not None:
        raise primary_error
    if status is None or not status.terminal:
        raise RuntimeError("offline mapping CLI exited without a terminal status")
    if received_signal is not None:
        return 128 + received_signal
    return 0 if status.state.value == "succeeded" else 1


if __name__ == "__main__":
    sys.exit(main())

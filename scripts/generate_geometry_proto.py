#!/usr/bin/env python3
"""Generate the checked-in geometry Python protobuf bindings deterministically."""

from __future__ import annotations

import argparse
import importlib.metadata
import os
from pathlib import Path
import shutil
import tempfile
from typing import List

from grpc_tools import protoc


ROOT = Path(__file__).resolve().parents[1]
PROTO = (
    ROOT
    / "services"
    / "geometry"
    / "nuwax_geometry_runtime"
    / "proto"
    / "geometry_runtime.proto"
)
CHECKED_IN_DIR = ROOT / "services" / "geometry" / "proto_gen"
GENERATED_FILES = ("geometry_runtime_pb2.py", "geometry_runtime_pb2_grpc.py")
GRPC_TOOLS_VERSION = "1.70.0"
ABSOLUTE_PB2_IMPORT = "import geometry_runtime_pb2 as geometry__runtime__pb2"
RELATIVE_PB2_IMPORT = "from . import geometry_runtime_pb2 as geometry__runtime__pb2"


def _require_codegen_version() -> None:
    actual = importlib.metadata.version("grpcio-tools")
    if actual != GRPC_TOOLS_VERSION:
        raise RuntimeError(
            "geometry proto generation requires grpcio-tools=={}; found {}".format(
                GRPC_TOOLS_VERSION, actual
            )
        )


def _generate(output_dir: Path) -> None:
    _require_codegen_version()
    result = protoc.main(
        [
            "grpc_tools.protoc",
            "-I{}".format(PROTO.parent),
            "--python_out={}".format(output_dir),
            "--grpc_python_out={}".format(output_dir),
            str(PROTO),
        ]
    )
    if result != 0:
        raise RuntimeError("grpc_tools.protoc failed with exit code {}".format(result))

    grpc_module = output_dir / "geometry_runtime_pb2_grpc.py"
    source = grpc_module.read_text(encoding="utf-8")
    if source.count(ABSOLUTE_PB2_IMPORT) != 1:
        raise RuntimeError(
            "generated gRPC module did not contain the expected unique pb2 import"
        )
    grpc_module.write_text(
        source.replace(ABSOLUTE_PB2_IMPORT, RELATIVE_PB2_IMPORT),
        encoding="utf-8",
    )


def _changed_files(generated_dir: Path) -> List[str]:
    return [
        name
        for name in GENERATED_FILES
        if (generated_dir / name).read_bytes() != (CHECKED_IN_DIR / name).read_bytes()
    ]


def _install(generated_dir: Path) -> None:
    for name in GENERATED_FILES:
        destination = CHECKED_IN_DIR / name
        temporary = destination.with_name(destination.name + ".tmp")
        shutil.copyfile(generated_dir / name, temporary)
        os.replace(str(temporary), str(destination))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if checked-in bindings differ from deterministic generation",
    )
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="nuwax-geometry-proto-") as directory:
        generated_dir = Path(directory)
        _generate(generated_dir)
        if args.check:
            changed = _changed_files(generated_dir)
            if changed:
                parser.error(
                    "checked-in geometry bindings are stale: {}".format(", ".join(changed))
                )
        else:
            _install(generated_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

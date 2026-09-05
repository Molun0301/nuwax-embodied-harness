#!/usr/bin/env python3
"""Fail-closed static negative gates for release-critical code paths."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Optional, Pattern, Sequence, Tuple

try:
    from .common import FAIL, PASS, find_repo_root
except ImportError:  # Direct script execution.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from common import FAIL, PASS, find_repo_root


PRODUCTION_ROOTS = (
    "gateways",
    "providers",
    "services",
    "drivers",
    "core",
    "adapters",
    "scripts",
)

SOURCE_SUFFIXES = {
    ".c",
    ".cc",
    ".cpp",
    ".go",
    ".h",
    ".hh",
    ".hpp",
    ".json",
    ".proto",
    ".py",
    ".rs",
    ".sh",
    ".toml",
    ".yaml",
    ".yml",
}

PRUNE_DIR_NAMES = {
    ".git",
    ".omc",
    ".omx",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "node_modules",
    "release_gates",
    "runtime_data",
    "target",
    "tests",
}

GLOBAL_EXCLUDED_PREFIXES = (
    "scripts/ci/",
)


@dataclass(frozen=True)
class GateSpec:
    name: str
    description: str
    pattern_text: str
    include_prefixes: Tuple[str, ...] = ()
    exclude_prefixes: Tuple[str, ...] = ()
    flags: int = 0
    baseline: int = 0
    severity: str = "BLOCKING"

    def compile(self) -> Pattern[str]:
        return re.compile(self.pattern_text, self.flags | re.MULTILINE)


@dataclass(frozen=True)
class GateHit:
    file_path: str
    line_no: int
    line_text: str

    def to_dict(self) -> dict:
        return {
            "file": self.file_path,
            "line": self.line_no,
            "text": self.line_text.strip()[:200],
        }


@dataclass
class GateResult:
    spec: GateSpec
    hits: List[GateHit] = field(default_factory=list)

    @property
    def hit_count(self) -> int:
        return len(self.hits)

    def is_pass(self) -> bool:
        return self.hit_count <= self.spec.baseline

    def status(self) -> str:
        if self.is_pass():
            return "PASS"
        if self.spec.baseline:
            return "FAIL(baseline {} -> {})".format(
                self.spec.baseline, self.hit_count
            )
        return "FAIL"

    def to_dict(self) -> dict:
        return {
            "name": self.spec.name,
            "description": self.spec.description,
            "forbidden_pattern": self.spec.pattern_text,
            "severity": self.spec.severity,
            "baseline": self.spec.baseline,
            "hit_count": self.hit_count,
            "status": self.status(),
            "hits_sample": [hit.to_dict() for hit in self.hits[:5]],
        }


def build_gate_specs() -> List[GateSpec]:
    """Build the seven release-negative gates."""
    application_paths = (
        "gateways/",
        "providers/",
        "services/",
        "drivers/",
        "core/",
        "adapters/",
    )
    navigation_paths = (
        "gateways/",
        "providers/",
        "services/navigation/",
        "services/geometry/",
    )
    return [
        GateSpec(
            name="gate-1-active-ros2-fallback",
            description="检查实际 ROS2 导入和进程启动，兼容格式说明不属于运行依赖",
            pattern_text=(
                r"^\s*(?:from|import)\s+(?:rclpy|r2r|rclcpp|nav2_msgs|tf2_ros|rosidl_runtime_py)\b|"
                r"^\s*use\s+(?:r2r|rclrs)::|"
                r"^\s*(?:find_package|ament_target_dependencies)\s*\([^\n)]*\b(?:rclcpp|rclpy|nav2_msgs|rtabmap_ros)\b|"
                r"^\s*(?:exec\s+)?(?:ros2|colcon|rosdep)\s+|"
                r"[\"']ros2[\"']\s*,\s*[\"'](?:run|launch)[\"']"
            ),
            include_prefixes=application_paths,
            flags=re.IGNORECASE,
        ),
        GateSpec(
            name="gate-2-truthy-dict-success",
            description="Navigation handlers must not infer success from truthy dicts",
            pattern_text=(
                r"def\s+(?:handle_|\w*_nav_\w*|\w*_explore_\w*)"
                r"[\s\S]{0,240}bool\([^)]*dict"
            ),
            include_prefixes=navigation_paths,
        ),
        GateSpec(
            name="gate-3-fixed-accepted-stub",
            description="Stub handlers must not hard-code status/state ACCEPTED",
            pattern_text=(
                r"[\"']?(?:status|state)[\"']?\s*(?:=|:)\s*"
                r"[\"']ACCEPTED[\"']"
            ),
            include_prefixes=application_paths,
        ),
        GateSpec(
            name="gate-4-private-provider-bypass",
            description="Private legacy exploration providers may not grow",
            pattern_text=r"_legacy_exploration_placeholder_disabled\.",
            include_prefixes=("gateways/", "providers/", "services/geometry/"),
            baseline=9,
        ),
        GateSpec(
            name="gate-5-stage-hint-fake-success",
            description="Stage hints must not fabricate published=True success",
            pattern_text=(
                r"published\s*=\s*True[^\n]*\bstage\b|"
                r"\bstage\b[^\n]*published\s*=\s*True"
            ),
            include_prefixes=application_paths,
        ),
        GateSpec(
            name="gate-6-fake-lcm-backend",
            description="Fake or mock LCM backends are forbidden",
            pattern_text=(
                r"LcmBackendKind::Fake|FakeLcmBackend|MockLcmBackend|fake-lcm"
            ),
            include_prefixes=(
                "services/",
                "gateways/",
                "providers/",
                "drivers/",
                "core/",
            ),
        ),
        GateSpec(
            name="gate-7-motion-client-direct-call",
            description="Actuator writers must be owned by the motion safety guard",
            pattern_text=r"\.(?:send_twist|stop_motion|Move|StopMove)\s*\(",
            include_prefixes=PRODUCTION_ROOTS,
            exclude_prefixes=(
                "services/navigation/motion_safety_guard/",
            ),
        ),
    ]


def _iter_source_files(root: Path) -> Iterable[Path]:
    for relative_root in PRODUCTION_ROOTS:
        base = root / relative_root
        if not base.is_dir():
            continue
        for dirpath, dirnames, filenames in os.walk(str(base)):
            dirnames[:] = [
                dirname
                for dirname in dirnames
                if dirname not in PRUNE_DIR_NAMES
            ]
            for filename in filenames:
                path = Path(dirpath) / filename
                if path.suffix.lower() in SOURCE_SUFFIXES:
                    yield path


def _path_selected(relative_path: str, spec: GateSpec) -> bool:
    normalized = relative_path.replace("\\", "/")
    if any(normalized.startswith(prefix) for prefix in GLOBAL_EXCLUDED_PREFIXES):
        return False
    if spec.include_prefixes and not any(
        normalized.startswith(prefix) for prefix in spec.include_prefixes
    ):
        return False
    if any(normalized.startswith(prefix) for prefix in spec.exclude_prefixes):
        return False
    return True


def _is_comment_line(line: str) -> bool:
    stripped = line.lstrip()
    return stripped.startswith(("#", "//", "/*", "*", '"""', "'''"))


def scan_gate(root: Path, spec: GateSpec, files: Sequence[Path]) -> GateResult:
    pattern = spec.compile()
    hits: List[GateHit] = []
    seen = set()
    for path in files:
        relative_path = path.relative_to(root).as_posix()
        if not _path_selected(relative_path, spec):
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            raise RuntimeError("cannot read {}: {}".format(path, exc))
        lines = text.splitlines()
        for match in pattern.finditer(text):
            line_no = text.count("\n", 0, match.start()) + 1
            line_text = lines[line_no - 1] if line_no <= len(lines) else ""
            if _is_comment_line(line_text):
                continue
            key = (relative_path, line_no)
            if key in seen:
                continue
            seen.add(key)
            hits.append(GateHit(relative_path, line_no, line_text))
    return GateResult(spec=spec, hits=hits)


def run_gates(
    root: Path, specs: Optional[Sequence[GateSpec]] = None
) -> List[GateResult]:
    source_files = list(_iter_source_files(root))
    return [
        scan_gate(root, spec, source_files)
        for spec in (list(specs) if specs is not None else build_gate_specs())
    ]


def blocking_failure_count(results: Sequence[GateResult]) -> int:
    return sum(
        1
        for result in results
        if not result.is_pass() and result.spec.severity == "BLOCKING"
    )


def _print_text_report(root: Path, results: Sequence[GateResult]) -> None:
    print("Static release negative gates")
    print("root: {}".format(root))
    for result in results:
        print(
            "{:<38} {:<22} hits={}".format(
                result.spec.name, result.status(), result.hit_count
            )
        )
        for hit in result.hits[:5]:
            print(
                "  {}:{}: {}".format(
                    hit.file_path, hit.line_no, hit.line_text.strip()[:140]
                )
            )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=find_repo_root())
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--gate", action="append", choices=[spec.name for spec in build_gate_specs()], help="只执行指定检查，可重复指定")
    parser.add_argument(
        "--report-only",
        action="store_true",
        help="Report violations but force process exit 0",
    )
    args = parser.parse_args(argv)
    root = args.root.resolve()
    if not root.is_dir():
        print("ERROR: root does not exist: {}".format(root), file=sys.stderr)
        return 2

    try:
        selected = [spec for spec in build_gate_specs() if not args.gate or spec.name in args.gate]
        results = run_gates(root, selected)
    except RuntimeError as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 2

    failures = blocking_failure_count(results)
    would_exit_code = FAIL if failures else PASS
    exit_code = PASS if args.report_only else would_exit_code
    if args.json:
        print(
            json.dumps(
                {
                    "root": str(root),
                    "blocking_fails": failures,
                    "report_only": args.report_only,
                    "would_exit_code": would_exit_code,
                    "exit_code": exit_code,
                    "gates": [result.to_dict() for result in results],
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        _print_text_report(root, results)
        print("blocking_fails={} exit_code={}".format(failures, exit_code))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Print the shared CPU/memory-aware build budget for shell entrypoints."""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.module_installer.hardware import resolve_build_jobs  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jobs", type=int)
    args = parser.parse_args()
    try:
        print(resolve_build_jobs(ROOT, args.jobs))
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""Compatibility entry point for the transactional deployment installer."""

from __future__ import annotations

import sys
from typing import Optional, Sequence


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Forward the legacy module entry point to the only supported CLI."""
    from scripts.deploy_installer import cli

    if argv is None:
        argv = sys.argv[1:]
    return int(cli.main(list(argv)))


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Verify string-enum parity and warning protection on Python 3.8 and newer."""
from __future__ import annotations

import builtins
import json
import logging
import sys
import unittest
from enum import Enum
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.compat import StrEnum


def fallback_enum():
    """Exercise the actual compatibility source even on native-StrEnum Python."""
    original_import = builtins.__import__

    def legacy_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "enum" and "StrEnum" in fromlist:
            raise ImportError("exercise pre-3.11 fallback")
        return original_import(name, globals, locals, fromlist, level)

    namespace = {"__builtins__": dict(vars(builtins), __import__=legacy_import)}
    source = Path(__file__).resolve().parents[1] / "contracts" / "compat.py"
    exec(compile(source.read_text(), str(source), "exec"), namespace)
    return namespace["StrEnum"]


class EnumCompatibility(unittest.TestCase):
    def check_strings(self, base):
        class Severity(base):
            WARNING = "warning"
            ERROR = "error"

        value = Severity.WARNING
        self.assertEqual(str(value), "warning")
        self.assertEqual("%s" % value, "warning")
        self.assertEqual(f"{value}", "warning")
        self.assertEqual(format(value, ">10"), "   warning")
        self.assertEqual(repr(value), "<Severity.WARNING: 'warning'>")
        self.assertEqual(json.dumps({"severity": value}), '{"severity": "warning"}')
        self.assertEqual(value, "warning")
        self.assertEqual(hash(value), hash("warning"))
        self.assertIs(Severity("warning"), value)
        self.assertIs(Severity.__members__["WARNING"], value)
        self.assertEqual(list(Severity), [Severity.WARNING, Severity.ERROR])
        self.assertIsInstance(value, str)
        self.assertIsInstance(value, Enum)

    def test_runtime_selected_enum(self):
        self.check_strings(StrEnum)

    def test_forced_legacy_fallback(self):
        self.check_strings(fallback_enum())




if __name__ == "__main__":
    unittest.main(verbosity=2)

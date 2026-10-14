#!/usr/bin/env python3
"""Offline exact-size experiment, leaving the production builder unchanged."""
import inspect
import json
import sys
import unittest
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gateways import feedback
from core.errors import GatewayError

baseline = feedback.build_mcp_tool_result
source = inspect.getsource(baseline)
needle = '    if _json_bytes(result) > total_limit:'
replacement = '''    envelope_size = _json_bytes({**result, "structuredContent": None})
    # Invalid Unicode already makes the placeholder envelope over-limit through
    # the existing _json_bytes sentinel. Do not replace that error with encode's.
    if envelope_size <= total_limit:
        envelope_size += len(result["content"][0]["text"].encode("utf-8")) - _json_bytes(None)
    if envelope_size > total_limit:'''
assert source.count(needle) == 1
namespace = {}
exec(source.replace(needle, replacement), baseline.__globals__, namespace)
candidate = namespace['build_mcp_tool_result']

def outcome(fn, value, limit=feedback.MCP_RESULT_MAX_BYTES):
    try:
        return ('ok', fn(value, max_total_bytes=limit))
    except GatewayError as error:
        return ('error', error.error_code, error.http_status, error.jsonrpc_code, error.details, str(error))

def wire_size(result):
    return len(json.dumps(result, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode('utf-8'))

class SizeReuseChecks(unittest.TestCase):
    def test_values_and_exact_boundaries(self):
        @dataclass
        class Row:
            label: str
            values: list
        class Color(Enum):
            BLUE = '蓝色'
        values = [None, True, False, 0, -1, 2.5, '', '照片😀\n\t"\\' * 100,
                  list(range(80)), {'values': [None, False, -0.0, Color.BLUE]},
                  Row('相机', [1, 2, {'key': 'value'}]),
                  {'api_key': 'private', 'payload': '安全文本' * 60},
                  {'error': {'code': 'offline', 'message': '不可用😀' * 30}},
                  {'result': {'observations': [{'camera_id': 'front', 'metadata': {'x': 'a' * 256}}]}}]
        for value in values:
            with self.subTest(value=type(value).__name__):
                full = baseline(value)
                self.assertEqual(candidate(value), full)
                boundary = wire_size(full)
                for limit in [0, 256, boundary - 1, boundary, boundary + 1]:
                    self.assertEqual(outcome(candidate, value, limit), outcome(baseline, value, limit))
                self.assertEqual(json.loads(full['content'][0]['text']), full['structuredContent'])

    def test_invalid_and_over_limit_parity(self):
        cycle = []; cycle.append(cycle)
        for value in [float('nan'), float('inf'), {'x': -float('inf')}, b'raw',
                      {'x': bytearray(b'raw')}, cycle, '\ud800', {'text': '\udfff'},
                      {'text': 'x' * 8192}, {1: 'one', '1': 'duplicate'}]:
            for limit in [256, 4096, feedback.MCP_RESULT_MAX_BYTES]:
                with self.subTest(kind=type(value).__name__, limit=limit):
                    self.assertEqual(outcome(candidate, value, limit), outcome(baseline, value, limit))

    def test_result_ownership(self):
        original = {'items': [{'values': [1, 2]}]}
        first = candidate(original); second = candidate(original)
        first['structuredContent']['items'][0]['values'].append(3)
        self.assertEqual(original, {'items': [{'values': [1, 2]}]})
        self.assertEqual(second['structuredContent'], original)

if __name__ == '__main__':
    unittest.main()

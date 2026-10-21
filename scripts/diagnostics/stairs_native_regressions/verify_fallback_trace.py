#!/usr/bin/env python3
"""Check actual manager branch diagnostics, not a duplicated predicate."""
import json
import sys
from pathlib import Path
calls = [row for row in map(json.loads, Path(sys.argv[1]).read_text().splitlines()) if row["event"] == "rebound_input"]
assert len(calls) == 6, calls
actual = [row["ordered_xyz_fallback"] for row in calls]
assert actual == [0, 0, 1, 0, 1, 0], actual
assert all(row["route_height_ready"] for row in calls)
print("first/history/random/reset excluded; explicit deterministic and active-reference bypass selected: PASS")

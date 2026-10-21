#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
NATIVE="$ROOT/services/nuwax-workspace/crates/nuwax-nav-scan-native/native"
TESTS="$ROOT/scripts/diagnostics/stairs_native_regressions"
OUT="${STAIRS_TEST_OUTPUT:-$ROOT/runtime_data/stairs_native_regressions}"
: "${EIGEN3_INCLUDE_DIR:=/usr/include/eigen3}"
: "${SCAN_NATIVE_LIBRARY:?Set SCAN_NATIVE_LIBRARY to the just-built libnuwax_scan_native.a; build the simulator first.}"
mkdir -p "$OUT/include/plan_manage"
# Test-only visibility shim; changes no object layout or production source.
sed 's/^  private:/  public:/' "$NATIVE/author_a/include/plan_manage/planner_manager.h" > "$OUT/include/plan_manage/planner_manager.h"
INC=(-I"$OUT/include" -I"$EIGEN3_INCLUDE_DIR" -I"$NATIVE/author_a/include" -I"$NATIVE/include")
for m in path_searching bspline_opt traj_utils plan_manage; do INC+=(-I"$NATIVE/upstream/$m/include"); done
for test in vertical_query_oracle route_height_reference_test route_height_entry_test height_lifecycle_test; do
  "${CXX:-c++}" -std=c++17 -O2 "${INC[@]}" "$TESTS/$test.cpp" "$SCAN_NATIVE_LIBRARY" -lpthread -o "$OUT/$test"
  "$OUT/$test"
done
"$OUT/vertical_query_oracle" .11 .23
"$OUT/vertical_query_oracle" .23 .11

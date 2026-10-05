#!/usr/bin/env python3
"""Serialize parity fixtures using existing lcm-gen 1.5.2 Python output.

This does not run code generation or hand-encode any message/fingerprint.
The scalar fixture matches Rust sample_candidate(1, 100); the trajectory
fixture matches candidate_owner_test_candidate().
"""
import argparse
import importlib.util
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--generated-python-root", type=Path, required=True)
parser.add_argument("--output-dir", type=Path, required=True)
args = parser.parse_args()
module_path = args.generated_python_root / "nuwax_msgs" / "CorrectionCandidate.py"
spec = importlib.util.spec_from_file_location("generated_correction_candidate", module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Candidate = module.CorrectionCandidate
candidate = Candidate()
fields = dict(
    schema_version=Candidate.SCHEMA_VERSION_V3,
    backend=Candidate.BACKEND_LIBRARY_PGO,
    transform_semantics=Candidate.TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3,
    runtime_generation=11, backend_generation=12, candidate_sequence=1,
    mapping_session_id="mapping-session-a", mapping_session_generation=13,
    mapping_backend_kind="voxel", mapping_backend_generation=1,
    map_frame_generation=14,
    mapping_commit_boot_nonce=bytes.fromhex("102030405060708090a0b0c0d0e0f001"),
    mapping_commit_sequence=15, occupancy_revision=16, cloud_revision=7,
    committed_map_source_timestamp_ns=100,
    registered_cloud_backend=1, registered_cloud_runtime_generation=31,
    registered_cloud_source_sequence=17, registered_cloud_source_timestamp_ns=100,
    registered_cloud_point_count=60,
    correlated_odom_source_sequence=18, correlated_odom_source_timestamp_ns=99,
    correlated_odom_translation_m=[0.0, 0.0, 0.0],
    correlated_odom_rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
    committed_map_width=101, committed_map_height=79,
    committed_map_resolution_m=0.05, committed_map_origin_x_m=-3.25,
    committed_map_origin_y_m=4.5, source_timestamp_ns=100,
    map_frame_id="robot_map", odom_frame_id="robot_odom",
    body_frame_id="robot_body", points_frame_id="robot_body",
    correction_lease_boot_nonce=bytes.fromhex("88776655443322111020304050607080"),
    correction_lease_generation=19,
    translation_m=[1.0, -2.0, 0.5], rotation_xyzw=[0.5, 0.5, 0.5, 0.5],
    trajectory_count=0, trajectory_timestamp_ns=[],
    trajectory_translation_m=[], trajectory_rotation_xyzw=[],
)
assert set(fields) == set(Candidate.__slots__), "all generated fields are explicit"
for name, value in fields.items():
    setattr(candidate, name, value)
args.output_dir.mkdir(parents=True, exist_ok=True)
for suffix, timestamps, translations, rotations in [
    ("", [], [], []),
    ("_trajectory", [99, 100], [[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]],
     [[0.0, 0.0, 0.0, 1.0], [0.5, 0.5, 0.5, 0.5]]),
]:
    candidate.trajectory_count = len(timestamps)
    candidate.trajectory_timestamp_ns = timestamps
    candidate.trajectory_translation_m = translations
    candidate.trajectory_rotation_xyzw = rotations
    wire = candidate.encode()
    assert Candidate.decode(wire).encode() == wire
    destination = args.output_dir / f"correction_candidate_v3_generated{suffix}.hex"
    destination.write_text(wire.hex() + "\n")
    print(f"{destination.name}: bytes={len(wire)}, fingerprint={wire[:8].hex()}, source={module_path}")

# Unknown-ground navigation verification, 2026-10-14

## Evidence boundaries

- Supplied source: an earlier G1 capture, with 159 canonical fine blocks, 88,558 occupied points and 1,320 measured ground layers
- The offline example consumes only `精细地图.bin`, `净空环境变量.json`, and `pose.json` from that supplied capture. The official archive/block/layer decoders supply every original occupancy and measured layer
- Exact archived pose, quaternion, asymmetric full-body envelope, body/support calibration, planner voxel 0.1 m, fine voxel 0.05 m, step-up/down 0/0 and slope 18 degrees are preserved. Both original uncertainty controls (0 and 0.02 m) are retained
- The supplied baseline log is historical evidence from the attachment's stated `09db973421b58676043c214f2e06fc42978090d4` build and remains outside the repository. It is not a fresh baseline run of checkout `580312e`. Source edits were concurrent, so no fresh unmodified baseline is claimed
- That baseline rejects both uncertainty controls: full pose `InsufficientClearance` with an observed near-floor fine-cell witness, and current support `MissingSupport`. Their exact cells, poses and coordinates remain in the private diagnostic output
- The final-run failing maps and complete route inputs were not supplied. This work cannot reproduce or establish the algorithmic cause of their final no-route event, household arrival, or apple transport success
- The separate final-run fine-block deadline/stale-input event is an input-availability failure. It is not relabeled as missing ground, and an old frozen capture is not evidence of current input freshness
- No simulator floor, task scoring positions, robot connection, paid model call, deployment or checksum calculation is part of this verification

## New verification artifacts

1. `crates/nuwax-nav-mls-planner/examples/frozen_unknown_ground.rs`: exact archived pose replay, official layer witness details, same-source geometry identity, offline requested 2/5/10 cm departure projection/sweep probes and optional pass assertions
2. Explicitly labeled synthetic negative controls in that example: cabinet column, tabletop above the archived floor, and a recent torso obstacle. Each is constructed in a separate immutable derived snapshot; the original archive and baseline source are unchanged
3. `crates/nuwax-nav-pct/tests/unknown_ground_surface_contract.rs`: public `SurfaceSnapshot -> PctPlanner` integration tests for blind start, missing middle strip, unknown endpoint, all-unknown geometry, no mutation of source/cache, a forbidden mask covering absent physical columns, wall and low top, recent-obstacle actual swept-volume rejection, a known cliff beyond an unknown gap, disjoint building levels, and a local-crop boundary that must not hide a real wall
4. `crates/nuwax-nav-stack/examples/frozen_unknown_pipeline.rs`: same archive driver with public PCT global-reference and Scan local-trajectory calls, followed by every actual swept-body segment and a 5 mm diagnostic arrival check. It starts no transport or controller
5. Private input metadata and raw logs remain outside the repository. Independent point count agrees between root block counts, fine evidence rows and `points.bin` byte length

The PCT source/cache/route tests verify global reference planning. The wall/top/recent cases additionally query the actual MLS swept body volume. These are not live robot execution tests. Map-refresh owner continuity and stale-source behavior are covered by the separate stack tests added with the production change.

## Reproduction

From the repository root, with a compatible Rust/native-dependency environment and the existing target cache:

```sh
source /path/to/neh-build-env.sh
cargo run --release --locked --offline \
  --manifest-path services/nuwax-workspace/Cargo.toml \
  -p nuwax-nav-mls-planner --example frozen_unknown_ground -- \
  '/path/to/user-provided-frozen-input' \
  --require-pass
cargo test --release --locked --offline \
  --manifest-path services/nuwax-workspace/Cargo.toml \
  -p nuwax-nav-pct --test unknown_ground_surface_contract
cargo run --release --locked --offline \
  --manifest-path services/nuwax-workspace/Cargo.toml \
  -p nuwax-nav-stack --features mls --example frozen_unknown_pipeline -- \
  '/path/to/user-provided-frozen-input' --require-pass
```

No clean build or alternate target directory is required. Builds are serialized with the main development lane. The repository ignores `**/tests/` and `*.md`; include the new PCT integration test and this sanitized report explicitly when publishing.

## Execution status

### Exact archived capture: static and departure checks passed

The latest executed optimized replay accepts both the full body pose and the compatibility current-body view at unchanged uncertainty 0 and 0.02 m. Zero is the primary qualification case; increasing the parameter was not used to make the capture pass. The returned view shares the source geometry. `GroundQuery` remains `Unknown` with no fabricated height, normal, source members or observed floor.

Both cabinet and tabletop counterfactuals reject an actual deliberately inserted fine voxel. The recent-torso counterfactual rejects a `Recent` occupancy witness. The assertions do not count a repeated original floor rejection as a successful obstacle control.

The diagnostic also passes independent assertions that the original source geometry and original observation geometry are unchanged, the archive/pose/configuration files are byte-identical before and after execution, and the original point/layer counts are unchanged. No checksum routine is used.

Initial intermediate runs were retained privately and failed before the contact correction; the supplied pre-change baseline remains preserved separately. The failed capture had real occupancy but no associated measured-layer records at its rejected floor columns. The correction handles that legacy measured-surface contact interpretation while keeping the actual rotated prism and source-owned occupied members; it does not populate unknown ground.

The subsequently added departure probes request 2/5/10 cm motion along the archived body heading. They reuse the unchanged original pose/model, call body projection and actual swept-volume validation, and report original errors/witnesses. These are synthetic requested motions for offline qualification, not task/world targets or executed robot motions. The MLS V3 and V4 standalone runs, and the subsequent combined frozen PCT → Scan run, pass all three projections and all three swept-volume checks at each uncertainty control. Height and quaternion remain the archived values. This verifies an offline body-motion departure, not actuator execution.

### Frozen PCT → Scan: all six departures passed

The combined optimized executable completed with exit code 0 and `--require-pass`. At each unchanged uncertainty control (0 primary and 0.02 m diagnostic), all 2/5/10 cm relative departure requests pass the public PCT global-reference planner, Scan local planner, every resulting actual swept-body segment, and the explicit 5 mm 3D arrival sphere.

Each global reference contains 2 poses. The 2/5/10 cm requests produce 2/3/4 local poses, respectively, with a maximum final error of approximately 4.0 mm. All six report `smoothed=false`. These are nontrivial trajectories rather than the already-arrived short circuit. The same executable reruns static pose/view admission, strict inserted cabinet/tabletop and Recent controls, and source/file invariance; all pass. Original ground classification remains `Unknown`.

The driver uses the same decoded source and body model. It derives basic kinematics from the archived omnidirectional speed declarations and applies the production forward-only conversion: forward 0.2 m/s, yaw 0.2 rad/s, no reverse/lateral command. Scan uses its printed defaults (5 m horizon, 1 m refresh distance, 12 optimization iterations, 0.9 m reference corridor). The explicitly selected 5 mm diagnostic tolerance forces these short departures; the archived ordinary tolerance would already count these synthetic nearby targets as reached. These request settings are not claimed as a capture of the final live session.

Final source alignment: the diagnostic was rebuilt and rerun after typed radius-status propagation and the final MLS error-wording cleanup. The final run again exited 0 with all six departures and every control passing. The relevant MLS/PCT/Scan/runtime/stack/type source files and manifests had unchanged sizes and modification timestamps across the final serialized batch; the source diff inventory and metadata are retained privately without new checksum calculations. The earlier pre-status-correction successful log is also retained. No actuator, control owner or transport service was started.

### Public-API integration: 9/9 passed

The final source-aligned independent PCT release target passed all nine integration tests, with zero failures or ignored tests. The separate final PCT library run passed 60 tests (3 opt-in benchmarks ignored). Both include calibrated reference-motion, no-envelope routing, the MLS uncached known-terrain source-reader changes, and typed resource/radius-status propagation. The independent target was rebuilt after the final MLS error-wording cleanup. Coverage of the original eight:

- Unknown start, missing middle strip and unknown endpoint keep the same source observations and cached tomogram
- Authoritative all-unknown fine geometry yields a nontrivial route without adding observed tomogram cells
- Forbidden masks remain hard barriers in columns absent from the physical source
- Known walls and low tops reject both reference routes and actual swept-body motion
- A fresh recent obstacle rejects actual motion while leaving the old static route/cache unchanged
- An observed half-meter cliff beyond an unknown gap cannot be replaced by a floating reference
- An unknown gap cannot connect separate known building levels
- A wall outside a small local materialization returns `SearchDomainLimited`; the same wall inside a complete larger window is rejected as known geometry

A ninth integration test now pairs a known shallow landing (10 cm against the configured 20 cm minimum tread) with a permitted 5 cm step on a complete tread, through both public planar projection and explicit-offset route APIs. The same no-envelope fixture checks all-unknown traversal with unchanged calibrated height and cached observed geometry. This ninth test passed in the latest combined run.

The first combined run passed 49 PCT library tests (3 ignored) and six integration tests, with two setup failures from calling the raw point-cloud constructor with an empty array. Those setup-only failures were corrected without production edits: the empty-source fixture uses a historical coarse allocation seed outside the bounded task plus authoritative complete-empty fine observations. Assertions confirm the seed contributes no tomogram cells or observed ground. The final independent rerun passed both previously unreached behavior assertions.

### Supporting owner/transport suites

Exploration V11 passed 78 unit tests and 2 doc tests, with 5 diagnostics ignored in the main suite. Its `unknown_frontier_poll_work_profile` diagnostic and the non-ignored shared blocked-route budget test were separately rerun and passed; the other four ignored diagnostics remain unexecuted. The compile-fail privacy doc test intentionally emits a compiler error and passes. Earlier biased-height and continuation/budget fixture failures are retained privately; final fixture admission uses the actual candidate heading and real collision-free body endpoints, without relaxing production continuation. These synthetic cloud-only results are not robot throughput measurements.

The final wording-aligned MLS suite passed 327 tests (17 ignored), and Scan passed 53 (1 ignored), with zero failures. The rebuilt runtime surface subset passed 88 tests (14 ignored); the separate legacy-scheduler test also passed. An intermediate runtime batch had one test-oracle failure: the final test restores the baseline allowance for an explicitly raised 3D sweep while retaining constant-height occupied-volume rejection and controller rejection, strengthened with a concrete occupied witness. Its exact regression also passed separately and is already included in the 88-test count. That correction changed tests only; the failed intermediate log remains preserved.

The final source-aligned codec suite passed 110 tests, with zero failures and 3 opt-in timing diagnostics ignored. This includes the fine-map protocol validation used by the separate map-refresh changes. The refreshed, source-aligned stack owner/mission/empty-root/budget/recent-source filters passed 69 unique tests, with zero failures and 4 diagnostics ignored: mission reference 5, legacy reference 2, quick budget 3, planning budget 5, selective refresh 31, map retry 8, fine chunks 11, and recent source 4. The focused empty-root codec test is already included in the full codec count and is not added twice. The separate geometry binary-target filters passed all 37 tests: compute backend 21, original cancellation-token ownership 1, mission 7, runtime mission 6, resource-limited terminal behavior 1, and distinct stop-reason mapping 1. These exercise failure without inappropriate return-on-completion or candidate blacklisting; they do not execute robot motion.

### Remaining qualification boundary

Frozen public PCT → Scan execution and the nine independent integration tests have passed against the final relevant source phase stated above. The supporting suites above separately cover execution-owner retry, source-age/cancellation/clear fencing, exploration, and active mission/home estimation. All selected offline suites listed above have completed successfully. Counts describe their stated library or filtered scope; ignored diagnostics are explicitly accounted for below and are not silently counted as passing. This is not a claim that every repository target or optional diagnostic was executed. The frozen capture must be rerun if later source edits affect its query path. No robot motion or final-session reproduction has been performed.

## Ignored-test inventory and separate execution

These were ignored by their main suites; that status is not counted as a pass. A separately executed exploration profile is identified below. Recorded-input diagnostics require their separate original inputs; timing benchmarks require a separate quiet timing lane. They are not prerequisites silently counted as successful in this correctness report.

### MLS (17)

- `surface::body::contact_domain_tests::recorded_contact_domain_and_fit_exact_differential_benchmark`
- `surface::body::scalar_response_tests::recorded_c_positive_fine_obstacle_unchanged`
- `surface::body::terrain_contract_tests::dense_rough_rejection_attribution_diagnostic`
- `surface::body::terrain_contract_tests::terrain_contract_matrix_diagnostic`
- `surface::body::tests::benchmark_full_footprint_block_probe`
- `surface::body::tests::benchmark_indexed_level_selection_prototype`
- `surface::body::tests::benchmark_observation_polygon_once`
- `surface::body::tests::benchmark_ordered_observation_difference`
- `surface::body::tests::dense_contact_ordering::dense_order_reachable_fraction_microbenchmark`
- `surface::body::tests::footprint_container_experiment::benchmark_container_difference`
- `surface::body::tests::footprint_container_experiment::benchmark_container_paths`
- `surface::body::tests::recent_query_tests::recent_query_complete_pose_ab_benchmark`
- `surface::empty_mask_reuse_tests::empty_exclusion_derivation_cost_baseline`
- `surface::observations::tests::benchmark_evidence_only_block_refresh`
- `surface::observations::tests::benchmark_observation_lookup_and_ordered_set_construction`
- `surface::recent::columns_diagnostics::sorted_columns_paired_builder_cost`
- `surface::update::replay_diagnostics::inspect_saved_surface_with_full_reference`

### PCT (3)

- `tomogram::allocation_benchmark_tests::tomogram_allocation_frozen_baseline`
- `tomogram::connect_trial_tests::cardinal_connect_matched_benchmark`
- `tomogram::masked_surface_tests::masked_surface_alternating_baseline`

### Codec (3)

- `lcm::recent_cloud::tests::recent_nested_cloud_decode_cost`
- `lcm::schema::navigation_map_3d_tests::navigation_nested_cloud_decode_cost`
- `lcm::schema::navigation_map_3d_tests::warmed_fine_map_codec_baseline`

### Scan (1)

- `recent_detour_diagnostic_tests::recent_wide_obstacle_fixed_start_reference_ab`

### Exploration (5 ignored in main suite)

- `surface_adapter::tests::original_long_body_fixture_exhausts_radially_clear_frontiers`
- `surface_adapter::tests::private_wait_frontier_profile`
- `surface_adapter::tests::private_wait_production_profile`
- `surface_adapter::tests::surface_view_ownership_profile`
- `surface_adapter::tests::unknown_frontier_poll_work_profile`: separately executed and passed; four other entries above were not executed

### Stack owned filters (4)

- `lcm_dataflow::surface_dataflow::tests::quick_budget_tests::calibrated_quick_and_full_budget_cost_baseline`
- `surface_chunks::tests::captured_dense_doorway_reproduces_exact_connector_rejection`
- `surface_chunks::tests::captured_dense_doorway_static_connector_probes`

- `lcm_dataflow::surface_dataflow::recent_source::ingestion_diagnostics::baseline_complete_recent_ingestion_cost`

### Runtime surface subset (14)

- `surface_follower::body_control::current_view_recheck_diagnostic::current_pose_recheck_then_view_refresh_cost`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_dense_doorway_cheap_alignment_continues_to_goal`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_dense_doorway_far_bearing_has_guarded_command`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_dense_doorway_single_state_reproduces_exact_rejection`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_dense_doorway_straight_intent_continues_to_goal`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_exit_state_reproduces_exact_rejection`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_exit_translation_advice_continues_to_goal`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_translation_advice_from_first_failure_reaches_goal`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_translation_advice_from_original_start_reaches_goal`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_translation_advice_reflected_y_from_original_start_reaches_goal`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::captured_translation_advice_rotated_90_from_original_start_reaches_goal`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::measured_collinear_progress_at_captured_entry`
- `surface_follower::body_control::dense_doorway_checkpoint_diagnostic::measured_collinear_progress_cold_old_new`
- `surface_follower::body_control::reconstructed_receipt_geometry_diagnostic::reconstructed_attribution_v1_geometry_callback_differential`

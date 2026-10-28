# Common-band height response: bounded repair

Base: `301898085509c591eb96ae95ffd0052207055596`.

## Problem and scope

Previously, every positive-weight contact-domain interval participated equally in a hard common-interval clamp. An observation entering the response kernel with negligible weight could abruptly move the estimated body height, although that observation remained outside the actual body/support region. At the recorded A boundary, 0.75 mm motion passed but 1.125 mm induced a 5.673 mm height drop and rejection.

This change replaces only common-horizontal-band height selection with the minimizer of:

`sum(w * distance(z, interval)^2) + alpha * (z - reference_height)^2`.

`alpha = rho * sum(w)`, `rho = U/(U+S)`, `U = sum(w * width^2 / 12)`, and `S = sum(w * horizontal_distance^2)`. These express a geometric regularization preference, not calibrated sensor variance or a new uncertainty allowance. An already-feasible reference is returned exactly. Zero-width and noncommon cases use the original fit, including its original spatial-rank requirement.

The exact scalar root uses sorted active endpoint displacements and prefix weight/moment sums. It replaces experimental repeated bisection, without adding a parameter, lock, application hash, collision threshold or permission bypass. The existing original fit is evaluated first; the change is not claimed to eliminate all old work or allocations. The endpoint list is an additional local allocation on the affected path.

All pose/motion, obstacle, support, step, observation and swept-geometry checks remain. A smooth estimate does not override their outcome. No simulator response model, hardware control, robot-specific production branch or new deployment mode is included.

## Independent geometric evidence

An analytic fixture declares its true support plane and body offset before invoking either estimator. A lower patch enters the response kernel but remains at least 37.6 mm outside the swept body/support region. Valid 5 cm observations encode it; band lower bounds are not substituted for the declared true exposed plane.

- Flat plane, eight forward/reverse segments: old maximum body-height error 50 mm and final height residual -40 mm; candidate maximum error about 0.077 micrometres and final residual about 1e-15 m
- Quantized true ramp, forty segments: both versions retain about 2.900 mm maximum height error and 1.049 degrees maximum normal error; this repair does not solve the old noncommon fitting error
- Analytic truth paths and candidate paths pass full geometry checks
- An initial invalid flat-fixture support offset failed and is retained in evidence; it was not repaired by loosening the algorithm's thresholds

Recorded A short motion passes projection and full sweep with the candidate. Two/six subdivisions over three forward/reverse rounds total 48 successful steps, returning to the original height/orientation. The separate C positive-turn obstacle remains rejected at fine cell `[-84,-13,-3]` through direct sweep and projected motion.

## Numerical and integrated qualification

- 2003 fixed/random root comparisons against independent bisection: maximum height difference 1.78e-15 m, objective difference 7.11e-15
- Actual integrated checkout release qualification: MLS body selection 133 passing tests; original recorded C obstacle test explicitly run, 1 passing; related runtime selection 99 passing
- The C test requires the original XYZT file through `NUWAX_OFF_SOURCE_XYZT` and is explicitly ignored in the default portable suite. Default new tests include frozen rows and generated analytic geometry, not workstation paths
- Existing five workstation diagnostic changes were preserved and excluded from the publication patch. These are focused native tests, not a full workspace or hardware certification

The former strict flat identity assertion failed by approximately 0.972 nanometres under the intentional soft-interval objective. Its epsilon was not enlarged: the repository test now compares against an independent root for the declared new objective, demands zero tilt and runs the complete swept permission check. Historical failures remain in the qualification artifacts. Scalar feedback of the previous fitted height is not the same as the production base/next relative response evaluated at one reference height.

## Performance interpretation

Prototype ABBA measurements on the workstation: repeated bisection about 7.43–8.17 microseconds per fit, exact endpoint solver about 2.16–3.07. Complete cold micro-motion query was about 160 to 150 microseconds; warmed queries did not refit.

These compare two experimental implementations of the new objective. They do **not** establish a speedup over the previously shipped hard clamp, nor overall navigation or robot CPU improvement. Earlier original-versus-bisection cold-query comparisons showed a roughly 6–7% regression; the exact solver targets that extra work. Timings came from frozen overlays before the final integration; integrated tests establish entry-point correctness without claiming another timing run.

## Failed complete replay: retain the distinction

Each frozen complete OFF variant was run once. Baseline exhausted 120 simulated seconds, remained 5.100 m away, and performed 910 global plans. Candidate stopped at 51.25 simulated seconds, remained 5.112 m away, and recorded one generated-motion geometry failure after four plans. Different stopping times make their CPU/wall times incomparable as an end-to-end speedup.

The replay's `Terrain::body_at` is a separate implementation of the **old** interval estimator, including the disputed hard clamp. It is not measured robot Z/quaternion history or a contact-dynamics engine. Tracing one saved failed step identifies entering column `[-43,8]`: old simulated contact height clamps from -0.444327120006 to -0.450000006706 m, producing a 5.672887 mm drop. Candidate predicted about 2.502418 micrometres. Both geometry-map versions reject the old generated pose and accept the candidate pose.

The actual old-generated sweep rejection is `Ground / ContactBelowGround` at fine cell `[-39,4,-6]`: -0.2513418315 < -0.2500000037, a 1.342 mm violation. This is a genuine geometric checker result for that **generated** pose, not an expected-pose equality assertion. It is not independently measured physical truth. The nearest recorded XY is about 9.52 cm away and cannot be reused as same-step ground truth.

Thus the legacy-environment complete qualification remains failed. No complete recorded OFF arrival, original live ON repair, or physical false-pass conclusion is established here. The simulator was not altered to make this candidate pass.

## Remaining limitations

Common/noncommon branch changes and old normal selection can still be discontinuous. A quantized boundary experiment produced approximately -0.417413 mm height change, 0.085963 degrees orientation change, and -2.235725 micrometres reverse residual, with both sweeps passing; the old noncommon branch already contributes about -0.364360 mm. This is a bounded common-band repair, not a global terrain/robot-response redesign. Layer association, self echoes, actual contact responsibility, and terrain-response uncertainty remain open work.

## Retained artifacts

- Full frozen algorithm/research evidence: `libfile_721b4abc54908191ab2d8ccb54ea9f8c`
- Exact integrated production/test patch and logs: `libfile_03d22986268c8191b4942ad1f34faaa2`
- Fast solver and complete OFF results: `libfile_50e9adb0879c81918bac1345e8924cc8`
- Old simulator-model causal audit: `libfile_04b29f34dcf48191bd1341cbc2d854c3`
- Actual quantized branch-switch experiment: `libfile_4c3e867574688191bbc8170657ce7d0c`

Parent cloud review checked the exact source/test patch and evidence; it has no Rust toolchain and did not rerun native tests. Publication excludes experimental Cargo/path overlays. No deployment or live robot/simulator operation is implied by the Git commit.

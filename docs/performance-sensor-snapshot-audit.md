# Sensor snapshot transport audit

Date: 2026-10-13 UTC
Audited baseline: `6f849311208a534f88a0ce913eed4673040813f2`
Scope: offline, synthetic point-cloud snapshot transport in Python 3.8.10. **No production behavior change.**

## Decision

Keep the existing implementation. The largest tested native-LIO snapshot had a median delivery latency of **13.32 ms** and sustained the existing approximately 20 Hz snapshot cadence. This does not establish a user-visible performance problem requiring another implementation.

Unchanged point clouds are already omitted, and pending display snapshots already coalesce on both sides of the process boundary. Parent installation and ordinary concurrent status reads were inexpensive. No additional cache, point limit, metadata removal, encoding, shared-memory mechanism, or control channel is proposed by this audit.

## Method and limits

The process-isolation branch in `drivers/go2/data_plane.py` is used for `fastlio2`/`pointlio`. The final measurements used `fastlio2` configuration and the existing calibration profile. Synthetic PointCloud2-shaped messages passed through the real `_on_point_cloud_message` decoder to produce the actual `PointCloudFrame` payload, including XYZ, intensity, per-point offsets and canonical metadata.

The diagnostic measured:

- 40 repetitions of `ForkingPickler` serialization/deserialization and cross-process `Connection.send`/`recv`
- The actual `Go2SensorProcess` with a fake producer, for two seconds per fresh/unchanged scenario and point count
- Parent `_install_process_snapshot` without application callbacks
- Concurrent bridge and input-delivery status reads, sampled approximately every 5 ms
- A deliberately stalled receiver to establish shared status-channel backpressure behavior

No robot network, DDS connection, native SDK download, runtime server or real recording was used. The script imports the Python driver and contract modules; it does not initialize hardware. Fixture cloud buffers were prepared before the timed transport loop. Pose and IMU were absent, so this measures cloud transport rather than their decoding or application consumers. Snapshot latency starts when the child builds its status, excluding the preceding 50 ms polling interval and sensor acquisition latency.

CPU benchmark windows were coordinated with another task. These are bounded cloud-host observations, not robot throughput guarantees or long-duration load tests. Host scheduling outliers remain in the raw results; one unchanged 100k trial included a 16.5 ms isolated status-call outlier, which is not attributed to cloud transport.

## Native-LIO measurements

| Measurement | 10,000 points | 100,000 points | 250,000 points |
|---|---:|---:|---:|
| Fresh serialized snapshot, bytes | 563,638 | 5,603,672 | 14,003,672 |
| Unchanged serialized snapshot, bytes | 817 | 819 | 819 |
| Serialization median, ms | 0.086 | 0.456 | 1.921 |
| Deserialization median, ms | 0.056 | 0.205 | 0.876 |
| Cross-process send-to-received median, ms | 0.695 | 7.177 | 12.250 |
| Cross-process send-to-received p95, ms | 2.044 | 10.043 | 14.577 |
| Actual snapshot delivery median, ms | 1.367 | 5.864 | 13.318 |
| Actual snapshot delivery p95, ms | 1.947 | 10.033 | 19.206 |
| Actual parent installation median, ms | 0.030 | 0.032 | 0.037 |
| Concurrent status query p95, ms | 0.057 | 0.056 | 0.078 |
| Concurrent 5 ms ticker excess p95, ms | 0.225 | 0.323 | 0.896 |

Each two-second fresh trial delivered 39 snapshots containing fresh clouds. Each unchanged trial delivered 39 snapshots but included a cloud only once. All these trials closed without reported process errors. Unchanged median snapshot latency was 0.34–0.38 ms; its trial includes the initial full-cloud transmission.

### Payload accounting

The full-field fixture occupies approximately **56 bytes per point**, plus about 3.7 KB of model/field/status metadata:

- Primary XYZ float32 array: 12 bytes per point
- Metadata source XYZ array: 12 bytes per point
- Metadata intensity and uint32 offsets: 4 + 4 bytes per point
- Canonical packed XYZ, intensity and uint64 offsets: 12 + 4 + 8 bytes per point

Pydantic JSON field exclusion does not exclude ndarray attributes from Python object pickling. At hypothetical continuously fresh 20 Hz, the serialized payload would total approximately 11.3/112.1/280.1 MB/s for the three tested sizes. These are calculated rates, not observed robot frame rates; actual cloud freshness/cadence can be lower.

## Backpressure limit

Snapshot and error messages share `status_send_lock` and one `Connection`. The diagnostic reproduced that locking/send arrangement, paused receiver reads for 100 ms, and attempted an error send while the large snapshot held the lock. Error sends waited approximately **99.5/103.7/111.8 ms** for 10k/100k/250k respectively.

This documents an existing limitation: error notification is not independent of a blocked snapshot transfer. It is not evidence that a comparable stall occurred during normal operation, and it does not justify another channel in the current scope.

## Existing semantics retained

Complete acquisition/canonical delivery and recording remain separate from replaceable display snapshots. This audit changes none of their behavior, including:

- Generation checks and rejection after failure or shutdown
- Input-delivery sequence gating
- Queue depth, bytes, age and explicit failure behavior
- Recording-drop accounting and drained markers
- Source timestamps, freshness, error propagation and process cleanup

Existing public point-cloud consumers can inspect canonical payloads and aligned metadata arrays. Metadata stripping or arbitrary point truncation is therefore not established as compatible. Parent installation already assigns references and uses shallow copies; it was not a meaningful bottleneck in these measurements. The separate provider polling method that deep-copies a point cloud was outside this audit.

## Evidence and reproduction

The bounded diagnostic and raw results are retained outside the repository in the current cloud workspace:

- `/workspace/scratch/71cdb9d17732/neh-evidence/sensor-snapshot-20261013/measure_snapshot.py`
- `/workspace/scratch/71cdb9d17732/neh-evidence/sensor-snapshot-20261013/results-native.json`
- `/workspace/scratch/71cdb9d17732/neh-evidence/sensor-snapshot-20261013/run-native.jsonl`

An earlier device-mode control is retained separately as `results-device-control.json`; the table above uses only the final native-LIO run.

From the repository root in that workspace:

```sh
PYTHONPATH="$PWD" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /workspace/scratch/71cdb9d17732/neh-toolchain/venv-py3810/bin/python \
  /workspace/scratch/71cdb9d17732/neh-evidence/sensor-snapshot-20261013/measure_snapshot.py \
  /tmp/neh-sensor-snapshot-rerun.json
```

This command uses existing dependencies, starts only local multiprocessing workers and anonymous pipes, and writes the specified result JSON. Evidence paths are workspace-local and are not portable repository dependencies. A rerun elsewhere requires copying the diagnostic and using a compatible Python environment.

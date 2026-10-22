# Incremental cloud: native consumer contract

Normal mapping commits carry a bounded `CommittedCloudMapMetadata` plus the real cloud commit receipt. Producer epoch, layout generation, session generation, revision, point count and voxel size fence publication and read-only correction validation. These paths do not flatten cumulative XYZ bytes, compare full payloads, or hash the full cloud.

`/nav/map3d` fine-chunk publication builds an empty legacy inline cloud directly. It transports the fine manifest and bounded included blocks without first constructing and clearing full XYZ bytes. A legacy no-fine navigation source explicitly requests a lazily validated full export.

## Legacy full PointCloud2 demand

`/global_map#sensor_msgs.PointCloud2` is full compatibility output. Ordinary mapping does not emit it by default. Existing native person-follow still requires this format:

- The geometry task owner runs a cancellable 500 ms demand watcher
- It uses the existing person-follow ready marker, verifies subscriptions_ready plus boot ID, PID and process start time, and rejects stale, malformed, duplicate-field or symlink markers
- AUTOSTART=true alone never enables full exports
- The local gateway launcher propagates the same absolute marker path into geometry; the geometry systemd unit shares the follower unit's /run marker default
- A live follower enables full export automatically, including a one-time replay of an unchanged map on activation/restart
- Marker removal or a dead/reused process revokes demand on the next watcher observation; ordinary commits then return to metadata/fine chunks only
- Repeated watcher ticks do not resend an unchanged map for the same demand identity

An external legacy full consumer can set `NUWAX_GEOMETRY_LEGACY_GLOBAL_MAP_FULL=true`. The default is false. This option is ORed with verified live follower demand; false never disables a running native follower. If an external consumer starts after its opt-in publication, it must arrange replay/restart itself: LCM has no subscriber introspection or durable subscriber history here.

Full export occurs only after metadata and the entire receipt are revalidated under the cloud writer lock. Ordering and cancellation additionally fence producer epoch and layout generation. Once requested, full output necessarily has O(total cloud bytes) materialization/encoding cost. This is intentionally excluded from the no-full-copy incremental path.

Checkpoint/export and explicit activation retention remain full-payload boundaries. Activation retains separately supplied immutable target bytes, while its guards carry metadata and receipts. Persistent checkpoint hashes remain part of the existing export contract.

## Verification boundary

Verified on x86_64 Linux with Rust 1.88.0:

- `cargo check --locked --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --tests` checks all production/test API references
- `cargo build --locked --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime` builds and links the production binary without test configuration; the binary was not started
- Eight selected runtime-binary tests pass: page membership and dirty tracking, private-work page reuse, complete UTF-8 receipt identity, receipt/producer/layout guards, live marker validation, demand/delivery state transitions, and v3 SHM reference metadata
- The historical interoperability experiment used the actual Rust writer and Python mmap reader across bootstrap, append, edit, delete, empty reset and new-session data
- Nineteen Python page tests, 27 legacy acquisition tests and six structural native-consumer checks pass

Full/fine publication lifecycle tests compile but are deliberately not executed while their existing application checksum paths await migration. The executed native tests do not start LCM or a robot, and do not call those full/fine export paths. Host tests do not establish aarch64 hardware behavior.

该跨语言一次性实验及配套忽略用例已于2026-10-22退役；现有点云提交行为回归保留在 `pointcloud::tests`。

源码文本断言检查已退役。当前点云业务验证使用 `cargo test --locked --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime pointcloud::tests`，构建环境按项目构建规范准备。

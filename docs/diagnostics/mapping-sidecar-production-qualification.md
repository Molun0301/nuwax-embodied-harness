# 生产 mapping sidecar 协议诊断

`scripts/diagnostics/qualify_mapping_sidecar.py` 直接通过 stdin/stdout 驱动普通
`nuwax-geometry-runtime` 的 `--nuwax-mapping-raytracing-sidecar-v1` 入口。
它只编码协议和检查结果，不复制射线、occupancy 或 checkpoint 恢复算法，不调用 Cargo。
脚本使用 Python 3.8 兼容语法、标准库及 POSIX 管道；结果记录实际 Python 版本。

## 准备独立 oracle

从仓库根执行，先加载本机正常 geometry 构建所需的 Rust、liblcm、GTSAM、protobuf
等环境。所有 Cargo 命令串行复用同一个 target、原默认 features 和 release/LTO；
不要在构建或验证期间改源文件，也不要用 test binary 代替生产 binary。
`NEH_SIDECAR_OUTPUT` 应指向仓库外的持久绝对路径，原始证据不要提交进 Git。

```sh
set -euo pipefail
WORK="${NEH_SIDECAR_OUTPUT:?Set an absolute output directory outside the repository}"
case "$WORK" in /*) ;; *) echo "NEH_SIDECAR_OUTPUT must be absolute" >&2; exit 1 ;; esac
mkdir -p "$WORK/oracle"
MANIFEST=services/geometry/nuwax_geometry_runtime/Cargo.toml
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$PWD/services/nuwax-workspace/target}"
PYTHON="${PYTHON:-python3}"
git rev-parse HEAD > "$WORK/oracle-source.txt"
unset NEH_RAY_COUNT_REFERENCE_DIR
export NEH_RAY_COUNT_SNAPSHOT_DIR="$WORK/oracle"
for selector in \
  mapping_backend::ray_count_equivalence_tests::ray_count_compute_skip_flush_checkpoint_restore_continue \
  mapping_backend::ray_count_equivalence_tests::ray_count_restore_preserves_serialized_count_until_next_candidate
do
  cargo test --locked --release -j4 --manifest-path "$MANIFEST" \
    --bin nuwax-geometry-runtime "$selector" -- --exact --nocapture --test-threads=1
done
unset NEH_RAY_COUNT_SNAPSHOT_DIR
```

确认两个精确测试都实际运行并通过，生成
`ray-count-kernel-lifecycle.json` 和 `ray-count-restore-original-counter.json`。
这是现有真实 kernel 的确定性合成输入，包含完整浮点 bits、occupancy cells 和 opaque
checkpoint，不是机器人采集数据。回归验证应先在可信对照版本生成并保留 oracle；
失败后不能用候选输出覆盖 oracle。若 oracle 与生产 binary 来自同一实现，结果仅证明
测试入口与生产子进程的一致性，不构成两个生产版本的 A/B。

## 构建普通 binary 并执行

```sh
"$PYTHON" -B scripts/diagnostics/qualify_mapping_sidecar.py \
  --baseline "$WORK/oracle" --self-check
cargo build --locked --release -j4 --manifest-path "$MANIFEST" \
  --bin nuwax-geometry-runtime --message-format=json-render-diagnostics \
  > "$WORK/production-build.jsonl" 2> "$WORK/production-build.stderr.log"
"$PYTHON" -B scripts/diagnostics/qualify_mapping_sidecar.py \
  --baseline "$WORK/oracle" \
  --binary "$CARGO_TARGET_DIR/release/nuwax-geometry-runtime" \
  --build-json "$WORK/production-build.jsonl" \
  --output "$WORK/qualification-run"
```

`--self-check` 只检查 fixture/codec 形状，不启动进程。实际执行要求 Cargo JSON 确认
`kind=bin`、`profile.test=false` 和 `build-finished.success=true`，并核对普通 release
executable。运行前后检查同一 binary 的 device/inode/size/mtime；不接受验证中替换 binary。
`--output` 必须不存在；重试使用新目录，保留失败证据。整批通过以退出码 0 和
`result.json` 的 `status=passed` 为准，单个 `passed-cases.json` 不能代替整批结果。
每个 child 保存原始请求/响应、stderr、trace 及终态 exit/EOF；失败清理不算正常关闭通过。

## 11 个独立生产子进程场景

1. 完整 START/COMPUTE/RESOLVE/FLUSH/EXPORT/RESTORE/STOP 生命周期，包含 1ms 跳帧、dirty export、错误 flush token、空/NaN 输入和继续计算
2. 新进程恢复 checkpoint 后继续计算
3. 保留合法 checkpoint 中不等于当前 healthy 数量的历史计数
4. COMPUTE 待决时 RESOLVE(false) 后退出
5. FLUSH 待决时 RESOLVE(false) 后退出
6. RESTORE 待决时 RESOLVE(false) 后退出
7. 待决 COMPUTE 的 STOP/STOPPED/EOF
8. 待决 COMPUTE 的 stdin EOF 退出
9. 错误 resolution transaction ID 返回 ERROR，exit 1/EOF
10. 未 RESOLVE 即 EXPORT 返回 PendingResolution 阶段 ERROR，exit 1/EOF
11. token/input source identity 不一致返回 ERROR，exit 1/EOF

候选除 `compute_elapsed_us` 外的业务 wire bytes 和完整 checkpoint 必须逐 byte 相等；
该耗时字段仍保存在原始响应中。`accepted_points_map` 不在真实 wire 中，只作为请求输入，
不能称为 sidecar 输出。RESOLVE(false) 的语义是 ACK 后终止该进程，不是在原进程内回滚继续。

## 边界与已验证记录

这里不运行生产 Rust 父 supervisor、channel/cancellation-token、commit-owner、retained-root
提交链、watchdog、PDEATHSIG 或 process-group 回收。3 秒交换界限与协议 timeout 一致，
不能当作父 watchdog 已验证。也不覆盖真实 LCM、机器人/ARM、实际采集链路、非法配置的
运行中注入、端侧时延或分配性能。sidecar 分支在 LCM 初始化前进入，但完整 binary 仍有
liblcm/native 动态加载依赖；加载失败与 multicast 路由缺失须分别报告，不改权限或网络绕过。

2026-10-17 的云端 x86_64 记录使用 Rust 1.88.0、Python 3.8.10；普通生产 binary 来自
`c79cf370f08a63308ce57aa32fb657d71d5ffe14`，对照来自先前保存的旧 kernel fixture。
初次第 10 项预期漏写生产 `Option` 调试文本的 `Some(...)`，前 9 项通过后停止；原失败保留。
只纠正该精确预期字符串，保留 ERROR 帧、阶段、exit 1/EOF 断言，同一 binary 全量重跑 11 项通过。
这不改变既有 real-LCM/UDS 环境限制或导航 normal-owner/late-rejoin 的未解决结果。

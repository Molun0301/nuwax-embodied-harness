# 完整点云资产接纳扫描性能

## 改动与调用路径

`services/geometry/dense_cloud.py::validate_dense_archive` 对每个最多 65,536 点的 XYZ 块执行完整有限值检查和精确边界验证。原实现通过 `points.min(axis=0)` / `max(axis=0)` 对交错 XYZ 做归约；现在对三个坐标轴的步进视图分别归约。视图共享当前块的字节缓冲区，不复制整张点云，不抽样，也不改变块大小。

真实 assigned 地图激活的 Python 接纳路径为：

1. `MapAssetBundleRepository` 在 `_for_activation=True` 读取中调用 `MappingBackendCheckpointStore.load(schema_version=2)`，生成保留检查点的 `_AssignedActivationRead`
2. `MappingBackendCheckpointStore._load_v2_optional` → `validate_checkpoint_bundle` → `_validate_checkpoint_artifacts` → `validate_dense_archive`
3. `prepare_retained_assigned_map_activation` 将同一已接纳检查点交给 `_prepare_assigned_map_activation`，不再次加载全部载荷
4. 后续 provider 上传、原生恢复、激活提交、定位和网页显示不在本次测量范围内

保持已有来源身份、边界、NaN/Inf 拒绝逻辑、不可变载荷所有权、接纳缓存和激活原子性；不添加哈希、不删减验证。

## 可复现测量

云环境：Python **3.8.10**，NumPy **1.24.4**，2026-10-13。没有使用现场地图或机器人。先预热，再交替运行旧/新实现各 7 次；测量窗口未运行其他工作者的 Cargo 构建或性能探针。

```bash
OPENBLAS_NUM_THREADS=1 python scripts/diagnostics/dense_archive_admission_performance.py
```

诊断脚本上限为每云 4,000,000 点（约 48 MB），每组最多 31 次。基线使用相同完整验证函数，仅将两个归约表达式恢复为原来的 `axis=0`，因此对比的是这次改动本身。加载测量调用真实 `MappingBackendCheckpointStore.publish/load`：先在临时目录发布合法 schema-2 检查点，测量反复加载，文件系统缓存已预热。为隔离此因素，基线加载仅临时替换存储模块引用的密集云验证函数；文件读取、不可变冻结、其余接纳逻辑与候选实现相同。

合成检查点包括合法的 1 点 canonical voxel cloud 和单独的完整扫描云，两者持有同一来源提交身份，但保留各自点数；不把完整扫描点数冒充降采样 canonical 点数。计时不包括初次发布。脚本输出 Python/NumPy 版本、范围、全部样本和中位数。

| 完整云点数 | 资产字节数 | 完整验证旧/新中位数 | 检查点加载旧/新中位数 |
| --- | ---: | ---: | ---: |
| 65,536 | 786,646 | 4.482 / 0.406 ms | 7.237 / 2.956 ms |
| 1,000,000 | 12,000,216 | 69.023 / 5.232 ms | 71.124 / 11.789 ms |
| 4,000,000 | 48,000,216 | 259.257 / 15.523 ms | 298.398 / 39.697 ms |

4M 点样本中，完整验证约 **16.7 倍**，该检查点加载约 **7.5 倍**。这不是整机地图激活速度，也不能外推为任意磁盘、任意地图或最大 8 GiB 资产的结果。冷盘 I/O、RPC/原生恢复、定位、UI 和实机结果未测量。

## 验证边界

Python 3.8.10 下 `tests/test_dense_archive_bounds.py` 六项测试全部通过，覆盖：

- `bytes` 与 `FrozenPayload` 两种来源；单点、多个点、65,536 点边界前后和多块
- 随机有限值与原 `axis=0` 参考边界一致
- 正负零、float32 次正规值和最大有限值
- 每个轴、块首末与跨块处的 NaN、正无穷、负无穷保持拒绝
- 每个错误最小/最大边界、截断/多余字节、空载荷/零点数、来源身份不符保持拒绝
- `FrozenPayload` 读取继续受原 786,432 字节块预算约束

为了不加载相机和原生运行时等无关依赖，可使用源模块命名空间运行这些存储测试；这不覆盖服务启动：

```bash
python - <<'PY'
import pathlib, sys, types, unittest
root = pathlib.Path.cwd()
for name in ('services', 'services.geometry', 'services.mapping'):
    package = types.ModuleType(name)
    package.__path__ = [str(root.joinpath(*name.split('.')))]
    sys.modules[name] = package
suite = unittest.defaultTestLoader.discover('tests', pattern='test_dense_archive_bounds.py')
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
PY
```

相邻 `test_checkpoint_finite_scan.py`（5 项）与 `test_checkpoint_identity_v2.py`（43 项）也全部通过；连同本次 6 项回归，共 **54/54**。覆盖真实 protobuf 导出、存储重启重开重传、assigned 资产准备与最终化、身份冲突拒绝、单次不可变冻结等。恢复环境缺少的 SQLite 扩展和已提交 fixture 后重新运行获得该结果。

生产模块、测试及诊断脚本通过 Python 3.8.10 `py_compile`。本次不改 Rust，不触发全量构建。

相邻 `test_assigned_asset_storage.py` 在当前基线 65d15c5 下为 18/19 通过。唯一失败 `test_assigned_finalize_rejects_legacy_current_before_hash_or_body_read` 仍期望拒绝 legacy current；当前 `finalize_assigned_map_asset_bundle` 已允许将该情形发布为独立 assigned 版本。恢复旧密集云归约后该测试仍以相同方式失败，且该 fixture 没有 dense archive。测试与存储文件均核对为 65d15c5 的精确 Git blob。本次保留该行为及测试，未把这项既有不一致计作通过。

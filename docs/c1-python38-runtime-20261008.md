# Python 3.8 真实运行兼容修复

> 历史验证记录：文中一次性实验及已失效检查入口已于2026-10-22防御性编码优化中退役；下列旧命令仅说明当时证据，不作为当前运行入口。

日期：2026-10-08。基线：`cb80236497088eb9d5358212dfe09b93cff53002`。

## 修改

Python 代码以部署端的 **3.8.10** 为兼容基线，本次使用隔离 CPython **3.8.20** 实际执行，而非仅在新解释器中指定 AST 语法版本。两者版本标签不混用。

- `contracts/map_activation.py`：即时求值的 `set[str]` / `dict[str, Any]` 改用 `typing.Set` / `typing.Dict`，解决导入即报错；原数据和校验行为保持
- `contracts/runtime_views.py`：`Annotated` 从 `typing_extensions` 导入；通用、模块化和兼容 requirements 及 Jetson constraints 显式声明 `typing_extensions==4.13.2`，其官方包元数据支持 Python >=3.8，满足现有 Pydantic/FastAPI 约束
- 场景决策服务：以既有事件循环默认 executor 加 `copy_context().run` 替代 3.9 才有的 `asyncio.to_thread`，保留线程卸载、上下文传递和异常传播，没有另建线程池
- 重定位返回格式：以 `startswith` 和切片替代 `str.removeprefix`，只移除开头的一次固定前缀，未修改算法或阈值
- 导航文件身份测试：替换 3.9 的 `Path.is_relative_to`，保持原路径范围断言

初次 3.8 导入的 `set[str]` 和 `typing.Annotated` 失败记录保留；修复后才执行以下资格检查。

## 实际检查

环境：CPython 3.8.20、SQLite 3.46.0、Pydantic 2.10.6、NumPy 1.24.4、FastAPI 0.115.14、typing_extensions 4.13.2。环境不替换系统 Python，也未改用户机器。

- 571 个 Git 跟踪 Python 文件用实际 3.8 AST 解析；该范围无语法错误
- 40 个 contracts 模块导入及 156 个本模块 Pydantic 模型 JSON schema 求值通过
- `scripts/check_map_snapshot_revisions.py`：31 项通过
- 两项现有 assigned DTO 定向检查通过：`test_request_normalization_immutable_roundtrip_and_small_target_admission`、`test_all_retained_phases_roundtrip_without_source_payload`
- `scripts/check_python38_runtime_compatibility.py`：8 项通过，使用实际 FastAPI 已注册 endpoint 和 lifespan、假 controller；覆盖线程非阻塞、ContextVar 复制且不回漏、原错误映射、关闭路径、实际枚举和固定前缀行为
- `scripts/check_navigation_file_identity.py`：31 项通过
- 已发布 SQLite 改动的 17 项聚焦测试和 205 项定向回归，加 recall equivalence 脚本，通过；详见 [SQLite 资格](c1-sqlite-reader-pool-20261008.md)
- 配置备份／迁移的 12 项聚焦和 11 项既有离线检查通过；详见 [去摘要资格](c1-config-migration-hashfree-20261008.md)

各组包含重叠路径，不合并为全平台通过率。最小复现命令：

```sh
python3.8 -B scripts/check_python38_runtime_compatibility.py
python3.8 -B scripts/check_navigation_file_identity.py
python3.8 -B scripts/check_map_snapshot_revisions.py
python3.8 -B scripts/check_memory_sqlite_reader_pool.py
```

## 限定

本批替换使用的 typing 容器、ContextVar / 默认 executor、字符串切片和 pathlib 路径操作均已存在于 3.8.10；相关已安装包元数据声明 Python >=3.8，没有更高补丁版本门槛。线程上下文用法见 [PEP 567](https://peps.python.org/pep-0567/#offloading-execution-to-other-threads)。这不等于整个 3.8 补丁系列行为完全一致，也不代替 3.8.10 的实际运行；目标机 SQLite 编译版本、原生依赖和硬件资格另行区分。

审计覆盖当前语法、明显新版本 API 和合同模型求值；它不是所有惰性执行路径的兼容证明。新测试直接调用注册 endpoint，没有启动 HTTP listener、模型进程或设备；没有执行完整依赖安装、原生 SDK 或 Jetson CUDA/TensorRT 集成。本次也未恢复已搁置的三维定位／支撑专项，用户实机测试独立进行。

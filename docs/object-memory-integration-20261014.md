# 物体记忆集成验收 · 2026-10-14

## 结果

- 新增真实摄入链路验收：**18/18 通过**，最后一次在最终巡检位姿调用清理后重跑
- 既有记忆集成回归：**77/77 通过**，77 个节点互不重复
- 生命周期补充复查：**30/30 通过**，其中 18 项是上述验收的重跑，另 12 项与生命周期专题测试重叠，不能重复累计
- Python 3.8.10；最终各轮应用内容哈希尝试均为 **0**
- 诊断脚本编译检查及 `git diff --check` 通过

此报告仅统计自己的验收选择，不叠加其他专题的单元测试、JavaScript 测试或性能测试。

## 验收范围

入口为 `scripts/diagnostics/object_memory_integration_20261014.py`。使用合成 `PerceptionContext`、真实二维/三维投影、实例关联、`MemoryService.ingest_perception_observation`、SQLite 写入和关闭重开，以及公共查询、列表、快照和实际 `/api/memory/instances` 路由。HTTP 通过离线 FastAPI 客户端及实际能力处理器验证；网关授权分发、定位供应方使用测试适配器。后台模型线程关闭，嵌入向量为明确的固定测试值。

关键结果：

- 伞在换相机、换跟踪号、图像位置大幅变化、18 小时后重启等条件下更新原 ID
- 两只宽 8 cm、中心相距 10 cm 的杯子保持两个 ID；分开扫描、批次重排和次日 5 mm 位置抖动均不混并
- 三维检测尺寸、方向、计数和选定 ID 可真实落盘并重载；公共语义检索和 HTTP 返回对应数据
- 后续无几何帧保留可靠位置和尺寸；较弱的可见点支持不缩小已有检测框；尺寸未知保持 null
- 同一观测重放不会重复累计；同批两相机的最终证据完整发布，每个实例只写入一次，累计 4/6 次后重启保持一致
- 同相机、同来源、显式同跟踪号的 3D/2D 成对证据只记一次，保留检测器尺寸；缺失或不同跟踪号的同帧检测仍各自独立
- 不同记忆库、坐标系、已分配地图资产分开；同一资产的定位会话和运行时来源代际变化不制造新物体
- 捕获快照保留自己的地图资产身份；临时没有定位会话不会抹掉旧资产绑定
- 点云采样密度改变时，显示框仍位于测得边界的中点，导航位置继续使用原有平滑结果

同一时刻的首批多视角观测仍须满足原有静态历史条件才会持久化，没有为了测试放宽准入。

## 历史数据和接口边界

**已有重复记录不会自动消失。没有执行历史记录合并、删除或清理迁移。** 两条历史重复记录在读取后仍保留原 ID；后续扫描选择并更新其中一条，不再为同一观测新增第三条。

实例列表增加可空的 `object_extent` 字段，其他列表字段及导航位置语义保留。原生命周期比较夹具只增加 `object_extent: null`，未移除原有断言。旧边界元数据可直接读取，不需要查询时重写数据库。

这些检查验证数据链路与合成几何，不能证明实机测量精度或不可见部分的完整尺寸；未进行机器人测试、真实浏览器/WebGL 画面验收、完整构建或部署。

## 本轮发现并修复的问题

- 旧 STATIC 记录没有短时观测窗口时，重见一帧曾被降级并从 SQLite 删除；现在保留与当前几何相容的既有静态证据
- 可见点分布变化后，框偏移量曾相对于错误的平滑位置计算；现在按最终位置重定基准，测得中点不漂移
- 同批多来源命中一个 ID 时曾只发布第一份结果；现在提交最终累计证据一次，计数和最新尺寸可重载
- 没有定位会话的帧曾用空值覆盖旧资产 ID；现在保留已经建立的绑定

另保留一次测试预期修正记录：首批同时观测尚不满足静态持久化条件，这是原有策略，非生产缺陷。

## 复现

在仓库根目录，使用现有 Python 3.8 环境：

```bash
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
python scripts/diagnostics/object_memory_integration_20261014.py \
  --output-dir ../neh-evidence/object-memory-integration-20261014/final-acceptance
```

77 项既有选择为：持久索引集成 21、快照载荷所有权 4、制品生命周期 27、摘要投影 2、场景身份集成 23。可用现有保护器复现该选择：

```bash
python - <<'PY'
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch
from scripts.diagnostics import verify_python_guarded_integration as guard

targets = [
    'scripts/check_memory_durable_integration.py',
    'scripts/check_memory_snapshot_payload_ownership.py',
    'scripts/check_memory_artifact_lifetimes.py',
    'scripts/check_memory_summary_projection.py',
    'scripts/check_memory_scene_identity_service.py',
]
with tempfile.TemporaryDirectory() as temporary:
    selection = Path(temporary) / 'targets.txt'
    selection.write_text('\n'.join(targets) + '\n')
    argv = ['guard', '--output-dir', '../neh-evidence/object-memory-integration-20261014/recheck']
    with patch.object(guard, 'TARGETS', selection), patch.object(sys, 'argv', argv):
        raise SystemExit(guard.main())
PY
```

最后的生命周期复查使用同一无应用哈希保护，选择新增诊断脚本与 `scripts/check_memory_instance_lifecycle_views.py`，共 30 项。

## 原始证据

原始 JSON、日志及修复前失败证据保留在云工作区的 `../neh-evidence/object-memory-integration-20261014/`，不写入仓库：

- `final-acceptance/object_memory_integration.json`、`final-acceptance.log`
- `existing-regressions.json`、`existing-regressions.log`
- `post-final-guard.json`、`post-final-guard.log`
- `legacy-static-before-fix.json`、`extent-rebase-before-fix.json`
- `multi-camera-fixture-expectation.json`

既有回归耗时 18.56 秒；30 项复查耗时 2.37 秒。pytest 仅出现一次已导入 `anyio` 无法重复断言重写的提示，无测试失败。

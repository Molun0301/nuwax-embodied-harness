# 点云配置声明与执行状态资格记录（2026-10-08）

## 范围

在 53f67836 的点云执行链上修正配置的可观察语义，没有改变扫描采样、建图、MLS/PCT 分辨率或运动校验。十个旧点云预处理字段和 `NUWAX_MAPPING_VOXEL_SIZE` 目前仅被读取/保存，不被内置点云执行链消费。保持旧键和值以兼容外部消费者，明确标为 `inactive`，不把保存、重启或 Python 读取配置当作执行回执。

两个预处理元数据节点保留原字段，追加 `status=inactive`、`reason=legacy_configuration_not_consumed`、`value_semantics=requested_configuration`。新元数据的 enabled 等值是请求声明，不代表执行。没有改写历史资产；旧资产未记录执行状态，不能用当前参数倒填历史。

配置 schema 不给 inactive 项提供伪造的 effective_value 或 pending_restart。仅保存这些字段不要求重启；混合修改仍保留真实参数的重启行为。设置页面显示“兼容保留，当前流水线不使用”。真正由独立 Rust 进程消费的导航粗体素和 raytracing 体素使用 external_process 口径，网关读到的值不构成后端采用回执。

RuntimeInputBridge 状态新增 scan_sampling，直接读取实例的触发/目标点数，区分原生同步字段并保留扫描端点/最大时间偏移的策略与旧帧 stride 策略。这里只报告既有策略，没有新锁、点云扫描、全量复制或哈希。

## 验证

- Python 3.8.20：8 项新增配置真实性测试和 28 项既有配置修订测试通过；既有测试的 ast.unparse 改成结构化 ast.dump 比较，使测试本身兼容 Python 3.8
- Python 3.12.14：8 项新增测试通过
- Node：执行真实设置行渲染函数的 4 种状态检查通过，覆盖 inactive、独立进程未知、当前进程未知和待重启；这不是完整浏览器端到端验收
- 既有 T10 RuntimeInputBridge harness 通过，实际状态包含实例 100000/50000 阈值
- git diff --check 通过
- 精确 Python 3.8.10 没有执行，不将 3.8.20 冒称 3.8.10

初次导入路径错误、catalog 字段缺失、测试预期/启动快照夹具修正以及既有测试 ast.unparse 在 3.8 的失败均保留日志。最终结果使用修正后的源码，不覆盖初次失败。

## 仍未解决

本次不启用旧 max_points=1200 预处理，不将语义逐帧数据换成累积地图，也不修复动态足端回波、时序配对或局部搜索。实际 fine 分辨率由当前 manifest/归档提供，新建默认 5cm 不代表所有历史地图都是 5cm。显示点云粗化、导航索引和连续运动采样不能混为同一种精度。

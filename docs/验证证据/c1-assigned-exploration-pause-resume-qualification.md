# 分配式探索暂停、保存与恢复

日期：2026-10-06 UTC。唯一工作分支 `fix/v3-legacy-cloud-seal-20261006`，
远端基线 `c0a996858ef6bf4bedad36cfba076ffc91810de0`，
源树 `00e1e496f32d3988314d282be1bbe354b41c38ea`，对应本地同树检查点
`4d727671d1341d4ea75b295b9c9c7d3ff36d9152`。发布只把本次源码增量接到真实远端父提交。

## 本阶段接通的行为

- 公共探索暂停沿原停车屏障取原生停止进度，完成本次完整 schema-2 点云保存，再发布检查点
- 新检查点保存明确的 asset ID、mapping checkpoint ID 与原始不透明进度；内层已验证源凭据
  核对原 map/frame/session/generation，外层 metadata 摘要不能冒充源证据
- 第二次暂停或正常探索完成以原任务操作 ID 关联新保存，原建图会话内内容推进也会重新导出；
  相同操作重试重开原 current，不重新分配身份。调用者普通 metadata 不能伪造该操作字段
- 已有 cloud current 使用准确前驱 ID 交换；候选准备、完整验证、树 fsync 与退役目录清理
  在目录守卫外完成，只沿既有短目录守卫提交拥有方检查、交换日志、rename 与小型元数据
- 前驱/候选身份、固定目录 inode 与原目录创建身份组成明确恢复证明；日志保留到目录资产
  元数据持久化完成，不通过放宽普通 getter 的身份要求恢复，也不生成应用内容摘要
- 恢复使用公共 activate_map/relocalize_active_map 及既有显式前驱停止生命周期；源进度会话
  与新安装会话分别核对，不能把旧 source session 当成新 installed session
- 原重定位 request ID/timeout 在调用前保留到任务元数据，完整原生请求沿既有目录日志持久化。
  未跨 dispatch 的 prepared 原请求可显式继续；已 dispatch/未知结果只查询同一原请求
- 返回前核对准确终态请求、当前安装、同一 READY 会话、工作区与新鲜原生 sample。READY
  之后被替换、降级、缺失真实 sample 或改变目录拥有方都会拒绝继续；不使用历史 pose seed
- 完整资产读取、原生 RPC、发布回调不占用运行态/provider/session 元数据锁；未新增全局锁

二维/显式旧格式读取保留原协议。旧探索记录在 current 被新资产替换后仍可列出，但明确不可
恢复，不把新地图绑定到旧记录。记录写入的 rename/fsync 失败允许重试原字节和 created_at。

## 验证范围

**Python 83/83 通过**：57 个新增用例、26 个直接受影响回归；分组为 56/56 存储合同、
20/20 公共两轮循环与恢复、7/7 旧格式兼容。两处审查发现已修复并用针对性回归覆盖。
新增测试 Ruff、修改源码 critical Ruff/编译、Python 3.8 语法、分层硬规则及 diff 检查通过。
触及文件原有的一处 PEP604 参数注解被兼容性检查发现，已窄改为 Union 并通过复核，失败日志保留。

最终精确命令、用例和源码身份见配套
`c1-assigned-exploration-pause-resume-results.json`。只运行新增及直接受影响行为，未重跑分支
清理测试或完整项目套件。本阶段不修改 Rust/protobuf，不重编原生 SDK 或重复之前的原生资格运行。

公共 Python 验收使用真实 gateway、workspace、目录/资产存储、provider 合同、定位服务及
READY 发布。原生 RPC、任务外壳和外围记忆/导航端点由 fixture 模拟；两轮完整保存/恢复并非
实机探索、LCM、完整导航执行或停车屏障硬件资格。原始实时障碍和运动准入逻辑保持原样。

## 后续边界

- saved dense-cloud/display 消费者尚需配对 assigned 身份
- 回到实际局部规划/通行性性能验收，验证最新整合代码的功能与代价
- mapping-cloud/place/georeference/Nominatim 的旧摘要 authority 路径仍是较低优先级未完成项
- 本阶段不声称多楼层合同、所有 PGO 配置或硬件机器人已完成资格

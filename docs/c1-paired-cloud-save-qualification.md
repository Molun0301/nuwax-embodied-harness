# 普通点云保存与分配式工作区加载配对

日期：2026-10-06 UTC。基线为唯一工作分支
`fix/v3-legacy-cloud-seal-20261006` 的远端提交
`5b4afa1939e4af043c48fd6cac1d559352677836`，源树
`3d33a73c6d1f5972c8dbcae68e06297cf5bfaed1`。
本地 `27e9389aa10445bfbbb0ad24e02ec2250f9526b3` 是同树映射，
不是要发布的真实远端历史。

## 本阶段接通的路径

公开 `finalize_active_map` 根据实际原生建图会话选择协议：voxel/raytracing
使用 checkpoint schema 2；slam_toolbox 保留二维图路径。未知后端拒绝保存，
schema-2 导出失败不会改走 schema 1。低层显式 schema-1 默认值和旧资产读取兼容保留。

顺序是：准备可变草稿 → 完成同一原生 checkpoint 的 SQLite/Nav2/PCD/成员落盘 →
只分配一次 `AssetIdentityV2` → 构造不可变候选 → 原子 finalize → 按精确身份 reopen →
短目录事务记录 schema-2 元数据。资产 ID 与原生 checkpoint/member ID 保持不同职责，
没有新摘要、摘要别名、额外资产版本时钟或保存全局锁。

保存后显式 `activate_map` 使用已发布的工作区激活实现：记录原六字段请求，明确停止
所拥有的前驱，再加载同一资产/checkpoint 并采用真实回执。保存本身不激活、不播种 pose。
活动资产保存后的状态是 MAPPING、localization_ready=false；未安装 assigned owner 时
工作区 getter 不返回伪造工作区，定位同步也不创建旧协议会话。加载后为 LOCALIZING。

## 恢复、并发和开销

- current 已发布但元数据写入失败时，重启/重试读取同一个 current，不重新导出或分配 ID
- rename 可见而目录 fsync 失败时，显式 finalize 重试重新完成父目录和小型元数据持久化
- 保留目录 inode、创建时间和明确资产身份，删除/同名重建不能接收旧保存结果
- 正文读取、原生 RPC、资产准备/发布、StateStore/EventBus 回调在目录/运行态守卫外
- 目录 guard 只保护已有小型元数据事务；二维/显式旧路径的原有锁范围未作全局重构
- 事件/状态回调若替换拥有方，旧调用不返回成功；条件清理只撤销自己的缓存对象
- 准备取消清理原暂存 inode，提交后的 current 和别人替换的暂存目录不会被删
- schema-2 导出固定原 stub/连接与 runtime attach owner，核对后端及会话代次
- 已准入的 fine/dense artifact 在原 stub 上释放一次；清理失败不覆盖原错误/取消
- 占据栅格保留 packed bytes，Nav2 读取行视图；PCD 流式写出，免除整云 protobuf 重建
- schema-2 使用 checkpoint 所有的小型身份 SQLite，不复制无关 live RTAB 数据库

## 验证

最终 focused Python 资格验证：**46/46 通过**，其中公开保存测试 25、运行态/导出新测试 15、
直接受影响的既有准备/导出/重开回归 6。逐例名称、运行命令、日志与源文件摘要见
`c1-paired-cloud-save-results.json`。独立源码审查通过，新增文件 Ruff、修改文件编译、
Python 3.8 语法检查、分层硬规则和 diff 检查通过。

测试用真实 workspace/runtime/provider、protobuf 准入、SQLite/Nav2/PCD、资产与目录
存储、MappingService/LocalizationService；仅原生 RPC 和外围记忆/导航端点模拟。
覆盖普通 voxel/ray 保存到加载、字面旧资产读取、二维分派/后端漂移、hash/copy/锁探针、
持久化失败、取消、创建拥有方替换和回调竞争。新保存资产丢失 ActivateMap 回复后，
新 runtime/service 只查询 GetMappingState 对账，保留原六字段请求，安装次数仍为一。

## 明确的剩余边界

- 没有运行完整 Python 套件、重编译原生代码、实机测试或真实 gRPC 网络测试
- 原生 assigned 安装仍没有可宣称的真实重定位目标；本阶段不提供 READY/导航能力
- 探索暂停检查点、楼层资产合同、finalized navigation authority 和 saved dense-cloud
  查询仍有旧摘要协议消费者；它们尚未完成 assigned 端到端配对，不能以摘要别名绕过
- 冷启动只采用当前原生封存证据；未知结果、代次改变或查询失败不授权猜测重执行
- 原生导航/探索运动守卫及实时障碍检查没有放宽

以上是普通点云资产保存/加载与恢复的有界提交，不是所有导航、楼层或探索流程完成迁移。

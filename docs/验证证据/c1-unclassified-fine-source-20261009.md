# C1 单帧未分类 fine 来源接线资格记录

日期：2026-10-09。父提交：919ff00e77b650ee899f8326db7f837cf17e8252。本阶段只接通单帧未分类返回的临时来源能力。

`AcceptedRegisteredCloud` 以共享 owner 保留已有 envelope、原始 map 点及类型化注册绑定；异步接线传递惰性能力，沿用已有 canonical admission、活动会话、代次、模式和 correction 检查。只有真实、成功且非空的 fine 累积才首次获取 `AdmittedRegisteredScan`，保留同一体素内的不同返回，不预先将它们合成一个表面。已有 history/correction 与 fine 暂存可共享同一 body 点分配。

v0 在深层检查前调用 `scan()`，会提前物化 body 点；v1 改为共享惰性 owner。冷 canonical 重复、过期代次以及等待 lifecycle 锁后模式失效的输入均不触发 body capture。v0 仅在已有 root 的 rehome 中清理暂存，遗漏 rootless 初始帧切换；v1 在真实 frame/producer 所有权切换处统一退休暂存，覆盖无 root 和在途 private work，未增加拒绝门槛。

`latest_unclassified_fine_scan` 表示最近一次成功且非空的 fine accumulation 来源，不表示最新输入或当前已发布 root。空输入和被拒绝输入不替换它；无 typed source 的成功 legacy 累积、实际 owner 切换及新建/读档生命周期按既有入口清空它。该暂存只保留一帧；历史保留仍依赖已有 ScanHistory，不能将本阶段当作历史或撤销账本。

真实成本：新增 accepted owner 的 Arc 分配；首次有效 scan 仍需 O(P) 逆转换以及 body Vec/元数据分配和既有有限性记账。惰性 owner 克隆不复制原始 map/envelope 缓冲，但可能延长其在异步任务期间的存活；已有异步 points.to_vec() 未改。共享已捕获的 body 分配不等于全过程无分配，冷拒绝测试只证明未触发 body capture。

验证：Rust 1.88.0、既有 SDK/target、增量 release、locked/offline。共 **55 个不同测试通过，0 失败，4 项原有测试保持忽略**。覆盖真实 in-process 接线路径、冷拒绝、深层过期代次、等待锁后失效、同体素不同返回保留、首次捕获及 history/correction 分配共享、空输入语义、发布 root 复用、实际初始帧切换和在途工作退休、fine 读档与既有 pointcloud/来源/历史回归。v0 的 50 项记录及 v0/v1 候选独立保留；本次发布未重复构建或测试。

旧档仅有原占据证据，不能恢复原始返回或统计；本阶段不新增统计量、表面关联、历史贡献模型、持久化/传输格式、PGO 重建或导航消费。没有修改拟合、真实避障、净空或控制器，也未证明或解决地面高度跳变、局部规划卡住。后续关联和持久化另阶段审查。本次不部署、不更换 18899 服务、不启动机器人；远端 CI 仅以实际返回状态报告，未知或零项不能视为通过。

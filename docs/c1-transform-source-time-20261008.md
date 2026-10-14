# TF 源时间保留（2026-10-08）

## 缺陷与修复

通用 GeometryRuntimeProvider._build_frame_tree 从 protobuf Transform 转换时没有提供 timestamp，契约默认值因此变成处理时刻。协议已经携带每条变换的 uint64 timestamp_ns，相邻 _build_pose 也已保留源时刻，信息丢失发生在 Python 转换边界。

现在把各条变换的源纳秒时间转为 UTC datetime：整数秒加微秒余数，保留现有 Python 契约精度，不经过浮点秒。零值或缺失字段延续 Pose 的未知 epoch 哨兵，不伪造为新鲜采样。不增加过期阈值、不改变变换数值或准入条件。FrameTree 自身仍是构造快照时刻，不代表所有边共有同一个采样时间。Assigned localization 的配对边路径保持原行为，仅更新已过时的解释注释。

## 验证

- 修改前新增 3 项检查全部失败，确认为源时间被处理时间替换
- 最终检查首例使用真实 protobuf，覆盖非零纳秒时间与 Pose 一致、亚微秒截断、零/缺失哨兵、不同边各自时刻、拓扑和空树
- Python 3.8.20、3.12.14 各 3 项通过
- Python 3.8.20 既有 assigned localization sample 18 项通过
- git diff --check 通过；没有重建 Rust 或运行设备
- 未执行精确 Python 3.8.10，也没有证明相机/点云/TF 已严格同步

## 后续边界

语义投影仍在模型推理后读取 latest 点云/TF；保留 TF 源时间不会自动修复跨时刻的动态逆变换。入口捕获引用、同帧原始与已注册数组的共同索引复用须单独验证，不能把本次修复描述为整个语义同步完成。

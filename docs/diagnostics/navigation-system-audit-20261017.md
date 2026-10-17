# 导航链路专项检查与修复（2026-10-17）

## 范围与基线

基线为 `19c238c`，保留控制台预览页面的既有更新。本轮覆盖 Python 导航编排、三维地图发布/接收、地面解释、局部搜索、运行时控制复核，以及共享地图的探索接入。测试环境为云端 x86_64、Rust 1.88、Python 3.8.10；没有连接或操作真实机器人。

历史实机 CPU 接近满载、规划反复等待，以及 10 月 14 日旧版本日志是排查线索，不能作为当前版本仍存在同一根因的证据。本轮以源码调用链、失败回归、同条件前后测试确定改动。

## 修复点

1. **目标状态所有权。** 旧目标终态、同 ID 重试期间连续迟到终态，不能清除正在提交的新目标。后端权威状态统一投影到 full/lightweight 读取；本地旧状态不能把 RUNNING/PLANNING 改成 FAILED/IDLE。匿名终态不继承新目标的诊断身份。复用既有身份和目标引用，无新增锁或线程。
2. **地面解释一致性。** 原绕行提示依赖中心粗地面锚点，可能把实际机体检查认可的细粒度实测地面当障碍。机体检查与提示现在共用单细列选择。保留实测成员、未知事实、真实占据和扫掠检查；不合成观测、不增加支撑硬门槛。
3. **搜索预算与到达。** 同一精确七维姿态已有更低代价时，严格跳过旧队列标签，与入队支配关系一致。已满足既有三维到达区及朝向条件的状态，直接结束，不再向点击中心转过去再转回来。未扩大预算、合并姿态、改变启发式或碰撞规则。
4. **连续地图更新的控制复核。** 完整连续的 Columns 更新，在地图查询语义未变时，可将已选命令的完整停车轨迹在最新地图上重新验证，不必每次重新选控。Unknown、来源链断裂、语义变更仍重新计算；近障、原许可截止及最早观测期限仍有效。
5. **禁区几何与提交时间分离。** 保留旧完整地图的比较不再把 `map_load_time` 当作几何变化；仍比较坐标系、分辨率、尺寸、原点姿态及像素。只影响可重试缺块期间的保留判断，不延长旧来源期限。
6. **优先传输变化块。** 每条消息仍包含完整根，inline 128 KiB 额度优先给同域同分辨率下真正变化的对象，避免空间排序靠前的旧块挤掉新块。晚加入或漏帧接收者仍按完整根可靠补取；不完整根不安装。
7. **探索接入复合地图修订。** 点云根未变而禁区/occupancy 更新时，按原有 cloud 与 occupancy 修订、owner commit 接入；安装顺序在同域内比较。清空、换域与旧结果隔离保留，不创建新的版本时钟。

## 已验证的边界与量化结果

- Python：16 项新生命周期回归、24 项激活/取消、7 项地图身份、3 项失败反馈，共 50 项通过。重复 full/light 交错与同 ID 跨地图重试经独立检查。
- 地面：133 项 body 测试通过；新增错误提示反例先失败后通过。另有 9 项显式忽略的诊断/基准，不算功能通过数。
- 地面性能：5 对交替样本，每组 20 次，独占运行。Measured 场景配对中位耗时为旧实现的 0.83–0.96 倍；Legacy 场景仍有额外语义查询，样例最大增加约 0.282 ms。初版整粗列扫描的回退已经消除。这里只是 hint 查询，不等于整机导航加速比例。
- Scan：profile 配置整套 58 通过、5 忽略，另执行 5 个实际近期导引案例与 exported B。一个固定 256 展开场景的较差同姿态重复出队由 6 降为 0；有效新节点及扫掠略增，不能据此声称总延迟或内存降低。
- 控制复核：3 次远处完整 Columns 更新，完整选控次数从 4 降为 1，并执行 3 次真实轨迹复核。真实近障、Unknown 合并和查询语义变化负例通过。
- 地图保留：8 项 incomplete_refresh 测试通过，包括真实提交时间变化、禁区变化撤销、源期限、近期扫描/位姿期限、清空换域及取消。
- Geometry 在线 binary：18 项目标测试通过（transfer 3、探索表面 9、metadata/noop 1、地图身份/clear/双层禁区 4、occupancy watch/exposure 1）。旧空间前缀在尾块变化例内联 10 块仍需 1 次补取；新 delta 仅 1 块 12,304 字节，已有缓存 0 次补取，晚加入者补取其余 11 块并完整恢复。
- 最终运行时默认配置：245 通过、0 失败；5 项已确认环境 EPERM 的 UDS 测试明确过滤，58 项默认忽略。随后独立执行其中 2 项，均通过，另 56 项未执行。
- 额外连续控制：原有 held 规划用例通过，20 Hz 消费 tick 最大 51.50 ms（原上限 75 ms），内层控制最大 9.22 ms（原上限 50 ms），地图更新最大 0.21 ms，未接受陈旧/过期控制。只验证 1.02 m 连续行进，不冒称到达该用例 7 m 终点。
- 额外动态恢复：真实调度器加假运动端口的清障用例在 13.69 s 完成同一原始目标；236 次夹具回执、733 条实际算法运动边，完整制动/碰撞历史审计无失败。不是实机。

## 旧测试与环境限制

运行时整套首次有 2 个非环境失败。精确恢复基线 `19c238c` 的相关生产文件、同 Release 配置重跑，两项均原样失败。原夹具仅删去地面采样，却仍期待“必须阻塞”或“必然局部规划失败”，与已有 Unknown 可候选语义不一致。

测试修订保留真实算法：缺测通行正例、真实 40 cm 占据阶面 ON/OFF 负例、真实 Scan 滚动窗口与恢复；另加 typed Budget/SearchExhausted 注入，只验证状态处理，再恢复真实 owner 检验恢复。未降低生产门槛或延长测试时限来凑通过。

5 项真实 Unix socket 夹具测试在云端返回 EPERM；正常提升执行重试仍失败，未修改系统权限。它们不能计为通过，也不能据此认定生产算法失败。

云端解包 PCL/FLANN 配置引用系统绝对路径，以及 HDF5 serial 搜索目录缺失，已仅在工作区工具链/测试 shell 修正；未修改项目部署逻辑或系统安全设置。

## 一手资料与适用范围

- [PCT 原作者论文](https://byangw.github.io/assets/pdf/yang2024efficient.pdf)：地面与上方障碍分层表示；不支持把所有点直接混合平均。
- [ANYbotics elevation_mapping](https://github.com/ANYbotics/elevation_mapping)：分辨率、测量/位姿方差与融合分别建模。
- [robot_body_filter 官方实现](https://github.com/ctu-vras/robot_body_filter)：基于模型、TF 与时间处理自体回波；不是任意擦除近身占据。
- [Nav2 搜索](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/include/nav2_smac_planner/a_star_impl.hpp)及[平滑](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/src/smoother.cpp)：搜索状态、路径有效性及失败降级的参考。
- [Nav2 一致重规划](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/trees/nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid/)：按目标、路径失效与周期触发，区分地图更新与任务生命周期。
- [ROS 2 Action 设计](https://design.ros2.org/articles/actions.html)与[Nav2 非阻塞编排](https://docs.nav2.org/jazzy/configuration_and_development/simple_commander_api/simple_commander_api/)：反馈、结果、取消与每目标状态职责。
- [Voxblox 论文](https://arxiv.org/abs/1611.03631)与[官方更新块接口](https://github.com/ethz-asl/voxblox/blob/master/voxblox_ros/include/voxblox_ros/conversions.h)：增量更新与更新块传输的参考；NEH 仍使用自己的完整根与可靠补取协议。

这些资料用于核对原则，不是本项目修复有效的替代证据。实机最终验收仍需在同一地图与传感器配置下记录成功率、路径质量、停顿/恢复、CPU 和端到端延迟。

## 复现入口

在配置好项目原生依赖的环境中使用 Release 构建，复用已有 target：

```sh
python3.8 scripts/check_navigation_service_lifecycle.py
cargo test --release --locked --manifest-path services/nuwax-workspace/Cargo.toml -p nuwax-nav-mls-planner --lib surface::body::tests::
cargo test --release --locked --manifest-path services/nuwax-workspace/Cargo.toml -p nuwax-nav-scan --features profile --lib
cargo test --release --locked --manifest-path services/nuwax-workspace/Cargo.toml -p nuwax-nav-stack --features mls incomplete_refresh -- --test-threads=1
cargo test --release --locked --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime navigation_map_transfer -- --test-threads=1
cargo test --release --locked --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime surface_composite_mask_receipts_advance_without_rebuilding_cloud -- --test-threads=1
```

实际运行时与性能测试须串行、停止其他构建，并核对 Cargo 返回的测试产物与执行计数；`--lib` 不覆盖 Geometry 的在线 main binary。首次误选 lib 的零测试、夹具配置失败与测试受阻均未计入通过结果。

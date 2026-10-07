# Assigned-map 组合测试：重定位量化偏差记录

2026年10月7日（北京时间）。**组合测试仍未通过，重定位精度问题没有被修复。** 本阶段只修正测试根目录的创建方式，并保存已有算法回执和目标点云，未修改生产匹配算法、原场景、校准或20 mm断言。

## 执行与源码

- 基线：`4c4c7bce70f32e8e6912d3342febf6dbc559f070`
- 唯一代码改动位于测试文件 `services/geometry/nuwax_geometry_runtime/src/assigned_navigation_composition.rs`
- 新建 UUID 测试根目录使用0700；不修改已有目录、umask或生产 journal 的权限要求
- 诊断增加：保存已有 canonical XYZ和DC1；打印已有重定位终态及定位样本
- 本机实际编译并执行的修改后文件：46481字节，SHA-256 `b98d68de755f5b637fb77410f1d62ca34aaf59606cb38254871b72b8e3c9731f`
- 测试二进制：124639344字节，SHA-256 `e9ad4969ecf81e43ae4dd8f5e6888e4ce15325dcdf2ba702c6f9bb2cfdcab45f`
- 继续使用已记录的混合 test profile；不是新 release 性能对照

完整 selector：

`server::lifecycle_tests::assigned_navigation_composition::assigned_v2_restored_production_navigation_local_off_arrives_and_expiry_stops`

## 运行顺序与结果

1. 原版 OFF 在默认受限环境的 Unix socket bind 处返回 EPERM，未进入 Geometry。
2. 同一测试经正常环境授权执行后，越过 bind，但测试预建根目录为0775，不满足已有 journal 的0700要求。
3. 父任务指定的测试目录修正编译通过后，OFF进入重定位，Z残差约24.54 mm，超过原20 mm断言；未进入导航目标阶段。
4. 保持原输入及断言，仅增加上述诊断后，OFF运行一次，退出码101，耗时5.624180秒。父进程运行包装器，子进程实际执行组合一次；两层失败报告对应同一次执行，不算两个不同场景。Z残差为24.602243199525753 mm。ON没有运行。

既有失败记录全部保留。测试仍使用合成传感器和 SimulatedActuator，没有真实机器人运动。

## 本次实际保留的证据

- 被准入 source：50000点。
- Canonical target：6031点，体素约0.05 m；5551个地面点的Z均为−0.375 m。
- DC1：18201点、6067个唯一点，地面Z为−0.40000000596 m。
- 两份目标表示的地面差约25.000006 mm。
- 算法回执：converged=true、accepted=true、迭代4次、fitness=1，MSE=0.0001097718846766694 m²。
- 回执变换与实际应用的定位样本一致，矩阵组合残差约4.44×10⁻¹⁶；未发现应用层另外增加25 mm偏移的证据。

完整日志、算法矩阵和原始目标文件已保留在执行机的本次诊断目录。成功路径原有临时清理规则未改变；本次失败保留了原始目录。

## 能说明什么，不能说明什么

当前 assigned-cloud 合同明确使用 canonical occupied-cell centers，而不是DC1原坐标历史：

- `pointcloud.rs`保留体素键，并按 `(key + 0.5) × voxel_size`输出中心坐标。
- `CanonicalCheckpointOwnerV2`、成员角色及安装收据绑定 canonical target；`correction_coordinator.rs`读取该拥有方的XYZ作为匹配目标。
- DC1保存原坐标和重复观测，但它的历史／动态过滤含义与canonical不同，不能直接替换目标而声称行为等价。
- 先前由canonical目标字节生成source的配对测试，验证了身份与链路，不证明原始非中心点场景的无偏精度。

本次+24.60 mm偏差与保存表示的+25 mm差异相符，说明原始场景精度受表示量化影响。单点相对体素中心的位移界，**不是ICP位姿误差的通用保证**。该证据不支持给所有坐标减去常数，也不支持无条件改用DC1。

原20 mm断言仍保留，未将失败改为跳过或通过。组合功能和原始场景重定位精度需要分别定义验收要求；在进一步设计前，不把调整场景或断言视为精度修复。

## 后续

先明确原始扫描重定位的精度要求、目标表示和过滤语义，再决定是否需要保留测量坐标／质心或引入独立、版本绑定的注册目标。真实机器人、生产运动执行闭环、持续障碍自动恢复及Go2负载仍未通过本次验证。

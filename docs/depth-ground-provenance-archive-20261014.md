# 地面观测来源归档升级

本次升级保留地面样本的原始生产者与源时间，格式边界为 Manifest v4 / `NUWAXAR4` 和 `NUWAXBL3`。新 Writer 的根统一写 v4；仅含旧来源的块仍可复用 BL1/BL2。地面成员、拟合平面、法向和数字样本字段不因来源元数据升级而重新计算。

## 读取与写入边界

- 新读端读取 AR1/AR2/AR3/AR4。AR1/AR2 仅允许 BL1；AR3 允许 BL1/BL2；AR4 允许 BL1/BL2/BL3
- BL3 在每层增加可省略的 `sample_sources: [{source, sample_positions}]`。每个生产者只保存一份描述，通过数组位置关联原有数字样本；未列出的样本仍是 Legacy
- v4 根增加可省略的 `source_watermarks: [{source, source_timestamp_ns, source_sequence}]`。空列表可省略；根中不要求为每个保留样本补造水位
- `source.kind = lidar` 携带 backend、runtime_generation、sensor_frame_id；`auxiliary` 携带 sensor_id、clock_id、runtime_generation、session_generation。显式 Legacy 来源组和 Legacy 水位不允许写入
- 来源组不可重复、重叠或引用越界位置。水位按完整来源身份去重；不要求输入数组排序。辅助源允许原始 sequence 为 0，原始源时间不改写成 LiDAR 时间
- AR1/AR2/AR3 和 BL1/BL2 中没有的信息不会从外层检查点、加载时刻或当前运行代际推断出来。旧数据保持 Legacy；保存后也不会凭空获得生产者或时间元数据

各来源水位只在对应生产者域内有意义，不与根时间或其他来源的样本时钟作跨域比较；不同 clock_id、runtime_generation 或 session_generation 不混用水位。根时间仍是规范地图提交收据的时间，不保证来自 LiDAR。实际队列与 TF 配对、样本时间不超过对应占据观察时间以及占据观察时间不超过根时间的既有约束保持不变。持久地面观测不是 live 传感器健康续期证据。

clock_id 在这里区分来源身份，没有引入数值时钟转换或自动同步。调用方提供的辅助帧时间仍须适配现有 LiDAR/TF 时间轴：队列只接纳前一 LiDAR 时间之后、当前 LiDAR 时间以内的辅助帧，并按原时间查 TF、提交辅助观测后再提交当前 LiDAR。直接 FineCloud/归档 fixture 的低时间值验证命名空间隔离，不证明未同步相机可端到端接入。

辅助请求的摘要身份已移除。重复与冲突判断直接比较规范化后的已接纳点、原始保留索引、来源/时间/序号、标定、坐标系、原点，以及冻结采集绑定与 mapping 标志。等价的线布局或填充、仅发生在已过滤无效点或机体返回中的差异，不再制造冲突；不保留第二份原始请求，也没有替代内容摘要。旧归档没有类型化水位，新生产者代际和 clock 命名空间仅与自身持久标记比较。

## 配对部署与回滚

外层 assigned artifact identity schema 保持 2；归档字节、成员身份与原 occupancy/cloud receipts 的绑定规则不变。Python checkpoint 保存、重开、重传保留精确归档字节，不重写时间、不转换来源、不新增内容摘要。

旧严格读端不能读取 AR4/BL3，不能仅依赖外层 identity schema 2 判断兼容。运行时在现有 `RuntimeStatus.supported_assigned_fine_archive_versions` 中显式公布 4；Python 在 AR4 上传和激活前要求该能力，缺失时明确拒绝。AR2/AR3 的旧配对约定保持不变；AR1 仍要求原有的显式能力。

升级应配对部署 geometry runtime 与 Python 读端。新 Writer 保存后的 AR4 不支持由旧二进制直接回滚读取；回滚前保留升级前、旧程序可读的地图备份。这里没有执行用户地图迁移、覆盖或实机部署。

## 验证范围

合成回归覆盖旧版本读取、新版本结构、混合 LiDAR/辅助来源、独立时钟、辅助 sequence 0、来源重复与位置冲突、未知版本拒绝，以及 checkpoint 保存、重开和重传时精确保留原始字节。Python 3.8.10 实际运行；对 `services` 包初始化作隔离，不替换被测 archive、provider、store 或 protobuf 模块。

主要入口：

- `scripts/check_fine_map_archive_identity.py`
- `tests/test_fine_archive_vector.py`
- `tests/test_checkpoint_identity_v2.py`
- `tests/test_legacy_cloud_import.py`
- `tests/test_assigned_ar1_import.py`
- `scripts/diagnostics/check_ground_provenance_roundtrip.py <Rust生成的AR4路径>`
- Rust `planar_artifacts::assigned_identity_tests` 的 AR4 传输、原来源重开和旧版本读取测试

CPython 3.8.10 五组回归通过 127 项：archive 26、scalar/vector 8、checkpoint 44、legacy import 36、capability 13。另用实际 Rust Writer 生成的 1762 字节 AR4/BL3 完成 Python 读取及 checkpoint 导出、保存、重开、重传；三份来源与水位、辅助 sequence 0、i64 最大 LiDAR runtime_generation、不同来源下相同数字样本身份全部精确保留，禁止内容摘要的断言始终开启。

这些离线合成测试不是实机传感器回放、运动安全或 CPU/延迟验收。Rust 测试定义与本页 Python/跨语言结果分开计数；原始归档和运行日志保存在仓库外，不提交到 Git。

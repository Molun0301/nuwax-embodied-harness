## ADDED Requirements

### Requirement: 感知快速节奏 HIL 延迟验收
发布验证 SHALL 对 YOLO-World/TensorRT 低延迟模式记录硬件在环检测链路结果，并把快速 detector/overlay cadence 与完整感知 total 分开报告。

#### Scenario: HIL 延迟基准运行
- **WHEN** 目标 Jetson/GPU 上使用 `.engine` 连续执行感知检测基准
- **THEN** 验证报告 MUST 记录 detector 耗时、快速循环耗时、`video.detections` 发布间隔、快速处理帧数、连续观测摄入计数、全链路 detector/geometry/tracker/scene_description/total 的 P50/P95/最大值，并标明模型路径、runtime、图像尺寸和 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC`

#### Scenario: 完整链路仍慢但快速节奏恢复
- **WHEN** HIL 报告显示 全链路 total 仍为秒级，但 `video.detections` 与 连续观测摄入 接近配置频率
- **THEN** 运维报告 SHALL 判定检测框和三角测量输入已从慢链路解耦，并将 全链路 慢阶段作为后续独立优化项记录

#### Scenario: 快速节奏仍慢
- **WHEN** HIL 报告显示 `video.detections` 实际间隔仍明显大于 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC`
- **THEN** 运维报告 SHALL 优先拆分 detector 耗时、video source readiness、tracker duration、发布间隔 和 runtime scheduling，不得直接回退到点云预算、provider API 或 stream 点云限流方向

### Requirement: 感知延迟文档同步
部署与运维文档 SHALL 将感知检测链路解耦方案、OpenSpec delta 和 source-map 保持同步，使实施者能够从规范追溯到详细排查文档。

#### Scenario: 维护详细延迟文档
- **WHEN** 本次感知延迟方案新增或修改详细中文方案文档
- **THEN** `openspec/source-map.json` MUST 将该文档列入 `感知跟踪` 与 `部署运维` 的来源列表，并且 `scripts/check_openspec.sh` SHALL 通过 source-map 校验

#### Scenario: 硬件不可用
- **WHEN** 本地 CI 或开发机无法执行目标硬件延迟基准
- **THEN** 验证报告 SHALL 明确标记 HIL 未执行，并仍运行配置解析、OpenSpec、单元测试和静态检查

#### Scenario: HIL 延迟结果已收集
- **WHEN** 目标硬件完成 `.engine` 感知延迟基准
- **THEN** 实施记录或详细方案文档 SHALL 回填 detector 耗时、快速循环耗时、`video.detections` 发布间隔、Web 检测框刷新观察、连续观测摄入计数、全链路 stage_duration、runtime running/enabled/auto_start 和 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC`

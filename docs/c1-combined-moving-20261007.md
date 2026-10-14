# 合并候选的移动链复验与生产线程边界

2026-10-07 UTC。只对已接受夜间改动的组合源码运行三个既有exact移动用例，各一次；**1通过、2失败**。没有生产算法改动、放宽阈值或挑选重跑。OFF/ON均先完成原位置/朝向目标和独立运动oracle；held先完成连续性。ON/held随后在原75ms消费节拍断言失败，held另有真实记录的65.927426ms控制调用，超过50ms。

## 精确来源及构建

真实远端`baa93057e005132b6326fdb6338a3ada68289bb0`，tree`c9d7bea863ff94258901dbdaba699412c4963218`。已核对全部1641个文件/模式，并把已发布51文件增量物化为同树本地检查点`914014667eea78f3bfdc1169b47d1e282ede7209`。本地历史仍是映射，不是待推送的真实远端祖先。

复用现有SDK/cache，Rust/Cargo1.88，Cargo jobs2/CMake jobs1。增量编译9.13s通过，没有SDK重建。实际Cargo记录：runtime与MLS opt3，其余相关PCT/Scan/control/mapping/codec/map-chunks为opt0；debug0，debug assertions/overflow checks开启，body-query-profile开启。**不是release**。13项source/lock构建前后相等，22项依赖profile由compiler-artifact核对。

二进制SHA256：`65b3205883b414cfa63f4d6877534063291a68be5e8b401cd82c864f0fa0b86c`。

## 实际结果

单位ms，分位数为原fixture打印结果；没有将不同运行或嵌套阶段合并。

| 用例 | 功能结果 | gap median/P95/max | >75ms | control median/P95/max | map max |
| --- | --- | --- | ---: | --- | ---: |
| OFF | 21.300s到达，位置误差0.062269m | 50.001/50.121/54.101 | 0 | 1.239/2.263/7.511 | 0.027422 |
| ON | 21.656s到达，位置误差0.146755m | 50.001/50.385/130.771 | 2 | 1.230/2.893/13.707 | 0.030671 |
| held | 连续性完成，位移1.068999m，6个held非零命令；非全程到达 | 50.001/50.545/124.016 | 3 | 1.357/5.640/65.927 | 1.368345 |

三者无超时、无delivered negative、无丢帧；消费间隔超过50ms仍为219/222/151次。held有2次新计划提交/采用。位置/姿态、真实边及receipt独立oracle在节拍断言前通过，不能把这叫作实际MotionGuard或物理制动通过。

OFF运行到并通过后续50ms控制/map断言；ON和held在`route_moving_worker_tests.rs:908`的75ms断言先失败，`:910`的50ms断言未执行。ON打印control/map>50ms均0；held打印control>50ms=1、map=0，独立数据已经违反该控制上限。当前OFF单次通过不抹去历史OFF失败，当前ON/held失败也不被历史单次通过覆盖。没有matched前后性能比较。

| 整个子进程 | wall s | user+system CPU s | peak RSS KiB |
| --- | ---: | ---: | ---: |
| OFF | 22.050979 | 1.336830 | 66828 |
| ON | 22.428952 | 1.441264 | 70324 |
| held | 15.350326 | 1.772727 | 77984 |

资源由wait4取得，包含setup、传感器、规划、诊断和后置oracle，不是内层planner CPU/RSS归因。

## 五个节拍异常与一个独立控制慢样本

每个gap均按原记录重建为前一body+bookkeeping+当前sleep window，无剩余项；不能由此把原因统一归为运行环境。

| 场景/tick | gap | prior body | bookkeeping | sleep window | requested | excess | schedstat runtime/runqueue增长 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| ON42 | 96.549789 | .378425 | .016716 | 96.154648 | 49.391880 | 46.762768 | .502561/2.335755 |
| ON50 | 130.770753 | .232270 | .003856 | 130.534627 | 25.184991 | 105.349636 | 59.840414/.003821 |
| held173 | 124.015877 | .047906 | .002463 | 123.965508 | 49.178606 | 74.786902 | .005781/.105310 |
| held175 | 94.163864 | .008854 | .001625 | 94.153385 | 49.945729 | 44.207656 | .060644/42.647985 |
| held177 | 102.603155 | .023208 | .002720 | 102.577227 | 49.579352 | 52.997875 | 24.307217/0 |

schedstat是采样的内核已记账值，有记账延迟，并非精确阶段CPU时钟。只有held175的runqueue增长接近excess；ON50/held177还存在明显runtime增长，不能声称全部是off-CPU等待。三个确切接受的receipt的publication→retrieval为ON42=91.913294ms、ON50=2.056514ms、held173=122.088751ms；held175/177没有已接受非零receipt。ON50证明大gap不等于已发布结果等了同样久。

held的控制慢样本是**step86/source-view22**，不是首次view更新，jobs=1/admitted=1、held=false/phase0，发生在第二个规划hold之前。准确inclusive阶段：controller65.567109ms、candidate ranking sweeps65.430514ms、observed sweep36.250633ms、projection5.333809ms；contact-domain仅0.473553ms、contact-frame0.577884ms。不能归咎于held hook或已接受的contact BFS优化，也不能将嵌套阶段相加当CPU。

step86的详细schedstat对没有输出到保存日志：现有trace只打印与>75ms消费窗口重叠的compute对，而step86不在后面五个窗口中。未新增测量、猜测缺失数据或把一个nonzero计算结果冒充确切执行器回执。

## 生产线程/fixture边界（独立源码核对）

- LCM独立OS owner收发；C回调复制到Bytes，订阅适配再分离payload。各stack owner有独立OS线程和current-thread Tokio runtime（`lcm_dataflow.rs:145–157`）
- surface-planner ingress同步decode/validate/order，再以spawn_blocking执行私有SurfaceMapCache构建；FineMapReader/缺块RPC属于这个构建任务。全局和新图quick搜索各有blocking任务
- surface-nav-scheduler是另一owner；LCM输入与tokio::spawn的NavScheduler共享其单线程runtime。它取计算结果、准备/验证MotionGuard请求并await socket；`tokio::spawn`本身不是另一个OS线程
- `nav-local-compute`在exchange锁外做地图/path安装、当前机体、候选排名、观察/制动sweep与controller；发布短暂重取锁并核对拥有方。源Arc克隆不是整云复制
- ON时`nav-local-planning`独立执行局部PCT/Scan，active/latest-pending/completion有界，计算和退役对象释放在exchange锁外；规划使用共享不可变证据与私有可丢弃缓存
- 本fixture仅有合成5Hz传感器、std::thread::sleep的50ms测试消费线程，以及上述真实local-compute/local-planning；它绕过LCM、SurfaceMapCache/FineMapReader、全局planner、NavScheduler仲裁和真正MotionGuard。因此本次不能合并宣称所有夜间七条路径已经在生产闭环复验

## 后续候选与非结论

独立源码审查发现两项具体后续候选，但本阶段均未实施：

1. `surface_dataflow.rs:885`传quick budget0，已标定机体分支`:450`却丢弃budget并调用完整PCT body_route；新几何可能与旧全局任务并行做第二次完整搜索。既有用例证明新路线能跨过held旧任务，却不证明零展开。修改前必须先界定quick工作合同，保留新图接续和取消
2. `decode_navigation_map_3d_body`已validate，同一不可变Arc进入`SurfaceMapCache::build`又validate；两次都解析manifest/扫描内联block摘要。第一遍占ingress，第二遍占blocking worker。应先量化再考虑内部已验证入口，保留公开/不可信入口和Receiver冲突检查

这两项均不在本移动fixture中，不能解释其失败。既有锁/边界克隆是真实工作，但没有证明有害竞争，不据此增加全局门或陈旧观察缓存。下一优先事项是另外已确认的canonical mapping每帧内容hash，需先测并传递现有producer epoch/sequence；不得把普通HashMap/HashSet容器本身当内容hash删除。

真实LCM→MotionGuard组合仍为已记录的AF_UNIX EPERM阻断；本阶段未重试/换传输。未来获准环境的OFF/ON/过期归零命令仍见`c1-assigned-navigation-composition-20261006.md`，是待执行计划而非本次通过证据。

## 复现与证据

原始目录`/workspace/shared/neh-combined-moving-20261007`保留build.sh/jsonl/log/exit、toolchain-and-flags、13项source检查、22依赖profile、冻结binary identity、三个完整日志、runner、资源结果和提取结果。配套JSON保存精确数值、命令和日志SHA。源码与实际日志由独立审查复核；整理本记录没有额外运行。

构建命令在workspace目录使用locked/offline、`-p nuwax-nav-runtime --features nuwax-nav-runtime/body-query-profile --lib --no-run --message-format=json -vv`，并固定MLS/runtime test opt-level=3。三个精确selector分别为`moving_measured_5cm_primed_off_full_goal`、`moving_measured_5cm_primed_on_full_goal`、`moving_5cm_primed_held_planning_continuity_only`，均加前缀`route_moving_worker_tests::`和参数`--exact --ignored --nocapture --test-threads=1`。

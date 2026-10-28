# Assigned-map 生产导航组合：编译通过，运行受环境阻断

2026-10-06 UTC。**本项没有取得导航组合通过证据。** 最新源码可编译；实际 Motion Guard
在创建 AF_UNIX socket 时被运行环境以 EPERM 拒绝。普通执行与一次获准的提升权限重试均
在同一位置失败。未绕过限制、替换传输或将失败改为跳过；ON 未重试。

## 新增测试的范围

- `src/server.rs` 的 `lifecycle_tests` 引入独立测试文件
  `src/assigned_navigation_composition.rs`，不改变生产接口、依赖或锁文件
- 设计路径：现有 schema-2 原生导出/AR2、DC1 转移 → tonic ActivateMap/Relocalize →
  当前时间的注册传感器帧与规范修正 TF → 真实 LCM/生产 Geometry 消费者就绪 →
  Mls3D StackRunner → 实际 MotionGuardCore/GuardActor/UnixServer → 记录执行器
- 执行器按实际 TWIST/ZERO 事件时间积分。机体为显式通用 0.30m 方形，支撑偏移0.4m；
  初始机体建立地图原点，所以地面Z=-0.4m、目标机体Z=0。没有实机运动或制动资格含义
- OFF/ON 使用相同场景、标定及1s观察TTL。目标距起点1.2m、末朝向0.35rad，位置/朝向
  容差0.06m/0.08rad。ON 的断言要求身份匹配的非空 `local_path`，OFF 要求保持为空
- 另一个移动目标停止云输入、保留里程计，要求真实观察到期诊断与 APPLIED 零速保持；
  保持必须晚于最后实际地图源时间+1s，且取消前没有非零脉冲。随后取消必须取得精确
  ZeroAck。APPLIED/ZeroAck 分别查询生产 GetOperationResult/GetGuardState

以上是**源码接线和断言设计**，不是本次已执行到的行为。

## 实际构建和运行记录

统一为混合 test profile：MLS planner 与 nav-runtime opt-level=3，geometry 测试目标
opt-level=0、debuginfo=0、debug assertions/overflow checks 开启。不是 release 性能测试。
实际链接 liblcm 1.5.2；构建使用 `--locked --offline --bin nuwax-geometry-runtime`。

| 构建 | UTC时间 | 结果 |
|---|---|---|
| 1 | 15:47:41–15:50:41，180s | exit101；两项测试代码错误：模块路径缺少 `crate::`，PoseState 未实现 Debug |
| 2 | 15:54:41–15:58:00，199s | 两项机械修正后 exit0 |
| 3 | 16:10:47–16:11:47，60s | 平地能力修正后 exit0；Cargo报告59.87s |

实际运行均为 local OFF：

1. 第一次在真实 Python 能力解析处失败：`stairs_supported=false` 却声明上下阶0.1m。
   仅将两个阶高收窄为0，保留不支持上下阶；随后实际生产能力解析器验证通过
2. 修正后普通执行失败：`socket.socket(AF_UNIX)` 抛出
   `PermissionError: [Errno 1] Operation not permitted`
3. 一次获准的提升权限执行仍在同一 socket 创建位置得到 EPERM，故停止环境内重试

第三次运行保留文件证明 Python 依赖已导入、持久 boot generation=1、启动零速已确认、
lease为Idle且fenced；执行器日志仅有一个未持租的启动ZERO。socket路径71字节，错误
发生在创建socket而非路径绑定。这些证据不能代替服务端连接、导航或取消执行证据。

**未执行到/未证实：** 目的端生产 Geometry 启动、真实LCM消费、组合内恢复/重定位、
导航目标准入、非零APPLIED、PlannerWorker/Scan实际采用、1m以上积分推进、完整位置+
朝向到达、观察过期保持以及取消精确归零。local ON 没有本次运行结果。不存在实机证据。

## 冻结身份和原始证据

- helper SHA256：`537fa1c783cdc783de16bd3aded9ba9b8607f895a5a0c2b5ca939a0d396b89f1`
- server.rs SHA256：`f2924c19f57f385946ccc04cf1a381128369b1d5ba47667ae185e7da53484223`
- 第3次保留测试二进制 SHA256：`67fe78cd6166506fc456ab34ccee8d4bc0e7e14040563e5a6f49871748300df5`
- 二进制大小：124,308,696 bytes；Cargo测试目标 `nuwax-geometry-runtime`
- 本地原始证据目录：`/workspace/shared/neh-restored-navigation-composition-20261006/`
  包含三次构建日志/退出码/源码核对、`binary-identity.json`、能力解析验证、测试清单及
  `local-off-attempt-{1,2,3}.log/.exit`。失败源码与原始日志保留，不回写成通过

本次没有 commit、push、release 构建或硬件测试。

## 后续获准环境中的精确命令

以下是待执行命令，不是本次通过记录。仅在支持真实 AF_UNIX、loopback TCP、LCM UDP
及所需文件权限的获准环境运行；遇到权限拒绝应报告阻断，不换传输规避。准备已有SDK/
Cargo缓存环境脚本与可导入本仓生产Python服务的解释器，不在测试内安装依赖。

```bash
set -euo pipefail
# 按获准环境填写这三条绝对路径。
export CHECKOUT=/absolute/path/to/nuwax-embodied-harness
export NEH_NATIVE_ENV=/absolute/path/to/native-project-env.sh
export GUARD_PYTHON=/absolute/path/to/existing-test-env/bin/python
source "$NEH_NATIVE_ENV"
export REPO="$CHECKOUT"
export NUWAX_TEST_GUARD_PYTHON="$GUARD_PYTHON"
export PYTHONPATH="$REPO"
export CARGO_PROFILE_TEST_DEBUG=0
cd "$REPO/services/geometry/nuwax_geometry_runtime"

cargo test --locked --offline --bin nuwax-geometry-runtime \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  server::lifecycle_tests::assigned_navigation_composition::assigned_v2_restored_production_navigation_local_off_arrives_and_expiry_stops \
  -- --exact --nocapture --test-threads=1

cargo test --locked --offline --bin nuwax-geometry-runtime \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  server::lifecycle_tests::assigned_navigation_composition::assigned_v2_restored_production_navigation_local_on_arrives_and_expiry_stops \
  -- --exact --nocapture --test-threads=1
```

测试各自在独立子进程中固定ON/OFF配置、隔离LCM地址与文件，并清理继承的导航/执行器
覆盖值。应保存实际二进制摘要、完整日志与终态；任何后续源码修正须重新构建并重新验证。

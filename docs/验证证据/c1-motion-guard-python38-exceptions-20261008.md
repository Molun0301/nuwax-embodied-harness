# MotionGuard Python3.8异常链兼容（2026-10-08）

基线 `a1a656b1bec2f32f364ddc9234731f1654ffa6da`。

## 已复现的问题

`services/navigation/motion_guard_client.py` 在客户端／会话上下文退出的双重失败，以及注册会话失败后清理fence再次失败时，无条件调用异常对象的 `add_note`。该接口为Python3.11引入；实际Python3.8.20三个路径均抛出AttributeError，遮蔽了应传播的MotionGuardError，使调用方原有该异常捕获路径失效。此证据不表示底层fence被跳过，也不构成实机运动问题结论。

## 修复

删除三个附加note调用（共五行），保留已有 `raise close_error from exc` 和 `raise fence_error from registration_error`。原MotionGuardError对象及先前失败均通过明确异常链保留。Python3.11+也不再附加这三条重复note；项目内没有找到其消费者。未增加运行时版本判断、兼容shim、校验、锁、重试或缓存；原停止／注册／清理调用顺序与异常优先级不变。

## 实际验证

新 `scripts/check_motion_guard_exception_compat.py` 使用真实导入的客户端和会话类，mock仅位于连接、通信、清理fence、续租边界，socket.socket设为一旦调用即失败，未启动服务或操作设备。

- Python3.8.20候选11/11通过
- Python3.12.14候选11/11通过
- 完整历史模块在3.8.20下8项通过，恰好三个双重失败用例因add_note产生预期AttributeError，原失败日志保留
- 当前三个用例均验证传播原MotionGuardError对象，并以原body／registration错误作为 `__cause__`
- 其余覆盖正常退出、仅body失败、仅close失败、注册失败但fence成功、成功注册与续租入口
- 现有 `check_python38_runtime_compatibility.py` 在3.8.20下8/8通过；定向PEP604静态检查与diff检查通过

可运行 `python scripts/check_motion_guard_exception_compat.py`。若使用 NEH_MOTION_GUARD_BASELINE 环境变量，脚本会加载执行指定的历史模块文件，必须来自可信Git源。

本阶段只有离线异常行为资格，不是完整MotionGuard服务、真实传输、制动安全或Python3.8.10实机资格。没有性能提速声明。

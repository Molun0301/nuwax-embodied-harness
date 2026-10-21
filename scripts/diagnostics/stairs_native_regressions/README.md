# 楼梯原生回归入口

先按项目锁定环境运行 `bash scripts/diagnostics/start_navigation_simulator.sh --build-only`。
设置 `SCAN_NATIVE_LIBRARY` 指向该次构建生成的 `release/build/nuwax-nav-scan-native-*/out/libnuwax_scan_native.a`，并使用与构建相同的 `EIGEN3_INCLUDE_DIR`；然后运行 `bash scripts/diagnostics/stairs_native_regressions/run.sh`。不得选择旧版本库冒充本次源。输出默认为 `runtime_data/stairs_native_regressions`，可用 `STAIRS_TEST_OUTPUT` 指定工作区目录。

入口验证：实际原GridMap的8925个半开实体正体积交叠查询（对称及非对称高度带、负坐标、float32分辨率、格相位、清占据）；有序路径同XY多层、8字、U形、竖直段、拐角连续与纯函数；真实FSM完整路径进入高度源且ready；任务退休清高度与捕获回调、保留map/optimizer/A*资源，并验证相同XY新层高不串用旧任务。

生命周期测试仅在输出目录生成可见性shim，访问原manager私有资源以做断言，既不修改生产头文件，也不更改对象布局。旧回调捕获由测试人工植入，用于检验退休释放，不声称完整并发竞态认证。

这是组件回归，不替代完整导航、实际碰撞裁判或实录。半开格仅正长度相交算占据；不以epsilon或ceil改floor消除薄障碍。原仿真矩阵仍须独立按180秒预算运行。Rust来源语义及PCT保持断言位于原native_map单元测试。

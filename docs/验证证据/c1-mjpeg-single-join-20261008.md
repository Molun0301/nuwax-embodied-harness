# MJPEG 封装消除一次整帧中间复制（2026-10-08）

基线 `70948202e54c3eb8619474c97a1d692bf26643b1`。

## 改动与所有权

`gateways/ws/data_streams.py::_format_mjpeg_frame` 原返回 `headers + data + b"\r\n"`，两次左结合 bytes 拼接先产生含整份JPEG的中间值，再复制到最终multipart。改为 `b"".join((headers, data, b"\r\n"))`，直接形成最终字节串，移除这份中间整帧复制。仅一行生产改动。

真实调用路径为 `VideoShmReader.read_latest()` → `_SharedMjpegFrameSource` 读取线程 → 缓存不可变multipart → `wait_for_frame()` → `/streams/video.mjpeg` HTTP生成器 → 控制台视频。SHM读取返回的独立bytes拷贝保留：它用于与可变共享内存及映射生命周期脱离，不能当作无用复制删除。

格式化原本已在源锁外运行，本改动不声称减少锁竞争。最终同一不可变multipart继续由多客户端共享；源／摄像头匹配、引用计数、通知、帧身份、节流和最后消费者退出保持。不新增锁、校验、缓存、线程或分块传输。

## 离线验证

实际 CPython 3.8.20：

- 新 `scripts/check_mjpeg_frame_encoding.py` 5项通过，含128组字节等价子场景：空、小、二进制、512KiB负载；无／有序列、序列0与大整数、摄像头百分号编码、时间戳微秒边界
- 字面值头部断言检验 Content-Length、边界、尾部CRLF及摄像头／序列／时间来源
- 真实 VideoShmReader 读取临时合成SHM，确认返回独立bytes在底层修改及reader关闭后仍有效
- 假reader验证两个消费者共享同一最终bytes、后续帧不修改旧帧、释放／重新获取和取消等待清理
- 同一5项测试对完整历史模块运行亦通过，验证输出与生命周期契约一致
- 原 `scripts/check_pointcloud_encoding_reuse.py` 21项通过，含实际router生命周期与fanout检查
- PEP604静态检查通过；其固定兼容性提示不代表实际运行过Python3.8.10

可执行 `python scripts/check_mjpeg_frame_encoding.py` 重现当前定向检查。测试使用合成文件和假reader，无真实摄像头、机器人、模型或网络调用。

## 结论边界

当前bytes生产路径消除一次中间整帧拼接，由源代码分配方式支持；没有测量峰值内存、FPS或视频端到端延迟，不给出数量化性能收益。Python3.8.10及现场视频链路仍未实跑。

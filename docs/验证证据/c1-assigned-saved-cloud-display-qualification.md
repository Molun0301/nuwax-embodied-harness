# 分配式保存点云、dense 与页面显示配对

日期：2026-10-06 UTC。唯一工作分支 `fix/v3-legacy-cloud-seal-20261006`，
真实远端父提交 `61d529f11e848ee39cbad938bb737de365e534b6`，源树
`a3a21ccf8f57de0b73b525ffcd758ca8b28b5e25`；对应本地同树基线
`80e31a03e01144d3cd36008558c86b5a037eb8ab`。发布只接本次增量，不携带本地合成历史。

## 行为与资源边界

- 从当前安装已经准入的 stored checkpoint 捕获不可变显示句柄，保留原 canonical/dense
  载荷与准确安装/准备对象；不再次 Store.load、读 PCD、计算内容摘要或复制整图拥有方
- assigned 保存页使用只读 1,024 点视图，原页和原载荷可复用；协议明确完整 reset、
  source_incremental=false。保存地图不冒充实时生产者增量来源
- 页与 dense 共用已有 asset ID、checkpoint ID、安装 generation；源 epoch 直接由这些
  已有标识投影，不新建摘要身份。dense 的 session/revision 仍是原归档源身份
- 首个 dense 请求也绑定显示来源，浏览器在 decode/append 前核对 typed identity、frame、
  display epoch 与 dense epoch；不可变 assigned 保存来源拒绝 delta
- 相同标量身份下更换实际准备拥有方仍会重新交付当前页面，不做可能扫描整图的载荷相等比较
- 旧 live SHM/配置 PCD 不能覆盖已选择的 assigned 来源；真正属于当前安装会话的 live
  更新仍可使用，普通 live revision 前进不会被误判为换图
- 缓存、HTTP await、WS 每包编码及发布回调边界复核拥有方；旧请求/迟到响应不能清除或填充
  后继显示。清除地图、重连、重复 reset、换样式和隐藏/显示均有针对性检查
- 浏览器 assigned 保存页使用已有分页渲染器，_allPositions 保持为空；显式旧保存渲染保持原行为
- 离线保存地图显示无需 provider 连接或 READY；动作/位姿准入仍使用原有更强原生证据

运行态/provider/cache/source 守卫内不做 assigned 载荷读取；原 SourceSwitch 路由 RLock
串行机制保持不变。本阶段没有声称所有层级都无锁，也没有改变实时障碍或运动准入逻辑。

## 明确限制

- 保留现有 32 MiB canonical 显示预算，超限明确拒绝；不静默抽样或扩大预算
- assigned 保存资产的旧 WS/SSE 全图适配器返回有界 unavailable/pages-required 通知；
  使用 `/streams/pointcloud/ws` 分页入口。schema-1 旧协议兼容不变，不重新拼接 assigned 整图
- dense 按既有有界块交付；传输编码和浏览器 GPU 几何需要的块级复制不等于复制另一个整图源
- 无 Rust/protobuf 修改，无本阶段原生重建或原生测试；实机和原局部规划性能仍待下一阶段验收

## 实际验证

**Python 55/55 通过**：39 个新增 retained-source/gateway/HTTP/WS 用例，16 个直接受影响
传输/发布兼容回归。最终运行分为 38、1、10、6 个用例。

**新增 JavaScript 23/23 通过**，含真实 schema-2 仓储/安装 fixture → Python
PointCloudTransfer → JS assembler → 实际 viewer 模块的 5,000 点检查；另有分页资源 21、
wire 12、voxel identity 27 个检查及既有 viewer、wire、overlay 复用脚本通过。
这些为 CPU/Three 模块检查，所谓 native-wire 脚本此处使用 Python mmap fixture，未运行 Rust。

新增文件 Ruff、修改源码 critical Ruff/编译、Python 3.8 语法、分层硬规则、diff 检查通过。
独立源码审查通过；已发现的同身份拥有方替换、首块 epoch、清除地图、旧来源回退和异步
交付问题均修复并有针对性回归。初次网关测试使用缺少 dense 的旧 fixture 而失败，随后改用
实际含 dense 的准入 fixture；失败日志保留，不计入最终通过。

**浏览器/WebGL 未验证**。新建独立 dot cloud 浏览器标签页请求 `http://127.0.0.1:8898/`
返回 `ERR_BLOCKED_BY_CLIENT`；检查生成的失败页另遇只允许 http/https 协议的策略错误。
独立检查还确认临时测试服务器未监听。未尝试绕过或替代网络路径，未运行渲染断言、无成功
截图，也不把这些现象描述为 WebGL 本身不可用。实际模块 QA harness 与两份准入 wire fixture
保留在验证产物目录。精确用例、命令、源码摘要与日志见配套 results JSON。

下一步回到原局部规划/通行性功能与性能验收；mapping-cloud/place/Nominatim 仍是独立未完成项。

# C1 精细块流式校验验证记录

2026-10-06 UTC。仅移除 NavigationMap3D 校验中解码后立即丢弃的 Block 映射，保留真实读取所需的物化解码。22 项不同的聚焦回归最终通过；其中 C/Python 互操作首轮因现有 LCM 头文件未进入直接调用的 C 编译器搜索路径而失败，补充已安装依赖的进程环境后，同一二进制的精确重试通过。失败和重试均保留。

同一冻结夹具的 7 组成对测量中，校验、编码、解码的操作中位数分别下降 74.0604%、75.6884%、75.3413%。结论限于普通 opt-level=0 测试构建的预热编码组件；不是 release、完整传输、RSS 节省或实时截止期结论。真实四块物化解码存在较慢样本，不能宣称普遍加速。

## 生产改动与边界

- `nuwax-map-chunks/src/lib.rs` 将原 `decode_block` 的块头、计数、精确长度、局部键顺序及证据配对检查提取为私有、静态分发的 `visit_block(bytes, impl FnMut(u16, Evidence)) -> Result<u32, String>`
- `decode_block` 仍只解析一遍，并逐记录插入一个 BTreeMap；没有增加最大时间计算或第二次解析
- 新增 `block_summary(bytes) -> Result<(u32, u64), String>`，完整校验成功后返回体素数与最大观测时间。全零旧证据的最大时间为零。成功路径只维护栈上摘要；非法输入仍分配原有错误 String
- `nuwax-codec/src/lcm/schema.rs` 的 inclusion 循环改用摘要，并在完整块校验成功后检查根的体素数与时间上界；原错误文本保持不变
- 必须先完整检查正文。首条记录超过根时间、根点数也不匹配，而后续记录重复时，仍先返回“精细块重复、无序或无效证据”，不能提前返回根证据错误
- 未新增内容哈希、全局缓存、锁、装箱回调、trait object 或通用 DTO 层；Receiver、RegionReader、FineMapReader 及其既有缓存复用逻辑未改动。此前已接受的复用证据见 [已准入精细块复用记录](c1-admitted-block-reuse-20261006.md)，本轮未把那些测试算入新的 22 项

在本夹具每次成功调用验证器时，旧路径构建并丢弃 4 个 Block BTreeMap，进行 4096 次插入，并检查其值中的时间证据；新路径逐记录校验和累计摘要，移除这 4 个临时映射、4096 次插入和映射值遍历。这是代码与固定输入推导出的操作减少，不是 malloc 调用次数或堆字节测量。根 Manifest 的 JSON 解析、根引用映射、编码缓冲区、解码消息缓冲区等仍分配内存，不能称整个验证器或流水线无分配。

## 构建与源码身份

本次是未提交候选工作树。证据记录的本地基点为 `2bf54cc108a63dfa728a486fefd4de8ce8331620`，基点 tree 为 `1fabe2eac417c4fecde21552148f22b2b93dbe72`；记录的远端提交为 `410290dbc45c20db10bf2782556b555a0175bcc4`。它们不代表本候选已提交或推送。

基线构建为 18:09:44–18:10:18 UTC，候选为 18:24:33–18:24:44 UTC，均退出 0。两边构建前后相关三份源码校验一致。普通 test profile：codec/map-chunks 的 opt-level=0、debug=0、debug_assertions=true、overflow_checks=true，features=[]。命令中已有的 MLS/runtime opt-level=3 配置不涉及被计时的两包；这不是 release 性能结果。

```sh
source /workspace/shared/neh-native-20261006/project-env.sh
cd /workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006/services/nuwax-workspace
cargo test --locked --offline \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  -p nuwax-codec -p nuwax-map-chunks --lib --no-run --message-format=json
```

上面是候选构建；基线同命令只选择 `-p nuwax-codec`。完整运行环境、原命令和退出码见证据目录中两边的 `build.sh`、`build.jsonl`、`build.log`、`build.exit`。编写本记录时没有重跑构建或测试。

源码路径相对 `services/nuwax-workspace`：

| 文件 | 基线 SHA-256 | 候选 SHA-256 |
| --- | --- | --- |
| `crates/nuwax-codec/src/lcm/schema.rs` | `5eaf1916664049c96d8cf3545fea55807948c40099c184f9966656e76ce65cc9` | `e70a4dc7cf467816b75a1f78caab0371951034749c75bc0c8a000ca675465adb` |
| `crates/nuwax-map-chunks/src/lib.rs` | `2e08aaf4cbd10dba820eddabce7d86769e41d8527cf457d5d6dac91708d8cb17` | `074ccc3910fed71e0aeea3c1da56e6db1f2eb613b7b6c24c1dbd8aa2f66b5b64` |
| `crates/nuwax-map-chunks/src/tests.rs` | `b2705f0cf16675e2829f28685174478c000657355314c74786ba4d629c369c54` | `6342318d7a358e5147c16c4f9651702eaca6f3f1eaa0158fb197414d0f3ece91` |

| 测试二进制 | 字节数 | SHA-256 |
| --- | ---: | --- |
| 基线 `nuwax-codec-test` | 4482448 | `aa630ace6439853924c8013adf74980654378c5f5beef974cc7229dcbcb487cc` |
| 候选 `nuwax-codec-test` / `binaries/nuwax_codec` | 4578288 | `bf4b61cfee05d690bb37cccd6e9648a6fc9e6b8bf650395ce3d318217c93b758` |
| 候选 `binaries/nuwax_map_chunks` | 5333112 | `37ea9f297d9718f800f2a6da589b6dc0a261b737388e5995b4a4d567a4da9cbd` |

冻结计时夹具位于 `schema.rs:2920–3019`，该完整区段 SHA-256 为 `7858575c5280731fa3a0fef8b4e481171bd76d41e6fe8c1bc5ef457a5cb1066b`，生产优化前后未修改。独立源码复核通过：去除本次指定新增及替换后，三份文件均还原为基线哈希；原有回归、协议检查和缓存复用代码保持原样。编写本记录时再次核对候选源码及夹具哈希一致。

## 聚焦正确性验证

本轮最终通过 22 项不同的功能回归：map-chunks 8 项，NavigationMap3D 14 项。常规批次中计时用例被显式忽略，随后单独执行；它不计入这 22 项。首轮功能批次合计 21 通过、1 失败、1 忽略，失败用例随后精确重试通过；不能把首轮日志改写为一次全绿，也不代表整个 workspace 测试通过。

| 日志（相对 `Q/candidate`） | 结果 | 范围 |
| --- | --- | --- |
| `frozen-parser-corpus.log` | 3 通过 | 下列三个新增冻结参考测试 |
| `block-roundtrip.log` | 1 通过 | 负坐标、证据和块往返 |
| `archive-malformed.log` | 1 通过 | 两种归档版本的截断、尾随、计数和证据拒绝 |
| `archive-conflict.log` | 1 通过 | 两种归档中同 ID、不同等长正文拒绝 |
| `archive-v2-binding.log` | 1 通过 | AR2 流式往返和精确引用绑定 |
| `archive-legacy-alias.log` | 1 通过 | AR1 原始 JSON 与不透明历史别名保留 |
| `navigation-map-codec.log` | 13 通过、1 失败、1 忽略 | 包含两个新增 codec 守卫；互操作失败原因如下 |
| `interop-retry.log` | 1 通过 | 同一互操作用例的精确重试，不重复累计为新用例 |

五项新增测试：

1. `tests::block_summary_matches_frozen_header_count_and_error_precedence`
2. `tests::block_summary_matches_frozen_record_corpus`
3. `tests::block_summary_matches_frozen_maximum_valid_block`
4. `lcm::schema::navigation_map_3d_tests::fine_block_summary_preserves_bounds_and_full_parse_error_precedence`
5. `lcm::schema::navigation_map_3d_tests::fine_inclusion_budgets_precede_membership_and_body_errors`

参考解码器是优化前函数正文的逐字测试副本，仅函数名改变，不调用共享解析器。每个语料条目都检查新 decode 与冻结参考的完整结果或精确错误字符串相同；摘要与参考成功结果的长度、最大时间相同；成功正文重新编码逐字节相同。覆盖 0–15 字节短头、错误 magic、四个保留字节分别非零、超最大长度、零计数、1/32768/32769/u32::MAX 计数、缺失记录、1–11 字节尾随残片、最大 32768 点合法块；还覆盖升序、重复、降序、合法前缀后非法记录，局部键 0/32767/32768/u16::MAX，零/非零证据配对及 u64::MAX/u16::MAX。复合错误检查保持头、计数、记录的原优先级。

新增 codec 测试对 Assigned 与 Legacy 身份均检查：根时间上界相等接受、超出拒绝、根点数不匹配拒绝、全零旧证据接受；“错误点数 + 首条未来证据 + 后续重复键”仍首先报正文错误。域、修订、正的替代分辨率、源时间或空成员列表与包头不一致时，先报根绑定错误；根外 inclusion 和块数/总字节预算也先于正文错误。根绑定、点数/时间、正文与根外 inclusion 用例核对 validator 的 String、编码的 LcmEncode，以及完整 wire/纯消息体解码的 LcmDecode 错误变体和文本；预算优先级用例核对 validator 与编码的精确错误。

已有通过测试继续覆盖 schema/fingerprint 版本、隐藏数据、重复 inclusion、未知/混合/零/截断对象身份、占据图与点云 receipt、云头、虚拟障碍、无效布局和非有限坐标；缺失 inclusion 仍允许。成功消息往返与重编码相等。新鲜生成的 C/Python 与 Rust 及已提交绑定互操作在下述环境修正后通过。

### 保留互操作首轮失败及精确重试

首轮 `navigation-map-codec.log` 中，`fresh_generated_c_and_python_interoperate_with_rust_and_committed_bindings` 调用 C 编译器失败，退出 101，明确错误是：`fatal error: lcm/lcm_coretypes.h: No such file or directory`。测试直接调用 `cc`，当时其搜索路径未包含已经安装的 LCM 头文件。

重试使用原候选二进制、同一个精确用例和已有安装路径，仅补充该进程的编译器环境；没有修改源代码、SDK、权限或依赖安装。实际重试脚本为 `Q/candidate/interop-retry.sh`，关键命令如下：

```sh
source /workspace/shared/neh-native-20261006/project-env.sh
export REPO=/workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006
Q=/workspace/shared/neh-codec-validation-20261006/candidate
export C_INCLUDE_PATH="$(pkg-config --variable=includedir lcm)${C_INCLUDE_PATH:+:$C_INCLUDE_PATH}"
export LIBRARY_PATH="$(pkg-config --variable=libdir lcm)${LIBRARY_PATH:+:$LIBRARY_PATH}"
test -r "$(pkg-config --variable=includedir lcm)/lcm/lcm_coretypes.h"
"$Q/binaries/nuwax_codec" \
  lcm::schema::navigation_map_3d_tests::fresh_generated_c_and_python_interoperate_with_rust_and_committed_bindings \
  --exact --nocapture --test-threads=1
```

`interop-compiler-environment.txt` 记录的实际值：

```text
C_INCLUDE_PATH=/workspace/scratch/71cdb9d17732/nuwax-embodied-harness/.nuwax-test-env/lib/python3.12/site-packages/include
LIBRARY_PATH=/workspace/scratch/71cdb9d17732/nuwax-embodied-harness/.nuwax-test-env/lib/python3.12/site-packages/lib:/workspace/shared/neh-native-20261006/sdk/lib:/workspace/shared/neh-native-20261006/sdk/libjpeg-package/opt/libjpeg-turbo/lib64
```

精确重试退出 0，1 通过、0 失败、0 忽略，测试日志耗时 3.53 s。原始 `candidate-results.json` 仍保留首轮失败；最终 22 项结论需要同时读取它与 `interop-retry.log`、`interop-retry.exit`。

## 冻结输入、线格式与计时方法

- 有效的完整 NavigationMap3D 包头与 receipt；4096 个体素，4 个不同 assigned-ID 块，每块 1024 个升序局部键，4 块全部内联
- 内联正文恰为 49216 字节，完整 fingerprinted wire 为 50574 字节；指纹保持 `0x131a5884d2d68d28`
- 基线和候选保存的 `wire.bin` 直接比较完全相同，两者 SHA-256 均为 `14177aa0a919f17f2c29d3aeccf2c7a23ce96c87eae1a972c8aabe90d04d0a2e`
- 每个阶段紧邻计时前预热 8 次，随后执行 128 次，输入/结果使用 black_box，拥有型返回值销毁包含在计时内。夹具准备、断言与可选 wire 文件输出在阶段计时外
- 四个独立阶段依次为验证消息、编码完整 wire、从预先编码的 wire 解码消息、直接物化解码全部 4 个块。直接解码每“操作”是 4 块，不是单块
- 7 组成对比较均使用已冻结并核对身份的基线/候选二进制；每次全新进程，奇数对先基线、偶数对先候选。14 次全部退出 0，输入签名完全一致，没有丢弃慢样本或重采样

执行选择器为 `lcm::schema::navigation_map_3d_tests::warmed_fine_map_codec_baseline --exact --ignored --nocapture --test-threads=1`。`compare.py` 从原始阶段纳秒数除以 128 和 1000000 得到 ms/操作；固定 wire 逐字节比较及二进制哈希核对发生在这 14 次测量之前。

## 7 组成对结果

下表使用各组 7 次操作值的中位数。变化定义为 `(候选中位数 / 基线中位数 - 1) × 100%`，百分比由 JSON 未舍入值计算，显示四位小数。“配对比中位数”先对每一对求候选/基线，再取 7 个比值的中位数，与两个分组中位数之比不同。

| 阶段 | 基线中位数 ms | 候选中位数 ms | 中位数变化 | 候选更快 | 配对比中位数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| validate | 4.726116 | 1.225938 | -74.0604% | 6/7 | 0.230609 |
| encode | 5.040879 | 1.225517 | -75.6884% | 7/7 | 0.234986 |
| decode | 4.668106 | 1.151096 | -75.3413% | 7/7 | 0.229060 |
| 直接物化四块 | 5.631524 | 4.272487 | -24.1327% | 5/7 | 0.932925 |

| 配对 | 执行顺序 | validate 基线→候选 ms | encode 基线→候选 ms | decode 基线→候选 ms | 四块物化基线→候选 ms |
| ---: | --- | --- | --- | --- | --- |
| 1 | 基线、候选 | 3.942332→4.420294 | 4.075601→1.780412 | 4.668106→2.113094 | 8.588927→8.962154 |
| 2 | 候选、基线 | 4.055430→0.935218 | 4.755219→1.124746 | 4.594799→1.295056 | 4.476261→4.272487 |
| 3 | 基线、候选 | 4.723544→2.361998 | 7.087575→1.280616 | 5.311604→1.151096 | 9.208855→4.398203 |
| 4 | 候选、基线 | 5.179996→1.070253 | 5.040879→0.893389 | 4.285529→0.981643 | 7.494723→3.724221 |
| 5 | 基线、候选 | 5.470646→1.225938 | 5.215278→1.225517 | 6.393335→1.188618 | 5.631524→3.950020 |
| 6 | 候选、基线 | 5.572946→1.390273 | 7.987181→1.389964 | 5.999537→1.046113 | 4.047724→5.661558 |
| 7 | 基线、候选 | 4.726116→0.886400 | 4.070991→0.980527 | 3.890422→0.945267 | 4.051663→3.779897 |

validate 的第 1 对候选慢 12.1239%。四块物化的第 1、6 对分别慢 4.3454%、39.8702%；其配对比中位数为 0.9329247912，仅对应配对变化中位数 -6.7075%，不能用分组中位数的 -24.1327% 宣称真实解码普遍加速。四块物化范围为基线 4.047724–9.208855 ms、候选 3.724221–8.962154 ms。当前结果未显示一致的物化退化，但小样本波动明显，不能排除其他负载或构建配置的退化。

### 首次单次试跑同样保留

在 7 对比较之前，两边各有一次独立初始试跑，原始日志为 `baseline/codec-baseline.log` 与 `candidate/codec-candidate.log`。它们没有被加入上述 7 对统计，也没有被替换或删除。

| 阶段 | 首次基线 ms/操作 | 首次候选 ms/操作 | 变化 |
| --- | ---: | ---: | ---: |
| validate | 4.168549 | 1.100916 | -73.5899% |
| encode | 4.671672 | 1.017056 | -78.2293% |
| decode | 4.089379 | 1.034304 | -74.7076% |
| 直接物化四块 | 3.565395 | 4.853812 | +36.1367% |

首次真实物化确实变慢；连同成对比较中的慢样本一并保留，而不是仅选有利结果。源码只证明物化路径保持一次解析/一次插入，不足以单独推出耗时相同。

## 解释限制与后续边界

- 本结论只覆盖该完整包头、全内联四块输入的预热组件操作；不是全地图规模、所有块分布或冷启动测量
- 未测量分配器调用次数、堆分配字节、峰值堆或释放行为；原始结果中的进程资源包含测试框架与夹具准备，不作为摘要的 RSS 节省证据
- 未运行完整 workspace 原生套件、release 基准、LCM/UDS 传输、真实进程缺块恢复、传感器、执行器或实机 Motion Guard 全链路；本轮没有创建 socket 或尝试绕过环境限制
- 不据此改动新鲜度、根绑定、碰撞/制动或运动权限边界，不将协议往返合格写成实时或部署资格
- 本文对应本地未提交结果，本次未创建分支、提交、推送或修改冻结源码

## 原始证据位置

`Q=/workspace/shared/neh-codec-validation-20261006`。下列文件位于当前验证环境，不是仓库克隆自带附件：

- `baseline/`、`candidate/` 的 `build.sh`、`build.jsonl`、`build.log`、`build.exit`、起止时间和 `source-before-build.sha256`、`source-after-build.check`：构建过程与源码冻结
- 两边 `binary-identity.json`，以及 `candidate/source-binaries.json`：真实二进制、profile 和源码身份
- `candidate/candidate-results.json` 与上述 8 份功能测试日志：首轮聚焦运行及原失败、重试
- `candidate/interop-retry.sh`、`interop-compiler-environment.txt`、`interop-retry.exit`：现有 LCM 路径的精确重试环境和退出码
- 两边 `wire.bin`：固定完整线格式；两边 `codec-*.log`、`baseline/baseline-result.json`、`candidate/candidate-benchmark-result.json`：初始试跑
- `compare.py`、`matched/results.json`、`matched/01-baseline.log` 至 `matched/07-candidate.log`：全部 14 次成对原始测量、输入签名、退出码及汇总

| 证据 | SHA-256 |
| --- | --- |
| `matched/results.json` | `03c7f4310af3d6c961a5c785d8bcf0d1316d42c57eac064291ee5baabb3c5b75` |
| `candidate/candidate-results.json` | `1b3ff8c8936c59d105ab05942e2254f8fb3ea9b261d5bb8152440324eabacbf6` |
| `candidate/interop-retry.log` | `e3a93ff20fb42493ee25293aedf377237d7d1bc42b29ee75e52300a426e899e0` |
| `candidate/interop-compiler-environment.txt` | `ce8c44f8c885d2ff8a5175bc3d3b71f159b4b2e2c4830afcb11c2b884bb230c0` |
| `candidate/source-binaries.json` | `8f95429e7922c8b5af9fcef317ff6970496647dfe4b39563e5b54d7f7c3bcf55` |
| 两边 `wire.bin` | `14177aa0a919f17f2c29d3aeccf2c7a23ce96c87eae1a972c8aabe90d04d0a2e` |

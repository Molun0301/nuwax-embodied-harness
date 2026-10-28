# 重定位请求编号

适用于 `relocalize_active_map`（MCP/能力入口）和 `POST /api/localization/relocalize`，以及调用 `MapWorkspaceService.relocalize_active_map(metadata=...)` 的内部编排。

新请求使用 `business_request_id`，例如 `{"map_name":"floor-1","business_request_id":"order-000123"}`。原值直接保存在目录 intent、终态、原生请求及原生持久化账本的 `request_id` 中；不做哈希、前缀编码、裁剪或大小写转换，也不建立业务编号到另一操作编号的映射。显式编号若没有既有 pending 且不是当前成功回执，先以完整请求查询原生终态；只有明确 `not_found` 才允许新建 intent 和 dispatch。历史终态只读返回，`pending`、身份冲突、未知状态或查询异常不改目录和 session。相同编号表示同一次操作，地图安装拥有方、算法策略、逻辑超时和二维初始位置等参数必须一致。更换参数需要明确的新业务编号。

编号是 1–128 个 ASCII 字母、数字、`.`、`_`、`:`、`-` 组成的字符串。数字编号也要以 JSON 字符串传入，前导零保留。不支持的字符或长度报 `invalid_params`，不会暗中转换。两字段互斥。

兼容恢复使用 `request_id`，例如 `{"map_name":"floor-1","request_id":"relocalize:此前返回的内部编号"}`。此入口只恢复已有 pending intent 或当前成功回执；其他编号仅查询原生历史终态，不提交新的 Relocalize 调用。查询到历史成功、拒绝或取消，一律以业务错误 `relocalization_historical_terminal` 明确拒绝重新采用，`details.terminal` 携带原终态；网关异步调用通过任务错误返回此结果。它不表示重新采用成功，不创建 intent、不改写当前成功回执或定位会话，也不把历史回执与当前位姿配对。既不重新计算旧哈希，也不把未识别的旧原始业务编号当成新请求：未找到返回 `relocalization_request_not_found`，冲突返回 `relocalization_request_conflict`，仍未确定则保持恢复待定。只有 prepared 的旧请求不跨越 dispatch。旧版本没有保存原始业务编号与内部编号的对应关系，因此调用方应使用此前任务输入或回执中的内部编号。

省略两个字段时，网关为本次受理分配一次 UUID（保留 `relocalize:` 前缀），并在 task 的 `input_payload.business_request_id` 返回；该编号沿用同一准入查询，初次受理多一次只读 RPC，不增加隐藏的身份标志。JSON-RPC 的传输编号不作为业务身份。重试应带回已返回的编号；再次省略会表示一次新受理。内部入口无编号时沿用已有准备/恢复流程。

已 dispatch 但回复未知时保留原 durable intent，重复调用与重启后只查询同一原请求，不换编号；not_found/pending/conflict 不释放未知拥有方。已有原生精确请求账本继续负责重复准入、参数冲突和重启后的终态复用。本变更不更改 native 协议、终态格式、global stop 恢复凭据或机器人避障。

逻辑超时属于请求身份，外层 `timeout_sec` 仅限制异步任务等待时间。旧请求若目录已无原请求，查询只能采用当前安装及当前策略/请求参数；若其历史拥有方或参数不匹配会明确冲突，绝不重新执行。

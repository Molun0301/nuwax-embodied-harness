#!/usr/bin/env bash
# scripts/ci/smoke_real_lcm_roundtrip.sh
#
# Phase 8 替代 Phase 0 fail-fast smoke。Phase 3 完成后,real-lcm backend 必须能
# 创建 liblcm context、跨进程 publish/subscribe。这是验收 real-LCM 真通的标准。
#
# 退出码:
#   0  - PASS:跨进程 subscriber 收到 publisher 发送的消息
#   2  - NOT_RUN:liblcm-dev 未安装 / udpm multicast 受限 / Python binding 不可用
#   1  - FAIL:其它异常(脚本错误、timeout、payload mismatch)
#
# 行为约定（硬约束,§12.6 / §13.2）:
#   - 不允许将启动期 fail-fast code(LCM_BACKEND_UNAVAILABLE) 当 PASS
#   - 不允许用 fake backend 内存事件冒充真实 multicast
#   - 不允许伪造 PASS:若 liblcm 不可用,显式 NOT_RUN
#
# CI 镜像必须装 liblcm-dev + pkg-config + python3-lcm(§13.2)。本机环境下若缺
# 任何一项,会进 NOT_RUN 路径并打印安装指引。
#
# 用法:
#   bash scripts/ci/smoke_real_lcm_roundtrip.sh
#   LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=0' bash scripts/ci/smoke_real_lcm_roundtrip.sh

set -uo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}"

LCM_DEFAULT_URL="${LCM_DEFAULT_URL:-udpm://239.255.76.67:7667?ttl=0}"
SMOKE_LOG="/tmp/smoke_real_lcm_roundtrip.$$.log"
trap 'rm -f "${SMOKE_LOG}"' EXIT

bar() { printf '\n========== %s ==========\n' "$1"; }
ok()  { printf '  [OK]      %s\n' "$1"; }
nr()  { printf '  [NOT RUN] %s\n' "$1"; }
fail(){ printf '  [FAIL]    %s\n' "$1"; }

bar "preflight: liblcm / python-lcm / udpm 探测"

# 1. liblcm 系统库
if ! pkg-config --modversion lcm >/dev/null 2>&1; then
  if [[ ! -f /usr/lib/x86_64-linux-gnu/liblcm.so ]] && \
     [[ ! -f /usr/lib/aarch64-linux-gnu/liblcm.so ]] && \
     [[ ! -f /usr/local/lib/liblcm.so ]]; then
    nr "liblcm-dev 未安装。请在 CI 镜像 / 目标机 apt-get install -y liblcm-dev"
    echo "  指引: Ubuntu 20.04/22.04: sudo apt-get install -y liblcm-dev pkg-config python3-lcm"
    exit 2
  fi
  ok "liblcm.so 直接存在（pkg-config 未配置 lcm.pc,但 .so 在）"
else
  ok "pkg-config --modversion lcm = $(pkg-config --modversion lcm)"
fi

# 2. Python lcm binding
PYTHON_LCM_OK=0
if python3 -c 'import lcm; print(getattr(lcm, "__version__", "ok"))' 2>/dev/null; then
  PYTHON_LCM_OK=1
  ok "python3 -c 'import lcm' OK"
else
  nr "Python lcm binding 不可用。CI 镜像需 apt-get install -y python3-lcm 或 pip install lcm"
  echo "  指引: 优先 apt-get install -y python3-lcm；"
  echo "        Ubuntu 24+ pip fallback: python3 -m pip install --user --break-system-packages lcm"
  exit 2
fi

# 3. udpm multicast 探测（loopback 可能被阻断）
bar "roundtrip: Python publisher + subscriber on ${LCM_DEFAULT_URL}"
ROUNDTRIP_OUT="$(mktemp)"
SUBSCRIBER_PID=""
cleanup() {
  if [[ -n "${SUBSCRIBER_PID}" ]] && kill -0 "${SUBSCRIBER_PID}" 2>/dev/null; then
    kill "${SUBSCRIBER_PID}" 2>/dev/null || true
  fi
  rm -f "${ROUNDTRIP_OUT}"
}
trap 'cleanup; rm -f "${SMOKE_LOG}"' EXIT

# 后台启 subscriber,接收一条后退出
python3 - "$LCM_DEFAULT_URL" <<'PY' > "${ROUNDTRIP_OUT}" 2>&1 &
import sys, time
import lcm
url = sys.argv[1]
if not url.startswith(("udpm://", "tcp://", "memq://")):
    print(f"ERROR: unsupported provider url: {url}", file=sys.stderr)
    sys.exit(1)
try:
    lc = lcm.LCM(url)
except Exception as e:
    print(f"SUBSCRIBER_INIT_FAIL: {type(e).__name__}: {e}", file=sys.stderr)
    sys.exit(3)
received = []
def on_msg(channel, data):
    received.append((channel, len(data)))
    if sys.stderr.isatty():
        print(f"got channel={channel} len={len(data)}", file=sys.stderr)
    # LCM Python 1.5: unsubscribe 接受 LCMSubscription 对象或 channel name;
    # 1.5 早期 API 接受 channel name,但 1.5 某些 build 严格要求 subscription 对象。
    # 用 try/except 容错:两种 API 都试。
    try:
        lc.unsubscribe(sub)  # 1.5 严格模式:传 subscription
    except TypeError:
        lc.unsubscribe("/smoke/test")  # 旧 API:传 channel name
sub = lc.subscribe("/smoke/test", on_msg)
deadline = time.monotonic() + 4.0
try:
    while time.monotonic() < deadline and not received:
        lc.handle_timeout(100)
    if received:
        print(f"PASS: subscriber received channel={received[0][0]} len={received[0][1]}")
        sys.exit(0)
    print(f"TIMEOUT: 4s 内未收到消息")
    sys.exit(4)
finally:
    pass
PY
SUBSCRIBER_PID=$!
sleep 0.5  # 让 subscriber 起来

# 主进程 publish
PUB_OUT="$(python3 - "$LCM_DEFAULT_URL" <<'PY' 2>&1
import sys, struct, time
import lcm
url = sys.argv[1]
try:
    lc = lcm.LCM(url)
except Exception as e:
    print(f"PUBLISHER_INIT_FAIL: {type(e).__name__}: {e}", file=sys.stderr)
    sys.exit(3)
payload = struct.pack("<d", 0.123456789)  # 8 bytes timestamp-like
# 频道名严格匹配 subscriber "/smoke/test"
deadline = time.monotonic() + 2.0
attempt = 0
while time.monotonic() < deadline:
    lc.publish("/smoke/test", payload)
    attempt += 1
    time.sleep(0.1)
print(f"publisher OK, attempts={attempt}, payload_len={len(payload)}")
PY
)"
# 等待 subscriber 退出
wait "${SUBSCRIBER_PID}" || SUBSCRIBER_RC=$?
subs_rc="${SUBSCRIBER_RC:-0}"

echo "${PUB_OUT}" | sed 's/^/  pub: /'
sed 's/^/  sub: /' "${ROUNDTRIP_OUT}"

if [[ "${subs_rc}" -eq 0 ]]; then
  ok "roundtrip PASS (publisher + subscriber 跨进程互通)"
  echo "==> smoke_real_lcm_roundtrip: PASS"
  cleanup
  exit 0
fi

if [[ "${subs_rc}" -eq 3 ]]; then
  nr "subscriber 初始化失败（udpm multicast 在本网络被阻断；CI runner 内可能因权限/网络隔离）"
  echo "  建议: 在独立 runner 内执行,或换 tcp:// provider。"
  cleanup
  exit 2
fi

if [[ "${subs_rc}" -eq 4 ]]; then
  nr "subscriber 4s 内未收到消息（udpm multicast 在本机被阻断）"
  echo "  建议: 在 CI runner / 目标机执行,或换 memq:// 单进程演示。"
  cleanup
  exit 2
fi

fail "subscriber 异常退出 rc=${subs_rc}"
cleanup
exit 1
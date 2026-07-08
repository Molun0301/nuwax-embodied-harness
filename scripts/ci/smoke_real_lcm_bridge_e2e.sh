#!/usr/bin/env bash
# scripts/ci/smoke_real_lcm_bridge_e2e.sh
#
# 2026-07-08 v5 实施:端到端 real-lcm bridge 跨进程 + 独立 observer + nuwax bridge 业务链
# 吸收 R3 M-4 + R4 C-1' + R4 P3 退步 + R5 M-R5-3 + R8-M-1 + R11 N-R11-1 + R12 N-R12-1
# 退出码:0=PASS / 2=NOT_RUN / 1=FAIL

set -uo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}"

bar() { printf '\n========== %s ==========\n' "$1"; }
ok()  { printf '  [OK] %s\n' "$1"; }
nr()  { printf '  [NOT RUN] %s\n' "$1"; }
fail(){ printf '  [FAIL] %s\n' "$1"; }

# 0. V-0 前置
bar "V-0 前置"
bash scripts/ci/V0_preflight.sh 2>&1 | tail -3 || { nr "V-0 preflight FAIL"; exit 2; }

# 1. test_lcm_url() 端口公式
LCM_URL=$(python3 -c "import sys; sys.path.insert(0, '$REPO_ROOT'); from services.navigation.port_utils import test_lcm_url; print(test_lcm_url())")
echo "==> LCM URL: ${LCM_URL}"

# 2. process A: publisher(用 _RealLcmBackend,32B typed Twist payload)
bar "process A:publisher(用 _RealLcmBackend)"
PUB_OUT=$(mktemp)
python3 -c "
import sys, time
sys.path.insert(0, '$REPO_ROOT')
from services.navigation.lcm_motion_command_bridge import _RealLcmBackend
import lcm
backend = _RealLcmBackend('${LCM_URL}', lcm)
# 32B typed Twist payload(linear=0.3, ang=0; stamp=time.time_ns())
import struct
payload = struct.pack('<6fQ', 0.3, 0, 0, 0, 0, 0, int(time.time_ns()))
backend.publish('/cmd_vel_test', payload)
print(f'PUBLISHED: len={len(payload)} hex={payload.hex()}', flush=True)
time.sleep(0.5)
" > "${PUB_OUT}" 2>&1 &
PUB_PID=$!

# 3. process B: 第三方独立 observer(lcm-logger)
LOG_OUT=$(mktemp)
timeout 4 lcm-logger -c /cmd_vel_test -o "${LOG_OUT}" 2>/dev/null &
LOG_PID=$!

# 4. process D: nuwax bridge subscriber(R4 P3 退步修复)
D_OUT=$(mktemp)
timeout 4 python3 scripts/inline/nuwax_bridge_subscriber.py "${LCM_URL}" /cmd_vel_test > "${D_OUT}" 2>&1 &
D_PID=$!

sleep 0.5

# 5. 等全部退出
wait $LOG_PID 2>/dev/null || true
wait $D_PID 2>/dev/null || D_RC=$?
wait $PUB_PID 2>/dev/null || true
D_RC=${D_RC:-0}

# 6. verifier 多维断言
bar "verifier 多维断言"

# 6.1 process A publisher 真发
[[ -s "${PUB_OUT}" ]] || { fail "process A publisher 未发帧"; exit 1; }
ok "process A publisher 真发(hex 已记录)"

# 6.2 process B lcm-logger 收到帧
[[ -s "${LOG_OUT}" ]] || { fail "process B lcm-logger 未收到帧"; exit 1; }
FRAME_SIZE=$(stat -c %s "${LOG_OUT}")
[[ $FRAME_SIZE -ge 48 ]] || { fail "lcm-logger 帧过小 ($FRAME_SIZE < 48)"; exit 1; }
ok "process B lcm-logger 真收(frame_size=$FRAME_SIZE)"

# 6.3 R4 C-1' 强化:用 lcm.EventLog 解析 payload(N-R11-1 修:属性访问非 tuple)
PAYLOAD_HEX_FROM_PUB=$(grep 'PUBLISHED:' "${PUB_OUT}" | head -1 | sed 's/.*hex=\([0-9a-f]*\).*/\1/')
if python3 -c "
import lcm
log = lcm.EventLog('${LOG_OUT}', 'r')
ev = log.read_next_event()
if ev is None:
    print('READ_NONE', flush=True); raise SystemExit(1)
# N-R11-1 修:用属性访问,不是 tuple unpacking
print(f'CHANNEL={ev.channel}', flush=True)
print(f'DATA_LEN={len(ev.data)}', flush=True)
print(f'DATA_HEX={ev.data.hex()}', flush=True)
if ev.channel == '/cmd_vel_test' and len(ev.data) == 32 and ev.data.hex() == '${PAYLOAD_HEX_FROM_PUB}':
    print('HEX_MATCH=OK', flush=True)
else:
    print('HEX_MISMATCH', flush=True); raise SystemExit(1)
log.close()
" > /tmp/v0_hex.txt 2>&1; then
    ok "R4 C-1' hex 字节级匹配 PASS"
else
    # fallback:v5 §1.4 决策(EventLog 不可用时)
    PAYLOAD_HEX_FROM_LCM=$(xxd -p -l 48 "${LOG_OUT}" | head -1 | sed 's/^.\{16\}//' | head -c 64)
    if [[ -n "${PAYLOAD_HEX_FROM_PUB}" && "${PAYLOAD_HEX_FROM_LCM}" == "${PAYLOAD_HEX_FROM_PUB}" ]]; then
        ok "R4 C-1' hex 字节级匹配 PASS(xxd fallback)"
    else
        fail "R4 C-1' hex 字节级匹配 FAIL(EventLog parse 失败 + xxd fallback 不匹配)"
        exit 1
    fi
fi

# 6.4 R4 P3 退步修复:process D nuwax bridge subscriber 真收帧
if [[ $D_RC -eq 0 ]]; then
    ok "process D nuwax bridge subscriber 收到帧"
    grep -q "RECEIVED:" "${D_OUT}" || { fail "process D 输出缺 RECEIVED 标记"; exit 1; }
    TS_DEV=$(grep -oP 'ts_deviation_ns=\K[0-9]+' "${D_OUT}" | head -1)
    if [[ -n "${TS_DEV}" ]] && [[ ${TS_DEV} -lt 1000000000 ]]; then
        ok "process D timestamp 偏差 < 1s (${TS_DEV} ns)"
    else
        fail "process D timestamp 偏差 ≥ 1s (${TS_DEV} ns)"; exit 1
    fi
elif [[ $D_RC -eq 4 ]]; then
    fail "process D nuwax bridge subscriber 4s 内未收到帧"
    exit 1
else
    fail "process D nuwax bridge subscriber rc=$D_RC"
    cat "${D_OUT}" >&2
    exit 1
fi

# 6.5 R4 N-1 修:多进程端口验
LCM_URL_2=$(python3 -c "import sys; sys.path.insert(0, '$REPO_ROOT'); from services.navigation.port_utils import test_lcm_url; print(test_lcm_url())")
[[ "${LCM_URL}" != "${LCM_URL_2}" ]] || { fail "多进程端口相同(R4 N-1 修复失败)"; exit 1; }
ok "多进程端口不同 PASS(R4 N-1 修有效)"

ok "端到端 real-lcm bridge 跨进程 + 独立 observer + nuwax bridge 业务链 PASS"
rm -f "${PUB_OUT}" "${LOG_OUT}" "${D_OUT}"
exit 0
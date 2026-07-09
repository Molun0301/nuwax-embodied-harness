#!/usr/bin/env bash
# scripts/ci/V0_preflight.sh
#
# 2026-07-08 v5 实施:V-0 实机 API 探针前置(实施前必做,失败终止 V-3~V-18)
# 吸收 R5 C-R5-1 + R7 N-R7-3 + R11 N-R11-2 + R15
# 退出码:0=PASS / 1=FAIL / 2=NOT_RUN(liblcm-dev 未装)

set -uo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}"

bar() { printf '\n========== %s ==========\n' "$1"; }
ok()  { printf '  [OK] %s\n' "$1"; }
nr()  { printf '  [NOT RUN] %s\n' "$1"; }
fail(){ printf '  [FAIL] %s\n' "$1"; }

# 0. 装包检查
bar "V-0 探针 0:liblcm / python-lcm / 端口"
pkg-config --modversion lcm >/dev/null 2>&1 || { nr "liblcm-dev 未装"; exit 2; }
ok "pkg-config --modversion lcm = $(pkg-config --modversion lcm)"
python3 -c 'import lcm; print(getattr(lcm, "__version__", "ok"))' 2>/dev/null || { nr "python3-lcm 未装"; exit 2; }
ok "python3 -c 'import lcm' OK"

# V-0a: API 签名
bar "V-0a:lcm-python API 签名完整性"
python3 -c "
import lcm
required = ['LCM', 'EventLog', 'LCMSubscription']
missing = [r for r in required if not hasattr(lcm, r)]
if missing:
    print(f'MISSING: {missing}', flush=True)
    import sys; sys.exit(1)
print('ALL_PRESENT', flush=True)
" || { fail "V-0a FAIL:lcm-python 缺关键 API"; exit 1; }
ok "V-0a:lcm.LCM / lcm.EventLog / lcm.LCMSubscription 全存在"

# V-0b: subscribe callback signature
bar "V-0b:subscribe callback 形态(1.3.1 双参 vs 1.5 单参)"
python3 -c "
import lcm, time, sys
# 用 lcm.LCM 收 + lcm.EventLog 'w' 写 1 帧(避免 lcm-logger 异步起进程被 GC 释放)
import os
if os.path.exists('/tmp/v0b_test.log'): os.remove('/tmp/v0b_test.log')
log = lcm.EventLog('/tmp/v0b_test.log', 'w')
received = []
def cb(*args):
    received.append(args)
    print(f'CB_ARGS={len(args)}', flush=True)
lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
sub = lc.subscribe('/test_v0b', cb)
time.sleep(0.2)
lc.publish('/test_v0b', b'x' * 32)
for _ in range(5):
    lc.handle_timeout(100)
# 把收到的帧写到 log(EventLog 'w' 创建新文件可正常)
if received:
    log.write_event(int(time.time_ns()), '/test_v0b', received[0][1] if len(received[0]) > 1 else b'')
print(f'CALLBACK_INVOKED={len(received) > 0}', flush=True)
log.close()
lc.unsubscribe(sub)
" > /tmp/v0b_out.txt 2>&1
RECEIVED=$(grep -c 'CB_ARGS=' /tmp/v0b_out.txt 2>/dev/null | head -1)
RECEIVED=${RECEIVED:-0}
if [[ ${RECEIVED} -ge 1 ]]; then
    ARGS_LEN=$(grep 'CB_ARGS=' /tmp/v0b_out.txt | head -1 | sed 's/.*CB_ARGS=\([0-9]*\).*/\1/')
    ok "V-0b:subscribe callback 收到帧,args 长度=${ARGS_LEN}(1.3.1=2,1.5=1)"
else
    fail "V-0b FAIL:subscribe callback 0 帧"
    exit 1
fi

# V-0c: handle_timeout 阻塞模型
bar "V-0c:handle_timeout 阻塞模型实测"
python3 -c "
import lcm, time
lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
t0 = time.monotonic_ns()
lc.handle_timeout(50)
elapsed_ms = (time.monotonic_ns() - t0) / 1e6
# lcm handle_timeout 是"至少 N ms"语义(> N),允许 ~2x 抖动;Python GIL 定时器精度 ±10ms
assert 40 <= elapsed_ms <= 150, f'elapsed_ms={elapsed_ms} ∉ [40, 150]'
print(f'ELAPSED_MS={elapsed_ms:.2f}', flush=True)
" || { fail "V-0c FAIL"; exit 1; }
ok "V-0c:handle_timeout(50) 阻塞 40-150ms(实测区间,lcm '至少 N' 语义 + GIL 抖动)"

# V-0d: 跨进程 send/recv 5s 实测
bar "V-0d:跨进程 send/recv 5s"
LCM_URL="${LCM_URL:-udpm://239.255.76.67:7667?ttl=1}"
python3 - "$LCM_URL" <<'PY' > /tmp/v0d_sub.txt 2>&1 &
import sys, time, lcm
url = sys.argv[1] if len(sys.argv) > 1 else 'udpm://239.255.76.67:7667?ttl=1'
lc = lcm.LCM(url)
got = []
def cb(ch, data):
    got.append((ch, len(data)))
lc.subscribe('/v0d_test', cb)
deadline = time.monotonic() + 4.0
while time.monotonic() < deadline and not got:
    lc.handle_timeout(100)
if got:
    print(f'V0D_RECEIVED={got[0]}', flush=True)
    sys.exit(0)
print('V0D_TIMEOUT', file=sys.stderr)
sys.exit(4)
PY
SUB_PID=$!
sleep 0.5
python3 -c "
import lcm
lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
lc.publish('/v0d_test', b'x' * 32)
print('V0D_PUBLISHED', flush=True)
"
wait ${SUB_PID} || { fail "V-0d FAIL"; exit 1; }
grep -q 'V0D_RECEIVED' /tmp/v0d_sub.txt || { fail "V-0d FAIL:subscriber 收不到"; exit 1; }
ok "V-0d:跨进程 send/recv 真通"

# V-0e: callback signature 1.3.1 (channel, data) vs 1.5 (data)
bar "V-0e:callback signature 1.3.1 vs 1.5"
# 已在 V-0b 探,根据 CB_ARGS 长度判定
if [[ ${ARGS_LEN} -eq 2 ]]; then
    ok "V-0e:lcm-python 1.3.1 callback 是 (channel, data) 双参(v5 lambda 包装必要)"
elif [[ ${ARGS_LEN} -eq 1 ]]; then
    ok "V-0e:lcm-python 1.5 callback 是 (data) 单参(v5 lambda 包装兼容)"
else
    fail "V-0e FAIL:callback args 长度 ${ARGS_LEN} 既非 1 也非 2"
    exit 1
fi

# V-0f: wait_for_data_in_read 参数支持性
bar "V-0f:LCM(url, wait_for_data_in_read=False) 参数支持性"
python3 -c "
import lcm
try:
    lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=1', wait_for_data_in_read=False)
    print('SUPPORTED', flush=True)
except TypeError as e:
    print(f'NOT_SUPPORTED: {e}', flush=True)
" || true
ok "V-0f:已记录(此参数不被 v5 使用,仅诊断)"

# V-0g: 同 lc 实例 publish + handle_timeout 并发死锁
bar "V-0g:同 lc 实例 publish + handle_timeout 并发(60s 必返)"
python3 -c "
import lcm, threading, time
lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
stop = threading.Event()
def pub():
    for i in range(20):
        if stop.is_set(): break
        try: lc.publish('/v0g', b'x' * 8)
        except: pass
        time.sleep(0.05)
t = threading.Thread(target=pub, daemon=True)
t.start()
t0 = time.monotonic()
for _ in range(5):
    lc.handle_timeout(50)
elapsed = time.monotonic() - t0
stop.set()
t.join(timeout=1.0)
assert elapsed < 5.0, f'5 次 handle_timeout 耗时 {elapsed:.2f}s > 5s(可能死锁)'
print(f'OK elapsed={elapsed:.2f}s', flush=True)
" || { fail "V-0g FAIL:可能死锁"; exit 1; }
ok "V-0g:同 lc 实例 publish + handle_timeout 并发不死锁"

# V-0h: subscribe 后立即 unsubscribe 间隔
bar "V-0h:subscribe 后立即 unsubscribe 间隔"
python3 -c "
import lcm, time
lc = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
sub = lc.subscribe('/v0h', lambda c, d: None)
try:
    lc.unsubscribe(sub)
    print('IMMEDIATE_OK', flush=True)
except Exception as e:
    print(f'IMMEDIATE_FAIL: {type(e).__name__}: {e}', file=__import__('sys').stderr)
    import sys; sys.exit(1)
print('OK', flush=True)
" || { fail "V-0h FAIL"; exit 1; }
ok "V-0h:subscribe 后立即 unsubscribe 成功"

# V-0i: lcm.EventLog 存在性 + 返回类型(N-R11-1 实证修)
bar "V-0i:lcm.EventLog 存在性 + read_next_event 返回类型"
python3 -c "
import lcm, os, time, sys
# 用 lcm.EventLog 'w' 写 1 帧 + 'r' 读回(避免 subprocess.Popen GC 释放)
log_path = '/tmp/v0i_test.log'
if os.path.exists(log_path): os.remove(log_path)
try:
    w = lcm.EventLog(log_path, 'w')
    w.write_event(int(time.time_ns()), '/test_v0i', b'x' * 32)
    w.close()
except Exception as e:
    print(f'WRITE_FAIL: {e}', file=sys.stderr); sys.exit(1)

has_eventlog = hasattr(lcm, 'EventLog')
print(f'EVENTLOG_EXISTS={has_eventlog}', flush=True)
if has_eventlog:
    try:
        log = lcm.EventLog(log_path, 'r')
        ev = log.read_next_event()
        if ev is None:
            print('READ_NONE: log 文件空或 parse 失败', file=sys.stderr)
            print('RETURN_TYPE=None', flush=True)
            print('IS_EVENT=False', flush=True)
        else:
            print(f'RETURN_TYPE={type(ev).__name__}', flush=True)
            print(f'IS_EVENT={isinstance(ev, lcm.Event)}', flush=True)
            print(f'HAS_TIMESTAMP={hasattr(ev, \"timestamp\")}', flush=True)
            print(f'HAS_CHANNEL={hasattr(ev, \"channel\")}', flush=True)
            print(f'HAS_DATA={hasattr(ev, \"data\")}', flush=True)
        log.close()
    except Exception as e:
        print(f'EVENTLOG_FAIL: {type(e).__name__}: {e}', file=sys.stderr)
        sys.exit(1)
else:
    print('EVENTLOG_NOT_EXISTS: 走 v5 §1.4 fallback(lcm-logger + xxd 字节级 parse)', file=sys.stderr)
    sys.exit(2)  # 走 fallback
" > /tmp/v0i_out.txt 2>&1
V0I_RC=$?
if [[ ${V0I_RC} -eq 0 ]]; then
    grep 'EVENTLOG_EXISTS' /tmp/v0i_out.txt
    grep 'RETURN_TYPE\|IS_EVENT' /tmp/v0i_out.txt
    ok "V-0i:lcm.EventLog 存在,read_next_event 返回类型已记录"
elif [[ ${V0I_RC} -eq 2 ]]; then
    nr "V-0i:lcm.EventLog 不存在 → v5 §1.4 fallback(lcm-logger + xxd)"
    echo "  v5 §1.4 决策:verifier 改用 lcm-logger + xxd 字节级 parse 头部 8B fp + 8B seqno + 32B payload"
    # 退出码 0,视为 PASS 但走 fallback
else
    fail "V-0i FAIL"
    exit 1
fi

ok "V-0 全 9 条探针 PASS — 实施可继续"
exit 0
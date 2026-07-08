# services/navigation/port_utils.py
"""多进程端口分配工具(RALPLAN R8 N-R7-2 实证修复)。

v3 抄 Rust test_lcm_url() 用模块级 counter 失败原因:
- Rust cargo test 默认单进程多线程 → counter 单进程内递增 OK
- Python pytest / shell smoke / CI runner 多进程并行 → 各进程独立 import 模块
  → counter 全 = 0 → 端口公式退化为 pid*10000+0 → 撞端口

v4/v5 修:用 itertools.count() + threading.Lock 取代 random.Random()。
"""
import os
import threading
from itertools import count

_TEST_PORT_COUNTER = count(0)  # 模块级单调递增 counter
_TEST_PORT_LOCK = threading.Lock()


def test_lcm_url() -> str:
    """端口公式 V5 改造(R8 N-R7-2 修):多进程并行不撞端口。

    R8 实证提议方案:port = (pid * 10000 + counter) % 60000 + 7667
    区间 [7667, 67666] 256 个端口可并行。

    Returns:
        udpm://239.255.76.67:<port>?ttl=1 字符串
    """
    with _TEST_PORT_LOCK:
        counter = next(_TEST_PORT_COUNTER)
    pid = os.getpid()
    port = (pid * 10_000 + counter) % 60_000 + 7_667  # 区间 [7667, 67666]
    return f"udpm://239.255.76.67:{port}?ttl=1"

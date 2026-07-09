#!/usr/bin/env python3
"""R4 P3 退步修复:nuwax bridge subscriber 子进程。

smoke shell inline 调用;不在工作区(feedback_no_test_commits 一致)。
LcmMotionCommandBridge 启 driver thread + subscribe + RecordingSink,
3s 内收 1 帧 → 写 "RECEIVED: <channel> <source> <ts_deviation_ns>" 到 stdout 后 exit 0。
"""
import sys, time
sys.path.insert(0, '/home/ml/Deeprob/nuwax_robot_bridge')
from services.navigation.lcm_motion_command_bridge import (
    LcmMotionCommandBridge, MotionClampConfig
)


class RecordingSink:
    def __init__(self):
        self.received = []
        self.cmd_count = 0
        self.zero_count = 0

    def __call__(self, twist, source):
        self.received.append((time.monotonic_ns(), source))
        if source == "cmd":
            self.cmd_count += 1
        else:
            self.zero_count += 1


def main():
    if len(sys.argv) != 3:
        print("usage: nuwax_bridge_subscriber.py <lcm_url> <channel>", file=sys.stderr)
        sys.exit(2)
    lc_url = sys.argv[1]
    channel = sys.argv[2]
    sink = RecordingSink()
    bridge = LcmMotionCommandBridge(
        lcm_url=lc_url, channel=channel, twist_sink=sink,
        clamp=MotionClampConfig()
    )
    bridge.start()
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        if sink.received:
            ts_dev = abs(time.monotonic_ns() - sink.received[0][0])
            src = sink.received[0][1]
            print(f"RECEIVED: channel={channel} source={src} ts_deviation_ns={ts_dev}", flush=True)
            sys.exit(0)
        time.sleep(0.05)
    print("TIMEOUT: 4s 内未收到帧", file=sys.stderr)
    sys.exit(4)


if __name__ == "__main__":
    main()
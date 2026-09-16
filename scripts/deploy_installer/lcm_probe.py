from __future__ import annotations

import os
import secrets
import sys
import time


def main() -> int:
    try:
        import lcm
    except ImportError as exc:
        print(f"LCM probe cannot import lcm: {exc}", file=sys.stderr)
        return 2
    url = os.environ.get("NUWAX_LCM_URL", "udpm://239.255.76.67:7667?ttl=0")
    channel = f"NUWAX_INSTALL_PROBE_{os.getpid()}_{secrets.token_hex(4)}"
    payload = secrets.token_bytes(32)
    received = []
    bus = lcm.LCM(url)
    subscription = bus.subscribe(channel, lambda _channel, data: received.append(bytes(data)))
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not received:
            bus.publish(channel, payload)
            bus.handle_timeout(250)
        if received != [payload]:
            print("LCM multicast roundtrip did not receive the probe payload", file=sys.stderr)
            return 1
        print(f"LCM multicast roundtrip PASS: {channel}")
        return 0
    finally:
        bus.unsubscribe(subscription)


if __name__ == "__main__":
    raise SystemExit(main())

"""Stream mirror tool — subscribes to a stream topic via SSE and writes JSONL.

vis-three-fixes task 1.6：把 video.detections 流镜像到 JSONL 文件，
用于阶段 0 闸门 5 分钟窗口采集（perception_history_limit=100 不够）。

Usage:
    python tools/stream_mirror.py --topic video.detections \
        --out /tmp/perception_stream_mirror.jsonl --duration 600

可选 --host / --port，默认 127.0.0.1:8080（开发网关默认端口）。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from typing import Optional


def fetch_sse_block(url: str, timeout_sec: float) -> Optional[str]:
    """Read one SSE block (terminated by a blank line). Returns raw text or None on EOF/error."""
    try:
        req = urllib.request.Request(url, headers={"Accept": "text/event-stream"})
        with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
            buf = []
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                line = resp.readline()
                if not line:
                    return None
                text = line.decode("utf-8", errors="replace").rstrip("\n").rstrip("\r")
                buf.append(text)
                if text == "":
                    return "\n".join(buf)
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        print(f"[stream_mirror] SSE 连接错误: {exc}", file=sys.stderr)
        return None
    return None


def parse_sse_payload(block: str) -> Optional[dict]:
    """解析 SSE 块，提取 data: 行作为 JSON dict。"""
    data_lines: list[str] = []
    for raw in block.splitlines():
        line = raw.rstrip()
        if line.startswith("data:"):
            data_lines.append(line[5:].lstrip())
    if not data_lines:
        return None
    payload_text = "\n".join(data_lines)
    try:
        return json.loads(payload_text)
    except json.JSONDecodeError:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description="把 stream topic 镜像到 JSONL 文件。")
    parser.add_argument("--topic", required=True, help="流主题，例如 video.detections")
    parser.add_argument("--out", required=True, help="输出 JSONL 文件路径")
    parser.add_argument("--host", default="127.0.0.1", help="网关 host（默认 127.0.0.1）")
    parser.add_argument("--port", type=int, default=8080, help="网关端口（默认 8080）")
    parser.add_argument("--duration", type=float, default=600.0, help="采集时长（秒，默认 600）")
    parser.add_argument("--block-timeout", type=float, default=30.0, help="单次 SSE block 读取超时（秒）")
    args = parser.parse_args()

    sse_url = f"http://{args.host}:{args.port}/streams/sse?topics={args.topic}"
    deadline = time.monotonic() + args.duration
    written = 0
    start_at = time.time()

    with open(args.out, "a", encoding="utf-8") as fout:
        print(f"[stream_mirror] topic={args.topic} out={args.out} duration={args.duration}s url={sse_url}",
              file=sys.stderr)
        while time.monotonic() < deadline:
            block = fetch_sse_block(sse_url, args.block_timeout)
            if block is None:
                time.sleep(0.5)
                continue
            payload = parse_sse_payload(block)
            if payload is None:
                continue
            payload["_mirrored_at"] = time.time()
            fout.write(json.dumps(payload, ensure_ascii=False) + "\n")
            fout.flush()
            written += 1
            if written % 50 == 0:
                print(f"[stream_mirror] 已写入 {written} 条，距结束 {int(deadline - time.monotonic())}s",
                      file=sys.stderr)

    elapsed = time.time() - start_at
    print(f"[stream_mirror] 完成。写入 {written} 条，耗时 {elapsed:.1f}s -> {args.out}",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
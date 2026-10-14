"""网页、安装入口与SSH薄包装共用同一个网关更新事务。"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def api(url, path="", payload=None):
    body = None if payload is None else json.dumps(payload).encode()
    request = urllib.request.Request(
        url.rstrip("/") + "/api/updates" + path, data=body, headers={"Content-Type": "application/json"} if body else {}
    )
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request, timeout=130) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        try:
            message = json.load(error).get("error", {}).get("message", str(error.code))
        except (ValueError, AttributeError):
            message = "更新接口返回错误 " + str(error.code)
        raise RuntimeError(message) from error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", choices=("check", "install", "status"), default="check")
    parser.add_argument("--url", default="http://127.0.0.1:8766", help="已运行的网关地址")
    parser.add_argument("--yes", action="store_true", help="确认执行检查到的版本更新")
    parser.add_argument("--wait", action="store_true", help="等待事务完成，关闭客户端不会取消后台更新")
    args = parser.parse_args(argv)
    try:
        if args.command == "status":
            result = api(args.url)
        else:
            result = api(args.url, "/check", {})
            if args.command == "install" and result.get("available"):
                if not args.yes:
                    parser.error("安装会停止运行服务，需要 --yes 明确确认；先停止任务并停用设备电机")
                result = api(args.url, "/install", {"revision": result["latest_revision"]})
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        if args.wait and result.get("transaction", {}).get("state") in {
            "queued",
            "fetching",
            "stopping",
            "building",
            "starting",
            "verifying",
            "rolling_back",
        }:
            request_id = result["transaction"]["request_id"]
            deadline = time.monotonic() + 7500
            while time.monotonic() < deadline:
                time.sleep(3)
                try:
                    result = api(args.url)
                except (OSError, ValueError):
                    continue
                transaction = result.get("transaction", {})
                if transaction.get("request_id") != request_id:
                    raise RuntimeError("更新事务身份改变，请检查状态；没有重新提交请求")
                status = transaction.get("state")
                if status in {"succeeded", "failed", "rolled_back", "recovery_required"}:
                    print(json.dumps(result, ensure_ascii=False, indent=2))
                    return 0 if status == "succeeded" else 1
            raise RuntimeError("等待超时；后台事务不会因此取消，请使用 update status 检查")
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

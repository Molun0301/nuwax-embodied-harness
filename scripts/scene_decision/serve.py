#!/usr/bin/env python3
"""启动隔离模型服务；默认仅监听本机，不在请求中接受模型路径。"""
import argparse
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    from huggingface_hub import snapshot_download
    import uvicorn
    from services.scene_decision.local_model import MODEL_REVISION, MODEL_SUBFOLDER, PROCESSOR_REVISION
    from services.scene_decision.local_server import ModelProcess, create_app

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8791)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--load-timeout-sec", type=float, default=180.0)
    parser.add_argument("--max-tokens", type=int, default=4096)
    parser.add_argument("--load-in-4bit", action="store_true", help="使用bitsandbytes NF4量化加载，计算保持BF16；默认完整BF16")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not 0 < args.timeout_sec <= 600 or not 0 < args.load_timeout_sec <= 600 or not 128 <= args.max_tokens <= 8192:
        parser.error("端口、超时或上下文长度超出支持范围。")
    token = os.environ.get("NUWAX_SCENE_DECISION_TOKEN", "")
    if args.host not in {"127.0.0.1", "::1", "localhost"} and not token:
        parser.error("监听其它主机前须设置NUWAX_SCENE_DECISION_TOKEN。")
    model = snapshot_download("AlexWortega/openjev", revision=MODEL_REVISION, local_files_only=True)
    processor = snapshot_download("Qwen/Qwen3.5-4B", revision=PROCESSOR_REVISION, local_files_only=True)
    controller = ModelProcess(model_options={"model_path": str(Path(model) / MODEL_SUBFOLDER), "processor_path": processor,
                                             "max_tokens": args.max_tokens, "load_in_4bit": args.load_in_4bit}, timeout_sec=args.timeout_sec,
                              load_timeout_sec=args.load_timeout_sec)
    uvicorn.run(create_app(controller, token=token), host=args.host, port=args.port,
                limit_concurrency=8, backlog=16, timeout_keep_alive=5, timeout_graceful_shutdown=10)


if __name__ == "__main__":
    main()

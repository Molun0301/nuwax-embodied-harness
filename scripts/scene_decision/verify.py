#!/usr/bin/env python3
"""通过真实本机接口验证文本及合成图像；输出不能作为房间精度验收。"""
import argparse
import base64
from io import BytesIO
import json
import os
from pathlib import Path
import sys
import time
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    from PIL import Image
    from contracts.scene_decision import SceneDecisionRequest
    from services.scene_decision import create_scene_decision_backend
    from services.scene_decision.local_model import MODEL_ID

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8791")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=180)
    args = parser.parse_args()
    health_url = args.base_url.rstrip("/") + "/health"
    deadline = time.monotonic() + 240
    health = None
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(health_url, timeout=5) as response:
                health = json.load(response)
            if health["ready"]:
                break
        except urllib.error.HTTPError as exc:
            data = json.load(exc)
            if data.get("status") == "failed":
                raise RuntimeError("模型加载失败：" + str(data)) from exc
        time.sleep(1)
    if not health or not health["ready"]:
        raise RuntimeError("模型未在期限内就绪。")
    backend = create_scene_decision_backend(backend="openjev_http", base_url=args.base_url, model=MODEL_ID,
                                             api_key=os.environ.get("NUWAX_SCENE_DECISION_TOKEN", ""), timeout_sec=args.timeout_sec)
    text_request = SceneDecisionRequest(state={"observations": ["An enclosed room contains six office desks, monitors and swivel chairs.", "People work at computers. A sign says engineering office."]},
                                        categories={"office": "An office where people work at desks with computers.",
                                                    "corridor": "A corridor used as a passage between rooms.",
                                                    "unknown": "The evidence is insufficient to identify the room category."})
    cases = {"text_office": backend.decide(text_request).model_dump(mode="json")}
    image_categories = {"red": "The image is predominantly red.", "blue": "The image is predominantly blue.",
                        "unknown": "The dominant image colour cannot be determined."}
    images = []
    for color in ("red", "blue"):
        buffer = BytesIO()
        Image.new("RGB", (320, 240), color=color).save(buffer, format="PNG")
        image = "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")
        images.append(image)
        request = SceneDecisionRequest(state={"observation": "The attached image is the visual evidence."},
                                       categories=image_categories, images=[image])
        cases["synthetic_" + color] = backend.decide(request).model_dump(mode="json")
    cases["two_views"] = backend.decide(SceneDecisionRequest(state={"observation": "Two different views."},
                                                               categories=image_categories, images=images)).model_dump(mode="json")
    backend.close()
    payload = {"health": health, "cases": cases,
               "validation_scope": "真实模型加载与文本、图像、多图接口执行；不代表房间识别精度验收。"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({name: {"scores": item["category_scores"], "inference_duration_sec": item["metadata"]["inference_duration_sec"],
                             "peak_gpu_bytes": item["metadata"]["peak_gpu_bytes"]} for name, item in cases.items()}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

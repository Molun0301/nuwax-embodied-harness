#!/usr/bin/env python3
"""仅下载固定4B图文模型、图像处理器及审阅参考源码。"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from services.scene_decision.local_model import MODEL_REVISION, MODEL_SUBFOLDER, PROCESSOR_REVISION


def main():
    from huggingface_hub import snapshot_download

    model = snapshot_download("AlexWortega/openjev", revision=MODEL_REVISION,
                              allow_patterns=[MODEL_SUBFOLDER + "/*", "code/eval_image_nli.py", "code/doom_vision.py", "code/train.py", "README.md"], max_workers=2)
    processor = snapshot_download("Qwen/Qwen3.5-4B", revision=PROCESSOR_REVISION,
                                  allow_patterns=["preprocessor_config.json", "processor_config.json"], max_workers=2)
    print("模型目录：", str(Path(model) / MODEL_SUBFOLDER))
    print("处理器目录：", processor)


if __name__ == "__main__":
    main()

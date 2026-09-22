"""固定OpenJev图文检查点的真实三分类推理，仅在独立进程显式加载。"""
from __future__ import annotations

import base64
import io
import json
import time
import warnings
from typing import Any, Dict

from contracts.scene_decision import SceneDecisionRequest, SceneDecisionResult


MODEL_ID = "AlexWortega/openjev:qwen3.5-4b-nli-v2"
MODEL_REVISION = "f004f37e52695d6ddfb914a64dbf93942839ba1e"
PROCESSOR_REVISION = "851bf6e806efd8d0a36b00ddf55e13ccb7b8cd0a"
MODEL_SUBFOLDER = "qwen3.5-4b-nli-v2"


class LocalModelInputError(ValueError):
    """实际分词或解码时发现的输入限制。"""


class OpenJevModel:
    def __init__(self, *, model_path: str, processor_path: str, max_tokens: int = 4096) -> None:
        # 平台导入此模块不会触发GPU或远程代码加载。
        import torch
        from transformers import AutoImageProcessor, AutoModelForSequenceClassification, AutoTokenizer

        started = time.monotonic()
        torch.set_num_threads(4)
        if not torch.cuda.is_available():
            raise RuntimeError("本地OpenJev服务需要可用CUDA设备。")
        torch.cuda.reset_peak_memory_stats()
        self._torch = torch
        self._max_tokens = max_tokens
        self.tokenizer = AutoTokenizer.from_pretrained(model_path, local_files_only=True, trust_remote_code=False)
        self.tokenizer.padding_side = "right"
        self.model = AutoModelForSequenceClassification.from_pretrained(
            model_path, local_files_only=True, trust_remote_code=False,
            dtype=torch.bfloat16, attn_implementation="sdpa",
        ).to("cuda").eval()
        self.model.config.get_text_config().pad_token_id = self.tokenizer.pad_token_id
        self.labels = [self.model.config.id2label[index] for index in range(3)]
        if self.labels != ["contradiction", "entailment", "neutral"]:
            raise RuntimeError("检查点不是预期的OpenJev三分类模型。")
        self.template = self.model.config.nli_template
        self.processor = AutoImageProcessor.from_pretrained(processor_path, local_files_only=True, trust_remote_code=False)
        self.image_token_id = self.tokenizer.convert_tokens_to_ids("<|image_pad|>")
        torch.cuda.synchronize()
        self.metrics = {"load_duration_sec": round(time.monotonic() - started, 3),
                        "loaded_gpu_bytes": torch.cuda.memory_allocated(),
                        "peak_gpu_bytes": torch.cuda.max_memory_allocated(),
                        "device": torch.cuda.get_device_name(), "dtype": "bfloat16",
                        "model_revision": MODEL_REVISION, "processor_revision": PROCESSOR_REVISION,
                        "max_tokens": max_tokens}

    def _image(self, data_url: str):
        from PIL import Image, UnidentifiedImageError

        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", Image.DecompressionBombWarning)
                source = Image.open(io.BytesIO(base64.b64decode(data_url.split(",", 1)[1], validate=True)))
                if source.format not in {"PNG", "JPEG", "WEBP"} or source.width * source.height > 16_000_000:
                    raise LocalModelInputError("图像格式不支持或超过1600万像素。")
                source.load()
                image = source.convert("RGB")
            original_size = list(image.size)
            image.thumbnail((448, 448), Image.Resampling.LANCZOS)
            visual = self.processor(images=[image], return_tensors="pt")
            count = int(visual["image_grid_thw"].prod()) // self.processor.merge_size ** 2
            return visual, count, {"original_size": original_size, "processed_size": list(image.size), "image_tokens": count}
        except (UnidentifiedImageError, OSError, Image.DecompressionBombError, Image.DecompressionBombWarning) as exc:
            raise LocalModelInputError("图像内容无法解码。") from exc

    def decide(self, request: SceneDecisionRequest) -> SceneDecisionResult:
        torch = self._torch
        started = time.monotonic()
        torch.cuda.reset_peak_memory_stats()
        premise = json.dumps(request.state, ensure_ascii=False, allow_nan=False, sort_keys=True)
        category_items = list(request.categories.items())
        views = request.images or [None]
        raw_views = []
        totals = {key: 0.0 for key in request.categories}
        for image_index, image_data in enumerate(views):
            visual = None
            image_metadata = {}
            view_premise = premise
            if image_data is not None:
                visual, count, image_metadata = self._image(image_data)
                block = "<|vision_start|>" + "<|image_pad|>" * count + "<|vision_end|>"
                view_premise = "Observed image: " + block + "\nObserved facts: " + premise
            scores = {}
            # 两个候选一批，避免32类和8图将缓存同时扩张。
            for start in range(0, len(category_items), 2):
                batch = category_items[start:start + 2]
                texts = [self.template.format(premise=view_premise,
                         hypothesis="The observed scene matches category " + key + ": " + description)
                         for key, description in batch]
                encoded = self.tokenizer(texts, add_special_tokens=False, padding=True, return_tensors="pt")
                if encoded["input_ids"].shape[1] > self._max_tokens:
                    raise LocalModelInputError("事实与图像超过模型服务上下文上限；请减少输入。")
                inputs = {key: value.to("cuda") for key, value in encoded.items()}
                if visual is not None:
                    inputs.update({"pixel_values": visual["pixel_values"].repeat(len(batch), 1).to("cuda"),
                                   "image_grid_thw": visual["image_grid_thw"].repeat(len(batch), 1).to("cuda"),
                                   "mm_token_type_ids": (inputs["input_ids"] == self.image_token_id).long()})
                with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
                    logits = self.model(**inputs, use_cache=False).logits.float()
                if not torch.isfinite(logits).all():
                    raise RuntimeError("模型输出包含非有限数值。")
                probabilities = torch.softmax(logits, dim=-1).cpu().tolist()
                logit_rows = logits.cpu().tolist()
                for (key, _), row, raw_logits in zip(batch, probabilities, logit_rows):
                    scores[key] = {"probabilities": dict(zip(self.labels, row)), "logits": dict(zip(self.labels, raw_logits))}
                    totals[key] += row[1]
                del inputs, encoded, logits
            raw_views.append({"image_index": image_index if image_data is not None else None,
                              "image": image_metadata, "nli": scores})
        torch.cuda.synchronize()
        return SceneDecisionResult(
            category_scores={key: value / len(views) for key, value in totals.items()},
            score_kind="nli_entailment", model_id=MODEL_ID,
            raw={"labels": self.labels, "views": raw_views},
            metadata={"backend": "openjev_http", "model_revision": MODEL_REVISION,
                      "processor_revision": PROCESSOR_REVISION, "view_count": len(views),
                      "aggregation": "arithmetic_mean_entailment_across_views",
                      "cross_category_normalized": False, "inference_duration_sec": round(time.monotonic() - started, 3),
                      "peak_gpu_bytes": torch.cuda.max_memory_allocated(), "max_tokens": self._max_tokens},
        )

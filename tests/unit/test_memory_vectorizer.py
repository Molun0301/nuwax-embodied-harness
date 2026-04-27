from __future__ import annotations

from pathlib import Path
import sys
import types

import numpy as np
import pytest
import httpx

from services.model_providers import (
    resolve_detection_provider_spec,
    resolve_segmentation_provider_spec,
)
from services.memory.vectorizer import (
    build_text_embedder,
    build_vision_language_embedder,
)


def test_build_text_embedder_accepts_local_directory_and_disables_auto_download(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """本地文本模型目录应可直接加载，并强制仅使用本地文件。"""

    model_dir = tmp_path / "BAAI__bge-m3"
    model_dir.mkdir()
    calls = {}

    class _FakeSentenceTransformer:
        def __init__(self, model_name, device=None, local_files_only=False):
            calls["model_name"] = model_name
            calls["device"] = device
            calls["local_files_only"] = local_files_only

        def encode(self, texts, normalize_embeddings=True, convert_to_numpy=True):
            del texts, normalize_embeddings, convert_to_numpy
            return np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32)

    monkeypatch.setitem(
        sys.modules,
        "sentence_transformers",
        types.SimpleNamespace(SentenceTransformer=_FakeSentenceTransformer),
    )

    embedder = build_text_embedder(str(model_dir), dimension=4)

    assert embedder.dimension == 4
    assert calls["model_name"] == str(model_dir)
    assert calls["local_files_only"] is True


def test_build_vision_language_embedder_accepts_local_directory_and_disables_auto_download(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """本地 CLIP 模型目录应可直接加载，并强制仅使用本地文件。"""

    model_dir = tmp_path / "openai__clip-vit-base-patch32"
    model_dir.mkdir()
    calls = {}

    class _FakeProcessor:
        @classmethod
        def from_pretrained(cls, model_name, local_files_only=False):
            calls["processor_model_name"] = model_name
            calls["processor_local_files_only"] = local_files_only
            return cls()

    class _FakeModel:
        config = types.SimpleNamespace(projection_dim=8)

        @classmethod
        def from_pretrained(cls, model_name, local_files_only=False):
            calls["model_name"] = model_name
            calls["model_local_files_only"] = local_files_only
            return cls()

        def to(self, device):
            calls["device"] = device

        def eval(self):
            calls["eval_called"] = True

    class _FakeTorch:
        class cuda:
            @staticmethod
            def is_available() -> bool:
                return False

    monkeypatch.setitem(
        sys.modules,
        "transformers",
        types.SimpleNamespace(CLIPModel=_FakeModel, CLIPProcessor=_FakeProcessor),
    )
    monkeypatch.setitem(sys.modules, "torch", _FakeTorch)

    embedder = build_vision_language_embedder(str(model_dir), dimension=8)

    assert embedder is not None
    assert calls["processor_model_name"] == str(model_dir)
    assert calls["processor_local_files_only"] is True
    assert calls["model_name"] == str(model_dir)
    assert calls["model_local_files_only"] is True
    assert calls["device"] == "cpu"
    assert calls["eval_called"] is True


def test_build_dashscope_text_embedder_supports_fused_text_image(monkeypatch) -> None:
    """DashScope 文本向量器应能按官方融合格式发送 text+image。"""

    calls = []

    class _FakeResponse:
        def raise_for_status(self) -> None:
            return None

        def json(self):
            return {
                "output": {
                    "embeddings": [
                        {"embedding": [1.0] + ([0.0] * 767)},
                    ]
                }
            }

    class _FakeClient:
        def __init__(self, *, timeout=None, headers=None):
            calls.append({"timeout": timeout, "headers": headers})

        def post(self, url, json):
            calls.append({"url": url, "json": json})
            return _FakeResponse()

    monkeypatch.setattr("services.memory.vectorizer.httpx.Client", _FakeClient)

    embedder = build_text_embedder(
        "tongyi-embedding-vision-flash-2026-03-06",
        dimension=768,
        api_key="sk-test",
        base_url="https://dashscope.aliyuncs.com/api/v1",
        timeout_sec=12.0,
    )

    text_vector = embedder.embed_text("机器人看到充电桩")
    fused_vector = embedder.embed_fused_text_image(
        text="机器人看到充电桩",
        image_bytes=b"\xff\xd8\xff\xdbdemo-jpeg",
    )

    assert text_vector.shape == (768,)
    assert fused_vector.shape == (768,)
    assert calls[0]["headers"]["Authorization"] == "Bearer sk-test"
    assert calls[1]["url"].endswith("/services/embeddings/multimodal-embedding/multimodal-embedding")
    assert calls[1]["json"]["input"]["contents"] == [{"text": "机器人看到充电桩"}]
    assert calls[2]["json"]["input"]["contents"][0]["text"] == "机器人看到充电桩"
    assert calls[2]["json"]["input"]["contents"][0]["image"].startswith("data:image/jpeg;base64,")
    assert calls[2]["json"]["parameters"]["dimension"] == 768


def test_build_dashscope_text_embedder_requires_api_key() -> None:
    """在线多模态向量默认实现缺少 API Key 时应明确报错。"""

    with pytest.raises(RuntimeError, match="API Key"):
        build_text_embedder(
            "tongyi-embedding-vision-flash-2026-03-06",
            dimension=768,
        )


def test_build_dashscope_text_embedder_clamps_unsupported_dimension(monkeypatch) -> None:
    """Tongyi 在线向量模型配置非法维度时应自动回退到合法值。"""

    calls = []

    class _FakeResponse:
        def raise_for_status(self) -> None:
            return None

        def json(self):
            return {
                "output": {
                    "embeddings": [
                        {"embedding": [1.0] + ([0.0] * 767)},
                    ]
                }
            }

    class _FakeClient:
        def __init__(self, *, timeout=None, headers=None):
            del timeout, headers

        def post(self, url, json):
            calls.append({"url": url, "json": json})
            return _FakeResponse()

    monkeypatch.setattr("services.memory.vectorizer.httpx.Client", _FakeClient)

    embedder = build_text_embedder(
        "tongyi-embedding-vision-flash-2026-03-06",
        dimension=1024,
        api_key="sk-test",
    )
    vector = embedder.embed_text("测试记忆查询")

    assert vector.shape == (768,)
    assert embedder.dimension == 768
    assert calls[0]["json"]["parameters"]["dimension"] == 768


def test_build_dashscope_text_embedder_surfaces_response_body_message(monkeypatch) -> None:
    """DashScope HTTP 错误应把服务端返回原因带到异常里。"""

    class _FakeResponse:
        status_code = 400
        text = '{"code":"BadRequest","message":"dimension must be one of [64, 128, 256, 512, 768], got 1024"}'

        def json(self):
            return {
                "code": "BadRequest",
                "message": "dimension must be one of [64, 128, 256, 512, 768], got 1024",
            }

    class _FakeClient:
        def __init__(self, *, timeout=None, headers=None):
            del timeout, headers

        def post(self, url, json):
            request = httpx.Request("POST", url, json=json)
            response = httpx.Response(status_code=400, request=request, json=_FakeResponse().json())
            raise httpx.HTTPStatusError("bad request", request=request, response=response)

    monkeypatch.setattr("services.memory.vectorizer.httpx.Client", _FakeClient)

    embedder = build_text_embedder(
        "tongyi-embedding-vision-flash-2026-03-06",
        dimension=768,
        api_key="sk-test",
    )

    with pytest.raises(RuntimeError, match="dimension must be one of \\[64, 128, 256, 512, 768\\], got 1024"):
        embedder.embed_text("测试记忆查询")




def test_resolve_segmentation_provider_spec_declares_primary_route() -> None:
    """主路线应声明 GroundingDINO + SAM2 能力并暴露质量与输出策略。"""

    spec = resolve_segmentation_provider_spec(
        enabled=True,
        backend_name="",
        backend_kind="groundingdino_sam2",
        route="primary",
        default_backend_name="segmentation_disabled",
        primary_backend_name="groundingdino_sam2",
        low_latency_backend_name="yolo_world_sam2",
        allow_bbox_only_fallback=True,
        quality_thresholds={"score": 0.66, "predicted_iou": 0.7},
        output_policy={"emit_polygon": True, "emit_raw_mask": False},
    )

    metadata = spec.to_metadata()

    assert spec.capability == "segmentation"
    assert spec.provider_kind == "primary"
    assert spec.provider_name == "groundingdino_sam2"
    assert metadata["route"] == "primary"
    assert metadata["grounding_provider"] == "groundingdino"
    assert metadata["segmentation_provider"] == "sam2"
    assert metadata["capabilities"]["ground"] is True
    assert metadata["capabilities"]["segment"] is True
    assert metadata["availability"] == "not_implemented"
    assert metadata["fallback_reason"] == "provider_not_available"
    assert metadata["quality_thresholds"]["score"] == 0.66
    assert metadata["output_policy"]["emit_polygon"] is True


def test_resolve_segmentation_provider_spec_declares_low_latency_route() -> None:
    """低时延路线应声明 YOLO-World + SAM2，且 contract 与主路线一致。"""

    spec = resolve_segmentation_provider_spec(
        enabled=True,
        backend_name="fast_segmenter",
        backend_kind="yolo_world_sam2",
        route="low_latency",
        low_latency_backend_name="yolo_world_sam2",
    )

    metadata = spec.to_metadata()

    assert spec.provider_kind == "low_latency"
    assert spec.provider_name == "yolo_world_sam2"
    assert spec.backend_name == "fast_segmenter"
    assert metadata["route"] == "low_latency"
    assert metadata["grounding_provider"] == "yolo_world"
    assert metadata["segmentation_provider"] == "sam2"
    assert metadata["capabilities"]["ground"] is True
    assert metadata["capabilities"]["segment"] is True


def test_resolve_segmentation_provider_spec_supports_passthrough_and_disabled() -> None:
    """禁用与检测属性透传应通过同一 provider spec 表达。"""

    disabled = resolve_segmentation_provider_spec(enabled=False, backend_name="sam2")
    passthrough = resolve_segmentation_provider_spec(
        enabled=True,
        backend_name="detection_attributes_segmentation",
        backend_kind="detection_attributes",
        route="passthrough",
    )

    disabled_metadata = disabled.to_metadata()
    passthrough_metadata = passthrough.to_metadata()

    assert disabled.provider_kind == "disabled"
    assert disabled_metadata["availability"] == "disabled"
    assert disabled_metadata["fallback_reason"] == "disabled_by_config"
    assert disabled_metadata["capabilities"]["segment"] is False
    assert passthrough.provider_kind == "passthrough"
    assert passthrough.provider_name == "detection_attributes"
    assert passthrough_metadata["source"] == "detection_attributes"
    assert passthrough_metadata["fallback_reason"] == "reuse_detector_segmentation_attributes"
    assert passthrough_metadata["capabilities"]["segment"] is True


def test_resolve_segmentation_provider_spec_supports_bbox_only_and_unsupported() -> None:
    """bbox-only 与未知路线应保留可判定的降级原因。"""

    bbox_only = resolve_segmentation_provider_spec(
        enabled=True,
        backend_name="bbox_only_segmentation",
        backend_kind="bbox_only_fallback",
        route="bbox_only",
        allow_bbox_only_fallback=True,
    )
    unsupported = resolve_segmentation_provider_spec(
        enabled=True,
        backend_name="experimental_segmenter",
        backend_kind="experimental",
        route="experimental",
    )

    bbox_only_metadata = bbox_only.to_metadata()
    unsupported_metadata = unsupported.to_metadata()

    assert bbox_only.provider_kind == "bbox_only_fallback"
    assert bbox_only.provider_name == "bbox_only_fallback"
    assert bbox_only_metadata["source"] == "bbox_only_fallback"
    assert bbox_only_metadata["fallback_reason"] == "bbox_only_fallback_requested"
    assert bbox_only_metadata["segment_capability_available"] is False
    assert bbox_only_metadata["capabilities"]["segment"] is False
    assert unsupported.provider_kind == "unsupported"
    assert unsupported.provider_name == "experimental"
    assert unsupported_metadata["availability"] == "unsupported"
    assert unsupported_metadata["fallback_reason"] == "unsupported_segmentation_route"
    assert unsupported_metadata["capabilities"]["segment"] is False

def test_resolve_detection_provider_spec_declares_metadata_detector() -> None:
    spec = resolve_detection_provider_spec(backend_name="metadata_detector", backend_kind="metadata")

    metadata = spec.to_metadata()

    assert spec.capability == "detection"
    assert spec.provider_kind == "local"
    assert spec.provider_name == "metadata_detector"
    assert metadata["availability"] == "available"
    assert metadata["fallback_used"] is False
    assert metadata["fallback_reason"] == ""
    assert metadata["requested_provider_name"] == "metadata"
    assert metadata["resolved_provider_name"] == "metadata_detector"
    assert metadata["requested_backend"] == "metadata_detector"
    assert metadata["resolved_backend"] == "metadata_detector"
    assert metadata["capabilities"]["detect"] is True


def test_resolve_detection_provider_spec_declares_yolo_detector() -> None:
    spec = resolve_detection_provider_spec(
        backend_name="ultralytics_yolo_tensorrt",
        backend_kind="ultralytics_yolo_tensorrt",
        model_name="yolov8n.pt",
    )

    metadata = spec.to_metadata()

    assert spec.provider_kind == "local"
    assert spec.provider_name == "ultralytics_yolo"
    assert spec.model_name == "yolov8n.pt"
    assert metadata["availability"] == "available"
    assert metadata["requested_provider_name"] == "ultralytics_yolo_tensorrt"
    assert metadata["resolved_provider_name"] == "ultralytics_yolo"
    assert metadata["capabilities"]["detect"] is True


def test_resolve_detection_provider_spec_declares_disabled_and_unsupported() -> None:
    disabled = resolve_detection_provider_spec(enabled=False, backend_name="disabled_detector", backend_kind="none")
    unsupported = resolve_detection_provider_spec(backend_name="experimental_detector", backend_kind="experimental")

    disabled_metadata = disabled.to_metadata()
    unsupported_metadata = unsupported.to_metadata()

    assert disabled.provider_kind == "disabled"
    assert disabled.provider_name == "disabled"
    assert disabled_metadata["availability"] == "disabled"
    assert disabled_metadata["fallback_reason"] == "disabled_by_config"
    assert disabled_metadata["capabilities"]["detect"] is False
    assert unsupported.provider_kind == "unsupported"
    assert unsupported.provider_name == "experimental"
    assert unsupported_metadata["availability"] == "unsupported"
    assert unsupported_metadata["fallback_reason"] == "unsupported_provider"
    assert unsupported_metadata["requested_provider_name"] == "experimental"
    assert unsupported_metadata["resolved_provider_name"] == "experimental"
    assert unsupported_metadata["capabilities"]["detect"] is False

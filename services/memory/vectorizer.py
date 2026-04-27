from __future__ import annotations

from abc import ABC, abstractmethod
import base64
from io import BytesIO
import hashlib
import json
import logging
from pathlib import Path
import re
from typing import Iterable, List, Optional

import httpx
import numpy as np

from services.model_providers import resolve_text_embedding_provider_spec, resolve_vision_language_provider_spec


VECTORIZER_LOGGER = logging.getLogger("nuwax_robot_bridge.memory.vectorizer")


class VectorCodecMixin:
    """向量编解码与相似度公共逻辑。"""

    model_name = "unknown"
    dimension = 0

    def cosine_similarity(self, left: np.ndarray, right: np.ndarray) -> float:
        if left.size == 0 or right.size == 0:
            return 0.0
        left_norm = float(np.linalg.norm(left))
        right_norm = float(np.linalg.norm(right))
        if left_norm == 0.0 or right_norm == 0.0:
            return 0.0
        return float(np.dot(left, right) / (left_norm * right_norm))

    def encode_vector(self, vector: np.ndarray) -> str:
        return json.dumps([round(float(item), 8) for item in vector.tolist()], ensure_ascii=False)

    def decode_vector(self, payload: str) -> np.ndarray:
        values = json.loads(payload)
        return np.asarray(values, dtype=np.float32)

    def _normalize_vector(self, vector: np.ndarray) -> np.ndarray:
        result = np.asarray(vector, dtype=np.float32).reshape(-1)
        norm = float(np.linalg.norm(result))
        if norm > 0.0:
            result = result / norm
        return result


class TextEmbedder(VectorCodecMixin, ABC):
    """文本向量化接口。"""

    @abstractmethod
    def embed_text(self, text: str) -> np.ndarray:
        """把文本编码成归一化向量。"""

    def embed_fused_text_image(self, *, text: str, image_bytes: bytes) -> np.ndarray:
        """把文本与图像融合成一个共享检索向量。"""

        del text, image_bytes
        raise NotImplementedError("当前文本向量化器不支持图文融合向量。")


class VisionLanguageEmbedder(VectorCodecMixin, ABC):
    """图像与文本共享空间的多模态向量化接口。"""

    @abstractmethod
    def embed_image_bytes(self, image_bytes: bytes) -> np.ndarray:
        """把图像字节编码成归一化向量。"""

    @abstractmethod
    def embed_text(self, text: str) -> np.ndarray:
        """把文本编码到与图像同构的空间。"""


class HashingTextEmbedder(TextEmbedder):
    """无外部依赖的文本向量化器。"""

    def __init__(self, *, model_name: str = "hashing-v1", dimension: int = 256) -> None:
        if dimension <= 0:
            raise ValueError("向量维度必须为正整数。")
        self.model_name = model_name
        self.dimension = dimension

    def embed_text(self, text: str) -> np.ndarray:
        features = self._build_features(text)
        vector = np.zeros(self.dimension, dtype=np.float32)
        if not features:
            return vector

        for feature in features:
            digest = hashlib.blake2b(feature.encode("utf-8"), digest_size=8).digest()
            feature_hash = int.from_bytes(digest, byteorder="big", signed=False)
            bucket = feature_hash % self.dimension
            sign = 1.0 if ((feature_hash >> 1) & 1) == 0 else -1.0
            vector[bucket] += sign

        return self._normalize_vector(vector)

    def _build_features(self, text: str) -> List[str]:
        normalized = self._normalize_text(text)
        if not normalized:
            return []

        base_tokens = normalized.split()
        condensed = normalized.replace(" ", "")
        features = list(base_tokens)
        if condensed:
            features.append(condensed)
            features.extend(self._iter_char_ngrams(condensed, 2))
            features.extend(self._iter_char_ngrams(condensed, 3))
        return list(dict.fromkeys(item for item in features if item))

    def _normalize_text(self, text: str) -> str:
        raw = str(text or "").strip().lower()
        if not raw:
            return ""
        normalized = re.sub(r"[^0-9a-z\u4e00-\u9fff]+", " ", raw)
        return " ".join(part for part in normalized.split() if part)

    def _iter_char_ngrams(self, text: str, size: int) -> Iterable[str]:
        if size <= 1:
            return []
        if len(text) <= size:
            return [text]
        return [text[index : index + size] for index in range(0, len(text) - size + 1)]


def _strip_backend_prefix(model_name: str, prefix: str) -> str:
    """去掉后端前缀，保留真正的模型标识。"""

    normalized = str(model_name or "").strip()
    if normalized.startswith(prefix):
        return normalized.split(":", 1)[1].strip()
    return normalized


def _resolve_local_model_path(model_name: str) -> Optional[Path]:
    """把显式本地路径解析成绝对路径。"""

    normalized = str(model_name or "").strip()
    if not normalized:
        return None

    candidate = Path(normalized).expanduser()
    if candidate.is_absolute():
        return candidate
    if candidate.exists():
        return candidate.resolve()

    if normalized.startswith("./") or normalized.startswith("../"):
        return (Path.cwd() / candidate).resolve()

    return None


def _ensure_local_model_directory(model_name: str) -> str:
    """校验本地模型目录存在，避免第三方库把缺失路径当成远端仓库名。"""

    local_path = _resolve_local_model_path(model_name)
    if local_path is None:
        return str(model_name or "").strip()
    if not local_path.exists():
        raise RuntimeError(
            "本地模型目录不存在："
            f"{local_path}。请先运行 `python3 scripts/prepare_memory_models.py` 下载模型。"
        )
    if not local_path.is_dir():
        raise RuntimeError(f"本地模型路径不是目录：{local_path}")
    return str(local_path)


def _build_manual_download_error(
    *,
    model_kind: str,
    requested_model_name: str,
    resolved_model_name: str,
    error: Exception,
) -> RuntimeError:
    """构造统一的本地模型缺失提示。"""

    return RuntimeError(
        f"{model_kind}未在本地准备完成，启动阶段不会自动下载。"
        "请先运行 `python3 scripts/prepare_memory_models.py`，"
        "或把配置改成已存在的本地模型目录。"
        f" requested={requested_model_name}"
        f" resolved={resolved_model_name}"
        f" error={error}"
    )


def _is_local_model_reference(model_name: str) -> bool:
    """判断当前配置是否显式指向本地目录。"""

    return _resolve_local_model_path(model_name) is not None


def _normalize_dashscope_model_name(model_name: str) -> str:
    """标准化 DashScope 模型名。"""

    return _strip_backend_prefix(model_name, "dashscope:")


def _is_dashscope_multimodal_model(model_name: str) -> bool:
    """判断是否为 DashScope 多模态向量模型。"""

    normalized = _normalize_dashscope_model_name(model_name).lower()
    return normalized.startswith("tongyi-embedding-vision") or normalized.startswith("qwen3-vl-embedding") or normalized.startswith("qwen2.5-vl-embedding") or normalized.startswith("multimodal-embedding-v1")


def _resolve_dashscope_endpoint(base_url: str) -> str:
    """把用户提供的 DashScope 基地址解析成最终请求地址。"""

    normalized = str(base_url or "").strip() or "https://dashscope.aliyuncs.com/api/v1"
    normalized = normalized.rstrip("/")
    suffix = "/services/embeddings/multimodal-embedding/multimodal-embedding"
    if normalized.endswith(suffix):
        return normalized
    if normalized.endswith("/api/v1"):
        return normalized + suffix.replace("/api/v1", "", 1)
    return normalized + suffix


_DASHSCOPE_ALLOWED_DIMENSIONS = {
    "tongyi-embedding-vision-flash-2026-03-06": (64, 128, 256, 512, 768),
}


def _resolve_dashscope_dimension(model_name: str, requested_dimension: Optional[int]) -> int:
    """把 DashScope 自定义维度配置收敛到模型允许范围。"""

    dimension = int(requested_dimension or 0)
    if dimension <= 0:
        return 0

    normalized = _normalize_dashscope_model_name(model_name).lower()
    allowed_dimensions = _DASHSCOPE_ALLOWED_DIMENSIONS.get(normalized)
    if not allowed_dimensions:
        return dimension
    if dimension in allowed_dimensions:
        return dimension

    fallback_dimension = max(allowed_dimensions)
    VECTORIZER_LOGGER.warning(
        "DashScope 向量维度配置不受支持，已自动回退到合法维度。model=%s requested=%s fallback=%s allowed=%s",
        normalized,
        dimension,
        fallback_dimension,
        list(allowed_dimensions),
    )
    return fallback_dimension


def _guess_image_mime_type(image_bytes: bytes) -> str:
    """从图像字节头部推断 MIME（多用途互联网邮件扩展）类型。"""

    if image_bytes.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    if image_bytes.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    if image_bytes.startswith((b"GIF87a", b"GIF89a")):
        return "image/gif"
    if image_bytes.startswith(b"BM"):
        return "image/bmp"
    if image_bytes.startswith((b"II*\x00", b"MM\x00*")):
        return "image/tiff"
    if image_bytes[:4] == b"RIFF" and image_bytes[8:12] == b"WEBP":
        return "image/webp"
    return "image/jpeg"


class _DashScopeEmbeddingBase(VectorCodecMixin):
    """DashScope 多模态向量公共调用逻辑。"""

    def __init__(
        self,
        *,
        model_name: str,
        dimension: Optional[int] = None,
        api_key: str = "",
        base_url: str = "https://dashscope.aliyuncs.com/api/v1",
        timeout_sec: float = 20.0,
    ) -> None:
        self.model_name = _normalize_dashscope_model_name(model_name)
        self.dimension = _resolve_dashscope_dimension(self.model_name, dimension)
        self._api_key = str(api_key or "").strip()
        if not self._api_key:
            raise RuntimeError(
                "当前 DashScope 多模态向量模型需要配置 API Key。"
                "请设置 `NUWAX_MEMORY_EMBEDDING_API_KEY` 或 `DASHSCOPE_API_KEY`。"
            )
        self._endpoint_url = _resolve_dashscope_endpoint(base_url)
        self._timeout_sec = max(1.0, float(timeout_sec))
        self._supports_custom_dimension = self._infer_dimension_support(self.model_name)
        self._client = httpx.Client(
            timeout=httpx.Timeout(self._timeout_sec),
            headers={
                "Authorization": "Bearer %s" % self._api_key,
                "Content-Type": "application/json",
            },
        )

    def _infer_dimension_support(self, model_name: str) -> bool:
        lowered = model_name.lower()
        return lowered.startswith("qwen3-vl-embedding") or lowered.startswith("qwen2.5-vl-embedding") or lowered.endswith("-2026-03-06")

    def _embed_contents(self, contents: List[dict]) -> np.ndarray:
        payload = {
            "model": self.model_name,
            "input": {"contents": contents},
        }
        parameters = {}
        if self.dimension > 0 and self._supports_custom_dimension:
            parameters["dimension"] = self.dimension
        if parameters:
            payload["parameters"] = parameters
        try:
            response = self._client.post(self._endpoint_url, json=payload)
            response.raise_for_status()
        except httpx.HTTPError as exc:
            status_code = None
            body_code = ""
            body_message = ""
            response = getattr(exc, "response", None)
            if response is not None:
                status_code = response.status_code
                try:
                    body = response.json()
                except ValueError:
                    body_message = response.text.strip()
                else:
                    body_code = str(body.get("code") or "").strip()
                    body_message = str(body.get("message") or body.get("msg") or "").strip()
            raise RuntimeError(
                "调用 DashScope 多模态向量接口失败。"
                f" model={self.model_name}"
                f" endpoint={self._endpoint_url}"
                f" status={status_code}"
                f" body_code={body_code}"
                f" body_message={body_message}"
                f" error={exc}"
            ) from exc

        try:
            body = response.json()
        except ValueError as exc:
            raise RuntimeError("DashScope 多模态向量接口返回了无法解析的 JSON。") from exc

        code = str(body.get("code") or "").strip()
        if code:
            raise RuntimeError(
                "DashScope 多模态向量接口返回业务错误。"
                f" code={code}"
                f" message={body.get('message') or body.get('msg') or ''}"
            )

        output = body.get("output") or {}
        embeddings = output.get("embeddings") or body.get("data") or []
        if not embeddings:
            raise RuntimeError(
                "DashScope 多模态向量接口未返回 embedding。"
                f" body_keys={sorted(body.keys())}"
            )
        vector = embeddings[0].get("embedding")
        if vector is None:
            raise RuntimeError("DashScope 多模态向量接口返回缺少 embedding 字段。")
        result = self._normalize_vector(np.asarray(vector, dtype=np.float32))
        if self.dimension <= 0:
            self.dimension = int(result.shape[0])
        elif int(result.shape[0]) != self.dimension:
            raise ValueError(
                "DashScope 返回的向量维度与配置不一致："
                f"{int(result.shape[0])} != {self.dimension}"
            )
        return result

    def _build_image_data_uri(self, image_bytes: bytes) -> str:
        mime_type = _guess_image_mime_type(image_bytes)
        encoded = base64.b64encode(image_bytes).decode("utf-8")
        return "data:%s;base64,%s" % (mime_type, encoded)


class DashScopeTextEmbedder(_DashScopeEmbeddingBase, TextEmbedder):
    """基于 DashScope 多模态向量接口的文本向量器。"""

    def embed_text(self, text: str) -> np.ndarray:
        return self._embed_contents([{"text": str(text or "")}])

    def embed_fused_text_image(self, *, text: str, image_bytes: bytes) -> np.ndarray:
        return self._embed_contents(
            [
                {
                    "text": str(text or ""),
                    "image": self._build_image_data_uri(image_bytes),
                }
            ]
        )


class DashScopeVisionLanguageEmbedder(_DashScopeEmbeddingBase, VisionLanguageEmbedder):
    """基于 DashScope 多模态向量接口的图文共享空间向量器。"""

    def embed_image_bytes(self, image_bytes: bytes) -> np.ndarray:
        return self._embed_contents([{"image": self._build_image_data_uri(image_bytes)}])

    def embed_text(self, text: str) -> np.ndarray:
        return self._embed_contents([{"text": str(text or "")}])

    def embed_fused_text_image(self, *, text: str, image_bytes: bytes) -> np.ndarray:
        return self._embed_contents(
            [
                {
                    "text": str(text or ""),
                    "image": self._build_image_data_uri(image_bytes),
                }
            ]
        )


class SentenceTransformerTextEmbedder(TextEmbedder):
    """基于 sentence-transformers 的文本向量化器，可直接加载 BGE。"""

    def __init__(
        self,
        *,
        model_name: str,
        dimension: Optional[int] = None,
        device: Optional[str] = None,
    ) -> None:
        try:
            from sentence_transformers import SentenceTransformer
        except ImportError as exc:
            raise RuntimeError(
                "当前环境未安装 sentence-transformers，无法启用 BGE 文本向量化。"
            ) from exc

        resolved_model_name = self._resolve_model_name(model_name)
        self.model_name = model_name
        try:
            self._model = SentenceTransformer(
                resolved_model_name,
                device=device,
                local_files_only=True,
            )
        except Exception as exc:
            raise _build_manual_download_error(
                model_kind="文本向量模型",
                requested_model_name=model_name,
                resolved_model_name=resolved_model_name,
                error=exc,
            ) from exc
        probe_vector = self._model.encode(
            ["向量维度探针"],
            normalize_embeddings=True,
            convert_to_numpy=True,
        )[0]
        self.dimension = int(np.asarray(probe_vector).shape[0])
        if dimension is not None and int(dimension) != self.dimension:
            raise ValueError(
                "配置的文本向量维度与模型实际维度不一致："
                f"{dimension} != {self.dimension}"
            )

    def embed_text(self, text: str) -> np.ndarray:
        encoded = self._model.encode(
            [str(text or "")],
            normalize_embeddings=True,
            convert_to_numpy=True,
        )[0]
        return self._normalize_vector(np.asarray(encoded, dtype=np.float32))

    def _resolve_model_name(self, model_name: str) -> str:
        normalized = _strip_backend_prefix(model_name, "sentence-transformers:")
        return _ensure_local_model_directory(normalized)


class CLIPVisionLanguageEmbedder(VisionLanguageEmbedder):
    """基于 transformers CLIP 的图文共享空间向量化器。"""

    def __init__(
        self,
        *,
        model_name: str,
        dimension: Optional[int] = None,
        device: Optional[str] = None,
    ) -> None:
        try:
            from transformers import CLIPModel, CLIPProcessor
        except ImportError as exc:
            raise RuntimeError(
                "当前环境未安装 transformers，无法启用 CLIP 多模态向量化。"
            ) from exc
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError("当前环境未安装 torch，无法启用 CLIP 多模态向量化。") from exc

        self._torch = torch
        resolved_model_name = self._resolve_model_name(model_name)
        self.model_name = model_name
        try:
            self._processor = CLIPProcessor.from_pretrained(
                resolved_model_name,
                local_files_only=True,
            )
            self._model = CLIPModel.from_pretrained(
                resolved_model_name,
                local_files_only=True,
            )
        except Exception as exc:
            raise _build_manual_download_error(
                model_kind="图像向量模型",
                requested_model_name=model_name,
                resolved_model_name=resolved_model_name,
                error=exc,
            ) from exc
        self._device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self._model.to(self._device)
        self._model.eval()
        self.dimension = int(self._model.config.projection_dim)
        if dimension is not None and int(dimension) != self.dimension:
            raise ValueError(
                "配置的图像向量维度与模型实际维度不一致："
                f"{dimension} != {self.dimension}"
            )

    def embed_image_bytes(self, image_bytes: bytes) -> np.ndarray:
        from PIL import Image

        image = Image.open(BytesIO(image_bytes)).convert("RGB")
        inputs = self._processor(images=image, return_tensors="pt")
        inputs = {key: value.to(self._device) for key, value in inputs.items()}
        with self._torch.no_grad():
            vector = self._model.get_image_features(**inputs)[0].detach().cpu().numpy()
        return self._normalize_vector(np.asarray(vector, dtype=np.float32))

    def embed_text(self, text: str) -> np.ndarray:
        inputs = self._processor(text=[str(text or "")], return_tensors="pt", padding=True)
        inputs = {key: value.to(self._device) for key, value in inputs.items()}
        with self._torch.no_grad():
            vector = self._model.get_text_features(**inputs)[0].detach().cpu().numpy()
        return self._normalize_vector(np.asarray(vector, dtype=np.float32))

    def _resolve_model_name(self, model_name: str) -> str:
        normalized = _strip_backend_prefix(model_name, "transformers-clip:")
        return _ensure_local_model_directory(normalized)


def build_text_embedder(
    model_name: str,
    dimension: int,
    *,
    api_key: str = "",
    base_url: str = "https://dashscope.aliyuncs.com/api/v1",
    timeout_sec: float = 20.0,
) -> TextEmbedder:
    """按配置构造文本向量化器。"""

    normalized = str(model_name or "").strip()
    spec = resolve_text_embedding_provider_spec(normalized)
    if spec.provider_name == "hashing":
        return HashingTextEmbedder(model_name=spec.model_name or "hashing-v1", dimension=dimension)
    if spec.provider_name == "dashscope":
        return DashScopeTextEmbedder(
            model_name=spec.model_name,
            dimension=dimension,
            api_key=api_key,
            base_url=base_url,
            timeout_sec=timeout_sec,
        )
    if spec.provider_name == "sentence_transformers":
        return SentenceTransformerTextEmbedder(model_name=spec.model_name, dimension=dimension)
    raise ValueError(f"不支持的文本向量化模型：{model_name}")


def build_vision_language_embedder(
    model_name: str,
    dimension: int,
    *,
    api_key: str = "",
    base_url: str = "https://dashscope.aliyuncs.com/api/v1",
    timeout_sec: float = 20.0,
) -> Optional[VisionLanguageEmbedder]:
    """按配置构造图文共享空间向量化器。"""

    normalized = str(model_name or "").strip()
    spec = resolve_vision_language_provider_spec(normalized)
    if not spec.is_enabled():
        return None
    if spec.provider_name == "dashscope":
        return DashScopeVisionLanguageEmbedder(
            model_name=spec.model_name,
            dimension=dimension,
            api_key=api_key,
            base_url=base_url,
            timeout_sec=timeout_sec,
        )
    if spec.provider_name == "clip":
        return CLIPVisionLanguageEmbedder(model_name=spec.model_name, dimension=dimension)
    raise ValueError(f"不支持的图文向量化模型：{model_name}")

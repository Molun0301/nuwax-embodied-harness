"""场景类别决策的有界输入与保留分数语义的结果。"""
from __future__ import annotations

import base64
import json
import math
import re
from typing import Any, Dict, List, Literal

from pydantic import Field, field_validator, model_validator

from contracts.base import ContractModel


MAX_REQUEST_BYTES = 12 * 1024 * 1024
MAX_IMAGE_BYTES = 2 * 1024 * 1024
MAX_STATE_BYTES = 64 * 1024
_IMAGE_PREFIX = re.compile(r"^data:image/(png|jpeg|webp);base64,")


class SceneDecisionRequest(ContractModel):
    """图片只接受内联字节，文件路径与外部地址不属于公共输入。"""

    state: Dict[str, Any]
    categories: Dict[str, str]
    images: List[str] = Field(default_factory=list)

    @field_validator("state")
    @classmethod
    def _bounded_state(cls, value: Dict[str, Any]) -> Dict[str, Any]:
        pending = [(value, 0)]
        while pending:
            item, depth = pending.pop()
            if depth > 16:
                raise ValueError("场景事实最多嵌套16层。")
            if isinstance(item, dict):
                pending.extend((child, depth + 1) for child in item.values())
            elif isinstance(item, (list, tuple)):
                pending.extend((child, depth + 1) for child in item)
        try:
            encoded = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
        except (TypeError, ValueError) as exc:
            raise ValueError("场景事实须为有限数值及可编码的JSON数据。") from exc
        if len(encoded) > MAX_STATE_BYTES:
            raise ValueError("场景事实最多64KiB。")
        return value

    @field_validator("categories")
    @classmethod
    def _bounded_categories(cls, value: Dict[str, str]) -> Dict[str, str]:
        if not 2 <= len(value) <= 32 or "unknown" not in value:
            raise ValueError("场景类别须包含unknown，且共2至32类。")
        for key, description in value.items():
            if not key.strip() or key != key.strip() or len(key) > 80:
                raise ValueError("类别标识须为1至80个字符且无首尾空白。")
            if not description.strip() or len(description) > 1000:
                raise ValueError("每个类别须有1至1000个字符的说明。")
        return value

    @field_validator("images")
    @classmethod
    def _bounded_images(cls, value: List[str]) -> List[str]:
        if len(value) > 8:
            raise ValueError("每次最多8张图像。")
        for item in value:
            match = _IMAGE_PREFIX.match(item)
            if match is None:
                raise ValueError("图像须为PNG/JPEG/WebP的base64数据地址。")
            payload = item[match.end():]
            if len(payload) > ((MAX_IMAGE_BYTES + 2) // 3) * 4:
                raise ValueError("每张图像解码后最多2MiB。")
            try:
                decoded = base64.b64decode(payload, validate=True)
            except (ValueError, base64.binascii.Error) as exc:
                raise ValueError("图像base64编码无效。") from exc
            if not decoded or len(decoded) > MAX_IMAGE_BYTES:
                raise ValueError("图像内容为空或超过2MiB。")
        return value

    @model_validator(mode="after")
    def _bounded_request(self) -> "SceneDecisionRequest":
        if len(self.model_dump_json().encode("utf-8")) > MAX_REQUEST_BYTES:
            raise ValueError("决策请求最多12MiB。")
        return self


class SceneDecisionResult(ContractModel):
    """自然语言蕴含分数逐类独立，不声明为房间类别概率。"""

    category_scores: Dict[str, float]
    score_kind: Literal["choice_probability", "nli_entailment"]
    model_id: str = Field(min_length=1, max_length=512)
    raw: Dict[str, Any] = Field(default_factory=dict)
    metadata: Dict[str, Any] = Field(default_factory=dict)

    @field_validator("category_scores", mode="before")
    @classmethod
    def _valid_scores(cls, value: Any) -> Dict[str, float]:
        if not isinstance(value, dict) or not 2 <= len(value) <= 32 or "unknown" not in value:
            raise ValueError("结果须包含全部类别及unknown。")
        for score in value.values():
            if isinstance(score, bool) or not isinstance(score, (int, float)):
                raise ValueError("类别分数须为数值。")
            if not math.isfinite(score) or not 0.0 <= score <= 1.0:
                raise ValueError("类别分数须为0至1的有限数值。")
        return value

    @model_validator(mode="after")
    def _valid_distribution(self) -> "SceneDecisionResult":
        if not self.model_id.strip():
            raise ValueError("实际模型标识不能为空。")
        json.dumps(self.raw, ensure_ascii=False, allow_nan=False)
        json.dumps(self.metadata, ensure_ascii=False, allow_nan=False)
        if self.score_kind == "choice_probability" and not math.isclose(
            sum(self.category_scores.values()), 1.0, rel_tol=0.0, abs_tol=1e-4
        ):
            raise ValueError("类别选择概率之和须为1，不自动归一化非法输出。")
        return self

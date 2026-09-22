"""Jev及本地OpenJev的同步HTTP适配，不依赖模型运行库。"""
from __future__ import annotations

import json
import http.client
import math
import socket
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict

from pydantic import ValidationError

from contracts.scene_decision import SceneDecisionRequest, SceneDecisionResult
from core.errors import GatewayError


class SceneDecisionError(GatewayError):
    """可交由现有网关错误处理的决策失败。"""


class _RejectRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise SceneDecisionError("模型服务不允许重定向。", error_code="scene_decision_redirect", http_status=502)


class HTTPSceneDecisionBackend:
    """每个调用使用独立连接；关闭后拒绝新请求。"""

    def __init__(self, *, backend: str, base_url: str, model: str, api_key: str, timeout_sec: float) -> None:
        try:
            parsed = urllib.parse.urlsplit(base_url.strip())
            parsed.port
        except ValueError as exc:
            raise SceneDecisionError("模型服务地址无效。", error_code="scene_decision_invalid_config") from exc
        if parsed.scheme not in {"http", "https"} or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise SceneDecisionError("模型服务地址须为不含凭据、查询及片段的HTTP地址。", error_code="scene_decision_invalid_config")
        path = parsed.path.rstrip("/")
        if path not in {"", "/v1"}:
            raise SceneDecisionError("模型服务基础地址只接受根路径或/v1。", error_code="scene_decision_invalid_config")
        if not model.strip() or not math.isfinite(timeout_sec) or not 0 < timeout_sec <= 600:
            raise SceneDecisionError("须配置模型标识及0至600秒内的超时。", error_code="scene_decision_invalid_config")
        if backend == "jev_api" and not api_key.strip():
            raise SceneDecisionError("Jev接口尚未配置密钥。", error_code="scene_decision_credentials_missing", http_status=503)
        self._backend = backend
        self._base_url = urllib.parse.urlunsplit((parsed.scheme, parsed.netloc, path, "", ""))
        self._endpoint = self._base_url + ("" if path == "/v1" else "/v1") + ("/systemone" if backend == "jev_api" else "/scene-decisions")
        self._model = model.strip()
        self._api_key = api_key.strip()
        self._timeout_sec = float(timeout_sec)
        self._closed = threading.Event()

    def describe(self) -> Dict[str, Any]:
        return {"backend": self._backend, "base_url": self._base_url, "model_id": self._model,
                "supports_images": self._backend == "openjev_http", "timeout_sec": self._timeout_sec,
                "closed": self._closed.is_set()}

    def close(self) -> None:
        self._closed.set()

    def decide(self, request: SceneDecisionRequest) -> SceneDecisionResult:
        if self._closed.is_set():
            raise SceneDecisionError("决策后端已经关闭。", error_code="scene_decision_closed", http_status=503)
        # 重新验证防止调用者通过model_copy绕过边界校验。
        request = SceneDecisionRequest.model_validate(request.model_dump())
        if self._backend == "jev_api":
            if request.images:
                raise SceneDecisionError("当前Jev适配只支持文本事实，不支持直接图像输入。", error_code="scene_decision_images_unsupported", http_status=422)
            payload = {"model": self._model, "state": request.state, "questions": {"room_category": {
                "type": "choice", "instructions": "Which room category is supported by the supplied observations? Choose unknown when the evidence is insufficient or conflicting.",
                "criteria": request.categories}}}
        else:
            payload = request.model_dump(mode="json")
        started = time.monotonic()
        response = self._post(payload)
        try:
            if self._backend == "jev_api":
                answer = response["answers"]["room_category"]
                if answer.get("type") != "choice" or answer.get("choice") not in request.categories:
                    raise ValueError("不是合法类别选择结果")
                confidence = answer.get("confidence")
                if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
                    raise ValueError("选择置信度无效")
                result = SceneDecisionResult(category_scores=answer["probabilities"], score_kind="choice_probability",
                                             model_id=response["model"], raw=response,
                                             metadata={"backend": self._backend})
                if result.category_scores[answer["choice"]] < max(result.category_scores.values()) - 1e-6:
                    raise ValueError("选择项不是最大概率项")
            else:
                result = SceneDecisionResult.model_validate(response)
                if result.score_kind != "nli_entailment":
                    raise ValueError("本地模型未返回自然语言蕴含分数")
                if result.model_id != self._model:
                    raise SceneDecisionError("本地服务模型与配置不一致。", error_code="scene_decision_model_mismatch", http_status=502)
            if set(result.category_scores) != set(request.categories):
                raise ValueError("返回类别与请求不一致")
        except (KeyError, TypeError, ValueError, ValidationError) as exc:
            raise SceneDecisionError("模型服务返回的分数或结果结构无效。", error_code="scene_decision_invalid_response", http_status=502) from exc
        result.metadata["request_duration_ms"] = round((time.monotonic() - started) * 1000, 3)
        return result

    def _post(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        headers = {"Content-Type": "application/json", "Accept": "application/json"}
        if self._api_key:
            headers["Authorization"] = "Bearer " + self._api_key
        request = urllib.request.Request(self._endpoint, data=json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8"), headers=headers, method="POST")
        deadline = time.monotonic() + self._timeout_sec
        try:
            # 禁止代理环境变量将本地模型请求送往其它主机。
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _RejectRedirect())
            with opener.open(request, timeout=self._timeout_sec) as response:
                body = bytearray()
                while len(body) <= 2 * 1024 * 1024:
                    if time.monotonic() >= deadline:
                        raise TimeoutError("请求总时间超过期限")
                    chunk = response.read1(min(65536, 2 * 1024 * 1024 + 1 - len(body)))
                    if not chunk:
                        break
                    body.extend(chunk)
                if time.monotonic() >= deadline:
                    raise TimeoutError("请求总时间超过期限")
            if len(body) > 2 * 1024 * 1024:
                raise SceneDecisionError("模型响应超过2MiB。", error_code="scene_decision_response_too_large", http_status=502)
            result = json.loads(body)
            if not isinstance(result, dict):
                raise ValueError("结果不是对象")
            return result
        except urllib.error.HTTPError as exc:
            # 不转发服务返回正文，避免在平台日志中泄漏认证或输入内容。
            status = int(exc.code)
            local_code = None
            if self._backend == "openjev_http":
                try:
                    error_body = json.loads(exc.read1(4096))
                    candidate = error_body.get("error") if isinstance(error_body, dict) else None
                    if candidate in {"scene_decision_not_ready", "scene_decision_busy", "scene_decision_timeout",
                                     "scene_decision_closed", "scene_decision_invalid_input", "scene_decision_inference_failed"}:
                        local_code = candidate
                except (ValueError, UnicodeError, OSError):
                    pass
            exc.close()
            raise SceneDecisionError("模型接口返回HTTP错误。", error_code=local_code or "scene_decision_http_error",
                                     http_status=status if local_code else 502,
                                     details={"upstream_status": status}) from exc
        except (socket.timeout, TimeoutError) as exc:
            raise SceneDecisionError("模型请求超时。", error_code="scene_decision_timeout", http_status=504) from exc
        except urllib.error.URLError as exc:
            timed_out = isinstance(exc.reason, (socket.timeout, TimeoutError))
            raise SceneDecisionError("模型请求超时。" if timed_out else "无法连接模型服务。",
                                     error_code="scene_decision_timeout" if timed_out else "scene_decision_unavailable",
                                     http_status=504 if timed_out else 503) from exc
        except (OSError, http.client.HTTPException) as exc:
            raise SceneDecisionError("模型连接中断或响应不完整。", error_code="scene_decision_unavailable", http_status=503) from exc
        except (ValueError, UnicodeError) as exc:
            raise SceneDecisionError("模型接口返回无效JSON。", error_code="scene_decision_invalid_response", http_status=502) from exc


def create_scene_decision_backend(*, backend: str, base_url: str, model: str, api_key: str = "", timeout_sec: float = 30.0) -> HTTPSceneDecisionBackend:
    if backend not in {"jev_api", "openjev_http"}:
        raise SceneDecisionError("未知场景决策后端。", error_code="scene_decision_invalid_config")
    return HTTPSceneDecisionBackend(backend=backend, base_url=base_url, model=model, api_key=api_key, timeout_sec=timeout_sec)

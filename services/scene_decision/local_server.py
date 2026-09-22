"""有界本机推理服务；超时终止独立模型进程，不遗留后台推理。"""
from __future__ import annotations

import asyncio
import hmac
import multiprocessing
import queue
import threading
from contextlib import asynccontextmanager
from typing import Any, Dict

from contracts.scene_decision import MAX_REQUEST_BYTES, SceneDecisionRequest
from services.scene_decision.backends import SceneDecisionError
from services.scene_decision.local_model import MODEL_ID


def _model_worker(requests, results, model_options) -> None:
    from services.scene_decision.local_model import LocalModelInputError, OpenJevModel

    try:
        model = OpenJevModel(**model_options)
        results.put({"ready": True, "metrics": model.metrics})
    except Exception as exc:
        results.put({"ready": False, "error_type": type(exc).__name__})
        raise
    while True:
        payload = requests.get()
        if payload is None:
            return
        try:
            request = SceneDecisionRequest.model_validate(payload)
            result = model.decide(request)
            results.put({"result": result.model_dump(mode="json")})
        except LocalModelInputError as exc:
            results.put({"error": "scene_decision_invalid_input", "message": str(exc), "status": 422})
        except Exception as exc:
            results.put({"error": "scene_decision_inference_failed", "message": "本地模型推理失败。", "status": 503,
                         "error_type": type(exc).__name__})


class ModelProcess:
    """仅允许一个在途推理；等待队列不堆积图像请求。"""

    def __init__(self, *, model_options: Dict[str, Any], timeout_sec: float = 60.0, load_timeout_sec: float = 180.0) -> None:
        self._ctx = multiprocessing.get_context("spawn")
        self._requests = self._ctx.Queue(maxsize=1)
        self._results = self._ctx.Queue(maxsize=1)
        self._process = self._ctx.Process(target=_model_worker, args=(self._requests, self._results, model_options), daemon=True)
        self._lock = threading.Lock()
        self._status = "not_started"
        self._metrics = {}
        self._error_type = None
        self._timeout_sec = timeout_sec
        self._load_timeout_sec = load_timeout_sec
        self._loader = None

    def start(self) -> None:
        self._status = "loading"
        self._process.start()
        self._loader = threading.Thread(target=self._await_load, daemon=True)
        self._loader.start()

    def _await_load(self) -> None:
        try:
            loaded = self._results.get(timeout=self._load_timeout_sec)
            if self._status == "closed":
                return
            self._metrics = loaded.get("metrics", {})
            self._error_type = loaded.get("error_type")
            self._status = "ready" if loaded.get("ready") else "failed"
        except (queue.Empty, OSError, ValueError):
            if self._status != "closed":
                self._status = "failed"
                self._error_type = "ModelLoadTimeout"
                self._terminate()

    def health(self) -> Dict[str, Any]:
        if self._status == "ready" and not self._process.is_alive():
            self._status = "failed"
            self._error_type = "ModelProcessExited"
        return {"status": self._status, "ready": self._status == "ready", "model_id": MODEL_ID,
                "busy": self._lock.locked(), "metrics": dict(self._metrics), "error_type": self._error_type}

    def decide(self, request: SceneDecisionRequest) -> Dict[str, Any]:
        if not self.health()["ready"]:
            raise SceneDecisionError("模型尚未就绪。", error_code="scene_decision_not_ready", http_status=503)
        if not self._lock.acquire(blocking=False):
            raise SceneDecisionError("模型正在处理另一请求。", error_code="scene_decision_busy", http_status=429)
        try:
            self._requests.put_nowait(request.model_dump(mode="json"))
            try:
                response = self._results.get(timeout=self._timeout_sec)
            except queue.Empty as exc:
                self._status = "failed"
                self._error_type = "InferenceTimeout"
                self._terminate()
                raise SceneDecisionError("推理超过期限，模型进程已终止；请重启服务。", error_code="scene_decision_timeout", http_status=504) from exc
            except (OSError, ValueError, EOFError) as exc:
                raise SceneDecisionError("模型进程已经关闭。", error_code="scene_decision_closed", http_status=503) from exc
            if self._status == "closed":
                raise SceneDecisionError("模型进程已经关闭。", error_code="scene_decision_closed", http_status=503)
            if "error" in response:
                raise SceneDecisionError(response["message"], error_code=response["error"], http_status=response["status"])
            return response["result"]
        finally:
            self._lock.release()

    def _terminate(self) -> None:
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.kill()
            self._process.join(timeout=3.0)

    def close(self) -> None:
        self._status = "closed"
        self._terminate()
        # 唤醒加载等待线程，关闭后不保留一分钟以上的加载等待。
        try:
            self._results.put_nowait({"ready": False})
        except (queue.Full, ValueError):
            pass
        if self._loader:
            self._loader.join(timeout=1.0)
        for channel in (self._requests, self._results):
            channel.cancel_join_thread()
            channel.close()


def create_app(controller: ModelProcess, *, token: str = ""):
    from fastapi import FastAPI, Request
    from fastapi.responses import JSONResponse
    from pydantic import ValidationError

    @asynccontextmanager
    async def lifespan(app):
        controller.start()
        try:
            yield
        finally:
            await asyncio.to_thread(controller.close)

    app = FastAPI(title="场景决策本地服务", docs_url=None, redoc_url=None, openapi_url=None, lifespan=lifespan)

    @app.get("/health")
    async def health():
        value = controller.health()
        return JSONResponse(value, status_code=200 if value["ready"] else 503)

    # Request必须可由FastAPI解析，避免延迟注解引用函数局部导入。
    async def decision(request):
        if token and not hmac.compare_digest(request.headers.get("authorization", "").encode("utf-8"), ("Bearer " + token).encode("utf-8")):
            return JSONResponse({"error": "unauthorized"}, status_code=401)
        try:
            declared = request.headers.get("content-length")
            if declared is not None and (int(declared) < 0 or int(declared) > MAX_REQUEST_BYTES):
                return JSONResponse({"error": "request_too_large"}, status_code=413)
            if not request.headers.get("content-type", "").lower().startswith("application/json"):
                return JSONResponse({"error": "content_type_required"}, status_code=415)

            async def read_body():
                body = bytearray()
                async for chunk in request.stream():
                    if len(body) + len(chunk) > MAX_REQUEST_BYTES:
                        raise SceneDecisionError("请求超过12MiB。", error_code="request_too_large", http_status=413)
                    body.extend(chunk)
                return body

            body = await asyncio.wait_for(read_body(), timeout=10.0)
            parsed = SceneDecisionRequest.model_validate_json(body)
            result = await asyncio.to_thread(controller.decide, parsed)
            return JSONResponse(result)
        except asyncio.TimeoutError:
            return JSONResponse({"error": "request_read_timeout"}, status_code=408)
        except (ValidationError, ValueError):
            return JSONResponse({"error": "invalid_request"}, status_code=422)
        except RecursionError:
            return JSONResponse({"error": "request_nesting_too_deep"}, status_code=422)
        except SceneDecisionError as exc:
            return JSONResponse({"error": exc.error_code, "message": exc.message}, status_code=exc.http_status)

    decision.__annotations__["request"] = Request
    app.post("/v1/scene-decisions")(decision)
    return app

"""用本地 BGE-M3 和 YOLO-World 权重，在独立进程比较冷启动后两次查询。"""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

from services.memory.embeddings import Embeddings
from services.perception.detectors import YoloWorldDetectorBackend


def main(model, weights):
    root_pending = threading.Event()
    transforms_pending = threading.Event()
    calls = []

    def trace(frame, event, arg):
        if event != "call":
            return None
        name = frame.f_globals.get("__name__")
        if frame.f_code.co_name != "<module>":
            return None
        if name == "torchvision":
            calls.append([threading.current_thread().name, name])
            if threading.current_thread().name == "perception-runtime":
                root_pending.set()
                transforms_pending.wait(10)
        if name == "torchvision.transforms.functional":
            calls.append([threading.current_thread().name, name])
            if threading.current_thread().name == "memory-embeddings-query":
                transforms_pending.set()
        return None

    def detect():
        threading.current_thread().name = "perception-runtime"
        sys.settrace(trace)
        try:
            detector = YoloWorldDetectorBackend(weights=str(weights), device="cpu")
            return type(detector._get_model()).__name__
        finally:
            sys.settrace(None)

    def query():
        threading.current_thread().name = "memory-embeddings-query"
        sys.settrace(trace)
        try:
            return dimension()
        finally:
            sys.settrace(None)

    def dimension():
        vectors = embeddings.query("苹果在哪", snapshot)
        return vectors["text"][1].size if "text" in vectors else 0

    service = SimpleNamespace(
        _embedding_model=str(model),
        _embedding_dimension=1024,
        _image_embedding_model="",
        _image_embedding_dimension=512,
        _embedding_api_key="",
        _embedding_base_url="",
        _embedding_timeout_sec=20,
    )
    snapshot = SimpleNamespace(vectors={})
    embeddings = Embeddings(service)
    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            vision = executor.submit(detect)
            assert root_pending.wait(30), "感知没有进入 torchvision 根模块"
            text = executor.submit(query)
            first_dimension = text.result()
            try:
                vision_result = vision.result()
            except Exception as error:
                vision_result = repr(error)
        second_dimension = dimension()
        print(json.dumps({
            "imports": calls,
            "first_dimension": first_dimension,
            "second_dimension": second_dimension,
            "embedding_error": embeddings.initialization_error,
            "vision_result": vision_result,
        }, ensure_ascii=False))
        assert first_dimension == second_dimension == 1024
        assert vision_result == "YOLOWorld"
        assert embeddings.initialization_error is None
    finally:
        embeddings.close()


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))

"""内存向量队列；模型运算在线程中完成，结果交唯一写线程落库。"""
from __future__ import annotations
import logging
import queue
import threading
from services.memory.store import MODELS
from services.memory.vectorizer import build_text_embedder, build_vision_language_embedder

LOGGER = logging.getLogger(__name__)


class Embeddings:
    def __init__(self, service, *, text_embedder=None, image_embedder=None):
        self.service = service
        self.text = text_embedder
        self.image = image_embedder
        self._initialized = False
        self._initialization_error = None
        self._queue = queue.Queue()
        self._pending = set()
        self._lock = threading.Lock()
        self._model_lock = threading.RLock()
        self.error = None
        self._thread = threading.Thread(target=self._run, name="memory-embeddings")
        self._thread.start()

    def enqueue(self, library, records):
        for kind, identity in records:
            if kind not in {"location", "memory", "instance", "observation", "scene_space"}:
                continue
            key = (library, kind, identity)
            with self._lock:
                if key in self._pending:
                    continue
                self._pending.add(key)
            self._queue.put(key)

    def missing(self, snapshot):
        expected = []
        if self.text is not None or self.service._embedding_model != "disabled":
            expected.append("text")
        if self.image is not None or self.service._image_embedding_model != "disabled":
            expected.append("image")
        for kind in ("location", "memory", "instance", "observation", "scene_space"):
            for record in snapshot.read(kind):
                identity = getattr(record, MODELS[kind][1])
                if any((kind, identity, name) not in snapshot.vectors
                       for name in expected if name != "image" or self.service._record_photos(record)):
                    self.enqueue(snapshot.name, [(kind, identity)])

    def initialize(self):
        if self._initialization_error is not None:
            raise self._initialization_error
        if self._initialized:
            return
        service = self.service
        with self._model_lock:
            if self.text is None:
                self.text = build_text_embedder(service._embedding_model, service._embedding_dimension,
                    api_key=service._embedding_api_key, base_url=service._embedding_base_url,
                    timeout_sec=service._embedding_timeout_sec)
            if self.image is None:
                self.image = build_vision_language_embedder(service._image_embedding_model,
                    service._image_embedding_dimension, api_key=service._embedding_api_key,
                    base_url=service._embedding_base_url, timeout_sec=service._embedding_timeout_sec)
            self._initialized = True
            service._association.set_text_embedder(self.text)

    def query(self, text):
        if not self._model_lock.acquire(blocking=False):
            return {}
        try:
            return ({name: (provider, provider.embed_text(text))
                     for name, provider in (("text", self.text), ("image", self.image))
                     if provider is not None} if self._initialized else {})
        finally:
            self._model_lock.release()

    def _run(self):
        while True:
            key = self._queue.get()
            if key is None:
                return
            with self._lock:
                self._pending.discard(key)
            name, kind, identity = key
            snapshot = self.service.snapshot()
            if snapshot.name != name:
                continue
            record = next((record for record in snapshot.read(kind)
                           if getattr(record, MODELS[kind][1]) == identity), None)
            if record is None:
                continue
            try:
                self.initialize()
                with self._model_lock:
                    vectors = []
                    text = self.service._record_text(kind, record)
                    if self.text is not None:
                        vectors.append(("text", self.text.model_name, self.text.embed_text(text)))
                    if self.image is not None and self.service._record_photos(record):
                        data = self.service.read_photo(self.service._record_photos(record)[0], library_name=name)
                        vectors.append(("image", self.image.model_name, self.image.embed_image_bytes(data)))
                def save(store):
                    if store is not None and store.name == name and store.has(kind, identity):
                        for vector_name, model, vector in vectors:
                            store.put_vector(kind, identity, vector_name, model, vector)
                self.service._writer.call(save)
                self.error = None
            except Exception as error:
                if not self._initialized:
                    self._initialization_error = error
                self.error = str(error)
                LOGGER.exception("记忆向量计算失败")

    def close(self):
        self._queue.put(None)
        self._thread.join()

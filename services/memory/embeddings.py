"""内存向量队列；模型运算在线程中完成，结果交唯一写线程落库。"""
from __future__ import annotations
import logging
import queue
import threading
from services.memory.store import MODELS
from services.memory.vectorizer import build_text_embedder, build_vision_language_embedder, embedding_model_identity
from services.model_providers import resolve_text_embedding_provider_spec, resolve_vision_language_provider_spec

LOGGER = logging.getLogger(__name__)


class Embeddings:
    def __init__(self, service, *, text_embedder=None, image_embedder=None):
        self.service = service
        self.text = text_embedder
        self.image = image_embedder
        self._configured = {}
        for name, spec, dimension, build in (
            ("text", resolve_text_embedding_provider_spec(service._embedding_model),
             service._embedding_dimension, build_text_embedder),
            ("image", resolve_vision_language_provider_spec(service._image_embedding_model),
             service._image_embedding_dimension, build_vision_language_embedder),
        ):
            if spec.is_enabled() and getattr(self, name) is None:
                model, dimension = embedding_model_identity(spec, dimension)
                self._configured[name] = (build, spec.model_name, model, dimension)
        self._initialization_errors = {}
        self._queue = queue.Queue()
        self._pending = set()
        self._lock = threading.Lock()
        self._model_lock = threading.RLock()
        self._computation_error = None
        self._thread = threading.Thread(target=self._run, name="memory-embeddings")
        self._thread.start()

    def _needed(self, snapshot, kind, record):
        identity = getattr(record,MODELS[kind][1])
        if kind == "instance" and identity in (snapshot.transient or {}):
            return []
        needed = []
        for name, provider in (("text", self.text), ("image", self.image)):
            if name == "image" and not self.service._record_photos(record):
                continue
            if provider is not None:
                model, dimension = provider.model_name, provider.dimension
            elif name in self._configured and name not in self._initialization_errors:
                _, _, model, dimension = self._configured[name]
            else:
                continue
            existing = snapshot.vectors.get((kind, identity, name))
            if existing is None or (existing.model, existing.dimension) != (model, dimension):
                needed.append(name)
        return needed

    def enqueue(self, library, records):
        snapshot = self.service.snapshot()
        if snapshot.name != library:
            return
        for kind,identity in records:
            if kind not in {'location','memory','instance','observation','scene_space'}:
                continue
            record = snapshot.get(kind,identity)
            if record is None or not self._needed(snapshot,kind,record):
                continue
            key = (library,kind,identity)
            with self._lock:
                if key in self._pending:
                    continue
                self._pending.add(key)
            self._queue.put(key)

    def missing(self, snapshot):
        for kind in ('location','memory','instance','observation','scene_space'):
            self.enqueue(snapshot.name, ((kind,identity) for identity in snapshot.model_map(kind)))

    @property
    def initialization_error(self):
        return "；".join(self._initialization_errors.copy().values()) or None

    @property
    def error(self):
        return self._computation_error or self.initialization_error

    def load(self, name):
        with self._model_lock:
            provider = getattr(self, name)
            if provider is not None or name not in self._configured or name in self._initialization_errors:
                return provider
            build, requested_model, _, dimension = self._configured[name]
            service = self.service
            try:
                provider = build(requested_model, dimension, api_key=service._embedding_api_key,
                                 base_url=service._embedding_base_url, timeout_sec=service._embedding_timeout_sec)
                setattr(self, name, provider)
            except Exception as error:
                LOGGER.exception("%s嵌入模型初始化失败，继续使用词法检索", "文本" if name == "text" else "图像")
                self._initialization_errors[name] = str(error)
            return provider

    def query(self, text, snapshot):
        if not self._model_lock.acquire(blocking=False):
            return {}
        try:
            names = ["text"]
            if any(key[2] == "image" for key in snapshot.vectors):
                names.append("image")
            vectors = {}
            for name in names:
                provider = self.load(name)
                if provider is not None:
                    vectors[name] = (provider, provider.embed_text(text))
            return vectors
        finally:
            self._model_lock.release()

    def _run(self):
        while True:
            key = self._queue.get()
            if key is None:
                return
            with self._lock:
                self._pending.discard(key)
            self._process(key)

    def _process(self, key):
        name, kind, identity = key
        snapshot = self.service.snapshot()
        if snapshot.name != name:
            return
        record = snapshot.get(kind,identity)
        if record is None:
            return
        try:
            needed = self._needed(snapshot,kind,record)
            if not needed:
                return
            with self._model_lock:
                vectors = []
                for vector_name in needed:
                    provider = self.load(vector_name)
                    if provider is None:
                        continue
                    if vector_name == "text":
                        text = self.service._record_text(kind, record)
                        vector = provider.embed_text(text)
                    else:
                        data = self.service.read_photo(self.service._record_photos(record)[0], library_name=name)
                        vector = provider.embed_image_bytes(data)
                    vectors.append((vector_name, provider.model_name, vector))
            if not vectors:
                return
            def save(store):
                if store is not None and store.name == name:
                    current = store.get(kind,identity)
                    if current is None or not store.has(kind,identity):
                        return
                    for vector_name,model,vector in vectors:
                        same = (self.service._record_text(kind,current,store) == text if vector_name == 'text'
                                else self.service._record_photos(current) == self.service._record_photos(record))
                        if same:
                            store.put_vector(kind,identity,vector_name,model,vector)
            self.service._writer.call(save)
            self._computation_error = None
        except Exception as error:
            self._computation_error = str(error)
            LOGGER.exception("记忆向量计算失败")

    def close(self):
        self._queue.put(None)
        self._thread.join()

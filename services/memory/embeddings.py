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

    def _needed(self, snapshot, kind, record):
        identity = getattr(record,MODELS[kind][1])
        if kind == "instance" and identity in (snapshot.transient or {}):
            return ()
        return tuple((name,provider) for name,provider in (('text',self.text),('image',self.image))
            if provider is not None and (name != 'image' or self.service._record_photos(record))
            and ((kind,identity,name) not in snapshot.vectors
                 or snapshot.vectors[kind,identity,name].model != provider.model_name
                 or snapshot.vectors[kind,identity,name].dimension != provider.dimension))

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

    def initialize(self):
        if self._initialization_error is not None:
            return False
        if self._initialized:
            return True
        service = self.service
        with self._model_lock:
            try:
                if self.text is None:
                    self.text = build_text_embedder(service._embedding_model,service._embedding_dimension,
                        api_key=service._embedding_api_key,base_url=service._embedding_base_url,
                        timeout_sec=service._embedding_timeout_sec)
                if self.image is None:
                    self.image = build_vision_language_embedder(service._image_embedding_model,service._image_embedding_dimension,
                        api_key=service._embedding_api_key,base_url=service._embedding_base_url,
                        timeout_sec=service._embedding_timeout_sec)
                self._initialized = True
                service._association.set_text_embedder(self.text)
                return True
            except Exception as error:
                self._initialization_error = error
                self.error = str(error)
                self.text = self.image = None
                service._association.set_text_embedder(None)
                LOGGER.exception('嵌入模型初始化失败，本次服务保持词法检索')
                return False

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
            if not self.initialize():
                return
            needed = self._needed(snapshot,kind,record)
            if not needed:
                return
            with self._model_lock:
                vectors = []
                text = self.service._record_text(kind, record)
                if any(name == 'text' for name,_ in needed):
                    vectors.append(("text", self.text.model_name, self.text.embed_text(text)))
                if any(name == 'image' for name,_ in needed):
                    data = self.service.read_photo(self.service._record_photos(record)[0], library_name=name)
                    vectors.append(("image", self.image.model_name, self.image.embed_image_bytes(data)))
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
            self.error = None
        except Exception as error:
            if not self._initialized:
                self._initialization_error = error
            self.error = str(error)
            LOGGER.exception("记忆向量计算失败")

    def close(self):
        self._queue.put(None)
        self._thread.join()

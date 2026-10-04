"""无模型、无内容哈希的固定测试向量；不代表语义质量或历史向量空间。"""
from __future__ import annotations
import numpy as np
from services.memory.vectorizer import TextEmbedder


class ExplicitTextEmbedder(TextEmbedder):
    def __init__(self, *, dimension=256, model_name='fixture-constant-v1'):
        if dimension <= 0:
            raise ValueError('fixture dimension must be positive')
        self.dimension = dimension
        self.model_name = model_name

    def embed_text(self, text):
        vector = np.zeros(self.dimension, dtype=np.float32)
        if str(text or '').strip():
            vector[0] = 1.0
        return vector

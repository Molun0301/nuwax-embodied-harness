"""按检测标签和类别匹配实例类别先验。"""
from __future__ import annotations

import re

from contracts.spatial_memory import InstanceDynamicState


_STATIC_CLASS_KEYWORDS = ('wall', 'door', 'dock', 'station', 'shelf', 'charger', 'table', 'cabinet')
_DYNAMIC_CLASS_KEYWORDS = (
    'person', 'human', 'pedestrian', 'dog', 'cat', 'visitor', 'worker', 'vehicle',
    'car', 'truck', 'bus', 'bicycle', 'motorcycle',
)
_QUASI_STATIC_CLASS_KEYWORDS = ('chair', 'cart', 'box', 'bag', 'cone', 'barrier', 'barricade', 'trash')
_CLASS_KEYWORDS = (
    (InstanceDynamicState.DYNAMIC, _DYNAMIC_CLASS_KEYWORDS),
    (InstanceDynamicState.QUASI_STATIC, _QUASI_STATIC_CLASS_KEYWORDS),
    (InstanceDynamicState.STATIC, _STATIC_CLASS_KEYWORDS),
)
_WORDS = re.compile(r'[^\W_]+')


def classify_instance_category(label, category):
    words = tuple(_WORDS.findall(str(value or '').lower()) for value in (label, category))
    for class_prior, keywords in _CLASS_KEYWORDS:
        for keyword in keywords:
            sequence = _WORDS.findall(keyword)
            size = len(sequence)
            if any(tokens[start:start + size] == sequence
                   for tokens in words for start in range(len(tokens) - size + 1)):
                return class_prior
    return InstanceDynamicState.UNKNOWN

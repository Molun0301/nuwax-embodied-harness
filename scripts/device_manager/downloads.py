"""安装时下载的第三方文件缓存；固定来源、大小和摘要才能进入候选环境。"""
from __future__ import annotations


MODEL_SETTINGS = {"NUWAX_PERCEPTION_YOLO_WORLD_MODEL", "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL",
                  "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL", "NUWAX_MEMORY_EMBEDDING_MODEL"}


def uses_default_model(value):
    """与共同安装器保持一致：词法模式及旧占位符都不是用户模型路径。"""
    value = str(value).strip().lower()
    return value in {"", "lexical-only", "lexical_only", "disabled"} or value.startswith("hashing")


def apply_model_overrides(existing, overrides):
    """重新启用记忆模型时让原安装器恢复模型维度，不沿用停用时的占位维度。"""
    existing = dict(existing)
    for kind in ("TEXT", "IMAGE"):
        key = "NUWAX_MEMORY_" + kind + "_EMBEDDING_MODEL"
        dimension = "NUWAX_MEMORY_" + kind + "_EMBEDDING_DIM"
        if key in overrides and uses_default_model(existing.get(key, "")):
            remembered = existing.get("_NUWAX_INSTALLER_SAVED_" + dimension, "")
            if remembered:
                existing[dimension] = remembered
            else:
                existing.pop(dimension, None)
    existing.update(overrides)
    return existing

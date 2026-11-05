"""记忆业务包；算法模块可独立导入。"""


def __getattr__(name):
    if name == "MemoryService":
        from services.memory.service import MemoryService
        return MemoryService
    raise AttributeError(name)


__all__ = ["MemoryService"]

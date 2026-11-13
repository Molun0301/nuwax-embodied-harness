"""当前任务、状态与数据流的公共类型。

名称在首次取用时才导入对应子模块：设备管理的配置探针、监督者等只用
core.config 的进程不再连带加载事件总线、任务执行器与全部契约模型。
"""
from importlib import import_module

_EXPORTS = {
    "Capability": "core.capability", "CapabilityTable": "core.capability",
    "EventBus": "core.event_bus", "EventSubscription": "core.event_bus",
    "StateEntry": "core.state_store", "StateNamespace": "core.state_store", "StateStore": "core.state_store",
    "StreamBus": "core.stream_bus", "StreamSubscription": "core.stream_bus",
    "Cancelled": "core.tasks", "TaskContext": "core.tasks", "TaskRunner": "core.tasks",
    "TaskState": "core.tasks", "TaskStatus": "core.tasks",
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError("module 'core' has no attribute %r" % name)
    return getattr(import_module(_EXPORTS[name]), name)

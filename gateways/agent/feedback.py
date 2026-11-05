"""智能体任务状态的传输编码。"""
from gateways.serialization import to_jsonable

TERMINAL_STATES = frozenset({"succeeded", "failed", "cancelled", "timed_out"})


def task_record(status):
    """所有任务结果直接来自任务执行器，不建立结果副本。"""
    return {"task": to_jsonable(status)}

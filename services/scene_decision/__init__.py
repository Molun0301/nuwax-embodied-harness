"""场景决策客户端；导入时不加载推理框架或模型。"""

from services.scene_decision.backends import SceneDecisionError, create_scene_decision_backend

__all__ = ["SceneDecisionError", "create_scene_decision_backend"]

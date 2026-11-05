"""Go2公开机身动作的数据契约与纯参数表生成。"""
from __future__ import annotations
from dataclasses import dataclass,field
from typing import Dict,Tuple

def _object_schema(
    properties: Dict[str, object] = None,
    *,
    required: Tuple[str, ...] = (),
) -> Dict[str, object]:
    return {
        "type": "object",
        "properties": dict(properties or {}),
        "required": list(required),
        "additionalProperties": False,
    }


@dataclass(frozen=True)
class Go2SportCommandSpec:
    """Go2 已公开的机身动作命令规范。"""

    name: str
    display_name: str
    description: str
    params_schema: Dict[str, object] = field(default_factory=dict)
    required_params: Tuple[str, ...] = ()

    def to_schema_variant(self) -> Dict[str, object]:
        """导出单条动作的输入模式分支。"""

        return {
            "type": "object",
            "title": self.display_name,
            "description": self.description,
            "properties": {
                "action": {
                    "const": self.name,
                    "description": f"{self.display_name}（{self.name}）",
                },
                "params": _object_schema(self.params_schema, required=self.required_params),
            },
            "required": ["action"],
            "additionalProperties": False,
        }

    def to_catalog_item(self) -> Dict[str, object]:
        """导出供 MCP 和运行时查询的动作目录条目。"""

        return {
            "display_name": self.display_name,
            "description": self.description,
            "params_schema": _object_schema(self.params_schema, required=self.required_params),
            "required_params": list(self.required_params),
        }


GO2_SPORT_COMMAND_SPECS = (
    Go2SportCommandSpec("damp", "阻尼模式", "让机器人进入阻尼模式。"),
    Go2SportCommandSpec("balance_stand", "平衡站立", "让机器人进入平衡站立状态。"),
    Go2SportCommandSpec("stop_move", "停止移动", "立即停止当前底盘运动。"),
    Go2SportCommandSpec("stand_up", "起立", "让机器人从非站立状态起立。"),
    Go2SportCommandSpec("stand_down", "趴下", "让机器人进入趴下状态。"),
    Go2SportCommandSpec("recovery_stand", "恢复站立", "触发恢复站立流程。"),
    Go2SportCommandSpec("sit", "坐下", "让机器人执行坐下动作。"),
    Go2SportCommandSpec("rise_sit", "坐起", "让机器人从坐姿起身。"),
    Go2SportCommandSpec("hello", "打招呼", "执行打招呼动作。"),
    Go2SportCommandSpec("stretch", "伸展", "执行伸展动作。"),
    Go2SportCommandSpec("dance1", "舞蹈一", "执行舞蹈动作一。"),
    Go2SportCommandSpec("dance2", "舞蹈二", "执行舞蹈动作二。"),
    Go2SportCommandSpec("front_flip", "前空翻", "执行前空翻动作。"),
    Go2SportCommandSpec("front_jump", "前跳", "执行前跳动作。"),
    Go2SportCommandSpec("front_pounce", "前扑", "执行前扑动作。"),
    Go2SportCommandSpec("left_flip", "左翻", "执行左翻动作。"),
    Go2SportCommandSpec("back_flip", "后空翻", "执行后空翻动作。"),
    Go2SportCommandSpec("free_walk", "自由行走", "进入自由行走模式。"),
    Go2SportCommandSpec(
        "hand_stand",
        "倒立",
        "切换倒立状态。",
        params_schema={
            "flag": {
                "type": "boolean",
                "default": True,
                "description": "true 表示进入或开启；false 表示退出或关闭。",
            }
        },
    ),
    Go2SportCommandSpec(
        "free_jump",
        "自由跳跃",
        "切换自由跳跃模式。",
        params_schema={
            "flag": {
                "type": "boolean",
                "default": True,
                "description": "true 表示进入或开启；false 表示退出或关闭。",
            }
        },
    ),
    Go2SportCommandSpec(
        "free_bound",
        "自由弹跳",
        "切换自由弹跳模式。",
        params_schema={
            "flag": {
                "type": "boolean",
                "default": True,
                "description": "true 表示进入或开启；false 表示退出或关闭。",
            }
        },
    ),
    Go2SportCommandSpec(
        "walk_upright",
        "直立行走",
        "切换直立行走模式。",
        params_schema={
            "flag": {
                "type": "boolean",
                "default": True,
                "description": "true 表示进入或开启；false 表示退出或关闭。",
            }
        },
    ),
    Go2SportCommandSpec(
        "cross_step",
        "交叉步",
        "切换交叉步模式。",
        params_schema={
            "flag": {
                "type": "boolean",
                "default": True,
                "description": "true 表示进入或开启；false 表示退出或关闭。",
            }
        },
    ),
    Go2SportCommandSpec(
        "switch_joystick",
        "切换遥杆模式",
        "开启或关闭遥杆模式。",
        params_schema={
            "on": {
                "type": "boolean",
                "default": True,
                "description": "true 表示开启；false 表示关闭。",
            }
        },
    ),
    Go2SportCommandSpec(
        "pose",
        "姿态模式",
        "切换姿态模式。",
        params_schema={
            "flag": {
                "type": "boolean",
                "default": True,
                "description": "true 表示进入或开启；false 表示退出或关闭。",
            }
        },
    ),
    Go2SportCommandSpec(
        "classic_walk",
        "经典步态",
        "切换经典步态。",
        params_schema={
            "flag": {
                "type": "boolean",
                "default": True,
                "description": "true 表示进入或开启；false 表示退出或关闭。",
            }
        },
    ),
)


GO2_SPORT_COMMAND_NAMES = tuple(spec.name for spec in GO2_SPORT_COMMAND_SPECS)


def build_go2_sport_command_description() -> str:
    """构造供 MCP 工具面使用的机身动作详细说明。"""

    lines = [
        "执行 Go2 已公开的高层机身动作命令。",
        "支持的 action 如下：",
    ]
    for spec in GO2_SPORT_COMMAND_SPECS:
        lines.append(f"- {spec.name}：{spec.display_name}，{spec.description}")
        if spec.params_schema:
            param_chunks = []
            for param_name, schema in spec.params_schema.items():
                default_text = ""
                if isinstance(schema, dict) and "default" in schema:
                    default_text = f"，默认 {schema['default']}"
                description = ""
                if isinstance(schema, dict) and schema.get("description"):
                    description = f"，{schema['description']}"
                param_chunks.append(f"{param_name}{default_text}{description}")
            lines.append(f"  参数：{'；'.join(param_chunks)}")
    lines.append("不包含 relative_move、set_body_pose、set_speed_level、switch_control_mode 和低层关节控制；这些能力请使用各自专用工具。")
    return "\n".join(lines)


def build_go2_sport_command_input_schema() -> Dict[str, object]:
    """构造 execute_sport_command 的完整输入结构模式。"""

    return {
        "type": "object",
        "description": "执行 Go2 已公开的高层机身动作命令。action 的完整取值和 params 结构见 x-nuwax-action-catalog。",
        "properties": {
            "action": {
                "type": "string",
                "enum": list(GO2_SPORT_COMMAND_NAMES),
                "description": "机身动作命令名称。",
            },
            "params": {
                "type": "object",
                "default": {},
                "description": "动作参数对象；不同 action 的参数结构不同。",
            },
        },
        "required": ["action"],
        "additionalProperties": False,
        "oneOf": [spec.to_schema_variant() for spec in GO2_SPORT_COMMAND_SPECS],
        "x-nuwax-action-catalog": {
            spec.name: spec.to_catalog_item() for spec in GO2_SPORT_COMMAND_SPECS
        },
    }

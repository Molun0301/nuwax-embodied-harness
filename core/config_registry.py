"""配置注册表（SSOT 元数据）。

个人机器狗 Web 配置面板的单一事实源：集中描述 settings.py / drivers/go2/settings.py
读取的每个环境变量的「展示元数据」（中文名、所属板块、常规/高级、是否敏感、目标文件、
alias 老名）。

设计要点（见实施计划 §七 R8）：
- 本注册表**不誊写默认值**。默认值在加载时从 settings.py 源码 AST 直接解析（唯一事实源），
  避免「registry 默认值 vs 代码默认值」漂移。
- 板块划分与常规/高级归类来自实施计划 §二（评审定稿）。
- 鉴权已移除：网关不再有 agent/admin 令牌，故无相关敏感项。
"""

from __future__ import annotations

import ast
import json
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_REPO_ROOT = Path(__file__).resolve().parent.parent
_CORE_SETTINGS = _REPO_ROOT / "core" / "settings.py"
_GO2_SETTINGS = _REPO_ROOT / "drivers" / "go2" / "settings.py"
_DESCRIPTIONS_JSON = _REPO_ROOT / "core" / "config_descriptions.json"

# 目标文件标识：core 主 .env / go2 数据面独立 env（drivers/go2/data_plane.env）。
TARGET_CORE = "core_env"
TARGET_GO2 = "go2_data_plane_env"


@dataclass(frozen=True)
class ConfigField:
    """单个配置项的展示元数据。"""

    key: str
    zh_name: str
    description: str
    type: str  # str | int | bool | float | csv
    section_id: str
    section_title: str
    advanced: bool
    sensitive: bool
    target_file: str
    default_repr: str  # 源码解析出的默认值文本（仅展示，不作运行时来源）
    alias_of: Optional[str] = None
    choices: Tuple[str, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class _RawKey:
    """从源码 AST 解析出的原始键信息（默认值唯一事实源）。"""

    key: str
    type: str
    default_repr: str
    alias_of: Optional[str]
    target_file: str


def _node_source(node: ast.AST, source: str) -> str:
    """取 AST 节点对应的源码文本，Python 3.8 兼容。

    优先 ``ast.get_source_segment``（3.8 起可用）；缺失时回退到 ``ast.unparse``
    （3.9+）；都没有则空串。替代直接用 ``ast.unparse``——后者 3.8 不存在，
    实机（Python 3.8）会 AttributeError 导致 build_registry 崩溃。
    """

    seg = ast.get_source_segment(source, node)
    if seg is not None:
        return seg
    unparse = getattr(ast, "unparse", None)
    if unparse is not None:
        return unparse(node)
    return ""


def _parse_settings_keys(path: Path, target_file: str) -> Dict[str, _RawKey]:
    """AST 解析 settings 文件，提取每个 _cfg_* 调用的键/类型/默认值/alias。

    默认值取源码字面量文本（如 ``True`` / ``'/utlidar/robot_odom'`` / 嵌套 _cfg_* 表达式），
    仅用于展示；运行时真实值仍由 settings.py 的 load_config() 决定。
    """

    raw: Dict[str, _RawKey] = {}
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)):
            continue
        fn = node.func.id
        if fn == "_prefer_platform_str":
            # _prefer_platform_str(platform_value, "KEY", default) -> 内部走 _cfg_str("KEY", default)
            args = node.args
            if len(args) >= 2 and isinstance(args[1], ast.Constant) and isinstance(args[1].value, str):
                key = args[1].value
                default_repr = ""
                if len(args) >= 3:
                    try:
                        default_repr = repr(ast.literal_eval(args[2]))
                    except Exception:
                        default_repr = _node_source(args[2], source)
                raw.setdefault(key, _RawKey(key, "str", default_repr, None, target_file))
            continue
        if not fn.startswith("_cfg_"):
            continue
        typ = fn.replace("_cfg_", "").replace("_alias", "")
        is_alias = fn.endswith("_alias")
        args = node.args
        if not args or not isinstance(args[0], ast.Constant) or not isinstance(args[0].value, str):
            continue
        key = args[0].value
        alias_of: Optional[str] = None
        default_idx = 1
        if (
            is_alias
            and len(args) > 1
            and isinstance(args[1], ast.Constant)
            and isinstance(args[1].value, str)
            and args[1].value.isupper()
        ):
            alias_of = args[1].value
            default_idx = 2
        default_repr = ""
        if len(args) > default_idx:
            try:
                default_repr = repr(ast.literal_eval(args[default_idx]))
            except Exception:
                default_repr = _node_source(args[default_idx], source)
        # 同键多次出现以首次为准（settings.py 内通常唯一）。
        raw.setdefault(key, _RawKey(key, typ, default_repr, alias_of, target_file))
    return raw


def _all_raw_keys() -> Dict[str, _RawKey]:
    merged: Dict[str, _RawKey] = {}
    merged.update(_parse_settings_keys(_CORE_SETTINGS, TARGET_CORE))
    for key, rk in _parse_settings_keys(_GO2_SETTINGS, TARGET_GO2).items():
        merged.setdefault(key, rk)
    return merged


# 板块定义（顺序即面板展示顺序）。来自实施计划 §二。
SECTIONS: Tuple[Tuple[str, str], ...] = (
    ("robot", "机器人与运行入口"),
    ("gateway", "网关与服务"),
    ("telemetry", "性能监控（遥测）"),
    ("logging", "日志"),
    ("geometry_topics", "几何引擎·话题与坐标系"),
    ("geometry_quality", "几何引擎·点云与质量"),
    ("rtabmap", "建图后端（RTAB-Map）"),
    ("nav2", "导航（Nav2）"),
    ("external_geometry", "外部几何栈与桥接"),
    ("perception_detect", "感知·检测与跟踪"),
    ("perception_vision", "感知·分割/云端视觉/延迟投影"),
    ("perception_stream", "感知·关键帧与流"),
    ("runtime_data", "记忆与制品"),
    ("tts_doubao", "语音·豆包 TTS"),
    ("tts_player", "语音·播放与音量"),
    ("tts_log_bridge", "语音·日志转 TTS"),
    ("go2_input", "Go2 数据面·输入流"),
    ("go2_official", "Go2 数据面·官方控制"),
    ("go2_video", "Go2 数据面·视频推流"),
    ("relay", "容器内 Relay 转发"),
    ("misc", "其它"),
)
_SECTION_TITLES = dict(SECTIONS)

# 分类规则：(section_id, 前缀元组)。按列表顺序匹配，**首个命中**生效，故更具体的前缀在前。
_SECTION_RULES: Tuple[Tuple[str, Tuple[str, ...]], ...] = (
    ("telemetry", ("NUWAX_GATEWAY_TELEMETRY",)),
    ("gateway", ("NUWAX_GATEWAY",)),
    ("relay", ("NUWAX_RELAY",)),
    ("tts_log_bridge", ("TTS_LOG_BRIDGE", "NUWAX_LOG_TTS")),
    ("tts_player", ("TTS_PLAYER", "TTS_LOCAL", "TTS_ROBOT", "GO2_VUI", "GO2_AUDIO", "NUWAX_VOLUME", "TTS_APE")),
    ("tts_doubao", ("DOUBAO", "TTS_")),
    ("logging", ("NUWAX_LOG", "GO2_PROXY_LOG")),
    ("rtabmap", ("NUWAX_RTABMAP", "NUWAX_BACKEND")),
    ("nav2", ("NUWAX_NAV2",)),
    ("external_geometry", ("NUWAX_GEOMETRY_EXTERNAL", "NUWAX_GEOMETRY_INPUT", "NUWAX_GEOMETRY_POINT_CLOUD_BRIDGE",
                            "NUWAX_GEOMETRY_ODOM_TF", "NUWAX_GEOMETRY_CMD_VEL", "NUWAX_GEOMETRY_PROVIDES")),
    ("geometry_quality", ("NUWAX_GEOMETRY_VOXEL", "NUWAX_GEOMETRY_CROP", "NUWAX_GEOMETRY_MAX_RANGE",
                           "NUWAX_GEOMETRY_TRAVERSABILITY", "NUWAX_GEOMETRY_MAX_", "NUWAX_GEOMETRY_ODOM_COV",
                           "NUWAX_GEOMETRY_TIMESTAMP", "NUWAX_GEOMETRY_POINTCLOUD")),
    ("geometry_topics", ("NUWAX_GEOMETRY", "NUWAX_ROS", "CYCLONEDDS")),
    ("perception_detect", ("NUWAX_PERCEPTION_YOLO", "NUWAX_PERCEPTION_BYTETRACK", "NUWAX_PERCEPTION_TRACKER")),
    ("perception_vision", ("NUWAX_PERCEPTION_OPENAI", "NUWAX_PERCEPTION_SEGMENTATION", "NUWAX_PERCEPTION_DEFERRED",
                            "OPENAI_", "ARK_")),
    ("perception_stream", ("NUWAX_PERCEPTION_STREAM", "NUWAX_PERCEPTION_KEYFRAME", "NUWAX_PERCEPTION_BURST",
                            "NUWAX_PERCEPTION",)),
    ("runtime_data", ("NUWAX_MEMORY", "NUWAX_MAP", "NUWAX_ARTIFACT", "NUWAX_RUNTIME", "NUWAX_STATE",
                       "NUWAX_DIAGNOSTIC", "NUWAX_ACTIVE_MAP", "NUWAX_LOCALIZATION_HISTORY",
                       "NUWAX_NAVIGATION_HISTORY", "NUWAX_OBSERVATION_HISTORY", "DASHSCOPE")),
    ("go2_video", ("GO2_VIDEO_STREAM",)),
    ("go2_official", ("GO2_OFFICIAL",)),
    ("go2_input", ("GO2_DATA_PLANE", "GO2_DIRECT_DDS", "GO2_CYCLONEDDS", "GO2_DDS")),
    ("robot", ("NUWAX_ACTIVE_ROBOT", "ROBOT_IP", "GO2_SDK", "GO2_LOW_LEVEL", "GO2_NETWORK", "GO2_")),
)

# 敏感键（脱敏不回显）。鉴权已移除，故仅余各类外部服务密钥/令牌。
_SENSITIVE_KEYS = frozenset({
    "NUWAX_RELAY_INCOMING_TOKENS",
    "NUWAX_RELAY_UPSTREAM_TOKEN",
    "DOUBAO_ACCESS_KEY",
    "NUWAX_PERCEPTION_OPENAI_API_KEY",
    "OPENAI_API_KEY",
    "ARK_API_KEY",
    "NUWAX_MEMORY_EMBEDDING_API_KEY",
    "DASHSCOPE_API_KEY",
    "GO2_VIDEO_STREAM_ACCESS_TOKEN",
})

# 常规项（高频，默认展开）。来自实施计划 §二「常规项」列；其余一律高级（默认折叠）。
_REGULAR_KEYS = frozenset({
    "NUWAX_ACTIVE_ROBOT", "GO2_DDS_IFACE", "ROBOT_IP",
    "NUWAX_GATEWAY_HOST", "NUWAX_GATEWAY_PORT",
    "NUWAX_GATEWAY_TELEMETRY_ENABLED", "NUWAX_GATEWAY_TELEMETRY_INTERVAL_SEC",
    "NUWAX_LOG_LEVEL",
    "NUWAX_GEOMETRY_ENABLED", "NUWAX_GEOMETRY_STACK",
    "NUWAX_BACKEND_PROFILE_NAME", "NUWAX_BACKEND_ENABLED",
    "NUWAX_NAV2_MODE", "NUWAX_NAV2_AUTOSTART",
    "NUWAX_GEOMETRY_EXTERNAL_ENABLED",
    "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED", "NUWAX_PERCEPTION_YOLO_WORLD_CONFIDENCE",
    "NUWAX_PERCEPTION_STREAM_AUTO_START",
    "NUWAX_PERCEPTION_OPENAI_ENABLED", "NUWAX_PERCEPTION_SEGMENTATION_ENABLED",
    "NUWAX_PERCEPTION_STREAM_ENABLED", "NUWAX_PERCEPTION_STREAM_INTERVAL_SEC",
    "NUWAX_MEMORY_EMBEDDING_API_KEY", "NUWAX_MAP_CATALOG_ROOT",
    "DOUBAO_APP_ID", "DOUBAO_ACCESS_KEY", "DOUBAO_DEFAULT_SPEAKER",
    "TTS_PLAYER_AUTO_START", "TTS_PLAYER_VOLUME",
    "TTS_LOG_BRIDGE_ENABLED",
    "GO2_DATA_PLANE_ENABLED", "GO2_DIRECT_DDS_ENABLED",
    "GO2_OFFICIAL_BACKEND_ENABLED", "GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED",
    "GO2_VIDEO_STREAM_ENABLED", "GO2_VIDEO_STREAM_FPS",
    "NUWAX_RELAY_ENABLED",
})

# 枚举型键的可选值（用于前端下拉）。
_CHOICES: Dict[str, Tuple[str, ...]] = {
    "NUWAX_LOG_LEVEL": ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
    "NUWAX_GEOMETRY_STACK": ("external_slam_nav2", "rtabmap", "none"),
    "NUWAX_NAV2_MODE": ("nav_only", "slam", "localization", "disabled"),
}


def _classify_section(key: str) -> str:
    for section_id, prefixes in _SECTION_RULES:
        if key.startswith(prefixes):
            return section_id
    return "misc"


@lru_cache(maxsize=1)
def _env_example_desc() -> Dict[str, str]:
    """读取冻结的描述表 core/config_descriptions.json。

    注意：描述**不**从 .env.example 收割——.env.example 现在是由本 registry 生成的产物，
    若反向收割会形成循环依赖（每次重生成都把注释叠加一次）。描述独立冻结为 JSON，
    源自历史策展模板，可手工增补。
    """

    path = _DESCRIPTIONS_JSON
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return {str(k): str(v) for k, v in data.items() if v}


# 高频键的精修中文名（其余由 _derive_zh_name 机械生成）。
_ZH_NAME_OVERRIDES: Dict[str, str] = {
    "NUWAX_ACTIVE_ROBOT": "激活机器人",
    "NUWAX_GATEWAY_HOST": "网关监听地址",
    "NUWAX_GATEWAY_PORT": "网关端口",
    "NUWAX_GATEWAY_TELEMETRY_ENABLED": "启用性能监控",
    "NUWAX_GATEWAY_TELEMETRY_INTERVAL_SEC": "性能监控采集间隔",
    "NUWAX_LOG_LEVEL": "日志级别",
    "NUWAX_GEOMETRY_ENABLED": "启用几何引擎",
    "NUWAX_GEOMETRY_STACK": "几何栈类型",
    "NUWAX_GEOMETRY_EXTERNAL_ENABLED": "启用统一几何外部进程",
    "NUWAX_GEOMETRY_EXTERNAL_PROCESS_NAME": "统一几何进程名",
    "NUWAX_GEOMETRY_EXTERNAL_SCRIPT": "统一几何启动脚本",
    "NUWAX_GEOMETRY_OCCUPANCY_TOPIC": "占据栅格话题",
    "NUWAX_GEOMETRY_ODOM_TOPIC": "里程计话题",
    "NUWAX_GEOMETRY_POINT_CLOUD_TOPIC": "点云话题",
    "NUWAX_GEOMETRY_MAP_FRAME": "地图坐标系",
    "NUWAX_GEOMETRY_ODOM_FRAME": "里程计坐标系",
    "NUWAX_GEOMETRY_BASE_FRAME": "机器人基座坐标系",
    "NUWAX_NAV2_MODE": "Nav2 模式",
    "NUWAX_NAV2_AUTOSTART": "自动启动导航",
    "NUWAX_BACKEND_PROFILE_NAME": "几何配置名称",
    "NUWAX_BACKEND_ENABLED": "启用建图后端",
    "NUWAX_PERCEPTION_OPENAI_ENABLED": "启用 OpenAI 视觉分析",
    "NUWAX_PERCEPTION_OPENAI_API_KEY": "OpenAI API 密钥",
    "NUWAX_PERCEPTION_OPENAI_MODEL": "OpenAI 视觉模型",
    "NUWAX_PERCEPTION_SEGMENTATION_ENABLED": "启用实例分割",
    "NUWAX_PERCEPTION_STREAM_ENABLED": "启用持续视频感知",
    "NUWAX_PERCEPTION_STREAM_AUTO_START": "启动时自动开启视频感知",
    "NUWAX_PERCEPTION_STREAM_INTERVAL_SEC": "视频感知间隔",
    "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED": "启用 YOLO-World 检测",
    "NUWAX_PERCEPTION_YOLO_WORLD_CONFIDENCE": "YOLO-World 置信度阈值",
    "NUWAX_MEMORY_EMBEDDING_API_KEY": "记忆嵌入 API 密钥",
    "NUWAX_MAP_CATALOG_ROOT": "地图目录",
    "DOUBAO_APP_ID": "豆包 App ID",
    "DOUBAO_ACCESS_KEY": "豆包 Access Key",
    "DOUBAO_DEFAULT_SPEAKER": "豆包默认音色",
    "TTS_PLAYER_VOLUME": "播放音量",
    "TTS_PLAYER_AUTO_START": "自动启动播放器",
    "GO2_DATA_PLANE_ENABLED": "启用 Go2 数据面",
    "GO2_VIDEO_STREAM_ENABLED": "启用视频推流",
    "GO2_VIDEO_STREAM_FPS": "推流帧率",
    "NUWAX_RELAY_ENABLED": "启用 Relay 转发",
    "GO2_DDS_IFACE": "DDS 网卡",
    "ROBOT_IP": "机器人 IP",
}

_TOKEN_ZH: Dict[str, str] = {
    "enabled": "启用",
    "auto": "自动",
    "start": "启动",
    "host": "监听地址",
    "port": "端口",
    "model": "模型",
    "path": "路径",
    "root": "根目录",
    "topic": "话题",
    "frame": "坐标系",
    "interval": "间隔",
    "timeout": "超时",
    "confidence": "置信度",
    "threshold": "阈值",
    "source": "来源",
    "target": "目标",
    "map": "地图",
    "odom": "里程计",
    "base": "基座",
    "point": "点",
    "cloud": "云",
    "bridge": "桥接",
    "process": "进程",
    "name": "名称",
    "script": "脚本",
    "database": "数据库",
    "directory": "目录",
    "file": "文件",
    "level": "级别",
    "format": "格式",
    "fps": "帧率",
    "stream": "流",
    "video": "视频",
    "api": "API",
    "key": "密钥",
    "url": "URL",
    "tts": "TTS",
    "relay": "Relay",
    "nav2": "Nav2",
    "rtabmap": "RTAB-Map",
    "go2": "Go2",
    "ros": "ROS",
    "dds": "DDS",
    "yolo": "YOLO",
    "world": "World",
    "openai": "OpenAI",
}


def _derive_zh_name(key: str, section_id: str) -> str:
    """为未精修的键机械生成可读中文名，避免裸露 enabled/path/topic 等后缀。"""

    if key in _ZH_NAME_OVERRIDES:
        return _ZH_NAME_OVERRIDES[key]
    suffix = key
    for _sid, prefixes in _SECTION_RULES:
        for p in sorted(prefixes, key=len, reverse=True):
            if key.startswith(p):
                suffix = key[len(p):].lstrip("_") or key
                break
        else:
            continue
        break
    tokens = [part.lower() for part in suffix.split("_") if part]
    if not tokens:
        return _SECTION_TITLES.get(section_id, key)
    translated = [_TOKEN_ZH.get(token, token.upper() if len(token) <= 3 else token) for token in tokens]
    pretty = "".join(translated)
    return pretty or key


@lru_cache(maxsize=1)
def build_registry() -> Tuple[ConfigField, ...]:
    """组装完整注册表（板块顺序 → 板块内常规先于高级 → 键名）。"""

    raw = _all_raw_keys()
    descs = _env_example_desc()
    fields: List[ConfigField] = []
    for key, rk in raw.items():
        section_id = _classify_section(key)
        fields.append(ConfigField(
            key=key,
            zh_name=_derive_zh_name(key, section_id),
            description=descs.get(key, ""),
            type=rk.type,
            section_id=section_id,
            section_title=_SECTION_TITLES.get(section_id, "其它"),
            advanced=key not in _REGULAR_KEYS,
            sensitive=key in _SENSITIVE_KEYS,
            target_file=rk.target_file,
            default_repr=rk.default_repr,
            alias_of=rk.alias_of,
            choices=_CHOICES.get(key, ()),
        ))
    section_order = {sid: i for i, (sid, _t) in enumerate(SECTIONS)}
    fields.sort(key=lambda f: (section_order.get(f.section_id, 999), f.advanced, f.key))
    return tuple(fields)


def all_keys() -> Tuple[str, ...]:
    return tuple(f.key for f in build_registry())


def get_field(key: str) -> Optional[ConfigField]:
    for f in build_registry():
        if f.key == key:
            return f
    return None


def sensitive_keys() -> frozenset:
    return _SENSITIVE_KEYS
